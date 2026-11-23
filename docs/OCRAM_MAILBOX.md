# The OCRAM mailbox (proposed)

A proposed third path between the HPS and a core, beside the GPO/GPI ("SPI") link and the LW bridge window of
`docs/LW_BRIDGE.md`. It is for bulk: OSD rows, disk blocks, replies. The HPS writes records into its own 64 KB
**on-chip RAM** (OCRAM, physical `0xFFFF0000`), and the core fetches them as an AXI master on the Cyclone V
**FPGA-to-HPS bridge**, which `sys/` already instantiates and leaves unused.

**Status: a design, not a build.** Nothing in `sys/` implements this document, no macro below exists yet, no figure in
it is measured, and several facts it rests on are unverified ("What is not verified", at the end). It is published
with the LW framework so that core and host authors can read both paths side by side and say which they would build.

- **The core sees nothing new.** OSD rows land in the same `osd_buffer`, blocks arrive as the same
  `sd_ack`/`sd_buff_wr`/`sd_buff_dout` strobes, exactly as with the LW tiers.
- **The SPI link is untouched,** and so is the LW window. Every path is a second or third carrier, never a
  replacement, and a host that knows none of them drives the core over SPI as before.
- **No block memory.** The payload waits in the HPS's RAM, not in a fabric FIFO. Every OCRAM tier is logic only.
- **Nothing the HPS does here can hang it.** The HPS reads and writes its own memory. The fabric is the master, so
  a core that stalls stalls only its own fetch.

## Why a core author might want it

| | SPI link | LW window | OCRAM mailbox |
|---|---|---|---|
| who moves the bytes | the HPS, two register stores per word | the HPS, one bridge store per 32-bit word | the HPS stores to memory; the fabric fetches |
| where a burst waits | nowhere: the HPS is held | a fabric FIFO (block memory) | the HPS's on-chip RAM |
| block memory per function | none | OSD 2, block reads 3, frame lane 3, pads 1 | none |
| logic | none new | a floor, then a block per function | a second floor, then a consumer per function |
| an access nobody answers | cannot happen | hangs the SoC, hence the firewall and the host rules | cannot happen to the HPS |
| needs from the host | nothing | discovery, the enable, per-block protocol | discovery, a hello, records |

The mailbox is not free. Its floor (the bridge master and its control logic) is logic a core pays once, beside the
LW floor if it builds both. For a core whose only bulk function is the OSD, the LW OSDBUF block is smaller in logic
and costs two block memories; the mailbox costs more logic and none. The mailbox pays for a core with two or more
bulk functions, for a core that cannot spare block memory, and for a core that builds no LW bridge at all.

## The menu

Every path is its own macro, off by default; a path that is not defined builds no logic. An OCRAM tier needs the
OCRAM floor, as an LW tier needs `MISTER_LW`. Naming two paths for one function builds both, and the host binds one.

| macro (proposed) | builds | cost (ESTIMATED, by analogy with the fitted LW blocks less their FIFOs) |
|---|---|---|
| `MISTER_OCRAM` | the floor: the bridge master, the poller, the hello, two GPI bits | 180-260 ALMs, no block memory |
| `MISTER_OCRAM_OSD` | OSD rows from the OSD ring into each `osd`'s `osd_buffer` | 60-80 ALMs |
| `MISTER_OCRAM_SECTOR_SLOT` and its geometry | block read frames for one `hps_io` slot and up to two save slots | 180-230 ALMs, 280-350 with two save slots |
| `MISTER_OCRAM_EVT` | the status back-channel and `hps_io`'s flags as records | 70-120 ALMs |
| `MISTER_OCRAM_LANE` | link frames from a ring into the frame-lane player | 130-180 ALMs |
| `MISTER_OCRAM_PAD` | pad stores from a ring | 40-60 ALMs |
| `MISTER_OCRAM_POLL` | the idle poll period, as a power of two of `clk_100m` cycles (default 12: 41 us) | - |

**Three set menus** a `.qsf` could carry as commented blocks:

| set | macros | for |
|---|---|---|
| LW pads, OCRAM bulk | `MISTER_LW`, `MISTER_LW_PAD`, `MISTER_LW_EVT`, `MISTER_OCRAM`, `MISTER_OCRAM_OSD`; a disc core adds `MISTER_OCRAM_SECTOR_SLOT` | a core with a disc, or two bulk functions: pads keep the path proven on hardware, the blocks that cost block memory move |
| lean, no LW bridge | `MISTER_OCRAM`, `MISTER_OCRAM_OSD`, `MISTER_OCRAM_EVT`; a disc core adds `MISTER_OCRAM_SECTOR_SLOT`; pads stay on SPI or take `MISTER_OCRAM_PAD` | a core short of logic or block memory, or one that wants no bridge the HPS must be answered on |
| LW only | the tiers of `docs/LW_BRIDGE.md` | a cartridge core; any core until the mailbox is proven |

## How it works

### The master

One module wires the existing `cyclonev_hps_interface_fpga2hps` atom at its 32-bit width and is the only logic that
drives it.

- **It runs on `clk_100m`** (`h2f_user0_clk`), and no fabric reset holds it. It never abandons a transaction it has
  started: a core reset, a stopped `clk_sys` or a host that has gone cannot leave one in flight.
- **It cannot address anything but the mailbox.** Address bits 31:16 are the constant `0xFFFF`, and the low bits come
  only from constants below `0xF000`. Through this bridge a fabric master can otherwise reach HPS memory and
  peripherals, so this rule is structural and checked statically, never a matter of register values.
- **One transaction at a time,** single-beat, full 32-bit words.
- **It is gated.** It issues a transaction only while the host has set a run bit (proposed: GPO bit 28) and no HPS
  reset is seen. A host that does not know the mailbox never sets that bit (the stock Main writes GPO bits 16 and
  21-28 as zeros), so under such a host the master issues nothing at all and the core is a stock core.

Consumers run on `clk_sys`. Each fetches one word at a time through one held-data handshake built from the cells of
`sys/lw_cdc.sv`, so no crossing carries a clock ratio and none reaches a video clock. That is why no FIFO is needed:
the RAM is the queue, and the consumer's own pace sets the fetch rate.

### The map

Offsets are from `0xFFFF0000`. The fabric declares this geometry in the control page and a host reads it there, so a
later layout needs no host change.

| offset | size | ring | direction | carries |
|---|---|---|---|---|
| `0x0000` | `0x0100` | control page | both | the hello, binds, every index, the fabric's descriptors |
| `0x0400` | `0x0400` | PAD | host to core | pad stores |
| `0x1000` | `0x1000` | LANE | host to core | link frames |
| `0x2000` | `0x2000` | OSD | host to core | image rows, 65 words each |
| `0x4000` | `0x2000` | EVT | core to host | status, block requests, save-block writes, replies |
| `0x7000` | `0x8000` | SECTOR | host to core | block read frames: thirteen of 2,352 bytes |
| `0xF000` | `0x1000` | never touched | - | the boot ROM's workspace |

A ring whose tier is not built has no descriptor, and its span is never read or written.

### Rings and records

- **A ring** is a power-of-two count of 32-bit words with two indices, each a free-running word count modulo 65,536
  and each a full word in the control page. **Each index is written by one side only:** the producer's head, the
  consumer's tail. `head - tail` words are waiting.
- **A record** is one header word, `{kind[31:24], arg[23:16], len[15:0]}`, then `len` payload words. It may wrap
  the end of its ring.
- **Publishing is one store.** The producer stores every word of the record, then its head. The consumer acts only on
  records wholly below the head, so a producer that stops mid-record has published nothing. There is no partial
  frame, and so no abort, no resend and no drop count.
- **Flow control is the reader's tail.** A producer reads the other side's tail and stores only what fits. Neither
  side waits inside the protocol: a host keeps what did not fit for its next pass, and the fabric holds a request
  standing or coalesces a status to the latest.
- A consumer skips a kind it does not know by its length.

### The control page

| offset | word | written by | content |
|---|---|---|---|
| `0x00` | H_HELLO | host | `{0x4F43, nonce[15:0]}`; 0 = no session. Stored last |
| `0x04` | H_BIND | host | the functions the host takes from SPI: SECTOR slots, EVT, LANE, PAD |
| `0x08` | H_LINE | host | bit *r*: ring *r*'s interrupt line may rise |
| `0x10` + 4*r* | H_IDX[*r*] | host | its head of a host-to-core ring, its tail of a core-to-host ring |
| `0x80` | C_HELLO | fabric | the echo of H_HELLO. Stored last |
| `0x84` | C_VERSION | fabric | `{major, minor}` of the layout |
| `0x88` | C_CAPS | fabric | bit *r*: ring *r* is built |
| `0x8C` | C_STATE | fabric | hellos taken; per-ring fault and at-rest bits |
| `0x90` + 4*r* | C_IDX[*r*] | fabric | its tail of a host-to-core ring, its head of a core-to-host ring |
| `0xB0` + 4*r* | C_RING[*r*] | fabric | `{kind, line ordinal, log2 words, offset in words}`; 0 = absent |
| `0xD0`... | C_K[*n*] | fabric | per-kind constants: SECTOR slots and frame lengths, OSD rows |

## The host's side

### Mapping

Map `0xFFFF0000`, 64 KB, **uncached and strongly ordered**: on ARMv7 Linux an `mmap` of `/dev/mem` over this range
is exactly that, because the range is not kernel RAM. Each store is then one bus write that completes before the
next begins; nothing merges, repeats or reorders; and no cache on either side holds a copy. There is nothing to
flush and no barrier to issue. A cached or write-combined mapping is not legal.

### Before the first store: the warm-boot rule

After boot the on-chip RAM still holds the preloader, and a current U-Boot arms the boot ROM to run that image again
on a warm reset (the system manager's `romcodegrp.warmramgrp.enable`, `0xFFD080E0`, holds `0xAE9EFEBC`). The mailbox
overwrites the image. **So a host stores 0 to that register before its first store to the on-chip RAM**; a warm
reset then loads the preloader from the boot medium, as a cold boot does. A host that skips this risks a machine
that does not come back from a warm reset.

### Discovery and the hello

Reading the on-chip RAM can never hang, so nothing here needs a firewall. What a host needs is proof that a live
master in *this* bitstream is on the other side: the RAM outlives every core load and still holds the last core's
words.

1. **GPI bit 26 = 1** (proposed; in the I/O view, with `gp_out[31] = 1`): the master is in this bitstream. Stock
   bitstreams read 0, and a host that sees 0 touches nothing: not the RAM, not the register above, not the run bit.
2. Store 0 to `warmramgrp.enable`; zero the control page with full-word stores; set the run bit; store H_HELLO with
   a nonce not used since boot.
3. The master starts polling the host half. On a valid H_HELLO it waits until every consumer's `clk_sys` half is at
   rest, zeroes its indices, stores its version, capabilities, descriptors and constants, then C_HELLO.
4. Read C_HELLO until it equals H_HELLO, for at most 1 ms. No echo: no mailbox this session; clear the run bit and
   use the LW window or SPI. An echo: read the descriptors and bind each function once.

**The session ends** when the run bit falls, when the fabric sees an HPS reset, or when H_HELLO changes. Every bind
and line drops, and each function returns to SPI. A host that stops leaves the run bit set and the master polling a
page nobody changes, which is harmless; the next program's start ends it.

### The rule that replaces the firewall

The mirrored danger is **a fabric transaction in flight while the bridge is reset or the FPGA is programmed.**

- **Set the run bit only while the FPGA-to-HPS bridge is out of reset.**
- **Before resetting a bridge, programming the FPGA or requesting an HPS reset: clear the run bit, then read GPI
  bit 23 (proposed: the master holds no transaction) until it is 1.** The wait is one transaction long.
- While `gp_out[31]` is 0 (the ID view, or the core reset through GPO) the master takes no hello and starts no
  record.

### Doorbells

- **Host to core: none.** The master polls the host half every `2^MISTER_OCRAM_POLL` cycles and back to back while a
  ring holds a record. The worst case, 41 us by default, is invisible to an OSD row and small beside a block's
  service time.
- **Core to host: one level line per ring that needs one,** declared in the ring's descriptor with the ordinals of
  `docs/LW_BRIDGE.md` ("Interrupt lines"). It is high while `C_IDX != H_IDX`, and while it is high the master
  re-reads the host's tail back to back, so the line falls within a few transactions of the store that empties the
  ring. A host with no interrupt node reads C_IDX from its loop instead: one memory read.

### Records

| ring | kind | payload | meaning |
|---|---|---|---|
| OSD | ROW, `arg` = row | 64 words: the row's 256 bytes | written into `osd_buffer` as the LW OSDBUF writer does, only while both `osd` selects are low |
| SECTOR | FRAME, `arg` = slot index | the LBA, then the frame's words | played into the slot if it answers the core's standing request; discarded if not |
| EVT | STATUS | four words | a status transfer landed, whole |
| EVT | REQUEST, `arg` = slot index and direction | the LBA | a bound slot's request opened and no queued frame answers it |
| EVT | WRITE, `arg` = slot index | the LBA, then the block's words | a save slot's write, carried with its bytes |
| EVT | REPLY | reply words | the frame lane's captured replies |
| LANE | FRAME, `arg` = select and capture | the opcode word, then data words | one link transaction, as the LW frame lane plays it |
| PAD | STORE | opcode and index, then data | one pad set-command |

Block frames carry their LBA, so a host may queue blocks ahead of the core's requests; a wrong guess costs only the
stores.

## What a host must not assume

- that the mailbox exists because the RAM holds a plausible control page: only GPI bit 26 and the echo of a fresh
  nonce say so;
- that a record is seen before its head is stored, or that a head may be stored before its record;
- that a store has any effect without the run bit;
- that the run bit may stay set across a bridge reset or a core load;
- that a warm reset is safe before `warmramgrp.enable` is cleared;
- that the top 4 KB (`0xF000`-`0xFFFF`) is the mailbox's: it is never read or written;
- that the session survives an identification, a core reset through GPO, or an HPS reset;
- that an index it does not write holds still, or that an index it writes is read at once: the fabric polls.

## Timing rule

A core that builds an OCRAM tier ships only with a timing report in which no failing setup, hold, recovery or
removal path starts or ends in the master or in any `ocram_*` module. A failing path in the master can leave a
transaction unfinished, which is the danger the gate exists for.

## What is not verified

- **Nothing here has been built, fitted or run.** Every cost is an estimate.
- That the bridge reaches the on-chip RAM with secure, non-cacheable single-beat transactions. The HPS manual says
  the bridge has "full visibility into the HPS address space" and that the RAM is always in the peripheral region
  that every bridge master sees; the connectivity table itself was not checked.
- The atom's `port_size_config` value for the 32-bit width.
- The bridge's latency and throughput, and what its traffic costs other HPS masters.
- Whether a transaction in flight at a bridge reset can hold the HPS interconnect. The rule above assumes it can.
- The on-chip RAM's ECC state after boot. The protocol reads only words the other side has published, and stores
  full words only.
- That a warm reset boots once `warmramgrp.enable` is cleared.
- The GPO and GPI bit numbers are proposals.
