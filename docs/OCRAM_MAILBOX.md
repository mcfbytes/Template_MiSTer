# The OCRAM mailbox (proposed)

A proposed third path between the HPS and a core, beside the GPO/GPI ("SPI") link and the LW bridge window of
`docs/LW_BRIDGE.md`. It is for bulk: OSD rows, disk blocks, replies. The HPS writes records into its own 64 KB
**on-chip RAM** (OCRAM, physical `0xFFFF0000`), and the core fetches them as an AXI master on the Cyclone V
**FPGA-to-HPS bridge**, whose atom `sys/` already instantiates with no port connected.

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
| block memory per function | none | OSD 2, block reads 2-3, frame lane 3, pads 1 | none |
| logic | none new | a floor, then a block per function | a second floor, then a consumer per function |
| an access nobody answers | cannot happen | hangs the SoC, hence the firewall and the host rules | cannot happen to the HPS |
| needs from the host | nothing | discovery, the enable, per-block protocol | discovery, a hello, records |

The mailbox is not free. Its floor (the bridge master and its control logic) is logic a core pays once, beside the
LW floor if it builds both. For a core whose only bulk function is the OSD, the LW OSDBUF block is smaller in logic
and costs two block memories; the mailbox costs more logic and none. The mailbox pays for a core with two or more
bulk functions, for a core that cannot spare block memory, and for a core that builds no LW bridge at all. Where a
free port to the HPS's SDRAM exists, a shared DDR region moves megabytes faster than either.

## The menu

Every path is its own macro, off by default; a path that is not defined builds no logic. An OCRAM tier needs the
OCRAM floor, as an LW tier needs `MISTER_LW`. Naming two paths for one function builds both, and the host binds one.
The two floors share the crossing cells of `sys/lw_cdc.sv` and the HPS reset that `sysmem` exports, which
`MISTER_LW` alone builds today; the floor's change widens both to either floor.

| macro (proposed) | builds | needs | cost (ESTIMATED, by analogy with the fitted LW blocks less their FIFOs) |
|---|---|---|---|
| `MISTER_OCRAM` | the floor: the bridge master, its arbiter and poller, the hello, one GPI bit | - | 180-260 ALMs, no block memory |
| `MISTER_OCRAM_OSD` | OSD rows from the OSD ring into each `osd`'s `osd_buffer` | the floor | 60-80 ALMs |
| `MISTER_OCRAM_SECTOR_SLOT` and its geometry | block read frames for one `hps_io` slot whose blocks the core can ask for again (a disc); its requests as records | the floor | 180-230 ALMs |
| `MISTER_OCRAM_EVT` | the status back-channel and `hps_io`'s flags as records | the floor | 70-120 ALMs |
| `MISTER_OCRAM_LANE` | link frames from a ring into the frame-lane player; replies as records | `MISTER_OCRAM_EVT` | 130-180 ALMs |
| `MISTER_OCRAM_PAD` | pad stores from a ring | the floor | 40-60 ALMs |
| `MISTER_OCRAM_CONFSTR` | the configuration string as one record after the hello | `MISTER_OCRAM_EVT` | 30-50 ALMs |
| `MISTER_OCRAM_BELL` | an LW register whose store starts a poll at once (only if the poll proves too slow) | both floors | 10-20 ALMs |
| `MISTER_OCRAM_POLL` | the idle poll period, as a power of two of `clk_100m` cycles (default 12: 41 us) | the floor | - |

**Three set menus** a `.qsf` could carry as commented blocks:

| set | macros | for |
|---|---|---|
| LW pads, OCRAM bulk | `MISTER_LW`, `MISTER_LW_PAD`, `MISTER_LW_EVT`, `MISTER_OCRAM`, `MISTER_OCRAM_OSD`; a disc core adds `MISTER_OCRAM_SECTOR_SLOT` and keeps its save slots on SPI | a core with a disc, or two bulk functions: pads keep the path proven on hardware, the blocks that cost block memory move |
| lean, no LW bridge | `MISTER_OCRAM`, `MISTER_OCRAM_OSD`, `MISTER_OCRAM_EVT`; a disc core adds `MISTER_OCRAM_SECTOR_SLOT`; pads stay on SPI or take `MISTER_OCRAM_PAD` | a core short of logic or block memory, or one that wants no bridge the HPS must be answered on |
| LW only | the tiers of `docs/LW_BRIDGE.md` | a cartridge core; any core until the mailbox is proven |

## How it works

### The master

One module wires the existing `cyclonev_hps_interface_fpga2hps` atom at its 32-bit width and is the only logic that
drives it.

- **It runs on `clk_100m`** (`h2f_user0_clk`), and no fabric reset holds it. It never abandons a transaction it has
  started: a core reset, a stopped `clk_sys` or a host that has gone cannot leave one in flight.
- **It cannot address anything but the mailbox.** Address bits 31:16 are the constant `0xFFFF`, so it reaches the
  on-chip RAM and nothing else; through this bridge a fabric master could otherwise reach HPS memory and peripherals.
  Inside the RAM every address is a constant base plus an index masked to its ring's size, and the sum is below
  `0xF000` for every ring. Both bounds are structural and checked statically, never a matter of register values.
- **One transaction at a time,** single-beat, full 32-bit words, secure and non-cacheable.
- **It is gated by a run latch.** The latch sets on a rising edge of the host's run bit (proposed: GPO bit 28) seen
  while the link is in its I/O view (`gp_out[31]` = 1) and no HPS reset is seen. It clears when the run bit falls,
  when `gp_out[31]` falls or on an HPS reset, and the return of `gp_out[31]` does not set it. The master starts a
  transaction only while the latch is set. A host that does not know the mailbox never sets the run bit (the stock
  Main writes GPO bits 16 and 21-28 as zeros), so under such a host the master issues nothing at all and the core is
  a stock core.
- **When the run bit or `gp_out[31]` clears the latch** it completes the transaction in flight, stores C_HELLO =
  `0x51540000` as its one last transaction, and issues nothing more until the next rising edge of the run bit.
- **One arbiter, fixed order:** that last store; an interrupt line's tail re-read and the poll when due; the
  core-to-host ring's words; then the consumers, round robin.

Consumers run on `clk_sys`. Each fetches one word at a time through one held-data handshake built from the cells of
`sys/lw_cdc.sv`, so no crossing carries a clock ratio and none reaches a video clock. That is why no FIFO is needed:
the RAM is the queue, and the consumer's own pace sets the fetch rate.

### The map

Offsets are from `0xFFFF0000`. The fabric declares this geometry in the control page and a host reads it there, so a
later layout needs no host change.

| offset | size | ring | *r* | direction | carries |
|---|---|---|---|---|---|
| `0x0000` | `0x0100` | control page | - | both | the hello, binds, every index, the fabric's descriptors |
| `0x0400` | `0x0400` | PAD | 0 | host to core | pad stores |
| `0x1000` | `0x1000` | LANE | 1 | host to core | link frames |
| `0x2000` | `0x2000` | OSD | 2 | host to core | image rows, 65 words each |
| `0x4000` | `0x2000` | EVT | 3 | core to host | status, block requests, replies |
| `0x7000` | `0x8000` | SECTOR | 4 | host to core | block read frames: thirteen of 2,352 bytes |
| `0xF000` | `0x1000` | never touched | - | - | the boot ROM's workspace |

*r* is the ring's fixed number in every per-ring word of the control page. A ring whose tier is not built has no
descriptor, and its span is never read or written.

### Rings and records

- **A ring** is a power-of-two count of 32-bit words with two indices, each a free-running word count modulo 65,536
  and each a full word in the control page. **Each index is written by one side only:** the producer's head, the
  consumer's tail. `(head - tail) & 0xFFFF` words are waiting, and the ring is full when that equals its size.
- **A record** is one header word, `{kind[31:24], arg[23:16], len[15:0]}`, then `len` payload words. It may wrap
  the end of its ring.
- **Publishing is one store.** The producer stores every word of the record, then its head. The consumer acts only on
  records wholly below the head, so a producer that stops mid-record has published nothing. There is no partial
  frame, and so no resend and no drop count.
- **Flow control is the reader's tail.** A producer reads the other side's tail and stores only what fits. Neither
  side waits inside the protocol: a host keeps what did not fit for its next pass, and the fabric holds a request
  standing or coalesces a status to the latest.
- A consumer skips a kind it does not know by its length.

### The control page

Each word has one writer. In a host of several threads, H_SECTOR and each H_IDX belong to the thread that serves
that ring. A host stores the fabric's half only to zero it, and only while the master is stopped ("The rule that
replaces the firewall").

| offset | word | written by | content |
|---|---|---|---|
| `0x00` | H_HELLO | host | `{0x4F43, nonce[15:0]}`, the nonce never 0; 0 = no session. Stored last |
| `0x04` | H_BIND | host | the functions the host takes from SPI for the whole session: EVT `[8]`, LANE `[9]`, PAD `[10]`, OSD `[11]` |
| `0x08` | H_LINE | host | bit *r*: ring *r*'s interrupt line may rise |
| `0x0C` | H_SECTOR | host | bit 0: the SECTOR slot is served here. It may change within a session ("Records") |
| `0x10` + 4*r* | H_IDX[*r*] | host | its head of a host-to-core ring, its tail of a core-to-host ring |
| `0x80` | C_HELLO | fabric | the master's state, below; in a session, the echo of H_HELLO, stored last |
| `0x84` | C_VERSION | fabric | `{major, minor}` of the layout |
| `0x88` | C_CAPS | fabric | `[4:0]` ring *r* is built; `[8]` the bell register; `[9]` a configuration-string record follows the hello; the rest 0 |
| `0x8C` | C_STATE | fabric | `[7:0]` hellos taken since configuration; `[12:8]` ring *r* is stopped on a fault; `[20:16]` ring *r* ended the session before this one mid-record |
| `0x90` + 4*r* | C_IDX[*r*] | fabric | its tail of a host-to-core ring, its head of a core-to-host ring |
| `0xA4` | C_BIND | fabric | the binds in effect: H_BIND's bits, and bit 0 = the SECTOR slot's mux is on the mailbox side. Stored at every change; a bind or its drop has taken effect when this word shows it |
| `0xB0` + 4*r* | C_RING[*r*] | fabric | `{kind[31:27], line ordinal[26:22], log2 words[21:18], 0[17:14], offset in words[13:0]}`; kind is the function's kind in the LW directory (PAD 1, SECTOR 2, OSD 3, EVT 4, LANE 5); 0 = absent |
| `0xD0` + 4*n* | C_K[*n*], *n* 0-11 | fabric | `[0..7]` SECTOR: the SECTOR block's K0-K7 for its one slot, with K0's FIFO field 0 and K2 = 1. `[8]` OSD: OSDBUF's K0, `{rows, row bytes}`. `[9..11]` 0 |

| C_HELLO | the master is | stored by |
|---|---|---|
| 0 | unknown: no master has stored since the host's zero | the host, before every rising edge of the run bit |
| `0x4F430000` | running, armed, in no session | the master, on reading H_HELLO = 0 |
| `0x57540000` | running, waiting for H_HELLO = 0 | the master, on reading any other H_HELLO it has not echoed |
| `{0x4F43, nonce}` | in the session of that hello | the master, last of the hello's stores |
| `0x51540000` | stopped | the master, as the last store of a run |

## The host's side

### Mapping

Map `0xFFFF0000`, 64 KB, **uncached and strongly ordered**: on ARMv7 Linux an `mmap` of `/dev/mem` over this range
is exactly that, because the range is not kernel RAM. Each store is then one bus write that completes before the
next begins; nothing merges, repeats or reorders; and no cache on either side holds a copy. There is nothing to
flush and no barrier to issue. A cached mapping is not legal. Nor is a write-combined one as this guide stands: its
stores would need an ordering step before each publish that nobody has measured.

### Before the first store: the warm-boot rule

After boot the on-chip RAM still holds the preloader, and a current U-Boot arms the boot ROM to run that image again
on a warm reset (the system manager's `romcodegrp.warmramgrp.enable`, `0xFFD080E0`, holds `0xAE9EFEBC`). The mailbox
overwrites the image. **So a host stores 0 to that register before its first store to the on-chip RAM**; a warm
reset then takes the boot ROM's path through the boot medium, as a cold boot does. Whether that path boots a given
board after a warm reset is not verified here. A host that skips the store risks a machine that does not come back
from a warm reset; a host that makes it must know how its board behaves. If that path proves unusable, the map
moves above the preloader's image and the register is left alone; a project then builds every core and every host
for that one map, because nothing a host reads before its first store tells two maps apart.

### Discovery and the hello

Reading the on-chip RAM can never hang, so nothing here needs a firewall. What a host needs is proof that a live
master in *this* bitstream is on the other side: the RAM outlives every core load and still holds the last core's
words.

1. **GPI bit 26 = 1** (proposed), **sampled only in the I/O view** (`gp_out[31] = 1`): the master is in this
   bitstream. In the ID view the bit belongs to the core magic and reads 1 on every bitstream. Stock bitstreams read
   0 in the I/O view, and a host that sees 0 touches nothing: not the RAM, not the register above, not the run bit.
2. With the master stopped (proven by the rule below, or not yet started on this bitstream): store 0 to
   `warmramgrp.enable` and zero the whole control page with full-word stores. Then make a rising edge on the run
   bit: store it as 0, then as 1.
3. The edge starts the master, and its first transaction reads H_HELLO. **It arms only on reading H_HELLO = 0,** and
   says so by storing C_HELLO = `0x4F430000`; on any other value it stores `0x57540000` and polls on. A page that
   an earlier session left holding a plausible hello therefore starts nothing. Read the armed value within 1 ms of
   the edge: the run bit reached the fabric and a live master is polling.
4. Store H_HELLO with a non-zero nonce. The armed master waits until every consumer's `clk_sys` half is at rest,
   starts every index at 0, stores its version, capabilities, state, descriptors and constants, then C_HELLO =
   H_HELLO.
5. Read C_HELLO until it equals H_HELLO, for at most 1 ms. No armed value or no echo: no mailbox this session; clear
   the run bit, take the stopped read of the rule below, and use the LW window or SPI. An echo: read the descriptors
   and bind each function once. A bind is in effect when C_BIND shows it.

**The session ends** when the master stops (the run bit falls, `gp_out[31]` falls for the ID view or the core reset
through GPO, or the fabric sees an HPS reset), or when it reads an H_HELLO that is not the one it echoed: a 0 arms it
again, any other value leaves it waiting for a 0. Every bind and line drops, each function returns to SPI, and
records still queued are abandoned unplayed: a queued OSD row never draws into the next program's OSD, and a block
frame in flight is zero-filled from the first word not fetched, so the core is never left waiting. After its own
core reset or ID view a host starts again from step 2. A host that stops leaves the run bit set and the master
polling a page nobody changes, which is harmless; the next program's first GPO store carries the run bit as 0, which
stops the master.

**One program has the link,** as with the LW window. Stores carry no identity, so a second host that stores into a
live session corrupts its rings. A program's start stops the master its predecessor left running; two programs live
at once are as undetected here as they are on the SPI link.

### The rule that replaces the firewall

The mirrored danger is **a fabric transaction in flight while the bridge is reset or the FPGA is programmed.**

- **Raise the run bit only while the FPGA-to-HPS bridge is out of reset,** and hold it until the armed read.
- **Before resetting a bridge, programming the FPGA or zeroing the control page: clear the run bit or `gp_out[31]`,
  then read C_HELLO until it is `0x51540000`.** The host stored 0 there before the edge, and nothing the master
  stores there while it runs is 0 or that value, so the value is the last store of this run and nothing older: a
  transition, not a level. The wait is a few bus cycles; bound it at 1 ms.
- **If the value never comes,** the master is absent, was never started or cannot finish a transaction. A host
  cannot repair any of those: it notes the fault, makes no rising edge again on this bitstream, and goes on.
- **Before the first rising edge on a bitstream** there is no run to prove stopped: the latch is clear from
  configuration. A program that starts on a bitstream it did not load relies on its own first GPO store, which
  carried the run bit as 0, and zeroes the page no sooner than 1 ms after it.
- The store that drops `gp_out[31]` for a core reset carries the run bit as 0.

### Doorbells

- **Host to core: none.** The master polls the host half every `2^MISTER_OCRAM_POLL` cycles, and back to back only
  while a record can be consumed or a line is high. The worst case, 41 us by default, is invisible to an OSD row
  and small beside a block's service time.
- **Core to host: one level line per ring that needs one,** declared in the ring's descriptor with the ordinals of
  `docs/LW_BRIDGE.md` ("Interrupt lines"). It is high while `C_IDX != H_IDX`, and while it is high the master
  re-reads the host's tail back to back, so the line falls within a few transactions of the store that empties the
  ring. A host that re-arms sooner sees one more interrupt and an empty ring; its drain must take one empty pass.
  A host with no interrupt node reads C_IDX from its loop instead: one memory read.

### Records

| ring | kind | payload | meaning |
|---|---|---|---|
| OSD | `0x01` ROW, `arg` = row | 64 words: the row's 256 bytes | written into `osd_buffer` as the LW OSDBUF writer does, under the same guard |
| SECTOR | `0x10` FRAME, `arg` = slot index | the LBA, then the frame's words | played into the slot if it answers the core's standing request |
| EVT | `0x20` STATUS | four words | a status transfer landed, whole |
| EVT | `0x21` REQUEST, `arg` = slot index | the LBA | the bound slot's request opened and no queued frame answers it |
| EVT | `0x22` REPLY | reply words | the frame lane's captured replies |
| EVT | `0x23` MARK, `arg` = ring | its deepest fill | a ring's high-water mark, for sizing |
| EVT | `0x24` CONFSTR | the string, four bytes a word | the configuration string, once after the hello |
| LANE | `0x30` LFRAME, `arg` = select and capture | the opcode word, then data words | one link transaction, as the LW frame lane plays it |
| PAD | `0x40` STORE, `arg` = pad | two words | one pad set-command |

- **OSD.** A byte in rows 8-15 sets highres, as an LW or SPI write does, so a host sends an SPI OSD control word
  only when the OSD ring is drained (`C_IDX` equals its head), and binds the OSD before its first row. There is no
  atomic flip: a repaint is visible as it lands.
- **Block frames** carry their slot and LBA. While no request stands, the frame at the ring's tail waits. When one
  stands, the fabric plays the tail frame if it answers it and discards it if it does not, and posts a REQUEST when
  the ring holds no answer. So a host may queue blocks ahead of the core's requests, and a wrong guess costs only
  the stores.
- **The block slot's handover is the SECTOR block's own.** The tier is a second feeder behind `sys/lw_blk_mux.sv`
  and `hps_io`'s commit mark, which then build for either path. So the slot's mux moves only between requests, moves
  to the mailbox side only while the commit mark reads 0, and one request is never shown on both paths
  (`docs/LW_BRIDGE.md`, "One request, one path"). H_SECTOR bit 0 asks for the slot and C_BIND bit 0 is the mux:
  publish frames only while it reads 1, and serve the slot over SPI, from GET_SDSTAT, while it reads 0.
- **Changing what the slot holds** (a mount, a disc change): clear H_SECTOR's bit. A frame already playing
  finishes, every frame behind it is discarded, and the mux returns to SPI at the slot's next idle point. Wait for
  C_BIND, serve SPI meanwhile, then set the bit again. No stale frame survives and no request is left unanswered.
- **Save slots are not carried.** A save slot's read must reach the core whole or not at all, which the SECTOR
  block keeps by holding the frame in its FIFO; a consumer with no FIFO cannot withdraw a frame that the session's
  end cuts short. Save slots stay on SPI, or on the SECTOR block of `docs/LW_BRIDGE.md` where that is built.

## What a host must not assume

- that the mailbox exists because the RAM holds a plausible control page: only GPI bit 26, the armed value and the
  echo of its own nonce say so;
- that a record is seen before its head is stored, or that a head may be stored before its record;
- that a store has any effect without the run bit, or that the run bit's level starts anything: only its rising
  edge does;
- that a zero in C_HELLO means the master is stopped: only `0x51540000` after the host's own zero does;
- that the run bit may stay set across a bridge reset or a core load;
- that a warm reset is safe before `warmramgrp.enable` is cleared, or that it boots after;
- that the top 4 KB (`0xF000`-`0xFFFF`) is the mailbox's: it is never read or written;
- that the session survives an identification, a core reset through GPO, or an HPS reset;
- that a bind is in effect because it was stored: C_BIND says when;
- that a save slot can be served here;
- that an index it does not write holds still, or that an index it writes is read at once: the fabric polls;
- that an interrupt means the ring holds a record: one may arrive after the drain.

## Timing rule

A core that builds an OCRAM tier ships only with a timing report in which no failing setup, hold, recovery or
removal path starts or ends in the master or in any `ocram_*` module. A failing path in the master can leave a
transaction unfinished, which is the danger the gate exists for.

## What is not verified

- **Nothing here has been built, fitted or run.** Every cost is an estimate.
- That the bridge reaches the on-chip RAM with secure, non-cacheable single-beat transactions. The HPS manual says
  the bridge has "full visibility into the HPS address space" and that the RAM is always in the peripheral region
  that every bridge master sees; the connectivity table itself was not checked, and U-Boot marks the RAM non-secure.
- The atom's `port_size_config` value for the 32-bit width.
- The bridge's latency and throughput, and what its traffic costs other HPS masters.
- Whether a transaction in flight at a bridge reset can hold the HPS interconnect. The rule above assumes it can.
- The on-chip RAM's ECC state after boot. The protocol reads only words the other side has published, and stores
  full words only.
- That a warm reset boots once `warmramgrp.enable` is cleared.
- The GPO and GPI bit numbers are proposals, and the static checks of `tb/` that pin today's GPI terms and interrupt
  lines change with the floor.
