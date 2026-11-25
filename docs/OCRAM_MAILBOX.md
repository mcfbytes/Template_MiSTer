# The OCRAM mailbox (proposed)

A proposed third path between the HPS and a core, beside the GPO/GPI ("SPI") link and the LW bridge window of
`docs/LW_BRIDGE.md`. It is for bulk: OSD rows, disk blocks, replies. The HPS writes records into its own 64 KB
**on-chip RAM** (OCRAM, physical `0xFFFF0000`), and the core fetches them as an AXI master on the Cyclone V
**FPGA-to-HPS bridge**, whose atom `sys/` already instantiates with no port connected.

**Status: the floor, the OSD tier, the SECTOR tier and the download tier are built, for test cores only.** `MISTER_OCRAM` builds the
bridge master, the run latch, the poller, the hello, the EVT ring's writer and its line, and the consumer handshake.
`MISTER_OCRAM_OSD` and `MISTER_OCRAM_SECTOR_SLOT` each build their half of `sys/hps_ocram_rx.sv`, one walker for both
host-to-core rings: the OSD half writes the OSD ring's rows into every `osd`'s `osd_buffer` through the LW OSDBUF
writer's port, and the SECTOR half plays block frames into one `hps_io` slot and posts its REQUEST records ("The
SECTOR tier as built"). `MISTER_OCRAM_FIO` adds a third host-to-core ring to that walker, ring 1: link frames
played into `hps_io`'s download block through `lw_link`, with a mark line on `f2h_irq[12]` ("The download tier as
built"). `MISTER_OCRAM_MOUNT0_SLOT` and `MISTER_OCRAM_MOUNT1_SLOT` add two mount slots beside the SECTOR slot: their
reads from ring 0 and their writes as EVT records ("The mount slots as built"). `MISTER_OCRAM_DIAG` adds the diagnostic MARK records, the latency counters, the GPO 27/26
test knobs, the GPO 25 freeze knob and the host-to-fabric torture sink (below); it cannot be built with the SECTOR
tier. Nothing else in this document is built: no EVT, LANE or PAD tier. The SECTOR, download and mount tiers are benched, not yet fitted
or run on hardware; several facts the floor rests on are unverified ("What is not verified", at the end). It is published with the LW framework so that
core and host authors can read both paths side by side and say which they would build.

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
| `MISTER_OCRAM` | the floor: the bridge master, its arbiter and poller, the hello, the core-to-host ring's writer and its interrupt line, one GPI bit | - | 180-260 ALMs, no block memory |
| `MISTER_OCRAM_OSD` | OSD rows from the OSD ring into each `osd`'s `osd_buffer`, through the LW OSDBUF writer's port | the floor | 60-80 ALMs |
| `MISTER_OCRAM_SECTOR_SLOT` and its geometry | block read frames for one `hps_io` slot whose blocks the core can ask for again (a disc); its requests as records | the floor and `MISTER_LW` (it drives the LW sector bits of `HPS_BUS`); not `MISTER_OCRAM_DIAG`, not `MISTER_LW_SECTOR_SLOT` | about 130 ALMs beside the OSD tier, about 250 alone |
| `MISTER_OCRAM_FIO` (PROVISIONAL), `MISTER_OCRAM_FIO_GAP` (4-7, default 4), `MISTER_OCRAM_FIO_RING` (log2 words, 2-10 or 13, default 10) | file downloads: link frames from ring 1 into `hps_io`'s download block through `lw_link`, and ring 1's mark line | the floor; not `MISTER_LW_LANE` (`link_on_two_players`) | 110-150 ALMs (ESTIMATED) |
| `MISTER_OCRAM_UIO` (PROVISIONAL) | write-only link frames on the UIO select (0) and both OSD selects (2 HDMI, 3 VGA) from ring 1, played by the download tier's player into `hps_io`'s, `sys_top`'s and each `osd`'s own decoders; declared by C_K[9]'s select mask, no C_CAPS bit | `MISTER_OCRAM_FIO` (`ocram_uio_needs_MISTER_OCRAM_FIO`) | about 10 ALMs (ESTIMATED) |
| `MISTER_OCRAM_MOUNT0_SLOT`, `MISTER_OCRAM_MOUNT1_SLOT`, `MISTER_OCRAM_MOUNT_FRAME` (1-1,020, default 128) (PROVISIONAL) | two mount slots (block slot index 1, 2) through `lw_save_mux`: reads from ring 0, writes as WRITE records | `MISTER_OCRAM_SECTOR_SLOT` (`ocram_mount_needs_MISTER_OCRAM_SECTOR_SLOT`), so `MISTER_LW`; not `MISTER_LW_SECTOR_SLOT` | 195-205 ALMs (ESTIMATED) |
| `MISTER_DISABLE_SPI_OSD` (PROVISIONAL, test-only) | removes `osd.v`'s SPI row writes (`0x20`); declared by C_CAPS bit 17 | `MISTER_OCRAM_OSD` or `MISTER_LW_OSD` (`spi_osd_off_needs_an_osd_tier`) | -36 ALMs (ESTIMATED) |
| `MISTER_DISABLE_SPI_BLK` (PROVISIONAL, test-only), `MISTER_DISABLE_SPI_BLK_UNUSED` (a slot mask, default 0) | removes `hps_io`'s block arms (`0x16`-`0x18`) and ties the commit mark to 0; declared by C_CAPS bit 16 | every slot below VDNUM on the SECTOR or a mount slot, or in the mask, never both (`spi_blk_off_needs_a_block_tier`, `spi_blk_off_slot_without_a_tier`, `spi_blk_off_unused_slot_on_a_tier`); the LW SECTOR family is not counted | -60 ALMs (ESTIMATED) |
| `MISTER_OCRAM_EVT` (PROVISIONAL) | `hps_io`'s core->host flags as STATUS, INFO, UPLOAD and RUMBLE records on the EVT ring, bound by H_BIND bit 8; declared by C_CAPS bit 11 | `MISTER_OCRAM_SECTOR_SLOT` (`ocram_evt_needs_MISTER_OCRAM_SECTOR_SLOT`), `MISTER_LW` (`ocram_evt_needs_MISTER_LW`), never `MISTER_LW_EVT` (`evt_on_two_tiers`) | 90-110 ALMs (ESTIMATED) |
| `MISTER_DISABLE_SPI_EVT` (PROVISIONAL, test-only) | removes `hps_io`'s event replies (`0x29`, `0x36`, `0x3C`, `0x?3F`); declared by C_CAPS bit 18 | `MISTER_OCRAM_EVT` (`spi_evt_off_needs_MISTER_OCRAM_EVT`) | -40 ALMs (ESTIMATED) |
| `MISTER_OCRAM_LANE` (PROVISIONAL) | capture LFRAMEs on ring 1: the download tier's player samples each link word's reply and the master posts one REPLY record a frame on the EVT ring; declared by C_CAPS bit 12 | `MISTER_OCRAM_FIO` (`ocram_lane_needs_MISTER_OCRAM_FIO`) and `MISTER_OCRAM_SECTOR_SLOT` (`ocram_lane_needs_MISTER_OCRAM_SECTOR_SLOT`); not `MISTER_OCRAM_EVT` | 60-80 ALMs (ESTIMATED) |
| `MISTER_OCRAM_PAD` | pad stores from a ring | the floor | 40-60 ALMs |
| `MISTER_OCRAM_CONFSTR` | the configuration string as one record after the hello | `MISTER_OCRAM_EVT` | 30-50 ALMs |
| `MISTER_OCRAM_BELL` | an LW register whose store starts a poll at once (only if the poll proves too slow) | both floors | 10-20 ALMs |
| `MISTER_OCRAM_POLL` | the idle poll period, as a power of two of `clk_100m` cycles (default 12: 41 us) | the floor | - |
| `MISTER_OCRAM_DIAG` | test-core diagnostics: MARK records with the latency counters (below), and the GPO 27/26 knobs | the floor | test cores only |

**The set menu.** Move bulk to the mailbox and pads to the LW window; leave status and events on the link unless a
measurement says otherwise, because each of them costs more fabric off the link than on it. Every cost is
ESTIMATED unless marked fitted.

| set | macros | status, events, saves | cost beyond stock | for |
|---|---|---|---|---|
| **default, a disc core: LW pads, OCRAM bulk, SPI status** | `MISTER_LW`, `MISTER_LW_PAD`, `MISTER_OCRAM`, `MISTER_OCRAM_OSD`, `MISTER_OCRAM_SECTOR_SLOT` and its geometry | all on SPI, unchanged | LW floor and pads about 310 ALMs + 1 M10K; mailbox about 530 ALMs, 0 M10K (ESTIMATED) | a core with a disc and little room: pads keep the proven low-latency path, bulk costs no block memory |
| add events on LW | the default plus `MISTER_LW_EVT` | status and flags on an LW line; saves on SPI | about +120 ALMs (104 fitted) | a core whose status or block-request latency is measured to matter, and that has the room |
| LW only | the tiers of `docs/LW_BRIDGE.md`, save slots included | LW | about 1,065 ALMs + 5 M10K (fitted, a disc core's build) | a cartridge core; a core that wants save blocks off SPI |
| lean, no LW bridge | `MISTER_OCRAM`, `MISTER_OCRAM_OSD`; pads on SPI | SPI | about 160 + 160 ALMs, 0 M10K (ESTIMATED) | a core that wants no bridge the HPS must be answered on. A disc core cannot take it yet: the SECTOR tier needs `MISTER_LW` |
| area-critical extras | `MISTER_DISABLE_SPI_PAD` (needs `MISTER_LW_PAD`); `CONF_STR_BRAM(1)` | — | about -85 ALMs (ESTIMATED; a stock Main loses the pads); about -350 ALMs for 4 M10K (ESTIMATED) | only when the fit needs it; never a default |

## Test-core diagnostics

`MISTER_OCRAM_DIAG` is a test-core build only: it posts a MARK record (`kind 0x23`, `arg` 3, 40 payload words) on
the EVT ring in a session whenever the ring has room and either flood is set or the MARK period
(`MISTER_OCRAM_MARK`, a power of two of `clk_100m` cycles) has passed. Word 0 carries the ring's deepest fill,
the two knobs and the hello count; words 1-7 the read and write counts, max/min latency, latency sums and error
counts; words 8-39 the two 16-bin latency histograms. The MARK record's own header and payload stores are not
counted, and every counter clears at each hello.

Two GPO bits are test knobs, sampled at the run latch's set. **GPO 27 = 1 sets `AxPROT[1]` (non-secure) for that
run.** **GPO 26 = 1 sets flood, so MARK records are posted back to back.** Neither is a protocol bit, and a
shipping build (`MISTER_OCRAM` without `MISTER_OCRAM_DIAG`) ignores both. Flood starves the consumers by design:
a back-to-back MARK stream keeps the arbiter's EVT-record slot busy, so a consumer waits behind it.

The same build carries the host-to-fabric torture sink: consumer port 1 walks the SECTOR span as a ring (r = 4,
base `0x7000`, 8,192 words) in every session, with no bind, and checks each TSEQ record's payload word `g`
against `g ^ {g[15:0], g[31:16]} ^ 0x9E3779B9` and each record's `seq` against its number since the hello. Its
counters, and the OSD walker's finished-row count and deepest fill, fill a VERDICT record (`kind 0x25`, `arg` 4,
8 payload words) posted on the MARK period right after each MARK; flood posts none. A ring fault is C_STATE bit
12, a session cut mid-record bit 20.

With `MISTER_OCRAM_OSD`, GPO 25 is a freeze knob that models a stopped `clk_sys`: while it is 1 the OSD writer
latches a word but writes no byte, launches no done and reports not at rest, so a hello waits unechoed until it
is cleared. Without `MISTER_OCRAM_DIAG` GPO 25 is never read.

The host side of all of this is `tb/hps/ocram_probe.c` (usage in its header; `tb/hps/kmod/ocram_wc.c` is the
write-combined map's test module). `torture --map <m>` runs the ring full, as fast as the fabric consumes.
`torture --map <m> --band` holds it part-full: each cycle reads C_IDX[4] once and asserts the fill is below 8,192,
publishes records back to back, with no read of any mailbox word, until a clock-based estimate of the fill reaches a
random HI (50-90% of the ring), then waits by the clock, touching no mailbox word, until the estimate falls to a
random LO (5-45%). The estimate runs at `--rate` words/s (default 6.3 M, re-fitted from the tail's travel while the
ring never runs empty). Every 2^20 words (`--empty-words`) the ring runs empty and the VERDICT must equal the host's
records, words and sum, with 0 errors and a deepest fill below 8,192. The last line is PASS or FAIL, with the lowest
fill read and the fabric's deepest; it fails also if more than 5% of the reads found the ring empty or above 60%
(it was not part-full). A record that became visible before its payload shows as a payload or seq error in the
VERDICT. `tb/run.sh` arm H runs the probe's logic against a model fabric, `tb/hps/ocram_probe_fake.h`.

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
  core-to-host ring's words and the fabric's own control words; then the consumers, round robin.

Consumers run on `clk_sys`. Each fetches one word at a time through one held-data handshake built from the cells of
`sys/lw_cdc.sv`, so no crossing carries a clock ratio and none reaches a video clock. That is why no FIFO is needed:
the RAM is the queue, and the consumer's own pace sets the fetch rate.

### The map

Offsets are from `0xFFFF0000`. The fabric declares this geometry in the control page and a host reads it there, so a
later layout needs no host change.

| offset | size | ring | *r* | direction | carries |
|---|---|---|---|---|---|
| `0x0000` | `0x0100` | control page | - | both | the hello, binds, every index, the fabric's descriptors |
| `0x0400` | `0x0400` | PAD | 0 | host to core | pad stores (not built; never in a build with the mount ring) |
| `0x1000` | `0x1000` | LANE | 1 | host to core | link frames |
| `0x2000` | `0x2000` | OSD | 2 | host to core | image rows, 65 words each |
| `0x4000` | `0x2000` | EVT | 3 | core to host | status, block requests, writes, binds, replies |
| `0x6000` | `0x1000` | mount | 0 | host to core | the mount slots' block frames, 1,024 words (PROVISIONAL) |
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
| `0x0C` | H_SECTOR | host | bit 0: the SECTOR slot is served here; bits 1, 2: the mount slots k = 1, 2 are (PROVISIONAL). Each may change within a session ("Records") |
| `0x10` + 4*r* | H_IDX[*r*] | host | its head of a host-to-core ring, its tail of a core-to-host ring |
| `0x30` + 4*r* | H_MARK[*r*] | host | a host-to-core ring's mark, where its descriptor declares a line: `[31]` armed, `[15:0]` an index. Only H_MARK[1] (`0x34`) is built (PROVISIONAL) |
| `0x80` | C_HELLO | fabric | the master's state, below; in a session, the echo of H_HELLO, stored last |
| `0x84` | C_VERSION | fabric | `{major, minor}` of the layout |
| `0x88` | C_CAPS | fabric | `[4:0]` ring *r* is built; `[8]` the bell register; `[9]` a configuration-string record follows the hello; `[10]` the master posts a BIND record after every C_BIND store (PROVISIONAL); `[11]` the events tier's records (`MISTER_OCRAM_EVT`, PROVISIONAL); `[12]` capture LFRAMEs and REPLY (`MISTER_OCRAM_LANE`, PROVISIONAL); `[16]` the link's block arms are not built (`MISTER_DISABLE_SPI_BLK`), `[17]` nor its OSD row writes (`MISTER_DISABLE_SPI_OSD`), `[18]` nor its event replies (`MISTER_DISABLE_SPI_EVT`) (PROVISIONAL); the rest 0 |
| `0x8C` | C_STATE | fabric | `[7:0]` hellos taken since configuration; `[12:8]` ring *r* is stopped on a fault; `[20:16]` ring *r* ended the session before this one mid-record |
| `0x90` + 4*r* | C_IDX[*r*] | fabric | its tail of a host-to-core ring, its head of a core-to-host ring |
| `0xA4` | C_BIND | fabric | the binds in effect: H_BIND's bits, bit 0 = the SECTOR slot's mux is on the mailbox side, bits 1, 2 = each mount slot's is (PROVISIONAL). Stored at every change; a bind or its drop has taken effect when this word shows it |
| `0xB0` + 4*r* | C_RING[*r*] | fabric | `{kind[31:27], line ordinal[26:22], log2 words[21:18], 0[17:14], offset in words[13:0]}`; kind is the function's kind in the LW directory (PAD 1, SECTOR 2, OSD 3, EVT 4, LANE 5); 0 = absent |
| `0xD0` + 4*n* | C_K[*n*], *n* 0-11 | fabric | `[0..7]` SECTOR: the SECTOR block's K0-K7 for its one slot, with K0's FIFO field 0 and K2 = 1. `[8]` OSD: OSDBUF's K0, `{rows, row bytes}`. `[9]` the download tier: `{select mask[31:16], gap[15:0]}`, mask bit *s* set when an LFRAME may name `sel` *s*: `0x0002` (select 1, the file-I/O block), or with `MISTER_OCRAM_UIO` `0x000F` (also 0 UIO, 2 OSD on HDMI, 3 OSD on VGA; `0x000B` with `MISTER_DEBUG_NOHDMI`) (PROVISIONAL). `[10..11]` the mount slots k = 1, 2: `{RD 0x01[31:24], slot[19:16], frame words[12:0]}`, 0 for an undeclared slot (PROVISIONAL) |

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
   starts every index at 0, stores its version, capabilities, state, C_BIND = 0, descriptors and constants, then
   C_HELLO = H_HELLO.
5. Read C_HELLO until it equals H_HELLO, for at most 1 ms. No armed value or no echo: no mailbox this session; clear
   the run bit, take the stopped read of the rule below, and use the LW window or SPI. An echo: read the descriptors
   and bind each of H_BIND's functions once. The block slot's bind is H_SECTOR ("Records"). **A bind is in effect
   only while C_BIND shows it and C_HELLO still holds the echo:** the words of an ended session are stale, C_BIND
   among them.

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
- **The mark line: one per host-to-core ring that declares one.** It is high while H_MARK[*r*] is armed and the
  ring's tail has reached its index (`(C_IDX[r] - index) & 0x8000` is 0), and while it is high the master re-reads
  H_MARK[*r*] back to back, so it falls within a few transactions of a store that disarms it or moves the index
  ahead. Only ring 1 builds it: ordinal 5, `f2h_irq[12]`. A host with no node for it reads C_IDX[1] instead.

### Records

| ring | kind | payload | meaning |
|---|---|---|---|
| OSD | `0x01` ROW, `arg` = row | 64 words: the row's 256 bytes | written into `osd_buffer` as the LW OSDBUF writer does, under the same guard |
| SECTOR | `0x10` FRAME, `arg` = `{blk_cnt[5:0], k[1:0]}`, 0 | the LBA, then the frame's words | played into the slot if it answers the core's standing request |
| mount | `0x10` FRAME, `arg` = `{blk_cnt[5:0], k[1:0]}`, k 1 or 2 | the LBA, then the block's `(blk_cnt + 1) << (BLKSZ + 5)` words | played into slot k if it answers its standing read with that LBA and length (PROVISIONAL) |
| EVT | `0x20` STATUS, `arg` 0, `len` 4 | `status_req`, `[31:0]` first | a status transfer landed, whole (PROVISIONAL) |
| EVT | `0x28` INFO, `arg` = the info byte, `len` 0 | none | the core raised `info_req` (PROVISIONAL) |
| EVT | `0x29` UPLOAD, `arg` = `ioctl_upload_index`, `len` 0 | none | the core raised `ioctl_upload_req` (PROVISIONAL) |
| EVT | `0x2A` RUMBLE, `arg` = pad (0-5), `len` 1 | `{16'd0, level}` | a pad's rumble level differs from the last posted (PROVISIONAL) |
| EVT | `0x21` REQUEST, `arg` = `{blk_cnt[5:0], k[1:0]}` | the LBA | the bound slot's request stands and the ring holds no answer, including a request that already stood when the bind arrived |
| EVT | `0x26` WRITE, `arg` = `{blk_cnt[5:0], k[1:0]}`, `len` = 1 + n | the LBA, then the block's n words | mount slot k's write, read from the core's buffer; its head store is the publish (PROVISIONAL) |
| EVT | `0x27` BIND, `arg` = C_BIND[7:0], `len` 0 | none | posted after every C_BIND store (PROVISIONAL) |
| EVT | `0x22` REPLY, `arg` = the capture frame's `arg`, echoed whole, `len` = its `len` | word *i* = `{reply[2i+1], reply[2i]}`, low half first; past an odd frame's end the high half is 0 | one capture frame played; `reply[j]` is `io_dout \| io_dout_sys` at link word *j*'s acknowledge, *j* = 0 the opcode's (PROVISIONAL) |
| EVT | `0x23` MARK, `arg` = ring | its deepest fill | a ring's high-water mark, for sizing |
| EVT | `0x24` CONFSTR | the string, four bytes a word | the configuration string, once after the hello |
| LANE | `0x30` LFRAME, `arg` = `{odd[7], tag[6:3], capture[2], sel[1:0]}` | the opcode word, then data words, two a ring word | one link transaction on select `sel`, as the LW frame lane plays it; `tag` and `capture` 0 unless `MISTER_OCRAM_LANE` (PROVISIONAL) |
| PAD | `0x40` STORE, `arg` = pad | two words | one pad set-command |

- **OSD.** A byte in rows 8-15 sets highres, as an LW or SPI write does, so a host sends an SPI OSD control word
  only when the OSD ring is drained (`C_IDX` equals its head), and binds the OSD before its first row. There is no
  atomic flip: a repaint is visible as it lands.
- **FRAME, REQUEST and WRITE share one `arg`,** `{blk_cnt[5:0], k[1:0]}`. For k = 0 (the SECTOR slot) it is 0 in
  both directions: the frame's length is C_K[4]'s.
- **BIND `0x27`:** `arg` = C_BIND[7:0], no payload. Posted after every C_BIND store, under the rule that holds the
  bit-0 drop behind a record in flight. A REQUEST read before a BIND record that shows its slot's bit clear is void.
- **Capture (`MISTER_OCRAM_LANE`, PROVISIONAL).** An LFRAME with capture (`arg[2]`) takes any tag and a `len` of
  1-1,020; one without capture still needs tag 0. It is held, unconsumed and unplayed, until the EVT ring has
  `len + 1` words free, and the frames behind it on ring 1 wait with it while rings 0, 2 and 4 are served. Then
  its REPLY's header is written, the frame plays, and each ring word's two replies follow as payload: one REPLY a
  capture frame, in frame order. The record is open while the frame plays, so other records wait behind it; it
  goes after an owed BIND, REQUEST or WRITE and before the events tier's records, and needs no H_BIND bit. A build
  without the lane skips a capture frame by length. A host gates capture frames on C_CAPS bit 12.
- **The events tier's records (`MISTER_OCRAM_EVT`, PROVISIONAL).** Each cause is latched in `hps_io` on its edge,
  bound or not, as the link's `0x29`/`0x36`/`0x3C` replies latch it; a record is posted only while C_BIND bit 8 is
  stored 1, so a cause standing at the bind is its first record. STATUS is one transfer, whole: a later transfer
  replaces an unposted one, and a STATUS whose flag moves while it is written is abandoned (its head never stored)
  and the later one posted. A RUMBLE is posted while a pad's level differs from the last posted, which is 0 at each
  hello. They go behind an owed BIND, REQUEST or WRITE. The fabric reads one offer at a time from `hps_io`'s port
  (`hps_ocram_evt`, a 42-bit word), so a STATUS is four offers; an INFO or UPLOAD taken in the cycles before a
  session ends can be lost, and a host re-reads the status at each hello.
- **The mount slots** each keep the SECTOR slot's rule: a request that already stands when its bind arrives is that
  bind's first REQUEST. A host that has stopped polling GET_SDSTAT relies on it.
- **Ring 1:** no bind bit. A published LFRAME is played; nothing is taken from the link.
- **Block frames** carry their slot and LBA. While no request stands, the frame at the ring's tail waits. When one
  stands, the fabric plays the tail frame if it answers it and discards it if it does not, and posts a REQUEST when
  the ring holds no answer. So a host may queue blocks ahead of the core's requests, and a wrong guess costs only
  the stores.
- **The block slot's handover follows the SECTOR block's rules.** The slot's mux moves only between requests, moves
  to the mailbox side only while `hps_io`'s commit mark reads 0, and one request is never shown on both paths
  (`docs/LW_BRIDGE.md`, "One request, one path"). The rules and the parts are reused: the mailbox's feeder drives the
  same LW sector bits of the 384-bit `HPS_BUS` the SECTOR block drives, so the core hooks the slot through
  `lw_blk_mux` exactly as for the LW block, and the tier needs `MISTER_LW`. A build of both paths' SECTOR macros
  stops (`sector_slot_on_two_feeders`): one slot has one feeder. A plain-port mux, for a disc core with no LW floor,
  is not built; its trigger is the first such core.
- **H_SECTOR bit 0 asks for the slot, and C_BIND bit 0 says the mux is on the mailbox's feeder.** Publish frames
  only while H_SECTOR's bit is set, C_BIND's reads 1 and C_HELLO holds the echo; serve the slot over SPI, from
  GET_SDSTAT, whenever they do not.
- **A REQUEST record belongs to one bind.** The fabric stores C_BIND's bit as 1 before it posts that bind's first
  REQUEST, and a request that already stands when the mux arrives is that first record. It posts none after it has
  stored the bit as 0. Read C_BIND after a REQUEST you act on, never before. A REQUEST read while H_SECTOR's bit is
  clear is void. **Drain the core-to-host ring after reading C_BIND = 0 and before setting the bit again,** so every
  REQUEST of the bind that ended is read, and void, before the next bind can post one. Records of other kinds are
  handled as ever: only the slot's REQUESTs belong to a bind.
- **Never wait on C_BIND:** read it on each pass and take the path it shows. Serving SPI is what clears the commit
  mark, so a set bit that has not taken effect costs slow blocks, never a stall. Before setting the bit, serve over
  SPI what the last GET_SDSTAT showed.
- **Changing what the slot holds** (a mount, a disc change): clear H_SECTOR's bit. A frame already playing
  finishes, every frame behind it is discarded, and the mux returns to SPI at the slot's next idle point. A request
  that stood on the mailbox then shows in the SPI poll, and if it still stands at the next bind the fabric posts it
  again. Serve SPI; once C_BIND shows 0 or C_HELLO no longer holds the echo, drain the core-to-host ring, change
  the mount, then set the bit again. No stale frame survives and no request is left unanswered.
- **Mount slots are carried, with no hold and no staging.** A block the session's end cuts ends as a SECTOR frame
  does: a read is zero-filled from the cut, a WRITE is never published, and the core never waits. A published frame
  plays whole while RUN stays up.

### The SECTOR tier as built

`MISTER_OCRAM_SECTOR_SLOT` (the `hps_io` slot), `MISTER_OCRAM_SECTOR_FRAME` (a frame's 32-bit words, 1-4096,
default 128), `MISTER_OCRAM_SECTOR_GAP` (cycles between strobes, 2-15, default 2) and `MISTER_OCRAM_BLKSZ` (`hps_io`'s
BLKSZ, default 2) declare it; a 2,352-byte disc frame is 588 words at BLKSZ 3. A build stops naming its guard
without `MISTER_LW` (`ocram_sector_needs_MISTER_LW`), with `MISTER_OCRAM_DIAG` (`ocram_sector_excludes_MISTER_OCRAM_DIAG`:
the torture sink and the REQUEST writer share ring 4's span and the EVT writer), beside `MISTER_LW_SECTOR_SLOT`
(`sector_slot_on_two_feeders`), and at a frame or gap outside those ranges (`ocram_sector_unservable_geometry`).
The hello stores C_CAPS bit 4, C_RING[4] = `{kind 2, no line, 8,192 words, 0x7000}` and C_K[0, 1, 2, 4, 5]: K0
`{0, gap}`, K1 `{BLKSZ[10:8], wide[0]}`, K2 = 1, K4 `{RD 0x01, slot[19:16], frame words[12:0]}`, K5 = 32 (LBA bits);
K3, K6 and K7 read 0. The wire items (the kind, the K words, the REQUEST's `arg` 0) are PROVISIONAL.

- **The walker** (`clk_100m`) serves both host-to-core rings at record granularity, alternating when both have work.
  It reads H_SECTOR in the session poll. Bit 0 set raises the bind level to `clk_sys`; C_BIND bit 0 is stored as 1
  once the feeder's mux reads back on the mailbox.
- **A request stands** when its record `{offered, lba}` crosses from `clk_sys` with offered set. With none standing the
  tail frame waits and nothing is fetched. With one standing and the ring not empty, the walker reads the tail
  frame's header and LBA and crosses SEC_HEAD with the LBA; the feeder answers **accept** (the walker streams the
  frame's words, one crossing each, then stores the tail), **mismatch** (the tail moves past the frame) or **no
  request** (it fell in between: the frame stays, and the walker waits for a new record or the next poll).
- **A REQUEST is posted** (`{0x21, 0, 1}` then the LBA, behind a room check of two words) when a request stands, the
  ring is empty after a head read made since that request's record landed, and none is posted yet for it. The ring
  is read on the poll period, and back to back only while a record can be consumed.
- **H_SECTOR bit 0 cleared:** a frame being played finishes; the walker then reads H_IDX[4] and stores C_IDX[4] =
  that head, which discards every queued frame; only after that does the mux return to SPI and C_BIND bit 0 = 0 get
  stored. Every C_BIND store waits for a record in flight to finish, no REQUEST starts once a store is owed, and the
  store's BIND record (C_CAPS bit 10) is the next record posted.
- **A fault** on ring 4 (a head more than the ring ahead, a length past the head: C_STATE bit 12) or on ring 3 (C_STATE
  bit 11) drops the bind the same way, without the flush. A session that ends mid-record sets bit 20 at the next
  hello.
- **The feeder** (`clk_sys`) re-creates `hps_io`'s strobes as the SECTOR block's primary feeder does, cycle for cycle
  from the ack's rise (the trace-compare bench, arm C of `tb/run.sh`): with `io_wide` two 16-bit strobes a word, else
  four bytes, GAP apart, the first GAP after the ack. Each word is held in the crossing's capture register until its
  strobes have issued. It zero-fills the rest of a frame on the consumer's reset, or once the session's enable or the
  bind level has been low for two cycles while a word is awaited.

### The mount slots as built

`MISTER_OCRAM_MOUNT0_SLOT` and `MISTER_OCRAM_MOUNT1_SLOT` (each an `hps_io` slot, 0-15, neither the SECTOR slot nor the
other's) and `MISTER_OCRAM_MOUNT_FRAME` (the largest block in 32-bit words, 1-1,020, default 128) declare them; a
build stops on an unservable one (`ocram_mount_unservable_geometry`) and without `MISTER_OCRAM_SECTOR_SLOT`
(`ocram_mount_needs_MISTER_OCRAM_SECTOR_SLOT`), whose feeder, walker and record writer they share. The core hooks
each slot through `lw_save_mux`, its `core_din` the slot's `sd_buff_din`. The hello stores C_CAPS bit 0, C_IDX[0] = 0,
C_RING[0] = `{kind 2, no line, 1,024 words, 0x6000}` (`0x10281800`) and C_K[10, 11]. Every wire item here is
PROVISIONAL.

- **The binds:** H_SECTOR bit k asks for mount slot k and C_BIND bit k answers, under the SECTOR slot's rules, with
  no flush: no frame waits in ring 0.
- **A request** crosses from `clk_sys` as `{k, wr, blk_cnt, offered, lba}`, the two slots' records in turn; a block longer
  than `MISTER_OCRAM_MOUNT_FRAME` is never offered. The save-slot bits of `HPS_BUS` carry no `blk_cnt`, so the
  Template ties it to 0: one block a request.
- **A read** is posted at once as REQUEST `{0x21, {blk_cnt, k}, 1}` then the LBA. The host answers with one FRAME on
  ring 0; the feeder accepts it only if the request stands with that LBA and length, raises the slot's ack and plays
  `hps_io`'s strobes as for the SECTOR slot. Any other record on ring 0 is skipped by its length.
- **A write:** the feeder raises the slot's ack, steps the buffer address from 0 and reads each word back through
  `lw_save_mux`, whose LBA field carries the buffer word while the slot's mux and ack are up. The master posts WRITE
  `{0x26, {blk_cnt, k}, 1 + n}`, the LBA and the n words, behind a room check of n + 2 words; the ack falls once the
  head store has landed. With no room the WRITE waits, its ack up.
- **A WRITE holds the walker** from its accept to the ack's fall, so rings 1, 2 and 4 wait up to one block (1,020
  words at most) behind it.
- **The record writer** posts every record one at a time: a REQUEST or WRITE starts only while its slot's C_BIND
  bit is stored 1 and no C_BIND store or BIND is owed; a C_BIND store waits for a record in flight.
- **A fault** on ring 0 (C_STATE bit 8) drops both mount binds; a session that ends mid-record sets bit 16 at the
  next hello.

### The download tier as built

`MISTER_OCRAM_FIO` declares it, with `MISTER_OCRAM_FIO_GAP` (cycles between strobes, 4-7, default 4) and
`MISTER_OCRAM_FIO_RING` (ring 1's log2 words, default 10). A ring of 2-10 sits at `0x1000`; a ring of 13 takes ring
4's span at `0x7000` and so cannot be built beside the SECTOR tier or `MISTER_OCRAM_DIAG`
(`ocram_fio_ring_on_ring4_span`). A gap or ring outside those ranges stops the build
(`ocram_fio_unservable_geometry`), and so does `MISTER_LW_LANE` beside it (`link_on_two_players`: two players on one
set of link nets). The hello stores C_CAPS bit 1, C_IDX[1] = 0, C_RING[1] = `{kind 5, line 5, log2, 0, offset}`
(`0x29680400` at the defaults) and C_K[9]. Every wire item here is PROVISIONAL. No H_BIND bit takes ring 1: the
walker serves it in every session.

- **A record is an LFRAME,** `{0x30, arg, len}` with `arg = {odd[7], 0[6:2], sel[1:0]}` and `len` ring words. Each
  ring word carries two link words, low half first; `odd` drops the last word's high half. Only a `sel` in C_K[9]'s
  mask is served: 1 (the FPGA select, the file-I/O block), and with `MISTER_OCRAM_UIO` also 0, 2 and 3 (below). A
  record of any other kind, a `sel` outside the mask, a non-zero `arg[6:2]` or a zero length is skipped by its
  length, silently: it is not a fault, and nothing of it reaches the link, so a host gates what it sends on C_K[9]
  (and on C_CAPS for the bits above `sel`). A length past the head stops the ring on a fault (C_STATE bit 9); a
  session that ends mid-record sets bit 17 at the next hello.
- **The walker** reads H_IDX[1] in its session poll and serves ring 1 at record granularity, in turn with rings 2
  and 4. Each header and ring word crosses to `clk_sys`; after the record's last word is answered it stores C_IDX[1].
- **The player** (`clk_sys`) plays a frame as the LW lane's player does, through `lw_link`: it takes the link's nets
  only between link transactions (no select up, the I/O view on), raises the select, strobes each link word GAP
  cycles apart, holds the next strobe while `ioctl_wait` (or the link's own wait) is up, and drops the select
  between frames. A core must raise `ioctl_wait` within GAP - 3 cycles of an `ioctl_wr` to stop the next one. The
  last ring word is answered only once the select has dropped, so C_IDX[1] reaching a frame's end means the frame
  has played. The session's enable falling ends a frame short: the select drops and the rest of the record is
  discarded by the walker.
- **The mark line** (ordinal 5, `f2h_irq[12]`): a host waiting for room stores H_MARK[1] armed half a ring behind
  its head and waits on the line, and the fabric plays the other half meanwhile. A host waiting for the download to
  finish stores the mark at its head; the line rises once the last frame has played.

### The UIO and OSD frames as built

`MISTER_OCRAM_UIO` (PROVISIONAL) needs the download tier (`ocram_uio_needs_MISTER_OCRAM_FIO`) and adds no record
kind and no C_CAPS bit: it widens the walker's select mask, and C_K[9] declares it (`0x000F0004` at gap 4,
`0x000B0004` with `MISTER_DEBUG_NOHDMI`, which has no HDMI osd). An LFRAME with `sel` 0 is one UIO transaction, with
`sel` 2 or 3 one transaction on that OSD select. The player is the download tier's, unchanged: it plays only between
link transactions, into the link's own decoders through `lw_link`, so each frame's end runs the stock steps (the
RTC and timestamp toggles, the mount strobe's fall, an OSD control word's enable).

- **One select a frame.** The link's OSD transaction raises both OSD selects together; on ring 1 it is two frames,
  `sel` 2 then `sel` 3, each only if the mask serves it.
- **Write-only words.** The frame's replies are not read back. A word whose reply the host acts on stays on the
  link (or goes by capture, `MISTER_OCRAM_LANE`). So do `0x14` and `0x30`: `0x30` holds its frame on `vs_wait`,
  and ring 1 behind it, for up to a video frame.
- **Order.** Ring 1 plays in record order with the downloads; a host that also sends on the link keeps its own
  order by waiting for C_IDX[1] before the next link word (the fabric does not order the two carriers).

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
- that a mount request longer than `MISTER_OCRAM_MOUNT_FRAME` is served by a host that no longer polls: size the
  macro to the core's largest request;
- that an index it does not write holds still, or that an index it writes is read at once: the fabric polls;
- that an interrupt means the ring holds a record: one may arrive after the drain.

## Timing rule

A core that builds an OCRAM tier ships only with a timing report in which no failing setup, hold, recovery or
removal path starts or ends in the master or in any `ocram_*` module. A failing path in the master can leave a
transaction unfinished, which is the danger the gate exists for.

## Measured performance

Measured on a DE10-Nano, 2026-11-22..24 (milestones M0-M7). The full records live in the greenfield rewrite's
`harness/rig/ocram-m*-measured.md`.

There are four different rates here, and they are easy to mix up. Every consumer rate is per consumer; consumers
share the master at record granularity, so two running together split one budget (below).

| path | what limits it | measured |
|---|---|---|
| fabric -> on-chip RAM (this master, single-beat AXI3, 32-bit) | bridge latency, flat: 9 `clk_100m` cycles a read, 10 a write | ~36 MB/s reads (8.9 M words/s), ~32 MB/s writes (8.07 M words/s) |
| CPU -> on-chip RAM (the host's own loads and stores) | the A9's mapping and instruction choice | `STR` 72.5 ns/word (~55 MB/s); `STM` x8 37.8 ns/word (~105 MB/s); `LDM` x8 13.9 ns/word (~290 MB/s, reads) |
| host -> ring -> torture sink (DIAG), end to end | the bridge: the sink consumes on `clk_100m`, with no crossing | ~6.3 M words/s (~25 MB/s), the same on strongly-ordered and write-combined maps |
| host -> ring -> OSD consumer, end to end | the consumer's clock-domain handshake, one crossing a word | 640 ns a word: ~1.56 M words/s (~6.2 MB/s) at the Template's 20 MHz `clk_sys` |

- `STR` stores one register; `STM`/`LDM` store/load several consecutive registers in one instruction, so the bus
  can burst. A host should write payloads with multi-register stores (a `memcpy`), not a word-at-a-time loop.
- The CPU outruns the fabric by 2-4x, so a write-combined mapping buys nothing today: the fabric is the bottleneck.
- The handshake: each word read at `clk_100m` crosses into the consumer's `clk_sys` through a holding register and
  a toggle, two flip-flops each way (`lw_xfer`). That round trip is ~640 ns a word at a 20 MHz `clk_sys`, against
  90 ns for the read itself, and it is most of a row's ~41 us. It scales with `clk_sys`, not with the bridge.
- The bench (`tb/ocram_tb.sv` built with `OCRAM_TB_RATE`: its bridge model at a fixed 10-cycle latency, which reproduces both rig figures:
  6.28 M words/s for the sink, 638 ns a word for the OSD at 20 MHz) gives, per consumer, ring pre-filled:

| consumer | 20 MHz `clk_sys` | 33.8688 MHz `clk_sys` |
|---|---|---|
| OSD, `hps_ocram_osd` (the M7 consumer) | 608 ns a word, 1.64 M words/s, 38.9 us a row | 450 ns, 2.22 M words/s, 28.8 us a row |
| OSD, `hps_ocram_rx` | 612 ns, 1.63 M words/s, 39.2 us a row | 455 ns, 2.20 M words/s, 29.1 us a row |
| SECTOR, `hps_ocram_rx`, 588-word frames, gap 2 | 608 ns, 1.64 M words/s | 451 ns, 2.22 M words/s, 265 us a frame |
| OSD and SECTOR together, `hps_ocram_rx` | about 1.7 M words/s in all; a row waits behind a frame, about 330 us a row | about 2.3 M words/s in all; about 245 us a row, the SECTOR stream 2.0 M words/s |

  `hps_ocram_rx` keeps the one-word handshake, so its rates are the old consumer's: it saves area, not time. Crossing several words at once, or a small dual-clock FIFO, is the obvious
  next step; read bursts would then amortize the 90 ns latency as well. Neither is built.
- The region is ~60 KB at most and is used as rings: the rate is how fast a stream flows through, the size is how
  far a producer can run ahead. Per job:

| job | size | over OCRAM | over SPI |
|---|---|---|---|
| one OSD row | 256 B | ~41-49 us | ~0.86 ms |
| full 16-row OSD repaint | 4 KB | ~0.64 ms | ~14-18 ms |
| one CD sector | 2,352 B | ~265 us at 33.8688 MHz `clk_sys` (bench) | — |
| PSX 2x CD-ROM stream | 150 sectors/s | ~4 % of the path's time (bench) | — |

- Cost (Quartus 17.0.2, seed 1, Template): the floor (master, poller, control page) is 158 ALMs and 0 M10K;
  `MISTER_OCRAM_OSD` adds 399 ALMs and 0 M10K (the consumer 338.5, the master's port 57.5). That is about five
  times the first estimate. The handshake above is the OSD tier's time, not its area: the area went to three
  registers holding the same word, four address adders and a walker of its own, which `hps_ocram_rx.sv` replaces with
  one walker, one adder and one held word (ESTIMATED about 160 ALMs alone, not yet fitted). The bridge itself is free hard silicon: stock
  `sys/sysmem.sv` already instantiates the `fpga2hps` atom, unused (`port_size_config` `2'b11`); this tier sets
  `2'b00` (32-bit).
- Verified on the rig: secure and non-secure transactions both pass; 10^9 fabric-written words read with 0 errors
  (M4); 10^9 host-written words consumed with 0 errors, plus 2x10^8 on each write-combined variant (M7); 1,000/1,000
  quiesced reloads clean, while a raw bridge reset with RUN high strands the master (M5, which is why the stop proof
  precedes every bridge reset); the OSD drawn over the ring is visually identical to the same bytes over SPI for
  three patterns, with no tearing seen.

## What is not verified

- Write-combined ordering with the ring part-full (every torture run kept it full; `torture --band` is the probe, not yet run on the rig).
- What this traffic costs the other HPS masters (a loaded run lost 3.5 %, not attributed).
- Read bursts and a multi-word crossing: designed in prose only.
- The SECTOR tier, the mount slots and `hps_ocram_rx.sv`: benched only (`tb/ocram_tb.sv`, `tb/sfeed_tb.sv`), not
  fitted, not run on hardware. The UIO and OSD frames (`MISTER_OCRAM_UIO`) the same: arm U only. The events tier
  (`MISTER_OCRAM_EVT`, arm I) and `MISTER_DISABLE_SPI_EVT` (arm Y) the same; `sys_top`'s port wiring is not
  simulated, only the master, `hps_ocram_evt` and `hps_io`. The capture lane (`MISTER_OCRAM_LANE`, arm A) the same;
  its reply word is `hps_io`'s `io_dout` alone in the bench, not `sys_top`'s `io_dout | io_dout_sys`.
- A greenfield host driving a real core's menu over the ring on hardware (the probe has; the host path is in diagnosis).
- A stock Main host.
- A warm reset with `warmramgrp.enable` cleared: stock U-Boot hangs on a hardware warm reset with or without this
  tier, so every restore is a cold reboot.
- The GPO and GPI bit numbers (GPO 28 RUN, GPI 26 present, GPO 25 the test freeze) are provisional, POC rows.
