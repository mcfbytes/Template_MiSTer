# The LW bridge framework

An optional extension of `sys/` that gives the HPS a second, memory-mapped path into the core, beside the
GPO/GPI ("SPI") link that `hps_io` decodes. It lives on the Cyclone V **HPS-to-FPGA lightweight bridge**
(`lwhps2fpga`, physical `0xFF200000`, 2 MB). The HPS stores pads, OSD pixels and disk blocks as plain 32-bit
writes, and reads the core's status back-channel and block requests as registers. Interrupt lines tell it when
to look. An optional frame lane carries whole link transactions, and their replies, over the same bridge.

- **The core sees nothing new.** Pads land in the same `hps_io` registers (`joystick_0`, ...), blocks arrive as the
  same `sd_ack`/`sd_buff_wr`/`sd_buff_dout` strobes, and the status back-channel is the stock
  `status_in`/`status_set` handshake.
- **The SPI link is untouched.** `hps_io` keeps every SPI decode, so a host that knows nothing about the window (the
  stock MiSTer Main) drives an LW-enabled bitstream exactly as before. The window is a second carrier, not a
  replacement.
- **Opt-in, in tiers.** Without `MISTER_LW` the framework is not built: `HPS_BUS` stays 46 bits, `CONF_STR` is the
  stock string byte for byte, and `sys/` preprocesses to upstream's text. With it, a core builds the floor and
  only the tiers its `.qsf` names. This repository's `Template.qsf` carries the defines commented out, so the
  Template builds stock until they are uncommented.

Status: proof of concept. The directory is layout 2 (magic `LWDR`, version 2.0): 32-word entries with common CAUSE
and LINE words, every offset window-relative. A minor revision may add blocks (new kinds), header words, entry
words or a larger stride, and a host that walks by the rules under "Directory" keeps working. Every cost figure
below is an estimate, and this revision has not been fitted or run on hardware ("Revision and validation").

## Adopting it in a core

1. **The defines.** In the core's `.qsf` (in `Template.qsf` the lines are already there, commented out): the
   floor, then each tier the core wants. Every tier is opt-in, and a tier macro without `MISTER_LW` stops the
   build with an error naming `lw_tier_needs_MISTER_LW`.

   ```
   set_global_assignment -name VERILOG_MACRO "MISTER_LW=1"
   set_global_assignment -name VERILOG_MACRO "MISTER_LW_PAD=1"
   set_global_assignment -name VERILOG_MACRO "MISTER_LW_EVT=1"
   ```

2. **The declaration.** Put `` `SYS_LW_CAPS `` in field 1 of `CONF_STR`, with no comma after it (the macro
   carries its own):

   ```verilog
   localparam CONF_STR = {
       "MyCore;", `SYS_LW_CAPS "SS3E000000:400000;",   // or `SYS_LW_CAPS ";" when field 1 is empty
       ...
   ```

   With `MISTER_LW` it expands to `"AXI:40000:80000:LW,",` whatever blocks are built: the directory lists the
   blocks and declares their lines. Without it, it expands to nothing.

3. **Nothing else for pads, the OSD image and the status back-channel**, each with its tier macro
   (`MISTER_LW_PAD`, `MISTER_LW_OSD`, `MISTER_LW_EVT`). A core that passes `HPS_BUS` to `hps_io` gets them.
   `Template.sv` shows the status back-channel: the first pad button cycles an option and writes the new status
   word back with `status_in`/`status_set`. With `MISTER_LW_SECTOR_SLOT` it also builds a block reader on that
   slot, served over SECTOR or, without the host's EN, over SPI from an image mounted on its `S` line.

4. **Optional: the SECTOR block**, for a core that reads blocks through an `hps_io` slot ("Hooking the SECTOR block",
   below).

5. **Optional: the frame lane** (`MISTER_LW_LANE`). It needs nothing in the core either: it lives in `sys_top`.

6. **The timing rule** ("Tiers and their cost"): the core ships only with a timing report in which no failing path
   starts or ends in LW logic.

A core that instantiates its own modules on `HPS_BUS` must widen those ports to `[383:0]` under `MISTER_LW`; bits
45:0 keep their stock meaning. A port left at `[45:0]` truncates the bus with only a width warning: the pieces
beyond it then show no presence bit, and their blocks are not listed in the directory ("`HPS_BUS` with
`MISTER_LW`", below, has the bit map and who drives each field).

**Reserved names.** In an LW build the module names `lw_*` and `hps_lw_*` and the macro names `MISTER_LW*` and
`LW_*` belong to the framework; a core uses none of them for its own. Without `MISTER_LW` every LW file
preprocesses to nothing and leaves no macro defined.

### Macros

| macro | default | effect |
|---|---|---|
| `MISTER_LW` | off | the floor: the bridge, the window, its directory, the host enable and GPI bits 25, 24 and 22 |
| `MISTER_LW_PAD` | absent | with `MISTER_LW`: the PAD block (`lw_pad.sv`) and `hps_io`'s LW write arm |
| `MISTER_LW_EVT` | absent | with `MISTER_LW`: the EVT block (`lw_evt.sv`): `hps_io`'s status, block-request, info and upload flags as causes, `hps_io`'s exports and its line |
| `MISTER_LW_OSD` | absent | with `MISTER_LW`: the OSDBUF block (`lw_osd.sv`) and a write port into each `osd`'s `osd_buffer` |
| `LW_DIAG` | 0 | 1 = debug build: SCRATCH and the SECTOR counters/timestamps read back |
| `MISTER_LW_SECTOR_SLOT` | absent | the `hps_io` slot the SECTOR block feeds, 0-15; its presence builds the block |
| `MISTER_LW_SECTOR_FRAME` | 128 | 32-bit words per primary frame, 1-4096 (128 = one 512-byte block) |
| `MISTER_LW_SECTOR_GAP` | 2 | `clk_sys` cycles between write strobes, and from the ack to the first, 2-15 |
| `MISTER_LW_BLKSZ` | 2 | the core's `hps_io` `BLKSZ`, reported in the GET_SDSTAT words |
| `MISTER_LW_SAVE0_SLOT`, `_WSRC`, `_EXTENT` | absent | save slot 0 (slot index 1): `hps_io` slot (0-15), DDR image base, image bytes |
| `MISTER_LW_SAVE0_FRAME` | 256 | its 32-bit words per frame, 1-510 (the FIFO's entries - 2; 256 = one 1 KB block) |
| `MISTER_LW_SAVE0_LBA_BITS` | 32 | its LBA bits kept, crossed and compared, 1-32 |
| `MISTER_LW_SAVE1_SLOT`, `_WSRC`, `_EXTENT`, `_FRAME`, `_LBA_BITS` | absent, 256, 32 | save slot 1 (slot index 2), likewise |
| `MISTER_LW_LANE` | absent | with `MISTER_LW`: builds the frame lane (LANE block) and its link front, `lw_link` |

A save slot is declared by its `_SLOT` macro, needs its `_WSRC` and needs `MISTER_LW_SECTOR_SLOT` too. **An
unservable geometry stops the build** (in synthesis too), naming the undefined module
`lw_sector_unservable_geometry`: a primary frame outside 1-4096 words, a save slot's outside 1-510, an LBA width
outside 1-32, a gap outside 2-15, an `hps_io` slot above 15, two slots on one `hps_io` slot, or a save slot without
an image base. Frames are one fixed length per slot; a slot whose requests vary in length or use `sd_blk_cnt`
stays on SPI by not being declared.

### Tiers and their cost

A core pays only for what its `.qsf` names. Every figure is **ESTIMATED**: registers counted from the RTL, ALMs by
analogy with a fit of an earlier revision. None of this revision has been fitted.

| tier | macro | builds | cost (ESTIMATED) |
|---|---|---|---|
| L0 floor | `MISTER_LW` | the firewall, the decode, the directory and its header words, the host enable, GPI bits 25, 24 and 22, the 384-bit `HPS_BUS` | 150-200 ALMs, no block memory |
| L1 pads | `MISTER_LW_PAD` | the PAD block, `hps_io`'s LW write arm | 150-185 ALMs, 1 M10K |
| L2 events | `MISTER_LW_EVT` | the EVT block, `hps_io`'s event exports, line 2 | 85-120 ALMs, 132 of its registers being the status hold on `clk_sys`; no block memory |
| L3 SECTOR | `MISTER_LW_SECTOR_SLOT` and its geometry | the SECTOR block and its enable link, line 1, the core-side muxes, `hps_io`'s commit mark | 325-400 ALMs and about 12 per mux, 3 M10K |
| L3 lane | `MISTER_LW_LANE` | the LANE block and its enable link, `lw_link`, line 3 | 225-255 ALMs, 3 M10K |
| L3 OSD | `MISTER_LW_OSD` | the OSDBUF block and its enable link, a second write source in each `osd` | 75-100 ALMs, 2 M10K |
| debug | `LW_DIAG=1` | SCRATCH, the SECTOR counters and timestamps | not estimated |

`MISTER_LW` alone is legal: a safe, discoverable window with COUNT = 0 and a host enable.

**A tier also costs timing margin, and the timing rule binds every core that builds one.** The bridge's clock,
`clk_100m` (`h2f_user0_clk`), also clocks the scaler's Avalon side and the frame-buffer port, and everything the
firewall's promise rests on runs on it: its state machines, its reset marks and idle bit, the read path, the gates.
A failing path there is not a picture fault: a setup failure in the firewall can leave a transaction unanswered,
which hangs the SoC, and one in the read path returns wrong data. So **a core that builds any tier ships only with a
timing report in which no failing setup, hold, recovery or removal path starts or ends in LW logic**, on any
clock: a register of `hps_lw_bridge`, of `lw_sys_window`, of any `lw_*` module, or of the LW text in `hps_io`,
`osd` and `sys_top`. What else in the core fails is the core's own business; a core that ships with negative slack
elsewhere checks its report by these names. The deepest LW path is the read: the firewall's address register,
through the block select and the block's register mux, to the read data register, in one 10 ns cycle, and it grows
with each tier. The store path is the other: `lw_write`, the enable gate, a block's OPEN bit, a register's enable.

**Measured on an earlier revision of this branch** (Quartus 17.0.2 Lite, 2026-11-14, before the host enable, the
tier modules and the write-through OSD; not a measurement of this revision): the Template with the floor, PAD, EVT
and its example took 7,768 ALMs against stock's 7,273 (+495), 12,150 registers against 11,226 (+924) and 68 RAM
blocks against 59 (+9). Of the +924 registers, `hps_io` accounted for +261 and `emu` for +132, most of it the
example's status and pad logic, which the stock Template leaves unconnected and Quartus prunes, so a core that
already uses those ports pays less. The HDMI clock failed setup in two of three seeds (-0.125 and -0.154 ns), every
failing path inside the scaler (`ascal` to `ascal`), and the bridge clock had 2.2-2.9 ns to spare, its worst path
inside the PAD drain. That build with `MISTER_LW` off gave an rbf bit-identical to upstream's. The frame lane
without reply capture, measured on a larger core, took 165-177 ALMs plus 6-7 for `lw_link`, and 2 M10K.

### Beside the window: the OCRAM mailbox (proposed)

`docs/OCRAM_MAILBOX.md` proposes a third carrier for bulk data, beside SPI and this window. The HPS writes records
into its own 64 KB on-chip RAM, and the core fetches them as a master on the FPGA-to-HPS bridge, which `sys/` already
instantiates with no port connected. The payload waits in the HPS's RAM, so its tiers cost logic and no block memory, and
the HPS never issues an access the fabric must answer. **Its floor, OSD tier and SECTOR tier are built, for test cores
only; its other tiers are a design.**

It moves no interface in this document. Its SECTOR tier (`MISTER_OCRAM_SECTOR_SLOT`) feeds the same SECTOR bits of
`HPS_BUS` the SECTOR block feeds, through this window, so it needs `MISTER_LW`, the core hooks its slot through
`lw_blk_mux` as below, and `hps_io` builds the commit mark under either SECTOR macro; a build of both stops, one slot
having one feeder. No existing bus bit, port or hooking line here moves. Otherwise either floor builds without the
other, each function has its own macro on each path, SPI stays under both, and a host binds one carrier per function.
Where both are built the proposal is:

| function | carrier proposed | why |
|---|---|---|
| pads | the PAD block here | one store per change, proven on hardware; a mailbox pad arrives one poll period late |
| status and `hps_io`'s flags | the EVT block here | no block memory already; the mailbox version exists for a core with no LW bridge |
| the OSD image | the mailbox | OSDBUF's FIFO is two block memories; a mailbox row is 65 memory stores and publishes whole |
| block reads | the mailbox | SECTOR's FIFO is block memory; a mailbox frame is whole by construction, and blocks may be queued ahead of the core's requests |
| save slots | the SECTOR block here, or SPI | a save block must reach the core whole or not at all, which needs the block's FIFO; the mailbox has none |
| link frames | either | the LANE block's FIFOs are block memory; the mailbox keeps the same player |

The mailbox adds a second floor in logic, so it is not the smaller choice for a core whose only bulk function is the
OSD. Its guide has the menu, the cost estimates and the list of what is still unverified.

## Discovery and safety

An LW access the fabric does not answer **hangs the SoC**. Nothing here is probed, so the HPS must know the
window exists before it touches it:

- **`CONF_STR` field 1, `AXI:40000:80000:LW`:** a window at LW offset `0x40000`, `0x80000` bytes long.
- **GPI bit 25 = 1** (the FPGA manager's `gpi` register, read with `gp_out[31] = 1`; reading it never touches the
  bridge, so it is always safe): the firewall below is in this bitstream. The firewall's instance drives the bit,
  so no bitstream without it can show 1. Stock bitstreams leave it 0, and the stock Main never reads bits 21-27.
- **GPI bit 24** is the firewall's idle state, a register: 1 while it holds no transaction on either channel. It
  toggles with every access, so it means something only to a host that knows it has none of its own in flight.
- **GPI bit 22** is the window's host enable (below): 1 while an LW host has the window enabled. Reading it
  changes nothing and can hang nothing, so a program looks here before it takes the link (see "The host enable").
- **The directory** at the window's first word: MAGIC `LWDR`, a directory major the host knows (2), and WIN_BASE
  and WIN_SIZE equal to the token's two numbers; then the entries.

In the ID view (`gp_out[31] = 0`) GPI carries the core magic, which has bits 25, 24 and 22 clear, and an
unconfigured FPGA reads all ones, so the bits count only in a sample that also shows bit 31 = 0.

**The four host rules.** The firewall answers every access that reaches it; these rules are about the accesses
that cannot reach it.

1. **What bit 25 covers.** It proves the firewall is in the bitstream and says nothing about the HPS side: the
   bridge must also be out of reset (the reset manager's bit for this bridge reads 0), which a host that did not
   release the bridge itself checks.
2. **Quiesce.** Before anything resets the bridge, asserts the fabric reset or programs the FPGA, every LW access of
   every thread has completed, and none starts again before rule 3. An access during programming, through a
   bridge in reset, or into a newly loaded bitstream without the firewall hangs the SoC.
3. **Discover, and discover again.** Before its first LW access, and again after every core load and every bridge
   or HPS reset (with every cached directory value discarded first), a host takes **one** GPI sample in the I/O
   view that shows bit 31 = 0, bit 25 = 1 and bit 24 = 1. Bit 24 = 0 at that point, with no access of its own in
   flight, means the firewall still holds a transaction nobody will finish: stay on SPI until the core is loaded
   again. Then the `CONF_STR` token, then MAGIC, VERSION, WIN_BASE and WIN_SIZE. Any failure: never touch the
   bridge.
4. **Sample before each access.** A periodic task that accesses the window samples GPI first and requires bit
   31 = 0 and bit 25 = 1 before each access (not bit 24, which another thread's access makes 0). A failed sample
   means no access now, and the whole of rule 3 before the next.

**What the rules cannot close.** A second program that resets the bridge or loads a core without coordinating with
the LW host can land between a sample and its access (one GPI read plus one LW access wide). So one program owns
the window, the link and core loading, or the programs that share them coordinate.

**Resets.** `hps_lw_bridge.sv` answers **every** address of the 2 MB bridge, every burst and both channels, in
fixed cycles. No reset holds it, so a read during a core reset, or while the HPS holds the fabric reset, still
completes. After an HPS reset it drops only a transaction the reset orphaned: one whose master made no handshake on
its channel between the reset reaching the fabric and its release. The first access after the reset is answered;
nothing else is touched. A live transaction whose master makes no handshake on its channel for the whole of a
fabric reset that software asserted looks the same and is dropped too, which rule 2 rules out. A reset of the
bridge alone is invisible to the fabric, so **no LW access may be in flight when anything resets the bridge,
asserts the fabric reset or programs the FPGA**: a transaction left in flight through a bridge-only reset holds its
channel until the next HPS reset or core load. Addresses outside every block read `0xDEADBEEF`, and writes there
are dropped.

Two orderings on the HPS side, which the fabric cannot see, decide whether its reset handling alone is enough:
every reset that can orphan a transaction must reach the fabric as an HPS reset (a cold reset does; whether a warm
reset does is set by the reset manager's warm-reset masks, which software owns), and the bridge's master, once out
of reset, must issue nothing until the fabric has seen the reset end (about four `clk_100m` cycles after the HPS
releases it, and never while software holds the fabric reset). A host that keeps rule 3 depends on neither: bit 24
= 0 shows the transaction left behind. RESETS (below) counts the HPS resets the fabric saw and the state machines
it freed, so a host can tell whether a reset it knows of reached the fabric at all. A stuck state machine is freed
by an HPS reset or a core load, and by nothing a host can do through the window.

Nothing back-pressures. A store is accepted in fixed cycles. Where a FIFO is full, the store is dropped and counted,
and the protocol detects the loss (PAD spinner drops, OSDBUF drops, SECTOR `ACCEPT`, LANE DROPS).

### When a store is in effect

**A store is a single-beat write** (`AWLEN` 0). The beats of a burst write are accepted, answered OKAY and dropped,
so a `memcpy` into the window, or a write-combined or cached mapping that merges stores into bursts, loses them.

**A store is in effect once its write response has returned.** The fabric applies a store in the cycle its data
beat is accepted and answers it in the next; a read samples its word one cycle after its address is accepted. The
read and write channels are independent, and the fabric orders nothing between them, so between a store and any
read whose meaning depends on it the host must have the store's response. Map the window strongly ordered, where
each access completes before the next begins (on ARMv7 Linux an `mmap` of `/dev/mem` or a UIO mapping, both
non-cached), or, with a Device mapping (`ioremap`), issue a DSB after the store. A write-combined, cached or
Normal-memory mapping is never legal: its stores may merge into bursts, reorder or repeat.

The sequences that depend on it: a CAUSE store-back, then the block's level registers; the ENABLE store, then the
ENABLE read; an EN store, then REQ; the last DATA, then ACCEPT; RPOP, then RHEAD; END, then RSTAT or PLAYED. A read
shows bridge-clock registers only. What a store sets moving on `clk_sys` shows in the level registers later, which
is why every procedure below reads levels after events.

Read bursts of every type return the words of the addresses they step through; one that would cross a 4 KB
boundary (an AXI master never issues one) wraps inside its 4 KB.

## The window

Offsets are from the window start; absolute LW offset = `0x40000` + offset. An address in no block that is built
reads `0xDEADBEEF` and drops stores, inside the window or outside it, and an unassigned offset inside a block reads
0 and ignores stores.

| window offset | absolute LW | size | block |
|---|---|---|---|
| `0x00000` | `0x40000` | `0x1000` | directory: header and entries; the rest of the page reads 0 |
| `0x03000` | `0x43000` | `0x1000` | EVT (with `MISTER_LW_EVT`) |
| `0x04000` | `0x44000` | `0x2000` | OSDBUF (with `MISTER_LW_OSD`) |
| `0x06000` | `0x46000` | `0x1000` | LANE (with `MISTER_LW_LANE`) |
| `0x08000` | `0x48000` | `0x8000` | SECTOR (with `MISTER_LW_SECTOR_SLOT`): registers at `+0x0000`, DATA at `+0x4000` |
| `0x20000` | `0x60000` | `0x20000` | reserved for a core's own blocks; not decoded in this revision |
| `0x40000` | `0x80000` | `0x40000` | PAD (with `MISTER_LW_PAD`) |

**The register bus.** Behind the firewall every block sits on one bus and keeps two rules: it answers every address
combinationally from `clk_100m` registers and constants, and it never back-pressures. The bus has no read strobe,
so no read of any address, size or burst changes anything. Each block's select compares the address's high bits
with a constant, the selects are disjoint, each block's word is 0 unless it is selected, and the read word is one
AND-OR of the block words and the `0xDEADBEEF` default. A block added later keeps the same rules.

### Directory

**Header** (16 words, read-only except as noted):

| offset | word | value |
|---|---|---|
| `0x00` | MAGIC | `0x5244574C` ("LWDR"); it does not change in a later major |
| `0x04` | VERSION | `0x00020000`: major 2 `[31:16]`, minor 0 `[15:0]` |
| `0x08` | LAYOUT | `{entry_stride[31:16], header_len[15:0]}` in bytes: `0x00800040` |
| `0x0C` | COUNT | entries present, one per block built, at most 31 |
| `0x10` | WIN_BASE | `0x00040000`: the window's absolute LW offset; equals the token's |
| `0x14` | WIN_SIZE | `0x00080000`; equals the token's |
| `0x18` | ENABLE | read/write: the host enable (below) |
| `0x1C` | - | 0, reserved |
| `0x20` | PENDING | bit *i* = entry *i*'s CAUSE word is non-zero, enabled or not |
| `0x24` | OWNED | bit *i* = entry *i*'s block has not yet returned to SPI: its core-clock half has not acknowledged rest since the enable last fell (or since configuration), or the enable fell before the half answered its rise; only blocks with an enable link (SECTOR, OSDBUF, LANE) ever set it |
| `0x28` | SCRATCH | read/write with `LW_DIAG=1` (while enabled, like every store); 0 otherwise |
| `0x2C` | OPEN | bit *i* = entry *i*'s block takes stores now: the enable for a block without an enable link, and for SECTOR, OSDBUF and LANE the enable and the block's core-clock half having answered its rise; a store to a linked block before its bit is set is dropped and sets `stale` |
| `0x30` | LINE_EN | read/write while enabled: bit *i* = entry *i*'s cause may raise its line; reads 0 whenever the enable does, and after every ENABLE taken |
| `0x34` | RESETS | `{freed[15:8], hrst[7:0]}`: the firewall's state machines freed at a reset's release, and the HPS resets the fabric saw; each counts from configuration and stops at 255 |
| `0x38`-`0x3C` | - | 0, reserved |

**Entries.** Entry *i* is at `header_len + i x entry_stride`, `0x40 + 0x80 x i` in this revision: 8 common words,
then the kind's own words K0-K23. The table is compact and in ascending kind order, listing only the blocks this
bitstream builds, so an entry's index (and its PENDING, OWNED and OPEN bits) depends on the core's configuration.

| word | offset | name | value |
|---|---|---|---|
| 0 | `+0x00` | KIND | `[15:0]`: 0 = empty, skip; `0x0001`-`0x7FFF` framework kinds; `0x8000`-`0xFFFF` a core's own |
| 1 | `+0x04` | VERSION | `{major[31:16], minor[15:0]}` of the kind's register map |
| 2 | `+0x08` | OFFSET | the block's start, window-relative |
| 3 | `+0x0C` | SIZE | the block's length in bytes |
| 4 | `+0x10` | CAUSE | the window-relative offset of the block's CAUSE word; 0 = none |
| 5 | `+0x14` | LINE | `[7:0]` the line ordinal (below); 0 = no line |
| 6 | `+0x18` | CAUSE_BITS | the CAUSE event bits this build can raise |
| 7 | `+0x1C` | TAG | 0 for a framework kind; for a core's own kind, a tag the core chooses (a FourCC) |
| 8-31 | `+0x20` | K0-K23 | per kind; unused words read 0 |

| kind | block | version | offset / size | CAUSE | LINE | CAUSE_BITS | kind words |
|---|---|---|---|---|---|---|---|
| 1 | PAD | `0x00010001` | `0x40000` / `0x40000` | 0 | 0 | 0 | K0-K7: the honoured-opcode bitmap; K8 `{spinner FIFO, keyed slots}` = `0x00100018` |
| 2 | SECTOR | `0x00020000` | `0x08000` / `0x08000` | `0x08000` | 1 | bit *k* per wired slot (`0x1`-`0x7`) | see SECTOR |
| 3 | OSDBUF | `0x00020000` | `0x04000` / `0x02000` | 0 | 0 | 0 | K0 `{rows, row bytes}` = `0x00100100` (`0x00140100` with `MENU_CORE`); K1 `{FIFO entries, 0}` = `0x01000000` |
| 4 | EVT | `0x00020000` | `0x03000` / `0x01000` | `0x03000` | 2 | `0xF` | K0 = 4: the status words |
| 5 | LANE | `0x00010002` | `0x06000` / `0x01000` | `0x06024` | 3 | `0x7` | see LANE |

**Walking it.** A host:
- refuses the directory, and drives everything over SPI, on an unknown MAGIC, a major it does not know, or a
  WIN_BASE or WIN_SIZE that is not the token's;
- walks with LAYOUT's header length and stride, never with constants, and requires the table to fit the page;
- skips kind 0, a kind it does not know, a kind major it does not know, and a core's own kind whose TAG it does not
  know;
- checks that OFFSET and SIZE lie inside the window and CAUSE inside the block, and skips the entry (never the whole
  directory) when they do not;
- gives no line to an entry whose ordinal another entry also names, or that lies outside its kind's range (1-15
  for framework kinds, 16-31 for a core's own).

**Versioning.** The directory's major changes only if a header word or a common entry word changes meaning. A minor
revision only adds: reserved words that read 0 today, a larger header or stride, new kinds. A kind number is never
reused; a kind's major changes only if one of its registers changes meaning. Reserved bits read 0 and are written 0.
A version bumps at first publication, not per unpublished revision: a bit that read 0 before and is added before
the revision ships in a core keeps the version it has.

**What a host that skips an entry is owed:**
- **An ordinal belongs to one entry**, in the kind's range, so a line never carries a cause the host cannot clear.
- **A line rises only for an entry whose LINE_EN bit a host has set**, so an entry it skipped never interrupts it.
- **A block acts only on explicit stores under the enable.** An entry nobody stores to takes nothing from SPI and
  changes nothing in the core. This binds every block a later minor adds, and every core's own block.
- **A minor adds no obligation.** A cause bit added in a minor is advisory: a level register shows the same
  condition, so a host that clears it unserved loses latency and nothing else. An event that must be served comes
  with a new kind major, or a new kind. The drain's store-back clears bits the host does not know, which this makes
  safe.
- **The ENABLE gate is shared, and that is a cost.** An ENABLE is taken only while every linked block in the
  bitstream is at rest, the ones the host skips included, because a block that has not returned could still run a
  previous host's work under the new enable. So a linked block that cannot return keeps the window from every
  host. Every linked block, in this revision, in a later one and among a core's own, therefore has its core-clock
  half on `clk_sys`, and on its way to rest waits only on what a stock link word waits on: a running `clk_sys`, no
  link select asserted, and the core's link wait released. The window is then closed to an ENABLE for each block's
  bounded return, and for as long as the stock link is itself stopped or held, which stops a stock session too.
  OWNED names the entry that holds the gate; a host that cannot enable stays on SPI.
- **A host never waits for a block it does not serve to open.** OPEN is read per entry, and a block that does not
  open costs a host that block over LW, and nothing else.

A core's own blocks (kinds `0x8000`-`0xFFFF` with a TAG, ordinals 16-31, window `0x20000`-`0x3FFFF`) are reserved
and not built in this revision. The revision that adds them defines their core-side interface, and a linked one
keeps the rule above.

### The host enable

**No store has effect without the host enable, except a store to ENABLE itself.** Every other store, to any block
or header word, is answered and dropped, and ENABLE's `stale` bit records it. Every block is off until a host that
knows this window enables it and then binds each block with its own stores (SECTOR's EN, OSD image stores, lane
frames, LINE_EN, pad stores), so a stock program that never touches the window meets a stock core.

**ENABLE** is set by a single full-word store of exactly `0x4C570001`; any other store to it is ignored. The store
is taken only while the enable is clear, `quiet` is 1, no HPS reset is seen and GPO bit 31 is 1. Then the enable is
set, `stale` and `burst` clear, LINE_EN reads 0, and the causes are reloaded (below).

**Nothing times it and no host clears it.** It clears only when the fabric sees an HPS reset, or GPO bit 31 = 0:
the ID view, which every program on this link (the stock Main among them) selects to identify the core when it
starts, or the core reset through GPO. So a host that dies is recovered by whatever comes next: a reset of the SoC,
or the next program's start, which finds every block off and the core on stock SPI, with nothing of the old host's
left to run. While a host runs, nothing in the fabric takes a block from it.

A read returns `{en[31], quiet[30], stale[29], burst[28], 12'd0, hrst[15], 3'd0, idv[11], 11'd0}`:

| field | meaning |
|---|---|
| `en` | the enable is set; GPI bit 22 shows the same register |
| `quiet` | every enable link is at rest: an ENABLE may be taken. A block that cannot return (a stopped core clock, or a core that never answers the link) holds it at 0 for every host; OWNED names it |
| `stale` | a store other than to ENABLE was dropped since the last ENABLE taken |
| `burst` | a beat of a burst write was dropped since the last ENABLE taken |
| `hrst` | the fabric sees the HPS reset now; no ENABLE is taken while it is 1 |
| `idv` | GPO bit 31 reads 0 now (the ID view, or the core reset through GPO); no ENABLE is taken while it is 1 |

**When the enable falls**, every line falls, LINE_EN reads 0, OSD image stores still queued are discarded unwritten,
and the SECTOR block's EN bits clear. Only a frame already playing completes: the SECTOR frame the feeder has begun
(a primary frame zero-filled from the first word the host did not store; each slot returns to SPI once its frame is
done) and the LANE frame being played. Every entry queued behind it is discarded unplayed, **committed or not**:
a SECTOR BEGIN_S or WACK_S among them goes with its DATA and leaves its request standing for SPI, so the core gets
none of that block over LW, and a stock Main serves it. A host or tool must not take a block as delivered because
its frame was stored in full, or because ACCEPT read `ok`. A save-slot frame not stored in full never reached the
core. The open LANE frame is abandoned, queued LANE frames are discarded unplayed, and reply words still to come
are discarded. PAD levels already delivered stay in the core, as a stock program's would, until the next pad store
over either path.

**The enable links.** SECTOR, OSDBUF and LANE each carry the enable to their core-clock half over a link with one
level each way: the enable level, and the half's acknowledge that it is at rest (nothing queued, nothing in flight,
every handshake idle). The half's only enable input is that level, and it reaches rest from any state once the level
is low. A link opens a block's stores only after the half has seen the level rise, and the level falls only after
the half has answered it, so an enable that falls within a few cycles of its rise still leaves the link closed until
the half answers. An ENABLE is taken only while every link is at rest (`quiet`). At an ENABLE the causes are
reloaded: EVT ORs in STATUS and every level PEND shows (a predecessor may have cleared a cause and stopped before
serving it); SECTOR's word starts clear and raises a slot's bit when EN is set over a request that stands; LANE's
word starts clear. While GPO bit 31 is 0 (the ID view) neither the lane player nor the OSD writer starts anything,
so nothing queued by a host that has gone plays inside the next program's identification.

**The enable needs the link's I/O view.** A host that holds it never leaves the I/O view, identifies the core
before it enables, and enables again after its own core reset through GPO.

**One program owns the window**, and with it the link and whatever loads cores and resets the bridge (or it
coordinates with what does).

**A host's start:**
1. **Look before touching the link.** Before its first GPO store and its first link word, read the FPGA manager's
   GPO register itself (not a software copy: the usual first act, storing GPO = 0, clears a live enable). If bit 31
   reads 1, take one GPI sample: bit 31 = 0, bit 25 = 1 and bit 22 = 1 mean another LW program has the window.
   Then store nothing to GPO, send no link word, and stop; or, if this program is configured as the machine's one
   LW program, go on and take over by identifying. With no timer there is nothing to wait out.
2. **Identify the core** (this clears any enable, and every block starts its return to SPI) and run discovery
   (rule 3).
3. **Read ENABLE until `quiet` reads 1.** Stay on SPI meanwhile, and for good if it never comes (a stopped `clk_sys`,
   a link select held, the core's link wait held): OWNED names the block that cannot return.
4. **Store `0x4C570001`**, read ENABLE and require `en`.
5. **Read OPEN until the bits of the linked entries it serves are set**: a few reads while `clk_sys` runs. Store
   nothing to an entry before its bit is set. An entry whose bit does not come within the host's own bound stays on
   SPI, and may be bound later if its bit sets. The host never waits for an entry it does not serve.
6. **Issue one GET_SDSTAT and serve over SPI what it shows** ("One request, one path", below), then bind the blocks
   and re-store every pad level.
7. **Store LINE_EN** with the bits of the entries it serves, and arm its interrupt nodes, each level-high. Every
   block reloaded its cause word at the ENABLE, so standing requests ring at once; a host without lines reads
   PENDING once.

While it runs, a host does nothing to keep the enable, and nothing to give it up: the next program's start, a core
load and an HPS reset each clear it. Before its own GPO core reset or a core load it does nothing either: the reset
clears the enable, and the host enables again after it (with discovery again after a load).

**What the start order cannot close.**
- The fabric cannot tell one program from another before it stores. A stock program, or one that skips step 1,
  takes the link by starting and clears the enable by identifying.
- Stores carry no identity. If a previous program's threads still store after the next program has enabled, their
  stores land in its open blocks. A program that takes over a live one (step 1's configured path) owns that risk.
- A program that takes the link without identifying meets whatever the previous host left bound: SECTOR slots on
  LW, queued lane frames that play whenever the link lets them. The stock Main identifies the core first.

### Interrupt lines

A line is declared in one place: the LINE word of the entry it serves. Ordinal *k* is `f2h_irq[7 + k]`, the GIC's
shared interrupt 47 + *k*, and no two entries name the same ordinal.

| ordinal | f2h line | GIC SPI | entry in this revision |
|---|---|---|---|
| 1 | `f2h_irq[8]` | 48 | SECTOR |
| 2 | `f2h_irq[9]` | 49 | EVT |
| 3 | `f2h_irq[10]` | 50 | LANE |
| 4-15 | `f2h_irq[11]`-`[22]` | 51-62 | framework kinds to come |
| 16-31 | `f2h_irq[23]`-`[38]` | 63-78 | a core's own kinds |

A line is a register, high while the enable is set, its entry's LINE_EN bit is set and its CAUSE word is non-zero;
it falls only when one of the three stops being true, and never pulses per event. **Configure it level-sensitive,
active high,** at the interrupt controller (`IRQ_TYPE_LEVEL_HIGH` in a Linux device tree). An edge-configured line
is not legal: a raise in the cycle of a store-back keeps the line high, so an edge-triggered input would see no new
edge after it and the entry would be silent for good. (Upstream's own two lines, `f2h_irq[0]` and `[1]`, are sync
pulses and are taken as edges; these are not.) The HPS kernel must expose each line the host wants to userspace
(for example as a `uio_pdrv_genirq` node); a host with no node for an ordinal polls PENDING instead, whatever
LINE_EN holds.

**Cause words.** A CAUSE word's `[15:0]` are event bits and `[31:16]` read 0; the word is 0 when nothing is pending.
Every bit is raised by each occurrence of its event, detected in the domain that makes it; none is the edge of a
level that the host's own store or service also moves. **Reads change nothing**, in any size or burst. **A full
32-bit store clears each bit written as 1**; a bit raised in the store's own cycle stays set, and a partial-width
store clears nothing. Each block also has level registers, computed from the same registers that raise its events,
so a level read after an event shows it.

**Drain:** read CAUSE; store the value back; once that store's response has returned, read the block's level
registers and serve what they show; after the last service, read the level again until it shows nothing; then
re-arm the interrupt. That is: RPOP until RSTAT's replies field, `[15:0]`, reads 0 (LANE: its `reply_drops` field
never returns to 0); BEGIN_S or WACK_S until REQ reads 0 (SECTOR); the SPI service until PEND's bits read 0 (EVT). An event before the store-back is in the level read after it, and one
after it sets its bit again, so the line is high at the re-arm. The re-read costs one access and keeps a host safe
from a block that gets the event rule wrong. A bit the host does not know is cleared by the store-back.

### PAD block (kind 1)

One store **is** one SPI set-command: a full 32-bit word (all four byte enables) at `PAD + (opcode << 10) + (index << 2)`.

| opcode | index | lands in |
|---|---|---|
| `0x02`, `0x03`, `0x10`-`0x13` | 0 | `joystick_0` ... `joystick_5`, all 32 bits |
| `0x1A` | 0-5 | `joystick_l_analog_<index>` ← `data[15:0]` |
| `0x1A` | `{p, 15}` | `index[7:4]` = *p*: `paddle_<p>` (0-5) or `spinner_<p-8>` (8-13) ← `data[7:0]` |
| `0x3D` | 0-5 | `joystick_r_analog_<index>` ← `data[15:0]` |

- **Every level is latest-wins:** each of the 24 levels (the six digital pads, both sticks and the paddle of each
  player) keeps one slot, so a burst of stores delivers the last and nothing absolute is ever dropped, even while
  `clk_sys` is stopped.
- **Spinners** are events (`hps_io` toggles bit 8 per store), so their stores queue in a 16-deep FIFO. A store to a
  full FIFO is dropped and counted in STAT, in every build.
- **Ignored:** a store to an opcode outside the bitmap, a digital store at index ≠ 0, a `0x1A` or `0x3D` store at
  an index the table does not list (a stick index above 5 or with a high nibble, a paddle or spinner player outside
  0-5 and 8-13), and a partial-width store.
- **Reads:** `PAD + 0x0` STAT `{16'd0, spinner drops}` (a free-running 16-bit count); `PAD + 0x4` ID `0x31444150`
  ("PAD1"); every other word reads 0.
- **Delivery:** a held-data handshake drains the slots to `clk_sys`, a scan visiting one slot or the FIFO per cycle.
  `hps_io` then writes the same registers its SPI arms write. Analog triggers (`0x46`) and every other opcode stay
  on SPI.
- **No enable link:** everything PAD holds is bridge-clock state, so a store taken under the enable is delivered
  whenever `clk_sys` runs, and the levels stay as stored when the enable falls.

### OSDBUF block (kind 3)

Image stores written into `osd_buffer` itself, in each `osd` instance (HDMI and VGA), through one write port beside
the SPI one.

| offset | register | |
|---|---|---|
| `0x0000`-`0x13FF` | IMAGE | W: byte *a* = `osd_buffer[a]`, row `a[12:8]`, column `a[7:0]`: one byte = 8 vertical pixels. Rows 16-19 (the `MENU_CORE` header rows) only with `MENU_CORE`. Byte enables honoured; reads 0 |
| `0x1800` | STATUS | R: `{7'd0, level[24:16], drops[15:0]}`: entries waiting (0-256); stores dropped on a full FIFO, free-running |
| `0x1804` | ID | `0x3244534F` ("OSD2") |

- **The FIFO:** each store takes one entry `{byte enables, word, data}` of a 256-entry FIFO; a store that finds it
  full is dropped and counted. The address rides each entry, so a dropped store loses only its own word and storing
  it again repairs it.
- **The writer** (on `clk_sys`) writes one enabled byte per cycle to every `osd`, and only while both `osd` selects
  are low and the link is in its I/O view, in that cycle and the two before it. An SPI OSD write needs its select
  high, so the two never meet. A word leaves the FIFO once its last byte is written: `level` = 0 means every byte
  stored is in `osd_buffer`.
- **Highres:** an LW byte in rows 8-15 sets it, as an SPI write command with bit 3 does; the SPI disable clears it.
- **Behind the enable:** OSDBUF has an enable link. Stores are taken only while its OPEN bit is set; once the enable
  falls the writer discards every queued entry unwritten, so a repaint still queued for a host that has gone never
  draws into the next program's OSD.
- **No atomic flip:** a repaint is visible as it lands, as over SPI.
- **Still on SPI:** enable/disable, the info box, rotation and `osd_status`. The pixel path is upstream's.
- **Host rules:** compare `drops` around a repaint and store again if it moved; send SPI OSD control words only at
  `level` = 0, so a late byte of rows 8-15 cannot set highres after a disable.
- **Cost:** the FIFO, 256 x 47 bits (two M10K). Nothing of the block reaches a video clock.

### EVT block (kind 4, "EVT2"): the core's flags

`hps_io`'s core-to-host flags as causes on one line (ordinal 2, `f2h_irq[9]`), so a host need not poll `0x16`,
`0x29`, `0x36` or `0x3C` on any core, including one whose blocks stay on SPI. The SPI arms still answer as before,
and serving them is how a host clears the levels.

| offset | register | |
|---|---|---|
| `0x00` | CAUSE | bit 0 STATUS: a status transfer landed (and at a taken ENABLE); bit 1 BLK: a slot's block request rose, or a slot's SPI ack fell with its request still raised; bit 2 INFO: `info_req` rose; bit 3 UPLOAD: `ioctl_upload_req` rose. A store clears each bit written as 1 |
| `0x04` | STFLG | `{SEQ[15:0], 8'h00, 4'hA, flag[3:0]}`: the `0x29` head byte, plus a count of status transfers |
| `0x08`-`0x14` | STATUS[0..3] | the transferred `status_req`, `[31:0]` first |
| `0x18` | PEND | levels: bit 1 a block request stands at `hps_io`, bit 2 info pending (`0x36` clears it), bit 3 upload pending (`0x3C` clears it); each bit an independent sample |
| `0x1C` | ID | `0x32545645` ("EVT2") |

- **The status crosses whole.** When the core raises `status_set`, `{status_req, flag}` crosses to the bridge in
  one held-data handshake: a transfer is never torn, at any `clk_sys`. Requests faster than the handshake coalesce
  into the latest, and the latest always arrives.
- **Read the status in a bracket:** STFLG, any of the four words, STFLG. If the two STFLG words are equal, the
  words are one transfer, whichever words were read and however (single reads or a burst). If not, read again. A
  host that reads no STATUS word needs no bracket.
- **Why the bracket is exact,** with no timing argument: the 132 bits are held on `clk_sys` from launch to
  acknowledge and land under one enable, so a capture is never torn; STFLG's flag and the four words read one
  register, and SEQ changes one `clk_100m` cycle after it; and any two reads are at least two cycles apart, as
  beats of a burst or as single accesses.
- **Drain:** read CAUSE and store it back; read STFLG and the words if STATUS was set; then serve over SPI what
  the level bits show (the block request with `0x16`, info with `0x36`, upload with `0x3C`) and read PEND again
  until the bits served read 0.
- **BLK** is raised again when a slot's SPI ack falls with its request still raised, the request an SPI host serves
  next; a slot the SECTOR block has claimed raises none.

### SECTOR block (kind 2, version 2.0, "SEC3")

The block serves one primary `hps_io` read slot and up to two save slots through one feeder. Every per-slot field
(the EN bit, the CAUSE bit, the slot's registers and its descriptor) is indexed by the slot index *k*: 0 the
primary, 1 save slot 0, 2 save slot 1; never by the `hps_io` slot number. The feeder replays `hps_io`'s own
handshake into the core: `sd_ack`, then, as `hps_io`'s `io_wide` (`WIDE`) says, two 16-bit `sd_buff_wr` strobes per
32-bit word, low half first, at the halfword index, or four 8-bit strobes, byte 0 first, at the byte index.
Strobes are `strobe_gap` `clk_sys` cycles apart, and the ack leads the first by `strobe_gap`.

**Kind words** (the entry's CAUSE word gives the CAUSE register, LINE ordinal 1):

| word | value |
|---|---|
| K0 | `{fifo_entries[31:16] = 512, 0, strobe_gap[7:0]}`: `MISTER_LW_SECTOR_GAP` |
| K1 | `{0, BLKSZ[10:8], 0, wide[0]}`: 1 = 16-bit strobes, 0 = 8-bit (`hps_io`'s `io_wide`) |
| K2 | the slot descriptors that follow, 1-3 |
| K3 | 0 |
| K4 + 4*k* | `{flags[31:24], 0, hps_io slot[19:16], 0, frame_words[12:0]}`; flags bit 0 = RD, bit 1 = WR; flags 0 = declared but not wired, not servable |
| K5 + 4*k* | `{0, lba_bits[5:0]}`: 32 for the primary; a save slot's `_LBA_BITS` |
| K6 + 4*k* | DDR image base, physical (0 for the primary) |
| K7 + 4*k* | DDR image size in bytes (0 for the primary) |

An undeclared save slot's words are 0.

**Registers, from the block's start:**

| offset | register | |
|---|---|---|
| `0x000` | CAUSE | bit *k*: slot *k*'s request opened or re-opened while its EN is set (a refused or withdrawn answer re-opens it); a store clears each bit written as 1 |
| `0x004` | REQ | `{pending[31], 0, k[17:16], GET_SDSTAT word[15:0]}` of the highest-priority open request (*k* = 0, 1, 2); 0 when none |
| `0x008` | CTRL | `[2:0]` EN by *k* (settable only for a wired slot), `[8]` ABORT (zero-fills the open primary frame, withdraws a save-slot frame not stored in full; one that finds the FIFO full is held and enters first, every store behind it refused until then; reads 0), `[9]` TEST, `[10]` CLR (TEST/CLR: `LW_DIAG=1`) |
| `0x00C` | STATUS | `[2:0]` `en_sys` by *k*, `[6:4]` the feeder's ack by *k*, `[8]` feeder busy, `[9]` ABORT held, `[10]` `hps_io`'s commit mark; with `LW_DIAG=1` also `frames[31:16]`. Independent samples: no decision may rest on a combination of them |
| `0x010` | ACCEPT | `{ok[31], closed[30], 0, drops[27:16], 0, next_index[12:0]}` for the frame being stored; `ok`: every word taken (the primary), the frame committed (a save slot); `closed`: no frame is open, because the last BEGIN_S or WACK_S was refused (a full FIFO or a held ABORT) or its frame was withdrawn, and DATA is ignored until the next one; `drops` counts stores refused by a full FIFO |
| `0x014` | ID | `0x33434553` ("SEC3") |
| `0x018`-`0x030` | DROPS `{0, high-water[23:16], drops[15:0]}`, COUNTS, RINGS, TSTAMP_REQ, TSTAMP_DONE, CYCLES, SUM | `LW_DIAG=1` only; read 0 otherwise |
| `0x100 + 0x20k` | RSTAT_S[k] | `{offered[31], answered[30], 0, GET_SDSTAT word[15:0]}` from slot *k*'s latest request record |
| `0x104 + 0x20k` | LBA_S[k] | slot *k*'s LBA, from its latest request record |
| `0x108 + 0x20k` | BEGIN_S[k] | write the LBA: opens a READ frame of slot *k*'s frame_words |
| `0x10C + 0x20k` | WACK_S[k] | write the LBA: answers a save slot's WRITE (*k* = 1, 2) with an ack pulse and no strobes |
| `0x4000 + 4i` | DATA[i] | word *i* of the open frame, taken only at *i* = `next_index` (an earlier word's resend, a later word and a store past the frame's length are ignored) |

GET_SDSTAT word: `{1, 6'd0, BLKSZ[2:0], hps_io slot[3:0], wr, rd}`, with a zero block count.

**One frame:**
1. The core raises `sd_rd[slot]`. While EN is set, `lw_blk_mux` hides the request from `hps_io`, the slot's
   CAUSE bit sets, the entry's line (ordinal 1, `f2h_irq[8]`) rises and REQ shows the request. Without EN the
   request raises nothing here and stands for SPI.
2. The host reads CAUSE, stores it back, then reads REQ (or RSTAT_S) and the slot's LBA_S.
3. The host writes BEGIN_S[k] with that LBA. A BEGIN_S whose slot, direction or LBA is not the core's current
   request is rejected, and the request re-opens with the LBA the core asks for now; so does a BEGIN_S met while
   another frame streams, which zero-fills that frame. A request still raised two `clk_sys` cycles after its
   frame's ack falls is offered again, as `hps_io` shows it to an SPI host.
4. The host stores DATA[0 .. n-1] in order, then reads ACCEPT.
5. If ACCEPT's `ok` is clear and `closed` is clear, the host resends from `next_index`: a word is taken only there.
   If `closed` is set, no frame is open and DATA is ignored: the BEGIN_S (or WACK_S) was refused, by a FIFO still
   full of earlier frames or by a held ABORT, or its save-slot frame was withdrawn. Its request stands, REQ shows
   it, and while EN is set its CAUSE bit rises again; the host stores the BEGIN_S again, then every word from 0. A
   second BEGIN_S over a frame that is open is no retry: met while the primary streams, it zero-fills that frame.

**Save slots:**
- A save slot's READ is the same flow on its BEGIN_S.
- A save slot's WRITE is answered by its WACK_S; the host takes the bytes from the core's own DDR image at `wsrc`
  (below).
- **A save slot's READ frame is all or nothing.** None of it reaches the core until its last word is stored; from
  then no store aborts it, and once the feeder has begun it, it plays out whole. One still queued when the enable
  falls is discarded unplayed, its request standing for SPI ("When the enable falls"). A frame not stored in full is withdrawn by ABORT, by any BEGIN_S or
  WACK_S, and when the enable falls: its request stands, its answered mark clears, and while EN is set its CAUSE bit
  rises again, unless the store that withdrew it answers the same slot. So a host that dies mid-frame leaves the
  core's save block untouched, for the next host or SPI. A save slot's frame must fit the FIFO: 1-510 words.
- **The primary streams:** each word reaches the core as it is stored. ABORT, or the consumer reset on the primary's
  mux, zero-fills the rest of its open frame, so the core is never left waiting. A host that dies mid-frame costs
  the primary's consumer one zero-filled block (`hps_io`'s handshake has no cancel), so the primary is read-only and
  for data the core can ask for again or lose once, such as an optical sector; anything the core writes back
  belongs on a save slot.

**Write requests and WACK.** Let *FB* be 4 x the slot's `frame_words`, in bytes.
- **Which bytes.** A write request with LBA *n* on slot *k* concerns image bytes `[K6 + n x FB, K6 + (n + 1) x FB)`
  of that slot's descriptor. The host reads that range for the request REQ or RSTAT_S shows (direction `wr`), and
  bound-checks it against K7.
- **One WACK, one block.** A WACK_S[k] store with LBA *n* answers exactly that block: one ack pulse, no strobes. A
  range past K7 has no bytes: the host still stores WACK (a core paused on its save must never wait) and persists
  nothing.
- **The core's rules.** (1) The bytes are there first: a write beat an f2sdram port has accepted is only queued, and
  the proof that it was carried out is a later read on the same port returning, since one port serves its commands
  in order; so the core raises the request only after the block's last write beat was accepted and a read issued
  on that port after it has returned its data. (2) The bytes hold still: the block does not change from the rise
  of the request until its ack falls. (3) No cache holds a copy: the image lies in memory the HPS operating system
  does not manage, as every core's DDR region above the kernel's limit does.
- **The host's rules.** (1) It reads the image through an uncached mapping (on Linux, an `mmap` of `/dev/mem` over
  memory outside the kernel's range is one), or invalidates the range before each read. (2) It copies the block
  before it stores WACK, because the core may change it after the ack. When it persists the copy is its own
  policy; a host that persists after the ack owns the loss if it dies in between.
- A host that dies before WACK leaves the request standing; once the enable falls the slot returns to SPI, and a
  stock Main serves the write by the stock path. A write served twice is the same write.
- Without the core's first rule and the host's first, a WACK can persist stale bytes with no fault visible
  anywhere.

### LANE block (kind 5, "LAN1")

The frame lane carries link transactions over the bridge. The HPS stores a frame (BEGIN, its data words, END),
and a player on `clk_sys` replays it onto `sys_top`'s link nets, so `hps_io`, `hps_ext`, both `osd` instances and
`sys_top`'s own command arms each decode one ordinary link transaction. The lane adds no decoder. A frame opened
with the capture bit also records the link's reply word after each of its strobes, which makes the lane a
request/reply mailbox for any opcode.

| offset | access | register |
|---|---|---|
| `0x00` | W | BEGIN `{capture[18], sel[17:16], op[15:0]}`: opens a frame. sel 0 = UIO, 1 = FPGA (FIO), 2 = OSD on HDMI, 3 = OSD on VGA |
| `0x04` | W | WORD `[15:0]`: the next data word of the open frame |
| `0x08` | W | END: commits the frame |
| `0x0C` | W | RPOP: removes the oldest reply word (ignored when there is none) |
| `0x10` | R | PLAYED: frames played, free-running 16-bit; compare modulo 2^16 |
| `0x14` | R | DROPS: frames discarded whole at END because they did not fit, free-running 16-bit |
| `0x18` | R | ID `0x314E414C` ("LAN1") |
| `0x1C` | R | RSTAT `{reply_drops[31:16], replies[15:0]}`: reply words dropped on a full reply FIFO (free-running, never cleared), and waiting |
| `0x20` | R | RHEAD `[15:0]`: the oldest reply word, 0 when none is waiting. Reading it has no side effect |
| `0x24` | R, W1C | CAUSE: `[0]` PLAYED, a frame finished playing; `[1]` REPLY, a reply word landed; `[2]` DROP, a frame was discarded at END or a reply word was dropped. Clear at an ENABLE |

Stores must be full words (all four byte enables). Every other offset reads 0.

**Kind words** (the entry's CAUSE word is `0x06024`, its LINE 3):

| word | value |
|---|---|
| K0-K7 | the UIO (sel 0) opcodes verified for write-only replay, PAD's bitmap shape: `0x01`, `0x1C`, `0x1D`, `0x1E`, `0x22`, `0x24`, `0x31`, `0x32`, `0x33`, `0x3B` |
| K8 | first-word replies the lane stands in for, one byte per opcode: `[7:0]` = 3, the reply of `sys_top`'s `0x20` arm |
| K9 | `{fifo_depth[31:16] = 1024, strobe_gap[15:0] = 4}` |
| K10 | `{reply_depth[31:16] = 256, 0}` (minor 1) |

**Frames:**
- A frame of *n* words needs *n* + 2 FIFO entries, so the longest frame is 1,022 words into an empty FIFO. A stock
  gamma table (`0x33`, 768 words) fits.
- A frame is never played before its END is stored. A BEGIN abandons an open frame without counting it; a WORD or
  END with no frame open is ignored.
- A frame that would not leave room for its END is discarded whole at END and counted in DROPS. No store waits.
- **One select per frame.** The link raises both OSD selects for one OSD transaction; on the lane that is two
  frames, sel 2 then sel 3.

**Playing:**
- The player starts a frame only after the link's own selects have been low, and the link in its I/O view, for two
  `clk_sys` cycles, so it never splits a link transaction.
- After each strobe it waits `strobe_gap` cycles with `io_wait` and `vs_wait` low, the same terms that hold the
  link's ack, so `ioctl_wait` and `0x30`'s vsync wait pace the lane as they pace the link.
- At END it drops the select for two cycles, so every falling-select arm fires (`hps_io`'s RTC and timestamp toggles,
  the `img_mounted` clear, the OSD enable). On the lane `img_mounted` is high for `strobe_gap + 1` `clk_sys` cycles.
- **While a frame plays, the link waits.** `lw_link` freezes the link's strobe and ack for the frame plus one
  cycle. A link word stored meanwhile is held, not lost, and strobes once the frame ends: its ack can take up to
  `(1 + n) x (strobe_gap + 2) + 4` `clk_sys` cycles for an *n*-word frame. A host that also drives the link must
  allow for that, or keep long frames for times it is not using the link.

**Replies (the capture bit):**
- Each strobe of a capture frame records one reply word, the link's `io_dout` (`hps_io`'s, `hps_ext`'s and
  `sys_top`'s, ORed as the HPS reads them), sampled after the strobe's pacing wait. An *n*-word frame records
  *n* + 1 replies, the first being the reply to the opcode word. This is the word the HPS would read from the
  link after each acknowledged word.
- Replies queue in order across frames in a 256-word FIFO. Read RSTAT, then RHEAD and a store to RPOP per word.
- Once PLAYED has passed a capture frame, all of that frame's replies are in the FIFO.
- A reply that finds the FIFO full is dropped and counted in RSTAT; the frame still plays whole.
- **The lane's line (ordinal 3).** CAUSE's bits are raised per event: PLAYED per frame played, REPLY in every
  cycle a reply word lands (a word that lands while another waits, or as the last one is popped, raises it too),
  DROP per frame or reply word dropped. The levels are RSTAT's replies field and PLAYED: drain by RPOP until
  RSTAT`[15:0]` reads 0. RSTAT as a whole is no drain test: `reply_drops` is a free-running count that nothing
  clears (an ENABLE neither), so after one dropped reply RSTAT never reads 0 again until the count wraps.

## Hooking the SECTOR block

The block replaces the HPS side of one `hps_io` slot. The core keeps its own reader. `sys/lw_blk_mux.sv` sits
between `hps_io` and that reader:

```verilog
// .qsf: VERILOG_MACRO "MISTER_LW_SECTOR_SLOT=1", "MISTER_LW_SECTOR_FRAME=128", "MISTER_LW_BLKSZ=2"
wire        blk_req, blk_ack, blk_wr;
wire [13:0] blk_addr;
wire [15:0] blk_data;
lw_blk_mux blk_lw
(
    .HPS_BUS(HPS_BUS), .reset(reset),
    .core_req(blk_req), .core_lba(sd_lba[1]), .core_ack(blk_ack), .core_wr(blk_wr), .core_addr(blk_addr),
    .core_data(blk_data),
    .hps_req(sd_rd[1]), .hps_ack(sd_ack[1]), .hps_wr(sd_buff_wr), .hps_addr(sd_buff_addr), .hps_data(sd_buff_dout)
);
// the reader takes blk_req/blk_ack/blk_wr/blk_addr/blk_data where it took sd_rd[1]/sd_ack[1]/sd_buff_wr/
// sd_buff_addr/sd_buff_dout (with WIDE = 0 the data is blk_data[7:0], the address a byte index)
```

- **The mux only switches between requests.** While the host's EN is off, everything passes through to `hps_io` and
  SPI serves the slot. Each slot's mux switches only while the slot is idle and its SPI ack is low, and moves to
  the LW side only while `hps_io`'s commit mark reads 0: from a GET_SDSTAT answer that shows the SPI host a request
  until that host's block transfer ends, or until a GET_SDSTAT shows none, no slot's mux moves. One mark covers
  every slot, so a bind may take effect one SPI transfer late.
- **One request, one path.** Serve a request by the path that showed it: over SPI what GET_SDSTAT shows (EVT's BLK
  and PEND say when to ask), over LW what REQ offers; the fabric never shows one request on both. Issue a block
  transfer only for the request the last GET_SDSTAT showed, as a stock host does. A host that starts beside a
  predecessor's unfinished SPI service issues one GET_SDSTAT after its ENABLE and serves what it shows before its
  first EN store; until then the slots stay on SPI, which is safe and slow.
- **The core's write strobe is muxed, never `hps_io`'s shared `sd_buff_wr`.** An SPI transfer on another slot
  therefore cannot land in an LW frame.
- **Save slots** use `lw_save_mux #(.SAVE(0|1))` the same way, on `sd_rd`/`sd_wr`/`sd_buff_addr`. A save slot's LBA
  keeps `_LBA_BITS` bits (the rest are not crossed or compared), and its image must sit in DDR at `wsrc`, where the
  host reads it for writes under the rules above. Its frame must fit the FIFO (at most 510 words). Every declared
  slot sits on an `hps_io` slot of its own, 0-15.
- **Frames are block-sized:** `frame_words` x 4 bytes is what the reader expects per request. 588 words (2,352 bytes)
  serves a raw CD sector.
- **The host decides.** It serves a slot only when it knows what a frame on that slot means. A core gains nothing
  from the block until the host knows it.

## Clock-domain contracts

The bridge side runs on `clk_100m` (`h2f_user0_clk`). Every crossing to the core is on the core's `clk_sys`, and
every crossing is built from the cells of `sys/lw_cdc.sv`: a two-flop synchroniser, an asynchronously set reset
chain, and a held-data handshake with its capture register. Nothing else crosses, and no crossing reaches a video
clock.

| block | crossing | how | bound on `clk_sys` |
|---|---|---|---|
| floor | the HPS reset; GPO bit 31 | each an asynchronously set chain, then a two-flop synchroniser | none |
| floor | the enable level out and the rest acknowledge back, per linked block (SECTOR, OSDBUF, LANE) | two-flop synchronisers; the acknowledge needs four `clk_sys` cycles of rest and the bridge side ten consecutive samples of it | none; a stopped `clk_sys` holds the link (and `quiet`) where it is |
| PAD | slots and spinner events to `hps_io` | a held-data handshake, the data held until it is acknowledged | below about 220 MHz: two `clk_sys` periods must exceed the 8 ns net-delay bound plus setup |
| EVT | the status flag and the 132-bit snapshot | one held-data handshake, landing on `clk_100m`; requests coalesce into the latest | none |
| EVT | the BLK, INFO and UPLOAD pulses | a second held-data handshake; pulses coalesce into the next | none |
| EVT | the PEND levels | two-flop synchronisers | none; independent samples |
| SECTOR | each slot's request (offered, new, rejected with the rejected entry's tag, direction, LBA) | one record per slot in a held-data handshake, round robin; a change during a launch sends the slot again | none |
| SECTOR | the FIFO | dual-clock, Gray pointers | none |
| SECTOR | EN, TEST, the status bits | two-flop synchronisers | none; independent samples |
| SECTOR | FRAMES, TSTAMP_DONE and SUM (`LW_DIAG=1`) | one held-data handshake word, so they change on the same `clk_100m` edge; frames that complete while a word is in flight are all counted by the next, and SUM then shows the latest data frame's | none |
| OSDBUF | image stores | a dual-clock FIFO, Gray pointers stepping one entry per update, into the writer on `clk_sys`, which writes `osd_buffer` on `clk_sys` as the SPI path does | none |
| LANE | frames and replies | dual-clock FIFOs, Gray pointers stepping one entry per update | none |
| LANE | PLAYED and the reply-drop count | toggles, at most one per `strobe_gap + 5` and per `strobe_gap + 2` `clk_sys` cycles; each must outlast two `clk_100m` periods | below about 250 MHz |

No crossing carries a clock-ratio parameter, and every edge is detected in the domain that makes it. Every readable
bit is a `clk_100m` register or a constant: a read never samples a core-clock net.

**The constraints.** `sys/lw_bridge.sdc`, listed in `sys/sys.qip` after `sys_top.sdc`, cuts the paths into every
first synchroniser stage, into the asynchronously set reset chains and along the foreign data path of every bundle,
and bounds the synchroniser and bundle paths instead: 8 ns from the source register into the first stage, 8 ns from
a bundle's hold register to its capture, and 5 ns of skew across every Gray bus. It names every crossing source
by the cells' register names, so it holds whatever clock group the core puts `clk_sys` in. Without `MISTER_LW` it
collects nothing and constrains nothing.

**The clock model.**
- `clk_100m` comes from the HPS. A core can neither stop nor reconfigure it, and everything the firewall's promise
  needs is on it: nothing on `clk_100m` needs `clk_sys` state to answer.
- `clk_sys` may run at any rate up to the bounds in the table, change rate, and stop for any time. Every crossing
  waits, and an enable link keeps an ENABLE out until its half is back.
- `clk_sys` may also glitch. A core that reconfigures its `clk_sys` PLL while it runs (one that switches video
  standard through `pll_cfg`, for example) may see runt pulses that clock some flops and not others. No synchronous
  design survives that unharmed, stock `hps_io` included. What the framework keeps: a pointer and its Gray copy
  never disagree for more than a cycle, because each Gray source is recomputed from its pointer in every cycle; a
  held-data handshake converges from any state, losing or repeating at most one transfer; and every core-clock
  half reaches rest from any state once its enable level is low, so nothing stuck survives the enable's fall and
  the next host starts from empty queues. A link's acknowledge is recomputed from its half's own registers in
  every cycle. An upset in the last few `clk_sys` cycles of a return, or of a rise, can still make a half look at
  rest, or open, for up to three of its cycles; above about 30 MHz the ten samples the bridge side asks for outlast
  that, below it they may not, and the cost is then that of an upset at the start of the next enable.
- **What an upset costs inside the enable it happens in:** PAD, one pad store lost or repeated, repaired by the next
  store to that level; EVT, one event lost or repeated (PEND still shows what stands; a status transfer is lost or
  repeated whole, never torn); OSDBUF, up to 255 bytes wrong or lost, repaired by a repaint (`level` and `drops`
  show that something moved); SECTOR, the open frame wrong or zero-filled, and queued answers lost or rejected,
  until the host's ABORT or the enable's fall; LANE, **queued frames lost, repeated or played garbled, which is
  stray link traffic.**
- **So:** a host leaves the lane empty (PLAYED caught up, nothing stored) across any action that makes the core
  reconfigure its `clk_sys` PLL. A core that reconfigures that PLL at moments no host can know does not build
  `MISTER_LW_LANE`, and puts no write-back data on SECTOR's primary slot.

## `HPS_BUS` with `MISTER_LW`

384 bits whatever the tiers. `lw_sys_window` owns bits `[383:46]` through one port and drives every to-core bit
(0 for a tier not built); a from-core bit is driven only by the piece named, when its tier is built. A bit no piece
drives reads 0. Each piece drives a constant 1 on its presence bit, and the window lists a block only when its tier
is built and its presence bit reads 1 (SECTOR: `hps_io`'s commit mark and at least one slot's mux; a slot whose
mux is absent has flags 0 in its descriptor and an EN bit that cannot be set). LANE and OSDBUF use no `HPS_BUS` bit
and have no presence bit.

| bits | direction | content |
|---|---|---|
| `[45:0]` | both | stock |
| `[94:46]` | to core | PAD store `{we, op[7:0], idx[7:0], data[31:0]}` (`hps_io`) |
| `[95]`, `[96]` | to core | SECTOR primary `en_sys`, `ack` (`lw_blk_mux`) |
| `[97]`, `[113:98]`, `[127:114]` | to core | the shared strobe, its data `[15:0]` (`[7:0]` with byte strobes) and address `[13:0]` |
| `[128]`, `[129]` / `[130]`, `[131]` | to core | save slot 0 / 1 `en_sys`, `ack` (`lw_save_mux`) |
| `[166:132]` | from core | SECTOR primary `{lba[31:0], req, hps_ack, reset}` (bit 132 = reset) |
| `[201:167]`, `[236:202]` | from core | save slot 0, 1 `{lba[31:0], rd, wr, hps_ack}` (low bit = hps_ack; the slot's `_LBA_BITS` used) |
| `[239:237]` | from core | event pulses, one `clk_sys` cycle: `{upload, info, blk}` (`hps_io`, with `MISTER_LW_EVT`) |
| `[243:240]`, `[371:244]` | from core | `stflg[3:0]`, `status_req[127:0]` (`hps_io`, with `MISTER_LW_EVT`) |
| `[374:372]` | from core | event levels `{upload pending, info pending, a block request stands}` (`hps_io`, with `MISTER_LW_EVT`) |
| `[375]`, `[376]` | from core | presence, constant 1: `hps_io`'s LW write arm; its event exports |
| `[377]`, `[378]`, `[379]` | from core | presence, constant 1: the primary's mux; save slot 0's; save slot 1's |
| `[380]`, `[381]` | from core | SECTOR's commit mark (`hps_io`, with `MISTER_LW_SECTOR_SLOT` or `MISTER_OCRAM_SECTOR_SLOT`); presence, constant 1: the mark |

The primary's bits (`[95]`-`[127:114]` to the core, `[166:132]`, `[377]` and `[380]`-`[381]` from it) are also fed and
read by the OCRAM mailbox's SECTOR feeder (`MISTER_OCRAM_SECTOR_SLOT`, `docs/OCRAM_MAILBOX.md`) with the same meaning;
the window stays the one driver of every to-core bit, and lists no SECTOR entry in that build.
| `[383:382]` | - | reserved: nothing drives them; every reader ignores them |

## What a host must not assume

- that a store has effect without the enable: only the ENABLE store does;
- that a store to SECTOR, OSDBUF or LANE has effect before that entry's OPEN bit is set;
- that a store is in effect before its write response has returned;
- that a burst write does anything: its beats are dropped;
- that a line is high without the enable and its entry's LINE_EN bit, or that it is an edge: it is a level;
- that the enable survives an identification or a core reset through GPO, its own or another program's, or an HPS
  reset;
- that the enable ends by itself: nothing times it, and a host that dies leaves every block bound until the next
  program identifies the core, or the SoC is reset;
- that PAD levels clear when the enable falls: they stay in the core until the next pad store over either path;
- that the OSD has an atomic flip: a repaint is visible as it lands;
- that STATUS bits (SECTOR's STATUS, EVT's PEND) are one sample: each bit is its own;
- that anything keeps a second program out: the look at GPI bit 22 is a rule the fabric cannot enforce;
- that a block it skips cannot hold the ENABLE gate (`quiet`): OWNED names it;
- that an entry's index, its PENDING, OWNED and OPEN bits, or its line, are fixed: they move with the core's tier
  set.

## Verification

`tb/run.sh` runs the testbench in arms, one configuration each: the off-state guard (without `MISTER_LW`, `sys/` and
`Template.sv` read upstream's token stream under every macro set upstream's files test, and every LW file is empty);
static checks on the preprocessed top level (the GPI terms with both LW instances or neither, the lines, every
crossing built from the cells of `sys/lw_cdc.sv` with every source register named, `sys/lw_bridge.sdc` covering
every crossing source, and the wording of the framework's files and of this guide); `sys_top.v` elaborated against
stand-ins for off and each of the 32 tier sets in three upstream variants; `LW_DIAG=1` and `LW_DIAG=0`;
`MENU_CORE`; the floor alone; the Template's own configuration; the SECTOR block at its default geometry and on an
`hps_io` with `WIDE = 0`; the frame lane; each tier macro without `MISTER_LW` and each unservable geometry, which
must stop the compile naming its guard module; a short run in each of the 32 tier sets and with a core side that
leaves a piece out; the lane arm at eight `clk_sys` rates from 12 to 150 MHz, three seeds each, with delay and
aperture models in every crossing cell; a front-end lint; and the HPS-side probe compiled with warnings as errors.
Among what they cover:
- `LW_DIAG=1`; `LW_DIAG=0`; `MENU_CORE`, which checks the header rows;
- no SECTOR block, the Template's own configuration;
- the SECTOR block at the default geometry (128-word primary, 256-word save slots): a save-slot frame longer
  than the primary's completes, and the primary is served after it;
- the SECTOR block on an `hps_io` with `WIDE = 0`: a 4,096-word primary on slot 9 lands byte for byte as `hps_io`'s
  own SPI transfer of the block does, a 510-word save slot on slot 12 at its byte addresses, every strobe 5 cycles
  apart and 5 after the ack, and 32-bit save-slot LBAs round-trip and are compared whole;
- the frame lane: the whole reference run with every decoder behind `lw_link`, then lane frames against the same
  frames sent as link words (status, RTC, timestamp, mount, gamma, FIO, both OSDs), the `vs_wait` and
  `ioctl_wait` holds, a link word raised mid-frame, full-FIFO drops, unended frames, the longest frame, and reply
  capture (`0x29` word for word against the link, ordering across frames, overflow), and the lane's cause and
  line (REPLY raised by a word that lands while another waits and at the edge of the last word's RPOP, and a
  host that sleeps on the line and strands no reply over 10,000 random interleavings).

It needs Icarus Verilog 12 or newer, or set `LW_TB_IMAGE` to a container image that has it.

The SECTOR arms use a reference CD-style configuration: a 588-word primary on `hps_io` slot 1, and save slots on
slots 2 and 3 (1 KB and 768-byte frames). They cover:
- the host enable: every store dropped without it, every other word refused, no timer, each way it ends (an HPS
  reset, GPO bit 31 low for one cycle and long), the look at GPO and GPI bit 22 before identifying, a takeover, a
  stock start beside a live host, a new host starting from stock after every block was bound, the SECTOR and LANE
  blocks in every state at an ID view (with `clk_sys` running and stopped), and RESETS against a model of the
  firewall;
- the enable links, with a checker on each in every arm (the level rises only on an acknowledge of rest and falls
  only after the half has answered; no store reaches a linked block outside OPEN) and every ENABLE taken checked
  against the testbench's own rest predicate: ENABLEs stored as fast as the bus allows after an ID view, an ENABLE
  followed at once by work, the enable ended by GPO bit 31 and an HPS reset in every cycle of the rise, every
  core-clock register scrambled while enabled and during the return, a half that never answers and one that cannot
  return, a dead host's select with frames queued behind it, the causes at an ENABLE, a BEGIN discarded with the
  enable gone and its request found by the next host's EN, and a primary frame stored in full with the enable ended
  at each offset to its last strobe;
- the lines as levels, compared every cycle with `enable & LINE_EN & cause`: a raise in the cycle of a store-back
  keeps the line high, and a host woken only by the level serves every event across 10,000 random interleavings;
- the directory word by word, a host's walk of it (and of altered copies: other majors, entries to skip, a shared
  ordinal, a larger header and stride), and each entry's cause on its own PENDING bit and line;
- the window's decode: every 4 KB page of the 2 MB reads its block or `0xDEADBEEF`, and a store outside every block
  changes nothing;
- every pad op, including stores while `clk_sys` is stopped;
- the default responder and fixed latency;
- frames that are lost, resent, out of order, aborted, rejected or reset mid-frame;
- a save-slot read cut after every store count by ABORT, by a BEGIN or WACK and by an ID view, and by a host that
  falls silent: the core's buffer untouched with the request standing (and then served over SPI), or the frame
  whole; one whose BEGIN is taken after its EN fell is rejected whole;
- SECTOR requests dropped and raised again one `clk_sys` cycle later, held through their frame, and changed or
  rejected in every phase of a record's launch: each seen, answered and served once, and no slot left hidden;
- a save slot's read and write request shown by GET_SDSTAT and served over SPI with the EN store landing before,
  in and around the answer, across the transfer and after it: the mux never moves inside, the transfer is byte for
  byte and the next request is offered over LW; a mark left by a GET_SDSTAT with no transfer holds every slot on
  SPI until a GET_SDSTAT shows none; and, through the whole suite, no request acknowledged by both paths;
- save-slot reads and writes, each at its own frame length, and their priority;
- the EVT block: status brackets of four shapes at 64 phase offsets, a 64-offset read/capture race, and BLK, INFO
  and UPLOAD pulses at every spacing to sixteen `clk_sys` cycles, none lost;
- the OSD image over LW in both `osd_buffer`s byte for byte as over SPI, drawn the same through both pixel paths
  (plain, rotated, the info box and the `MENU_CORE` header rows), the FIFO full with `clk_sys` stopped, and queued
  bytes discarded when the enable falls.

**What the testbench cannot prove.** Crossing safety rests on structure and on timing reports, not on simulation:
every crossing is built from the cells of `sys/lw_cdc.sv`, which the static check holds, and `sys/lw_bridge.sdc`
bounds each one, which only a fitted design's timing report confirms. The delay models make the simulator resolve
synchronisers late, skew Gray buses and tear captures, so they reach cases a zero-delay run never does, but they
sample what a real synchroniser can do; they do not cover it. The off-state guard compares token streams; that an
off build's bitstream is upstream's is shown only by comparing the two rbfs, built on the same day (the build date
is part of `CONF_STR`). The timing rule above is checked on each core's own report.

`tb/hps/lw_probe.c` is an HPS-side program that walks this protocol from the look to the end (discovery, the
directory, the enable, OPEN, each block in turn, and with `--template` the Template's example) and prints every
step. It runs with no other program on the link, since it identifies the core. The testbench only compiles it.

Re-run the testbench after any `sys/` merge.

## Revision and validation

- **Template base:** MiSTer-devel/Template_MiSTer `e901a9f` (2026-11-03). This branch applies the LW changes to
  that revision's own `sys/`; its `hps_io` (FFB, analog triggers) is newer than the one the framework was first
  built on.
- **Register map:** the blocks' registers are those of a build validated on a DE10-Nano on 2026-10-25, which
  covered pads, the OSD image, SECTOR (CD and save slots) and the status line, except where this document
  states otherwise: CAUSE words clear by write-one and carry no class tag, a store is a single-beat write, PAD is
  version 1.1, which keeps every level latest-wins where that build queued the sticks and paddles with the
  spinners, EVT is version 2.0, which crosses the status whole and adds the BLK, INFO and UPLOAD causes where that
  build had a status block alone, OSDBUF is version 2.0, which writes `osd_buffer` itself where that build kept a
  second image RAM beside it, and SECTOR is version 2.0 at window `0x08000`, which indexes every per-slot register
  by slot index and raises a CAUSE bit per slot, only with that slot's EN set, where that build raised one bit for
  any request.
  The directory is layout 2, which that build did not have: a host written for its layout (magic `SYS1`) finds no
  directory here and stays on SPI.
- **The host side** is whatever HPS program speaks the protocol in this document: discovery, the directory, the
  host enable, and each block's registers, causes and lines. Nothing else about the host is assumed, and a host
  that never touches the window drives the core over SPI as before.
- **Not yet measured:** this revision has not been fitted, so every cost above is an estimate and the timing rule
  has not been checked on it; the off build's rbf has not been compared with upstream's at this revision.
- **This Template build has not been run on hardware.**
