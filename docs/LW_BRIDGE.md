# The LW bridge framework

An optional extension of `sys/` that gives the HPS a second, memory-mapped path into the core, beside the GPO/GPI ("SPI") link that `hps_io` decodes. It lives on the Cyclone V **HPS-to-FPGA lightweight bridge** (`lwhps2fpga`, physical `0xFF200000`, 2 MB). The HPS stores pads, OSD pixels and disk blocks as plain 32-bit writes, and reads the core's status back-channel and block requests as registers. Interrupt lines tell it when to look. An optional frame lane carries whole link transactions, and their replies, over the same bridge.

- **The core sees nothing new.** Pads land in the same `hps_io` registers (`joystick_0`, ...), blocks arrive as the same `sd_ack`/`sd_buff_wr`/`sd_buff_dout` strobes, and the status back-channel is the stock `status_in`/`status_set` handshake.

## Adopting it in a core

1. **The defines.** In the core's `.qsf` (in `Template.qsf` the lines are already there, commented out): the floor, then each tier the core wants. Every tier is opt-in, and a tier macro without `MISTER_LW` stops the build with an error naming `lw_tier_needs_MISTER_LW`.

### Macros

| macro | default | effect |
|---|---|---|
| `MISTER_LW` | off | the floor: the bridge, the window, its directory, the host enable and GPI bits 25, 24 and 22 |
| `MISTER_LW_PAD` | absent | with `MISTER_LW`: the PAD block (`lw_pad.sv`) and `hps_io`'s LW write arm |
| `MISTER_LW_EVT` | absent | with `MISTER_LW`: the EVT block (`lw_evt.sv`): `hps_io`'s status, block-request, info and upload flags as causes, `hps_io`'s exports and its line |
| `MISTER_LW_OSD` | absent | with `MISTER_LW`: the OSDBUF block (`lw_osd.sv`) and a write port into each `osd`'s `osd_buffer` |

**An unservable geometry stops the build** (in synthesis too), naming the undefined module `lw_sector_unservable_geometry`: a primary frame outside 1-4096 words, a save slot's outside 1-510, an LBA width outside 1-32, a gap outside 2-15, an `hps_io` slot above 15, two slots on one `hps_io` slot, or a save slot without an image base. Frames are one fixed length per slot; a slot whose requests vary in length or use `sd_blk_cnt` stays on SPI by not being declared.

### Tiers and their cost

A core pays only for what its `.qsf` names. Every figure is **ESTIMATED**: registers counted from the RTL, ALMs by analogy with a fit of an earlier revision. None of this revision has been fitted.

| tier | macro | builds | cost (ESTIMATED) |
|---|---|---|---|
| L0 floor | `MISTER_LW` | the firewall, the decode, the directory and its header words, the host enable, GPI bits 25, 24 and 22, the 384-bit `HPS_BUS` | 150-200 ALMs, no block memory |
| L1 pads | `MISTER_LW_PAD` | the PAD block, `hps_io`'s LW write arm | 150-185 ALMs, 1 M10K |
| L2 events | `MISTER_LW_EVT` | the EVT block, `hps_io`'s event exports, line 2 | 85-120 ALMs, 132 of its registers being the status hold on `clk_sys`; no block memory |
| L3 OSD | `MISTER_LW_OSD` | the OSDBUF block and its enable link, a second write source in each `osd` | 75-100 ALMs, 2 M10K |

## Discovery and safety

An LW access the fabric does not answer **hangs the SoC**. Nothing here is probed, so the HPS must know the window exists before it touches it:

- **`CONF_STR` field 1, `AXI:40000:80000:LW`:** a window at LW offset `0x40000`, `0x80000` bytes long.

### When a store is in effect

**A store is a single-beat write** (`AWLEN` 0). The beats of a burst write are accepted, answered OKAY and dropped, so a `memcpy` into the window, or a write-combined or cached mapping that merges stores into bursts, loses them.

**A store is in effect once its write response has returned.** The fabric applies a store in the cycle its data beat is accepted and answers it in the next; a read samples its word one cycle after its address is accepted. The read and write channels are independent, and the fabric orders nothing between them, so between a store and any read whose meaning depends on it the host must have the store's response. Map the window strongly ordered, where each access completes before the next begins (on ARMv7 Linux an `mmap` of `/dev/mem` or a UIO mapping, both non-cached), or, with a Device mapping (`ioremap`), issue a DSB after the store. A write-combined, cached or Normal-memory mapping is never legal: its stores may merge into bursts, reorder or repeat.

## The window

Offsets are from the window start; absolute LW offset = `0x40000` + offset. An address in no block that is built reads `0xDEADBEEF` and drops stores, inside the window or outside it, and an unassigned offset inside a block reads 0 and ignores stores.

| window offset | absolute LW | size | block |
|---|---|---|---|
| `0x00000` | `0x40000` | `0x1000` | directory: header and entries; the rest of the page reads 0 |
| `0x03000` | `0x43000` | `0x1000` | EVT (with `MISTER_LW_EVT`) |
| `0x04000` | `0x44000` | `0x2000` | OSDBUF (with `MISTER_LW_OSD`) |
| `0x20000` | `0x60000` | `0x20000` | reserved for a core's own blocks; not decoded in this revision |
| `0x40000` | `0x80000` | `0x40000` | PAD (with `MISTER_LW_PAD`) |

**The register bus.** Behind the firewall every block sits on one bus and keeps two rules: it answers every address combinationally from `clk_100m` registers and constants, and it never back-pressures. The bus has no read strobe, so no read of any address, size or burst changes anything. Each block's select compares the address's high bits with a constant, the selects are disjoint, each block's word is 0 unless it is selected, and the read word is one AND-OR of the block words and the `0xDEADBEEF` default. A block added later keeps the same rules.

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
| `0x2C` | OPEN | bit *i* = entry *i*'s block takes stores now: the enable for a block without an enable link, and for SECTOR, OSDBUF and LANE the enable and the block's core-clock half having answered its rise; a store to a linked block before its bit is set is dropped and sets `stale` |
| `0x30` | LINE_EN | read/write while enabled: bit *i* = entry *i*'s cause may raise its line; reads 0 whenever the enable does, and after every ENABLE taken |
| `0x34` | RESETS | `{freed[15:8], hrst[7:0]}`: the firewall's state machines freed at a reset's release, and the HPS resets the fabric saw; each counts from configuration and stops at 255 |
| `0x38`-`0x3C` | - | 0, reserved |

**Entries.** Entry *i* is at `header_len + i x entry_stride`, `0x40 + 0x80 x i` in this revision: 8 common words, then the kind's own words K0-K23. The table is compact and in ascending kind order, listing only the blocks this bitstream builds, so an entry's index (and its PENDING, OWNED and OPEN bits) depends on the core's configuration.

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
| 2 | SECTOR | `0x00020000` | `0x08000` / `0x08000` | `0x08000` | 1 | bit *k* per wired slot (`0x1`-`0x7`) | see SECTOR |
| 4 | EVT | `0x00020000` | `0x03000` / `0x01000` | `0x03000` | 2 | `0xF` | K0 = 4: the status words |
| 5 | LANE | `0x00010002` | `0x06000` / `0x01000` | `0x06024` | 3 | `0x7` | see LANE |

### The host enable

**No store has effect without the host enable, except a store to ENABLE itself.** Every other store, to any block or header word, is answered and dropped, and ENABLE's `stale` bit records it. Every block is off until a host that knows this window enables it and then binds each block with its own stores (SECTOR's EN, OSD image stores, lane frames, LINE_EN, pad stores), so a stock program that never touches the window meets a stock core.

**ENABLE** is set by a single full-word store of exactly `0x4C570001`; any other store to it is ignored. The store is taken only while the enable is clear, `quiet` is 1, no HPS reset is seen and GPO bit 31 is 1. Then the enable is set, `stale` and `burst` clear, LINE_EN reads 0, and the causes are reloaded (below).

| field | meaning |
|---|---|
| `en` | the enable is set; GPI bit 22 shows the same register |
| `quiet` | every enable link is at rest: an ENABLE may be taken. A block that cannot return (a stopped core clock, or a core that never answers the link) holds it at 0 for every host; OWNED names it |
| `stale` | a store other than to ENABLE was dropped since the last ENABLE taken |
| `burst` | a beat of a burst write was dropped since the last ENABLE taken |
| `hrst` | the fabric sees the HPS reset now; no ENABLE is taken while it is 1 |
| `idv` | GPO bit 31 reads 0 now (the ID view, or the core reset through GPO); no ENABLE is taken while it is 1 |

### Interrupt lines

A line is declared in one place: the LINE word of the entry it serves. Ordinal *k* is `f2h_irq[7 + k]`, the GIC's shared interrupt 47 + *k*, and no two entries name the same ordinal.

| ordinal | f2h line | GIC SPI | entry in this revision |
|---|---|---|---|
| 1 | `f2h_irq[8]` | 48 | SECTOR |
| 2 | `f2h_irq[9]` | 49 | EVT |
| 3 | `f2h_irq[10]` | 50 | LANE |
| 4-15 | `f2h_irq[11]`-`[22]` | 51-62 | framework kinds to come |
| 16-31 | `f2h_irq[23]`-`[38]` | 63-78 | a core's own kinds |

### PAD block (kind 1)

One store **is** one SPI set-command: a full 32-bit word (all four byte enables) at `PAD + (opcode << 10) + (index << 2)`.

| opcode | index | lands in |
|---|---|---|
| `0x02`, `0x03`, `0x10`-`0x13` | 0 | `joystick_0` ... `joystick_5`, all 32 bits |
| `0x1A` | 0-5 | `joystick_l_analog_<index>` ← `data[15:0]` |
| `0x1A` | `{p, 15}` | `index[7:4]` = *p*: `paddle_<p>` (0-5) or `spinner_<p-8>` (8-13) ← `data[7:0]` |
| `0x3D` | 0-5 | `joystick_r_analog_<index>` ← `data[15:0]` |
| `0x05` (`MISTER_LW_KBM`) | 0 | `ps2_key`, from the key frame's bytes, last byte lowest (as the link's frame leaves them in `ps2_key_raw`), formed and toggled as the link's frame end does it |
| `0x04` (`MISTER_LW_KBM`) | 1 | `ps2_mouse_ext` ← `data[15:0]` |
| `0x04` (`MISTER_LW_KBM`) | 0 | `ps2_mouse[23:0]` ← `data[23:0]`, and bit 24 toggles |
| `0x46` (`MISTER_LW_KBM`) | 0-5 | `{joystick_rt_analog_<index>, joystick_lt_analog_<index>}` ← `data[15:0]` |

- **Every level is latest-wins:** each of the 24 levels (the six digital pads, both sticks and the paddle of each player) keeps one slot, so a burst of stores delivers the last and nothing absolute is ever dropped, even while `clk_sys` is stopped.

### OSDBUF block (kind 3)

Image stores written into `osd_buffer` itself, in each `osd` instance (HDMI and VGA), through one write port beside the SPI one.

| offset | register | |
|---|---|---|
| `0x0000`-`0x13FF` | IMAGE | W: byte *a* = `osd_buffer[a]`, row `a[12:8]`, column `a[7:0]`: one byte = 8 vertical pixels. Rows 16-19 (the `MENU_CORE` header rows) only with `MENU_CORE`. Byte enables honoured; reads 0 |
| `0x1800` | STATUS | R: `{7'd0, level[24:16], drops[15:0]}`: entries waiting (0-256); stores dropped on a full FIFO, free-running |
| `0x1804` | ID | `0x3244534F` ("OSD2") |

- **The FIFO:** each store takes one entry `{byte enables, word, data}` of a 256-entry FIFO; a store that finds it full is dropped and counted. The address rides each entry, so a dropped store loses only its own word and storing it again repairs it.

### EVT block (kind 4, "EVT2"): the core's flags

`hps_io`'s core-to-host flags as causes on one line (ordinal 2, `f2h_irq[9]`), so a host need not poll `0x16`, `0x29`, `0x36` or `0x3C` on any core, including one whose blocks stay on SPI. The SPI arms still answer as before, and serving them is how a host clears the levels.

| offset | register | |
|---|---|---|
| `0x00` | CAUSE | bit 0 STATUS: a status transfer landed (and at a taken ENABLE); bit 1 BLK: a slot's block request rose, or a slot's SPI ack fell with its request still raised; bit 2 INFO: `info_req` rose; bit 3 UPLOAD: `ioctl_upload_req` rose. A store clears each bit written as 1 |
| `0x04` | STFLG | `{SEQ[15:0], 8'h00, 4'hA, flag[3:0]}`: the `0x29` head byte, plus a count of status transfers |
| `0x08`-`0x14` | STATUS[0..3] | the transferred `status_req`, `[31:0]` first |
| `0x18` | PEND | levels: bit 1 a block request stands at `hps_io`, bit 2 info pending (`0x36` clears it), bit 3 upload pending (`0x3C` clears it); each bit an independent sample |
| `0x1C` | ID | `0x32545645` ("EVT2") |

- **The status crosses whole.** When the core raises `status_set`, `{status_req, flag}` crosses to the bridge in one held-data handshake: a transfer is never torn, at any `clk_sys`. Requests faster than the handshake coalesce into the latest, and the latest always arrives.

## Clock-domain contracts

The bridge side runs on `clk_100m` (`h2f_user0_clk`). Every crossing to the core is on the core's `clk_sys`, and every crossing is built from the cells of `sys/lw_cdc.sv`: a two-flop synchroniser, an asynchronously set reset chain, and a held-data handshake with its capture register. Nothing else crosses, and no crossing reaches a video clock.

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
| OSDBUF | image stores | a dual-clock FIFO, Gray pointers stepping one entry per update, into the writer on `clk_sys`, which writes `osd_buffer` on `clk_sys` as the SPI path does | none |
| LANE | frames and replies | dual-clock FIFOs, Gray pointers stepping one entry per update | none |
| LANE | PLAYED and the reply-drop count | toggles, at most one per `strobe_gap + 5` and per `strobe_gap + 2` `clk_sys` cycles; each must outlast two `clk_100m` periods | below about 250 MHz |

## `HPS_BUS` with `MISTER_LW`

384 bits whatever the tiers. `lw_sys_window` owns bits `[383:46]` through one port and drives every to-core bit (0 for a tier not built); a from-core bit is driven only by the piece named, when its tier is built. A bit no piece drives reads 0. Each piece drives a constant 1 on its presence bit, and the window lists a block only when its tier is built and its presence bit reads 1 (SECTOR: `hps_io`'s commit mark and at least one slot's mux; a slot whose mux is absent has flags 0 in its descriptor and an EN bit that cannot be set). LANE and OSDBUF use no `HPS_BUS` bit and have no presence bit.

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
| `[383]` | - | reserved: nothing drives it; every reader ignores it |

## What a host must not assume

- that a store has effect without the enable: only the ENABLE store does;
- that a store to SECTOR, OSDBUF or LANE has effect before that entry's OPEN bit is set;
