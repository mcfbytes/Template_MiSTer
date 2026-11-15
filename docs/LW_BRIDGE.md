# The LW bridge framework

An optional extension of `sys/` that gives the HPS a second, memory-mapped path into the core, beside the
GPO/GPI ("SPI") link that `hps_io` decodes. It lives on the Cyclone V **HPS-to-FPGA lightweight bridge**
(`lwhps2fpga`, physical `0xFF200000`, 2 MB). The HPS stores pads, OSD pixels and disk blocks as plain 32-bit
writes, and reads the core's status back-channel and block requests as registers. Two interrupt lines tell it
when to look.

- **The core sees nothing new.** Pads land in the same `hps_io` registers (`joystick_0`, ...), blocks arrive as the
  same `sd_ack`/`sd_buff_wr`/`sd_buff_dout` strobes, and the status back-channel is the stock
  `status_in`/`status_set` handshake.
- **The SPI link is untouched.** `hps_io` keeps every SPI decode, so a host that knows nothing about the window (the
  stock MiSTer Main) drives an LW-enabled bitstream exactly as before. The window is a second carrier, not a
  replacement.
- **Opt-in.** Without `MISTER_LW` the framework is not built: `HPS_BUS` stays 46 bits, `CONF_STR` is the stock
  string byte for byte, and `sys/` elaborates to the stock netlist.

Status: proof of concept. The register map is stable for this revision; later revisions may add blocks (new
directory kinds) and use entry word 15. See "Revision and validation" at the end.

## Adopting it in a core

1. **The define.** In the core's `.qsf`:

   ```
   set_global_assignment -name VERILOG_MACRO "MISTER_LW=1"
   ```

2. **The declaration.** Put `` `SYS_LW_CAPS `` in field 1 of `CONF_STR`, with no comma after it (the macro
   carries its own):

   ```verilog
   localparam CONF_STR = {
       "MyCore;", `SYS_LW_CAPS "SS3E000000:400000;",   // or `SYS_LW_CAPS ";" when field 1 is empty
       ...
   ```

   With `MISTER_LW` it expands to `"AXI:40000:80000:LW,",` (plus `IRQ:BLK=1:41000,` when the core declares a
   SECTOR block). Without it, it expands to nothing.

3. **Nothing else for pads, the OSD shadow and the status back-channel.** A core that uses `hps_io` gets all
   three. `Template.sv` shows the status back-channel: the first pad button cycles an option and writes the new
   status word back with `status_in`/`status_set`.

4. **Optional: the SECTOR block**, for a core that reads blocks through an `hps_io` slot ("Hooking the SECTOR block",
   below).

A core that instantiates its own modules on `HPS_BUS` must widen those ports to `[313:0]` under `MISTER_LW`; bits
45:0 keep their stock meaning.

### Macros

| macro | default | effect |
|---|---|---|
| `MISTER_LW` | off | builds the bridge, the window, the `hps_io` LW path, the OSD shadow and GPI bit 27 |
| `LW_DIAG` | 0 | 1 = debug build: SCRATCH, the analog-drop count and the SECTOR counters/timestamps read back |
| `MISTER_LW_SECTOR_SLOT` | absent | the `hps_io` slot the SECTOR block feeds; its presence builds the block |
| `MISTER_LW_SECTOR_FRAME` | 128 | 32-bit words per primary frame, 1-1023 (128 = one 512-byte block) |
| `MISTER_LW_BLKSZ` | 2 | the core's `hps_io` `BLKSZ`, reported in the GET_SDSTAT words |
| `MISTER_LW_CARD0_SLOT`, `_WSRC`, `_EXTENT` | absent | save slot 0: `hps_io` slot (0-3), DDR image base, image bytes |
| `MISTER_LW_CARD1_SLOT`, `_WSRC`, `_EXTENT` | absent | save slot 1, likewise |
| `MISTER_LW_CARD_FRAME` | 256 | 32-bit words per save-slot frame (256 = one 1 KB block) |

A save slot is declared only when its `_WSRC` macro is set, and needs `MISTER_LW_SECTOR_SLOT` too.

## Discovery and safety

An LW access the fabric does not answer **hangs the SoC**. Nothing here is probed, so the HPS must know the
window exists before it touches it:

- **`CONF_STR` field 1, `AXI:40000:80000:LW`:** a window at LW offset `0x40000`, `0x80000` bytes long.
- **GPI bit 27 = 1** (the FPGA manager's `gpi` register, read with `gp_out[31] = 1`, always safe): a directory is
  decoded at LW `0x40000`. Stock bitstreams leave it 0, and the stock Main never reads bits 21-27.
- **The directory magic `SYS1`** at the window's first word, then the entries.

`hps_lw_bridge.sv` answers **every** address of the 2 MB bridge, every burst and both channels, in fixed cycles.
It has no reset, so a read during a core reset still completes. Addresses outside every block read `0xDEADBEEF`,
and writes there are dropped.

Nothing back-pressures. A store is accepted in fixed cycles. Where a FIFO is full, the store is dropped and counted,
and the protocol detects the loss (PAD analog drops, SECTOR `ACCEPT`).

## The window

Offsets are from the window start; absolute LW offset = `0x40000` + offset.

| window offset | absolute LW | size | block |
|---|---|---|---|
| `0x00000` | `0x40000` | `0x110` | directory: header + 4 entries (to `0x1000`: reads `0xDEADBEEF`) |
| `0x01000` | `0x41000` | `0x2000` | SECTOR (reads 0 when not built) |
| `0x03000` | `0x43000` | `0x1000` | SYS |
| `0x04000` | `0x44000` | `0x2000` | OSDBUF |
| `0x40000` | `0x80000` | `0x40000` | PAD |

### Directory

| offset | word | value |
|---|---|---|
| `0x00` | MAGIC | `0x31535953` ("SYS1") |
| `0x04` | VERSION | `0x00010000`: major 1 `[31:16]`, minor 0 `[15:0]` |
| `0x08` | COUNT | 4 |
| `0x0C` | SCRATCH | read/write with `LW_DIAG=1`; 0 otherwise |
| `0x10 + 0x40*i` | entry *i* | 16 words: 0 kind, 1 version, 2 offset (window-relative), 3 size, 4-14 per kind, 15 reserved (0) |

| entry | kind | version | offset / size | words 4+ |
|---|---|---|---|---|
| 0 | 1 PAD | `0x00010000` | `0x40000` / `0x40000` | 4-11: the honoured-opcode bitmap, bit *n* = opcode *n*: `0x040F000C`, `0x20000000`, then 0 |
| 1 | 2 SECTOR | `0x00010001` | `0x1000` / `0x2000` | see SECTOR. All 16 words are 0 when the block is not built |
| 2 | 3 OSDBUF | `0x00010000` | `0x4000` / `0x2000` | 4: geometry `{height, width}` = `0x00800100` (256 x 128, 1 bpp) |
| 3 | 4 SYS | `0x00010000` | `0x3000` / `0x1000` | 4: line = pool ordinal 2; 5: CAUSE absolute = `0x43000`; 6: class tag `0x00100000`; 7: causes = `0x1` (STATUS) |

A host matches entries by kind, refuses a major it does not know, and checks that every block lies inside the window.

### Interrupt lines

| f2h line | GIC SPI | pool ordinal | raised by | level |
|---|---|---|---|---|
| `f2h_irq[8]` | 48 | 1 | SECTOR CAUSE (`IRQ:BLK=1:41000`) | high while the CAUSE word is non-zero |
| `f2h_irq[9]` | 49 | 2 | SYS CAUSE (directory entry 3) | high while the CAUSE word is non-zero |

Both are level lines into the GIC. The HPS kernel must expose them to userspace (for example as
`uio_pdrv_genirq` nodes); a host without them can poll the CAUSE words instead. A CAUSE word's `[23:16]` is a one-hot
class tag (bit 16 = BLK, bit 20 = SYS) and `[15:0]` its payload. A **word-exact read clears it**: a single 32-bit
aligned beat, never a byte read or a burst passing through. If a new event arrives between the read's fetch and its
completion, the bit stays set and the line stays high.

### PAD block (kind 1)

One store **is** one SPI set-command: a full 32-bit word (all four byte enables) at `PAD + (opcode << 10) + (index << 2)`.

| opcode | index | lands in |
|---|---|---|
| `0x02`, `0x03`, `0x10`-`0x13` | 0 | `joystick_0` ... `joystick_5`, all 32 bits |
| `0x1A` | 0-5 | `joystick_l_analog_<index>` ← `data[15:0]` |
| `0x1A` | `{p, 15}` | `index[7:4]` = *p*: `paddle_<p>` (0-5) or `spinner_<p-8>` (8-13) ← `data[7:0]` |
| `0x3D` | 0-5 | `joystick_r_analog_<index>` ← `data[15:0]` |

- **Digital pads** keep one latest-wins slot per opcode, so a burst of stores delivers the last.
- **Analog stores** queue in a 16-deep FIFO. A store to a full FIFO is dropped (with `LW_DIAG=1`, the count reads at
  `PAD + (0x1A << 10)`).
- **Ignored:** a store to an opcode outside the bitmap, a digital store at index ≠ 0, and a partial-width store.
- **Reads** return 0.
- **Delivery:** a toggle handshake drains the slots round-robin to `clk_sys`. `hps_io` then writes the same registers
  its SPI arms write. Analog triggers (`0x46`) and every other opcode stay on SPI.

### OSDBUF block (kind 3)

A shadow of `osd_buffer` in each `osd` instance (HDMI and VGA), written over the bridge.

| offset | register | |
|---|---|---|
| `0x0000`-`0x0FFF` | IMAGE | byte at `row * 256 + column` (row 0-15), `osd_buffer`'s layout: one byte = 8 vertical pixels. Byte enables honoured |
| `0x1000` | CTRL | `[0]` LWSEL: video reads the shadow instead of `osd_buffer`. `[1]` HR: with LWSEL, the 16-row (highres) size |
| `0x1004` | ID | `0x3144534F` ("OSD1") |

- **Still on SPI:** enable/disable, the info box, rotation and `osd_status`.
- **Highres:** the SPI rule OR (LWSEL & HR).
- **Header rows** (the `MENU_CORE` title rows) never read the shadow.
- **Cost:** four 1024x8 M10K per `osd` instance.

### SYS block (kind 4): the status back-channel

`hps_io`'s `0x29` back-channel without the poll: when the core raises `status_set`, the block captures
`status_req` and raises STATUS on `f2h_irq[9]`. The SPI `0x29` arm still answers as before.

| offset | register | |
|---|---|---|
| `0x00` | CAUSE | `0x00100001` while pending, else 0; a word-exact read clears it |
| `0x04` | STFLG | `{SEQ[15:0], 8'h00, 4'hA, flag[3:0]}`: the `0x29` head byte, plus a capture count |
| `0x08`-`0x14` | STATUS[0..3] | the captured `status_req`, `[31:0]` first |
| `0x1C` | ID | `0x31535453` ("STS1") |

**Read it as a seqlock:**
1. Read CAUSE.
2. Read STFLG.
3. Read the four STATUS words.
4. Read STFLG again.

If the two STFLG words are equal, the four words are one capture. If not, read again. Captures that arrive faster
than the host reads coalesce into the latest. None is lost.

### SECTOR block (kind 2, "SEC2")

The block serves one primary `hps_io` read slot and up to two save slots ("cards") through one feeder. The feeder
replays `hps_io`'s own handshake into the core: `sd_ack`, then two 16-bit `sd_buff_wr` strobes per 32-bit word,
low half first.

**Directory entry 1, words 4-12:**

| word | value |
|---|---|
| 4 | `{slot[31:24], 0, frame_words[9:0]}` |
| 5 | the CAUSE word's absolute LW offset, `0x41000` (the `IRQ:BLK` token's) |
| 6 | `{fifo_depth[31:16] = 512, strobe_gap[15:0] = 2}` |
| 7, 9 | card 0 / card 1: `{0, slot[27:24], flags[23:16], frame_words[9:0]}`, flags bit 0 = RD, bit 1 = WR |
| 8, 10 | card 0 / card 1: DDR image base (physical) |
| 11, 12 | card 0 / card 1: image size in bytes |

An undeclared card's words are 0.

**Registers, from the block's start:**

| offset | register | |
|---|---|---|
| `0x00` | CAUSE | `{0, 4'b0001, GET_SDSTAT word}` while pending; word-exact read clears; a write clears |
| `0x04` | REQ | `{pending[31], 0, GET_SDSTAT word[15:0]}` of the highest-priority open request (primary, card 0, card 1) |
| `0x08` | LBA | the primary's LBA, latched when its request rises |
| `0x0C` | BEGIN | write the LBA: opens a primary frame |
| `0x10` | CTRL | `[0]` EN primary, `[1]` ABORT (zero-fills the open frame), `[2]` CLR, `[3]` TEST (CLR/TEST: `LW_DIAG=1`), `[4+s]` EN card slot *s* |
| `0x14` | STATUS | `[3:0]` `{rejecting, streaming, ack, en_sys}`; with `LW_DIAG=1` also `{frames[31:16], high-water[15:8]}` |
| `0x18` | ACCEPT | `{ok[31], drops[25:16], next_index[9:0]}` for the frame in flight |
| `0x1C`-`0x34` | DROPS, COUNTS, GEN, TSTAMP_REQ, TSTAMP_DONE, CYCLES, SUM | `LW_DIAG=1` only; read 0 otherwise |
| `0x38` | ID | `0x32434553` ("SEC2") |
| `0x40 + 4s` | BEGIN_S[s] | write the LBA: opens a READ frame for slot *s*; `BEGIN_S[primary]` = BEGIN |
| `0x50 + 4s` | WACK_S[s] | write the LBA: answers a card WRITE with an ack pulse and no strobes |
| `0x60 + 4s` | LBA_S[s] | slot *s*'s latched LBA |
| `0x1000 + 4i` | DATA[i] | word *i* of the open frame, *i* < that frame's frame_words (a store past it is ignored) |

GET_SDSTAT word: `{1, 6'd0, BLKSZ[2:0], slot[3:0], wr, rd}`.

**One frame:**
1. The core raises `sd_rd[slot]`. While EN is set, `lw_blk_mux` hides the request from `hps_io`. REQ and CAUSE show
   it, and `f2h_irq[8]` rises.
2. The host reads CAUSE (which clears it) and LBA.
3. The host writes BEGIN with that LBA. A BEGIN whose LBA is not the core's current request is rejected, and the
   request re-opens.
4. The host stores DATA[0 .. n-1], then reads ACCEPT.
5. If ACCEPT's `ok` is clear, the host resends from `next_index`.

**Cards:**
- A card READ is the same flow on BEGIN_S.
- A card WRITE is answered by WACK_S; the host takes the bytes from the core's own DDR image at `wsrc`.
- ABORT, or a core reset mid-frame, zero-fills the rest of the open frame, so the core is never left waiting.

## Hooking the SECTOR block

The block replaces the HPS side of one `hps_io` slot. The core keeps its own reader. `sys/lw_blk_mux.sv` sits
between `hps_io` and that reader:

```verilog
// .qsf: VERILOG_MACRO "MISTER_LW_SECTOR_SLOT=1", "MISTER_LW_SECTOR_FRAME=128", "MISTER_LW_BLKSZ=2"
wire        blk_req, blk_ack, blk_wr;
wire [15:0] blk_data;
lw_blk_mux blk_lw
(
    .HPS_BUS(HPS_BUS), .reset(reset),
    .core_req(blk_req), .core_lba(sd_lba[1]), .core_ack(blk_ack), .core_wr(blk_wr), .core_data(blk_data),
    .hps_req(sd_rd[1]), .hps_ack(sd_ack[1]), .hps_wr(sd_buff_wr), .hps_data(sd_buff_dout)
);
// the reader takes blk_req/blk_ack/blk_wr/blk_data where it took sd_rd[1]/sd_ack[1]/sd_buff_wr/sd_buff_dout
```

- **The mux only switches between requests.** While the host's EN is off, everything passes through to `hps_io` and
  SPI serves the slot. Each slot's mux switches only while the slot is idle and its SPI ack is low.
- **The core's write strobe is muxed, never `hps_io`'s shared `sd_buff_wr`.** An SPI transfer on another slot
  therefore cannot land in an LW frame.
- **Card slots** use `lw_card_mux #(.CARD(0|1))` the same way, on `sd_rd`/`sd_wr`/`sd_buff_addr`. A card's LBA is 7
  bits (128 blocks) in this revision, and the card's image must sit in DDR at `wsrc`, where the host reads it for
  writes. A card slot must differ from the primary and lie in 0-3.
- **Frames are block-sized:** `frame_words` x 4 bytes is what the reader expects per request. 588 words (2,352 bytes)
  serves a raw CD sector.
- **The host decides.** It serves a slot only when it knows what a frame on that slot means. A core gains nothing
  from the block until the host knows it.

## Clock-domain contracts

The bridge side runs on `clk_100m` (`h2f_user0_clk`). Every crossing to the core is on the core's `clk_sys`.

- **PAD:** a toggle handshake with the data held at least two `clk_sys` cycles. No ratio bound.
- **SYS:** the flag crosses as a registered Gray code. `hps_io` moves it at most once per two `clk_sys` cycles, so
  `clk_sys` must stay below about 100 MHz (50 MHz has margin). A faster core must widen the crossing to a handshake.
- **SECTOR reject path:** a rejected answer's `{slot, tag}` and its toggle coexist for one `clk_sys` period. Keep
  `clk_sys` below about 50 MHz, the conservative bound; the analysed edge is near 70-100 MHz. A faster core must hold
  `F_REJECT` for `ceil(2 x f_sys / 100 MHz)` cycles, or widen the crossing.
- **SECTOR FIFO:** dual-clock with Gray pointers. No ratio bound.
- **OSDBUF CTRL:** quasi-static bits through two-flop synchronisers in each `osd`.
  - In this Template, `clk_100m` and the video clocks are in exclusive clock groups (`sys_top.sdc`), so no extra
    constraint is needed.
  - A core whose video clock comes from a PLL outside those groups needs a false path from
    `*lw_sys_window|octrl[0]` to `*|lwsel_v0`.

## `HPS_BUS` with `MISTER_LW`

| bits | direction | content |
|---|---|---|
| `[45:0]` | both | stock |
| `[94:46]` | to core | PAD store `{we, op[7:0], idx[7:0], data[31:0]}` (`hps_io`) |
| `[113:95]` | to core | SECTOR primary `{en_sys, ack, wr, data[15:0]}` (`lw_blk_mux`) |
| `[148:114]` | from core | SECTOR primary `{reset, hps_ack, req, lba[31:0]}` |
| `[157:149]` | to core | card strobe address `{word, half}` |
| `[159:158]`, `[171:170]` | to core | card 0, card 1 `{ack, en_sys}` |
| `[169:160]`, `[181:172]` | from core | card 0, card 1 `{hps_ack, wr, rd, lba[6:0]}` |
| `[313:182]` | from core | `{stflg[3:0], status_req[127:0]}` (`hps_io`) |

## Verification

`tb/run.sh` runs the testbench in five arms:
- `LW_DIAG=1`;
- `LW_DIAG=0`;
- `MENU_CORE`, which checks the header rows;
- no SECTOR block, the Template's own configuration;
- the SECTOR block at the default geometry (128-word primary, 256-word save slots): a save-slot frame longer
  than the primary's completes, and the primary is served after it.

It needs Icarus Verilog 12 or newer, or set `LW_TB_IMAGE` to a container image that has it.

The SECTOR arms use a reference CD-style configuration: a 588-word primary on slot 1, and 1 KB cards on slots 2
and 3. They cover:
- the directory word by word;
- every pad op, including stores while `clk_sys` is stopped;
- the default responder and fixed latency;
- frames that are lost, resent, out of order, aborted, rejected or reset mid-frame;
- card reads and writes, and their priority;
- the SYS seqlock, including a 64-offset read/capture race;
- the OSD shadow drawn through both `osd` pixel paths.

Re-run it after any `sys/` merge.

## Revision and validation

- **Template base:** MiSTer-devel/Template_MiSTer `e901a9f` (2026-11-03). This branch applies the LW changes to
  that revision's own `sys/`; its `hps_io` (FFB, analog triggers) is newer than the one the framework was first
  built on.
- **Register map:** identical to the build validated on a DE10-Nano on 2026-10-25, which covered pads, OSD shadow,
  SECTOR (CD and cards) and the SYS status line. That build was driven by the reference host at revisions
  `8a4726d6f`/`ed20140c1`. The testbench's reference configuration reproduces that build's directory word for word.
  The host's directory, PAD, OSDBUF, SYS and SECTOR parsers at revision `b7ee0f09e` (2026-11-14) were checked
  against this branch.
- **This Template build has not been run on hardware.**

## Cost and fit (Quartus 17.0.2 Lite, this Template, 2026-11-14)

| build | ALMs | registers | block memory bits | RAM blocks | rbf |
|---|---|---|---|---|---|
| upstream `e901a9f`, stock | 7,273 | 11,226 | 384,501 | 59 | md5 `0b49f57e…` |
| this branch, `MISTER_LW` off (upstream core files, this `sys/`) | 7,273 | 11,226 | 384,501 | 59 | md5 `0b49f57e…`, bit-identical |
| this branch, `MISTER_LW=1`, no SECTOR | 7,768 (+495) | 12,150 (+924) | 450,357 (+65,856) | 68 (+9) | |

Timing, worst setup slack in ns (hold is positive in every build):

| build | HDMI clock | bridge clock (`h2f_user0_clk`) |
|---|---|---|
| stock, seed 1 | +0.514 | +3.461 |
| `MISTER_LW=1`, seed 1 | **-0.125** | +2.771 |
| `MISTER_LW=1`, seed 2 | +0.261 | +2.926 |
| `MISTER_LW=1`, seed 3 | **-0.154** | +2.215 |

- **The HDMI failures are inside the scaler.** Every failing path runs from `ascal` to `ascal` (`o_vcpt_pre3` →
  `o_fload`, `o_hcpt` → `o_vcpt`); the extra logic moves its placement. The OSD shadow's own path into
  `osd_pixel` has +0.38 ns.
- **The bridge clock's worst path** is inside the PAD drain (`arp` → `rr`), with 2.2-2.9 ns to spare.
- **Seed 2 closes timing on every clock.**

A SECTOR build with two save slots elaborates from the macros (synthesis checked the decoded parameters). Its cost is
not measured here, because the Template has no block reader to keep the feeder's outputs alive.
