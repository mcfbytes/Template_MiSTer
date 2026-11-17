# The LW bridge framework

An optional extension of `sys/` that gives the HPS a second, memory-mapped path into the core, beside the
GPO/GPI ("SPI") link that `hps_io` decodes. It lives on the Cyclone V **HPS-to-FPGA lightweight bridge**
(`lwhps2fpga`, physical `0xFF200000`, 2 MB). The HPS stores pads, OSD pixels and disk blocks as plain 32-bit
writes, and reads the core's status back-channel and block requests as registers. Two interrupt lines tell it
when to look. An optional frame lane carries whole link transactions, and their replies, over the same bridge.

- **The core sees nothing new.** Pads land in the same `hps_io` registers (`joystick_0`, ...), blocks arrive as the
  same `sd_ack`/`sd_buff_wr`/`sd_buff_dout` strobes, and the status back-channel is the stock
  `status_in`/`status_set` handshake.
- **The SPI link is untouched.** `hps_io` keeps every SPI decode, so a host that knows nothing about the window (the
  stock MiSTer Main) drives an LW-enabled bitstream exactly as before. The window is a second carrier, not a
  replacement.
- **Opt-in.** Without `MISTER_LW` the framework is not built: `HPS_BUS` stays 46 bits, `CONF_STR` is the stock
  string byte for byte, and `sys/` elaborates to the stock netlist. This repository's `Template.qsf` carries the
  define commented out, so the Template builds stock until it is uncommented.

Status: proof of concept. The register map is stable for this revision; later revisions may add blocks (new
directory kinds) and use entry word 15. See "Revision and validation" at the end.

## Adopting it in a core

1. **The define.** In the core's `.qsf` (in `Template.qsf` the line is already there, commented out):

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

5. **Optional: the frame lane** (`MISTER_LW_LANE`). It needs nothing in the core either: it lives in `sys_top`.

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
| `MISTER_LW_LANE` | absent | with `MISTER_LW`: builds the frame lane (LANE block) and its link front, `lw_link` |

A save slot is declared only when its `_WSRC` macro is set, and needs `MISTER_LW_SECTOR_SLOT` too.

## Discovery and safety

An LW access the fabric does not answer **hangs the SoC**. Nothing here is probed, so the HPS must know the
window exists before it touches it:

- **`CONF_STR` field 1, `AXI:40000:80000:LW`:** a window at LW offset `0x40000`, `0x80000` bytes long.
- **GPI bit 27 = 1** (the FPGA manager's `gpi` register, read with `gp_out[31] = 1`, always safe): a directory is
  decoded at LW `0x40000`. Stock bitstreams leave it 0, and the stock Main never reads bits 21-27.
- **The directory magic `SYS1`** at the window's first word, then the entries.

`hps_lw_bridge.sv` answers **every** address of the 2 MB bridge, every burst and both channels, in fixed cycles.
No reset holds it, so a read during a core reset, or while the HPS holds the fabric reset, still completes. After an
HPS reset it drops only a transaction the reset orphaned: one whose master made no handshake on its channel between
the reset reaching the fabric and its release. The first access after the reset is answered; nothing else is
touched. A reset of the bridge alone is invisible to the fabric, so **no LW access may be in flight when anything
resets the bridge, asserts the fabric reset or programs the FPGA**: a transaction left in flight through a
bridge-only reset holds its channel until the next HPS reset or core load. Addresses outside every block read
`0xDEADBEEF`, and writes there are dropped.

Nothing back-pressures. A store is accepted in fixed cycles. Where a FIFO is full, the store is dropped and counted,
and the protocol detects the loss (PAD analog drops, SECTOR `ACCEPT`).

**A store is a single-beat write** (`AWLEN` 0). The beats of a burst write are accepted, answered OKAY and dropped,
so a `memcpy` into the window, or a write-combined or cached mapping that merges stores into bursts, loses them.
**A store is in effect once its write response has returned**: read anything that depends on it (a CAUSE word after
clearing it, ACCEPT after the last DATA, RHEAD after RPOP) only after that. Map the window strongly ordered, where
each access completes before the next begins (on ARMv7 Linux an `mmap` of `/dev/mem` or a UIO mapping, both
non-cached), or, with a Device mapping (`ioremap`), issue a DSB after the store. Read bursts of every type return
the words of the addresses they step through; one that would cross a 4 KB boundary (an AXI master never issues
one) wraps inside its 4 KB.

## The window

Offsets are from the window start; absolute LW offset = `0x40000` + offset.

| window offset | absolute LW | size | block |
|---|---|---|---|
| `0x00000` | `0x40000` | `0x110` | directory: header + 4 entries, or `0x150` and 5 with the lane (to `0x1000`: reads `0xDEADBEEF`) |
| `0x01000` | `0x41000` | `0x2000` | SECTOR (reads 0 when not built) |
| `0x03000` | `0x43000` | `0x1000` | SYS |
| `0x04000` | `0x44000` | `0x2000` | OSDBUF |
| `0x06000` | `0x46000` | `0x1000` | LANE (with `MISTER_LW_LANE`; otherwise reads `0xDEADBEEF`) |
| `0x40000` | `0x80000` | `0x40000` | PAD |

### Directory

| offset | word | value |
|---|---|---|
| `0x00` | MAGIC | `0x31535953` ("SYS1") |
| `0x04` | VERSION | `0x00010000`: major 1 `[31:16]`, minor 0 `[15:0]` |
| `0x08` | COUNT | 4, or 5 with the lane |
| `0x0C` | SCRATCH | read/write with `LW_DIAG=1`; 0 otherwise |
| `0x10 + 0x40*i` | entry *i* | 16 words: 0 kind, 1 version, 2 offset (window-relative), 3 size, 4-14 per kind, 15 reserved (0) |

| entry | kind | version | offset / size | words 4+ |
|---|---|---|---|---|
| 0 | 1 PAD | `0x00010000` | `0x40000` / `0x40000` | 4-11: the honoured-opcode bitmap, bit *n* = opcode *n*: `0x040F000C`, `0x20000000`, then 0 |
| 1 | 2 SECTOR | `0x00010001` | `0x1000` / `0x2000` | see SECTOR. All 16 words are 0 when the block is not built |
| 2 | 3 OSDBUF | `0x00010000` | `0x4000` / `0x2000` | 4: geometry `{height, width}` = `0x00800100` (256 x 128, 1 bpp) |
| 3 | 4 SYS | `0x00010000` | `0x3000` / `0x1000` | 4: line = pool ordinal 2; 5: CAUSE absolute = `0x43000`; 6: class tag `0x00100000`; 7: causes = `0x1` (STATUS) |
| 4 | 5 LANE | `0x00010001` | `0x6000` / `0x1000` | see LANE. Present only with `MISTER_LW_LANE` |

A host matches entries by kind, refuses a major it does not know, and checks that every block lies inside the window.

### Interrupt lines

| f2h line | GIC SPI | pool ordinal | raised by | level |
|---|---|---|---|---|
| `f2h_irq[8]` | 48 | 1 | SECTOR CAUSE (`IRQ:BLK=1:41000`) | high while the CAUSE word is non-zero |
| `f2h_irq[9]` | 49 | 2 | SYS CAUSE (directory entry 3) | high while the CAUSE word is non-zero |

Both are level lines into the GIC. The HPS kernel must expose them to userspace (for example as
`uio_pdrv_genirq` nodes); a host without them can poll the CAUSE words instead. A CAUSE word's `[15:0]` are event
bits, each raised by every occurrence of its event, and `[23:16]` a read-only one-hot class tag (bit 16 = BLK, bit
20 = SYS), shown while a bit is set; the word is 0 when nothing is pending. **Reads change nothing**, in any size or
burst. **A full 32-bit store clears each bit written as 1**; a bit raised in the store's own cycle stays set, and a
partial-width store clears nothing.

**Drain:** read CAUSE; store the value back; once that store's response has returned, read the block's level
registers and serve what they show; after the last service, read the level again until it shows nothing; then
re-arm the interrupt. An event before the store-back is in the level read after it, and one after it sets its bit
again, so the line is high at the re-arm. A bit the host does not know is cleared by the store-back.

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
| `0x00` | CAUSE | `0x00100001` while pending, else 0: bit 0 = STATUS, raised by every capture; a store of bit 0 clears it |
| `0x04` | STFLG | `{SEQ[15:0], 8'h00, 4'hA, flag[3:0]}`: the `0x29` head byte, plus a capture count |
| `0x08`-`0x14` | STATUS[0..3] | the captured `status_req`, `[31:0]` first |
| `0x1C` | ID | `0x31535453` ("STS1") |

**Read it as a seqlock:**
1. Read CAUSE and store the value back.
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
| `0x00` | CAUSE | `0x00010001` while pending, else 0: bit 0 = a request opened or re-opened; a store of bit 0 clears it |
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
1. The core raises `sd_rd[slot]`. While EN is set, `lw_blk_mux` hides the request from `hps_io`. CAUSE bit 0
   sets, `f2h_irq[8]` rises and REQ shows the request.
2. The host reads CAUSE, stores it back, then reads REQ and the slot's LBA.
3. The host writes BEGIN with that LBA. A BEGIN whose LBA is not the core's current request is rejected, and the
   request re-opens.
4. The host stores DATA[0 .. n-1], then reads ACCEPT.
5. If ACCEPT's `ok` is clear, the host resends from `next_index`.

**Cards:**
- A card READ is the same flow on BEGIN_S.
- A card WRITE is answered by WACK_S; the host takes the bytes from the core's own DDR image at `wsrc`.
- ABORT, or a core reset mid-frame, zero-fills the rest of the open frame, so the core is never left waiting.

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
| `0x1C` | R | RSTAT `{reply_drops[31:16], replies[15:0]}`: reply words dropped on a full reply FIFO (free-running), and waiting |
| `0x20` | R | RHEAD `[15:0]`: the oldest reply word, 0 when none is waiting. Reading it has no side effect |

Stores must be full words (all four byte enables). Every other offset reads 0.

**Directory entry 4:**

| word | value |
|---|---|
| 4-11 | the UIO (sel 0) opcodes verified for write-only replay, PAD's bitmap shape: `0x01`, `0x1C`, `0x1D`, `0x1E`, `0x22`, `0x24`, `0x31`, `0x32`, `0x33`, `0x3B` |
| 12 | first-word replies the lane stands in for, one byte per opcode: `[7:0]` = 3, the reply of `sys_top`'s `0x20` arm |
| 13 | `{fifo_depth[31:16] = 1024, strobe_gap[15:0] = 4}` |
| 14 | `{reply_depth[31:16] = 256, 0}` (minor 1) |
| 15 | 0: the lane raises no line |

**Frames:**
- A frame of *n* words needs *n* + 2 FIFO entries, so the longest frame is 1,022 words into an empty FIFO. A stock
  gamma table (`0x33`, 768 words) fits.
- A frame is never played before its END is stored. A BEGIN abandons an open frame without counting it; a WORD or
  END with no frame open is ignored.
- A frame that would not leave room for its END is discarded whole at END and counted in DROPS. No store waits.
- **One select per frame.** The link raises both OSD selects for one OSD transaction; on the lane that is two
  frames, sel 2 then sel 3.

**Playing:**
- The player starts a frame only after the link's own selects have been low for two `clk_sys` cycles, so it never
  splits a link transaction.
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
- There is no reply line or cause word in this revision: the host polls PLAYED and RSTAT.

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
- **SECTOR diagnostics (`LW_DIAG=1`):** FRAMES, TSTAMP_DONE and SUM cross together in one held-data handshake word,
  so they change on the same `clk_100m` edge. Frames that complete while a word is in flight are all counted by
  the next one, and SUM then shows the latest data frame's. No ratio bound.
- **LANE:** frames and replies cross in dual-clock FIFOs with Gray pointers that step one entry per update. No
  ratio bound. PLAYED and the reply-drop count cross as toggles, at most one per `strobe_gap + 5` and per
  `strobe_gap + 2` `clk_sys` cycles; each toggle must outlast two `clk_100m` periods, which holds below about
  250 MHz of `clk_sys`.
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

`tb/run.sh` runs the testbench in six arms:
- `LW_DIAG=1`;
- `LW_DIAG=0`;
- `MENU_CORE`, which checks the header rows;
- no SECTOR block, the Template's own configuration;
- the SECTOR block at the default geometry (128-word primary, 256-word save slots): a save-slot frame longer
  than the primary's completes, and the primary is served after it;
- the frame lane: the whole reference run with every decoder behind `lw_link`, then lane frames against the same
  frames sent as link words (status, RTC, timestamp, mount, gamma, FIO, both OSDs), the `vs_wait` and
  `ioctl_wait` holds, a link word raised mid-frame, full-FIFO drops, unended frames, the longest frame, and reply
  capture (`0x29` word for word against the link, ordering across frames, overflow).

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
- **Register map:** identical to a build validated on a DE10-Nano on 2026-10-25, which covered pads, the OSD
  shadow, SECTOR (CD and save slots) and the SYS status line. The testbench's reference configuration reproduces
  that build's directory word for word.
- **The host side** is whatever HPS program speaks the protocol in this document: discovery, the directory, and
  each block's registers, causes and lines. Nothing else about the host is assumed, and a host that never touches
  the window drives the core over SPI as before.
- **This Template build has not been run on hardware.**

## Cost and fit (Quartus 17.0.2 Lite, this Template, 2026-11-14)

| build | ALMs | registers | block memory bits | RAM blocks | rbf |
|---|---|---|---|---|---|
| upstream `e901a9f`, stock | 7,273 | 11,226 | 384,501 | 59 | md5 `0b49f57e…` |
| this branch, `MISTER_LW` off (upstream core files, this `sys/`) | 7,273 | 11,226 | 384,501 | 59 | md5 `0b49f57e…`, bit-identical |
| this branch, `MISTER_LW=1`, no SECTOR | 7,768 (+495) | 12,150 (+924) | 450,357 (+65,856) | 68 (+9) | |

The `MISTER_LW=1` row includes the `Template.sv` example. The stock Template leaves `status_in`, `status_set` and
`joystick_0` unconnected, so Quartus prunes that logic; the example connects them. Of the +924 registers, `hps_io`
accounts for +261 and `emu` for +132, most of it the example's status and pad logic. A core that already uses those
ports pays up to about 400 fewer registers than the table shows.

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

The frame lane has not been fitted in this Template. Without reply capture, on a larger core, the lane measured
165-177 ALMs plus 6-7 for `lw_link`, and 2 M10K. Reply capture adds a 256 x 16 FIFO (one M10K) and its pointers;
that cost is an estimate, not a measurement. With `MISTER_LW_LANE` absent, `sys_top`'s link front is the stock
one and the window builds no lane.
