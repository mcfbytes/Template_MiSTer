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

Status: proof of concept. The directory is layout 2 (magic `LWDR`, version 2.0): 32-word entries with common CAUSE
and LINE words, every offset window-relative. A minor revision may add blocks (new kinds), header words, entry
words or a larger stride, and a host that walks by the rules under "Directory" keeps working. See "Revision and
validation" at the end.

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

   With `MISTER_LW` it expands to `"AXI:40000:80000:LW,",` whatever blocks are built: the directory lists the
   blocks and declares their lines. Without it, it expands to nothing.

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
| `MISTER_LW` | off | builds the bridge, the window, the `hps_io` LW path, the OSD shadow and GPI bits 25, 24 and 22 |
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
- **GPI bit 25 = 1** (the FPGA manager's `gpi` register, read with `gp_out[31] = 1`; reading it never touches the
  bridge, so it is always safe): the firewall below is in this bitstream. The firewall's instance drives the bit,
  so no bitstream without it can show 1. Stock bitstreams leave it 0, and the stock Main never reads bits 21-27.
- **GPI bit 24** is the firewall's idle state, a register: 1 while it holds no transaction on either channel. It
  toggles with every access, so it means something only to a host that knows it has none of its own in flight.
- **GPI bit 22** is the window's lease register (below): 1 while a host holds the lease. Reading it changes nothing
  and can hang nothing, so a program looks here before it takes the link (see "The lease").
- **The directory** at the window's first word: MAGIC `LWDR`, a directory major the host knows (2), and WIN_BASE
  and WIN_SIZE equal to the token's two numbers; then the entries.

In the ID view (`gp_out[31] = 0`) GPI carries the core magic, which has bits 25, 24 and 22 clear, and an unconfigured
FPGA reads all ones, so the bits count only in a sample that also shows bit 31 = 0. Before its first LW access,
and again after every core load and every bridge or HPS reset (with every cached directory value discarded), a
host takes **one** GPI sample in the I/O view that shows bit 31 = 0, bit 25 = 1 and bit 24 = 1. Bit 24 = 0 at that
point, with no access of its own in flight, means the firewall still holds a transaction nobody will finish: stay
on SPI until the core is loaded again. Then the `CONF_STR` token, then MAGIC, VERSION, WIN_BASE and WIN_SIZE. Any
failure: never touch the
bridge. Bit 25 says nothing about the HPS side: the bridge must also be out of reset (the reset manager's bit for
this bridge reads 0), which a host that did not release the bridge itself checks. A periodic task that accesses
the window samples GPI first and requires bit 31 = 0 and bit 25 = 1 before each access (not bit 24, which another
thread's access makes 0); a failed sample means no access now and the whole discovery again before the next.

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

Offsets are from the window start; absolute LW offset = `0x40000` + offset. Each block's range is decoded by a
compare of the address's high bits; an address in no block that is built reads `0xDEADBEEF` and drops stores,
inside the window or outside it, and an unassigned offset inside a block reads 0.

| window offset | absolute LW | size | block |
|---|---|---|---|
| `0x00000` | `0x40000` | `0x1000` | directory: header and entries; the rest of the page reads 0 |
| `0x01000` | `0x41000` | `0x2000` | SECTOR (with `MISTER_LW_SECTOR_SLOT`) |
| `0x03000` | `0x43000` | `0x1000` | SYS |
| `0x04000` | `0x44000` | `0x2000` | OSDBUF |
| `0x06000` | `0x46000` | `0x1000` | LANE (with `MISTER_LW_LANE`) |
| `0x20000` | `0x60000` | `0x20000` | reserved for a core's own blocks; not decoded in this revision |
| `0x40000` | `0x80000` | `0x40000` | PAD |

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
| `0x18` | LEASE | read/write: the lease (below) |
| `0x1C` | TIMEBASE | `0x190186A0`: `[23:0]` the bridge clock in kHz (100,000); `[31:24]` = 25, the log2 of a time unit in bridge clocks (2^25 cycles, about 0.34 s) |
| `0x20` | PENDING | bit *i* = entry *i*'s CAUSE word is non-zero, with or without a lease |
| `0x24` | OWNED | bit *i* = entry *i*'s block has not yet returned from a lease: its core-clock half has not acknowledged rest since the last lease ended (or since configuration), or a lease ended before the half answered its start; only blocks with a lease link (SECTOR, LANE) ever set it |
| `0x28` | SCRATCH | read/write with `LW_DIAG=1` (under the lease, like every store); 0 otherwise |
| `0x2C` | OPEN | bit *i* = entry *i*'s block takes stores now: the lease for a block without a lease link, and for SECTOR and LANE the lease and the block's core-clock half having answered its start; a store to a linked block before its bit is set is dropped and sets `stale` |
| `0x30` | LINE_EN | read/write under the lease: bit *i* = entry *i*'s cause may raise its line; reads 0 whenever no lease is held |
| `0x34` | RESETS | `{freed[15:8], hrst[7:0]}`: the firewall's state machines freed at a reset's release, and the HPS resets the fabric saw; each counts from configuration and stops at 255 |
| `0x38`-`0x3C` | - | 0, reserved |

**Entries.** Entry *i* is at `header_len + i x entry_stride`, `0x40 + 0x80 x i` in this revision: 8 common words,
then the kind's own words K0-K23. The table is compact and in ascending kind order, listing only the blocks this
bitstream builds, so an entry's index (and its PENDING bit) depends on the core's configuration.

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
| 1 | PAD | `0x00010000` | `0x40000` / `0x40000` | 0 | 0 | 0 | K0-K7: the honoured-opcode bitmap |
| 2 | SECTOR | `0x00010001` | `0x01000` / `0x02000` | `0x01000` | 1 | `0x1` | see SECTOR |
| 3 | OSDBUF | `0x00010000` | `0x04000` / `0x02000` | 0 | 0 | 0 | K0: geometry `{height, width}` = `0x00800100` |
| 4 | SYS | `0x00010000` | `0x03000` / `0x01000` | `0x03000` | 2 | `0x1` | none |
| 5 | LANE | `0x00010001` | `0x06000` / `0x01000` | 0 | 0 | 0 | see LANE |

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

A minor revision only adds: reserved words that read 0 today, a larger header or stride, new kinds. A kind number is
never reused; a kind's major changes only if one of its registers changes meaning. Reserved bits read 0 and are
written 0.

### The lease

**No store has effect without the lease, except a store to LEASE itself.** Every other store, to any block or
header word, is answered and dropped, and LEASE's `stale` bit records it. One program holds the lease at a time,
and with it the window and the link.

A store to LEASE is a single full-word store `{0x4C57[31:16], id[15:6], verb[5:4], ttl[3:0]}`; any other store to
it is ignored. `id` is the storing program's own 10-bit lease id; `ttl` 1-15 holds the lease for `ttl` time units
(TIMEBASE: 2^25 bridge clocks, 0.34 s each, so 0.34 s to 5.03 s); `ttl` 0 or `verb` 0 makes the store ignored.

| verb | effect |
|---|---|
| 1 CLAIM | taken only while no lease is held, `quiet` is 1, no HPS reset is seen and GPO bit 31 is 1: `owner` = `id`, `epoch` + 1, the count starts, `stale` and `burst` clear, LINE_EN reads 0, and the causes are reloaded (below) |
| 2 RENEW | restarts the count, only if the lease is held by `id` |
| 3 RELEASE | ends the lease now, only if it is held by `id`; `ttl` is ignored |

For id `0x155`: claim for 1 s `0x4C575553`, renew `0x4C575563`, release `0x4C575570`.

A read returns `{held[31], quiet[30], stale[29], burst[28], left[27:24], epoch[23:16], hrst[15], 0, end[13:12],
idv[11], 0, owner[9:0]}`:

| field | meaning |
|---|---|
| `held` | a lease is held; GPI bit 22 shows the same register |
| `quiet` | every lease link is at rest: a CLAIM may be taken. A block that cannot return (a stopped core clock, or a core that never answers the link) holds it at 0 for every host; OWNED names it |
| `stale` | a store other than to LEASE was dropped since the last CLAIM taken |
| `burst` | a beat of a burst write was dropped since the last CLAIM taken |
| `left` | the count's top four bits: whole time units left |
| `epoch` | CLAIMs taken since configuration, modulo 256 |
| `hrst` | the fabric sees the HPS reset now; no CLAIM is taken while it is 1 |
| `end` | why the last lease ended: 0 none yet, or a RELEASE; 1 the count ran out; 2 an HPS reset; 3 GPO bit 31 was seen 0 |
| `idv` | GPO bit 31 reads 0 now (the ID view, or the core reset through GPO); no CLAIM is taken while it is 1 |
| `owner` | the id of the last CLAIM taken |

The count runs out `ttl` units and one bridge clock after the CLAIM or RENEW that set it; a RENEW in the last cycle
wins. **When the lease ends**, however it ends, every line falls, LINE_EN and OSDBUF CTRL read 0, the SECTOR block's
EN bits clear (a frame in flight completes, zero-filled from the first word the host did not store, and each slot
returns to SPI once its frame is done; in this revision a save-slot frame cut this way is zero-filled too), queued
SECTOR entries are discarded (a BEGIN not yet taken leaves its request standing for SPI), the open LANE frame is
abandoned, queued LANE frames are discarded unplayed (one already playing finishes), and reply words still to come
are discarded. A frame stored in full before the end is delivered whole.

**The lease links.** SECTOR and LANE each carry the lease to their core-clock half over a link with one level each
way: the lease level, and the half's acknowledge that it is at rest (nothing queued, nothing in flight, every
handshake idle). The half's only lease input is that level, and it reaches rest from any state once the level is low.
A link opens a block's stores only after the half has seen the level rise, and the level falls only after the half has
answered it, so a lease that ends within a few cycles of its claim still leaves the link closed until the half
answers. A CLAIM is taken only while every link is at rest (`quiet`). At a CLAIM the causes are reloaded: SYS raises
STATUS; SECTOR's word starts clear and raises a slot's bit when EN is set over a request that stands; LANE's starts
clear. While GPO bit 31 is 0 (the ID view) the lane player starts no frame, so nothing queued by a dead host plays
inside a stock program's identification.

**The lease needs the link's I/O view.** GPO bit 31 = 0 ends it and refuses a CLAIM (`end` 3): the identification
of the core in the ID view does it, and so does the core reset through GPO, whoever performs them. So a dead
program's lease ends when the next program starts, and a host that holds the lease identifies the core before it
claims, never leaves the I/O view while it holds it, releases before a core reset through GPO and claims again
after it.

**A host's procedures:**
- **Start.** (1) Before its first GPO store and its first link word, read the FPGA manager's GPO register itself
  (not a software copy: the usual first act, storing GPO = 0, ends a live lease). If bit 31 reads 1, take one GPI
  sample: bit 31 = 0, bit 25 = 1 and bit 22 = 1 mean another program holds the lease. Then store nothing to GPO and
  send no link word; stop, or sample until bit 22 reads 0 (at most the longest TTL, 5.1 s), or, if this program is
  configured as the machine's one LW program, go on and take over by identifying. (2) Identify the core and run
  discovery. (3) Read LEASE: `held` means another program claimed meanwhile: stop. (4) Store CLAIM with the host's
  own id, read LEASE, and require `held` and `owner` = its id; remember `epoch`. A CLAIM stored while `quiet` reads
  0 is ignored: read LEASE until `quiet` reads 1, and stay on SPI meanwhile. (5) Read OPEN until the bits of the
  entries it serves are set; store nothing to an entry before its bit is set. An entry whose bit does not set stays
  on SPI while the lease is kept for the others, and may be bound later if its bit sets. (6) Store LINE_EN with those entries' bits, and re-store every pad level.
- **Refresh** every quarter of the TTL, from a thread that may be late (never the real-time one): sample GPI (bit
  31 = 0, bit 25 = 1), store RENEW, read LEASE, and require `held`, `owner` = its id and the remembered `epoch`.
  Otherwise the lease has ended and `end` says why: every thread stops storing at once and the host starts over. It
  never releases a lease it no longer holds.
- **Exit:** store RELEASE with its id. Before a core load or a bridge reset, release at once.
- A second program that takes the lease by RELEASE with the `owner` it read, then CLAIM, takes it on purpose. The id
  is an accident guard, not a secret; one program owns the window and core loading, or the two coordinate.

### Interrupt lines

A line is declared in one place: the LINE word of the entry it serves. Ordinal *k* is `f2h_irq[7 + k]`, the GIC's
shared interrupt 47 + *k*, and no two entries name the same ordinal.

| ordinal | f2h line | GIC SPI | entry in this revision |
|---|---|---|---|
| 1 | `f2h_irq[8]` | 48 | SECTOR |
| 2 | `f2h_irq[9]` | 49 | SYS |
| 3-15 | `f2h_irq[10]`-`[22]` | 50-62 | framework kinds to come |
| 16-31 | `f2h_irq[23]`-`[38]` | 63-78 | a core's own kinds |

A line is a register, high while the lease is held, its entry's LINE_EN bit is set and its CAUSE word is non-zero;
it falls only when one of the three stops being true, and never pulses per event. Configure it level-sensitive,
active high, at the interrupt controller (`IRQ_TYPE_LEVEL_HIGH` in a Linux device tree): a raise in the cycle of a
store-back keeps the line high, so an edge-triggered input would see no new edge after it. The HPS kernel must expose it to userspace (for
example as a `uio_pdrv_genirq` node); a host without one polls PENDING instead. A CAUSE word's `[15:0]` are event
bits, each raised by every occurrence of its event, and `[31:16]` read 0; the word is 0 when nothing is pending.
**Reads change nothing**, in any size or burst. **A full 32-bit store clears each bit written as 1**; a bit raised
in the store's own cycle stays set, and a partial-width store clears nothing.

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
| `0x1000` | CTRL | `[0]` LWSEL: video reads the shadow instead of `osd_buffer`. `[1]` HR: with LWSEL, the 16-row (highres) size. Reads 0 whenever no lease is held |
| `0x1004` | ID | `0x3144534F` ("OSD1") |

- **Still on SPI:** enable/disable, the info box, rotation and `osd_status`.
- **Highres:** the SPI rule OR (LWSEL & HR).
- **Header rows** (the `MENU_CORE` title rows) never read the shadow.
- **Cost:** four 1024x8 M10K per `osd` instance.

### SYS block (kind 4): the status back-channel

`hps_io`'s `0x29` back-channel without the poll: when the core raises `status_set`, the block captures
`status_req` and raises STATUS on its entry's line (ordinal 2, `f2h_irq[9]`). The SPI `0x29` arm still answers as
before.

| offset | register | |
|---|---|---|
| `0x00` | CAUSE | `0x00000001` while pending, else 0: bit 0 = STATUS, raised by every capture; a store of bit 0 clears it |
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

**Kind words** (the entry's CAUSE word gives the CAUSE register, LINE ordinal 1):

| word | value |
|---|---|
| K0 | `{slot[31:24], 0, frame_words[9:0]}` |
| K1 | 0, reserved |
| K2 | `{fifo_depth[31:16] = 512, strobe_gap[15:0] = 2}` |
| K3, K5 | card 0 / card 1: `{0, slot[27:24], flags[23:16], frame_words[9:0]}`, flags bit 0 = RD, bit 1 = WR |
| K4, K6 | card 0 / card 1: DDR image base (physical) |
| K7, K8 | card 0 / card 1: image size in bytes |

An undeclared card's words are 0.

**Registers, from the block's start:**

| offset | register | |
|---|---|---|
| `0x00` | CAUSE | `0x00000001` while pending, else 0: bit 0 = a request opened or re-opened; a store of bit 0 clears it |
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
   sets, the entry's line (ordinal 1, `f2h_irq[8]`) rises and REQ shows the request.
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

**Kind words** (the entry's CAUSE and LINE words are 0: the lane has no cause word and raises no line):

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
- **Lease links (SECTOR, LANE):** one level each way, each through a two-flop synchroniser; the acknowledge needs
  four `clk_sys` cycles of rest and the bridge side ten consecutive samples of it. No ratio bound; a stopped
  `clk_sys` holds the link (and `quiet`) where it is.
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
- the lease: every store dropped without it, each verb's refusals, the expiry at 1, 2 and 15 units to the cycle,
  a RENEW swept across the last live cycle, each way a lease ends with its code, two programs with different ids,
  the look at GPO and GPI bit 22 before identifying, a takeover, a stock start beside a live host, the SECTOR and
  LANE blocks in every state when a lease runs out (with `clk_sys` running and stopped), and RESETS against a model
  of the firewall;
- the lease links, with a checker on each in every arm (the level rises only on an acknowledge of rest and falls
  only after the half has answered; no store reaches a linked block outside OPEN) and every CLAIM taken checked
  against the testbench's own rest predicate: CLAIMs stored as fast as the bus allows after a RELEASE, a CLAIM
  followed at once by work, the lease ended by RELEASE, GPO bit 31 and an HPS reset in every cycle of the rise,
  every core-clock register scrambled mid-lease and during the return, a half that never answers and one that
  cannot return, a dead host's select with frames queued behind it, the causes at a claim, and a slot-0 frame
  stored in full with the lease ended at each offset to its last strobe;
- the lines as levels, compared every cycle with `lease & LINE_EN & cause`: a raise in the cycle of a store-back
  keeps the line high, and a host woken only by the level serves every event across 10,000 random interleavings;
- the directory word by word, a host's walk of it (and of altered copies: other majors, entries to skip, a shared
  ordinal, a larger header and stride), and each entry's cause on its own PENDING bit and line;
- the window's decode: every 4 KB page of the 2 MB reads its block or `0xDEADBEEF`, and a store outside every block
  changes nothing;
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
- **Register map:** the blocks' registers are those of a build validated on a DE10-Nano on 2026-10-25, which
  covered pads, the OSD shadow, SECTOR (CD and save slots) and the SYS status line, except where this document
  states otherwise: CAUSE words clear by write-one and carry no class tag, and a store is a single-beat write. The
  directory is layout 2, which that build did not have: a host written for its layout (magic `SYS1`) finds no
  directory here and stays on SPI.
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
