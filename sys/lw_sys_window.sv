//============================================================================
//  lw_sys_window.sv -- part of the LW bridge framework (docs/LW_BRIDGE.md)
//
//  This program is free software; you can redistribute it and/or modify it
//  under the terms of the GNU General Public License as published by the Free
//  Software Foundation; either version 3 of the License, or (at your option)
//  any later version.
//
//  This program is distributed in the hope that it will be useful, but WITHOUT
//  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
//  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
//  more details.
//
//  You should have received a copy of the GNU General Public License along
//  with this program.  If not, see <https://www.gnu.org/licenses/>.
//============================================================================

`ifdef MISTER_LW
`default_nettype none

// LW window declared by `SYS_LW_CAPS (emu_ports.vh): the directory (the L0 floor, MISTER_LW), then the blocks of
// the tiers the core builds: PAD (L1, MISTER_LW_PAD, lw_pad.sv), EVT (L2, MISTER_LW_EVT, lw_evt.sv), SECTOR
// (MISTER_LW_SECTOR_SLOT, lw_sector.sv), OSDBUF (MISTER_LW_OSD, lw_osd.sv) and LANE (MISTER_LW_LANE, lw_lane.sv).
// Each tier is a parameter here (sys_top.v sets it from its macro), and a tier set without its macro stops the
// build.
// Bridge side runs on clk_100m with no reset.
// HPS_BUS[383:46] is one port, hps_lw, whatever the tiers: this module drives every to-core bit (0 for a tier
// not built), and a from-core bit is driven only by the core-side piece that owns it. Each core-side piece
// drives a constant 1 on its own presence bit, and a block is listed, decoded and wired only when its tier is
// built and its presence bit reads 1: hps_io's LW write arm for PAD, hps_io's event exports for EVT, and for
// SECTOR hps_io's commit mark and at least one slot's mux (a slot whose mux is absent is not wired: its
// descriptor's flags read 0 and its EN bit cannot be set). A truncated bus therefore shows as a block that is not
// in the directory.
// Directory, layout 2: a 16-word header (MAGIC "LWDR", VERSION 2.0, LAYOUT = {entry stride, header length} in
// bytes, COUNT, WIN_BASE and WIN_SIZE, ENABLE, RSVD, PENDING, OWNED, SCRATCH, OPEN, LINE_EN, RESETS), then one
// 32-word entry per block listed, in ascending kind order with no gaps. An entry's 8 common words are KIND,
// VERSION, OFFSET, SIZE, CAUSE, LINE, CAUSE_BITS and TAG; its kind's own words follow. Every offset in the
// directory is window-relative. A line is declared in one place, the entry's LINE word: the same localparam
// picks the entry's bit of `lines` (ordinal k = lines[k-1] = f2h_irq[7+k]), and no two entries name one ordinal.
// The host enable: en_q, one clk_100m register, is the write enable of the whole window. Every store but one to
// ENABLE sees lw_write & en_q, here, above every block; a store dropped at that gate sets ENABLE.stale. The ENABLE
// store (one word, the key and bit 0) is taken only with en_q clear, no HPS reset seen (lw_hrst), GPO[31] high
// (lw_idv low) and every enable link at rest (quiet). en_q has no timer and no host clears it: it clears on lw_hrst
// and on lw_idv, the ID view or the core reset through GPO, which every program that starts on this link performs
// first, so each new host meets every block off and the core on stock SPI. en_q is GPI[22] and crosses no clock
// itself. SECTOR, OSDBUF and LANE, whose queues a clk_sys half consumes, each have an enable link (lw_fence, lw_cdc.sv)
// that carries en_q to that half as one level and its acknowledge back: a store to such a block also needs the
// link open (its OPEN bit), which it is only once the half has seen the enable, and a store dropped there sets
// stale too. quiet keeps a half still returning to SPI from taking work under the next enable. OWNED shows each
// link not yet back at rest. Every other block's OPEN bit is en_q, its OWNED bit 0.
// At a taken ENABLE each block reloads its CAUSE word: EVT ORs in STATUS and its standing levels, SECTOR and LANE
// clear their own.
// Lines: each is a register, en_q & the entry's LINE_EN bit & the entry's CAUSE word non-zero, so a line is a
// level that falls only when one of the three does. LINE_EN reads 0 whenever en_q does. PENDING is not
// gated. RESETS counts rises of lw_hrst and the state machines the firewall freed, each saturating at 255.
// Decode: each block's select compares the address's high bits with a constant, so the selects are exclusive;
// every block's word is 0 unless it is selected, and the read word is one AND-OR of the block words and the
// 0xDEADBEEF default. An address in no block that is listed reads 0xDEADBEEF and drops stores; an unassigned
// offset inside a block reads 0.
// The SECTOR block is one primary hps_io read slot plus up to two save slots; its kind words declare each slot by
// its slot index, with its hps_io slot, frame, LBA width and (for a save slot) its DDR image base and extent.
// OSDBUF queues image stores for a clk_sys writer that writes them through each osd instance's own osd_buffer port
// (lw_osd.sv, osd.v). EVT raises hps_io's core->host flags (status, block request, info, upload) as causes on its
// own line. LANE replays ordered link frames onto sys_top's link nets and can capture their replies; its CAUSE
// (played, reply, drop) is on its own line.
// LW_DIAG=0 (shipping) compiles out SCRATCH (reads 0, writes ignored) and lw_sector's diagnostics; LW_DIAG=1
// (debug builds) keeps them.
// See docs/LW_BRIDGE.md for the register map and the adoption recipe.

module lw_sys_window #(
	// The tiers (sys_top.v sets each from its macro): 1 = build the block and list it while its piece is present.
	parameter        PAD_EN    = 0,          // L1, MISTER_LW_PAD
	parameter        EVT_EN    = 0,          // L2, MISTER_LW_EVT
	parameter        OSD_EN    = 0,          // MISTER_LW_OSD
	// SECTOR block, as the core declares it (sys_top.v maps the MISTER_LW_SECTOR_* / MISTER_LW_SAVE* macros).
	// The geometry is checked here (an unservable one stops the build, naming lw_sector_unservable_geometry).
	parameter        SEC_EN    = 0,          // 1 = build the SECTOR block
	parameter [31:0] SEC_SLOT  = 32'd1,      // the hps_io slot the primary frames feed (sd_rd[SEC_SLOT]), 0-15
	parameter [31:0] SEC_FRAME = 32'd128,    // 32-bit DATA words per primary frame, 1-4096 (128 = one 512-byte block)
	parameter [31:0] SEC_GAP   = 32'd2,      // clk_sys cycles between write strobes, and from the ack to the first, 2-15
	parameter  [2:0] BLKSZ     = 3'd2,       // the core's hps_io BLKSZ, reported in the GET_SDSTAT word
	parameter        SAVE0_EN    = 0,        // 1 = save slot 0 (slot index 1) is declared: its hps_io slot (0-15),
	parameter [31:0] SAVE0_SLOT  = 32'hE,    //   DDR image base (required) and extent, frame (32-bit words, 1 to
	parameter [31:0] SAVE0_WSRC  = 32'd0,    //   the FIFO's entries - 2) and LBA bits kept, crossed and compared
	parameter [31:0] SAVE0_EXT   = 32'd0,    //   (1-32)
	parameter [31:0] SAVE0_FRAME = 32'd256,
	parameter [31:0] SAVE0_LBA_BITS = 32'd32,
	parameter        SAVE1_EN    = 0,        // save slot 1 (slot index 2), likewise
	parameter [31:0] SAVE1_SLOT  = 32'hF,
	parameter [31:0] SAVE1_WSRC  = 32'd0,
	parameter [31:0] SAVE1_EXT   = 32'd0,
	parameter [31:0] SAVE1_FRAME = 32'd256,
	parameter [31:0] SAVE1_LBA_BITS = 32'd32,
	// LANE block (sys_top.v sets it with MISTER_LW_LANE): 1 = build the frame lane and list its entry.
	parameter        LANE_EN   = 0,
	// The OCRAM SECTOR feeder (sys_top.v sets it with MISTER_OCRAM_SECTOR_SLOT): 1 = the primary slot's to-core bits
	// of HPS_BUS take the xsec_* inputs; no SECTOR entry is listed. With SEC_EN the build stops.
	parameter        SEC_EXT   = 0,
	// The OCRAM mount slots (sys_top.v sets it with MISTER_OCRAM_MOUNT0_SLOT, _MOUNT1_SLOT; beside SEC_EXT): bit
	// i = 1, save slot i's to-core bits of HPS_BUS take the xmnt_* inputs. Its save-slot entry is not listed.
	parameter  [1:0] MNT_EXT   = 2'd0
)
(
	input wire         clk,          // clk_100m

	input wire  [20:0] lw_raddr,
	output wire [31:0] lw_rdata,

	input wire  [20:0] lw_waddr,
	input wire         lw_write,
	input wire  [31:0] lw_wdata,
	input wire   [3:0] lw_be,
	input wire         lw_wburst,    // a dropped beat of a burst write (the firewall's pulse)

	input wire         lw_hrst,      // the HPS reset as a clk level (the firewall's lw_arst)
	input wire   [1:0] lw_freed,     // the firewall's machines freed at a reset's release, pulses
	input wire         lw_idv,       // GPO[31] reads 0, as a clk level (sys_top's lw_arst of gp_out[31])
	output wire        held,         // the host enable en_q: GPI[22]

	output wire  [2:0] lines,        // line ordinal k = lines[k-1] -> f2h_irq[7+k], level-high

	input wire         clk_sys,
	inout wire [383:46] hps_lw,      // HPS_BUS[383:46] on clk_sys, the bit map of docs/LW_BRIDGE.md
	input wire         io_wide,      // HPS_BUS[32], hps_io's io_wide: a constant, 1 = 16-bit strobes

	input wire         lane_busy,    // clk_sys: any of the link's own selects, or its ID view (GPO[31] low)
	input wire         lane_wait,    // clk_sys: io_wait | vs_wait
	input wire  [15:0] lane_dout,    // clk_sys: the link's reply word, io_dout | io_dout_sys
	output wire        lane_own,     // clk_sys: the lane player, to lw_link
	output wire        lane_hold,
	output wire  [3:0] lane_sel,
	output wire        lane_strobe,
	output wire [15:0] lane_din,

	input wire   [1:0] osd_sel,      // clk_sys: both osd selects as each osd sees them, {vga, hdmi}
	input wire         osd_view,     // clk_sys: GPO[31], 1 = the link's I/O view
	output wire        osd_we,       // clk_sys: one byte of every osd's osd_buffer (lw_osd.sv)
	output wire [12:0] osd_addr,
	output wire  [7:0] osd_byte,

	// The OCRAM SECTOR feeder (hps_ocram_rx.sv), on clk_sys: its nets to and from the primary slot's bits
	input wire         xsec_en,      // en_sys
	input wire         xsec_ack,
	input wire         xsec_wr,
	input wire  [15:0] xsec_dat,
	input wire  [13:0] xsec_adr,
	output wire        xsec_reset,   // HPS_BUS[132]
	output wire        xsec_hps_ack, // [133]
	output wire        xsec_req,     // [134], with the mux's and the mark's presence
	output wire [31:0] xsec_lba,     // [166:135]
	output wire        xsec_commit,  // [380]
	// The OCRAM mount slots' bits, {save slot 1, save slot 0} (MNT_EXT)
	input wire   [1:0] xmnt_en,      // en_sys
	input wire   [1:0] xmnt_ack,
	output wire  [1:0] xmnt_hps_ack, // [202], [167]
	output wire  [1:0] xmnt_rd,      // [204], [169], with the mux's and the mark's presence
	output wire  [1:0] xmnt_wr,      // [203], [168], likewise
	output wire [63:0] xmnt_lba      // [236:205], [201:170]
);

`ifdef LW_DIAG
localparam        DIAG     = `LW_DIAG;
`else
localparam        DIAG     = 0;
`endif

// The window (absolute LW offset and size: the `SYS_LW_CAPS token's two numbers) and its blocks, window-relative.
// Every block is aligned to its size.
localparam [20:0] WIN_OFF   = 21'h40000;
localparam [20:0] WIN_SIZE  = 21'h80000;
localparam [20:0] DIR_OFF   = 21'h00000;   // the directory: header and entries, the rest of its page reads 0
localparam [20:0] SEC_OFF   = 21'h08000;
localparam [20:0] SEC_SIZE  = 21'h08000;
localparam [20:0] EVT_OFF   = 21'h03000;
localparam [20:0] EVT_SIZE  = 21'h01000;
localparam [20:0] OSD_OFF   = 21'h04000;
localparam [20:0] OSD_SIZE  = 21'h02000;
localparam [20:0] LANE_OFF  = 21'h06000;
localparam [20:0] LANE_SIZE = 21'h01000;
localparam [20:0] PAD_OFF   = 21'h40000;
localparam [20:0] PAD_SIZE  = 21'h40000;

localparam [20:0] DIR_A  = WIN_OFF + DIR_OFF;      // absolute block starts, for the selects
localparam [20:0] SEC_A  = WIN_OFF + SEC_OFF;
localparam [20:0] EVT_A  = WIN_OFF + EVT_OFF;
localparam [20:0] OSD_A  = WIN_OFF + OSD_OFF;
localparam [20:0] LANE_A = WIN_OFF + LANE_OFF;
localparam [20:0] PAD_A  = WIN_OFF + PAD_OFF;

// The directory's header (bytes).
localparam [15:0] HDR_LEN  = 16'h0040;
localparam [15:0] STRIDE   = 16'h0080;

// Kinds, versions and lines. Ordinal 0 = no line; framework kinds use 1-15, each ordinal at most once.
localparam [15:0] PAD_KIND  = 16'd1;
localparam [31:0] PAD_VER   = 32'h00010001;
localparam  [7:0] PAD_LINE  = 8'd0;
localparam [15:0] SEC_KIND  = 16'd2;
localparam [31:0] SEC_VER   = 32'h00020000;
localparam  [7:0] SEC_LINE  = 8'd1;                        // f2h_irq[8], GIC SPI 48
localparam [15:0] OSD_KIND  = 16'd3;
localparam [31:0] OSD_VER   = 32'h00020000;
localparam  [7:0] OSD_LINE  = 8'd0;
localparam [15:0] EVT_KIND  = 16'd4;
localparam [31:0] EVT_VER   = 32'h00020000;
localparam  [7:0] EVT_LINE  = 8'd2;                        // f2h_irq[9], GIC SPI 49
localparam [15:0] EVT_BITS  = 16'h000F;                    // STATUS, BLK, INFO, UPLOAD
localparam [15:0] LANE_KIND = 16'd5;
localparam [31:0] LANE_VER  = 32'h00010002;
localparam  [7:0] LANE_LINE = 8'd3;                        // f2h_irq[10], GIC SPI 50
localparam [20:0] LANE_CAUSE = LANE_OFF + 21'h00024;
localparam [15:0] LANE_BITS = 16'h0007;                    // PLAYED, REPLY, DROP

localparam        SEC_AW    = 9;          // FIFO depth 512
localparam  [7:0] SEC_RD    = 8'h01;      // a descriptor's flags: RD, BEGIN_S frames; WR, WACK_S with the bytes
localparam  [7:0] SEC_RDWR  = 8'h03;      //   from the DDR image
localparam        SAVE_DECL0 = SEC_EN && SAVE0_EN;
localparam        SAVE_DECL1 = SEC_EN && SAVE1_EN;
// An unservable SECTOR geometry: a primary frame outside 1-4096 words, a save slot's outside 1 to the FIFO's entries
// - 2, an LBA width outside 1-32, a gap outside 2-15, an hps_io slot above 15, two slots on one hps_io slot, or a
// save slot without an image base.
localparam        SAVE_BAD0 = SAVE0_FRAME == 0 || SAVE0_FRAME > (1 << SEC_AW) - 2 || SAVE0_LBA_BITS == 0 ||
                              SAVE0_LBA_BITS > 32 || SAVE0_SLOT > 15 || SAVE0_SLOT == SEC_SLOT || SAVE0_WSRC == 0;
localparam        SAVE_BAD1 = SAVE1_FRAME == 0 || SAVE1_FRAME > (1 << SEC_AW) - 2 || SAVE1_LBA_BITS == 0 ||
                              SAVE1_LBA_BITS > 32 || SAVE1_SLOT > 15 || SAVE1_SLOT == SEC_SLOT || SAVE1_WSRC == 0;
localparam        SEC_BAD   = SEC_EN && (SEC_FRAME == 0 || SEC_FRAME > 4096 || SEC_GAP < 2 || SEC_GAP > 15 ||
                              SEC_SLOT > 15 || (SAVE_DECL0 && SAVE_BAD0) || (SAVE_DECL1 && SAVE_BAD1) ||
                              (SAVE_DECL0 && SAVE_DECL1 && SAVE0_SLOT == SAVE1_SLOT));
`ifdef MENU_CORE
localparam [15:0] OSD_ROWS  = 16'd20;     // the image's rows of 256 bytes, header rows 16-19 included
`else
localparam [15:0] OSD_ROWS  = 16'd16;
`endif
localparam        OSD_AW    = 8;          // FIFO depth 256
localparam        LANE_AW   = 10;         // FIFO depth 1024: a 256-entry 0x33 gamma table is 770 entries
localparam  [7:0] LANE_GAP  = 8'd4;       // clk_sys cycles without a wait between strobes, 2..7
localparam        LANE_RAW  = 8;          // reply FIFO depth 256

// LANE kind words K0-K7: the UIO (sel 0) opcodes verified for write-only replay; K8, the first-word reply the lane
// stands in for, one byte per op: [7:0] = 0x20's 'b11 (sys_top answers it); a capture frame also records it.
localparam [255:0] LANE_OPS = (256'd1 << 8'h01) | (256'd1 << 8'h1C) | (256'd1 << 8'h1D) | (256'd1 << 8'h1E) |
                              (256'd1 << 8'h22) | (256'd1 << 8'h24) | (256'd1 << 8'h31) | (256'd1 << 8'h32) |
                              (256'd1 << 8'h33) | (256'd1 << 8'h3B);
localparam [31:0] LANE_CAPS = 32'h0000_0003;

// PAD kind words K0-K7: the honoured-opcode bitmap; K8 {spinner FIFO depth, keyed slots}; K9 bit 0: the link's
// pad arm is not built (MISTER_DISABLE_SPI_PAD in hps_io.sv), so the pads have no path but this block.
`ifdef MISTER_DISABLE_SPI_PAD
localparam [31:0] PAD_K9    = 32'd1;
`else
localparam [31:0] PAD_K9    = 32'd0;
`endif
localparam [255:0] PAD_OPS = (256'd1 << 8'h02) | (256'd1 << 8'h03) |
                             (256'd1 << 8'h10) | (256'd1 << 8'h11) |
                             (256'd1 << 8'h12) | (256'd1 << 8'h13) |
                             (256'd1 << 8'h1A) | (256'd1 << 8'h3D);

// The header's read/write words, as word indices of the directory page.
localparam  [9:0] H_ENA   = 10'h006, H_LINEN = 10'h00C;
localparam [31:0] ENA_W   = 32'h4C570001;               // the ENABLE store: the key and bit 0

// The presence bits: a constant 1 from each core-side piece, 0 where it is absent (a net with no source).
wire        pres_pad = hps_lw[375];                     // hps_io's LW write arm
wire        pres_evt = hps_lw[376];                     // hps_io's event exports
wire  [2:0] pres_sec = hps_lw[379:377];                 // {save slot 1's mux, save slot 0's, the primary's}
wire        pres_cmt = hps_lw[381];                     // hps_io's commit mark
wire        wide_c   = io_wide;                         // hps_io's io_wide, a constant (K1, and the feeder's strobes)

// What is listed: a tier built with its piece present. A SECTOR slot is wired when it is declared and its mux and
// hps_io's commit mark are present; the block is listed with one wired slot.
wire        has_pad = PAD_EN != 0 && pres_pad;
wire        has_evt = EVT_EN != 0 && pres_evt;
wire  [2:0] wired   = {SAVE_DECL1 && pres_sec[2], SAVE_DECL0 && pres_sec[1], SEC_EN != 0 && pres_sec[0]} &
                      {3{pres_cmt}};
wire        has_sec = |wired;

// The entries, in ascending kind order: each listed block's index, and the count.
wire  [4:0] e_pad   = 5'd0;
wire  [4:0] e_sec   = {4'd0, has_pad};
wire  [4:0] e_osd   = e_sec + {4'd0, has_sec};
wire  [4:0] e_evt   = e_osd + (OSD_EN ? 5'd1 : 5'd0);
wire  [4:0] e_lane  = e_evt + {4'd0, has_evt};
wire  [4:0] count   = e_lane + (LANE_EN ? 5'd1 : 5'd0);
wire [31:0] present = (32'd1 << count) - 32'd1;         // bit i = entry i is listed

// The host enable and the lines.
reg            en_q    = 0;           // the host enable
reg            stale   = 0;           // a store other than ENABLE was dropped at the gate, since the last ENABLE
reg            lburst  = 0;           // a burst-write beat was dropped, since the last ENABLE
reg     [31:0] line_en = 0;           // LINE_EN
reg      [2:0] line_q  = 0;           // the lines
reg      [7:0] n_hrst  = 0, n_freed = 0;
reg            hrst_d  = 1'b1;        // lw_hrst is high from configuration: configuration is no rise
wire           quiet;                 // every enable link lets an ENABLE through

reg [31:0] scratch = 0;

// Selects: high address bits against a constant, read and write side alike, for listed blocks only.
wire        r_dir  = lw_raddr[20:12] == DIR_A[20:12];
wire        r_sec  = has_sec && lw_raddr[20:15] == SEC_A[20:15];
wire        r_evt  = has_evt && lw_raddr[20:12] == EVT_A[20:12];
wire        r_osd  = OSD_EN && lw_raddr[20:13] == OSD_A[20:13];
wire        r_lane = LANE_EN && lw_raddr[20:12] == LANE_A[20:12];
wire        r_pad  = has_pad && lw_raddr[20:18] == PAD_A[20:18];
wire        r_none = ~(r_dir | r_sec | r_evt | r_osd | r_lane | r_pad);

wire        w_dir  = lw_waddr[20:12] == DIR_A[20:12];
wire        w_sec  = has_sec && lw_waddr[20:15] == SEC_A[20:15];
wire        w_evt  = has_evt && lw_waddr[20:12] == EVT_A[20:12];
wire        w_osd  = OSD_EN && lw_waddr[20:13] == OSD_A[20:13];
wire        w_lane = LANE_EN && lw_waddr[20:12] == LANE_A[20:12];
wire        w_pad  = has_pad && lw_waddr[20:18] == PAD_A[20:18];

// The gate: ENABLE takes the raw store, every other store needs en_q, and a store to a block with an enable link
// needs that link open as well.
wire        sec_open, sec_owned, sec_idle, osd_open, osd_owned, osd_idle, lane_open, lane_owned, lane_idle;
wire        w_ena_a = w_dir && lw_waddr[11:2] == H_ENA;
wire        wr      = lw_write & en_q;
wire        wr_sec  = wr & sec_open;
wire        wr_osd  = wr & osd_open;
wire        wr_lane = wr & lane_open;
assign      quiet   = sec_idle & osd_idle & lane_idle;
wire        e_take  = lw_write && w_ena_a && &lw_be && lw_wdata == ENA_W && !en_q && !lw_hrst && !lw_idv && quiet;

// Block-relative addresses: the address bits below each block's size.
wire [14:0] rsec  = lw_raddr[14:0];
wire [14:0] wsec  = lw_waddr[14:0];
wire [11:0] revt  = lw_raddr[11:0];
wire [11:0] wevt  = lw_waddr[11:0];
wire [12:0] rosd  = lw_raddr[12:0];
wire [12:0] wosd  = lw_waddr[12:0];
wire [11:0] rlane = lw_raddr[11:0];
wire [11:0] wlane = lw_waddr[11:0];
wire [17:0] rpad  = lw_raddr[17:0];
wire [17:2] wpad  = lw_waddr[17:2];

wire        sec_bell, evt_irq, lane_irq;   // each block's CAUSE word is non-zero
wire [31:0] sec_rdata, evt_rdata, osd_rdata, lane_rdata, pad_rdata;

// A block's cause level, for the lines and PENDING: only a listed block's.
wire        sec_lvl = has_sec & sec_bell;
wire        evt_lvl = has_evt & evt_irq;
wire        lane_lvl = LANE_EN != 0 && lane_irq;

// The lines: each entry's cause level and LINE_EN bit on the bit its LINE word names, registered under en_q.
function automatic [2:0] line_bit(input [7:0] ord);
	line_bit = (ord >= 8'd1 && ord <= 8'd3) ? (3'd1 << (ord - 8'd1)) : 3'd0;
endfunction

wire  [2:0] line_d = ({3{sec_lvl & line_en[e_sec]}} & line_bit(SEC_LINE)) |
                     ({3{evt_lvl & line_en[e_evt]}} & line_bit(EVT_LINE)) |
                     ({3{lane_lvl & line_en[e_lane]}} & line_bit(LANE_LINE));

always @(posedge clk) line_q <= {3{en_q}} & line_d;

assign lines = line_q;
assign held  = en_q;

function automatic [7:0] sat_add(input [7:0] v, input [1:0] n);
	sat_add = ({1'b0, v} + {7'd0, n} > 9'd255) ? 8'd255 : v + {6'd0, n};
endfunction

always @(posedge clk) begin
	hrst_d  <= lw_hrst;
	n_hrst  <= sat_add(n_hrst, {1'b0, lw_hrst & ~hrst_d});
	n_freed <= sat_add(n_freed, {1'b0, lw_freed[0]} + {1'b0, lw_freed[1]});

	if (lw_hrst || lw_idv) en_q <= 0;
	else if (e_take)       en_q <= 1;

	if (e_take) begin
		stale  <= 0;
		lburst <= 0;
	end
	else begin
		if (lw_write && !w_ena_a && (!en_q || (w_sec && !sec_open) || (w_osd && !osd_open) || (w_lane && !lane_open)))
			stale <= 1;
		if (lw_wburst) lburst <= 1;
	end

	if (!en_q) line_en <= 0;
	else if (wr && w_dir && lw_waddr[11:2] == H_LINEN && &lw_be) line_en <= lw_wdata & present;
end

wire [31:0] ena_rd = {en_q, quiet, stale, lburst, 12'd0, lw_hrst, 3'd0, lw_idv, 11'd0};

// PENDING: bit i = entry i's CAUSE word is non-zero.
wire [31:0] pending = ({31'd0, sec_lvl} << e_sec) | ({31'd0, evt_lvl} << e_evt) | ({31'd0, lane_lvl} << e_lane);

// OPEN and OWNED: bit i is entry i's. A linked block's bits are its link's; every other block's OPEN is en_q.
wire [31:0] linked  = ({31'd0, has_sec} << e_sec) | ((OSD_EN ? 32'd1 : 32'd0) << e_osd) |
                      ((LANE_EN ? 32'd1 : 32'd0) << e_lane);
wire [31:0] open_w  = ({32{en_q}} & present & ~linked) | ({31'd0, has_sec & sec_open} << e_sec) |
                      ({31'd0, osd_open} << e_osd) | ({31'd0, lane_open} << e_lane);
wire [31:0] owned_w = ({31'd0, has_sec & sec_owned} << e_sec) | ({31'd0, osd_owned} << e_osd) |
                      ({31'd0, lane_owned} << e_lane);

// One entry word: the 8 common words, then the kind's word kw (K0-K23).
function automatic [31:0] ent(input [4:0] w, input [15:0] kind, input [31:0] ver, input [20:0] off,
                              input [20:0] size, input [20:0] cause, input [7:0] line, input [15:0] bits,
                              input [31:0] kw);
	case (w)
		5'd0:    ent = {16'd0, kind};
		5'd1:    ent = ver;
		5'd2:    ent = {11'd0, off};
		5'd3:    ent = {11'd0, size};
		5'd4:    ent = {11'd0, cause};
		5'd5:    ent = {24'd0, line};
		5'd6:    ent = {16'd0, bits};
		5'd7:    ent = 32'd0;                                // TAG: 0 for a framework kind
		default: ent = kw;
	endcase
endfunction

// The kind words, K0 first.
function automatic [31:0] pad_kw(input [4:0] k);
	pad_kw = (k < 5'd8) ? PAD_OPS[{k[2:0], 5'd0} +: 32] : (k == 5'd8) ? {16'd16, 16'd24} : (k == 5'd9) ? PAD_K9 : 32'd0;
endfunction

// SECTOR: K0 {FIFO entries, 0, strobe gap}, K1 {BLKSZ[10:8], wide[0]} (hps_io's io_wide), K2 the descriptors that
// follow, then four words per slot index k from K4 + 4k: {flags, hps_io slot, frame words}, LBA bits, DDR image
// base and size. A slot's flags read 0 unless it is wired (wd, by k); an undeclared slot's words read 0.
function automatic [31:0] sec_desc(input [7:0] flags, input [3:0] slot, input [12:0] frame);
	sec_desc = {flags, 4'd0, slot, 3'd0, frame};
endfunction
function automatic [31:0] sec_kw(input [4:0] k, input [2:0] wd);
	case (k)
		5'd0:    sec_kw = {16'd1 << SEC_AW, 8'd0, SEC_GAP[7:0]};
		5'd1:    sec_kw = {21'd0, BLKSZ, 7'd0, wide_c};
		5'd2:    sec_kw = SAVE_DECL1 ? 32'd3 : SAVE_DECL0 ? 32'd2 : 32'd1;
		5'd4:    sec_kw = sec_desc(wd[0] ? SEC_RD : 8'd0, SEC_SLOT[3:0], SEC_FRAME[12:0]);
		5'd5:    sec_kw = 32'd32;
		5'd8:    sec_kw = SAVE_DECL0 ? sec_desc(wd[1] ? SEC_RDWR : 8'd0, SAVE0_SLOT[3:0], SAVE0_FRAME[12:0]) : 32'd0;
		5'd9:    sec_kw = SAVE_DECL0 ? SAVE0_LBA_BITS : 32'd0;
		5'd10:   sec_kw = SAVE_DECL0 ? SAVE0_WSRC : 32'd0;
		5'd11:   sec_kw = SAVE_DECL0 ? SAVE0_EXT : 32'd0;
		5'd12:   sec_kw = SAVE_DECL1 ? sec_desc(wd[2] ? SEC_RDWR : 8'd0, SAVE1_SLOT[3:0], SAVE1_FRAME[12:0]) : 32'd0;
		5'd13:   sec_kw = SAVE_DECL1 ? SAVE1_LBA_BITS : 32'd0;
		5'd14:   sec_kw = SAVE_DECL1 ? SAVE1_WSRC : 32'd0;
		5'd15:   sec_kw = SAVE_DECL1 ? SAVE1_EXT : 32'd0;
		default: sec_kw = 32'd0;                             // K3, and K6-K7: the primary has no image
	endcase
endfunction

function automatic [31:0] osd_kw(input [4:0] k);
	case (k)
		5'd0:    osd_kw = {OSD_ROWS, 16'd256};                 // {rows, bytes per row}
		5'd1:    osd_kw = {16'd1 << OSD_AW, 16'd0};            // FIFO entries
`ifdef MISTER_DISABLE_SPI_OSD
		5'd2:    osd_kw = 32'd1;                               // bit 0: the link's row writes are not built (osd.v)
`endif
		default: osd_kw = 32'd0;
	endcase
endfunction

function automatic [31:0] evt_kw(input [4:0] k);
	evt_kw = (k == 5'd0) ? 32'd4 : 32'd0;                  // K0: the status words
endfunction

function automatic [31:0] lane_kw(input [4:0] k);
	case (k)
		5'd8:    lane_kw = LANE_CAPS;
		5'd9:    lane_kw = {16'd1 << LANE_AW, 8'd0, LANE_GAP};
		5'd10:   lane_kw = {16'd1 << LANE_RAW, 16'd0};       // reply FIFO depth
		default: lane_kw = (k < 5'd8) ? LANE_OPS[{k[2:0], 5'd0} +: 32] : 32'd0;
	endcase
endfunction

// The directory's word: the header's registers and constants, then the entry table (dx = word offset from the
// first entry: entry de, its word dew, kind word dk).
wire  [9:0] dw  = lw_raddr[11:2];
wire  [9:0] dx  = dw - 10'd16;
wire  [4:0] de  = dx[9:5];
wire  [4:0] dew = dx[4:0];
wire  [4:0] dk  = dew - 5'd8;
reg  [31:0] dir_ent, dir_rdata;
always @(*) begin
	dir_ent = 32'd0;
	if (has_pad && de == e_pad)
		dir_ent = ent(dew, PAD_KIND, PAD_VER, PAD_OFF, PAD_SIZE, 21'd0, PAD_LINE, 16'd0, pad_kw(dk));
	if (has_sec && de == e_sec)             // CAUSE_BITS: one per wired slot
		dir_ent = ent(dew, SEC_KIND, SEC_VER, SEC_OFF, SEC_SIZE, SEC_OFF, SEC_LINE, {13'd0, wired},
		              sec_kw(dk, wired));
	if (OSD_EN && de == e_osd)
		dir_ent = ent(dew, OSD_KIND, OSD_VER, OSD_OFF, OSD_SIZE, 21'd0, OSD_LINE, 16'd0, osd_kw(dk));
	if (has_evt && de == e_evt)
		dir_ent = ent(dew, EVT_KIND, EVT_VER, EVT_OFF, EVT_SIZE, EVT_OFF, EVT_LINE, EVT_BITS, evt_kw(dk));
	if (LANE_EN && de == e_lane)
		dir_ent = ent(dew, LANE_KIND, LANE_VER, LANE_OFF, LANE_SIZE, LANE_CAUSE, LANE_LINE, LANE_BITS, lane_kw(dk));

	if (dw < 10'd16) case (dw[3:0])
		4'd0:    dir_rdata = 32'h5244574C;                   // MAGIC "LWDR"
		4'd1:    dir_rdata = 32'h00020000;                   // VERSION 2.0
		4'd2:    dir_rdata = {STRIDE, HDR_LEN};              // LAYOUT
		4'd3:    dir_rdata = {27'd0, count};
		4'd4:    dir_rdata = {11'd0, WIN_OFF};               // WIN_BASE
		4'd5:    dir_rdata = {11'd0, WIN_SIZE};
		4'd6:    dir_rdata = ena_rd;                         // ENABLE
		4'd8:    dir_rdata = pending;
		4'd9:    dir_rdata = owned_w;                        // OWNED
		4'd10:   dir_rdata = DIAG ? scratch : 32'd0;
		4'd11:   dir_rdata = open_w;                         // OPEN: each listed block takes stores now
		4'd12:   dir_rdata = line_en;                        // LINE_EN
		4'd13:   dir_rdata = {16'd0, n_freed, n_hrst};       // RESETS
		default: dir_rdata = 32'd0;                          // 0x1C, 0x38-0x3C
	endcase
	else dir_rdata = dir_ent;
end

// The read word: one AND-OR over the exclusive selects.
assign lw_rdata = ({32{r_dir}}  & dir_rdata)  | ({32{r_sec}} & sec_rdata) | ({32{r_evt}} & evt_rdata) |
                  ({32{r_osd}}  & osd_rdata)  | ({32{r_lane}} & lane_rdata) | ({32{r_pad}} & pad_rdata) |
                  ({32{r_none}} & 32'hDEADBEEF);

always @(posedge clk) begin
	if (DIAG && wr && w_dir && lw_waddr[11:0] == 12'h028) begin
		if (lw_be[0]) scratch[ 7: 0] <= lw_wdata[ 7: 0];
		if (lw_be[1]) scratch[15: 8] <= lw_wdata[15: 8];
		if (lw_be[2]) scratch[23:16] <= lw_wdata[23:16];
		if (lw_be[3]) scratch[31:24] <= lw_wdata[31:24];
	end
end

// HPS_BUS's to-core bits not driven by a block below: the PAD store [94:46], SECTOR's [131:95].
wire [48:0] pad_q;
wire        sec_en_sys, sec_ack, sec_wr;
wire [15:0] sec_dat;
wire [13:0] sec_adr;
wire  [1:0] sec_xen, sec_xack;

assign hps_lw[94:46]   = pad_q;                          // {we, op, idx, data}
assign hps_lw[95]      = sec_en_sys;                     // the primary's en_sys and ack
assign hps_lw[96]      = sec_ack;
assign hps_lw[97]      = sec_wr;                         // the shared strobe, its data and address
assign hps_lw[113:98]  = sec_dat;
assign hps_lw[127:114] = sec_adr;
// the save slots' en_sys, ack; a slot the OCRAM feeder serves takes its bits
assign hps_lw[131:128] = {MNT_EXT[1] ? xmnt_ack[1] : sec_xack[1], MNT_EXT[1] ? xmnt_en[1] : sec_xen[1],
                          MNT_EXT[0] ? xmnt_ack[0] : sec_xack[0], MNT_EXT[0] ? xmnt_en[0] : sec_xen[0]};

generate
`ifdef MISTER_LW_PAD
if (PAD_EN) begin : pad
	lw_pad pad
	(
		.clk(clk),
		.raddr(rpad), .rdata(pad_rdata),
		.waddr(wpad), .wsel(w_pad), .write(wr), .wdata(lw_wdata), .be(lw_be),
		.clk_sys(clk_sys), .q(pad_q)
	);
end
`else
// PAD_EN without the tier macro: lw_pad.sv is empty, so the build stops here, naming the macro
if (PAD_EN) begin : pad
	lw_pad_needs_MISTER_LW_PAD guard();
end
`endif
else begin : nopad
	assign pad_rdata = 32'd0;
	assign pad_q     = 49'd0;
end
endgenerate

generate
`ifdef MISTER_LW_EVT
if (EVT_EN) begin : evt
	lw_evt evt
	(
		.clk(clk),
		.raddr(revt[9:0]), .rsel(r_evt && revt[11:10] == 2'd0), .rdata(evt_rdata),
		.waddr(wevt[9:0]), .wsel(w_evt && wevt[11:10] == 2'd0), .write(wr), .wdata(lw_wdata), .be(lw_be),
		.take(e_take),
		.irq(evt_irq),
		.clk_sys(clk_sys), .st(hps_lw[371:240]), .ev(hps_lw[239:237]), .lvl(hps_lw[374:372])
	);
end
`else
// EVT_EN without the tier macro: lw_evt.sv is empty, so the build stops here, naming the macro
if (EVT_EN) begin : evt
	lw_evt_needs_MISTER_LW_EVT guard();
end
`endif
else begin : noevt
	assign evt_rdata = 32'd0;
	assign evt_irq   = 1'b0;
end
endgenerate

generate
`ifdef MISTER_LW_SECTOR_SLOT
if (SEC_EN) begin : sec
	if (SEC_BAD) begin : geometry
		lw_sector_unservable_geometry guard();
	end
	// synthesis translate_off
	initial if (SEC_BAD) begin
		$display({"lw_sys_window: unservable SECTOR geometry: slot %0d, frame %0d, gap %0d; save slot 0 (%0d) on ",
		          "slot %0d, frame %0d, %0d LBA bits, image 0x%08h; save slot 1 (%0d) on slot %0d, frame %0d, %0d LBA ",
		          "bits, image 0x%08h"}, SEC_SLOT, SEC_FRAME, SEC_GAP, SAVE_DECL0, SAVE0_SLOT, SAVE0_FRAME,
		         SAVE0_LBA_BITS, SAVE0_WSRC, SAVE_DECL1, SAVE1_SLOT, SAVE1_FRAME, SAVE1_LBA_BITS, SAVE1_WSRC);
		$finish;
	end
	// synthesis translate_on
	lw_sector #(.SLOT(SEC_SLOT[7:0]), .FRAME(SEC_FRAME[12:0]), .BLKSZ(BLKSZ), .AW(SEC_AW), .DIAG(DIAG),
	            .XSLOT0(SAVE0_SLOT[3:0]), .XSLOT1(SAVE1_SLOT[3:0]), .XFRAME0(SAVE0_FRAME[12:0]),
	            .XFRAME1(SAVE1_FRAME[12:0]), .XLB0(SAVE0_LBA_BITS[5:0]), .XLB1(SAVE1_LBA_BITS[5:0]),
	            .GAP(SEC_GAP[3:0])) sector
	(
		.clk(clk),
		.raddr(rsec), .rsel(r_sec), .rdata(sec_rdata),
		.waddr(wsec), .wsel(w_sec), .write(wr_sec), .wdata(lw_wdata), .be(lw_be), .hen(en_q),
		.take(e_take), .f_open(sec_open), .f_owned(sec_owned), .f_idle(sec_idle),
		.wired(wired),
		.doorbell(sec_bell),
		.clk_sys(clk_sys),
		// HPS_BUS[166:132], the primary's mux: {lba[31:0], req, hps_ack, reset}
		.c_reset(hps_lw[132]), .hps_ack(hps_lw[133]), .c_req(hps_lw[134] & wired[0]), .c_lba(hps_lw[166:135]),
		.c_commit(hps_lw[380]),                     // HPS_BUS[380], hps_io's commit mark
		.c_wide(wide_c),
		.en_sys(sec_en_sys), .o_ack(sec_ack), .o_wr(sec_wr), .o_dat(sec_dat), .o_adr(sec_adr),
		// HPS_BUS[201:167] and [236:202], the save slots' muxes: {lba[31:0], rd, wr, hps_ack}; an undeclared or
		// absent slot's request lines are held low, so the feeder never serves it
		.x_hps_ack({hps_lw[202], hps_lw[167]}),
		.x_wr({hps_lw[203], hps_lw[168]} & wired[2:1]), .x_rd({hps_lw[204], hps_lw[169]} & wired[2:1]),
		.x_lba({hps_lw[236:205], hps_lw[201:170]}),
		.x_en_sys(sec_xen), .x_ack(sec_xack)
	);
end
`else
// SEC_EN without the tier macro: lw_sector.sv is empty, so the build stops here, naming the macro
if (SEC_EN) begin : sec
	lw_sector_needs_MISTER_LW_SECTOR_SLOT guard();
end
`endif
else begin : nosec
	assign sec_rdata  = 32'd0;
	assign sec_bell   = 1'b0;
	assign sec_open   = 1'b0;
	assign sec_owned  = 1'b0;
	assign sec_idle   = 1'b1;
	// with SEC_EXT the OCRAM feeder drives the primary's bits; the assigns above stay the one driver of each
	assign sec_en_sys = SEC_EXT ? xsec_en  : 1'b0;
	assign sec_ack    = SEC_EXT ? xsec_ack : 1'b0;
	assign sec_wr     = SEC_EXT ? xsec_wr  : 1'b0;
	assign sec_dat    = SEC_EXT ? xsec_dat : 16'd0;
	assign sec_adr    = SEC_EXT ? xsec_adr : 14'd0;
	assign sec_xen    = 2'd0;
	assign sec_xack   = 2'd0;
end
// both SECTOR macros: two feeders for one slot's bits; the build stops here
if (SEC_EN && SEC_EXT) begin : sec2
	sector_slot_on_two_feeders guard();
end
endgenerate

// The OCRAM feeder's view of the primary slot: the mux's request (wired by its presence and the mark's) and LBA.
assign xsec_reset   = SEC_EXT ? hps_lw[132] : 1'b0;
assign xsec_hps_ack = SEC_EXT ? hps_lw[133] : 1'b0;
assign xsec_req     = SEC_EXT ? hps_lw[134] & pres_sec[0] & pres_cmt : 1'b0;
assign xsec_lba     = SEC_EXT ? hps_lw[166:135] : 32'd0;
assign xsec_commit  = SEC_EXT ? hps_lw[380] : 1'b0;
// and of the mount slots
wire  [1:0] mnt_wired = MNT_EXT & pres_sec[2:1] & {2{pres_cmt}};
assign xmnt_hps_ack = MNT_EXT & {hps_lw[202], hps_lw[167]};
assign xmnt_rd      = mnt_wired & {hps_lw[204], hps_lw[169]};
assign xmnt_wr      = mnt_wired & {hps_lw[203], hps_lw[168]};
assign xmnt_lba     = {MNT_EXT[1] ? hps_lw[236:205] : 32'd0, MNT_EXT[0] ? hps_lw[201:170] : 32'd0};

generate
`ifdef MISTER_LW_LANE
if (LANE_EN) begin : lane
	lw_lane #(.AW(LANE_AW), .GAP(LANE_GAP[2:0]), .RAW(LANE_RAW)) lane
	(
		.clk(clk),
		.raddr(rlane), .rdata(lane_rdata),
		.waddr(wlane), .wsel(w_lane), .write(wr_lane), .wdata(lw_wdata), .be(lw_be), .hen(en_q),
		.take(e_take), .f_open(lane_open), .f_owned(lane_owned), .f_idle(lane_idle), .irq(lane_irq),
		.clk_sys(clk_sys), .l_busy(lane_busy), .l_wait(lane_wait), .l_dout(lane_dout),
		.own(lane_own), .hold(lane_hold), .sel(lane_sel), .strobe(lane_strobe), .din(lane_din)
	);
end
`else
// LANE_EN without the tier macro: lw_lane.sv is empty, so the build stops here, naming the macro
if (LANE_EN) begin : lane
	lw_lane_needs_MISTER_LW_LANE guard();
end
`endif
else begin : nolane
	assign lane_rdata  = 32'd0;
	assign lane_open   = 1'b0;
	assign lane_owned  = 1'b0;
	assign lane_idle   = 1'b1;
	assign lane_irq    = 1'b0;
	assign lane_own    = 1'b0;
	assign lane_hold   = 1'b0;
	assign lane_sel    = 4'd0;
	assign lane_strobe = 1'b0;
	assign lane_din    = 16'd0;
end
endgenerate

generate
`ifdef MISTER_LW_OSD
if (OSD_EN) begin : osd
	lw_osd #(.ROWS(OSD_ROWS), .AW(OSD_AW)) osd
	(
		.clk(clk),
		.raddr(rosd), .rdata(osd_rdata),
		.waddr(wosd), .wsel(w_osd), .write(wr_osd), .wdata(lw_wdata), .be(lw_be), .hen(en_q),
		.take(e_take), .f_open(osd_open), .f_owned(osd_owned), .f_idle(osd_idle),
		.clk_sys(clk_sys), .sel(osd_sel), .view(osd_view),
		.we(osd_we), .addr(osd_addr), .dbyte(osd_byte)
	);
end
`else
// OSD_EN without the tier macro: lw_osd.sv is empty, so the build stops here, naming the macro
if (OSD_EN) begin : osd
	lw_osd_needs_MISTER_LW_OSD guard();
end
`endif
else begin : noosd
	assign osd_rdata = 32'd0;
	assign osd_open  = 1'b0;
	assign osd_owned = 1'b0;
	assign osd_idle  = 1'b1;
	assign osd_we    = 1'b0;
	assign osd_addr  = 13'd0;
	assign osd_byte  = 8'd0;
end
endgenerate

endmodule

`default_nettype wire
`endif
