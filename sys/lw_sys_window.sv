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

// LW window declared by `SYS_LW_CAPS (emu_ports.vh): the directory, then the PAD, SECTOR, OSDBUF, SYS and LANE
// blocks. Bridge side runs on clk_100m with no reset.
// Directory, layout 2: a 16-word header (MAGIC "LWDR", VERSION 2.0, LAYOUT = {entry stride, header length} in
// bytes, COUNT, WIN_BASE and WIN_SIZE, LEASE, TIMEBASE, PENDING, OWNED, SCRATCH, OPEN, LINE_EN, RESETS), then one
// 32-word entry per block built, in ascending kind order with no gaps. An entry's 8 common words are KIND, VERSION, OFFSET, SIZE, CAUSE, LINE, CAUSE_BITS and
// TAG; its kind's own words follow. Every offset in the directory is window-relative. A line is declared in one
// place, the entry's LINE word: the same localparam picks the entry's bit of `lines` (ordinal k = lines[k-1] =
// f2h_irq[7+k]), and no two entries name one ordinal.
// The lease: live_q, one clk_100m register, is the write enable of the whole window. Every store but one to LEASE
// sees lw_write & live_q, here, above every block; a store dropped at that gate sets LEASE.stale. A CLAIM names the
// storing host's id and is taken only with no lease held, no HPS reset seen (lw_hrst) and GPO[31] high (lw_idv
// low); RENEW and RELEASE act only for the id that holds the lease. live_q clears, in this priority, on lw_hrst, on
// lw_idv, on a RELEASE, or when the counter has run out (a RENEW in that cycle wins), and LEASE.end says which.
// live_q is GPI[22] (held). It crosses no clock itself. SECTOR and LANE, whose queues a clk_sys half consumes, each
// have a lease link (lw_fence, lw_cdc.sv) that carries the lease to that half as one level and its acknowledge
// back: a store to such a block also needs the link open (its OPEN bit), which it is only once the half has
// seen the lease, and a store dropped there sets stale too. A CLAIM is taken only while every link is idle with
// the acknowledge of rest standing (LEASE.quiet), so no half still returning from a lease takes work under the
// next. OWNED shows each link that holds the claim back. Every other block's OPEN bit is live_q, its OWNED bit 0.
// At a taken CLAIM each block reloads its CAUSE word: SYS raises STATUS, SECTOR clears its own.
// Lines: each is a register, live_q & the entry's LINE_EN bit & the entry's CAUSE word non-zero, so a line is a
// level that falls only when one of the three does. LINE_EN reads 0 whenever no lease is held. PENDING is not
// gated. RESETS counts rises of lw_hrst and the state machines the firewall freed, each saturating at 255.
// Decode: each block's select compares the address's high bits with a constant, so the selects are exclusive;
// every block's word is 0 unless it is selected, and the read word is one AND-OR of the block words and the
// 0xDEADBEEF default. An address in no block that is built reads 0xDEADBEEF and drops stores; an unassigned
// offset inside a block reads 0.
// PAD stores park in per-op latest-wins slots (digital) or a 16-deep FIFO (both analog ops, drop-and-count when
// full), both in one M10K; a held-data handshake (lw_xfer, lw_cdc.sv) drains them round-robin to clk_sys.
// The SECTOR block (lw_sector.sv) is built only when the core declares it (SEC_EN): one primary hps_io read slot
// plus up to two save slots ("cards"); its kind words declare each card slot, its DDR image base and extent.
// OSDBUF decodes image stores and CTRL; the shadow RAMs live in each osd instance (osd.v).
// SYS (lw_sys_status.sv) raises the status back-channel's cause on its own line.
// LANE (lw_lane.sv, LANE_EN) replays ordered link frames onto sys_top's link nets and can capture their replies.
// LW_DIAG=0 (shipping) compiles out SCRATCH (reads 0, writes ignored), the analog-drop count (reads 0)
// and lw_sector's diagnostics; LW_DIAG=1 (debug builds) keeps them.
// See docs/LW_BRIDGE.md for the register map and the adoption recipe.

module lw_sys_window #(
	// SECTOR block, as the core declares it (sys_top.v maps the MISTER_LW_SECTOR_* / MISTER_LW_CARD* macros).
	parameter        SEC_EN    = 0,          // 1 = build the SECTOR block and list its directory entry
	parameter  [7:0] SEC_SLOT  = 8'd1,       // the hps_io slot the primary frames feed (sd_rd[SEC_SLOT])
	parameter  [9:0] SEC_FRAME = 10'd128,    // 32-bit DATA words per primary frame (128 = one 512-byte block)
	parameter  [2:0] BLKSZ     = 3'd2,       // the core's hps_io BLKSZ, reported in the GET_SDSTAT word
	parameter  [3:0] MC_SLOT0  = 4'hE,       // card 0: hps_io slot (0-3), its DDR image base and extent;
	parameter [31:0] MC_WSRC0  = 32'd0,      //   a card with wsrc 0 is not declared (slot 4'hE never decodes)
	parameter [31:0] MC_EXT0   = 32'd0,
	parameter  [3:0] MC_SLOT1  = 4'hF,       // card 1, likewise
	parameter [31:0] MC_WSRC1  = 32'd0,
	parameter [31:0] MC_EXT1   = 32'd0,
	parameter  [9:0] MC_FRAME  = 10'd256,    // 32-bit DATA words per card frame (256 = one 1 KB block)
	// LANE block (sys_top.v sets it with MISTER_LW_LANE): 1 = build the frame lane and list its entry.
	parameter        LANE_EN   = 0,
	// the lease unit, 2^n clk cycles (25: 0.3355 s at 100 MHz); TIMEBASE reports it
	parameter  [7:0] LEASE_UNIT_LOG2 = 8'd25
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
	output wire        held,         // the lease register live_q: GPI[22]

	output wire  [2:0] lines,        // line ordinal k = lines[k-1] -> f2h_irq[7+k], level-high

	input wire         clk_sys,
	output wire [48:0] lw_pad,       // {we, op[7:0], idx[7:0], data[31:0]} on clk_sys
	input wire  [34:0] blk_up,       // {reset, hps_ack, req, lba[31:0]} on clk_sys, from emu
	output wire [18:0] blk_dn,       // {en_sys, ack, wr, data[15:0]} on clk_sys, to emu
	input wire  [19:0] card_up,      // {card 1, card 0}, each {hps_ack, wr, rd, lba[6:0]} on clk_sys, from emu
	output wire [12:0] card_dn,      // {ack1, en_sys1, ack0, en_sys0, addr[8:0]} on clk_sys, to emu
	input wire [131:0] st_up,        // {stflg[3:0], status_req[127:0]} on clk_sys, from hps_io

	input wire         lane_busy,    // clk_sys: any of the link's own selects, or its ID view (GPO[31] low)
	input wire         lane_wait,    // clk_sys: io_wait | vs_wait
	input wire  [15:0] lane_dout,    // clk_sys: the link's reply word, io_dout | io_dout_sys
	output wire        lane_own,     // clk_sys: the lane player, to lw_link
	output wire        lane_hold,
	output wire  [3:0] lane_sel,
	output wire        lane_strobe,
	output wire [15:0] lane_din,

	output wire        osd_we,       // OSDBUF image store, clk_100m: word osd_addr of the 256x128 image
	output wire  [9:0] osd_addr,
	output wire [31:0] osd_data,
	output wire  [3:0] osd_be,
	output wire  [1:0] osd_ctrl      // {LWSEL & HR, LWSEL}, clk_100m registers; each osd synchronizes them
);

`ifdef LW_DIAG
localparam        DIAG     = `LW_DIAG;
`else
localparam        DIAG     = 0;
`endif

// The window (absolute LW offset and size: the `SYS_LW_CAPS token's two numbers) and its blocks, window-relative.
// Every block is aligned to its size, except SECTOR, which spans two 4 KB pages from 0x01000.
localparam [20:0] WIN_OFF   = 21'h40000;
localparam [20:0] WIN_SIZE  = 21'h80000;
localparam [20:0] DIR_OFF   = 21'h00000;   // the directory: header and entries, the rest of its page reads 0
localparam [20:0] SEC_OFF   = 21'h01000;
localparam [20:0] SEC_SIZE  = 21'h02000;
localparam [20:0] SYS_OFF   = 21'h03000;
localparam [20:0] SYS_SIZE  = 21'h01000;
localparam [20:0] OSD_OFF   = 21'h04000;
localparam [20:0] OSD_SIZE  = 21'h02000;
localparam [20:0] LANE_OFF  = 21'h06000;
localparam [20:0] LANE_SIZE = 21'h01000;
localparam [20:0] PAD_OFF   = 21'h40000;
localparam [20:0] PAD_SIZE  = 21'h40000;

localparam [20:0] DIR_A  = WIN_OFF + DIR_OFF;      // absolute block starts, for the selects
localparam [20:0] SEC_A  = WIN_OFF + SEC_OFF;
localparam [20:0] SEC_A2 = SEC_A + 21'h01000;
localparam [20:0] SYS_A  = WIN_OFF + SYS_OFF;
localparam [20:0] OSD_A  = WIN_OFF + OSD_OFF;
localparam [20:0] LANE_A = WIN_OFF + LANE_OFF;
localparam [20:0] PAD_A  = WIN_OFF + PAD_OFF;

// The directory's header (bytes) and the entries, in ascending kind order: only blocks that are built are listed.
localparam [15:0] HDR_LEN  = 16'h0040;
localparam [15:0] STRIDE   = 16'h0080;
localparam  [4:0] E_PAD    = 5'd0;
localparam  [4:0] E_SEC    = E_PAD + 5'd1;                 // listed with SEC_EN only
localparam  [4:0] E_OSD    = E_SEC + (SEC_EN ? 5'd1 : 5'd0);
localparam  [4:0] E_SYS    = E_OSD + 5'd1;
localparam  [4:0] E_LANE   = E_SYS + 5'd1;                 // listed with LANE_EN only
localparam  [4:0] COUNT    = E_LANE + (LANE_EN ? 5'd1 : 5'd0);

// Kinds, versions and lines. Ordinal 0 = no line; framework kinds use 1-15, each ordinal at most once.
localparam [15:0] PAD_KIND  = 16'd1;
localparam [31:0] PAD_VER   = 32'h00010000;
localparam  [7:0] PAD_LINE  = 8'd0;
localparam [15:0] SEC_KIND  = 16'd2;
localparam [31:0] SEC_VER   = 32'h00010001;
localparam  [7:0] SEC_LINE  = 8'd1;                        // f2h_irq[8], GIC SPI 48
localparam [15:0] SEC_BITS  = 16'h0001;                    // the request event
localparam [15:0] OSD_KIND  = 16'd3;
localparam [31:0] OSD_VER   = 32'h00010000;
localparam  [7:0] OSD_LINE  = 8'd0;
localparam [15:0] SYS_KIND  = 16'd4;
localparam [31:0] SYS_VER   = 32'h00010000;
localparam  [7:0] SYS_LINE  = 8'd2;                        // f2h_irq[9], GIC SPI 49
localparam [15:0] SYS_BITS  = 16'h0001;                    // STATUS, per capture
localparam [15:0] LANE_KIND = 16'd5;
localparam [31:0] LANE_VER  = 32'h00010001;
localparam  [7:0] LANE_LINE = 8'd0;

localparam        SEC_AW    = 9;          // FIFO depth 512
localparam [15:0] SEC_GAP   = 16'd2;      // clk_sys cycles between write strobes
localparam  [7:0] MC_FLAGS  = 8'h03;      // RD: BEGIN_S frames; WR: WACK_S, bytes from the DDR image
localparam        MC_DECL0  = SEC_EN && MC_WSRC0 != 0;
localparam        MC_DECL1  = SEC_EN && MC_WSRC1 != 0;
localparam [12:0] OSD_CTRL  = 13'h1000;   // block-relative; the image is 0x0000-0x0FFF
localparam [31:0] OSD_GEOM  = {16'd128, 16'd256};
localparam        LANE_AW   = 10;         // FIFO depth 1024: a 256-entry 0x33 gamma table is 770 entries
localparam  [7:0] LANE_GAP  = 8'd4;       // clk_sys cycles without a wait between strobes, 2..7
localparam        LANE_RAW  = 8;          // reply FIFO depth 256

// LANE kind words K0-K7: the UIO (sel 0) opcodes verified for write-only replay; K8, the first-word reply the lane
// stands in for, one byte per op: [7:0] = 0x20's 'b11 (sys_top answers it); a capture frame also records it.
localparam [255:0] LANE_OPS = (256'd1 << 8'h01) | (256'd1 << 8'h1C) | (256'd1 << 8'h1D) | (256'd1 << 8'h1E) |
                              (256'd1 << 8'h22) | (256'd1 << 8'h24) | (256'd1 << 8'h31) | (256'd1 << 8'h32) |
                              (256'd1 << 8'h33) | (256'd1 << 8'h3B);
localparam [31:0] LANE_CAPS = 32'h0000_0003;

// The header's read/write words, as word indices of the directory page.
localparam  [9:0] H_LEASE = 10'h006, H_LINEN = 10'h00C;
localparam [31:0] PRESENT = (32'd1 << COUNT) - 32'd1;   // bit i = entry i is listed
localparam        LCW     = 4 + LEASE_UNIT_LOG2;        // the lease counter: {TTL, unit}

localparam [255:0] PAD_OPS = (256'd1 << 8'h02) | (256'd1 << 8'h03) |
                             (256'd1 << 8'h10) | (256'd1 << 8'h11) |
                             (256'd1 << 8'h12) | (256'd1 << 8'h13) |
                             (256'd1 << 8'h1A) | (256'd1 << 8'h3D);

// The lease and the lines.
reg            live_q  = 0;           // the lease
reg  [LCW-1:0] lcnt    = 0;           // counts down while live_q is set
reg      [9:0] owner   = 0;           // the id of the last taken CLAIM
reg      [7:0] epoch   = 0;           // taken CLAIMs, modulo 256
reg      [1:0] lend    = 0;           // why the last lease ended: 0 RELEASE or none, 1 ran out, 2 HPS reset, 3 GPO[31]
reg            stale   = 0;           // a store other than LEASE was dropped at the gate, since the last CLAIM
reg            lburst  = 0;           // a burst-write beat was dropped, since the last CLAIM
reg     [31:0] line_en = 0;           // LINE_EN
reg      [2:0] line_q  = 0;           // the lines
reg      [7:0] n_hrst  = 0, n_freed = 0;
reg            hrst_d  = 1'b1;        // lw_hrst is high from configuration: configuration is no rise
wire           quiet;                 // every lease link lets a CLAIM through

reg [31:0] scratch = 0;
reg [31:0] adrop   = 0;               // analog stores dropped on a full FIFO
reg  [1:0] octrl   = 0;               // OSD CTRL, {HR, LWSEL}
reg        ohr     = 0;

// Selects: high address bits against a constant, read and write side alike.
wire        r_dir  = lw_raddr[20:12] == DIR_A[20:12];
wire        r_sec  = SEC_EN && (lw_raddr[20:12] == SEC_A[20:12] || lw_raddr[20:12] == SEC_A2[20:12]);
wire        r_sys  = lw_raddr[20:12] == SYS_A[20:12];
wire        r_osd  = lw_raddr[20:13] == OSD_A[20:13];
wire        r_lane = LANE_EN && lw_raddr[20:12] == LANE_A[20:12];
wire        r_pad  = lw_raddr[20:18] == PAD_A[20:18];
wire        r_none = ~(r_dir | r_sec | r_sys | r_osd | r_lane | r_pad);

wire        w_dir  = lw_waddr[20:12] == DIR_A[20:12];
wire        w_sec  = SEC_EN && (lw_waddr[20:12] == SEC_A[20:12] || lw_waddr[20:12] == SEC_A2[20:12]);
wire        w_sys  = lw_waddr[20:12] == SYS_A[20:12];
wire        w_osd  = lw_waddr[20:13] == OSD_A[20:13];
wire        w_lane = LANE_EN && lw_waddr[20:12] == LANE_A[20:12];
wire        w_pad  = lw_waddr[20:18] == PAD_A[20:18];

// The gate: LEASE takes the raw store, every other store needs the lease, and a store to a block with a lease link
// needs that link open as well.
wire        sec_open, sec_owned, sec_idle, lane_open, lane_owned, lane_idle;
wire        w_lease_a = w_dir && lw_waddr[11:2] == H_LEASE;
wire        wr        = lw_write & live_q;
wire        wr_sec    = wr & sec_open;
wire        wr_lane   = wr & lane_open;
assign      quiet     = sec_idle & lane_idle;
wire        l_st      = lw_write && w_lease_a && &lw_be && lw_wdata[31:16] == 16'h4C57;   // the key
wire  [9:0] l_id      = lw_wdata[15:6];
wire  [1:0] l_verb    = lw_wdata[5:4];
wire  [3:0] l_ttl     = lw_wdata[3:0];
wire        l_claim   = l_st && l_verb == 2'd1 && l_ttl != 4'd0 && !live_q && !lw_hrst && !lw_idv && quiet;
wire        l_renew   = l_st && l_verb == 2'd2 && l_ttl != 4'd0 && live_q && l_id == owner;
wire        l_release = l_st && l_verb == 2'd3 && live_q && l_id == owner;

// Block-relative addresses: the address bits below each block's size (SECTOR's second page is its upper half).
wire [12:0] rsec  = {lw_raddr[12] ^ SEC_A[12], lw_raddr[11:0]};
wire [12:0] wsec  = {lw_waddr[12] ^ SEC_A[12], lw_waddr[11:0]};
wire [11:0] rsys  = lw_raddr[11:0];
wire [11:0] wsys  = lw_waddr[11:0];
wire [12:0] rosd  = lw_raddr[12:0];
wire [12:0] wosd  = lw_waddr[12:0];
wire [11:0] rlane = lw_raddr[11:0];
wire [11:0] wlane = lw_waddr[11:0];
wire [17:0] rpad  = lw_raddr[17:0];
wire [17:2] wpad  = lw_waddr[17:2];

wire        sec_lvl, sys_lvl;          // each block's CAUSE word is non-zero
wire [31:0] sec_rdata, sys_rdata, lane_rdata;

// The lines: each entry's cause level and LINE_EN bit on the bit its LINE word names, registered under the lease.
function automatic [2:0] line_bit(input [7:0] ord);
	line_bit = (ord >= 8'd1 && ord <= 8'd3) ? (3'd1 << (ord - 8'd1)) : 3'd0;
endfunction

wire  [2:0] line_d = ({3{sec_lvl & line_en[E_SEC]}} & line_bit(SEC_LINE)) |
                     ({3{sys_lvl & line_en[E_SYS]}} & line_bit(SYS_LINE));

always @(posedge clk) line_q <= {3{live_q}} & line_d;

assign lines = line_q;
assign held  = live_q;

function automatic [7:0] sat_add(input [7:0] v, input [1:0] n);
	sat_add = ({1'b0, v} + {7'd0, n} > 9'd255) ? 8'd255 : v + {6'd0, n};
endfunction

always @(posedge clk) begin
	hrst_d  <= lw_hrst;
	n_hrst  <= sat_add(n_hrst, {1'b0, lw_hrst & ~hrst_d});
	n_freed <= sat_add(n_freed, {1'b0, lw_freed[0]} + {1'b0, lw_freed[1]});

	if (lw_hrst) begin
		if (live_q) lend <= 2'd2;
		live_q <= 0;
		lcnt   <= 0;
	end
	else if (lw_idv) begin
		if (live_q) lend <= 2'd3;
		live_q <= 0;
		lcnt   <= 0;
	end
	else if (l_release) begin
		lend   <= 2'd0;
		live_q <= 0;
		lcnt   <= 0;
	end
	else if (l_renew) lcnt <= {l_ttl, {LEASE_UNIT_LOG2{1'b0}}};
	else if (live_q && lcnt == 0) begin
		lend   <= 2'd1;
		live_q <= 0;
	end
	else if (l_claim) begin
		live_q <= 1;
		owner  <= l_id;
		epoch  <= epoch + 8'd1;
		lcnt   <= {l_ttl, {LEASE_UNIT_LOG2{1'b0}}};
	end
	else if (live_q) lcnt <= lcnt - 1'd1;

	if (l_claim) begin
		stale  <= 0;
		lburst <= 0;
	end
	else begin
		if (lw_write && !w_lease_a && (!live_q || (w_sec && !sec_open) || (w_lane && !lane_open))) stale <= 1;
		if (lw_wburst) lburst <= 1;
	end

	if (!live_q) line_en <= 0;
	else if (wr && w_dir && lw_waddr[11:2] == H_LINEN && &lw_be) line_en <= lw_wdata & PRESENT;
end

wire [31:0] lease_rd = {live_q, quiet, stale, lburst, lcnt[LCW-1 -: 4], epoch, lw_hrst, 1'b0, lend, lw_idv, 1'b0,
                        owner};

// PENDING: bit i = entry i's CAUSE word is non-zero.
wire [31:0] pending = ({31'd0, sec_lvl} << E_SEC) | ({31'd0, sys_lvl} << E_SYS);

// OPEN and OWNED: bit i is entry i's. A linked block's bits are its link's; every other block's OPEN is live_q.
localparam [31:0] LINKED = (SEC_EN ? 32'd1 << E_SEC : 32'd0) | (LANE_EN ? 32'd1 << E_LANE : 32'd0);
wire [31:0] open_w  = ({32{live_q}} & PRESENT & ~LINKED) | ({31'd0, sec_open} << E_SEC) |
                      ({31'd0, lane_open} << E_LANE);
wire [31:0] owned_w = ({31'd0, sec_owned} << E_SEC) | ({31'd0, lane_owned} << E_LANE);

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
	pad_kw = (k < 5'd8) ? PAD_OPS[{k[2:0], 5'd0} +: 32] : 32'd0;    // the honoured-opcode bitmap
endfunction

function automatic [31:0] sec_kw(input [4:0] k);
	case (k)
		5'd0:    sec_kw = {SEC_SLOT, 8'd0, 6'd0, SEC_FRAME};
		5'd2:    sec_kw = {16'd1 << SEC_AW, SEC_GAP};
		5'd3:    sec_kw = MC_DECL0 ? {4'd0, MC_SLOT0, MC_FLAGS, 6'd0, MC_FRAME} : 32'd0;
		5'd4:    sec_kw = MC_DECL0 ? MC_WSRC0 : 32'd0;
		5'd5:    sec_kw = MC_DECL1 ? {4'd0, MC_SLOT1, MC_FLAGS, 6'd0, MC_FRAME} : 32'd0;
		5'd6:    sec_kw = MC_DECL1 ? MC_WSRC1 : 32'd0;
		5'd7:    sec_kw = MC_DECL0 ? MC_EXT0 : 32'd0;
		5'd8:    sec_kw = MC_DECL1 ? MC_EXT1 : 32'd0;
		default: sec_kw = 32'd0;                             // K1 is reserved: the CAUSE word is common
	endcase
endfunction

function automatic [31:0] lane_kw(input [4:0] k);
	case (k)
		5'd8:    lane_kw = LANE_CAPS;
		5'd9:    lane_kw = {16'd1 << LANE_AW, 8'd0, LANE_GAP};
		5'd10:   lane_kw = {16'd1 << LANE_RAW, 16'd0};       // reply FIFO depth
		default: lane_kw = (k < 5'd8) ? LANE_OPS[{k[2:0], 5'd0} +: 32] : 32'd0;
	endcase
endfunction

// Entry word x of the table (x = word offset from the first entry): a constant of the build.
function automatic [31:0] dir_entry(input [9:0] x);
	reg [4:0] e, w, k;
	begin
		e = x[9:5];
		w = x[4:0];
		k = w - 5'd8;
		dir_entry = 32'd0;
		if (e == E_PAD)
			dir_entry = ent(w, PAD_KIND, PAD_VER, PAD_OFF, PAD_SIZE, 21'd0, PAD_LINE, 16'd0, pad_kw(k));
		if (SEC_EN && e == E_SEC)
			dir_entry = ent(w, SEC_KIND, SEC_VER, SEC_OFF, SEC_SIZE, SEC_OFF, SEC_LINE, SEC_BITS, sec_kw(k));
		if (e == E_OSD)
			dir_entry = ent(w, OSD_KIND, OSD_VER, OSD_OFF, OSD_SIZE, 21'd0, OSD_LINE, 16'd0,
			                (k == 5'd0) ? OSD_GEOM : 32'd0);    // K0: {height, width} in pixels, 1 bit each
		if (e == E_SYS)
			dir_entry = ent(w, SYS_KIND, SYS_VER, SYS_OFF, SYS_SIZE, SYS_OFF, SYS_LINE, SYS_BITS, 32'd0);
		if (LANE_EN && e == E_LANE)
			dir_entry = ent(w, LANE_KIND, LANE_VER, LANE_OFF, LANE_SIZE, 21'd0, LANE_LINE, 16'd0, lane_kw(k));
	end
endfunction

// The directory's word: the header's registers and constants, then the entry table.
wire  [9:0] dw = lw_raddr[11:2];
reg  [31:0] dir_rdata;
always @(*) begin
	if (dw < 10'd16) case (dw[3:0])
		4'd0:    dir_rdata = 32'h5244574C;                   // MAGIC "LWDR"
		4'd1:    dir_rdata = 32'h00020000;                   // VERSION 2.0
		4'd2:    dir_rdata = {STRIDE, HDR_LEN};              // LAYOUT
		4'd3:    dir_rdata = {27'd0, COUNT};
		4'd4:    dir_rdata = {11'd0, WIN_OFF};               // WIN_BASE
		4'd5:    dir_rdata = {11'd0, WIN_SIZE};
		4'd6:    dir_rdata = lease_rd;                       // LEASE
		4'd7:    dir_rdata = {LEASE_UNIT_LOG2, 24'd100000};  // TIMEBASE {lease unit log2, bridge clock kHz}
		4'd8:    dir_rdata = pending;
		4'd9:    dir_rdata = owned_w;                        // OWNED
		4'd10:   dir_rdata = DIAG ? scratch : 32'd0;
		4'd11:   dir_rdata = open_w;                         // OPEN: each listed block takes stores now
		4'd12:   dir_rdata = line_en;                        // LINE_EN
		4'd13:   dir_rdata = {16'd0, n_freed, n_hrst};       // RESETS
		default: dir_rdata = 32'd0;                          // 0x38-0x3C
	endcase
	else dir_rdata = dir_entry(dw - 10'd16);
end

// The read word: one AND-OR over the exclusive selects.
wire [31:0] osd_rdata = (rosd == OSD_CTRL)          ? {30'd0, octrl} :
                        (rosd == OSD_CTRL + 13'd4)  ? 32'h3144534F :   // "OSD1"
                        32'd0;
wire [31:0] pad_rdata = (DIAG && rpad == {8'h1A, 10'd0}) ? adrop : 32'd0;

assign lw_rdata = ({32{r_dir}}  & dir_rdata)  | ({32{r_sec}} & sec_rdata) | ({32{r_sys}} & sys_rdata) |
                  ({32{r_osd}}  & osd_rdata)  | ({32{r_lane}} & lane_rdata) | ({32{r_pad}} & pad_rdata) |
                  ({32{r_none}} & 32'hDEADBEEF);

generate
`ifdef MISTER_LW_SECTOR_SLOT
if (SEC_EN) begin : sec
	// an undeclared card's request lines are held low, so the feeder never serves it
	wire [1:0] xdecl = {MC_DECL1 ? 1'b1 : 1'b0, MC_DECL0 ? 1'b1 : 1'b0};

	lw_sector #(.SLOT(SEC_SLOT), .FRAME(SEC_FRAME), .BLKSZ(BLKSZ), .AW(SEC_AW), .DIAG(DIAG),
	            .XSLOT0(MC_SLOT0), .XSLOT1(MC_SLOT1), .XFRAME(MC_FRAME)) sector
	(
		.clk(clk),
		.raddr(rsec), .rsel(r_sec), .rdata(sec_rdata),
		.waddr(wsec), .wsel(w_sec), .write(wr_sec), .wdata(lw_wdata), .be(lw_be), .live(live_q),
		.claim(l_claim), .f_open(sec_open), .f_owned(sec_owned), .f_idle(sec_idle),
		.doorbell(sec_lvl),
		.clk_sys(clk_sys),
		.c_reset(blk_up[34]), .hps_ack(blk_up[33]), .c_req(blk_up[32]), .c_lba(blk_up[31:0]),
		.en_sys(blk_dn[18]), .o_ack(blk_dn[17]), .o_wr(blk_dn[16]), .o_dat(blk_dn[15:0]),
		.o_adr(card_dn[8:0]),
		.x_hps_ack({card_up[19], card_up[9]}),
		.x_wr({card_up[18], card_up[8]} & xdecl), .x_rd({card_up[17], card_up[7]} & xdecl),
		.x_lba({card_up[16:10], card_up[6:0]}),
		.x_en_sys({card_dn[11], card_dn[9]}), .x_ack({card_dn[12], card_dn[10]})
	);
end
`else
// SEC_EN without the tier macro: lw_sector.sv is empty, so the build stops here, naming the macro
if (SEC_EN) begin : sec
	lw_sector_needs_MISTER_LW_SECTOR_SLOT guard();
end
`endif
else begin : nosec
	assign sec_rdata = 32'd0;
	assign sec_lvl   = 1'b0;
	assign sec_open  = 1'b0;
	assign sec_owned = 1'b0;
	assign sec_idle  = 1'b1;
	assign blk_dn    = 19'd0;
	assign card_dn   = 13'd0;
end
endgenerate

generate
`ifdef MISTER_LW_LANE
if (LANE_EN) begin : lane
	lw_lane #(.AW(LANE_AW), .GAP(LANE_GAP[2:0]), .RAW(LANE_RAW)) lane
	(
		.clk(clk),
		.raddr(rlane), .rdata(lane_rdata),
		.waddr(wlane), .wsel(w_lane), .write(wr_lane), .wdata(lw_wdata), .be(lw_be), .live(live_q),
		.claim(l_claim), .f_open(lane_open), .f_owned(lane_owned), .f_idle(lane_idle),
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
	assign lane_own    = 1'b0;
	assign lane_hold   = 1'b0;
	assign lane_sel    = 4'd0;
	assign lane_strobe = 1'b0;
	assign lane_din    = 16'd0;
end
endgenerate

lw_sys_status sys_status
(
	.clk(clk),
	.raddr(rsys[9:0]), .rsel(r_sys && rsys[11:10] == 2'd0), .rdata(sys_rdata),
	.waddr(wsys[9:0]), .wsel(w_sys && wsys[11:10] == 2'd0), .write(wr), .wdata(lw_wdata), .be(lw_be),
	.claim(l_claim),
	.irq(sys_lvl),
	.clk_sys(clk_sys), .stflg(st_up[131:128]), .status_req(st_up[127:0])
);

// OSDBUF: image stores go to both osd shadows (byte lanes honour BE); CTRL is {HR, LWSEL}, and reads 0 whenever
// no lease is held, so no shadow is shown without one.
assign osd_we   = wr && w_osd && ~wosd[12];
assign osd_addr = wosd[11:2];
assign osd_data = lw_wdata;
assign osd_be   = lw_be;
assign osd_ctrl = {ohr, octrl[0]};

always @(posedge clk) begin
	if (!live_q) begin
		octrl <= 0;
		ohr   <= 0;
	end
	else if (wr && w_osd && wosd == OSD_CTRL && lw_be[0]) begin
		octrl <= lw_wdata[1:0];
		ohr   <= &lw_wdata[1:0];
	end
end

// clk_100m side: SCRATCH, the pending slots, and the round-robin drain into the handshake.
function automatic [2:0] slot_of(input [7:0] op);
	case (op)
		8'h02: slot_of = 3'd0;
		8'h03: slot_of = 3'd1;
		8'h10: slot_of = 3'd2;
		8'h11: slot_of = 3'd3;
		8'h12: slot_of = 3'd4;
		8'h13: slot_of = 3'd5;
		default: slot_of = 3'd7;
	endcase
endfunction

function automatic [7:0] op_of(input [2:0] slot);
	case (slot)
		3'd0: op_of = 8'h02;
		3'd1: op_of = 8'h03;
		3'd2: op_of = 8'h10;
		3'd3: op_of = 8'h11;
		3'd4: op_of = 8'h12;
		default: op_of = 8'h13;
	endcase
endfunction

// one M10K: digital masks at 0..5, the analog FIFO {right, idx, data[15:0]} at 16..31
(* ramstyle = "M10K" *) reg [31:0] pram [0:31];
reg [31:0] pq;
reg  [5:0] dirty = 0;

reg  [4:0] awp = 0, arp = 0;
wire       aempty = (awp == arp);
wire       afull  = (awp - arp) == 5'd16;

wire       st   = wr && w_pad && &lw_be;
wire [7:0] wop  = wpad[17:10];
wire [7:0] widx = wpad[9:2];
wire [2:0] wsl  = slot_of(wop);
wire       st_d = st && wsl != 3'd7 && widx == 8'd0;
wire       st_a = st && (wop == 8'h1A || wop == 8'h3D) && !st_d;

wire       busy;

wire [6:0] ready = {~aempty, dirty};
reg  [2:0] rr = 0;
reg  [2:0] pick;
reg        pick_v;
reg  [1:0] ds = 0;                     // drain: 0 pick, 1 read, 2 launch
reg  [2:0] psel = 0;
integer    i, k;

always @(*) begin
	pick_v = 0;
	pick   = 0;
	for (i = 0; i < 7; i = i + 1) begin
		k = rr + i;
		if (k >= 7) k = k - 7;
		if (!pick_v && ready[k]) begin
			pick_v = 1;
			pick   = k[2:0];
		end
	end
end

wire [4:0] pra = (psel == 3'd6) ? {1'b1, arp[3:0]} : {2'b00, psel};

always @(posedge clk) begin
	if (st_d) pram[{2'b00, wsl}] <= lw_wdata;
	else if (st_a && !afull) pram[{1'b1, awp[3:0]}] <= {7'd0, wop == 8'h3D, widx, lw_wdata[15:0]};
	pq <= pram[pra];
end

always @(posedge clk) begin
	if (DIAG && wr && w_dir && lw_waddr[11:0] == 12'h028) begin
		if (lw_be[0]) scratch[ 7: 0] <= lw_wdata[ 7: 0];
		if (lw_be[1]) scratch[15: 8] <= lw_wdata[15: 8];
		if (lw_be[2]) scratch[23:16] <= lw_wdata[23:16];
		if (lw_be[3]) scratch[31:24] <= lw_wdata[31:24];
	end

	case (ds)
		2'd0: if (!busy && pick_v) begin
			psel <= pick;
			rr   <= (pick == 3'd6) ? 3'd0 : pick + 3'd1;
			ds   <= 2'd1;
		end
		2'd1: begin
			if (psel == 3'd6) arp <= arp + 5'd1;
			else dirty[psel] <= 0;
			ds <= 2'd2;
		end
		default: ds <= 2'd0;   // the handshake takes the word in this cycle
	endcase

	// after the drain, so a store landing in the read cycle stays dirty (its RAM read returned the old word)
	if (st_d) dirty[wsl] <= 1;
	else if (st_a) begin
		if (afull) begin
			if (DIAG) adrop <= adrop + 32'd1;
		end
		else awp <= awp + 5'd1;
	end
end

// to clk_sys: the store's word, and its strobe the cycle after the capture
wire [47:0] pad_w = (psel == 3'd6) ? {pq[24] ? 8'h3D : 8'h1A, pq[23:16], 16'd0, pq[15:0]} : {op_of(psel), 8'd0, pq};

lw_xfer #(.W(48)) pad_x
(
	.s_clk(clk), .s_go(ds == 2'd2), .s_d(pad_w), .s_busy(busy),
	.d_clk(clk_sys), .d_q(lw_pad[47:0]), .d_v(lw_pad[48])
);

endmodule

`default_nettype wire
`endif
