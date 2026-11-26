//============================================================================
//  hps_ocram_master.sv -- part of the OCRAM mailbox (docs/OCRAM_MAILBOX.md)
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

`ifdef MISTER_OCRAM
`default_nettype none

// The contract is docs/OCRAM_MAILBOX.md; this file is layout 1 only.

`ifndef MISTER_OCRAM_POLL
`define MISTER_OCRAM_POLL 12
`endif

module hps_ocram_master #(
	// The SECTOR tier's declaration (ocram_cfg.vh maps the MISTER_OCRAM_SECTOR_* macros), for C_K[0..7]
	parameter [31:0] SEC_SLOT  = 32'd1,
	parameter [31:0] SEC_FRAME = 32'd128,
	parameter [31:0] SEC_GAP   = 32'd2,
	parameter  [2:0] SEC_BLKSZ = 3'd2,
	// The download tier's declaration (MISTER_OCRAM_FIO_RING, MISTER_OCRAM_FIO_GAP), for C_RING[1] and C_K[9]
	parameter [31:0] FIO_RING  = 32'd10,
	parameter [31:0] FIO_GAP   = 32'd4,
	// the selects an LFRAME may name, {osd_vga, osd_hdmi, fpga, uio}: ocram_cfg.vh's OCRAM_FIO_SELS, also the walker's
	parameter  [3:0] FIO_SELS  = 4'b0010,
	// The mount slots' declaration (MISTER_OCRAM_MOUNT0_SLOT, _MOUNT1_SLOT, _MOUNT_FRAME), for C_K[10, 11]
	parameter [31:0] MNT0_SLOT = 32'd2,
	parameter [31:0] MNT1_SLOT = 32'd3,
	parameter [31:0] MNT_FRAME = 32'd128
)
(
	input  wire        clk,          // clk_100m == h2f_user0_clk
	input  wire        hps_rst_n,    // the HPS's fabric reset (h2f_rst_n), asynchronous
	/* verilator lint_off UNUSEDSIGNAL */
	input  wire [31:0] gp_out,
	/* verilator lint_on UNUSEDSIGNAL */

	output wire        present,      // the constant 1: this floor is built
	output reg         line,         // EVT ring line ordinal 4 (f2h_irq[11]), a clk register

	// consumer port 0: the OSD walker (MISTER_OCRAM_OSD; tied off when unbuilt)
	input  wire        c0_req,
	input  wire        c0_we,
	input  wire [15:0] c0_off,
	input  wire [31:0] c0_wdata,
	output wire        c0_grant,
	output wire        c0_done,
	output wire        c0_ok,
	output wire [31:0] c0_rdata,
	output wire        s_hello,      // one cycle: the hello's first store is about to run
	output wire        s_live,       // P_SESSION
	output wire        s_tick,       // the poll period's pulse
	output wire        osd_bind,     // the OSD bind (H_BIND bit 11 read this session)
	input  wire        c0_rest,      // the OSD half's enable link is idle
	input  wire        c0_fault,     // one cycle: the OSD ring stopped on a fault
	input  wire        c0_mid,       // level: an OSD record is part-consumed
	input  wire        c0_open,      // the OSD enable link's OPEN
	/* verilator lint_off UNUSEDSIGNAL */
	input  wire [15:0] osd_rows,     // ROW records the OSD walker finished this session
	input  wire [15:0] osd_deepest,  // its deepest fill
	output wire  [2:0] sec_ask,      // H_SECTOR bits 2:0, read this session
	output wire        evt_stop,     // the EVT ring stopped on a fault
	input  wire        c0_sfault,    // one cycle: the SECTOR ring stopped on a fault
	input  wire        c0_smid,      // level: a SECTOR record is part-consumed
	input  wire  [2:0] c0_sbind,     // the C_BIND bits 2:0 to store
	input  wire        c0_wide,      // hps_io's io_wide, through a crossing cell (C_K[1])
	input  wire        rq_post,      // a REQUEST for rq_lba is owed
	input  wire [31:0] rq_lba,
	/* verilator lint_on UNUSEDSIGNAL */
	output wire        rq_start,     // one cycle: that REQUEST's header store is launched
	/* verilator lint_off UNUSEDSIGNAL */   // the mount slots' (MISTER_OCRAM_MOUNT0_SLOT, _MOUNT1_SLOT)
	input  wire        c0_mfault,    // one cycle: the mount ring stopped on a fault
	input  wire        c0_mmid,      // level: a mount record is part-consumed
	input  wire        m_post,       // a mount slot's REQUEST or WRITE is owed
	input  wire        m_write,      // it is a WRITE
	input  wire  [7:0] m_arg,        // {blk_cnt, k}
	input  wire [10:0] m_len,        // its len: 1 for a REQUEST, 1 + the block's words for a WRITE
	input  wire [31:0] m_lba,
	input  wire        m_wv,         // a WRITE's next word is in m_wd
	input  wire [31:0] m_wd,
	/* verilator lint_on UNUSEDSIGNAL */
	output wire        m_start,      // one cycle: that record's header store is launched
	output wire        m_wtake,      // one cycle: a WRITE word's store is launched
	output wire        m_pub,        // one cycle: the WRITE's head store is done
	/* verilator lint_off UNUSEDSIGNAL */   // the download tier's (MISTER_OCRAM_FIO)
	input  wire [15:0] f_tail,       // the download ring's tail, as last stored in C_IDX[1]
	input  wire        c0_ffault,    // one cycle: the download ring stopped on a fault
	input  wire        c0_fmid,      // level: a download record is part-consumed
	/* verilator lint_on UNUSEDSIGNAL */
	output reg         mark_line,    // the download ring's mark line, ordinal 5 (f2h_irq[12]), a clk register
	// the events tier's port (MISTER_OCRAM_EVT, hps_ocram_evt below): ask for the next offer, take it
	output wire        e_ask,        // one cycle: the master asks for hps_io's next offer
	/* verilator lint_off UNUSEDSIGNAL */   // without MISTER_OCRAM_EVT the offers are not read
	input  wire        e_v,          // one cycle: an offer is in e_d
	input  wire [41:0] e_d,          // {kind[1:0], arg[7:0], data[31:0]}, held until the next
	/* verilator lint_on UNUSEDSIGNAL */
	// the capture lane's REPLY (MISTER_OCRAM_LANE): hps_ocram_rx's capture frame and its replies
	output wire [15:0] e_free,       // the EVT ring's free words
	/* verilator lint_off UNUSEDSIGNAL */   // without MISTER_OCRAM_LANE the capture lane is not read
	input  wire        p_post,       // a capture frame is held for its REPLY record
	input  wire  [7:0] p_arg,        // the frame's arg
	input  wire [10:0] p_n,          // its len: the REPLY's payload words
	input  wire        p_wv,         // a payload word is in p_wd
	input  wire [31:0] p_wd,
	/* verilator lint_on UNUSEDSIGNAL */
	output wire        p_start,      // one cycle: the REPLY's header is written; the frame may play
	output wire        p_wtake,      // one cycle: the master takes p_wd

	// fpga2hps AXI3 master, 32-bit data
	output wire  [7:0] awid,
	output wire [31:0] awaddr,
	output wire  [3:0] awlen,
	output wire  [2:0] awsize,
	output wire  [1:0] awburst,
	output wire  [1:0] awlock,
	output wire  [3:0] awcache,
	output wire  [2:0] awprot,
	output wire  [4:0] awuser,
	output wire        awvalid,
	input  wire        awready,
	output wire  [7:0] wid,
	output wire [31:0] wdata,
	output wire  [3:0] wstrb,
	output wire        wlast,
	output wire        wvalid,
	input  wire        wready,
	/* verilator lint_off UNUSEDSIGNAL */   // single-beat, one outstanding: the floor never needs a response ID
	input  wire  [7:0] bid,
	/* verilator lint_on UNUSEDSIGNAL */
	input  wire  [1:0] bresp,
	input  wire        bvalid,
	output wire        bready,
	output wire  [7:0] arid,
	output wire [31:0] araddr,
	output wire  [3:0] arlen,
	output wire  [2:0] arsize,
	output wire  [1:0] arburst,
	output wire  [1:0] arlock,
	output wire  [3:0] arcache,
	output wire  [2:0] arprot,
	output wire  [4:0] aruser,
	output wire        arvalid,
	input  wire        arready,
	/* verilator lint_off UNUSEDSIGNAL */   // single-beat, one outstanding: the floor never needs a response ID
	input  wire  [7:0] rid,
	/* verilator lint_on UNUSEDSIGNAL */
	input  wire [31:0] rdata,
	input  wire  [1:0] rresp,
	/* verilator lint_off UNUSEDSIGNAL */   // a single-beat read: RLAST is implied and never read
	input  wire        rlast,
	/* verilator lint_on UNUSEDSIGNAL */
	input  wire        rvalid,
	output wire        rready
);

///////////////////////////////  the map  ///////////////////////////////////

// The tiers' enables, the OSD rows and C_CAPS, from the .qsf macros.
`include "sys/ocram_cfg.vh"

localparam [15:0] H_HELLO = 16'h0000;      // host: {0x4F43, nonce}, 0 = no session
localparam [15:0] H_BIND  = 16'h0004;      // host: bit 11 = the OSD bind
localparam [15:0] H_LINE  = 16'h0008;      // host: bit 3 lets the EVT line rise
localparam [15:0] H_SECTOR = 16'h000C;     // host: bit 0 asks for the SECTOR slot
localparam [15:0] H_IDX3  = 16'h001C;      // host: the EVT ring's tail
localparam [15:0] H_MARK1 = 16'h0034;      // host: the download ring's mark, {arm[31], 0, index[15:0]}
localparam [15:0] C_IDX0  = 16'h0090;      // fabric: the mount ring's tail
localparam [15:0] C_RING0 = 16'h00B0;      // fabric: the mount ring's descriptor
localparam [15:0] C_K10   = 16'h00F8;      // fabric: the mount slots' {RD, slot, frame words}
localparam [15:0] C_K11   = 16'h00FC;
localparam [15:0] C_HELLO = 16'h0080;      // fabric: the master's state
localparam [15:0] C_VERSION = 16'h0084;    // fabric: {major, minor} = 1.0
localparam [15:0] C_CAPS  = 16'h0088;
localparam [15:0] C_STATE = 16'h008C;      // fabric: hello count, fault and previous-session-mid bits
localparam [15:0] C_IDX1  = 16'h0094;      // fabric: the download ring's tail
localparam [15:0] C_IDX2  = 16'h0098;      // fabric: the OSD ring's tail
localparam [15:0] C_IDX3  = 16'h009C;      // fabric: the EVT ring's head
localparam [15:0] C_IDX4  = 16'h00A0;
localparam [15:0] C_BIND  = 16'h00A4;      // fabric: bit 11 = the OSD bind is in effect
localparam [15:0] C_RING1 = 16'h00B4;      // fabric: the download ring's descriptor
localparam [15:0] C_RING2 = 16'h00B8;      // fabric: the OSD ring's descriptor
localparam [15:0] C_RING3 = 16'h00BC;      // fabric: the EVT ring's descriptor
localparam [15:0] C_RING4 = 16'h00C0;
localparam [15:0] C_K0    = 16'h00D0;      // fabric: the SECTOR block's K0, K1, K2, K4, K5
localparam [15:0] C_K1    = 16'h00D4;
localparam [15:0] C_K2    = 16'h00D8;
localparam [15:0] C_K4    = 16'h00E0;
localparam [15:0] C_K5    = 16'h00E4;
localparam [15:0] C_K8    = 16'h00F0;      // fabric: OSDBUF's K0, {rows, row bytes}
localparam [15:0] C_K9    = 16'h00F4;      // fabric: the download player's {sel mask, gap}
localparam [15:0] EVT_BASE = 16'h4000;     // fabric: EVT word k at EVT_BASE + ((head + k) & 2047) * 4
localparam [15:0] EVT_WORDS = 16'd2048;

localparam [31:0] C_HELLO_ARMED = 32'h4F43_0000;
localparam [31:0] C_HELLO_WAIT  = 32'h5754_0000;
localparam [31:0] C_HELLO_STOP  = 32'h5154_0000;
localparam [15:0] HELLO_MAGIC   = 16'h4F43;
localparam [31:0] C_VERSION_1_0 = 32'h0001_0000;
localparam [31:0] C_RING2_OSD   = {5'd3, 5'd0, 4'd11, 4'd0, 14'h0800};    // OSD, no line, 2048 words, 0x2000
localparam [31:0] C_RING3_EVT   = {5'd4, 5'd4, 4'd11, 4'd0, 14'h1000};    // EVT, line 4, 2048 words, 0x4000
/* verilator lint_off UNUSEDPARAM */
localparam [31:0] C_RING4_TS    = {5'd31, 5'd0, 4'd13, 4'd0, 14'h1C00};
localparam [31:0] C_RING4_SEC   = {5'd2, 5'd0, 4'd13, 4'd0, 14'h1C00};    // SECTOR, no line, 8192 words, 0x7000
// LANE, line 5, 2^FIO_RING words at 0x1000 (8,192 at 0x7000); the select mask (the FIO_SELS parameter, the one
// hps_ocram_rx plays by) and the gap.
localparam [13:0] FIO_OFF       = (FIO_RING == 32'd13) ? 14'h1C00 : 14'h0400;
localparam [31:0] C_RING1_FIO   = {5'd5, 5'd5, FIO_RING[3:0], 4'd0, FIO_OFF};
localparam [31:0] C_K9_FIO      = {12'd0, FIO_SELS, FIO_GAP[15:0]};
localparam [31:0] C_K0_SEC      = {24'd0, SEC_GAP[7:0]};                 // {FIFO entries 0, 0, strobe gap}
localparam [31:0] C_K4_SEC      = {8'h01, 4'd0, SEC_SLOT[3:0], 3'd0, SEC_FRAME[12:0]};   // {RD, slot, frame words}
localparam [31:0] REQ_HEADER    = {8'h21, 8'd0, 16'd1};                  // kind REQUEST, arg 0, len 1
// the mount ring: SECTOR, no line, 1,024 words at 0x6000
localparam [31:0] C_RING0_MNT   = {5'd2, 5'd0, 4'd10, 4'd0, 14'h1800};
localparam [31:0] C_K8_OSD      = {OCRAM_OSD_ROWS[15:0], 16'd256};
localparam [31:0] MARK_HEADER   = {8'h23, 8'h03, 16'd40};                // kind MARK, arg 3, len 40
localparam [31:0] VERDICT_HEADER= {8'h25, 8'd4, 16'd8};                  // kind VERDICT, arg 4, len 8
/* verilator lint_on UNUSEDPARAM */
localparam integer POLL = `MISTER_OCRAM_POLL;

localparam OSD_EN  = OCRAM_OSD_EN;
localparam TS_EN   = OCRAM_TS_EN;
localparam SEC_EN  = OCRAM_SEC_EN;
localparam FIO_EN  = OCRAM_FIO_EN;
localparam MNT0_EN = OCRAM_MNT0_EN;
localparam MNT1_EN = OCRAM_MNT1_EN;
localparam MNT_EN = MNT0_EN != 0 || MNT1_EN != 0;
localparam [2:0] SEC_MASK = {MNT1_EN != 0, MNT0_EN != 0, 1'b1};    // the H_SECTOR bits this build serves
// a mount slot's K word {RD, slot, frame words}, 0 if undeclared
/* verilator lint_off UNUSEDPARAM */   // a mount build's
localparam [31:0] C_K10_MNT     = MNT0_EN ? {8'h01, 4'd0, MNT0_SLOT[3:0], 3'd0, MNT_FRAME[12:0]} : 32'd0;
localparam [31:0] C_K11_MNT     = MNT1_EN ? {8'h01, 4'd0, MNT1_SLOT[3:0], 3'd0, MNT_FRAME[12:0]} : 32'd0;
/* verilator lint_on UNUSEDPARAM */
localparam RX_EN = OSD_EN != 0 || SEC_EN != 0 || FIO_EN != 0;     // hps_ocram_rx takes port 0

// A static assertion: an address that could leave the mailbox stops elaboration.
generate
	if (EVT_BASE + EVT_WORDS * 4 > 16'hF000) begin : g_evt_span
		ocram_evt_span_above_0xF000 guard();
	end
	if (H_HELLO >= 16'h0100 || H_BIND >= 16'h0100 || H_LINE >= 16'h0100 || H_SECTOR >= 16'h0100 ||
	    H_IDX3 >= 16'h0100 || H_MARK1 >= 16'h0100 || C_IDX1 >= 16'h0100 || C_RING1 >= 16'h0100 ||
	    C_K9 >= 16'h0100 || C_K0 >= 16'h0100 || C_K1 >= 16'h0100 || C_K2 >= 16'h0100 ||
	    C_K4 >= 16'h0100 || C_K5 >= 16'h0100 ||
	    C_HELLO >= 16'h0100 || C_VERSION >= 16'h0100 || C_CAPS >= 16'h0100 ||
	    C_STATE >= 16'h0100 || C_IDX2 >= 16'h0100 || C_IDX3 >= 16'h0100 || C_IDX4 >= 16'h0100 ||
	    C_BIND >= 16'h0100 || C_RING2 >= 16'h0100 || C_RING3 >= 16'h0100 || C_RING4 >= 16'h0100 ||
	    C_K8 >= 16'h0100 || C_IDX0 >= 16'h0100 || C_RING0 >= 16'h0100 || C_K10 >= 16'h0100 ||
	    C_K11 >= 16'h0100) begin : g_ctrl_span
		ocram_control_constant_above_0xFF guard();
	end
	// the mount slots: hps_io slots 0-15, none the disc slot's or the other's; a frame of one hps_io block
	// (32 << BLKSZ words) to 1,020 words
	if ((MNT0_EN != 0 && (MNT0_SLOT > 15 || MNT0_SLOT == SEC_SLOT)) ||
	    (MNT1_EN != 0 && (MNT1_SLOT > 15 || MNT1_SLOT == SEC_SLOT)) ||
	    (MNT0_EN != 0 && MNT1_EN != 0 && MNT0_SLOT == MNT1_SLOT) ||
	    (MNT_EN && (MNT_FRAME < (32'd32 << SEC_BLKSZ) || MNT_FRAME > 1020))) begin : g_mnt_geometry
		ocram_mount_unservable_geometry guard();
	end
endgenerate

// The capture lane's frames are the download tier's.
`ifdef MISTER_OCRAM_LANE
`ifndef MISTER_OCRAM_FIO
ocram_lane_needs_MISTER_OCRAM_FIO lane_fio_guard();
`endif
`endif

assign present = 1'b1;

////////////////////////////  clock-domain entry  ///////////////////////////

// Every GPO bit and hps_rst_n enters only through an lw_arst cell.
wire idv_q, rn_q, hrst_q;
lw_arst idv_c (.clk(clk), .rst_n(gp_out[31]), .q(idv_q));
lw_arst rn_c  (.clk(clk), .rst_n(gp_out[28]), .q(rn_q));
lw_arst hrst_c(.clk(clk), .rst_n(hps_rst_n), .q(hrst_q));

reg rn_d = 1'b1;                    // lw_arst's q is high from configuration
always @(posedge clk) rn_d <= rn_q;
wire run_rise = rn_d & ~rn_q;       // a fall of lw_arst's q: RUN's 0 store was seen
wire run_set  = run_rise & ~idv_q & ~hrst_q;

reg run = 1'b0;
always @(posedge clk) begin
	if (rn_q | idv_q | hrst_q) run <= 1'b0;   // the clear wins
	else if (run_rise) run <= 1'b1;
end

reg hrst_d = 1'b1;
always @(posedge clk) hrst_d <= hrst_q;
wire hrst_rise = hrst_q & ~hrst_d;
wire hrst_fall = ~hrst_q & hrst_d;

reg run_d = 1'b0;
always @(posedge clk) run_d <= run;
wire run_fall = run_d & ~run;

wire ns = 1'b0;

/////////////////////////////////  the bus  /////////////////////////////////

// ocram_axi.sv is the engine; ocram_arb.sv (below, after the requesters) picks its commands.
wire        engine_idle, t_done, t_ok;
wire        t_go, t_we, t_counted;
wire [15:0] t_off;
wire [31:0] t_wdata;
/* verilator lint_off UNUSEDSIGNAL */
wire        counted_q, drop_q;
/* verilator lint_on UNUSEDSIGNAL */

ocram_axi axi
(
	.clk(clk), .hrst_q(hrst_q), .hrst_fall(hrst_fall), .ns(ns),
	.t_go(t_go), .t_we(t_we), .t_off(t_off), .t_wdata(t_wdata), .t_counted(t_counted),
	.engine_idle(engine_idle), .t_done(t_done), .t_ok(t_ok), .t_counted_q(counted_q), .t_drop(drop_q),
	.awid(awid), .awaddr(awaddr), .awlen(awlen), .awsize(awsize), .awburst(awburst), .awlock(awlock),
	.awcache(awcache), .awprot(awprot), .awuser(awuser), .awvalid(awvalid), .awready(awready),
	.wid(wid), .wdata(wdata), .wstrb(wstrb), .wlast(wlast), .wvalid(wvalid), .wready(wready),
	.bresp(bresp), .bvalid(bvalid), .bready(bready),
	.arid(arid), .araddr(araddr), .arlen(arlen), .arsize(arsize), .arburst(arburst), .arlock(arlock),
	.arcache(arcache), .arprot(arprot), .aruser(aruser), .arvalid(arvalid), .arready(arready),
	.rresp(rresp), .rvalid(rvalid), .rready(rready)
);

///////////////////////////////  the run  ///////////////////////////////////

// A run's phase. P_REST is the hello's wait for every consumer's clk_sys half; P_HELLO covers the hello's
// stores, which run in order and in one piece; P_SESSION is a session with the hello echoed.
localparam [2:0] P_OFF = 3'd0, P_START = 3'd1, P_ARMED = 3'd2, P_WAIT = 3'd3,
                 P_SESSION = 3'd4, P_HELLO = 3'd5, P_REST = 3'd6;

localparam [2:0] R_NONE = 3'd0, R_HELLO = 3'd1, R_LINE = 3'd2, R_IDX = 3'd3, R_BIND = 3'd4, R_SECT = 3'd5,
                 R_MARK = 3'd6;
localparam [2:0] W_NONE = 3'd0, W_HELLO = 3'd1, W_ONE = 3'd2, W_TERM = 3'd5, W_FAULT = 3'd6, W_BIND = 3'd7;

reg  [2:0]  phase = P_OFF;
reg  [7:0]  hello_cnt = 8'd0;          // hellos taken since configuration, no reset; counts each echo
reg  [31:0] echo_q = 32'd0;            // the H_HELLO value this session echoed
reg         line_en_q = 1'b0;          // H_LINE bit 3 at its last read
reg         stopped_q = 1'b0;          // the EVT ring stopped on a fault this session
reg         prev_mid_q = 1'b0;         // the EVT record of the session before this one was part-written
reg         osd_bind_q = 1'b0;         // H_BIND bit 11 read: the OSD bind asked for this session
reg  [3:0]  bind_q = 4'd0;             // the C_BIND bits {11, 2:0} last stored
`ifdef MISTER_OCRAM_EVT
reg         evt_bind_q = 1'b0;         // H_BIND bit 8 read: the events bind asked for this session
reg         evt_bound_q = 1'b0;        // C_BIND bit 8 as last stored
`endif
reg  [2:0]  sec_ask_q = 3'd0;          // H_SECTOR bits 2:0 read this session (those this build serves)
reg         bind_pend = 1'b0;          // a C_BIND store is owed
reg         osd_stopped_q = 1'b0;      // the OSD ring stopped on a fault this session
reg         ts_stopped_q = 1'b0;
reg         prev_mid_osd_q = 1'b0;     // the OSD record of the previous session was part-consumed
reg         prev_mid_ts_q = 1'b0;
reg         fio_stopped_q = 1'b0;      // the download ring stopped on a fault this session
reg         prev_mid_fio_q = 1'b0;     // the download record of the previous session was part-consumed
reg         mnt_stopped_q = 1'b0;      // the mount ring stopped on a fault this session
reg         prev_mid_mnt_q = 1'b0;     // the mount record of the previous session was part-consumed
reg         mark_arm_q = 1'b0;         // H_MARK[1] bit 31 at its last read this session
reg  [15:0] mark_idx_q = 16'd0;        // H_MARK[1][15:0] at its last read
reg  [2:0]  poll_step = 3'd0;          // session poll: 1 H_BIND, 5 H_SECTOR, 6 H_MARK[1], 2 H_LINE, 3 H_IDX, then
                                       // 4 H_IDX again while the EVT line is high, 7 H_MARK[1] while the mark line is
reg         poll_pend = 1'b0;          // a poll period has passed
reg         boot_pend = 1'b0;          // the run's first H_HELLO read is not taken yet
reg         boot_first = 1'b1;         // it is tried at once; a failed one waits for the next poll
reg         term_pend = 1'b0;          // the terminal store of a stopped run is owed
reg         sto_pend = 1'b0;           // one C_HELLO state store is owed
reg  [31:0] sto_data = 32'd0;
reg         fault_pend = 1'b0;         // C_STATE's fault bits are owed "at once"
reg         hj_act = 1'b0;             // the hello's stores
reg  [4:0]  hj_k = 5'd0;
reg  [31:0] hj_state = 32'd0;          // C_STATE as stored at this hello
reg  [2:0]  rd_act = R_NONE;           // the read in flight
reg  [2:0]  wr_act = W_NONE;           // the write in flight (the record writer keeps its own)
localparam [POLL-1:0] POLL_T0 = {POLL{1'b0}};
localparam [POLL-1:0] POLL_ONE = 1;
reg  [POLL-1:0] poll_t = {POLL{1'b0}}; // free-running; its period is 2^POLL cycles
wire poll_tick = (poll_t == POLL_T0);

always @(posedge clk) poll_t <= run_set ? POLL_T0 : poll_t + POLL_ONE;   // a run's polls count from its start

wire run_ended = hrst_rise | run_fall;

// The EVT ring's writer's state, for the session (ocram_evt_wr below)
wire [15:0] w_head, w_tail;            // the ring's head, last published, and its tail, last read
/* verilator lint_off UNUSEDSIGNAL */
wire [15:0] w_head_nx, w_free;         // the open record's head store value; 2,048 - (head - tail)
/* verilator lint_on UNUSEDSIGNAL */
wire        w_act;                     // a record is open: it is part-written until its head store
/* verilator lint_off UNUSEDSIGNAL */   // the record sources' (and the bench's rec_k)
wire [10:0] rec_k;                     // its next store: 0 header, 1..len payload, len + 1 head
wire        w_first;                   // rec_k is 1
wire  [2:0] w_src;
wire        w_wfly;                    // a header or payload store of it is in flight
/* verilator lint_on UNUSEDSIGNAL */
wire        w_hfly;                    // its head store is in flight

// The consumers' session returns and the hello's wait for their rest.
wire c1_rest, c1_fault, c1_mid;
wire s_live_w = (phase == P_SESSION);
wire consumer_rest = (!RX_EN || c0_rest) && (!TS_EN || c1_rest);
wire hello_start = (phase == P_REST) && consumer_rest && !run_ended;
assign s_live   = s_live_w;
assign s_tick   = poll_tick;
assign s_hello  = hello_start;
assign osd_bind = osd_bind_q;
assign sec_ask  = sec_ask_q;
assign evt_stop = stopped_q;

// C_STATE: [7:0] hellos taken, [12:8] ring r stopped on a fault this session, [20:16] ring r ended the previous
// session mid-record; every other bit 0.
function [31:0] state_word(input [7:0] cnt, input fe, input fo, input ft, input ff, input fm, input me, input mo,
                           input mt, input mf, input mm);
	begin
		state_word = {24'd0, cnt};
		if (me) state_word[19] = 1'b1;
		if (mo) state_word[18] = 1'b1;
		if (mt) state_word[20] = 1'b1;
		if (mf) state_word[17] = 1'b1;
		if (mm) state_word[16] = 1'b1;
		if (fe) state_word[11] = 1'b1;
		if (fo) state_word[10] = 1'b1;
		if (ft) state_word[12] = 1'b1;
		if (ff) state_word[9] = 1'b1;
		if (fm) state_word[8] = 1'b1;
	end
endfunction

// The download ring's tail has reached a mark index: it is at or past it, modulo 2^16, within half the index space.
function mark_reached(input [15:0] tail, input [15:0] idx);
	mark_reached = ((tail - idx) & 16'h8000) == 16'd0;
endfunction

// The hello's stores, in their fixed order, the last C_HELLO (HJ_LAST).
localparam [4:0] HJ_LAST = 5'd23;
function hj_has(input [4:0] k);
	begin
		hj_has = (k <= 5'd23);
`ifndef MISTER_OCRAM_OSD
		if (k == 5'd3 || k == 5'd7 || k == 5'd10) hj_has = 1'b0;
`endif
`ifndef MISTER_OCRAM_SECTOR_SLOT
		if (k == 5'd5 || k == 5'd9) hj_has = 1'b0;
`endif
`ifndef MISTER_OCRAM_SECTOR_SLOT
		if (k >= 5'd11 && k <= 5'd15) hj_has = 1'b0;
`endif
`ifndef MISTER_OCRAM_FIO
		if (k >= 5'd16 && k <= 5'd18) hj_has = 1'b0;
`endif
`ifndef MISTER_OCRAM_MOUNT0_SLOT
`ifndef MISTER_OCRAM_MOUNT1_SLOT
		if (k >= 5'd19 && k <= 5'd22) hj_has = 1'b0;
`endif
`endif
	end
endfunction

// The step after k. Without the download tier its three steps take no cycle, nor without the mount slots their four:
// C_HELLO follows the last step built at once.
function [4:0] hj_next(input [4:0] k);
	begin
		hj_next = k + 5'd1;
`ifndef MISTER_OCRAM_FIO
		if (k == 5'd15) hj_next = 5'd19;
`endif
`ifndef MISTER_OCRAM_MOUNT0_SLOT
`ifndef MISTER_OCRAM_MOUNT1_SLOT
		if (hj_next == 5'd19) hj_next = 5'd23;
`endif
`endif
	end
endfunction

function [15:0] hj_off(input [4:0] k);
	begin
		case (k)
		5'd0: hj_off = C_VERSION;
		5'd1: hj_off = C_CAPS;
		5'd2: hj_off = C_STATE;
		5'd3: hj_off = C_IDX2;
		5'd4: hj_off = C_IDX3;
		5'd5: hj_off = C_IDX4;
		5'd6: hj_off = C_BIND;
		5'd7: hj_off = C_RING2;
		5'd8: hj_off = C_RING3;
		5'd9: hj_off = C_RING4;
		5'd10: hj_off = C_K8;
		5'd11: hj_off = C_K0;
		5'd12: hj_off = C_K1;
		5'd13: hj_off = C_K2;
		5'd14: hj_off = C_K4;
		5'd15: hj_off = C_K5;
		5'd16: hj_off = C_IDX1;
		5'd17: hj_off = C_RING1;
		5'd18: hj_off = C_K9;
		5'd19: hj_off = C_IDX0;
		5'd20: hj_off = C_RING0;
		5'd21: hj_off = C_K10;
		5'd22: hj_off = C_K11;
		default: hj_off = C_HELLO;
		endcase
	end
endfunction

function [31:0] hello_data(input [4:0] k, input [31:0] state, input [31:0] echo, input wide);
	begin
		case (k)
		5'd0: hello_data = C_VERSION_1_0;
		5'd1: hello_data = OCRAM_C_CAPS;
		5'd2: hello_data = state;
		5'd3: hello_data = 32'd0;
		5'd4: hello_data = 32'd0;
		5'd5: hello_data = 32'd0;
		5'd6: hello_data = 32'd0;
		5'd7: hello_data = C_RING2_OSD;
		5'd8: hello_data = C_RING3_EVT;
`ifdef MISTER_OCRAM_SECTOR_SLOT
		5'd9: hello_data = C_RING4_SEC;
`else
		5'd9: hello_data = C_RING4_TS;
`endif
		5'd10: hello_data = C_K8_OSD;
		5'd11: hello_data = C_K0_SEC;
		5'd12: hello_data = {21'd0, SEC_BLKSZ, 7'd0, wide};         // {BLKSZ[10:8], wide[0]}
		5'd13: hello_data = 32'd1;                                   // one descriptor
		5'd14: hello_data = C_K4_SEC;
		5'd15: hello_data = 32'd32;                                  // LBA bits
		5'd16: hello_data = 32'd0;
		5'd17: hello_data = C_RING1_FIO;
		5'd18: hello_data = C_K9_FIO;
		5'd19: hello_data = 32'd0;
		5'd20: hello_data = C_RING0_MNT;
		5'd21: hello_data = C_K10_MNT;
		5'd22: hello_data = C_K11_MNT;
		default: hello_data = echo;
		endcase
	end
endfunction

// The hello the read found: a valid H_HELLO in ARMED moves to the consumers' rest wait.
wire hello_hit = t_done && (rd_act == R_HELLO) && t_ok && (phase == P_ARMED) &&
                 (rdata[31:16] == HELLO_MAGIC) && (rdata[15:0] != 16'd0);
/* verilator lint_off UNUSEDSIGNAL */
wire diag_clr = hello_start;
/* verilator lint_on UNUSEDSIGNAL */

// The read the poll (or the boot, or the line) wants next, after the consumers' declarations below: a step-4 read
// is the EVT line's back-to-back tail re-read and a step-7 read the mark line's H_MARK[1] re-read; each yields to a
// requesting consumer (line_yield_q).
reg  poll_sel;
reg  [2:0] poll_rd_kind;

//////////////////////////////  the consumers  //////////////////////////////

// An unbuilt port's request is the constant 0.
wire c0_req_v = RX_EN && c0_req;
wire  [1:0] a_cdone;                   // the consumer ports' completions
wire [31:0] a_crdata;
wire        c1_we = 1'b0;
wire [15:0] c1_off = 16'd0;
wire [31:0] c1_wdata = 32'd0;
wire c1_req_v = 1'b0;
assign c1_rest = 1'b1;                 // the unbuilt port is always at rest
assign c1_fault = 1'b0;
assign c1_mid = 1'b0;

wire consumer_req = c0_req_v | c1_req_v;   // a consumer is waiting: the line's re-read gives it a turn
reg  line_yield_q = 1'b0;                  // a step-4 re-read ran and the consumers have not had their turn

always @(*) begin
	poll_sel = 1'b0; poll_rd_kind = R_NONE;
	if (phase == P_START) begin
		if (boot_pend && (boot_first || poll_pend)) begin
			poll_sel = 1'b1; poll_rd_kind = R_HELLO;
		end
	end
	else if (phase == P_ARMED || phase == P_WAIT) begin
		if (poll_pend) begin poll_sel = 1'b1; poll_rd_kind = R_HELLO; end
	end
	else if (phase == P_SESSION) begin
		if (poll_pend) begin
			poll_sel = 1'b1; poll_rd_kind = R_HELLO;
		end
		else if (poll_step == 3'd1) begin
			poll_sel = 1'b1; poll_rd_kind = R_BIND;
		end
		else if (poll_step == 3'd5) begin
			poll_sel = 1'b1; poll_rd_kind = R_SECT;
		end
		else if (poll_step == 3'd6) begin
			poll_sel = 1'b1; poll_rd_kind = R_MARK;
		end
		else if (poll_step == 3'd2) begin
			poll_sel = 1'b1; poll_rd_kind = R_LINE;
		end
		else if (poll_step == 3'd3) begin
			poll_sel = 1'b1; poll_rd_kind = R_IDX;
		end
		else if (poll_step == 3'd4 && !(line_yield_q && consumer_req)) begin
			poll_sel = 1'b1; poll_rd_kind = R_IDX;      // at most one re-read between two consumer grants
		end
		else if (poll_step == 3'd7 && !(line_yield_q && consumer_req)) begin
			poll_sel = 1'b1; poll_rd_kind = R_MARK;     // the mark line's, by the same rule
		end
	end
end

// the poll's read, a table of the kind
reg  [15:0] poll_off;
always @(*) begin
	case (poll_rd_kind)
	R_LINE:  poll_off = H_LINE;
	R_IDX:   poll_off = H_IDX3;
	R_BIND:  poll_off = H_BIND;
	R_SECT:  poll_off = H_SECTOR;
	R_MARK:  poll_off = H_MARK1;
	default: poll_off = H_HELLO;
	endcase
end

// the arbiter's ports, in its order: the terminal store, the fault's C_STATE store, the poll's or the line's reads
// when due, the hello's stores, a state store, a C_BIND store, the EVT ring's writer, then the two consumers
localparam integer NP = 7;
localparam [3:0] J_NONE = 4'd0, J_TERM = 4'd1, J_FAULT = 4'd2, J_POLL = 4'd3, J_HELLO = 4'd4,
                 J_STO = 4'd5, J_BIND = 4'd6, J_REC = 4'd7, J_C0 = 4'd10, J_C1 = 4'd11;
wire [NP+1:0] gnt;
reg   [3:0] t_job;                     // the job the arbiter chose this cycle, J_NONE if none
always @(*) begin
	t_job = J_NONE;
	if (gnt[0]) t_job = J_TERM;
	if (gnt[1]) t_job = J_FAULT;
	if (gnt[2]) t_job = J_POLL;
	if (gnt[3]) t_job = J_HELLO;
	if (gnt[4]) t_job = J_STO;
	if (gnt[5]) t_job = J_BIND;
	if (gnt[6]) t_job = J_REC;
	if (gnt[7]) t_job = J_C0;
	if (gnt[8]) t_job = J_C1;
end
wire c0_go = (t_job == J_C0);
/* verilator lint_off UNUSEDSIGNAL */
wire c1_go = (t_job == J_C1);
/* verilator lint_on UNUSEDSIGNAL */

assign c0_grant = c0_go;
assign c0_done  = a_cdone[0];
assign c0_ok    = t_ok;
assign c0_rdata = a_crdata;
/* verilator lint_off UNUSEDSIGNAL */
wire a_c1done = a_cdone[1];
/* verilator lint_on UNUSEDSIGNAL */

/////////////////////////////  the EVT ring  ////////////////////////////////

// ocram_evt_wr.sv writes the ring's records.
localparam integer NS = 7;
/* verilator lint_off UNUSEDPARAM */   // each build names its own sources
localparam [2:0] S_BIND = 3'd0, S_DISC = 3'd1, S_MNT = 3'd2, S_REP = 3'd3, S_EV = 3'd4, S_MARK = 3'd5, S_VD = 3'd6;
/* verilator lint_on UNUSEDPARAM */
/* verilator lint_off UNUSEDSIGNAL */   // each build reads its own sources'
wire [NS-1:0]   w_start, w_wtake, w_pub;
/* verilator lint_on UNUSEDSIGNAL */
// the first payload word of a mount slot's record (its LBA) or the events tier's, as it was at the record's start
/* verilator lint_off UNUSEDSIGNAL */   // a mount build's, an events build's
reg  [31:0] rec_lba_q = 32'd0;
/* verilator lint_on UNUSEDSIGNAL */

`ifdef MISTER_OCRAM_SECTOR_SLOT
// The SECTOR tier's sources.
reg         brec_pend = 1'b0;          // the BIND record after a C_BIND store is owed
reg         rec_mw_q = 1'b0;           // the mount record open is a WRITE
/* verilator lint_off UNUSEDSIGNAL */   // the REPLY's and the events tier's
wire        brec_owed = brec_pend;
/* verilator lint_on UNUSEDSIGNAL */
wire        b_post = brec_pend;
wire [31:0] b_hdr  = {8'h27, 5'd0, bind_q[2:0], 16'd0};
wire        d_post = rq_post && c0_sbind[0] && bind_q[0] && !bind_pend && !brec_pend;
wire [31:0] d_hdr  = REQ_HEADER;
wire [31:0] d_wd   = rq_lba;           // the disc REQUEST's LBA is hps_ocram_rx's request register
wire        m_bound = (m_arg[1:0] == 2'd2) ? (c0_sbind[2] && bind_q[2]) : (c0_sbind[1] && bind_q[1]);
wire        n_post = MNT_EN && m_post && m_bound && !bind_pend && !brec_pend;
wire [31:0] n_hdr  = {m_write ? 8'h26 : 8'h21, m_arg, 5'd0, m_len};
wire        n_wv   = w_first || m_wv;  // a WRITE word waits for hps_ocram_rx's
wire [31:0] n_wd   = w_first ? rec_lba_q : m_wd;
wire        bind_hold = w_act || (brec_pend && !stopped_q);
assign rq_start = w_start[S_DISC];
assign m_start  = w_start[S_MNT];
assign m_wtake  = w_wtake[S_MNT] && rec_mw_q && !w_first;
assign m_pub    = w_pub[S_MNT] && rec_mw_q;
`else
/* verilator lint_off UNUSEDSIGNAL */   // the REPLY's and the events tier's
wire        brec_owed = 1'b0;
/* verilator lint_on UNUSEDSIGNAL */
wire        b_post = 1'b0, d_post = 1'b0, n_post = 1'b0, n_wv = 1'b0;
wire [31:0] b_hdr = 32'd0, d_hdr = 32'd0, d_wd = 32'd0, n_hdr = 32'd0, n_wd = 32'd0;
wire        bind_hold = 1'b0;
assign rq_start = 1'b0;
assign m_start = 1'b0, m_wtake = 1'b0, m_pub = 1'b0;
`endif

`ifdef MISTER_OCRAM_LANE
// The capture lane's REPLY {0x22, arg, n}: each payload word as the walker hands it over. It opens behind an owed
// BIND, REQUEST and WRITE, with room for its n words and its header.
wire        r_post = p_post && !bind_pend && !brec_owed;
wire [31:0] r_hdr  = {8'h22, p_arg, 5'd0, p_n};
wire        r_wv   = p_wv;
wire [31:0] r_wd   = p_wd;
assign p_start  = w_start[S_REP];
assign p_wtake  = w_wtake[S_REP];
assign e_free   = w_free;
`else
wire        r_post = 1'b0, r_wv = 1'b0;
wire [31:0] r_hdr = 32'd0, r_wd = 32'd0;
assign p_start = 1'b0, p_wtake = 1'b0, e_free = 16'd0;
`endif

`ifdef MISTER_OCRAM_EVT
// The events tier's records.
localparam [1:0] EK_STATUS = 2'd0, EK_INFO = 2'd1, EK_UPLOAD = 2'd2, EK_RUMBLE = 2'd3;
reg         ev_have = 1'b0;            // an offer is in e_d and not spent
reg         ev_wait = 1'b0;            // an offer is asked for and not yet in
reg         ev_void = 1'b0;            // the offer in flight was asked for by an ended session: dropped on arrival
reg   [1:0] rec_ek_q = EK_STATUS;      // the events record's kind
reg   [2:0] rec_pad_q = 3'd0;          // a RUMBLE's pad
reg   [3:0] st_flag_q = 4'd0;          // the STATUS being written: its stflg
reg   [3:0] st_last_q = 4'd0;          // the stflg of the last STATUS published this session (0 at the hello)
reg  [95:0] rum_last_q = 96'd0;        // each pad's level last published this session (0 at the hello)
/* verilator lint_off UNUSEDSIGNAL */   // the bench's: the protocol monitor forgets an abandoned record's words
reg         ev_abandon = 1'b0;         // one cycle: a STATUS record is abandoned, its head not stored
/* verilator lint_on UNUSEDSIGNAL */
wire  [1:0] ev_k    = e_d[41:40];
wire  [7:0] ev_arg  = e_d[39:32];
wire  [2:0] ev_pad  = (ev_arg[2:0] > 3'd5) ? 3'd5 : ev_arg[2:0];
wire [15:0] ev_rlast = rum_last_q[{ev_pad, 4'd0} +: 16];
wire        ev_bound = evt_bound_q && phase == P_SESSION && !stopped_q;
// the offer in hand opens a record
wire        ev_rec  = (ev_k == EK_STATUS) ? (ev_arg[7:6] == 2'd0 && ev_arg[3:0] != st_last_q) :
                      (ev_k == EK_INFO) ? (ev_arg != 8'd0) : (ev_k == EK_UPLOAD) ? 1'b1 : (e_d[15:0] != ev_rlast);
wire [31:0] ev_hdr  = (ev_k == EK_STATUS) ? {8'h20, 8'd0, 16'd4} : (ev_k == EK_INFO) ? {8'h28, ev_arg, 16'd0} :
                      (ev_k == EK_UPLOAD) ? {8'h29, ev_arg, 16'd0} : {8'h2A, 5'd0, ev_pad, 16'd1};
// its first payload word: a STATUS's word 0, a RUMBLE's level
wire [31:0] ev_w1   = (ev_k == EK_RUMBLE) ? {16'd0, e_d[15:0]} : e_d[31:0];
// a STATUS record's words 1-3: the offer in hand is that word with the record's stflg
wire        st_open = w_act && w_src == S_EV && rec_ek_q == EK_STATUS;
wire        st_wait = st_open && rec_k >= 11'd2 && rec_k <= 11'd4;
wire        st_wok  = ev_have && ev_k == EK_STATUS && {9'd0, ev_arg[7:6]} == rec_k - 11'd1 && ev_arg[3:0] == st_flag_q;
// a STATUS word that is not the open record's next, with its stflg, abandons that record before its head
wire        ev_drop = st_wait && ev_have && !st_wok && !w_wfly && !run_ended;
wire        e_post = ev_bound && ev_have && ev_rec && !bind_pend && !brec_owed;
wire [31:0] e_hdr  = ev_hdr;
wire        e_wv   = !st_wait || st_wok;   // a STATUS word not yet offered lets the consumers in
wire [31:0] e_wd   = st_wait ? e_d[31:0] : rec_lba_q;
assign e_ask = ev_bound && !ev_have && !ev_wait;
`else
wire        ev_drop = 1'b0;
wire        e_post = 1'b0, e_wv = 1'b0;
wire [31:0] e_hdr = 32'd0, e_wd = 32'd0;
/* verilator lint_off UNUSEDSIGNAL */   // no events record starts
wire [31:0] ev_w1 = 32'd0;
/* verilator lint_on UNUSEDSIGNAL */
assign e_ask = 1'b0;
`endif

wire        k_post = 1'b0, k_wv = 1'b0, v_post = 1'b0;
wire [31:0] k_hdr = 32'd0, mark_word = 32'd0, v_hdr = 32'd0, vd_word = 32'd0;
wire        w_hold = 1'b0;

// The session's abandonments of the record open: an HPS reset, a stop or a run's start; the session's end; a host
// tail past the ring's size; and the events tier's out-of-turn STATUS word. A head store in flight still lands.
wire        idx_fault = !stopped_q && (((w_head - rdata[15:0]) & 16'hFFFF) > EVT_WORDS);
wire        idx_done  = t_done && (rd_act == R_IDX) && !run_ended && t_ok;
wire        sess_end  = t_done && (rd_act == R_HELLO) && !run_ended && t_ok && (phase == P_SESSION) &&
                        (rdata != echo_q);
wire        w_drop = hrst_rise | run_fall | run_set | sess_end | (idx_done && idx_fault) | ev_drop;

wire        w_req, w_counted;
wire [15:0] w_off;
wire [31:0] w_wdata;
ocram_evt_wr #(.N(NS)) evt_wr
(
	.clk(clk), .clr(hello_start), .live((phase == P_SESSION) && !stopped_q), .drop(w_drop),
	.kill(hrst_rise | (run_set & ~run_fall)), .ended(run_ended), .t_set(idx_done && !idx_fault), .t_d(rdata[15:0]),
	.head(w_head), .tail(w_tail), .head_nx(w_head_nx), .free(w_free), .act(w_act), .k(rec_k), .first(w_first),
	.src(w_src), .wfly(w_wfly), .hfly(w_hfly),
	.post({v_post, k_post, e_post, r_post, n_post, d_post, b_post}),
	.hdr({v_hdr, k_hdr, e_hdr, r_hdr, n_hdr, d_hdr, b_hdr}),
	.wv({1'b1, k_wv, e_wv, r_wv, n_wv, 1'b1, 1'b1}),
	.wd({vd_word, mark_word, e_wd, r_wd, n_wd, d_wd, 32'd0}),
	.start(w_start), .wtake(w_wtake), .pub(w_pub),
	.req(w_req), .off(w_off), .wdata(w_wdata), .counted(w_counted), .grant(gnt[6]), .done(t_done)
);

`ifdef MISTER_OCRAM_EVT
wire [31:0] bind_word = {20'd0, c0_open, 2'd0, evt_bind_q, 5'd0, c0_sbind};
`else
wire [31:0] bind_word = {20'd0, c0_open, 8'd0, c0_sbind};
`endif

// The arbiter (ocram_arb.sv): each port's request, offset and data from registers or a table; a consumer only in a
// session, and never while a MARK's first payload word waits.
ocram_arb #(.F(NP)) arb
(
	.clk(clk), .en(engine_idle && !hrst_q && !run_fall && !run_set), .kill(hrst_rise),
	.req({(phase == P_SESSION) && c1_req_v && !w_hold, (phase == P_SESSION) && c0_req_v && !w_hold, w_req,
	      bind_pend && !bind_hold, sto_pend, hj_act && hj_has(hj_k), poll_sel, fault_pend, term_pend}),
	.we({c1_we, c0_we, 1'b1, 1'b1, 1'b1, 1'b1, 1'b0, 1'b1, 1'b1}),
	.off({c1_off, c0_off, w_off, C_BIND, C_HELLO, hj_off(hj_k), poll_off, C_STATE, C_HELLO}),
	.wdata({c1_wdata, c0_wdata, w_wdata, bind_word, sto_data, hello_data(hj_k, hj_state, echo_q, c0_wide), 32'd0,
	        state_word(hello_cnt, stopped_q, osd_stopped_q, ts_stopped_q, fio_stopped_q, mnt_stopped_q,
	                   prev_mid_q, prev_mid_osd_q, prev_mid_ts_q, prev_mid_fio_q, prev_mid_mnt_q),
	        C_HELLO_STOP}),
	.counted({1'b1, 1'b1, w_counted, 1'b1, 1'b1, 1'b1, 1'b1, 1'b1, 1'b0}),
	.grant(gnt),
	.t_go(t_go), .t_we(t_we), .t_off(t_off), .t_wdata(t_wdata), .t_counted(t_counted), .t_done(t_done),
	.rdata(rdata), .c_done(a_cdone), .c_rdata(a_crdata)
);

//////////////////////////////  the sequencer  //////////////////////////////

wire fault_go = (t_job == J_FAULT);

always @(posedge clk) begin
	// poll bookkeeping: a launched poll read clears the pending period; a new one sets it
	if (t_go && !t_we && poll_sel && (poll_rd_kind == R_HELLO)) poll_pend <= 1'b0;
	else if (poll_tick && run && !hrst_q) poll_pend <= 1'b1;

	// the line's back-to-back tail re-read yields one turn to the consumers after each of its own reads, and
	// takes it back once a consumer is granted or none is waiting
	if ((t_job == J_POLL) && (((poll_step == 3'd4) && (poll_rd_kind == R_IDX)) ||
	                          ((poll_step == 3'd7) && (poll_rd_kind == R_MARK))))
		line_yield_q <= 1'b1;
	else if ((t_job == J_C0) || (t_job == J_C1) || !consumer_req)
		line_yield_q <= 1'b0;

	// a consumer fault: the ring's bit and the C_STATE store it owes (a fault in the store's own cycle is kept)
	if (OSD_EN != 0 && c0_fault) osd_stopped_q <= 1'b1;
	if (TS_EN != 0 && c1_fault) ts_stopped_q <= 1'b1;
	if (SEC_EN != 0 && c0_sfault) ts_stopped_q <= 1'b1;
	if (FIO_EN != 0 && c0_ffault) fio_stopped_q <= 1'b1;
	if (MNT_EN && c0_mfault) mnt_stopped_q <= 1'b1;
	if (fault_go) fault_pend <= 1'b0;
	if ((OSD_EN != 0 && c0_fault) || (TS_EN != 0 && c1_fault) || (SEC_EN != 0 && c0_sfault) ||
	    (FIO_EN != 0 && c0_ffault) || (MNT_EN && c0_mfault)) fault_pend <= 1'b1;

	// the mark line, raised while a session is idle between polls: its re-read starts at once
	if (FIO_EN != 0 && phase == P_SESSION && poll_step == 3'd0 && mark_line) poll_step <= 3'd7;

	// the OSD bind's C_BIND store is owed whenever the enable link's open differs from the stored bit
	if (phase == P_SESSION && ({c0_open, c0_sbind} != bind_q)) bind_pend <= 1'b1;
`ifdef MISTER_OCRAM_EVT
	if (phase == P_SESSION && evt_bind_q != evt_bound_q) bind_pend <= 1'b1;
`endif

	// the hello clears the session's state when its first store is about to run (the writer its ring's)
	if (hello_start) begin
		phase       <= P_HELLO;
		hj_act      <= 1'b1;
		hj_k        <= 5'd0;
		stopped_q   <= 1'b0;
		line_en_q   <= 1'b0;
		poll_step   <= 3'd0;
		sto_pend    <= 1'b0;            // this hello's C_STATE and C_HELLO supersede a pending state word
		fault_pend  <= 1'b0;
		osd_stopped_q <= 1'b0;
		ts_stopped_q  <= 1'b0;
		fio_stopped_q <= 1'b0;
		mark_arm_q  <= 1'b0;
		osd_bind_q  <= 1'b0;
`ifdef MISTER_OCRAM_EVT
		evt_bind_q <= 1'b0; evt_bound_q <= 1'b0;
`endif
		bind_q      <= 4'd0;
		sec_ask_q   <= 3'd0;
		bind_pend   <= 1'b0;
		mnt_stopped_q <= 1'b0;
		hj_state    <= state_word(hello_cnt + 8'd1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0,
		                          prev_mid_q, prev_mid_osd_q, prev_mid_ts_q, prev_mid_fio_q, prev_mid_mnt_q);
	end

	// the hello the read found: the stores wait for every consumer's rest
	if (hello_hit && !run_ended) begin
		phase  <= P_REST;
		echo_q <= rdata;
	end

	if (hrst_rise) begin
		// an HPS reset ends the run: nothing starts, no terminal store, the engine drops what is unanswered
		// when the level falls
		phase      <= P_OFF;
		term_pend  <= 1'b0;
		fault_pend <= 1'b0;
		sto_pend   <= 1'b0;
		hj_act     <= 1'b0;
		`ifdef MISTER_OCRAM_SECTOR_SLOT
			brec_pend <= 1'b0;
		`endif
		boot_pend  <= 1'b0;
		boot_first <= 1'b1;
		poll_pend  <= 1'b0;
		poll_step  <= 3'd0;
		rd_act     <= R_NONE;
		wr_act     <= W_NONE;
		osd_bind_q <= 1'b0;
`ifdef MISTER_OCRAM_EVT
		evt_bind_q <= 1'b0; evt_bound_q <= 1'b0;
`endif
		bind_q     <= 4'd0;
		sec_ask_q  <= 3'd0;
		bind_pend  <= 1'b0;
		if (w_act) prev_mid_q <= 1'b1;
		if (OSD_EN != 0 && c0_mid) prev_mid_osd_q <= 1'b1;
		if (TS_EN != 0 && c1_mid) prev_mid_ts_q <= 1'b1;
		if (SEC_EN != 0 && c0_smid) prev_mid_ts_q <= 1'b1;
		if (FIO_EN != 0 && c0_fmid) prev_mid_fio_q <= 1'b1;
		if (MNT_EN && c0_mmid) prev_mid_mnt_q <= 1'b1;
	end
	else if (run_fall) begin
		// the stop: the transaction in flight completes in the engine; a record being written is abandoned and its head
		// is not stored; one terminal store follows unless the HPS reset ended the run.
		phase      <= P_OFF;
		if (w_act && !w_hfly) prev_mid_q <= 1'b1;
		if (OSD_EN != 0 && c0_mid) prev_mid_osd_q <= 1'b1;
		if (TS_EN != 0 && c1_mid) prev_mid_ts_q <= 1'b1;
		if (SEC_EN != 0 && c0_smid) prev_mid_ts_q <= 1'b1;
		if (FIO_EN != 0 && c0_fmid) prev_mid_fio_q <= 1'b1;
		if (MNT_EN && c0_mmid) prev_mid_mnt_q <= 1'b1;
		`ifdef MISTER_OCRAM_SECTOR_SLOT
			brec_pend <= 1'b0;
		`endif
		hj_act     <= 1'b0;
		sto_pend   <= 1'b0;
		fault_pend <= 1'b0;
		boot_pend  <= 1'b0;
		boot_first <= 1'b1;
		poll_pend  <= 1'b0;
		poll_step  <= 3'd0;
		rd_act     <= R_NONE;
		osd_bind_q <= 1'b0;
`ifdef MISTER_OCRAM_EVT
		evt_bind_q <= 1'b0; evt_bound_q <= 1'b0;
`endif
		bind_q     <= 4'd0;
		sec_ask_q  <= 3'd0;
		bind_pend  <= 1'b0;
		if (!hrst_q) term_pend <= 1'b1;
	end
	else if (run_set) begin
		phase      <= P_START;
		boot_pend  <= 1'b1;
		boot_first <= 1'b1;
		poll_pend  <= 1'b0;
		poll_step  <= 3'd0;
		fault_pend <= 1'b0;
		sto_pend   <= 1'b0;
		hj_act     <= 1'b0;
		`ifdef MISTER_OCRAM_SECTOR_SLOT
			brec_pend <= 1'b0;
		`endif
		rd_act     <= R_NONE;
		wr_act     <= W_NONE;
	end
	else if (t_go) begin
		// take the command
		if (t_we) begin
			case (t_job)
			J_TERM: begin wr_act <= W_TERM; term_pend <= 1'b0; end
			J_FAULT: begin wr_act <= W_FAULT; fault_pend <= 1'b0; end
			J_HELLO: wr_act <= W_HELLO;
			J_STO: begin wr_act <= W_ONE; sto_pend <= 1'b0; end
			J_BIND: begin
				wr_act <= W_BIND; bind_pend <= 1'b0; bind_q <= {c0_open, c0_sbind};
`ifdef MISTER_OCRAM_EVT
				evt_bound_q <= evt_bind_q;
`endif
`ifdef MISTER_OCRAM_SECTOR_SLOT
				brec_pend <= !stopped_q;
`endif
			end
			default: ;
			endcase
		end
		else begin
			rd_act <= poll_rd_kind;
			if (poll_sel && (poll_rd_kind == R_HELLO) && (phase == P_START)) boot_first <= 1'b0;
		end
		// a record's start: its source takes what the record keeps
`ifdef MISTER_OCRAM_SECTOR_SLOT
		if (w_start[S_BIND]) brec_pend <= 1'b0;
		if (w_start[S_MNT]) begin rec_lba_q <= m_lba; rec_mw_q <= m_write; end
`endif
`ifdef MISTER_OCRAM_EVT
		if (w_start[S_EV]) begin
			// the offer is spent into the record: a STATUS's word 0 or a RUMBLE's level is its first payload
			rec_lba_q <= ev_w1;
			rec_ek_q  <= ev_k;
			rec_pad_q <= ev_pad;
			st_flag_q <= ev_arg[3:0];
		end
`endif
	end

	// a write's completion advances its job; BRESP non-OKAY is counted by the diag and otherwise ignored
	if (t_done && !run_ended) begin
		if (wr_act == W_HELLO) begin
			wr_act <= W_NONE;
			if (hj_act) begin
				if (hj_k == HJ_LAST) begin
					hj_act    <= 1'b0;
					phase     <= P_SESSION;
					poll_step <= 3'd0;        // the first poll reads H_HELLO, then H_BIND and the rest
					hello_cnt <= hello_cnt + 8'd1;
					prev_mid_q<= 1'b0;
					prev_mid_osd_q <= 1'b0;
					prev_mid_ts_q  <= 1'b0;
					prev_mid_fio_q <= 1'b0;
					prev_mid_mnt_q <= 1'b0;
				end
				else hj_k <= hj_next(hj_k);
			end
		end
		else if (wr_act != W_NONE) wr_act <= W_NONE;
`ifdef MISTER_OCRAM_EVT
		if (w_pub[S_EV] && rec_ek_q == EK_STATUS) st_last_q <= st_flag_q;
		if (w_pub[S_EV] && rec_ek_q == EK_RUMBLE) rum_last_q[{rec_pad_q, 4'd0} +: 16] <= rec_lba_q[15:0];
`endif
	end

	// a hello step that this build does not store is passed without a transaction
	if (hj_act && !hj_has(hj_k)) hj_k <= hj_next(hj_k);

	if (t_done && (rd_act != R_NONE) && !run_ended) begin
		rd_act <= R_NONE;
		if (!t_ok) begin
			// a read with a non-OKAY response yields no value: act as if it had not happened
			if (phase == P_SESSION) poll_step <= 3'd0;
			if ((rd_act == R_HELLO) && (phase == P_START)) boot_first <= 1'b0;
		end
		else if (rd_act == R_HELLO) begin
			if (phase == P_START) begin
				boot_pend <= 1'b0;
				phase     <= (rdata == 32'd0) ? P_ARMED : P_WAIT;
				sto_pend  <= 1'b1;
				sto_data  <= (rdata == 32'd0) ? C_HELLO_ARMED : C_HELLO_WAIT;
			end
			else if (phase == P_ARMED) begin
				if (((rdata[31:16] != HELLO_MAGIC) || (rdata[15:0] == 16'd0)) && (rdata != 32'd0)) begin
					phase    <= P_WAIT;
					sto_pend <= 1'b1;
					sto_data <= C_HELLO_WAIT;
				end
			end
			else if (phase == P_WAIT) begin
				if (rdata == 32'd0) begin
					phase    <= P_ARMED;
					sto_pend <= 1'b1;
					sto_data <= C_HELLO_ARMED;
				end
			end
			else if (phase == P_SESSION) begin
`ifdef MISTER_OCRAM_EVT
				if (rdata == echo_q) poll_step <= ((OSD_EN != 0 && !osd_bind_q) || !evt_bind_q) ? 3'd1 :
				                                  SEC_EN != 0 ? 3'd5 : FIO_EN != 0 ? 3'd6 : 3'd2;
`else
				if (rdata == echo_q) poll_step <= (OSD_EN != 0 && !osd_bind_q) ? 3'd1 : SEC_EN != 0 ? 3'd5 :
				                                  FIO_EN != 0 ? 3'd6 : 3'd2;
`endif
				else begin
					// the session ends: its line drops, and each record part-written is recorded for the next hello
					`ifdef MISTER_OCRAM_SECTOR_SLOT
						brec_pend <= 1'b0;
					`endif
					if (w_act) prev_mid_q <= 1'b1;
					if (OSD_EN != 0 && c0_mid) prev_mid_osd_q <= 1'b1;
					if (TS_EN != 0 && c1_mid) prev_mid_ts_q <= 1'b1;
					if (SEC_EN != 0 && c0_smid) prev_mid_ts_q <= 1'b1;
					if (FIO_EN != 0 && c0_fmid) prev_mid_fio_q <= 1'b1;
					if (MNT_EN && c0_mmid) prev_mid_mnt_q <= 1'b1;
					poll_step <= 3'd0;
					sto_pend  <= 1'b1;
					sto_data  <= (rdata == 32'd0) ? C_HELLO_ARMED : C_HELLO_WAIT;
					phase     <= (rdata == 32'd0) ? P_ARMED : P_WAIT;
					osd_bind_q<= 1'b0;
`ifdef MISTER_OCRAM_EVT
					evt_bind_q <= 1'b0; evt_bound_q <= 1'b0;
`endif
					bind_q    <= 4'd0;
					sec_ask_q <= 3'd0;
					bind_pend <= 1'b0;
				end
			end
		end
		else if (rd_act == R_BIND) begin
			if (rdata[11]) osd_bind_q <= 1'b1;
`ifdef MISTER_OCRAM_EVT
			if (rdata[8]) evt_bind_q <= 1'b1;
`endif
			poll_step <= SEC_EN != 0 ? 3'd5 : FIO_EN != 0 ? 3'd6 : 3'd2;
		end
		else if (rd_act == R_SECT) begin
			sec_ask_q <= rdata[2:0] & SEC_MASK;
			poll_step <= FIO_EN != 0 ? 3'd6 : 3'd2;
		end
		else if (rd_act == R_MARK) begin
			// the periodic read goes on to H_LINE; a re-read gives the EVT line its turn, or goes again while the
			// word it read keeps the mark line up
			mark_arm_q <= rdata[31];
			mark_idx_q <= rdata[15:0];
			if (poll_step == 3'd6) poll_step <= 3'd2;
			else if (line_en_q && !stopped_q && (w_head != w_tail)) poll_step <= 3'd4;
			else if (rdata[31] && mark_reached(f_tail, rdata[15:0])) poll_step <= 3'd7;
			else poll_step <= 3'd0;
		end
		else if (rd_act == R_LINE) begin
			line_en_q <= rdata[3];
			poll_step <= 3'd3;
		end
		else if (rd_act == R_IDX) begin
			// the tail itself is the writer's (t_set)
			if (idx_fault) begin
				// a host defect: bit 11 at once, the ring stops for the rest of the session
				stopped_q  <= 1'b1;
				fault_pend <= 1'b1;
				poll_step  <= 3'd0;
				`ifdef MISTER_OCRAM_SECTOR_SLOT
					brec_pend <= 1'b0;
				`endif
				if (w_act) prev_mid_q <= 1'b1;
			end
			else if (!stopped_q) begin
				poll_step <= (FIO_EN != 0 && mark_line) ? 3'd7 : (line_en_q && (w_head != rdata[15:0])) ? 3'd4 : 3'd0;
			end
			else poll_step <= 3'd0;
		end
	end

`ifdef MISTER_OCRAM_EVT
	// the events tier's offers: one asked for at a time; spent into a record, or at once when it needs none
	ev_abandon <= 1'b0;
	if (e_ask) ev_wait <= 1'b1;
	if (e_v) begin ev_wait <= 1'b0; ev_have <= !ev_void; ev_void <= 1'b0; end
	else if (w_start[S_EV] || (w_wtake[S_EV] && st_wait)) ev_have <= 1'b0;
	else if (ev_have && ev_bound && !st_open && !ev_rec) ev_have <= 1'b0;
	if (ev_drop) ev_abandon <= 1'b1;
	// a session's end spends the offer in hand, and voids the one in flight: none is posted in the next session
	if (run_ended || hello_start) begin
		ev_have <= 1'b0;
		if ((ev_wait && !e_v) || e_ask) ev_void <= 1'b1;
	end
	if (hello_start) begin
		st_last_q  <= 4'd0;
		rum_last_q <= 96'd0;
	end
`endif

	// the EVT line: high in a session, allowed by H_LINE, the ring not stopped, the head not the tail
	line <= (phase == P_SESSION) && line_en_q && !stopped_q && (w_head != w_tail);
	// the mark line: high in a session while H_MARK[1] is armed and the download ring's tail has reached its index
	mark_line <= FIO_EN != 0 && (phase == P_SESSION) && mark_arm_q && mark_reached(f_tail, mark_idx_q);
end

endmodule

`ifdef MISTER_OCRAM_EVT
//============================================================================
// hps_ocram_evt -- the events tier's port between hps_io and the master (MISTER_OCRAM_EVT only) hps_io presents one
// offer at a time on HPS_BUS[374:333], {kind[1:0], arg[7:0], data[31:0]}, its rotor's item (sys/hps_io.sv).

/* verilator lint_off DECLFILENAME */   // the port shares the master's file, as the sink does
module hps_ocram_evt
(
	input  wire        clk,          // clk_100m: the master's side
	input  wire        ask,          // one cycle: the master asks for the next offer
	output wire        e_v,          // one cycle: the offer is in e_d
	output wire [41:0] e_d,          // held until the next offer
	input  wire        clk_sys,
	input  wire [41:0] o_d,          // hps_io's offer
	output wire        take          // one cycle: the offer is launched; hps_io's rotor steps
);

wire a_busy, a_v, r_busy;
reg  a_owed = 1'b0;                  // clk: an ask not yet launched
reg  r_owed = 1'b0;                  // clk_sys: an ask arrived, its offer not yet launched
/* verilator lint_off PINCONNECTEMPTY */   // the ask carries no data
lw_xfer #(.W(1)) ask_x(.s_clk(clk), .s_go(ask || a_owed), .s_d(1'b1), .s_busy(a_busy),
                       .d_clk(clk_sys), .d_q(), .d_v(a_v));
/* verilator lint_on PINCONNECTEMPTY */
always @(posedge clk) a_owed <= (ask || a_owed) && a_busy;

assign take = (a_v || r_owed) && !r_busy;
always @(posedge clk_sys) r_owed <= (a_v || r_owed) && r_busy;
lw_xfer #(.W(42)) rep_x(.s_clk(clk_sys), .s_go(take), .s_d(o_d), .s_busy(r_busy),
                        .d_clk(clk), .d_q(e_d), .d_v(e_v));

endmodule
/* verilator lint_on DECLFILENAME */
`endif

`default_nettype wire
`endif
`undef MISTER_OCRAM_POLL
