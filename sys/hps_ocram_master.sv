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

// The floor of the OCRAM mailbox: the AXI3 master on the fpga2hps bridge, its run latch, the poller, the hello,
// the EVT ring's writer and interrupt line, its consumer port, and (with MISTER_OCRAM_DIAG) the diagnostic
// counters and the host-to-fabric torture sink. The design note is the law, spec/ocram-mailbox-design.md (the
// Template's reader-facing copy is docs/OCRAM_MAILBOX.md); this file is layout 1 only.
//
//  * It runs on clk_100m and no fabric reset holds it. It never abandons a transaction it started: a core reset,
//    a stopped clk_sys or a host that has gone cannot leave one in flight.
//  * It cannot address anything but the mailbox: every address is {16'hFFFF, off}, and off is a localparam
//    (below 0x100), an EVT-ring word, EVT_BASE + ((index & (EVT_WORDS - 1)) << 2), or a consumer's own bounded
//    offset (hps_ocram_rx.sv, hps_ocram_tsink below). tb/static.sh holds it.
//  * One transaction at a time, single beat, full 32-bit words. Write: AWVALID and WVALID together, each held
//    with its payload until its ready, then BREADY while the response is awaited. Read: ARVALID held until
//    ARREADY, then RREADY while the response is awaited.
//  * One arbiter, fixed order: the terminal store; the fault's C_STATE store; the poll's and the line's reads when
//    due; the hello's stores; a C_HELLO state store; a C_BIND store; the EVT ring's records (MARK, VERDICT); then
//    the consumers, round robin between the two ports (the last granted goes last). A consumer is granted only in
//    P_SESSION and never in a latch-set or latch-clear cycle.
//  * The run latch (§2.6): it sets on a rising edge of GPO bit 28 (RUN) seen while GPO bit 31 is 1 and no HPS
//    reset is seen; it clears on RUN low, bit 31 low or an HPS reset, and bit 31's return never sets it. Every GPO bit
//    and hps_rst_n enters through an lw_arst cell, never raw.
//  * On a stop: the transaction in flight completes, a record being written is abandoned (its head is not
//    stored), then one terminal store C_HELLO = 0x51540000. An HPS reset ends a run with no terminal store.
//  * An HPS reset makes the master start nothing; a transaction in flight keeps its valids and readies as AXI
//    requires and takes its response if one comes; in the cycle the reset level falls, one still unanswered is
//    dropped.
//  * The hello waits for every built consumer's clk_sys half to rest before its first store. While it waits it
//    starts nothing but the terminal store; a consumer that never rests (a frozen clk_sys) leaves the hello
//    unechoed, which is the design's answer, not a hang.
//  * MARK records, the counters that fill them, the torture sink and the GPO 27/26 test knobs exist only with
//    MISTER_OCRAM_DIAG; hps_ocram_diag and hps_ocram_tsink are separate modules so the fit reports their ALMs
//    apart. The host-to-core rings' consumer is hps_ocram_rx (MISTER_OCRAM_OSD, MISTER_OCRAM_SECTOR_SLOT).
//  * MISTER_OCRAM_SECTOR_SLOT (not with MISTER_OCRAM_DIAG: the record writer and the span are shared): the SECTOR
//    ring r = 4, H_SECTOR read in the session poll, C_BIND bit 0 (stored as hps_ocram_rx reports it), C_K[0..7],
//    and the REQUEST record {0x21, 0, 1} then the LBA, written by the EVT ring's writer behind a room check of 2
//    words. A store of C_BIND that drops bit 0 waits for a record in flight, and no REQUEST starts once it is owed.
//
// Address map, layout 1 (offsets from 0xFFFF0000): the host half of the control page H_HELLO, H_BIND, H_LINE,
// H_IDX[2], H_IDX[3], H_IDX[4]; the fabric half C_HELLO, C_VERSION, C_CAPS, C_STATE, C_IDX[2], C_IDX[3],
// C_IDX[4], C_BIND, C_RING[2..4], C_K[8]; the EVT ring at EVT_BASE, 2,048 words; the torture ring at TS_BASE,
// 8,192 words (the SECTOR span, DIAG test cores only); H_SECTOR and C_K[0..2, 4, 5] with the SECTOR tier. The
// OSD and SECTOR rings are hps_ocram_rx.sv's.
// Every other word is never read and never written.

`ifndef MISTER_OCRAM_POLL
`define MISTER_OCRAM_POLL 12
`endif
`ifndef MISTER_OCRAM_MARK
`define MISTER_OCRAM_MARK 20
`endif

module hps_ocram_master #(
	// The SECTOR tier's declaration (sys_top.v maps the MISTER_OCRAM_SECTOR_* macros), for C_K[0..7]
	parameter [31:0] SEC_SLOT  = 32'd1,
	parameter [31:0] SEC_FRAME = 32'd128,
	parameter [31:0] SEC_GAP   = 32'd2,
	parameter  [2:0] SEC_BLKSZ = 3'd2
)
(
	input  wire        clk,          // clk_100m == h2f_user0_clk
	input  wire        hps_rst_n,    // the HPS's fabric reset (h2f_rst_n), asynchronous
	/* verilator lint_off UNUSEDSIGNAL */   // the floor reads bits 31, 28 and (DIAG) 27, 26; GPO's other users own the rest
	input  wire [31:0] gp_out,       // raw GPO: bits 31 and 28, and with DIAG 27 and 26
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
	/* verilator lint_off UNUSEDSIGNAL */   // the VERDICT record's (DIAG), the REQUEST's (SECTOR): one or the other
	input  wire [15:0] osd_rows,     // ROW records the OSD walker finished this session
	input  wire [15:0] osd_deepest,  // its deepest fill
	output wire        sec_ask,      // H_SECTOR bit 0, read this session
	output wire        evt_stop,     // the EVT ring stopped on a fault
	input  wire        c0_sfault,    // one cycle: the SECTOR ring stopped on a fault
	input  wire        c0_smid,      // level: a SECTOR record is part-consumed
	input  wire        c0_sbind,     // the C_BIND bit 0 to store
	input  wire        c0_wide,      // hps_io's io_wide, through a crossing cell (C_K[1])
	input  wire        rq_post,      // a REQUEST for rq_lba is owed
	input  wire [31:0] rq_lba,
	/* verilator lint_on UNUSEDSIGNAL */
	output wire        rq_start,     // one cycle: that REQUEST's header store is launched

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

localparam [15:0] H_HELLO = 16'h0000;      // host: {0x4F43, nonce}, 0 = no session
localparam [15:0] H_BIND  = 16'h0004;      // host: bit 11 = the OSD bind
localparam [15:0] H_LINE  = 16'h0008;      // host: bit 3 lets the EVT line rise
localparam [15:0] H_SECTOR = 16'h000C;     // host: bit 0 asks for the SECTOR slot
localparam [15:0] H_IDX3  = 16'h001C;      // host: the EVT ring's tail
localparam [15:0] C_HELLO = 16'h0080;      // fabric: the master's state
localparam [15:0] C_VERSION = 16'h0084;    // fabric: {major, minor} = 1.0
localparam [15:0] C_CAPS  = 16'h0088;      // fabric: bit 2 OSD, bit 3 EVT, bit 4 the torture sink
localparam [15:0] C_STATE = 16'h008C;      // fabric: hello count, fault and previous-session-mid bits
localparam [15:0] C_IDX2  = 16'h0098;      // fabric: the OSD ring's tail
localparam [15:0] C_IDX3  = 16'h009C;      // fabric: the EVT ring's head
localparam [15:0] C_IDX4  = 16'h00A0;      // fabric: the torture ring's tail
localparam [15:0] C_BIND  = 16'h00A4;      // fabric: bit 11 = the OSD bind is in effect
localparam [15:0] C_RING2 = 16'h00B8;      // fabric: the OSD ring's descriptor
localparam [15:0] C_RING3 = 16'h00BC;      // fabric: the EVT ring's descriptor
localparam [15:0] C_RING4 = 16'h00C0;      // fabric: the torture ring's descriptor
localparam [15:0] C_K0    = 16'h00D0;      // fabric: the SECTOR block's K0, K1, K2, K4, K5
localparam [15:0] C_K1    = 16'h00D4;
localparam [15:0] C_K2    = 16'h00D8;
localparam [15:0] C_K4    = 16'h00E0;
localparam [15:0] C_K5    = 16'h00E4;
localparam [15:0] C_K8    = 16'h00F0;      // fabric: OSDBUF's K0, {rows, row bytes}
localparam [15:0] EVT_BASE = 16'h4000;     // fabric: EVT word k at EVT_BASE + ((head + k) & 2047) * 4
localparam [15:0] EVT_WORDS = 16'd2048;

localparam [31:0] C_HELLO_ARMED = 32'h4F43_0000;
localparam [31:0] C_HELLO_WAIT  = 32'h5754_0000;
localparam [31:0] C_HELLO_STOP  = 32'h5154_0000;
localparam [15:0] HELLO_MAGIC   = 16'h4F43;
localparam [31:0] C_VERSION_1_0 = 32'h0001_0000;
localparam [31:0] C_CAPS_W      = 32'h0000_0008
`ifdef MISTER_OCRAM_OSD
                                  | 32'h0000_0004
`endif
`ifdef MISTER_OCRAM_DIAG
                                  | 32'h0000_0010
`endif
`ifdef MISTER_OCRAM_SECTOR_SLOT
                                  | 32'h0000_0010
`endif
                                  ;
localparam [31:0] C_RING2_OSD   = {5'd3, 5'd0, 4'd11, 4'd0, 14'h0800};    // OSD, no line, 2048 words, 0x2000
localparam [31:0] C_RING3_EVT   = {5'd4, 5'd4, 4'd11, 4'd0, 14'h1000};    // EVT, line 4, 2048 words, 0x4000
/* verilator lint_off UNUSEDPARAM */   // a DIAG build's words or a SECTOR build's, never both
localparam [31:0] C_RING4_TS    = {5'd31, 5'd0, 4'd13, 4'd0, 14'h1C00};   // DIAG, no line, 8192 words, 0x7000
localparam [31:0] C_RING4_SEC   = {5'd2, 5'd0, 4'd13, 4'd0, 14'h1C00};    // SECTOR, no line, 8192 words, 0x7000
localparam [31:0] C_K0_SEC      = {24'd0, SEC_GAP[7:0]};                 // {FIFO entries 0, 0, strobe gap}
localparam [31:0] C_K4_SEC      = {8'h01, 4'd0, SEC_SLOT[3:0], 3'd0, SEC_FRAME[12:0]};   // {RD, slot, frame words}
localparam [31:0] REQ_HEADER    = {8'h21, 8'd0, 16'd1};                  // kind REQUEST, arg 0, len 1
`ifdef MENU_CORE
localparam [15:0] OSD_ROWS      = 16'd20;                                // lw_osd's ROWS with MENU_CORE
`else
localparam [15:0] OSD_ROWS      = 16'd16;
`endif
localparam [31:0] C_K8_OSD      = {OSD_ROWS, 16'd256};
localparam [31:0] MARK_HEADER   = {8'h23, 8'h03, 16'd40};                // kind MARK, arg 3, len 40
localparam [31:0] VERDICT_HEADER= {8'h25, 8'd4, 16'd8};                  // kind VERDICT, arg 4, len 8
/* verilator lint_on UNUSEDPARAM */
localparam integer POLL = `MISTER_OCRAM_POLL;

`ifdef MISTER_OCRAM_OSD
localparam OSD_EN = 1;
`else
localparam OSD_EN = 0;
`endif
`ifdef MISTER_OCRAM_DIAG
localparam TS_EN = 1;
`else
localparam TS_EN = 0;
`endif
`ifdef MISTER_OCRAM_SECTOR_SLOT
localparam SEC_EN = 1;
`else
localparam SEC_EN = 0;
`endif
localparam RX_EN = OSD_EN != 0 || SEC_EN != 0;     // hps_ocram_rx takes port 0

// A static assertion: an address that could leave the mailbox stops elaboration.
generate
	if (EVT_BASE + EVT_WORDS * 4 > 16'hF000) begin : g_evt_span
		ocram_evt_span_above_0xF000 guard();
	end
	if (H_HELLO >= 16'h0100 || H_BIND >= 16'h0100 || H_LINE >= 16'h0100 || H_SECTOR >= 16'h0100 ||
	    H_IDX3 >= 16'h0100 || C_K0 >= 16'h0100 || C_K1 >= 16'h0100 || C_K2 >= 16'h0100 ||
	    C_K4 >= 16'h0100 || C_K5 >= 16'h0100 ||
	    C_HELLO >= 16'h0100 || C_VERSION >= 16'h0100 || C_CAPS >= 16'h0100 ||
	    C_STATE >= 16'h0100 || C_IDX2 >= 16'h0100 || C_IDX3 >= 16'h0100 || C_IDX4 >= 16'h0100 ||
	    C_BIND >= 16'h0100 || C_RING2 >= 16'h0100 || C_RING3 >= 16'h0100 || C_RING4 >= 16'h0100 ||
	    C_K8 >= 16'h0100) begin : g_ctrl_span
		ocram_control_constant_above_0xFF guard();
	end
endgenerate

// The SECTOR tier and the DIAG sink share ring 4's span and the EVT ring's record writer: a build of both stops.
`ifdef MISTER_OCRAM_DIAG
`ifdef MISTER_OCRAM_SECTOR_SLOT
ocram_sector_excludes_MISTER_OCRAM_DIAG diag_guard();
`endif
`endif

assign present = 1'b1;

////////////////////////////  clock-domain entry  ///////////////////////////

// Every GPO bit and hps_rst_n enters only through an lw_arst cell. idv_q is high in the ID view (bit 31 low),
// rn_q is high while RUN is 0 (a 0 of any length sets the chain asynchronously), hrst_q is high while the HPS
// reset is seen. With DIAG, nsn_q is high while the non-secure knob (bit 27) is 0 and fln_q while flood (bit 26)
// is 0; both are sampled at the latch set only.
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

`ifdef MISTER_OCRAM_DIAG
wire nsn_q, fln_q;
lw_arst ns_c(.clk(clk), .rst_n(gp_out[27]), .q(nsn_q));
lw_arst fl_c(.clk(clk), .rst_n(gp_out[26]), .q(fln_q));

reg run_ns = 1'b0, run_flood = 1'b0;   // sampled at the latch set, fixed for the run
always @(posedge clk) if (run_set) begin
	run_ns    <= ~nsn_q;
	run_flood <= ~fln_q;
end
wire ns = run_ns;
`else
wire ns = 1'b0;
`endif

////////////////////////////////  AXI engine  ///////////////////////////////

// One transaction at a time. The valids are combinational from the state register and hold their payloads
// (off_q, wdata_q) until each ready; the response readies are high in the response states. hrst_q gates
// starting only: a transaction in flight is never abandoned at the reset's rise, and one still unanswered in
// the cycle the level falls is dropped.

localparam [2:0] E_IDLE = 3'd0, E_WR = 3'd1, E_BRSP = 3'd2, E_AR = 3'd3, E_RRSP = 3'd4;

reg  [2:0]  est = E_IDLE;
reg  [15:0] off_q = 16'd0;
reg  [31:0] wdata_q = 32'd0;
reg         aw_pend = 1'b0, w_pend = 1'b0, ar_pend = 1'b0;
/* verilator lint_off UNUSEDSIGNAL */   // the DIAG counters' inputs
reg         counted_q = 1'b0;
reg         drop_q = 1'b0;
/* verilator lint_on UNUSEDSIGNAL */

wire engine_idle = (est == E_IDLE);
wire b_fire = bvalid & bready;
wire r_fire = rvalid & rready;
wire t_done = (est == E_BRSP) ? b_fire : (est == E_RRSP) ? r_fire : 1'b0;
wire t_ok   = (est == E_BRSP) ? (bresp == 2'b00) : (rresp == 2'b00);

// the controller's command, valid in an idle cycle
reg         t_go;
reg         t_we;
reg  [15:0] t_off;
reg  [31:0] t_wdata;
reg         t_counted;

always @(posedge clk) begin
	drop_q <= 1'b0;
	if (hrst_fall && !engine_idle && !t_done) begin
		est     <= E_IDLE;                    // the reset orphaned it: drop it
		aw_pend <= 1'b0; w_pend <= 1'b0; ar_pend <= 1'b0;
		drop_q  <= 1'b1;
	end
	else case (est)
	E_IDLE: if (t_go && !hrst_q) begin
		off_q     <= t_off;
		wdata_q   <= t_wdata;
		counted_q <= t_counted;
		if (t_we) begin
			est <= E_WR; aw_pend <= 1'b1; w_pend <= 1'b1;
		end
		else begin
			est <= E_AR; ar_pend <= 1'b1;
		end
	end
	E_WR: begin
		aw_pend <= aw_pend & ~awready;
		w_pend  <= w_pend  & ~wready;
		if ((!aw_pend || awready) && (!w_pend || wready)) est <= E_BRSP;
	end
	E_BRSP: if (b_fire) est <= E_IDLE;
	E_AR: if (ar_pend && arready) begin ar_pend <= 1'b0; est <= E_RRSP; end
	E_RRSP: if (r_fire) est <= E_IDLE;
	default: est <= E_IDLE;
	endcase
end

assign awid    = 8'd0;
assign wid     = 8'd0;
assign arid    = 8'd0;
assign awaddr  = {16'hFFFF, off_q};    // the constant upper half: OCRAM and nothing else
assign araddr  = {16'hFFFF, off_q};
assign awlen   = 4'd0;
assign arlen   = 4'd0;
assign awsize  = 3'b010;               // a 32-bit word
assign arsize  = 3'b010;
assign awburst = 2'b01;                // INCR
assign arburst = 2'b01;
assign awlock  = 2'b00;
assign arlock  = 2'b00;
assign awcache = 4'b0000;
assign arcache = 4'b0000;
assign awuser  = 5'd0;
assign aruser  = 5'd0;
assign awprot  = {1'b0, ns, 1'b0};     // {instruction, non-secure, privileged}: secure unless the DIAG knob
assign arprot  = {1'b0, ns, 1'b0};
assign wdata   = wdata_q;
assign wstrb   = 4'hF;
assign wlast   = 1'b1;
assign awvalid = (est == E_WR) && aw_pend;
assign wvalid  = (est == E_WR) && w_pend;
assign bready  = (est == E_BRSP);
assign arvalid = (est == E_AR) && ar_pend;
assign rready  = (est == E_RRSP);

///////////////////////////////  the run  ///////////////////////////////////

// A run's phase. P_REST is the hello's wait for every consumer's clk_sys half; P_HELLO covers the hello's
// stores, which run in order and in one piece; P_SESSION is a session with the hello echoed.
localparam [2:0] P_OFF = 3'd0, P_START = 3'd1, P_ARMED = 3'd2, P_WAIT = 3'd3,
                 P_SESSION = 3'd4, P_HELLO = 3'd5, P_REST = 3'd6;

localparam [2:0] R_NONE = 3'd0, R_HELLO = 3'd1, R_LINE = 3'd2, R_IDX = 3'd3, R_BIND = 3'd4, R_SECT = 3'd5;
/* verilator lint_off UNUSEDPARAM */   // the record jobs are a DIAG build's or a SECTOR build's
localparam [2:0] W_NONE = 3'd0, W_HELLO = 3'd1, W_ONE = 3'd2, W_REC = 3'd3,
                 W_RECHEAD = 3'd4, W_TERM = 3'd5, W_FAULT = 3'd6, W_BIND = 3'd7;
/* verilator lint_on UNUSEDPARAM */

reg  [2:0]  phase = P_OFF;
reg  [7:0]  hello_cnt = 8'd0;          // hellos taken since configuration, no reset; counts each echo
reg  [31:0] echo_q = 32'd0;            // the H_HELLO value this session echoed
reg  [15:0] head_q = 16'd0;            // the EVT ring's head, last published
reg  [15:0] tail_q = 16'd0;            // the EVT ring's tail, last read
reg         line_en_q = 1'b0;          // H_LINE bit 3 at its last read
reg         stopped_q = 1'b0;          // the EVT ring stopped on a fault this session
reg         prev_mid_q = 1'b0;         // the EVT record of the session before this one was part-written
reg         rec_mid_q = 1'b0;          // an EVT record is being written and not yet published
reg         osd_bind_q = 1'b0;         // H_BIND bit 11 read: the OSD bind asked for this session
reg  [1:0]  bind_q = 2'd0;             // the C_BIND bits {11, 0} last stored
reg         sec_ask_q = 1'b0;          // H_SECTOR bit 0 read this session
reg         bind_pend = 1'b0;          // a C_BIND store is owed
reg         osd_stopped_q = 1'b0;      // the OSD ring stopped on a fault this session
reg         ts_stopped_q = 1'b0;       // the torture ring stopped on a fault this session
reg         prev_mid_osd_q = 1'b0;     // the OSD record of the previous session was part-consumed
reg         prev_mid_ts_q = 1'b0;      // the torture record of the previous session was part-read
`ifdef MISTER_OCRAM_DIAG
reg  [15:0] deepest_q = 16'd0;         // deepest EVT waiting this session, for MARK word 0
reg         verdict_pend = 1'b0;       // a VERDICT follows the period's MARK
`endif
reg  [2:0]  poll_step = 3'd0;          // session poll: 1 H_BIND, 5 H_SECTOR, 2 H_LINE, 3 H_IDX, 4 another while line high
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
reg  [2:0]  wr_act = W_NONE;           // the write in flight
localparam [POLL-1:0] POLL_T0 = {POLL{1'b0}};
localparam [POLL-1:0] POLL_ONE = 1;
reg  [POLL-1:0] poll_t = {POLL{1'b0}}; // free-running; its period is 2^POLL cycles
wire poll_tick = (poll_t == POLL_T0);

always @(posedge clk) poll_t <= run_set ? POLL_T0 : poll_t + POLL_ONE;   // a run's polls count from its start

wire run_ended = hrst_rise | run_fall;

// The consumers' session returns and the hello's wait for their rest. c1_* are the torture sink's outputs
// (driven below with DIAG, constant when unbuilt). The hello's first store runs only when every built
// consumer's clk_sys half reports rest; while it waits the arbiter starts only the terminal store.
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
// session mid-record; every other bit 0. The ring numbers are fixed: EVT r = 3 (bits 11, 19), OSD r = 2 (bits
// 10, 18), the torture ring r = 4 (bits 12, 20). cnt is the hello count the word reports: the incremented one at
// a hello, the current one at a fault.
function [31:0] state_word(input [7:0] cnt, input fe, input fo, input ft, input me, input mo, input mt);
	begin
		state_word = {24'd0, cnt};
		if (me) state_word[19] = 1'b1;
		if (mo) state_word[18] = 1'b1;
		if (mt) state_word[20] = 1'b1;
		if (fe) state_word[11] = 1'b1;
		if (fo) state_word[10] = 1'b1;
		if (ft) state_word[12] = 1'b1;
	end
endfunction

// The hello's stores, in their fixed order, the last C_HELLO (HJ_LAST). hj_has says whether a step is stored in this
// build; a step that is
// not is passed without a transaction. Args only: Quartus drops a module-scope read in a function.
localparam [4:0] HJ_LAST = 5'd16;
function hj_has(input [4:0] k);
	begin
		hj_has = (k <= 5'd16);
`ifndef MISTER_OCRAM_OSD
		if (k == 5'd3 || k == 5'd7 || k == 5'd10) hj_has = 1'b0;
`endif
`ifndef MISTER_OCRAM_DIAG
`ifndef MISTER_OCRAM_SECTOR_SLOT
		if (k == 5'd5 || k == 5'd9) hj_has = 1'b0;
`endif
`endif
`ifndef MISTER_OCRAM_SECTOR_SLOT
		if (k >= 5'd11 && k <= 5'd15) hj_has = 1'b0;
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
		default: hj_off = C_HELLO;
		endcase
	end
endfunction

function [31:0] hello_data(input [4:0] k, input [31:0] state, input [31:0] echo, input wide);
	begin
		case (k)
		5'd0: hello_data = C_VERSION_1_0;
		5'd1: hello_data = C_CAPS_W;
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
		default: hello_data = echo;
		endcase
	end
endfunction

// The hello the read found: a valid H_HELLO in ARMED moves to the consumers' rest wait. diag_clr clears the
// DIAG counters when the hello's first store is about to run, at the same edge the bench's own clear sees.
wire hello_hit = t_done && (rd_act == R_HELLO) && t_ok && (phase == P_ARMED) &&
                 (rdata[31:16] == HELLO_MAGIC) && (rdata[15:0] != 16'd0);
/* verilator lint_off UNUSEDSIGNAL */   // the DIAG counters' clear
wire diag_clr = hello_start;
/* verilator lint_on UNUSEDSIGNAL */

// The read the poll (or the boot, or the line) wants next, after the consumers' declarations below: a step-4
// read is the line's back-to-back tail re-read and yields to a requesting consumer (line_yield_q). Steps 1-3 and
// the periodic H_HELLO poll are unchanged.
reg  poll_sel;
reg  [2:0] poll_rd_kind;

//////////////////////////////  the consumers  //////////////////////////////

// Port 0 is the OSD walker (hps_ocram_osd, MISTER_OCRAM_OSD); port 1 is the torture sink, built below with DIAG.
// An unbuilt port's request is the constant 0. A consumer asks for one transaction at a time and holds c_req
// until c_grant; c_done is the R or B handshake's cycle, with c_ok and c_rdata. Round robin: the last granted
// goes last, and only a session grants.
wire c0_req_v = RX_EN && c0_req;
`ifdef MISTER_OCRAM_DIAG
wire        c1_req, c1_we;
wire [15:0] c1_off;
wire [31:0] c1_wdata;
wire        c1_grant, c1_done, c1_ok;
wire [31:0] c1_rdata;
wire [31:0] ts_records, ts_words, ts_errors, ts_first_g, ts_first_got, ts_sum;
wire [15:0] ts_skipped, ts_deepest;
wire c1_req_v = c1_req;
`else
wire c1_req_v = 1'b0;
assign c1_rest = 1'b1;                 // the unbuilt port is always at rest
assign c1_fault = 1'b0;
assign c1_mid = 1'b0;
`endif

reg  c_last = 1'b0;                    // the consumer port granted last: 0 or 1
wire c0_sel = (phase == P_SESSION) && c0_req_v && (!c1_req_v || c_last);
/* verilator lint_off UNUSEDSIGNAL */   // port 1 is built with DIAG
wire c1_sel = (phase == P_SESSION) && c1_req_v && (!c0_req_v || !c_last);
/* verilator lint_on UNUSEDSIGNAL */

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
		else if (poll_step == 3'd2) begin
			poll_sel = 1'b1; poll_rd_kind = R_LINE;
		end
		else if (poll_step == 3'd3) begin
			poll_sel = 1'b1; poll_rd_kind = R_IDX;
		end
		else if (poll_step == 3'd4 && !(line_yield_q && consumer_req)) begin
			poll_sel = 1'b1; poll_rd_kind = R_IDX;      // at most one re-read between two consumer grants
		end
	end
end

reg  c_act_v = 1'b0;                   // a consumer transaction is in flight
reg  c_act_port = 1'b0;
reg  c_act_we = 1'b0;

reg  [3:0] t_job = 4'd0;               // the job the arbiter chose this cycle, J_NONE if none
/* verilator lint_off UNUSEDPARAM */   // the record jobs are a DIAG build's or a SECTOR build's
localparam [3:0] J_NONE = 4'd0, J_TERM = 4'd1, J_FAULT = 4'd2, J_POLL = 4'd3, J_HELLO = 4'd4,
                 J_STO = 4'd5, J_BIND = 4'd6, J_REC = 4'd7, J_MARK = 4'd8, J_VERDICT = 4'd9,
                 J_C0 = 4'd10, J_C1 = 4'd11, J_RQ = 4'd12;
/* verilator lint_on UNUSEDPARAM */
wire c0_go = (t_job == J_C0);
/* verilator lint_off UNUSEDSIGNAL */   // port 1 is built with DIAG
wire c1_go = (t_job == J_C1);
/* verilator lint_on UNUSEDSIGNAL */

assign c0_grant = c0_go;
assign c0_done  = t_done & c_act_v & ~c_act_port;
assign c0_ok    = t_ok;
assign c0_rdata = c_act_we ? 32'd0 : rdata;
`ifdef MISTER_OCRAM_DIAG
assign c1_grant = c1_go;
assign c1_done  = t_done & c_act_v & c_act_port;
assign c1_ok    = t_ok;
assign c1_rdata = c_act_we ? 32'd0 : rdata;

hps_ocram_tsink tsink
(
	.clk       (clk),
	.s_hello   (hello_start),
	.s_live    (s_live_w),
	.s_tick    (poll_tick),
	.c_rest    (c1_rest),
	.c_fault   (c1_fault),
	.c_mid     (c1_mid),
	.c_req     (c1_req),
	.c_we      (c1_we),
	.c_off     (c1_off),
	.c_wdata   (c1_wdata),
	.c_grant   (c1_grant),
	.c_done    (c1_done),
	.c_ok      (c1_ok),
	.c_rdata   (c1_rdata),
	.v_records (ts_records),
	.v_words   (ts_words),
	.v_errors  (ts_errors),
	.v_first_g (ts_first_g),
	.v_first_got(ts_first_got),
	.v_sum     (ts_sum),
	.v_skipped (ts_skipped),
	.v_deepest (ts_deepest)
);
`endif

/////////////////////////////  the EVT ring  ////////////////////////////////

`ifdef MISTER_OCRAM_DIAG
// A MARK has room when 2,048 - waiting holds its 41 words (and, for a period MARK, the VERDICT's 9 beside it:
// the period's record is followed at once by the verdict, and flood posts no verdict); a VERDICT needs 9.
wire [15:0] wait_now = head_q - tail_q;
wire [15:0] room_now = EVT_WORDS - wait_now;
wire evt_room = !stopped_q && (wait_now <= EVT_WORDS);
wire [15:0] mark_need = run_flood ? 16'd41 : 16'd50;
wire mark_ok = (phase == P_SESSION) && evt_room && !rec_act && (run_flood || mark_due_q) &&
               (room_now >= mark_need);
wire verdict_ok = (phase == P_SESSION) && evt_room && !rec_act && verdict_pend && (room_now >= 16'd9);
wire [15:0] evt_index = head_q + {10'd0, rec_k};
wire [15:0] evt_off = EVT_BASE + ((evt_index & (EVT_WORDS - 1)) << 2);

localparam integer MARKW = `MISTER_OCRAM_MARK;
localparam [MARKW-1:0] MARK_T0 = {MARKW{1'b0}};
localparam [MARKW-1:0] MARK_ONE = 1;
reg  [5:0]  rec_k = 6'd0;              // 0 header, 1..len payload, len+1 the head store
reg         rec_act = 1'b0;
reg         rec_kind_q = 1'b0;         // 0 MARK, 1 VERDICT
reg  [5:0]  rec_len_q = 6'd40;
reg         mark_due_q = 1'b0;
reg  [MARKW-1:0] mark_t = {MARKW{1'b0}};
wire [31:0] mark_word;
wire        snap_ready;

// the VERDICT's eight payload words, one instant at the record's start
reg  [31:0] vd0 = 32'd0, vd1 = 32'd0, vd2 = 32'd0, vd3 = 32'd0;
reg  [31:0] vd4 = 32'd0, vd5 = 32'd0, vd6 = 32'd0, vd7 = 32'd0;
reg  [31:0] vd_word;
always @(*) begin
	case (rec_k)
	6'd1: vd_word = vd0;
	6'd2: vd_word = vd1;
	6'd3: vd_word = vd2;
	6'd4: vd_word = vd3;
	6'd5: vd_word = vd4;
	6'd6: vd_word = vd5;
	6'd7: vd_word = vd6;
	6'd8: vd_word = vd7;
	default: vd_word = 32'd0;
	endcase
end

wire mark_start = (t_job == J_MARK);
`endif

`ifndef MISTER_OCRAM_DIAG
`ifdef MISTER_OCRAM_SECTOR_SLOT
// The SECTOR tier's REQUEST record: its header, the LBA, then the head, behind a room check of 2 words; it starts
// only while C_BIND bit 0 is stored 1 and no C_BIND store is owed, and a store that drops bit 0 waits for it.
reg  [5:0]  rec_k = 6'd0;              // 0 header, 1 the LBA, 2 the head store
reg         rec_act = 1'b0;
wire [15:0] wait_now = head_q - tail_q;
wire [15:0] evt_index = head_q + {10'd0, rec_k};
wire [15:0] evt_off = EVT_BASE + ((evt_index & (EVT_WORDS - 1)) << 2);
wire rq_ok = (phase == P_SESSION) && !stopped_q && !rec_act && rq_post && c0_sbind && bind_q[0] && !bind_pend &&
             (wait_now <= EVT_WORDS - 16'd2);
wire bind_hold = bind_q[0] && !c0_sbind && rec_act;
assign rq_start = (t_job == J_RQ);
`else
wire bind_hold = 1'b0;
assign rq_start = 1'b0;
`endif
`else
wire bind_hold = 1'b0;
assign rq_start = 1'b0;
`endif

// The arbiter: the terminal store, the fault's C_STATE store, the poll's or the line's reads when due, the
// hello's stores, a state store, a C_BIND store, the EVT record's words, then the consumers.
always @(*) begin
	t_go = 1'b0; t_we = 1'b0; t_off = H_HELLO; t_wdata = 32'd0; t_counted = 1'b1; t_job = J_NONE;
	if (engine_idle && !hrst_q && !run_fall && !run_set) begin
		if (term_pend) begin
			t_go = 1'b1; t_we = 1'b1; t_off = C_HELLO; t_wdata = C_HELLO_STOP; t_counted = 1'b0;
			t_job = J_TERM;
		end
		else if (fault_pend) begin
			t_go = 1'b1; t_we = 1'b1; t_off = C_STATE; t_job = J_FAULT;
			t_wdata = state_word(hello_cnt, stopped_q, osd_stopped_q, ts_stopped_q,
			                     prev_mid_q, prev_mid_osd_q, prev_mid_ts_q);
		end
		else if (poll_sel) begin
			t_go = 1'b1; t_we = 1'b0; t_job = J_POLL;
			case (poll_rd_kind)
			R_LINE:  t_off = H_LINE;
			R_IDX:   t_off = H_IDX3;
			R_BIND:  t_off = H_BIND;
			R_SECT:  t_off = H_SECTOR;
			default: t_off = H_HELLO;
			endcase
		end
		else if (hj_act && hj_has(hj_k)) begin
			t_go = 1'b1; t_we = 1'b1; t_job = J_HELLO;
			t_off   = hj_off(hj_k);
			t_wdata = hello_data(hj_k, hj_state, echo_q, c0_wide);
		end
		else if (sto_pend) begin
			t_go = 1'b1; t_we = 1'b1; t_off = C_HELLO; t_wdata = sto_data; t_job = J_STO;
		end
		else if (bind_pend && !bind_hold) begin
			t_go = 1'b1; t_we = 1'b1; t_off = C_BIND; t_wdata = {20'd0, c0_open, 10'd0, c0_sbind}; t_job = J_BIND;
		end
`ifndef MISTER_OCRAM_DIAG
`ifdef MISTER_OCRAM_SECTOR_SLOT
		else if (rec_act) begin
			t_go = 1'b1; t_we = 1'b1; t_job = J_REC;
			if (rec_k == 6'd2) begin t_off = C_IDX3; t_wdata = {16'd0, head_q + 16'd2}; end
			else begin t_off = evt_off; t_wdata = rq_lba; end        // hps_ocram_rx's request register
		end
		else if (rq_ok) begin
			t_go = 1'b1; t_we = 1'b1; t_off = evt_off; t_wdata = REQ_HEADER; t_job = J_RQ;
		end
`endif
`endif
`ifdef MISTER_OCRAM_DIAG
		else if (rec_act) begin
			if (rec_k == 6'd1 && !rec_kind_q && !snap_ready) begin
				// a MARK's payload word 1 waits for the counter snapshot still draining older transactions
			end
			else begin
				t_go = 1'b1; t_we = 1'b1; t_job = J_REC;
				if (rec_k == rec_len_q + 6'd1) begin
					t_off = C_IDX3; t_wdata = {16'd0, head_q + {10'd0, rec_len_q} + 16'd1}; t_counted = 1'b1;
				end
				else begin
					t_off = evt_off;
					t_wdata = rec_kind_q ? vd_word : mark_word;
					t_counted = 1'b0;
				end
			end
		end
		else if (mark_ok) begin
			t_go = 1'b1; t_we = 1'b1; t_off = evt_off; t_wdata = MARK_HEADER; t_counted = 1'b0;
			t_job = J_MARK;
		end
		else if (verdict_ok) begin
			t_go = 1'b1; t_we = 1'b1; t_off = evt_off; t_wdata = VERDICT_HEADER; t_counted = 1'b0;
			t_job = J_VERDICT;
		end
`endif
		else if (c0_sel) begin
			t_go = 1'b1; t_we = c0_we; t_off = c0_off; t_wdata = c0_wdata; t_job = J_C0;
		end
`ifdef MISTER_OCRAM_DIAG
		else if (c1_sel) begin
			t_go = 1'b1; t_we = c1_we; t_off = c1_off; t_wdata = c1_wdata; t_job = J_C1;
		end
`endif
	end
end

`ifdef MISTER_OCRAM_DIAG
// The hello starts the torture sink's counters; a MARK record restarts its period. The period counter marks the
// period due once and holds it until the record begins.
wire mark_restart = diag_clr | mark_start;
always @(posedge clk) begin
	if (mark_restart) begin
		mark_t <= MARK_T0;
		mark_due_q <= 1'b0;
	end
	else if (mark_t == {MARKW{1'b1}}) mark_due_q <= 1'b1;
	else mark_t <= mark_t + MARK_ONE;
end
`endif

//////////////////////////////  the sequencer  //////////////////////////////

wire fault_go = (t_job == J_FAULT);

always @(posedge clk) begin
	// poll bookkeeping: a launched poll read clears the pending period; a new one sets it
	if (t_go && !t_we && poll_sel && (poll_rd_kind == R_HELLO)) poll_pend <= 1'b0;
	else if (poll_tick && run && !hrst_q) poll_pend <= 1'b1;

	// the line's back-to-back tail re-read yields one turn to the consumers after each of its own reads, and
	// takes it back once a consumer is granted or none is waiting
	if ((t_job == J_POLL) && (poll_step == 3'd4) && (poll_rd_kind == R_IDX))
		line_yield_q <= 1'b1;
	else if ((t_job == J_C0) || (t_job == J_C1) || !consumer_req)
		line_yield_q <= 1'b0;

	// a consumer fault: the ring's bit and the C_STATE store it owes (a fault in the store's own cycle is kept)
	if (OSD_EN != 0 && c0_fault) osd_stopped_q <= 1'b1;
	if (TS_EN != 0 && c1_fault) ts_stopped_q <= 1'b1;
	if (SEC_EN != 0 && c0_sfault) ts_stopped_q <= 1'b1;
	if (fault_go) fault_pend <= 1'b0;
	if ((OSD_EN != 0 && c0_fault) || (TS_EN != 0 && c1_fault) || (SEC_EN != 0 && c0_sfault)) fault_pend <= 1'b1;

	// the OSD bind's C_BIND store is owed whenever the enable link's open differs from the stored bit
	if (phase == P_SESSION && ({c0_open, c0_sbind} != bind_q)) bind_pend <= 1'b1;

	// the hello clears the session's ring state when its first store is about to run
	if (hello_start) begin
		phase       <= P_HELLO;
		hj_act      <= 1'b1;
		hj_k        <= 5'd0;
		head_q      <= 16'd0;
		tail_q      <= 16'd0;
		stopped_q   <= 1'b0;
		line_en_q   <= 1'b0;
		poll_step   <= 3'd0;
		rec_mid_q   <= 1'b0;
		sto_pend    <= 1'b0;            // this hello's C_STATE and C_HELLO supersede a pending state word
		fault_pend  <= 1'b0;
		osd_stopped_q <= 1'b0;
		ts_stopped_q  <= 1'b0;
		osd_bind_q  <= 1'b0;
		bind_q      <= 2'd0;
		sec_ask_q   <= 1'b0;
		bind_pend   <= 1'b0;
		hj_state    <= state_word(hello_cnt + 8'd1, 1'b0, 1'b0, 1'b0,
		                          prev_mid_q, prev_mid_osd_q, prev_mid_ts_q);
`ifdef MISTER_OCRAM_DIAG
		rec_act     <= 1'b0;
		rec_k       <= 6'd0;
		verdict_pend<= 1'b0;
		deepest_q   <= 16'd0;
`endif
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
		`ifdef MISTER_OCRAM_DIAG
			rec_act <= 1'b0;
			rec_k   <= 6'd0;
			verdict_pend <= 1'b0;
		`endif
		`ifndef MISTER_OCRAM_DIAG
		`ifdef MISTER_OCRAM_SECTOR_SLOT
			rec_act <= 1'b0;
			rec_k   <= 6'd0;
		`endif
		`endif
		boot_pend  <= 1'b0;
		boot_first <= 1'b1;
		poll_pend  <= 1'b0;
		poll_step  <= 3'd0;
		rd_act     <= R_NONE;
		wr_act     <= W_NONE;
		osd_bind_q <= 1'b0;
		bind_q     <= 2'd0;
		sec_ask_q  <= 1'b0;
		bind_pend  <= 1'b0;
		c_act_v    <= 1'b0;
		if (rec_mid_q) prev_mid_q <= 1'b1;
		if (OSD_EN != 0 && c0_mid) prev_mid_osd_q <= 1'b1;
		if (TS_EN != 0 && c1_mid) prev_mid_ts_q <= 1'b1;
		if (SEC_EN != 0 && c0_smid) prev_mid_ts_q <= 1'b1;
		rec_mid_q  <= 1'b0;
	end
	else if (run_fall) begin
		// the stop: the transaction in flight completes in the engine; a record being written is abandoned and
		// its head is not stored; one terminal store follows unless the HPS reset ended the run. A record head
		// store already in flight still lands, so it is not an abandoned record.
		phase      <= P_OFF;
		if (rec_mid_q && (wr_act != W_RECHEAD)) prev_mid_q <= 1'b1;
		if (OSD_EN != 0 && c0_mid) prev_mid_osd_q <= 1'b1;
		if (TS_EN != 0 && c1_mid) prev_mid_ts_q <= 1'b1;
		if (SEC_EN != 0 && c0_smid) prev_mid_ts_q <= 1'b1;
		rec_mid_q  <= 1'b0;
		`ifdef MISTER_OCRAM_DIAG
			rec_act <= 1'b0;
			rec_k   <= 6'd0;
			verdict_pend <= 1'b0;
		`endif
		`ifndef MISTER_OCRAM_DIAG
		`ifdef MISTER_OCRAM_SECTOR_SLOT
			rec_act <= 1'b0;
			rec_k   <= 6'd0;
		`endif
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
		bind_q     <= 2'd0;
		sec_ask_q  <= 1'b0;
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
		`ifdef MISTER_OCRAM_DIAG
			rec_act <= 1'b0;
			rec_k   <= 6'd0;
			verdict_pend <= 1'b0;
		`endif
		`ifndef MISTER_OCRAM_DIAG
		`ifdef MISTER_OCRAM_SECTOR_SLOT
			rec_act <= 1'b0;
			rec_k   <= 6'd0;
		`endif
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
			J_BIND: begin wr_act <= W_BIND; bind_pend <= 1'b0; bind_q <= {c0_open, c0_sbind}; end
`ifndef MISTER_OCRAM_DIAG
`ifdef MISTER_OCRAM_SECTOR_SLOT
			J_REC: begin
				wr_act <= (rec_k == 6'd2) ? W_RECHEAD : W_REC;
				rec_k  <= rec_k + 6'd1;
			end
			J_RQ: begin
				wr_act    <= W_REC;
				rec_act   <= 1'b1;
				rec_k     <= 6'd1;
				rec_mid_q <= 1'b1;
			end
`endif
`endif
`ifdef MISTER_OCRAM_DIAG
			J_REC: begin
				if (rec_k == rec_len_q + 6'd1) begin
					wr_act <= W_RECHEAD; rec_k <= rec_k + 6'd1;
				end
				else begin
					wr_act <= W_REC; rec_k <= rec_k + 6'd1;
				end
			end
			J_MARK, J_VERDICT: begin
				wr_act    <= W_REC;
				rec_act   <= 1'b1;
				rec_k     <= 6'd1;
				rec_mid_q <= 1'b1;
				if (t_job == J_VERDICT) begin
					// the eight verdict words are one instant, registered at the record's start
					rec_kind_q <= 1'b1;
					rec_len_q  <= 6'd8;
					vd0 <= ts_records;
					vd1 <= ts_words;
					vd2 <= ts_errors;
					vd3 <= ts_first_g;
					vd4 <= ts_first_got;
					vd5 <= ts_sum;
					vd6 <= {ts_skipped, ts_deepest};
					vd7 <= {osd_rows, osd_deepest};
					verdict_pend <= 1'b0;
				end
				else begin
					rec_kind_q <= 1'b0;
					rec_len_q  <= 6'd40;
					if (!run_flood) verdict_pend <= 1'b1;
				end
			end
`endif
			default: ;
			endcase
		end
		else begin
			rd_act <= poll_rd_kind;
			if (poll_sel && (poll_rd_kind == R_HELLO) && (phase == P_START)) boot_first <= 1'b0;
		end
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
				end
				else hj_k <= hj_k + 5'd1;
			end
		end
`ifdef MISTER_OCRAM_DIAG
		else if (wr_act == W_RECHEAD) begin
			wr_act  <= W_NONE;
			head_q  <= head_q + {10'd0, rec_len_q} + 16'd1;
			rec_act <= 1'b0;
			rec_mid_q <= 1'b0;
			rec_k   <= 6'd0;
			if (((head_q + {10'd0, rec_len_q} + 16'd1) - tail_q) > deepest_q)
				deepest_q <= (head_q + {10'd0, rec_len_q} + 16'd1) - tail_q;
		end
`endif
`ifndef MISTER_OCRAM_DIAG
`ifdef MISTER_OCRAM_SECTOR_SLOT
		else if (wr_act == W_RECHEAD) begin
			wr_act    <= W_NONE;
			head_q    <= head_q + 16'd2;
			rec_act   <= 1'b0;
			rec_mid_q <= 1'b0;
			rec_k     <= 6'd0;
		end
`endif
`endif
		else if (wr_act != W_NONE) wr_act <= W_NONE;
	end

	// a hello step that this build does not store is passed without a transaction
	if (hj_act && !hj_has(hj_k)) hj_k <= hj_k + 5'd1;

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
				if (rdata == echo_q) poll_step <= (OSD_EN != 0 && !osd_bind_q) ? 3'd1 : SEC_EN != 0 ? 3'd5 : 3'd2;
				else begin
					// the session ends: its line drops, and each record part-written is recorded for the next hello
					`ifdef MISTER_OCRAM_DIAG
						if (rec_act) begin
							rec_act <= 1'b0;
							rec_k <= 6'd0;
							if (rec_mid_q) prev_mid_q <= 1'b1;
							rec_mid_q <= 1'b0;
						end
					`endif
					`ifndef MISTER_OCRAM_DIAG
					`ifdef MISTER_OCRAM_SECTOR_SLOT
						if (rec_act) begin
							rec_act <= 1'b0;
							rec_k <= 6'd0;
							if (rec_mid_q) prev_mid_q <= 1'b1;
							rec_mid_q <= 1'b0;
						end
					`endif
					`endif
					if (OSD_EN != 0 && c0_mid) prev_mid_osd_q <= 1'b1;
					if (TS_EN != 0 && c1_mid) prev_mid_ts_q <= 1'b1;
					if (SEC_EN != 0 && c0_smid) prev_mid_ts_q <= 1'b1;
					rec_mid_q <= 1'b0;
					poll_step <= 3'd0;
					sto_pend  <= 1'b1;
					sto_data  <= (rdata == 32'd0) ? C_HELLO_ARMED : C_HELLO_WAIT;
					phase     <= (rdata == 32'd0) ? P_ARMED : P_WAIT;
					osd_bind_q<= 1'b0;
					bind_q    <= 2'd0;
					sec_ask_q <= 1'b0;
					bind_pend <= 1'b0;
				end
			end
		end
		else if (rd_act == R_BIND) begin
			if (rdata[11]) osd_bind_q <= 1'b1;
			poll_step <= SEC_EN != 0 ? 3'd5 : 3'd2;
		end
		else if (rd_act == R_SECT) begin
			sec_ask_q <= rdata[0];
			poll_step <= 3'd2;
		end
		else if (rd_act == R_LINE) begin
			line_en_q <= rdata[3];
			poll_step <= 3'd3;
		end
		else if (rd_act == R_IDX) begin
			if (!stopped_q && (((head_q - rdata[15:0]) & 16'hFFFF) > EVT_WORDS)) begin
				// a host defect: bit 11 at once, the ring stops for the rest of the session
				stopped_q  <= 1'b1;
				fault_pend <= 1'b1;
				poll_step  <= 3'd0;
				`ifdef MISTER_OCRAM_DIAG
					if (rec_act) begin
						rec_act <= 1'b0;
						rec_k <= 6'd0;
						if (rec_mid_q) prev_mid_q <= 1'b1;
						rec_mid_q <= 1'b0;
					end
				`endif
				`ifndef MISTER_OCRAM_DIAG
				`ifdef MISTER_OCRAM_SECTOR_SLOT
					if (rec_act) begin
						rec_act <= 1'b0;
						rec_k <= 6'd0;
						if (rec_mid_q) prev_mid_q <= 1'b1;
						rec_mid_q <= 1'b0;
					end
				`endif
				`endif
			end
			else if (!stopped_q) begin
				tail_q <= rdata[15:0];
`ifdef MISTER_OCRAM_DIAG
				if (((head_q - rdata[15:0]) & 16'hFFFF) > deepest_q)
					deepest_q <= (head_q - rdata[15:0]) & 16'hFFFF;
`endif
				poll_step <= (line_en_q && (head_q != rdata[15:0])) ? 3'd4 : 3'd0;
			end
			else begin
				tail_q <= rdata[15:0];
				poll_step <= 3'd0;
			end
		end
	end

	// a consumer transaction's handshake: its port sees the done, the arbiter turns to the other port next time
	if (t_done && c_act_v) c_act_v <= 1'b0;
`ifdef MISTER_OCRAM_DIAG
	if (c0_go || c1_go) begin
		c_act_v    <= 1'b1;
		c_act_port <= c1_go;
		c_act_we   <= c1_go ? c1_we : c0_we;
		c_last     <= c1_go;
	end
`else
	if (c0_go) begin
		c_act_v    <= 1'b1;
		c_act_port <= 1'b0;
		c_act_we   <= c0_we;
		c_last     <= 1'b0;
	end
`endif

	// the EVT line: high in a session, allowed by H_LINE, the ring not stopped, the head not the tail
	line <= (phase == P_SESSION) && line_en_q && !stopped_q && (head_q != tail_q);
end

`ifdef MISTER_OCRAM_DIAG
hps_ocram_diag diag
(
	.clk        (clk),
	.clr        (diag_clr),
	.mark_start (mark_start),
	.snap_ready (snap_ready),
	.mark_i     (rec_k - 6'd1),
	.mark_word  (mark_word),
	.counted    (counted_q),
	.awvalid    (awvalid),
	.arvalid    (arvalid),
	.bvalid     (bvalid),
	.bready     (bready),
	.bresp      (bresp),
	.rvalid     (rvalid),
	.rready     (rready),
	.rresp      (rresp),
	.drop       (drop_q),
	.deepest    (deepest_q),
	.ns         (run_ns),
	.flood      (run_flood),
	.hello_cnt  (hello_cnt)
);
`endif

endmodule

`ifdef MISTER_OCRAM_DIAG
//============================================================================
//  hps_ocram_tsink -- the DIAG host-to-fabric torture sink (MISTER_OCRAM_DIAG only)
//
//  A host-to-core ring, r = 4, in the SECTOR span (base 0x7000, 8,192 words), consumed on clk_100m through the
//  master's consumer port 1 in every session, with no bind. Records are {kind 0x50 TSEQ, seq[7:0], len[15:0]}
//  then len payload words, where payload word g (the count of payload words since the hello, from 0) must equal
//  v(g) = g ^ {g[15:0], g[31:16]} ^ 0x9E3779B9, and seq is the record's number since the hello, mod 256. A kind
//  other than 0x50 is skipped by its length and counted as skipped. A head more than the ring's size ahead, or a
//  length past the head, stops the ring for the session and pulses c_fault (C_STATE bit 12).
//
//  Counters, cleared at the hello: records (complete TSEQ records), words and the wrap-around sum of every
//  payload word read, errors (bad words and bad seqs), first_g and first_got (the first bad word's global index
//  and value; 0 until one), skipped and the deepest (head - tail) read. They are the VERDICT record's payload,
//  which the master snapshots at the record's start: {kind 0x25, arg 4, len 8}, posted by the EVT writer after
//  the MARK a period starts, with {osd_rows, osd_deepest} in its last word.
//
//  Address bound: H_IDX[4] or TS_BASE + ((index & (TS_WORDS - 1)) << 2); C_IDX[4] for its tail. Both controls
//  are below 0x100 and the span ends at 0xF000; a generate-time guard stops elaboration otherwise.
//============================================================================

/* verilator lint_off DECLFILENAME */   // the sink shares this file so the fit reports its ALMs apart
module hps_ocram_tsink
(
	input  wire        clk,          // clk_100m
	input  wire        s_hello,      // one cycle: the hello's first store is about to run
	input  wire        s_live,       // P_SESSION
	input  wire        s_tick,       // the poll period's pulse
	output wire        c_rest,       // the walker is idle: the half is at rest
	output wire        c_fault,      // one cycle: the ring stopped on a fault
	output wire        c_mid,        // level: a record is part-read

	output wire        c_req,
	output wire        c_we,
	output wire [15:0] c_off,
	output wire [31:0] c_wdata,
	input  wire        c_grant,
	input  wire        c_done,
	input  wire        c_ok,
	input  wire [31:0] c_rdata,

	output wire [31:0] v_records,
	output wire [31:0] v_words,
	output wire [31:0] v_errors,
	output wire [31:0] v_first_g,
	output wire [31:0] v_first_got,
	output wire [31:0] v_sum,
	output wire [15:0] v_skipped,
	output wire [15:0] v_deepest
);

localparam [15:0] H_IDX4 = 16'h0020;       // host: the torture ring's head
localparam [15:0] C_IDX4 = 16'h00A0;       // fabric: the torture ring's tail
localparam [15:0] TS_BASE = 16'h7000;      // the ring's first word
localparam [15:0] TS_WORDS = 16'd8192;

generate
	if (TS_BASE + TS_WORDS * 4 > 16'hF000) begin : g_ts_span
		ocram_ts_span_above_0xF000 guard();
	end
	if (H_IDX4 >= 16'h0100 || C_IDX4 >= 16'h0100) begin : g_ts_ctrl
		ocram_ts_control_constant_above_0xFF guard();
	end
endgenerate

localparam [2:0] T_IDLE = 3'd0, T_HEAD = 3'd1, T_HDR = 3'd2, T_PAY = 3'd3, T_TAIL = 3'd4, T_STOP = 3'd5;

reg  [2:0]  tst = T_IDLE;
reg         req = 1'b0;
reg         req_we = 1'b0;
reg  [15:0] req_off = 16'd0;
reg  [31:0] req_wdata = 32'd0;
reg  [15:0] tail = 16'd0;
reg  [15:0] head_c = 16'd0;
reg         head_due = 1'b1;
reg  [15:0] rec_len = 16'd0;
reg  [15:0] pay_i = 16'd0;             // the payload word being read, 0..len-1
reg         mid_q = 1'b0;
reg         fault_q = 1'b0;
reg         tseq = 1'b0;               // the record being read is a TSEQ
reg  [7:0]  exp_seq = 8'd0;
reg  [31:0] g = 32'd0;
reg  [31:0] records = 32'd0, words = 32'd0, errors = 32'd0;
reg  [31:0] first_g = 32'd0, first_got = 32'd0, sum = 32'd0;
reg         got_bad = 1'b0;
reg  [15:0] skipped = 16'd0, deepest = 16'd0;

assign c_req    = req;
assign c_we     = req_we;
assign c_off    = req_off;
assign c_wdata  = req_wdata;
assign c_fault  = fault_q;
assign c_mid    = mid_q;
assign c_rest   = (tst == T_IDLE) && !req;
assign v_records = records;
assign v_words   = words;
assign v_errors  = errors;
assign v_first_g = first_g;
assign v_first_got = first_got;
assign v_sum     = sum;
assign v_skipped = skipped;
assign v_deepest = deepest;

wire [15:0] pay1_idx = tail + 16'd1;
wire [15:0] pay2_idx = tail + 16'd1 + pay_i + 16'd1;
wire [15:0] hdr_off  = TS_BASE + ((tail & (TS_WORDS - 1)) << 2);
wire [15:0] pay1_off = TS_BASE + ((pay1_idx & (TS_WORDS - 1)) << 2);
wire [15:0] pay2_off = TS_BASE + ((pay2_idx & (TS_WORDS - 1)) << 2);
wire [15:0] head_gap = head_c - tail;
wire [15:0] new_gap  = d_data_q[15:0] - tail;          // every decision is one cycle after the response
wire [16:0] hdr_words = {1'b0, d_data_q[15:0]} + 17'd1;
wire [16:0] gap_words = {1'b0, head_gap};
wire [31:0] v_expect  = g ^ {g[15:0], g[31:16]} ^ 32'h9E37_79B9;

// The consumer's read word is registered before any decision: the fpga2hps atom's rdata reaches d_data_q and
// nothing else in the response cycle, so the header compare, the length arithmetic and the next offset sit on
// registers. The state machine acts one cycle after c_done, on that copy; a test sink can afford the cycle.
reg         d_v_q = 1'b0;              // a completion is held for this cycle
reg         d_ok_q = 1'b0;
reg  [31:0] d_data_q = 32'd0;

always @(posedge clk) begin
	fault_q <= 1'b0;
	d_v_q   <= 1'b0;
	if (c_done) begin
		d_v_q    <= 1'b1;
		d_ok_q   <= c_ok;
		d_data_q <= c_rdata;
	end

	if (s_hello) begin
		tst     <= T_IDLE;
		req     <= 1'b0;
		tail    <= 16'd0;
		head_c  <= 16'd0;
		head_due<= 1'b1;
		mid_q   <= 1'b0;
		tseq    <= 1'b0;
		exp_seq <= 8'd0;
		g       <= 32'd0;
		records <= 32'd0;
		words   <= 32'd0;
		errors  <= 32'd0;
		first_g <= 32'd0;
		first_got <= 32'd0;
		sum     <= 32'd0;
		got_bad <= 1'b0;
		skipped <= 16'd0;
		deepest <= 16'd0;
		pay_i   <= 16'd0;
		d_v_q   <= 1'b0;               // a completion of the ending session is not acted on
	end
	else begin
		if (s_tick) head_due <= 1'b1;

		case (tst)
		T_IDLE: if (s_live) begin
			if (head_due && (head_c == tail)) begin
				req      <= 1'b1;
				req_we   <= 1'b0;
				req_off  <= H_IDX4;
				head_due <= 1'b0;
				tst      <= T_HEAD;
			end
			else if (head_gap != 16'd0) begin
				req     <= 1'b1;
				req_we  <= 1'b0;
				req_off <= hdr_off;
				mid_q   <= 1'b1;
				tst     <= T_HDR;
			end
		end
		T_HEAD: begin
			if (!s_live) begin req <= 1'b0; tst <= T_IDLE; end
			else if (c_grant) req <= 1'b0;
			else if (d_v_q) begin
				if (!d_ok_q) req <= 1'b1;
				else if (new_gap > TS_WORDS) begin
					// a head more than the ring's size ahead: a host defect
					fault_q   <= 1'b1;
					req       <= 1'b0;
					mid_q     <= 1'b0;
					tst       <= T_STOP;
				end
				else begin
					head_c <= d_data_q[15:0];
					if (new_gap > deepest) deepest <= new_gap;
					if (d_data_q[15:0] == tail) tst <= T_IDLE;
					else begin
						req     <= 1'b1;
						req_we  <= 1'b0;
						req_off <= hdr_off;
						mid_q   <= 1'b1;
						tst     <= T_HDR;
					end
				end
			end
		end
		T_HDR: begin
			if (!s_live) begin req <= 1'b0; tst <= T_IDLE; end
			else if (c_grant) req <= 1'b0;
			else if (d_v_q) begin
				if (!d_ok_q) begin
					req <= 1'b1;
				end
				else if (hdr_words > gap_words || hdr_words > {1'b0, TS_WORDS}) begin
					// a length past the head or the ring: a host defect
					fault_q   <= 1'b1;
					req       <= 1'b0;
					mid_q     <= 1'b0;
					tst       <= T_STOP;
				end
				else if (d_data_q[31:24] != 8'h50) begin
					// an unknown kind: skipped by its length, counted
					skipped   <= skipped + 16'd1;
					tail      <= tail + 16'd1 + d_data_q[15:0];
					req       <= 1'b1;
					req_we    <= 1'b1;
					req_off   <= C_IDX4;
					req_wdata <= {16'd0, tail + 16'd1 + d_data_q[15:0]};
					tseq      <= 1'b0;
					tst       <= T_TAIL;
				end
				else begin
					if (d_data_q[23:16] != exp_seq) errors <= errors + 32'd1;
					exp_seq <= exp_seq + 8'd1;
					rec_len <= d_data_q[15:0];
					pay_i   <= 16'd0;
					tseq    <= 1'b1;
					req     <= 1'b1;
					req_we  <= 1'b0;
					if (d_data_q[15:0] == 16'd0) begin
						// a TSEQ with no payload words: the header alone is the record
						tail      <= tail + 16'd1;
						req_we    <= 1'b1;
						req_off   <= C_IDX4;
						req_wdata <= {16'd0, tail + 16'd1};
						tst       <= T_TAIL;
					end
					else begin
						req_off <= pay1_off;
						tst     <= T_PAY;
					end
				end
			end
		end
		T_PAY: begin
			if (!s_live) begin req <= 1'b0; tst <= T_IDLE; end
			else if (c_grant) req <= 1'b0;
			else if (d_v_q) begin
				if (!d_ok_q) req <= 1'b1;
				else if (!s_live) tst <= T_IDLE;
				else begin
					words <= words + 32'd1;
					sum   <= sum + d_data_q;
					if (d_data_q != v_expect) begin
						errors <= errors + 32'd1;
						if (!got_bad) begin
							first_g   <= g;
							first_got <= d_data_q;
							got_bad   <= 1'b1;
						end
					end
					g <= g + 32'd1;
					if (pay_i == rec_len - 16'd1) begin
						tail      <= tail + 16'd1 + rec_len;
						req       <= 1'b1;
						req_we    <= 1'b1;
						req_off   <= C_IDX4;
						req_wdata <= {16'd0, tail + 16'd1 + rec_len};
						tst       <= T_TAIL;
					end
					else begin
						pay_i   <= pay_i + 16'd1;
						req     <= 1'b1;
						req_we  <= 1'b0;
						req_off <= pay2_off;
					end
				end
			end
		end
		T_TAIL: begin
			if (!s_live) begin
				req <= 1'b0;                            // the session ended: the tail is not stored
				tst <= T_IDLE;
			end
			else if (c_grant) begin
				req   <= 1'b0;
				mid_q <= 1'b0;
				if (tseq) records <= records + 32'd1;
				head_due <= 1'b1;
			end
			else if (d_v_q && !d_ok_q) req <= 1'b1;
			else if (d_v_q) tst <= T_IDLE;
		end
		T_STOP: ;
		default: tst <= T_IDLE;
		endcase
	end
end

endmodule
/* verilator lint_on DECLFILENAME */
`endif

`ifdef MISTER_OCRAM_DIAG
//============================================================================
//  hps_ocram_diag -- the MARK record's counters (MISTER_OCRAM_DIAG only)
//
//  A counted transaction's latency is timed by a per-direction counter that starts at its address handshake
//  and advances one a clk, so no 32-bit subtract sits in front of the histogram logic. The accounting is
//  pipelined in three registered stages, one transaction in each:
//    A  the R/B handshake's edge: the latency, the error flag, the direction, and a tag saying whether the
//       transaction handed off at or before the snapshot request;
//    B  the next edge: the latency's histogram bin, its 16-bit saturated form, and the flags;
//    C  the next edge: every counter of that transaction together -- count, sum, min, max, errors and its
//       histogram bin -- so a snapshot never holds half a transaction.
//  The snapshot is requested by the MARK record's start; it completes when A and B hold no transaction tagged
//  older than the request, so it has every counted handshake at or before the request's edge and none after.
//  snap_ready then lets the master send the record's first payload word. A hello empties the pipeline and the
//  counters; an HPS reset drops only the transaction it orphaned.
//  Every array is logic (no block memory), and each histogram has one adder and one read mux with a decoded
//  write enable.
//============================================================================

/* verilator lint_off DECLFILENAME */   // the sheet keeps the counters in this file so the fit reports their ALMs apart
module hps_ocram_diag
(
	input  wire        clk,
	input  wire        clr,          // a hello: clear every counter and empty the pipeline
	input  wire        mark_start,   // a MARK record begins at this edge: request the snapshot
	output reg         snap_ready,   // the snapshot is complete; payload word 1 may be sent
	input  wire  [5:0] mark_i,       // payload word 0-39
	output reg  [31:0] mark_word,

	input  wire        counted,      // the transaction launched is a counted one
	input  wire        awvalid,
	input  wire        arvalid,
	input  wire        bvalid,
	input  wire        bready,
	input  wire  [1:0] bresp,
	input  wire        rvalid,
	input  wire        rready,
	input  wire  [1:0] rresp,
	input  wire        drop,         // the HPS reset dropped an unanswered transaction

	input  wire [15:0] deepest,
	input  wire        ns,
	input  wire        flood,
	input  wire  [7:0] hello_cnt
);

// The latency: one more each clk from the address handshake's cycle; the handshake cycle's value is the
// number of edges between ARVALID's/AWVALID's first rise and the R/B handshake, as before.
reg [31:0] w_age = 32'd0, r_age = 32'd0;
reg        w_meas = 1'b0, r_meas = 1'b0;

reg aw_d = 1'b0, ar_d = 1'b0;
wire aw_rise = awvalid & ~aw_d;
wire ar_rise = arvalid & ~ar_d;
always @(posedge clk) begin aw_d <= awvalid; ar_d <= arvalid; end

wire b_fire = bvalid & bready;
wire r_fire = rvalid & rready;
wire w_cap = b_fire & w_meas;
wire r_cap = r_fire & r_meas;

// stage A registers
reg        a_v = 1'b0, a_we = 1'b0, a_err = 1'b0, a_old = 1'b1;
reg [31:0] a_lat = 32'd0;
// stage B registers
reg        b_v = 1'b0, b_we = 1'b0, b_err = 1'b0, b_old = 1'b1;
reg [31:0] b_lat = 32'd0;
reg [15:0] b_lat16 = 16'd0;
reg  [3:0] b_bin = 4'd0;

// counts, sums, extrema and errors
reg [31:0] rd_n = 32'd0, wr_n = 32'd0;
reg [31:0] rd_sum = 32'd0, wr_sum = 32'd0;
reg [15:0] rd_max = 16'd0, wr_max = 16'd0, rd_min = 16'hFFFF, wr_min = 16'hFFFF;
reg [15:0] rd_err = 16'd0, wr_err = 16'd0;

(* ramstyle = "logic" *) reg [31:0] rd_hist [0:15];
(* ramstyle = "logic" *) reg [31:0] wr_hist [0:15];
// The snapshot a MARK record reads, word by word: word 0 carries the ring's deepest fill, the DIAG knobs and
// C_STATE's hello count; words 1-7 the counters; 8-23 and 24-39 the two histograms.
(* ramstyle = "logic" *) reg [31:0] snap [0:39];

// bins (cycles): 0-7, 8-15, ... 56-63, 64-127, 128-255, 256-511, 512-1023, 1024-2047, 2048-4095,
// 4096-65535, 65536 and more. Each boundary is a power of two, so the decode is bit tests, not compares.
/* verilator lint_off UNUSEDSIGNAL */   // bins 0-7 are 8 cycles wide, so bits [2:0] select nothing
function [3:0] bin_of(input [31:0] lat);
	begin
		if (|lat[31:17] || lat[16]) bin_of = 4'd15;
		else if (|lat[15:12])       bin_of = 4'd14;
		else if (lat[11])           bin_of = 4'd13;
		else if (lat[10])           bin_of = 4'd12;
		else if (lat[9])            bin_of = 4'd11;
		else if (lat[8])            bin_of = 4'd10;
		else if (lat[7])            bin_of = 4'd9;
		else if (lat[6])            bin_of = 4'd8;
		else                        bin_of = {1'b0, lat[5:3]};
	end
endfunction
/* verilator lint_on UNUSEDSIGNAL */

function [15:0] sat16(input [31:0] lat);
	sat16 = (|lat[31:16]) ? 16'hFFFF : lat[15:0];
endfunction

wire [31:0] w_inc = wr_hist[b_bin] + 32'd1;
wire [31:0] r_inc = rd_hist[b_bin] + 32'd1;

// the snapshot request; snap_ready is its completion level
reg snap_req = 1'b0;

integer hi;
always @(posedge clk) begin
	if (clr) begin
		rd_n <= 32'd0; wr_n <= 32'd0;
		rd_sum <= 32'd0; wr_sum <= 32'd0;
		rd_max <= 16'd0; wr_max <= 16'd0; rd_min <= 16'hFFFF; wr_min <= 16'hFFFF;
		rd_err <= 16'd0; wr_err <= 16'd0;
		for (hi = 0; hi < 16; hi = hi + 1) begin
			rd_hist[hi] <= 32'd0;
			wr_hist[hi] <= 32'd0;
		end
		a_v <= 1'b0; b_v <= 1'b0;
		w_meas <= 1'b0; r_meas <= 1'b0;
		snap_req <= 1'b0; snap_ready <= 1'b0;
	end
	else begin
		// stage C: all of B's counters land at this edge
		if (b_v && b_we) begin
			wr_n   <= wr_n + 32'd1;
			wr_sum <= wr_sum + b_lat;
			if (b_lat16 > wr_max) wr_max <= b_lat16;
			if (b_lat16 < wr_min) wr_min <= b_lat16;
			if (b_err && (wr_err != 16'hFFFF)) wr_err <= wr_err + 16'd1;
			for (hi = 0; hi < 16; hi = hi + 1)
				if (b_bin == hi[3:0]) wr_hist[hi] <= w_inc;
		end
		else if (b_v) begin
			rd_n   <= rd_n + 32'd1;
			rd_sum <= rd_sum + b_lat;
			if (b_lat16 > rd_max) rd_max <= b_lat16;
			if (b_lat16 < rd_min) rd_min <= b_lat16;
			if (b_err && (rd_err != 16'hFFFF)) rd_err <= rd_err + 16'd1;
			for (hi = 0; hi < 16; hi = hi + 1)
				if (b_bin == hi[3:0]) rd_hist[hi] <= r_inc;
		end

		// stage B: the bin, the saturated latency and the flags
		b_v     <= a_v;
		b_we    <= a_we;
		b_lat   <= a_lat;
		b_lat16 <= sat16(a_lat);
		b_bin   <= bin_of(a_lat);
		b_err   <= a_err;
		b_old   <= a_old;

		// stage A: the handshake, and the latency counters that feed it
		a_v <= 1'b0;
		if (w_cap) begin
			a_v <= 1'b1; a_we <= 1'b1; a_lat <= w_age; a_err <= (bresp != 2'b00); a_old <= ~snap_req;
			w_meas <= 1'b0;
		end
		else if (aw_rise && counted) begin
			w_meas <= 1'b1; w_age <= 32'd1;
		end
		else if (w_meas) w_age <= w_age + 32'd1;
		if (r_cap) begin
			a_v <= 1'b1; a_we <= 1'b0; a_lat <= r_age; a_err <= (rresp != 2'b00); a_old <= ~snap_req;
			r_meas <= 1'b0;
		end
		else if (ar_rise && counted) begin
			r_meas <= 1'b1; r_age <= 32'd1;
		end
		else if (r_meas) r_age <= r_age + 32'd1;
		if (drop) begin w_meas <= 1'b0; r_meas <= 1'b0; end

		// the snapshot: requested at the record's start, taken once no older transaction is in A or B. A
		// transaction that hands off at the request's own edge is in A by the next, so this edge cannot take it.
		if (mark_start) begin snap_req <= 1'b1; snap_ready <= 1'b0; end
		if (snap_req && !((a_v && a_old) || (b_v && b_old))) begin
			snap[0] <= {hello_cnt, 6'd0, flood, ns, deepest};
			snap[1] <= rd_n;
			snap[2] <= wr_n;
			snap[3] <= {rd_max, rd_min};
			snap[4] <= {wr_max, wr_min};
			snap[5] <= rd_sum;
			snap[6] <= wr_sum;
			snap[7] <= {rd_err, wr_err};
			for (hi = 0; hi < 16; hi = hi + 1) begin
				snap[8  + hi] <= rd_hist[hi];
				snap[24 + hi] <= wr_hist[hi];
			end
			snap_req <= 1'b0;
			snap_ready <= 1'b1;
		end
	end
end

integer mi;
always @(*) begin
	mark_word = 32'd0;
	for (mi = 0; mi < 40; mi = mi + 1)
		if (mark_i == mi[5:0]) mark_word = snap[mi];
end

endmodule
/* verilator lint_on DECLFILENAME */
`endif

`default_nettype wire
`endif
`undef MISTER_OCRAM_MARK
`undef MISTER_OCRAM_POLL
