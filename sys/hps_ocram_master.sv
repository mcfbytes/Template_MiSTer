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

// The floor of the OCRAM mailbox: the AXI3 master on the fpga2hps bridge, its run latch, the poller, the hello
// and the EVT ring's writer and interrupt line. The design note is the law, spec/ocram-mailbox-design.md (the
// Template's reader-facing copy is docs/OCRAM_MAILBOX.md); this file is layout 1 only.
//
//  * It runs on clk_100m and no fabric reset holds it. It never abandons a transaction it started: a core reset,
//    a stopped clk_sys or a host that has gone cannot leave one in flight.
//  * It cannot address anything but the mailbox: every address is {16'hFFFF, off}, and off is a localparam
//    (below 0x100) or an EVT-ring word, EVT_BASE + ((index & (EVT_WORDS - 1)) << 2). tb/static.sh holds it.
//  * One transaction at a time, single beat, full 32-bit words. Write: AWVALID and WVALID together, each held
//    with its payload until its ready, then BREADY while the response is awaited. Read: ARVALID held until
//    ARREADY, then RREADY while the response is awaited.
//  * The run latch (§2.6): it sets on a rising edge of GPO bit 28 (RUN) seen while GPO bit 31 is 1 and no HPS
//    reset is seen; it clears on RUN low, bit 31 low or an HPS reset, and bit 31's return never sets it. Every GPO bit
//    and hps_rst_n enters through an lw_arst cell, never raw.
//  * On a stop: the transaction in flight completes, a record being written is abandoned (its head is not
//    stored), then one terminal store C_HELLO = 0x51540000. An HPS reset ends a run with no terminal store.
//  * An HPS reset makes the master start nothing; a transaction in flight keeps its valids and readies as AXI
//    requires and takes its response if one comes; in the cycle the reset level falls, one still unanswered is
//    dropped.
//  * MARK records, the counters that fill them and the GPO 27/26 test knobs exist only with MISTER_OCRAM_DIAG;
//    hps_ocram_diag is a separate module so the fit reports its ALMs apart.
//
// Address map, layout 1 (offsets from 0xFFFF0000): the host half of the control page H_HELLO, H_LINE,
// H_IDX[3]; the fabric half C_HELLO, C_VERSION, C_CAPS, C_STATE, C_IDX[3], C_BIND, C_RING[3]; the EVT ring at
// EVT_BASE, 2,048 words. Every other word is never read and never written.

`ifndef MISTER_OCRAM_POLL
`define MISTER_OCRAM_POLL 12
`endif
`ifndef MISTER_OCRAM_MARK
`define MISTER_OCRAM_MARK 20
`endif

module hps_ocram_master
(
	input  wire        clk,          // clk_100m == h2f_user0_clk
	input  wire        hps_rst_n,    // the HPS's fabric reset (h2f_rst_n), asynchronous
	/* verilator lint_off UNUSEDSIGNAL */   // the floor reads bits 31, 28 and (DIAG) 27, 26; GPO's other users own the rest
	input  wire [31:0] gp_out,       // raw GPO: bits 31 and 28, and with DIAG 27 and 26
	/* verilator lint_on UNUSEDSIGNAL */

	output wire        present,      // the constant 1: this floor is built
	output reg         line,         // EVT ring line ordinal 4 (f2h_irq[11]), a clk register

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
localparam [15:0] H_LINE  = 16'h0008;      // host: bit 3 lets the EVT line rise
localparam [15:0] H_IDX3  = 16'h001C;      // host: the EVT ring's tail
localparam [15:0] C_HELLO = 16'h0080;      // fabric: the master's state
localparam [15:0] C_VERSION = 16'h0084;    // fabric: {major, minor} = 1.0
localparam [15:0] C_CAPS  = 16'h0088;      // fabric: bit 3 = the EVT ring is built
localparam [15:0] C_STATE = 16'h008C;      // fabric: hello count, fault, previous session mid-record
localparam [15:0] C_IDX3  = 16'h009C;      // fabric: the EVT ring's head
localparam [15:0] C_BIND  = 16'h00A4;      // fabric: the floor binds nothing
localparam [15:0] C_RING3 = 16'h00BC;      // fabric: the EVT ring's descriptor
localparam [15:0] EVT_BASE = 16'h4000;     // fabric: EVT word k at EVT_BASE + ((head + k) & 2047) * 4
localparam [15:0] EVT_WORDS = 16'd2048;

localparam [31:0] C_HELLO_ARMED = 32'h4F43_0000;
localparam [31:0] C_HELLO_WAIT  = 32'h5754_0000;
localparam [31:0] C_HELLO_STOP  = 32'h5154_0000;
localparam [15:0] HELLO_MAGIC   = 16'h4F43;
localparam [31:0] C_VERSION_1_0 = 32'h0001_0000;
localparam [31:0] C_CAPS_EVT    = 32'h0000_0008;
localparam [31:0] C_RING3_EVT   = {5'd4, 5'd4, 4'd11, 4'd0, 14'h1000};   // EVT, line 4, 2048 words, 0x1000
localparam [31:0] MARK_HEADER   = {8'h23, 8'h03, 16'd40};                // kind MARK, arg 3, len 40
localparam integer POLL = `MISTER_OCRAM_POLL;

// A static assertion: an address that could leave the mailbox stops elaboration.
generate
	if (EVT_BASE + EVT_WORDS * 4 > 16'hF000) begin : g_evt_span
		ocram_evt_span_above_0xF000 guard();
	end
	if (H_HELLO >= 16'h0100 || H_LINE >= 16'h0100 || H_IDX3 >= 16'h0100 ||
	    C_HELLO >= 16'h0100 || C_VERSION >= 16'h0100 || C_CAPS >= 16'h0100 ||
	    C_STATE >= 16'h0100 || C_IDX3 >= 16'h0100 || C_BIND >= 16'h0100 ||
	    C_RING3 >= 16'h0100) begin : g_ctrl_span
		ocram_control_constant_above_0xFF guard();
	end
endgenerate

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
reg         counted_q = 1'b0;
reg         drop_q = 1'b0;

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

// A run's phase. P_HELLO covers the hello's stores, which run in order and in one piece; P_SESSION is a
// session with the hello echoed.
localparam [2:0] P_OFF = 3'd0, P_START = 3'd1, P_ARMED = 3'd2, P_WAIT = 3'd3,
                 P_SESSION = 3'd4, P_HELLO = 3'd5;

localparam [1:0] R_NONE = 2'd0, R_HELLO = 2'd1, R_LINE = 2'd2, R_IDX = 2'd3;
localparam [2:0] W_NONE = 3'd0, W_HELLO = 3'd1, W_ONE = 3'd2, W_REC = 3'd3,
                 W_RECHEAD = 3'd4, W_TERM = 3'd5, W_FAULT = 3'd6;

reg  [2:0]  phase = P_OFF;
reg  [7:0]  hello_cnt = 8'd0;          // hellos taken since configuration, no reset; counts each echo
reg  [31:0] echo_q = 32'd0;            // the H_HELLO value this session echoed
reg  [15:0] head_q = 16'd0;            // the EVT ring's head, last published
reg  [15:0] tail_q = 16'd0;            // the ring's tail, last read
reg         line_en_q = 1'b0;          // H_LINE bit 3 at its last read
reg         stopped_q = 1'b0;          // the ring stopped on a fault this session
reg         prev_mid_q = 1'b0;         // the session before this one ended mid-record
reg         rec_mid_q = 1'b0;          // a record is being written and not yet published
`ifdef MISTER_OCRAM_DIAG
reg  [15:0] deepest_q = 16'd0;         // deepest EVT waiting this session, for MARK word 0
`endif
reg  [2:0]  poll_step = 3'd0;          // session poll: 2 H_LINE, 3 H_IDX, 4 another H_IDX while line high
reg         poll_pend = 1'b0;          // a poll period has passed
reg         boot_pend = 1'b0;          // the run's first H_HELLO read is not taken yet
reg         boot_first = 1'b1;         // it is tried at once; a failed one waits for the next poll
reg         term_pend = 1'b0;          // the terminal store of a stopped run is owed
reg         sto_pend = 1'b0;           // one C_HELLO state store is owed
reg  [31:0] sto_data = 32'd0;
reg         fault_pend = 1'b0;         // C_STATE bit 11 is owed "at once"
reg         hj_act = 1'b0;             // the hello's stores
reg  [2:0]  hj_k = 3'd0;
reg  [31:0] hj_state = 32'd0;          // C_STATE as stored at this hello
reg  [1:0]  rd_act = R_NONE;           // the read in flight
reg  [2:0]  wr_act = W_NONE;           // the write in flight
localparam [POLL-1:0] POLL_T0 = {POLL{1'b0}};
localparam [POLL-1:0] POLL_ONE = 1;
reg  [POLL-1:0] poll_t = {POLL{1'b0}}; // free-running; its period is 2^POLL cycles
wire poll_tick = (poll_t == POLL_T0);

always @(posedge clk) poll_t <= run_set ? POLL_T0 : poll_t + POLL_ONE;   // a run's polls count from its start

// C_STATE: [7:0] hellos taken, [11] the EVT ring stopped on a fault this session, [19] the previous session
// ended with an EVT record part-written; every other bit 0. cnt is the hello count the word reports: the
// incremented one at a hello, the current one at a fault.
function [31:0] state_word(input [7:0] cnt, input fault, input mid);
	begin
		state_word = {24'd0, cnt};
		if (mid)   state_word[19] = 1'b1;
		if (fault) state_word[11] = 1'b1;
	end
endfunction

// The hello's stores, in their fixed order.
function [31:0] hello_data(input [2:0] k);
	begin
		case (k)
		3'd0: hello_data = C_VERSION_1_0;
		3'd1: hello_data = C_CAPS_EVT;
		3'd2: hello_data = hj_state;
		3'd3: hello_data = 32'd0;
		3'd4: hello_data = 32'd0;
		3'd5: hello_data = C_RING3_EVT;
		default: hello_data = echo_q;
		endcase
	end
endfunction

// The hello the read found: a valid H_HELLO in ARMED starts the session's stores. diag_clr clears the DIAG
// counters at the same edge.
wire hello_hit = t_done && (rd_act == R_HELLO) && t_ok && (phase == P_ARMED) &&
                 (rdata[31:16] == HELLO_MAGIC) && (rdata[15:0] != 16'd0);
`ifdef MISTER_OCRAM_DIAG
wire diag_clr = hello_hit;

localparam integer MARKW = `MISTER_OCRAM_MARK;
localparam [MARKW-1:0] MARK_T0 = {MARKW{1'b0}};
localparam [MARKW-1:0] MARK_ONE = 1;
reg  [5:0]  rec_k = 6'd0;              // 0 header, 1..40 payload, 41 the head store
reg         rec_act = 1'b0;
reg         mark_due_q = 1'b0;
reg  [MARKW-1:0] mark_t = {MARKW{1'b0}};
wire [15:0] wait_now = head_q - tail_q;
wire [31:0] mark_word;
`endif

// The read the poll (or the boot, or the line) wants next.
reg  poll_sel;
reg  [1:0] poll_rd_kind;

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
		else if (poll_step == 3'd2) begin
			poll_sel = 1'b1; poll_rd_kind = R_LINE;
		end
		else if (poll_step == 3'd3 || poll_step == 3'd4) begin
			poll_sel = 1'b1; poll_rd_kind = R_IDX;
		end
	end
end

`ifdef MISTER_OCRAM_DIAG
// A MARK has room when 2,048 - waiting holds its 41 words. It starts when flood is set or the period since the
// last one began has passed. A stopped ring posts nothing.
wire [15:0] evt_index = head_q + {10'd0, rec_k};
wire [15:0] evt_off = EVT_BASE + ((evt_index & (EVT_WORDS - 1)) << 2);
wire ring_room = !stopped_q && (wait_now <= EVT_WORDS) && ((EVT_WORDS - wait_now) >= 16'd41);
wire mark_ok = (phase == P_SESSION) && ring_room && !rec_act && (run_flood || mark_due_q);
`endif

// The arbiter: the terminal store, the fault's C_STATE store, the poll's or the line's reads when due, the
// hello's stores, a state store, then the EVT record's words.
always @(*) begin
	t_go = 1'b0; t_we = 1'b0; t_off = H_HELLO; t_wdata = 32'd0; t_counted = 1'b1;
	if (engine_idle && !hrst_q && !run_fall && !run_set) begin
		if (term_pend) begin
			t_go = 1'b1; t_we = 1'b1; t_off = C_HELLO; t_wdata = C_HELLO_STOP; t_counted = 1'b0;
		end
		else if (fault_pend) begin
			t_go = 1'b1; t_we = 1'b1; t_off = C_STATE; t_wdata = state_word(hello_cnt, 1'b1, prev_mid_q);
		end
		else if (poll_sel) begin
			t_go = 1'b1; t_we = 1'b0;
			case (poll_rd_kind)
			R_LINE:  t_off = H_LINE;
			R_IDX:   t_off = H_IDX3;
			default: t_off = H_HELLO;
			endcase
		end
		else if (hj_act) begin
			t_go = 1'b1; t_we = 1'b1;
			case (hj_k)
			3'd0: t_off = C_VERSION;
			3'd1: t_off = C_CAPS;
			3'd2: t_off = C_STATE;
			3'd3: t_off = C_IDX3;
			3'd4: t_off = C_BIND;
			3'd5: t_off = C_RING3;
			default: t_off = C_HELLO;
			endcase
			t_wdata = hello_data(hj_k);
		end
		else if (sto_pend) begin
			t_go = 1'b1; t_we = 1'b1; t_off = C_HELLO; t_wdata = sto_data;
		end
`ifdef MISTER_OCRAM_DIAG
		else if (rec_act) begin
			t_go = 1'b1; t_we = 1'b1;
			if (rec_k == 6'd41) begin
				t_off = C_IDX3; t_wdata = {16'd0, head_q + 16'd41}; t_counted = 1'b1;
			end
			else begin
				t_off = evt_off; t_wdata = (rec_k == 6'd0) ? MARK_HEADER : mark_word;
				t_counted = 1'b0;
			end
		end
		else if (mark_ok) begin
			t_go = 1'b1; t_we = 1'b1; t_off = evt_off; t_wdata = MARK_HEADER; t_counted = 1'b0;
		end
`endif
	end
end

`ifdef MISTER_OCRAM_DIAG
wire mark_start = t_go & t_we & mark_ok & ~rec_act;
reg  mark_start_q = 1'b0;
always @(posedge clk) mark_start_q <= mark_start;

// The hello starts the session's counters; a MARK record restarts its period. The period counter marks the
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

wire run_ended = hrst_rise | run_fall;

always @(posedge clk) begin
	// poll bookkeeping: a launched poll read clears the pending period; a new one sets it
	if (t_go && !t_we && poll_sel && (poll_rd_kind == R_HELLO)) poll_pend <= 1'b0;
	else if (poll_tick && run && !hrst_q) poll_pend <= 1'b1;

	// the hello clears the session's ring state and the DIAG counters, then its stores run in order
	if (hello_hit && !run_ended) begin
		phase      <= P_HELLO;
		hj_act     <= 1'b1;
		hj_k       <= 3'd0;
		echo_q     <= rdata;
		head_q     <= 16'd0;
		tail_q     <= 16'd0;
		stopped_q  <= 1'b0;
		line_en_q  <= 1'b0;
		poll_step  <= 3'd0;
		`ifdef MISTER_OCRAM_DIAG
			rec_act <= 1'b0;
			rec_k   <= 6'd0;
		`endif
		rec_mid_q  <= 1'b0;
		sto_pend   <= 1'b0;            // this hello's C_STATE and C_HELLO supersede a pending state word
		fault_pend <= 1'b0;
		hj_state   <= state_word(hello_cnt + 8'd1, 1'b0, prev_mid_q);
`ifdef MISTER_OCRAM_DIAG
		deepest_q  <= 16'd0;
`endif
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
		`endif
		boot_pend  <= 1'b0;
		boot_first <= 1'b1;
		poll_pend  <= 1'b0;
		poll_step  <= 3'd0;
		rd_act     <= R_NONE;
		wr_act     <= W_NONE;
		if (rec_mid_q) prev_mid_q <= 1'b1;
		rec_mid_q  <= 1'b0;
	end
	else if (run_fall) begin
		// the stop: the transaction in flight completes in the engine; a record being written is abandoned and
		// its head is not stored; one terminal store follows unless the HPS reset ended the run. A record head
		// store already in flight still lands, so it is not an abandoned record.
		phase      <= P_OFF;
		if (rec_mid_q && (wr_act != W_RECHEAD)) prev_mid_q <= 1'b1;
		rec_mid_q  <= 1'b0;
		`ifdef MISTER_OCRAM_DIAG
			rec_act <= 1'b0;
			rec_k   <= 6'd0;
		`endif
		hj_act     <= 1'b0;
		sto_pend   <= 1'b0;
		fault_pend <= 1'b0;
		boot_pend  <= 1'b0;
		boot_first <= 1'b1;
		poll_pend  <= 1'b0;
		poll_step  <= 3'd0;
		rd_act     <= R_NONE;
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
		`endif
		rd_act     <= R_NONE;
		wr_act     <= W_NONE;
	end
	else if (t_go) begin
		// take the command
		if (t_we) begin
			if (term_pend) begin
				wr_act <= W_TERM; term_pend <= 1'b0;
			end
			else if (fault_pend) begin
				wr_act <= W_FAULT; fault_pend <= 1'b0;
			end
			else if (hj_act) begin
				wr_act <= W_HELLO;
			end
			else if (sto_pend) begin
				wr_act <= W_ONE; sto_pend <= 1'b0;
			end
`ifdef MISTER_OCRAM_DIAG
			else if (rec_act) begin
				if (rec_k == 6'd41) begin
					wr_act <= W_RECHEAD; rec_k <= 6'd42;
				end
				else begin
					wr_act <= W_REC; rec_k <= rec_k + 6'd1;
				end
			end
			else begin
				wr_act   <= W_REC;
				rec_act  <= 1'b1;
				rec_k    <= 6'd1;
				rec_mid_q<= 1'b1;
			end
`endif
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
				if (hj_k == 3'd6) begin
					hj_act    <= 1'b0;
					phase     <= P_SESSION;
					poll_step <= 3'd2;
					hello_cnt <= hello_cnt + 8'd1;
					prev_mid_q<= 1'b0;
				end
				else hj_k <= hj_k + 3'd1;
			end
		end
`ifdef MISTER_OCRAM_DIAG
		else if (wr_act == W_RECHEAD) begin
			wr_act  <= W_NONE;
			head_q  <= head_q + 16'd41;
			rec_act <= 1'b0;
			rec_mid_q <= 1'b0;
			rec_k   <= 6'd0;
			if (((head_q + 16'd41) - tail_q) > deepest_q) deepest_q <= (head_q + 16'd41) - tail_q;
		end
`endif
		else if (wr_act != W_NONE) wr_act <= W_NONE;
	end

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
				if (rdata == echo_q) poll_step <= 3'd2;
				else begin
					// the session ends: its line drops, a record part-written is recorded for the next hello
					`ifdef MISTER_OCRAM_DIAG
						if (rec_act) begin
							rec_act <= 1'b0;
							rec_k <= 6'd0;
							if (rec_mid_q) prev_mid_q <= 1'b1;
							rec_mid_q <= 1'b0;
						end
					`endif
					poll_step <= 3'd0;
					sto_pend  <= 1'b1;
					sto_data  <= (rdata == 32'd0) ? C_HELLO_ARMED : C_HELLO_WAIT;
					phase     <= (rdata == 32'd0) ? P_ARMED : P_WAIT;
				end
			end
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

	// the EVT line: high in a session, allowed by H_LINE, the ring not stopped, the head not the tail
	line <= (phase == P_SESSION) && line_en_q && !stopped_q && (head_q != tail_q);
end

`ifdef MISTER_OCRAM_DIAG
hps_ocram_diag diag
(
	.clk        (clk),
	.clr        (diag_clr),
	.mark_start (mark_start_q),
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
//  hps_ocram_diag -- the MARK record's counters (MISTER_OCRAM_DIAG only)
//
//  Every counted transaction's latency, from the cycle its ARVALID/AWVALID first rose to the cycle of its R/B
//  handshake; the counts, sums, min/max (16-bit, saturating), error counts (16-bit, saturating) and the two
//  32-bit histograms. A MARK record's own header and payload writes are not counted; every other transaction
//  is. The snapshot is taken when the record begins, so one record's 40 words are one consistent view without
//  the counters moving under it. All counters clear at each hello. Every array is logic (no block memory), and
//  each histogram has one adder and one read mux with a decoded write enable.
//============================================================================

/* verilator lint_off DECLFILENAME */   // the sheet keeps the counters in this file so the fit reports their ALMs apart
module hps_ocram_diag
(
	input  wire        clk,
	input  wire        clr,          // a hello: clear every counter
	input  wire        mark_start,   // a MARK record begins: snapshot
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

reg [31:0] cyc = 32'd0;
always @(posedge clk) cyc <= cyc + 32'd1;

reg aw_d = 1'b0, ar_d = 1'b0;
wire aw_rise = awvalid & ~aw_d;
wire ar_rise = arvalid & ~ar_d;
always @(posedge clk) begin aw_d <= awvalid; ar_d <= arvalid; end

wire b_fire = bvalid & bready;
wire r_fire = rvalid & rready;

reg        w_meas = 1'b0, r_meas = 1'b0;
reg [31:0] w_start = 32'd0, r_start = 32'd0;
wire [31:0] w_lat = cyc - w_start;
wire [31:0] r_lat = cyc - r_start;

// counts, sums, extrema and errors
reg [31:0] rd_n = 32'd0, wr_n = 32'd0;
reg [31:0] rd_sum = 32'd0, wr_sum = 32'd0;
reg [15:0] rd_max = 16'd0, wr_max = 16'd0, rd_min = 16'hFFFF, wr_min = 16'hFFFF;
reg [15:0] rd_err = 16'd0, wr_err = 16'd0;

(* ramstyle = "logic" *) reg [31:0] rd_hist [0:15];
(* ramstyle = "logic" *) reg [31:0] wr_hist [0:15];

// bins (cycles): 0-7, 8-15, ... 56-63, 64-127, 128-255, 256-511, 512-1023, 1024-2047, 2048-4095,
// 4096-65535, 65536 and more
function [3:0] bin_of(input [31:0] lat);
	begin
		if (lat < 32'd64)         bin_of = {1'b0, lat[5:3]};
		else if (lat < 32'd128)   bin_of = 4'd8;
		else if (lat < 32'd256)   bin_of = 4'd9;
		else if (lat < 32'd512)   bin_of = 4'd10;
		else if (lat < 32'd1024)  bin_of = 4'd11;
		else if (lat < 32'd2048)  bin_of = 4'd12;
		else if (lat < 32'd4096)  bin_of = 4'd13;
		else if (lat < 32'd65536) bin_of = 4'd14;
		else                      bin_of = 4'd15;
	end
endfunction

wire [3:0]  w_bin = bin_of(w_lat);
wire [3:0]  r_bin = bin_of(r_lat);
wire [15:0] w_lat16 = (w_lat > 32'h0000_FFFF) ? 16'hFFFF : w_lat[15:0];
wire [15:0] r_lat16 = (r_lat > 32'h0000_FFFF) ? 16'hFFFF : r_lat[15:0];
wire [31:0] w_inc = wr_hist[w_bin] + 32'd1;
wire [31:0] r_inc = rd_hist[r_bin] + 32'd1;
wire        w_add = b_fire & w_meas;
wire        r_add = r_fire & r_meas;

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
		w_meas <= 1'b0; r_meas <= 1'b0;
	end
	else begin
		if (aw_rise && counted) begin w_meas <= 1'b1; w_start <= cyc; end
		if (ar_rise && counted) begin r_meas <= 1'b1; r_start <= cyc; end
		if (drop) begin w_meas <= 1'b0; r_meas <= 1'b0; end
		if (w_add) begin
			w_meas  <= 1'b0;
			wr_n    <= wr_n + 32'd1;
			wr_sum  <= wr_sum + w_lat;
			if (w_lat16 > wr_max) wr_max <= w_lat16;
			if (w_lat16 < wr_min) wr_min <= w_lat16;
			if ((bresp != 2'b00) && (wr_err != 16'hFFFF)) wr_err <= wr_err + 16'd1;
			for (hi = 0; hi < 16; hi = hi + 1)
				if (w_bin == hi[3:0]) wr_hist[hi] <= w_inc;
		end
		if (r_add) begin
			r_meas  <= 1'b0;
			rd_n    <= rd_n + 32'd1;
			rd_sum  <= rd_sum + r_lat;
			if (r_lat16 > rd_max) rd_max <= r_lat16;
			if (r_lat16 < rd_min) rd_min <= r_lat16;
			if ((rresp != 2'b00) && (rd_err != 16'hFFFF)) rd_err <= rd_err + 16'd1;
			for (hi = 0; hi < 16; hi = hi + 1)
				if (r_bin == hi[3:0]) rd_hist[hi] <= r_inc;
		end
	end
end

// The snapshot a MARK record reads, word by word: word 0 carries the ring's deepest fill, the DIAG knobs and
// C_STATE's hello count; words 1-7 the counters; 8-23 and 24-39 the two histograms.
(* ramstyle = "logic" *) reg [31:0] snap [0:39];
integer si;
always @(posedge clk) if (mark_start) begin
	snap[0] <= {hello_cnt, 6'd0, flood, ns, deepest};
	snap[1] <= rd_n;
	snap[2] <= wr_n;
	snap[3] <= {rd_max, rd_min};
	snap[4] <= {wr_max, wr_min};
	snap[5] <= rd_sum;
	snap[6] <= wr_sum;
	snap[7] <= {rd_err, wr_err};
	for (si = 0; si < 16; si = si + 1) begin
		snap[8  + si] <= rd_hist[si];
		snap[24 + si] <= wr_hist[si];
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
