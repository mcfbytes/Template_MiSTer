//============================================================================
//  hps_ocram_rx.sv -- part of the OCRAM mailbox (docs/OCRAM_MAILBOX.md)
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
`ifdef MISTER_OCRAM_OSD
`define LW_OCRAM_RX_BUILT
`endif
`ifdef MISTER_OCRAM_SECTOR_SLOT
`define LW_OCRAM_RX_BUILT
`endif
`ifdef LW_OCRAM_RX_BUILT
`default_nettype none

// The host-to-core consumer of the OCRAM mailbox: ONE walker on clk_100m for every host-to-core ring it serves,
// one held-data word crossing each way, and the sinks on clk_sys. OSD_EN (MISTER_OCRAM_OSD) serves the OSD ring
// (r = 2, base 0x2000, 2,048 words) into every osd's osd_buffer through the LW OSDBUF writer's port; SEC_EN
// (MISTER_OCRAM_SECTOR_SLOT, which needs MISTER_LW) serves the SECTOR ring (r = 4, base 0x7000, 8,192 words) into
// one hps_io slot through the window's xsec feeder nets and lw_blk_mux.
//
//  * clk_100m: the walker. It takes the master's consumer port 0, one transaction at a time, and serves the rings at
//    record granularity, alternating when both have work. Per ring it keeps its tail and avail = head - tail, read
//    at a head read; a head more than the ring's size ahead, or a length past avail, stops that ring for the session
//    with a one-cycle fault. One adder addresses every ring word: tail + k (the header is k = 0, the tail store is
//    the same sum at the record's end). Every read response is registered before a decision, except a payload
//    word, which goes straight into the forward cell's hold register (the one register rdata reaches); no read
//    whose response the cell takes is issued while the cell is busy. A head is read only when the ring is caught
//    up and a poll tick, a finished record or (SECTOR) a new request asks for it.
//  * The forward lw_xfer carries {tag, d}: 0 OSD_HEAD (the ROW header, row in d[20:16]), 1 SEC_HEAD (the LBA), 2
//    DATA (of the last head's ring). The done lw_xfer answers each: 0 took, 1 accept, 2 mismatch, 3 no request.
//    One word is outstanding at a time.
//  * OSD: a ROW record crosses as its header and 64 DATA words. The writer on clk_sys writes each word's four bytes
//    from the capture register, little-endian, one a cycle, under the OSDBUF guard (both osd selects low and the
//    I/O view, in that cycle and the two before), and answers after the fourth; a word met with the enable low is
//    answered unwritten. An unknown kind, a ROW of another length or a row outside the OSD is skipped by length.
//  * SECTOR: a FRAME {0x10, 0, FRAME + 1}, then the LBA and FRAME words. The walker acts only while the slot is
//    bound (C_BIND bit 0 stored 1) and the request record says a request stands: it reads the header and the LBA
//    and crosses SEC_HEAD. Accept: the FRAME words stream, then the tail is stored. Mismatch: the tail moves past
//    the frame. No request: the frame stays until the request record changes or a poll tick. Other records are
//    skipped by length. No REQUEST is asked for unless a head read made after the request landed finds the ring
//    empty, once per request (rq_post, rq_start: the master's record writer).
//  * The bind: sec_ask (H_SECTOR bit 0) raises the bind level to clk_sys; c_sbind (C_BIND bit 0) is 1 once the
//    feeder's en_sys reads back 1. When the bit clears (or ring 3 or 4 faults) a frame being streamed finishes,
//    the level falls, the walker reads H_IDX[4] and stores C_IDX[4] = that head (every queued frame discarded), and
//    only then, with en_sys read back 0, c_sbind falls: a host that has read C_BIND = 0 finds no stale frame.
//  * The feeder (clk_sys) re-creates hps_io's strobes as lw_sector's primary slot does: en_sys follows the bind level
//    only while it is idle and the slot's SPI ack is low, and rises only while the commit mark reads 0; the ack
//    rises at an accept and the first strobe follows GAP cycles later; a word's strobes are GAP apart (two 16-bit
//    strobes with io_wide, else four bytes), its done goes with its last strobe, and the ack falls GAP - 1 cycles
//    after the frame's last strobe. A consumer reset, or the session's enable level low for two cycles while a word
//    is awaited, zero-fills from the next word; words arriving then are answered unplayed. The request record
//    {offered, lba} crosses whole at every change of offered (the slot's mux here, the core's request raised, the
//    ack low and two cycles past its fall).
//  * The enable link (lw_fence) carries the session: hen = s_live, take while idle, drain while no word waits for
//    its done, fin = 1. c_open (C_BIND bit 11) is the link's open and the OSD bind; c_rest is its idle, so the
//    hello waits for the clk_sys half to rest. A record the session's end cuts is abandoned; c_mid / c_smid are
//    high from its header read to its tail store's grant.
//  * MISTER_OCRAM_DIAG (OSD only): GPO 25 is the freeze knob through lw_sync (no byte, no done, no rest while 1),
//    and the rows finished and the deepest OSD fill for the VERDICT record.
//
// Address bound: H_IDX[2], H_IDX[4], C_IDX[2], C_IDX[4], or a ring word BASE + ((index & (WORDS - 1)) << 2). The
// controls are below 0x100 and both spans end below 0xF000; generate-time guards stop elaboration otherwise.
// tb/static.sh holds it.

module hps_ocram_rx #(parameter ROWS = 16, parameter OSD_EN = 1, parameter SEC_EN = 0,
                      parameter [31:0] FRAME = 32'd128, parameter [31:0] GAP = 32'd2)
(
	input  wire        clk,          // clk_100m: the walker
	input  wire        clk_sys,      // the core clock: the sinks

	/* verilator lint_off UNUSEDSIGNAL */   // GPO 25 is the DIAG freeze knob; without DIAG it is never read
	input  wire        gp_out25,
	/* verilator lint_on UNUSEDSIGNAL */

	// The master's session interface
	input  wire        s_hello,      // one cycle: the hello's first store is about to run
	input  wire        s_live,       // P_SESSION
	input  wire        s_tick,       // the poll period's pulse
	input  wire        bind_en,      // the OSD bind (H_BIND bit 11 read)
	input  wire        sec_ask,      // H_SECTOR bit 0, as last read
	input  wire        evt_stop,     // the EVT ring stopped on a fault
	output wire        c_rest,       // the enable link's idle: the clk_sys half is at rest
	output wire        c_fault,      // one cycle: the OSD ring stopped on a fault
	output wire        c_mid,        // level: an OSD record is part-consumed
	output wire        c_open,       // C_BIND bit 11
	output wire        c_sfault,     // one cycle: the SECTOR ring stopped on a fault
	output wire        c_smid,       // level: a SECTOR record is part-consumed
	output wire        c_sbind,      // C_BIND bit 0
	output wire        c_wide,       // hps_io's io_wide, for C_K[1]
	output wire        rq_post,      // a REQUEST is owed for rq_lba
	output wire [31:0] rq_lba,
	input  wire        rq_start,     // one cycle: the master begins that REQUEST
	output wire [15:0] osd_rows,     // ROW records finished this session (DIAG)
	output wire [15:0] osd_deepest,  // the deepest OSD fill read this session (DIAG)

	// The master's consumer port: one transaction at a time
	output wire        c_req,
	output wire        c_we,
	output wire [15:0] c_off,
	output wire [31:0] c_wdata,
	input  wire        c_grant,
	input  wire        c_done,
	input  wire        c_ok,
	input  wire [31:0] c_rdata,

	// The OSDBUF write port every osd takes (osd.v), on clk_sys
	input  wire  [1:0] sel,          // both osd selects as each osd sees them, {vga, hdmi}
	input  wire        view,         // GPO[31]: 1 = the link's I/O view
	output wire        we,
	output wire [12:0] addr,
	output wire  [7:0] dbyte,

	// The SECTOR feeder (the window's xsec nets), on clk_sys
	/* verilator lint_off UNUSEDSIGNAL */   // without SEC_EN the feeder's inputs are not read
	input  wire        x_reset,      // the consumer's reset: zero-fills an open frame
	input  wire        x_hps_ack,    // the slot's SPI ack
	input  wire        x_req,        // the core's request, wired
	input  wire [31:0] x_lba,
	input  wire        x_commit,     // hps_io's commit mark
	input  wire        x_wide,       // hps_io's io_wide, a constant
	/* verilator lint_on UNUSEDSIGNAL */
	output wire        x_en,         // en_sys: the slot's mux is on this feeder
	output wire        x_ack,
	output wire        x_wr,
	output wire [15:0] x_dat,
	output wire [13:0] x_adr
);

localparam [15:0] H_IDX2    = 16'h0018;     // host: the OSD ring's head
localparam [15:0] H_IDX4    = 16'h0020;     // host: the SECTOR ring's head
localparam [15:0] C_IDX2    = 16'h0098;     // fabric: the OSD ring's tail
localparam [15:0] C_IDX4    = 16'h00A0;     // fabric: the SECTOR ring's tail
localparam [15:0] OSD_BASE  = 16'h2000;
localparam [15:0] OSD_WORDS = 16'd2048;
localparam [15:0] SEC_BASE  = 16'h7000;
localparam [15:0] SEC_WORDS = 16'd8192;
localparam  [7:0] LAST_ROW  = ROWS - 1;
localparam [15:0] SEC_LEN   = FRAME[15:0] + 16'd1;      // a FRAME record's len: the LBA and the words
localparam [13:0] SEC_K     = FRAME[13:0] + 14'd1;      // its last payload word

generate
	if (OSD_BASE + OSD_WORDS * 4 > 16'hF000 || SEC_BASE + SEC_WORDS * 4 > 16'hF000) begin : g_rx_span
		ocram_rx_span_above_0xF000 guard();
	end
	if (H_IDX2 >= 16'h0100 || H_IDX4 >= 16'h0100 || C_IDX2 >= 16'h0100 || C_IDX4 >= 16'h0100) begin : g_rx_ctrl
		ocram_rx_control_constant_above_0xFF guard();
	end
	if (SEC_EN != 0 && (FRAME == 0 || FRAME > 4096 || GAP < 2 || GAP > 15)) begin : g_geometry
		ocram_sector_unservable_geometry guard();
	end
endgenerate

///////////////////////////////  the crossings  //////////////////////////////

`ifdef MISTER_OCRAM_DIAG
(* preserve *) reg frz_lwx = 1'b0;
always @(posedge clk) frz_lwx <= gp_out25;
wire frz;
lw_sync #(.W(1)) frz_sync(.clk(clk_sys), .d(frz_lwx), .q(frz));
`else
wire frz = 1'b0;
`endif

localparam [1:0] TG_OHEAD = 2'd0, TG_SHEAD = 2'd1, TG_DATA = 2'd2;
localparam [1:0] A_TOOK = 2'd0, A_ACCEPT = 2'd1, A_MISMATCH = 2'd2;   // 3: no request

wire [33:0] word_q;                    // clk_sys: the capture register {tag, d}
wire        word_v;
wire        fwd_busy;
wire  [1:0] done_q;
wire        done_v;
wire        back_busy;
reg         done_go = 1'b0;            // clk_sys
reg   [1:0] done_code = 2'd0;
wire        x_go;                      // clk: the forward launch
wire [33:0] x_d;

lw_xfer #(.W(34)) rx_fwd(.s_clk(clk), .s_go(x_go), .s_d(x_d), .s_busy(fwd_busy),
                         .d_clk(clk_sys), .d_q(word_q), .d_v(word_v));
lw_xfer #(.W(2)) rx_done(.s_clk(clk_sys), .s_go(done_go), .s_d(done_code), .s_busy(back_busy),
                         .d_clk(clk), .d_q(done_q), .d_v(done_v));

// the request record {offered, lba} and the bind level each way
wire [32:0] rq_q;
wire        rq_v;
wire        rc_busy;
wire        rc_go;
(* preserve *) reg bind_lwx = 1'b0;    // clk: the SECTOR bind level
(* preserve *) reg en_lwx = 1'b0;      // clk_sys: en_sys
(* preserve *) reg wide_lwx = 1'b0;    // clk_sys: io_wide
wire        bind_s, en_b, wide_b;
reg         offd = 1'b0;               // clk_sys: offered, registered
wire [32:0] rec = {offd, x_lba};
lw_xfer #(.W(33)) rx_req(.s_clk(clk_sys), .s_go(rc_go), .s_d(rec), .s_busy(rc_busy),
                         .d_clk(clk), .d_q(rq_q), .d_v(rq_v));
lw_sync #(.W(1)) bind_sync(.clk(clk_sys), .d(bind_lwx), .q(bind_s));
lw_sync #(.W(2)) back_sync(.clk(clk), .d({wide_lwx, en_lwx}), .q({wide_b, en_b}));
assign c_wide = wide_b;

wire        f_open, f_idle, hen_s, rest;
reg         word_out = 1'b0;           // a word launched, its done not back
wire        take = s_live & f_idle;
/* verilator lint_off PINCONNECTEMPTY */   // OWNED, the launched level and the fall have no user here
lw_fence fence
(
	.clk(clk), .hen(s_live), .take(take), .drain(!word_out), .fin(1'b1),
	.open(f_open), .owned(), .idle(f_idle), .lvl(), .fall(),
	.clk_sys(clk_sys), .rest(rest), .hen_s(hen_s)
);
/* verilator lint_on PINCONNECTEMPTY */

///////////////////////////////  the walker  ////////////////////////////////

localparam [3:0] W_IDLE = 4'd0, W_HEAD = 4'd1, W_HDR = 4'd2, W_LBA = 4'd3, W_HGO = 4'd4, W_ANS = 4'd5,
                 W_PAY = 4'd6, W_XFER = 4'd7, W_TAIL = 4'd8, W_FLUSH = 4'd9, W_FTAIL = 4'd10;

reg   [3:0] wst = W_IDLE;
reg         req = 1'b0;
reg         req_we = 1'b0;
reg  [15:0] req_off = 16'd0;
reg         rd_pend = 1'b0;            // a ring word read is owed at tail + k
reg         st_pend = 1'b0;            // the ring's tail store is owed: tail + k
reg         ring = 1'b0;               // the record's ring: 0 OSD, 1 SECTOR
reg         turn = 1'b0;               // the ring served last
reg  [15:0] tail_o = 16'd0, tail_s = 16'd0;
reg  [11:0] avail_o = 12'd0;
reg  [13:0] avail_s = 14'd0;
reg         due_o = 1'b1, due_s = 1'b1;   // read the head when caught up
reg         stop_o = 1'b0, stop_s = 1'b0;
reg  [13:0] k = 14'd0;
reg         mid_o = 1'b0, mid_s = 1'b0;
reg         flt_o = 1'b0, flt_s = 1'b0;
reg   [4:0] row_q = 5'd0;              // the ROW being sent: its OSD_HEAD carries the row alone
`ifdef MISTER_OCRAM_DIAG
reg         was_row = 1'b0;            // the OSD record being finished is a ROW
`endif
reg         s_wait = 1'b0;             // a no-request answer: wait for the record or a tick
reg         sb = 1'b0;                 // C_BIND bit 0
reg         fl_pend = 1'b0;            // the unbind's flush is owed
reg         rq_posted = 1'b0, fresh = 1'b0;
reg         drop_s = 1'b0;             // the bind is being dropped
`ifdef MISTER_OCRAM_DIAG
reg  [15:0] rows_q = 16'd0, deepest_q = 16'd0;
assign osd_rows = rows_q;
assign osd_deepest = deepest_q;
`else
assign osd_rows = 16'd0;
assign osd_deepest = 16'd0;
`endif

wire [15:0] tl     = ring ? tail_s : tail_o;
wire [15:0] widx   = tl + {2'd0, k};
wire [15:0] osd_off = OSD_BASE + ((widx & (OSD_WORDS - 1)) << 2);
wire [15:0] sec_off = SEC_BASE + ((widx & (SEC_WORDS - 1)) << 2);
// a response is decoded in its own cycle, as the bridge's rdata reaches each register in one stage
wire [15:0] new_gap = c_rdata[15:0] - tl;
wire [16:0] rlen    = {1'b0, c_rdata[15:0]} + 17'd1;
wire        c_new   = c_done && c_ok;
wire        c_bad   = c_done && !c_ok;
wire [16:0] ravail  = ring ? {3'd0, avail_s} : {5'd0, avail_o};

wire        stand  = rq_q[32];
wire        walk_o = OSD_EN != 0 && s_live && f_open && bind_en && !stop_o;
wire        walk_s = SEC_EN != 0 && s_live && f_open && !stop_s;
wire        want_s = walk_s && sec_ask && !evt_stop;
wire        job_o  = walk_o && (avail_o != 12'd0 || due_o);
wire        job_s  = walk_s && sb && !drop_s && ((avail_s != 14'd0 && stand && !s_wait) || (avail_s == 14'd0 && due_s));
wire        streaming = ring && wst != W_IDLE;

assign c_req    = req;
assign c_we     = req_we;
assign c_off    = req_off;
assign c_wdata  = {16'd0, widx};          // a tail store's value: k and the tail hold until its done
assign c_fault  = flt_o;
assign c_mid    = mid_o;
assign c_sfault = flt_s;
assign c_smid   = mid_s;
assign c_open   = f_open & bind_en;
assign c_rest   = f_idle;
assign c_sbind  = sb;
assign rq_lba   = rq_q[31:0];
// not in a record's landing cycle: its fresh read is the next one
assign rq_post  = SEC_EN != 0 && sb && !drop_s && bind_lwx && stand && !rq_posted && fresh && !rq_v &&
                  avail_s == 14'd0 && !stop_s;

// A payload word and a frame's LBA go to the cell in their read's response cycle (each read is issued only with the
// cell idle, and nothing else launches before its response); a ROW's head, its row alone, from row_q.
wire   x_rsp = (wst == W_PAY || wst == W_LBA) && c_new && s_live;
assign x_go = x_rsp || (wst == W_HGO && !fwd_busy && s_live);
assign x_d  = (wst == W_PAY) ? {TG_DATA, c_rdata} : (wst == W_LBA) ? {TG_SHEAD, c_rdata} :
              {TG_OHEAD, 11'd0, row_q, 16'd0};

always @(posedge clk) begin
	flt_o <= 1'b0;
	flt_s <= 1'b0;
	if (c_grant) req <= 1'b0;

	// the bind level: up while asked; it falls only between records
	if (want_s && !drop_s) bind_lwx <= 1'b1;
	else if (!(streaming && s_live)) bind_lwx <= 1'b0;
	if (bind_lwx && en_b && !drop_s) sb <= 1'b1;
	if (sb && !want_s && !drop_s && !(streaming && s_live)) begin
		drop_s  <= 1'b1;
		fl_pend <= !stop_s;
	end
	if (drop_s && !fl_pend && !en_b) begin
		drop_s <= 1'b0;
		sb     <= 1'b0;
	end

	// the request record: a new one asks for one fresh head read, and voids the post
	if (rq_v) begin
		rq_posted <= 1'b0;
		fresh     <= 1'b0;
		s_wait    <= 1'b0;
		if (rq_q[32]) due_s <= 1'b1;
	end
	else if (rq_start) rq_posted <= 1'b1;
	if (s_tick) begin due_o <= 1'b1; due_s <= 1'b1; s_wait <= 1'b0; end

	// a ring word read, addressed by the one adder; never while the forward cell is busy
	if (rd_pend && !req && !fwd_busy) begin
		rd_pend <= 1'b0;
		req     <= 1'b1;
		req_we  <= 1'b0;
		if (ring) req_off <= sec_off;
		else      req_off <= osd_off;
	end
	if (st_pend && !req) begin
		st_pend   <= 1'b0;
		req       <= 1'b1;
		req_we    <= 1'b1;
		if (ring) req_off <= C_IDX4;
		else      req_off <= C_IDX2;
	end

	if (s_hello) begin
		wst <= W_IDLE; req <= 1'b0; rd_pend <= 1'b0; st_pend <= 1'b0; word_out <= 1'b0;
		tail_o <= 16'd0; tail_s <= 16'd0; avail_o <= 12'd0; avail_s <= 14'd0;
		due_o <= 1'b1; due_s <= 1'b1; stop_o <= 1'b0; stop_s <= 1'b0;
		mid_o <= 1'b0; mid_s <= 1'b0; s_wait <= 1'b0; turn <= 1'b0;
		sb <= 1'b0; drop_s <= 1'b0; fl_pend <= 1'b0; bind_lwx <= 1'b0; fresh <= 1'b0; rq_posted <= 1'b0;
`ifdef MISTER_OCRAM_DIAG
		rows_q <= 16'd0; deepest_q <= 16'd0;
`endif
	end
	else if (!s_live && wst != W_IDLE) begin
		// the session ended: the record is abandoned, its mid bit left for the master
		wst <= W_IDLE; req <= 1'b0; rd_pend <= 1'b0; st_pend <= 1'b0; word_out <= 1'b0;
	end
	else if (!s_live) begin
		sb <= 1'b0; drop_s <= 1'b0; fl_pend <= 1'b0;
	end
	else case (wst)
	W_IDLE: begin
		mid_o <= 1'b0;
		mid_s <= 1'b0;
		if (SEC_EN != 0 && fl_pend) begin
			ring <= 1'b1; req <= 1'b1; req_we <= 1'b0; req_off <= H_IDX4; wst <= W_FLUSH;
		end
		else if (job_o && (!job_s || turn)) begin
			ring <= 1'b0; turn <= 1'b0;
			if (avail_o == 12'd0) begin
				req <= 1'b1; req_we <= 1'b0; req_off <= H_IDX2; due_o <= 1'b0; wst <= W_HEAD;
			end
			else begin k <= 14'd0; rd_pend <= 1'b1; mid_o <= 1'b1; wst <= W_HDR; end
		end
		else if (job_s) begin
			ring <= 1'b1; turn <= 1'b1;
			if (avail_s == 14'd0) begin
				req <= 1'b1; req_we <= 1'b0; req_off <= H_IDX4; due_s <= 1'b0; wst <= W_HEAD;
			end
			else begin k <= 14'd0; rd_pend <= 1'b1; mid_s <= 1'b1; wst <= W_HDR; end
		end
	end
	W_HEAD, W_FLUSH: if (c_done) begin
		if (!c_ok) begin req <= 1'b1; req_we <= 1'b0; end            // no value: read it again
		else if (new_gap > (ring ? SEC_WORDS : OSD_WORDS)) begin
			// a head more than the ring's size ahead: a host defect
			if (ring) begin stop_s <= 1'b1; flt_s <= 1'b1; fl_pend <= 1'b0; end
			else      begin stop_o <= 1'b1; flt_o <= 1'b1; end
			wst <= W_IDLE;
		end
		else if (wst == W_FLUSH) begin
			// the unbind: every queued frame is discarded by one tail store
			k <= new_gap[13:0]; st_pend <= 1'b1; wst <= W_FTAIL;
		end
		else begin
			if (ring) begin avail_s <= new_gap[13:0]; fresh <= 1'b1; end
			else      avail_o <= new_gap[11:0];
`ifdef MISTER_OCRAM_DIAG
			if (!ring && new_gap > deepest_q) deepest_q <= new_gap;
`endif
			wst <= W_IDLE;
		end
	end
	W_HDR: if (c_done) begin
		if (!c_ok) rd_pend <= 1'b1;
		else if (rlen > ravail) begin
			// a length past the head: a host defect
			if (ring) begin stop_s <= 1'b1; flt_s <= 1'b1; mid_s <= 1'b0; end
			else      begin stop_o <= 1'b1; flt_o <= 1'b1; mid_o <= 1'b0; end
			wst <= W_IDLE;
		end
		else if (!ring && c_rdata[31:24] == 8'h01 && c_rdata[15:0] == 16'd64 && c_rdata[23:16] <= LAST_ROW) begin
			row_q <= c_rdata[20:16]; k <= 14'd0; word_out <= 1'b1; wst <= W_HGO;
`ifdef MISTER_OCRAM_DIAG
			was_row <= 1'b1;
`endif
		end
		else if (ring && c_rdata[31:24] == 8'h10 && c_rdata[23:16] == 8'd0 && c_rdata[15:0] == SEC_LEN) begin
			k <= 14'd1; rd_pend <= 1'b1; wst <= W_LBA;
		end
		else begin
			// an unknown kind, a record of another length or argument: skipped by its length
			k <= rlen[13:0]; st_pend <= 1'b1; wst <= W_TAIL;
`ifdef MISTER_OCRAM_DIAG
			was_row <= 1'b0;
`endif
		end
	end
	W_LBA: if (c_done) begin
		if (!c_ok) rd_pend <= 1'b1;
		else begin word_out <= 1'b1; wst <= W_ANS; end               // x_go launched SEC_HEAD with the LBA
	end
	W_HGO: if (!fwd_busy) wst <= W_XFER;                              // x_go launches the ROW's head this cycle
	W_ANS: if (done_v) begin
		word_out <= 1'b0;
		if (done_q == A_ACCEPT) begin k <= 14'd2; rd_pend <= 1'b1; wst <= W_PAY; end
		else if (done_q == A_MISMATCH) begin
			k <= SEC_K + 14'd1; st_pend <= 1'b1; wst <= W_TAIL;
		end
		else begin s_wait <= 1'b1; wst <= W_IDLE; end                 // no request: the frame stays
	end
	W_PAY: if (c_done) begin
		if (!c_ok) rd_pend <= 1'b1;                                   // read it again
		else begin word_out <= 1'b1; wst <= W_XFER; end              // x_go launched it
	end
	W_XFER: if (done_v) begin
		word_out <= 1'b0;
		if (k == (ring ? SEC_K : 14'd64)) begin
			k <= k + 14'd1; st_pend <= 1'b1; wst <= W_TAIL;
		end
		else begin k <= k + 14'd1; rd_pend <= 1'b1; wst <= W_PAY; end
	end
	W_TAIL, W_FTAIL: begin
		if (c_new) begin
			// the tail store is done: the record is consumed
			wst <= W_IDLE;
			if (ring) begin
				tail_s <= widx; avail_s <= (wst == W_FTAIL) ? 14'd0 : avail_s - k; mid_s <= 1'b0; fresh <= 1'b0;
				if (avail_s == k || wst == W_FTAIL) due_s <= 1'b1;
				if (wst == W_FTAIL) fl_pend <= 1'b0;
			end
			else begin
				tail_o <= widx; avail_o <= avail_o - k[11:0]; mid_o <= 1'b0;
				if (avail_o == k[11:0]) due_o <= 1'b1;
`ifdef MISTER_OCRAM_DIAG
				if (was_row) rows_q <= rows_q + 16'd1;
`endif
			end
		end
		else if (c_bad) req <= 1'b1;                               // a failed store is tried again, same value
	end
	default: wst <= W_IDLE;
	endcase
end

///////////////////////////////  clk_sys  ///////////////////////////////////

reg         cur = 1'b0;                // the ring of the last head: 0 OSD, 1 SECTOR
wire  [1:0] tag = word_q[33:32];
wire [31:0] wd  = word_q[31:0];
wire        o_word_v = OSD_EN != 0 && word_v && (tag == TG_OHEAD || (tag == TG_DATA && !cur));
/* verilator lint_off UNUSEDSIGNAL */   // the feeder's, with SEC_EN
wire        s_word_v = SEC_EN != 0 && word_v && (tag == TG_SHEAD || (tag == TG_DATA && cur));
/* verilator lint_on UNUSEDSIGNAL */

// the OSD writer: the word waits in the capture register until its four bytes are written
reg         o_have = 1'b0;             // a DATA word to write
reg         o_hans = 1'b0;             // a header to answer
reg   [4:0] o_row = 5'd0;
reg   [5:0] o_word = 6'd0;
reg   [1:0] o_b = 2'd0;
reg   [1:0] qn = 2'd0;
wire        quiet = (sel == 2'b00) && view;
wire        wnow  = hen_s && !frz && o_have && quiet && qn == 2'd2;
assign we    = wnow;
assign addr  = {o_row, o_word, o_b};
assign dbyte = wd[{o_b, 3'd0} +: 8];

// the SECTOR feeder
wire        f_go, f_rest, f_ack;
wire  [1:0] f_code;
generate
if (SEC_EN != 0) begin : sec
	hps_ocram_sfeed #(.FRAME(FRAME), .GAP(GAP)) feeder
	(
		.clk_sys(clk_sys), .hen_s(hen_s), .bind_s(bind_s),
		.w_v(s_word_v), .w_head(tag == TG_SHEAD), .w_d(wd),
		.a_go(f_go), .a_code(f_code), .rest(f_rest),
		.c_reset(x_reset), .hps_ack(x_hps_ack), .c_req(x_req), .c_lba(x_lba), .c_commit(x_commit), .c_wide(x_wide),
		.en_sys(x_en), .o_ack(f_ack), .o_wr(x_wr), .o_dat(x_dat), .o_adr(x_adr)
	);
end
else begin : nosec
	assign f_go = 1'b0, f_code = 2'd0, f_rest = 1'b1, f_ack = 1'b0;
	assign x_en = 1'b0, x_wr = 1'b0, x_dat = 16'd0, x_adr = 14'd0;
end
endgenerate
assign x_ack = f_ack;

// the request record: offered = the mux here, the core's request raised, the ack low and two cycles past its fall
reg         dirty = 1'b0;
reg   [1:0] rearm = 2'd0;
wire        off_d = SEC_EN != 0 && x_en && x_req && !f_ack && rearm == 2'd0;
assign rc_go = dirty && !rc_busy;

// The half is at rest with the enable and the bind level low, no word held or arriving, nothing owed, every
// handshake settled. While frozen it is never at rest, which is what a stopped clk_sys looks like to the link.
assign rest = !frz && !hen_s && !bind_s && f_rest && !o_have && !o_hans && !back_busy && !word_v && !done_go &&
              !offd && !dirty && !rc_busy;

always @(posedge clk_sys) begin
	done_go  <= 1'b0;
	en_lwx   <= x_en;
	wide_lwx <= x_wide;
	qn <= !quiet ? 2'd0 : (qn == 2'd2) ? 2'd2 : qn + 2'd1;

	// a change of offered in a launch cycle stays dirty for the next record
	offd  <= off_d;
	dirty <= (off_d ^ offd) | (dirty & ~rc_go);
	rearm <= {rearm[0], f_ack | x_hps_ack};

	if (word_v) cur <= (tag == TG_SHEAD) ? 1'b1 : (tag == TG_OHEAD) ? 1'b0 : cur;
	if (o_word_v) begin
		if (tag == TG_OHEAD) begin o_row <= wd[20:16]; o_word <= 6'd0; o_hans <= 1'b1; end
		else begin o_have <= 1'b1; o_b <= 2'd0; end
	end

	// one answer a cycle: a word is outstanding alone, so the two never meet
	if (f_go) begin
		done_go   <= 1'b1;
		done_code <= f_code;
	end
	else if (o_hans && !frz) begin
		o_hans    <= 1'b0;
		done_go   <= 1'b1;
		done_code <= A_TOOK;
	end
	else if (o_have && !frz && !hen_s) begin
		// with the enable low the word is discarded and still answered, so both halves converge
		o_have    <= 1'b0;
		done_go   <= 1'b1;
		done_code <= A_TOOK;
	end
	else if (wnow) begin
		o_b <= o_b + 2'd1;
		if (o_b == 2'd3) begin
			o_have    <= 1'b0;
			o_word    <= o_word + 6'd1;
			done_go   <= 1'b1;
			done_code <= A_TOOK;
		end
	end
end

endmodule

//============================================================================
//  hps_ocram_sfeed -- the SECTOR feeder of hps_ocram_rx, on clk_sys (MISTER_OCRAM_SECTOR_SLOT)
//
//  lw_sector's primary-slot feeder with the forward crossing's capture register as its one-entry queue: w_v marks
//  an entry (a SEC_HEAD with the LBA, or a DATA word) held in w_d until it is answered. Strobes as hps_io's: with
//  c_wide two 16-bit strobes a word at the halfword index, low half first, else four bytes at the byte index;
//  GAP cycles apart, the first GAP after the ack. A word is popped only once its first strobe can issue in the cycle
//  after (GAP - 1 after the previous strobe at the earliest), so a word in hand is strobed as lw_sector strobes a
//  FIFO entry. A word's answer goes with its last strobe; a SEC_HEAD's with its decision. A module of its own so
//  the trace bench can drive it beside lw_sector and the fit reports its ALMs apart.
//============================================================================

/* verilator lint_off DECLFILENAME */   // the feeder shares hps_ocram_rx's file, as the sink shares the master's
module hps_ocram_sfeed #(parameter [31:0] FRAME = 32'd128, parameter [31:0] GAP = 32'd2)
(
	input  wire        clk_sys,
	input  wire        hen_s,        // the session's enable level
	input  wire        bind_s,       // the bind level
	input  wire        w_v,          // one cycle: an entry is in w_d
	input  wire        w_head,       // it is a SEC_HEAD
	input  wire [31:0] w_d,          // held until the entry is answered
	output reg         a_go = 1'b0,  // one cycle: the entry's answer
	output reg   [1:0] a_code = 2'd0,// 0 took, 1 accept, 2 mismatch, 3 no request
	output wire        rest,         // idle: no entry, no frame, the mux on SPI

	input  wire        c_reset,
	input  wire        hps_ack,
	input  wire        c_req,
	input  wire [31:0] c_lba,
	input  wire        c_commit,
	input  wire        c_wide,
	output reg         en_sys = 1'b0,
	output wire        o_ack,
	output reg         o_wr = 1'b0,
	output reg  [15:0] o_dat = 16'd0,
	output reg  [13:0] o_adr = 14'd0
);

localparam [1:0] A_TOOK = 2'd0, A_ACCEPT = 2'd1, A_MISMATCH = 2'd2, A_NOREQ = 2'd3;
localparam [1:0] F_IDLE = 2'd0, F_STREAM = 2'd2, F_FILL = 2'd3;
localparam [12:0] FLAST = FRAME[12:0] - 13'd1;

reg   [1:0] fs = F_IDLE;
reg         have = 1'b0;               // an entry waits
reg         qv = 1'b0;                 // the entry popped last cycle
reg         fack = 1'b0;
reg   [3:0] gc = 4'd0;
reg   [1:0] sn = 2'd0;
reg         em = 1'b0;
reg         fz = 1'b0;                 // the word being strobed is the fill's zero
reg  [12:0] fexp = 13'd0;
reg   [1:0] mt = 2'd0, ml = 2'd0;

wire        fidle  = fs == F_IDLE;
wire        head   = w_head;
wire  [1:0] slast  = c_wide ? 2'd1 : 2'd3;
wire        gap_ok = gc >= GAP[3:0];
wire        nxt_ok = gc >= GAP[3:0] - 4'd1;
wire        wend   = em && sn >= slast && nxt_ok;
// an entry is popped in the cycle it arrives (w_v) or later, as lw_sector pops a FIFO entry the cycle it is visible
wire        wplay  = w_v && (head ? fidle : fs == F_STREAM);     // the entry just arrived and can be played
wire        hv     = have || wplay;
wire        pop    = hv && !qv && ((fidle && head) || (fs == F_STREAM && (!em || wend) && nxt_ok));
wire        bmatch = en_sys && c_req && w_d == c_lba;
wire        cut    = fs == F_STREAM && !have && !qv && !em &&
                     ((!bind_s && mt == 2'd2) || (!hen_s && ml == 2'd2));
wire [31:0] wsrc   = fz ? 32'd0 : w_d;
function automatic [13:0] sadr(input wide, input [12:0] w, input [1:0] j);
	sadr = wide ? {w, j[0]} : {w[11:0], j};
endfunction

assign o_ack = fack;
assign rest  = fidle && !have && !qv && !em && !fack && !o_wr && !en_sys && !a_go;

always @(posedge clk_sys) begin
	a_go <= 1'b0;
	o_wr <= 1'b0;

	// an entry: a word with no frame to play (idle, or a fill) is answered unplayed at once
	if (w_v) begin
		if (!head && (fidle || fs == F_FILL)) begin a_go <= 1'b1; a_code <= A_TOOK; end
		else if (!pop) have <= 1'b1;
	end
	else if (have && !head && fs == F_FILL) begin
		have <= 1'b0; a_go <= 1'b1; a_code <= A_TOOK;
	end

	qv <= pop;
	if (pop) have <= 1'b0;
	gc <= (gc == 4'd15) ? gc : gc + 4'd1;
	mt <= (have || bind_s) ? 2'd0 : (mt == 2'd2) ? 2'd2 : mt + 2'd1;
	ml <= (have || hen_s) ? 2'd0 : (ml == 2'd2) ? 2'd2 : ml + 2'd1;

	// the mux follows the bind level only while the feeder is idle and the SPI ack is low, and moves here only
	// while the commit mark reads 0
	if (fidle && !hps_ack) en_sys <= bind_s & (en_sys | ~c_commit);

	if (fs == F_FILL && !em && gap_ok) begin
		o_wr  <= 1'b1;
		o_dat <= 16'd0;
		o_adr <= sadr(c_wide, fexp, 2'd0);
		fz    <= 1'b1;
		em    <= 1'b1;
		sn    <= 2'd0;
		gc    <= 4'd1;
	end

	if (fidle) fack <= 1'b0;

	if (qv) case (fs)
		F_IDLE: begin
			// a SEC_HEAD: accept the live request's LBA; a request for another one, or none, is answered so
			a_go <= 1'b1;
			if (hen_s && bmatch) begin
				a_code <= A_ACCEPT;
				fexp   <= 13'd0;
				fack   <= 1'b1;
				gc     <= 4'd1;
				fs     <= F_STREAM;
			end
			else a_code <= (hen_s && en_sys && c_req) ? A_MISMATCH : A_NOREQ;
		end
		F_STREAM:
			if (head) begin
				// a head while a frame streams (no walker sends one): fill the frame, answer no request
				fs <= F_FILL; a_go <= 1'b1; a_code <= A_NOREQ;
			end
			else begin
				o_wr  <= 1'b1;
				o_dat <= c_wide ? w_d[15:0] : {8'd0, w_d[7:0]};
				o_adr <= sadr(c_wide, fexp, 2'd0);
				fz    <= 1'b0;
				em    <= 1'b1;
				sn    <= 2'd0;
				gc    <= 4'd1;
			end
		default: ;
	endcase

	// a word's later strobes, GAP apart, its answer with the last; GAP - 1 after it the word ends
	if (em) begin
		if (sn < slast) begin
			if (gap_ok) begin
				o_wr  <= 1'b1;
				o_dat <= c_wide ? wsrc[31:16] : {8'd0, wsrc[{sn + 2'd1, 3'd0} +: 8]};
				o_adr <= sadr(c_wide, fexp, sn + 2'd1);
				sn    <= sn + 2'd1;
				gc    <= 4'd1;
				if (sn + 2'd1 == slast && !fz) begin a_go <= 1'b1; a_code <= A_TOOK; end
			end
		end
		else if (nxt_ok) begin
			em <= 1'b0;
			sn <= 2'd0;
			if (fexp >= FLAST) begin   // or past it: a frame ends from any count
				fexp <= 13'd0;
				fack <= 1'b0;
				fs   <= F_IDLE;
			end
			else fexp <= fexp + 13'd1;
		end
	end

	if (cut) fs <= F_FILL;
	// the consumer's reset zero-fills the frame
	if (c_reset && fs == F_STREAM) fs <= F_FILL;
end

endmodule
/* verilator lint_on DECLFILENAME */

`default_nettype wire
`endif
`undef LW_OCRAM_RX_BUILT
`endif
