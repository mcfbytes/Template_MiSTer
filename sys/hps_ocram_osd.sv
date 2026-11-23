//============================================================================
//  hps_ocram_osd.sv -- part of the OCRAM mailbox (docs/OCRAM_MAILBOX.md)
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
`default_nettype none

// The OSD consumer of the OCRAM mailbox (MISTER_OCRAM_OSD): the host's ROW records in the OSD ring (r = 2, base
// 0x2000, 2,048 words) written into every osd's osd_buffer through the write port the LW OSDBUF writer uses.
//
//  * clk_100m: the walker. It reads a record's header, then its 64 payload words one at a time through the master's
//    consumer port; each word goes to clk_sys through one held-data lw_xfer (43 bits: {row, word, data}) and the
//    walker waits for that half's done (a second lw_xfer, 1 bit, back) before reading the next. It holds no AXI
//    transaction while it waits. It reads H_IDX[2] only when it has no record left to consume and a poll tick (or
//    the record it just finished) asks for it. A head more than the ring's size ahead, or a length past the head,
//    is a host defect: the ring stops for the session and c_fault is a one-cycle pulse. An unknown kind, a ROW of
//    another length or a row outside the OSD is skipped by its length, not a fault.
//  * clk_sys: the writer. It latches every word the forward cell offers into its own pending register, then writes
//    its four bytes little-endian, one byte a cycle, under the OSDBUF guard: both osd selects low and the I/O view,
//    in that cycle and the two before. It answers done after the fourth byte; a word that reaches it with the
//    enable low is discarded and answered with done, so the two halves converge. A byte in rows 8-15 sets highres
//    in osd.v, as an LW byte does.
//  * The enable link (lw_fence) is lw_osd's: hen = s_live & bind, take while the link is idle (so a bind taken
//    while the half is not yet at rest is taken when it is), drain while no word the walker handed over still
//    waits for its done, fin = 1. C_BIND bit 11 is the link's open, which the master stores. c_rest is the link's
//    idle, so the hello waits until the half is at rest.
//  * The tail is stored only for a whole record: a record the session's end cuts is abandoned, and c_mid, high
//    from its header read to its tail store's grant, tells the master the previous session ended mid-record.
//  * With MISTER_OCRAM_DIAG, GPO 25 (the freeze knob) is sampled on clk_100m and crosses to clk_sys through
//    lw_sync. While it is high the half writes nothing, launches no done and reports rest = 0, which is what a
//    stopped clk_sys looks like to the master; on thaw the pending word's bytes and its done follow.
//
// Address bound: the walker reads H_IDX[2] or OSD_BASE + ((index & (OSD_WORDS - 1)) << 2), and writes C_IDX[2].
// Both controls are below 0x100 and the ring's span ends at 0x4000; a generate-time guard stops elaboration
// otherwise. tb/static.sh holds it.

module hps_ocram_osd #(parameter ROWS = 16)
(
	input  wire        clk,          // clk_100m: the walker
	input  wire        clk_sys,      // the core clock: the writer

	/* verilator lint_off UNUSEDSIGNAL */   // GPO 25 is the DIAG freeze knob; without DIAG it is never read
	input  wire        gp_out25,
	/* verilator lint_on UNUSEDSIGNAL */

	// The master's session interface
	input  wire        s_hello,      // one cycle: the hello's first store is about to run
	input  wire        s_live,       // P_SESSION
	input  wire        s_tick,       // the poll period's pulse
	input  wire        bind_en,      // the master's osd_bind_q (H_BIND bit 11 read)
	output wire        c_rest,       // the enable link's idle: the half is at rest
	output wire        c_fault,      // one cycle: the ring stopped on a fault
	output wire        c_mid,        // level: a record is part-consumed
	output wire        c_open,       // the enable link's OPEN, the value C_BIND bit 11 reports
	output wire [15:0] osd_rows,     // ROW records finished this session
	output wire [15:0] osd_deepest,  // the deepest fill the walker has read this session

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
	output wire        we,           // one byte of every osd's osd_buffer
	output wire [12:0] addr,
	output wire  [7:0] dbyte
);

localparam [15:0] H_IDX2   = 16'h0018;      // host: the OSD ring's head
localparam [15:0] C_IDX2   = 16'h0098;      // fabric: the OSD ring's tail
localparam [15:0] OSD_BASE = 16'h2000;      // the OSD ring's first word
localparam [15:0] OSD_WORDS = 16'd2048;
localparam  [7:0] LAST_ROW = ROWS - 1;

// A ring span above 0xF000, or a control constant above 0xFF, stops elaboration.
generate
	if (OSD_BASE + OSD_WORDS * 4 > 16'hF000) begin : g_osd_span
		ocram_osd_span_above_0xF000 guard();
	end
	if (H_IDX2 >= 16'h0100 || C_IDX2 >= 16'h0100) begin : g_osd_ctrl
		ocram_osd_control_constant_above_0xFF guard();
	end
endgenerate

///////////////////////////////  the link  //////////////////////////////////

// The freeze knob: GPO 25 on clk_100m into an lw_sync on clk_sys; the constant 0 without DIAG.
`ifdef MISTER_OCRAM_DIAG
(* preserve *) reg frz_lwx = 1'b0;
always @(posedge clk) frz_lwx <= gp_out25;
wire frz;
lw_sync #(.W(1)) frz_sync(.clk(clk_sys), .d(frz_lwx), .q(frz));
`else
wire frz = 1'b0;
`endif

// The forward word {row, word, data} and the 1-bit done answer, each through one held-data handshake.
wire [42:0] word_q;
wire        word_v;
wire        fwd_busy;
wire        done_v;
wire        back_busy;

reg         xfer_go = 1'b0;            // the forward cell's s_go pulse
reg  [42:0] xfer_d = 43'd0;
reg         done_go = 1'b0;

lw_xfer #(.W(43)) osd_fwd
(
	.s_clk(clk),
	.s_go(xfer_go),
	.s_d(xfer_d),
	.s_busy(fwd_busy),
	.d_clk(clk_sys),
	.d_q(word_q),
	.d_v(word_v)
);

/* verilator lint_off PINCONNECTEMPTY */   // the done word's data has no user on clk_100m
lw_xfer #(.W(1)) osd_ack
(
	.s_clk(clk_sys),
	.s_go(done_go),
	.s_d(1'b1),
	.s_busy(back_busy),
	.d_clk(clk),
	.d_q(),
	.d_v(done_v)
);
/* verilator lint_on PINCONNECTEMPTY */

wire        hen = s_live & bind_en;
wire        f_open, f_idle, hen_s;
wire        rest;
reg         word_out = 1'b0;           // a word launched to clk_sys, its done not back

// The enable link: lw_osd's, with the walker's outstanding word as its drain.
wire take = hen & f_idle;
/* verilator lint_off PINCONNECTEMPTY */   // OWNED, the launched level and the fall have no user here
lw_fence fence
(
	.clk(clk), .hen(hen), .take(take), .drain(!word_out), .fin(1'b1),
	.open(f_open), .owned(), .idle(f_idle), .lvl(), .fall(),
	.clk_sys(clk_sys), .rest(rest), .hen_s(hen_s)
);
/* verilator lint_on PINCONNECTEMPTY */

///////////////////////////////  the walker  ////////////////////////////////

localparam [2:0] O_IDLE = 3'd0, O_HEAD = 3'd1, O_HDR = 3'd2, O_PAY = 3'd3,
                 O_XGO = 3'd4, O_XFER = 3'd5, O_TAIL = 3'd6, O_STOP = 3'd7;

reg  [2:0]  ost = O_IDLE;
reg         req = 1'b0;
reg         req_we = 1'b0;
reg  [15:0] req_off = 16'd0;
reg  [31:0] req_wdata = 32'd0;
reg  [15:0] tail = 16'd0;              // the ring's free-running word index
reg  [15:0] head_c = 16'd0;            // the last H_IDX[2] read
reg         head_due = 1'b1;           // read the head when caught up
reg  [4:0]  row = 5'd0;                // the ROW being written
reg  [6:0]  k = 7'd1;                  // its word, 1..64
wire [5:0]  widx = k[5:0] - 6'd1;       // the word in the row, 0..63
reg         mid_q = 1'b0;              // a record is part-consumed
reg         stopped_q = 1'b0;          // the ring stopped on a fault this session
reg         fault_q = 1'b0;            // one cycle
reg         was_row = 1'b0;            // the record being finished is a ROW
reg  [15:0] rows_q = 16'd0;
reg  [15:0] deepest_q = 16'd0;

assign c_req   = req;
assign c_we    = req_we;
assign c_off   = req_off;
assign c_wdata = req_wdata;
assign c_fault = fault_q;
assign c_mid   = mid_q;
assign c_open  = f_open;
assign c_rest  = f_idle;
assign osd_rows = rows_q;
assign osd_deepest = deepest_q;

// The word index the payload read of word 1 and of word k+1 address, and their ring offsets.
wire [15:0] pay1_idx = tail + 16'd1;
wire [15:0] pay_next = tail + {9'd0, k} + 16'd1;
wire [15:0] hdr_off  = OSD_BASE + ((tail & (OSD_WORDS - 1)) << 2);
wire [15:0] pay1_off = OSD_BASE + ((pay1_idx & (OSD_WORDS - 1)) << 2);
wire [15:0] pay2_off = OSD_BASE + ((pay_next & (OSD_WORDS - 1)) << 2);

wire [15:0] head_gap = head_c - tail;
wire [15:0] new_gap  = c_rdata[15:0] - tail;
wire [16:0] hdr_words = {1'b0, c_rdata[15:0]} + 17'd1;
wire [16:0] gap_words = {1'b0, head_gap};
wire        walk = s_live && f_open && !stopped_q;

always @(posedge clk) begin
	fault_q <= 1'b0;
	xfer_go <= 1'b0;

	if (s_hello) begin
		// the session's first store is about to run: no record survives the hello
		ost       <= O_IDLE;
		req       <= 1'b0;
		req_we    <= 1'b0;
		tail      <= 16'd0;
		head_c    <= 16'd0;
		head_due  <= 1'b1;
		mid_q     <= 1'b0;
		stopped_q <= 1'b0;
		was_row   <= 1'b0;
		rows_q    <= 16'd0;
		deepest_q <= 16'd0;
		word_out  <= 1'b0;
		k         <= 7'd1;
	end
	else begin
		if (s_tick) head_due <= 1'b1;

		case (ost)
		O_IDLE: if (walk) begin
			if (head_due && (head_c == tail)) begin
				req      <= 1'b1;
				req_we   <= 1'b0;
				req_off  <= H_IDX2;
				head_due <= 1'b0;
				ost      <= O_HEAD;
			end
			else if (head_gap != 16'd0) begin
				req     <= 1'b1;
				req_we  <= 1'b0;
				req_off <= hdr_off;
				mid_q   <= 1'b1;
				ost     <= O_HDR;
			end
		end
		O_HEAD: begin
			if (!s_live) begin req <= 1'b0; ost <= O_IDLE; end
			else if (c_grant) req <= 1'b0;
			else if (c_done) begin
				if (!c_ok) begin
					// a read with a non-OKAY response yields no value: read the head again
					req <= 1'b1;
				end
				else if (new_gap > OSD_WORDS) begin
					// a head more than the ring's size ahead: a host defect
					stopped_q <= 1'b1;
					fault_q   <= 1'b1;
					req       <= 1'b0;
					mid_q     <= 1'b0;
					ost       <= O_STOP;
				end
				else begin
					head_c <= c_rdata[15:0];
					if (new_gap > deepest_q) deepest_q <= new_gap;
					if (c_rdata[15:0] == tail) ost <= O_IDLE;
					else begin
						req     <= 1'b1;
						req_we  <= 1'b0;
						req_off <= hdr_off;
						mid_q   <= 1'b1;
						ost     <= O_HDR;
					end
				end
			end
		end
		O_HDR: begin
			if (!s_live) begin req <= 1'b0; ost <= O_IDLE; end
			else if (c_grant) req <= 1'b0;
			else if (c_done) begin
				if (!c_ok) begin
					req <= 1'b1;                    // read it again
				end
				else if (hdr_words > gap_words || hdr_words > {1'b0, OSD_WORDS}) begin
					// a length past the head or the ring: a host defect
					stopped_q <= 1'b1;
					fault_q   <= 1'b1;
					req       <= 1'b0;
					mid_q     <= 1'b0;
					ost       <= O_STOP;
				end
				else if (c_rdata[31:24] == 8'h01 && c_rdata[15:0] == 16'd64 &&
				         c_rdata[23:16] <= LAST_ROW) begin
					row     <= c_rdata[20:16];
					k       <= 7'd1;
					was_row <= 1'b1;
					req     <= 1'b1;
					req_we  <= 1'b0;
					req_off <= pay1_off;
					ost     <= O_PAY;
				end
				else begin
					// an unknown kind, a ROW of another length or a row outside the OSD: skipped by its length
					tail      <= tail + 16'd1 + c_rdata[15:0];
					req       <= 1'b1;
					req_we    <= 1'b1;
					req_off   <= C_IDX2;
					req_wdata <= {16'd0, tail + 16'd1 + c_rdata[15:0]};
					was_row   <= 1'b0;
					ost       <= O_TAIL;
				end
			end
		end
		O_PAY: begin
			if (!s_live) begin req <= 1'b0; ost <= O_IDLE; end
			else if (c_grant) req <= 1'b0;
			else if (c_done) begin
				if (!c_ok) req <= 1'b1;                 // read it again
				else if (!s_live) begin
					ost <= O_IDLE;                      // the record is abandoned
				end
				else begin
					xfer_d   <= {row, widx, c_rdata};
					word_out <= 1'b1;
					ost      <= O_XGO;
				end
			end
		end
		O_XGO: begin
			if (!s_live) begin
				word_out <= 1'b0;                       // not handed over: nothing drains
				ost      <= O_IDLE;
			end
			else if (!fwd_busy) begin
				xfer_go <= 1'b1;
				ost     <= O_XFER;
			end
		end
		O_XFER: if (done_v) begin
			word_out <= 1'b0;
			if (!s_live) begin
				ost <= O_IDLE;                          // the record is abandoned
			end
			else if (k == 7'd64) begin
				tail      <= tail + 16'd65;
				req       <= 1'b1;
				req_we    <= 1'b1;
				req_off   <= C_IDX2;
				req_wdata <= {16'd0, tail + 16'd65};
				ost       <= O_TAIL;
			end
			else begin
				k       <= k + 7'd1;
				req     <= 1'b1;
				req_we  <= 1'b0;
				req_off <= pay2_off;
				ost     <= O_PAY;
			end
		end
		O_TAIL: begin
			if (!s_live && req) begin
				req <= 1'b0;                            // no grant came: the tail is not stored
				ost <= O_IDLE;
			end
			else if (c_grant) begin
				req   <= 1'b0;
				mid_q <= 1'b0;                          // the tail store is launched: the record is consumed
				if (was_row) rows_q <= rows_q + 16'd1;
				head_due <= 1'b1;
			end
			else if (c_done && !c_ok) req <= 1'b1;      // a failed store is tried again
			else if (c_done) ost <= O_IDLE;
		end
		O_STOP: ;
		default: ost <= O_IDLE;
		endcase
	end
end

///////////////////////////////  the writer  ////////////////////////////////

reg  [42:0] pend_q = 43'd0;            // the word the half holds, frozen or not
reg         pend_v = 1'b0;
reg   [3:0] bmask = 4'hF;              // the word's bytes not yet written
reg   [1:0] qn = 2'd0;                 // quiet cycles before this one, saturating at 2

wire quiet = (sel == 2'b00) && view;
wire  [1:0] b = bmask[0] ? 2'd0 : bmask[1] ? 2'd1 : bmask[2] ? 2'd2 : 2'd3;
wire  [3:0] bmask_n = bmask & ~(4'd1 << b);
wire wnow = hen_s && !frz && pend_v && quiet && qn == 2'd2;

assign we    = wnow;
assign addr  = {pend_q[42:38], pend_q[37:32], b};
assign dbyte = pend_q[{1'b0, b, 3'd0} +: 8];

// The half is at rest with the enable low, no word held or arriving, and both handshakes settled. While frozen
// it is never at rest, which is what a stopped clk_sys looks like to the link.
assign rest = !frz && !hen_s && !pend_v && !back_busy && !word_v;

always @(posedge clk_sys) begin
	done_go <= 1'b0;

	if (word_v) begin
		// the cell's word reaches the pending register even while frozen: no word is lost
		pend_q <= word_q;
		pend_v <= 1'b1;
		bmask  <= 4'hF;
	end
	else if (pend_v && !hen_s && !frz) begin
		// with the enable low the word is discarded and still answered, so both halves converge
		pend_v  <= 1'b0;
		done_go <= 1'b1;
	end
	else if (wnow) begin
		bmask <= bmask_n;
		if (bmask_n == 4'd0) begin
			pend_v  <= 1'b0;
			done_go <= 1'b1;             // the fourth byte is in this cycle: answer it
		end
	end

	qn <= !quiet ? 2'd0 : (qn == 2'd2) ? 2'd2 : qn + 2'd1;
end

endmodule

`default_nettype wire
`endif
`endif
