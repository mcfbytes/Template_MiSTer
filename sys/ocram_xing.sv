//============================================================================
//  ocram_xing.sv -- part of the OCRAM mailbox (docs/OCRAM_MAILBOX.md)
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
`define LW_OCRAM_XING_BUILT
`endif
`ifdef MISTER_OCRAM_SECTOR_SLOT
`define LW_OCRAM_XING_BUILT
`endif
`ifdef MISTER_OCRAM_FIO
`define LW_OCRAM_XING_BUILT
`endif
`ifdef LW_OCRAM_XING_BUILT
`default_nettype none

// hps_ocram_rx's crossing, the one module of the consumer on two clocks: every lw_xfer, lw_sync and lw_fence, and
// every register a crossing reads. Forward, one held-data word at a time {tag, wr, k, d}: 0 OSD_HEAD, 1 SEC_HEAD, 2
// DATA (of the last head's ring), 3 FIO_HEAD; on clk_sys it is a record sink entry for the OSD writer, the
// SECTOR feeder or the download player, held in the capture register (w_d) until answered. Back, the answer {d,
// code} (0 took, 1 accept, 2 mismatch, 3 no request), one a cycle in a fixed order (feeder, player, OSD writer: a
// word is outstanding alone, so they never meet). Without MNT_EN and the capture lane only {tag, d} and the code
// cross. Beside them: the bind levels (clk) and en_sys and io_wide (clk_sys) through lw_sync; the request records
// from clk_sys; the DIAG freeze knob (GPO 25); and the enable link: hen = s_live, take while idle, drain while no
// word waits for its done, rest = the clk_sys half at rest (the sinks idle, nothing held, arriving or owed).

module ocram_xing #(parameter OSD_EN = 1, parameter SEC_EN = 0, parameter FIO_EN = 0, parameter [1:0] MNT_EN = 2'd0)
(
	// clk_100m: the walker's side
	input  wire        clk,
	/* verilator lint_off UNUSEDSIGNAL */   // GPO 25 is the DIAG freeze knob; without DIAG it is never read
	input  wire        gp_out25,
	/* verilator lint_on UNUSEDSIGNAL */
	input  wire        s_live,
	input  wire        x_go,         // launch x_d
	/* verilator lint_off UNUSEDSIGNAL */   // without MNT_EN the forward word's wr and k do not cross
	input  wire [36:0] x_d,
	/* verilator lint_on UNUSEDSIGNAL */
	output wire        fwd_busy,
	input  wire        x_out,        // a word launched, its done not back
	output wire [33:0] done_q,       // the last answer {d, code}
	output wire        done_v,
	input  wire  [2:0] bind_d,       // the bind levels' next value
	output wire  [2:0] bind_q,       // the bind levels
	output wire  [2:0] en_b,         // en_sys, back
	output wire        wide_b,       // io_wide, back
	output wire [32:0] rq_q,         // the disc slot's request record
	output wire        rq_v,
	output wire [41:0] mq,           // a mount slot's request record
	output wire        mq_v,
	output wire        f_open,       // the enable link's open and idle
	output wire        f_idle,

	// clk_sys: the sinks' side
	input  wire        clk_sys,
	output wire        hen_s,        // the session's enable level
	output wire        frz,          // the DIAG freeze
	output wire  [2:0] bind_s,       // the bind levels
	output wire [31:0] w_d,          // the entry, held until answered
	output wire        w_wr,         // a SEC_HEAD's direction and slot index
	output wire  [1:0] w_k,
	output wire        o_v,          // one cycle: an entry for the OSD writer, a ROW's head with o_head
	output wire        o_head,
	output wire        s_v,          // for the feeder, a SEC_HEAD with s_head
	output wire        s_head,
	output wire        f_v,          // for the player, an LFRAME header with f_head
	output wire        f_head,
	input  wire        o_go,         // the answers
	input  wire        o_rest,
	input  wire        f_go,
	input  wire  [1:0] f_code,
	input  wire [31:0] f_dat,
	input  wire        f_rest,
	input  wire        p_go,
	/* verilator lint_off UNUSEDSIGNAL */   // without MISTER_OCRAM_LANE the replies are not read
	input  wire [31:0] p_dat,
	/* verilator lint_on UNUSEDSIGNAL */
	input  wire        p_rest,
	input  wire        rc_go,        // the request records
	input  wire [32:0] rc_d,
	output wire        rc_busy,
	/* verilator lint_off UNUSEDSIGNAL */   // without MNT_EN the mount channel's record is not sent
	input  wire        mrc_go,
	input  wire [41:0] mrc_d,
	/* verilator lint_on UNUSEDSIGNAL */
	output wire        mrc_busy,
	input  wire        q_rest,
	/* verilator lint_off UNUSEDSIGNAL */   // without MNT_EN the mount slots' en_sys do not cross
	input  wire        x_en,         // en_sys and io_wide
	input  wire  [1:0] xm_en,
	/* verilator lint_on UNUSEDSIGNAL */
	input  wire        x_wide
);

`ifdef MISTER_OCRAM_LANE
localparam        CAP_EN    = 1;            // the capture lane: its answers carry the replies
`else
localparam        CAP_EN    = 0;
`endif
localparam [1:0] TG_OHEAD = 2'd0, TG_SHEAD = 2'd1, TG_DATA = 2'd2, TG_FHEAD = 2'd3;
localparam [1:0] RG_OSD = 2'd0, RG_SEC = 2'd1, RG_FIO = 2'd2;   // the ring of the last head
localparam [1:0] A_TOOK = 2'd0;

`ifdef MISTER_OCRAM_DIAG
(* preserve *) reg frz_lwx = 1'b0;
always @(posedge clk) frz_lwx <= gp_out25;
lw_sync #(.W(1)) frz_sync(.clk(clk_sys), .d(frz_lwx), .q(frz));
`else
assign frz = 1'b0;
`endif

wire [36:0] word_q;                    // clk_sys: the capture register {tag, wr, k, d}
wire        word_v;
wire        back_busy;
reg         done_go = 1'b0;            // clk_sys: the answer
reg   [1:0] done_code = 2'd0;
/* verilator lint_off UNUSEDSIGNAL */   // without MNT_EN and the capture lane the answer's d does not cross
reg  [31:0] done_dat = 32'd0;
/* verilator lint_on UNUSEDSIGNAL */

generate
if (MNT_EN != 0 || CAP_EN != 0) begin : xw
	lw_xfer #(.W(37)) rx_fwd(.s_clk(clk), .s_go(x_go), .s_d(x_d), .s_busy(fwd_busy),
	                         .d_clk(clk_sys), .d_q(word_q), .d_v(word_v));
	lw_xfer #(.W(34)) rx_done(.s_clk(clk_sys), .s_go(done_go), .s_d({done_dat, done_code}), .s_busy(back_busy),
	                          .d_clk(clk), .d_q(done_q), .d_v(done_v));
end
else begin : xn
	wire [33:0] fq;
	wire  [1:0] dq;
	lw_xfer #(.W(34)) rx_fwd(.s_clk(clk), .s_go(x_go), .s_d({x_d[36:35], x_d[31:0]}), .s_busy(fwd_busy),
	                         .d_clk(clk_sys), .d_q(fq), .d_v(word_v));
	lw_xfer #(.W(2)) rx_done(.s_clk(clk_sys), .s_go(done_go), .s_d(done_code), .s_busy(back_busy),
	                         .d_clk(clk), .d_q(dq), .d_v(done_v));
	assign word_q = {fq[33:32], 3'd0, fq[31:0]};
	assign done_q = {32'd0, dq};
end
endgenerate

// the request record {offered, lba} and the bind levels each way, by slot index k
(* preserve *) reg [2:0] bind_lwx = 3'd0;   // clk: the bind levels
/* verilator lint_off UNUSEDSIGNAL */   // without MNT_EN bits 2:1 do not cross
(* preserve *) reg [2:0] en_lwx = 3'd0;     // clk_sys: en_sys
/* verilator lint_on UNUSEDSIGNAL */
(* preserve *) reg wide_lwx = 1'b0;         // clk_sys: io_wide
always @(posedge clk) bind_lwx <= bind_d;
assign bind_q = bind_lwx;
lw_xfer #(.W(33)) rx_req(.s_clk(clk_sys), .s_go(rc_go), .s_d(rc_d), .s_busy(rc_busy),
                         .d_clk(clk), .d_q(rq_q), .d_v(rq_v));
generate
if (MNT_EN != 0) begin : sw
	lw_sync #(.W(3)) bind_sync(.clk(clk_sys), .d(bind_lwx), .q(bind_s));
	lw_sync #(.W(4)) back_sync(.clk(clk), .d({wide_lwx, en_lwx}), .q({wide_b, en_b}));
end
else begin : sn
	// the disc slot's levels alone: the mount bits stay 0 and do not cross
	lw_sync #(.W(1)) bind_sync(.clk(clk_sys), .d(bind_lwx[0]), .q(bind_s[0]));
	lw_sync #(.W(2)) back_sync(.clk(clk), .d({wide_lwx, en_lwx[0]}), .q({wide_b, en_b[0]}));
	assign bind_s[2:1] = 2'd0, en_b[2:1] = 2'd0;
end
endgenerate

// the mount channel's record {k, wr, blk_cnt, offered, lba}
generate
if (MNT_EN != 0) begin : mx
	lw_xfer #(.W(42)) rx_mreq(.s_clk(clk_sys), .s_go(mrc_go), .s_d(mrc_d), .s_busy(mrc_busy),
	                          .d_clk(clk), .d_q(mq), .d_v(mq_v));
end
else begin : nomx
	assign mq = 42'd0, mq_v = 1'b0, mrc_busy = 1'b0;
end
endgenerate

wire        rest;
wire        take = s_live & f_idle;
/* verilator lint_off PINCONNECTEMPTY */   // OWNED, the launched level and the fall have no user here
lw_fence fence
(
	.clk(clk), .hen(s_live), .take(take), .drain(!x_out), .fin(1'b1),
	.open(f_open), .owned(), .idle(f_idle), .lvl(), .fall(),
	.clk_sys(clk_sys), .rest(rest), .hen_s(hen_s)
);
/* verilator lint_on PINCONNECTEMPTY */

// the sinks: a head names its ring, a DATA word goes to the last head's
reg   [1:0] cur = RG_OSD;
wire  [1:0] tag = word_q[36:35];
assign w_wr   = word_q[34];
assign w_k    = word_q[33:32];
assign w_d    = word_q[31:0];
assign o_head = tag == TG_OHEAD;
assign s_head = tag == TG_SHEAD;
assign f_head = tag == TG_FHEAD;
assign o_v    = OSD_EN != 0 && word_v && (o_head || (tag == TG_DATA && cur == RG_OSD));
assign s_v    = SEC_EN != 0 && word_v && (s_head || (tag == TG_DATA && cur == RG_SEC));
assign f_v    = FIO_EN != 0 && word_v && (f_head || (tag == TG_DATA && cur == RG_FIO));

// The half is at rest with the enable and the bind levels low, no word held or arriving, nothing owed, every
// handshake settled. While frozen it is never at rest, which is what a stopped clk_sys looks like to the link.
assign rest = !frz && !hen_s && bind_s == 3'd0 && f_rest && p_rest && o_rest && !back_busy && !word_v &&
              !done_go && q_rest && !rc_busy && !mrc_busy;

always @(posedge clk_sys) begin
	done_go  <= 1'b0;
	en_lwx   <= {xm_en, x_en};
	wide_lwx <= x_wide;
	if (word_v && tag != TG_DATA) cur <= s_head ? RG_SEC : f_head ? RG_FIO : RG_OSD;
	if (f_go) begin
		done_go   <= 1'b1;
		done_code <= f_code;
		done_dat  <= f_dat;
	end
	else if (p_go) begin
		done_go   <= 1'b1;
		done_code <= A_TOOK;
`ifdef MISTER_OCRAM_LANE
		done_dat  <= p_dat;                 // the ring word's two link replies
`endif
	end
	else if (o_go) begin
		done_go   <= 1'b1;
		done_code <= A_TOOK;
	end
end

endmodule

`default_nettype wire
`endif
`undef LW_OCRAM_XING_BUILT
`endif
