//============================================================================
//  lw_cdc.sv -- part of the LW bridge framework (docs/LW_BRIDGE.md)
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

// The crossing cells serve either floor. The derived macro is local to this file and undefined at its end, so
// a build of neither floor still finds no macro left defined.
`ifdef MISTER_LW
`define LW_CDC_BUILT
`endif
`ifdef MISTER_OCRAM
`define LW_CDC_BUILT
`endif

`ifdef LW_CDC_BUILT
`default_nettype none

/* verilator lint_off DECLFILENAME */   // the crossing cells share this file, as the LW framework lays them out
/* verilator lint_off UNUSEDPARAM */

// The clock-crossing cells.

// Two flops per bit with the synchroniser attribute: no enable, no reset, nothing but the source register on D.
// GRAY = 1 marks a Gray-coded pointer: only its delay model differs. INIT is both stages' configuration value.
module lw_sync #(parameter W = 1, parameter GRAY = 0, parameter [0:0] INIT = 1'b0)
(
	input wire          clk,        // the destination clock
	input wire  [W-1:0] d,          // a source register, in the source clock's domain
	output wire [W-1:0] q
);

(* altera_attribute = {"-name SYNCHRONIZER_IDENTIFICATION FORCED_IF_ASYNCHRONOUS"} *) reg [W-1:0] lws0_q = {W{INIT}};
(* altera_attribute = {"-name SYNCHRONIZER_IDENTIFICATION FORCED_IF_ASYNCHRONOUS"} *) reg [W-1:0] lws1_q = {W{INIT}};

assign q = lws1_q;

always @(posedge clk) begin
	lws0_q <= d;
	lws1_q <= lws0_q;
end

endmodule

// An asynchronous reset input made a clean clk level.
module lw_arst
(
	input wire  clk,
	input wire  rst_n,
	output wire q
);

(* altera_attribute = {"-name SYNCHRONIZER_IDENTIFICATION FORCED_IF_ASYNCHRONOUS"} *) reg lwa0_q = 1'b1;
(* altera_attribute = {"-name SYNCHRONIZER_IDENTIFICATION FORCED_IF_ASYNCHRONOUS"} *) reg lwa1_q = 1'b1;

always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		lwa0_q <= 1'b1;
		lwa1_q <= 1'b1;
	end
	else begin
		lwa0_q <= 1'b0;
		lwa1_q <= lwa0_q;
	end
end

lw_sync #(.W(1), .INIT(1'b1)) sync(.clk(clk), .d(lwa1_q), .q(q));

endmodule

// The capture register of a held bundle: D is the source hold register, the enable is local logic. Used only
// inside lw_xfer.
module lw_qstat #(parameter W = 1)
(
	input wire          clk,        // the destination clock
	input wire          en,         // local logic
	input wire  [W-1:0] d,          // the source hold register (lwsrc_*), stable while en can rise
	output wire [W-1:0] q
);

(* preserve *) reg [W-1:0] lwq_d = {W{1'b0}};

assign q = lwq_d;

always @(posedge clk) if (en) lwq_d <= d;

endmodule

// A held-data toggle handshake.
module lw_xfer #(parameter W = 1)
(
	input wire          s_clk,
	input wire          s_go,       // take s_d and launch; ignored while s_busy
	input wire  [W-1:0] s_d,
	output wire         s_busy,

	input wire          d_clk,
	output wire [W-1:0] d_q,        // the last word captured
	output reg          d_v = 1'b0  // one d_clk cycle: d_q holds a new word
);

(* preserve *) reg [W-1:0] lwsrc_d = {W{1'b0}};
reg                        lwtg_r  = 1'b0;   // the source's toggle
reg                        lwtg_a  = 1'b0;   // the destination's acknowledge toggle

wire tg_s, ack_s;
lw_sync #(.W(1)) sync_r(.clk(d_clk), .d(lwtg_r), .q(tg_s));
lw_sync #(.W(1)) sync_a(.clk(s_clk), .d(lwtg_a), .q(ack_s));

assign s_busy = lwtg_r ^ ack_s;

always @(posedge s_clk) if (s_go && !s_busy) begin
	lwsrc_d <= s_d;
	lwtg_r  <= ~lwtg_r;
end

wire take = tg_s ^ lwtg_a;

lw_qstat #(.W(W)) cap(.clk(d_clk), .en(take), .d(lwsrc_d), .q(d_q));

always @(posedge d_clk) begin
	lwtg_a <= tg_s;
	d_v    <= take;
end

endmodule

// An enable link: one level each way between the window and a block whose queue a clk_sys half consumes, and a
// four-phase handshake on them.
module lw_fence
(
	input wire  clk,        // clk_100m
	input wire  hen,        // the host enable register
	input wire  take,       // an ENABLE store is taken in this cycle
	input wire  drain,      // the block's clk half has nothing left to launch for a fallen enable
	input wire  fin,        // the block's last condition before IDLE (1 if it has none)
	output wire open,       // the block takes stores: its OPEN bit
	output wire owned,      // its OWNED bit
	output wire idle,       // IDLE with the acknowledge reading 1: this link lets an ENABLE through
	output wire lvl,        // the enable level as launched
	output wire fall,       // the level falls at this edge

	input wire  clk_sys,
	input wire  rest,       // the clk_sys half is at rest, combinational over its own registers
	output wire hen_s       // the enable level on clk_sys
);

localparam [2:0] L_DOWN = 3'd0, L_IDLE = 3'd1, L_RISE = 3'd2, L_OPEN = 3'd3, L_DRAIN = 3'd4;

reg            [2:0] st = L_DOWN;
reg            [3:0] n  = 4'd0;
(* preserve *) reg   hen_lwx = 1'b0;
(* preserve *) reg   ack_lwx   = 1'b0;     // clk_sys
reg            [1:0] rest_n    = 2'd0;     // clk_sys: consecutive cycles of rest, saturating at 3

wire ack_s;
lw_sync #(.W(1)) ack_sync(.clk(clk), .d(ack_lwx), .q(ack_s));
lw_sync #(.W(1)) hen_sync(.clk(clk_sys), .d(hen_lwx), .q(hen_s));

reg  [2:0] st_d;
reg  [3:0] n_d;
always @(*) begin
	st_d = st;
	n_d  = n;
	case (st)
		L_DOWN:
			if (!ack_s) n_d = 4'd0;
			else if (n < 4'd9) n_d = n + 4'd1;
			else if (fin) begin st_d = L_IDLE; n_d = 4'd0; end
		L_IDLE:
			if (take && ack_s) begin st_d = L_RISE; n_d = 4'd0; end
			else if (!ack_s) begin st_d = L_DOWN; n_d = 4'd0; end
		L_RISE:
			if (ack_s) n_d = 4'd0;
			else if (n == 4'd0) n_d = 4'd1;
			else begin st_d = hen ? L_OPEN : L_DRAIN; n_d = 4'd0; end
		L_OPEN:
			if (!hen) begin st_d = L_DRAIN; n_d = 4'd0; end
		L_DRAIN:
			if (!drain) n_d = 4'd0;
			else if (n < 4'd3) n_d = n + 4'd1;
			else begin st_d = L_DOWN; n_d = 4'd0; end
		default: begin st_d = L_DOWN; n_d = 4'd0; end
	endcase
end

wire up_d = st_d == L_RISE || st_d == L_OPEN || st_d == L_DRAIN;

always @(posedge clk) begin
	st        <= st_d;
	n         <= n_d;
	hen_lwx <= up_d;
end

assign open  = st == L_OPEN;
assign owned = st == L_DOWN || st == L_DRAIN || (st == L_RISE && !hen) || st > L_DRAIN;
assign idle  = st == L_IDLE && ack_s;
assign lvl   = hen_lwx;
assign fall  = hen_lwx && !up_d;

always @(posedge clk_sys) begin
	rest_n  <= !rest ? 2'd0 : (rest_n == 2'd3) ? 2'd3 : rest_n + 2'd1;
	ack_lwx <= rest && rest_n == 2'd3;
end

endmodule

/* verilator lint_on UNUSEDPARAM */
/* verilator lint_on DECLFILENAME */
`default_nettype wire
`endif
`undef LW_CDC_BUILT
