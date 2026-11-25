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

`ifdef MISTER_LW
`default_nettype none
`ifdef LW_TB_CDC
`timescale 1ns/1ps
`endif

// The clock-crossing cells. Every register of a crossing between clk_100m and a core clock lives in this
// file; a block instantiates these cells and declares no synchroniser of its own (tb/static.sh holds it).
// Register names, for the constraints and the static check:
//   lws0_*, lws1_*  first and second synchroniser stage (lw_sync)
//   lwa0_*, lwa1_*  the asynchronously set chain (lw_arst)
//   lwq_*           a bundle's capture register (lw_qstat)
//   lwsrc_*         a bundle's source hold register (lw_xfer); only a data hold register carries this name
//   lwtg_*          a handshake's two toggles (lw_xfer)
// In the instantiating blocks, a register that feeds an lw_sync is named *_lwx, or *_lwgray for a Gray pointer
// (recomputed from its binary pointer in every cycle), and carries a preserve attribute.
// LW_TB_CDC (testbench only) adds delay and aperture models: a zero-delay simulator otherwise never resolves a
// synchroniser late, never skews a bus and never tears a capture. The seed is the plusarg lw_seed.

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

`ifndef LW_TB_CDC
always @(posedge clk) begin
	lws0_q <= d;
	lws1_q <= lws0_q;
end
`else
// GRAY = 0: every bit, on every edge of its input, takes a random extra destination cycle or not.
// GRAY = 1: each bit arrives after a transport delay of up to 5 ns (the skew bound), fixed per run; a bit whose
// delayed edge falls within 300 ps of the sampling edge resolves either way; the whole bus may take an extra
// cycle. No per-bit extra cycle on a Gray bus: it would make codes the skew bound rules out.
integer          m_seed, m_n = 0;       // m_n: edges the model moved
integer          m_dl [0:W-1];          // per-bit transport delay, ps
reg      [W-1:0] m_e = {W{1'b0}};       // each bit, 300 ps before and after its delayed arrival
reg      [W-1:0] m_l = {W{1'b0}};
reg      [W-1:0] m_x = {W{1'b0}};
reg              m_xb = 1'b0;

initial begin : m_init
	string  nm;
	integer s, i;
	if (!$value$plusargs("lw_seed=%d", s)) s = 1;
	nm = $sformatf("%m");
	m_seed = s;
	for (i = 0; i < nm.len(); i = i + 1) m_seed = m_seed * 31 + nm[i];
	for (i = 0; i < W; i = i + 1) m_dl[i] = GRAY ? $unsigned($random(m_seed)) % 5001 : 0;
	#0;
	m_e = d;
	m_l = d;
end

genvar b;
for (b = 0; b < W; b = b + 1) begin : gm
	always @(d[b]) begin
		m_e[b] <= #((m_dl[b] > 300 ? m_dl[b] - 300 : 0) / 1000.0) d[b];
		m_l[b] <= #((m_dl[b] + 300) / 1000.0) d[b];
	end
end

always @(posedge clk) begin : m_step
	integer     i;
	reg [W-1:0] s;
	if (GRAY) begin
		for (i = 0; i < W; i = i + 1) s[i] = (m_e[i] === m_l[i] || ($random(m_seed) & 1)) ? m_e[i] : m_l[i];
		if (s !== lws0_q && !m_xb && ($random(m_seed) & 3) == 0) begin
			m_xb <= 1'b1;
			m_n   = m_n + 1;
		end
		else begin
			lws0_q <= s;
			m_xb   <= 1'b0;
		end
	end
	else begin
		s = lws0_q;
		for (i = 0; i < W; i = i + 1)
			if (d[i] !== lws0_q[i] && !m_x[i] && ($random(m_seed) & 1)) begin
				m_x[i] <= 1'b1;
				m_n     = m_n + 1;
			end
			else begin
				s[i]    = d[i];
				m_x[i] <= 1'b0;
			end
		lws0_q <= s;
	end
	lws1_q <= lws0_q;
end
`endif

endmodule

// An asynchronous reset input made a clean clk level. Two flops are set asynchronously while rst_n is low and
// shift in 0 on clk, so a low pulse is caught even with clk stopped; the chain's rise is asynchronous, so its
// output crosses through an lw_sync. q is high from configuration, and from each fall of rst_n, at most three clk
// edges after it; it falls on the fourth clk edge after rst_n is high again. However short the pulse, q is high
// for at least one cycle: the chain's synchronous fall cannot overtake its rise, which may resolve an edge late.
// rst_n is the output of a hard HPS register.
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

`ifndef LW_TB_CDC
always @(posedge clk) if (en) lwq_d <= d;
`else
// each bit arrives after a transport delay of up to 8 ns (the net-delay bound), fixed per run; m_n counts
// captures that took a bit still in flight, which a handshake within its bound never does
integer          m_seed, m_n = 0;
integer          m_dl [0:W-1];
reg      [W-1:0] m_d = {W{1'b0}};

initial begin : m_init
	string  nm;
	integer s, i;
	if (!$value$plusargs("lw_seed=%d", s)) s = 1;
	nm = $sformatf("%m");
	m_seed = s;
	for (i = 0; i < nm.len(); i = i + 1) m_seed = m_seed * 31 + nm[i];
	for (i = 0; i < W; i = i + 1) m_dl[i] = $unsigned($random(m_seed)) % 8001;
	#0;
	m_d = d;
end

genvar b;
for (b = 0; b < W; b = b + 1) begin : gm
	always @(d[b]) m_d[b] <= #(m_dl[b] / 1000.0) d[b];
end

always @(posedge clk) if (en) begin
	lwq_d <= m_d;
	if (m_d !== d) m_n = m_n + 1;
end
`endif

endmodule

// A held-data toggle handshake. The source takes s_d into its hold register and flips its toggle on s_go while
// not busy; the toggle crosses through lw_sync; the destination, whenever the synchronised toggle differs from
// its acknowledge toggle, captures the hold register into an lw_qstat and takes the toggle's value, and d_v
// pulses the cycle after, with the word in d_q. The acknowledge toggle crosses back; the source is busy while
// its toggle differs from it. Both sides compare levels, so any state of the registers converges: an upset
// costs repeats of the held word (at most three) or one lost word.
// Bound: the word is captured two destination cycles after its toggle is launched, so two destination periods
// must exceed the constraints' 8 ns net delay plus setup.
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
// four-phase handshake on them. hen_lwx (clk) is the block's enable level; through lw_sync it is hen_s, the
// only thing that tells the clk_sys half about the host enable. ack_lwx (clk_sys) is recomputed in every cycle as
// rest & (rest_n == 3): high after four consecutive cycles of rest, low within one cycle of hen_s rising, and
// holding no history. rest is the half's own predicate: hen_s low, its machines idle with idle outputs, its
// queues empty, nothing in flight, every Gray source equal to its pointer's code.
// On clk, five states (the level, OPEN / OWNED):
//   DOWN  (0, 0/1) to IDLE once ack_s has read 1 in ten consecutive cycles (two to believe it, eight for what the
//         half launched before it answered) and fin holds
//   IDLE  (0, 0/0) to RISE at an ENABLE taken while ack_s reads 1; back to DOWN if ack_s reads 0
//   RISE  (1, 0/~hen) once ack_s has read 0 in two consecutive cycles: to OPEN if hen, else to DRAIN
//   OPEN  (1, 1/0) to DRAIN in any cycle hen reads 0
//   DRAIN (1, 0/1) to DOWN once drain has held for four cycles: the level falls 40 ns after the block's clk half
//         launched its last word, so that word reaches clk_sys no later than the fall
// So the level rises only on an acknowledge of rest, and falls only after the half has answered the rise; an enable
// that falls inside the rise leaves the level up, the gate closed, until the half answers. Configuration starts
// in DOWN. No reset: nothing reaches the link but hen and a taken ENABLE.
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

`default_nettype wire
`endif
