//============================================================================
//  ocram_arb.sv -- part of the OCRAM mailbox (docs/OCRAM_MAILBOX.md)
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

// The mailbox's access layer: one bus request port per requester, {req, we, off, wdata, counted} held until grant,
// and a pick of one in an idle cycle for ocram_axi.sv. Ports 0 to F-1 go in fixed order, port 0 first; then the
// two consumer ports F and F + 1, round robin between them (the last granted goes last). The pick is a mux of the
// ports' own values, registers or a small table's words but for the EVT writer's (ocram_evt_wr.sv): its offset is
// one 16-bit add of registers, and its pick is one compare per variable-length source.
//
//  * en is the engine's idle cycle less every cycle the session forbids a start.
//  * A consumer port's completion is its own: c_done[n] is the done of the transaction it was granted, and kill
//    (an HPS reset's rise) leaves one in flight unanswered. c_rdata is 0 for a write. The other ports read the
//    engine's done themselves.

module ocram_arb #(parameter integer F = 7)
(
	input  wire              clk,
	input  wire              en,
	input  wire              kill,
	input  wire [F+1:0]      req,
	input  wire [F+1:0]      we,
	input  wire [16*F+31:0]  off,
	input  wire [32*F+63:0]  wdata,
	input  wire [F+1:0]      counted,
	output wire [F+1:0]      grant,

	// to ocram_axi
	output wire              t_go,
	output reg               t_we,
	output reg  [15:0]       t_off,
	output reg  [31:0]       t_wdata,
	output reg               t_counted,
	input  wire              t_done,
	input  wire [31:0]       rdata,

	// the consumer ports' completions
	output wire  [1:0]       c_done,
	output wire [31:0]       c_rdata
);

reg  c_last = 1'b0;                    // the consumer port granted last: 0 or 1
reg  c_act_v = 1'b0;                   // a consumer transaction is in flight
reg  c_act_port = 1'b0;
reg  c_act_we = 1'b0;

reg  [F+1:0] sel;
always @(*) begin : pick
	integer i;
	sel = {(F+2){1'b0}};
	if (req[F+1] && (!req[F] || !c_last)) sel[F+1] = 1'b1;
	if (req[F] && (!req[F+1] || c_last)) sel = {{(F+1){1'b0}}, 1'b1} << F;
	for (i = F - 1; i >= 0; i = i - 1)
		if (req[i]) sel = {{(F+1){1'b0}}, 1'b1} << i;
end
assign grant = en ? sel : {(F+2){1'b0}};
assign t_go  = grant != {(F+2){1'b0}};

always @(*) begin : mux
	integer i;
	t_we = 1'b0; t_off = 16'h0000; t_wdata = 32'd0; t_counted = 1'b1;
	for (i = 0; i < F + 2; i = i + 1)
		if (sel[i]) begin
			t_we      = we[i];
			t_off     = off[16*i +: 16];
			t_wdata   = wdata[32*i +: 32];
			t_counted = counted[i];
		end
end

assign c_done  = {t_done & c_act_v & c_act_port, t_done & c_act_v & ~c_act_port};
assign c_rdata = c_act_we ? 32'd0 : rdata;

always @(posedge clk) begin
	if (kill) c_act_v <= 1'b0;
	if (t_done && c_act_v) c_act_v <= 1'b0;
	if (grant[F] || grant[F+1]) begin
		c_act_v    <= 1'b1;
		c_act_port <= grant[F+1];
		c_act_we   <= grant[F+1] ? we[F+1] : we[F];
		c_last     <= grant[F+1];
	end
end

endmodule

`default_nettype wire
`endif
