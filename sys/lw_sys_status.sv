//============================================================================
//  lw_sys_status.sv -- part of the LW bridge framework (docs/LW_BRIDGE.md)
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

// SYS block (LW directory kind 4, "STS1"): the core->host status back-channel (hps_io's 0x29) as a
// cause bit, its own f2h_irq line, and a seqlock-style snapshot of status_req: read STFLG, the four
// STATUS words, then STFLG again; equal STFLG words mean the words between them are one snapshot.
// clk_sys side: hps_io's stflg as a registered Gray code. clk_100m side: the Gray code synchronised,
// a capture of status_req on every change of it (SEQ counts captures), the CAUSE word and its line.
// Bridge side has no reset and answers every address in fixed cycles (no back-pressure).
// Clock-ratio bound of the crossing: docs/LW_BRIDGE.md.

module lw_sys_status
(
	input wire         clk,          // clk_100m

	input wire   [9:0] raddr,        // block-relative, bytes
	input wire         rsel,
	input wire         ren,
	input wire         rd_ack,
	input wire         rd_word,
	output reg  [31:0] rdata,

	output wire        irq,          // level, high while CAUSE != 0

	input wire         clk_sys,
	input wire   [3:0] stflg,        // hps_io's status flag, clk_sys
	input wire [127:0] status_req    // hps_io's snapshot, clk_sys; changes only with stflg
);

localparam [31:0] TAG_SYS   = 32'h0010_0000;   // cause word [23:16] one-hot: bit 20 = SYS
localparam [15:0] C_STATUS  = 16'h0001;

`define LW_SYNC (* altera_attribute = {"-name SYNCHRONIZER_IDENTIFICATION FORCED_IF_ASYNCHRONOUS"} *)

/////////////////////////////// clk_sys ///////////////////////////////////

reg  [3:0] st_g = 0;
always @(posedge clk_sys) st_g <= stflg ^ (stflg >> 1);

/////////////////////////////// clk_100m //////////////////////////////////

`LW_SYNC reg [3:0] g_s0 = 0;
`LW_SYNC reg [3:0] g_s1 = 0;

function automatic [3:0] g2b(input [3:0] g);
	g2b = {g[3], ^g[3:2], ^g[3:1], ^g[3:0]};
endfunction

wire  [3:0] flg_s = g2b(g_s1);

reg   [3:0] flg  = 0;          // the snapshot's flag
reg  [15:0] seq  = 0;          // captures since configuration
reg [127:0] snap = 0;
reg  [15:0] cause = 0;
reg   [3:0] gen  = 0, cgen = 0;

wire        capture  = flg_s != flg;
wire        rd_cause = rsel && raddr[9:2] == 8'h00 && rd_word;
wire        clr      = rd_cause & rd_ack & (gen == cgen);

always @(posedge clk) begin
	{g_s1, g_s0} <= {g_s0, st_g};
	if (capture) begin
		flg  <= flg_s;
		seq  <= seq + 16'd1;
		snap <= status_req;     // quasi-static: stable since the flag's Gray code moved
	end
	// a raise beats a concurrent clear; a raise between fetch and ack keeps the bit
	gen   <= gen + capture;
	if (rd_cause & ren) cgen <= gen;
	cause <= (clr ? 16'd0 : cause) | (capture ? C_STATUS : 16'd0);
end

assign irq = |cause;

always @(*) begin
	rdata = 32'd0;
	if (rsel) case (raddr[9:2])
		8'h00: rdata = |cause ? (TAG_SYS | {16'd0, cause}) : 32'd0;
		8'h01: rdata = {seq, 8'd0, 4'hA, flg};          // STFLG: the 0x29 head byte, plus SEQ
		8'h02: rdata = snap[ 31:  0];
		8'h03: rdata = snap[ 63: 32];
		8'h04: rdata = snap[ 95: 64];
		8'h05: rdata = snap[127: 96];
		8'h07: rdata = 32'h31535453;                    // "STS1"
		default: rdata = 32'd0;
	endcase
end

`undef LW_SYNC

endmodule

`default_nettype wire
`endif
