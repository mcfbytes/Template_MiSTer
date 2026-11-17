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
// CAUSE [15:0] are event bits, STATUS (bit 0) raised by every capture; [31:16] read 0. A full-word store clears
// each bit written as 1, and a raise in the store's cycle wins. Reads change nothing.
// clk_sys side: hps_io's stflg as a registered Gray code. clk_100m side: the Gray code through lw_sync
// (lw_cdc.sv), a capture of status_req on every change of it (SEQ counts captures), the CAUSE word and its line.
// The capture reads status_req across the clocks with no cell: it relies on the words being stable from before
// the flag moves until after the flag is seen, and is the one such read tb/static.sh lists.
// Bridge side has no reset and answers every address in fixed cycles (no back-pressure).
// Clock-ratio bound of the crossing: docs/LW_BRIDGE.md.

module lw_sys_status
(
	input wire         clk,          // clk_100m

	input wire   [9:0] raddr,        // block-relative, bytes
	input wire         rsel,
	output reg  [31:0] rdata,
	input wire   [9:0] waddr,
	input wire         wsel,
	input wire         write,
	input wire  [31:0] wdata,
	input wire   [3:0] be,

	output wire        irq,          // level, high while CAUSE != 0

	input wire         clk_sys,
	input wire   [3:0] stflg,        // hps_io's status flag, clk_sys
	input wire [127:0] status_req    // hps_io's snapshot, clk_sys; changes only with stflg
);

localparam [15:0] C_STATUS  = 16'h0001;

/////////////////////////////// clk_sys ///////////////////////////////////

(* preserve *) reg [3:0] stflg_lwgray = 0;
always @(posedge clk_sys) stflg_lwgray <= stflg ^ (stflg >> 1);

/////////////////////////////// clk_100m //////////////////////////////////

wire [3:0] g_s;
lw_sync #(.W(4), .GRAY(1)) flg_sync(.clk(clk), .d(stflg_lwgray), .q(g_s));

function automatic [3:0] g2b(input [3:0] g);
	g2b = {g[3], ^g[3:2], ^g[3:1], ^g[3:0]};
endfunction

wire  [3:0] flg_s = g2b(g_s);

reg   [3:0] flg  = 0;          // the snapshot's flag
reg  [15:0] seq  = 0;          // captures since configuration
reg [127:0] snap = 0;
reg  [15:0] cause = 0;

wire        capture = flg_s != flg;
wire        w_cause = wsel && write && &be && waddr[9:2] == 8'h00;
wire [15:0] clr     = w_cause ? wdata[15:0] : 16'd0;

always @(posedge clk) begin
	if (capture) begin
		flg  <= flg_s;
		seq  <= seq + 16'd1;
		snap <= status_req;     // quasi-static: stable since the flag's Gray code moved
	end
	cause <= (cause & ~clr) | (capture ? C_STATUS : 16'd0);   // a raise beats a clear in the same cycle
end

assign irq = |cause;

always @(*) begin
	rdata = 32'd0;
	if (rsel) case (raddr[9:2])
		8'h00: rdata = {16'd0, cause};
		8'h01: rdata = {seq, 8'd0, 4'hA, flg};          // STFLG: the 0x29 head byte, plus SEQ
		8'h02: rdata = snap[ 31:  0];
		8'h03: rdata = snap[ 63: 32];
		8'h04: rdata = snap[ 95: 64];
		8'h05: rdata = snap[127: 96];
		8'h07: rdata = 32'h31535453;                    // "STS1"
		default: rdata = 32'd0;
	endcase
end

endmodule

`default_nettype wire
`endif
