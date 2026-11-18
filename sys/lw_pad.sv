//============================================================================
//  lw_pad.sv -- part of the LW bridge framework (docs/LW_BRIDGE.md)
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
`ifdef MISTER_LW_PAD
`default_nettype none

// PAD block (LW directory kind 1, the L1 tier, MISTER_LW_PAD): one full-word store at PAD + (opcode << 10) +
// (index << 2) is one link set-command for hps_io's LW write arm. The window gates every store (the host enable)
// and lists the block only while hps_io's LW write arm shows its presence bit on HPS_BUS.
// Digital stores (ops 0x02, 0x03, 0x10-0x13, index 0) park in per-op latest-wins slots; both analog ops (0x1A,
// 0x3D) go to a 16-deep FIFO, dropped and counted when full. Both live in one M10K, and a held-data handshake
// (lw_xfer, lw_cdc.sv) drains them round-robin to clk_sys.
// Reads: the analog-drop count at the 0x1A page's first word with DIAG=1; every other word reads 0.
// Bridge side runs on clk_100m with no reset and answers every address in fixed cycles.

module lw_pad #(parameter DIAG = 0)
(
	input wire         clk,          // clk_100m

	input wire  [17:0] raddr,        // block-relative, bytes
	output wire [31:0] rdata,
	input wire  [17:2] waddr,
	input wire         wsel,
	input wire         write,        // the window's gated store
	input wire  [31:0] wdata,
	input wire   [3:0] be,

	input wire         clk_sys,
	output wire [48:0] q             // {we, op[7:0], idx[7:0], data[31:0]} on clk_sys: HPS_BUS[94:46]
);

reg [31:0] adrop = 0;               // analog stores dropped on a full FIFO

assign rdata = (DIAG && raddr == {8'h1A, 10'd0}) ? adrop : 32'd0;

function automatic [2:0] slot_of(input [7:0] op);
	case (op)
		8'h02: slot_of = 3'd0;
		8'h03: slot_of = 3'd1;
		8'h10: slot_of = 3'd2;
		8'h11: slot_of = 3'd3;
		8'h12: slot_of = 3'd4;
		8'h13: slot_of = 3'd5;
		default: slot_of = 3'd7;
	endcase
endfunction

function automatic [7:0] op_of(input [2:0] slot);
	case (slot)
		3'd0: op_of = 8'h02;
		3'd1: op_of = 8'h03;
		3'd2: op_of = 8'h10;
		3'd3: op_of = 8'h11;
		3'd4: op_of = 8'h12;
		default: op_of = 8'h13;
	endcase
endfunction

// one M10K: digital masks at 0..5, the analog FIFO {right, idx, data[15:0]} at 16..31
(* ramstyle = "M10K" *) reg [31:0] pram [0:31];
reg [31:0] pq;
reg  [5:0] dirty = 0;

reg  [4:0] awp = 0, arp = 0;
wire       aempty = (awp == arp);
wire       afull  = (awp - arp) == 5'd16;

wire       st   = write && wsel && &be;
wire [7:0] wop  = waddr[17:10];
wire [7:0] widx = waddr[9:2];
wire [2:0] wsl  = slot_of(wop);
wire       st_d = st && wsl != 3'd7 && widx == 8'd0;
wire       st_a = st && (wop == 8'h1A || wop == 8'h3D) && !st_d;

wire       busy;

wire [6:0] ready = {~aempty, dirty};
reg  [2:0] rr = 0;
reg  [2:0] pick;
reg        pick_v;
reg  [1:0] ds = 0;                     // drain: 0 pick, 1 read, 2 launch
reg  [2:0] psel = 0;
integer    i, k;

always @(*) begin
	pick_v = 0;
	pick   = 0;
	for (i = 0; i < 7; i = i + 1) begin
		k = rr + i;
		if (k >= 7) k = k - 7;
		if (!pick_v && ready[k]) begin
			pick_v = 1;
			pick   = k[2:0];
		end
	end
end

wire [4:0] pra = (psel == 3'd6) ? {1'b1, arp[3:0]} : {2'b00, psel};

always @(posedge clk) begin
	if (st_d) pram[{2'b00, wsl}] <= wdata;
	else if (st_a && !afull) pram[{1'b1, awp[3:0]}] <= {7'd0, wop == 8'h3D, widx, wdata[15:0]};
	pq <= pram[pra];
end

always @(posedge clk) begin
	case (ds)
		2'd0: if (!busy && pick_v) begin
			psel <= pick;
			rr   <= (pick == 3'd6) ? 3'd0 : pick + 3'd1;
			ds   <= 2'd1;
		end
		2'd1: begin
			if (psel == 3'd6) arp <= arp + 5'd1;
			else dirty[psel] <= 0;
			ds <= 2'd2;
		end
		default: ds <= 2'd0;   // the handshake takes the word in this cycle
	endcase

	// after the drain, so a store landing in the read cycle stays dirty (its RAM read returned the old word)
	if (st_d) dirty[wsl] <= 1;
	else if (st_a) begin
		if (afull) begin
			if (DIAG) adrop <= adrop + 32'd1;
		end
		else awp <= awp + 5'd1;
	end
end

// to clk_sys: the store's word, and its strobe the cycle after the capture
wire [47:0] pad_w = (psel == 3'd6) ? {pq[24] ? 8'h3D : 8'h1A, pq[23:16], 16'd0, pq[15:0]} : {op_of(psel), 8'd0, pq};

lw_xfer #(.W(48)) pad_x
(
	.s_clk(clk), .s_go(ds == 2'd2), .s_d(pad_w), .s_busy(busy),
	.d_clk(clk_sys), .d_q(q[47:0]), .d_v(q[48])
);

endmodule

`default_nettype wire
`endif
`endif
