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

// PAD block (LW directory kind 1, version 1.1, ID "PAD1"; the L1 tier, MISTER_LW_PAD): one full-word store at
// PAD + (opcode << 10) + (index << 2) is one link set-command for hps_io's LW write arm. The window gates every
// store (the host enable) and lists the block only while hps_io's LW write arm shows its presence bit on HPS_BUS.
// Every level is a keyed latest-wins slot with a dirty bit, so nothing absolute is ever dropped: the six digital
// pads (ops 0x02, 0x03, 0x10-0x13, index 0), the left and right sticks (0x1A and 0x3D, index 0-5) and the
// paddles (0x1A, index {p, 4'hF}, p = 0-5). Spinners (0x1A, index {p, 4'hF}, p = 8-13) are events (hps_io toggles
// bit 8 per store), so they queue in a 16-deep FIFO; a store to a full FIFO is dropped and counted, in every
// build. Any other store is ignored. Slots and FIFO share one M10K, and a held-data handshake (lw_xfer,
// lw_cdc.sv) drains them to clk_sys: a scan pointer visits one source per cycle while the handshake is idle.
// Reads: PAD + 0x0 STAT {16'd0, spinner drops}, PAD + 0x4 ID; every other word reads 0.
// MISTER_LW_KBM (PROVISIONAL) adds the keys, the mouse and the analog triggers: a key frame (0x05, index 0) and a
// mouse store (0x04, index 1 the extension, index 0 the packet) are events in the same FIFO, its entry
// {opcode, 32 data bits}; the six trigger pairs (0x46, index 0-5) are latest-wins slots at the free source codes
// {p[2:1], 2'b11, p[0]}. STAT is then {11'd0, FIFO level[20:16], drops of every event store[15:0]}.
// No enable link: everything here is clk_100m state, a store taken under the enable is delivered whenever clk_sys
// runs, and the levels stay as stored when the enable falls (hps_io keeps the last word whichever path wrote it).
// Bridge side runs on clk_100m with no reset and answers every address in fixed cycles.

module lw_pad
(
	input wire         clk,          // clk_100m

/* verilator lint_off UNUSEDSIGNAL */
	input wire  [17:0] raddr,        // block-relative, bytes; word reads, so [1:0] is unused
/* verilator lint_on UNUSEDSIGNAL */
	output wire [31:0] rdata,
	input wire  [17:2] waddr,
	input wire         wsel,
	input wire         write,        // the window's gated store
	input wire  [31:0] wdata,
	input wire   [3:0] be,

	input wire         clk_sys,
	output wire [48:0] q             // {we, op[7:0], idx[7:0], data[31:0]} on clk_sys: HPS_BUS[94:46]
);

// A source is {group, player}: group 0 digital, 1 left stick, 2 right stick, 3 paddle; a level's RAM word is its
// source. FIFO is the event FIFO's source; the codes with player 6 or 7 are no source (with MISTER_LW_KBM, codes
// 6, 7, 14, 15, 22 and 23 are the trigger slots).
localparam [4:0] FIFO = 5'd30;

reg [15:0] sdrop = 0;               // event stores dropped on a full FIFO, free-running

reg  [4:0] awp = 0, arp = 0;
`ifdef MISTER_LW_KBM
wire [4:0] alevel = awp - arp;      // FIFO entries waiting, 0-16
assign rdata = (raddr[17:2] == 16'd0) ? {11'd0, alevel, sdrop} :
`else
assign rdata = (raddr[17:2] == 16'd0) ? {16'd0, sdrop} :
`endif
               (raddr[17:2] == 16'd1) ? 32'h31444150 :   // "PAD1"
               32'd0;

function automatic [7:0] op_of(input [2:0] p);
	case (p)
		3'd0: op_of = 8'h02;
		3'd1: op_of = 8'h03;
		3'd2: op_of = 8'h10;
		3'd3: op_of = 8'h11;
		3'd4: op_of = 8'h12;
		default: op_of = 8'h13;
	endcase
endfunction

// the store's key: a level's source, or a spinner event
wire       st   = write && wsel && &be;
wire [7:0] wop  = waddr[17:10];
wire [7:0] widx = waddr[9:2];
wire       ixp  = widx[7:3] == 5'd0 && widx[2:0] < 3'd6;   // index 0-5
wire       ixd  = widx[3:0] == 4'hF && !widx[7] && widx[6:4] < 3'd6;   // {p, 4'hF}, p = 0-5
wire       ixs  = widx[3:0] == 4'hF &&  widx[7] && widx[6:4] < 3'd6;   // {p, 4'hF}, p = 8-13
reg  [4:0] wkey;
reg        wlvl, wspn;
always @(*) begin
	wkey = 5'd0; wlvl = 0; wspn = 0;
	case (wop)
		8'h02: begin wkey = 5'd0; wlvl = widx == 8'd0; end
		8'h03: begin wkey = 5'd1; wlvl = widx == 8'd0; end
		8'h10: begin wkey = 5'd2; wlvl = widx == 8'd0; end
		8'h11: begin wkey = 5'd3; wlvl = widx == 8'd0; end
		8'h12: begin wkey = 5'd4; wlvl = widx == 8'd0; end
		8'h13: begin wkey = 5'd5; wlvl = widx == 8'd0; end
		8'h1A: begin
			wkey = ixd ? {2'd3, widx[6:4]} : {2'd1, widx[2:0]};
			wlvl = ixp || ixd;
			wspn = ixs;
		end
		8'h3D: begin wkey = {2'd2, widx[2:0]}; wlvl = ixp; end
`ifdef MISTER_LW_KBM
		8'h04: wspn = widx[7:1] == 7'd0;                                  // index 1 the extension, 0 the packet
		8'h05: wspn = widx == 8'd0;                                       // a key frame
		8'h46: begin wkey = {widx[2:1], 2'b11, widx[0]}; wlvl = ixp; end   // a trigger pair, player 0-5
`endif
		default: ;
	endcase
end
wire       st_l = st && wlvl;
wire       st_s = st && wspn;

`ifdef MISTER_LW_KBM
// one M10K: the level slots at their source (0-29), the event FIFO {opcode, word} at 32-47: a spinner's word
// {16'd0, index, data[7:0]}, a key frame's its data, a mouse store's {7'd0, index[0], data[23:0]}
(* ramstyle = "M10K" *) reg [39:0] pram [0:63];
reg [39:0] pq;
reg [31:0] dirty = 0;               // by source; only the 30 level sources are ever set
wire [31:0] ewd = (wop == 8'h1A) ? {16'd0, widx, wdata[7:0]} : (wop == 8'h04) ? {7'd0, widx[0], wdata[23:0]} : wdata;
`else
// one M10K: the level slots at their source (0-29), the spinner FIFO {index, data[7:0]} at 32-47
(* ramstyle = "M10K" *) reg [31:0] pram [0:63];
reg [31:0] pq;
reg [31:0] dirty = 0;               // by source; only the 24 level sources are ever set
`endif

wire       aempty = (awp == arp);
wire       afull  = (awp - arp) == 5'd16;

wire        busy;
wire [31:0] ready = dirty | ({31'd0, ~aempty} << FIFO);
reg   [4:0] sp = 0;                 // the scan pointer
reg   [1:0] ds = 0;                 // drain: 0 scan, 1 read, 2 launch
reg   [4:0] psel = 0;

`ifdef MISTER_LW_KBM
// the next source: every code from 0 to FIFO (the trigger slots sit at players 6 and 7), then the first again
wire  [4:0] sp_n = (sp == FIFO) ? 5'd0 : sp + 5'd1;
`else
// the next source: player 0-5 of each group, then FIFO, then the first again
wire  [4:0] sp_n = (sp == FIFO) ? 5'd0 : (sp == 5'd29) ? FIFO : (sp[2:0] == 3'd5) ? {sp[4:3] + 2'd1, 3'd0} :
                   sp + 5'd1;
`endif

wire  [5:0] pra = (psel == FIFO) ? {2'b10, arp[3:0]} : {1'b0, psel};

always @(posedge clk) begin
`ifdef MISTER_LW_KBM
	if (st_l) pram[{1'b0, wkey}] <= {8'd0, wdata};
	else if (st_s && !afull) pram[{2'b10, awp[3:0]}] <= {wop, ewd};
`else
	if (st_l) pram[{1'b0, wkey}] <= wdata;
	else if (st_s && !afull) pram[{2'b10, awp[3:0]}] <= {16'd0, widx, wdata[7:0]};
`endif
	pq <= pram[pra];
end

always @(posedge clk) begin
	case (ds)
		2'd0: if (!busy) begin
			sp <= sp_n;
			if (ready[sp]) begin
				psel <= sp;
				ds   <= 2'd1;
			end
		end
		2'd1: begin
			if (psel == FIFO) arp <= arp + 5'd1;
			else dirty[psel] <= 0;
			ds <= 2'd2;
		end
		default: ds <= 2'd0;   // the handshake takes the word in this cycle
	endcase

	// after the drain, so a store landing in the read cycle stays dirty (its RAM read returned the old word)
	if (st_l) dirty[wkey] <= 1;
	if (st_s) begin
		if (afull) sdrop <= sdrop + 16'd1;
		else awp <= awp + 5'd1;
	end
end

// to clk_sys: the store's word, and its strobe the cycle after the capture
reg [47:0] pad_w;
`ifdef MISTER_LW_KBM
always @(*) begin
	if (psel == FIFO)
		case (pq[39:32])
			8'h04:   pad_w = {8'h04, 7'd0, pq[24], 8'd0, pq[23:0]};
			8'h05:   pad_w = {8'h05, 8'd0, pq[31:0]};
			default: pad_w = {8'h1A, pq[15:8], 24'd0, pq[7:0]};
		endcase
	else if (psel[2:1] == 2'b11)
		pad_w = {8'h46, 5'd0, psel[4:3], psel[0], pq[31:0]};
	else
		case (psel[4:3])
			2'd0:    pad_w = {op_of(psel[2:0]), 8'd0, pq[31:0]};
			2'd1:    pad_w = {8'h1A, 5'd0, psel[2:0], pq[31:0]};
			2'd2:    pad_w = {8'h3D, 5'd0, psel[2:0], pq[31:0]};
			default: pad_w = {8'h1A, 1'b0, psel[2:0], 4'hF, pq[31:0]};
		endcase
end
`else
always @(*) begin
	case (psel[4:3])
		2'd0:    pad_w = {op_of(psel[2:0]), 8'd0, pq};
		2'd1:    pad_w = {8'h1A, 5'd0, psel[2:0], pq};
		2'd2:    pad_w = {8'h3D, 5'd0, psel[2:0], pq};
		default: pad_w = (psel == FIFO) ? {8'h1A, pq[15:8], 24'd0, pq[7:0]} : {8'h1A, 1'b0, psel[2:0], 4'hF, pq};
	endcase
end
`endif

lw_xfer #(.W(48)) pad_x
(
	.s_clk(clk), .s_go(ds == 2'd2), .s_d(pad_w), .s_busy(busy),
	.d_clk(clk_sys), .d_q(q[47:0]), .d_v(q[48])
);

endmodule

`default_nettype wire
`endif
`endif
