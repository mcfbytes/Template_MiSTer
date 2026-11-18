//============================================================================
//  lw_evt.sv -- part of the LW bridge framework (docs/LW_BRIDGE.md)
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
`ifdef MISTER_LW_EVT
`default_nettype none

// EVT block (LW directory kind 4, version 2.0, "EVT2"; the L2 tier, MISTER_LW_EVT): hps_io's core->host flags as
// causes on one line, so a host need not poll 0x16, 0x29, 0x36 or 0x3C. The window lists the block only while
// hps_io's event exports show their presence bit on HPS_BUS.
// CAUSE [15:0] are event bits: STATUS (bit 0) per status transfer landed; BLK (1) per rise of a slot's block
// request and per fall of a slot's SPI ack that leaves its request raised; INFO (2) per info_req rise; UPLOAD (3)
// per ioctl_upload_req rise; [31:16] read 0. A full-word store clears each bit written as 1, and a raise in the
// store's cycle wins. Reads change nothing. A taken ENABLE ORs the standing levels in, {PEND[3:1], STATUS}, so a
// new host reads the status once and is woken for whatever stands, and keeps what the word had.
// clk_sys side: a dirty bit set in every cycle hps_io's stflg differs from its value a cycle before; while it is
// set and the handshake is idle, {status_req, stflg} is launched whole through an lw_xfer (a change in the launch
// cycle stays dirty). hps_io's three event pulses are ORed into a pending vector and launched through a second
// lw_xfer; pulses that come while one is in flight join the next, and a pulse in the launch cycle stays pending.
// The three levels are registered once and cross through an lw_sync, as independent samples.
// clk_100m side: STFLG's flag and the four STATUS words read the transfer's capture register, so a capture is never
// torn; SEQ and STATUS follow one cycle later. The event vector is ORed into CAUSE from its capture register, in
// the cycle after it lands.
// Bridge side has no reset and answers every address in fixed cycles (no back-pressure).

module lw_evt
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
	input wire         take,         // an ENABLE store is taken in this cycle

	output wire        irq,          // level, high while CAUSE != 0

	input wire         clk_sys,
	input wire [131:0] st,           // hps_io's {status_req[127:0], stflg[3:0]}, clk_sys registers
	input wire   [2:0] ev,           // hps_io's one-cycle pulses {upload, info, blk}, clk_sys
	input wire   [2:0] lvl           // hps_io's levels {upload pending, info pending, a block request stands}
);

localparam [15:0] C_STATUS  = 16'h0001;

/////////////////////////////// clk_sys ///////////////////////////////////

reg  [3:0] flg_d  = 0;           // stflg a cycle before
reg        sdirty = 0;           // the status changed since its last launch
reg  [2:0] epend  = 0;           // event pulses not yet launched
(* preserve *) reg [2:0] lvl_lwx = 0;
wire       s_busy, e_busy;
wire       e_go = |epend;

always @(posedge clk_sys) begin
	flg_d   <= st[3:0];
	sdirty  <= (st[3:0] != flg_d) | (sdirty & s_busy);      // set wins over the launch's clear
	epend   <= ev | (epend & {3{e_busy}});
	lvl_lwx <= lvl;
end

/////////////////////////////// crossings /////////////////////////////////

wire [131:0] s_q;
wire   [2:0] e_q, lvl_s;
wire         s_v, e_v;

lw_xfer #(.W(132)) st_x(.s_clk(clk_sys), .s_go(sdirty), .s_d(st), .s_busy(s_busy), .d_clk(clk), .d_q(s_q), .d_v(s_v));
lw_xfer #(.W(3)) ev_x(.s_clk(clk_sys), .s_go(e_go), .s_d(epend), .s_busy(e_busy), .d_clk(clk), .d_q(e_q), .d_v(e_v));
lw_sync #(.W(3)) lvl_sync(.clk(clk), .d(lvl_lwx), .q(lvl_s));

/////////////////////////////// clk_100m //////////////////////////////////

reg  [15:0] seq   = 0;           // status transfers landed since configuration
reg  [15:0] cause = 0;

wire        w_cause = wsel && write && &be && waddr[9:2] == 8'h00;
wire [15:0] clr     = w_cause ? wdata[15:0] : 16'd0;
wire [15:0] raise   = (s_v ? C_STATUS : 16'd0) | (e_v ? {12'd0, e_q, 1'b0} : 16'd0) |
                      (take ? {12'd0, lvl_s, 1'b1} : 16'd0);

always @(posedge clk) begin
	if (s_v) seq <= seq + 16'd1;
	cause <= (cause & ~clr) | raise;                         // a raise beats a clear in the same cycle
end

assign irq = |cause;

always @(*) begin
	rdata = 32'd0;
	if (rsel) case (raddr[9:2])
		8'h00: rdata = {16'd0, cause};
		8'h01: rdata = {seq, 8'd0, 4'hA, s_q[3:0]};      // STFLG: the 0x29 head byte, plus SEQ
		8'h02: rdata = s_q[ 35:  4];
		8'h03: rdata = s_q[ 67: 36];
		8'h04: rdata = s_q[ 99: 68];
		8'h05: rdata = s_q[131:100];
		8'h06: rdata = {28'd0, lvl_s, 1'b0};             // PEND
		8'h07: rdata = 32'h32545645;                    // "EVT2"
		default: rdata = 32'd0;
	endcase
end

endmodule

`default_nettype wire
`endif
`endif
