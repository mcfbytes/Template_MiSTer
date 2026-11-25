//============================================================================
//  ocram_axi.sv -- part of the OCRAM mailbox (docs/OCRAM_MAILBOX.md)
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

// The mailbox's bus layer: the fpga2hps AXI3 engine, one single-beat transaction at a time (hps_ocram_master.sv
// owns it; ocram_arb.sv picks its commands).
//
//  * The valids are combinational from the state register and hold their payloads (off_q, wdata_q) until each
//    ready; the response readies are high in the response states. Write: AWVALID and WVALID together, then BREADY.
//    Read: ARVALID until ARREADY, then RREADY.
//  * Every address is {16'hFFFF, off_q}: it cannot address anything but the mailbox. tb/static.sh holds it.
//  * hrst_q gates starting only: a transaction in flight is never abandoned at the reset's rise, and one still
//    unanswered in the cycle the level falls (hrst_fall) is dropped.

module ocram_axi
(
	input  wire        clk,          // clk_100m
	input  wire        hrst_q,       // the HPS reset is seen: start nothing
	input  wire        hrst_fall,    // one cycle: its level falls; an unanswered transaction is dropped
	input  wire        ns,           // AxPROT's non-secure bit

	// the command, taken in an idle cycle
	input  wire        t_go,
	input  wire        t_we,
	input  wire [15:0] t_off,
	input  wire [31:0] t_wdata,
	input  wire        t_counted,    // the DIAG counters time it
	output wire        engine_idle,
	output wire        t_done,       // the R or B handshake's cycle
	output wire        t_ok,         // its response is OKAY
	output wire        t_counted_q,  // the transaction in flight is timed
	output wire        t_drop,       // one cycle: the reset dropped an unanswered transaction

	output wire  [7:0] awid,
	output wire [31:0] awaddr,
	output wire  [3:0] awlen,
	output wire  [2:0] awsize,
	output wire  [1:0] awburst,
	output wire  [1:0] awlock,
	output wire  [3:0] awcache,
	output wire  [2:0] awprot,
	output wire  [4:0] awuser,
	output wire        awvalid,
	input  wire        awready,
	output wire  [7:0] wid,
	output wire [31:0] wdata,
	output wire  [3:0] wstrb,
	output wire        wlast,
	output wire        wvalid,
	input  wire        wready,
	input  wire  [1:0] bresp,
	input  wire        bvalid,
	output wire        bready,
	output wire  [7:0] arid,
	output wire [31:0] araddr,
	output wire  [3:0] arlen,
	output wire  [2:0] arsize,
	output wire  [1:0] arburst,
	output wire  [1:0] arlock,
	output wire  [3:0] arcache,
	output wire  [2:0] arprot,
	output wire  [4:0] aruser,
	output wire        arvalid,
	input  wire        arready,
	input  wire  [1:0] rresp,
	input  wire        rvalid,
	output wire        rready
);

localparam [2:0] E_IDLE = 3'd0, E_WR = 3'd1, E_BRSP = 3'd2, E_AR = 3'd3, E_RRSP = 3'd4;

reg  [2:0]  est = E_IDLE;
reg  [15:0] off_q = 16'd0;
reg  [31:0] wdata_q = 32'd0;
reg         aw_pend = 1'b0, w_pend = 1'b0, ar_pend = 1'b0;
reg         counted_q = 1'b0;
reg         drop_q = 1'b0;
assign t_counted_q = counted_q;
assign t_drop = drop_q;

assign engine_idle = (est == E_IDLE);
wire b_fire = bvalid & bready;
wire r_fire = rvalid & rready;
assign t_done = (est == E_BRSP) ? b_fire : (est == E_RRSP) ? r_fire : 1'b0;
assign t_ok   = (est == E_BRSP) ? (bresp == 2'b00) : (rresp == 2'b00);

always @(posedge clk) begin
	drop_q <= 1'b0;
	if (hrst_fall && !engine_idle && !t_done) begin
		est     <= E_IDLE;                    // the reset orphaned it: drop it
		aw_pend <= 1'b0; w_pend <= 1'b0; ar_pend <= 1'b0;
		drop_q  <= 1'b1;
	end
	else case (est)
	E_IDLE: if (t_go && !hrst_q) begin
		off_q     <= t_off;
		wdata_q   <= t_wdata;
		counted_q <= t_counted;
		if (t_we) begin
			est <= E_WR; aw_pend <= 1'b1; w_pend <= 1'b1;
		end
		else begin
			est <= E_AR; ar_pend <= 1'b1;
		end
	end
	E_WR: begin
		aw_pend <= aw_pend & ~awready;
		w_pend  <= w_pend  & ~wready;
		if ((!aw_pend || awready) && (!w_pend || wready)) est <= E_BRSP;
	end
	E_BRSP: if (b_fire) est <= E_IDLE;
	E_AR: if (ar_pend && arready) begin ar_pend <= 1'b0; est <= E_RRSP; end
	E_RRSP: if (r_fire) est <= E_IDLE;
	default: est <= E_IDLE;
	endcase
end

assign awid    = 8'd0;
assign wid     = 8'd0;
assign arid    = 8'd0;
assign awaddr  = {16'hFFFF, off_q};    // the constant upper half: OCRAM and nothing else
assign araddr  = {16'hFFFF, off_q};
assign awlen   = 4'd0;
assign arlen   = 4'd0;
assign awsize  = 3'b010;               // a 32-bit word
assign arsize  = 3'b010;
assign awburst = 2'b01;                // INCR
assign arburst = 2'b01;
assign awlock  = 2'b00;
assign arlock  = 2'b00;
assign awcache = 4'b0000;
assign arcache = 4'b0000;
assign awuser  = 5'd0;
assign aruser  = 5'd0;
assign awprot  = {1'b0, ns, 1'b0};     // {instruction, non-secure, privileged}: secure unless the DIAG knob
assign arprot  = {1'b0, ns, 1'b0};
assign wdata   = wdata_q;
assign wstrb   = 4'hF;
assign wlast   = 1'b1;
assign awvalid = (est == E_WR) && aw_pend;
assign wvalid  = (est == E_WR) && w_pend;
assign bready  = (est == E_BRSP);
assign arvalid = (est == E_AR) && ar_pend;
assign rready  = (est == E_RRSP);

endmodule

`default_nettype wire
`endif
