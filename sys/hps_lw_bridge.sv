//============================================================================
//  hps_lw_bridge.sv -- part of the LW bridge framework (docs/LW_BRIDGE.md)
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

//============================================================================
//  hps_lw_bridge.sv  --  lwhps2fpga bridge instantiation + AXI3 firewall
//
//  Silicon-proven on the DE10-Nano.  Answers every address of the 2 MB
//  window, every burst, both channels, unconditionally: an unanswered LW
//  access hangs the SoC.
//   * NO reset input: a read landing in a reset window must still answer.
//   * Clock is clk_100m (h2f_user0_clk), never a reconfigurable PLL.
//   * AXI3 LEN/SIZE/BURST handled; IDs echoed; BRESP/RRESP always OKAY.
//  Atom port list (names, widths, directions) checked against Quartus 17.0.2
//  libraries/megafunctions/xml_info/cyclonev_hps_interface_hps2fpga_light_weight_info.xml.
//============================================================================

module hps_lw_bridge
(
	input             clk,          // clk_100m == h2f_user0_clk

	// simple synchronous register bus, read side
	output     [20:0] lw_raddr,     // byte address inside the 2MB window
	output            lw_ren,       // fetch pulse (NO side effects allowed)
	output            lw_rd_ack,    // accepted-R-beat pulse (side effects fire here)
	output            lw_rd_word,   // this beat is a word-exact single access
	input      [31:0] lw_rdata,     // combinational, must answer every lw_raddr

	// simple synchronous register bus, write side
	output     [20:0] lw_waddr,
	output            lw_write,     // accepted-W-beat pulse
	output     [31:0] lw_wdata,
	output      [3:0] lw_be
);

// fabric side of the lwhps2fpga bridge (HPS is the AXI3 master)
wire [11:0] awid;
wire [20:0] awaddr;
wire  [3:0] awlen;
wire  [2:0] awsize;
wire  [1:0] awburst;
wire  [1:0] awlock;
wire  [3:0] awcache;
wire  [2:0] awprot;
wire        awvalid;
wire        awready;
wire [11:0] wid;
wire [31:0] wdata;
wire  [3:0] wstrb;
wire        wlast;
wire        wvalid;
wire        wready;
wire [11:0] bid;
wire  [1:0] bresp;
wire        bvalid;
wire        bready;
wire [11:0] arid;
wire [20:0] araddr;
wire  [3:0] arlen;
wire  [2:0] arsize;
wire  [1:0] arburst;
wire  [1:0] arlock;
wire  [3:0] arcache;
wire  [2:0] arprot;
wire        arvalid;
wire        arready;
wire [11:0] rid;
wire [31:0] rdata;
wire  [1:0] rresp;
wire        rlast;
wire        rvalid;
wire        rready;

cyclonev_hps_interface_hps2fpga_light_weight hps2fpga_light_weight
(
	.clk(clk),

	.awid(awid),
	.awaddr(awaddr),
	.awlen(awlen),
	.awsize(awsize),
	.awburst(awburst),
	.awlock(awlock),
	.awcache(awcache),
	.awprot(awprot),
	.awvalid(awvalid),
	.awready(awready),

	.wid(wid),
	.wdata(wdata),
	.wstrb(wstrb),
	.wlast(wlast),
	.wvalid(wvalid),
	.wready(wready),

	.bid(bid),
	.bresp(bresp),
	.bvalid(bvalid),
	.bready(bready),

	.arid(arid),
	.araddr(araddr),
	.arlen(arlen),
	.arsize(arsize),
	.arburst(arburst),
	.arlock(arlock),
	.arcache(arcache),
	.arprot(arprot),
	.arvalid(arvalid),
	.arready(arready),

	.rid(rid),
	.rdata(rdata),
	.rresp(rresp),
	.rlast(rlast),
	.rvalid(rvalid),
	.rready(rready)
);

lw_axi_regbus adapter
(
	.clk(clk),

	.awid(awid), .awaddr(awaddr), .awlen(awlen), .awsize(awsize),
	.awburst(awburst), .awvalid(awvalid), .awready(awready),

	.wdata(wdata), .wstrb(wstrb), .wlast(wlast),
	.wvalid(wvalid), .wready(wready),

	.bid(bid), .bresp(bresp), .bvalid(bvalid), .bready(bready),

	.arid(arid), .araddr(araddr), .arlen(arlen), .arsize(arsize),
	.arburst(arburst), .arvalid(arvalid), .arready(arready),

	.rid(rid), .rdata(rdata), .rresp(rresp), .rlast(rlast),
	.rvalid(rvalid), .rready(rready),

	.lw_raddr(lw_raddr), .lw_ren(lw_ren), .lw_rd_ack(lw_rd_ack),
	.lw_rd_word(lw_rd_word), .lw_rdata(lw_rdata),
	.lw_waddr(lw_waddr), .lw_write(lw_write), .lw_wdata(lw_wdata),
	.lw_be(lw_be)
);

endmodule


//============================================================================
//  lw_axi_regbus -- AXI3 slave -> trivial synchronous register bus.
//  Pure RTL, no atom, no reset: simulable standalone (tb/lw_tb.sv drives it
//  through a stand-in atom).  One transaction in flight per channel
//  direction; read and write channels are fully independent (no shared
//  state), so neither can deadlock the other.
//
//  Read pipeline per beat: FETCH (assert lw_ren, capture combinational
//  lw_rdata) -> DATA (RVALID with registered data -- AXI requires RDATA
//  stable while RVALID & ~RREADY).  On RVALID&RREADY the lw_rd_ack pulse
//  commits downstream side effects; lw_raddr still holds the beat's
//  address at that instant and only steps afterwards.  Cost: 2 cycles
//  AR-accept -> RVALID ("registered decode, single wait state") -- count
//  this fixed overhead separately when timing an LW read from the HPS, or
//  the measured number is an artifact of this adapter.
//============================================================================

module lw_axi_regbus
(
	input             clk,

	// AXI3 slave
	input      [11:0] awid,
	input      [20:0] awaddr,
	input       [3:0] awlen,
	input       [2:0] awsize,
	input       [1:0] awburst,
	input             awvalid,
	output            awready,

	input      [31:0] wdata,
	input       [3:0] wstrb,
	input             wlast,
	input             wvalid,
	output            wready,

	output     [11:0] bid,
	output      [1:0] bresp,
	output            bvalid,
	input             bready,

	input      [11:0] arid,
	input      [20:0] araddr,
	input       [3:0] arlen,
	input       [2:0] arsize,
	input       [1:0] arburst,
	input             arvalid,
	output            arready,

	output     [11:0] rid,
	output     [31:0] rdata,
	output      [1:0] rresp,
	output            rlast,
	output            rvalid,
	input             rready,

	// register bus
	output     [20:0] lw_raddr,
	output            lw_ren,
	output            lw_rd_ack,
	output            lw_rd_word,
	input      [31:0] lw_rdata,
	output     [20:0] lw_waddr,
	output            lw_write,
	output     [31:0] lw_wdata,
	output      [3:0] lw_be
);

// burst address stepping: FIXED holds, INCR adds 1<<size, WRAP wraps inside
// the (len+1)<<size container (len+1 is a power of two for legal WRAP).
// Reserved burst 2'b11 is treated as INCR -- still answered, never stuck.
function [20:0] step_addr(input [20:0] a, input [2:0] size,
                          input [1:0] burst, input [20:0] mask);
	reg [20:0] inc;
	begin
		inc = 21'd1 << size;
		case (burst)
			2'b00:   step_addr = a;                              // FIXED
			2'b10:   step_addr = (a & ~mask) | ((a + inc) & mask); // WRAP
			default: step_addr = a + inc;                        // INCR
		endcase
	end
endfunction

////////////////////////////// read channel //////////////////////////////

localparam R_IDLE  = 2'd0;
localparam R_FETCH = 2'd1;
localparam R_DATA  = 2'd2;

reg  [1:0] rstate   = R_IDLE;
reg [11:0] rid_q    = 12'd0;
reg [20:0] raddr_q  = 21'd0;
reg  [3:0] rlen_q   = 4'd0;
reg  [3:0] rbeat    = 4'd0;
reg  [2:0] rsize_q  = 3'd0;
reg  [1:0] rburst_q = 2'd0;
reg [20:0] rmask_q  = 21'd0;
reg [31:0] rdata_q  = 32'd0;

assign arready   = (rstate == R_IDLE);
assign rvalid    = (rstate == R_DATA);
assign rdata     = rdata_q;
assign rid       = rid_q;
assign rresp     = 2'b00;                 // always OKAY: never a no-ack
assign rlast     = (rbeat == rlen_q);
assign lw_raddr  = raddr_q;
assign lw_ren    = (rstate == R_FETCH);
assign lw_rd_ack = rvalid & rready;       // side effects ONLY here
// Word-exact access qualifier: single beat, 32-bit, word-aligned.  ARSIZE and
// ARLEN are otherwise discarded on this register bus, which would make a byte
// read of 0x1003 -- or any burst whose stepped address merely passes through a
// read-to-clear word -- commit that word's clear while returning only a lane of
// it.  Destructive-read registers gate their commit on this bit; the read data
// itself is returned regardless, so answer-everything is untouched.
assign lw_rd_word = (rlen_q == 4'd0) & (rsize_q == 3'd2) & (raddr_q[1:0] == 2'd0);

always @(posedge clk) begin
	case (rstate)
	R_IDLE: if (arvalid) begin
		rid_q    <= arid;
		raddr_q  <= araddr;
		rlen_q   <= arlen;
		rsize_q  <= arsize;
		rburst_q <= arburst;
		rmask_q  <= (({17'd0, arlen} + 21'd1) << arsize) - 21'd1;
		rbeat    <= 4'd0;
		rstate   <= R_FETCH;
	end

	R_FETCH: begin
		rdata_q <= lw_rdata;
		rstate  <= R_DATA;
	end

	R_DATA: if (rready) begin
		if (rbeat == rlen_q) rstate <= R_IDLE;
		else begin
			rbeat   <= rbeat + 4'd1;
			raddr_q <= step_addr(raddr_q, rsize_q, rburst_q, rmask_q);
			rstate  <= R_FETCH;
		end
	end

	default: rstate <= R_IDLE;
	endcase
end

////////////////////////////// write channel /////////////////////////////

localparam W_IDLE = 2'd0;
localparam W_DATA = 2'd1;
localparam W_RESP = 2'd2;

reg  [1:0] wstate   = W_IDLE;
reg [11:0] bid_q    = 12'd0;
reg [20:0] waddr_q  = 21'd0;
reg  [2:0] wsize_q  = 3'd0;
reg  [1:0] wburst_q = 2'd0;
reg [20:0] wmask_q  = 21'd0;

assign awready  = (wstate == W_IDLE);
assign wready   = (wstate == W_DATA);
assign bvalid   = (wstate == W_RESP);     // BRESP returned unconditionally
assign bid      = bid_q;
assign bresp    = 2'b00;                  // always OKAY
assign lw_waddr = waddr_q;
assign lw_write = wvalid & wready;        // accepted W beat only
assign lw_wdata = wdata;
assign lw_be    = wstrb;

always @(posedge clk) begin
	case (wstate)
	W_IDLE: if (awvalid) begin
		bid_q    <= awid;
		waddr_q  <= awaddr;
		wsize_q  <= awsize;
		wburst_q <= awburst;
		wmask_q  <= (({17'd0, awlen} + 21'd1) << awsize) - 21'd1;
		wstate   <= W_DATA;
	end

	W_DATA: if (wvalid) begin
		waddr_q <= step_addr(waddr_q, wsize_q, wburst_q, wmask_q);
		if (wlast) wstate <= W_RESP;
	end

	W_RESP: if (bready) wstate <= W_IDLE;

	default: wstate <= W_IDLE;
	endcase
end

endmodule
