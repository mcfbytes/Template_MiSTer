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

`ifdef MISTER_LW
`default_nettype none

//============================================================================
//  hps_lw_bridge.sv  --  lwhps2fpga bridge instantiation + AXI3 firewall
//
//  Silicon-proven on the DE10-Nano.  Answers every address of the 2 MB
//  window, every burst, both channels, unconditionally: an unanswered LW
//  access hangs the SoC.
//   * No reset holds it: a read landing in a reset window must still answer.
//     After an HPS reset (hps_rst_n), it drops only a transaction the reset
//     orphaned: one whose master made no handshake on its channel between
//     the reset reaching the fabric and the release.  `idle` shows whether
//     both state machines are idle.
//   * `present` (a constant 1) and `idle` are the top level's GPI bits 25
//     and 24, so each bit is 1 only in a bitstream that holds this firewall.
//   * Clock is clk_100m (h2f_user0_clk), never a reconfigurable PLL.
//   * AXI3 LEN/SIZE/BURST handled; IDs echoed; BRESP/RRESP always OKAY.
//   * Reads step every burst type on the address's low 12 bits.  Only a
//     single-beat store reaches the register bus; the beats of a burst write
//     are accepted, answered and dropped.  A store reaches the bus in the
//     cycle its data beat is accepted and is answered in the next: it is in
//     effect once its write response has returned.
//   * Reads are pure: the register bus carries no read strobe, so no read of
//     any address, size or burst can change state.
//  Atom port list (names, widths, directions) checked against Quartus 17.0.2
//  libraries/megafunctions/xml_info/cyclonev_hps_interface_hps2fpga_light_weight_info.xml.
//============================================================================

module hps_lw_bridge
(
	input wire         clk,          // clk_100m == h2f_user0_clk
	input wire         hps_rst_n,    // the HPS's fabric reset (h2f_rst_n), asynchronous

	output wire        lw_hrst,      // hps_rst_n as a clk level: high while the reset is seen
	output wire  [1:0] lw_freed,     // pulses: [0] the read, [1] the write state machine was freed at a release
	output wire        present,      // the constant 1: this firewall is built
	output wire        idle,         // a register: both state machines are idle

	// simple synchronous register bus, read side
	output wire [20:0] lw_raddr,     // byte address inside the 2MB window
	input wire  [31:0] lw_rdata,     // combinational, must answer every lw_raddr

	// simple synchronous register bus, write side
	output wire [20:0] lw_waddr,
	output wire        lw_write,     // the accepted beat of a single-beat store
	output wire [31:0] lw_wdata,
	output wire  [3:0] lw_be,
	output wire        lw_wburst     // an accepted beat of a burst write, dropped
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

assign present = 1'b1;

lw_arst hrst_c(.clk(clk), .rst_n(hps_rst_n), .q(lw_hrst));

lw_axi_regbus adapter
(
	.clk(clk),
	.hrst(lw_hrst), .freed(lw_freed), .idle(idle),

	.awid(awid), .awaddr(awaddr), .awlen(awlen),
	.awvalid(awvalid), .awready(awready),

	.wdata(wdata), .wstrb(wstrb), .wlast(wlast),
	.wvalid(wvalid), .wready(wready),

	.bid(bid), .bresp(bresp), .bvalid(bvalid), .bready(bready),

	.arid(arid), .araddr(araddr), .arlen(arlen), .arsize(arsize),
	.arburst(arburst), .arvalid(arvalid), .arready(arready),

	.rid(rid), .rdata(rdata), .rresp(rresp), .rlast(rlast),
	.rvalid(rvalid), .rready(rready),

	.lw_raddr(lw_raddr), .lw_rdata(lw_rdata),
	.lw_waddr(lw_waddr), .lw_write(lw_write), .lw_wdata(lw_wdata),
	.lw_be(lw_be), .lw_wburst(lw_wburst)
);

endmodule


//============================================================================
//  lw_axi_regbus -- AXI3 slave -> trivial synchronous register bus.
//  Pure RTL, no atom, no reset: simulable standalone (tb/lw_tb.sv drives it
//  through a stand-in atom).  One transaction in flight per channel
//  direction; read and write channels are fully independent (no shared
//  state), so neither can deadlock the other.
//
//  HPS reset: hrst stops nothing.  In the cycle hrst rises, each state
//  machine that will hold a transaction after that cycle is marked; in any
//  other cycle a handshake on the machine's own channels (R; W or B) clears
//  the mark, as does an idle machine.  In the cycle hrst falls, a machine
//  still marked goes idle (freed pulses the cycle after), unless a handshake
//  in that cycle takes its normal step.  A marked machine has waited on its
//  master, without one handshake, since the reset was seen: its master was
//  reset with it.  A transaction accepted while hrst is high is never marked.
//
//  Read pipeline per beat: FETCH (capture combinational lw_rdata) -> DATA
//  (RVALID with registered data -- AXI requires RDATA stable while
//  RVALID & ~RREADY).  The read side is lw_raddr alone: no strobe leaves
//  it, so a slave cannot tell a read happened.  Cost: RD_LAT cycles
//  AR-accept -> RVALID ("registered decode, single wait state") -- count
//  this fixed overhead separately when timing an LW read from the HPS, or
//  the measured number is an artifact of this adapter.
//  Write side: lw_write pulses in the cycle the data beat of a single-beat
//  store is accepted, and BVALID follows in the next cycle.
//============================================================================

module lw_axi_regbus
(
	input wire         clk,
	input wire         hrst,         // the HPS reset as a clk level
	output wire  [1:0] freed,        // [0] read, [1] write: freed at the release, one cycle
	output wire        idle,         // a register: both state machines are idle

	// AXI3 slave
	input wire  [11:0] awid,
	input wire  [20:0] awaddr,
	input wire   [3:0] awlen,
	input wire         awvalid,
	output wire        awready,

	input wire  [31:0] wdata,
	input wire   [3:0] wstrb,
	input wire         wlast,
	input wire         wvalid,
	output wire        wready,

	output wire [11:0] bid,
	output wire  [1:0] bresp,
	output wire        bvalid,
	input wire         bready,

	input wire  [11:0] arid,
	input wire  [20:0] araddr,
	input wire   [3:0] arlen,
	input wire   [2:0] arsize,
	input wire   [1:0] arburst,
	input wire         arvalid,
	output wire        arready,

	output wire [11:0] rid,
	output wire [31:0] rdata,
	output wire  [1:0] rresp,
	output wire        rlast,
	output wire        rvalid,
	input wire         rready,

	// register bus
	output wire [20:0] lw_raddr,
	input wire  [31:0] lw_rdata,
	output wire [20:0] lw_waddr,
	output wire        lw_write,
	output wire [31:0] lw_wdata,
	output wire  [3:0] lw_be,
	output wire        lw_wburst
);

// Read latency: cycles from the one in which AR is accepted to the first with RVALID high; FETCH lasts
// RD_LAT - 1 cycles and samples lw_rdata in its last. A register stage on the slaves' words raises it by
// one. The testbench's fixed-latency check reads it.
localparam [2:0] RD_LAT = 3'd2;
localparam [2:0] RWAIT  = RD_LAT - 3'd2;   // FETCH cycles before the one that samples lw_rdata

// Read burst stepping on the low 12 bits only: an AXI burst never crosses 4 KB, so [20:12] holds for the
// whole transaction. FIXED holds, INCR and reserved 2'b11 add 1 << size, WRAP wraps inside the
// (len + 1) << size container (exact for the legal lengths 2, 4, 8, 16). A size above the 32-bit bus
// steps as a word.
function [11:0] step_addr(input [11:0] a, input [1:0] size, input [1:0] burst, input [11:0] mask);
	reg [11:0] inc;
	begin
		inc = 12'd1 << size;
		case (burst)
			2'b00:   step_addr = a;                                 // FIXED
			2'b10:   step_addr = (a & ~mask) | ((a + inc) & mask);  // WRAP
			default: step_addr = a + inc;                           // INCR
		endcase
	end
endfunction

//////////////////////////////// HPS reset ///////////////////////////////

reg  hrst_q = 1'b1;                       // lw_arst's q is high from configuration: no rise there
wire hrst_rise = hrst & ~hrst_q;
wire hrst_fall = ~hrst & hrst_q;

always @(posedge clk) hrst_q <= hrst;

////////////////////////////// read channel //////////////////////////////

localparam R_IDLE  = 2'd0;
localparam R_FETCH = 2'd1;
localparam R_DATA  = 2'd2;

wire [1:0] arsize_w = (arsize > 3'd2) ? 2'd2 : arsize[1:0];

reg  [1:0] rstate   = R_IDLE;
reg [11:0] rid_q    = 12'd0;
reg [20:0] raddr_q  = 21'd0;
reg  [3:0] rlen_q   = 4'd0;
reg  [3:0] rbeat    = 4'd0;
reg  [1:0] rsize_q  = 2'd0;
reg  [1:0] rburst_q = 2'd0;
reg [11:0] rmask_q  = 12'd0;
reg  [2:0] rwait    = 3'd0;
reg [31:0] rdata_q  = 32'd0;

assign arready   = (rstate == R_IDLE);
assign rvalid    = (rstate == R_DATA);
assign rdata     = rdata_q;
assign rid       = rid_q;
assign rresp     = 2'b00;                 // always OKAY: never a no-ack
assign rlast     = (rbeat == rlen_q);
assign lw_raddr  = raddr_q;

wire r_hs   = rvalid & rready;
wire r_hold = (rstate == R_IDLE && arvalid) || rstate == R_FETCH ||
              (rstate == R_DATA && !(rready && rbeat == rlen_q));   // holds a transaction after this cycle
reg  rmark   = 1'b0;
reg  rfreed  = 1'b0;
wire r_free  = hrst_fall & rmark & ~r_hs;

always @(posedge clk) begin
	if (hrst_rise) rmark <= r_hold;
	else if (hrst_fall || r_hs || rstate == R_IDLE) rmark <= 1'b0;
	rfreed <= r_free;
end

always @(posedge clk) begin
	if (r_free) rstate <= R_IDLE;
	else case (rstate)
	R_IDLE: if (arvalid) begin
		rid_q    <= arid;
		raddr_q  <= araddr;
		rlen_q   <= arlen;
		rsize_q  <= arsize_w;
		rburst_q <= arburst;
		rmask_q  <= (({8'd0, arlen} + 12'd1) << arsize_w) - 12'd1;
		rbeat    <= 4'd0;
		rwait    <= RWAIT;
		rstate   <= R_FETCH;
	end

	R_FETCH: if (rwait == 3'd0) begin
		rdata_q <= lw_rdata;
		rstate  <= R_DATA;
	end
	else rwait <= rwait - 3'd1;

	R_DATA: if (rready) begin
		if (rbeat == rlen_q) rstate <= R_IDLE;
		else begin
			rbeat   <= rbeat + 4'd1;
			raddr_q <= {raddr_q[20:12], step_addr(raddr_q[11:0], rsize_q, rburst_q, rmask_q)};
			rwait   <= RWAIT;
			rstate  <= R_FETCH;
		end
	end

	default: rstate <= R_IDLE;
	endcase
end

////////////////////////////// write channel /////////////////////////////

// Only a single-beat transaction (AWLEN 0) reaches the register bus: a store is one full-word single beat.
// Every beat of a burst is still accepted and answered, and reported on lw_wburst instead of lw_write.
// waddr_q never steps.

localparam W_IDLE = 2'd0;
localparam W_DATA = 2'd1;
localparam W_RESP = 2'd2;

reg  [1:0] wstate    = W_IDLE;
reg [11:0] bid_q     = 12'd0;
reg [20:0] waddr_q   = 21'd0;
reg        wsingle_q = 1'b0;

assign awready   = (wstate == W_IDLE);
assign wready    = (wstate == W_DATA);
assign bvalid    = (wstate == W_RESP);    // BRESP returned unconditionally
assign bid       = bid_q;
assign bresp     = 2'b00;                 // always OKAY
assign lw_waddr  = waddr_q;
assign lw_write  = wvalid & wready & wsingle_q;   // the accepted beat of a single-beat store
assign lw_wburst = wvalid & wready & ~wsingle_q;  // an accepted beat of a burst: dropped
assign lw_wdata  = wdata;
assign lw_be     = wstrb;

wire w_hs   = (wvalid & wready) | (bvalid & bready);
wire w_hold = (wstate == W_IDLE && awvalid) || wstate == W_DATA || (wstate == W_RESP && !bready);
reg  wmark   = 1'b0;
reg  wfreed  = 1'b0;
wire w_free  = hrst_fall & wmark & ~w_hs;

always @(posedge clk) begin
	if (hrst_rise) wmark <= w_hold;
	else if (hrst_fall || w_hs || wstate == W_IDLE) wmark <= 1'b0;
	wfreed <= w_free;
end

always @(posedge clk) begin
	if (w_free) wstate <= W_IDLE;
	else case (wstate)
	W_IDLE: if (awvalid) begin
		bid_q     <= awid;
		waddr_q   <= awaddr;
		wsingle_q <= (awlen == 4'd0);
		wstate    <= W_DATA;
	end

	W_DATA: if (wvalid && wlast) wstate <= W_RESP;

	W_RESP: if (bready) wstate <= W_IDLE;

	default: wstate <= W_IDLE;
	endcase
end

////////////////////////////////// idle //////////////////////////////////

// 1 exactly while both state machines are in IDLE: the next states, registered.
reg idle_q = 1'b1;

always @(posedge clk) idle_q <= !(r_hold && !r_free) && !(w_hold && !w_free);

assign idle  = idle_q;
assign freed = {wfreed, rfreed};

endmodule

`default_nettype wire
`endif
