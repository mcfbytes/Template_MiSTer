//============================================================================
//  top_tb.sv -- part of the LW bridge framework (docs/LW_BRIDGE.md)
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

// Arm E of tb/run.sh: sys_top.v elaborated with the real LW files, hps_io, osd, sysmem and the Template's emu,
// against tb/sys_stubs.sv, in one configuration of macros (tb/static.sh --configs). It checks the top level's
// LW wiring by hierarchical reference:
//   - every LW net and port tb/static.sh lists for this configuration (lw_e_nets.vh) is driven: no bit reads z,
//     except the from-core SECTOR nets, which only a core's lw_blk_mux and lw_card_mux drive and the Template
//     does not instantiate: those must read z in every bit, so nothing in sys/ drives a bit the core owns;
//   - GPI: the ID view reads the core magic (bits 25, 24 and 22 clear); in the I/O view bit 25 is the firewall's
//     present output, 1, bit 24 its idle register and bit 22 the window's lease register, in every clk_100m cycle
//     of the run, each moving when its source is forced; without MISTER_LW bits 27:21 are clear;
//   - a CLAIM stored through the bridge atom sets GPI[22]; GPO[31] low raises lw_idv, which ends the lease;
//   - HPS_BUS has the LW width at emu and at hps_io, and each field reaches the other end: the PAD store as
//     hps_io decodes it, the status hold from hps_io's registers to lw_sys_window;
//   - forcing each bit of lw_sys_window's lines (ordinal k) moves f2h_irq[7+k] at the interrupt atom and no other;
//   - each osd instance that exists carries the window's OSDBUF store ports, on clk_100m;
//   - the link's selects, data and acknowledge reach hps_io, the osds and GPI as the stock decode makes them,
//     also through the lane's link front;
//   - an LW read through the bridge atom returns the directory magic and the default word;
//   - h2f_rst_n reaches the firewall through sysmem: low raises lw_hrst, and a read its master abandoned is
//     freed at the release, after which a read is answered.
// Implicit nets and the elaboration itself are judged by tb/run.sh.

`timescale 1ns/1ps
`default_nettype none

module top_tb;

`ifndef LW_TB_SYS_HP_PS
`define LW_TB_SYS_HP_PS 14762
`endif

reg clk50, clk_100m, clk_sys, clk_hdmi, clk_audio;
initial {clk50, clk_100m, clk_sys, clk_hdmi, clk_audio} = 0;
always #10.0   clk50     = ~clk50;
always #5.0    clk_100m  = ~clk_100m;
always #(`LW_TB_SYS_HP_PS / 1000.0) clk_sys = ~clk_sys;
always #6.734  clk_hdmi  = ~clk_hdmi;
always #20.345 clk_audio = ~clk_audio;

// Driven into the design by the stubs: the HPS reset, GPO, and the LW master's address, valid and RREADY lines.
reg        h2f_rst_n = 0;
reg [31:0] gp_out = 0;
reg [20:0] araddr = 0, awaddr = 0;
reg        arvalid = 0, awvalid = 0, wvalid = 0, rready = 1;
reg [31:0] wdata = 0;

sys_top dut
(
	.FPGA_CLK1_50(clk50),
	.FPGA_CLK2_50(clk50),
	.FPGA_CLK3_50(clk50),
	.HDMI_TX_INT(1'b1),
	.ADC_SDO(1'b0),
	.SD_SPI_MISO(1'b1),
	.KEY(2'b11),
	.SW(4'd0)
`ifndef MISTER_DUAL_SDRAM
	,
	.VGA_EN(1'b1),
	.BTN_USER(1'b1),
	.BTN_OSD(1'b1),
	.BTN_RESET(1'b1)
`endif
);

`ifdef MISTER_LW
localparam BUSW = 314;
`else
localparam BUSW = 46;
`endif

integer checks = 0, fails = 0, nnets = 0, nports = 0, core_z = 0, stock_z = 0;

task check(input string what, input [255:0] got, input [255:0] exp);
	checks = checks + 1;
	if (got !== exp) begin
		fails = fails + 1;
		$display("FAIL %s: got %0h exp %0h", what, got, exp);
	end
	else $display("pass %s = %0h", what, got);
endtask

function integer zc(input [1023:0] v);
	integer i;
	begin
		zc = 0;
		for (i = 0; i < 1024; i = i + 1) if (v[i] === 1'bz) zc = zc + 1;
	end
endfunction

// The from-core SECTOR nets: a core's muxes drive them; the Template has none.
function core_side(input string n);
	core_side = n == "lw_blk_up" || n == "lw_card_up";
endfunction

task lw_net(input string n, input integer z, input integer w);
	begin
		checks = checks + 1; nnets = nnets + 1;
		if (core_side(n) ? z != w : z != 0) begin
			fails = fails + 1;
			if (core_side(n)) $display("FAIL net %s: %0d of %0d bits driven; only a core drives it", n, w - z, w);
			else $display("FAIL net %s: %0d of %0d bits undriven", n, z, w);
		end
		else if (core_side(n)) $display("pass net %s: %0d bits, none driven (core side)", n, w);
		else $display("pass net %s: %0d bits", n, w);
	end
endtask

// A port of HPS_BUS carries the core-side nets and the stock bits too: every other bit of it must be driven.
task lw_port(input string p, input integer z, input integer w);
	integer exp;
	begin
		exp = (p == "emu.HPS_BUS" || p == "emu.hps_io.HPS_BUS") ? core_z + stock_z : 0;
		checks = checks + 1; nports = nports + 1;
		if (z != exp) begin
			fails = fails + 1;
			$display("FAIL port %s: %0d of %0d bits undriven, expected %0d", p, z, w, exp);
		end
		else $display("pass port %s: %0d bits, %0d undriven", p, w, z);
	end
endtask

task sys_cycles(input integer n); repeat (n) @(posedge clk_sys); #1; endtask

`ifdef MISTER_LW
integer n_freed = 0;   // the firewall's read machine freed at a release
always @(posedge clk_100m) if (dut.hps_lw_bridge.lw_freed === 2'b01) n_freed = n_freed + 1;

// GPI in the I/O view against the firewall, every clk_100m cycle: bit 25 its present output, bit 24 its idle
// register; gpi_on gates the monitor off while a check forces either source.
integer n_gpi = 0, n_gpi_bad = 0, n_gpi_busy = 0, n_gpi_held = 0;
reg     gpi_on = 0;
always @(negedge clk_100m) if (gpi_on && gp_out[31]) begin
	n_gpi = n_gpi + 1;
	if (dut.hps_lw_bridge.idle === 1'b0) n_gpi_busy = n_gpi_busy + 1;
	if (dut.lw_sys_window.live_q === 1'b1) n_gpi_held = n_gpi_held + 1;
	if (dut.h2f_gp.gp_in[25:24] !== {1'b1, dut.hps_lw_bridge.idle} || dut.hps_lw_bridge.idle === 1'bx ||
	    dut.h2f_gp.gp_in[22] !== dut.lw_sys_window.live_q || dut.lw_sys_window.live_q === 1'bx) begin
		n_gpi_bad = n_gpi_bad + 1;
		if (n_gpi_bad <= 4) $display("   GPI at %0t: gp_in[25:22] %b, the firewall's idle %b, the lease %b", $time,
		                             dut.h2f_gp.gp_in[25:22], dut.hps_lw_bridge.idle, dut.lw_sys_window.live_q);
	end
end

task lw_write(input [20:0] a, input [31:0] v);
	integer n;
	reg aw_hs, w_hs;
	begin
		@(negedge clk_100m);
		awaddr = a; awvalid = 1; wdata = v; wvalid = 1; n = 0;
		while (dut.hps_lw_bridge.bvalid !== 1'b1 && n <= 64) begin
			aw_hs = awvalid & dut.hps_lw_bridge.awready; w_hs = wvalid & dut.hps_lw_bridge.wready;
			@(negedge clk_100m); n = n + 1;
			if (aw_hs) awvalid = 0;
			if (w_hs) wvalid = 0;
		end
		awvalid = 0; wvalid = 0;
		checks = checks + 1;
		if (n > 64) begin fails = fails + 1; $display("FAIL LW store %06h: no BVALID", a); end
		@(negedge clk_100m);
	end
endtask

task lw_read(input [20:0] a, output [31:0] d);
	integer n;
	reg ar_hs;
	begin
		@(negedge clk_100m);
		araddr = a; arvalid = 1; n = 0;
		while (dut.hps_lw_bridge.rvalid !== 1'b1 && n <= 64) begin
			ar_hs = arvalid & dut.hps_lw_bridge.arready;
			@(negedge clk_100m); n = n + 1;
			if (ar_hs) arvalid = 0;
		end
		arvalid = 0;
		d = dut.hps_lw_bridge.rdata;
		checks = checks + 1;
		if (n > 64) begin fails = fails + 1; $display("FAIL LW read %06h: no RVALID", a); end
		@(negedge clk_100m);
	end
endtask
`endif

reg [31:0] d;
reg [48:0] pad;
reg [131:0] st;
reg  [2:0] ss;
integer i, k;
reg [63:0] irq0, irq1;
localparam [63:0] IRQ_LW = 64'hFFFF_FFFF_FFFF_FFFC;   // f2h_irq less upstream's two lines

// The Template leaves hps_io's ioctl_wait unconnected, and synthesis ties an unconnected input low.
`ifndef VERILATOR
initial force dut.emu.hps_io.ioctl_wait = 1'b0;
`endif

initial begin
	#200 h2f_rst_n = 1;
	gp_out = 32'h8000_0000;
	sys_cycles(200);

	$display("-- the LW nets and ports of this configuration are driven");
`ifdef MISTER_LW
	core_z  = zc(dut.lw_blk_up) + zc(dut.lw_card_up);
	stock_z = zc(dut.emu.HPS_BUS[45:0]);
`endif
`include "lw_e_nets.vh"
	$display("%0d LW nets, %0d LW ports", nnets, nports);

	$display("-- GPI: the core magic in the ID view; the firewall's bits 25 and 24 and the free bits in the I/O view");
	gp_out = 32'h0000_0000; #20;
`ifdef MISTER_DUAL_SDRAM
	check("ID view", dut.h2f_gp.gp_in, 32'h5CA623A8);
`else
	check("ID view", dut.h2f_gp.gp_in, 32'h5CA623A4);
`endif
	check("ID view gp_in[25:24] and gp_in[22]", {dut.h2f_gp.gp_in[25:24], dut.h2f_gp.gp_in[22]}, 3'b000);
	gp_out = 32'h8000_0000; #20;
`ifdef MISTER_LW
	check("I/O view gp_in[27:21], the firewall idle", dut.h2f_gp.gp_in[27:21], 7'b0011000);
	@(negedge clk_100m);
	force dut.hps_lw_bridge.adapter.idle_q = 1'b0; #1;
	check("the firewall's idle register forced 0: gp_in[25:24]", dut.h2f_gp.gp_in[25:24], 2'b10);
	release dut.hps_lw_bridge.adapter.idle_q;
	@(negedge clk_100m);
	check("released, the next clk_100m edge: gp_in[25:24]", dut.h2f_gp.gp_in[25:24], 2'b11);
	force dut.hps_lw_bridge.present = 1'b0; #1;
	check("the firewall's present output forced 0: gp_in[25:24]", dut.h2f_gp.gp_in[25:24], 2'b01);
	release dut.hps_lw_bridge.present; #1;
	check("released: gp_in[25:24]", dut.h2f_gp.gp_in[25:24], 2'b11);
	@(negedge clk_100m);
	gpi_on = 1;
`else
	check("I/O view gp_in[27:21]", dut.h2f_gp.gp_in[27:21], 7'b0000000);
`endif

	$display("-- HPS_BUS width at emu and at hps_io");
	check("emu.HPS_BUS width", $bits(dut.emu.HPS_BUS), BUSW);
	check("hps_io.HPS_BUS width", $bits(dut.emu.hps_io.HPS_BUS), BUSW);

`ifdef MISTER_LW
	$display("-- HPS_BUS: each LW field reaches the other end");
	for (k = 0; k < 2; k = k + 1) begin
		pad = (k != 0) ? ~49'h0_A5C3_1E87_D2B4 : 49'h1_5A3C_E178_2D4B;
		force dut.lw_sys_window.lw_pad = pad;
		sys_cycles(1);
		check("PAD store at hps_io {we, op, idx, data}",
		      {dut.emu.hps_io.lw_we, dut.emu.hps_io.lw_op, dut.emu.hps_io.lw_idx, dut.emu.hps_io.lw_dat}, pad);
		release dut.lw_sys_window.lw_pad;
		st = {4'hA, 128'h0123_4567_89AB_CDEF_FEDC_BA98_7654_3210};
		if (k != 0) st = ~st;
		force dut.emu.hps_io.stflg = st[131:128];
		force dut.emu.hps_io.status_req = st[127:0];
		sys_cycles(1);
		check("status hold at lw_sys_window.st_up", dut.lw_sys_window.st_up, st);
		release dut.emu.hps_io.stflg;
		release dut.emu.hps_io.status_req;
	end
	sys_cycles(4);

	$display("-- f2h_irq: each LW line ordinal k moves f2h_irq[7+k] and no other bit");
	force dut.lw_sys_window.lines = 3'b000; #1;
	irq0 = dut.interrupts.irq;
	check("f2h_irq[63:2] with every line low", {irq0 & IRQ_LW}, 64'd0);
	for (k = 1; k <= 3; k = k + 1) begin
		force dut.lw_sys_window.lines = 3'd1 << (k - 1); #1;
		irq1 = dut.interrupts.irq;
		check($sformatf("line ordinal %0d: f2h_irq[63:2] bits moved", k), {(irq0 ^ irq1) & IRQ_LW}, 64'd1 << (7 + k));
	end
	release dut.lw_sys_window.lines;
`else
	$display("-- f2h_irq: no LW line");
	check("f2h_irq[63:2]", {dut.interrupts.irq & IRQ_LW}, 64'd0);
`endif

`ifdef MISTER_LW
	$display("-- osd: each instance carries the OSDBUF store ports on clk_100m");
	for (k = 0; k < 2; k = k + 1) begin
		force dut.lw_sys_window.osd_we   = (k != 0) ? 1'b0 : 1'b1;
		force dut.lw_sys_window.osd_addr = (k != 0) ? 10'h15A : 10'h2A5;
		force dut.lw_sys_window.osd_data = (k != 0) ? 32'h3F00_11EE : 32'hC0FF_EE11;
		force dut.lw_sys_window.osd_be   = (k != 0) ? 4'h6 : 4'h9;
		force dut.lw_sys_window.osd_ctrl = (k != 0) ? 2'b01 : 2'b10;
		#1;
		check("vga_osd {lw_we, lw_addr, lw_data, lw_be, lw_ctrl}",
		      {dut.vga_osd.lw_we, dut.vga_osd.lw_addr, dut.vga_osd.lw_data, dut.vga_osd.lw_be, dut.vga_osd.lw_ctrl},
		      {dut.lw_sys_window.osd_we, dut.lw_sys_window.osd_addr, dut.lw_sys_window.osd_data,
		       dut.lw_sys_window.osd_be, dut.lw_sys_window.osd_ctrl});
`ifndef MISTER_DEBUG_NOHDMI
		check("hdmi_osd {lw_we, lw_addr, lw_data, lw_be, lw_ctrl}",
		      {dut.hdmi_osd.lw_we, dut.hdmi_osd.lw_addr, dut.hdmi_osd.lw_data, dut.hdmi_osd.lw_be, dut.hdmi_osd.lw_ctrl},
		      {dut.lw_sys_window.osd_we, dut.lw_sys_window.osd_addr, dut.lw_sys_window.osd_data,
		       dut.lw_sys_window.osd_be, dut.lw_sys_window.osd_ctrl});
`endif
	end
	release dut.lw_sys_window.osd_we; release dut.lw_sys_window.osd_addr; release dut.lw_sys_window.osd_data;
	release dut.lw_sys_window.osd_be; release dut.lw_sys_window.osd_ctrl;
	k = 0;
	for (i = 0; i < 8; i = i + 1) begin
		#1.25;
		if (dut.vga_osd.clk_lw !== clk_100m) k = k + 1;
`ifndef MISTER_DEBUG_NOHDMI
		if (dut.hdmi_osd.clk_lw !== clk_100m) k = k + 1;
`endif
	end
	check("osd clk_lw samples that differ from clk_100m", k, 0);
`endif

	$display("-- the link: selects, data and acknowledge as the stock decode makes them");
	for (i = 0; i < 8; i = i + 1) begin
		ss = i[2:0];
		gp_out = {1'b1, 10'd0, ss, 1'b0, 1'b0, {16'h1234 ^ {i[3:0], 12'd0}}};
		sys_cycles(6);
		check("io_fpga", dut.io_fpga, {~ss[1] & ss[0]});
		check("io_uio", dut.io_uio, {~ss[1] & ss[2]});
		check("hps_io fp_enable", dut.emu.hps_io.fp_enable, {~ss[1] & ss[0]});
		check("hps_io io_enable", dut.emu.hps_io.io_enable, {~ss[1] & ss[2]});
		check("hps_io io_din", dut.emu.hps_io.io_din, {16'h1234 ^ {i[3:0], 12'd0}});
		check("vga_osd io_osd", dut.vga_osd.io_osd, {ss[1] & ~ss[2]});
`ifndef MISTER_DEBUG_NOHDMI
		check("hdmi_osd io_osd", dut.hdmi_osd.io_osd, {ss[1] & ~ss[0]});
`endif
	end
	gp_out = 32'h8000_0000;
	sys_cycles(6);
	if ((dut.io_wait | dut.vs_wait) === 1'b0) begin
		gp_out[17] = 1; sys_cycles(8);
		check("GPI acknowledge after the link clock rises", dut.h2f_gp.gp_in[17], 1'b1);
		gp_out[17] = 0; sys_cycles(8);
		check("GPI acknowledge after the link clock falls", dut.h2f_gp.gp_in[17], 1'b0);
	end
	else begin
		checks = checks + 1; fails = fails + 1;
		$display("FAIL the link waits (io_wait %b, vs_wait %b): the acknowledge cannot be checked", dut.io_wait, dut.vs_wait);
	end

`ifdef MISTER_LW
	$display("-- an LW read through the bridge atom");
	lw_read(21'h40000, d); check("directory magic at 0x40000", d, 32'h5244574C);
	lw_read(21'h00000, d); check("undecoded 0x00000", d, 32'hDEADBEEF);

	$display("-- the lease: GPI[22] is the window's lease register; GPO[31] low ends it");
	check("no lease: gp_in[22]", dut.h2f_gp.gp_in[22], 1'b0);
	lw_write(21'h40018, {16'h4C57, 10'h2A7, 2'd1, 4'd15});
	lw_read(21'h40018, d);
	check("a CLAIM through the bridge atom: held, owner", {d[31], d[9:0]}, {1'b1, 10'h2A7});
	check("gp_in[22] shows it", dut.h2f_gp.gp_in[22], 1'b1);
	gpi_on = 0;
	@(negedge clk_100m);
	force dut.lw_sys_window.held = 1'b0; #1;
	check("the window's held output forced 0: gp_in[27:21]", dut.h2f_gp.gp_in[27:21], 7'b0011000);
	release dut.lw_sys_window.held; #1;
	check("released: gp_in[27:21]", dut.h2f_gp.gp_in[27:21], 7'b0011010);
	check("no GPO[31] low seen yet: lw_idv", dut.lw_idv, 1'b0);
	gp_out[31] = 1'b0;
	repeat (8) @(negedge clk_100m);
	check("GPO[31] low: lw_idv", dut.lw_idv, 1'b1);
	check("GPO[31] low: the lease ended", dut.lw_sys_window.live_q, 1'b0);
	check("the ID view: gp_in[22]", dut.h2f_gp.gp_in[22], 1'b0);
	gp_out[31] = 1'b1;
	repeat (8) @(negedge clk_100m);
	check("GPO[31] high again: lw_idv", dut.lw_idv, 1'b0);
	gpi_on = 1;
	lw_read(21'h40018, d);
	check("LEASE: not held, end 3 (GPO[31] seen low)", {d[31], d[13:12]}, {1'b0, 2'd3});
`ifdef MISTER_LW_LANE
	$display("-- the link's ID view reaches the lane player's busy input");
	check("the I/O view, no select: the player's busy input", dut.lw_sys_window.lane_busy, 1'b0);
	gp_out[31] = 1'b0; sys_cycles(3);
	check("GPO[31] low: the player's busy input", dut.lw_sys_window.lane_busy, 1'b1);
	check("GPO[31] low: the lane's own input", dut.lw_sys_window.lane.lane.l_busy, 1'b1);
	gp_out[31] = 1'b1; sys_cycles(3);
	check("GPO[31] high again: not busy", dut.lw_sys_window.lane_busy, 1'b0);
`endif
	// the next CLAIM waits for the lease links' return: LEASE.quiet
	k = 0; d = 0;
	while (!d[30] && k < 200) begin lw_read(21'h40018, d); k = k + 1; end
	check("the lease links returned: quiet", d[30], 1'b1);
	lw_write(21'h40018, {16'h4C57, 10'h2A7, 2'd1, 4'd15});
	repeat (4) @(negedge clk_100m);
	check("claimed again: gp_in[22]", dut.h2f_gp.gp_in[22], 1'b1);

	$display("-- the HPS reset reaches the firewall");
	check("lw_hrst low, the firewall idle", {dut.hps_lw_bridge.lw_hrst, dut.hps_lw_bridge.idle}, 2'b01);
	rready = 0;
	@(negedge clk_100m); araddr = 21'h40000; arvalid = 1;
	@(negedge clk_100m); arvalid = 0;
	repeat (4) @(negedge clk_100m);
	check("a read its master abandoned: RVALID held, not idle", {dut.hps_lw_bridge.rvalid, dut.hps_lw_bridge.idle}, 2'b10);
	check("a read its master abandoned: gp_in[25:24]", dut.h2f_gp.gp_in[25:24], 2'b10);
	k = n_freed;
	h2f_rst_n = 0;
	repeat (6) @(negedge clk_100m);
	check("h2f_rst_n low: sysmem's hps_rst_n", dut.sysmem.hps_rst_n, 1'b0);
	check("h2f_rst_n low: lw_hrst", dut.hps_lw_bridge.lw_hrst, 1'b1);
	h2f_rst_n = 1;
	repeat (16) @(negedge clk_100m);
	check("after the release: lw_hrst", dut.hps_lw_bridge.lw_hrst, 1'b0);
	check("after the release: read machines freed", n_freed - k, 1);
	check("after the release: RVALID dropped, idle", {dut.hps_lw_bridge.rvalid, dut.hps_lw_bridge.idle}, 2'b01);
	check("after the release: gp_in[25:24]", dut.h2f_gp.gp_in[25:24], 2'b11);
	rready = 1;
	lw_read(21'h40000, d); check("a read after the release", d, 32'h5244574C);

	$display("-- GPI against the firewall in every clk_100m cycle of the I/O view");
	gpi_on = 0;
	$display("     %0d cycles, %0d with a transaction held, %0d with the lease held", n_gpi, n_gpi_busy, n_gpi_held);
	check("cycles where gp_in[25:22] differs from {1, the firewall's idle, 0, the lease}", n_gpi_bad, 0);
	check("cycles compared, with and without a transaction held, with and without the lease",
	      {n_gpi > 100, n_gpi_busy > 10, n_gpi_held > 10, n_gpi - n_gpi_held > 10}, 4'b1111);
`endif

	$display("%s: %0d checks, %0d failures", (fails != 0) ? "FAILED" : "PASSED", checks, fails);
	$finish;
end

initial begin #2000000 $display("FAILED: timeout"); $finish; end

endmodule

`default_nettype wire
