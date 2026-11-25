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
//   - every LW net and port tb/static.sh lists for this configuration (lw_e_nets.vh) is driven: no bit reads z.
//     HPS_BUS[383:46] (sys_top's lw_bus) has a weak pull-down here, as synthesis ties a net with no source low,
//     and each of its bits is driven exactly when the bit map says: every to-core bit by lw_sys_window, the PAD
//     and EVT exports and presence bits by hps_io when their tier is built, SECTOR's commit mark and its
//     presence bit by hps_io and the primary's request, LBA and mux presence bit by the Template's block reader
//     (its lw_blk_mux) when SECTOR is built, and nothing else (the Template has no save-slot mux);
//   - presence: PAD, EVT and SECTOR are listed exactly when built (the Template passes the wide bus to hps_io and
//     to its block reader's mux), OSDBUF and LANE when built, and COUNT agrees;
//   - GPI: the ID view reads the core magic (bits 25, 24 and 22 clear); in the I/O view bit 25 is the firewall's
//     present output, 1, bit 24 its idle register and bit 22 the window's enable register, in every clk_100m cycle
//     of the run, each moving when its source is forced; without MISTER_LW bits 27:21 are clear;
//   - an ENABLE stored through the bridge atom sets GPI[22]; GPO[31] low raises lw_idv, which clears it;
//   - HPS_BUS has the LW width at emu and at hps_io, and each field reaches the other end: the PAD store as
//     hps_io decodes it, the status hold, the event pulses and the event levels from hps_io to lw_sys_window, and
//     the commit mark from hps_io to lw_sector, each when its tier is built;
//   - forcing each bit of lw_sys_window's lines (ordinal k) moves f2h_irq[7+k] at the interrupt atom and no other;
//   - with the lane, its entry names line 3 and its CAUSE word, and a frame stored through the bridge atom and
//     played raises the lane's cause and f2h_irq[10] once its LINE_EN bit is set; the store-back lowers it;
//   - with SECTOR, the Template's block reader: a block served over LW lands in its RAM in strobe order, its sum
//     and LBA come back once through status_in, and hps_io never sees the request; with the slot's EN clear the
//     next request reaches hps_io's sd_rd with the next LBA;
//   - with the OSD tier, each osd instance that exists carries the window's OSDBUF write port, and the window takes
//     both osd selects as each osd sees them (the HDMI one 0 without an HDMI osd) and GPO[31] on clk_sys;
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

`ifdef MISTER_OCRAM
localparam [6:0] GPI_OCRAM = 7'b0100000;   // gp_in[26], the OCRAM floor's present bit
`else
localparam [6:0] GPI_OCRAM = 7'b0000000;
`endif

`ifdef MISTER_LW
localparam BUSW = 384;
`else
localparam BUSW = 46;
`endif

integer checks = 0, fails = 0, nnets = 0, nports = 0, stock_z = 0;

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

task lw_net(input string n, input integer z, input integer w);
	begin
		checks = checks + 1; nnets = nnets + 1;
		if (z != 0) begin
			fails = fails + 1;
			$display("FAIL net %s: %0d of %0d bits undriven", n, z, w);
		end
		else $display("pass net %s: %0d bits", n, w);
	end
endtask

// A port of HPS_BUS carries the stock bits too, some of which nothing drives: every other bit of it must be.
task lw_port(input string p, input integer z, input integer w);
	integer exp;
	begin
		exp = (p == "emu.HPS_BUS" || p == "emu.hps_io.HPS_BUS") ? stock_z : 0;
		checks = checks + 1; nports = nports + 1;
		if (z != exp) begin
			fails = fails + 1;
			$display("FAIL port %s: %0d of %0d bits undriven, expected %0d", p, z, w, exp);
		end
		else $display("pass port %s: %0d bits, %0d undriven", p, w, z);
	end
endtask

task sys_cycles(input integer n); repeat (n) @(posedge clk_sys); #1; endtask

`ifdef MISTER_LW_SECTOR_SLOT
`define LW_TB_ANY_SECTOR
`elsif MISTER_OCRAM_SECTOR_SLOT
`define LW_TB_ANY_SECTOR
`endif
`ifdef MISTER_LW
// HPS_BUS[383:46]: a net with no source reads 0, as synthesis ties it; a bit any piece drives overrides the pull.
assign (weak0, weak1) dut.lw_bus = 338'd0;
`ifdef MISTER_LW_EVT
// The Template leaves these hps_io inputs open, and synthesis ties an open input port low; the event exports read
// them. With SECTOR the Template's block reader drives the requests.
assign (weak0, weak1) dut.emu.hps_io.info_req = 1'b0;
assign (weak0, weak1) dut.emu.hps_io.ioctl_upload_req = 1'b0;
`ifndef LW_TB_ANY_SECTOR
assign (weak0, weak1) dut.emu.hps_io.sd_rd = 1'b0;
assign (weak0, weak1) dut.emu.hps_io.sd_wr = 1'b0;
`endif
`endif

`ifdef MISTER_LW_PAD
localparam TB_PAD = 1;
`else
localparam TB_PAD = 0;
`endif
`ifdef MISTER_LW_EVT
localparam TB_EVT = 1;
`else
localparam TB_EVT = 0;
`endif
`ifdef MISTER_LW_SECTOR_SLOT
localparam TB_SEC = 1, TB_SLOT = `MISTER_LW_SECTOR_SLOT;
`else
localparam TB_SEC = 0;
`endif
// The OCRAM mailbox's SECTOR feeder drives the same bus bits through the window, which lists no SECTOR entry.
`ifdef MISTER_OCRAM_SECTOR_SLOT
localparam TB_XSEC = 1;
`else
localparam TB_XSEC = 0;
`endif
`ifdef MISTER_LW_OSD
localparam TB_OSD = 1;
`else
localparam TB_OSD = 0;
`endif
`ifdef MISTER_LW_LANE
localparam TB_LANE = 1;
`else
localparam TB_LANE = 0;
`endif

// The bits of HPS_BUS[383:46] some piece drives in this configuration (bit i - 46 of the mask is bus bit i): every
// to-core bit [131:46]; with PAD hps_io's presence bit [375]; with EVT the event pulses [239:237], the status
// hold [371:240], the event levels [374:372] and [376]; with SECTOR hps_io's commit mark [380] and [381], and the
// Template's block reader: its mux's {lba, req, ack, reset} [166:132] and presence bit [377]; the same with the
// OCRAM SECTOR tier.
function [383:46] bus_driven;
	reg [383:0] m;
	begin
		m = 0;
		m[131:46] = '1;
		if (TB_PAD) m[375] = 1'b1;
		if (TB_EVT) begin m[374:237] = '1; m[376] = 1'b1; end
		if (TB_SEC || TB_XSEC) begin m[381:380] = 2'b11; m[166:132] = '1; m[377] = 1'b1; end
		bus_driven = m[383:46];
	end
endfunction

// The bits of lw_bus that something drives: a bit whose strength reads weak follows the pull alone. Each bit is
// probed by a constant index (a variable one reads the value without its strength).
reg [383:46] bus_strong;
event        bus_probe;
genvar       gb;
generate for (gb = 46; gb < 384; gb = gb + 1) begin : bw
	always @(bus_probe) begin : p
		string t;
		t = $sformatf("%v", dut.lw_bus[gb]);
		bus_strong[gb] = t.substr(0, 1) != "We";
	end
end endgenerate

`ifdef MISTER_LW_SECTOR_SLOT
// The Template's block reader: its status write-backs, and any sign of its request at hps_io while blk_watch is set.
integer       n_set = 0, n_hrq = 0;
reg   [127:0] set_val;
reg           blk_watch = 0;
always @(posedge clk_sys) begin
	if (dut.emu.status_set === 1'b1) begin n_set = n_set + 1; set_val = dut.emu.status_in; end
	if (blk_watch && (dut.emu.hps_io.sd_rd[TB_SLOT] !== 1'b0 || dut.emu.hps_io.sd_ack[TB_SLOT] !== 1'b0))
		n_hrq = n_hrq + 1;
end

task blk_press;   // the Template's second pad button, "Read block" (hps_io's joystick_0 reads x: no SPI host)
	begin
		force dut.emu.joystick_0 = 32'h00; sys_cycles(2);
		force dut.emu.joystick_0 = 32'h20; sys_cycles(4);
		force dut.emu.joystick_0 = 32'h00; sys_cycles(2);
		release dut.emu.joystick_0;
	end
endtask
`endif

integer n_freed = 0;   // the firewall's read machine freed at a release
always @(posedge clk_100m) if (dut.hps_lw_bridge.lw_freed === 2'b01) n_freed = n_freed + 1;

// GPI in the I/O view against the firewall, every clk_100m cycle: bit 25 its present output, bit 24 its idle
// register; gpi_on gates the monitor off while a check forces either source.
integer n_gpi = 0, n_gpi_bad = 0, n_gpi_busy = 0, n_gpi_held = 0;
reg     gpi_on = 0;
always @(negedge clk_100m) if (gpi_on && gp_out[31]) begin
	n_gpi = n_gpi + 1;
	if (dut.hps_lw_bridge.idle === 1'b0) n_gpi_busy = n_gpi_busy + 1;
	if (dut.lw_sys_window.en_q === 1'b1) n_gpi_held = n_gpi_held + 1;
	if (dut.h2f_gp.gp_in[25:24] !== {1'b1, dut.hps_lw_bridge.idle} || dut.hps_lw_bridge.idle === 1'bx ||
	    dut.h2f_gp.gp_in[22] !== dut.lw_sys_window.en_q || dut.lw_sys_window.en_q === 1'bx) begin
		n_gpi_bad = n_gpi_bad + 1;
		if (n_gpi_bad <= 4) $display("   GPI at %0t: gp_in[25:22] %b, the firewall's idle %b, the enable %b", $time,
		                             dut.h2f_gp.gp_in[25:22], dut.hps_lw_bridge.idle, dut.lw_sys_window.en_q);
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
reg [383:46] bus_w, bus_e;
reg [48:0] pad;
reg [131:0] st;
reg  [2:0] ss;
integer i, k;
reg [63:0] irq0, irq1;
localparam [63:0] IRQ_LW = 64'hFFFF_FFFF_FFFF_FFFC;   // f2h_irq less upstream's two lines
`ifdef MISTER_LW_SECTOR_SLOT
reg [31:0] blk_w, blk_w0;
reg [15:0] blk_sum;
`endif

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
	stock_z = zc(dut.emu.HPS_BUS[45:0]);
`endif
`include "lw_e_nets.vh"
	$display("%0d LW nets, %0d LW ports", nnets, nports);
`ifdef MISTER_LW
	$display("-- HPS_BUS[383:46]: each bit driven exactly as the bit map says for this tier set");
	-> bus_probe; #0.01;
	bus_w = bus_strong;
	bus_e = bus_driven();
	k = 0;
	for (i = 46; i < 384; i = i + 1) if (bus_w[i] !== bus_e[i]) begin
		if (k < 6) $display("   HPS_BUS[%0d]: %s, the map says %s", i, bus_w[i] ? "driven" : "undriven",
		                    bus_e[i] ? "driven" : "undriven");
		k = k + 1;
	end
	check("HPS_BUS[383:46] bits whose driver differs from the bit map", k, 0);
	check("HPS_BUS[383:46] reads no x or z", ^dut.lw_bus === 1'bx, 1'b0);
`endif

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
	check("I/O view gp_in[27:21], the firewall idle", dut.h2f_gp.gp_in[27:21], 7'b0011000 | GPI_OCRAM);
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
`ifdef MISTER_OCRAM
	check("I/O view gp_in[27:21], the OCRAM present bit", dut.h2f_gp.gp_in[27:21], 7'b0100000);
	check("GPI[26] is the OCRAM master's present", dut.h2f_gp.gp_in[26], dut.ocram_master.present);
`else
	check("I/O view gp_in[27:21]", dut.h2f_gp.gp_in[27:21], 7'b0000000);
`endif
`endif

	$display("-- HPS_BUS width at emu and at hps_io");
	check("emu.HPS_BUS width", $bits(dut.emu.HPS_BUS), BUSW);
	check("hps_io.HPS_BUS width", $bits(dut.emu.hps_io.HPS_BUS), BUSW);

`ifdef MISTER_LW
	$display("-- HPS_BUS: each LW field reaches the other end");
	for (k = 0; k < 2; k = k + 1) begin
`ifdef MISTER_LW_PAD
		pad = (k != 0) ? ~49'h0_A5C3_1E87_D2B4 : 49'h1_5A3C_E178_2D4B;
		force dut.lw_sys_window.pad_q = pad;
		sys_cycles(1);
		check("PAD store at hps_io {we, op, idx, data}",
		      {dut.emu.hps_io.lw_we, dut.emu.hps_io.lw_op, dut.emu.hps_io.lw_idx, dut.emu.hps_io.lw_dat}, pad);
		release dut.lw_sys_window.pad_q;
`endif
`ifdef MISTER_LW_EVT
		st = {4'hA, 128'h0123_4567_89AB_CDEF_FEDC_BA98_7654_3210};
		if (k != 0) st = ~st;
		force dut.emu.hps_io.stflg = st[131:128];
		force dut.emu.hps_io.status_req = st[127:0];
		sys_cycles(1);
		check("status hold at lw_evt {stflg, status_req}",
		      {dut.lw_sys_window.evt.evt.st[3:0], dut.lw_sys_window.evt.evt.st[131:4]}, st);
		release dut.emu.hps_io.stflg;
		release dut.emu.hps_io.status_req;
		// the pulses {upload, info, blk} and the levels {upload pending, info pending, a request stands}
		force dut.emu.hps_io.ioctl_upload_req = 1'b1; force dut.emu.hps_io.old_upload_req = k[0];
		force dut.emu.hps_io.info_req = 1'b1; force dut.emu.hps_io.old_info = !k[0];
		force dut.emu.hps_io.ev_rq = !k[0]; force dut.emu.hps_io.ev_rq_d = 1'b0; force dut.emu.hps_io.ev_ack_d = 1'b0;
		force dut.emu.hps_io.upload_req = !k[0]; force dut.emu.hps_io.info_n = k[0] ? 8'h5A : 8'h00;
		sys_cycles(1);
		check("event pulses at lw_evt {upload, info, blk}", dut.lw_sys_window.evt.evt.ev, k[0] ? 3'b010 : 3'b101);
		check("event levels at lw_evt {upload, info, request}", dut.lw_sys_window.evt.evt.lvl, k[0] ? 3'b010 : 3'b101);
		release dut.emu.hps_io.ioctl_upload_req; release dut.emu.hps_io.old_upload_req;
		release dut.emu.hps_io.info_req; release dut.emu.hps_io.old_info;
		release dut.emu.hps_io.ev_rq; release dut.emu.hps_io.ev_rq_d; release dut.emu.hps_io.ev_ack_d;
		release dut.emu.hps_io.upload_req; release dut.emu.hps_io.info_n;
`endif
`ifdef MISTER_LW_SECTOR_SLOT
		force dut.emu.hps_io.sd_commit = !k[0];
		sys_cycles(1);
		check("the commit mark at lw_sector", dut.lw_sys_window.sec.sector.c_commit, !k[0]);
		release dut.emu.hps_io.sd_commit;
`endif
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
`ifdef MISTER_OCRAM
	$display("-- f2h_irq: the OCRAM EVT line only");
	check("f2h_irq[11] is the master's line", dut.interrupts.irq[11], dut.ocram_master.line);
	check("f2h_irq[63:12] and f2h_irq[10:2]", {dut.interrupts.irq[63:12], dut.interrupts.irq[10:2]}, 62'd0);
`else
	$display("-- f2h_irq: no LW line");
	check("f2h_irq[63:2]", {dut.interrupts.irq & IRQ_LW}, 64'd0);
`endif
`endif

`ifdef MISTER_LW_OSD
	$display("-- osd: each instance carries the OSDBUF write port");
	for (k = 0; k < 2; k = k + 1) begin
		force dut.lw_sys_window.osd_we   = (k != 0) ? 1'b0 : 1'b1;
		force dut.lw_sys_window.osd_addr = (k != 0) ? 13'h15A5 : 13'h0A5A;
		force dut.lw_sys_window.osd_byte = (k != 0) ? 8'h3C : 8'hC3;
		#1;
		check("vga_osd {lw_we, lw_addr, lw_byte}", {dut.vga_osd.lw_we, dut.vga_osd.lw_addr, dut.vga_osd.lw_byte},
		      {dut.lw_sys_window.osd_we, dut.lw_sys_window.osd_addr, dut.lw_sys_window.osd_byte});
`ifndef MISTER_DEBUG_NOHDMI
		check("hdmi_osd {lw_we, lw_addr, lw_byte}", {dut.hdmi_osd.lw_we, dut.hdmi_osd.lw_addr, dut.hdmi_osd.lw_byte},
		      {dut.lw_sys_window.osd_we, dut.lw_sys_window.osd_addr, dut.lw_sys_window.osd_byte});
`endif
	end
	release dut.lw_sys_window.osd_we; release dut.lw_sys_window.osd_addr; release dut.lw_sys_window.osd_byte;
`endif

`ifdef MISTER_OCRAM_OSD
	$display("-- osd: the OCRAM writer's bytes reach each osd, and the mux takes the LW writer when it is quiet");
	force dut.ocram_rx.we = 1'b1;
	force dut.ocram_rx.addr = 13'h0A5A;
	force dut.ocram_rx.dbyte = 8'hC3;
	#1;
	check("vga_osd {lw_we, lw_addr, lw_byte} from the OCRAM writer",
	      {dut.vga_osd.lw_we, dut.vga_osd.lw_addr, dut.vga_osd.lw_byte}, {1'b1, 13'h0A5A, 8'hC3});
`ifndef MISTER_DEBUG_NOHDMI
	check("hdmi_osd {lw_we, lw_addr, lw_byte} from the OCRAM writer",
	      {dut.hdmi_osd.lw_we, dut.hdmi_osd.lw_addr, dut.hdmi_osd.lw_byte}, {1'b1, 13'h0A5A, 8'hC3});
`endif
	force dut.ocram_rx.we = 1'b0; #1;
`ifdef LW_OSD_PORTS
	force dut.lw_sys_window.osd_we = 1'b1;
	force dut.lw_sys_window.osd_addr = 13'h15A5;
	force dut.lw_sys_window.osd_byte = 8'h3C;
	#1;
	check("the mux takes the LW writer while the OCRAM writer is quiet",
	      {dut.vga_osd.lw_we, dut.vga_osd.lw_addr, dut.vga_osd.lw_byte}, {1'b1, 13'h15A5, 8'h3C});
	release dut.lw_sys_window.osd_we; release dut.lw_sys_window.osd_addr; release dut.lw_sys_window.osd_byte;
`endif
	release dut.ocram_rx.we; release dut.ocram_rx.addr; release dut.ocram_rx.dbyte;
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
`ifdef MISTER_LW_OSD
`ifdef MISTER_DEBUG_NOHDMI
		check("the OSD writer's selects: the VGA osd's, and 0 for the HDMI osd that is not built",
		      dut.lw_sys_window.osd_sel, {dut.vga_osd.io_osd, 1'b0});
`else
		check("the OSD writer's selects: {VGA, HDMI} as each osd sees them", dut.lw_sys_window.osd_sel,
		      {dut.vga_osd.io_osd, dut.hdmi_osd.io_osd});
`endif
		check("the OSD writer's view: the I/O view", dut.lw_sys_window.osd_view, 1'b1);
`endif
	end
`ifdef MISTER_LW_OSD
	gp_out = 32'h0000_0000;
	sys_cycles(6);
	check("the OSD writer's view: the ID view, GPO[31] on clk_sys", dut.lw_sys_window.osd_view, 1'b0);
`endif
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

	$display("-- presence: the tiers built and wired are listed; SECTOR through the Template's block reader");
	check("PAD listed: built, and hps_io's write arm present", dut.lw_sys_window.has_pad, TB_PAD);
	check("EVT listed: built, and hps_io's exports present", dut.lw_sys_window.has_evt, TB_EVT);
	check("SECTOR listed: built, the block reader's mux and hps_io's commit mark present", dut.lw_sys_window.has_sec,
	      TB_SEC);
	check("SECTOR's wired slots: the primary alone", dut.lw_sys_window.wired, TB_SEC ? 3'b001 : 3'b000);
	check("hps_io's commit mark present exactly when a SECTOR feeder is built", dut.lw_sys_window.pres_cmt,
	      TB_SEC | TB_XSEC);
	lw_read(21'h4000C, d);
	check("COUNT: PAD, SECTOR, OSDBUF, EVT and LANE as built", d, TB_PAD + TB_SEC + TB_EVT + TB_OSD + TB_LANE);
	lw_read(21'h80000, d); check("the PAD block's first word", d, TB_PAD ? 32'd0 : 32'hDEADBEEF);
	lw_read(21'h4301C, d); check("the EVT block's ID word", d, TB_EVT ? 32'h32545645 : 32'hDEADBEEF);
	lw_read(21'h48014, d); check("the SECTOR block's ID word", d, TB_SEC ? 32'h33434553 : 32'hDEADBEEF);

	$display("-- the host enable: GPI[22] is the window's enable register; GPO[31] low clears it");
	check("no enable: gp_in[22]", dut.h2f_gp.gp_in[22], 1'b0);
	lw_write(21'h40018, 32'h4C570001);
	lw_read(21'h40018, d);
	check("an ENABLE through the bridge atom: set", d[31], 1'b1);
	check("gp_in[22] shows it", dut.h2f_gp.gp_in[22], 1'b1);
	gpi_on = 0;
	@(negedge clk_100m);
	force dut.lw_sys_window.held = 1'b0; #1;
	check("the window's held output forced 0: gp_in[27:21]", dut.h2f_gp.gp_in[27:21], 7'b0011000 | GPI_OCRAM);
	release dut.lw_sys_window.held; #1;
	check("released: gp_in[27:21]", dut.h2f_gp.gp_in[27:21], 7'b0011010 | GPI_OCRAM);
	check("no GPO[31] low seen yet: lw_idv", dut.lw_idv, 1'b0);
	gp_out[31] = 1'b0;
	repeat (8) @(negedge clk_100m);
	check("GPO[31] low: lw_idv", dut.lw_idv, 1'b1);
	check("GPO[31] low: the enable cleared", dut.lw_sys_window.en_q, 1'b0);
	check("the ID view: gp_in[22]", dut.h2f_gp.gp_in[22], 1'b0);
	gp_out[31] = 1'b1;
	repeat (8) @(negedge clk_100m);
	check("GPO[31] high again: lw_idv", dut.lw_idv, 1'b0);
	gpi_on = 1;
	lw_read(21'h40018, d);
	check("ENABLE: clear", d[31], 1'b0);
`ifdef MISTER_LW_LANE
	$display("-- the link's ID view reaches the lane player's busy input");
	check("the I/O view, no select: the player's busy input", dut.lw_sys_window.lane_busy, 1'b0);
	gp_out[31] = 1'b0; sys_cycles(3);
	check("GPO[31] low: the player's busy input", dut.lw_sys_window.lane_busy, 1'b1);
	check("GPO[31] low: the lane's own input", dut.lw_sys_window.lane.lane.l_busy, 1'b1);
	gp_out[31] = 1'b1; sys_cycles(3);
	check("GPO[31] high again: not busy", dut.lw_sys_window.lane_busy, 1'b0);
`endif
	// the next ENABLE waits for the enable links' return: ENABLE.quiet
	k = 0; d = 0;
	while (!d[30] && k < 200) begin lw_read(21'h40018, d); k = k + 1; end
	check("the enable links returned: quiet", d[30], 1'b1);
	lw_write(21'h40018, 32'h4C570001);
	repeat (4) @(negedge clk_100m);
	check("enabled again: gp_in[22]", dut.h2f_gp.gp_in[22], 1'b1);
`ifdef MISTER_LW_LANE
	$display("-- line 3: a lane frame played raises f2h_irq[10] through the lane's cause and its LINE_EN bit");
	i = TB_PAD + TB_SEC + TB_OSD + TB_EVT;              // the lane's entry: the last listed
	lw_read(21'h40050 + 21'h80 * i, d); check("the lane's entry: CAUSE at window offset 0x06024", d, 32'h00006024);
	lw_read(21'h40054 + 21'h80 * i, d); check("the lane's entry: LINE 3", d, 32'd3);
	lw_read(21'h40058 + 21'h80 * i, d); check("the lane's entry: CAUSE_BITS", d, 32'h7);
	k = 0; d = 0;
	while (!d[i] && k < 200) begin lw_read(21'h4002C, d); k = k + 1; end
	check("the lane's OPEN bit", d[i], 1'b1);
	lw_write(21'h40030, 32'd1 << i);                    // LINE_EN: the lane's bit
	lw_write(21'h46000, 32'h00000000);                  // BEGIN: UIO, opcode 0 (no decoder acts on it)
	lw_write(21'h46008, 32'h00000000);                  // END
	k = 0; d = 0;
	while (d[15:0] != 16'd1 && k < 200) begin lw_read(21'h46010, d); k = k + 1; end
	check("the frame played: PLAYED", d[15:0], 16'd1);
	repeat (4) @(negedge clk_100m);
	lw_read(21'h46024, d); check("the lane's CAUSE: PLAYED", d, 32'h1);
	check("f2h_irq[10] high, and no other LW line", {dut.interrupts.irq & IRQ_LW}, 64'd1 << 10);
	lw_write(21'h46024, 32'h7);
	repeat (2) @(negedge clk_100m);
	check("the store-back: f2h_irq[10] low", {dut.interrupts.irq & IRQ_LW}, 64'd0);
`endif

`ifdef MISTER_LW_SECTOR_SLOT
	$display("-- the Template's block reader: a block served over LW fills its RAM; its sum comes back as status");
	i = TB_PAD;                                         // SECTOR's entry: after PAD's
	k = 0; d = 0;
	while (!d[i] && k < 200) begin lw_read(21'h4002C, d); k = k + 1; end
	check("SECTOR's OPEN bit", d[i], 1'b1);
	lw_write(21'h48008, 32'd1);                         // CTRL: EN for slot 0, the primary
	k = 0; d = 0;
	while (!d[0] && k < 200) begin lw_read(21'h4800C, d); k = k + 1; end
	check("STATUS: the primary's en_sys", d[0], 1'b1);
	check("the block reader's mux takes the LW side", dut.emu.blk_lw.en, 1'b1);
	// sysmem holds the core in its power-on reset for 20 ms, the whole run: the case lifts the Template's own reset
	force dut.emu.reset = 1'b0;
	sys_cycles(4);
	n_set = 0; n_hrq = 0; blk_watch = 1;
	blk_press;
	k = 0; d = 0;
	while (!d[31] && k < 200) begin lw_read(21'h48004, d); k = k + 1; end
	check("REQ: the primary's read on its hps_io slot, BLKSZ 2", d,
	      32'h8000_8081 | ({28'd0, TB_SLOT[3:0]} << 2));
	lw_read(21'h48104, d); check("LBA_S[0]: the first block after a reset, 0", d, 32'd0);
	lw_write(21'h48108, 32'd0);                         // BEGIN_S[0], the LBA echoed
	blk_sum = 0;
	for (k = 0; k < 128; k = k + 1) begin
		blk_w = 32'h9E37_79B9 * (k + 1);
		lw_write(21'h4C000 + 4 * k, blk_w);             // DATA[k]
		blk_sum = blk_sum + blk_w[15:0] + blk_w[31:16];
		if (k == 0) blk_w0 = blk_w;
	end
	k = 0;
	while (n_set == 0 && k < 4000) begin sys_cycles(1); k = k + 1; end
	sys_cycles(8);
	check("status written back once", n_set, 1);
	check("status_in[63:32]: {LBA 0, the 16-bit sum of the block's 256 words}", set_val[63:32], {16'd0, blk_sum});
	check("the RAM, words 0 and 1: DATA[0], low half first", {dut.emu.blk_ram[1], dut.emu.blk_ram[0]}, blk_w0);
	check("the RAM, word 255: DATA[127]'s high half", dut.emu.blk_ram[255], blk_w[31:16]);
	check("the request seen at hps_io (sd_rd, sd_ack): cycles", n_hrq, 0);
	blk_watch = 0;
	$display("-- the Template's block reader: with the slot's EN clear, its next request goes to hps_io");
	lw_write(21'h48008, 32'd0);
	k = 0; d = 1;
	while (d[0] && k < 200) begin lw_read(21'h4800C, d); k = k + 1; end
	check("STATUS: the primary's en_sys clear", d[0], 1'b0);
	blk_press;
	check("hps_io's sd_rd[slot]", dut.emu.hps_io.sd_rd[TB_SLOT], 1'b1);
	check("hps_io's sd_lba[slot]: the next block, 1", dut.emu.hps_io.sd_lba[TB_SLOT], 32'd1);
	check("the block reader's mux keeps the SPI side", dut.emu.blk_lw.en, 1'b0);
	release dut.emu.reset;
	sys_cycles(2);
	check("the Template's reset again: the request dropped", dut.emu.hps_io.sd_rd[TB_SLOT], 1'b0);
`endif

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
	$display("     %0d cycles, %0d with a transaction held, %0d with the enable set", n_gpi, n_gpi_busy, n_gpi_held);
	check("cycles where gp_in[25:22] differs from {1, the firewall's idle, 0, the enable}", n_gpi_bad, 0);
	check("cycles compared, with and without a transaction held, with and without the enable",
	      {n_gpi > 100, n_gpi_busy > 10, n_gpi_held > 10, n_gpi - n_gpi_held > 10}, 4'b1111);
`endif

	$display("%s: %0d checks, %0d failures", (fails != 0) ? "FAILED" : "PASSED", checks, fails);
	$finish;
end

initial begin #2000000 $display("FAILED: timeout"); $finish; end

endmodule

`default_nettype wire

`undef LW_TB_ANY_SECTOR
