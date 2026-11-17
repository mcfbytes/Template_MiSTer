//============================================================================
//  lw_tb.sv -- part of the LW bridge framework (docs/LW_BRIDGE.md)
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

// LW bridge framework testbench: hps_lw_bridge, lw_sys_window (directory, PAD, SECTOR, OSDBUF, SYS), hps_io's
// LW write path and both osd shadows, driven through a stand-in for the HPS LW bridge atom (lw_stub_atom.sv).
// The reference configuration is a CD-style core: a 588-word primary frame on hps_io slot 1 and two 1 KB memory
// cards on slots 2 and 3, each card image in DDR. Without MISTER_LW_SECTOR_SLOT the window has no SECTOR block and
// the consumers meet hps_io directly (the Template's own configuration); the directory, the block's addresses and
// the doorbell must then be silent.
// LW_TB_DEFGEOM runs the SECTOR block at the documented default geometry instead (a 128-word primary frame,
// 256-word save-slot frames): a save-slot frame longer than the primary's must complete.
// MISTER_LW_LANE adds the frame lane: every decoder then reads the link nets through lw_link, as sys_top wires
// them, and the lane cases replay frames over LW against the same frames sent as link words.
// Run: tb/run.sh (Icarus Verilog >= 12).

`timescale 1ns/1ps

module emu_ports_probe(
`include "emu_ports.vh"
);
endmodule

// One osd output, captured: bit (y, x) of the OSD as the pixel path draws it (din = 0, so dout[21] =
// OSD_COLOR[2] marks the OSD box and dout[23] is the pixel). `lines` is the box height of the last frame.
module osd_cap(input clk, input [23:0] d, input de, input vs);
reg        cap [0:32767];
integer    x = 0, y = 0, lines = -1, frames = 0, wbad = 0, wbad_l = 0;
reg        vs_d = 0, de_d = 0, has = 0;
always @(posedge clk) begin
	vs_d <= vs;
	de_d <= de;
	if (vs && !vs_d) begin
		lines  <= y;
		wbad_l <= wbad;
		frames <= frames + 1;
		y      <= 0;
		wbad   <= 0;
	end
	if (de && d[21] === 1'b1) begin
		if (y < 128 && x < 256) cap[y*256 + x] <= d[23];
		x   <= x + 1;
		has <= 1;
	end
	if (!de && de_d) begin
		if (has) begin
			y <= y + 1;
			if (x != 256) wbad <= wbad + 1;
		end
		x   <= 0;
		has <= 0;
	end
end
endmodule

// The checker bound to each lease link (lw_fence): its level rises only on an acknowledge of rest, and falls only
// after the acknowledge has read 0 since its rise; the acknowledge never reads 0 in IDLE (off: a case that
// scrambles or forces a link); no store strobe reaches the block outside OPEN. Sampled at every clk_100m negedge.
module fence_chk(input clk, input off, input [2:0] st, input lvl, input ack_s, input stb);
integer n_rise = 0, n_fall = 0, n_bad = 0, n_stb = 0;
reg     lvl_p = 0, ack_p = 0, seen0 = 0, init = 0;
task bad(input string what);
	begin
		n_bad = n_bad + 1;
		if (n_bad <= 6) $display("   lease link %m at %0t: %s", $time, what);
	end
endtask
always @(negedge clk) begin
	if (init && lvl && !lvl_p) begin
		n_rise = n_rise + 1;
		if (ack_p !== 1'b1) bad("the level rose while the acknowledge read 0");
		seen0 = 0;
	end
	if (init && !lvl && lvl_p) begin
		n_fall = n_fall + 1;
		if (!seen0) bad("the level fell with no 0 of the acknowledge since its rise");
	end
	if (lvl && ack_s === 1'b0) seen0 = 1;
	if (!off && st == 3'd1 && ack_s !== 1'b1) bad("the acknowledge read 0 in IDLE");
	if (stb === 1'b1) begin
		n_stb = n_stb + 1;
		if (st != 3'd3) bad("a store strobe reached the block outside OPEN");
	end
	lvl_p = lvl; ack_p = ack_s; init = 1;
end
endmodule

// A save-slot consumer (a memory card's usual HPS handshake): rd or wr with the LBA, both dropped on ack; the
// transfer ends when ack falls. The buffer (dpram port B) is written on write & ack at addr.
module mc_model(input clk, input reset, output reg rd = 0, output reg wr = 0, output reg [6:0] lba = 0,
                input ack, input write, input [8:0] addr, input [15:0] din, input [8:0] haddr, output reg [15:0] dout);
reg  [15:0] ram [0:511];
integer     strobes = 0, done = 0;
reg         busy = 0, seen = 0;
always @(posedge clk) begin
	dout <= ram[haddr];
	if (write && ack) begin
		ram[addr] <= din;
		strobes   <= strobes + 1;
	end
	if (ack) begin rd <= 0; wr <= 0; end
	if (reset) begin rd <= 0; wr <= 0; busy <= 0; seen <= 0; end
	else if (busy) begin
		if (ack) seen <= 1;
		else if (seen) begin busy <= 0; seen <= 0; done <= done + 1; end
	end
end
task go(input is_wr, input [6:0] l);
	begin @(negedge clk); lba = l; if (is_wr) wr = 1; else rd = 1; busy = 1; seen = 0; end
endtask
endmodule

module tb;

`ifdef LW_DIAG
localparam DIAG = `LW_DIAG;
`else
localparam DIAG = 0;
`endif
function [31:0] dg(input [31:0] v); dg = DIAG ? v : 32'd0; endfunction

reg clk = 0;            // clk_100m
reg clk_run = 1;        // 0 = clk_100m stopped (low)
always #5 clk = clk_run ? ~clk : 1'b0;

// clk_sys: half period LW_TB_SYS_HP_PS (default 33.8688 MHz), stretched by 50 ppm so its phase against clk_100m
// sweeps at every rate, 100 MHz included. Edges are placed from an accumulated real time, so the 1 ps
// precision does not round the offset away.
`ifndef LW_TB_SYS_HP_PS
`define LW_TB_SYS_HP_PS 14762
`endif
localparam real SYS_HP_NS = `LW_TB_SYS_HP_PS * 1.00005 / 1000.0;
localparam real SYS_MHZ   = 500.0 / SYS_HP_NS;
// A wait for clk_sys progress written in clk_100m cycles spans as many clk_sys cycles at a slower rate as at the
// default one: slow(n) stretches it by the ratio, and returns n unchanged at the default rate and above.
localparam real SYS_SLOW  = `LW_TB_SYS_HP_PS > 14762 ? `LW_TB_SYS_HP_PS / 14762.0 : 1.0;
function integer slow(input integer n); slow = SYS_SLOW > 1.0 ? $rtoi($itor(n) * SYS_SLOW + 0.999) : n; endfunction
reg      sys_run = 1;   // 0 = core held in reset with clk_sys stopped
reg      clk_sys = 0;
realtime sys_t;
initial begin
	sys_t = 0;
	forever begin
		sys_t = sys_t + SYS_HP_NS;
		#(sys_t - $realtime);
		if (sys_run) clk_sys = ~clk_sys;
	end
end

// AXI3 master side, read by the stub atom
reg [11:0] awid = 0, arid = 0;
reg [20:0] awaddr = 0, araddr = 0;
reg  [3:0] awlen = 0, arlen = 0;
reg  [2:0] awsize = 2, arsize = 2;
reg  [2:0] rsize = 2;   // ARSIZE used by axi_read
reg  [1:0] arburst = 2'b01, awburst = 2'b01;   // INCR unless a case sets another type
reg        awvalid = 0, wvalid = 0, wlast = 1, bready = 1, arvalid = 0, rready = 1;
reg [31:0] wdata = 0;
reg  [3:0] wstrb = 4'hF;
reg        hps_rst_n = 1;   // the HPS's fabric reset

wire [20:0] lw_raddr, lw_waddr;
wire        lw_write, lw_wburst;
wire [31:0] lw_rdata, lw_wdata;
wire  [3:0] lw_be;
wire [48:0] lw_pad;
wire  [2:0] lines;           // line ordinal k = lines[k-1]
wire        doorbell = lines[0];   // ordinal 1: SECTOR's entry
wire        sys_irq  = lines[1];   // ordinal 2: SYS's entry
wire [131:0] st_up;
wire [34:0] blk_up;
wire [18:0] blk_dn;
wire [19:0] card_up;
wire [12:0] card_dn;
wire        osd_we;
wire  [9:0] osd_addr;
wire [31:0] osd_data;
wire  [3:0] osd_be;
wire  [1:0] osd_ctrl;

wire        lw_hrst, fw_present, fw_idle;
wire  [1:0] lw_freed;

// GPO[31] as the HPS drives it (1: the link's I/O view), made a clk_100m level as sys_top makes it.
reg         gpo31 = 1;
// GPO[31] on clk_sys through two registers, as sys_top's gp_outd/gp_outr carry it: the lane player's view term
reg         gpo31_d = 1, gpo31_s = 1;
always @(posedge clk_sys) begin gpo31_d <= gpo31; gpo31_s <= gpo31_d; end
wire        lw_idv, lw_held;
lw_arst idv_c(.clk(clk), .rst_n(gpo31), .q(lw_idv));

hps_lw_bridge bridge
(
	.clk(clk),
	.hps_rst_n(hps_rst_n), .lw_hrst(lw_hrst), .lw_freed(lw_freed), .present(fw_present), .idle(fw_idle),
	.lw_raddr(lw_raddr), .lw_rdata(lw_rdata),
	.lw_waddr(lw_waddr), .lw_write(lw_write), .lw_wdata(lw_wdata), .lw_be(lw_be), .lw_wburst(lw_wburst)
);

// The register bus's store pulses, the dropped burst beats and the OSD image stores, counted.
integer n_lwwr = 0, n_lwwb = 0, n_osdwe = 0, n_padwe = 0;
always @(posedge clk_sys) if (lw_pad[48]) n_padwe = n_padwe + 1;
always @(posedge clk) begin
	if (lw_write)  n_lwwr  = n_lwwr + 1;
	if (lw_wburst) n_lwwb  = n_lwwb + 1;
	if (osd_we)    n_osdwe = n_osdwe + 1;
end

`ifdef MISTER_LW_SECTOR_SLOT
localparam TB_SEC = 1;
`else
localparam TB_SEC = 0;
`endif
`ifdef LW_TB_DEFGEOM
localparam TB_FRAME = 128;
`else
localparam TB_FRAME = 588;
`endif
`ifdef MISTER_LW_LANE
localparam TB_LANE = 1;
`else
localparam TB_LANE = 0;
`endif
// The lease unit: 2^TB_LU clk_100m cycles (the RTL's default is 25).
`ifdef LW_TB_LEASE_LOG2
localparam TB_LU = `LW_TB_LEASE_LOG2;
`else
localparam TB_LU = 12;
`endif

lw_sys_window #(
	.SEC_EN(TB_SEC), .SEC_SLOT(8'd1), .SEC_FRAME(TB_FRAME), .BLKSZ(3'd3),
	.MC_SLOT0(4'd2), .MC_WSRC0(32'h30100000), .MC_EXT0(32'h00020000),
	.MC_SLOT1(4'd3), .MC_WSRC1(32'h30200000), .MC_EXT1(32'h00020000), .MC_FRAME(10'd256),
	.LANE_EN(TB_LANE), .LEASE_UNIT_LOG2(TB_LU)
) win
(
	.clk(clk),
	.lw_raddr(lw_raddr), .lw_rdata(lw_rdata),
	.lw_waddr(lw_waddr), .lw_write(lw_write), .lw_wdata(lw_wdata), .lw_be(lw_be), .lw_wburst(lw_wburst),
	.lw_hrst(lw_hrst), .lw_freed(lw_freed), .lw_idv(lw_idv), .held(lw_held),
	.lines(lines),
	.clk_sys(clk_sys), .lw_pad(lw_pad), .blk_up(blk_up), .blk_dn(blk_dn), .card_up(card_up), .card_dn(card_dn),
	.st_up(st_up),
	.osd_we(osd_we), .osd_addr(osd_addr), .osd_data(osd_data), .osd_be(osd_be), .osd_ctrl(osd_ctrl),
`ifdef MISTER_LW_LANE
	.lane_busy(io_enable | io_fpga | io_osd | ~gpo31_s), .lane_wait(hps_bus[37] | vs_wait), .lane_dout(hps_bus[15:0]),
	.lane_own(lane_own), .lane_hold(lane_hold), .lane_sel(lane_sel), .lane_strobe(lane_strobe), .lane_din(lane_din)
`else
	.lane_busy(1'b0), .lane_wait(1'b0), .lane_dout(16'd0),
	.lane_own(), .lane_hold(), .lane_sel(), .lane_strobe(), .lane_din()
`endif
);

// The link nets every decoder reads: the tb's own (io_*), or with the lane sys_top's link front, lw_link.
`ifdef MISTER_LW_LANE
reg         io_fpga = 0;
reg         ioctl_wait = 0, vs_wait = 0;
wire        lane_own, lane_hold, lane_strobe;
wire  [3:0] lane_sel;
wire [15:0] lane_din;
wire  [3:0] m_sel;
wire        m_strobe, m_ack;
wire [15:0] m_din;
lw_link link
(
	.clk_sys(clk_sys), .l_sel({io_osd, io_osd, io_fpga, io_enable}), .l_clk(io_strobe), .l_din(io_din),
	.l_wait(hps_bus[37] | vs_wait),
	.p_own(lane_own), .p_hold(lane_hold), .p_sel(lane_sel), .p_strobe(lane_strobe), .p_din(lane_din),
	.io_sel(m_sel), .io_strobe(m_strobe), .io_din(m_din), .io_ack(m_ack)
);
`else
wire  [3:0] m_sel = {io_osd, io_osd, 1'b0, io_enable};
wire        m_strobe = io_strobe;
wire [15:0] m_din = io_din;
`endif

// Both osd instances as sys_top wires them: one SPI select (the HPS drives ss0 and ss2 together), one LW store bus.
reg clk_vid = 0;
always #3.25 clk_vid = ~clk_vid;
reg   [8:0] vh = 0, vv = 0;           // 272x140 active in 288x148: ce_pix every clock, OSD 256 wide
always @(posedge clk_vid) begin
	vh <= (vh == 287) ? 9'd0 : vh + 9'd1;
	if (vh == 287) vv <= (vv == 147) ? 9'd0 : vv + 9'd1;
end
wire        v_de = vh < 272 && vv < 140;
wire        v_hs = vh >= 276 && vh < 282;
wire        v_vs = vv >= 142 && vv < 144;
reg         io_osd = 0;
wire [23:0] ho_d, vo_d;
wire        ho_de, ho_vs, ho_hs, vo_de, vo_vs, vo_hs, osd_status;

osd hdmi_osd
(
	.clk_sys(clk_sys), .io_osd(m_sel[2]), .io_strobe(m_strobe), .io_din(m_din),
	.clk_video(clk_vid), .din(24'd0), .de_in(v_de), .vs_in(v_vs), .hs_in(v_hs),
	.dout(ho_d), .de_out(ho_de), .vs_out(ho_vs), .hs_out(ho_hs),
	.clk_lw(clk), .lw_we(osd_we), .lw_addr(osd_addr), .lw_data(osd_data), .lw_be(osd_be), .lw_ctrl(osd_ctrl)
);

osd vga_osd
(
	.clk_sys(clk_sys), .io_osd(m_sel[3]), .io_strobe(m_strobe), .io_din(m_din), .osd_status(osd_status),
	.clk_video(clk_vid), .din(24'd0), .de_in(v_de), .vs_in(v_vs), .hs_in(v_hs),
	.dout(vo_d), .de_out(vo_de), .vs_out(vo_vs), .hs_out(vo_hs),
	.clk_lw(clk), .lw_we(osd_we), .lw_addr(osd_addr), .lw_data(osd_data), .lw_be(osd_be), .lw_ctrl(osd_ctrl)
);

osd_cap cap_h(.clk(clk_vid), .d(ho_d), .de(ho_de), .vs(ho_vs));
osd_cap cap_v(.clk(clk_vid), .d(vo_d), .de(vo_de), .vs(vo_vs));

wire awready = bridge.hps2fpga_light_weight.awready;
wire wready  = bridge.hps2fpga_light_weight.wready;
wire bvalid  = bridge.hps2fpga_light_weight.bvalid;
wire arready = bridge.hps2fpga_light_weight.arready;
wire rvalid  = bridge.hps2fpga_light_weight.rvalid;
wire rlast   = bridge.hps2fpga_light_weight.rlast;
wire [31:0] rdata = bridge.hps2fpga_light_weight.rdata;
wire [11:0] rid   = bridge.hps2fpga_light_weight.rid;
wire [11:0] bid   = bridge.hps2fpga_light_weight.bid;

// The firewall's HPS-reset rules as a reference model, built from the AXI handshakes and lw_hrst alone, and
// compared with the RTL in every cycle of the run: which state machine holds a transaction, each one's mark, the
// machines freed at a release, and idle. fm_mk_r/fm_mk_w: the marks the last rise of lw_hrst set; fm_st_r/fm_st_w:
// the state each machine was in, in that rise's cycle.
reg      fm_rout = 0, fm_wout = 0, fm_rmark = 0, fm_wmark = 0, fm_hq = 1, fm_rfreed = 0, fm_wfreed = 0;
reg      fm_mk_r = 0, fm_mk_w = 0;
integer  fm_st_r = -1, fm_st_w = -1, fm_cycles = 0, fm_bad = 0, fw_glitch = 0, n_fr = 0, n_fw = 0, fw_absent = 0;
realtime fw_tpos = 0;
always @(posedge clk) begin : fw_model
	reg ar_hs, r_hs, r_end, aw_hs, b_hs, wb_hs, rise, fall, ra, wa, rf, wf;
	fw_tpos = $realtime;
	ar_hs = arvalid & arready; r_hs = rvalid & rready; r_end = r_hs & rlast;
	aw_hs = awvalid & awready; b_hs = bvalid & bready; wb_hs = (wvalid & wready) | b_hs;
	rise = lw_hrst & ~fm_hq; fall = ~lw_hrst & fm_hq;
	ra = (fm_rout & ~r_end) | ar_hs;           // holds a transaction after this cycle
	wa = (fm_wout & ~b_hs) | aw_hs;
	rf = fall & fm_rmark & ~r_hs;
	wf = fall & fm_wmark & ~wb_hs;
	if (rise) begin
		fm_rmark <= ra; fm_wmark <= wa; fm_mk_r <= ra; fm_mk_w <= wa;
		fm_st_r <= bridge.adapter.rstate; fm_st_w <= bridge.adapter.wstate;
	end
	else begin
		if (fall || r_hs || !fm_rout) fm_rmark <= 1'b0;
		if (fall || wb_hs || !fm_wout) fm_wmark <= 1'b0;
	end
	fm_rout <= ra & ~rf; fm_wout <= wa & ~wf;
	fm_rfreed <= rf; fm_wfreed <= wf;
	fm_hq <= lw_hrst;
	if (lw_freed[0]) n_fr = n_fr + 1;
	if (lw_freed[1]) n_fw = n_fw + 1;
end
always @(negedge clk) begin
	fm_cycles = fm_cycles + 1;
	if (fw_present !== 1'b1) fw_absent = fw_absent + 1;
	if (fw_idle !== !(fm_rout || fm_wout) || lw_freed !== {fm_wfreed, fm_rfreed} ||
	    bridge.adapter.rmark !== fm_rmark || bridge.adapter.wmark !== fm_wmark ||
	    (bridge.adapter.rstate != 2'd0) !== fm_rout || (bridge.adapter.wstate != 2'd0) !== fm_wout) begin
		fm_bad = fm_bad + 1;
		if (fm_bad <= 8)
			$display("   firewall model at %0t: idle %b freed %b marks %b%b states %0d %0d; model %b %b%b %b%b %b%b", $time,
			         fw_idle, lw_freed, bridge.adapter.rmark, bridge.adapter.wmark, bridge.adapter.rstate,
			         bridge.adapter.wstate, !(fm_rout || fm_wout), fm_wfreed, fm_rfreed, fm_rmark, fm_wmark, fm_rout, fm_wout);
	end
end
// idle is a register: it changes only at a clk_100m edge
always @(fw_idle) if ($realtime > 0 && $realtime != fw_tpos) fw_glitch = fw_glitch + 1;

// GPI as sys_top builds it, for the host models: the core magic in the ID view; in the I/O view bit 31 clear, bit
// 25 the firewall's present, 24 its idle, 22 the window's held. Bit 22 is compared with live_q in every cycle.
wire [31:0] tb_gpi = !gpo31 ? 32'h5CA623A4 : {6'd0, fw_present, fw_idle, 1'b0, lw_held, 22'd0};
integer gpi22_n = 0, gpi22_bad = 0, gpi22_on = 0;
always @(negedge clk) begin
	gpi22_n = gpi22_n + 1;
	if (gpo31 && tb_gpi[22] !== win.live_q) gpi22_bad = gpi22_bad + 1;
	if (gpo31 && tb_gpi[22] === 1'b1) gpi22_on = gpi22_on + 1;
end

// Each response handshake: the count, and the last beat's ID, data and RLAST; fr_log keeps the read beats.
integer     fr_n = 0, fb_n = 0;
reg  [11:0] fr_id = 0, fb_id = 0;
reg  [31:0] fr_d = 0;
reg         fr_last = 0;
reg  [31:0] fr_log [0:31];
reg  [11:0] fr_idlog [0:31];
reg         fr_lastlog [0:31];
reg  [20:0] fw_wa = 0;
reg  [31:0] fw_wd = 0;
always @(posedge clk) begin
	if (rvalid && rready) begin
		fr_log[fr_n % 32] = rdata; fr_idlog[fr_n % 32] = rid; fr_lastlog[fr_n % 32] = rlast;
		fr_n = fr_n + 1; fr_id = rid; fr_d = rdata; fr_last = rlast;
	end
	if (bvalid && bready) begin fb_n = fb_n + 1; fb_id = bid; end
	if (lw_write) begin fw_wa = lw_waddr; fw_wd = lw_wdata; end
end

// HPS_BUS as sys_top wires it: {st_up, card_up/dn, blk_up, blk_dn, lw_pad, ...}; the SPI side is driven by the tb (io_*)
reg         io_enable = 0, io_strobe = 0;
reg  [15:0] io_din = 0;
wire [313:0] hps_bus;
assign st_up = hps_bus[313:182];
assign card_up = {hps_bus[181:172], hps_bus[169:160]};
assign {hps_bus[171:170], hps_bus[159:158], hps_bus[157:149]} = {card_dn[12:11], card_dn[10:9], card_dn[8:0]};
assign blk_up = hps_bus[148:114];
assign hps_bus[113:95] = blk_dn;
assign hps_bus[94:46] = lw_pad;
assign hps_bus[45:38] = 0;
assign hps_bus[35:33] = {m_sel[1], m_sel[0], m_strobe};
assign hps_bus[31:16] = m_din;

localparam CONF_STR = {"CORE;", `SYS_LW_CAPS "SS3E000000:400000;", "V,v1;"};

wire [31:0] joy0, joy1, joy2, joy3;
wire [15:0] ana0, ana3, rana0, rana3;
wire  [7:0] pdl1;
reg         cd_reset = 0;
reg         cd_req = 0;
reg  [31:0] cd_lba = 32'hFFFFFFFF;
wire        cd_ack, cd_wr;
wire [15:0] cd_data;
wire  [3:0] sd_rd, sd_wr, sd_ack;
wire [15:0] sd_buff_dout;
wire        sd_buff_wr;
wire  [8:0] sd_buff_addr;
wire [31:0] sd_lba [4];
wire [15:0] sd_buff_din [4];
assign sd_rd[0] = 0;
assign sd_wr[1:0] = 0;
assign sd_lba[0] = 0;
assign sd_lba[1] = cd_lba;
assign sd_lba[2] = {25'd0, mc1.lba};
assign sd_lba[3] = {25'd0, mc2.lba};
assign sd_buff_din[0] = 0;
assign sd_buff_din[1] = 0;
assign sd_buff_din[2] = mc1.dout;
assign sd_buff_din[3] = mc2.dout;

// The core's status request: a rising edge of status_set latches status_in.
reg [127:0] st_in = 0;
reg         st_set = 0;

`ifdef MISTER_LW_LANE
// The hps_io registers the lane cases compare (gamma_bus[21]: the core has gamma).
wire [127:0] hst;
wire  [64:0] rtc;
wire  [32:0] tstamp;
wire   [3:0] img_m;
wire         img_ro;
wire  [63:0] img_sz;
wire  [21:0] gbus;
wire  [15:0] sdsz;
wire   [7:0] umode;
wire  [31:0] uspeed;
assign gbus[21] = 1'b1;
// no hps_ext: EXT_BUS[32] low, so HPS_BUS[15:0] (the reply word the lane captures) is hps_io's own io_dout
wire  [35:0] ebus;
assign ebus[32] = 1'b0;
assign ebus[15:0] = 16'd0;
`endif

hps_io #(.CONF_STR(CONF_STR), .WIDE(1), .VDNUM(4), .BLKSZ(3)) hps_io
(
	.clk_sys(clk_sys),
	.HPS_BUS(hps_bus),
`ifdef MISTER_LW_LANE
	.ioctl_wait(ioctl_wait), .status(hst), .RTC(rtc), .TIMESTAMP(tstamp), .img_mounted(img_m), .img_readonly(img_ro),
	.img_size(img_sz), .gamma_bus(gbus), .sdram_sz(sdsz), .uart_mode(umode), .uart_speed(uspeed), .EXT_BUS(ebus),
`endif
	.status_in(st_in), .status_set(st_set),
	.joystick_0(joy0), .joystick_1(joy1), .joystick_2(joy2), .joystick_3(joy3),
	.joystick_l_analog_0(ana0), .joystick_l_analog_3(ana3),
	.joystick_r_analog_0(rana0), .joystick_r_analog_3(rana3), .paddle_1(pdl1),
	.sd_lba(sd_lba), .sd_rd(sd_rd), .sd_wr(sd_wr), .sd_ack(sd_ack),
	.sd_buff_dout(sd_buff_dout), .sd_buff_din(sd_buff_din), .sd_buff_wr(sd_buff_wr), .sd_buff_addr(sd_buff_addr)
);

// Two memory cards on hps_io slots 2 and 3, each behind an lw_card_mux when the SECTOR block is built.
wire        mc1_ack, mc1_write, mc2_ack, mc2_write;
wire  [8:0] mc1_addr, mc2_addr;
wire [15:0] mc1_din, mc2_din;
mc_model mc1(.clk(clk_sys), .reset(cd_reset), .ack(mc1_ack), .write(mc1_write), .addr(mc1_addr), .din(mc1_din),
             .haddr(sd_buff_addr));
mc_model mc2(.clk(clk_sys), .reset(cd_reset), .ack(mc2_ack), .write(mc2_write), .addr(mc2_addr), .din(mc2_din),
             .haddr(sd_buff_addr));
`ifdef MISTER_LW_SECTOR_SLOT
lw_card_mux #(.CARD(0)) mc1_lw
(
	.HPS_BUS(hps_bus),
	.core_rd(mc1.rd), .core_wr(mc1.wr), .core_lba(mc1.lba), .core_ack(mc1_ack), .core_write(mc1_write),
	.core_addr(mc1_addr), .core_data(mc1_din),
	.hps_rd(sd_rd[2]), .hps_wr(sd_wr[2]), .hps_ack(sd_ack[2]), .hps_write(sd_buff_wr), .hps_addr(sd_buff_addr),
	.hps_data(sd_buff_dout)
);
lw_card_mux #(.CARD(1)) mc2_lw
(
	.HPS_BUS(hps_bus),
	.core_rd(mc2.rd), .core_wr(mc2.wr), .core_lba(mc2.lba), .core_ack(mc2_ack), .core_write(mc2_write),
	.core_addr(mc2_addr), .core_data(mc2_din),
	.hps_rd(sd_rd[3]), .hps_wr(sd_wr[3]), .hps_ack(sd_ack[3]), .hps_write(sd_buff_wr), .hps_addr(sd_buff_addr),
	.hps_data(sd_buff_dout)
);

// The CD on hps_io slot 1, behind an lw_blk_mux when the SECTOR block is built.
lw_blk_mux cd_lw
(
	.HPS_BUS(hps_bus), .reset(cd_reset),
	.core_req(cd_req), .core_lba(cd_lba), .core_ack(cd_ack), .core_wr(cd_wr), .core_data(cd_data),
	.hps_req(sd_rd[1]), .hps_ack(sd_ack[1]), .hps_wr(sd_buff_wr), .hps_data(sd_buff_dout)
);
`else
// No SECTOR block, so no mux: each consumer meets hps_io directly, as in a core that declares none.
assign sd_rd[2] = mc1.rd, sd_wr[2] = mc1.wr, mc1_ack = sd_ack[2], mc1_write = sd_buff_wr, mc1_addr = sd_buff_addr;
assign mc1_din  = sd_buff_dout;
assign sd_rd[3] = mc2.rd, sd_wr[3] = mc2.wr, mc2_ack = sd_ack[3], mc2_write = sd_buff_wr, mc2_addr = sd_buff_addr;
assign mc2_din  = sd_buff_dout;
assign sd_rd[1] = cd_req, cd_ack = sd_ack[1], cd_wr = sd_buff_wr, cd_data = sd_buff_dout;
`endif

// A CD fetch FSM: req with the LBA, drop req on ack, then exactly 2 x TB_FRAME strobes, low half then high
// half, into one staging buffer. It consumes ANY strobe.
reg   [1:0] cst = 0;
reg   [9:0] fcnt = 0;
reg  [15:0] clo = 0;
reg  [31:0] cbuf [0:587];
integer     cd_frames = 0, cd_strobes = 0, cd_go = 0;
reg  [31:0] cd_next = 0;

always @(posedge clk_sys) begin
	if (cd_wr) cd_strobes <= cd_strobes + 1;
	if (cd_reset) begin
		cd_req <= 0;
		cst    <= 0;
	end
	else case (cst)
		0: if (cd_go && !cd_ack) begin
			cd_go  <= 0;
			cd_req <= 1;
			cd_lba <= cd_next;
			cst    <= 1;
		end
		1: if (cd_ack) begin
			cd_req <= 0;
			fcnt   <= 0;
			cst    <= 2;
		end
		2: if (cd_wr) begin
			clo <= cd_data;
			cst <= 3;
		end
		3: if (cd_wr) begin
			cbuf[fcnt] <= {cd_data, clo};
			if (fcnt == TB_FRAME - 1) begin
				cst       <= 0;
				cd_frames <= cd_frames + 1;
			end
			else begin
				fcnt <= fcnt + 10'd1;
				cst  <= 2;
			end
		end
	endcase
end

// The memory card on hps_io slot 2, which gates the shared strobe with its own ack.
integer mc_words = 0, mc_sum = 0;
always @(posedge clk_sys) if (sd_buff_wr && sd_ack[2]) begin
	mc_words <= mc_words + 1;
	mc_sum   <= mc_sum + sd_buff_dout;
end

// Reads have no side effect: the state a read could move, by hierarchical reference. With clk_sys stopped nothing
// else moves it, so it must be identical before and after any reads.
function [511:0] pure_win;
	pure_win = {255'd0, win.scratch, win.adrop, win.octrl, win.ohr, win.dirty, win.awp, win.arp, win.ds, win.psel, win.rr,
	           doorbell, sys_irq, win.sys_status.flg, win.sys_status.seq, win.sys_status.snap, win.sys_status.cause};
endfunction
function [511:0] pure_sec;
`ifdef MISTER_LW_SECTOR_SLOT
	pure_sec = {206'd0, win.sec.sector.pending, win.sec.sector.gen, win.sec.sector.ans, win.sec.sector.btag,
	           win.sec.sector.lba, win.sec.sector.xlba, win.sec.sector.wptr, win.sec.sector.exp_w,
	           win.sec.sector.flen_w, win.sec.sector.fdrops, win.sec.sector.bfail, win.sec.sector.en_lwx,
	           win.sec.sector.xen_lwx, win.sec.sector.test_lwx, win.sec.sector.frames, win.sec.sector.drops,
	           win.sec.sector.hw, win.sec.sector.n_rej, win.sec.sector.n_skip, win.sec.sector.n_abort,
	           win.sec.sector.n_rfill, win.sec.sector.sum, win.sec.sector.tsreq, win.sec.sector.tsdone,
	           win.sec.sector.dseen, win.sec.sector.sb2};
`else
	pure_sec = 0;
`endif
endfunction
function [255:0] pure_lane;
`ifdef MISTER_LW_LANE
	pure_lane = {146'd0, win.lane.lane.wspec, win.lane.lane.wcom, win.lane.lane.wvis, win.lane.lane.open, win.lane.lane.ovf,
	            win.lane.lane.played, win.lane.lane.drops, win.lane.lane.rrp, win.lane.lane.rhead,
	            win.lane.lane.rdrops, win.lane.lane.pt_d, win.lane.lane.rdt_d};
`else
	pure_lane = 0;
`endif
endfunction

// The readable view: every word of the directory and the blocks (0x40000-0x47FFC), then one word per KB of the
// rest of the window. SECTOR's CYCLES counts clk_100m cycles and is left out.
localparam PURE_N = 8192 + 480;
reg [31:0] pure_v [0:PURE_N-1];
function [20:0] pure_addr(input integer i);
	integer j;
	begin
		j = i - 8192;
		pure_addr = (i < 8192) ? 21'h40000 + {i[18:0], 2'b00} : 21'h48000 + {j[10:0], 10'd0};
	end
endfunction
task pure_view(input do_cmp, output integer nbad);
	integer m, nb;
	reg [31:0] v, vl;
	begin
		nbad = 0;
		for (m = 0; m < PURE_N; m = m + 1) begin
			axi_read(pure_addr(m), 0, v, vl, nb);
			if (pure_addr(m) == 21'h41030 || pure_addr(m) == LEASE) v = 0;
			if (do_cmp && v !== pure_v[m]) begin
				if (nbad == 0) $display("     view: %06h was %08h, now %08h", pure_addr(m), pure_v[m], v);
				nbad = nbad + 1;
			end
			pure_v[m] = v;
		end
	end
endtask

// The clk_100m edge of SECTOR's last cause raise.
realtime t_fire = 0;
`ifdef MISTER_LW_SECTOR_SLOT
always @(posedge clk) if (win.sec.sector.fire) t_fire = $realtime;
`endif

// SECTOR's DIAG SUM, sampled on every clk_100m edge: it may change only on an edge where FRAMES changes too, so
// a host that sees a frame counted reads that frame's sum.
integer sum_moves = 0, sum_alone = 0;
`ifdef MISTER_LW_SECTOR_SLOT
reg [19:0] sum_p = 0;
reg [15:0] frames_p = 0;
always @(posedge clk) begin
	if (win.sec.sector.sum !== sum_p) begin
		sum_moves = sum_moves + 1;
		if (win.sec.sector.frames === frames_p) sum_alone = sum_alone + 1;
	end
	sum_p    <= win.sec.sector.sum;
	frames_p <= win.sec.sector.frames;
end
`endif

integer fails = 0, checks = 0, lchecks = 0;
// The read latency every read is checked against is the firewall's own; the write latency is the first one seen.
integer wlat_ref = -1, rlat_ref;
initial rlat_ref = bridge.adapter.RD_LAT;

// The crossing cells alone (lw_cdc.sv): a 32-bit lw_xfer each way between clk_100m and clk_sys, and a Gray count
// each way through lw_sync. Each receiver sorts every word: the next one its sender launched, a repeat of the
// last one, or neither.
reg         cx_go_a = 0, cx_go_b = 0;
reg  [31:0] cx_d_a = 0, cx_d_b = 0;
wire        cx_busy_a, cx_busy_b, cx_v_a, cx_v_b;
wire [31:0] cx_q_a, cx_q_b;
lw_xfer #(.W(32)) cx_a(.s_clk(clk), .s_go(cx_go_a), .s_d(cx_d_a), .s_busy(cx_busy_a),
                       .d_clk(clk_sys), .d_q(cx_q_a), .d_v(cx_v_a));
lw_xfer #(.W(32)) cx_b(.s_clk(clk_sys), .s_go(cx_go_b), .s_d(cx_d_b), .s_busy(cx_busy_b),
                       .d_clk(clk), .d_q(cx_q_b), .d_v(cx_v_b));
function [31:0] cx_pat(input integer k); cx_pat = k[0] ? ~(32'h01010101 * k) : 32'h01010101 * k; endfunction
integer     cx_ka = 0, cx_kb = 0;                         // words launched
integer     cx_na = 0, cx_nb = 0, cx_ra = 0, cx_rb = 0, cx_ea = 0, cx_eb = 0;   // next words, repeats, neither
always @(posedge clk_sys) if (cx_v_a) begin
	if (cx_q_a === cx_pat(cx_na + 1)) cx_na <= cx_na + 1;
	else if (cx_q_a === cx_pat(cx_na)) cx_ra <= cx_ra + 1;
	else cx_ea <= cx_ea + 1;
end
always @(posedge clk) if (cx_v_b) begin
	if (cx_q_b === cx_pat(cx_nb + 1)) cx_nb <= cx_nb + 1;
	else if (cx_q_b === cx_pat(cx_nb)) cx_rb <= cx_rb + 1;
	else cx_eb <= cx_eb + 1;
end
reg         cg_run = 0;
reg  [11:0] cg_a = 0, cg_b = 0, cg_ma = 0, cg_mb = 0;     // source counts; the last value each receiver decoded
reg  [11:0] cg_a_lwgray = 0, cg_b_lwgray = 0;
wire [11:0] cg_sa, cg_sb;
integer     cg_ea = 0, cg_eb = 0;                         // decoded values the source never held, or out of order
always @(posedge clk) begin
	if (cg_run) cg_a <= cg_a + 12'd1;
	cg_a_lwgray <= cg_a ^ (cg_a >> 1);
end
always @(posedge clk_sys) begin
	if (cg_run) cg_b <= cg_b + 12'd1;
	cg_b_lwgray <= cg_b ^ (cg_b >> 1);
end
lw_sync #(.W(12), .GRAY(1)) cg_xa(.clk(clk_sys), .d(cg_a_lwgray), .q(cg_sa));
lw_sync #(.W(12), .GRAY(1)) cg_xb(.clk(clk), .d(cg_b_lwgray), .q(cg_sb));
function [11:0] cg_bin(input [11:0] g);
	integer n;
	begin cg_bin[11] = g[11]; for (n = 10; n >= 0; n = n - 1) cg_bin[n] = cg_bin[n + 1] ^ g[n]; end
endfunction
always @(posedge clk_sys) begin
	if (cg_bin(cg_sa) - cg_ma > cg_a - cg_ma) cg_ea <= cg_ea + 1;
	cg_ma <= cg_bin(cg_sa);
end
always @(posedge clk) begin
	if (cg_bin(cg_sb) - cg_mb > cg_b - cg_mb) cg_eb <= cg_eb + 1;
	cg_mb <= cg_bin(cg_sb);
end

// n words each way, each launched as soon as its sender is not busy
task cx_send(input integer n);
	fork
		repeat (n) begin
			@(negedge clk); while (cx_busy_a) @(negedge clk);
			cx_ka = cx_ka + 1; cx_d_a = cx_pat(cx_ka); cx_go_a = 1;
			@(negedge clk) cx_go_a = 0;
		end
		repeat (n) begin
			@(negedge clk_sys); while (cx_busy_b) @(negedge clk_sys);
			cx_kb = cx_kb + 1; cx_d_b = cx_pat(cx_kb); cx_go_b = 1;
			@(negedge clk_sys) cx_go_b = 0;
		end
	join
	repeat (slow(20)) @(negedge clk);
endtask

// The cells: every word crosses whole and in order. From each of the 64 states of a handshake's toggles and
// synchroniser stages, each way, it is idle within a bound, delivers nothing but repeats of its held word (at
// most three: one per edge its forward chain can hold), and the words launched after it cross whole and in
// order. Every Gray code a receiver samples is one its source held, in order.
// The settle after idle for a scrambled state's repeats: ten cycles, twenty under the delay models, whose extra
// destination cycle per edge can hold a repeat past ten at the default rate.
`ifdef LW_TB_CDC
localparam integer CX_SETTLE = 20;
`else
localparam integer CX_SETTLE = 10;
`endif
task cdc_cells;
	integer st, spur, bad, idle, wait_n, r0;
	begin
		$display("-- CDC cells: lw_xfer words cross whole and in order, each way");
		cx_send(200);
		check("xfer to clk_sys: words received", cx_na, cx_ka);
		check("xfer to clk_sys: repeated or other words", cx_ra + cx_ea, 0);
		check("xfer to clk_100m: words received", cx_nb, cx_kb);
		check("xfer to clk_100m: repeated or other words", cx_rb + cx_eb, 0);

		$display("-- CDC cells: lw_xfer from every state of its toggles converges");
		bad = 0; spur = 0; idle = 0;
		for (st = 0; st < 64; st = st + 1) begin
			@(negedge clk);
			{cx_a.lwtg_r, cx_a.lwtg_a, cx_a.sync_r.lws0_q, cx_a.sync_r.lws1_q, cx_a.sync_a.lws0_q, cx_a.sync_a.lws1_q} = st;
			{cx_b.lwtg_r, cx_b.lwtg_a, cx_b.sync_r.lws0_q, cx_b.sync_r.lws1_q, cx_b.sync_a.lws0_q, cx_b.sync_a.lws1_q} = st;
			wait_n = 0;
			repeat (slow(4)) @(negedge clk);
			while ((cx_busy_a || cx_busy_b) && wait_n < slow(40)) begin @(negedge clk); wait_n = wait_n + 1; end
			if (cx_busy_a || cx_busy_b) idle = idle + 1;
			repeat (slow(CX_SETTLE)) @(negedge clk);
			if (cx_ra > 3 || cx_rb > 3 || cx_na != cx_ka || cx_nb != cx_kb) spur = spur + 1;
			r0 = cx_ra + cx_rb;
			cx_send(3);
			if (cx_na != cx_ka || cx_nb != cx_kb || cx_ea + cx_eb != 0 || cx_ra + cx_rb != r0) begin
				bad = bad + 1;
				$display("   xfer state %0d: sent %0d/%0d, received %0d/%0d, other %0d/%0d, repeats %0d -> %0d", st, cx_ka,
				         cx_kb, cx_na, cx_nb, cx_ea, cx_eb, r0, cx_ra + cx_rb);
			end
			cx_ra = 0; cx_rb = 0;
		end
		check("xfer: states not idle within 44 clk_100m cycles (scaled below the default rate)", idle, 0);
		check("xfer: states that delivered more than three repeats, or a word not launched", spur, 0);
		check("xfer: states after which three words did not cross whole and in order", bad, 0);

		$display("-- CDC cells: lw_sync Gray counts, each way");
		cg_run = 1;
		repeat (3000) @(negedge clk);
		cg_run = 0;
		repeat (slow(20)) @(negedge clk);
		check("Gray to clk_sys: codes out of order or never held", cg_ea, 0);
		check("Gray to clk_100m: codes out of order or never held", cg_eb, 0);
		check("Gray to clk_sys: the last count arrives", cg_ma, cg_a);
		check("Gray to clk_100m: the last count arrives", cg_mb, cg_b);
	end
endtask

task check(input string what, input [31:0] got, input [31:0] exp);
	checks = checks + 1;
	if (got !== exp) begin
		fails = fails + 1;
		$display("FAIL %s: got %08h exp %08h", what, got, exp);
	end
	else $display("pass %s = %08h", what, got);
endtask

task lat_check(input string what, input integer lat, inout integer ref_lat);
	checks = checks + 1;
	lchecks = lchecks + 1;
	if (ref_lat < 0) ref_lat = lat;
	if (lat !== ref_lat) begin
		fails = fails + 1;
		$display("FAIL %s latency %0d, expected fixed %0d", what, lat, ref_lat);
	end
endtask

// ---- The lease, and the bench's own host ----
// LEASE store: {key, id[15:6], verb[5:4], ttl[3:0]}; verbs 1 CLAIM, 2 RENEW, 3 RELEASE.
localparam [20:0] SYSB = 21'h43000;               // the SYS block
localparam [20:0] LEASE = 21'h40018, OWNEDA = 21'h40024, OPENA = 21'h4002C, LINEN = 21'h40030, RESETSA = 21'h40034;
function [31:0] lease_w(input [9:0] id, input [1:0] verb, input [3:0] ttl); lease_w = {16'h4C57, id, verb, ttl}; endfunction
// The bench's host claims the lease with its id before its first store, enables every line, and keeps it: when a
// quarter of its TTL has passed, the next access it makes is preceded by a RENEW and a read of LEASE (held, its
// own id, the epoch of its claim). A lapse while it keeps the lease fails the arm.
localparam [9:0] KP_ID  = 10'h2A7;
localparam [3:0] KP_TTL = 4'd15;
localparam real  KP_NS  = 10.0 * (2.0 ** TB_LU) * 15.0 / 4.0;
reg      kp_on = 0, kp_in = 0, kp_live_d = 0;
reg [7:0] kp_epoch = 0;
realtime kp_t = 0;
integer  kp_n = 0, kp_lost = 0, kp_lapse = 0;
always @(posedge clk) begin
	if (kp_on && kp_live_d && !win.live_q) begin
		kp_lapse = kp_lapse + 1;
		$display("   the bench's lease ended at %0t while its keeper held it (end %0d)", $time, win.lend);
	end
	kp_live_d <= win.live_q;
end

// One store and one read with no keeper step: the keeper's own accesses.
task raw_write(input [20:0] a, input [31:0] d);
	integer n;
	reg aw_hs, w_hs;
	begin
		@(negedge clk);
		awaddr = a; awlen = 0; awsize = 2; awvalid = 1;
		wdata = d; wstrb = 4'hF; wlast = 1; wvalid = 1;
		n = 0;
		while (!bvalid && n <= 64) begin
			aw_hs = awvalid & awready; w_hs = wvalid & wready;
			@(negedge clk); n = n + 1;
			if (aw_hs) awvalid = 0;
			if (w_hs) wvalid = 0;
		end
		awvalid = 0; wvalid = 0;
		if (!bvalid) begin $display("FAIL write %06h: no BVALID", a); fails = fails + 1; end
		lat_check("BVALID", n, wlat_ref);
		@(negedge clk);
	end
endtask
task raw_read(input [20:0] a, output [31:0] d);
	integer n;
	reg ar_hs;
	begin
		@(negedge clk);
		araddr = a; arlen = 0; arsize = 2; arvalid = 1;
		n = 0;
		while (!rvalid && n <= 64) begin
			ar_hs = arvalid & arready;
			@(negedge clk); n = n + 1;
			if (ar_hs) arvalid = 0;
		end
		arvalid = 0;
		if (!rvalid) begin $display("FAIL read %06h: no RVALID", a); fails = fails + 1; end
		lat_check("RVALID", n, rlat_ref);
		d = rdata;
		@(negedge clk);
	end
endtask
task kp_renew;
	reg [31:0] v;
	begin
		kp_in = 1;
		raw_write(LEASE, lease_w(KP_ID, 2'd2, KP_TTL));
		raw_read(LEASE, v);
		if (!(v[31] && v[9:0] == KP_ID && v[23:16] == kp_epoch)) begin
			kp_lost = kp_lost + 1;
			$display("   the bench's RENEW at %0t found LEASE %08h", $time, v);
		end
		kp_t = $realtime; kp_n = kp_n + 1;
		kp_in = 0;
	end
endtask
task kp_tick; if (kp_on && !kp_in && $realtime - kp_t > KP_NS) kp_renew; endtask
// The keeper also renews while the bench waits with the bus idle: the bus tasks hold bus_busy and wait for it.
reg bus_busy = 0;
task bus_get; begin while (kp_in) @(negedge clk); bus_busy = 1; end endtask
initial forever begin
	@(negedge clk); #0.001;
	if (kp_on && !kp_in && !bus_busy && $realtime - kp_t > KP_NS) kp_renew;
end

// Drive at negedge, sample at negedge: cycles from VALID asserted to BVALID/RVALID seen.
task axi_write(input [20:0] a, input [31:0] d, input [3:0] s);
	integer n;
	reg aw_hs, w_hs;
	begin
		bus_get;
		kp_tick;
		@(negedge clk);
		awaddr = a; awlen = 0; awsize = 2; awvalid = 1;
		wdata = d; wstrb = s; wlast = 1; wvalid = 1;
		n = 0;
		while (!bvalid && n <= 64) begin
			aw_hs = awvalid & awready; w_hs = wvalid & wready;
			@(negedge clk); n = n + 1;
			if (aw_hs) awvalid = 0;
			if (w_hs) wvalid = 0;
		end
		awvalid = 0; wvalid = 0;
		if (!bvalid) begin $display("FAIL write %06h: no BVALID", a); fails = fails + 1; end
		lat_check("BVALID", n, wlat_ref);
		@(negedge clk);
		bus_busy = 0;
	end
endtask

// Every beat of the last read: its data and its ID (ARBURST is the arburst register's).
reg [31:0] rb_d  [0:15];
reg [11:0] rb_id [0:15];

task axi_read(input [20:0] a, input [3:0] len, output [31:0] d0, output [31:0] dl, output integer beats);
	integer n;
	reg ar_hs, done;
	begin
		bus_get;
		kp_tick;
		@(negedge clk);
		araddr = a; arlen = len; arsize = rsize; arvalid = 1;
		n = 0; beats = 0;
		while (!rvalid && n <= 64) begin
			ar_hs = arvalid & arready;
			@(negedge clk); n = n + 1;
			if (ar_hs) arvalid = 0;
		end
		arvalid = 0;
		if (!rvalid) begin $display("FAIL read %06h: no RVALID", a); fails = fails + 1; end
		lat_check("RVALID", n, rlat_ref);
		d0 = rdata; dl = rdata;
		done = !rvalid;
		while (!done) begin
			if (rvalid) begin
				if (beats < 16) begin rb_d[beats] = rdata; rb_id[beats] = rid; end
				beats = beats + 1; dl = rdata;
				done = rlast;
			end
			@(negedge clk);
		end
		bus_busy = 0;
	end
endtask

// A write burst of len + 1 beats and type bt (data a pattern, all strobes): AW and the first W together, one
// beat per W handshake, WLAST on the last. nb counts the write responses until 16 cycles after the first.
task axi_wburst(input [20:0] a, input [3:0] len, input [1:0] bt, output integer nb);
	integer n, q;
	reg [3:0] k;
	reg aw_hs, w_hs;
	begin
		bus_get;
		@(negedge clk);
		awaddr = a; awlen = len; awsize = 2; awburst = bt; awvalid = 1;
		k = 0; wdata = 32'hB0057000; wstrb = 4'hF; wlast = (len == 4'd0); wvalid = 1;
		n = 0; nb = 0; q = -1;
		while (n <= 64 && (q < 0 || n < q + 16)) begin
			aw_hs = awvalid & awready; w_hs = wvalid & wready;
			if (bvalid) begin nb = nb + 1; if (q < 0) q = n; end
			@(negedge clk); n = n + 1;
			if (aw_hs) awvalid = 0;
			if (w_hs) begin
				if (k == len) wvalid = 0;
				else begin k = k + 4'd1; wdata = {28'hB005700, k}; wlast = (k == len); end
			end
		end
		awvalid = 0; wvalid = 0; awlen = 0; awburst = 2'b01; wlast = 1;
		checks = checks + 1;
		if (k != len) begin fails = fails + 1; $display("FAIL write burst %06h: %0d of %0d beats taken", a, k + 1, len + 1); end
		@(negedge clk);
		bus_busy = 0;
	end
endtask

// The clk_100m edges of the last AR and W handshakes.
realtime t_arhs = 0, t_whs = 0;
always @(posedge clk) begin
	if (arvalid && arready) t_arhs = $realtime;
	if (wvalid && wready)   t_whs  = $realtime;
end

// A store, then a read whose ARVALID rises in the cycle after the write response is taken: the earliest read of
// a host that waits for the response.
task store_read(input [20:0] wa, input [31:0] wd, input [20:0] ra, output [31:0] rv);
	integer n, nb;
	reg aw_hs, w_hs;
	reg [31:0] vl;
	begin
		bus_get;
		@(negedge clk);
		awaddr = wa; awlen = 0; awsize = 2; awvalid = 1;
		wdata = wd; wstrb = 4'hF; wlast = 1; wvalid = 1;
		n = 0;
		while (!bvalid && n <= 64) begin
			aw_hs = awvalid & awready; w_hs = wvalid & wready;
			@(negedge clk); n = n + 1;
			if (aw_hs) awvalid = 0;
			if (w_hs) wvalid = 0;
		end
		awvalid = 0; wvalid = 0;
		if (!bvalid) begin $display("FAIL write %06h: no BVALID", wa); fails = fails + 1; end
		lat_check("BVALID", n, wlat_ref);
		bus_busy = 0;
		axi_read(ra, 0, rv, vl, nb);   // BREADY is high: the response is taken at the next edge
	end
endtask

// A store's AW alone, then its data beat and a read placed so that the read's AR is accepted `off` cycles after
// the data beat (before it when negative, -2 at the earliest). Returns the word read, and checks the placement.
task race_store_read(input [20:0] wa, input [31:0] wd, input [20:0] ra, input integer off, output [31:0] rv);
	integer dt;
	begin
		bus_get;
		@(negedge clk);
		awaddr = wa; awlen = 0; awsize = 2; awvalid = 1; wvalid = 0;
		@(negedge clk);
		awvalid = 0;
		fork
			begin
				repeat (3) @(negedge clk);
				wdata = wd; wstrb = 4'hF; wlast = 1; wvalid = 1;
				@(negedge clk); wvalid = 0;
			end
			begin
				repeat (3 + off) @(negedge clk);
				araddr = ra; arlen = 0; arsize = 2; arvalid = 1;
				@(negedge clk); arvalid = 0;
				while (!rvalid) @(negedge clk);
				rv = rdata;
			end
		join
		repeat (4) @(negedge clk);
		bus_busy = 0;
		dt = $rtoi((t_arhs - t_whs + 1000.0) / 10.0 + 0.5) - 100;
		check($sformatf("race: AR accepted %0d cycles from the data beat", off), dt, off);
	end
endtask

function [20:0] pad_addr(input [7:0] op, input [7:0] idx);
	pad_addr = 21'h40000 + 21'h40000 + {op, 10'd0} + {idx, 2'b00};
endfunction

reg [31:0] d, dl;
integer beats, i;

task rd(input string what, input [20:0] a, input [31:0] exp);
	begin axi_read(a, 0, d, dl, beats); check(what, d, exp); end
endtask

task settle; repeat (slow(100)) @(negedge clk); endtask

// The host's first two drain steps on a CAUSE word: read it, store the value back (write-one-to-clear).
task cause_drain(input [20:0] a);
	reg [31:0] c, cl;
	integer nb;
	begin axi_read(a, 0, c, cl, nb); axi_write(a, c, 4'hF); end
endtask

// The entries with a lease link (SECTOR, LANE): a store to one needs its OPEN bit.
localparam TB_E_SEC = 1, TB_E_SYS = TB_SEC ? 3 : 2, TB_E_LANE = TB_E_SYS + 1;
localparam [31:0] TB_LINKED = (TB_SEC ? 32'd1 << TB_E_SEC : 32'd0) | (TB_LANE ? 32'd1 << TB_E_LANE : 32'd0);
// The host's step before a CLAIM: LEASE read until quiet, for a bound (a stopped clk_sys never answers).
task wait_quiet;
	reg [31:0] v;
	integer n;
	begin
		n = 0;
		axi_read(LEASE, 0, v, dl, beats);
		while (!v[30] && n < slow(4000)) begin
			repeat (6) @(negedge clk); n = n + 10;
			axi_read(LEASE, 0, v, dl, beats);
		end
	end
endtask
// A CLAIM stored once the lease links let it through.
task claim_w(input [9:0] id, input [3:0] ttl); begin wait_quiet; axi_write(LEASE, lease_w(id, 2'd1, ttl), 4'hF); end endtask
// The host's step after a CLAIM: OPEN read until the linked entries' bits are set, for a bound.
task wait_open;
	reg [31:0] v;
	integer n;
	begin
		n = 0;
		axi_read(OPENA, 0, v, dl, beats);
		while ((v & TB_LINKED) != TB_LINKED && n < slow(1000)) begin
			repeat (6) @(negedge clk); n = n + 10;
			axi_read(OPENA, 0, v, dl, beats);
		end
	end
endtask
// The bench's host takes the lease (releasing whoever holds it first, then waiting for quiet), waits for its
// linked entries to open, with no keeper and no line.
task lease_take;
	reg [31:0] v;
	begin
		kp_on = 0;
		axi_read(LEASE, 0, v, dl, beats);
		if (v[31]) axi_write(LEASE, lease_w(v[9:0], 2'd3, 4'd0), 4'hF);
		claim_w(KP_ID, KP_TTL);
		axi_read(LEASE, 0, v, dl, beats);
		check("the bench's host holds the lease", {v[31], v[9:0]}, {1'b1, KP_ID});
		kp_epoch = v[23:16];
		wait_open;
	end
endtask
// The same, then every line enabled, the status read once (the claim raised STATUS) and the lease kept.
task lease_up;
	begin
		lease_take;
		axi_write(LINEN, 32'hFFFFFFFF, 4'hF);
		cause_drain(SYSB);
		kp_t = $realtime; kp_on = 1;
	end
endtask
// The bench's host stops keeping the lease and releases it.
task lease_off;
	reg [31:0] v;
	begin
		kp_on = 0;
		axi_write(LEASE, lease_w(KP_ID, 2'd3, 4'd0), 4'hF);
		axi_read(LEASE, 0, v, dl, beats);
		check("the bench's host released the lease", v[31], 0);
	end
endtask
task lease_rd(output [31:0] v); axi_read(LEASE, 0, v, dl, beats); endtask

// ---- The lease links: the checker on each, the bench's own rest predicate, claims against it ----
reg fc_off = 0;                    // a case scrambles or forces a link
`ifdef MISTER_LW_SECTOR_SLOT
fence_chk fc_sec(.clk(clk), .off(fc_off), .st(win.sec.sector.fence.st), .lvl(win.sec.sector.fence.lease_lwx),
                 .ack_s(win.sec.sector.fence.ack_s), .stb(win.sec.sector.wsel & win.sec.sector.write));
`endif
`ifdef MISTER_LW_LANE
fence_chk fc_lane(.clk(clk), .off(fc_off), .st(win.lane.lane.fence.st), .lvl(win.lane.lane.fence.lease_lwx),
                  .ack_s(win.lane.lane.fence.ack_s), .stb(win.lane.lane.wsel & win.lane.lane.write));
`endif
function [10:0] tb_g2b(input [10:0] g);
	integer n;
	begin tb_g2b[10] = g[10]; for (n = 9; n >= 0; n = n - 1) tb_g2b[n] = tb_g2b[n + 1] ^ g[n]; end
endfunction
// Each linked half at rest, as the bench reads its registers: the lease level low, the machine idle with idle
// outputs, the queue empty against the synchronised write pointer, the Gray sources current, every EN and mux
// (SECTOR) on SPI. rh_*: the predicate at the last four clk_sys edges.
wire tr_sec, tr_lane;
`ifdef MISTER_LW_SECTOR_SLOT
assign tr_sec = !win.sec.sector.lease_s && win.sec.sector.cs1 == 0 && !win.sec.sector.en_sys &&
                win.sec.sector.x_en_sys == 0 && (win.sec.sector.fs == 0 || win.sec.sector.fs == 4) &&
                !win.sec.sector.qv && !win.sec.sector.em && win.sec.sector.fack == 0 && !win.sec.sector.o_wr &&
                {1'b0, win.sec.sector.rptr} == tb_g2b({1'b0, win.sec.sector.wg_s}) &&
                win.sec.sector.sector_r_lwgray == (win.sec.sector.rptr ^ (win.sec.sector.rptr >> 1));
`else
assign tr_sec = 1'b1;
`endif
`ifdef MISTER_LW_LANE
assign tr_lane = !win.lane.lane.lease_s && win.lane.lane.ps == 0 && !win.lane.lane.own && !win.lane.lane.hold &&
                 !win.lane.lane.strobe && win.lane.lane.sel == 0 && win.lane.lane.rptr == tb_g2b(win.lane.lane.wg_s) &&
                 win.lane.lane.lane_r_lwgray == (win.lane.lane.rptr ^ (win.lane.lane.rptr >> 1)) &&
                 win.lane.lane.lane_rw_lwgray == (win.lane.lane.rwp ^ (win.lane.lane.rwp >> 1));
`else
assign tr_lane = 1'b1;
`endif
reg [3:0] rh_sec = 0, rh_lane = 0;
always @(posedge clk_sys) begin rh_sec <= {rh_sec[2:0], tr_sec}; rh_lane <= {rh_lane[2:0], tr_lane}; end
integer n_claims = 0, n_offrest = 0;
always @(posedge clk) if (win.l_claim) begin
	n_claims = n_claims + 1;
	if (!(tr_sec && tr_lane && &rh_sec && &rh_lane)) begin
		n_offrest = n_offrest + 1;
		if (n_offrest <= 4) $display("   a CLAIM taken at %0t with a half off rest: SECTOR %b %b, LANE %b %b", $time,
		                             tr_sec, rh_sec, tr_lane, rh_lane);
	end
end
`ifdef MISTER_LW_LANE
// Frames committed (an END taken with no overflow) and played (the played toggle seen on clk_100m).
integer n_commit = 0, n_play = 0;
always @(posedge clk) begin
	if (win.lane.lane.w_end && !win.lane.lane.ovf) n_commit = n_commit + 1;
	if (win.lane.lane.pt_s ^ win.lane.lane.pt_d) n_play = n_play + 1;
end
`endif

// The clk_100m edge of the last full-word store to each CAUSE word, seen on the register bus.
realtime t_wsec = 0, t_wsys = 0;
always @(posedge clk) if (lw_write && &lw_be) begin
	if (lw_waddr == 21'h41000) t_wsec = $realtime;
	if (lw_waddr == 21'h43000) t_wsys = $realtime;
end

// LEASE moves with time (`left`, a RENEW) and with the bench's own stores (`burst`): word comparisons leave it out.
function [31:0] lmask(input [20:0] a, input [31:0] v); lmask = (a == 21'h40018) ? 32'd0 : v; endfunction

// The word a beat of a read burst must carry: wref[] holds the single-read words of 0x40000 .. 0x400FC.
reg [31:0] wref [0:63];
function [20:0] beat_addr(input [20:0] a, input [3:0] len, input [1:0] bt, input [3:0] j);
	reg [20:0] sz, lo;
	begin
		sz = {13'd0, {2'b00, len} + 6'd1, 2'b00};
		lo = a & ~(sz - 21'd1);
		case (bt)
			2'b00:   beat_addr = a;
			2'b10:   beat_addr = lo + ((a - lo + {15'd0, j, 2'b00}) % sz);
			default: beat_addr = a + {15'd0, j, 2'b00};
		endcase
	end
endfunction

// One read burst against wref[]: the beat count (so RLAST came on the last beat), every beat's word and ID.
task burst_vs_ref(input string what, input [20:0] a, input [3:0] len, input [1:0] bt);
	integer j, nb, nbad, nbad_id;
	reg [20:0] ba;
	reg  [5:0] wi;
	begin
		arburst = bt; arid = {8'h5A, len};
		axi_read(a, len, d, dl, beats);
		arburst = 2'b01; arid = 0;
		nb = len;
		nb = nb + 1;
		check({what, ": beats"}, beats, nb);
		if (beats < nb) nb = beats;
		nbad = 0; nbad_id = 0;
		for (j = 0; j < nb; j = j + 1) begin
			ba = beat_addr(a, len, bt, j[3:0]);
			wi = ba[7:2];
			if (lmask(ba, rb_d[j]) !== lmask(ba, wref[wi])) begin
				if (nbad == 0) $display("     %s: beat %0d at %06h got %08h exp %08h", what, j, ba, rb_d[j], wref[wi]);
				nbad = nbad + 1;
			end
			if (rb_id[j] !== {8'h5A, len}) nbad_id = nbad_id + 1;
		end
		check({what, ": stepped words"}, nbad, 0);
		check({what, ": ID echoed on every beat"}, nbad_id, 0);
	end
endtask


// The address a beat of a size-2 read burst reads: the low 12 bits step, [20:12] hold.
function [20:0] beat12(input [20:0] a, input [3:0] len, input [1:0] bt, input [3:0] j);
	reg [11:0] sz, lo;
	begin
		sz = ({8'd0, len} + 12'd1) << 2;
		lo = a[11:0] & ~(sz - 12'd1);
		case (bt)
			2'b00:   beat12 = a;
			2'b10:   beat12 = {a[20:12], lo + ((a[11:0] - lo + {6'd0, j, 2'b00}) % sz)};
			default: beat12 = {a[20:12], a[11:0] + {6'd0, j, 2'b00}};
		endcase
	end
endfunction

// One read burst against single reads of the addresses beat12 gives (reads are pure): beats, words and ID.
task burst_vs_single(input string what, input [20:0] a, input [3:0] len, input [1:0] bt);
	integer j, nb, nbad;
	reg [31:0] e [0:15];
	reg [31:0] v, vl;
	reg [20:0] ba;
	begin
		for (j = 0; j <= len; j = j + 1) begin axi_read(beat12(a, len, bt, j[3:0]), 0, v, vl, nb); e[j] = v; end
		arburst = bt; arid = {8'hC4, len};
		axi_read(a, len, d, dl, beats);
		arburst = 2'b01; arid = 0;
		nb = len;
		nb = nb + 1;
		check({what, ": beats"}, beats, nb);
		if (beats < nb) nb = beats;
		nbad = 0;
		for (j = 0; j < nb; j = j + 1) begin
			ba = beat12(a, len, bt, j[3:0]);
			if (lmask(ba, rb_d[j]) !== lmask(ba, e[j]) || rb_id[j] !== {8'hC4, len}) begin
				if (nbad == 0) $display("     %s: beat %0d at %06h got %08h/%03h exp %08h", what, j, ba, rb_d[j], rb_id[j], e[j]);
				nbad = nbad + 1;
			end
		end
		check({what, ": every beat's word and ID, stepped inside its 4 KB"}, nbad, 0);
	end
endtask

localparam [20:0] DIRB = 21'h40000;           // the directory
localparam [20:0] SCR  = DIRB + 21'h28;      // its SCRATCH word
localparam [20:0] SEC = 21'h41000;
reg [31:0] gen0, d0;
integer n0, s0, overlap = 0;

// a memcard strobe while the LW feeder holds the CD ack: the overlap the mux exists for
always @(posedge clk_sys) if (sd_buff_wr && sd_ack[2] && blk_dn[17] && !blk_dn[16]) overlap = 1;

function [31:0] pat(input [31:0] lba, input integer i);
	pat = (lba * 32'h9E3779B1) ^ (i * 32'h01000193) ^ {i[15:0], 16'h5A00};
endfunction

function [31:0] frame_sum(input [31:0] lba, input integer n);
	integer m;
	reg [31:0] w;
	begin
		frame_sum = 0;
		for (m = 0; m < n; m = m + 1) begin
			w = pat(lba, m);
			frame_sum = frame_sum + w[31:24] + w[23:16] + w[15:8] + w[7:0];
		end
	end
endfunction

task sec_wr(input [20:0] off, input [31:0] v); axi_write(SEC + off, v, 4'hF); endtask
task sec_data(input integer i, input [31:0] v); axi_write(SEC + 21'h1000 + 4*i, v, 4'hF); endtask

// Below the default clk_sys rate the host model paces its DATA stores by the rate ratio, so a frame fills the
// FIFO no further than it does at the default rate; drops and resends have their own cases.
task frame_data(input [31:0] lba, input integer from, input integer to);
	integer m;
	for (m = from; m < to; m = m + 1) begin
		sec_data(m, pat(lba, m));
		repeat (slow(5) - 5) @(negedge clk);
	end
endtask

task cd_request(input [31:0] lba);
	integer t;
	begin
		@(negedge clk_sys);
		cd_next = lba; cd_go = 1;
		t = 0;
		while (!(cd_req && cd_lba == lba) && t < 10000) begin @(negedge clk_sys); t = t + 1; end
	end
endtask

task wait_frame(input integer n);
	integer t;
	begin
		t = 0;
		while (cd_frames == n && t < 20000) begin @(negedge clk_sys); t = t + 1; end
		checks = checks + 1;
		if (cd_frames == n) begin fails = fails + 1; $display("FAIL frame did not complete"); end
	end
endtask

// words [0, n) carry the pattern, the rest of the frame must be zero (fill)
task check_buf(input string what, input [31:0] lba, input integer n);
	integer m, bad, first;
	reg [31:0] e;
	begin
		bad = 0; first = -1;
		for (m = 0; m < TB_FRAME; m = m + 1) begin
			e = (m < n) ? pat(lba, m) : 32'd0;
			if (cbuf[m] !== e) begin bad = bad + 1; if (first < 0) first = m; end
		end
		if (bad) $display("     %s: first bad word %0d got %08h", what, first, cbuf[first]);
		check(what, bad, 0);
	end
endtask

// hps_io SPI side: one command word then n data words, one strobe every `gap` clk_sys
task spi_sector(input [15:0] cmd, input integer n, input [15:0] seed, input integer gap);
	integer m;
	begin
		@(negedge clk_sys); io_enable = 1;
		io_din = cmd; io_strobe = 1; @(negedge clk_sys); io_strobe = 0; repeat (gap - 1) @(negedge clk_sys);
		for (m = 0; m < n; m = m + 1) begin
			io_din = seed + m; io_strobe = 1; @(negedge clk_sys); io_strobe = 0; repeat (gap - 1) @(negedge clk_sys);
		end
		io_enable = 0; @(negedge clk_sys);
	end
endtask

// stock path: UIO_SECTOR_RD on slot 1 with the frame as 1176 16-bit words
task spi_frame(input [31:0] lba);
	integer m;
	reg [31:0] w;
	begin
		@(negedge clk_sys); io_enable = 1;
		io_din = 16'h0117; io_strobe = 1; @(negedge clk_sys); io_strobe = 0; @(negedge clk_sys);
		for (m = 0; m < 1176; m = m + 1) begin
			w = pat(lba, m >> 1);
			io_din = m[0] ? w[31:16] : w[15:0]; io_strobe = 1; @(negedge clk_sys); io_strobe = 0; @(negedge clk_sys);
		end
		io_enable = 0; @(negedge clk_sys);
	end
endtask

// OSDBUF: the tb's models of the two images, the stores that fill them and the pixel-path check.
reg [31:0] stf0, stf1;
reg [127:0] stw;

function [127:0] st_pat(input integer k);
	st_pat = {32'hA5000000 ^ k, 32'h5A000000 + k, 32'h00C30000 | k, {k[23:0], 8'h3C}};
endfunction

// One core status request: st_in, then a rising edge of status_set (held one clk_sys, then dropped).
// The last capture of the snapshot, for the race sweep's level property.
realtime t_cap = 0;
always @(posedge clk) if (win.sys_status.capture) t_cap = $realtime;

task st_req(input [127:0] v);
	begin
		@(negedge clk_sys); st_in = v; st_set = 1;
		@(negedge clk_sys); st_set = 0;
	end
endtask

// The HPS's seqlock read: STFLG, the four words, STFLG; the snapshot only when both STFLG words agree.
task st_read(output [31:0] f0, output [31:0] f1, output [127:0] w);
	begin
		axi_read(SYSB + 21'h04, 0, f0, dl, beats);
		axi_read(SYSB + 21'h08, 0, w[31:0], dl, beats);
		axi_read(SYSB + 21'h0C, 0, w[63:32], dl, beats);
		axi_read(SYSB + 21'h10, 0, w[95:64], dl, beats);
		axi_read(SYSB + 21'h14, 0, w[127:96], dl, beats);
		axi_read(SYSB + 21'h04, 0, f1, dl, beats);
	end
endtask

localparam [20:0] OSD = 21'h44000;
reg  [7:0] spi_img [0:4095];
reg  [7:0] lw_img  [0:4095];

function [7:0] lwp(input integer i, input integer k);
	lwp = (i * 37) ^ (i >> 4) ^ (k * 8'h5B) ^ 8'hA5;
endfunction
function [7:0] spp(input integer i, input integer k);
	spp = (i * 113) ^ (i >> 6) ^ (k * 8'hC7) ^ 8'h3C;
endfunction

task osd_word(input integer w, input [31:0] v, input [3:0] be);
	integer b;
	begin
		axi_write(OSD + 4*w, v, be);
		for (b = 0; b < 4; b = b + 1) if (be[b]) lw_img[4*w + b] = v[8*b +: 8];
	end
endtask

task lw_row(input integer r, input integer k);
	integer w;
	for (w = r*64; w < r*64 + 64; w = w + 1) osd_word(w, {lwp(4*w+3, k), lwp(4*w+2, k), lwp(4*w+1, k), lwp(4*w, k)}, 4'hF);
endtask

task spi_osd_begin(input [7:0] cmd);
	begin
		@(negedge clk_sys); io_osd = 1;
		io_din = {8'h00, cmd}; io_strobe = 1; @(negedge clk_sys); io_strobe = 0; @(negedge clk_sys);
	end
endtask
task spi_osd_end; begin io_osd = 0; repeat (2) @(negedge clk_sys); end endtask
task spi_osd_cmd(input [7:0] cmd); begin spi_osd_begin(cmd); spi_osd_end; end endtask

// OSDCMDWRITE row r: 0x20|r, so bit 3 (rows 8-15) re-arms the SPI highres (osd.v:80)
task spi_osd_row(input integer r, input integer k);
	integer m;
	begin
		spi_osd_begin(8'h20 | r[4:0]);
		for (m = 0; m < 256; m = m + 1) begin
			spi_img[r*256 + m] = spp(r*256 + m, k);
			io_din = {8'h00, spi_img[r*256 + m]}; io_strobe = 1; @(negedge clk_sys); io_strobe = 0; @(negedge clk_sys);
		end
		spi_osd_end;
	end
endtask

// After `nf` frames, both outputs drew an OSD box `exp_lines` high and 256 wide, bit (y, x) from the image.
task osd_check(input string what, input from_lw, input integer exp_lines, input integer nf);
	integer f, y, x, bh, bv;
	reg [7:0] b;
	begin
		f = cap_h.frames;
		wait (cap_h.frames >= f + nf);
		@(negedge clk_vid);
		bh = 0; bv = 0;
		for (y = 0; y < exp_lines && y < 128; y = y + 1)
			for (x = 0; x < 256; x = x + 1) begin
				b = from_lw ? lw_img[(y >> 3)*256 + x] : spi_img[(y >> 3)*256 + x];
				if (cap_h.cap[y*256 + x] !== b[y & 7]) bh = bh + 1;
				if (cap_v.cap[y*256 + x] !== b[y & 7]) bv = bv + 1;
			end
		check({what, ": hdmi lines"}, cap_h.lines, exp_lines);
		check({what, ": vga lines"}, cap_v.lines, exp_lines);
		check({what, ": widths 256"}, cap_h.wbad_l + cap_v.wbad_l, 0);
		check({what, ": hdmi pixels"}, bh, 0);
		check({what, ": vga pixels"}, bv, 0);
	end
endtask

function [31:0] lanes_h(input integer w);
	lanes_h = {hdmi_osd.lw_buf3[w], hdmi_osd.lw_buf2[w], hdmi_osd.lw_buf1[w], hdmi_osd.lw_buf0[w]};
endfunction
function [31:0] lanes_v(input integer w);
	lanes_v = {vga_osd.lw_buf3[w], vga_osd.lw_buf2[w], vga_osd.lw_buf1[w], vga_osd.lw_buf0[w]};
endfunction
function [31:0] img_w(input integer w);
	img_w = {lw_img[4*w+3], lw_img[4*w+2], lw_img[4*w+1], lw_img[4*w]};
endfunction

integer r, bad;
string  caps;

// The header rows (ra[12], MENU_CORE builds) always read osd_buffer, never the shadow: on every pixel step
// with LWSEL set, a header byte must be the SPI byte. Old values sampled at the edge, pbyte read after.
integer hdr_n = 0, hdr_bad = 0, shd_n = 0, c_n = 0, l_n = 0;
always @(posedge clk_vid) begin : hdr_watch
	reg h, l, c;
	reg [7:0] o;
	c = hdmi_osd.ce_pix; h = hdmi_osd.pix.lw_hdr; l = hdmi_osd.lwsel_v1; o = hdmi_osd.pix.osd_byte;
	#0.5;
	if (c) c_n = c_n + 1;
	if (l) l_n = l_n + 1;
	if (c && l && h) begin
		hdr_n = hdr_n + 1;
		if (hdmi_osd.pix.pbyte !== o) hdr_bad = hdr_bad + 1;
	end
	if (c && l && !h && hdmi_osd.pix.pbyte !== o) shd_n = shd_n + 1;
end

`ifdef MISTER_LW_LANE
// LANE: frames stored over LW against the same frames sent as link words (the HPS's handshake on io_clk/io_ack).
localparam [20:0] LANE = 21'h46000;
reg         tb_vsync = 0, cfg_has = 0;
// sys_top's 0x30 arm, the one config decode the link's ack reads: vs_wait from the command word to a vsync
always @(posedge clk_sys) begin
	if (~m_sel[0]) cfg_has <= 0;
	else if (m_strobe && !cfg_has) begin
		cfg_has <= 1;
		if (m_din[7:0] == 8'h30) vs_wait <= 1;
	end
	if (tb_vsync) vs_wait <= 0;
end
initial begin hps_io.RTC = 0; hps_io.TIMESTAMP = 0; end
// vs_auto: a vsync every 2000 clk_sys cycles, as a running video output gives one, for the cases whose stray link
// words may select sys_top's 0x30 arm
reg     vs_auto = 0;
integer vs_n = 0;
always @(negedge clk_sys) if (vs_auto) begin
	vs_n = vs_n + 1;
	if (vs_n % 2000 == 0) tb_vsync = 1; else if (vs_n % 2000 == 1) tb_vsync = 0;
end

// What every decoder saw: the strobed {sel, word} stream, gamma writes, toggles and img_mounted pulses.
integer     bus_n = 0, gam_n = 0, rtc_t = 0, ts_t = 0, mnt_n = 0, mnt_w = 0;
reg  [31:0] bus_h = 0, gam_h = 0;
reg   [3:0] mnt_v = 0;
reg         rtc_d = 0, ts_d = 0, mnt_d = 0;
integer     lo_run = 0, lo_min = 1000;       // the shortest all-selects-low run between two transactions
always @(posedge clk_sys) begin
	if (m_sel == 0) lo_run <= lo_run + 1;
	else begin
		if (lo_run > 0 && lo_run < lo_min) lo_min <= lo_run;
		lo_run <= 0;
	end
	if (m_strobe) begin
		bus_n <= bus_n + 1;
		bus_h <= {bus_h[26:0], bus_h[31:27]} ^ {12'd0, m_sel, m_din};
	end
	if (hps_io.gamma_wr) begin
		gam_n <= gam_n + 1;
		gam_h <= {gam_h[26:0], gam_h[31:27]} ^ {14'd0, hps_io.gamma_wr_addr, hps_io.gamma_value};
	end
	rtc_d <= rtc[64];
	ts_d  <= tstamp[32];
	if (rtc[64] !== rtc_d) rtc_t <= rtc_t + 1;
	if (tstamp[32] !== ts_d) ts_t <= ts_t + 1;
	mnt_d <= |img_m === 1'b1;
	if (|img_m === 1'b1) begin
		mnt_w <= mnt_w + 1;
		mnt_v <= img_m;
		if (!mnt_d) mnt_n <= mnt_n + 1;
	end
end

function [431:0] snap;
	snap = {hst, rtc[63:0], tstamp[31:0], img_sz, img_ro, hps_io.cfg, sdsz, umode, uspeed, hps_io.gamma_en, 6'd0,
	        hps_io.ioctl_index, hps_io.ioctl_file_ext};
endfunction

reg [431:0] s_ref;
integer     t_ref [0:8];
reg  [15:0] fw [0:1023];
reg  [15:0] fop = 0;
reg   [1:0] fsel = 0;
integer     fn = 0, spin = 0, spin_max = 0, lane_end_t = 0, link_first_t = 0;

task trk_clear;
	begin
		@(negedge clk_sys);
		bus_n = 0; bus_h = 0; gam_n = 0; gam_h = 0; rtc_t = 0; ts_t = 0; mnt_n = 0; mnt_w = 0; mnt_v = 0;
	end
endtask
task trk_save;
	begin
		s_ref = snap();
		t_ref[0] = bus_n; t_ref[1] = bus_h; t_ref[2] = gam_n; t_ref[3] = gam_h;
		t_ref[4] = rtc_t; t_ref[5] = ts_t; t_ref[6] = mnt_n; t_ref[7] = mnt_v; t_ref[8] = mnt_w;
	end
endtask
task trk_cmp(input string what);
	begin
		check({what, ": registers byte-equal"}, snap() === s_ref, 1);
		check({what, ": strobes"}, bus_n, t_ref[0]);
		check({what, ": {sel, word} stream"}, bus_h, t_ref[1]);
		check({what, ": gamma writes"}, gam_n, t_ref[2]);
		check({what, ": gamma stream"}, gam_h, t_ref[3]);
		check({what, ": RTC toggles"}, rtc_t, t_ref[4]);
		check({what, ": TIMESTAMP toggles"}, ts_t, t_ref[5]);
		check({what, ": img_mounted pulses"}, mnt_n, t_ref[6]);
		check({what, ": img_mounted value"}, mnt_v, t_ref[7]);
	end
endtask

task f_set(input [1:0] s, input [15:0] op, input integer n); begin fsel = s; fop = op; fn = n; end endtask

// The HPS's link word: data, strobe up, spin on ack, strobe down, spin on ack low. `spin` = clk_sys cycles to ack.
task hps_word(input [15:0] w);
	begin
		@(negedge clk_sys); io_din = w; io_strobe = 1;
		spin = 0;
		while (m_ack !== 1'b1) begin @(negedge clk_sys); spin = spin + 1; end
		if (spin > spin_max) spin_max = spin;
		io_strobe = 0;
		while (m_ack !== 1'b0) @(negedge clk_sys);
	end
endtask
task link_send;
	integer m;
	begin
		@(negedge clk_sys);
		if (fsel == 0) io_enable = 1; else if (fsel == 1) io_fpga = 1; else io_osd = 1;
		repeat (2) @(negedge clk_sys);
		hps_word(fop);
		for (m = 0; m < fn; m = m + 1) hps_word(fw[m]);
		@(negedge clk_sys); io_enable = 0; io_fpga = 0; io_osd = 0;
		repeat (3) @(negedge clk_sys);
	end
endtask
// BEGIN {capture[18], sel[17:16], op[15:0]}, the WORDs, then END
task lane_send_c(input do_end, input cap);
	integer m;
	begin
		axi_write(LANE, {13'd0, cap, fsel, fop}, 4'hF);
		for (m = 0; m < fn; m = m + 1) axi_write(LANE + 21'h4, {16'd0, fw[m]}, 4'hF);
		if (do_end) axi_write(LANE + 21'h8, 32'd0, 4'hF);
	end
endtask
task lane_send(input do_end); lane_send_c(do_end, 1'b0); endtask
task lane_count(output [31:0] p, output [31:0] dr);
	begin axi_read(LANE + 21'h10, 0, p, dl, beats); axi_read(LANE + 21'h14, 0, dr, dl, beats); end
endtask
// PLAYED reaches `exp`; with `now`, return at once (no settling) so the next read races the frame's end
task lane_wait_n(input [31:0] exp, input now);
	integer n;
	reg [31:0] p;
	begin
		n = 0;
		axi_read(LANE + 21'h10, 0, p, dl, beats);
		while (p != exp && n < 4000) begin
			if (!now) repeat (40) @(negedge clk);
			axi_read(LANE + 21'h10, 0, p, dl, beats);
			n = n + 1;
		end
		check("PLAYED reaches the frames committed", p, exp);
		if (!now) repeat (8) @(negedge clk_sys);
	end
endtask
task lane_wait(input [31:0] exp); lane_wait_n(exp, 1'b0); endtask

// One frame both ways: the link (reference), the link again with every word inverted, then the lane.
task lane_vs_link(input string what);
	integer m;
	reg [15:0] keep [0:1023];
	reg [31:0] p, dr;
	begin
		trk_clear; link_send; trk_save;
		for (m = 0; m < fn; m = m + 1) begin keep[m] = fw[m]; fw[m] = ~fw[m]; end
		link_send;
		for (m = 0; m < fn; m = m + 1) fw[m] = keep[m];
		lane_count(p, dr);
		trk_clear; lane_send(1); lane_wait(p + 1);
		trk_cmp(what);
	end
endtask

// One osd instance's command state and the first 8 bytes of row 3, as its decoder left them.
function [191:0] osd_st_h;
	integer b;
	begin
		osd_st_h = {hdmi_osd.osd_enable, hdmi_osd.osd_status, hdmi_osd.info, hdmi_osd.infoh, hdmi_osd.infow,
		            hdmi_osd.infox, hdmi_osd.infoy, hdmi_osd.rot, hdmi_osd.osd_h, hdmi_osd.osd_w, 64'd0};
		for (b = 0; b < 8; b = b + 1) osd_st_h[8*b +: 8] = hdmi_osd.osd_buffer[768 + b];
	end
endfunction
function [191:0] osd_st_v;
	integer b;
	begin
		osd_st_v = {vga_osd.osd_enable, vga_osd.osd_status, vga_osd.info, vga_osd.infoh, vga_osd.infow,
		            vga_osd.infox, vga_osd.infoy, vga_osd.rot, vga_osd.osd_h, vga_osd.osd_w, 64'd0};
		for (b = 0; b < 8; b = b + 1) osd_st_v[8*b +: 8] = vga_osd.osd_buffer[768 + b];
	end
endfunction

// An OSD frame: the link raises both osd selects at once; the lane plays it as sel 2 (HDMI), then sel 3 (VGA).
// Each lane frame must move only its own osd, to the state the link frame left. perturb_op (no data,
// 'hFFFF = none) runs on the link after the inverted pass, so the lane must restore what the frame sets.
task osd_vs_link(input string what, input [15:0] perturb_op);
	integer m;
	reg [191:0] rh, rv, ph, pv;
	reg [15:0] keep [0:7];
	reg [31:0] p, dr;
	reg [15:0] op;
	begin
		link_send; rh = osd_st_h(); rv = osd_st_v();
		check({what, ": the link moves both osds alike"}, rh === rv, 1);
		op = fop;
		for (m = 0; m < fn; m = m + 1) begin keep[m] = fw[m]; fw[m] = ~fw[m]; end
		link_send;
		if (perturb_op != 16'hFFFF) begin fop = perturb_op; m = fn; fn = 0; link_send; fn = m; fop = op; end
		for (m = 0; m < fn; m = m + 1) fw[m] = keep[m];
		ph = osd_st_h(); pv = osd_st_v();
		check({what, ": perturbed away from the reference"}, ph !== rh, 1);
		lane_count(p, dr);
		fsel = 2; lane_send(1); lane_wait(p + 1);
		check({what, ": sel 2 sets the HDMI osd"}, osd_st_h() === rh, 1);
		check({what, ": sel 2 leaves the VGA osd"}, osd_st_v() === pv, 1);
		fsel = 3; lane_send(1); lane_wait(p + 2);
		check({what, ": sel 3 sets the VGA osd"}, osd_st_v() === rv, 1);
		check({what, ": sel 3 leaves the HDMI osd"}, osd_st_h() === rh, 1);
	end
endtask

// The reply FIFO: RSTAT {drops[31:16], count[15:0]}; RHEAD the oldest word; a store to RPOP removes it.
task lane_rstat(output [31:0] rs); axi_read(LANE + 21'h1C, 0, rs, dl, beats); endtask
task lane_pop(output [15:0] w);
	reg [31:0] h;
	begin axi_read(LANE + 21'h20, 0, h, dl, beats); w = h[15:0]; axi_write(LANE + 21'h0C, 32'd0, 4'hF); end
endtask
`endif

// The arm's clk_sys rate, measured over 1000 periods.
task sys_rate;
	realtime t0;
	begin
		@(posedge clk_sys) t0 = $realtime;
		repeat (1000) @(posedge clk_sys);
		$display("-- clk_sys %0.4f MHz (half period %0d ps + 50 ppm), measured %0.4f", SYS_MHZ, `LW_TB_SYS_HP_PS,
		         1000.0 * 1000.0 / ($realtime - t0));
		t0 = ($realtime - t0) * 1000.0 - 2000.0 * `LW_TB_SYS_HP_PS * 1.00005;
		check("clk_sys: 1000 periods within 1 ps of 2 x LW_TB_SYS_HP_PS x 1.00005", {31'd0, t0 < -1.0 || t0 > 1.0}, 0);
	end
endtask

// ---- The firewall across an HPS reset ----
// OSD CTRL is the store target (two readable bits, in every arm); the directory magic the read target.
localparam [20:0] FW_CTRL  = OSD + 21'h1000;
localparam [31:0] FW_MAGIC = 32'h5244574C;   // the directory's MAGIC, "LWDR"

// At negedges: until lw_hrst has been high for a full cycle (its rise is behind), or until the cycle it falls in.
task fw_wait_high;
	integer n;
	begin
		n = 0; @(negedge clk);
		while (!(lw_hrst === 1'b1 && fm_hq === 1'b1) && n < 200) begin @(negedge clk); n = n + 1; end
		if (n >= 200) begin fails = fails + 1; $display("FAIL lw_hrst never rose"); end
	end
endtask
task fw_wait_fall;
	integer n;
	begin
		n = 0; @(negedge clk);
		while (!(lw_hrst === 1'b0 && fm_hq === 1'b1) && n < 400) begin @(negedge clk); n = n + 1; end
		if (n >= 400) begin fails = fails + 1; $display("FAIL lw_hrst never fell"); end
	end
endtask

// Called at a negedge: present AR (AW) until it is accepted, and return at the negedge after its handshake.
task fw_ar(input [20:0] a, input [3:0] len, input [11:0] id);
	integer n;
	begin
		araddr = a; arlen = len; arsize = 2; rsize = 2; arburst = 2'b01; arid = id; arvalid = 1; n = 0;
		while (arready !== 1'b1 && n < 200) begin @(negedge clk); n = n + 1; end
		@(negedge clk); arvalid = 0; arid = 0;
	end
endtask
task fw_aw(input [20:0] a, input [11:0] id);
	integer n;
	begin
		awaddr = a; awlen = 0; awsize = 2; awburst = 2'b01; awid = id; awvalid = 1; n = 0;
		while (awready !== 1'b1 && n < 200) begin @(negedge clk); n = n + 1; end
		@(negedge clk); awvalid = 0;
	end
endtask
// One W beat, presented for exactly one cycle.
task fw_w1(input [31:0] v);
	begin wdata = v; wstrb = 4'hF; wlast = 1; wvalid = 1; @(negedge clk); wvalid = 0; end
endtask

// One reset case. A transaction of `kind` (0 a read of the magic, 1 a store to OSD CTRL whose data beat waits,
// 2 a store to OSD CTRL whose response waits) is issued `off` cycles before hps_rst_n falls (after it when
// negative), or, with off 100, so that it is accepted on the edge lw_hrst rises on (seen one cycle ahead in the
// synchroniser's first stage, which the delay models may hold a cycle), and with 101 in the rise's own cycle;
// hps_rst_n is low for 10 cycles. Its master then completes it per `pol`: 0 only well after the
// release, 1 while lw_hrst is high, 2 in the cycle lw_hrst falls, 3 in the cycle after. A transaction the rise of
// lw_hrst found in the machine is marked, and freed at the fall unless a handshake was made since; every other
// one completes with its own data and ID. Then a read and a store are answered in the fixed latency.
// cov: bit (1 << state) for each machine state a marked transaction held at the rise (IDLE: one accepted in the
// rise's own cycle, which counts as outstanding).
task fw_case(input integer kind, input integer off, input integer pol, inout integer cov);
	integer fr0, fw0, nr0, nb0, nw0, nr1, nb1, mk, fe, j;
	reg [11:0] id;
	reg [31:0] oc, v, rv;
	reg [20:0] a;
	reg  [1:0] st;
	string     w;
	reg [31:0] lv;
	begin
		lease_take;                                    // the reset ends it
		axi_read(FW_CTRL, 0, oc, dl, beats);
		v  = {30'd0, ~oc[1:0]};
		id = 12'h800 | (kind << 8) | (pol << 4) | ((off + 4) & 15);
		a  = (kind != 0) ? FW_CTRL : 21'h40000;
		rready = 0; bready = (kind != 2);
		fr0 = n_fr; fw0 = n_fw; nr0 = fr_n; nb0 = fb_n; nw0 = n_lwwr;
		fork
			begin
				repeat (4) @(negedge clk); hps_rst_n = 0;
				repeat (10) @(negedge clk); hps_rst_n = 1;
			end
			begin
				if (off < 100) repeat (4 - off) @(negedge clk);
				else begin
					j = 0; @(negedge clk);
					while (j < 40 && !(off == 100 ? (bridge.hrst_c.sync.lws0_q === 1'b1 && lw_hrst === 1'b0)
					                               : (lw_hrst === 1'b1 && fm_hq === 1'b0))) begin
						@(negedge clk); j = j + 1;
					end
				end
				case (kind)
					0: fw_ar(a, 0, id);
					1: fw_aw(a, id);
					default: begin
						wdata = v; wstrb = 4'hF; wlast = 1; wvalid = 1;
						fw_aw(a, id);
						j = 0; while (wready !== 1'b1 && j < 20) begin @(negedge clk); j = j + 1; end
						@(negedge clk); wvalid = 0;
					end
				endcase
				case (pol)
					1: begin
						fw_wait_high;
						j = 0;
						if (kind == 0) begin
							while (rvalid !== 1'b1 && j < 20) begin @(negedge clk); j = j + 1; end
							rready = 1;
						end
						else if (kind == 1) begin
							while (wready !== 1'b1 && j < 20) begin @(negedge clk); j = j + 1; end
							fw_w1(v);
						end
						else begin
							while (bvalid !== 1'b1 && j < 20) begin @(negedge clk); j = j + 1; end
							bready = 1;
						end
						@(negedge clk);
						check("the handshake is made while lw_hrst is high", lw_hrst, 1);
					end
					2, 3: begin
						fw_wait_fall;
						if (pol == 3) @(negedge clk);
						if (kind == 0) rready = 1;
						else if (kind == 1) fw_w1(v);
						else bready = 1;
					end
					default: ;
				endcase
			end
		join
		if (pol < 2) fw_wait_fall;
		repeat (6) @(negedge clk);
		mk = (kind == 0) ? fm_mk_r : fm_mk_w;
		st = (kind == 0) ? fm_st_r : fm_st_w;
		if (mk) cov = cov | (1 << st);
		// a marked store whose AW was accepted in the rise's cycle has its data beat taken after it: that
		// handshake clears the mark, and the store's response waits for its master like any other
		fe = mk && (pol == 0 || pol == 3) && !(kind == 2 && st == 2'd0);
		nr1 = fr_n - nr0; nb1 = fb_n - nb0;
		if (off == 100)      w = $sformatf("kind %0d, accepted on lw_hrst's rising edge", kind);
		else if (off == 101) w = $sformatf("kind %0d, issued in lw_hrst's first cycle", kind);
		else                 w = $sformatf("kind %0d, %0d cycles before hps_rst_n falls", kind, off);
		if (mk) w = $sformatf("%s, marked in state %0d", w, st);
		else    w = {w, ", not marked"};
		check({w, ": its machine freed"}, ((kind == 0) ? n_fr - fr0 : n_fw - fw0), fe);
		check({w, ": the other machine untouched"}, ((kind == 0) ? n_fw - fw0 : n_fr - fr0), 0);
		if (pol == 1 || pol == 2) check({w, ": answered before its master's cleanup"}, (kind == 0) ? nr1 : nb1, 1);
		// a master that has not finished does so now
		rready = 1; bready = 1;
		if (bridge.adapter.wstate == 2'd1) fw_w1(v);
		j = 0; while (fw_idle !== 1'b1 && j < 50) begin @(negedge clk); j = j + 1; end
		repeat (2) @(negedge clk);
		check({w, ": idle after"}, fw_idle, 1);
		if (kind == 0) begin
			check({w, ": responses"}, fr_n - nr0, fe ? 0 : 1);
			if (!fe) check({w, ": the beat's ID, data and RLAST"}, {fr_id, fr_last, fr_d == FW_MAGIC}, {id, 2'b11});
		end
		else begin
			check({w, ": stores that reached the bus"}, n_lwwr - nw0, (kind == 1 && fe) ? 0 : 1);
			check({w, ": write responses"}, fb_n - nb0, fe ? 0 : 1);
			if (!fe) check({w, ": the response's ID"}, fb_id, id);
		end
		if (kind != 0 && n_lwwr - nw0 == 1)
			check({w, ": the store on the bus, its address and data"}, {fw_wa == a, fw_wd == v}, 2'b11);
		// the reset ended the lease, so the store had no effect: OSD CTRL reads 0 without one
		lease_rd(lv);
		check({w, ": the lease ended, by the HPS reset"}, {lv[31], lv[13:12]}, {1'b0, 2'd2});
		axi_read(FW_CTRL, 0, rv, dl, beats);
		check({w, ": OSD CTRL, with no lease"}, rv, 0);
		// the next accesses are answered in the fixed latency (axi_read and axi_write check it)
		rd({w, ": a read after"}, 21'h40000, FW_MAGIC);
		lease_take;
		axi_write(FW_CTRL, v, 4'hF);
		rd({w, ": a store after, under a new lease"}, FW_CTRL, v);
		axi_write(FW_CTRL, oc, 4'hF);
	end
endtask

// A monitor for case (i): cycles in which idle read 1 while fw_i_mon is set.
reg     fw_i_mon = 0;
integer fw_i_ones = 0;
always @(negedge clk) if (fw_i_mon && fw_idle !== 1'b0) fw_i_ones = fw_i_ones + 1;

task fw_reset_cases;
	integer kind, off, pol, cov, fr0, fw0, nr0, nb0, nw0, j, n, nm;
	reg [31:0] oc, v, ref16 [0:15];
	reg aw_hs, ar_hs, w_hs;
	begin
		for (pol = 0; pol < 4; pol = pol + 1)
			for (kind = 0; kind < 3; kind = kind + 1) begin
				$display("-- firewall, HPS reset: %s, %s",
				         kind == 0 ? "a read" : kind == 1 ? "a store waiting for its data beat" :
				                                            "a store waiting for its response",
				         pol == 0 ? "its master waits past the release" :
				         pol == 1 ? "its master completes it during the reset" :
				         pol == 2 ? "its master completes it in the release cycle" :
				                    "its master completes it the cycle after");
				cov = 0;
				for (off = 3; off >= -3; off = off - 1) fw_case(kind, off, pol, cov);
				fw_case(kind, 100, pol, cov);
				fw_case(kind, 101, pol, cov);
				// the rise found the machine in each state a transaction can be in
				check("states a marked transaction held at the rise", cov, (kind == 1) ? 32'h3 : 32'h7);
			end

		$display("-- firewall, HPS reset: a read and a store accepted while it is held complete across the release");
		lease_take;
		axi_read(FW_CTRL, 0, oc, dl, beats); v = {30'd0, ~oc[1:0]};
		rready = 0; bready = 0; fr0 = n_fr; fw0 = n_fw; nr0 = fr_n; nb0 = fb_n;
		hps_rst_n = 0; fw_wait_high;
		fw_ar(21'h40000, 0, 12'h0C3);
		wdata = v; wstrb = 4'hF; wlast = 1; wvalid = 1;
		fw_aw(FW_CTRL, 12'h0C4);
		j = 0; while (wready !== 1'b1 && j < 20) begin @(negedge clk); j = j + 1; end
		@(negedge clk); wvalid = 0;
		repeat (4) @(negedge clk); hps_rst_n = 1;
		fw_wait_fall; repeat (6) @(negedge clk);
		check("both still answering after the release", {rvalid, bvalid}, 2'b11);
		check("both marks clear", {bridge.adapter.rmark, bridge.adapter.wmark}, 0);
		rready = 1; bready = 1; repeat (4) @(negedge clk);
		check("the read's beats", fr_n - nr0, 1);
		check("the read's beat: ID", fr_id, 12'h0C3);
		check("the read's beat: data", fr_d, FW_MAGIC);
		check("the store's responses", fb_n - nb0, 1);
		check("the store's response: ID", fb_id, 12'h0C4);
		check("nothing freed", (n_fr - fr0) + (n_fw - fw0), 0);
		check("the store reached the bus with its address and data", {fw_wa == FW_CTRL, fw_wd == v}, 2'b11);
		rd("the store had no effect: the reset ended the lease", FW_CTRL, 32'd0);

		$display("-- firewall, HPS reset: a 16-beat read carried across the whole reset, beats taken during it");
		axi_read(21'h40000, 15, d, dl, beats);
		for (j = 0; j < 16; j = j + 1) ref16[j] = rb_d[j];
		rready = 0; fr0 = n_fr; nr0 = fr_n;
		@(negedge clk); fw_ar(21'h40000, 15, 12'h0D1);
		rready = 1; n = 0; while (fr_n - nr0 < 4 && n < 50) begin @(negedge clk); n = n + 1; end
		rready = 0;
		hps_rst_n = 0; fw_wait_high;
		check("the read is marked at the rise", fm_mk_r, 1);
		rready = 1; n = 0; while (fr_n - nr0 < 8 && n < 50) begin @(negedge clk); n = n + 1; end
		rready = 0;
		check("beats taken by now", fr_n - nr0, 8);
		check("beats 4-7 taken while lw_hrst is high", lw_hrst, 1);
		repeat (4) @(negedge clk); hps_rst_n = 1;
		fw_wait_fall; repeat (4) @(negedge clk);
		rready = 1; n = 0; while (fr_n - nr0 < 16 && n < 100) begin @(negedge clk); n = n + 1; end
		repeat (4) @(negedge clk);
		check("beats delivered", fr_n - nr0, 16);
		n = 0;
		for (j = 0; j < 16; j = j + 1)
			if ((j != 6 && j != 9 && j != 11 && j != 12 && j != 13 && fr_log[(nr0 + j) % 32] !== ref16[j]) || fr_idlog[(nr0 + j) % 32] !== 12'h0D1 ||
			    fr_lastlog[(nr0 + j) % 32] !== (j == 15)) n = n + 1;
		check("beats that differ from a plain 16-beat read (data but LEASE, OWNED, OPEN, LINE_EN and RESETS, which the reset moves; ID, RLAST)", n, 0);
		check("nothing freed", n_fr - fr0, 0);
		rd("a read after", 21'h40000, FW_MAGIC);

		$display("-- firewall, HPS reset held low while reads and stores run: every access answers, nothing is freed");
		lease_take;
		axi_read(FW_CTRL, 0, oc, dl, beats); v = {30'd0, ~oc[1:0]};
		fr0 = n_fr; fw0 = n_fw;
		hps_rst_n = 0; fw_wait_high;
		for (j = 0; j < 4; j = j + 1) begin
			rd("magic, reset held", 21'h40000, FW_MAGIC);
			axi_write(FW_CTRL, v, 4'hF);
			rd("a store, reset held: dropped, no lease", FW_CTRL, 32'd0);
			axi_write(LEASE, lease_w(KP_ID, 2'd1, KP_TTL), 4'hF);
			axi_read(LEASE, 0, oc, dl, beats);
			check("a CLAIM, reset held: not taken, hrst shown, the lease ended by the reset",
			      {oc[31], oc[15], oc[13:12]}, {1'b0, 1'b1, 2'd2});
			check("a store, reset held: stale", oc[29], 1);
		end
		oc = 0;
		axi_read(21'h40000, 15, d, dl, beats);
		check("a 16-beat read, reset held", beats, 16);
		check("lw_hrst still high", lw_hrst, 1);
		hps_rst_n = 1; fw_wait_fall; repeat (4) @(negedge clk);
		check("nothing freed", (n_fr - fr0) + (n_fw - fw0), 0);
		check("idle", fw_idle, 1);

		$display("-- firewall, HPS reset with clk_100m stopped across the pulse: once the clock runs, the orphan is freed");
		rready = 0; fr0 = n_fr; fw0 = n_fw; nr0 = fr_n;
		@(negedge clk); fw_ar(21'h40000, 0, 12'h0E7);
		n = 0; while (rvalid !== 1'b1 && n < 20) begin @(negedge clk); n = n + 1; end
		clk_run = 0; #40;
		hps_rst_n = 0; #25; hps_rst_n = 1; #25;
		check("clock stopped: the chain caught the pulse", bridge.hrst_c.lwa1_q, 1);
		check("clock stopped: lw_hrst not yet high", lw_hrst, 0);
		clk_run = 1;
		// a pulse this short may leave lw_hrst high for one cycle only: the first stage can take the chain's rise
		// an edge late, while its fall, synchronous, is never late
		n = 0; @(negedge clk); while (lw_hrst !== 1'b1 && n < 20) begin @(negedge clk); n = n + 1; end
		check("lw_hrst rose once the clock ran", lw_hrst, 1);
		fw_wait_fall;
		check("the read was marked at the rise", fm_mk_r, 1);
		repeat (4) @(negedge clk);
		check("the read machine freed", n_fr - fr0, 1);
		check("the write machine not", n_fw - fw0, 0);
		check("RVALID dropped, idle", {rvalid, fw_idle}, 2'b01);
		rready = 1;
		rd("a read after", 21'h40000, FW_MAGIC);
		check("responses: only the new read's", fr_n - nr0, 1);

		$display("-- firewall, a bridge-only reset (records): an abandoned read holds AR until an HPS reset pulse");
		rready = 0; fr0 = n_fr; nr0 = fr_n;
		@(negedge clk); fw_ar(21'h40000, 0, 12'h0E1);
		n = 0; while (rvalid !== 1'b1 && n < 20) begin @(negedge clk); n = n + 1; end
		araddr = 21'h40000; arid = 12'h0E2; arlen = 0; arvalid = 1;
		n = 0; for (j = 0; j < 40; j = j + 1) begin @(negedge clk); n = n + arready; end
		check("records: the next AR not accepted in 40 cycles", n, 0);
		hps_rst_n = 0; repeat (6) @(negedge clk); hps_rst_n = 1;
		fw_wait_fall;
		n = 0; while (arready !== 1'b1 && n < 20) begin @(negedge clk); n = n + 1; end
		@(negedge clk); arvalid = 0; arid = 0;
		rready = 1; repeat (6) @(negedge clk);
		check("the pulse freed the read machine", n_fr - fr0, 1);
		check("the held AR answered after the release: beats", fr_n - nr0, 1);
		check("the held AR answered after the release: ID", fr_id, 12'h0E2);
		check("the held AR answered after the release: data", fr_d, FW_MAGIC);

		$display("-- firewall, a master revived before the release (records): idle reads 0 throughout");
		lease_take;
		axi_read(FW_CTRL, 0, oc, dl, beats); v = {30'd0, ~oc[1:0]};
		rready = 0; fr0 = n_fr; fw0 = n_fw; nr0 = fr_n; nb0 = fb_n; nw0 = n_lwwr;
		@(negedge clk); fw_ar(21'h000100, 0, 12'h0A1);
		fw_aw(21'h000104, 12'h0A2);
		fw_i_mon = 1; fw_i_ones = 0;
		hps_rst_n = 0; fw_wait_high;
		check("both orphans marked", {fm_mk_r, fm_mk_w}, 2'b11);
		// the new master issues a read and a store together, with every READY high
		araddr = 21'h40000; arid = 12'h0B1; arlen = 0; arvalid = 1; rready = 1;
		awaddr = FW_CTRL; awid = 12'h0B2; awlen = 0; awvalid = 1; bready = 1;
		wdata = v; wstrb = 4'hF; wlast = 1; wvalid = 1;
		n = 0;
		while ((arvalid || awvalid || wvalid || fr_n - nr0 < 2) && n < 40) begin
			ar_hs = arvalid & arready; aw_hs = awvalid & awready; w_hs = wvalid & wready;
			@(negedge clk); n = n + 1;
			if (ar_hs) arvalid = 0;
			if (aw_hs) awvalid = 0;
			if (w_hs) wvalid = 0;
		end
		arid = 0;
		check("records: the first beat the new master took is the orphan's (its ID)", fr_idlog[nr0 % 32], 12'h0A1);
		check("records: the second is its own read (ID)", fr_idlog[(nr0 + 1) % 32], 12'h0B1);
		check("records: the second is its own read (data)", fr_log[(nr0 + 1) % 32], FW_MAGIC);
		check("records: its W beat is one store", n_lwwr - nw0, 1);
		check("records: stored at the orphan's address", fw_wa, 21'h000104);
		check("records: one write response", fb_n - nb0, 1);
		check("records: answered with the orphan's ID", fb_id, 12'h0A2);
		check("records: its AW accepted alone, the write machine waiting for data", bridge.adapter.wstate, 1);
		hps_rst_n = 1; fw_wait_fall; repeat (4) @(negedge clk);
		check("no machine freed: each orphan saw a handshake", (n_fr - fr0) + (n_fw - fw0), 0);
		fw_w1(oc);                                     // its next store's data beat: lands on OSD CTRL's address
		fw_i_mon = 0;
		repeat (4) @(negedge clk);
		check("idle read 0 from the orphans to the last data beat", fw_i_ones, 0);
		check("records: the next beat completes the accepted AW (ID)", fb_id, 12'h0B2);
		check("idle after", fw_idle, 1);
		check("the next beat's store on the bus: OSD CTRL's address, its data", {fw_wa == FW_CTRL, fw_wd == oc}, 2'b11);
		rd("OSD CTRL: no lease after the reset", FW_CTRL, 32'd0);

		$display("-- firewall, the fabric reset held low with an orphan: idle and the mark stay until the net rises");
		rready = 0; fr0 = n_fr; fw0 = n_fw;
		@(negedge clk); fw_ar(21'h40000, 0, 12'h0C1);
		hps_rst_n = 0; fw_wait_high;
		n = 0; nm = 0;
		for (j = 0; j < 200; j = j + 1) begin
			@(negedge clk);
			n = n + (fw_idle !== 1'b0); nm = nm + (bridge.adapter.rmark !== 1'b1);
		end
		check("held 200 cycles: cycles idle read other than 0", n, 0);
		check("held 200 cycles: cycles the read's mark was not set", nm, 0);
		check("held 200 cycles: nothing freed", (n_fr - fr0) + (n_fw - fw0), 0);
		check("held 200 cycles: lw_hrst high", lw_hrst, 1);
		hps_rst_n = 1; fw_wait_fall; repeat (4) @(negedge clk);
		check("freed at the release: the read machine", n_fr - fr0, 1);
		check("freed at the release: not the write machine", n_fw - fw0, 0);
		check("idle", fw_idle, 1);
		rready = 1;
		rd("a read after", 21'h40000, FW_MAGIC);
	end
endtask

// ---- The directory, layout 2 ----
// dm[] is the directory page this build must read, word for word, built here from the register map; em_* lists
// its entries as a host must find them. TB_LINE_K is the ordinal the bench raises nothing on: no entry names it.
reg  [31:0] dm [0:1023];
integer     em_n;
reg  [15:0] em_kind  [0:31];
reg  [31:0] em_ver   [0:31], em_off [0:31], em_size [0:31], em_cause [0:31], em_bits [0:31];
reg   [7:0] em_line  [0:31];

task dm_entry(input integer e, input [15:0] kind, input [31:0] ver, input [31:0] off, input [31:0] size,
              input [31:0] cause, input [7:0] line, input [31:0] bits);
	integer b;
	begin
		b = 16 + 32 * e;
		dm[b] = {16'd0, kind}; dm[b + 1] = ver; dm[b + 2] = off; dm[b + 3] = size; dm[b + 4] = cause;
		dm[b + 5] = {24'd0, line}; dm[b + 6] = bits; dm[b + 7] = 0;
		em_kind[e] = kind; em_ver[e] = ver; em_off[e] = off; em_size[e] = size; em_cause[e] = cause;
		em_line[e] = line; em_bits[e] = bits;
	end
endtask

task dm_build;
	integer i, e, b;
	begin
		for (i = 0; i < 1024; i = i + 1) dm[i] = 0;
		e = 0;
		dm_entry(e, 1, 32'h00010000, 32'h40000, 32'h40000, 0, 0, 0);               // PAD
		dm[16 + 32 * e + 8] = 32'h040F000C; dm[16 + 32 * e + 9] = 32'h20000000;       // K0-K7: the opcode bitmap
		e = e + 1;
		if (TB_SEC) begin
			dm_entry(e, 2, 32'h00010001, 32'h01000, 32'h02000, 32'h01000, 1, 1);       // SECTOR: line 1, cause bit 0
			b = 16 + 32 * e + 8;
			dm[b]     = {8'd1, 8'd0, 6'd0, TB_FRAME[9:0]};                             // K0 {slot, frame words}
			dm[b + 2] = 32'h02000002;                                                  // K2 {FIFO depth, gap}
			dm[b + 3] = 32'h02030100; dm[b + 4] = 32'h30100000;                       // K3-K8: the two cards
			dm[b + 5] = 32'h03030100; dm[b + 6] = 32'h30200000;
			dm[b + 7] = 32'h00020000; dm[b + 8] = 32'h00020000;
			e = e + 1;
		end
		dm_entry(e, 3, 32'h00010000, 32'h04000, 32'h02000, 0, 0, 0);               // OSDBUF
		dm[16 + 32 * e + 8] = 32'h00800100;                                            // K0 {height, width}
		e = e + 1;
		dm_entry(e, 4, 32'h00010000, 32'h03000, 32'h01000, 32'h03000, 2, 1);       // SYS: line 2, STATUS
		e = e + 1;
		if (TB_LANE) begin
			dm_entry(e, 5, 32'h00010001, 32'h06000, 32'h01000, 0, 0, 0);           // LANE: no cause, no line
			b = 16 + 32 * e + 8;
			dm[b] = 32'h70000002; dm[b + 1] = 32'h080E0014;                            // K0-K7: the replay bitmap
			dm[b + 8] = 32'h00000003; dm[b + 9] = 32'h04000004; dm[b + 10] = 32'h01000000;
			e = e + 1;
		end
		em_n = e;
		dm[0] = 32'h5244574C; dm[1] = 32'h00020000; dm[2] = 32'h00800040; dm[3] = e;
		dm[4] = 32'h00040000; dm[5] = 32'h00080000; dm[6] = 32'h40000000; dm[7] = {TB_LU[7:0], 24'd100000};
	end
endtask

// A host's walk of the directory, by the guide's rules, from the bus (src 0) or the bench's copy syn[] (src 1):
// refuse an unknown MAGIC or directory major, and a WIN_BASE or WIN_SIZE that is not the token's; walk COUNT entries
// by the LAYOUT word's header length and stride; skip kind 0, an unknown kind or kind major, a private kind (this
// host knows no TAG) and an entry whose OFFSET, SIZE or CAUSE is outside the window or its block; give no line to an
// entry whose ordinal another listed entry names too, or that is out of its kind's range (1-15 framework, 16-31
// private). The result: wk_n entries, each with its index in the table.
reg  [31:0] syn [0:1023];
integer     wk_n, wk_refused;
string      wk_why;
reg   [4:0] wk_idx   [0:31];
reg  [15:0] wk_kind  [0:31];
reg  [31:0] wk_ver   [0:31], wk_off [0:31], wk_size [0:31], wk_cause [0:31], wk_bits [0:31];
reg   [7:0] wk_line  [0:31];

task dir_word(input integer src, input [11:0] off, output [31:0] v);
	reg [31:0] vl;
	integer nb;
	if (src == 0) axi_read(DIRB + {9'd0, off}, 0, v, vl, nb);
	else v = syn[off[11:2]];
endtask

function integer kind_major(input [15:0] k);     // the kind majors this host knows; 0 = an unknown kind
	kind_major = (k >= 16'd1 && k <= 16'd5) ? 1 : 0;
endfunction

task dir_walk(input integer src, input [31:0] tok_base, input [31:0] tok_size);
	reg  [31:0] v, hl, st, cnt, b, kind, ver, off, size, cause, line, bits;
	reg   [7:0] ord [0:31];
	reg         ok;
	integer     i, j, shared;
	begin
		wk_n = 0; wk_refused = 0; wk_why = "";
		dir_word(src, 12'h000, v); if (v !== 32'h5244574C) begin wk_refused = 1; wk_why = "MAGIC"; end
		dir_word(src, 12'h004, v); if (!wk_refused && v[31:16] !== 16'd2) begin wk_refused = 1; wk_why = "major"; end
		dir_word(src, 12'h010, v); if (!wk_refused && v !== tok_base) begin wk_refused = 1; wk_why = "WIN_BASE"; end
		dir_word(src, 12'h014, v); if (!wk_refused && v !== tok_size) begin wk_refused = 1; wk_why = "WIN_SIZE"; end
		dir_word(src, 12'h008, v); hl = v[15:0]; st = v[31:16];
		dir_word(src, 12'h00C, v); cnt = v[15:0];
		if (!wk_refused && (hl < 32'h40 || st < 32'h80 || hl[1:0] != 0 || st[1:0] != 0 || cnt > 31 ||
		                    hl + cnt * st > 32'h1000)) begin wk_refused = 1; wk_why = "LAYOUT"; end
		if (!wk_refused) begin
			for (i = 0; i < cnt; i = i + 1) begin
				dir_word(src, hl + i * st, kind); dir_word(src, hl + i * st + 20, line);
				ord[i] = (kind[15:0] == 0 || line[31:8] != 0) ? 8'd0 : line[7:0];
			end
			for (i = 0; i < cnt; i = i + 1) begin
				b = hl + i * st;
				dir_word(src, b,      kind);  dir_word(src, b + 4,  ver);   dir_word(src, b + 8,  off);
				dir_word(src, b + 12, size);  dir_word(src, b + 16, cause); dir_word(src, b + 24, bits);
				ok = kind[31:16] == 0 && kind[15:0] != 0 && kind[15:0] < 16'h8000 &&
				     kind_major(kind[15:0]) != 0 && ver[31:16] == kind_major(kind[15:0]) &&
				     size != 0 && off[1:0] == 0 && size[1:0] == 0 && off < tok_size && size <= tok_size - off &&
				     (cause == 0 || (cause >= off && cause < off + size && cause[1:0] == 0));
				if (ok) begin
					shared = 0;
					for (j = 0; j < cnt; j = j + 1) if (j != i && ord[j] == ord[i]) shared = 1;
					wk_idx[wk_n] = i; wk_kind[wk_n] = kind[15:0]; wk_ver[wk_n] = ver; wk_off[wk_n] = off;
					wk_size[wk_n] = size; wk_cause[wk_n] = cause; wk_bits[wk_n] = bits;
					wk_line[wk_n] = (ord[i] >= 8'd1 && ord[i] <= 8'd15 && !shared) ? ord[i] : 8'd0;
					wk_n = wk_n + 1;
				end
			end
		end
	end
endtask

// The walk against the expected entries, with an ordinal the case expects in place of one entry's (kind k_ln).
task walk_check(input string what, input [15:0] k_ln, input [7:0] ln);
	integer i, bad;
	reg [7:0] el;
	begin
		check({what, ": not refused"}, wk_refused, 0);
		check({what, ": entries found"}, wk_n, em_n);
		bad = 0;
		for (i = 0; i < em_n && i < wk_n; i = i + 1) begin
			el = (em_kind[i] == k_ln) ? ln : em_line[i];
			if (wk_idx[i] !== i || wk_kind[i] !== em_kind[i] || wk_ver[i] !== em_ver[i] || wk_off[i] !== em_off[i] ||
			    wk_size[i] !== em_size[i] || wk_cause[i] !== em_cause[i] || wk_line[i] !== el ||
			    wk_bits[i] !== em_bits[i]) begin
				if (bad == 0) $display("     %s: entry %0d kind %0d found as index %0d kind %0d line %0d", what, i,
				                       em_kind[i], wk_idx[i], wk_kind[i], wk_line[i]);
				bad = bad + 1;
			end
		end
		check({what, ": every entry as listed"}, bad, 0);
	end
endtask

// Copy the directory page into syn[].
task syn_copy;
	integer i;
	reg [31:0] v, vl;
	integer nb;
	for (i = 0; i < 1024; i = i + 1) begin axi_read(DIRB + {7'd0, i[9:0], 2'b00}, 0, v, vl, nb); syn[i] = v; end
endtask

// One entry appended to syn[] (index n, the default layout), COUNT raised to n + 1.
task syn_add(input integer n, input [31:0] kind, input [31:0] ver, input [31:0] off, input [31:0] size,
             input [31:0] cause, input [31:0] line, input [31:0] tag);
	integer b, i;
	begin
		b = 16 + 32 * n;
		for (i = 0; i < 32; i = i + 1) syn[b + i] = 32'hA5A5_0000 | i;     // kind words a host must not read
		syn[b] = kind; syn[b + 1] = ver; syn[b + 2] = off; syn[b + 3] = size; syn[b + 4] = cause;
		syn[b + 5] = line; syn[b + 6] = 32'h1; syn[b + 7] = tag;
		syn[3] = n + 1;
	end
endtask

// The token, the page word for word, a host's walk on the bus and on altered copies.
task dir_cases;
	integer i, n, bad, nl, tb_off, tb_size;
	reg [31:0] v, vl;
	reg  [7:0] seen [0:31];
	integer    nb;
	begin
		$display("-- directory: the token's numbers are the window's and the header's");
		// `SYS_LW_CAPS carries its own concatenation comma, so it is read inside {} with a closing ";"
		caps = {`SYS_LW_CAPS ";"};
		n = $sscanf(caps, "AXI:%h:%h:LW,;", tb_off, tb_size);
		check("SYS_LW_CAPS parses as AXI:<offset>:<size>:LW", n, 2);
		check("SYS_LW_CAPS is that token alone, in every block set",
		      caps == $sformatf("AXI:%0h:%0h:LW,;", tb_off, tb_size), 1);
		check("token offset = the window's WIN_OFF", tb_off, win.WIN_OFF);
		check("token size = the window's WIN_SIZE", tb_size, win.WIN_SIZE);
		rd("WIN_BASE = the token's offset", DIRB + 21'h10, tb_off);
		rd("WIN_SIZE = the token's size", DIRB + 21'h14, tb_size);

		$display("-- directory: the header, then the page word for word");
		dm_build;
		rd("MAGIC LWDR",                 DIRB + 21'h00, 32'h5244574C);
		rd("VERSION 2.0",                DIRB + 21'h04, 32'h00020000);
		rd("LAYOUT {stride 0x80, header 0x40}", DIRB + 21'h08, 32'h00800040);
		rd("COUNT: one entry per block built", DIRB + 21'h0C, em_n);
		rd("LEASE: none held, quiet",    DIRB + 21'h18, 32'h40000000);
		rd("TIMEBASE",                   DIRB + 21'h1C, {TB_LU[7:0], 24'd100000});
		rd("PENDING: no cause",          DIRB + 21'h20, 32'd0);
		rd("OWNED: nothing",             DIRB + 21'h24, 32'd0);
		rd("SCRATCH reset",              SCR,           32'd0);
		rd("OPEN reads 0",               DIRB + 21'h2C, 32'd0);
		rd("LINE_EN reads 0",            DIRB + 21'h30, 32'd0);
		rd("RESETS reads 0",             DIRB + 21'h34, 32'd0);
		bad = 0;
		for (i = 0; i < 1024; i = i + 1) begin
			axi_read(DIRB + {7'd0, i[9:0], 2'b00}, 0, v, vl, nb);
			if (v !== dm[i]) begin
				if (bad < 4) $display("     directory word %03h: %08h, expected %08h", 4 * i, v, dm[i]);
				bad = bad + 1;
			end
		end
		check("the directory page, 1,024 words, word for word", bad, 0);

		$display("-- directory: a host's walk finds every entry; one ordinal per entry, framework ordinals 1-15");
		dir_walk(0, tb_off, tb_size);
		walk_check("walk on the bus", 0, 0);
		nl = 0; bad = 0;
		for (i = 0; i < em_n; i = i + 1) begin
			axi_read(DIRB + 21'h40 + 21'h80 * i + 21'h14, 0, v, vl, nb);
			if (v != 0) begin
				if (v > 15) bad = bad + 1;
				for (n = 0; n < nl; n = n + 1) if (seen[n] == v[7:0]) bad = bad + 1;
				seen[nl] = v[7:0]; nl = nl + 1;
			end
		end
		check("LINE words: each non-zero ordinal named once, each in 1-15", bad, 0);
		check("entries with a line", nl, TB_SEC ? 2 : 1);
		for (i = 0; i < wk_n; i = i + 1) if (wk_cause[i] != 0) begin
			axi_read(DIRB + wk_cause[i], 0, v, vl, nb);
			check($sformatf("kind %0d: its CAUSE word reads as an idle cause", wk_kind[i]), v, 0);
			check($sformatf("kind %0d: CAUSE_BITS within [15:0]", wk_kind[i]), wk_bits[i][31:16], 0);
		end

		$display("-- directory: the walk refuses majors 1 and 3, the layout-1 magic and a window not the token's");
		syn_copy;
		dir_walk(1, tb_off, tb_size); walk_check("the copy", 0, 0);
		syn[1] = 32'h00010000; dir_walk(1, tb_off, tb_size); check("major 1 refused", wk_refused, 1);
		syn[1] = 32'h00030000; dir_walk(1, tb_off, tb_size); check("major 3 refused", wk_refused, 1);
		syn[1] = 32'h00020007; dir_walk(1, tb_off, tb_size); walk_check("minor 7: walked", 0, 0);
		syn[1] = 32'h00020000;
		syn[0] = 32'h31535953; dir_walk(1, tb_off, tb_size); check("the layout-1 magic refused", wk_refused, 1);
		syn[0] = 32'h5244574C;
		syn[4] = 32'h00041000; dir_walk(1, tb_off, tb_size); check("WIN_BASE not the token's: refused", wk_refused, 1);
		syn[4] = tb_off;
		syn[5] = 32'h00040000; dir_walk(1, tb_off, tb_size); check("WIN_SIZE not the token's: refused", wk_refused, 1);
		syn[5] = tb_size;

		$display("-- directory: the walk skips kind 0, an unknown kind or major, a private kind and a block outside");
		n = em_n;
		syn_add(n,     32'h0000, 32'h00010000, 32'hFFFFF000, 32'h1000, 0, 32'd1, 0);             // kind 0
		syn_add(n + 1, 32'h0077, 32'h00010000, 32'h07000, 32'h1000, 0, 32'd4, 0);                // unknown kind
		syn_add(n + 2, 32'h0001, 32'h00090000, 32'h40000, 32'h40000, 0, 32'd0, 0);               // PAD, major 9
		syn_add(n + 3, 32'h8001, 32'h00010000, 32'h20000, 32'h1000, 32'h20000, 32'd16, 32'h54534554);  // private
		syn_add(n + 4, 32'h0003, 32'h00010000, 32'h7F000, 32'h2000, 0, 32'd0, 0);                // OSD, past the end
		syn_add(n + 5, 32'h0004, 32'h00010000, 32'h03000, 32'h1000, 32'h05000, 32'd0, 0);        // SYS, CAUSE outside
		dir_walk(1, tb_off, tb_size); walk_check("six entries to skip", 0, 0);
		syn[16 + 32 * (n + 1) + 5] = 32'd2;                         // the unknown kind names the SYS entry's ordinal
		dir_walk(1, tb_off, tb_size); walk_check("an ordinal two entries name: no line for either", 4, 0);
		syn[16 + 32 * (n + 1) + 5] = 32'd4;
		syn[16 + 5] = 32'd20;                                      // PAD names a private ordinal
		dir_walk(1, tb_off, tb_size); walk_check("a framework ordinal out of 1-15: no line", 1, 0);

		$display("-- directory: the walk follows LAYOUT, not constants (header 0x80, stride 0x100)");
		syn_copy;
		for (i = 1023; i >= 0; i = i - 1)
			if (i >= 32) syn[i] = ((i - 32) % 64 < 32 && (i - 32) / 64 < em_n) ? syn[16 + 32 * ((i - 32) / 64) + (i - 32) % 64] : 0;
			else if (i >= 16) syn[i] = 0;
		syn[1] = 32'h00020001; syn[2] = 32'h01000080;
		dir_walk(1, tb_off, tb_size); walk_check("a larger header and stride", 0, 0);
	end
endtask

// The block selects are exclusive: in every clk_100m cycle exactly one read select (the default's included) is high,
// and at most one store select.
integer sel_cycles = 0, sel_bad = 0;
always @(posedge clk) begin
	sel_cycles = sel_cycles + 1;
	if (win.r_dir + win.r_sec + win.r_sys + win.r_osd + win.r_lane + win.r_pad + win.r_none != 1 ||
	    win.w_dir + win.w_sec + win.w_sys + win.w_osd + win.w_lane + win.w_pad > 1) sel_bad = sel_bad + 1;
end

// The causes the entries declare: each raise sets exactly its entry's PENDING bit and its LINE's bit of the lines,
// and a store of the value read, at the CAUSE offset the entry gives, clears both.
integer line3_high = 0;
always @(posedge clk) if (lines[2] !== 1'b0) line3_high = line3_high + 1;

task dir_causes;
	integer i, j, n, nb, tb_off, tb_size;
	reg [31:0] v, vl;
	begin
		$display("-- directory: each entry's cause sets exactly its PENDING bit and its own line, and clears at its CAUSE");
		n = $sscanf({`SYS_LW_CAPS ";"}, "AXI:%h:%h:LW,;", tb_off, tb_size);
		dir_walk(0, tb_off, tb_size);
		if (TB_SEC) begin sec_wr(21'h10, 32'h0); settle; end
		for (i = 0; i < wk_n; i = i + 1) if (wk_cause[i] != 0) begin
			for (j = 0; j < wk_n; j = j + 1) if (wk_cause[j] != 0) cause_drain(DIRB + wk_cause[j]);
			rd($sformatf("kind %0d: PENDING clear before", wk_kind[i]), DIRB + 21'h20, 32'd0);
			check($sformatf("kind %0d: lines low before", wk_kind[i]), lines, 0);
			if (wk_kind[i] == 2) cd_request(32'h00007700);
			else st_req(st_pat(700));
			settle;
			rd($sformatf("kind %0d: PENDING has its bit alone", wk_kind[i]), DIRB + 21'h20, 32'd1 << wk_idx[i]);
			check($sformatf("kind %0d: its line alone (ordinal %0d)", wk_kind[i], wk_line[i]), lines,
			      (wk_line[i] >= 1 && wk_line[i] <= 3) ? 3'd1 << (wk_line[i] - 1) : 3'd0);
			axi_read(DIRB + wk_cause[i], 0, v, vl, nb);
			check($sformatf("kind %0d: CAUSE set", wk_kind[i]), v != 0, 1);
			check($sformatf("kind %0d: CAUSE within its CAUSE_BITS", wk_kind[i]), v & ~wk_bits[i], 0);
			axi_write(DIRB + wk_cause[i], v, 4'hF);
			rd($sformatf("kind %0d: PENDING clear after the store-back", wk_kind[i]), DIRB + 21'h20, 32'd0);
			check($sformatf("kind %0d: lines low after the store-back", wk_kind[i]), lines, 0);
			if (wk_kind[i] == 2) begin
				@(negedge clk_sys) cd_reset = 1; repeat (4) @(negedge clk_sys); cd_reset = 0; settle;
				if (doorbell) cause_drain(SEC);
			end
		end
		check("no entry names ordinal 3: its line never rose", line3_high, 0);
		$display("     %0d cycles with the block selects compared", sel_cycles);
		check("cycles with no read select, two read selects or two store selects", sel_bad, 0);
	end
endtask

// Every 4 KB page of the 2 MB against the window's map: a block's page reads that block (never the default), every
// other page reads 0xDEADBEEF at its first, middle and last word and drops a store; each block's ID word is at its
// offset.
function integer blk_at(input [20:0] a);
	if (a >= 21'h40000 && a < 21'h41000) blk_at = 1;                    // the directory
	else if (TB_SEC && a >= 21'h41000 && a < 21'h43000) blk_at = 2;     // SECTOR
	else if (a >= 21'h43000 && a < 21'h44000) blk_at = 3;               // SYS
	else if (a >= 21'h44000 && a < 21'h46000) blk_at = 4;               // OSDBUF
	else if (TB_LANE && a >= 21'h46000 && a < 21'h47000) blk_at = 5;    // LANE
	else if (a >= 21'h80000 && a < 21'hC0000) blk_at = 6;               // PAD
	else blk_at = 0;
endfunction

task decode_sweep;
	reg [511:0] sw0, ss0;
	reg [255:0] sl0;
	reg  [20:0] a;
	integer p, k, b, nblk, nnone, bad, nbad;
	begin
		$display("-- decode: every block's pages read the block, every other page 0xDEADBEEF, in the 2 MB");
		nblk = 0; nnone = 0; bad = 0;
		for (p = 0; p < 512; p = p + 1) begin
			b = blk_at({p[8:0], 12'h000});
			if (b != 0) nblk = nblk + 1; else nnone = nnone + 1;
			for (k = 0; k < 3; k = k + 1) begin
				a = {p[8:0], 12'h000} + ((k == 0) ? 21'h000 : (k == 1) ? 21'h7FC : 21'hFFC);
				axi_read(a, 0, d, dl, beats);
				if ((b == 0) != (d === 32'hDEADBEEF)) begin
					if (bad < 4) $display("     %06h (block %0d) reads %08h", a, b, d);
					bad = bad + 1;
				end
			end
		end
		$display("     %0d pages in blocks, %0d outside", nblk, nnone);
		check("pages in blocks", nblk, 1 + 2 * TB_SEC + 1 + 2 + TB_LANE + 64);
		check("every page reads as the map says", bad, 0);
		rd("the directory's MAGIC at its offset", DIRB, FW_MAGIC);
		if (TB_SEC) rd("SECTOR's ID at its offset", SEC + 21'h38, 32'h32434553);
		rd("SYS's ID at its offset", SYSB + 21'h1C, 32'h31535453);
		rd("OSDBUF's ID at its offset", OSD + 21'h1004, 32'h3144534F);
		if (TB_LANE) rd("LANE's ID at its offset", 21'h46018, 32'h314E414C);
		rd("PAD reads 0", pad_addr(8'h02, 0), 32'd0);
		rd("the directory page past the entries reads 0", DIRB + 21'h40 + 21'h80 * em_n, 32'd0);
		rd("the directory page's last word reads 0", DIRB + 21'hFFC, 32'd0);

		$display("-- decode: a store outside every block changes nothing");
		sys_run = 0;
		repeat (50) @(negedge clk);
		sw0 = pure_win(); ss0 = pure_sec(); sl0 = pure_lane();
		pure_view(0, nbad);
		for (p = 0; p < 512; p = p + 1)
			if (blk_at({p[8:0], 12'h000}) == 0) begin
				axi_write({p[8:0], 12'h000}, 32'hFFFFFFFF, 4'hF);
				axi_write({p[8:0], 12'hFFC}, 32'hFFFFFFFF, 4'hF);
			end
		pure_view(1, nbad);
		check("stores outside: the readable view is unchanged", nbad, 0);
		check("stores outside: the window's, PAD's and SYS's state is unchanged", pure_win() === sw0, 1);
		check("stores outside: SECTOR's state is unchanged", pure_sec() === ss0, 1);
		check("stores outside: the lane's state is unchanged", pure_lane() === sl0, 1);
		sys_run = 1;
		settle;
	end
endtask

// ---- The lease: the gate, the verbs, the ends, the lines, two hosts and the start order ----
// live_q's edges, the cycle a RENEW is taken, and the lines against their definition in every cycle: line k is the
// register of live_q & its entry's LINE_EN bit & its entry's CAUSE word non-zero (SECTOR ordinal 1, SYS 2).
realtime t_lrise = 0, t_lfall = 0, t_renew = 0;
integer  n_lrise = 0, n_lfall = 0, n_renew0 = 0;
always @(win.live_q) if ($realtime > 0) begin
	if (win.live_q) begin t_lrise = $realtime; n_lrise = n_lrise + 1; end
	else begin t_lfall = $realtime; n_lfall = n_lfall + 1; end
end
always @(posedge clk) if (win.l_renew) begin
	t_renew = $realtime;
	if (win.lcnt == 0) n_renew0 = n_renew0 + 1;           // a RENEW in the last live cycle
end
reg  [2:0] lm_q = 0;
integer    lm_n = 0, lm_bad = 0, lm_fall_c = 0, lm_fall_e = 0, lm_fall_l = 0, lm_rises = 0, lm_falls = 0;
always @(posedge clk) begin : line_model
	reg [2:0] en, lv, nx;
	en = {1'b0, win.line_en[TB_E_SYS], win.line_en[TB_E_SEC]};
	lv = {1'b0, win.sys_lvl, win.sec_lvl};
	nx = {3{win.live_q}} & en & lv;
	for (i_lm = 0; i_lm < 2; i_lm = i_lm + 1) if (lm_q[i_lm] && !nx[i_lm]) begin
		if (!win.live_q) lm_fall_l = lm_fall_l + 1;
		else if (!en[i_lm]) lm_fall_e = lm_fall_e + 1;
		else lm_fall_c = lm_fall_c + 1;
	end
	for (i_lm = 0; i_lm < 3; i_lm = i_lm + 1) begin
		if (nx[i_lm] && !lm_q[i_lm]) lm_rises = lm_rises + 1;
		if (!nx[i_lm] && lm_q[i_lm]) lm_falls = lm_falls + 1;
	end
	lm_q <= nx;
end
integer i_lm;
always @(negedge clk) begin
	lm_n = lm_n + 1;
	if (lines !== lm_q) begin
		lm_bad = lm_bad + 1;
		if (lm_bad <= 4) $display("   lines at %0t: %b, the definition %b", $time, lines, lm_q);
	end
end

// The lease ends by itself: the bench's host stops renewing, and the bench waits for live_q to fall.
task wait_lapse(input integer ttl);
	integer n;
	reg [31:0] v;
	begin
		kp_on = 0;
		n = 0;
		while (win.live_q && n < (ttl + 1) * (1 << TB_LU) + 100) begin @(negedge clk); n = n + 1; end
		lease_rd(v);
		check("the lease ran out", {v[31], v[13:12]}, {1'b0, 2'd1});
	end
endtask
// The bench's host claims for ttl units with every line enabled, and does not renew.
task lease_short(input [3:0] ttl);
	reg [31:0] v;
	begin
		kp_on = 0;
		axi_read(LEASE, 0, v, dl, beats);
		if (v[31]) axi_write(LEASE, lease_w(v[9:0], 2'd3, 4'd0), 4'hF);
		claim_w(KP_ID, ttl);
		axi_write(LINEN, 32'hFFFFFFFF, 4'hF);
		axi_read(LEASE, 0, v, dl, beats);
		check("a short lease taken", {v[31], v[9:0]}, {1'b1, KP_ID});
		wait_open;
	end
endtask

// A second host's look before it touches the link: the GPO value as left (the bench's gpo31), then one GPI sample.
// 1: a lease is held (store nothing); 0: go on.
function look_held;
	look_held = gpo31 && tb_gpi[31] == 1'b0 && tb_gpi[25] && tb_gpi[22];
endfunction
// A program's identification of the core: the ID view for n clk_100m cycles, then the I/O view again.
task identify(input integer n); begin @(negedge clk); gpo31 = 0; repeat (n) @(negedge clk); gpo31 = 1; repeat (6) @(negedge clk); end endtask

localparam [9:0] ID_B = 10'h155;

task lease_cases;
	reg [31:0] v, v0, oc, lc0;
	integer k, n, j, e0, nw0, nr, nf, tt;
	realtime t0;
	begin
		$display("-- lease: without it every store but LEASE is dropped and sets stale");
		kp_on = 0;
		claim_w(KP_ID, 4'd15);                                       // a claim clears stale and burst
		axi_write(LEASE, lease_w(KP_ID, 2'd3, 4'd0), 4'hF);
		wait_quiet;
		lease_rd(v);
		e0 = v[23:16];
		check("released: none held, quiet, nothing stale, left 0, end 0, the last owner", v,
		      {8'h40, e0[7:0], 6'd0, KP_ID});
		rd("OPEN: no block takes stores", OPENA, 32'd0);
		rd("OWNED: nothing owned", OWNEDA, 32'd0);
		check("GPI[22] reads 0", tb_gpi[22], 0);
		nw0 = n_lwwr;
		v0 = win.scratch;   // what earlier cases left (0 without LW_DIAG)
		axi_write(SCR, 32'h12345678, 4'hF);
		rd("no lease: SCRATCH unchanged", SCR, v0);
		axi_write(LINEN, 32'hFFFFFFFF, 4'hF);
		rd("no lease: LINE_EN unchanged", LINEN, 32'd0);
		n = n_padwe; axi_write(pad_addr(8'h02, 0), 32'h00005555, 4'hF); settle;
		check("no lease: a pad store is dropped, nothing reaches hps_io", n_padwe - n, 0);
		axi_write(OSD + 21'h1000, 32'h3, 4'hF);
		rd("no lease: OSD CTRL unchanged", OSD + 21'h1000, 32'd0);
		n = n_osdwe; axi_write(OSD, 32'hFFFFFFFF, 4'hF);
		check("no lease: an image store does not reach the osds", n_osdwe - n, 0);
		st_req(st_pat(900)); settle;
		axi_write(SYSB, 32'h1, 4'hF);
		rd("no lease: a W1C store on SYS CAUSE is dropped, the cause stands", SYSB, 32'h1);
		if (TB_SEC) begin
			sec_wr(21'h10, 32'h1);
			rd("no lease: SECTOR CTRL unchanged", SEC + 21'h10, 32'd0);
		end
		if (TB_LANE) begin
			axi_read(21'h46010, 0, v0, dl, beats);
			axi_write(21'h46000, 32'h0000001E, 4'hF); axi_write(21'h46008, 32'h0, 4'hF);
			repeat (slow(200)) @(negedge clk);
			rd("no lease: no frame committed, none played (PLAYED)", 21'h46010, v0);
			check("no lease: the lane's write pointers unmoved", win.lane.lane.wcom == win.lane.lane.wspec, 1);
		end
		check("no lease: every store reached the bus", n_lwwr - nw0, 6 + TB_SEC + 2 * TB_LANE);
		lease_rd(v);
		check("no lease: stale set, nothing else", v, {8'h60, e0[7:0], 6'd0, KP_ID});

		$display("-- lease: a wrong key, VERB 0, TTL 0, partial strobes and RENEW or RELEASE without a lease do nothing");
		axi_write(LEASE, {16'h4C56, KP_ID, 2'd1, 4'd15}, 4'hF);
		axi_write(LEASE, lease_w(KP_ID, 2'd0, 4'd15), 4'hF);
		axi_write(LEASE, lease_w(KP_ID, 2'd1, 4'd0), 4'hF);
		axi_write(LEASE, lease_w(KP_ID, 2'd1, 4'd15), 4'h7);
		axi_write(LEASE, lease_w(KP_ID, 2'd1, 4'd15), 4'hE);
		axi_write(LEASE, lease_w(KP_ID, 2'd2, 4'd15), 4'hF);
		axi_write(LEASE, lease_w(KP_ID, 2'd3, 4'd0), 4'hF);
		lease_rd(v);
		check("refused stores: LEASE unchanged (stale set before)", v, {8'h60, e0[7:0], 6'd0, KP_ID});
		axi_wburst(SCR, 4'd1, 2'b01, n);
		lease_rd(v);
		check("a burst write without the lease: burst set", v[28], 1);

		$display("-- lease: CLAIM sets held, owner and the epoch, clears stale and burst; stores take effect");
		axi_write(LEASE, lease_w(KP_ID, 2'd1, 4'd15), 4'hF);
		lease_rd(v);
		check("CLAIM: {held, stale, burst}", {v[31], v[29:28]}, 3'b100);
		check("CLAIM: quiet only with no lease link", v[30], TB_LINKED == 0);
		check("CLAIM: left, epoch, hrst, end, idv, owner", {v[27:24], v[23:16], v[15:11], v[9:0]},
		      {4'd14, e0[7:0] + 8'd1, 5'b00000, KP_ID});
		check("GPI[22] shows the lease", tb_gpi[22], 1);
		wait_open;
		rd("OPEN: every listed block", OPENA, (32'd1 << em_n) - 1);
		rd("OWNED: none", OWNEDA, 32'd0);
		axi_write(SCR, 32'h12345678, 4'hF);
		rd("under the lease: SCRATCH", SCR, dg(32'h12345678));
		axi_write(LINEN, 32'hFFFFFFFF, 4'hF);
		rd("under the lease: LINE_EN holds the listed entries' bits", LINEN, (32'd1 << em_n) - 1);
		axi_write(pad_addr(8'h02, 0), 32'h00005555, 4'hF); settle;
		check("under the lease: the pad store lands", joy0, 32'h00005555);
		axi_write(pad_addr(8'h02, 0), 32'h00000000, 4'hF); settle;
		axi_write(SYSB, 32'h1, 4'hF);
		rd("under the lease: the W1C clears SYS CAUSE", SYSB, 32'd0);
		axi_wburst(SCR, 4'd3, 2'b01, n);
		lease_rd(v);
		check("a burst write under the lease: burst set, stale clear", v[29:28], 2'b01);
		axi_write(SCR, 32'h0, 4'hF);
		if (TB_LINKED != 0) begin
			$display("-- lease: a store to a linked block before its OPEN bit is dropped and sets stale");
			axi_write(LEASE, lease_w(KP_ID, 2'd3, 4'd0), 4'hF);
			wait_quiet;
			sys_run = 0; repeat (20) @(negedge clk);       // the far halves cannot answer the rise
			axi_write(LEASE, lease_w(KP_ID, 2'd1, 4'd15), 4'hF);
			lease_rd(v);
			check("clk_sys stopped: the CLAIM is taken, not quiet, nothing stale", {v[31:29]}, 3'b100);
			rd("clk_sys stopped: OPEN shows the unlinked entries only", OPENA, ((32'd1 << em_n) - 1) & ~TB_LINKED);
			rd("clk_sys stopped: OWNED none (the links rise under a held lease)", OWNEDA, 32'd0);
			n = n_lwwr;
			if (TB_SEC) sec_wr(21'h10, 32'h1);
			if (TB_LANE) axi_write(LANE, 32'h0000001E, 4'hF);
			lease_rd(v);
			check("a store to a linked block, not open: stale set", v[29], 1);
			if (TB_SEC) check("not open: SECTOR's EN unchanged", win.sec.sector.en_lwx, 0);
			if (TB_LANE) check("not open: the lane's frame not begun", win.lane.lane.open, 0);
			axi_write(pad_addr(8'h02, 0), 32'h0BADF00D, 4'hF);   // PAD is not linked: kept for the restart
			repeat (20) @(negedge clk);
			check("clk_sys stopped: the pad word not yet delivered", joy0 == 32'h0BADF00D, 0);
			sys_run = 1;
			settle;
			check("clk_sys running again: the PAD store kept and delivered", joy0, 32'h0BADF00D);
			wait_open;
			rd("clk_sys running again: OPEN every listed block", OPENA, (32'd1 << em_n) - 1);
			if (TB_SEC) begin
				sec_wr(21'h10, 32'h1);
				rd("open: SECTOR's EN store lands", SEC + 21'h10, 32'h1);
				sec_wr(21'h10, 32'h0);
			end
		end

		$display("-- lease: RELEASE ends it at its response, end 0; the next CLAIM bumps the epoch");
		store_read(LEASE, lease_w(KP_ID, 2'd3, 4'd0), LEASE, v);
		check("RELEASE: held clear at its response, end 0", {v[31], v[13:12]}, {1'b0, 2'd0});
		check("RELEASE: live_q fell in the store's bus cycle", (t_lfall > t_whs - 1.0) && (t_lfall < t_whs + 21.0), 1);
		rd("no lease: LINE_EN reads 0", LINEN, 32'd0);
		rd("no lease: OPEN reads 0", OPENA, 32'd0);
		check("GPI[22] clear", tb_gpi[22], 0);
		claim_w(KP_ID, 4'd1);
		lease_rd(v);
		check("CLAIM again: the epoch moved by one", {v[31], v[23:16]}, {1'b1, e0[7:0] + 8'd2 + (TB_LINKED != 0)});
		rd("after a claim: LINE_EN reads 0", LINEN, 32'd0);

		$display("-- lease: expiry at TTL 1, 2 and 15, each exactly TTL units and one cycle; left counts down");
		for (k = 0; k < 3; k = k + 1) begin
			tt = (k == 0) ? 1 : (k == 1) ? 2 : 15;
			if (win.live_q) axi_write(LEASE, lease_w(KP_ID, 2'd3, 4'd0), 4'hF);
			claim_w(KP_ID, tt[3:0]);
			t0 = t_lrise;
			if (tt == 15)
				for (j = 0; j < 15; j = j + 1) begin
					while ($realtime < t0 + 10.0 * (2.0 ** TB_LU) * (j + 0.5)) @(negedge clk);
					lease_rd(v);
					checks = checks + 1;
					if (v[27:24] !== 14 - j) begin
						fails = fails + 1; $display("FAIL TTL 15: left %0d after %0d.5 units", v[27:24], j);
					end
				end
			n = 0; while (win.live_q && n < 20 * (1 << TB_LU)) begin @(negedge clk); n = n + 1; end
			check($sformatf("TTL %0d: live for TTL units and one cycle", tt), $rtoi((t_lfall - t0) / 10.0 + 0.5),
			      tt * (1 << TB_LU) + 1);
			lease_rd(v);
			check($sformatf("TTL %0d: end 1, the counter ran out", tt), {v[31], v[13:12]}, {1'b0, 2'd1});
		end
		$display("-- lease: RENEW restarts the count from its own cycle");
		claim_w(KP_ID, 4'd2);
		t0 = t_lrise;
		while ($realtime < t0 + 10.0 * (2.0 ** TB_LU) * 1.5) @(negedge clk);
		axi_write(LEASE, lease_w(KP_ID, 2'd2, 4'd1), 4'hF);
		n = 0; while (win.live_q && n < 4 * (1 << TB_LU)) begin @(negedge clk); n = n + 1; end
		check("RENEW at 1.5 units with TTL 1: live one unit and one cycle past the RENEW",
		      $rtoi((t_lfall - t_renew) / 10.0 + 0.5), (1 << TB_LU) + 1);

		$display("-- lease: a RENEW swept across the last live cycle keeps the lease whole or finds it gone, never both");
		st_req(st_pat(901)); settle;                       // SYS cause pending: its line follows the lease
		nr = 0; nf = 0; n_renew0 = 0;
		for (k = -10; k <= 10; k = k + 1) begin
			if (win.live_q) axi_write(LEASE, lease_w(KP_ID, 2'd3, 4'd0), 4'hF);
			repeat (4) @(negedge clk);
			wait_quiet;
			e0 = n_lrise + n_lfall; j = lm_rises + lm_falls;
			axi_write(LEASE, lease_w(KP_ID, 2'd1, 4'd1), 4'hF);
			axi_write(LINEN, 32'hFFFFFFFF, 4'hF);
			t0 = t_lrise;
			// the RENEW's data beat reaches the bus 3 cycles after its first negedge
			while ($realtime < t0 + 10.0 * ((1 << TB_LU) + k) - 30.0) @(negedge clk);
			axi_write(LEASE, lease_w(KP_ID, 2'd2, 4'd1), 4'hF);
			n = (t_renew > t0) ? 1 : 0;
			nr = nr + n; nf = nf + (1 - n);
			checks = checks + 1;
			if (n != (t_whs <= t0 + 10.0 * ((1 << TB_LU) + 1) + 5.0)) begin
				fails = fails + 1; $display("FAIL renew sweep %0d: taken %0d, its beat at %0t, the lease from %0t", k, n, t_whs, t0);
			end
			n = 0; while (win.live_q && n < 3 * (1 << TB_LU)) begin @(negedge clk); n = n + 1; end
			repeat (4) @(negedge clk);
			check($sformatf("renew sweep %0d: held rose once and fell once", k), n_lrise + n_lfall - e0, 2);
			check($sformatf("renew sweep %0d: the SYS line rose once and fell once", k), lm_rises + lm_falls - j, 2);
		end
		$display("     renewed %0d, too late %0d, a RENEW taken in the last live cycle %0d times", nr, nf, n_renew0);
		check("renew sweep: both outcomes, and a RENEW in the last live cycle", {nr > 0, nf > 0, n_renew0 > 0}, 3'b111);

		$display("-- lease: the ends, each with its code: an HPS reset (2), GPO[31] low for one cycle (3) and long (3)");
		lease_take;
		@(negedge clk); hps_rst_n = 0; repeat (3) @(negedge clk); hps_rst_n = 1;
		fw_wait_fall; repeat (2) @(negedge clk);
		lease_rd(v);
		check("an HPS reset: end 2", {v[31], v[13:12]}, {1'b0, 2'd2});
		lease_take;
		@(negedge clk); #2 gpo31 = 0; #10 gpo31 = 1;
		repeat (8) @(negedge clk);
		lease_rd(v);
		check("GPO[31] low for one clk_100m cycle: end 3", {v[31], v[13:12]}, {1'b0, 2'd3});
		lease_take;
		@(negedge clk); gpo31 = 0; repeat (8) @(negedge clk);
		check("GPO[31] low: GPI[22] reads 0 in the ID view", tb_gpi[22], 0);
		for (j = 0; j < 3; j = j + 1) axi_write(LEASE, lease_w(KP_ID, 2'd1, 4'd15), 4'hF);
		lease_rd(v);
		check("GPO[31] held low: no CLAIM taken, idv shown, end 3", {v[31], v[11], v[13:12]}, {1'b0, 1'b1, 2'd3});
		repeat (200) @(negedge clk);
		axi_write(LEASE, lease_w(KP_ID, 2'd1, 4'd15), 4'hF);
		check("GPO[31] held low for 200 cycles more: still none", win.live_q, 0);
		gpo31 = 1; repeat (6) @(negedge clk);
		lease_take;
		check("GPO[31] high again: a CLAIM is taken", win.live_q, 1);

		$display("-- lease: two hosts: another id's RELEASE and RENEW do nothing; one owner; a takeover on purpose");
		lease_rd(v0);
		repeat (2 * 1024) @(negedge clk);
		lc0 = win.lcnt;
		axi_write(LEASE, lease_w(ID_B, 2'd3, 4'd0), 4'hF);
		axi_write(LEASE, lease_w(ID_B, 2'd2, 4'd15), 4'hF);
		lease_rd(v);
		check("B's RELEASE and RENEW: A's lease, owner and epoch kept", {v[31], v[23:16], v[9:0]}, {1'b1, v0[23:16], KP_ID});
		check("B's RENEW did not reload A's counter", win.lcnt < lc0, 1);
		axi_write(LEASE, lease_w(ID_B, 2'd1, 4'd15), 4'hF);
		lease_rd(v);
		check("B's CLAIM over A's lease: refused", {v[31], v[23:16], v[9:0]}, {1'b1, v0[23:16], KP_ID});
		axi_write(LEASE, lease_w(KP_ID, 2'd3, 4'd0), 4'hF);
		wait_quiet;
		// back to back: B first, then A; each reads who won
		axi_write(LEASE, lease_w(ID_B, 2'd1, 4'd15), 4'hF);
		axi_write(LEASE, lease_w(KP_ID, 2'd1, 4'd15), 4'hF);
		lease_rd(v);
		check("two CLAIMs back to back: one owner, the first", {v[31], v[9:0]}, {1'b1, ID_B});
		check("A learns from owner that it lost", v[9:0] != KP_ID, 1);
		e0 = v[23:16];
		// A takes over on purpose: RELEASE with the owner it read, then quiet, then its CLAIM
		axi_write(LEASE, lease_w(v[9:0], 2'd3, 4'd0), 4'hF);
		wait_quiet;
		lease_rd(v);
		check("the takeover's RELEASE with the owner read: held clear, quiet", v[31:30], 2'b01);
		axi_write(LEASE, lease_w(KP_ID, 2'd1, 4'd15), 4'hF);
		lease_rd(v);
		check("the takeover's CLAIM: A owns it, the epoch moved", {v[31], v[9:0], v[23:16]}, {1'b1, KP_ID, e0[7:0] + 8'd1});
		axi_write(LEASE, lease_w(ID_B, 2'd2, 4'd15), 4'hF);
		lease_rd(v);
		check("B's next refresh fails: the owner is not B", v[9:0] != ID_B, 1);

		$display("-- lines: low without the lease whatever the causes; PENDING shows them either way; LINE_EN per entry");
		axi_write(LEASE, lease_w(KP_ID, 2'd3, 4'd0), 4'hF);
		st_req(st_pat(902)); settle;
		if (TB_SEC) begin cd_request(32'h00007E00); settle; end
		axi_read(DIRB + 21'h20, 0, v, dl, beats);
		check("no lease: PENDING shows SYS (and SECTOR)", v, (TB_SEC ? 32'd1 << TB_E_SEC : 0) | 32'd1 << TB_E_SYS);
		check("no lease: every line low", lines, 0);
		rd("OWNED reads 0", OWNEDA, 32'd0);
		lease_take;
		settle;
		check("a lease, LINE_EN 0: every line low", lines, 0);
		rd("PENDING under the lease: SYS's (SECTOR's word cleared at the claim)", DIRB + 21'h20, 32'd1 << TB_E_SYS);
		if (TB_SEC) begin
			sec_wr(21'h10, 32'h1); settle;
			rd("an EN rise raises SECTOR's cause for its standing request", DIRB + 21'h20, v);
		end
		axi_write(LINEN, 32'd1 << TB_E_SYS, 4'hF); settle;
		check("LINE_EN with SYS's bit: SYS's line alone", lines, 3'b010);
		if (TB_SEC) begin
			axi_write(LINEN, 32'd1 << TB_E_SEC, 4'hF); settle;
			check("LINE_EN with SECTOR's bit: SECTOR's line alone", lines, 3'b001);
			axi_write(LINEN, (32'd1 << TB_E_SEC) | (32'd1 << TB_E_SYS), 4'hF); settle;
			check("both bits: both lines", lines, 3'b011);
		end
		axi_write(LINEN, ~((32'd1 << TB_E_SEC) | (32'd1 << TB_E_SYS)), 4'hF); settle;
		check("the other bits: no line", lines, 0);
		axi_write(LINEN, 32'hFFFFFFFF, 4'hF); settle;
		axi_write(LEASE, lease_w(KP_ID, 2'd3, 4'd0), 4'hF);
		rd("LINE_EN reads 0 after the lease ends", LINEN, 32'd0);
		check("the lease ended: every line low", lines, 0);
		lease_take;
		rd("LINE_EN reads 0 after a claim", LINEN, 32'd0);
		if (TB_SEC) begin
			$display("-- lines: a host that skips SECTOR is never woken by it; the block takes nothing from SPI");
			axi_write(LINEN, 32'd1 << TB_E_SYS, 4'hF);
			cause_drain(SYSB); settle;
			check("after the claim: SECTOR's cause clear, its request standing", {win.sec_lvl, sd_rd[1]}, 2'b01);
			@(negedge clk_sys) cd_reset = 1; repeat (4) @(negedge clk_sys); cd_reset = 0;
			cd_request(32'h00007E00); settle;
			check("a new request: SECTOR's cause pending", win.sec_lvl, 1);
			n = 0;
			fork
				begin repeat (slow(400)) @(negedge clk); end
				begin repeat (slow(400)) begin @(negedge clk); if (lines[0]) n = n + 1; end end
			join
			check("SECTOR's request stands, its line never rose", {sd_rd[1], n == 0}, 2'b11);
			check("the slot stays with SPI: no EN", blk_dn[18], 0);
			n0 = cd_frames;
			spi_frame(32'h00007E00);
			wait_frame(n0);
			check_buf("the skipped block's request served over SPI", 32'h00007E00, TB_FRAME);
			st_req(st_pat(903)); settle;
			check("the host's other line works: SYS", lines, 3'b010);
			cause_drain(SYSB); cause_drain(SEC);
		end
		else cause_drain(SYSB);
		axi_write(LEASE, lease_w(KP_ID, 2'd3, 4'd0), 4'hF);
	end
endtask

// The lines are levels. A raise swept across the store-back's cycle: where it meets the clear, the line has
// no low cycle; then the host model, woken only by the level, serves every event across random interleavings.
`ifndef LW_TB_CA2_N
`define LW_TB_CA2_N 10000
`endif
reg     ca_mon = 0;
// The host's drain on SYS: read CAUSE, store it back, read the level (STFLG with SEQ), then wait for the line again.
task ca_drain(inout integer seq_seen, inout integer wakes);
	reg [31:0] c, v;
	integer nb;
	begin
		wakes = wakes + 1;
		axi_read(SYSB, 0, c, dl, nb);
		axi_write(SYSB, c, 4'hF);
		axi_read(SYSB + 21'h04, 0, v, dl, nb);
		seq_seen = v[31:16];
	end
endtask
integer ca_low = 0;
always @(negedge clk) if (ca_mon && !lines[1]) ca_low = ca_low + 1;
task line_cases;
	reg [31:0] v, c;
	integer k, n, same, low, seq_seen, seed, nev, done, gap, wakes, nb;
	realtime tw;
	begin
		$display("-- lines are levels: a raise swept across the store-back's cycle; where it wins, no low cycle");
		lease_up;
		axi_write(LINEN, 32'd1 << TB_E_SYS, 4'hF);
		same = 0;
		// 2.5 ns steps over a window scaled with clk_sys: a step scaled with it, at 12 MHz, could straddle the one
		// alignment where the two edges meet in every trial of a seed
		for (k = 0; k < slow(120); k = k + 1) begin
			st_req(st_pat(1000 + k)); settle;
			ca_low = 0; ca_mon = 1;
			fork
				begin #(k * 2.5); axi_write(SYSB, 32'h1, 4'hF); end
				begin st_req(st_pat(2000 + k)); end
			join
			repeat (slow(60)) @(negedge clk);
			ca_mon = 0; low = ca_low;
			// the capture's edge and the store's edge: both seen on the bus
			if (t_cap == t_wsys) begin
				same = same + 1;
				check($sformatf("raise %0d in the clear's cycle: the cause stands", k), win.sys_lvl, 1);
				check($sformatf("raise %0d in the clear's cycle: the line had no low cycle", k), low, 0);
			end
			cause_drain(SYSB); settle;
		end
		$display("     %0d raises met the clear in its cycle", same);
		check("a raise met the store-back in its cycle", same > 0, 1);

		$display("-- lines are levels: the host, woken by the level alone, serves every event (%0d interleavings)",
		         `LW_TB_CA2_N);
		seed = 11; nev = 0; done = 0; wakes = 0;
		fork
			begin
				while (nev < `LW_TB_CA2_N) begin
					gap = $unsigned($random(seed)) % 16;
					repeat (gap) @(negedge clk_sys);
					st_req(st_pat(5000 + nev));
					nev = nev + 1;
				end
				done = 1;
			end
			begin
				seq_seen = -1;
				while (!done) begin
					@(negedge clk);
					if (lines[1]) ca_drain(seq_seen, wakes);
				end
			end
		join
		repeat (slow(200)) @(negedge clk);
		while (lines[1]) ca_drain(seq_seen, wakes);
		axi_read(SYSB + 21'h04, 0, v, dl, nb);
		$display("     %0d events, %0d wakes", nev, wakes);
		check("every event served: the last SEQ the host read is the block's", seq_seen, v[31:16]);
		check("quiescent: the cause clear, the line low", {win.sys_lvl, lines[1]}, 2'b00);
		lease_off;
	end
endtask

// Look before identifying. A second host model reads the GPO value as left and one GPI sample; a held
// lease makes it store nothing. It waits the lease out, or takes over on purpose by identifying; a stock start,
// GPO = 0 first, ends a live host's lease too.
task look_cases;
	reg [31:0] v, v0, p, dr, oc;
	integer n, nw0;
	realtime t0;
	begin
		$display("-- the start order: a second host looks at GPO and GPI[22] first, sees the lease and stores nothing");
		lease_up;
		if (TB_SEC) sec_wr(21'h10, 32'h1);
		axi_write(OSD + 21'h1000, 32'h1, 4'hF);
		axi_write(OSD, 32'h5A5AA5A5, 4'hF);
`ifdef MISTER_LW_LANE
		lane_count(p, dr);
		io_enable = 1;                                      // the link busy: frames queue
		f_set(0, 16'h001E, 2); fw[0] = 16'h1601; fw[1] = 16'h1602;
		lane_send(1); lane_send(1);
		repeat (slow(100)) @(negedge clk);
`endif
		lease_rd(v0);
		nw0 = n_lwwr;
		check("second host: the GPO value it reads is the I/O view, and GPI says a lease is held", look_held(), 1);
		repeat (50) @(negedge clk);                         // it stops: no store, no GPO change
		check("second host: stored nothing", n_lwwr - nw0, 0);
		lease_rd(v);
		check("the first host's lease, owner and epoch as they were", {v[31], v[23:16], v[9:0]},
		      {1'b1, v0[23:16], KP_ID});
		if (TB_SEC) rd("the first host's SECTOR binding as it was", SEC + 21'h10, 32'h1);
		rd("the first host's OSD CTRL as it was", OSD + 21'h1000, 32'h1);
		check("the first host's image word as it was", hdmi_osd.lw_buf0[0], 8'hA5);
`ifdef MISTER_LW_LANE
		lane_count(v, dr);
		check("the first host's frames still queued", v, p);
		io_enable = 0;
		lane_wait(p + 2);
`endif
		n = kp_lost;
		kp_renew;
		check("the first host's next refresh succeeds", kp_lost - n, 0);

		$display("-- the start order: the first host stops; GPI[22] falls within the TTL; the second host then starts");
		lease_short(4'd2);
		t0 = $realtime;
		n = 0; while (look_held() && n < 3 * (1 << TB_LU)) begin @(negedge clk); n = n + 1; end
		check("GPI[22] fell within the TTL", ($realtime - t0) < 10.0 * (2.0 * (1 << TB_LU) + 10), 1);
		identify(20);
		claim_w(ID_B, 4'd15);
		lease_rd(v);
		check("the second host claims", {v[31], v[9:0]}, {1'b1, ID_B});
		axi_write(SCR, 32'h0B0B0B0B, 4'hF);
		rd("the second host's store lands", SCR, dg(32'h0B0B0B0B));
		axi_write(LEASE, lease_w(ID_B, 2'd3, 4'd0), 4'hF);

		$display("-- the start order: a takeover on purpose: the second host identifies at once; end 3; the first stops");
		lease_take;
		identify(4);
		lease_rd(v);
		check("the identification ended the lease: end 3", {v[31], v[13:12]}, {1'b0, 2'd3});
		axi_write(LEASE, lease_w(KP_ID, 2'd2, 4'd15), 4'hF);
		lease_rd(v);
		check("the first host's refresh fails: not held", v[31], 0);

		$display("-- the start order: a stock start (GPO = 0 first) beside a live host: its lease ends, its stores drop");
		lease_take;
		axi_write(OSD + 21'h1000, 32'h2, 4'hF);
		@(negedge clk); gpo31 = 0;                            // the stock first act: GPO = 0, then the ID view
		repeat (40) @(negedge clk);
		axi_write(OSD + 21'h1000, 32'h1, 4'hF);              // the live host goes on storing
		gpo31 = 1; repeat (6) @(negedge clk);                 // the stock program speaks in the I/O view
		axi_write(SCR, 32'h0C0C0C0C, 4'hF);
		lease_rd(v);
		check("a stock start: the lease ended, end 3, stale set", {v[31], v[13:12], v[29]}, {1'b0, 2'd3, 1'b1});
		rd("a stock start: the live host's stores dropped (OSD CTRL cleared with the lease)", OSD + 21'h1000, 32'd0);
		rd("a stock start: SCRATCH not stored", SCR, dg(32'h0B0B0B0B));
		axi_write(LEASE, lease_w(KP_ID, 2'd2, 4'd15), 4'hF);
		lease_rd(v);
		check("a stock start: the live host's next refresh fails", v[31], 0);
		if (TB_SEC) check("a stock start: SECTOR's EN gone", blk_dn[18], 0);
		$display("     GPI[22] compared with live_q in %0d cycles, %0d of them held", gpi22_n, gpi22_on);
	end
endtask

`ifdef MISTER_LW_SECTOR_SLOT
// SECTOR when its lease runs out, in each state of the primary slot, with clk_sys running or
// stopped across the expiry. The ack completes, en_sys drops, the slot is SPI's again, and an SPI frame serves it.
integer sx_fs = -1;     // the feeder's state in the cycle the lease ended
always @(negedge win.live_q) sx_fs = win.sec.sector.fs;
task sec_expiry(input integer sc, input stopped);
	reg [31:0] lba, v;
	integer nst, n, t;
	string  w;
	begin
		case (sc)
			0: w = "idle";
			1: w = "a request unanswered";
			2: w = "mid-BEGIN";
			3: w = "mid-DATA";
			4: w = "mid zero-fill";
			default: w = "FIFO full";
		endcase
		w = {w, stopped ? ", clk_sys stopped across it" : ", clk_sys running"};
		$display("-- SECTOR at the lease's expiry: %s", w);
		lba = 32'h00007A00 + {sc[3:0], stopped, 4'd0};
		lease_short((sc == 5) ? 4'd6 : 4'd3);
		sec_wr(21'h10, 32'h1); settle;
		check({w, ": bound"}, blk_dn[18], 1);
		n0 = cd_frames; nst = 0;
		if (sc >= 1) begin cd_request(lba); settle; cause_drain(SEC); end
		if (sc >= 2) begin sec_wr(21'h0C, lba); settle; end
		if (sc == 3) begin frame_data(lba, 0, 294); nst = 294; end
		if (sc == 4) begin
			frame_data(lba, 0, 20); nst = 20;
			while (win.lcnt > 300) @(negedge clk);
			sec_wr(21'h10, 32'h3);                          // CTRL.ABORT: the fill runs across the expiry
		end
		if (sc == 5) begin
			sys_run = 0; repeat (20) @(negedge clk);
			frame_data(lba, 0, TB_FRAME);
			axi_read(SEC + 21'h18, 0, v, dl, beats); nst = v[9:0];
			check({w, ": the FIFO filled, stores dropped"}, v[25:16] != 0, 1);
			while (win.lcnt > 200) @(negedge clk);
			if (!stopped) sys_run = 1;
		end
		if (stopped && sc != 5) begin sys_run = 0; repeat (20) @(negedge clk); end
		n = 0; while (win.live_q && n < 4 * (1 << TB_LU)) begin @(negedge clk); n = n + 1; end
		lease_rd(v);
		check({w, ": the lease ran out"}, {v[31], v[13:12]}, {1'b0, 2'd1});
		if (sc == 4 && !stopped) check({w, ": the fill was running at the expiry"}, sx_fs, 3);
		if (stopped) begin
			repeat (200) @(negedge clk);
			check({w, ": EN cleared on clk_100m while clk_sys is stopped"}, win.sec.sector.en_lwx, 0);
			sys_run = 1;
		end
		if (sc >= 2) begin
			wait_frame(n0);
			check_buf({w, ": the frame in flight completes, filled from its first missing word"}, lba, nst);
		end
		t = 0; while ((blk_dn[18] || blk_dn[17]) && t < slow(5000)) begin @(negedge clk); t = t + 1; end
		check({w, ": the ack done, en_sys dropped"}, blk_dn[18:17], 2'b00);
		if (sc != 1) begin cd_request(lba + 1); lba = lba + 1; end
		settle;
		check({w, ": the request reaches hps_io"}, sd_rd[1], 1);
		n0 = cd_frames;
		spi_frame(lba);
		wait_frame(n0);
		check_buf({w, ": an SPI frame serves it"}, lba, TB_FRAME);
		lease_take; cause_drain(SEC); axi_write(LEASE, lease_w(KP_ID, 2'd3, 4'd0), 4'hF);
	end
endtask
task sec_expiry_cases;
	integer sc;
	begin
		for (sc = 0; sc < 6; sc = sc + 1) sec_expiry(sc, 1'b0);
		sec_expiry(1, 1'b1);
		sec_expiry(3, 1'b1);
		sec_expiry(5, 1'b1);
	end
endtask
`endif

`ifdef MISTER_LW_LANE
// The lane when its lease runs out, with a frame open, three queued and one held mid-play by the link's
// wait: the one in flight completes, the queued never strobe, the open one is abandoned, a link word after is
// intact. A capture frame cut mid-play: its replies land after the lease and are discarded.
task lane_expiry_cases;
	reg [31:0] p, dr, rs;
	integer b0, k;
	begin
		$display("-- LANE at the lease's expiry: one frame mid-play, three queued, one open");
		f_set(0, 16'h001E, 4);
		for (k = 0; k < 4; k = k + 1) fw[k] = 16'h7100 + k;
		trk_clear; link_send; trk_save;                    // the reference for the link frame sent after
		lease_short(4'd3);
		lane_count(p, dr);
		ioctl_wait = 1;
		lane_send(1);
		repeat (slow(200)) @(negedge clk);
		check("expiry: the first frame holds after its first strobe", {lane_own, bus_n[7:0] - t_ref[0][7:0]}, {1'b1, 8'd1});
		for (k = 0; k < 3; k = k + 1) begin fw[0] = 16'h7200 + k; lane_send(1); end
		fw[0] = 16'h7300; lane_send(0);                    // open: BEGIN and its words, no END
		fw[0] = 16'h7100;
		b0 = bus_n;
		wait_lapse(3);
		check("expiry: the open frame abandoned", win.lane.lane.wspec, win.lane.lane.wcom);
		@(negedge clk_sys); ioctl_wait = 0;
		lane_wait(p + 1);
		repeat (slow(400)) @(negedge clk);
		check("expiry: the frame in flight completed its strobes", bus_n - b0, 4);
		check("expiry: the queued frames were popped, none strobed", {lane_own, win.lane.lane.rptr == win.lane.lane.wcom},
		      2'b01);
		lane_count(v_l, dr);
		check("expiry: PLAYED counts the one frame", v_l, p + 1);
		trk_clear; link_send;
		trk_cmp("expiry: a link frame after");

		$display("-- LANE at the lease's expiry: a capture frame cut mid-play; its replies are discarded");
		lease_short(4'd3);
		lane_count(p, dr);
		lane_rstat(rs); rd0 = rs[31:16];
		f_set(0, 16'h0029, 8);
		for (k = 0; k < 8; k = k + 1) fw[k] = 16'h0000;
		ioctl_wait = 1;
		lane_send_c(1, 1'b1);
		repeat (slow(200)) @(negedge clk);
		wait_lapse(3);
		rw0 = win.lane.lane.rwp;
		lease_rd(rs);
		check("cut capture, held by the link's wait: not quiet", rs[30], 0);
		axi_write(LEASE, lease_w(KP_ID, 2'd1, 4'd15), 4'hF);
		check("cut capture, held: a CLAIM is refused", win.live_q, 0);
		@(negedge clk_sys); ioctl_wait = 0;
		lane_wait(p + 1);
		repeat (slow(400)) @(negedge clk);
		check("cut capture: its replies landed after the lease", win.lane.lane.rwp != rw0, 1);
		lane_rstat(rs);
		check("cut capture: RSTAT shows none waiting", rs[15:0], 0);
		wait_quiet;
		lease_rd(rs);
		check("cut capture: quiet returns once the player is at rest", rs[30], 1);
		lease_up;
		check("cut capture: a CLAIM is then taken", win.live_q, 1);
		lane_rstat(rs);
		check("cut capture: RSTAT under the new lease: none waiting, no drop counted", rs, {rd0, 16'd0});
	end
endtask
reg [31:0] v_l;
reg  [8:0] rw0;
reg [15:0] rd0;
`endif

// ---- The lease links: a claim waits for every linked half's rest; stores wait for its acknowledge ----
// A CLAIM stored again and again, as fast as the bus allows, until one is taken or max stores were made.
task claim_hammer(input [9:0] id, input integer max, output integer tries);
	begin
		tries = 0;
		while (!win.live_q && tries < max) begin axi_write(LEASE, lease_w(id, 2'd1, 4'd15), 4'hF); tries = tries + 1; end
	end
endtask
// The link states as the bench reads them, 7 where a block is not built.
`ifdef MISTER_LW_SECTOR_SLOT
wire [2:0] lk_sec = win.sec.sector.fence.st;
`else
wire [2:0] lk_sec = 3'd7;
`endif
`ifdef MISTER_LW_LANE
wire [2:0] lk_lane = win.lane.lane.fence.st;
`else
wire [2:0] lk_lane = 3'd7;
`endif
// A primary request the core stands on: a fresh one if none is raised.
task sec_stand(input [31:0] lba);
	begin
		if (!cd_req) cd_request(lba);
		settle;
	end
endtask
// The SECTOR slot after a case: the core's request served over SPI if it still stands, and no ring left.
task sec_spi_clear;
	integer t;
	begin
		t = 0; while ((blk_dn[18] || blk_dn[17]) && t < slow(20000)) begin @(negedge clk); t = t + 1; end
		if (cd_req && !cd_ack && blk_dn[18:17] == 0) begin n0 = cd_frames; spi_frame(cd_lba); wait_frame(n0); end
	end
endtask

// RELEASE with a lane frame held mid-play and two queued, and a SECTOR frame begun with no data; then a
// CLAIM stored as fast as the bus allows. None is taken while a half is off rest (the claim monitor, and quiet
// read 0 while the frame is held); the one taken finds REQ 0; the frame in flight completes, the queued ones
// never strobe, the SECTOR frame is zero-filled.
task ls6;
	reg [31:0] v, p, dr;
	integer tries, b0, c0, n1, k;
	begin
		$display("-- lease links: RELEASE, then CLAIMs as fast as the bus allows: none before every half is at rest");
		lease_up;
		c0 = n_claims;
`ifdef MISTER_LW_LANE
		lane_count(p, dr);
		f_set(0, 16'h001E, 4);
		for (k = 0; k < 4; k = k + 1) fw[k] = 16'h6600 + k;
		ioctl_wait = 1;
		lane_send(1);
		repeat (slow(200)) @(negedge clk);
		fw[0] = 16'h6700; lane_send(1); fw[0] = 16'h6800; lane_send(1);
		b0 = bus_n;
`endif
		if (TB_SEC) begin
			sec_wr(21'h10, 32'h1); sec_stand(32'h00006600); cause_drain(SEC);
			n1 = cd_frames;
			sec_wr(21'h0C, cd_lba);
			// RELEASE once the half has popped the BEGIN and opened its frame (it would discard one still queued)
`ifdef MISTER_LW_SECTOR_SLOT
			k = 0; while ((win.sec.sector.fs == 3'd0 || win.sec.sector.fs == 3'd4) && k < slow(2000)) begin
				@(negedge clk); k = k + 1; end
			check("the BEGIN popped and its frame open before the RELEASE", k < slow(2000), 1);
`endif
		end
		kp_on = 0;
		axi_write(LEASE, lease_w(KP_ID, 2'd3, 4'd0), 4'hF);
`ifdef MISTER_LW_LANE
		claim_hammer(KP_ID, 60, tries);
		lease_rd(v);
		check("a lane frame held by the link's wait: 60 CLAIMs refused, quiet 0", {win.live_q, v[30]}, 2'b00);
		@(negedge clk_sys); ioctl_wait = 0;
`endif
		claim_hammer(KP_ID, slow(4000), tries);
		$display("     taken after %0d CLAIM stores", tries);
		check("a CLAIM is taken once every half is at rest", win.live_q, 1);
		check("taken claims, every half at rest", n_offrest, 0);
		if (TB_SEC) begin
			rd("the claim finds REQ 0", SEC + 21'h04, 32'd0);
			wait_frame(n1);
			check_buf("the SECTOR frame begun with no data: zero-filled", cd_lba, 0);
		end
`ifdef MISTER_LW_LANE
		repeat (slow(400)) @(negedge clk);
		lane_count(v, dr);
		check("the frame in flight completed; the queued two were popped", v, p + 1);
		check("its four remaining strobes, and no other", bus_n - b0, 4);
		check("the player idle", {lane_own, lane_hold}, 2'b00);
`endif
		wait_open;
		lease_rd(v); kp_epoch = v[23:16];
		axi_write(LINEN, 32'hFFFFFFFF, 4'hF); cause_drain(SYSB); kp_t = $realtime; kp_on = 1;
		if (TB_SEC) begin cause_drain(SEC); sec_wr(21'h10, 32'h0); sec_spi_clear; end
	end
endtask

// A predecessor read and cleared the causes and died with nothing served; the next CLAIM reloads them.
task ls10;
	reg [31:0] v;
	begin
		$display("-- lease links: at a claim SYS raises STATUS and SECTOR's word is clear until EN rises over a request");
		lease_short(4'd1);
		st_req(st_pat(950)); settle;
		cause_drain(SYSB);
		if (TB_SEC) begin sec_stand(32'h00006A00); if (win.sec_lvl) cause_drain(SEC); end
		wait_lapse(1);
		claim_w(KP_ID, 4'd15); wait_open;
		rd("after the claim: SYS CAUSE shows STATUS", SYSB, 32'h1);
		check("LINE_EN 0: no line", lines, 0);
		axi_write(LINEN, 32'd1 << TB_E_SYS, 4'hF); settle;
		check("SYS's LINE_EN bit: its line rises", lines[1], 1);
		if (TB_SEC) begin
			rd("after the claim: SECTOR CAUSE clear, its request standing", SEC, 32'd0);
			check("the request stands", sd_rd[1] | cd_req, 1);
			sec_wr(21'h10, 32'h1); settle;
			rd("EN rises over the standing request: SECTOR CAUSE", SEC, 32'h1);
			sec_wr(21'h10, 32'h3);                 // ABORT: nothing begun, so nothing to fill
			cause_drain(SEC); sec_wr(21'h10, 32'h0);
		end
		cause_drain(SYSB);
		axi_write(LEASE, lease_w(KP_ID, 2'd3, 4'd0), 4'hF);
		if (TB_SEC) sec_spi_clear;
	end
endtask

// A CLAIM followed at once by work, without waiting for OPEN, after a quiet return, after a lapse and
// after a takeover. Every lane frame plays whole or was dropped at the gate (stale set); none is popped unplayed.
// A SECTOR EN store lands or is dropped with stale. A host that waits for OPEN loses nothing.
task ls11;
	reg [31:0] v, en;
	integer k, j, c0, p0, b0, s0;
	string  w;
	begin
		for (k = 0; k < 3; k = k + 1) begin
			w = (k == 0) ? "after a quiet return" : (k == 1) ? "after a lapse" : "after a takeover";
			$display("-- lease links: a CLAIM followed at once by work, %s", w);
			kp_on = 0;
			if (k == 1) begin lease_short(4'd1); wait_lapse(1); end
			if (k == 2) begin
				lease_rd(v); if (v[31]) axi_write(LEASE, lease_w(v[9:0], 2'd3, 4'd0), 4'hF);
				claim_w(ID_B, 4'd15);
				lease_rd(v); axi_write(LEASE, lease_w(v[9:0], 2'd3, 4'd0), 4'hF);   // the takeover's RELEASE
			end
			if (k == 0) begin lease_rd(v); if (v[31]) axi_write(LEASE, lease_w(v[9:0], 2'd3, 4'd0), 4'hF); end
			wait_quiet;
`ifdef MISTER_LW_LANE
			c0 = n_commit; p0 = n_play; b0 = bus_n;
`endif
			axi_write(LEASE, lease_w(KP_ID, 2'd1, 4'd15), 4'hF);
			check({w, ": the CLAIM is taken"}, win.live_q, 1);
`ifdef MISTER_LW_LANE
			f_set(0, 16'h001E, 2);
			for (j = 0; j < 4; j = j + 1) begin fw[0] = 16'h1100 + j; fw[1] = 16'h1180 + j; lane_send(1); end
`endif
			if (TB_SEC) sec_wr(21'h10, 32'h1);
			wait_open;
			lease_rd(v);
			if (TB_SEC) begin
				axi_read(SEC + 21'h10, 0, en, dl, beats);
				check({w, ": the EN store landed, or was dropped with stale"}, en[0] | v[29], 1);
			end
`ifdef MISTER_LW_LANE
			$display("     %s: %0d of 4 frames committed", w, n_commit - c0);
			check({w, ": a frame not committed was dropped with stale"}, (n_commit - c0 == 4) | v[29], 1);
			for (j = 0; j < 2; j = j + 1) begin fw[0] = 16'h1200 + j; fw[1] = 16'h1280 + j; lane_send(1); end
			s0 = 0; while (n_play - p0 < n_commit - c0 && s0 < slow(8000)) begin @(negedge clk); s0 = s0 + 1; end
			repeat (slow(100)) @(negedge clk);
			check({w, ": every committed frame played, none popped unplayed"}, n_play - p0, n_commit - c0);
			check({w, ": the two frames stored after OPEN committed"}, n_commit - c0 >= 2, 1);
			check({w, ": three strobes per frame played"}, bus_n - b0, 3 * (n_play - p0));
`endif
			if (TB_SEC) sec_wr(21'h10, 32'h0);
		end
		axi_write(LEASE, lease_w(KP_ID, 2'd3, 4'd0), 4'hF);
	end
endtask

// A host dies with a link select asserted, lane frames queued behind it and SECTOR bound; a stock
// successor starts (GPO = 0: the selects drop and the ID view is selected), identifies the core in the ID view
// and sends link words. v: 0 the selects and bit 31 together; 1 bit 31 a clk_sys cycle first; 2 the selects first;
// 3 with clk_sys stopped across the GPO store.
task ls12_view(input integer v);
	reg [31:0] p, dr, lv;
	integer k, n1, pl0;
	string  w;
	begin
		w = (v == 0) ? "the selects and bit 31 together" : (v == 1) ? "bit 31 a cycle first" :
		    (v == 2) ? "the selects a cycle first" : "clk_sys stopped across the GPO store";
		$display("-- lease links, the view: a dead host's select and queued frames; a stock successor (%s)", w);
`ifdef MISTER_LW_LANE
		f_set(0, 16'h001E, 2); fw[0] = 16'h3E01; fw[1] = 16'h3E02;
		trk_clear; link_send; trk_save;                    // the successor's own link frame, the reference
		f_set(0, 16'h001E, 2); fw[0] = 16'h0000; fw[1] = 16'h0000; link_send;
`endif
		lease_up;
		if (TB_SEC) sec_wr(21'h10, 32'h1);
`ifdef MISTER_LW_LANE
		@(negedge clk_sys); io_enable = 1;                 // the dead host's select
		f_set(0, 16'h001E, 2);
		for (k = 0; k < 3; k = k + 1) begin fw[0] = 16'h3D00 + k; fw[1] = 16'h3D80 + k; lane_send(1); end
		repeat (slow(100)) @(negedge clk);
		pl0 = n_play;
`endif
		kp_on = 0;                                         // the host is dead
		if (v == 3) begin
			@(negedge clk_sys); sys_run = 0; repeat (20) @(negedge clk);
			gpo31 = 0; io_enable = 0; repeat (40) @(negedge clk); sys_run = 1;
		end
		else begin
			@(negedge clk_sys);
			gpo31 = 0;
			if (v == 2) io_enable = 0;
			else begin repeat (v == 0 ? 1 : 2) @(negedge clk_sys); io_enable = 0; end
		end
		repeat (slow(2000)) @(negedge clk);                // the ID view: start-up and identification
		gpo31 = 1; repeat (2) @(negedge clk_sys);
`ifdef MISTER_LW_LANE
		trk_clear; f_set(0, 16'h001E, 2); fw[0] = 16'h3E01; fw[1] = 16'h3E02; link_send;
		repeat (slow(400)) @(negedge clk);
		trk_cmp({w, ": the successor's link words intact, no queued frame strobed"});
		check({w, ": no queued frame played"}, n_play - pl0, 0);
`endif
		lease_rd(lv);
		check({w, ": the lease ended by the view, end 3"}, {lv[31], lv[13:12]}, {1'b0, 2'd3});
		if (TB_SEC) begin
			check({w, ": SECTOR's slot back on SPI"}, blk_dn[18], 0);
			sec_stand(32'h00006C00 + v);
			n1 = cd_frames; spi_frame(cd_lba); wait_frame(n1);
			check_buf({w, ": an SPI frame serves the slot"}, cd_lba, TB_FRAME);
		end
	end
endtask
task ls12;
	reg [31:0] p, dr, lv;
	integer v, pl0, k;
	begin
		for (v = 0; v < 4; v = v + 1) ls12_view(v);
`ifdef MISTER_LW_LANE
		$display("-- lease links, the view: a successor that never leaves the I/O view (recorded)");
		lease_short(4'd2);
		@(negedge clk_sys); io_enable = 1;
		f_set(0, 16'h001E, 2);
		for (k = 0; k < 2; k = k + 1) begin fw[0] = 16'h3F00 + k; fw[1] = 16'h3F80 + k; lane_send(1); end
		pl0 = n_play;
		@(negedge clk_sys); io_enable = 0;                 // the successor drops the select, stays in the I/O view
		f_set(0, 16'h0029, 0); link_send;
		wait_lapse(2);
		$display("     %0d queued frames played in the successor's gaps before the lease ran out", n_play - pl0);
		check("recorded: the queued frames played while the lease lasted", n_play - pl0, 2);
		lane_count(p, dr);
		repeat (slow(400)) @(negedge clk);
		lane_count(lv, dr);
		check("none after the lease", lv, p);
`endif
	end
endtask

// A lease that ends inside the rise. The lease is ended by RELEASE, by GPO[31] low and by an HPS reset,
// k clk_100m cycles after the CLAIM's store took effect, k from the claim's own cycle to twenty cycles past the
// last OPEN bit. Then CLAIMs as fast as the bus allows; the one taken is followed at once by lane frames and a
// SECTOR EN and BEGIN. The link checker, the claim monitor, and: every lane frame plays or was dropped with stale;
// the BEGIN is served or its request stands.
integer l15_rise = 0, l15_open = 0;
reg     l15_on = 0;
always @(negedge win.live_q) if (l15_on) begin
	if (lk_sec == 3'd2 || lk_lane == 3'd2) l15_rise = l15_rise + 1;
	if (lk_sec == 3'd3 || lk_lane == 3'd3) l15_open = l15_open + 1;
end
task ls15;
	reg [31:0] v;
	integer kind, k, kmin, past, tries, c0, p0, n1, s0, bad_p, bad_s, nit, r0, o0;
	realtime tr;
	string  w;
	begin
		for (kind = 0; kind < 3; kind = kind + 1) begin
			w = (kind == 0) ? "RELEASE" : (kind == 1) ? "GPO[31] low" : "an HPS reset";
			$display("-- lease links: a lease ended inside its rise by %s, in every cycle to twenty past OPEN", w);
			kmin = (kind == 0) ? 0 : -3;
			past = 0; bad_p = 0; bad_s = 0; nit = 0; r0 = l15_rise; o0 = l15_open;
			for (k = kmin; past < 20 && k < slow(400); k = k + 1) begin
				kp_on = 0;
				lease_rd(v); if (v[31]) axi_write(LEASE, lease_w(v[9:0], 2'd3, 4'd0), 4'hF);
				wait_quiet;
				l15_on = 1;
				if (kind == 0) begin
					axi_write(LEASE, lease_w(KP_ID, 2'd1, 4'd15), 4'hF);
					tr = t_lrise;
					while ($realtime < tr + 10.0 * k - 30.0) @(negedge clk);
					axi_write(LEASE, lease_w(KP_ID, 2'd3, 4'd0), 4'hF);
				end
				else fork
					axi_write(LEASE, lease_w(KP_ID, 2'd1, 4'd15), 4'hF);
					begin
						// the store takes effect 3 cycles after its first negedge; the end lands about 3 later
						repeat (k + 3 > 0 ? k + 3 : 0) @(negedge clk);
						if (kind == 1) begin #2 gpo31 = 0; #10 gpo31 = 1; end   // off the edge the GPI monitor samples
						else begin hps_rst_n = 0; repeat (3) @(negedge clk); hps_rst_n = 1; end
					end
				join
				if (kind == 2) begin fw_wait_fall; repeat (2) @(negedge clk); end
				repeat (8) @(negedge clk);
				l15_on = 0;
				past = l15_open - o0;
				if (win.live_q) axi_write(LEASE, lease_w(KP_ID, 2'd3, 4'd0), 4'hF);
				// the next claim, and work at once
`ifdef MISTER_LW_LANE
				c0 = n_commit; p0 = n_play;
`endif
				if (TB_SEC) sec_stand(32'h00007500 + k[7:0]);
				claim_hammer(KP_ID, slow(4000), tries);
`ifdef MISTER_LW_LANE
				f_set(0, 16'h001E, 2); fw[0] = 16'h5100 + k; fw[1] = 16'h5180 + k; lane_send(1); lane_send(1);
`endif
				if (TB_SEC) begin
					n1 = cd_frames;
					sec_wr(21'h10, 32'h1);
					sec_wr(21'h0C, cd_lba);
					sec_wr(21'h10, 32'h3);                  // ABORT: the frame, if served, is filled
				end
				wait_open;
`ifdef MISTER_LW_LANE
				s0 = 0; while (n_play - p0 < n_commit - c0 && s0 < slow(4000)) begin @(negedge clk); s0 = s0 + 1; end
				lease_rd(v);
				if (n_play - p0 != n_commit - c0 || (n_commit - c0 != 2 && !v[29])) bad_p = bad_p + 1;
`endif
				if (TB_SEC) begin
					s0 = 0; while (cd_frames == n1 && s0 < slow(4000)) begin @(negedge clk); s0 = s0 + 1; end
					sec_wr(21'h10, 32'h0);
					axi_write(LEASE, lease_w(KP_ID, 2'd3, 4'd0), 4'hF);
					s0 = 0; while ((blk_dn[18] || blk_dn[17]) && s0 < slow(4000)) begin @(negedge clk); s0 = s0 + 1; end
					settle;
					if (cd_frames == n1 && !sd_rd[1]) bad_s = bad_s + 1;   // neither served nor standing for SPI
					sec_spi_clear;
				end
				else axi_write(LEASE, lease_w(KP_ID, 2'd3, 4'd0), 4'hF);
				nit = nit + 1;
			end
			$display("     %0d ends, %0d with a link in RISE and %0d in OPEN at the end", nit, l15_rise - r0, l15_open - o0);
			if (kind != 0) check({w, ": ends inside the rise were reached"}, l15_rise - r0 > 0, 1);
			check({w, ": ends in OPEN, twenty"}, past >= 20, 1);
			check({w, ": every lane frame played or dropped with stale"}, bad_p, 0);
			check({w, ": every BEGIN served, or its request standing for SPI"}, bad_s, 0);
		end
		check("lease links: CLAIMs taken with a half off rest", n_offrest, 0);

		$display("-- lease links: clk_sys stopped from before a claim to after its end: the link waits in RISE, OWNED set");
		lease_rd(v); if (v[31]) axi_write(LEASE, lease_w(v[9:0], 2'd3, 4'd0), 4'hF);
		wait_quiet;
		sys_run = 0; repeat (20) @(negedge clk);
		axi_write(LEASE, lease_w(KP_ID, 2'd1, 4'd15), 4'hF);
		repeat (20) @(negedge clk);
		axi_write(LEASE, lease_w(KP_ID, 2'd3, 4'd0), 4'hF);
		repeat (40) @(negedge clk);
		if (TB_SEC) check("stopped: SECTOR's link in RISE", lk_sec, 3'd2);
		if (TB_LANE) check("stopped: the lane's link in RISE", lk_lane, 3'd2);
		rd("stopped: OWNED shows the linked entries", OWNEDA, TB_LINKED);
		lease_rd(v);
		check("stopped: not quiet", v[30], TB_LINKED == 0);
		claim_hammer(KP_ID, 8, tries);
		check("stopped: no CLAIM taken", win.live_q, 0);
		sys_run = 1;
		wait_quiet;
		rd("restarted: the return completes, OWNED clear", OWNEDA, 32'd0);
		claim_w(KP_ID, 4'd15);
		check("restarted: a CLAIM is taken", win.live_q, 1);
		axi_write(LEASE, lease_w(KP_ID, 2'd3, 4'd0), 4'hF);
	end
endtask

// One link's acknowledge forced high across a claim (a half that never answers): that entry never opens,
// the others do and work, its requests are served over SPI, and with the force gone a late bind works. Forced low
// after a lease (a half that cannot return): quiet stays 0, OWNED names it alone, no CLAIM is taken, SPI works.
task ls17;
	reg [31:0] v, p, dr;
	integer tries, n1;
	begin
`ifdef MISTER_LW_SECTOR_SLOT
		$display("-- lease links: a SECTOR half that never answers costs only SECTOR");
		kp_on = 0;
		lease_rd(v); if (v[31]) axi_write(LEASE, lease_w(v[9:0], 2'd3, 4'd0), 4'hF);
		wait_quiet;
		fc_off = 1;
		force win.sec.sector.fence.ack_lwx = 1'b1;
		claim_w(KP_ID, 4'd15);
		repeat (slow(600)) @(negedge clk);
		rd("OPEN: every entry but SECTOR's", OPENA, ((32'd1 << em_n) - 1) & ~(32'd1 << TB_E_SEC));
		sec_wr(21'h10, 32'h1);
		rd("a store to SECTOR is dropped", SEC + 21'h10, 32'd0);
`ifdef MISTER_LW_LANE
		lane_count(p, dr);
		f_set(0, 16'h001E, 2); fw[0] = 16'h1701; fw[1] = 16'h1702; lane_send(1);
		lane_wait(p + 1);
`endif
		sec_stand(32'h00007700);
		n1 = cd_frames; spi_frame(cd_lba); wait_frame(n1);
		check_buf("the unanswered block's request served over SPI", cd_lba, TB_FRAME);
		release win.sec.sector.fence.ack_lwx;
		wait_open;
		rd("the force removed under the same lease: OPEN every entry", OPENA, (32'd1 << em_n) - 1);
		sec_wr(21'h10, 32'h1);
		rd("a late bind works: EN", SEC + 21'h10, 32'h1);
		sec_wr(21'h10, 32'h0);
		axi_write(LEASE, lease_w(KP_ID, 2'd3, 4'd0), 4'hF);

		$display("-- lease links: a SECTOR half that cannot return holds the shared gate (recorded)");
		repeat (slow(200)) @(negedge clk);
		force win.sec.sector.fence.ack_lwx = 1'b0;
		repeat (slow(200)) @(negedge clk);
		lease_rd(v);
		check("not quiet", v[30], 0);
		rd("OWNED names SECTOR alone", OWNEDA, 32'd1 << TB_E_SEC);
		claim_hammer(KP_ID, 10, tries);
		claim_hammer(ID_B, 10, tries);
		check("no host's CLAIM is taken", win.live_q, 0);
		sec_stand(32'h00007701);
		n1 = cd_frames; spi_frame(cd_lba); wait_frame(n1);
		check_buf("SPI works", cd_lba, TB_FRAME);
		release win.sec.sector.fence.ack_lwx;
		claim_w(KP_ID, 4'd15);
		check("the force removed: a CLAIM is taken", win.live_q, 1);
		fc_off = 0;
`ifdef MISTER_LW_LANE
		$display("-- lease links: a host that skips SECTOR waits on the lane's OPEN bit alone");
		tries = 0;
		axi_read(OPENA, 0, v, dl, beats);
		while (!v[TB_E_LANE] && tries < 100) begin tries = tries + 1; axi_read(OPENA, 0, v, dl, beats); end
		lane_count(p, dr);
		f_set(0, 16'h001E, 2); fw[0] = 16'h1703; fw[1] = 16'h1704; lane_send(1);
		lane_wait(p + 1);
`endif
		axi_write(LEASE, lease_w(KP_ID, 2'd3, 4'd0), 4'hF);
`endif
	end
endtask

// Random values in every clk_sys register of the lane, SECTOR, PAD, SYS and osd halves, and in each lease
// link's clk_sys registers. Pass 1 mid-lease, then RELEASE and CLAIMs as fast as the bus allows: none taken
// before every half is at rest (the claim monitor), a stock SPI transfer before the claim, and a frame of each
// block correct under the new lease. Pass 2 during the return: the links converge and a later lease works.
task scramble;
	begin
		@(negedge clk_sys);
		win.pad_x.lwtg_a = $random; win.pad_x.d_v = $random; win.pad_x.cap.lwq_d = {$random, $random};
		win.pad_x.sync_r.lws0_q = $random; win.pad_x.sync_r.lws1_q = $random;
		win.sys_status.stflg_lwgray = $random;
		hdmi_osd.lwhr_s0 = $random; hdmi_osd.lwhr_s1 = $random; vga_osd.lwhr_s0 = $random; vga_osd.lwhr_s1 = $random;
`ifdef MISTER_LW_SECTOR_SLOT
		win.sec.sector.q = {$random, $random}; win.sec.sector.rptr = $random; win.sec.sector.sector_r_lwgray = $random;
		win.sec.sector.fs = $random; win.sec.sector.fexp = $random; win.sec.sector.em = $random;
		win.sec.sector.qv = $random; win.sec.sector.tmode = $random; win.sec.sector.ph = $random;
		win.sec.sector.ackc = $random; win.sec.sector.hi = $random; win.sec.sector.fsum = $random;
		win.sec.sector.evt = $random; win.sec.sector.dcnt = $random; win.sec.sector.dsent = $random;
		win.sec.sector.dsum = $random; win.sec.sector.fsel = $random; win.sec.sector.fwack = $random;
		win.sec.sector.rjs = $random; win.sec.sector.rjt = $random; win.sec.sector.rjp = $random;
		win.sec.sector.fack = $random; win.sec.sector.mt = $random; win.sec.sector.ml = $random;
		win.sec.sector.en_sys = $random; win.sec.sector.x_en_sys = $random; win.sec.sector.o_wr = $random;
		win.sec.sector.o_dat = $random; win.sec.sector.o_adr = $random; win.sec.sector.sys_lwx = $random;
		win.sec.sector.ctl_sync.lws0_q = $random; win.sec.sector.ctl_sync.lws1_q = $random;
		win.sec.sector.wg_sync.lws0_q = $random; win.sec.sector.wg_sync.lws1_q = $random;
		win.sec.sector.fence.rest_n = $random; win.sec.sector.fence.ack_lwx = $random;
		win.sec.sector.fence.lease_sync.lws0_q = $random; win.sec.sector.fence.lease_sync.lws1_q = $random;
`endif
`ifdef MISTER_LW_LANE
		win.lane.lane.rptr = $random; win.lane.lane.q = $random; win.lane.lane.fresh = $random;
		win.lane.lane.ps = $random; win.lane.lane.quiet = $random; win.lane.lane.cnt = $random;
		win.lane.lane.capf = $random; win.lane.lane.rwp = $random; win.lane.lane.ml = $random;
		win.lane.lane.own = $random; win.lane.lane.hold = $random; win.lane.lane.sel = $random;
		win.lane.lane.strobe = $random; win.lane.lane.lane_r_lwgray = $random; win.lane.lane.lane_rw_lwgray = $random;
		win.lane.lane.pt_lwx = $random; win.lane.lane.rdt_lwx = $random;
		win.lane.lane.wg_sync.lws0_q = $random; win.lane.lane.wg_sync.lws1_q = $random;
		win.lane.lane.rrg_sync.lws0_q = $random; win.lane.lane.rrg_sync.lws1_q = $random;
		win.lane.lane.fence.rest_n = $random; win.lane.lane.fence.ack_lwx = $random;
		win.lane.lane.fence.lease_sync.lws0_q = $random; win.lane.lane.fence.lease_sync.lws1_q = $random;
`endif
	end
endtask
task ls8;
	reg [31:0] v, p, dr;
	integer pass, j, tries, t, n1;
	begin
`ifdef MISTER_LW_LANE
		vs_auto = 1;
`endif
		for (pass = 0; pass < 4; pass = pass + 1) begin
			$display("-- lease links: every clk_sys register scrambled mid-lease (seed %0d); then RELEASE and CLAIMs", pass);
			lease_up;
			if (TB_SEC) sec_wr(21'h10, 32'h1);
			fc_off = 1;
			for (j = 0; j < 3; j = j + 1) begin scramble; repeat (slow(30)) @(negedge clk); end
			repeat (slow(200)) @(negedge clk);
			kp_on = 0;
			axi_write(LEASE, lease_w(KP_ID, 2'd3, 4'd0), 4'hF);
			if (pass[0]) begin
				// CLAIMs as fast as the bus allows from the RELEASE on: the claim monitor checks the one taken
				claim_hammer(KP_ID, slow(20000), tries);
				$display("     taken after %0d CLAIM stores", tries);
				check("after the scramble: a CLAIM is taken", win.live_q, 1);
				check("after the scramble: CLAIMs taken with a half off rest", n_offrest, 0);
				fc_off = 0;
			end
			else begin
				t = 0; while (TB_LINKED != 0 && t < slow(20000)) begin
					axi_read(OWNEDA, 0, v, dl, beats); if (v == 0) t = slow(20000); else t = t + 10;
				end
				check("after the scramble: the return completes, OWNED clear", v, 0);
				repeat (slow(100)) @(negedge clk);
				fc_off = 0;
				check("after the return: every SECTOR ack idle", {blk_dn[17], card_dn[12], card_dn[10]}, 3'd0);
`ifdef MISTER_LW_LANE
				check("after the return: the player idle", {lane_own, lane_hold}, 2'd0);
`endif
				if (TB_SEC) begin
					sec_stand(32'h00007800 + pass);
					n1 = cd_frames; spi_frame(cd_lba); wait_frame(n1);
					check_buf("a stock SPI transfer before the claim", cd_lba, TB_FRAME);
				end
				claim_hammer(KP_ID, slow(4000), tries);
				check("a CLAIM is taken", win.live_q, 1);
			end
			wait_open; lease_rd(v); kp_epoch = v[23:16];
			axi_write(LINEN, 32'hFFFFFFFF, 4'hF); cause_drain(SYSB); kp_t = $realtime; kp_on = 1;
			axi_write(pad_addr(8'h02, 0), 32'h00C0FFEE ^ pass, 4'hF); settle;
			check("a pad store lands under the new lease", joy0, 32'h00C0FFEE ^ pass);
			st_req(st_pat(970 + pass)); settle;
			check("a status capture raises SYS's line", lines[1], 1);
			cause_drain(SYSB);
`ifdef MISTER_LW_LANE
			f_set(0, 16'h001E, 2); fw[0] = 16'h8800 + pass; fw[1] = 16'h8880 + pass;
			lane_vs_link("after the scramble: a lane frame");
`endif
			if (TB_SEC) begin
				sec_wr(21'h10, 32'h1);
				sec_stand(32'h00007900 + pass); if (win.sec_lvl) cause_drain(SEC);
				n1 = cd_frames;
				sec_wr(21'h0C, cd_lba);
				frame_data(cd_lba, 0, TB_FRAME);
				wait_frame(n1);
				check_buf("after the scramble: a SECTOR frame byte-exact", cd_lba, TB_FRAME);
				cause_drain(SEC); sec_wr(21'h10, 32'h0);
			end
		end
		$display("-- lease links: the clk_sys registers scrambled during the return: the links converge, a lease works");
		for (pass = 0; pass < 8; pass = pass + 1) begin
			lease_up;
`ifdef MISTER_LW_LANE
			@(negedge clk_sys); io_enable = 1;
			f_set(0, 16'h001E, 2); fw[0] = 16'h8900; fw[1] = 16'h8901; lane_send(1); lane_send(1);
`endif
			kp_on = 0;
			fc_off = 1;
			fork
				axi_write(LEASE, lease_w(KP_ID, 2'd3, 4'd0), 4'hF);
				begin repeat (2 + 3 * pass) @(negedge clk); scramble; end
			join
`ifdef MISTER_LW_LANE
			@(negedge clk_sys); io_enable = 0;
`endif
			t = 0; while (t < slow(20000)) begin
				axi_read(LEASE, 0, v, dl, beats); if (v[30]) t = slow(20000); else t = t + 10;
			end
			check($sformatf("scrambled in the return (%0d): quiet again", pass), v[30], 1);
			repeat (slow(50)) @(negedge clk);
			fc_off = 0;
			lease_up;
`ifdef MISTER_LW_LANE
			lane_count(p, dr);
			f_set(0, 16'h001E, 2); fw[0] = 16'h8A00 + pass; fw[1] = 16'h8A80 + pass; lane_send(1);
			lane_wait(p + 1);
`endif
			if (TB_SEC) begin
				sec_wr(21'h10, 32'h1); rd("a later lease: SECTOR's EN lands", SEC + 21'h10, 32'h1); sec_wr(21'h10, 32'h0);
				sec_spi_clear;
			end
		end
`ifdef MISTER_LW_LANE
		vs_auto = 0;
		@(negedge clk_sys); tb_vsync = 0;
`endif
		lease_off;
	end
endtask

// SECTOR's sweep: a primary frame stored in full, the lease ended by RELEASE at each offset from the last store's
// response to after the frame's last strobe (every cycle for the first 24, then a stride): byte-exact every time.
task sec7_sweep;
	reg [31:0] lba;
	integer off, n1, n_bef, n_aft, stride;
	begin
		$display("-- SECTOR: a frame stored in full, the lease ended at each offset to its last strobe: byte-exact");
		n_bef = 0; n_aft = 0; stride = 1;
		for (off = 0; n_aft < 2 && off < slow(20000); off = off + stride) begin
			lease_up;
			sec_wr(21'h10, 32'h1);
			sec_stand(32'h00007C00 + off); lba = cd_lba; if (win.sec_lvl) cause_drain(SEC);
			n1 = cd_frames;
			sec_wr(21'h0C, lba);
			frame_data(lba, 0, TB_FRAME);
			repeat (off) @(negedge clk);
			kp_on = 0;
			axi_write(LEASE, lease_w(KP_ID, 2'd3, 4'd0), 4'hF);
			if (cd_frames == n1) n_bef = n_bef + 1; else n_aft = n_aft + 1;
			wait_frame(n1);
			check_buf($sformatf("ended %0d cycles after the last store: byte-exact", off), lba, TB_FRAME);
			sec_spi_clear;
			if (off >= 23) stride = slow(500);
		end
		$display("     %0d ends landed before the frame's last strobe, %0d after", n_bef, n_aft);
		check("ends landed after the frame's last strobe", n_aft > 0, 1);
		// at 150 MHz a frame can drain before the earliest RELEASE takes effect: recorded, not required
		if (`LW_TB_SYS_HP_PS >= 5000) check("ends landed while the frame still played", n_bef > 0, 1);
	end
endtask

// RESETS: the rises of lw_hrst and the machines freed, from configuration, saturating at 255.
integer fm_nh = 0;
always @(posedge clk) if (lw_hrst && !fm_hq) fm_nh = fm_nh + 1;
function [31:0] resets_exp;
	resets_exp = {16'd0, (n_fr + n_fw > 255) ? 8'd255 : n_fr[7:0] + n_fw[7:0], (fm_nh > 255) ? 8'd255 : fm_nh[7:0]};
endfunction
task resets_cases;
	integer j, n;
	begin
		$display("-- RESETS: each rise of lw_hrst and each machine freed, counted from configuration");
		$display("     %0d rises, %0d freed", fm_nh, n_fr + n_fw);
		rd("RESETS against the firewall model", RESETSA, resets_exp());
		check("RESETS has counted both", {fm_nh > 0, n_fr + n_fw > 0}, 2'b11);
		$display("-- RESETS: 300 more pulses, each with an orphaned read: both counts saturate at 255");
		for (j = 0; j < 300; j = j + 1) begin
			rready = 0;
			@(negedge clk); fw_ar(21'h40000, 0, 12'h0F5);
			n = 0; while (rvalid !== 1'b1 && n < 20) begin @(negedge clk); n = n + 1; end
			hps_rst_n = 0; repeat (3) @(negedge clk); hps_rst_n = 1;
			fw_wait_fall; repeat (3) @(negedge clk);
			rready = 1;
			if (j == 100) rd("RESETS after 100 more", RESETSA, resets_exp());
		end
		rd("RESETS saturated", RESETSA, 32'h0000FFFF);
		check("the model agrees", resets_exp(), 32'h0000FFFF);
	end
endtask

// The lease's every-cycle checks, at the end of an arm: the bench's own lease never lapsed under its keeper, GPI[22]
// was live_q, the lines were their definition, and each of a line's three reasons to fall was seen.
task lease_summary(input cov_on);
	begin
		$display("-- lease: the bench's keeper, GPI[22] and the lines' definition over the whole run");
		$display("     %0d renews, %0d cycles GPI[22] compared (%0d held), %0d cycles the lines compared", kp_n, gpi22_n,
		         gpi22_on, lm_n);
		$display("     line falls: %0d by its cause, %0d by LINE_EN, %0d by the lease's end", lm_fall_c, lm_fall_e, lm_fall_l);
		check("the bench's lease lapsed or was lost under its keeper", kp_lapse + kp_lost, 0);
		check("cycles in which GPI[22] differs from live_q", gpi22_bad, 0);
		check("cycles in which a line differs from its definition", lm_bad, 0);
		$display("     lease links: %0d CLAIMs taken, %0d with a half off rest", n_claims, n_offrest);
		check("CLAIMs taken with a linked half off rest (the bench's own predicate, now and four clk_sys cycles before)",
		      n_offrest, 0);
`ifdef MISTER_LW_SECTOR_SLOT
		$display("     SECTOR's link: %0d rises, %0d falls, %0d store strobes", fc_sec.n_rise, fc_sec.n_fall, fc_sec.n_stb);
		check("SECTOR's link checker", fc_sec.n_bad, 0);
`endif
`ifdef MISTER_LW_LANE
		$display("     the lane's link: %0d rises, %0d falls, %0d store strobes", fc_lane.n_rise, fc_lane.n_fall,
		         fc_lane.n_stb);
		check("the lane's link checker", fc_lane.n_bad, 0);
`endif
		if (cov_on) check("a line fell by each of its three reasons", {lm_fall_c > 0, lm_fall_e > 0, lm_fall_l > 0}, 3'b111);
	end
endtask

`ifdef MENU_CORE
integer f;
initial begin
	repeat (4) @(negedge clk);
	sys_rate;
	cdc_cells;
	lease_up;
	$display("-- MENU_CORE: the header rows never read the shadow");
	for (r = 0; r < 19; r = r + 1) spi_osd_row(r, 3);
	for (r = 0; r < 16; r = r + 1) lw_row(r, 3);
	axi_write(OSD + 21'h1000, 32'h1, 4'hF);    // LWSEL, no HR: 64 + 24 header lines fit the tb's 140
	spi_osd_cmd(8'h40);                        // clears the highres rows 8-15 re-armed
	spi_osd_cmd(8'h41);
	f = cap_h.frames;
	wait (cap_h.frames >= f + 4);
	$display("     header pixel steps %0d, shadow-differing steps %0d (ce %0d, lwsel %0d)", hdr_n, shd_n, c_n, l_n);
	check("header steps with LWSEL seen", hdr_n > 0, 1);
	check("the shadow was read elsewhere", shd_n > 0, 1);
	check("header bytes all from osd_buffer", hdr_bad, 0);
	lease_summary(0);
	$display("%s: %0d checks, %0d failures", fails ? "FAILED" : "PASSED", checks, fails);
	$finish;
end
`elsif LW_TB_DEFGEOM
// The documented default geometry: frames are bounded by the OPEN frame's length, so a save-slot frame longer
// than the primary's completes, and the shared feeder still serves the primary after it.
initial begin
	repeat (4) @(negedge clk);
	sys_rate;
	cdc_cells;
	lease_up;
	$display("-- default geometry: a %0d-word primary, 256-word save slots", TB_FRAME);
	rd("SECTOR K0 slot/frame 128", 21'h400E0, 32'h01000080);
	rd("SECTOR K3 card 1 slot/flags/frame 256", 21'h400EC, 32'h02030100);
	sec_wr(21'h10, 32'hC1); settle;

	$display("-- a 128-word primary frame");
	n0 = cd_frames; s0 = cd_strobes;
	cd_request(32'h00000100); settle;
	cause_drain(SEC);
	sec_wr(21'h0C, 32'h00000100);
	frame_data(32'h00000100, 0, 128);
	rd("ACCEPT ok, 128", SEC + 21'h18, 32'h80000080);
	wait_frame(n0);
	check_buf("primary frame 0x100", 32'h00000100, 128);
	repeat (200) @(negedge clk_sys);
	check("256 strobes", cd_strobes - s0, 256);

	$display("-- a 256-word save-slot frame, longer than the primary's");
	r = mc1.done; s0 = mc1.strobes;
	mc1.go(0, 7'h12); settle;
	cause_drain(SEC);
	sec_wr(21'h48, 32'h00000012);
	frame_data(32'h00000012, 0, 256);
	rd("ACCEPT ok, 256: stores past the primary's length land", SEC + 21'h18, 32'h80000100);
	repeat (1500) @(negedge clk_sys);
	check("card 1 frame done", mc1.done - r, 1);
	check("card 1 took 512 strobes", mc1.strobes - s0, 512);
	bad = 0; for (i = 0; i < 256; i = i + 1) if ({mc1.ram[2*i+1], mc1.ram[2*i]} !== pat(32'h12, i)) bad = bad + 1;
	check("card 1 buffer byte-exact (1 KB)", bad, 0);
	axi_read(SEC + 21'h14, 0, d, dl, beats);
	check("STATUS idle after the card frame, ack low", d[3:1], 3'b000);

	$display("-- the primary after the card: the shared feeder is free");
	n0 = cd_frames;
	cd_request(32'h00000101); settle;
	cause_drain(SEC);
	sec_wr(21'h0C, 32'h00000101);
	frame_data(32'h00000101, 0, 128);
	rd("ACCEPT ok, 128 after the card", SEC + 21'h18, 32'h80000080);
	wait_frame(n0);
	check_buf("primary frame 0x101 after the card", 32'h00000101, 128);
	rd("REQ idle", SEC + 21'h04, 32'd0);
	check("no ring left", doorbell, 0);
	lease_summary(0);

	$display("%s: %0d checks (%0d functional + %0d fixed-latency, one per transaction), %0d failures",
	         fails ? "FAILED" : "PASSED", checks, checks - lchecks, lchecks, fails);
	$finish;
end
`else
initial begin
	repeat (4) @(negedge clk);
	sys_rate;
	cdc_cells;

	dir_cases;
	lease_up;

	$display("-- scratch");
	axi_write(SCR, 32'hA5A55A5A, 4'hF);
	rd("scratch", SCR, dg(32'hA5A55A5A));
	axi_write(SCR, 32'h00C30000, 4'h4);
	rd("scratch byte lane", SCR, dg(32'hA5C35A5A));

	$display("-- pads");
	axi_write(pad_addr(8'h02, 0), 32'h8001F00F, 4'hF); settle;
	check("joystick_0", joy0, 32'h8001F00F);
	axi_write(pad_addr(8'h03, 0), 32'hDEAD0011, 4'hF); settle;
	check("joystick_1", joy1, 32'hDEAD0011);
	axi_write(pad_addr(8'h10, 0), 32'h12345678, 4'hF); settle;
	check("joystick_2", joy2, 32'h12345678);
	axi_write(pad_addr(8'h11, 0), 32'hFFFFFFFF, 4'hF); settle;
	check("joystick_3", joy3, 32'hFFFFFFFF);
	axi_write(pad_addr(8'h1A, 8'h00), 32'h0000817F, 4'hF); settle;
	check("analog_0", {16'd0, ana0}, 32'h0000817F);
	axi_write(pad_addr(8'h3D, 8'h00), 32'h00007F81, 4'hF); settle;
	check("right analog_0", {16'd0, rana0}, 32'h00007F81);
	check("left analog_0 untouched by 3D", {16'd0, ana0}, 32'h0000817F);
	axi_write(pad_addr(8'h1A, 8'h03), 32'h00001122, 4'hF);
	axi_write(pad_addr(8'h3D, 8'h03), 32'h00003344, 4'hF); settle;
	check("left analog_3", {16'd0, ana3}, 32'h00001122);
	check("right analog_3", {16'd0, rana3}, 32'h00003344);
	axi_write(pad_addr(8'h1A, 8'h1F), 32'h000000C8, 4'hF); settle;
	check("paddle_1 (idx {pdsp=1,stick=15})", {24'd0, pdl1}, 32'h000000C8);
	axi_write(pad_addr(8'h02, 1), 32'h11111111, 4'hF); settle;
	check("op 02 idx 1 dropped", joy0, 32'h8001F00F);
	axi_write(pad_addr(8'h05, 0), 32'h22222222, 4'hF); settle;
	check("unhonoured op dropped", joy0, 32'h8001F00F);
	axi_write(pad_addr(8'h02, 0), 32'h0000A0A0, 4'hF);
	axi_write(pad_addr(8'h03, 0), 32'h0000B0B0, 4'hF);
	axi_write(pad_addr(8'h10, 0), 32'h0000C0C0, 4'hF);
	axi_write(pad_addr(8'h11, 0), 32'h0000D0D0, 4'hF);
	settle;
	check("back-to-back joystick_0", joy0, 32'h0000A0A0);
	check("back-to-back joystick_1", joy1, 32'h0000B0B0);
	check("back-to-back joystick_2", joy2, 32'h0000C0C0);
	check("back-to-back joystick_3", joy3, 32'h0000D0D0);
	rd("pad block read", pad_addr(8'h02, 0), 32'd0);
	rd("pad block unhonoured read", pad_addr(8'h05, 3), 32'd0);

	$display("-- default responder");
	rd("LW 0x000000", 21'h000000, 32'hDEADBEEF);
	rd("below window", 21'h03FFFC, 32'hDEADBEEF);
	rd("directory page past the entries", DIRB + 21'h40 + 21'h80 * em_n, 32'd0);
	rd("directory page end, below SECTOR", 21'h040FFC, 32'd0);
	rd("above SECTOR: SYS CAUSE idle", 21'h043000, 32'd0);
	rd("SYS block end", 21'h043FFC, 32'd0);
	if (TB_LANE) rd("above LANE", 21'h047000, 32'hDEADBEEF);
	else rd("above OSDBUF", 21'h046000, 32'hDEADBEEF);
	rd("above window", 21'h0C0000, 32'hDEADBEEF);
	rd("LW 0x1FFFFC", 21'h1FFFFC, 32'hDEADBEEF);
	axi_write(21'h000100, 32'h0, 4'hF);
	axi_read(DIRB + 21'h1C, 4'd3, d, dl, beats);
	check("burst beats", beats, 4);
	check("burst last (scratch)", dl, dg(32'hA5C35A5A));

	decode_sweep;

	$display("-- firewall: read bursts of every type step their words on 12 bits; a size above a word steps a word");
	for (i = 0; i < 64; i = i + 1) begin axi_read(21'h40000 + {13'd0, i[5:0], 2'b00}, 0, d, dl, beats); wref[i] = d; end
	burst_vs_ref("INCR 2",  21'h40008, 4'd1,  2'b01);
	burst_vs_ref("INCR 4",  21'h40010, 4'd3,  2'b01);
	burst_vs_ref("INCR 8",  21'h40024, 4'd7,  2'b01);
	burst_vs_ref("INCR 16", 21'h40040, 4'd15, 2'b01);
	burst_vs_ref("FIXED 4",  21'h40014, 4'd3,  2'b00);
	burst_vs_ref("FIXED 16", 21'h40000, 4'd15, 2'b00);
	burst_vs_ref("WRAP 2 from the second word",  21'h40054, 4'd1,  2'b10);
	burst_vs_ref("WRAP 4 from the third word",   21'h40018, 4'd3,  2'b10);
	burst_vs_ref("WRAP 8 from the eighth word",  21'h4005C, 4'd7,  2'b10);
	burst_vs_ref("WRAP 16 from the sixth word",  21'h40094, 4'd15, 2'b10);
	burst_vs_ref("WRAP 16 from its first word",  21'h400C0, 4'd15, 2'b10);
	burst_vs_ref("reserved burst type 3, 4 beats", 21'h40030, 4'd3, 2'b11);
	rsize = 3;
	burst_vs_ref("INCR 4 of 8-byte size steps a word", 21'h40010, 4'd3, 2'b01);
	burst_vs_ref("WRAP 4 of 8-byte size wraps on words", 21'h40018, 4'd3, 2'b10);
	rsize = 7;
	burst_vs_ref("INCR 8 of 128-byte size steps a word", 21'h40020, 4'd7, 2'b01);
	burst_vs_ref("WRAP 16 of 128-byte size wraps on words", 21'h40094, 4'd15, 2'b10);
	rsize = 2;
	arburst = 2'b10; axi_read(21'h40004, 4'd2, d, dl, beats); arburst = 2'b01;
	check("WRAP of an illegal length (3 beats) answered", beats, 3);

	$display("-- firewall: a read burst that would cross 4 KB steps inside its 4 KB, and is answered");
	burst_vs_single("INCR 4 across the directory's 4 KB end", 21'h40FF8, 4'd3, 2'b01);
	burst_vs_single("INCR 16 across the window's first 4 KB boundary", 21'h3FFF8, 4'd15, 2'b01);
	burst_vs_single("INCR 4 across the top of SYS", 21'h43FF8, 4'd3, 2'b01);
	burst_vs_single("INCR 4 across the window's last 4 KB end, onto PAD", 21'h7FFF8, 4'd3, 2'b01);
	burst_vs_single("INCR 4 across the top of the 2 MB", 21'h1FFFF8, 4'd3, 2'b01);
	burst_vs_single("FIXED 16 at the last word", 21'h1FFFFC, 4'd15, 2'b00);
	burst_vs_single("reserved type 3, 16 beats across 4 KB", 21'h40FE0, 4'd15, 2'b11);
	for (i = 0; i < 4; i = i + 1) begin
		awid = {10'h0F0, i[1:0]};
		axi_wburst(21'h000100, 4'd3, i[1:0], beats);
		check($sformatf("undecoded write burst, type %0d: one response", i), beats, 1);
		check($sformatf("undecoded write burst, type %0d: BID echoed", i), {20'd0, bid}, {22'd0, 8'hF0, i[1:0]});
	end
	axi_wburst(21'h1FFFF8, 4'd15, 2'b01, beats);
	check("write burst across the top of the 2 MB: one response", beats, 1);
	awid = 0;
	bad = 0;
	for (i = 0; i < 64; i = i + 1) begin
		axi_read(21'h40000 + {13'd0, i[5:0], 2'b00}, 0, d, dl, beats);
		if (lmask(21'h40000 + {13'd0, i[5:0], 2'b00}, d) !== lmask(21'h40000 + {13'd0, i[5:0], 2'b00}, wref[i])) bad = bad + 1;
	end
	check("the directory reads the same after every burst", bad, 0);

	$display("-- firewall: a burst write has no effect, every beat answered; a single-beat store of any type does");
	begin : burst_writes
		reg [511:0] sw0, ss0;
		reg [255:0] sl0;
		reg [20:0]  a;
		reg  [3:0]  len;
		reg [31:0]  oc;
		integer b, t, m, nw0, nb0, no0, nbeats, nresp, nbad;
		// clk_sys stopped, so only a store can move the state compared
		sys_run = 0;
		repeat (50) @(negedge clk);
		sw0 = pure_win(); ss0 = pure_sec(); sl0 = pure_lane();
		pure_view(0, nbad);
		nw0 = n_lwwr; nb0 = n_lwwb; no0 = n_osdwe; nbeats = 0;
		for (b = 0; b < 13; b = b + 1) begin
			case (b)
				0:  a = SEC + 21'h10;                  // SECTOR CTRL
				1:  a = SEC + 21'h0C;                  // BEGIN
				2:  a = SEC + 21'h44;                  // BEGIN_S[1]
				3:  a = SEC + 21'h00;                  // SECTOR CAUSE
				4:  a = SEC + 21'h1000;                // DATA[0]
				5:  a = pad_addr(8'h02, 0);            // PAD
				6:  a = pad_addr(8'h1A, 0);            // PAD, analog
				7:  a = OSD;                           // IMAGE
				8:  a = OSD + 21'h1000;                // OSD CTRL
				9:  a = SYSB;                          // SYS CAUSE
				10: a = SCR;                     // SCRATCH
				11: a = 21'h46000;                     // LANE BEGIN, WORD, END, RPOP
				default: a = 21'h4600C;
			endcase
			for (t = 0; t < 4; t = t + 1)
				for (m = 0; m < 3; m = m + 1) begin
					len = (m == 0) ? 4'd1 : (m == 1) ? 4'd3 : 4'd15;
					axi_wburst(a, len, t[1:0], nresp);
					checks = checks + 1;
					if (nresp != 1) begin
						fails = fails + 1;
						$display("FAIL burst write %06h type %0d, %0d beats: %0d responses", a, t, len + 1, nresp);
					end
					nbeats = nbeats + len + 1;
				end
		end
		check("burst writes: no beat reached the register bus", n_lwwr - nw0, 0);
		check("burst writes: no beat reached the OSD image", n_osdwe - no0, 0);
		check("burst writes: every beat reported dropped", n_lwwb - nb0, nbeats);
		pure_view(1, nbad);
		check("burst writes: the readable view is unchanged", nbad, 0);
		check("burst writes: the window's, PAD's and SYS's state is unchanged", pure_win() === sw0, 1);
		check("burst writes: SECTOR's state is unchanged", pure_sec() === ss0, 1);
		check("burst writes: the lane's state is unchanged", pure_lane() === sl0, 1);
		sys_run = 1;
		settle;
		// a single beat (AWLEN 0) is a store whatever its burst type
		axi_read(OSD + 21'h1000, 0, oc, dl, beats);
		nw0 = n_lwwr; nb0 = n_lwwb;
		for (t = 0; t < 4; t = t + 1) begin
			awburst = t[1:0];
			axi_write(OSD + 21'h1000, {30'd0, ~t[1:0]}, 4'hF);
			rd($sformatf("single store, burst type %0d: OSD CTRL", t), OSD + 21'h1000, {30'd0, ~t[1:0]});
			axi_write(SCR, 32'h5C000000 | t, 4'hF);
			rd($sformatf("single store, burst type %0d: SCRATCH", t), SCR, dg(32'h5C000000 | t));
		end
		awburst = 2'b01;
		check("single stores: one bus pulse each", n_lwwr - nw0, 8);
		check("single stores: none dropped", n_lwwb - nb0, 0);
		axi_write(OSD + 21'h1000, oc, 4'hF);
		rd("OSD CTRL restored", OSD + 21'h1000, oc);
	end

	$display("-- clk_sys stopped (core held in reset)");
	sys_run = 0;
	rd("magic, clk_sys stopped", 21'h40000, FW_MAGIC);
	axi_write(SCR, 32'h0BADF00D, 4'hF);
	rd("scratch, clk_sys stopped", SCR, dg(32'h0BADF00D));
	for (i = 1; i <= 5; i = i + 1) axi_write(pad_addr(8'h02, 0), i, 4'hF);
	axi_write(pad_addr(8'h03, 0), 32'h00000077, 4'hF);
	axi_write(pad_addr(8'h03, 0), 32'h00000000, 4'hF);
	for (i = 1; i <= 10; i = i + 1) axi_write(pad_addr(8'h1A, 0), i, 4'hF);
	for (i = 11; i <= 20; i = i + 1) axi_write(pad_addr(8'h3D, 0), i, 4'hF);
	rd("undecoded, clk_sys stopped", 21'h1FFFFC, 32'hDEADBEEF);
	check("joystick_0 unchanged while stopped", joy0, 32'h0000A0A0);
	rd("analog drops (16 queued of 20, a pad store in flight)", pad_addr(8'h1A, 0), dg(32'd4));
	sys_run = 1; repeat (slow(300)) @(negedge clk);
	check("joystick_0 last of 5 lands on restart", joy0, 32'h00000005);
	check("joystick_1 release (last) lands on restart", joy1, 32'h00000000);
	check("analog_0 last left queued lands on restart", {16'd0, ana0}, 32'h0000000A);
	check("right analog_0 last queued lands on restart", {16'd0, rana0}, 32'h00000010);

`ifndef MISTER_LW_SECTOR_SLOT
	$display("-- no SECTOR block: its addresses read 0xDEADBEEF, stores are ignored, no doorbell");
	rd("SECTOR CAUSE reads the default", 21'h041000, 32'hDEADBEEF);
	rd("SECTOR ID reads the default", 21'h041038, 32'hDEADBEEF);
	axi_write(21'h041010, 32'h1, 4'hF);
	axi_write(21'h04100C, 32'h0, 4'hF);
	rd("SECTOR CTRL still the default", 21'h041010, 32'hDEADBEEF);
	rd("SECTOR DATA reads the default", 21'h042000, 32'hDEADBEEF);
	rd("SECTOR end reads the default", 21'h042FFC, 32'hDEADBEEF);
	check("no doorbell", doorbell, 0);
	check("blk_dn idle", blk_dn, 0);
	check("card_dn idle", card_dn, 0);
`else

	$display("-- SECTOR block: registers");
	rd("ID SEC2", SEC + 21'h38, 32'h32434553);
	rd("CTRL reset", SEC + 21'h10, 32'd0);
	rd("STATUS idle", SEC + 21'h14, 32'd0);
	rd("REQ idle", SEC + 21'h04, 32'd0);
	rd("CAUSE idle", SEC + 21'h00, 32'd0);
	rd("BEGIN reads 0", SEC + 21'h0C, 32'd0);
	rd("register hole", SEC + 21'h3C, 32'd0);
	rd("block hole", SEC + 21'hFFC, 32'd0);
	rd("DATA[0] reads 0", SEC + 21'h1000, 32'd0);
	rd("DATA[588] reads 0", SEC + 21'h1000 + 4*588, 32'd0);
	rd("block end", SEC + 21'h1FFC, 32'd0);
	check("doorbell low", doorbell, 0);

	$display("-- doorbell on cd_hps_req");
	axi_read(SEC + 21'h24, 0, gen0, dl, beats);
	cd_request(32'h00001234);
	settle;
	check("doorbell raised", doorbell, 1);
	check("SPI slot 1 offered while EN=0", sd_rd[1], 1);
	rd("REQ hides a request whose EN is 0", SEC + 21'h04, 32'd0);
	rd("LBA captured", SEC + 21'h08, 32'h00001234);
	rd("GEN +1", SEC + 21'h24, dg(gen0 + 1));
	rsize = 0; rd("CAUSE byte read: data", SEC + 21'h00, 32'h00000001); rsize = 2;
	check("byte read does not clear", doorbell, 1);
	rd("CAUSE word read: bit 0", SEC + 21'h00, 32'h00000001);
	check("word read does not clear", doorbell, 1);
	arburst = 2'b00; axi_read(SEC + 21'h00, 4'd15, d, dl, beats); arburst = 2'b01;
	check("a 16-beat FIXED burst on CAUSE: every beat the cause", {d, dl}, {32'h00000001, 32'h00000001});
	check("burst read does not clear", doorbell, 1);
	axi_write(SEC + 21'h00, 32'hFFFFFFFE, 4'hF);
	check("a store with bit 0 clear clears nothing", doorbell, 1);
	axi_write(SEC + 21'h00, 32'h00000001, 4'h7);
	check("a partial-width store clears nothing", doorbell, 1);
	rd("CAUSE kept", SEC + 21'h00, 32'h00000001);
	axi_write(SEC + 21'h00, 32'h00000001, 4'hF);
	check("a store of bit 0 clears the line", doorbell, 0);
	rd("CAUSE zero iff idle", SEC + 21'h00, 32'd0);
	sec_wr(21'h10, 32'h1); settle;
	rd("REQ is a level (pending once EN)", SEC + 21'h04, 32'h800080C5);
	rd("LBA_S[1] aliases LBA", SEC + 21'h64, 32'h00001234);
	rd("STATUS en_sys", SEC + 21'h14, 32'h00000001);
	check("SPI slot 1 hidden while en_sys", sd_rd[1], 0);

	$display("-- full 588-store frame");
	n0 = cd_frames; s0 = cd_strobes;
	sec_wr(21'h0C, 32'h00001234);
	frame_data(32'h00001234, 0, 588);
	rd("ACCEPT ok, 588", SEC + 21'h18, 32'h8000024C);
	wait_frame(n0);
	check_buf("frame 0x1234", 32'h00001234, 588);
	repeat (200) @(negedge clk_sys);
	check("1176 strobes", cd_strobes - s0, 1176);
	check("the CD reader saw ack then dropped req", cd_req, 0);
	rd("REQ cleared by ack", SEC + 21'h04, 32'd0);
	axi_read(SEC + 21'h14, 0, d, dl, beats);
	check("STATUS: 1 frame, idle, en_sys", {d[31:16], d[3:0]}, DIAG ? 20'h00011 : 20'h00001);
	if (!DIAG) check("STATUS[31:4] lean", d[31:4], 0);
	$display("     FIFO high-water %0d entries", d[15:8]);
	rd("SUM", SEC + 21'h34, dg(frame_sum(32'h00001234, 588)));
	axi_read(SEC + 21'h28, 0, d, dl, beats);
	axi_read(SEC + 21'h2C, 0, dl, d0, beats);
	if (DIAG) check("TSTAMP_DONE after TSTAMP_REQ", dl > d, 1);
	else check("TSTAMP_REQ/DONE lean", {d, dl}, 0);
	if (!DIAG) begin
		rd("lean CYCLES reads 0", SEC + 21'h30, 32'd0);
		rd("lean GEN reads 0", SEC + 21'h24, 32'd0);
		rd("lean DROPS reads 0", SEC + 21'h1C, 32'd0);
		rd("lean COUNTS reads 0", SEC + 21'h20, 32'd0);
	end
	$display("     req -> done %0d clk_100m cycles", dl - d);

	$display("-- out-of-order and duplicated stores");
	n0 = cd_frames;
	cd_request(32'h00001235); settle;
	sec_wr(21'h0C, 32'h00001235);
	frame_data(32'h00001235, 0, 10);
	sec_data(11, pat(32'h00001235, 11));
	sec_data(10, pat(32'h00001235, 10));
	sec_data(10, pat(32'h00001235, 10));
	sec_data(11, pat(32'h00001235, 11));
	frame_data(32'h00001235, 12, 588);
	rd("ACCEPT ok", SEC + 21'h18, 32'h8000024C);
	wait_frame(n0);
	check_buf("frame 0x1235 not shifted", 32'h00001235, 588);
	rd("COUNTS skipped 2", SEC + 21'h20, dg(32'h00020000));

	$display("-- lost store detected by ACCEPT, resent from first_drop");
	n0 = cd_frames;
	cd_request(32'h00001236); settle;
	sec_wr(21'h0C, 32'h00001236);
	frame_data(32'h00001236, 0, 100);
	frame_data(32'h00001236, 101, 588);
	rd("ACCEPT !ok, first_drop 100", SEC + 21'h18, 32'h00000064);
	repeat (slow(2000)) @(negedge clk);
	check("frame held open (the CD reader waits)", cd_frames, n0);
	frame_data(32'h00001236, 100, 588);
	rd("ACCEPT ok after resend", SEC + 21'h18, 32'h8000024C);
	wait_frame(n0);
	check_buf("frame 0x1236 after resend", 32'h00001236, 588);

	$display("-- FIFO full with clk_sys stopped: no back-pressure, drops, fixed latency, resend");
	n0 = cd_frames;
	cd_request(32'h00001237); settle;
	sec_wr(21'h0C, 32'h00001237);
	settle;
	sys_run = 0;
	frame_data(32'h00001237, 0, 588);
	rd("ACCEPT drops 76, first_drop 512", SEC + 21'h18, 32'h004C0200);
	rd("DROPS 76", SEC + 21'h1C, dg(32'd76));
	axi_read(SEC + 21'h14, 0, d, dl, beats);
	check("STATUS high-water saturates at 255", d[15:8], dg(255));
	rd("CAUSE pending, clk_sys stopped", SEC + 21'h00, 32'h00000001);
	check("CAUSE read keeps the line with clk_sys stopped", doorbell, 1);
	axi_write(SEC + 21'h00, 32'h00000001, 4'hF);
	check("CAUSE store clears with clk_sys stopped", doorbell, 0);
	rd("ID, clk_sys stopped", SEC + 21'h38, 32'h32434553);
	rd("undecoded, clk_sys stopped", 21'h1FFFFC, 32'hDEADBEEF);
	sys_run = 1;
	repeat (slow(8000)) @(negedge clk);
	frame_data(32'h00001237, 512, 588);
	rd("ACCEPT ok after resend (drops kept)", SEC + 21'h18, 32'h804C024C);
	wait_frame(n0);
	check_buf("frame 0x1237 after FIFO-full resend", 32'h00001237, 588);

	$display("-- ABORT mid-frame zero-fills");
	n0 = cd_frames;
	cd_request(32'h00001238); settle;
	sec_wr(21'h0C, 32'h00001238);
	frame_data(32'h00001238, 0, 300);
	sec_wr(21'h10, 32'h3);
	wait_frame(n0);
	check_buf("frame 0x1238: 300 words then zeros", 32'h00001238, 300);
	axi_read(SEC + 21'h20, 0, d, dl, beats);
	check("COUNTS: rejected 0, aborted 1, fills 0", {d[31:24], d[15:0]}, dg(24'h000100));
	check("COUNTS: skipped saturated (lost-store + resend skips)", d[23:16], dg(8'hFF));
	rd("CTRL keeps EN", SEC + 21'h10, 32'h1);

	$display("-- memcard SECTOR_RD over SPI overlapping an LW frame");
	n0 = cd_frames; s0 = cd_strobes; overlap = 0;
	cd_request(32'h00001239); settle;
	sec_wr(21'h0C, 32'h00001239);
	fork
		frame_data(32'h00001239, 0, 588);
		begin repeat (slow(100)) @(negedge clk); spi_sector(16'h0217, 256, 16'h7700, 3); end
	join
	rd("ACCEPT ok", SEC + 21'h18, 32'h8000024C);
	wait_frame(n0);
	repeat (200) @(negedge clk_sys);
	check("memcard took its 256 words", mc_words, 256);
	check("memcard strobes landed while the LW frame was open", overlap, 1);
	check("the CD reader saw only its 1176 strobes", cd_strobes - s0, 1176);
	check_buf("frame 0x1239 untouched by the memcard", 32'h00001239, 588);

	$display("-- stale BEGIN rejected");
	n0 = cd_frames;
	cd_request(32'h0000123A); settle;
	sec_wr(21'h0C, 32'h00009999);
	frame_data(32'h00009999, 0, 8);
	settle;
	axi_read(SEC + 21'h14, 0, d, dl, beats);
	check("STATUS rejecting, ack low", d[3:1], 3'b100);
	axi_read(SEC + 21'h20, 0, d, dl, beats);
	check("COUNTS: rejected 1", {d[31:24], d[15:0]}, dg(24'h010100));
	sec_wr(21'h0C, 32'h0000123A);
	frame_data(32'h0000123A, 0, 588);
	rd("ACCEPT ok", SEC + 21'h18, 32'h8000024C);
	wait_frame(n0);
	check_buf("frame 0x123A after the reject", 32'h0000123A, 588);

	$display("-- core reset mid-frame zero-fills");
	n0 = cd_frames;
	cd_request(32'h0000123B); settle;
	sec_wr(21'h0C, 32'h0000123B);
	frame_data(32'h0000123B, 0, 200);
	@(negedge clk_sys) cd_reset = 1;
	repeat (4) @(negedge clk_sys);
	cd_reset = 0;
	repeat (2400) @(negedge clk_sys);
	axi_read(SEC + 21'h14, 0, d, dl, beats);
	check("STATUS idle after the fill, ack low", d[3:1], 3'b000);
	axi_read(SEC + 21'h20, 0, d, dl, beats);
	check("COUNTS: reset_fills 1", {d[31:24], d[15:0]}, dg(24'h010101));
	check("the CD reader (in reset) completed no frame", cd_frames, n0);
	frame_data(32'h0000123B, 200, 588);
	cd_request(32'h0000123C); settle;
	sec_wr(21'h0C, 32'h0000123C);
	frame_data(32'h0000123C, 0, 588);
	wait_frame(n0);
	check_buf("next frame after the reset", 32'h0000123C, 588);

	$display("-- TEST mode: SUM only, consumer untouched (LW_DIAG=0: TEST ignored, the BEGIN is rejected)");
	settle;                                     // the last frame's done event has crossed before FRAMES is read
	n0 = cd_frames; s0 = cd_strobes;
	axi_read(SEC + 21'h14, 0, d0, dl, beats);
	sec_wr(21'h10, 32'h9);
	settle;
	rd("CTRL TEST bit", SEC + 21'h10, DIAG ? 32'h9 : 32'h1);
	sec_wr(21'h0C, 32'h00ABCDEF);
	frame_data(32'h00ABCDEF, 0, 588);
	repeat (slow(8000)) @(negedge clk);
	rd("SUM", SEC + 21'h34, dg(frame_sum(32'h00ABCDEF, 588)));
	axi_read(SEC + 21'h14, 0, d, dl, beats);
	check("TEST frame counted", d[31:16] - d0[31:16], dg(1));
	if (!DIAG) check("lean: the BEGIN with no request is rejected", d[3:1], 3'b100);
	check("no strobes reached the CD reader", cd_strobes - s0, 0);
	check("no ack reached the CD reader", cd_frames, n0);
	sec_wr(21'h10, 32'h1);

	$display("-- CLR counters");
	sec_wr(21'h10, 32'h5);
	rd("COUNTS cleared", SEC + 21'h20, 32'd0);
	rd("DROPS cleared", SEC + 21'h1C, 32'd0);
	axi_read(SEC + 21'h14, 0, d, dl, beats);
	check("frames + high-water cleared", d[31:8], 0);

	$display("-- EN off: the slot goes back to hps_io");
	sec_wr(21'h10, 32'h0); settle;
	rd("STATUS en_sys 0", SEC + 21'h14, DIAG ? 32'd0 : 32'h8);   // lean: still rejecting the TEST-less BEGIN
	n0 = cd_frames;
	cause_drain(SEC);
	cd_request(32'h00002000); settle;
	check("SPI slot 1 offered", sd_rd[1], 1);
	check("doorbell still rings", doorbell, 1);
	spi_frame(32'h00002000);
	wait_frame(n0);
	check_buf("frame 0x2000 over SPI through the mux", 32'h00002000, 588);

	$display("-- CAUSE: a request racing the store-back is never lost; a raise in the clear's cycle survives");
	begin : sec_race
		integer k, lost, kept, same, gone, stale, m;
		realtime t0, lat_f, lat_w, off, lead;
		// one raise per round: the request rises (EN is 0, so nothing serves it), then a core reset drops it;
		// calibrated from the clk_sys edge each starts on; the store leads the request's edge by up to 48 ns
		cause_drain(SEC);
		fork
			begin @(negedge clk_sys) t0 = $realtime; end
			cd_request(32'h00002100);
		join
		settle; lat_f = t_fire - t0;
		@(negedge clk_sys) cd_reset = 1; repeat (4) @(negedge clk_sys); cd_reset = 0; settle;
		cause_drain(SEC);
		@(negedge clk_sys) t0 = $realtime; axi_write(SEC + 21'h00, 32'h00000000, 4'hF); lat_w = t_wsec - t0;
		m = $rtoi(64.0 / (2.0 * SYS_HP_NS)) + 2; lead = m * 2.0 * SYS_HP_NS;
		lost = 0; kept = 0; same = 0; gone = 0; stale = 0;
		for (k = 0; k < 64; k = k + 1) begin
			off = lat_f - lat_w + (k - 32) * 1.5;
			fork
				begin @(negedge clk_sys); if (lead + off > 0) #(lead + off); axi_write(SEC + 21'h00, 32'h00000001, 4'hF); end
				begin repeat (m) @(negedge clk_sys); cd_request(32'h00002200 + k); end
			join
			settle;
			if (t_fire >= t_wsec) begin kept = kept + 1; if (!doorbell) lost = lost + 1; end
			else begin gone = gone + 1; if (doorbell) stale = stale + 1; end
			if (t_fire == t_wsec) same = same + 1;
			// the drain's level read after the store-back: the request the bit no longer shows is in LBA
			axi_read(SEC + 21'h08, 0, d, dl, beats);
			if (!doorbell && d !== 32'h00002200 + k) lost = lost + 1;
			@(negedge clk_sys) cd_reset = 1; repeat (4) @(negedge clk_sys); cd_reset = 0; settle;
			if (doorbell) cause_drain(SEC);
		end
		$display("     %0d offsets: %0d raises before the clear, %0d in or after it, %0d in its cycle", 64, gone, kept,
		         same);
		check("no request lost across 64 offsets", lost, 0);
		check("some raises came before the clear", gone > 0, 1);
		check("the store cleared every raise before it", stale, 0);
		check("some raises landed in the clear's own cycle (the raise won)", same > 0, 1);
		check("no ring left after the sweep", doorbell, 0);
	end

	$display("-- memory cards: EN=0 leaves both cards on hps_io");
	cause_drain(SEC);                                 // clears the CD's pending cause
	n0 = mc1.done; s0 = mc1.strobes;
	mc1.go(0, 7'h05); settle;
	check("card 1 rd offered to hps_io", sd_rd[2], 1);
	check("card EN=0: no doorbell", doorbell, 0);
	rd("card EN=0: REQ idle", SEC + 21'h04, 32'd0);
	spi_sector(16'h0217, 512, 16'h3100, 3);
	repeat (50) @(negedge clk_sys);
	check("card 1 SPI read completed", mc1.done - n0, 1);
	check("card 1 SPI took 512 strobes", mc1.strobes - s0, 512);
	bad = 0; for (i = 0; i < 512; i = i + 1) if (mc1.ram[i] !== 16'h3100 + i) bad = bad + 1;
	check("card 1 SPI buffer", bad, 0);

	$display("-- memory cards: CTRL enables (CD bit 0, card 1 bit 6, card 2 bit 7)");
	sec_wr(21'h10, 32'hC1); settle;
	rd("CTRL readback EN mask", SEC + 21'h10, 32'h000000C1);

	$display("-- card 1 READ over LW");
	n0 = mc1.done; s0 = mc1.strobes; r = cd_strobes;
	mc1.go(0, 7'h12); settle;
	check("card 1 hidden from hps_io", sd_rd[2], 0);
	check("card rd rings", doorbell, 1);
	rd("CAUSE: the event bit", SEC + 21'h00, 32'h00000001);
	cause_drain(SEC);
	rd("REQ card 1 read", SEC + 21'h04, 32'h800080C9);
	rd("LBA_S[2]", SEC + 21'h68, 32'h00000012);
	rd("LBA_S[0] reads 0", SEC + 21'h60, 32'd0);
	sec_wr(21'h48, 32'h00000012);
	rd("REQ hides the answered read", SEC + 21'h04, 32'd0);
	frame_data(32'h00000012, 0, 256);
	rd("ACCEPT ok, 256", SEC + 21'h18, 32'h80000100);
	repeat (1500) @(negedge clk_sys);
	check("card 1 frame done", mc1.done - n0, 1);
	check("card 1 took 512 strobes", mc1.strobes - s0, 512);
	bad = 0; for (i = 0; i < 256; i = i + 1) if ({mc1.ram[2*i+1], mc1.ram[2*i]} !== pat(32'h12, i)) bad = bad + 1;
	check("card 1 buffer byte-exact (1 KB)", bad, 0);
	check("the CD reader saw no card strobe", cd_strobes - r, 0);
	rd("REQ idle after the frame", SEC + 21'h04, 32'd0);

	$display("-- card 2 WRITE answered by WACK");
	n0 = mc2.done; s0 = mc2.strobes; r = mc1.strobes;
	mc2.go(1, 7'h7F); settle;
	check("card 2 wr hidden from hps_io", sd_wr[3], 0);
	rd("CAUSE: the event bit again", SEC + 21'h00, 32'h00000001);
	cause_drain(SEC);
	rd("REQ card 2 write", SEC + 21'h04, 32'h800080CE);
	rd("LBA_S[3]", SEC + 21'h6C, 32'h0000007F);
	sec_wr(21'h5C, 32'h0000007F);
	rd("ACCEPT ok after a WACK (length 0)", SEC + 21'h18, 32'h80000000);
	repeat (100) @(negedge clk_sys);
	check("card 2 write acked", mc2.done - n0, 1);
	check("WACK puts no strobe on card 2", mc2.strobes - s0, 0);
	check("nor on card 1", mc1.strobes - r, 0);
	rd("REQ idle after the WACK", SEC + 21'h04, 32'd0);

	$display("-- CD first: CD and card 1 pending together");
	n0 = cd_frames; s0 = mc1.strobes; r = mc1.done;
	fork
		cd_request(32'h00003000);
		mc1.go(0, 7'h20);
	join
	settle;
	rd("REQ: the CD first", SEC + 21'h04, 32'h800080C5);
	rd("CAUSE: one event bit for both", SEC + 21'h00, 32'h00000001);
	cause_drain(SEC);
	sec_wr(21'h44, 32'h00003000);                          // BEGIN_S[1] aliases BEGIN
	rd("REQ: answered CD hidden, card 1 next", SEC + 21'h04, 32'h800080C9);
	frame_data(32'h00003000, 0, 588);
	rd("ACCEPT ok, 588", SEC + 21'h18, 32'h8000024C);
	wait_frame(n0);
	check_buf("CD frame 0x3000", 32'h00003000, 588);
	check("the CD frame put no strobe on card 1", mc1.strobes - s0, 0);
	n0 = cd_strobes;
	sec_wr(21'h48, 32'h00000020);
	frame_data(32'h00000020, 0, 256);
	repeat (1500) @(negedge clk_sys);
	check("card 1 frame after the CD", mc1.done - r, 1);
	bad = 0; for (i = 0; i < 256; i = i + 1) if ({mc1.ram[2*i+1], mc1.ram[2*i]} !== pat(32'h20, i)) bad = bad + 1;
	check("card 1 buffer after the CD", bad, 0);
	check("the card frame put no strobe on the CD reader", cd_strobes - n0, 0);

	$display("-- stale and mismatched card answers rejected");
	r = mc1.done; s0 = mc1.strobes;
	mc1.go(0, 7'h21); settle;
	cause_drain(SEC);
	sec_wr(21'h48, 32'h00000099); settle;
	axi_read(SEC + 21'h14, 0, d, dl, beats);
	check("STATUS rejecting after a stale BEGIN_S", d[3:1], 3'b100);
	sec_wr(21'h58, 32'h00000021); settle;                   // WACK for a READ request
	axi_read(SEC + 21'h14, 0, d, dl, beats);
	check("STATUS rejecting after a WACK on a read", d[3:1], 3'b100);
	sec_wr(21'h5C, 32'h00000021); settle;                   // a slot with no request
	check("no card strobe, no ack", mc1.strobes - s0 + mc1.done - r, 0);
	sec_wr(21'h48, 32'h00000021);
	frame_data(32'h00000021, 0, 256);
	repeat (1500) @(negedge clk_sys);
	check("card 1 frame after the rejects", mc1.done - r, 1);
	bad = 0; for (i = 0; i < 256; i = i + 1) if ({mc1.ram[2*i+1], mc1.ram[2*i]} !== pat(32'h21, i)) bad = bad + 1;
	check("card 1 buffer after the rejects", bad, 0);

	$display("-- reset mid card frame zero-fills");
	s0 = mc1.strobes;
	mc1.go(0, 7'h22); settle;
	sec_wr(21'h48, 32'h00000022);
	frame_data(32'h00000022, 0, 100);
	repeat (600) @(negedge clk_sys);
	@(negedge clk_sys) cd_reset = 1;
	repeat (4) @(negedge clk_sys);
	cd_reset = 0;
	repeat (1500) @(negedge clk_sys);
	check("the fill completed 512 strobes", mc1.strobes - s0, 512);
	bad = 0; for (i = 0; i < 256; i = i + 1) if ({mc1.ram[2*i+1], mc1.ram[2*i]} !== ((i < 100) ? pat(32'h22, i) : 32'd0)) bad = bad + 1;
	check("card 1: 100 words then zeros", bad, 0);
	axi_read(SEC + 21'h14, 0, d, dl, beats);
	check("STATUS idle after the card fill", d[3:1], 3'b000);

	$display("-- core reset during a card load: the stale BEGIN_S is rejected, the request re-opens");
	r = mc1.done; s0 = mc1.strobes;
	mc1.go(0, 7'h30); settle;
	cause_drain(SEC);                                    // the HPS drains the cause
	rd("REQ card 1 read", SEC + 21'h04, 32'h800080C9);
	rd("LBA_S[2] 0x30", SEC + 21'h68, 32'h00000030);       // the HPS fills LBA 0x30 ...
	@(negedge clk_sys) cd_reset = 1;
	repeat (4) @(negedge clk_sys);
	cd_reset = 0;
	mc1.go(0, 7'h00); settle;                              // ... the load restarts at LBA 0
	cause_drain(SEC);                                    // the restart's ring, drained early
	sec_wr(21'h48, 32'h00000030); settle;                  // the stale answer
	axi_read(SEC + 21'h14, 0, d, dl, beats);
	check("STATUS rejecting after the stale BEGIN_S", d[3:1], 3'b100);
	check("the reject re-rings the doorbell", doorbell, 1);
	rd("REQ shows card 1 again", SEC + 21'h04, 32'h800080C9);
	rd("LBA_S[2] 0", SEC + 21'h68, 32'h00000000);
	cause_drain(SEC);
	sec_wr(21'h48, 32'h00000000);
	frame_data(32'h00000000, 0, 256);
	repeat (1500) @(negedge clk_sys);
	check("card 1 frame at LBA 0 after the reject", mc1.done - r, 1);
	bad = 0; for (i = 0; i < 256; i = i + 1) if ({mc1.ram[2*i+1], mc1.ram[2*i]} !== pat(32'h00, i)) bad = bad + 1;
	check("card 1 buffer LBA 0", bad, 0);
	rd("REQ idle after the re-served frame", SEC + 21'h04, 32'd0);
	check("no ring left", doorbell, 0);

	$display("-- a stale reject behind a newer answer re-opens nothing (the entry's tag)");
	r = mc1.done; n0 = cd_frames;
	mc1.go(0, 7'h31); settle;
	cd_request(32'h00004000); settle;
	cause_drain(SEC);
	sec_wr(21'h0C, 32'h00004000);
	frame_data(32'h00004000, 0, 588);                       // the feeder is still streaming it ...
	sec_wr(21'h48, 32'h00000077);                           // ... when both card answers queue
	sec_wr(21'h48, 32'h00000031);
	wait_frame(n0);                                         // (the card data after: FIFO depth 512)
	frame_data(32'h00000031, 0, 256);
	repeat (1500) @(negedge clk_sys);
	check("card 1 frame at 0x31", mc1.done - r, 1);
	bad = 0; for (i = 0; i < 256; i = i + 1) if ({mc1.ram[2*i+1], mc1.ram[2*i]} !== pat(32'h31, i)) bad = bad + 1;
	check("card 1 buffer 0x31", bad, 0);
	check("the older answer's reject rang nothing", doorbell, 0);
	rd("REQ idle", SEC + 21'h04, 32'd0);

	$display("-- the card's LBA moves while its request stays high: the reject re-latches it");
	r = mc1.done;
	mc1.go(0, 7'h40); settle;
	cause_drain(SEC);
	rd("LBA_S[2] 0x40", SEC + 21'h68, 32'h00000040);
	@(negedge clk_sys) mc1.lba = 7'h41;                     // rd never fell
	settle;
	rd("LBA_S[2] still the latched 0x40", SEC + 21'h68, 32'h00000040);
	sec_wr(21'h48, 32'h00000040); settle;                   // answers the LBA it was shown
	check("the moved LBA's answer re-rings", doorbell, 1);
	rd("REQ shows card 1 again", SEC + 21'h04, 32'h800080C9);
	rd("LBA_S[2] re-latched 0x41", SEC + 21'h68, 32'h00000041);
	cause_drain(SEC);
	sec_wr(21'h48, 32'h00000041);
	frame_data(32'h00000041, 0, 256);
	repeat (1500) @(negedge clk_sys);
	check("card 1 frame at 0x41", mc1.done - r, 1);
	bad = 0; for (i = 0; i < 256; i = i + 1) if ({mc1.ram[2*i+1], mc1.ram[2*i]} !== pat(32'h41, i)) bad = bad + 1;
	check("card 1 buffer 0x41", bad, 0);
	rd("REQ idle after the moved LBA", SEC + 21'h04, 32'd0);
	check("no ring left after the moved LBA", doorbell, 0);

	$display("-- a card's reject re-opens that slot only: the CD answered behind it stays hidden");
	r = mc1.done; s0 = mc2.done; n0 = cd_frames;
	mc1.go(0, 7'h70);
	mc2.go(0, 7'h10);
	cd_request(32'h00006000); settle;
	cause_drain(SEC);
	sys_run = 0;                                            // every answer queues before the feeder runs
	sec_wr(21'h48, 32'h00000071);                           // card 1, the wrong LBA: rejected first
	sec_wr(21'h4C, 32'h00000010);                           // card 2, right: its frame holds the feeder
	frame_data(32'h00000010, 0, 256);
	sec_wr(21'h0C, 32'h00006000);                           // the CD, right, behind card 2
	sys_run = 1; repeat (40) @(negedge clk_sys);            // the reject is in; card 2 still streams
	rd("REQ shows card 1, not the answered CD", SEC + 21'h04, 32'h800080C9);
	repeat (1500) @(negedge clk_sys);
	check("card 2 frame at 0x10", mc2.done - s0, 1);
	frame_data(32'h00006000, 0, 588);
	wait_frame(n0);
	check_buf("CD buffer 0x6000 behind the card's reject", 32'h00006000, 588);
	cause_drain(SEC);
	sec_wr(21'h48, 32'h00000070);
	frame_data(32'h00000070, 0, 256);
	repeat (1500) @(negedge clk_sys);
	check("card 1 frame at 0x70", mc1.done - r, 1);
	rd("REQ idle after all three", SEC + 21'h04, 32'd0);
	check("no ring left after all three", doorbell, 0);

	$display("-- adjacent rejects of two slots: both requests re-open");
	r = mc1.done; s0 = mc2.done;
	mc1.go(0, 7'h24);
	mc2.go(0, 7'h34); settle;
	cause_drain(SEC);
	sys_run = 0;                                            // both stale answers queue back to back
	sec_wr(21'h48, 32'h00000025);
	sec_wr(21'h4C, 32'h00000035);
	sys_run = 1; settle;
	rd("REQ shows card 1 re-opened", SEC + 21'h04, 32'h800080C9);
	cause_drain(SEC);
	sec_wr(21'h48, 32'h00000024);
	frame_data(32'h00000024, 0, 256);
	repeat (1500) @(negedge clk_sys);
	check("card 1 frame at 0x24", mc1.done - r, 1);
	rd("REQ shows card 2 re-opened", SEC + 21'h04, 32'h800080CD);
	cause_drain(SEC);
	sec_wr(21'h4C, 32'h00000034);
	frame_data(32'h00000034, 0, 256);
	repeat (1500) @(negedge clk_sys);
	check("card 2 frame at 0x34", mc2.done - s0, 1);
	rd("REQ idle after both re-opened", SEC + 21'h04, 32'd0);

	$display("-- card EN off: back to hps_io, no doorbell");
	sec_wr(21'h10, 32'h1); settle;
	cause_drain(SEC);
	n0 = mc2.done; s0 = mc2.strobes;
	mc2.go(1, 7'h44); settle;
	check("card 2 wr offered to hps_io", sd_wr[3], 1);
	check("no doorbell for a disabled card", doorbell, 0);
	rd("REQ idle for a disabled card", SEC + 21'h04, 32'd0);
	spi_sector(16'h0318, 512, 16'h0, 3);                   // the stock save drain, hps_io reads the buffer
	repeat (50) @(negedge clk_sys);
	check("card 2 SPI write completed", mc2.done - n0, 1);
	check("SPI write: no strobe into card 2", mc2.strobes - s0, 0);
	sec_wr(21'h10, 32'h0); settle;

	if (DIAG) begin
		$display("-- DIAG: SUM and FRAMES move on the same edge");
		$display("     %0d SUM changes, %0d without FRAMES", sum_moves, sum_alone);
		check("SUM changed during the SECTOR cases", sum_moves > 0, 1);
		check("SUM never changed without FRAMES", sum_alone, 0);
	end

	sec_expiry_cases;
	lease_up;
`endif

	$display("-- SYS: the status back-channel (hps_io 0x29) as a cause, a line and a snapshot");
	rd("SYS ID STS1", SYSB + 21'h1C, 32'h31535453);
	check("SYS line low at reset", sys_irq, 0);
	rd("SYS CAUSE idle", SYSB, 32'd0);
	rd("SYS STFLG at reset", SYSB + 21'h04, 32'h000000A0);
	st_req(st_pat(1));
	settle;
	check("SYS line raised by status_set", sys_irq, 1);
	check("SECTOR line untouched", doorbell, 0);
	rsize = 0; axi_read(SYSB, 0, d, dl, beats); rsize = 2;
	check("SYS byte read returns the cause", d, 32'h00000001);
	check("SYS byte read does not clear", sys_irq, 1);
	rd("SYS CAUSE: bit 0 STATUS", SYSB, 32'h00000001);
	check("SYS word read does not clear", sys_irq, 1);
	arburst = 2'b10; axi_read(SYSB, 4'd3, d, dl, beats); arburst = 2'b01;
	check("SYS WRAP burst over CAUSE does not clear", sys_irq, 1);
	axi_write(SYSB, 32'hFFFFFFFE, 4'hF);
	check("SYS store with bit 0 clear clears nothing", sys_irq, 1);
	axi_write(SYSB, 32'h00000001, 4'h1);
	check("SYS partial-width store clears nothing", sys_irq, 1);
	rd("SYS CAUSE kept", SYSB, 32'h00000001);
	axi_write(SYSB, 32'h00000001, 4'hF);
	check("SYS store of bit 0 clears the line", sys_irq, 0);
	rd("SYS CAUSE idle after the store", SYSB, 32'd0);
	st_read(stf0, stf1, stw);
	check("STFLG {seq 1, A, flag 1}", stf0, 32'h000100A1);
	check("STFLG stable across the words", stf1, stf0);
	check("snapshot words [31:0]", stw[31:0], st_pat(1) >> 0);
	check("snapshot words [63:32]", stw[63:32], st_pat(1) >> 32);
	check("snapshot words [95:64]", stw[95:64], st_pat(1) >> 64);
	check("snapshot words [127:96]", stw[127:96], st_pat(1) >> 96);
	rd("SYS register hole", SYSB + 21'h18, 32'd0);
	rd("SYS past the registers", SYSB + 21'h400, 32'd0);

	$display("-- SYS: the SPI 0x29 path answers the same flag and words (hps_io unchanged)");
	begin : spi29
		reg [15:0] w29 [0:8];
		integer m;
		@(negedge clk_sys); io_enable = 1;
		for (m = 0; m < 9; m = m + 1) begin
			io_din = (m == 0) ? 16'h0029 : 16'h0000; io_strobe = 1; @(negedge clk_sys); io_strobe = 0;
			@(negedge clk_sys); w29[m] = hps_io.io_dout;
		end
		io_enable = 0; @(negedge clk_sys);
		check("0x29 head {A, flag 1}", {16'd0, w29[0]}, 32'h000000A1);
		bad = 0;
		stw = st_pat(1);
		for (m = 0; m < 8; m = m + 1) if (w29[m + 1] !== stw[16*m +: 16]) bad = bad + 1;
		check("0x29 words equal the LW snapshot", bad, 0);
	end
	check("an SPI read raises no SYS cause", sys_irq, 0);

	$display("-- SYS: requests at the fastest rate (every 2 clk_sys) coalesce to the last");
	for (i = 2; i < 6; i = i + 1) st_req(st_pat(i));
	settle;
	check("SYS line raised once for the burst", sys_irq, 1);
	rd("SYS CAUSE", SYSB, 32'h00000001);
	axi_write(SYSB, 32'h00000001, 4'hF);
	check("cleared by one store", sys_irq, 0);
	st_read(stf0, stf1, stw);
	check("STFLG flag 5 (the core's)", {stf0[7:4], stf0[3:0]}, {4'hA, hps_io.stflg});
	check("STFLG stable", stf1, stf0);
	check("snapshot is the last request", stw == st_pat(5), 1);

	$display("-- SYS: a capture racing the store-back is never lost; a raise in the clear's cycle survives");
	begin : race
		integer k, lost, kept, same, gone, stale, m;
		realtime t0, lat_c, lat_w, off, lead;
		// calibrate from the clk_sys edge each starts on: a request's capture, and a store's clear
		fork
			begin @(negedge clk_sys) t0 = $realtime; end
			st_req(st_pat(99));
		join
		settle; lat_c = t_cap - t0;
		axi_write(SYSB, 32'h00000001, 4'hF);
		@(negedge clk_sys) t0 = $realtime; axi_write(SYSB, 32'h00000000, 4'hF); lat_w = t_wsys - t0;
		// the request starts m clk_sys edges after the store's reference edge, so a store can lead it by 64 ns
		m = $rtoi(64.0 / (2.0 * SYS_HP_NS)) + 2; lead = m * 2.0 * SYS_HP_NS;
		lost = 0; kept = 0; same = 0; gone = 0; stale = 0;
		for (k = 0; k < 64; k = k + 1) begin
			off = lat_c - lat_w + (k - 32) * 1.5;   // the store's start after the request's edge
			fork
				begin @(negedge clk_sys); if (lead + off > 0) #(lead + off); axi_write(SYSB, 32'h00000001, 4'hF); end
				begin repeat (m) @(negedge clk_sys); st_req(st_pat(100 + k)); end
			join
			settle;
			// a capture in or after the clear's cycle keeps the bit: the line stays up
			if (t_cap >= t_wsys) begin kept = kept + 1; if (!sys_irq) lost = lost + 1; end
			else begin gone = gone + 1; if (sys_irq) stale = stale + 1; end
			if (t_cap == t_wsys) same = same + 1;
			// the drain's level read after the store-back: a capture the bit no longer shows is in the snapshot
			st_read(stf0, stf1, stw);
			if (!sys_irq && (stf0[3:0] !== hps_io.stflg || stw !== st_pat(100 + k))) lost = lost + 1;
			if (sys_irq) cause_drain(SYSB);
		end
		$display("     %0d offsets: %0d captures before the clear, %0d in or after it, %0d in its cycle", 64, gone,
		         kept, same);
		check("no request lost across 64 offsets", lost, 0);
		check("some captures came before the clear", gone > 0, 1);
		check("the store cleared every capture before it", stale, 0);
		check("some captures landed in the clear's own cycle (the raise won)", same > 0, 1);
		check("line idle after the sweep", sys_irq, 0);
	end

	$display("-- SYS: a capture mid-read changes STFLG, so the seqlock retries");
	axi_read(SYSB + 21'h04, 0, stf0, dl, beats);
	st_req(st_pat(200));
	settle;
	axi_read(SYSB + 21'h04, 0, stf1, dl, beats);
	check("STFLG moved across the capture", stf0 != stf1, 1);
	rd("SYS CAUSE", SYSB, 32'h00000001);

	$display("-- OSDBUF: registers");
	rd("OSD ID OSD1", OSD + 21'h1004, 32'h3144534F);
	rd("OSD CTRL reset", OSD + 21'h1000, 32'd0);
	rd("OSD image reads 0", OSD + 21'h0000, 32'd0);
	rd("OSD image end reads 0", OSD + 21'h0FFC, 32'd0);
	rd("OSD register hole", OSD + 21'h1008, 32'd0);
	rd("OSD block end", OSD + 21'h1FFC, 32'd0);
	check("LWSEL/HR low at reset", osd_ctrl, 0);

	$display("-- OSDBUF: fill both images (LW 1024 stores, SPI 16 rows)");
	for (r = 0; r < 16; r = r + 1) lw_row(r, 0);
	for (r = 0; r < 16; r = r + 1) spi_osd_row(r, 0);
	spi_osd_cmd(8'h40);                   // rows 8-15 re-armed highres; DISABLE clears it
	spi_osd_cmd(8'h41);
	check("osd_status after ENABLE", osd_status, 1);
	bad = 0;
	for (i = 0; i < 1024; i = i + 1) if (lanes_h(i) !== img_w(i) || lanes_v(i) !== img_w(i)) bad = bad + 1;
	check("both shadows hold the LW image", bad, 0);
	bad = 0;
	for (i = 0; i < 4096; i = i + 1) if (hdmi_osd.osd_buffer[i] !== spi_img[i] || vga_osd.osd_buffer[i] !== spi_img[i]) bad = bad + 1;
	check("both SPI buffers hold the SPI image", bad, 0);

	$display("-- OSDBUF: one store at (row 12, col 100) and a byte-lane store at (row 3, col 41)");
	osd_word(12*64 + 25, 32'hC3A55A3C, 4'hF);
	repeat (4) @(negedge clk);
	check("hdmi shadow word (12, 100..103)", lanes_h(12*64 + 25), 32'hC3A55A3C);
	check("vga shadow word (12, 100..103)", lanes_v(12*64 + 25), 32'hC3A55A3C);
	osd_word(3*64 + 10, 32'h0000E700, 4'h2);
	repeat (4) @(negedge clk);
	check("hdmi byte lane 1 only", lanes_h(3*64 + 10), img_w(3*64 + 10));
	check("vga byte lane 1 only", lanes_v(3*64 + 10), img_w(3*64 + 10));
	check("the stored byte", lw_img[3*256 + 41], 32'hE7);
	check("SPI buffer untouched by LW stores", hdmi_osd.osd_buffer[12*256 + 100], spi_img[12*256 + 100]);

	$display("-- OSDBUF: pixel path, SPI image (LWSEL=0, highres 0)");
	osd_check("LWSEL=0", 0, 64, 6);
	axi_write(OSD + 21'h1000, 32'h1, 4'hF);
	rd("CTRL LWSEL", OSD + 21'h1000, 32'h1);
	osd_check("LWSEL=1 HR=0", 1, 64, 3);
	axi_write(OSD + 21'h1000, 32'h3, 4'hF);
	osd_check("LWSEL=1 HR=1: 128 lines", 1, 128, 3);
	axi_write(OSD + 21'h1000, 32'h2, 4'hF);
	rd("CTRL HR only", OSD + 21'h1000, 32'h2);
	osd_check("HR without LWSEL is ignored", 0, 64, 3);

	$display("-- OSDBUF: SPI row and LW row in flight together (LWSEL=1)");
	axi_write(OSD + 21'h1000, 32'h3, 4'hF);
	fork
		spi_osd_row(2, 1);
		lw_row(5, 1);
	join
	bad = 0;
	for (i = 0; i < 256; i = i + 1) if (hdmi_osd.osd_buffer[2*256 + i] !== spi_img[2*256 + i] || vga_osd.osd_buffer[2*256 + i] !== spi_img[2*256 + i]) bad = bad + 1;
	check("SPI row 2 landed in both SPI buffers", bad, 0);
	bad = 0;
	for (i = 5*64; i < 6*64; i = i + 1) if (lanes_h(i) !== img_w(i) || lanes_v(i) !== img_w(i)) bad = bad + 1;
	check("LW row 5 landed in both shadows", bad, 0);
	osd_check("LWSEL=1 shows the shadow", 1, 128, 3);
	axi_write(OSD + 21'h1000, 32'h0, 4'hF);
	osd_check("LWSEL=0 shows the SPI buffer (row 2 new)", 0, 64, 3);

	$display("-- OSDBUF: highres = spi_highres | (LWSEL & HR)");
	spi_osd_row(9, 2);
	osd_check("SPI row 9 (bit 3) re-arms SPI highres", 0, 128, 3);
	spi_osd_cmd(8'h40);
	check("osd_status after DISABLE", osd_status, 0);
	osd_check("OSD disabled: no box", 0, 0, 3);
	axi_write(OSD + 21'h1000, 32'h3, 4'hF);
	spi_osd_cmd(8'h41);
	osd_check("LWSEL&HR survives DISABLE/ENABLE", 1, 128, 6);
	axi_write(OSD + 21'h1000, 32'h0, 4'hF);
	osd_check("SPI highres stays cleared by DISABLE", 0, 64, 3);

`ifdef MISTER_LW_LANE
	$display("-- LANE: registers");
	rd("LANE ID LAN1", LANE + 21'h18, 32'h314E414C);
	rd("LANE PLAYED reset", LANE + 21'h10, 32'd0);
	rd("LANE DROPS reset", LANE + 21'h14, 32'd0);
	rd("LANE RSTAT reset", LANE + 21'h1C, 32'd0);
	rd("LANE RHEAD reads 0 while empty", LANE + 21'h20, 32'd0);
	rd("LANE BEGIN reads 0", LANE, 32'd0);
	rd("LANE register hole", LANE + 21'h24, 32'd0);
	rd("LANE block end", LANE + 21'hFFC, 32'd0);
	check("player idle", {lane_own, lane_hold}, 0);

	lo_min = 1000;

	$display("-- LANE 1: a status burst");
	f_set(0, 16'h001E, 8);
	for (i = 0; i < 8; i = i + 1) fw[i] = 16'h1100 + 16'h0101 * i;
	lane_vs_link("lane 1 status");
	check("lane 1: status from the lane", hst[31:0], 32'h12011100);
	check("lane 1: nine strobes", bus_n, 9);

	$display("-- LANE 2: 0x22 then 0x24, one toggle each");
	f_set(0, 16'h0022, 4);
	for (i = 0; i < 4; i = i + 1) fw[i] = 16'h2200 + i;
	lane_vs_link("lane 2 RTC");
	check("lane 2: RTC toggled once", rtc_t, 1);
	check("lane 2: RTC words", rtc[63:32], 32'h22032202);
	check("lane 2: RTC words low", rtc[31:0], 32'h22012200);
	f_set(0, 16'h0024, 2);
	fw[0] = 16'h2400; fw[1] = 16'h2401;
	lane_vs_link("lane 2 TIMESTAMP");
	check("lane 2: TIMESTAMP toggled once", ts_t, 1);
	check("lane 2: TIMESTAMP words", tstamp[31:0], 32'h24012400);

	$display("-- LANE 3: a mount (0x1C) and its size (0x1D)");
	f_set(0, 16'h001C, 1);
	fw[0] = 16'h0082;                          // slot 1, read-only
	lane_vs_link("lane 3 mount");
	check("lane 3: one img_mounted pulse", mnt_n, 1);
	check("lane 3: img_mounted = slot 1", mnt_v, 4'b0010);
	check("lane 3: read-only", img_ro, 1);
	$display("     img_mounted width: link %0d, lane %0d clk_sys", t_ref[8], mnt_w);
	check("lane 3: lane pulse = the word's ack wait plus the END fetch (GAP + 1)", mnt_w, win.LANE_GAP + 1);
	check("lane 3: no shorter than the fastest link handshake", mnt_w >= t_ref[8], 1);
	f_set(0, 16'h001D, 4);
	for (i = 0; i < 4; i = i + 1) fw[i] = 16'h1D00 + i;
	lane_vs_link("lane 3 size");
	check("lane 3: img_size high", img_sz[63:32], 32'h1D031D02);
	check("lane 3: img_size low", img_sz[31:0], 32'h1D011D00);

	$display("-- LANE 4: gamma enable and a table (3 words per entry)");
	f_set(0, 16'h0032, 1);
	fw[0] = 16'h0001;
	lane_vs_link("lane 4 gamma_en");
	f_set(0, 16'h0033, 48);
	for (i = 0; i < 48; i = i + 1) fw[i] = {i[7:0], 8'h40 ^ i[7:0]};
	lane_vs_link("lane 4 table");
	check("lane 4: 48 gamma writes", gam_n, 48);

	$display("-- LANE 5: 0x30 holds its frame at vs_wait; the frame behind it waits");
	begin : l5
		reg [31:0] p, dr;
		trk_clear;
		spin_max = 0;
		f_set(0, 16'h0030, 0);
		fork
			link_send;
			begin repeat (300) @(negedge clk_sys); tb_vsync = 1; @(negedge clk_sys); tb_vsync = 0; end
		join
		f_set(0, 16'h001E, 8);
		for (i = 0; i < 8; i = i + 1) fw[i] = 16'h5500 + i;
		link_send;
		trk_save;
		check("lane 5 link: ack held to the vsync", spin_max >= 290, 1);
		for (i = 0; i < 8; i = i + 1) fw[i] = 16'h0000;
		link_send;
		lane_count(p, dr);
		trk_clear;
		f_set(0, 16'h0030, 0);
		lane_send(1);
		f_set(0, 16'h001E, 8);
		for (i = 0; i < 8; i = i + 1) fw[i] = 16'h5500 + i;
		lane_send(1);
		repeat (300) @(negedge clk_sys);
		check("lane 5: vs_wait set by the lane's 0x30", vs_wait, 1);
		check("lane 5: the player holds its frame", lane_own, 1);
		check("lane 5: only the 0x30 word strobed", bus_n, 1);
		check("lane 5: status not yet written", hst[15:0], 16'h0000);
		lane_count(dr, d);
		check("lane 5: PLAYED unchanged while held", dr, p);
		@(negedge clk_sys); tb_vsync = 1; @(negedge clk_sys); tb_vsync = 0;
		lane_wait(p + 2);
		trk_cmp("lane 5");
	end

	$display("-- LANE 6: ioctl_wait held high, the player holds after the strobe");
	begin : l6
		reg [31:0] p, dr;
		f_set(0, 16'h001E, 8);
		for (i = 0; i < 8; i = i + 1) fw[i] = 16'h6600 + i;
		trk_clear;
		ioctl_wait = 1;
		fork
			link_send;
			begin repeat (200) @(negedge clk_sys); check("lane 6 link: one strobe while waiting", bus_n, 1); ioctl_wait = 0; end
		join
		trk_save;
		for (i = 0; i < 8; i = i + 1) fw[i] = ~(16'h6600 + i);
		link_send;
		for (i = 0; i < 8; i = i + 1) fw[i] = 16'h6600 + i;
		lane_count(p, dr);
		trk_clear;
		ioctl_wait = 1;
		lane_send(1);
		repeat (200) @(negedge clk_sys);
		check("lane 6: one strobe while waiting", bus_n, 1);
		check("lane 6: the player holds", lane_own, 1);
		lane_count(dr, d);
		check("lane 6: PLAYED unchanged while held", dr, p);
		@(negedge clk_sys); ioctl_wait = 0;
		lane_wait(p + 1);
		trk_cmp("lane 6");
	end

	$display("-- LANE 7: a link transaction raised mid-frame lands intact after it");
	begin : l7
		reg [31:0] p, dr;
		// reference: the gamma frame, then the status burst, both as link words
		trk_clear;
		f_set(0, 16'h0033, 48);
		for (i = 0; i < 48; i = i + 1) fw[i] = {i[7:0] + 8'h80, 8'h77 ^ i[7:0]};
		link_send;
		f_set(0, 16'h001E, 8);
		for (i = 0; i < 8; i = i + 1) fw[i] = 16'h7700 + i;
		link_send;
		trk_save;
		for (i = 0; i < 8; i = i + 1) fw[i] = 16'h0000;
		link_send;
		lane_count(p, dr);
		trk_clear;
		f_set(0, 16'h0033, 48);
		for (i = 0; i < 48; i = i + 1) fw[i] = {i[7:0] + 8'h80, 8'h77 ^ i[7:0]};
		lane_send(1);
		wait (lane_own);
		repeat (40) @(negedge clk_sys);
		check("lane 7: the lane is mid-frame", lane_own && bus_n > 1 && bus_n < 49, 1);
		f_set(0, 16'h001E, 8);
		for (i = 0; i < 8; i = i + 1) fw[i] = 16'h7700 + i;
		spin_max = 0;
		fork
			link_send;
			begin
				wait (!lane_own); lane_end_t = bus_n;
				@(negedge clk_sys);
				while (!(m_strobe && m_sel[0])) @(negedge clk_sys);
				link_first_t = bus_n;
			end
		join
		lane_wait(p + 1);
		trk_cmp("lane 7");
		check("lane 7: the lane frame ended before the link word", lane_end_t, 49);
		check("lane 7: the link word strobed first after it", link_first_t, 49);
		$display("     held link word: ack after %0d clk_sys (bound (1 + 48) x (GAP + 2) + 4 = %0d)",
		         spin_max, 49 * (win.LANE_GAP + 2) + 4);
		check("lane 7: the held word's spin within one frame", spin_max <= 49 * (win.LANE_GAP + 2) + 4, 1);
	end

	$display("-- LANE 8: a full FIFO drops whole frames and counts them; BVALID is never withheld");
	begin : l8
		reg [31:0] p, dr, p1, dr1;
		integer k, nfit;
		nfit = ((1 << win.LANE_AW) - 1) / 10;     // frames of 10 entries that fit DEPTH - 1
		lane_count(p, dr);
		@(negedge clk_sys); io_enable = 1;          // the link holds its select: the player cannot start
		f_set(0, 16'h001E, 8);
		for (k = 0; k < nfit + 2; k = k + 1) begin
			for (i = 0; i < 8; i = i + 1) fw[i] = k * 16 + i;
			lane_send(1);
		end
		lane_count(p1, dr1);
		check("lane 8: (DEPTH - 1) / 10 frames of 10 entries fit, two dropped", dr1 - dr, 2);
		check("lane 8: nothing played while the link is selected", p1, p);
		@(negedge clk_sys); io_enable = 0;
		lane_wait(p + nfit);
		check("lane 8: the last committed frame's status", hst[31:0], {16'((nfit - 1) * 16 + 1), 16'((nfit - 1) * 16)});
		check("lane 8: status word 7", hst[127:112], 16'((nfit - 1) * 16 + 7));
		lane_count(p1, dr1);
		check("lane 8: DROPS unchanged by the drain", dr1 - dr, 2);
	end

	check("lanes 1-8: the selects drop at least two cycles between transactions", lo_min >= 2, 1);

	$display("-- LANE 9: a frame with no END never plays");
	begin : l9
		reg [31:0] p, dr, p1, dr1, h0;
		lane_count(p, dr);
		h0 = hst[31:0];
		f_set(0, 16'h001E, 8);
		for (i = 0; i < 8; i = i + 1) fw[i] = 16'h9900 + i;
		lane_send(0);
		repeat (400) @(negedge clk_sys);
		lane_count(p1, dr1);
		check("lane 9: PLAYED unchanged", p1, p);
		check("lane 9: status untouched", hst[31:0], h0);
		check("lane 9: the player stays idle", lane_own, 0);
		for (i = 0; i < 8; i = i + 1) fw[i] = 16'h9A00 + i;
		lane_send(1);                              // BEGIN abandons the open frame
		lane_wait(p + 1);
		check("lane 9: only the ended frame played", hst[31:0], 32'h9A019A00);
		check("lane 9: its last word", hst[127:112], 16'h9A07);
		axi_write(LANE + 21'h4, 32'h0000BEEF, 4'hF);   // WORD and END with no frame open
		axi_write(LANE + 21'h8, 32'd0, 4'hF);
		repeat (400) @(negedge clk_sys);
		lane_count(p1, dr1);
		check("lane 9: a stray WORD/END plays nothing", p1, p + 1);
		check("lane 9: and drops nothing", dr1, dr);
	end

	$display("-- LANE 10: a full gamma table (256 entries, 768 words) and the longest frame the FIFO holds");
	f_set(0, 16'h0033, 768);
	for (i = 0; i < 768; i = i + 1) fw[i] = {8'(i / 3), 8'(i * 7 + 3)};
	lane_vs_link("lane 10 gamma 256");
	check("lane 10: 768 gamma writes", gam_n, 768);
	begin : l10
		reg [31:0] p, dr, p1, dr1;
		integer nmax;
		nmax = (1 << win.LANE_AW) - 2;             // BEGIN + DEPTH - 2 WORDs + END fill an empty FIFO
		f_set(0, 16'h0033, nmax);
		for (i = 0; i < nmax; i = i + 1) fw[i] = {8'(i / 3), 8'(i * 5 + 1)};
		lane_vs_link("lane 10 longest frame");
		lane_count(p, dr);
		f_set(0, 16'h0033, nmax + 1);
		for (i = 0; i <= nmax; i = i + 1) fw[i] = {8'(i / 3), 8'(i * 3)};
		trk_clear; lane_send(1);
		repeat (400) @(negedge clk_sys);
		lane_count(p1, dr1);
		check("lane 10: one word longer is dropped whole", dr1 - dr, 1);
		check("lane 10: and plays nothing", p1, p);
		check("lane 10: no gamma write from the dropped frame", gam_n, 0);
		f_set(0, 16'h001E, 8);
		for (i = 0; i < 8; i = i + 1) fw[i] = 16'hA000 + i;
		lane_vs_link("lane 10 a frame after the drop");
	end

	$display("-- LANE 11: FIO frames (sel 1) against the link's FPGA select");
	f_set(1, 16'h0055, 1);
	fw[0] = 16'h00C3;
	lane_vs_link("lane 11 FIO file index");
	check("lane 11: ioctl_index", hps_io.ioctl_index, 16'h00C3);
	f_set(1, 16'h0056, 2);
	fw[0] = 16'h4249; fw[1] = 16'h4E00;
	lane_vs_link("lane 11 FIO file info");
	check("lane 11: ioctl_file_ext", hps_io.ioctl_file_ext, 32'h42494E00);

	$display("-- LANE 12: OSD frames (sels 2, 3) against the link's OSD select");
	f_set(2, 16'h0045, 5);                         // enable with info: x, y, w, h, rot
	fw[0] = 16'h0123; fw[1] = 16'h0045; fw[2] = 16'h0010; fw[3] = 16'h0008; fw[4] = 16'h0001;
	osd_vs_link("lane 12 OSD info", 16'h0040);
	f_set(2, 16'h0023, 8);                         // a write to row 3
	for (i = 0; i < 8; i = i + 1) fw[i] = 16'h00A5 ^ (i * 16'h0011);
	osd_vs_link("lane 12 OSD row write", 16'hFFFF);
	f_set(2, 16'h0040, 0);                         // disable
	osd_vs_link("lane 12 OSD disable", 16'h0045);   // re-enable, info kept: the disable must undo it

	$display("-- LANE 13: a capture frame returns the link's reply words (0x29 against the link)");
	begin : l13
		reg [15:0] wl [0:8];
		reg [15:0] ws [0:8];
		reg [31:0] p, dr, rs;
		reg [15:0] w;
		integer m;
		st_req(st_pat(300)); settle;
		cause_drain(SYSB);                         // drain the SYS cause the request raised
		// the link: the reply to each word is io_dout once its ack is seen (the HPS reads gp_in then)
		@(negedge clk_sys); io_enable = 1; repeat (2) @(negedge clk_sys);
		for (m = 0; m < 9; m = m + 1) begin hps_word((m == 0) ? 16'h0029 : 16'h0000); ws[m] = hps_bus[15:0]; end
		@(negedge clk_sys); io_enable = 0; repeat (3) @(negedge clk_sys);
		check("lane 13 link: the head {A, flag}", {ws[0][15:4], ws[0][3:0] == hps_io.stflg}, {12'h00A, 1'b1});
		lane_count(p, dr);
		f_set(0, 16'h0029, 8);
		for (i = 0; i < 8; i = i + 1) fw[i] = 16'h0000;
		lane_send_c(1, 1'b1);
		lane_wait_n(p + 1, 1'b1);
		lane_rstat(rs);                            // at once: PLAYED is never ahead of the frame's replies
		check("lane 13: nine replies when PLAYED moves", rs, 32'd9);
		bad = 0;
		for (m = 0; m < 9; m = m + 1) begin lane_pop(wl[m]); if (wl[m] !== ws[m] || ^ws[m] === 1'bx) bad = bad + 1; end
		check("lane 13: the replies equal the link's, word for word", bad, 0);
		check("lane 13: the status words", {wl[2], wl[1]}, st_pat(300) >> 0);
		lane_rstat(rs);
		check("lane 13: the reply FIFO is empty after nine pops", rs, 32'd0);
		rd("lane 13: RHEAD reads 0 when empty", LANE + 21'h20, 32'd0);
		axi_write(LANE + 21'h0C, 32'd0, 4'hF);       // a pop on empty
		lane_rstat(rs);
		check("lane 13: a pop on empty is ignored", rs, 32'd0);
	end

	$display("-- LANE 14: a frame without the capture bit records nothing; a write frame with it, one per strobe");
	begin : l14
		reg [31:0] p, dr, rs;
		reg [15:0] w;
		integer m;
		lane_count(p, dr);
		f_set(0, 16'h001E, 8);
		for (i = 0; i < 8; i = i + 1) fw[i] = 16'hE100 + i;
		lane_send(1);
		lane_wait(p + 1);
		lane_rstat(rs);
		check("lane 14: no capture, no replies", rs, 32'd0);
		check("lane 14: the frame still played", hst[31:0], 32'hE101E100);
		for (i = 0; i < 8; i = i + 1) fw[i] = 16'hE200 + i;
		lane_send_c(1, 1'b1);
		lane_wait(p + 2);
		lane_rstat(rs);
		check("lane 14: a captured 8-word frame leaves 9 replies", rs, 32'd9);
		check("lane 14: and played", hst[31:0], 32'hE201E200);
		for (m = 0; m < 9; m = m + 1) lane_pop(w);
		lane_rstat(rs);
		check("lane 14: drained", rs, 32'd0);
	end

	$display("-- LANE 15: replies past the reply FIFO's depth are dropped and counted; the frame still plays");
	begin : l15
		reg [31:0] p, dr, rs;
		reg [15:0] w;
		integer m, nr;
		nr = 1 << win.LANE_RAW;
		lane_count(p, dr);
		f_set(0, 16'h0033, nr + 44);                // nr + 45 replies: 45 past the depth
		for (i = 0; i < nr + 44; i = i + 1) fw[i] = {8'(i / 3), 8'(i * 11)};
		trk_clear;
		lane_send_c(1, 1'b1);
		lane_wait(p + 1);
		check("lane 15: the frame played whole", gam_n, nr + 44);
		lane_rstat(rs);
		check("lane 15: the reply FIFO holds its depth", rs[15:0], nr);
		check("lane 15: 45 replies dropped", rs[31:16], 45);
		for (m = 0; m < nr; m = m + 1) lane_pop(w);
		lane_rstat(rs);
		check("lane 15: drained, the drop count kept", rs, {16'd45, 16'd0});
		f_set(0, 16'h001E, 1);
		fw[0] = 16'hF00D;
		lane_send_c(1, 1'b1);
		lane_wait(p + 2);
		lane_rstat(rs);
		check("lane 15: a capture after the overflow", rs, {16'd45, 16'd2});
		for (m = 0; m < 2; m = m + 1) lane_pop(w);
	end

	$display("-- LANE 16: two captured frames queue their replies in order");
	begin : l16
		reg [31:0] p, dr, rs;
		reg [15:0] wa [0:8];
		reg [15:0] w;
		integer m;
		lane_count(p, dr);
		f_set(0, 16'h0029, 8);
		for (i = 0; i < 8; i = i + 1) fw[i] = 16'h0000;
		lane_send_c(1, 1'b1);
		f_set(0, 16'h001E, 2);
		fw[0] = 16'h1601; fw[1] = 16'h1602;
		lane_send_c(1, 1'b1);
		lane_wait(p + 2);
		lane_rstat(rs);
		check("lane 16: 9 + 3 replies", rs[15:0], 12);
		for (m = 0; m < 9; m = m + 1) lane_pop(wa[m]);
		check("lane 16: the first frame's head is 0x29's", {wa[0][15:4], 4'd0}, 16'h00A0);
		check("lane 16: the first frame's status words come first", {wa[2], wa[1]}, st_pat(300) >> 0);
		for (m = 0; m < 3; m = m + 1) lane_pop(w);
		lane_rstat(rs);
		check("lane 16: drained", rs[15:0], 0);
	end

	lane_expiry_cases;
`endif

	$display("-- a store is in effect at its response: a read issued after it sees it, one accepted before its beat may not");
	begin : store_order
		reg [31:0] v, oc, p, dr, rs, h;
		integer k, nbad;
		st_req(st_pat(600)); settle;
		check("order: SYS cause pending", sys_irq, 1);
		store_read(SYSB, 32'h1, SYSB, v);
		check("order: a W1C store on SYS CAUSE, then CAUSE reads clear", v, 32'd0);
		store_read(SCR, 32'h0D15EA5E, SCR, v);
		check("order: a store to SCRATCH, then SCRATCH", v, dg(32'h0D15EA5E));
		axi_read(OSD + 21'h1000, 0, oc, dl, beats);
		store_read(OSD + 21'h1000, 32'h2, OSD + 21'h1000, v);
		check("order: a store to OSD CTRL, then OSD CTRL", v, 32'h2);
		axi_write(OSD + 21'h1000, oc, 4'hF);
		// the read's address accepted two cycles and one before the W1C's data beat, in its cycle and after it
		for (k = -2; k <= 1; k = k + 1) begin
			st_req(st_pat(610 + k)); settle;
			race_store_read(SYSB, 32'h1, SYSB, k, v);
			check($sformatf("order: CAUSE read accepted %0d cycles from the W1C's data beat", k), v,
			      k < 0 ? 32'h00000001 : 32'd0);
			check("order: the W1C cleared it either way", sys_irq, 0);
		end
		if (TB_SEC) begin
			cause_drain(SEC);
			sec_wr(21'h10, 32'h0); settle;
			cd_request(32'h00007100); settle;
			check("order: SECTOR cause pending", doorbell, 1);
			store_read(SEC, 32'h1, SEC, v);
			check("order: a W1C store on SECTOR CAUSE, then CAUSE reads clear", v, 32'd0);
			store_read(SEC + 21'h10, 32'h1, SEC + 21'h04, v);
			check("order: an EN store, then REQ shows the open request", v[31], 1);
			sec_wr(21'h10, 32'h0);
			@(negedge clk_sys) cd_reset = 1; repeat (4) @(negedge clk_sys); cd_reset = 0; settle;
			if (doorbell) cause_drain(SEC);
			rd("order: REQ idle again", SEC + 21'h04, 32'd0);
			sec_wr(21'h10, 32'h1); settle;
			n0 = cd_frames;
			cd_request(32'h00007200); settle;
			cause_drain(SEC);
			sec_wr(21'h0C, 32'h00007200);
			frame_data(32'h00007200, 0, TB_FRAME - 1);
			store_read(SEC + 21'h1000 + 4*(TB_FRAME - 1), pat(32'h00007200, TB_FRAME - 1), SEC + 21'h18, v);
			check("order: the last DATA store, then ACCEPT counts it", v, 32'h80000000 | TB_FRAME);
			wait_frame(n0);
			check_buf("order: that frame completes", 32'h00007200, TB_FRAME);
			cause_drain(SEC);
			sec_wr(21'h10, 32'h0); settle;
		end
`ifdef MISTER_LW_LANE
		lane_count(p, dr);
		f_set(0, 16'h0029, 8);
		for (i = 0; i < 8; i = i + 1) fw[i] = 16'h0000;
		lane_send_c(1, 1'b1);
		lane_wait(p + 1);
		repeat (slow(100)) @(negedge clk);
		lane_rstat(rs);
		check("order: nine replies queued", rs[15:0], 9);
		nbad = 0;
		for (k = 0; k < 9; k = k + 1) begin
			if (k[0]) begin
				store_read(LANE + 21'h0C, 32'd0, LANE + 21'h1C, rs);   // RPOP, then RSTAT
				if (rs[15:0] !== 8 - k) nbad = nbad + 1;
			end
			else begin
				store_read(LANE + 21'h0C, 32'd0, LANE + 21'h20, h);    // RPOP, then RHEAD
				axi_read(LANE + 21'h20, 0, v, dl, beats);
				lane_rstat(rs);
				if (h !== v || rs[15:0] !== 8 - k) nbad = nbad + 1;
			end
		end
		check("order: RPOP, then RHEAD and RSTAT show the pop", nbad, 0);
		lane_rstat(rs);
		check("order: replies drained", rs[15:0], 0);
`endif
	end

	$display("-- reads have no side effect: causes pending, a frame half stored, replies queued");
	begin : pure_reads
		reg [511:0] sw0, ss0;
		reg [255:0] sl0;
		reg [20:0]  a;
		reg  [3:0]  len;
		integer m, b, t, nbad, seed, rv;
		reg [31:0] p, dr, rs;
		reg [15:0] w;
		st_req(st_pat(500)); settle;
		check("pure reads: SYS cause pending", sys_irq, 1);
		if (TB_SEC) begin
			sec_wr(21'h10, 32'h1); settle;
			cd_request(32'h00007000); settle;
			check("pure reads: SECTOR cause pending", doorbell, 1);
			n0 = cd_frames;
			sec_wr(21'h0C, 32'h00007000);
			frame_data(32'h00007000, 0, 294);
			settle;
		end
`ifdef MISTER_LW_LANE
		lane_count(p, dr);
		f_set(0, 16'h0029, 8);
		for (i = 0; i < 8; i = i + 1) fw[i] = 16'h0000;
		lane_send_c(1, 1'b1);
		lane_wait_n(p + 1, 1'b1);
		repeat (slow(100)) @(negedge clk);
		lane_rstat(rs);
		check("pure reads: nine replies queued", rs[15:0], 9);
		f_set(0, 16'h001E, 2);
		fw[0] = 16'h0F01; fw[1] = 16'h0F02;
		lane_send_c(0, 1'b0);                      // a frame half stored: BEGIN and its words, no END
`endif
		sys_run = 0;
		repeat (50) @(negedge clk);
		sw0 = pure_win(); ss0 = pure_sec(); sl0 = pure_lane();
		pure_view(0, nbad);
		// every byte and half of each block's first 64 bytes, at both ends of the 2 MB
		for (b = 0; b < 9; b = b + 1) begin
			case (b)
				0: a = 21'h040000; 1: a = 21'h041000; 2: a = 21'h043000; 3: a = 21'h044000; 4: a = 21'h045000;
				5: a = 21'h046000; 6: a = 21'h080000; 7: a = 21'h1FFFC0; default: a = 21'h000000;
			endcase
			for (m = 0; m < 64; m = m + 1) begin
				rsize = 0; axi_read(a + m[20:0], 0, d, dl, beats);
				if (!m[0]) begin rsize = 1; axi_read(a + m[20:0], 0, d, dl, beats); end
			end
			rsize = 2;
			// bursts of 2-16 beats of every type, words and bytes
			for (m = 1; m < 16; m = m + 1) begin
				for (t = 0; t < 4; t = t + 1) begin arburst = t[1:0]; axi_read(a, m[3:0], d, dl, beats); end
				rsize = 0; arburst = 2'b01; axi_read(a, m[3:0], d, dl, beats); rsize = 2;
			end
		end
		// 64 random addresses over the 2 MB: a word, a half, a byte and a burst of a random length and type
		seed = 7;
		for (m = 0; m < 64; m = m + 1) begin
			rv = $random(seed);
			a = {rv[20:2], 2'b00}; len = rv[24:21] | 4'd1; arburst = rv[26:25];
			rsize = 2; axi_read(a, 0, d, dl, beats);
			rsize = 1; axi_read(a + 21'd2, 0, d, dl, beats);
			rsize = 0; axi_read(a + 21'd3, 0, d, dl, beats); rsize = 2;
			axi_read(a, len, d, dl, beats);
		end
		arburst = 2'b01;
		pure_view(1, nbad);
		check("pure reads: the readable view is unchanged", nbad, 0);
		check("pure reads: the window's and SYS's state is unchanged", pure_win() === sw0, 1);
		check("pure reads: SECTOR's state is unchanged", pure_sec() === ss0, 1);
		check("pure reads: the lane's state is unchanged", pure_lane() === sl0, 1);
		check("pure reads: both lines still high", {30'd0, TB_SEC ? doorbell : 1'b1, sys_irq}, 32'd3);
		sys_run = 1;
		if (TB_SEC) begin
			frame_data(32'h00007000, 294, 588);
			wait_frame(n0);
			check_buf("pure reads: the half-stored frame completes", 32'h00007000, 588);
			cause_drain(SEC);
			sec_wr(21'h10, 32'h0); settle;
		end
		cause_drain(SYSB);
`ifdef MISTER_LW_LANE
		axi_write(LANE + 21'h8, 32'd0, 4'hF);
		lane_wait(p + 2);
		for (m = 0; m < 9; m = m + 1) lane_pop(w);
		lane_rstat(rs);
		check("pure reads: replies drained", rs[15:0], 0);
`endif
	end

	dir_causes;
	fw_reset_cases;
	resets_cases;
	lease_up;

	if (!DIAG) begin
		$display("-- LW_DIAG=0: the removed registers read 0");
		rd("lean SCRATCH", SCR, 32'd0);
		rd("lean analog drops", pad_addr(8'h1A, 0), 32'd0);
		if (TB_SEC) begin
			axi_read(SEC + 21'h14, 0, d, dl, beats);
			check("lean STATUS[31:4]", d[31:4], 0);
			for (i = 7; i < 14; i = i + 1) rd("lean SECTOR reg", SEC + 4*i, 32'd0);
			rd("lean ID kept", SEC + 21'h38, 32'h32434553);
		end
	end

`ifdef LW_TB_CDC
	// the models moved edges in this run: synchroniser bits, a Gray bus, and no bundle capture took a bit in flight
	if (!$value$plusargs("lw_seed=%d", i)) i = 1;
	$display("-- CDC models (seed %0d): edges moved: PAD handshake %0d, status flag %0d, the cell bench's Gray counts %0d",
	         i, win.pad_x.sync_r.m_n + win.pad_x.sync_a.m_n, win.sys_status.flg_sync.m_n, cg_xa.m_n + cg_xb.m_n);
	check("models: the PAD handshake's toggles took extra cycles", win.pad_x.sync_r.m_n + win.pad_x.sync_a.m_n > 0, 1);
	check("models: a Gray bus took an extra cycle", win.sys_status.flg_sync.m_n + cg_xa.m_n + cg_xb.m_n > 0, 1);
	check("models: captures that took a bit still in flight (PAD, the cell bench)",
	      win.pad_x.cap.m_n + cx_a.cap.m_n + cx_b.cap.m_n, 0);
`ifdef MISTER_LW_SECTOR_SLOT
	check("models: SECTOR's crossings took extra cycles",
	      win.sec.sector.sys_sync.m_n + win.sec.sector.ctl_sync.m_n > 0, 1);
`endif
`ifdef MISTER_LW_LANE
	check("models: the lane's pointers took extra cycles", win.lane.lane.wg_sync.m_n + win.lane.lane.rg_sync.m_n > 0, 1);
`endif
`endif

	lease_off;
	lease_cases;
	line_cases;
	look_cases;
	ls10;
	if (TB_LINKED != 0) begin
		ls6;
		ls11;
		ls12;
		ls15;
		ls17;
		ls8;
		if (TB_SEC) sec7_sweep;
	end
	lease_summary(1);

	$display("-- firewall: idle is a register, 1 exactly while both state machines are idle (the model, every cycle)");
	$display("     %0d cycles compared with the model of the marks, the machines freed and idle", fm_cycles);
	check("firewall model: cycles that differ", fm_bad, 0);
	check("idle changed off a clk_100m edge", fw_glitch, 0);
	check("cycles in which present (GPI bit 25) is not 1", fw_absent, 0);

	$display("fixed latency: BVALID %0d cycles after AWVALID, RVALID %0d cycles after ARVALID (the firewall's RD_LAT)",
	         wlat_ref, rlat_ref);
	$display("%s: %0d checks (%0d functional + %0d fixed-latency, one per transaction), %0d failures",
	         fails ? "FAILED" : "PASSED", checks, checks - lchecks, lchecks, fails);
	$finish;
end
`endif

initial begin #200000000 $display("FAILED: timeout"); $finish; end

endmodule
