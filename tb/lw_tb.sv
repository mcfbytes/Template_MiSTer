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
always #5 clk = ~clk;

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

wire [20:0] lw_raddr, lw_waddr;
wire        lw_ren, lw_rd_ack, lw_rd_word, lw_write;
wire [31:0] lw_rdata, lw_wdata;
wire  [3:0] lw_be;
wire [48:0] lw_pad;
wire        doorbell;
wire        sys_irq;
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

hps_lw_bridge bridge
(
	.clk(clk),
	.lw_raddr(lw_raddr), .lw_ren(lw_ren), .lw_rd_ack(lw_rd_ack), .lw_rd_word(lw_rd_word), .lw_rdata(lw_rdata),
	.lw_waddr(lw_waddr), .lw_write(lw_write), .lw_wdata(lw_wdata), .lw_be(lw_be)
);

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

lw_sys_window #(
	.SEC_EN(TB_SEC), .SEC_SLOT(8'd1), .SEC_FRAME(TB_FRAME), .BLKSZ(3'd3),
	.MC_SLOT0(4'd2), .MC_WSRC0(32'h30100000), .MC_EXT0(32'h00020000),
	.MC_SLOT1(4'd3), .MC_WSRC1(32'h30200000), .MC_EXT1(32'h00020000), .MC_FRAME(10'd256),
	.LANE_EN(TB_LANE)
) win
(
	.clk(clk),
	.lw_raddr(lw_raddr), .lw_ren(lw_ren), .lw_rd_ack(lw_rd_ack), .lw_rd_word(lw_rd_word), .lw_rdata(lw_rdata),
	.lw_waddr(lw_waddr), .lw_write(lw_write), .lw_wdata(lw_wdata), .lw_be(lw_be),
	.doorbell(doorbell), .sys_irq(sys_irq),
	.clk_sys(clk_sys), .lw_pad(lw_pad), .blk_up(blk_up), .blk_dn(blk_dn), .card_up(card_up), .card_dn(card_dn),
	.st_up(st_up),
	.osd_we(osd_we), .osd_addr(osd_addr), .osd_data(osd_data), .osd_be(osd_be), .osd_ctrl(osd_ctrl),
`ifdef MISTER_LW_LANE
	.lane_busy(io_enable | io_fpga | io_osd), .lane_wait(hps_bus[37] | vs_wait), .lane_dout(hps_bus[15:0]),
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

integer fails = 0, checks = 0, lchecks = 0;
integer wlat_ref = -1, rlat_ref = -1;

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
			repeat (slow(10)) @(negedge clk);
			if (cx_ra > 3 || cx_rb > 3 || cx_na != cx_ka || cx_nb != cx_kb) spur = spur + 1;
			r0 = cx_ra + cx_rb;
			cx_send(3);
			if (cx_na != cx_ka || cx_nb != cx_kb || cx_ea + cx_eb != 0 || cx_ra + cx_rb != r0) bad = bad + 1;
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

// Drive at negedge, sample at negedge: cycles from VALID asserted to BVALID/RVALID seen.
task axi_write(input [20:0] a, input [31:0] d, input [3:0] s);
	integer n;
	reg aw_hs, w_hs;
	begin
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
	end
endtask

// Every beat of the last read: its data and its ID (ARBURST is the arburst register's).
reg [31:0] rb_d  [0:15];
reg [11:0] rb_id [0:15];

task axi_read(input [20:0] a, input [3:0] len, output [31:0] d0, output [31:0] dl, output integer beats);
	integer n;
	reg ar_hs, done;
	begin
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
	end
endtask

// A write burst of len + 1 beats and type bt (data a pattern, all strobes): AW and the first W together, one
// beat per W handshake, WLAST on the last. nb counts the write responses until 16 cycles after the first.
task axi_wburst(input [20:0] a, input [3:0] len, input [1:0] bt, output integer nb);
	integer n, q;
	reg [3:0] k;
	reg aw_hs, w_hs;
	begin
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
			if (rb_d[j] !== wref[wi]) begin
				if (nbad == 0) $display("     %s: beat %0d at %06h got %08h exp %08h", what, j, ba, rb_d[j], wref[wi]);
				nbad = nbad + 1;
			end
			if (rb_id[j] !== {8'h5A, len}) nbad_id = nbad_id + 1;
		end
		check({what, ": stepped words"}, nbad, 0);
		check({what, ": ID echoed on every beat"}, nbad_id, 0);
	end
endtask


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
localparam [20:0] SYSB = 21'h43000;
reg [31:0] stf0, stf1;
reg [127:0] stw;

function [127:0] st_pat(input integer k);
	st_pat = {32'hA5000000 ^ k, 32'h5A000000 + k, 32'h00C30000 | k, {k[23:0], 8'h3C}};
endfunction

// One core status request: st_in, then a rising edge of status_set (held one clk_sys, then dropped).
// The last capture of the snapshot and the last CAUSE fetch, for the race sweep's level property.
realtime t_cap = 0, t_fetch = 0;
always @(posedge clk) begin
	if (win.sys_status.capture) t_cap = $realtime;
	if (win.sys_status.rd_cause && lw_ren) t_fetch = $realtime;
end

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

`ifdef MENU_CORE
integer f;
initial begin
	repeat (4) @(negedge clk);
	sys_rate;
	cdc_cells;
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
	$display("-- default geometry: a %0d-word primary, 256-word save slots", TB_FRAME);
	rd("e1 slot/frame 128", 21'h40060, 32'h01000080);
	rd("e1 w7 card 1 slot/flags/frame 256", 21'h4006C, 32'h02030100);
	sec_wr(21'h10, 32'hC1); settle;

	$display("-- a 128-word primary frame");
	n0 = cd_frames; s0 = cd_strobes;
	cd_request(32'h00000100); settle;
	axi_read(SEC + 21'h00, 0, d, dl, beats);
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
	axi_read(SEC + 21'h00, 0, d, dl, beats);
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
	axi_read(SEC + 21'h00, 0, d, dl, beats);
	sec_wr(21'h0C, 32'h00000101);
	frame_data(32'h00000101, 0, 128);
	rd("ACCEPT ok, 128 after the card", SEC + 21'h18, 32'h80000080);
	wait_frame(n0);
	check_buf("primary frame 0x101 after the card", 32'h00000101, 128);
	rd("REQ idle", SEC + 21'h04, 32'd0);
	check("no ring left", doorbell, 0);

	$display("%s: %0d checks (%0d functional + %0d fixed-latency, one per transaction), %0d failures",
	         fails ? "FAILED" : "PASSED", checks, checks - lchecks, lchecks, fails);
	$finish;
end
`else
initial begin
	repeat (4) @(negedge clk);
	sys_rate;
	cdc_cells;

	// `SYS_LW_CAPS carries its own concatenation comma, so it is read inside {} with a closing ";"
	checks = checks + 1;
	caps = {`SYS_LW_CAPS ";"};
	if (TB_SEC ? (caps != $sformatf("AXI:%0h:%0h:LW,IRQ:BLK=1:%0h,;", win.WIN_OFF, win.WIN_SIZE, win.WIN_OFF + win.SEC_OFF))
	           : (caps != $sformatf("AXI:%0h:%0h:LW,;", win.WIN_OFF, win.WIN_SIZE))) begin
		fails = fails + 1; $display("FAIL SYS_LW_CAPS %s does not match the decoded window", caps);
	end
	else $display("pass SYS_LW_CAPS = %s", caps);

	$display("-- directory");
	rd("magic",        21'h40000, 32'h31535953);
	rd("dir version",  21'h40004, 32'h00010000);
	rd("count",        21'h40008, TB_LANE ? 32'd5 : 32'd4);
	rd("scratch reset",21'h4000C, 32'd0);
	rd("e0 kind",      21'h40010, 32'd1);
	rd("e0 version",   21'h40014, 32'h00010000);
	rd("e0 offset",    21'h40018, 32'h00040000);
	rd("e0 size",      21'h4001C, 32'h00040000);
	rd("e0 bitmap w0", 21'h40020, 32'h040F000C);
	rd("e0 bitmap w1", 21'h40024, 32'h20000000);
	for (i = 2; i < 8; i = i + 1) rd("e0 bitmap wN", 21'h40020 + 4*i, 32'd0);
	for (i = 0; i < 4; i = i + 1) rd("e0 reserved", 21'h40040 + 4*i, 32'd0);
`ifndef MISTER_LW_SECTOR_SLOT
	for (i = 0; i < 16; i = i + 1) rd("e1 empty (no SECTOR)", 21'h40050 + 4*i, 32'd0);
`else
	rd("e1 kind SECTOR", 21'h40050, 32'd2);
	rd("e1 version 1.1", 21'h40054, 32'h00010001);
	rd("e1 offset",      21'h40058, 32'h00001000);
	rd("e1 size",        21'h4005C, 32'h00002000);
	rd("e1 slot/frame",  21'h40060, 32'h0100024C);
	rd("e1 cause (abs)", 21'h40064, 32'h00041000);
	rd("e1 depth/gap",   21'h40068, 32'h02000002);
	rd("e1 w7 card 1 slot/flags/frame", 21'h4006C, 32'h02030100);
	rd("e1 w8 card 1 wsrc",             21'h40070, 32'h30100000);
	rd("e1 w9 card 2 slot/flags/frame", 21'h40074, 32'h03030100);
	rd("e1 w10 card 2 wsrc",            21'h40078, 32'h30200000);
	rd("e1 w11 card 1 extent",          21'h4007C, 32'h00020000);
	rd("e1 w12 card 2 extent",          21'h40080, 32'h00020000);
	for (i = 13; i < 16; i = i + 1) rd("e1 reserved", 21'h40050 + 4*i, 32'd0);
`endif
	rd("e2 kind OSDBUF", 21'h40090, 32'd3);
	rd("e2 version",     21'h40094, 32'h00010000);
	rd("e2 offset",      21'h40098, 32'h00004000);
	rd("e2 size",        21'h4009C, 32'h00002000);
	rd("e2 geometry",    21'h400A0, 32'h00800100);
	for (i = 5; i < 16; i = i + 1) rd("e2 reserved", 21'h40090 + 4*i, 32'd0);
	rd("e3 kind SYS",    21'h400D0, 32'd4);
	rd("e3 version",     21'h400D4, 32'h00010000);
	rd("e3 offset",      21'h400D8, 32'h00003000);
	rd("e3 size",        21'h400DC, 32'h00001000);
	rd("e3 line ordinal 2 (f2h_irq[9])", 21'h400E0, 32'd2);
	rd("e3 cause (abs)", 21'h400E4, 32'h00043000);
	rd("e3 class tag SYS", 21'h400E8, 32'h00100000);
	rd("e3 causes: STATUS", 21'h400EC, 32'h00000001);
	for (i = 8; i < 16; i = i + 1) rd("e3 reserved", 21'h400D0 + 4*i, 32'd0);
`ifdef MISTER_LW_LANE
	rd("e4 kind LANE",   21'h40110, 32'd5);
	rd("e4 version 1.1", 21'h40114, 32'h00010001);
	rd("e4 offset",      21'h40118, 32'h00006000);
	rd("e4 size",        21'h4011C, 32'h00001000);
	rd("e4 UIO bitmap w0: 01 1C 1D 1E", 21'h40120, 32'h70000002);
	rd("e4 UIO bitmap w1: 22 24 31 32 33 3B", 21'h40124, 32'h080E0014);
	for (i = 6; i < 12; i = i + 1) rd("e4 bitmap wN", 21'h40110 + 4*i, 32'd0);
	rd("e4 w12 0x20 reply 'b11", 21'h40140, 32'h00000003);
	rd("e4 w13 depth/gap", 21'h40144, 32'h04000004);
	rd("e4 w14 reply depth", 21'h40148, 32'h01000000);
	rd("e4 w15 no line", 21'h4014C, 32'd0);
`endif

	$display("-- scratch");
	axi_write(21'h4000C, 32'hA5A55A5A, 4'hF);
	rd("scratch", 21'h4000C, dg(32'hA5A55A5A));
	axi_write(21'h4000C, 32'h00C30000, 4'h4);
	rd("scratch byte lane", 21'h4000C, dg(32'hA5C35A5A));

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
	rd("directory hole", TB_LANE ? 21'h040150 : 21'h040110, 32'hDEADBEEF);
	rd("below SECTOR", 21'h040FFC, 32'hDEADBEEF);
	rd("above SECTOR: SYS CAUSE idle", 21'h043000, 32'd0);
	rd("SYS block end", 21'h043FFC, 32'd0);
	if (TB_LANE) rd("above LANE", 21'h047000, 32'hDEADBEEF);
	else rd("above OSDBUF", 21'h046000, 32'hDEADBEEF);
	rd("above window", 21'h0C0000, 32'hDEADBEEF);
	rd("LW 0x1FFFFC", 21'h1FFFFC, 32'hDEADBEEF);
	axi_write(21'h000100, 32'h0, 4'hF);
	axi_read(21'h40000, 4'd3, d, dl, beats);
	check("burst beats", beats, 4);
	check("burst last (scratch)", dl, dg(32'hA5C35A5A));

	$display("-- firewall: read bursts of every type step their words; a burst across 4 KB is answered");
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
	arburst = 2'b10; axi_read(21'h40004, 4'd2, d, dl, beats); arburst = 2'b01;
	check("WRAP of an illegal length (3 beats) answered", beats, 3);
	axi_read(21'h40FF8, 4'd3, d, dl, beats);
	check("INCR across a 4 KB boundary inside the window answered", beats, 4);
	axi_read(21'h3FFF8, 4'd15, d, dl, beats);
	check("INCR 16 across the window's first 4 KB boundary answered", beats, 16);
	check("... its first beat is the default responder's", rb_d[0], 32'hDEADBEEF);
	axi_read(21'h1FFFF8, 4'd3, d, dl, beats);
	check("INCR across the top of the 2 MB answered", beats, 4);
	arburst = 2'b00; axi_read(21'h1FFFFC, 4'd15, d, dl, beats); arburst = 2'b01;
	check("FIXED 16 at the last word answered", beats, 16);
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
		if (d !== wref[i]) bad = bad + 1;
	end
	check("the directory reads the same after every burst", bad, 0);

	$display("-- clk_sys stopped (core held in reset)");
	sys_run = 0;
	rd("magic, clk_sys stopped", 21'h40000, 32'h31535953);
	axi_write(21'h4000C, 32'h0BADF00D, 4'hF);
	rd("scratch, clk_sys stopped", 21'h4000C, dg(32'h0BADF00D));
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
	$display("-- no SECTOR block: its addresses read 0, stores are ignored, no doorbell");
	rd("SECTOR CAUSE reads 0", 21'h041000, 32'd0);
	rd("SECTOR ID reads 0", 21'h041038, 32'd0);
	axi_write(21'h041010, 32'h1, 4'hF);
	axi_write(21'h04100C, 32'h0, 4'hF);
	rd("SECTOR CTRL still 0", 21'h041010, 32'd0);
	rd("SECTOR DATA reads 0", 21'h042000, 32'd0);
	rd("SECTOR end reads 0", 21'h042FFC, 32'd0);
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
	rsize = 0; rd("CAUSE byte read: data", SEC + 21'h00, 32'h000180C5); rsize = 2;
	check("byte read does not clear", doorbell, 1);
	rd("CAUSE word read: BLK tag + 0x80C5", SEC + 21'h00, 32'h000180C5);
	check("word read clears the line", doorbell, 0);
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
	rd("CAUSE pending, clk_sys stopped", SEC + 21'h00, 32'h000180C5);
	check("CAUSE read clears with clk_sys stopped", doorbell, 0);
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
	axi_read(SEC + 21'h00, 0, d, dl, beats);
	cd_request(32'h00002000); settle;
	check("SPI slot 1 offered", sd_rd[1], 1);
	check("doorbell still rings", doorbell, 1);
	spi_frame(32'h00002000);
	wait_frame(n0);
	check_buf("frame 0x2000 over SPI through the mux", 32'h00002000, 588);

	$display("-- memory cards: EN=0 leaves both cards on hps_io");
	axi_read(SEC + 21'h00, 0, d, dl, beats);           // clears the CD's pending cause
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
	rd("CAUSE names card 1 read", SEC + 21'h00, 32'h000180C9);
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
	rd("CAUSE names card 2 write", SEC + 21'h00, 32'h000180CE);
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
	rd("CAUSE: the CD first", SEC + 21'h00, 32'h000180C5);
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
	axi_read(SEC + 21'h00, 0, d, dl, beats);
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
	axi_read(SEC + 21'h00, 0, d, dl, beats);              // the HPS drains the cause
	rd("REQ card 1 read", SEC + 21'h04, 32'h800080C9);
	rd("LBA_S[2] 0x30", SEC + 21'h68, 32'h00000030);       // the HPS fills LBA 0x30 ...
	@(negedge clk_sys) cd_reset = 1;
	repeat (4) @(negedge clk_sys);
	cd_reset = 0;
	mc1.go(0, 7'h00); settle;                              // ... the load restarts at LBA 0
	axi_read(SEC + 21'h00, 0, d, dl, beats);              // the restart's ring, drained early
	sec_wr(21'h48, 32'h00000030); settle;                  // the stale answer
	axi_read(SEC + 21'h14, 0, d, dl, beats);
	check("STATUS rejecting after the stale BEGIN_S", d[3:1], 3'b100);
	check("the reject re-rings the doorbell", doorbell, 1);
	rd("REQ shows card 1 again", SEC + 21'h04, 32'h800080C9);
	rd("LBA_S[2] 0", SEC + 21'h68, 32'h00000000);
	axi_read(SEC + 21'h00, 0, d, dl, beats);
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
	axi_read(SEC + 21'h00, 0, d, dl, beats);
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
	axi_read(SEC + 21'h00, 0, d, dl, beats);
	rd("LBA_S[2] 0x40", SEC + 21'h68, 32'h00000040);
	@(negedge clk_sys) mc1.lba = 7'h41;                     // rd never fell
	settle;
	rd("LBA_S[2] still the latched 0x40", SEC + 21'h68, 32'h00000040);
	sec_wr(21'h48, 32'h00000040); settle;                   // answers the LBA it was shown
	check("the moved LBA's answer re-rings", doorbell, 1);
	rd("REQ shows card 1 again", SEC + 21'h04, 32'h800080C9);
	rd("LBA_S[2] re-latched 0x41", SEC + 21'h68, 32'h00000041);
	axi_read(SEC + 21'h00, 0, d, dl, beats);
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
	axi_read(SEC + 21'h00, 0, d, dl, beats);
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
	axi_read(SEC + 21'h00, 0, d, dl, beats);
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
	axi_read(SEC + 21'h00, 0, d, dl, beats);
	sys_run = 0;                                            // both stale answers queue back to back
	sec_wr(21'h48, 32'h00000025);
	sec_wr(21'h4C, 32'h00000035);
	sys_run = 1; settle;
	rd("REQ shows card 1 re-opened", SEC + 21'h04, 32'h800080C9);
	axi_read(SEC + 21'h00, 0, d, dl, beats);
	sec_wr(21'h48, 32'h00000024);
	frame_data(32'h00000024, 0, 256);
	repeat (1500) @(negedge clk_sys);
	check("card 1 frame at 0x24", mc1.done - r, 1);
	rd("REQ shows card 2 re-opened", SEC + 21'h04, 32'h800080CD);
	axi_read(SEC + 21'h00, 0, d, dl, beats);
	sec_wr(21'h4C, 32'h00000034);
	frame_data(32'h00000034, 0, 256);
	repeat (1500) @(negedge clk_sys);
	check("card 2 frame at 0x34", mc2.done - s0, 1);
	rd("REQ idle after both re-opened", SEC + 21'h04, 32'd0);

	$display("-- card EN off: back to hps_io, no doorbell");
	sec_wr(21'h10, 32'h1); settle;
	axi_read(SEC + 21'h00, 0, d, dl, beats);
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
	check("SYS byte read returns the cause", d, 32'h00100001);
	check("SYS byte read does not clear", sys_irq, 1);
	rd("SYS CAUSE: tag SYS (bit 20), payload STATUS", SYSB, 32'h00100001);
	check("SYS word read clears the line", sys_irq, 0);
	rd("SYS CAUSE idle after the read", SYSB, 32'd0);
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
	rd("SYS CAUSE", SYSB, 32'h00100001);
	check("cleared by one word read", sys_irq, 0);
	st_read(stf0, stf1, stw);
	check("STFLG flag 5 (the core's)", {stf0[7:4], stf0[3:0]}, {4'hA, hps_io.stflg});
	check("STFLG stable", stf1, stf0);
	check("snapshot is the last request", stw == st_pat(5), 1);

	$display("-- SYS: a request racing the CAUSE read is never lost (offset sweep)");
	begin : race
		integer k, lost, kept;
		lost = 0; kept = 0;
		for (k = 0; k < 64; k = k + 1) begin
			fork
				begin #(k * 2); axi_read(SYSB, 0, d, dl, beats); end
				st_req(st_pat(100 + k));
			join
			settle;
			// a capture the read did not return is still pending: the line stays (or comes back) up
			if (t_cap >= t_fetch) begin kept = kept + 1; if (!sys_irq) lost = lost + 1; end
			// The HPS's drain: the snapshot after the CAUSE read; a capture it missed keeps the line up
			st_read(stf0, stf1, stw);
			if (!sys_irq && (stf0[3:0] !== hps_io.stflg || stw !== st_pat(100 + k))) lost = lost + 1;
			if (sys_irq) axi_read(SYSB, 0, d, dl, beats);
		end
		check("no request lost across 64 offsets", lost, 0);
		check("some captures landed after the fetch", kept > 0, 1);
		check("line idle after the sweep", sys_irq, 0);
	end

	$display("-- SYS: a capture mid-read changes STFLG, so the seqlock retries");
	axi_read(SYSB + 21'h04, 0, stf0, dl, beats);
	st_req(st_pat(200));
	settle;
	axi_read(SYSB + 21'h04, 0, stf1, dl, beats);
	check("STFLG moved across the capture", stf0 != stf1, 1);
	rd("SYS CAUSE", SYSB, 32'h00100001);

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
		axi_read(SYSB, 0, d, dl, beats);           // drain the SYS cause the request raised
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
`endif

	if (!DIAG) begin
		$display("-- LW_DIAG=0: the removed registers read 0");
		rd("lean SCRATCH", 21'h4000C, 32'd0);
		rd("lean analog drops", pad_addr(8'h1A, 0), 32'd0);
		axi_read(SEC + 21'h14, 0, d, dl, beats);
		check("lean STATUS[31:4]", d[31:4], 0);
		for (i = 7; i < 14; i = i + 1) rd("lean SECTOR reg", SEC + 4*i, 32'd0);
		if (TB_SEC) rd("lean ID kept", SEC + 21'h38, 32'h32434553);
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

	$display("fixed latency: BVALID %0d cycles after AWVALID, RVALID %0d cycles after ARVALID", wlat_ref, rlat_ref);
	$display("%s: %0d checks (%0d functional + %0d fixed-latency, one per transaction), %0d failures",
	         fails ? "FAILED" : "PASSED", checks, checks - lchecks, lchecks, fails);
	$finish;
end
`endif

initial begin #200000000 $display("FAILED: timeout"); $finish; end

endmodule
