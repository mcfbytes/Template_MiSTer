//============================================================================
//  sys_stubs.sv -- part of the LW bridge framework (docs/LW_BRIDGE.md)
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

// Stand-ins for the modules arm E of tb/run.sh cannot compile when it elaborates sys_top.v (tb/top_tb.sv):
// empty modules with the ports the design connects. Each says why it is here. Outputs are constants, except
// the clocks, the HPS reset, GPO and the LW master, which come from top_tb. They prove nothing about the
// modules they replace: arm E checks the wiring between the modules it does compile.

`timescale 1ns/1ps
`default_nettype none
// verilator lint_off UNUSEDSIGNAL
// verilator lint_off UNUSEDPARAM

//////////////////////  Vendor atoms: no simulation model in this bench  //////////////////////

module cyclonev_hps_interface_mpu_general_purpose(input wire [31:0] gp_in, output wire [31:0] gp_out);
assign gp_out = top_tb.gp_out;
endmodule

module cyclonev_hps_interface_interrupts(input wire [63:0] irq);
endmodule

module cyclonev_hps_interface_clocks_resets
(
	input  wire f2h_cold_rst_req_n, input wire f2h_warm_rst_req_n, input wire f2h_pending_rst_ack,
	input  wire f2h_dbg_rst_req_n,
	output wire h2f_rst_n, output wire h2f_user0_clk
);
assign h2f_rst_n     = top_tb.h2f_rst_n;
assign h2f_user0_clk = top_tb.clk_100m;
endmodule

module cyclonev_hps_interface_hps2fpga_light_weight
(
	input  wire        clk,
	output wire [11:0] awid, output wire [20:0] awaddr, output wire [3:0] awlen, output wire [2:0] awsize,
	output wire  [1:0] awburst, output wire [1:0] awlock, output wire [3:0] awcache, output wire [2:0] awprot,
	output wire        awvalid, input wire awready,
	output wire [11:0] wid, output wire [31:0] wdata, output wire [3:0] wstrb, output wire wlast,
	output wire        wvalid, input wire wready,
	input  wire [11:0] bid, input wire [1:0] bresp, input wire bvalid, output wire bready,
	output wire [11:0] arid, output wire [20:0] araddr, output wire [3:0] arlen, output wire [2:0] arsize,
	output wire  [1:0] arburst, output wire [1:0] arlock, output wire [3:0] arcache, output wire [2:0] arprot,
	output wire        arvalid, input wire arready,
	input  wire [11:0] rid, input wire [31:0] rdata, input wire [1:0] rresp, input wire rlast,
	input  wire        rvalid, output wire rready
);
assign {awid, awaddr, awlen, awsize, awburst, awvalid} = {12'd0, top_tb.awaddr, 4'd0, 3'd2, 2'b01, top_tb.awvalid};
assign {wid, wdata, wstrb, wlast, wvalid} = {12'd0, top_tb.wdata, 4'hF, 1'b1, top_tb.wvalid};
assign {arid, araddr, arlen, arsize, arburst, arvalid} = {12'd0, top_tb.araddr, 4'd0, 3'd2, 2'b01, top_tb.arvalid};
assign {awlock, awcache, awprot, arlock, arcache, arprot} = 0;
assign bready = 1'b1;
assign rready = top_tb.rready;
endmodule

module cyclonev_hps_interface_peripheral_uart
(
	input  wire ri, input wire dsr, input wire dcd, input wire cts, input wire rxd,
	output wire dtr, output wire rts, output wire txd
);
assign {dtr, rts, txd} = 3'b001;
endmodule

module cyclonev_hps_interface_peripheral_spi_master
(
	input  wire rxd, input wire ss_in_n,
	output wire sclk_out, output wire txd, output wire ss_0_n
);
assign {sclk_out, txd, ss_0_n} = 3'b001;
endmodule

module cyclonev_hps_interface_peripheral_i2c(input wire scl, input wire sda, output wire out_clk, output wire out_data);
assign {out_clk, out_data} = 2'b00;
endmodule

module cyclonev_hps_interface_dbg_apb(input wire DBG_APB_DISABLE, input wire P_CLK_EN);
endmodule

module cyclonev_hps_interface_tpiu_trace(input wire traceclk_ctl);
endmodule

module cyclonev_hps_interface_boot_from_fpga
(
	input wire boot_from_fpga_ready, input wire boot_from_fpga_on_failure, input wire bsel_en, input wire csel_en,
	input wire [1:0] csel, input wire [2:0] bsel
);
endmodule

// The fpga2hps master: the design drives the inputs; every output is 0 here (arm E only wires the top level).
module cyclonev_hps_interface_fpga2hps
(
	input  wire        clk,
	input  wire  [7:0] awid,
	input  wire [31:0] awaddr,
	input  wire  [3:0] awlen,
	input  wire  [2:0] awsize,
	input  wire  [1:0] awburst,
	input  wire  [1:0] awlock,
	input  wire  [3:0] awcache,
	input  wire  [2:0] awprot,
	input  wire  [4:0] awuser,
	input  wire        awvalid,
	output wire        awready,
	input  wire  [7:0] wid,
	input  wire [127:0] wdata,
	input  wire [15:0] wstrb,
	input  wire        wlast,
	input  wire        wvalid,
	output wire        wready,
	output wire  [7:0] bid,
	output wire  [1:0] bresp,
	output wire        bvalid,
	input  wire        bready,
	input  wire  [7:0] arid,
	input  wire [31:0] araddr,
	input  wire  [3:0] arlen,
	input  wire  [2:0] arsize,
	input  wire  [1:0] arburst,
	input  wire  [1:0] arlock,
	input  wire  [3:0] arcache,
	input  wire  [2:0] arprot,
	input  wire  [4:0] aruser,
	input  wire        arvalid,
	output wire        arready,
	output wire  [7:0] rid,
	output wire [127:0] rdata,
	output wire  [1:0] rresp,
	output wire        rlast,
	output wire        rvalid,
	input  wire        rready,
	input  wire  [1:0] port_size_config
);
assign {awready, wready, bid, bresp, bvalid, arready, rid, rdata, rresp, rlast, rvalid} = 0;
endmodule

module cyclonev_hps_interface_hps2fpga(input wire [1:0] port_size_config);
endmodule

module cyclonev_hps_interface_fpga2sdram
(
	input  wire [15:0] cfg_rfifo_cport_map, input wire [15:0] cfg_wfifo_cport_map,
	input  wire [17:0] cfg_cport_rfifo_map, input wire [17:0] cfg_cport_wfifo_map,
	input  wire [11:0] cfg_cport_type, input wire [11:0] cfg_port_width, input wire [5:0] cfg_axi_mm_select,
	input  wire        cmd_port_clk_0, input wire cmd_port_clk_1, input wire cmd_port_clk_2,
	input  wire        cmd_valid_0, input wire cmd_valid_1, input wire cmd_valid_2,
	input  wire [59:0] cmd_data_0, input wire [59:0] cmd_data_1, input wire [59:0] cmd_data_2,
	output wire        cmd_ready_0, output wire cmd_ready_1, output wire cmd_ready_2,
	input  wire        rd_clk_0, input wire rd_clk_1, input wire rd_clk_2, input wire rd_clk_3,
	input  wire        rd_ready_0, input wire rd_ready_1, input wire rd_ready_2, input wire rd_ready_3,
	output wire        rd_valid_1, output wire rd_valid_2, output wire rd_valid_3,
	output wire [63:0] rd_data_0, output wire [63:0] rd_data_1, output wire [63:0] rd_data_2, output wire [63:0] rd_data_3,
	input  wire        wr_clk_0, input wire wr_clk_1, input wire wr_clk_2, input wire wr_clk_3,
	input  wire [89:0] wr_data_0, input wire [89:0] wr_data_1, input wire [89:0] wr_data_2, input wire [89:0] wr_data_3,
	input  wire        wrack_ready_0, input wire wrack_ready_1, input wire wrack_ready_2
);
assign {cmd_ready_0, cmd_ready_1, cmd_ready_2, rd_valid_1, rd_valid_2, rd_valid_3} = 0;
assign {rd_data_0, rd_data_1, rd_data_2, rd_data_3} = 0;
endmodule

module cyclonev_clkselect(input wire [1:0] clkselect, input wire [3:0] inclk, output wire outclk);
assign outclk = inclk[clkselect];
endmodule

module altddio_out
#(
	parameter extend_oe_disable = "OFF", parameter intended_device_family = "Cyclone V",
	parameter invert_output = "OFF", parameter lpm_hint = "UNUSED", parameter lpm_type = "altddio_out",
	parameter oe_reg = "UNREGISTERED", parameter power_up_high = "OFF", parameter width = 1
)
(
	input  wire [width-1:0] datain_h, input wire [width-1:0] datain_l, input wire outclock,
	input  wire aclr, input wire aset, input wire oe, input wire outclocken, input wire sclr, input wire sset,
	output wire [width-1:0] dataout
);
assign dataout = 0;
endmodule

// lfsr.v chains lcells into a ring with no register: in a zero-delay simulator the ring never settles.
module lcell(input wire in, output wire out);
assign out = 1'b0;
endmodule

//////////////////////  PLLs: generated around altera_pll, which has no model here  //////////////////////

module pll(input wire refclk, input wire rst, output wire outclk_0, output wire locked);
assign outclk_0 = top_tb.clk_sys;
assign locked   = 1'b1;
endmodule

module pll_hdmi
(
	input wire refclk, input wire rst, output wire outclk_0,
	input wire [63:0] reconfig_to_pll, output wire [63:0] reconfig_from_pll
);
assign outclk_0 = top_tb.clk_hdmi;
assign reconfig_from_pll = 0;
endmodule

module pll_audio(input wire refclk, input wire rst, output wire outclk_0);
assign outclk_0 = top_tb.clk_audio;
endmodule

module pll_cfg_hdmi
#(parameter reconf_width = 64, parameter device_family = "Cyclone V")
(
	input  wire mgmt_clk, input wire mgmt_reset,
	output wire [reconf_width-1:0] reconfig_to_pll, input wire [reconf_width-1:0] reconfig_from_pll,
	output wire mgmt_waitrequest, input wire [5:0] mgmt_address, input wire mgmt_write, input wire [31:0] mgmt_writedata
);
assign reconfig_to_pll  = 0;
assign mgmt_waitrequest = 1'b0;
endmodule

//////////////////////  VHDL entities: Icarus compiles no VHDL  //////////////////////

module pll_hdmi_adj
(
	input  wire llena, input wire [15:0] lltune, output wire locked,
	output wire i_waitrequest, input wire i_write, input wire [5:0] i_address, input wire [31:0] i_writedata,
	input  wire o_waitrequest, output wire o_write, output wire [5:0] o_address, output wire [31:0] o_writedata,
	input  wire clk, input wire reset_na
);
assign {locked, i_waitrequest, o_write, o_address, o_writedata} = {1'b1, 1'b0, 1'b0, 6'd0, 32'd0};
endmodule

module ascal
#(
	parameter MASK = 8'hFF, parameter RAMBASE = 0, parameter RAMSIZE = 32'h00800000, parameter INTER = "true",
	parameter HEADER = "true", parameter DOWNSCALE = "true", parameter BYTESWAP = "true", parameter PALETTE = "true",
	parameter PALETTE2 = "true", parameter ADAPTIVE = "true", parameter DOWNSCALE_NN = "false", parameter FRAC = 4,
	parameter OHRES = 2304, parameter IHRES = 2048, parameter N_DW = 128, parameter N_AW = 32, parameter N_BURST = 256
)
(
	input  wire [7:0] i_r, input wire [7:0] i_g, input wire [7:0] i_b,
	input  wire i_hs, input wire i_vs, input wire i_fl, input wire i_de, input wire i_ce, input wire i_clk,
	output wire [7:0] o_r, output wire [7:0] o_g, output wire [7:0] o_b,
	output wire o_hs, output wire o_vs, output wire o_de, output wire o_vbl, output wire o_brd,
	input  wire o_ce, input wire o_clk,
	input  wire o_fb_ena, input wire [11:0] o_fb_hsize, input wire [11:0] o_fb_vsize, input wire [5:0] o_fb_format,
	input  wire [31:0] o_fb_base, input wire [13:0] o_fb_stride,
	input  wire pal1_clk, input wire [47:0] pal1_dw, input wire [6:0] pal1_a, input wire pal1_wr, input wire pal_n,
	input  wire pal2_clk, input wire [23:0] pal2_dw, output wire [23:0] pal2_dr, input wire [7:0] pal2_a,
	input  wire pal2_wr,
	output wire [15:0] o_lltune,
	input  wire iauto, input wire [11:0] himin, input wire [11:0] himax, input wire [11:0] vimin,
	input  wire [11:0] vimax,
	input  wire run, input wire freeze, input wire [4:0] mode, input wire bob_deint,
	input  wire [11:0] htotal, input wire [11:0] hsstart, input wire [11:0] hsend, input wire [11:0] hdisp,
	input  wire [11:0] hmin, input wire [11:0] hmax, input wire [11:0] vtotal, input wire [11:0] vsstart,
	input  wire [11:0] vsend, input wire [11:0] vdisp, input wire [11:0] vmin, input wire [11:0] vmax,
	input  wire vrr, input wire [11:0] vrrmax, input wire swblack,
	input  wire poly_clk, input wire [9:0] poly_dw, input wire [FRAC+3:0] poly_a, input wire poly_wr,
	input  wire avl_clk, input wire avl_waitrequest, input wire [N_DW-1:0] avl_readdata, input wire avl_readdatavalid,
	output wire [7:0] avl_burstcount, output wire [N_DW-1:0] avl_writedata, output wire [N_AW-1:0] avl_address,
	output wire avl_write, output wire avl_read, output wire [N_DW/8-1:0] avl_byteenable,
	input  wire reset_na
);
assign {o_r, o_g, o_b, o_hs, o_vs, o_de, o_vbl, o_brd, pal2_dr, o_lltune} = 0;
assign {avl_burstcount, avl_writedata, avl_address, avl_write, avl_read, avl_byteenable} = 0;
endmodule

//////////////////////  Stock modules Icarus 12 cannot compile  //////////////////////

// yc_out.sv declares an unpacked struct.
module yc_out
(
	input  wire clk, input wire [39:0] PHASE_INC, input wire PAL_EN, input wire CVBS, input wire [16:0] COLORBURST_RANGE,
	input  wire hsync, input wire vsync, input wire csync, input wire de,
	input  wire [23:0] din, output wire [23:0] dout,
	output wire hsync_o, output wire vsync_o, output wire csync_o, output wire de_o
);
assign {dout, hsync_o, vsync_o, csync_o, de_o} = 0;
endmodule

// mcp23009.sv initialises an unpacked net array with an assignment pattern.
module mcp23009
(
	input  wire clk, output wire [2:0] btn, input wire [2:0] led,
	output wire flg_sd_cd, output wire flg_present, output wire flg_mode,
	output wire scl, inout wire sda
);
assign {btn, flg_sd_cd, flg_present, flg_mode, scl} = 7'b0000001;
endmodule

// audio_out.sv: an assignment pattern in a constant expression stops Icarus 12 with an internal assertion.
module audio_out
#(parameter CLK_RATE = 24576000)
(
	input  wire reset, input wire clk, input wire sample_rate,
	input  wire [31:0] flt_rate, input wire [39:0] cx, input wire [7:0] cx0, input wire [7:0] cx1, input wire [7:0] cx2,
	input  wire [23:0] cy0, input wire [23:0] cy1, input wire [23:0] cy2,
	input  wire [4:0] att, input wire [1:0] boost, input wire [1:0] mix,
	input  wire is_signed, input wire [15:0] core_l, input wire [15:0] core_r,
	input  wire [15:0] alsa_l, input wire [15:0] alsa_r,
	output wire i2s_bclk, output wire i2s_lrclk, output wire i2s_data, output wire spdif,
	output wire dac_l, output wire dac_r
);
assign {i2s_bclk, i2s_lrclk, i2s_data, spdif, dac_l, dac_r} = 0;
endmodule

// f2sdram_safe_terminator.sv assigns to nets declared as wire from always blocks, which Quartus accepts.
module f2sdram_safe_terminator
#(parameter DATA_WIDTH = 64, parameter BURSTCOUNT_WIDTH = 8)
(
	input  wire clk, input wire rst_req_sync,
	input  wire waitrequest_master, output wire [BURSTCOUNT_WIDTH-1:0] burstcount_master,
	output wire [31-$clog2(DATA_WIDTH/8):0] address_master, input wire [DATA_WIDTH-1:0] readdata_master,
	input  wire readdatavalid_master, output wire read_master, output wire [DATA_WIDTH-1:0] writedata_master,
	output wire [DATA_WIDTH/8-1:0] byteenable_master, output wire write_master,
	output wire waitrequest_slave, input wire [BURSTCOUNT_WIDTH-1:0] burstcount_slave,
	input  wire [31-$clog2(DATA_WIDTH/8):0] address_slave, output wire [DATA_WIDTH-1:0] readdata_slave,
	output wire readdatavalid_slave, input wire read_slave, input wire [DATA_WIDTH-1:0] writedata_slave,
	input  wire [DATA_WIDTH/8-1:0] byteenable_slave, input wire write_slave
);
assign {burstcount_master, address_master, read_master, writedata_master, byteenable_master, write_master} = 0;
assign {waitrequest_slave, readdata_slave, readdatavalid_slave} = 0;
endmodule

// ddr_svc.sv likewise assigns to an output declared as a net (ram_bcnt) from an always block.
module ddr_svc
(
	input  wire clk,
	input  wire ram_waitrequest, output wire [7:0] ram_burstcnt, output wire [28:0] ram_addr,
	input  wire [63:0] ram_readdata, input wire ram_read_ready, output wire ram_read,
	output wire [63:0] ram_writedata, output wire [7:0] ram_byteenable, output wire ram_write,
	output wire [7:0] ram_bcnt,
	input  wire [31:3] ch0_addr, input wire [7:0] ch0_burst, output wire [63:0] ch0_data, input wire ch0_req,
	output wire ch0_ready,
	input  wire [31:3] ch1_addr, input wire [7:0] ch1_burst, output wire [63:0] ch1_data, input wire ch1_req,
	output wire ch1_ready
);
assign {ram_burstcnt, ram_addr, ram_read, ram_writedata, ram_byteenable, ram_write, ram_bcnt} = 0;
assign {ch0_data, ch0_ready, ch1_data, ch1_ready} = 0;
endmodule

// rtl/cos.sv (the Template's core) initialises an unpacked net array with an assignment pattern.
module cos(input wire [9:0] x, output wire [7:0] y);
assign y = 0;
endmodule

// verilator lint_on UNUSEDPARAM
// verilator lint_on UNUSEDSIGNAL
`default_nettype wire
