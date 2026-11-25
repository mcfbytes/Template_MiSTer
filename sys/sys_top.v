//============================================================================
//
//  MiSTer hardware abstraction module
//  (c)2017-2020 Alexey Melnikov
//
//  This program is free software; you can redistribute it and/or modify it
//  under the terms of the GNU General Public License as published by the Free
//  Software Foundation; either version 2 of the License, or (at your option)
//  any later version.
//
//  This program is distributed in the hope that it will be useful, but WITHOUT
//  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
//  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
//  more details.
//
//  You should have received a copy of the GNU General Public License along
//  with this program; if not, write to the Free Software Foundation, Inc.,
//  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
//
//============================================================================

module sys_top
(
	/////////// CLOCK //////////
	input         FPGA_CLK1_50,
	input         FPGA_CLK2_50,
	input         FPGA_CLK3_50,

	//////////// HDMI //////////
	output        HDMI_I2C_SCL,
	inout         HDMI_I2C_SDA,

	output        HDMI_MCLK,
	output        HDMI_SCLK,
	output        HDMI_LRCLK,
	output        HDMI_I2S,

	output        HDMI_TX_CLK,
	output        HDMI_TX_DE,
	output [23:0] HDMI_TX_D,
	output        HDMI_TX_HS,
	output        HDMI_TX_VS,
	
	input         HDMI_TX_INT,

	//////////// SDR ///////////
	output [12:0] SDRAM_A,
	inout  [15:0] SDRAM_DQ,
	output        SDRAM_DQML,
	output        SDRAM_DQMH,
	output        SDRAM_nWE,
	output        SDRAM_nCAS,
	output        SDRAM_nRAS,
	output        SDRAM_nCS,
	output  [1:0] SDRAM_BA,
	output        SDRAM_CLK,
	output        SDRAM_CKE,

`ifdef MISTER_DUAL_SDRAM
	////////// SDR #2 //////////
	output [12:0] SDRAM2_A,
	inout  [15:0] SDRAM2_DQ,
	output        SDRAM2_nWE,
	output        SDRAM2_nCAS,
	output        SDRAM2_nRAS,
	output        SDRAM2_nCS,
	output  [1:0] SDRAM2_BA,
	output        SDRAM2_CLK,

`else
	//////////// VGA ///////////
	output  [5:0] VGA_R,
	output  [5:0] VGA_G,
	output  [5:0] VGA_B,
	inout         VGA_HS,
	output		  VGA_VS,
	input         VGA_EN,  // active low

	/////////// AUDIO //////////
	output		  AUDIO_L,
	output		  AUDIO_R,
	output		  AUDIO_SPDIF,

	//////////// SDIO ///////////
	inout   [3:0] SDIO_DAT,
	inout         SDIO_CMD,
	output        SDIO_CLK,

	//////////// I/O ///////////
	output        LED_USER,
	output        LED_HDD,
	output        LED_POWER,
	input         BTN_USER,
	input         BTN_OSD,
	input         BTN_RESET,
`endif

	////////// I/O ALT /////////
	output        SD_SPI_CS,
	input         SD_SPI_MISO,
	output        SD_SPI_CLK,
	output        SD_SPI_MOSI,

	inout         SDCD_SPDIF,
	output        IO_SCL,
	inout         IO_SDA,

	////////// ADC //////////////
	output        ADC_SCK,
	input         ADC_SDO,
	output        ADC_SDI,
	output        ADC_CONVST,

	////////// MB KEY ///////////
	input   [1:0] KEY,

	////////// MB SWITCH ////////
	input   [3:0] SW,

	////////// MB LED ///////////
	output  [7:0] LED,

	///////// USER IO ///////////
	inout   [6:0] USER_IO
);

//////////////////////  Secondary SD  ///////////////////////////////////
wire SD_CS, SD_CLK, SD_MOSI, SD_MISO, SD_CD;

`ifndef MISTER_DUAL_SDRAM
	wire   sd_cd       = SDCD_SPDIF & ~SW[2]; // SW[2]=ON workaround for faulty boards without SD card detect pin.
	assign SD_CD       = mcp_en ? mcp_sdcd : sd_cd;
	assign SD_MISO     = SD_CD | (mcp_en ? SD_SPI_MISO : (VGA_EN | SDIO_DAT[0]));
	assign SD_SPI_CS   = mcp_en ?  (mcp_sdcd  ? 1'bZ : SD_CS) : (sog & ~cs1 & ~VGA_EN) ? 1'b1 : 1'bZ;
	assign SD_SPI_CLK  = (~mcp_en | mcp_sdcd) ? 1'bZ : SD_CLK;
	assign SD_SPI_MOSI = (~mcp_en | mcp_sdcd) ? 1'bZ : SD_MOSI;
	assign {SDIO_CLK,SDIO_CMD,SDIO_DAT} = av_dis ? 6'bZZZZZZ : (mcp_en | sd_cd) ? {vga_g,vga_r,vga_b} : {SD_CLK,SD_MOSI,SD_CS,3'bZZZ};
`else
	assign SD_CD       = mcp_sdcd;
	assign SD_MISO     = mcp_sdcd | SD_SPI_MISO;
	assign SD_SPI_CS   = mcp_sdcd ? 1'bZ : SD_CS;
	assign SD_SPI_CLK  = mcp_sdcd ? 1'bZ : SD_CLK;
	assign SD_SPI_MOSI = mcp_sdcd ? 1'bZ : SD_MOSI;
`endif

//////////////////////  LEDs/Buttons  ///////////////////////////////////

reg [7:0] led_overtake = 0;
reg [7:0] led_state    = 0;

wire led_p =  led_power[1] ? ~led_power[0] : 1'b0;
wire led_d =  led_disk[1]  ? ~led_disk[0]  : ~(led_disk[0] | gp_out[29]);
wire led_u = ~led_user;
wire led_locked;

//LEDs on de10-nano board
assign LED = (led_overtake & led_state) | (~led_overtake & {1'b0,led_locked,1'b0, ~led_p, 1'b0, ~led_d, 1'b0, ~led_u});

wire [2:0] mcp_btn;
wire       mcp_sdcd;
wire       mcp_en;
wire       mcp_mode;
mcp23009 mcp23009
(
	.clk(FPGA_CLK2_50),

	.btn(mcp_btn),
	.led({led_p, led_d, led_u}),
	.flg_sd_cd(mcp_sdcd),
	.flg_present(mcp_en),
	.flg_mode(mcp_mode),

	.scl(IO_SCL),
	.sda(IO_SDA)
);

wire io_dig = mcp_en ? mcp_mode : SW[3];

`ifndef MISTER_DUAL_SDRAM
	wire   av_dis    = io_dig | VGA_EN;
	assign LED_POWER = av_dis ? 1'bZ : mcp_en ? de1          : led_p ? 1'bZ : 1'b0;
	assign LED_HDD   = av_dis ? 1'bZ : mcp_en ? (sog & ~cs1) : led_d ? 1'bZ : 1'b0;
	//assign LED_USER  = av_dis ? 1'bZ : mcp_en ? ~vga_tx_clk  : led_u ? 1'bZ : 1'b0;
	assign LED_USER  = VGA_TX_CLK;
	wire   BTN_DIS   = VGA_EN;
`else
	wire   BTN_RESET = 1'b1;
	wire   BTN_OSD   = 1'b1;
	wire   BTN_USER  = 1'b1;
	wire   BTN_DIS   = 1'b1;
`endif

reg BTN_EN = 0;
reg [25:0] btn_timeout = 0;
initial btn_timeout = 0;
always @(posedge FPGA_CLK2_50) begin
	reg btn_up = 0;
	reg btn_en = 0;

	btn_up <= BTN_RESET & BTN_OSD & BTN_USER;
	if(~reset & btn_up & ~&btn_timeout) btn_timeout <= btn_timeout + 1'd1;
	btn_en <= ~BTN_DIS;
	BTN_EN <= &btn_timeout & btn_en;
end

wire btn_r = (mcp_en | SW[3]) ? mcp_btn[1] : (BTN_EN & ~BTN_RESET);
wire btn_o = (mcp_en | SW[3]) ? mcp_btn[2] : (BTN_EN & ~BTN_OSD  );
wire btn_u = (mcp_en | SW[3]) ? mcp_btn[0] : (BTN_EN & ~BTN_USER );

reg btn_user, btn_osd;
always @(posedge FPGA_CLK2_50) begin
	integer div;
	reg [7:0] deb_user;
	reg [7:0] deb_osd;

	div <= div + 1'b1;
	if(div > 100000) div <= 0;

	if(!div) begin
		deb_user <= {deb_user[6:0], btn_u | ~KEY[1]};
		if(&deb_user) btn_user <= 1;
		if(!deb_user) btn_user <= 0;

		deb_osd <= {deb_osd[6:0], btn_o | ~KEY[0]};
		if(&deb_osd) btn_osd <= 1;
		if(!deb_osd) btn_osd <= 0;
	end
end

/////////////////////////  HPS I/O  /////////////////////////////////////

// MISTER_LW_LANE (with MISTER_LW): the link nets below come from lw_link (LW BRIDGE), the link's own or the
// LW frame lane's while it plays a frame. Without it the link front is the stock one.
// MISTER_LW_OSD (with MISTER_LW): each osd takes the LW OSDBUF writer's bytes through its own write port.
`ifdef MISTER_LW
`ifdef MISTER_LW_LANE
`define LW_LANE_LINK
`endif
`ifdef MISTER_LW_OSD
`define LW_OSD_PORTS
`endif
`endif

// MISTER_OCRAM_OSD: the OCRAM OSD writer drives the same osd_buffer port the LW OSDBUF writer uses. The host
// binds one writer, so the two never meet in a legal session; a build with neither keeps the port out of osd.v.
`ifdef MISTER_OCRAM
`ifdef MISTER_OCRAM_OSD
`define OCRAM_OSD_PORTS
`endif
`endif
`ifdef LW_OSD_PORTS
`define OSD_WPORTS
`endif
`ifdef OCRAM_OSD_PORTS
`ifndef OSD_WPORTS
`define OSD_WPORTS
`endif
`endif
// OCRAM_RX_PORTS: hps_ocram_rx is built, for the OSD ring, the SECTOR ring (MISTER_OCRAM_SECTOR_SLOT) or both.
`ifdef MISTER_OCRAM
`ifdef MISTER_OCRAM_OSD
`define OCRAM_RX_PORTS
`endif
`ifdef MISTER_OCRAM_SECTOR_SLOT
`ifndef OCRAM_RX_PORTS
`define OCRAM_RX_PORTS
`endif
`endif
`endif

// gp_in[31] = 0 - quick flag that FPGA is initialized (HPS reads 1 when FPGA is not in user mode)
//                 used to avoid lockups while JTAG loading
`ifdef MISTER_OCRAM
// gp_in[26] = 1 - the OCRAM floor is built and its master runs (hps_ocram_master.sv drives it)
wire        ocram_present, ocram_line;
// The master's consumer port 0 (hps_ocram_rx's walker) and its session interface. With neither ring the port is
// tied off and the OSD statistics are the constant 0; without the SECTOR tier its nets are constants.
wire        ocram_c0_req, ocram_c0_we;
wire [15:0] ocram_c0_off;
wire [31:0] ocram_c0_wdata;
wire        ocram_c0_grant, ocram_c0_done, ocram_c0_ok;
wire [31:0] ocram_c0_rdata;
wire        ocram_s_hello, ocram_s_live, ocram_s_tick, ocram_osd_bind;
wire        ocram_c0_rest, ocram_c0_fault, ocram_c0_mid, ocram_c0_open;
wire [15:0] ocram_osd_rows, ocram_osd_deepest;
wire        ocram_sec_ask, ocram_evt_stop, ocram_c0_sfault, ocram_c0_smid, ocram_c0_sbind, ocram_c0_wide;
wire        ocram_rq_post, ocram_rq_start;
wire [31:0] ocram_rq_lba;
// The SECTOR tier's declaration (its .qsf macros): the hps_io slot, the frame in 32-bit words (1-4096), the
// strobe gap (2-15) and BLKSZ; hps_ocram_rx stops the build on an unservable geometry.
`ifdef MISTER_OCRAM_SECTOR_SLOT
localparam        OCRAM_SEC_EN    = 1;
localparam [31:0] OCRAM_SEC_SLOT  = `MISTER_OCRAM_SECTOR_SLOT;
`else
localparam        OCRAM_SEC_EN    = 0;
localparam [31:0] OCRAM_SEC_SLOT  = 32'd1;
`endif
`ifdef MISTER_OCRAM_SECTOR_FRAME
localparam [31:0] OCRAM_SEC_FRAME = `MISTER_OCRAM_SECTOR_FRAME;
`else
localparam [31:0] OCRAM_SEC_FRAME = 32'd128;
`endif
`ifdef MISTER_OCRAM_SECTOR_GAP
localparam [31:0] OCRAM_SEC_GAP   = `MISTER_OCRAM_SECTOR_GAP;
`else
localparam [31:0] OCRAM_SEC_GAP   = 32'd2;
`endif
`ifdef MISTER_OCRAM_BLKSZ
localparam  [2:0] OCRAM_BLKSZ     = `MISTER_OCRAM_BLKSZ;
`else
localparam  [2:0] OCRAM_BLKSZ     = 3'd2;
`endif
`ifdef MISTER_OCRAM_SECTOR_SLOT
// the feeder's nets to the window's primary-slot bits (lw_sys_window's xsec_*)
wire        xsec_en, xsec_ack, xsec_wr, xsec_reset, xsec_hps_ack, xsec_req, xsec_commit;
wire [15:0] xsec_dat;
wire [13:0] xsec_adr;
wire [31:0] xsec_lba;
`endif
`ifdef MISTER_OCRAM_OSD
wire        ocram_osd_we;
wire [12:0] ocram_osd_addr;
wire  [7:0] ocram_osd_byte;
`endif
`ifndef OCRAM_RX_PORTS
assign ocram_c0_sfault = 1'b0;
assign ocram_c0_smid   = 1'b0;
assign ocram_c0_sbind  = 1'b0;
assign ocram_c0_wide   = 1'b0;
assign ocram_rq_post   = 1'b0;
assign ocram_rq_lba    = 32'd0;
assign ocram_c0_req   = 1'b0;
assign ocram_c0_we    = 1'b0;
assign ocram_c0_off   = 16'd0;
assign ocram_c0_wdata = 32'd0;
assign ocram_c0_rest  = 1'b1;
assign ocram_c0_fault = 1'b0;
assign ocram_c0_mid   = 1'b0;
assign ocram_c0_open  = 1'b0;
assign ocram_osd_rows = 16'd0;
assign ocram_osd_deepest = 16'd0;
`endif
`endif
`ifdef MISTER_LW
// gp_in[25] = 1 - the LW firewall answers every LW address (hps_lw_bridge.sv drives it)
// gp_in[24]     - the firewall holds no transaction (its idle register)
// gp_in[22]     - the LW host enable is set (lw_sys_window's enable register)
wire        lw_present, lw_idle, lw_held;
`ifdef MISTER_OCRAM
wire [31:0] gp_in = {1'b0, btn_user | btn[1], btn_osd | btn[0], io_dig, 1'b0, ocram_present, lw_present, lw_idle, 1'b0, lw_held, 1'b0, ~HDMI_TX_INT, io_ver, io_ack, io_wide, io_dout | io_dout_sys};
`else
wire [31:0] gp_in = {1'b0, btn_user | btn[1], btn_osd | btn[0], io_dig, 2'b00, lw_present, lw_idle, 1'b0, lw_held, 1'b0, ~HDMI_TX_INT, io_ver, io_ack, io_wide, io_dout | io_dout_sys};
`endif
`else
`ifdef MISTER_OCRAM
wire [31:0] gp_in = {1'b0, btn_user | btn[1], btn_osd | btn[0], io_dig, 1'b0, ocram_present, 5'd0, ~HDMI_TX_INT, io_ver, io_ack, io_wide, io_dout | io_dout_sys};
`else
wire [31:0] gp_in = {1'b0, btn_user | btn[1], btn_osd | btn[0], io_dig, 7'd0, ~HDMI_TX_INT, io_ver, io_ack, io_wide, io_dout | io_dout_sys};
`endif
`endif
wire [31:0] gp_out;

wire  [1:0] io_ver = 1; // 0 - obsolete. 1 - optimized HPS I/O. 2,3 - reserved for future.
wire        io_wait;
wire        io_wide;
wire [15:0] io_dout;
`ifdef LW_LANE_LINK
wire [15:0] io_din;
`else
wire [15:0] io_din = gp_outr[15:0];
`endif
wire        io_clk = gp_outr[17];
wire        io_ss0 = gp_outr[18];
wire        io_ss1 = gp_outr[19];
wire        io_ss2 = gp_outr[20];

`ifdef LW_LANE_LINK
wire  [3:0] io_sel;     // {osd_vga, osd_hdmi, fpga, uio}
wire        io_strobe;
wire        io_ack;

`ifndef MISTER_DEBUG_NOHDMI
	wire io_osd_hdmi = io_sel[2];
`endif

wire io_fpga     = io_sel[1];
wire io_uio      = io_sel[0];
`else
`ifndef MISTER_DEBUG_NOHDMI
	wire io_osd_hdmi = io_ss1 & ~io_ss0;
`endif

wire io_fpga     = ~io_ss1 & io_ss0;
wire io_uio      = ~io_ss1 & io_ss2;

reg  io_ack;
reg  rack;
wire io_strobe = ~rack & io_clk;

always @(posedge clk_sys) begin
	if(~(io_wait | vs_wait) | io_strobe) begin
		rack <= io_clk;
		io_ack <= rack;
	end
end
`endif

reg [31:0] gp_outr;
always @(posedge clk_sys) begin
	reg [31:0] gp_outd;
	gp_outr <= gp_outd;
	gp_outd <= gp_out;
end

`ifdef MISTER_DUAL_SDRAM
	wire  [7:0] core_type  = 'hA8; // generic core, dual SDRAM.
`else
	wire  [7:0] core_type  = 'hA4; // generic core.
`endif

// HPS will not communicate to core if magic is different
wire [31:0] core_magic = {24'h5CA623, core_type};

cyclonev_hps_interface_mpu_general_purpose h2f_gp
(
	.gp_in({~gp_out[31] ? core_magic : gp_in}),
	.gp_out(gp_out)
);


reg [15:0] cfg;
reg        cfg_set      = 0;

`ifdef MISTER_DEBUG_NOHDMI
	wire    vga_fb       = 0;
	wire    direct_video = 1;
`else
	wire    vga_fb       = cfg[12] | vga_force_scaler;
	wire    direct_video = cfg[10];
`endif

wire       audio_96k    = cfg[6];
wire       csync_en     = cfg[3];
`ifdef LW_LANE_LINK
wire       io_osd_vga   = io_sel[3];
`else
wire       io_osd_vga   = io_ss1 & ~io_ss2;
`endif
`ifndef MISTER_DUAL_SDRAM
	wire forced_scandoubler = cfg[4];
	wire ypbpr_en           = cfg[5];
	wire sog                = cfg[9];
	`ifdef MISTER_DEBUG_NOHDMI
		wire vga_scaler   = 0;
	`else
		wire vga_scaler   = cfg[2] | vga_force_scaler;
	`endif
	`ifdef MISTER_DISABLE_VGA_SCALER
		wire vgas_en      = 0;
		wire scaler_no_vga = 1;
	`elsif MISTER_DISABLE_DOWNSCALE
		wire scaler_no_vga = 0;
	`endif
`else
	// Dual SDRAM boards have no analog output, so MISTER_DISABLE_VGA_SCALER has nothing to remove.
	`ifdef MISTER_DISABLE_VGA_SCALER
		wire scaler_no_vga = 0;
	`elsif MISTER_DISABLE_DOWNSCALE
		wire scaler_no_vga = 0;
	`endif
`endif

// UIO_SET_VIDEO (0x20) reply: bit 0 pixel repetition, bit 1 VRR, bits 2+ a scaler feature compiled out.
// A set bit means the feature is absent, so a framework without these bits reads as complete. Without either
// macro the reply, and every line below, is upstream's text.
`ifdef MISTER_DISABLE_DOWNSCALE
	wire scaler_no_downscale = 1;
`elsif MISTER_DISABLE_VGA_SCALER
	wire scaler_no_downscale = 0;
`endif

reg        cfg_custom_t = 0;
reg  [5:0] cfg_custom_p1;
reg [31:0] cfg_custom_p2;

reg  [4:0] vol_att;
initial vol_att = 5'b11111;
reg  [1:0] vol_boost = 0;

reg  [11:0] coef_addr;
reg  [9:0] coef_data;
reg        coef_wr = 0;

wire[12:0] ARX, ARY;
reg [11:0] VSET = 0, HSET = 0;
reg        FREESCALE = 0;
reg  [2:0] scaler_flt;
reg        lowlat = 0;
reg        cfg_done = 0;

reg        vs_wait = 0;
reg [11:0] vs_line = 0;

reg        scaler_out = 0;
reg        vrr_mode = 0;
wire       hdmi_blackout;

reg [31:0] aflt_rate = 7056000;
reg [39:0] acx  = 4258969;
reg  [7:0] acx0 = 3;
reg  [7:0] acx1 = 3;
reg  [7:0] acx2 = 1;
reg [23:0] acy0 = -24'd6216759;
reg [23:0] acy1 =  24'd6143386;
reg [23:0] acy2 = -24'd2023767;
reg        areset = 0;
reg [12:0] arc1x = 0;
reg [12:0] arc1y = 0;
reg [12:0] arc2x = 0;
reg [12:0] arc2y = 0;
reg [15:0] io_dout_sys;

always@(posedge clk_sys) begin
	reg  [7:0] cmd;
	reg        has_cmd;
	reg  [7:0] cnt = 0;
	reg        vs_d0,vs_d1,vs_d2;
	reg  [4:0] acx_att;
	reg  [7:0] fb_crc;
	reg  [1:0] sl_r;

	coef_wr <= 0;
	sl_r <= FB_EN ? 2'b00 : scanlines;

`ifndef MISTER_DEBUG_NOHDMI
	shadowmask_wr <= 0;
`endif

	if(~io_uio) begin
		has_cmd <= 0;
		cmd <= 0;
		areset <= 0;
		acx_att <= 0;
		acx <= acx >> acx_att;
		io_dout_sys <= 0;
	end
	else
	if(io_strobe) begin
		io_dout_sys <= 0;
		if(!has_cmd) begin
			has_cmd <= 1;
			cmd <= io_din[7:0];
			cnt <= 0;
			if(io_din[7:0] == 'h30) vs_wait <= 1;
			if(io_din[7:0] == 'h39) begin
				aflt_rate <= 7056000;
				acx  <= 4258969;
				acx0 <= 3;
				acx1 <= 3;
				acx2 <= 1;
				acy0 <= -24'd6216759;
				acy1 <=  24'd6143386;
				acy2 <= -24'd2023767;
				areset <= 1;
				io_dout_sys <= 'b11;
			end
`ifdef MISTER_DISABLE_DOWNSCALE
			if(io_din[7:0] == 'h20) io_dout_sys <= {12'd0, scaler_no_vga, scaler_no_downscale, 2'b11};
`elsif MISTER_DISABLE_VGA_SCALER
			if(io_din[7:0] == 'h20) io_dout_sys <= {12'd0, scaler_no_vga, scaler_no_downscale, 2'b11};
`else
			if(io_din[7:0] == 'h20) io_dout_sys <= 'b11;
`endif
`ifdef MISTER_DISABLE_ADAPTIVE
			if(io_din[7:0] == 'h2B) io_dout_sys <= {fb_en, sl_r, 4'b0110};
`else
			if(io_din[7:0] == 'h2B) io_dout_sys <= {fb_en, sl_r, 4'b0111};
`endif
			if(io_din[7:0] == 'h2F) io_dout_sys <= 1;
			if(io_din[7:0] == 'h3E) io_dout_sys <= 1;
`ifndef MISTER_DEBUG_NOHDMI
			if(io_din[7:0] == 'h40) io_dout_sys <= fb_crc;
`endif
			if(io_din[7:0] == 'h42) io_dout_sys <= {1'b1, frame_cnt};
			if(io_din[7:0] == 'h44) io_dout_sys <= 1;
		end
		else begin
			cnt <= cnt + 1'd1;
			if(cmd == 1) begin
				cfg <= io_din;
				cfg_set <= 1;
				scaler_out <= 1;
			end
			if(cmd == 'h20) begin
				cfg_set <= 0;
				if(cnt<8) begin
					case(cnt[2:0])
						0: {HDMI_PR,vrr_mode,WIDTH} <= {io_din[15:14], io_din[11:0]};
						1: HFP    <= io_din[11:0];
						2: HS     <= {io_din[15], io_din[11:0]};
						3: HBP    <= io_din[11:0];
						4: HEIGHT <= io_din[11:0];
						5: VFP    <= io_din[11:0];
						6: VS     <= {io_din[15],io_din[11:0]};
						7: VBP    <= io_din[11:0];
					endcase
`ifndef MISTER_DEBUG_NOHDMI
					if(cnt == 1) begin
						cfg_custom_p1 <= 0;
						cfg_custom_p2 <= 0;
						cfg_custom_t <= ~cfg_custom_t;
					end
				end
				else begin
					if(cnt[1:0]==0) cfg_custom_p1 <= io_din[5:0];
					if(cnt[1:0]==1) cfg_custom_p2[15:0]  <= io_din;
					if(cnt[1:0]==2) begin
						cfg_custom_p2[31:16] <= io_din;
						cfg_custom_t <= ~cfg_custom_t;
						cnt[2:0] <= 3'b100;
					end
					if(cnt == 8) {lowlat,cfg_done} <= {io_din[15],1'b1};
`endif
				end
			end
			if(cmd == 'h2F) begin
				case(cnt[3:0])
					0: {LFB_EN,LFB_FLT,LFB_FMT} <= {io_din[15], io_din[14], io_din[5:0]};
					1: LFB_BASE[15:0]  <= io_din[15:0];
					2: LFB_BASE[31:16] <= io_din[15:0];
					3: LFB_WIDTH       <= io_din[11:0];
					4: LFB_HEIGHT      <= io_din[11:0];
					5: LFB_HMIN        <= io_din[11:0];
					6: LFB_HMAX        <= io_din[11:0];
					7: LFB_VMIN        <= io_din[11:0];
					8: LFB_VMAX        <= io_din[11:0];
					9: LFB_STRIDE      <= io_din[13:0];
				endcase
			end
			if(cmd == 'h25) {led_overtake, led_state} <= io_din;
			if(cmd == 'h26) vol_att <= io_din[4:0];
			if(cmd == 'h27) VSET <= io_din[11:0];
			if(cmd == 'h2A) begin
				if(cnt[0]) {coef_wr,coef_data} <= {1'b1,io_din[9:0]};
				else coef_addr <= io_din[11:0];
			end
			if(cmd == 'h2B) scaler_flt <= io_din[2:0];
			if(cmd == 'h37) {FREESCALE,HSET} <= {io_din[15],io_din[11:0]};
			if(cmd == 'h38) vs_line <= io_din[11:0];
			if(cmd == 'h39) begin
				case(cnt[3:0])
					 0: {vol_boost,acx_att} <= io_din[6:0];
					 1: aflt_rate[15:0]  <= io_din;
					 2: aflt_rate[31:16] <= io_din;
					 3: acx[15:0]        <= io_din;
					 4: acx[31:16]       <= io_din;
					 5: acx[39:32]       <= io_din[7:0];
					 6: acx0             <= io_din[7:0];
					 7: acx1             <= io_din[7:0];
					 8: acx2             <= io_din[7:0];
					 9: acy0[15:0]       <= io_din;
					10: acy0[23:16]      <= io_din[7:0];
					11: acy1[15:0]       <= io_din;
					12: acy1[23:16]      <= io_din[7:0];
					13: acy2[15:0]       <= io_din;
					14: acy2[23:16]      <= io_din[7:0];
				endcase
			end
			if(cmd == 'h3A) begin
				case(cnt[3:0])
					 0: arc1x <= io_din[12:0];
					 1: arc1y <= io_din[12:0];
					 2: arc2x <= io_din[12:0];
					 3: arc2y <= io_din[12:0];
				endcase
			end
`ifndef MISTER_DEBUG_NOHDMI
			if(cmd == 'h3E) {shadowmask_wr,shadowmask_data} <= {1'b1, io_din};
			if(cmd == 'h40) begin
				case(cnt[3:0])
					0: io_dout_sys <= {arxy, arx};
					1: io_dout_sys <= {arxy, ary};
					2: io_dout_sys <= {LFB_EN, FB_EN, FB_FMT};
					3: io_dout_sys <= FB_WIDTH;
					4: io_dout_sys <= FB_HEIGHT;
					5: io_dout_sys <= FB_BASE[15:0];
					6: io_dout_sys <= FB_BASE[31:16];
					7: io_dout_sys <= FB_STRIDE;
				endcase
			end
`endif
			if(cmd == 'h41) begin
				case(cnt[3:0])
`ifndef MISTER_DISABLE_YC
					0: {pal_en,cvbs,yc_en}    <= io_din[2:0];
					4: ColorBurst_Range[15:0] <= io_din;
					5: ColorBurst_Range[16]   <= io_din[0];
`endif
					// Subcarrier commands (independent of YC module)
					1: PhaseInc[15:0]         <= io_din;
					2: PhaseInc[31:16]        <= io_din;
					3: PhaseInc[39:32]        <= io_din[7:0];
`ifndef MISTER_DUAL_SDRAM
					6: subcarrier             <= io_din[0];
`endif
				endcase
			end
		end
	end

`ifndef MISTER_DEBUG_NOHDMI
	fb_crc <= {LFB_EN, FB_EN, FB_FMT}
				^ FB_WIDTH[7:0]  ^ FB_WIDTH[11:8] 
				^ FB_HEIGHT[7:0] ^ FB_HEIGHT[11:8] 
				^ arx[7:0] ^ arx[11:8] ^ arxy
				^ ary[7:0] ^ ary[11:8];
`endif

	vs_d0 <= HDMI_TX_VS;
	if(vs_d0 == HDMI_TX_VS) vs_d1 <= vs_d0;

	vs_d2 <= vs_d1;
	if(~vs_d2 & vs_d1) vs_wait <= 0;
end

reg [7:0] frame_cnt;
always @(posedge clk_sys) begin
	reg vs_r, vs_old;
	
	vs_r <= vs_fix;
	if(vs_r == vs_fix) vs_old <= vs_r;
	if(~vs_old & vs_r) frame_cnt <= frame_cnt + 1'd1;
end

cyclonev_hps_interface_peripheral_uart uart
(
	.ri(0),
	.dsr(uart_dsr),
	.dcd(uart_dsr),
	.dtr(uart_dtr),

	.cts(uart_cts),
	.rts(uart_rts),
	.rxd(uart_rxd),
	.txd(uart_txd)
);

`ifdef MISTER_LW
// LW lines and OSDBUF stores (lw_sys_window.sv).
// f2h_irq[7+k] = LW line ordinal k (GIC SPI 47+k), level-high: the entry whose LINE word is k (the LW directory).
wire  [2:0] lw_lines;

`ifdef LW_OSD_PORTS
// The OSDBUF writer's bytes (lw_sys_window, on clk_sys) to every osd's osd_buffer.
wire        lw_osd_we;
wire [12:0] lw_osd_addr;
wire  [7:0] lw_osd_byte;
`endif
`ifdef MISTER_OCRAM
// f2h_irq[11] = the OCRAM EVT ring's line, framework ordinal 4 (GIC SPI 51).
wire [63:0] f2h_irq = {52'd0, ocram_line, lw_lines, 6'd0, video_sync, HDMI_TX_VS};
`else
wire [63:0] f2h_irq = {53'd0, lw_lines, 6'd0, video_sync, HDMI_TX_VS};
`endif
`else
`ifdef MISTER_OCRAM
// f2h_irq[11] = the OCRAM EVT ring's line, framework ordinal 4 (GIC SPI 51).
wire [63:0] f2h_irq = {52'd0, ocram_line, 9'd0, video_sync, HDMI_TX_VS};
`else
wire [63:0] f2h_irq = {video_sync,HDMI_TX_VS};
`endif
`endif
cyclonev_hps_interface_interrupts interrupts
(
	.irq(f2h_irq)
);

// The osd write port's bytes (osd.v's lw_we/lw_addr/lw_byte): the OCRAM writer's when it writes, else the LW
// writer's. Both are built only in a prix-fixe build and the host binds one, so a collision cannot happen in a
// legal session. OSD_WPORTS means at least one writer is built.
`ifdef OSD_WPORTS
wire        osd_we;
wire [12:0] osd_addr;
wire  [7:0] osd_byte;
`ifdef OCRAM_OSD_PORTS
`ifdef LW_OSD_PORTS
assign osd_we   = ocram_osd_we | lw_osd_we;
assign osd_addr = ocram_osd_we ? ocram_osd_addr : lw_osd_addr;
assign osd_byte = ocram_osd_we ? ocram_osd_byte : lw_osd_byte;
`else
assign osd_we   = ocram_osd_we;
assign osd_addr = ocram_osd_addr;
assign osd_byte = ocram_osd_byte;
`endif
`else
assign osd_we   = lw_osd_we;
assign osd_addr = lw_osd_addr;
assign osd_byte = lw_osd_byte;
`endif
`endif

///////////////////////////  RESET  ///////////////////////////////////

reg reset_req = 0;
always @(posedge FPGA_CLK2_50) begin
	reg [1:0] resetd, resetd2;
	reg       old_reset;

	//latch the reset
	old_reset <= reset;
	if(~old_reset & reset) reset_req <= 1;

	//special combination to set/clear the reset
	//preventing of accidental reset control
	if(resetd==1) reset_req <= 1;
	if(resetd==2 && resetd2==0) reset_req <= 0;

	resetd  <= gp_out[31:30];
	resetd2 <= resetd;
end

////////////////////  SYSTEM MEMORY & SCALER  /////////////////////////

wire reset;
wire clk_100m;
`ifdef MISTER_LW
wire lw_hps_rst_n;     // the HPS's fabric reset, to the LW firewall and the OCRAM master
`else
`ifdef MISTER_OCRAM
wire ocram_hps_rst_n;  // the HPS's fabric reset, to the OCRAM master
`endif
`endif
`ifdef MISTER_OCRAM
// The fpga2hps AXI3 master bundle between hps_ocram_master and sysmem (32-bit data).
wire  [7:0] ocram_awid, ocram_wid, ocram_bid, ocram_arid, ocram_rid;
wire [31:0] ocram_awaddr, ocram_wdata, ocram_araddr, ocram_rdata;
wire  [3:0] ocram_awlen, ocram_awcache, ocram_wstrb, ocram_arlen, ocram_arcache;
wire  [2:0] ocram_awsize, ocram_awprot, ocram_arsize, ocram_arprot;
wire  [1:0] ocram_awburst, ocram_awlock, ocram_bresp, ocram_arburst, ocram_arlock, ocram_rresp;
wire  [4:0] ocram_awuser, ocram_aruser;
wire        ocram_awvalid, ocram_awready, ocram_wlast, ocram_wvalid, ocram_wready;
wire        ocram_bvalid, ocram_bready, ocram_arvalid, ocram_arready, ocram_rlast, ocram_rvalid, ocram_rready;
`endif

sysmem_lite sysmem
(
`ifdef MISTER_LW
	.hps_rst_n(lw_hps_rst_n),
`else
`ifdef MISTER_OCRAM
	.hps_rst_n(ocram_hps_rst_n),
`endif
`endif
`ifdef MISTER_OCRAM
	.awid(ocram_awid),
	.awaddr(ocram_awaddr),
	.awlen(ocram_awlen),
	.awsize(ocram_awsize),
	.awburst(ocram_awburst),
	.awlock(ocram_awlock),
	.awcache(ocram_awcache),
	.awprot(ocram_awprot),
	.awuser(ocram_awuser),
	.awvalid(ocram_awvalid),
	.awready(ocram_awready),
	.wid(ocram_wid),
	.wdata(ocram_wdata),
	.wstrb(ocram_wstrb),
	.wlast(ocram_wlast),
	.wvalid(ocram_wvalid),
	.wready(ocram_wready),
	.bid(ocram_bid),
	.bresp(ocram_bresp),
	.bvalid(ocram_bvalid),
	.bready(ocram_bready),
	.arid(ocram_arid),
	.araddr(ocram_araddr),
	.arlen(ocram_arlen),
	.arsize(ocram_arsize),
	.arburst(ocram_arburst),
	.arlock(ocram_arlock),
	.arcache(ocram_arcache),
	.arprot(ocram_arprot),
	.aruser(ocram_aruser),
	.arvalid(ocram_arvalid),
	.arready(ocram_arready),
	.rid(ocram_rid),
	.rdata(ocram_rdata),
	.rresp(ocram_rresp),
	.rlast(ocram_rlast),
	.rvalid(ocram_rvalid),
	.rready(ocram_rready),
`endif
	//Reset/Clock
	.reset_core_req(reset_req),
	.reset_out(reset),
	.clock(clk_100m),

	//DE10-nano has no reset signal on GPIO, so core has to emulate cold reset button.
	.reset_hps_cold_req(btn_r),

	//64-bit DDR3 RAM access
	.ram1_clk(ram_clk),
	.ram1_address(ram_address),
	.ram1_burstcount(ram_burstcount),
	.ram1_waitrequest(ram_waitrequest),
	.ram1_readdata(ram_readdata),
	.ram1_readdatavalid(ram_readdatavalid),
	.ram1_read(ram_read),
	.ram1_writedata(ram_writedata),
	.ram1_byteenable(ram_byteenable),
	.ram1_write(ram_write),

	//64-bit DDR3 RAM access
	.ram2_clk(clk_audio),
	.ram2_address(ram2_address),
	.ram2_burstcount(ram2_burstcount),
	.ram2_waitrequest(ram2_waitrequest),
	.ram2_readdata(ram2_readdata),
	.ram2_readdatavalid(ram2_readdatavalid),
	.ram2_read(ram2_read),
	.ram2_writedata(ram2_writedata),
	.ram2_byteenable(ram2_byteenable),
	.ram2_write(ram2_write),

	//128-bit DDR3 RAM access
	// HDMI frame buffer
	.vbuf_clk(clk_100m),
	.vbuf_address(vbuf_address),
	.vbuf_burstcount(vbuf_burstcount),
	.vbuf_waitrequest(vbuf_waitrequest),
	.vbuf_writedata(vbuf_writedata),
	.vbuf_byteenable(vbuf_byteenable),
	.vbuf_write(vbuf_write),
	.vbuf_readdata(vbuf_readdata),
	.vbuf_readdatavalid(vbuf_readdatavalid),
	.vbuf_read(vbuf_read)
);

`ifdef MISTER_OCRAM
// The OCRAM mailbox floor: the fpga2hps bridge's AXI3 master (docs/OCRAM_MAILBOX.md).
hps_ocram_master #(.SEC_SLOT(OCRAM_SEC_SLOT), .SEC_FRAME(OCRAM_SEC_FRAME), .SEC_GAP(OCRAM_SEC_GAP),
                   .SEC_BLKSZ(OCRAM_BLKSZ)) ocram_master
(
	.clk(clk_100m),
`ifdef MISTER_LW
	.hps_rst_n(lw_hps_rst_n),
`else
	.hps_rst_n(ocram_hps_rst_n),
`endif
	.gp_out(gp_out),

	.present(ocram_present),
	.line(ocram_line),

	.c0_req(ocram_c0_req),
	.c0_we(ocram_c0_we),
	.c0_off(ocram_c0_off),
	.c0_wdata(ocram_c0_wdata),
	.c0_grant(ocram_c0_grant),
	.c0_done(ocram_c0_done),
	.c0_ok(ocram_c0_ok),
	.c0_rdata(ocram_c0_rdata),
	.s_hello(ocram_s_hello),
	.s_live(ocram_s_live),
	.s_tick(ocram_s_tick),
	.osd_bind(ocram_osd_bind),
	.c0_rest(ocram_c0_rest),
	.c0_fault(ocram_c0_fault),
	.c0_mid(ocram_c0_mid),
	.c0_open(ocram_c0_open),
	.osd_rows(ocram_osd_rows),
	.osd_deepest(ocram_osd_deepest),
	.sec_ask(ocram_sec_ask),
	.evt_stop(ocram_evt_stop),
	.c0_sfault(ocram_c0_sfault),
	.c0_smid(ocram_c0_smid),
	.c0_sbind(ocram_c0_sbind),
	.c0_wide(ocram_c0_wide),
	.rq_post(ocram_rq_post),
	.rq_lba(ocram_rq_lba),
	.rq_start(ocram_rq_start),

	.awid(ocram_awid),
	.awaddr(ocram_awaddr),
	.awlen(ocram_awlen),
	.awsize(ocram_awsize),
	.awburst(ocram_awburst),
	.awlock(ocram_awlock),
	.awcache(ocram_awcache),
	.awprot(ocram_awprot),
	.awuser(ocram_awuser),
	.awvalid(ocram_awvalid),
	.awready(ocram_awready),
	.wid(ocram_wid),
	.wdata(ocram_wdata),
	.wstrb(ocram_wstrb),
	.wlast(ocram_wlast),
	.wvalid(ocram_wvalid),
	.wready(ocram_wready),
	.bid(ocram_bid),
	.bresp(ocram_bresp),
	.bvalid(ocram_bvalid),
	.bready(ocram_bready),
	.arid(ocram_arid),
	.araddr(ocram_araddr),
	.arlen(ocram_arlen),
	.arsize(ocram_arsize),
	.arburst(ocram_arburst),
	.arlock(ocram_arlock),
	.arcache(ocram_arcache),
	.arprot(ocram_arprot),
	.aruser(ocram_aruser),
	.arvalid(ocram_arvalid),
	.arready(ocram_arready),
	.rid(ocram_rid),
	.rdata(ocram_rdata),
	.rresp(ocram_rresp),
	.rlast(ocram_rlast),
	.rvalid(ocram_rvalid),
	.rready(ocram_rready)
);

`endif

wire [28:0] ram2_address;
wire  [7:0] ram2_burstcount;
wire  [7:0] ram2_byteenable;
wire        ram2_waitrequest;
wire [63:0] ram2_readdata;
wire [63:0] ram2_writedata;
wire        ram2_readdatavalid;
wire        ram2_read;
wire        ram2_write;
wire  [7:0] ram2_bcnt;

ddr_svc ddr_svc
(
	.clk(clk_audio),

	.ram_waitrequest(ram2_waitrequest),
	.ram_burstcnt(ram2_burstcount),
	.ram_addr(ram2_address),
	.ram_readdata(ram2_readdata),
	.ram_read_ready(ram2_readdatavalid),
	.ram_read(ram2_read),
	.ram_writedata(ram2_writedata),
	.ram_byteenable(ram2_byteenable),
	.ram_write(ram2_write),
	.ram_bcnt(ram2_bcnt),

`ifndef MISTER_DISABLE_ALSA
	.ch0_addr(alsa_address),
	.ch0_burst(1),
	.ch0_data(alsa_readdata),
	.ch0_req(alsa_req),
	.ch0_ready(alsa_ready),
`endif

	.ch1_addr(pal_addr),
	.ch1_burst(128),
	.ch1_data(pal_data),
	.ch1_req(pal_req),
	.ch1_ready(pal_wr)
);

wire clk_pal = clk_audio;


wire  [27:0] vbuf_address;
wire   [7:0] vbuf_burstcount;
wire         vbuf_waitrequest;
wire [127:0] vbuf_readdata;
wire         vbuf_readdatavalid;
wire         vbuf_read;
wire [127:0] vbuf_writedata;
wire  [15:0] vbuf_byteenable;
wire         vbuf_write;

wire  [23:0] hdmi_data;
wire         hdmi_vs, hdmi_hs, hdmi_de, hdmi_vbl, hdmi_brd;
wire         freeze;
wire         bob_deint;

`ifndef MISTER_DEBUG_NOHDMI
	wire clk_hdmi  = hdmi_clk_out;

	ascal 
	#(
		.RAMBASE(32'h20000000),
	`ifdef MISTER_SMALL_VBUF
		.RAMSIZE(32'h00200000),
	`else
		.RAMSIZE(32'h00800000),
	`endif
	`ifndef MISTER_FB
		.PALETTE2("false"),
	`else
		`ifndef MISTER_FB_PALETTE
			.PALETTE2("false"),
		`endif
	`endif
	`ifdef MISTER_DISABLE_ADAPTIVE
		.ADAPTIVE("false"),
	`endif
	`ifdef MISTER_DOWNSCALE_NN
		.DOWNSCALE_NN("true"),
	`endif
	`ifdef MISTER_DISABLE_DOWNSCALE
		.DOWNSCALE("false"),
	`endif
		.FRAC(8),
`ifdef MENU_CORE
		.N_BURST(2048),
`endif
		.N_DW(128),
		.N_AW(28)
	)
	ascal
	(
		.reset_na   (~reset_req),
		.run        (1),
		.freeze     (freeze),
		.bob_deint  (bob_deint),

		.i_clk    (clk_ihdmi),
		.i_ce     (ce_hpix),
		.i_r      (hr_out),
		.i_g      (hg_out),
		.i_b      (hb_out),
		.i_hs     (hhs_fix),
		.i_vs     (hvs_fix),
		.i_fl     (f1),
		.i_de     (hde_emu),
		.iauto    (1),
		.himin    (0),
		.himax    (0),
		.vimin    (0),
		.vimax    (0),

		.o_clk    (clk_hdmi),
		.o_ce     (scaler_out),
		.o_r      (hdmi_data[23:16]),
		.o_g      (hdmi_data[15:8]),
		.o_b      (hdmi_data[7:0]),
		.o_hs     (hdmi_hs),
		.o_vs     (hdmi_vs),
		.o_de     (hdmi_de),
		.o_vbl    (hdmi_vbl),
		.o_brd    (hdmi_brd),
		.o_lltune (lltune),
		.htotal   (WIDTH + HFP + HBP + HS[11:0]),
		.hsstart  (WIDTH + HFP),
		.hsend    (WIDTH + HFP + HS[11:0]),
		.hdisp    (WIDTH),
		.hmin     (hmin),
		.hmax     (hmax),
		.vtotal   (HEIGHT + VFP + VBP + VS[11:0]),
		.vsstart  (HEIGHT + VFP),
		.vsend    (HEIGHT + VFP + VS[11:0]),
		.vdisp    (HEIGHT),
		.vmin     (vmin),
		.vmax     (vmax),
		.vrr      (vrr_mode),
		.vrrmax   (HEIGHT + VBP + VS[11:0] + 12'd1),
		.swblack  (hdmi_blackout),

		.mode     ({~lowlat,LFB_EN ? LFB_FLT : |scaler_flt,2'b00}),
		.poly_clk (clk_sys),
		.poly_a   (coef_addr),
		.poly_dw  (coef_data),
		.poly_wr  (coef_wr),

		.pal1_clk (clk_pal),
		.pal1_dw  (pal_d),
		.pal1_a   (pal_a),
		.pal1_wr  (pal_wr),

	`ifdef MISTER_FB
		`ifdef MISTER_FB_PALETTE
			.pal2_clk (fb_pal_clk),
			.pal2_dw  (fb_pal_d),
			.pal2_dr  (fb_pal_q),
			.pal2_a   (fb_pal_a),
			.pal2_wr  (fb_pal_wr),
			.pal_n    (fb_en),
		`endif
	`endif

		.o_fb_ena         (FB_EN),
		.o_fb_hsize       (FB_WIDTH),
		.o_fb_vsize       (FB_HEIGHT),
		.o_fb_format      (FB_FMT),
		.o_fb_base        (FB_BASE),
		.o_fb_stride      (FB_STRIDE),

		.avl_clk          (clk_100m),
		.avl_waitrequest  (vbuf_waitrequest),
		.avl_readdata     (vbuf_readdata),
		.avl_readdatavalid(vbuf_readdatavalid),
		.avl_burstcount   (vbuf_burstcount),
		.avl_writedata    (vbuf_writedata),
		.avl_address      (vbuf_address),
		.avl_write        (vbuf_write),
		.avl_read         (vbuf_read),
		.avl_byteenable   (vbuf_byteenable)
	);
`endif

reg        LFB_EN     = 0;
reg        LFB_FLT    = 0;
reg  [5:0] LFB_FMT    = 0;
reg [11:0] LFB_WIDTH  = 0;
reg [11:0] LFB_HEIGHT = 0;
reg [11:0] LFB_HMIN   = 0;
reg [11:0] LFB_HMAX   = 0;
reg [11:0] LFB_VMIN   = 0;
reg [11:0] LFB_VMAX   = 0;
reg [31:0] LFB_BASE   = 0;
reg [13:0] LFB_STRIDE = 0;

reg        FB_EN     = 0;
reg  [5:0] FB_FMT    = 0;
reg [11:0] FB_WIDTH  = 0;
reg [11:0] FB_HEIGHT = 0;
reg [31:0] FB_BASE   = 0;
reg [13:0] FB_STRIDE = 0;

always @(posedge clk_sys) begin
	FB_EN <= LFB_EN | fb_en;
	if(LFB_EN) begin
		FB_FMT    <= LFB_FMT;
		FB_WIDTH  <= LFB_WIDTH;
		FB_HEIGHT <= LFB_HEIGHT;
		FB_BASE   <= LFB_BASE;
		FB_STRIDE <= LFB_STRIDE;
	end
	else begin
		FB_FMT    <= fb_fmt;
		FB_WIDTH  <= fb_width;
		FB_HEIGHT <= fb_height;
		FB_BASE   <= fb_base;
		FB_STRIDE <= fb_stride;
	end
end

`ifdef MISTER_FB
	reg fb_vbl;
	always @(posedge clk_vid) fb_vbl <= hdmi_vbl;
`endif

reg  ar_md_start;
wire ar_md_busy;
reg  [11:0] ar_md_mul1, ar_md_mul2, ar_md_div;
wire [11:0] ar_md_res;

sys_umuldiv #(12,12,12) ar_muldiv
(
	.clk(clk_vid),
	.start(ar_md_start),
	.busy(ar_md_busy),

	.mul1(ar_md_mul1),
	.mul2(ar_md_mul2),
	.div(ar_md_div),
	.result(ar_md_res)
);

reg [11:0] hmin;
reg [11:0] hmax;
reg [11:0] vmin;
reg [11:0] vmax;
reg [11:0] hdmi_height;
reg [11:0] hdmi_width;

reg [11:0] arx;
reg [11:0] ary;
reg        arxy;

always @(posedge clk_vid) begin
	reg [11:0] hmini,hmaxi,vmini,vmaxi;
	reg [11:0] wcalc,videow;
	reg [11:0] hcalc,videoh;
	reg  [2:0] state;

	hdmi_height <= (VSET && (VSET < HEIGHT)) ? VSET : HEIGHT;
	hdmi_width  <= (HSET && (HSET < WIDTH))  ? HSET << HDMI_PR : WIDTH << HDMI_PR;

	if(!ARY) begin
		if(ARX == 1) begin
			arx  <= arc1x[11:0];
			ary  <= arc1y[11:0];
			arxy <= arc1x[12] | arc1y[12];
		end
		else if(ARX == 2) begin
			arx  <= arc2x[11:0];
			ary  <= arc2y[11:0];
			arxy <= arc2x[12] | arc2y[12];
		end
		else begin
			arx  <= 0;
			ary  <= 0;
			arxy <= 0;
		end
	end
	else begin
		arx  <= ARX[11:0];
		ary  <= ARY[11:0];
		arxy <= ARX[12] | ARY[12];
	end
	
	ar_md_start <= 0;
	state <= state + 1'd1;
	case(state)
		0: if(LFB_EN) begin
				hmini <= LFB_HMIN;
				vmini <= LFB_VMIN;
				hmaxi <= LFB_HMAX;
				vmaxi <= LFB_VMAX;
				state <= 0;
			end
			else if(FREESCALE || !arx || !ary) begin
				wcalc <= hdmi_width;
				hcalc <= hdmi_height;
				state <= 6;
			end
			else if(arxy) begin
				wcalc <= arx;
				hcalc <= ary;
				state <= 6;
			end

		1: begin
				ar_md_mul1 <= hdmi_height;
				ar_md_mul2 <= arx;
				ar_md_div  <= ary;
				ar_md_start<= 1;
			end
		2: begin
				wcalc <= ar_md_res;
				if(ar_md_start | ar_md_busy) state <= 2;
			end

		3: begin
				ar_md_mul1 <= hdmi_width;
				ar_md_mul2 <= ary;
				ar_md_div  <= arx;
				ar_md_start<= 1;
			end
		4: begin
				hcalc <= ar_md_res;
				if(ar_md_start | ar_md_busy) state <= 4;
			end

		6: begin
				videow <= (wcalc > hdmi_width)  ? (hdmi_width >> HDMI_PR)  : (wcalc[11:0] >> HDMI_PR);
				videoh <= (hcalc > hdmi_height) ? hdmi_height : hcalc[11:0];
			end

		7: begin
				hmini <= ((WIDTH  - videow)>>1);
				hmaxi <= ((WIDTH  - videow)>>1) + videow - 1'd1;
				vmini <= ((HEIGHT - videoh)>>1);
				vmaxi <= ((HEIGHT - videoh)>>1) + videoh - 1'd1;
			end
	endcase
	
	hmin <= hmini;
	hmax <= hmaxi;
	vmin <= vmini;
	vmax <= vmaxi;
end

`ifndef MISTER_DEBUG_NOHDMI
	wire [15:0] lltune;
	pll_hdmi_adj pll_hdmi_adj
	(
		.clk(FPGA_CLK1_50),
		.reset_na(~reset_req),

		.llena(lowlat),
		.lltune({16{cfg_done}} & lltune),
		.locked(led_locked),
		.i_waitrequest(adj_waitrequest),
		.i_write(adj_write),
		.i_address(adj_address),
		.i_writedata(adj_data),
		.o_waitrequest(cfg_waitrequest),
		.o_write(cfg_write),
		.o_address(cfg_address),
		.o_writedata(cfg_data)
	);
`else
	assign led_locked = 0;
`endif

wire [63:0] pal_data;
wire [47:0] pal_d = {pal_data[55:32], pal_data[23:0]};
wire  [6:0] pal_a = ram2_bcnt[6:0];
wire        pal_wr;

reg  [28:0] pal_addr;
reg         pal_req = 0;
always @(posedge clk_pal) begin
	reg old_vs1, old_vs2;

	pal_addr <= LFB_BASE[31:3] - 29'd512;

	old_vs1 <= hdmi_vs;
	old_vs2 <= old_vs1;
	
	if(~old_vs2 & old_vs1 & ~FB_FMT[2] & FB_FMT[1] & FB_FMT[0] & FB_EN) pal_req <= ~pal_req;
end


/////////////////////////  HDMI output  /////////////////////////////////
`ifndef MISTER_DEBUG_NOHDMI
	wire hdmi_clk_out;
	pll_hdmi pll_hdmi
	(
		.refclk(FPGA_CLK1_50),
		.rst(reset_req),
		.reconfig_to_pll(reconfig_to_pll),
		.reconfig_from_pll(reconfig_from_pll),
		.outclk_0(hdmi_clk_out)
	);
`endif

//1920x1080@60 PCLK=148.5MHz CEA
reg  [11:0] WIDTH   = 1920;
reg  [11:0] HFP     = 88;
reg  [12:0] HS      = 48;
reg  [11:0] HBP     = 148;
reg  [11:0] HEIGHT  = 1080;
reg  [11:0] VFP     = 4;
reg  [12:0] VS      = 5;
reg  [11:0] VBP     = 36;
reg         HDMI_PR = 0;

wire [63:0] reconfig_to_pll;
wire [63:0] reconfig_from_pll;
wire        cfg_waitrequest,adj_waitrequest;
wire        cfg_write;
wire  [5:0] cfg_address;
wire [31:0] cfg_data;
reg         adj_write;
reg   [5:0] adj_address;
reg  [31:0] adj_data;

`ifndef MISTER_DEBUG_NOHDMI
	pll_cfg_hdmi pll_cfg_hdmi
	(
		.mgmt_clk(FPGA_CLK1_50),
		.mgmt_reset(reset_req),
		.mgmt_waitrequest(cfg_waitrequest),
		.mgmt_write(cfg_write),
		.mgmt_address(cfg_address),
		.mgmt_writedata(cfg_data),
		.reconfig_to_pll(reconfig_to_pll),
		.reconfig_from_pll(reconfig_from_pll)
	);

	reg cfg_got = 0;
	always @(posedge clk_sys) begin
		reg vsd, vsd2;
		if(~cfg_ready || ~cfg_set) cfg_got <= cfg_set;
		else begin
			vsd  <= HDMI_TX_VS;
			vsd2 <= vsd;
			if(~vsd2 & vsd) cfg_got <= cfg_set;
		end
	end

	reg cfg_ready = 0;
	always @(posedge FPGA_CLK1_50) begin
		reg gotd = 0, gotd2 = 0;
		reg custd = 0, custd2 = 0;
		reg old_wait = 0;

		gotd  <= cfg_got;
		gotd2 <= gotd;
		
		adj_write <= 0;
		
		custd <= cfg_custom_t;
		custd2 <= custd;
		if(custd2 != custd & ~gotd) begin
			adj_address <= cfg_custom_p1;
			adj_data <= cfg_custom_p2;
			adj_write <= 1;
		end

		if(~gotd2 & gotd) begin
			adj_address <= 2;
			adj_data <= 0;
			adj_write <= 1;
		end

		old_wait <= adj_waitrequest;
		if(old_wait & ~adj_waitrequest & gotd) cfg_ready <= 1;
	end
`else
	wire cfg_ready = 1;
`endif

assign HDMI_I2C_SCL = hdmi_scl_en ? 1'b0 : 1'bZ;
assign HDMI_I2C_SDA = hdmi_sda_en ? 1'b0 : 1'bZ;

wire hdmi_scl_en, hdmi_sda_en;
cyclonev_hps_interface_peripheral_i2c hdmi_i2c
(
	.out_clk(hdmi_scl_en),
	.scl(HDMI_I2C_SCL),
	.out_data(hdmi_sda_en),
	.sda(HDMI_I2C_SDA)
);

`ifndef MISTER_DEBUG_NOHDMI
	`ifdef MISTER_FB
	reg dis_output;
	always @(posedge clk_hdmi) begin
		reg dis;
		dis <= fb_force_blank & ~LFB_EN;
		dis_output <= dis;
	end
	`else
	wire dis_output = 0;
	`endif

	wire [23:0] hdmi_data_mask;
	wire        hdmi_de_mask, hdmi_vs_mask, hdmi_hs_mask;

	reg [15:0] shadowmask_data;
	reg        shadowmask_wr = 0;

	shadowmask HDMI_shadowmask
	(
		.clk(clk_hdmi),
		.clk_sys(clk_sys),

		.cmd_wr(shadowmask_wr),
		.cmd_in(shadowmask_data),

		.din(dis_output ? 24'd0 : hdmi_data),
		.hs_in(hdmi_hs),
		.vs_in(hdmi_vs),
		.de_in(hdmi_de),
		.brd_in(hdmi_brd),
		.enable(~LFB_EN),

		.dout(hdmi_data_mask),
		.hs_out(hdmi_hs_mask),
		.vs_out(hdmi_vs_mask),
		.de_out(hdmi_de_mask)
	);

	wire [23:0] hdmi_data_osd;
	wire        hdmi_de_osd, hdmi_vs_osd, hdmi_hs_osd;

	osd hdmi_osd
	(
		.clk_sys(clk_sys),

		.io_osd(io_osd_hdmi),
		.io_strobe(io_strobe),
		.io_din(io_din),

		.clk_video(clk_hdmi),
		.din(hdmi_data_mask),
		.hs_in(hdmi_hs_mask),
		.vs_in(hdmi_vs_mask),
		.de_in(hdmi_de_mask),

		.dout(hdmi_data_osd),
		.hs_out(hdmi_hs_osd),
		.vs_out(hdmi_vs_osd),
		.de_out(hdmi_de_osd)
`ifdef OSD_WPORTS
		,
		.lw_we(osd_we),
		.lw_addr(osd_addr),
		.lw_byte(osd_byte)
`endif
	);

	wire hdmi_cs_osd;
	csync csync_hdmi(clk_hdmi, hdmi_hs_osd, hdmi_vs_osd, hdmi_cs_osd);
`endif

reg [23:0] dv_data;
reg        dv_hs, dv_vs, dv_de;
wire [23:0] dv_data_osd;
wire dv_hs_osd, dv_vs_osd, dv_cs_osd;

always @(posedge clk_vid) begin
	reg [23:0] dv_d1, dv_d2;
	reg        dv_de1, dv_de2, dv_hs1, dv_hs2, dv_vs1, dv_vs2;
	reg [12:0] vsz, vcnt, vcnt_l, vcnt_ll;
	reg        old_hs, old_vs;
	reg        vde;
	reg  [3:0] hss;

	if(ce_pix) begin
		hss <= (hss << 1) | dv_hs_osd;

		old_hs <= dv_hs_osd;
		if(~old_hs && dv_hs_osd) begin
			old_vs <= dv_vs_osd;
			if(~&vcnt) vcnt <= vcnt + 1'd1;
			if(~old_vs & dv_vs_osd) begin
				if (vcnt != vcnt_ll || vcnt < vcnt_l) vsz <= vcnt;
				vcnt_l <= vcnt;
				vcnt_ll <= vcnt_l;
			end
			if(old_vs & ~dv_vs_osd) vcnt <= 0;
			
			if(vcnt == 1) vde <= 1;
			if(vcnt == vsz - 3) vde <= 0;
		end

		dv_de1 <= !{hss,dv_hs_osd} && vde;
	end

	dv_d1  <= dv_data_osd;
	dv_hs1 <= csync_en ? dv_cs_osd : dv_hs_osd;
	dv_vs1 <= dv_vs_osd;

	dv_d2  <= dv_d1;
	dv_de2 <= dv_de1;
	dv_hs2 <= dv_hs1;
	dv_vs2 <= dv_vs1;

	dv_data<= dv_d2;
	dv_de  <= dv_de2;
	dv_hs  <= dv_hs2;
	dv_vs  <= dv_vs2;
end

`ifndef MISTER_DISABLE_YC
	assign {dv_data_osd, dv_hs_osd, dv_vs_osd, dv_cs_osd } = ~yc_en ? {vga_data_osd, vga_hs_osd, vga_vs_osd, vga_cs_osd } : {yc_o, yc_hs, yc_vs, yc_cs };
`else
	assign {dv_data_osd, dv_hs_osd, dv_vs_osd, dv_cs_osd } = {vga_data_osd, vga_hs_osd, vga_vs_osd, vga_cs_osd };
`endif

wire hdmi_tx_clk;
`ifndef MISTER_DEBUG_NOHDMI
	cyclonev_clkselect hdmi_clk_sw
	( 
		.clkselect({1'b1, ~vga_fb & direct_video}),
		.inclk({clk_vid, hdmi_clk_out, 2'b00}),
		.outclk(hdmi_tx_clk)
	);
`else
	assign hdmi_tx_clk = clk_vid;
`endif

altddio_out
#(
	.extend_oe_disable("OFF"),
	.intended_device_family("Cyclone V"),
	.invert_output("OFF"),
	.lpm_hint("UNUSED"),
	.lpm_type("altddio_out"),
	.oe_reg("UNREGISTERED"),
	.power_up_high("OFF"),
	.width(1)
)
hdmiclk_ddr
(
	.datain_h(1'b0),
	.datain_l(1'b1),
	.outclock(hdmi_tx_clk),
	.dataout(HDMI_TX_CLK),
	.aclr(1'b0),
	.aset(1'b0),
	.oe(1'b1),
	.outclocken(1'b1),
	.sclr(1'b0),
	.sset(1'b0)
);

reg hdmi_out_hs;
reg hdmi_out_vs;
reg hdmi_out_de;
reg [23:0] hdmi_out_d;

always @(posedge hdmi_tx_clk) begin
	reg [23:0] hdmi_dv_data;
	reg        hdmi_dv_hs, hdmi_dv_vs, hdmi_dv_de;

	reg hs,vs,de;
	reg [23:0] d;
	
	hdmi_dv_data <= dv_data;
	hdmi_dv_hs   <= dv_hs;
	hdmi_dv_vs   <= dv_vs;
	hdmi_dv_de   <= dv_de;
	
`ifndef MISTER_DEBUG_NOHDMI
	hs <= (~vga_fb & direct_video) ? hdmi_dv_hs   : (direct_video & csync_en) ? hdmi_cs_osd : hdmi_hs_osd;
	vs <= (~vga_fb & direct_video) ? hdmi_dv_vs   : hdmi_vs_osd;
	de <= (~vga_fb & direct_video) ? hdmi_dv_de   : hdmi_de_osd;
	d  <= (~vga_fb & direct_video) ? hdmi_dv_data : hdmi_data_osd;
`else
	hs <= hdmi_dv_hs;
	vs <= hdmi_dv_vs;
	de <= hdmi_dv_de;
	d  <= hdmi_dv_data;
`endif

	hdmi_out_hs <= hs;
	hdmi_out_vs <= vs;
	hdmi_out_de <= de;
	hdmi_out_d  <= d;
end

assign HDMI_TX_HS = hdmi_out_hs;
assign HDMI_TX_VS = hdmi_out_vs;
assign HDMI_TX_DE = hdmi_out_de;
assign HDMI_TX_D  = hdmi_out_d;

/////////////////////////  VGA output  //////////////////////////////////

`ifndef MISTER_DUAL_SDRAM
	wire vga_tx_clk;
	`ifndef MISTER_DEBUG_NOHDMI
		cyclonev_clkselect vga_clk_sw
		( 
		`ifdef MISTER_DISABLE_VGA_SCALER
			.clkselect({1'b1, ~vgas_en}),
		`else
			.clkselect({1'b1, ~vga_fb & ~vga_scaler}),
		`endif
			.inclk({clk_vid, hdmi_clk_out, 2'b00}),
			.outclk(vga_tx_clk)
		);
	`else
		assign vga_tx_clk = clk_vid;
	`endif

	wire VGA_TX_CLK;
	altddio_out
	#(
		.extend_oe_disable("OFF"),
		.intended_device_family("Cyclone V"),
		.invert_output("OFF"),
		.lpm_hint("UNUSED"),
		.lpm_type("altddio_out"),
		.oe_reg("UNREGISTERED"),
		.power_up_high("OFF"),
		.width(1)
	)
	vgaclk_ddr
	(
		.datain_h(1'b0),
		.datain_l(1'b1),
		.outclock(vga_tx_clk),
		.dataout(VGA_TX_CLK),
		.aclr(~mcp_en & ~av_dis),
		.aset(1'b0),
		.oe(~av_dis & (mcp_en | ~led_u)),
		.outclocken(1'b1),
		.sclr(1'b0),
		.sset(1'b0)
	);
`endif

wire [23:0] vga_data_sl;
wire        vga_de_sl, vga_ce_sl, vga_vs_sl, vga_hs_sl;
scanlines #(0) VGA_scanlines
(
	.clk(clk_vid),

	.scanlines(scanlines),
	.din(de_emu ? {r_out, g_out, b_out} : 24'd0),
	.hs_in(hs_fix),
	.vs_in(vs_fix),
	.de_in(de_emu),
	.ce_in(ce_pix),

	.dout(vga_data_sl),
	.hs_out(vga_hs_sl),
	.vs_out(vga_vs_sl),
	.de_out(vga_de_sl),
	.ce_out(vga_ce_sl)
);

wire [23:0] vga_data_osd;
wire        vga_vs_osd, vga_hs_osd, vga_de_osd;
osd vga_osd
(
	.clk_sys(clk_sys),

	.io_osd(io_osd_vga),
	.io_strobe(io_strobe),
	.io_din(io_din),
	.osd_status(osd_status),

	.clk_video(clk_vid),
	.din(vga_data_sl),
	.hs_in(vga_hs_sl),
	.vs_in(vga_vs_sl),
	.de_in(vga_de_sl),

	.dout(vga_data_osd),
	.hs_out(vga_hs_osd),
	.vs_out(vga_vs_osd),
	.de_out(vga_de_osd)
`ifdef OSD_WPORTS
	,
	.lw_we(osd_we),
	.lw_addr(osd_addr),
	.lw_byte(osd_byte)
`endif
);

wire vga_cs_osd;
csync csync_vga(clk_vid, vga_hs_osd, vga_vs_osd, vga_cs_osd);

`ifndef MISTER_DISABLE_YC
	reg         pal_en;
	reg         yc_en;
	reg         cvbs;
	reg  [16:0] ColorBurst_Range;
	wire [23:0] yc_o;
	wire        yc_hs, yc_vs, yc_cs, yc_de;

	yc_out yc_out
	(
		.clk(clk_vid),
		.PAL_EN(pal_en),
		.CVBS(cvbs),
		.PHASE_INC(PhaseInc),
		.COLORBURST_RANGE(ColorBurst_Range),
		.hsync(vga_hs_osd),
		.vsync(vga_vs_osd),
		.csync(vga_cs_osd),
		.de(vga_de_osd),
		.dout(yc_o),
		.din(vga_data_osd),
		.hsync_o(yc_hs),
		.vsync_o(yc_vs),
		.csync_o(yc_cs),
		.de_o(yc_de)
	);
`endif

reg  [39:0] PhaseInc;

`ifndef MISTER_DUAL_SDRAM
	// Subcarrier generation for external encoders (independent of YC module)
	reg         subcarrier;

	reg  [39:0] sub_accum;
	always @(posedge clk_vid) sub_accum <= sub_accum + PhaseInc;

	// 1-bit output for positive/negative of wave, no LUT required. Output 1 if disabled for further logic
	reg subcarrier_out;
	always @(posedge clk_vid) subcarrier_out <= ~(subcarrier & csync_en & ~ypbpr_en & ~forced_scandoubler & ~vgas_en) | sub_accum[39];


	wire VGA_DISABLE;
	wire [23:0] vgas_o;
	wire vgas_hs, vgas_vs, vgas_cs, vgas_de;
	`ifdef MISTER_DISABLE_VGA_SCALER
		assign {vgas_o, vgas_hs, vgas_vs, vgas_cs, vgas_de} = 0;
	`elsif MISTER_DEBUG_NOHDMI
		assign {vgas_o, vgas_hs, vgas_vs, vgas_cs, vgas_de} = 0;
	`else
		vga_out vga_scaler_out
		(
			.clk(clk_hdmi),
			.ypbpr_en(ypbpr_en),
			.hsync(hdmi_hs_osd),
			.vsync(hdmi_vs_osd),
			.csync(hdmi_cs_osd),
			.de(hdmi_de_osd),
			.dout(vgas_o),
			.din({24{hdmi_de_osd}} & hdmi_data_osd),
			.hsync_o(vgas_hs),
			.vsync_o(vgas_vs),
			.csync_o(vgas_cs),
			.de_o(vgas_de)
		);
	`endif

	wire [23:0] vga_o, vga_o_t;
	wire vga_hs, vga_vs, vga_cs, vga_de, vga_hs_t, vga_vs_t, vga_cs_t, vga_de_t;
	vga_out vga_out
	(
		.clk(clk_vid),
		.ypbpr_en(ypbpr_en),
		.hsync(vga_hs_osd),
		.vsync(vga_vs_osd),
		.csync(vga_cs_osd),
		.de(vga_de_osd),
		.dout(vga_o_t),
		.din(vga_data_osd),
		.hsync_o(vga_hs_t),
		.vsync_o(vga_vs_t),
		.csync_o(vga_cs_t),
		.de_o(vga_de_t)
	);

	`ifndef MISTER_DISABLE_YC
		assign {vga_o, vga_hs, vga_vs, vga_cs, vga_de } = ~yc_en ? {vga_o_t, vga_hs_t, vga_vs_t, vga_cs_t, vga_de_t } : {yc_o, yc_hs, yc_vs, yc_cs, yc_de };
	`else
		assign {vga_o, vga_hs, vga_vs, vga_cs, vga_de } =  {vga_o_t, vga_hs_t, vga_vs_t, vga_cs_t, vga_de_t } ;
	`endif

	`ifndef MISTER_DISABLE_VGA_SCALER
	wire vgas_en = vga_fb | vga_scaler;
	`endif

	wire cs1 = vgas_en ? vgas_cs : vga_cs;
	wire de1 = vgas_en ? vgas_de : vga_de;

	assign VGA_VS = av_dis ? 1'bZ      :(((vgas_en ? (~vgas_vs ^ VS[12])                         : VGA_DISABLE ? 1'd1 : ~vga_vs) | csync_en) & subcarrier_out);
	assign VGA_HS = av_dis ? 1'bZ      :  (vgas_en ? ((csync_en ? ~vgas_cs : ~vgas_hs) ^ HS[12]) : VGA_DISABLE ? 1'd1 : (csync_en ? ~vga_cs : ~vga_hs));
	assign VGA_R  = av_dis ? 6'bZZZZZZ :   vgas_en ? vgas_o[23:18]                               : VGA_DISABLE ? 6'd0 : vga_o[23:18];
	assign VGA_G  = av_dis ? 6'bZZZZZZ :   vgas_en ? vgas_o[15:10]                               : VGA_DISABLE ? 6'd0 : vga_o[15:10];
	assign VGA_B  = av_dis ? 6'bZZZZZZ :   vgas_en ? vgas_o[7:2]                                 : VGA_DISABLE ? 6'd0 : vga_o[7:2]  ;

	wire [1:0] vga_r  = vgas_en ? vgas_o[17:16] : VGA_DISABLE ? 2'd0 : vga_o[17:16];
	wire [1:0] vga_g  = vgas_en ? vgas_o[9:8]   : VGA_DISABLE ? 2'd0 : vga_o[9:8];
	wire [1:0] vga_b  = vgas_en ? vgas_o[1:0]   : VGA_DISABLE ? 2'd0 : vga_o[1:0];
`endif

reg video_sync = 0;
always @(posedge clk_vid) begin
	reg [11:0] line_cnt  = 0;
	reg [11:0] sync_line = 0;
	reg  [1:0] hs_cnt = 0;
	reg        old_hs;

	old_hs <= hs_fix;
	if(~old_hs & hs_fix) begin

		video_sync <= (sync_line == line_cnt);

		line_cnt <= line_cnt + 1'd1;
		if(~hs_cnt[1]) begin	
			hs_cnt <= hs_cnt + 1'd1;
			if(hs_cnt[0]) begin
				sync_line <= (line_cnt - vs_line);
				line_cnt <= 0;
			end
		end
	end

	if(de_emu) hs_cnt <= 0;
end

/////////////////////////  Audio output  ////////////////////////////////

assign SDCD_SPDIF = (mcp_en & ~spdif) ? 1'b0 : 1'bZ;

`ifndef MISTER_DUAL_SDRAM
	wire analog_l, analog_r;

`endif

assign HDMI_MCLK = clk_audio;
wire clk_audio;

pll_audio pll_audio
(
	.refclk(FPGA_CLK3_50),
	.rst(0),
	.outclk_0(clk_audio)
);

wire spdif;
audio_out audio_out
(
	.reset(reset | areset),
	.clk(clk_audio),

	.att(vol_att),
	.boost(vol_boost),
	.mix(audio_mix),
	.sample_rate(audio_96k),

	.flt_rate(aflt_rate),
	.cx(acx),
	.cx0(acx0),
	.cx1(acx1),
	.cx2(acx2),
	.cy0(acy0),
	.cy1(acy1),
	.cy2(acy2),

	.is_signed(audio_s),
	.core_l(audio_l),
	.core_r(audio_r),

`ifndef MISTER_DISABLE_ALSA
	.alsa_l(alsa_l),
	.alsa_r(alsa_r),
`endif

	.i2s_bclk(HDMI_SCLK),
	.i2s_lrclk(HDMI_LRCLK),
	.i2s_data(HDMI_I2S),
`ifndef MISTER_DUAL_SDRAM
	.dac_l(analog_l),
	.dac_r(analog_r),
`endif
	.spdif(spdif)
);

`ifndef MISTER_DUAL_SDRAM
	reg aud_spdif_q, aud_r_q, aud_l_q;
	always @(posedge clk_audio) begin
		aud_spdif_q <= (SW[0] | mcp_en) ? HDMI_LRCLK : spdif;
		aud_r_q     <= (SW[0] | mcp_en) ? HDMI_I2S   : analog_r;
		aud_l_q     <= (SW[0] | mcp_en) ? HDMI_SCLK  : analog_l;
	end

	assign AUDIO_SPDIF = av_dis ? 1'bZ : aud_spdif_q;
	assign AUDIO_R     = av_dis ? 1'bZ : aud_r_q;
	assign AUDIO_L     = av_dis ? 1'bZ : aud_l_q;
`endif

`ifndef MISTER_DISABLE_ALSA
	wire aspi_sck,aspi_mosi,aspi_ss,aspi_miso;
	cyclonev_hps_interface_peripheral_spi_master spi
	(
		.sclk_out(aspi_sck),
		.txd(aspi_mosi), // mosi
		.rxd(aspi_miso), // miso

		.ss_0_n(aspi_ss),
		.ss_in_n(1)
	);

	wire [28:0] alsa_address;
	wire [63:0] alsa_readdata;
	wire        alsa_ready;
	wire        alsa_req;
	wire        alsa_late;

	wire [15:0] alsa_l, alsa_r;

	alsa alsa
	(
		.reset(reset),
		.clk(clk_audio),

		.ram_address(alsa_address),
		.ram_data(alsa_readdata),
		.ram_req(alsa_req),
		.ram_ready(alsa_ready),

		.spi_ss(aspi_ss),
		.spi_sck(aspi_sck),
		.spi_mosi(aspi_mosi),
		.spi_miso(aspi_miso),

		.pcm_l(alsa_l),
		.pcm_r(alsa_r)
	);
`endif

////////////////  User I/O (USB 3.0 connector) /////////////////////////

assign USER_IO[0] =                       !user_out[0]  ? 1'b0 : 1'bZ;
assign USER_IO[1] =                       !user_out[1]  ? 1'b0 : 1'bZ;
assign USER_IO[2] = !(SW[1] ? HDMI_I2S   : user_out[2]) ? 1'b0 : 1'bZ;
assign USER_IO[3] =                       !user_out[3]  ? 1'b0 : 1'bZ;
assign USER_IO[4] = !(SW[1] ? HDMI_SCLK  : user_out[4]) ? 1'b0 : 1'bZ;
assign USER_IO[5] = !(SW[1] ? HDMI_LRCLK : user_out[5]) ? 1'b0 : 1'bZ;
assign USER_IO[6] =                       !user_out[6]  ? 1'b0 : 1'bZ;

assign user_in[0] =         USER_IO[0];
assign user_in[1] =         USER_IO[1];
assign user_in[2] = SW[1] | USER_IO[2];
assign user_in[3] =         USER_IO[3];
assign user_in[4] = SW[1] | USER_IO[4];
assign user_in[5] = SW[1] | USER_IO[5];
assign user_in[6] =         USER_IO[6];


///////////////////  User module connection ////////////////////////////

wire        clk_sys;
`ifdef OCRAM_RX_PORTS
// The host-to-core rings' consumer (hps_ocram_rx.sv): ROW records into every osd's osd_buffer, and with the SECTOR
// tier the FRAME records into one hps_io slot through the window's xsec nets.
`ifdef MENU_CORE
localparam integer OCRAM_OSD_ROWS = 20;
`else
localparam integer OCRAM_OSD_ROWS = 16;
`endif
`ifdef MISTER_OCRAM_OSD
`ifdef MISTER_DEBUG_NOHDMI
wire [1:0] ocram_osd_sel = {io_osd_vga, 1'b0};       // no HDMI osd
`else
wire [1:0] ocram_osd_sel = {io_osd_vga, io_osd_hdmi};
`endif
`endif
`ifdef MISTER_OCRAM_OSD
localparam        OCRAM_OSD_EN = 1;
`else
localparam        OCRAM_OSD_EN = 0;
`endif
hps_ocram_rx #(.ROWS(OCRAM_OSD_ROWS), .OSD_EN(OCRAM_OSD_EN), .SEC_EN(OCRAM_SEC_EN), .FRAME(OCRAM_SEC_FRAME),
               .GAP(OCRAM_SEC_GAP)) ocram_rx
(
	.clk(clk_100m),
	.clk_sys(clk_sys),
	.gp_out25(gp_out[25]),

	.s_hello(ocram_s_hello),
	.s_live(ocram_s_live),
	.s_tick(ocram_s_tick),
	.bind_en(ocram_osd_bind),
	.sec_ask(ocram_sec_ask),
	.evt_stop(ocram_evt_stop),
	.c_sfault(ocram_c0_sfault),
	.c_smid(ocram_c0_smid),
	.c_sbind(ocram_c0_sbind),
	.c_wide(ocram_c0_wide),
	.rq_post(ocram_rq_post),
	.rq_lba(ocram_rq_lba),
	.rq_start(ocram_rq_start),
	.c_rest(ocram_c0_rest),
	.c_fault(ocram_c0_fault),
	.c_mid(ocram_c0_mid),
	.c_open(ocram_c0_open),
	.osd_rows(ocram_osd_rows),
	.osd_deepest(ocram_osd_deepest),

	.c_req(ocram_c0_req),
	.c_we(ocram_c0_we),
	.c_off(ocram_c0_off),
	.c_wdata(ocram_c0_wdata),
	.c_grant(ocram_c0_grant),
	.c_done(ocram_c0_done),
	.c_ok(ocram_c0_ok),
	.c_rdata(ocram_c0_rdata),

`ifdef MISTER_OCRAM_OSD
	.sel(ocram_osd_sel),
	.view(gp_outr[31]),
	.we(ocram_osd_we),
	.addr(ocram_osd_addr),
	.dbyte(ocram_osd_byte),
`else
	.sel(2'b00),
	.view(1'b0),
	.we(),
	.addr(),
	.dbyte(),
`endif
`ifdef MISTER_OCRAM_SECTOR_SLOT
	.x_reset(xsec_reset),
	.x_hps_ack(xsec_hps_ack),
	.x_req(xsec_req),
	.x_lba(xsec_lba),
	.x_commit(xsec_commit),
	.x_wide(io_wide),
	.x_en(xsec_en),
	.x_ack(xsec_ack),
	.x_wr(xsec_wr),
	.x_dat(xsec_dat),
	.x_adr(xsec_adr)
`else
	.x_reset(1'b0),
	.x_hps_ack(1'b0),
	.x_req(1'b0),
	.x_lba(32'd0),
	.x_commit(1'b0),
	.x_wide(1'b0),
	.x_en(),
	.x_ack(),
	.x_wr(),
	.x_dat(),
	.x_adr()
`endif
);
`endif
wire [15:0] audio_l, audio_r;
wire        audio_s;
wire  [1:0] audio_mix;
wire  [1:0] scanlines;
wire  [7:0] r_out, g_out, b_out, hr_out, hg_out, hb_out;
wire        vs_fix, hs_fix, de_emu, vs_emu, hs_emu, f1;
wire        hvs_fix, hhs_fix, hde_emu;
wire        clk_vid, ce_pix, clk_ihdmi, ce_hpix;
wire        vga_force_scaler;

wire        ram_clk;
wire [28:0] ram_address;
wire [7:0]  ram_burstcount;
wire        ram_waitrequest;
wire [63:0] ram_readdata;
wire        ram_readdatavalid;
wire        ram_read;
wire [63:0] ram_writedata;
wire [7:0]  ram_byteenable;
wire        ram_write;

wire        led_user;
wire  [1:0] led_power;
wire  [1:0] led_disk;
wire  [1:0] btn;

sync_fix sync_v(clk_vid, vs_emu, vs_fix);
sync_fix sync_h(clk_vid, hs_emu, hs_fix);

wire  [6:0] user_out, user_in;

assign clk_ihdmi= clk_vid;
assign ce_hpix  = vga_ce_sl;
assign hr_out   = vga_data_sl[23:16];
assign hg_out   = vga_data_sl[15:8];
assign hb_out   = vga_data_sl[7:0];
assign hhs_fix  = vga_hs_sl;
assign hvs_fix  = vga_vs_sl;
assign hde_emu  = vga_de_sl;

wire uart_dtr;
wire uart_dsr;
wire uart_cts;
wire uart_rts;
wire uart_rxd;
wire uart_txd;

wire osd_status;

wire        fb_en;
wire  [4:0] fb_fmt;
wire [11:0] fb_width;
wire [11:0] fb_height;
wire [31:0] fb_base;
wire [13:0] fb_stride;


`ifdef MISTER_FB
	`ifdef MISTER_FB_PALETTE
		wire        fb_pal_clk;
		wire  [7:0] fb_pal_a;
		wire [23:0] fb_pal_d;
		wire [23:0] fb_pal_q;
		wire        fb_pal_wr;
	`endif
	wire   fb_force_blank;
`else
	assign fb_en = 0;
	assign fb_fmt = 0;
	assign fb_width = 0;
	assign fb_height = 0;
	assign fb_base = 0;
	assign fb_stride = 0;
`endif

////////////////////  LW BRIDGE  ///////////////////////////////////
// MISTER_LW: lwhps2fpga (0xFF200000, 2 MB) answers every offset (0xDEADBEEF default); the `SYS_LW_CAPS
// window holds the directory, and each tier the core builds adds its block: PAD with MISTER_LW_PAD, EVT with
// MISTER_LW_EVT, SECTOR when the core declares it (MISTER_LW_SECTOR_SLOT), the OSDBUF with MISTER_LW_OSD and
// the frame lane with MISTER_LW_LANE. Without MISTER_LW none of this is built.
// HPS_BUS[383:46] is one vector, lw_bus, whatever the tiers; lw_sys_window owns its bit map.
// Register map and bit map: docs/LW_BRIDGE.md.
`ifdef MISTER_LW
wire [20:0] lw_raddr, lw_waddr;
wire        lw_write;
wire [31:0] lw_rdata, lw_wdata;
wire  [3:0] lw_be;
wire [383:46] lw_bus;
wire        lw_wburst, lw_hrst, lw_idv;
wire  [1:0] lw_freed;

// GPO[31] = 0, the ID view or the core reset through GPO, as a clk_100m level: it clears the LW host enable and refuses it
lw_arst lw_idv_arst
(
	.clk(clk_100m),
	.rst_n(gp_out[31]),
	.q(lw_idv)
);

// The tiers the core builds (its .qsf macros).
`ifdef MISTER_LW_PAD
localparam        LW_PAD_EN    = 1;
`else
localparam        LW_PAD_EN    = 0;
`endif
`ifdef MISTER_LW_EVT
localparam        LW_EVT_EN    = 1;
`else
localparam        LW_EVT_EN    = 0;
`endif
`ifdef LW_OSD_PORTS
localparam        LW_OSD_EN    = 1;
`else
localparam        LW_OSD_EN    = 0;
`endif

// The OCRAM SECTOR feeder drives the primary slot's bits through the window (MISTER_OCRAM_SECTOR_SLOT).
`ifdef MISTER_OCRAM
`ifdef MISTER_OCRAM_SECTOR_SLOT
`define LW_SEC_XNETS
`endif
`endif
`ifdef LW_SEC_XNETS
localparam        LW_SEC_EXT   = 1;
`else
localparam        LW_SEC_EXT   = 0;
`endif

// The core's SECTOR declaration (its .qsf macros); every one is optional. A save slot is declared by its _SLOT
// macro and needs its _WSRC; lw_sys_window stops the build on an unservable geometry.
`ifdef MISTER_LW_SECTOR_SLOT
localparam        LW_SEC_EN    = 1;
localparam [31:0] LW_SEC_SLOT  = `MISTER_LW_SECTOR_SLOT;
`else
localparam        LW_SEC_EN    = 0;
localparam [31:0] LW_SEC_SLOT  = 32'd1;
`endif
`ifdef MISTER_LW_SECTOR_FRAME
localparam [31:0] LW_SEC_FRAME = `MISTER_LW_SECTOR_FRAME;
`else
localparam [31:0] LW_SEC_FRAME = 32'd128;
`endif
`ifdef MISTER_LW_SECTOR_GAP
localparam [31:0] LW_SEC_GAP   = `MISTER_LW_SECTOR_GAP;
`else
localparam [31:0] LW_SEC_GAP   = 32'd2;
`endif
`ifdef MISTER_LW_BLKSZ
localparam  [2:0] LW_BLKSZ     = `MISTER_LW_BLKSZ;
`else
localparam  [2:0] LW_BLKSZ     = 3'd2;
`endif
`ifdef MISTER_LW_SAVE0_SLOT
localparam        LW_SAVE0_EN    = 1;
localparam [31:0] LW_SAVE0_SLOT  = `MISTER_LW_SAVE0_SLOT;
`else
localparam        LW_SAVE0_EN    = 0;
localparam [31:0] LW_SAVE0_SLOT  = 32'hE;
`endif
`ifdef MISTER_LW_SAVE0_WSRC
localparam [31:0] LW_SAVE0_WSRC  = `MISTER_LW_SAVE0_WSRC;
`else
localparam [31:0] LW_SAVE0_WSRC  = 32'd0;
`endif
`ifdef MISTER_LW_SAVE0_EXTENT
localparam [31:0] LW_SAVE0_EXT   = `MISTER_LW_SAVE0_EXTENT;
`else
localparam [31:0] LW_SAVE0_EXT   = 32'd0;
`endif
`ifdef MISTER_LW_SAVE0_FRAME
localparam [31:0] LW_SAVE0_FRAME = `MISTER_LW_SAVE0_FRAME;
`else
localparam [31:0] LW_SAVE0_FRAME = 32'd256;
`endif
`ifdef MISTER_LW_SAVE0_LBA_BITS
localparam [31:0] LW_SAVE0_LBAB  = `MISTER_LW_SAVE0_LBA_BITS;
`else
localparam [31:0] LW_SAVE0_LBAB  = 32'd32;
`endif
`ifdef MISTER_LW_SAVE1_SLOT
localparam        LW_SAVE1_EN    = 1;
localparam [31:0] LW_SAVE1_SLOT  = `MISTER_LW_SAVE1_SLOT;
`else
localparam        LW_SAVE1_EN    = 0;
localparam [31:0] LW_SAVE1_SLOT  = 32'hF;
`endif
`ifdef MISTER_LW_SAVE1_WSRC
localparam [31:0] LW_SAVE1_WSRC  = `MISTER_LW_SAVE1_WSRC;
`else
localparam [31:0] LW_SAVE1_WSRC  = 32'd0;
`endif
`ifdef MISTER_LW_SAVE1_EXTENT
localparam [31:0] LW_SAVE1_EXT   = `MISTER_LW_SAVE1_EXTENT;
`else
localparam [31:0] LW_SAVE1_EXT   = 32'd0;
`endif
`ifdef MISTER_LW_SAVE1_FRAME
localparam [31:0] LW_SAVE1_FRAME = `MISTER_LW_SAVE1_FRAME;
`else
localparam [31:0] LW_SAVE1_FRAME = 32'd256;
`endif
`ifdef MISTER_LW_SAVE1_LBA_BITS
localparam [31:0] LW_SAVE1_LBAB  = `MISTER_LW_SAVE1_LBA_BITS;
`else
localparam [31:0] LW_SAVE1_LBAB  = 32'd32;
`endif

`ifdef LW_LANE_LINK
localparam        LW_LANE_EN   = 1;
wire        lane_own, lane_hold, lane_strobe;
wire  [3:0] lane_sel;
wire [15:0] lane_din;
wire  [3:0] link_sel = {io_ss1 & ~io_ss2, io_ss1 & ~io_ss0, ~io_ss1 & io_ss0, ~io_ss1 & io_ss2};

lw_link lw_link
(
	.clk_sys(clk_sys),
	.l_sel(link_sel),
	.l_clk(io_clk),
	.l_din(gp_outr[15:0]),
	.l_wait(io_wait | vs_wait),
	.p_own(lane_own),
	.p_hold(lane_hold),
	.p_sel(lane_sel),
	.p_strobe(lane_strobe),
	.p_din(lane_din),
	.io_sel(io_sel),
	.io_strobe(io_strobe),
	.io_din(io_din),
	.io_ack(io_ack)
);
`else
localparam        LW_LANE_EN   = 0;
`endif

hps_lw_bridge hps_lw_bridge
(
	.clk(clk_100m),
	.hps_rst_n(lw_hps_rst_n),
	.lw_hrst(lw_hrst),
	.lw_freed(lw_freed),
	.present(lw_present),
	.idle(lw_idle),
	.lw_raddr(lw_raddr),
	.lw_rdata(lw_rdata),
	.lw_waddr(lw_waddr),
	.lw_write(lw_write),
	.lw_wdata(lw_wdata),
	.lw_be(lw_be),
	.lw_wburst(lw_wburst)
);

lw_sys_window #(
	.PAD_EN(LW_PAD_EN), .EVT_EN(LW_EVT_EN), .OSD_EN(LW_OSD_EN),
	.SEC_EN(LW_SEC_EN), .SEC_SLOT(LW_SEC_SLOT), .SEC_FRAME(LW_SEC_FRAME), .SEC_GAP(LW_SEC_GAP), .BLKSZ(LW_BLKSZ),
	.SAVE0_EN(LW_SAVE0_EN), .SAVE0_SLOT(LW_SAVE0_SLOT), .SAVE0_WSRC(LW_SAVE0_WSRC), .SAVE0_EXT(LW_SAVE0_EXT),
	.SAVE0_FRAME(LW_SAVE0_FRAME), .SAVE0_LBA_BITS(LW_SAVE0_LBAB),
	.SAVE1_EN(LW_SAVE1_EN), .SAVE1_SLOT(LW_SAVE1_SLOT), .SAVE1_WSRC(LW_SAVE1_WSRC), .SAVE1_EXT(LW_SAVE1_EXT),
	.SAVE1_FRAME(LW_SAVE1_FRAME), .SAVE1_LBA_BITS(LW_SAVE1_LBAB),
	.LANE_EN(LW_LANE_EN),
	.SEC_EXT(LW_SEC_EXT)
) lw_sys_window
(
	.clk(clk_100m),
	.lw_raddr(lw_raddr),
	.lw_rdata(lw_rdata),
	.lw_waddr(lw_waddr),
	.lw_write(lw_write),
	.lw_wdata(lw_wdata),
	.lw_be(lw_be),
	.lw_wburst(lw_wburst),
	.lw_hrst(lw_hrst),
	.lw_freed(lw_freed),
	.lw_idv(lw_idv),
	.held(lw_held),
	.lines(lw_lines),
	.clk_sys(clk_sys),
	.hps_lw(lw_bus),
	.io_wide(io_wide),
`ifdef LW_OSD_PORTS
`ifdef MISTER_DEBUG_NOHDMI
	.osd_sel({io_osd_vga, 1'b0}),            // no HDMI osd
`else
	.osd_sel({io_osd_vga, io_osd_hdmi}),
`endif
	.osd_view(gp_outr[31]),
	.osd_we(lw_osd_we),
	.osd_addr(lw_osd_addr),
	.osd_byte(lw_osd_byte),
`else
	.osd_sel(2'b00),
	.osd_view(1'b0),
	.osd_we(),
	.osd_addr(),
	.osd_byte(),
`endif
`ifdef LW_LANE_LINK
	.lane_busy(|link_sel | ~gp_outr[31]),   // a select, or the ID view: no lane frame starts
	.lane_wait(io_wait | vs_wait),
	.lane_dout(io_dout | io_dout_sys),
	.lane_own(lane_own),
	.lane_hold(lane_hold),
	.lane_sel(lane_sel),
	.lane_strobe(lane_strobe),
	.lane_din(lane_din),
`else
	.lane_busy(1'b0),
	.lane_wait(1'b0),
	.lane_dout(16'd0),
	.lane_own(),
	.lane_hold(),
	.lane_sel(),
	.lane_strobe(),
	.lane_din(),
`endif
`ifdef LW_SEC_XNETS
	.xsec_en(xsec_en),
	.xsec_ack(xsec_ack),
	.xsec_wr(xsec_wr),
	.xsec_dat(xsec_dat),
	.xsec_adr(xsec_adr),
	.xsec_reset(xsec_reset),
	.xsec_hps_ack(xsec_hps_ack),
	.xsec_req(xsec_req),
	.xsec_lba(xsec_lba),
	.xsec_commit(xsec_commit)
`else
	.xsec_en(1'b0),
	.xsec_ack(1'b0),
	.xsec_wr(1'b0),
	.xsec_dat(16'd0),
	.xsec_adr(14'd0),
	.xsec_reset(),
	.xsec_hps_ack(),
	.xsec_req(),
	.xsec_lba(),
	.xsec_commit()
`endif
);
`ifdef LW_SEC_XNETS
`undef LW_SEC_XNETS
`endif
`ifdef LW_LANE_LINK
`undef LW_LANE_LINK
`endif
`ifdef LW_OSD_PORTS
`undef LW_OSD_PORTS
`endif
`endif
`ifdef OSD_WPORTS
`undef OSD_WPORTS
`endif
`ifdef OCRAM_OSD_PORTS
`undef OCRAM_OSD_PORTS
`endif
`ifdef OCRAM_RX_PORTS
`undef OCRAM_RX_PORTS
`endif

emu emu
(
	.CLK_50M(FPGA_CLK2_50),
	.RESET(reset),
`ifdef MISTER_LW
	// HPS_BUS[383:46]: the LW window's clk_sys lines (lw_sys_window's hps_lw; bit map: docs/LW_BRIDGE.md)
	.HPS_BUS({lw_bus, f1, HDMI_TX_VS, 
				 clk_100m, clk_ihdmi,
				 ce_hpix, hde_emu, hhs_fix, hvs_fix, 
				 io_wait, clk_sys, io_fpga, io_uio, io_strobe, io_wide, io_din, io_dout}),
`else
	.HPS_BUS({f1, HDMI_TX_VS, 
				 clk_100m, clk_ihdmi,
				 ce_hpix, hde_emu, hhs_fix, hvs_fix, 
				 io_wait, clk_sys, io_fpga, io_uio, io_strobe, io_wide, io_din, io_dout}),
`endif

	.VGA_R(r_out),
	.VGA_G(g_out),
	.VGA_B(b_out),
	.VGA_HS(hs_emu),
	.VGA_VS(vs_emu),
	.VGA_DE(de_emu),
	.VGA_F1(f1),
	.VGA_SCALER(vga_force_scaler),

`ifndef MISTER_DUAL_SDRAM
	.VGA_DISABLE(VGA_DISABLE),
`endif

	.HDMI_WIDTH(direct_video ? 12'd0 : hdmi_width),
	.HDMI_HEIGHT(direct_video ? 12'd0 : hdmi_height),
	.HDMI_FREEZE(freeze),
	.HDMI_BLACKOUT(hdmi_blackout),
	.HDMI_BOB_DEINT(bob_deint),

	.LFB_FMT(LFB_FMT),
	.LFB_BASE(LFB_BASE),
	.LFB_WIDTH(LFB_WIDTH),
	.LFB_HEIGHT(LFB_HEIGHT),
	.LFB_STRIDE(LFB_STRIDE),

	.CLK_VIDEO(clk_vid),
	.CE_PIXEL(ce_pix),
	.VGA_SL(scanlines),
	.VIDEO_ARX(ARX),
	.VIDEO_ARY(ARY),

`ifdef MISTER_FB
	.FB_EN(fb_en),
	.FB_FORMAT(fb_fmt),
	.FB_WIDTH(fb_width),
	.FB_HEIGHT(fb_height),
	.FB_BASE(fb_base),
	.FB_STRIDE(fb_stride),
	.FB_VBL(fb_vbl),
	.FB_LL(lowlat),
	.FB_FORCE_BLANK(fb_force_blank),

`ifdef MISTER_FB_PALETTE
	.FB_PAL_CLK (fb_pal_clk),
	.FB_PAL_ADDR(fb_pal_a),
	.FB_PAL_DOUT(fb_pal_d),
	.FB_PAL_DIN (fb_pal_q),
	.FB_PAL_WR  (fb_pal_wr),
`endif

`endif

	.LED_USER(led_user),
	.LED_POWER(led_power),
	.LED_DISK(led_disk),

	.CLK_AUDIO(clk_audio),
	.AUDIO_L(audio_l),
	.AUDIO_R(audio_r),
	.AUDIO_S(audio_s),
	.AUDIO_MIX(audio_mix),

	.ADC_BUS({ADC_SCK,ADC_SDO,ADC_SDI,ADC_CONVST}),

	.DDRAM_CLK(ram_clk),
	.DDRAM_ADDR(ram_address),
	.DDRAM_BURSTCNT(ram_burstcount),
	.DDRAM_BUSY(ram_waitrequest),
	.DDRAM_DOUT(ram_readdata),
	.DDRAM_DOUT_READY(ram_readdatavalid),
	.DDRAM_RD(ram_read),
	.DDRAM_DIN(ram_writedata),
	.DDRAM_BE(ram_byteenable),
	.DDRAM_WE(ram_write),

	.SDRAM_DQ(SDRAM_DQ),
	.SDRAM_A(SDRAM_A),
	.SDRAM_DQML(SDRAM_DQML),
	.SDRAM_DQMH(SDRAM_DQMH),
	.SDRAM_BA(SDRAM_BA),
	.SDRAM_nCS(SDRAM_nCS),
	.SDRAM_nWE(SDRAM_nWE),
	.SDRAM_nRAS(SDRAM_nRAS),
	.SDRAM_nCAS(SDRAM_nCAS),
	.SDRAM_CLK(SDRAM_CLK),
	.SDRAM_CKE(SDRAM_CKE),

`ifdef MISTER_DUAL_SDRAM
	.SDRAM2_DQ(SDRAM2_DQ),
	.SDRAM2_A(SDRAM2_A),
	.SDRAM2_BA(SDRAM2_BA),
	.SDRAM2_nCS(SDRAM2_nCS),
	.SDRAM2_nWE(SDRAM2_nWE),
	.SDRAM2_nRAS(SDRAM2_nRAS),
	.SDRAM2_nCAS(SDRAM2_nCAS),
	.SDRAM2_CLK(SDRAM2_CLK),
	.SDRAM2_EN(io_dig),
`endif

	.BUTTONS(btn),
	.OSD_STATUS(osd_status),

	.SD_SCK(SD_CLK),
	.SD_MOSI(SD_MOSI),
	.SD_MISO(SD_MISO),
	.SD_CS(SD_CS),
	.SD_CD(SD_CD),

	.UART_CTS(uart_rts),
	.UART_RTS(uart_cts),
	.UART_RXD(uart_txd),
	.UART_TXD(uart_rxd),
	.UART_DTR(uart_dsr),
	.UART_DSR(uart_dtr),

	.USER_OUT(user_out),
	.USER_IN(user_in)
);

endmodule

/////////////////////////////////////////////////////////////////////

module sync_fix
(
	input clk,
	
	input sync_in,
	output sync_out
);

assign sync_out = sync_in ^ pol;

reg pol;
always @(posedge clk) begin
	reg [31:0] cnt;
	reg s1,s2;

	s1 <= sync_in;
	s2 <= s1;
	cnt <= s2 ? (cnt - 1) : (cnt + 1);

	if(~s2 & s1) begin
		cnt <= 0;
		pol <= cnt[31];
	end
end

endmodule

/////////////////////////////////////////////////////////////////////

// CSync generation
// Shifts HSync left by 1 HSync period during VSync

module csync
(
	input  clk,
	input  hsync,
	input  vsync,

	output csync
);

assign csync = (csync_vs ^ csync_hs);

reg csync_hs, csync_vs;
always @(posedge clk) begin
	reg prev_hs;
	reg [15:0] h_cnt, line_len, hs_len;

	// Count line/Hsync length
	h_cnt <= h_cnt + 1'd1;

	prev_hs <= hsync;
	if (prev_hs ^ hsync) begin
		h_cnt <= 0;
		if (hsync) begin
			line_len <= h_cnt - hs_len;
			csync_hs <= 0;
		end
		else hs_len <= h_cnt;
	end
	
	if (~vsync) csync_hs <= hsync;
	else if(h_cnt == line_len) csync_hs <= 1;
	
	csync_vs <= vsync;
end

endmodule
