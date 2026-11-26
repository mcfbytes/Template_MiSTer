//============================================================================
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

module emu
(
	`include "sys/emu_ports.vh"
);

///////// Default values for ports not used in this core /////////

assign ADC_BUS  = 'Z;
assign USER_OUT = '1;
assign {UART_RTS, UART_TXD, UART_DTR} = 0;
assign {SD_SCK, SD_MOSI, SD_CS} = 'Z;
assign {SDRAM_DQ, SDRAM_A, SDRAM_BA, SDRAM_CLK, SDRAM_CKE, SDRAM_DQML, SDRAM_DQMH, SDRAM_nWE, SDRAM_nCAS, SDRAM_nRAS, SDRAM_nCS} = 'Z;
assign {DDRAM_CLK, DDRAM_BURSTCNT, DDRAM_ADDR, DDRAM_DIN, DDRAM_BE, DDRAM_RD, DDRAM_WE} = '0;  

assign VGA_SL = 0;
assign VGA_F1 = 0;
assign VGA_SCALER  = 0;
assign VGA_DISABLE = 0;
assign HDMI_FREEZE = 0;
assign HDMI_BLACKOUT = 0;
assign HDMI_BOB_DEINT = 0;

assign AUDIO_S = 0;
assign AUDIO_L = 0;
assign AUDIO_R = 0;
assign AUDIO_MIX = 0;

assign LED_DISK = 0;
assign LED_POWER = 0;
assign BUTTONS = 0;

//////////////////////////////////////////////////////////////////

wire [1:0] ar = status[122:121];

assign VIDEO_ARX = (!ar) ? 12'd4 : (ar - 1'd1);
assign VIDEO_ARY = (!ar) ? 12'd3 : 12'd0;

`include "build_id.v" 
`ifdef MISTER_LW
// The LW example's CONF_STR text, only with MISTER_LW: `SYS_LW_CAPS (sys/emu_ports.vh) declares the LW window in
// field 1 (no comma after the macro), and TPL_LW_J names the example's pad buttons, with the SECTOR block after an
// S line that lets a stock Main mount an image on the block reader's slot.
`ifdef MISTER_LW_SECTOR_SLOT
`define TPL_BLK
`define TPL_BLK_SLOT `MISTER_LW_SECTOR_SLOT
`elsif MISTER_OCRAM_SECTOR_SLOT
`define TPL_BLK
`define TPL_BLK_SLOT `MISTER_OCRAM_SECTOR_SLOT
`endif
`ifdef TPL_BLK
localparam       BLK_SLOT  = `TPL_BLK_SLOT;            // the hps_io slot the SECTOR feeder serves: 1-9 here (below)
localparam [7:0] BLK_DIGIT = "0" + BLK_SLOT;
`define TPL_LW_J "S", BLK_DIGIT, ",IMG,Block image;", "J,Colour,Read block;",
`else
`define TPL_LW_J "J,Colour;",
`endif
`else
`define TPL_LW_J
`endif
localparam CONF_STR = {
`ifdef MISTER_LW
	"Template;", `SYS_LW_CAPS ";",
`else
	"Template;;",
`endif
	"-;",
	"O[122:121],Aspect ratio,Original,Full Screen,[ARC1],[ARC2];",
	"O[2],TV Mode,NTSC,PAL;",
	"O[4:3],Noise,White,Red,Green,Blue;",
	"-;",
	"P1,Test Page 1;",
	"P1-;",
	"P1-, -= Options in page 1 =-;",
	"P1-;",
	"P1O[5],Option 1-1,Off,On;",
	"d0P1F1,BIN;",
	"H0P1O[10],Option 1-2,Off,On;",
	"-;",
	"P2,Test Page 2;",
	"P2-;",
	"P2-, -= Options in page 2 =-;",
	"P2-;",
	"P2S0,DSK;",
	"P2O[7:6],Option 2,1,2,3,4;",
	"-;",
	"-;",
	`TPL_LW_J
	"T[0],Reset;",
	"R[0],Reset and close OSD;",
	"v,0;", // [optional] config version 0-99. 
	        // If CONF_STR options are changed in incompatible way, then change version number too,
			  // so all options will get default values on first start.
	"V,v",`BUILD_DATE 
};
`undef TPL_LW_J

wire forced_scandoubler;
wire   [1:0] buttons;
wire [127:0] status;
wire  [10:0] ps2_key;
`ifdef MISTER_LW
wire  [31:0] joystick_0;
reg  [127:0] status_in;
reg          status_set = 0;
`ifdef TPL_BLK
wire      [31:0] sd_lba[BLK_SLOT + 1];
wire [BLK_SLOT:0] sd_rd;
wire [BLK_SLOT:0] sd_ack;
wire      [12:0] sd_buff_addr;   // WIDE: a 16-bit word index
wire      [15:0] sd_buff_dout;
wire             sd_buff_wr;
`endif
`endif

hps_io #(.CONF_STR(CONF_STR)
`ifdef MISTER_LW
`ifdef TPL_BLK
	, .WIDE(1), .VDNUM(BLK_SLOT + 1)
`ifdef MISTER_LW_BLKSZ
	, .BLKSZ(`MISTER_LW_BLKSZ)
`elsif MISTER_OCRAM_BLKSZ
	, .BLKSZ(`MISTER_OCRAM_BLKSZ)
`endif
`endif
`endif
) hps_io
(
	.clk_sys(clk_sys),
	.HPS_BUS(HPS_BUS),
	.EXT_BUS(),
	.gamma_bus(),

	.forced_scandoubler(forced_scandoubler),

	.buttons(buttons),
	.status(status),
	.status_menumask({status[5]}),
`ifdef MISTER_LW
	.status_in(status_in),
	.status_set(status_set),
	.joystick_0(joystick_0),
`ifdef TPL_BLK
	.sd_lba(sd_lba),
	.sd_rd(sd_rd),
	.sd_wr('0),
	.sd_ack(sd_ack),
	.sd_buff_addr(sd_buff_addr),
	.sd_buff_dout(sd_buff_dout),
	.sd_buff_wr(sd_buff_wr),
`endif
`endif
	
	.ps2_key(ps2_key)
);

///////////////////////   CLOCKS   ///////////////////////////////

wire clk_sys;
pll pll
(
	.refclk(CLK_50M),
	.rst(0),
	.outclk_0(clk_sys)
);

wire reset = RESET | status[0] | buttons[1];

wire [1:0] col = status[4:3];

wire HBlank;
wire HSync;
wire VBlank;
wire VSync;
wire ce_pix;
wire hvcnt_atzero;
wire [7:0] video;

// Leave H/V sync always on. This stabilizes the video output while the core
// is in reset. This example releases the reset when H/V counters are at zero.
reg reset_core = 1;
always @(posedge clk_sys) begin
	if(reset) reset_core <= 1;
	else if(hvcnt_atzero) reset_core <= 0;
end

mycore mycore
(
	.clk(clk_sys),
	.reset(reset_core),

	.pal(status[2]),
	.scandouble(forced_scandoubler),

	.ce_pix(ce_pix),
	.hvcnt_atzero(hvcnt_atzero),

	.HBlank(HBlank),
	.HSync(HSync),
	.VBlank(VBlank),
	.VSync(VSync),

	.video(video)
);

assign CLK_VIDEO = clk_sys;
assign CE_PIXEL = ce_pix;

assign VGA_DE = ~(HBlank | VBlank);
assign VGA_HS = HSync;
assign VGA_VS = VSync;
assign VGA_G  = (!col || col == 2) ? video : 8'd0;
assign VGA_R  = (!col || col == 1) ? video : 8'd0;
assign VGA_B  = (!col || col == 3) ? video : 8'd0;

reg  [26:0] act_cnt;
always @(posedge clk_sys) act_cnt <= act_cnt + 1'd1; 
assign LED_USER    = act_cnt[26]  ? act_cnt[25:18]  > act_cnt[7:0]  : act_cnt[25:18]  <= act_cnt[7:0];

`ifdef MISTER_LW
///////////////////////   LW BRIDGE EXAMPLE   /////////////////////
// Built only with MISTER_LW.
`ifdef TPL_BLK
// With the SECTOR block (docs/LW_BRIDGE.md, "Hooking the SECTOR block"): a block reader on hps_io slot BLK_SLOT.
generate if (BLK_SLOT < 1 || BLK_SLOT > 9) begin : blk_slot
	lw_template_block_slot_1_to_9 guard();
end endgenerate
wire        blk_hreq, blk_ack, blk_wr;
wire [13:0] blk_addr;
wire [15:0] blk_data;
reg         blk_req = 0;
reg  [31:0] blk_lba = 0;
lw_blk_mux blk_lw
(
	.HPS_BUS(HPS_BUS), .reset(reset),
	.core_req(blk_req), .core_lba(blk_lba), .core_ack(blk_ack), .core_wr(blk_wr), .core_addr(blk_addr),
	.core_data(blk_data),
	.hps_req(blk_hreq), .hps_ack(sd_ack[BLK_SLOT]), .hps_wr(sd_buff_wr), .hps_addr({1'b0, sd_buff_addr}),
	.hps_data(sd_buff_dout)
);
assign sd_rd = blk_hreq << BLK_SLOT;
genvar blk_s;
generate for (blk_s = 0; blk_s <= BLK_SLOT; blk_s = blk_s + 1) begin : blk_lba_s
	assign sd_lba[blk_s] = blk_lba;   // only the reader's slot raises a request
end endgenerate
reg [15:0] blk_ram[256];
reg [15:0] blk_q;
reg  [8:0] blk_n = 0;
always @(posedge clk_sys) begin
	if (blk_wr) blk_ram[blk_addr[7:0]] <= blk_data;
	blk_q <= blk_ram[blk_n[7:0]];
end
// 0: idle; 1: the request raised, waiting for its ack; 2: the block's strobes, until the ack falls; 3: the sum,
// one word a cycle (blk_q is the word read the cycle before, blk_qv says it is one)
reg  [1:0] blk_st = 0;
reg        blk_qv = 0;
reg [15:0] blk_sum = 0;
reg        blk_done = 0;
always @(posedge clk_sys) begin
	reg old_rd = 0;
	old_rd   <= joystick_0[5];
	blk_done <= 0;
	if (blk_done) blk_lba <= blk_lba + 1'd1;
	if (reset) begin
		blk_st  <= 0;
		blk_req <= 0;
		blk_lba <= 0;
	end
	else case (blk_st)
		0: if (joystick_0[5] & ~old_rd) begin blk_req <= 1; blk_st <= 1; end
		1: if (blk_ack) begin blk_req <= 0; blk_st <= 2; end
		2: if (~blk_ack) begin blk_n <= 0; blk_qv <= 0; blk_sum <= 0; blk_st <= 3; end
		3: begin
			blk_qv <= ~blk_n[8];
			if (~blk_n[8]) blk_n <= blk_n + 1'd1;
			if (blk_qv) blk_sum <= blk_sum + blk_q;
			if (blk_n[8] & ~blk_qv) begin blk_done <= 1; blk_st <= 0; end
		end
	endcase
end
`endif
always @(posedge clk_sys) begin
	reg         old_btn = 0;
	reg [127:0] st;
	reg         wb;
	st = status;
	wb = joystick_0[4] & ~old_btn;
	if (wb) st[4:3] = st[4:3] + 1'd1;
`ifdef TPL_BLK
	if (blk_done) begin st[63:32] = {blk_lba[15:0], blk_sum}; wb = 1; end
`endif
	old_btn    <= joystick_0[4];
	status_set <= 0;
	if (wb) begin
		status_in  <= st;
		status_set <= 1;
	end
end
`endif
endmodule
