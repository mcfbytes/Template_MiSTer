//============================================================================
//  lw_sys_window.sv -- part of the LW bridge framework (docs/LW_BRIDGE.md)
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

// LW window declared by `SYS_LW_CAPS (emu_ports.vh): self-describing directory, PAD, SECTOR, OSDBUF and SYS blocks.
// Bridge side runs on clk_100m with no reset. PAD stores park in per-op latest-wins slots (digital)
// or a 16-deep FIFO (both analog ops, drop-and-count when full), both in one M10K; a held-data handshake
// (lw_xfer, lw_cdc.sv) drains them round-robin to clk_sys. The SECTOR block (lw_sector.sv) is built only when
// the core declares it (SEC_EN): one primary hps_io read slot plus up to two save slots ("cards"); entry 1
// words 7-12 declare each card slot, its DDR image base (wsrc) and extent. Without SEC_EN, entry 1
// reads 0 (no kind) and the block's addresses read 0.
// OSDBUF decodes image stores and CTRL; the shadow RAMs live in each osd instance (osd.v).
// SYS (lw_sys_status.sv) raises the status back-channel's cause on its own line; entry 3 declares
// the line's pool ordinal, the CAUSE word, the class tag and the cause bits, so the host reads them here.
// LANE (lw_lane.sv, LANE_EN) replays ordered link frames onto sys_top's link nets and can capture their
// replies; entry 4 declares it, and COUNT is 5. Without LANE_EN the directory is 4 entries and the block's
// addresses read 0xDEADBEEF.
// LW_DIAG=0 (shipping) compiles out SCRATCH (reads 0, writes ignored), the analog-drop count (reads 0)
// and lw_sector's diagnostics; LW_DIAG=1 (debug builds) keeps them.
// See docs/LW_BRIDGE.md for the register map and the adoption recipe.

module lw_sys_window #(
	// SECTOR block, as the core declares it (sys_top.v maps the MISTER_LW_SECTOR_* / MISTER_LW_CARD* macros).
	parameter        SEC_EN    = 0,          // 1 = build the SECTOR block and declare directory entry 1
	parameter  [7:0] SEC_SLOT  = 8'd1,       // the hps_io slot the primary frames feed (sd_rd[SEC_SLOT])
	parameter  [9:0] SEC_FRAME = 10'd128,    // 32-bit DATA words per primary frame (128 = one 512-byte block)
	parameter  [2:0] BLKSZ     = 3'd2,       // the core's hps_io BLKSZ, reported in the GET_SDSTAT word
	parameter  [3:0] MC_SLOT0  = 4'hE,       // card 0: hps_io slot (0-3), its DDR image base and extent;
	parameter [31:0] MC_WSRC0  = 32'd0,      //   a card with wsrc 0 is not declared (slot 4'hE never decodes)
	parameter [31:0] MC_EXT0   = 32'd0,
	parameter  [3:0] MC_SLOT1  = 4'hF,       // card 1, likewise
	parameter [31:0] MC_WSRC1  = 32'd0,
	parameter [31:0] MC_EXT1   = 32'd0,
	parameter  [9:0] MC_FRAME  = 10'd256,    // 32-bit DATA words per card frame (256 = one 1 KB block)
	// LANE block (sys_top.v sets it with MISTER_LW_LANE): 1 = build the frame lane and declare entry 4.
	parameter        LANE_EN   = 0
)
(
	input wire         clk,          // clk_100m

	input wire  [20:0] lw_raddr,
	input wire         lw_ren,
	input wire         lw_rd_ack,
	input wire         lw_rd_word,
	output reg  [31:0] lw_rdata,

	input wire  [20:0] lw_waddr,
	input wire         lw_write,
	input wire  [31:0] lw_wdata,
	input wire   [3:0] lw_be,

	output wire        doorbell,     // SECTOR cause level -> f2h_irq[8]
	output wire        sys_irq,      // SYS cause level -> f2h_irq[9]

	input wire         clk_sys,
	output wire [48:0] lw_pad,       // {we, op[7:0], idx[7:0], data[31:0]} on clk_sys
	input wire  [34:0] blk_up,       // {reset, hps_ack, req, lba[31:0]} on clk_sys, from emu
	output wire [18:0] blk_dn,       // {en_sys, ack, wr, data[15:0]} on clk_sys, to emu
	input wire  [19:0] card_up,      // {card 1, card 0}, each {hps_ack, wr, rd, lba[6:0]} on clk_sys, from emu
	output wire [12:0] card_dn,      // {ack1, en_sys1, ack0, en_sys0, addr[8:0]} on clk_sys, to emu
	input wire [131:0] st_up,        // {stflg[3:0], status_req[127:0]} on clk_sys, from hps_io

	input wire         lane_busy,    // clk_sys: any of the link's own selects
	input wire         lane_wait,    // clk_sys: io_wait | vs_wait
	input wire  [15:0] lane_dout,    // clk_sys: the link's reply word, io_dout | io_dout_sys
	output wire        lane_own,     // clk_sys: the lane player, to lw_link
	output wire        lane_hold,
	output wire  [3:0] lane_sel,
	output wire        lane_strobe,
	output wire [15:0] lane_din,

	output wire        osd_we,       // OSDBUF image store, clk_100m: word osd_addr of the 256x128 image
	output wire  [9:0] osd_addr,
	output wire [31:0] osd_data,
	output wire  [3:0] osd_be,
	output wire  [1:0] osd_ctrl      // {LWSEL & HR, LWSEL}, clk_100m registers; each osd synchronizes them
);

`ifdef LW_DIAG
localparam        DIAG     = `LW_DIAG;
`else
localparam        DIAG     = 0;
`endif

localparam [20:0] WIN_OFF  = 21'h40000;
localparam [20:0] WIN_SIZE = 21'h80000;
localparam [20:0] SEC_OFF  = 21'h01000;   // relative to the window
localparam [20:0] SEC_SIZE = 21'h02000;
localparam [20:0] PAD_OFF  = 21'h40000;
localparam [20:0] PAD_SIZE = 21'h40000;
localparam        SEC_AW    = 9;          // FIFO depth 512
localparam [15:0] SEC_GAP   = 16'd2;      // clk_sys cycles between write strobes
localparam  [7:0] MC_FLAGS  = 8'h03;      // RD: BEGIN_S frames; WR: WACK_S, bytes from the DDR image
localparam        MC_DECL0  = SEC_EN && MC_WSRC0 != 0;
localparam        MC_DECL1  = SEC_EN && MC_WSRC1 != 0;
localparam [20:0] OSD_OFF  = 21'h04000;
localparam [20:0] OSD_SIZE = 21'h02000;
localparam [12:0] OSD_CTRL = 13'h1000;   // block-relative; the image is 0x0000-0x0FFF
localparam [31:0] OSD_GEOM = {16'd128, 16'd256};
localparam [20:0] SYS_OFF  = 21'h03000;
localparam [20:0] SYS_SIZE = 21'h01000;
localparam  [7:0] SYS_LINE = 8'd2;        // pool ordinal 2 = f2h_irq[9] (GIC SPI 49)
localparam [31:0] SYS_TAG  = 32'h0010_0000;   // the cause word's class tag: bit 20 = SYS
localparam [31:0] SYS_CAUSES = 32'h0000_0001; // payload bits it raises: bit 0 = STATUS (0x29)
localparam [20:0] LANE_OFF = 21'h06000;
localparam [20:0] LANE_SIZE = 21'h01000;
localparam        LANE_AW  = 10;          // FIFO depth 1024: a 256-entry 0x33 gamma table is 770 entries
localparam  [7:0] LANE_GAP = 8'd4;        // clk_sys cycles without a wait between strobes, 2..7
localparam        LANE_RAW = 8;           // reply FIFO depth 256
localparam  [6:0] DIR_WORDS = LANE_EN ? 7'd84 : 7'd68;   // header + 5 or 4 entries

// LANE entry: the UIO (sel 0) opcodes verified for write-only replay; word 12, the first-word reply the lane
// stands in for, one byte per op: [7:0] = 0x20's 'b11 (sys_top answers it); a capture frame also records it.
localparam [255:0] LANE_OPS = (256'd1 << 8'h01) | (256'd1 << 8'h1C) | (256'd1 << 8'h1D) | (256'd1 << 8'h1E) |
                              (256'd1 << 8'h22) | (256'd1 << 8'h24) | (256'd1 << 8'h31) | (256'd1 << 8'h32) |
                              (256'd1 << 8'h33) | (256'd1 << 8'h3B);
localparam [31:0] LANE_CAPS = 32'h0000_0003;

localparam [255:0] PAD_OPS = (256'd1 << 8'h02) | (256'd1 << 8'h03) |
                             (256'd1 << 8'h10) | (256'd1 << 8'h11) |
                             (256'd1 << 8'h12) | (256'd1 << 8'h13) |
                             (256'd1 << 8'h1A) | (256'd1 << 8'h3D);

reg [31:0] scratch = 0;
reg [31:0] adrop   = 0;               // analog stores dropped on a full FIFO
reg  [1:0] octrl   = 0;               // OSD CTRL, {HR, LWSEL}
reg        ohr     = 0;

wire [20:0] rwin = lw_raddr - WIN_OFF;
wire [20:0] wwin = lw_waddr - WIN_OFF;
wire        r_in = (lw_raddr >= WIN_OFF) && (lw_raddr < WIN_OFF + WIN_SIZE);
wire        w_in = (lw_waddr >= WIN_OFF) && (lw_waddr < WIN_OFF + WIN_SIZE);
wire        r_pad = r_in && rwin >= PAD_OFF;
wire        w_pad = w_in && wwin >= PAD_OFF;
wire        r_sec = r_in && rwin >= SEC_OFF && rwin < SEC_OFF + SEC_SIZE;
wire        w_sec = w_in && wwin >= SEC_OFF && wwin < SEC_OFF + SEC_SIZE;
wire        r_osd = r_in && rwin >= OSD_OFF && rwin < OSD_OFF + OSD_SIZE;
wire        w_osd = w_in && wwin >= OSD_OFF && wwin < OSD_OFF + OSD_SIZE;
wire        r_sys = r_in && rwin >= SYS_OFF && rwin < SYS_OFF + SYS_SIZE;
wire        r_lane = LANE_EN && r_in && rwin >= LANE_OFF && rwin < LANE_OFF + LANE_SIZE;
wire        w_lane = LANE_EN && w_in && wwin >= LANE_OFF && wwin < LANE_OFF + LANE_SIZE;
wire [20:0] rlane = rwin - LANE_OFF;
wire [20:0] wlane = wwin - LANE_OFF;
wire [31:0] lane_rdata;
wire [20:0] rsys  = rwin - SYS_OFF;
wire [31:0] sys_rdata;
wire [12:0] rosd  = rwin[12:0];           // OSD_OFF is 8 KB aligned
wire [12:0] wosd  = wwin[12:0];
wire [20:0] wblk = wwin - PAD_OFF;
wire [20:0] rsec = rwin - SEC_OFF;
wire [20:0] wsec = wwin - SEC_OFF;
wire [31:0] sec_rdata;

always @(*) begin
	lw_rdata = 32'hDEADBEEF;
	if (r_pad) lw_rdata = (DIAG && rwin == PAD_OFF + (21'h1A << 10)) ? adrop : 32'd0;
	else if (r_sec) lw_rdata = SEC_EN ? sec_rdata : 32'd0;
	else if (r_sys) lw_rdata = sys_rdata;
	else if (r_lane) lw_rdata = lane_rdata;
	else if (r_osd) lw_rdata = (rosd == OSD_CTRL)       ? {30'd0, octrl} :
	                           (rosd == OSD_CTRL + 13'd4) ? 32'h3144534F :   // "OSD1"
	                           32'd0;
	else if (r_in && rwin < {DIR_WORDS, 2'b00}) begin
		case (rwin[8:2])
			7'd0:  lw_rdata = 32'h31535953;          // "SYS1"
			7'd1:  lw_rdata = 32'h00010000;
			7'd2:  lw_rdata = LANE_EN ? 32'd5 : 32'd4;
			7'd3:  lw_rdata = DIAG ? scratch : 32'd0;
			7'd4:  lw_rdata = 32'd1;                 // entry 0: kind PAD
			7'd5:  lw_rdata = 32'h00010000;
			7'd6:  lw_rdata = {11'd0, PAD_OFF};
			7'd7:  lw_rdata = {11'd0, PAD_SIZE};
			7'd8:  lw_rdata = PAD_OPS[ 31:  0];
			7'd9:  lw_rdata = PAD_OPS[ 63: 32];
			7'd10: lw_rdata = PAD_OPS[ 95: 64];
			7'd11: lw_rdata = PAD_OPS[127: 96];
			7'd12: lw_rdata = PAD_OPS[159:128];
			7'd13: lw_rdata = PAD_OPS[191:160];
			7'd14: lw_rdata = PAD_OPS[223:192];
			7'd15: lw_rdata = PAD_OPS[255:224];
			// entry 1: kind SECTOR, minor 1 ("SEC2": save slots in words 7-12); all 0 without SEC_EN
			7'd20: lw_rdata = SEC_EN ? 32'd2 : 32'd0;
			7'd21: lw_rdata = SEC_EN ? 32'h00010001 : 32'd0;
			7'd22: lw_rdata = SEC_EN ? {11'd0, SEC_OFF} : 32'd0;
			7'd23: lw_rdata = SEC_EN ? {11'd0, SEC_SIZE} : 32'd0;
			7'd24: lw_rdata = SEC_EN ? {SEC_SLOT, 8'd0, 6'd0, SEC_FRAME} : 32'd0;
			7'd25: lw_rdata = SEC_EN ? {11'd0, WIN_OFF + SEC_OFF} : 32'd0;  // CAUSE, absolute LW (= the IRQ:BLK offset)
			7'd26: lw_rdata = SEC_EN ? {16'd1 << SEC_AW, SEC_GAP} : 32'd0;
			7'd27: lw_rdata = MC_DECL0 ? {4'd0, MC_SLOT0, MC_FLAGS, 6'd0, MC_FRAME} : 32'd0;
			7'd28: lw_rdata = MC_DECL0 ? MC_WSRC0 : 32'd0;
			7'd29: lw_rdata = MC_DECL1 ? {4'd0, MC_SLOT1, MC_FLAGS, 6'd0, MC_FRAME} : 32'd0;
			7'd30: lw_rdata = MC_DECL1 ? MC_WSRC1 : 32'd0;
			7'd31: lw_rdata = MC_DECL0 ? MC_EXT0 : 32'd0;
			7'd32: lw_rdata = MC_DECL1 ? MC_EXT1 : 32'd0;
			7'd36: lw_rdata = 32'd3;                 // entry 2: kind OSDBUF
			7'd37: lw_rdata = 32'h00010000;
			7'd38: lw_rdata = {11'd0, OSD_OFF};
			7'd39: lw_rdata = {11'd0, OSD_SIZE};
			7'd40: lw_rdata = OSD_GEOM;              // {height, width} in pixels, 1 bit each
			7'd52: lw_rdata = 32'd4;                 // entry 3: kind SYS
			7'd53: lw_rdata = 32'h00010000;
			7'd54: lw_rdata = {11'd0, SYS_OFF};
			7'd55: lw_rdata = {11'd0, SYS_SIZE};
			7'd56: lw_rdata = {24'd0, SYS_LINE};     // the line: a 1-based pool ordinal, as IRQ: tokens
			7'd57: lw_rdata = {11'd0, WIN_OFF + SYS_OFF};   // CAUSE, absolute LW
			7'd58: lw_rdata = SYS_TAG;
			7'd59: lw_rdata = SYS_CAUSES;
			// entry 4: kind LANE, minor 1 (reply capture); reached only with LANE_EN (DIR_WORDS)
			7'd68: lw_rdata = 32'd5;
			7'd69: lw_rdata = 32'h00010001;
			7'd70: lw_rdata = {11'd0, LANE_OFF};
			7'd71: lw_rdata = {11'd0, LANE_SIZE};
			7'd72: lw_rdata = LANE_OPS[ 31:  0];
			7'd73: lw_rdata = LANE_OPS[ 63: 32];
			7'd74: lw_rdata = LANE_OPS[ 95: 64];
			7'd75: lw_rdata = LANE_OPS[127: 96];
			7'd76: lw_rdata = LANE_OPS[159:128];
			7'd77: lw_rdata = LANE_OPS[191:160];
			7'd78: lw_rdata = LANE_OPS[223:192];
			7'd79: lw_rdata = LANE_OPS[255:224];
			7'd80: lw_rdata = LANE_CAPS;
			7'd81: lw_rdata = {16'd1 << LANE_AW, 8'd0, LANE_GAP};
			7'd82: lw_rdata = {16'd1 << LANE_RAW, 16'd0};   // reply FIFO depth
			default: lw_rdata = 32'd0;
		endcase
	end
end

generate
`ifdef MISTER_LW_SECTOR_SLOT
if (SEC_EN) begin : sec
	// an undeclared card's request lines are held low, so the feeder never serves it
	wire [1:0] xdecl = {MC_DECL1 ? 1'b1 : 1'b0, MC_DECL0 ? 1'b1 : 1'b0};

	lw_sector #(.SLOT(SEC_SLOT), .FRAME(SEC_FRAME), .BLKSZ(BLKSZ), .AW(SEC_AW), .DIAG(DIAG),
	            .XSLOT0(MC_SLOT0), .XSLOT1(MC_SLOT1), .XFRAME(MC_FRAME)) sector
	(
		.clk(clk),
		.raddr(rsec[12:0]), .rsel(r_sec), .ren(lw_ren), .rd_ack(lw_rd_ack), .rd_word(lw_rd_word), .rdata(sec_rdata),
		.waddr(wsec[12:0]), .wsel(w_sec), .write(lw_write), .wdata(lw_wdata), .be(lw_be),
		.doorbell(doorbell),
		.clk_sys(clk_sys),
		.c_reset(blk_up[34]), .hps_ack(blk_up[33]), .c_req(blk_up[32]), .c_lba(blk_up[31:0]),
		.en_sys(blk_dn[18]), .o_ack(blk_dn[17]), .o_wr(blk_dn[16]), .o_dat(blk_dn[15:0]),
		.o_adr(card_dn[8:0]),
		.x_hps_ack({card_up[19], card_up[9]}),
		.x_wr({card_up[18], card_up[8]} & xdecl), .x_rd({card_up[17], card_up[7]} & xdecl),
		.x_lba({card_up[16:10], card_up[6:0]}),
		.x_en_sys({card_dn[11], card_dn[9]}), .x_ack({card_dn[12], card_dn[10]})
	);
end
`else
// SEC_EN without the tier macro: lw_sector.sv is empty, so the build stops here, naming the macro
if (SEC_EN) begin : sec
	lw_sector_needs_MISTER_LW_SECTOR_SLOT guard();
end
`endif
else begin : nosec
	assign sec_rdata = 32'd0;
	assign doorbell  = 1'b0;
	assign blk_dn    = 19'd0;
	assign card_dn   = 13'd0;
end
endgenerate

generate
`ifdef MISTER_LW_LANE
if (LANE_EN) begin : lane
	lw_lane #(.AW(LANE_AW), .GAP(LANE_GAP[2:0]), .RAW(LANE_RAW)) lane
	(
		.clk(clk),
		.raddr(rlane[11:0]), .rdata(lane_rdata),
		.waddr(wlane[11:0]), .wsel(w_lane), .write(lw_write), .wdata(lw_wdata), .be(lw_be),
		.clk_sys(clk_sys), .l_busy(lane_busy), .l_wait(lane_wait), .l_dout(lane_dout),
		.own(lane_own), .hold(lane_hold), .sel(lane_sel), .strobe(lane_strobe), .din(lane_din)
	);
end
`else
// LANE_EN without the tier macro: lw_lane.sv is empty, so the build stops here, naming the macro
if (LANE_EN) begin : lane
	lw_lane_needs_MISTER_LW_LANE guard();
end
`endif
else begin : nolane
	assign lane_rdata  = 32'd0;
	assign lane_own    = 1'b0;
	assign lane_hold   = 1'b0;
	assign lane_sel    = 4'd0;
	assign lane_strobe = 1'b0;
	assign lane_din    = 16'd0;
end
endgenerate

lw_sys_status sys_status
(
	.clk(clk),
	.raddr(rsys[9:0]), .rsel(r_sys && rsys < 21'h400), .ren(lw_ren), .rd_ack(lw_rd_ack), .rd_word(lw_rd_word),
	.rdata(sys_rdata),
	.irq(sys_irq),
	.clk_sys(clk_sys), .stflg(st_up[131:128]), .status_req(st_up[127:0])
);

// OSDBUF: image stores go to both osd shadows (byte lanes honour BE); CTRL is {HR, LWSEL}.
assign osd_we   = lw_write && w_osd && ~wosd[12];
assign osd_addr = wosd[11:2];
assign osd_data = lw_wdata;
assign osd_be   = lw_be;
assign osd_ctrl = {ohr, octrl[0]};

always @(posedge clk) if (lw_write && w_osd && wosd == OSD_CTRL && lw_be[0]) begin
	octrl <= lw_wdata[1:0];
	ohr   <= &lw_wdata[1:0];
end

// clk_100m side: SCRATCH, the pending slots, and the round-robin drain into the handshake.
function automatic [2:0] slot_of(input [7:0] op);
	case (op)
		8'h02: slot_of = 3'd0;
		8'h03: slot_of = 3'd1;
		8'h10: slot_of = 3'd2;
		8'h11: slot_of = 3'd3;
		8'h12: slot_of = 3'd4;
		8'h13: slot_of = 3'd5;
		default: slot_of = 3'd7;
	endcase
endfunction

function automatic [7:0] op_of(input [2:0] slot);
	case (slot)
		3'd0: op_of = 8'h02;
		3'd1: op_of = 8'h03;
		3'd2: op_of = 8'h10;
		3'd3: op_of = 8'h11;
		3'd4: op_of = 8'h12;
		default: op_of = 8'h13;
	endcase
endfunction

// one M10K: digital masks at 0..5, the analog FIFO {right, idx, data[15:0]} at 16..31
(* ramstyle = "M10K" *) reg [31:0] pram [0:31];
reg [31:0] pq;
reg  [5:0] dirty = 0;

reg  [4:0] awp = 0, arp = 0;
wire       aempty = (awp == arp);
wire       afull  = (awp - arp) == 5'd16;

wire       st   = lw_write && w_pad && &lw_be;
wire [7:0] wop  = wblk[17:10];
wire [7:0] widx = wblk[9:2];
wire [2:0] wsl  = slot_of(wop);
wire       st_d = st && wsl != 3'd7 && widx == 8'd0;
wire       st_a = st && (wop == 8'h1A || wop == 8'h3D) && !st_d;

wire       busy;

wire [6:0] ready = {~aempty, dirty};
reg  [2:0] rr = 0;
reg  [2:0] pick;
reg        pick_v;
reg  [1:0] ds = 0;                     // drain: 0 pick, 1 read, 2 launch
reg  [2:0] psel = 0;
integer    i, k;

always @(*) begin
	pick_v = 0;
	pick   = 0;
	for (i = 0; i < 7; i = i + 1) begin
		k = rr + i;
		if (k >= 7) k = k - 7;
		if (!pick_v && ready[k]) begin
			pick_v = 1;
			pick   = k[2:0];
		end
	end
end

wire [4:0] pra = (psel == 3'd6) ? {1'b1, arp[3:0]} : {2'b00, psel};

always @(posedge clk) begin
	if (st_d) pram[{2'b00, wsl}] <= lw_wdata;
	else if (st_a && !afull) pram[{1'b1, awp[3:0]}] <= {7'd0, wop == 8'h3D, widx, lw_wdata[15:0]};
	pq <= pram[pra];
end

always @(posedge clk) begin
	if (DIAG && lw_write && w_in && wwin == 21'h0C) begin
		if (lw_be[0]) scratch[ 7: 0] <= lw_wdata[ 7: 0];
		if (lw_be[1]) scratch[15: 8] <= lw_wdata[15: 8];
		if (lw_be[2]) scratch[23:16] <= lw_wdata[23:16];
		if (lw_be[3]) scratch[31:24] <= lw_wdata[31:24];
	end

	case (ds)
		2'd0: if (!busy && pick_v) begin
			psel <= pick;
			rr   <= (pick == 3'd6) ? 3'd0 : pick + 3'd1;
			ds   <= 2'd1;
		end
		2'd1: begin
			if (psel == 3'd6) arp <= arp + 5'd1;
			else dirty[psel] <= 0;
			ds <= 2'd2;
		end
		default: ds <= 2'd0;   // the handshake takes the word in this cycle
	endcase

	// after the drain, so a store landing in the read cycle stays dirty (its RAM read returned the old word)
	if (st_d) dirty[wsl] <= 1;
	else if (st_a) begin
		if (afull) begin
			if (DIAG) adrop <= adrop + 32'd1;
		end
		else awp <= awp + 5'd1;
	end
end

// to clk_sys: the store's word, and its strobe the cycle after the capture
wire [47:0] pad_w = (psel == 3'd6) ? {pq[24] ? 8'h3D : 8'h1A, pq[23:16], 16'd0, pq[15:0]} : {op_of(psel), 8'd0, pq};

lw_xfer #(.W(48)) pad_x
(
	.s_clk(clk), .s_go(ds == 2'd2), .s_d(pad_w), .s_busy(busy),
	.d_clk(clk_sys), .d_q(lw_pad[47:0]), .d_v(lw_pad[48])
);

endmodule

`default_nettype wire
`endif
