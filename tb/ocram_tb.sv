//============================================================================
//  ocram_tb.sv -- the OCRAM mailbox floor's and OSD tier's bench (arm O of tb/run.sh)
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

// Standalone: hps_ocram_master (+ hps_ocram_diag, hps_ocram_tsink), hps_ocram_rx (+ hps_ocram_sfeed) and sys/lw_cdc.sv with
// MISTER_OCRAM and MISTER_OCRAM_OSD (and, in the first compile shape, MISTER_OCRAM_DIAG), a small MARK and poll
// period, clk_100m, and a clk_sys of 50 MHz unrelated in phase. The AXI slave models the 64 KB OCRAM (a word
// array) with a seeded latency and ready-delay distribution (or a fast one for the torture cases), and modes
// normal, secure-refused (a non-OKAY response when AxPROT[1] = 0) and never-answer; it can also fail one chosen
// read and one chosen write once. The host model runs on its own 66.7 MHz clock and stores GPO 60-600 ns apart,
// and peeks and pokes the array as the HPS's strongly-ordered accesses.
//
// Two real sys/osd.v instances prove the OSD tier: osd_spi is fed only by the SPI command protocol through its
// io_osd/io_strobe/io_din ports, osd_ocr only by hps_ocram_osd's write port; the case compares the two
// osd_buffer arrays byte for byte by hierarchical reference.
//
// A protocol monitor checks every cycle's AXI constants, valid stability, address bounds (the control page's
// read and write sets, the EVT, OSD and torture ring spans) and the one-transaction-at-a-time rule, and keeps its
// own copy of the DIAG counters and the EVT records. The cases are the design note's M7 bench, §5 of
// M7-BRIEF.md, and the floor's §6 of M3-BRIEF.md. Run: tb/run.sh (arm O), one seed per run and one compile
// shape per pass; the DIAG-only cases do not run in the shipping shape.

`timescale 1ns/1ps
`default_nettype none

`ifndef MISTER_OCRAM_POLL
`define MISTER_OCRAM_POLL 12
`endif
`ifndef MISTER_OCRAM_MARK
`define MISTER_OCRAM_MARK 20
`endif

module ocram_tb;

localparam integer POLLC = `MISTER_OCRAM_POLL;
localparam integer MARKC = `MISTER_OCRAM_MARK;

localparam [15:0] H_HELLO = 16'h0000;
localparam [15:0] H_BIND  = 16'h0004;
localparam [15:0] H_LINE  = 16'h0008;
localparam [15:0] H_SECTOR = 16'h000C;
localparam [15:0] H_IDX2  = 16'h0018;
localparam [15:0] H_IDX3  = 16'h001C;
localparam [15:0] H_IDX4  = 16'h0020;
localparam [15:0] C_HELLO = 16'h0080;
localparam [15:0] C_VERSION = 16'h0084;
localparam [15:0] C_CAPS  = 16'h0088;
localparam [15:0] C_STATE = 16'h008C;
localparam [15:0] C_IDX2  = 16'h0098;
localparam [15:0] C_IDX3  = 16'h009C;
localparam [15:0] C_IDX4  = 16'h00A0;
localparam [15:0] C_BIND  = 16'h00A4;
localparam [15:0] C_RING2 = 16'h00B8;
localparam [15:0] C_RING3 = 16'h00BC;
localparam [15:0] C_RING4 = 16'h00C0;
localparam [15:0] C_K8    = 16'h00F0;
localparam [15:0] EVT_BASE = 16'h4000;
localparam integer EVT_WORDS = 2048;
localparam [15:0] OSD_BASE = 16'h2000;
localparam integer OSD_WORDS = 2048;
localparam [15:0] TS_BASE = 16'h7000;
localparam integer TS_WORDS = 8192;

localparam [31:0] C_HELLO_ARMED = 32'h4F43_0000;
localparam [31:0] C_HELLO_WAIT  = 32'h5754_0000;
localparam [31:0] C_HELLO_STOP  = 32'h5154_0000;
localparam [31:0] C_VERSION_1_0 = 32'h0001_0000;
localparam [31:0] MARK_HEADER   = 32'h2303_0028;
localparam [31:0] VERDICT_HEADER= 32'h2504_0008;
localparam [31:0] REQ_HEADER    = 32'h2100_0001;   // kind REQUEST, arg 0, len 1
localparam [31:0] C_RING2_OSD   = 32'h182C_0800;
localparam [31:0] C_RING3_EVT   = 32'h212C_1000;
localparam [31:0] C_RING4_TS    = 32'hF834_1C00;
localparam [31:0] C_RING4_SEC   = 32'h1034_1C00;   // kind 2 (SECTOR), no line, 8192 words at 0x7000
localparam [31:0] GPO_IO        = 32'h8000_0000;   // bit 31: the I/O view, RUN 0
localparam [31:0] GPO_RUN       = 32'h9000_0000;   // bit 31 and RUN
localparam [31:0] GPO_FRZ       = 32'h0200_0000;   // bit 25: the DIAG freeze knob

`ifdef MISTER_OCRAM_OSD
localparam OSD_B = 1;
`else
localparam OSD_B = 0;
`endif
`ifdef MISTER_OCRAM_DIAG
localparam TS_B = 1;
`else
localparam TS_B = 0;
`endif
`ifdef MISTER_OCRAM_SECTOR_SLOT
localparam SEC_B = 1;
`else
localparam SEC_B = 0;
`endif
`ifdef MISTER_OCRAM_SECTOR_FRAME
localparam integer SFRAME = `MISTER_OCRAM_SECTOR_FRAME;
`else
localparam integer SFRAME = 588;
`endif
`ifdef MISTER_OCRAM_SECTOR_GAP
localparam integer SGAP = `MISTER_OCRAM_SECTOR_GAP;
`else
localparam integer SGAP = 2;
`endif
`ifdef MISTER_OCRAM_SECTOR_SLOT
localparam integer SSLOT = `MISTER_OCRAM_SECTOR_SLOT;
`else
localparam integer SSLOT = 1;
`endif
localparam  [2:0] SBLKSZ = 3'd3;                 // a CD core's: 2,352-byte frames
// the SECTOR tier's C_K words: K0 {0, gap}, K1 {BLKSZ, wide}, K2 one descriptor, K4 {RD, slot, frame}, K5 32
localparam [31:0] C_K0_EXP = SGAP;
localparam [31:0] C_K1_EXP = {21'd0, SBLKSZ, 7'd0, 1'b1};
localparam [31:0] C_K4_EXP = {8'h01, 4'd0, SSLOT[3:0], 3'd0, SFRAME[12:0]};
`ifdef MENU_CORE
localparam integer ROWS = 20;
`else
localparam integer ROWS = 16;
`endif
localparam  [15:0] ROWS16 = ROWS;
localparam [31:0] C_CAPS_EXP = 32'h0000_0008 | (OSD_B ? 32'h4 : 32'h0) | ((TS_B || SEC_B) ? 32'h10 : 32'h0);
localparam [31:0] C_K8_EXP   = {ROWS16, 16'd256};

localparam integer M_NORMAL = 0, M_SECURE_REFUSE = 1, M_NEVER = 2;

integer checks = 0, fails = 0;
integer seed = 1;

task check(input string what, input [31:0] got, input [31:0] exp);
	begin
		checks = checks + 1;
		if (got !== exp) begin
			fails = fails + 1;
			$display("FAIL %s: got %08h exp %08h", what, got, exp);
		end
		else $display("pass %s = %08h", what, got);
	end
endtask

////////////////////////////////  clocks  ///////////////////////////////////

reg clk = 0;                 // clk_100m
always #5.0 clk = ~clk;
reg clk_host = 0;            // the host's own clock: 66.7 MHz, not a multiple of 100 MHz
always #7.5 clk_host = ~clk_host;
// 50 MHz, unrelated in phase to clk_100m; OCRAM_TB_SYS_HP_PS sets the half period in ps (14762: 33.8688 MHz, a CD core's;
// 25000: the Template's 20 MHz)
`ifdef OCRAM_TB_SYS_HP_PS
localparam real SYS_HP = `OCRAM_TB_SYS_HP_PS / 1000.0;
`else
localparam real SYS_HP = 10.0;
`endif
reg clk_sys = 0;
reg sys_run = 1'b1;          // 0 stops clk_sys low (the SECTOR tier's frozen-clock case)
initial begin #3.3; forever begin #(SYS_HP); clk_sys = sys_run ? ~clk_sys : 1'b0; end end
wire clk_video = clk_sys;

//////////////////////////////  the DUT  ////////////////////////////////////

reg  [31:0] gpo_reg = 32'd0;
wire [31:0] gpo = gpo_reg;
reg         hps_rst_n = 0;

wire        present_wire, line_out;
wire  [7:0] awid, wid, bid, arid, rid;
wire [31:0] awaddr, wdata, araddr, rdata;
wire  [3:0] awlen, awcache, wstrb, arlen, arcache;
wire  [2:0] awsize, awprot, arsize, arprot;
wire  [1:0] awburst, awlock, bresp, arburst, arlock, rresp;
wire  [4:0] awuser, aruser;
wire        awvalid, awready, wlast, wvalid, wready, bvalid, bready;
wire        arvalid, arready, rlast, rvalid, rready;

// the consumer port and the OSD writer
wire        ocram_c0_req, ocram_c0_we;
wire [15:0] ocram_c0_off;
wire [31:0] ocram_c0_wdata;
wire        ocram_c0_grant, ocram_c0_done, ocram_c0_ok;
wire [31:0] ocram_c0_rdata;
wire        s_hello, s_live, s_tick, osd_bind;
wire        c0_rest, c0_fault, c0_mid, c0_open;
wire [15:0] osd_rows, osd_deepest;
wire        sec_ask, evt_stop, c0_sfault, c0_smid, c0_sbind, c0_wide, rq_post, rq_start;
wire [31:0] rq_lba;
// the SECTOR slot's core side (the window's xsec nets): the bench's consumer model drives the request
reg         x_reset = 1'b0, x_hps_ack = 1'b0, x_req = 1'b0, x_commit = 1'b0;
reg  [31:0] x_lba = 32'd0;
reg         x_wide = 1'b1;
wire        x_en, x_ack, x_wr;
wire [15:0] x_dat;
wire [13:0] x_adr;
wire        ocram_we;
wire [12:0] ocram_addr;
wire  [7:0] ocram_byte;
reg   [1:0] sel_m = 2'b00;   // both osd selects low: the quiet guard
reg         view_m = 1'b1;   // the I/O view

hps_ocram_master #(.SEC_SLOT(SSLOT), .SEC_FRAME(SFRAME), .SEC_GAP(SGAP), .SEC_BLKSZ(SBLKSZ)) dut
(
	.clk(clk),
	.hps_rst_n(hps_rst_n),
	.gp_out(gpo),
	.present(present_wire),
	.line(line_out),
	.c0_req(ocram_c0_req), .c0_we(ocram_c0_we), .c0_off(ocram_c0_off), .c0_wdata(ocram_c0_wdata),
	.c0_grant(ocram_c0_grant), .c0_done(ocram_c0_done), .c0_ok(ocram_c0_ok), .c0_rdata(ocram_c0_rdata),
	.s_hello(s_hello), .s_live(s_live), .s_tick(s_tick), .osd_bind(osd_bind),
	.c0_rest(c0_rest), .c0_fault(c0_fault), .c0_mid(c0_mid), .c0_open(c0_open),
	.osd_rows(osd_rows), .osd_deepest(osd_deepest),
	.sec_ask(sec_ask), .evt_stop(evt_stop), .c0_sfault(c0_sfault), .c0_smid(c0_smid), .c0_sbind(c0_sbind),
	.c0_wide(c0_wide), .rq_post(rq_post), .rq_lba(rq_lba), .rq_start(rq_start),
	.awid(awid), .awaddr(awaddr), .awlen(awlen), .awsize(awsize), .awburst(awburst),
	.awlock(awlock), .awcache(awcache), .awprot(awprot), .awuser(awuser), .awvalid(awvalid),
	.awready(awready),
	.wid(wid), .wdata(wdata), .wstrb(wstrb), .wlast(wlast), .wvalid(wvalid), .wready(wready),
	.bid(bid), .bresp(bresp), .bvalid(bvalid), .bready(bready),
	.arid(arid), .araddr(araddr), .arlen(arlen), .arsize(arsize), .arburst(arburst),
	.arlock(arlock), .arcache(arcache), .arprot(arprot), .aruser(aruser), .arvalid(arvalid),
	.arready(arready),
	.rid(rid), .rdata(rdata), .rresp(rresp), .rlast(rlast), .rvalid(rvalid), .rready(rready)
);

hps_ocram_rx #(.ROWS(ROWS), .OSD_EN(OSD_B), .SEC_EN(SEC_B), .FRAME(SFRAME), .GAP(SGAP)) u_osd
(
	.clk(clk),
	.clk_sys(clk_sys),
	.gp_out25(gpo[25]),
	.s_hello(s_hello),
	.s_live(s_live),
	.s_tick(s_tick),
	.bind_en(osd_bind),
	.sec_ask(sec_ask),
	.evt_stop(evt_stop),
	.c_sfault(c0_sfault),
	.c_smid(c0_smid),
	.c_sbind(c0_sbind),
	.c_wide(c0_wide),
	.rq_post(rq_post),
	.rq_lba(rq_lba),
	.rq_start(rq_start),
	.c_rest(c0_rest),
	.c_fault(c0_fault),
	.c_mid(c0_mid),
	.c_open(c0_open),
	.osd_rows(osd_rows),
	.osd_deepest(osd_deepest),
	.c_req(ocram_c0_req),
	.c_we(ocram_c0_we),
	.c_off(ocram_c0_off),
	.c_wdata(ocram_c0_wdata),
	.c_grant(ocram_c0_grant),
	.c_done(ocram_c0_done),
	.c_ok(ocram_c0_ok),
	.c_rdata(ocram_c0_rdata),
	.sel(sel_m),
	.view(view_m),
	.we(ocram_we),
	.addr(ocram_addr),
	.dbyte(ocram_byte),
	.x_reset(x_reset), .x_hps_ack(x_hps_ack), .x_req(x_req), .x_lba(x_lba), .x_commit(x_commit), .x_wide(x_wide),
	.x_en(x_en), .x_ack(x_ack), .x_wr(x_wr), .x_dat(x_dat), .x_adr(x_adr)
);

// The two real osd.v instances: A only over SPI, B only from the OCRAM writer.
reg         io_osd_a = 1'b0, io_strobe_a = 1'b0;
reg  [15:0] io_din_a = 16'd0;

osd osd_spi
(
	.clk_sys(clk_sys),
	.io_osd(io_osd_a),
	.io_strobe(io_strobe_a),
	.io_din(io_din_a),
	.clk_video(clk_video),
	.din(24'd0), .de_in(1'b0), .vs_in(1'b0), .hs_in(1'b0),
	.dout(), .de_out(), .vs_out(), .hs_out(),
	.osd_status(),
	.lw_we(1'b0), .lw_addr(13'd0), .lw_byte(8'd0)
);

osd osd_ocr
(
	.clk_sys(clk_sys),
	.io_osd(1'b0), .io_strobe(1'b0), .io_din(16'd0),
	.clk_video(clk_video),
	.din(24'd0), .de_in(1'b0), .vs_in(1'b0), .hs_in(1'b0),
	.dout(), .de_out(), .vs_out(), .hs_out(),
	.osd_status(),
	.lw_we(ocram_we), .lw_addr(ocram_addr), .lw_byte(ocram_byte)
);

////////////////////////////////  the slave  ////////////////////////////////

// The 64 KB OCRAM, and the AXI3 slave of the fpga2hps bridge at 32 bits.
reg [31:0] mem [0:16383];
integer mi;
initial for (mi = 0; mi < 16384; mi = mi + 1) mem[mi] = 32'd0;

integer mode = M_NORMAL;
reg     fast_lat = 1'b0;               // the torture cases: a short latency and no ready delays
reg     inj_rd_en = 0, inj_wr_en = 0;
reg [15:0] inj_rd_addr = 0, inj_wr_addr = 0;
// the SECTOR cases' stalls: the next write of the data word stall_wd, or the next read of stall_ra, waits stall_n
reg     stall_w_en = 0, stall_r_en = 0, stall_on = 0;
reg [31:0] stall_wd = 0;
reg [15:0] stall_ra = 0;
integer stall_n = 3000;
// the transaction log: every completed transaction {we, off, data} while tl_on
reg     tl_on = 0;
integer tl_n = 0;
reg        tl_we  [0:4095];
reg [15:0] tl_off [0:4095];
reg [31:0] tl_dat [0:4095];

function integer lat_pick;
	integer r;
	begin
`ifdef OCRAM_TB_RATE
		if (1) lat_pick = 10; else
`endif
		if (fast_lat) lat_pick = 1;
		else begin
			r = ($random(seed) & 32'h7FFF_FFFF) % 1000;
			if (r < 50)        lat_pick = 1;
			else if (r < 950)  lat_pick = 20 + (($random(seed) & 32'h7FFF_FFFF) % 41);
			else               lat_pick = 200 + (($random(seed) & 32'h7FFF_FFFF) % 1801);
		end
	end
endfunction
function integer rdy_pick;
	begin
`ifdef OCRAM_TB_RATE
		if (1) rdy_pick = 0; else
`endif
		if (fast_lat) rdy_pick = 0;
		else rdy_pick = ($random(seed) & 32'h7FFF_FFFF) % 4;
	end
endfunction

// write channel
localparam [1:0] WS_IDLE = 0, WS_LAT = 1, WS_RSP = 2;
reg  [1:0]  wst = WS_IDLE;
reg         aw_seen = 0, w_seen = 0;
reg  [1:0]  awdly = 0, wdly = 0;
reg  [15:0] wlat = 0;
reg  [31:0] waddr_q = 0, wdata_q = 0;
reg  [2:0]  wprot_q = 0;
reg         werr_q = 0;
reg         bvalid_r = 0;
reg  [1:0]  bresp_r = 0;

assign awready = (mode != M_NEVER) && (wst == WS_IDLE) && !aw_seen && (awdly == 0);
assign wready  = (mode != M_NEVER) && (wst == WS_IDLE) && !w_seen && (wdly == 0);
assign bid     = 8'd0;
assign bresp   = bresp_r;
assign bvalid  = bvalid_r;

always @(posedge clk) begin
	if (!hps_rst_n) begin                 // the bridge resets with the HPS: its channels go idle
		wst <= WS_IDLE; aw_seen <= 1'b0; w_seen <= 1'b0;
		awdly <= 2'd0; wdly <= 2'd0; bvalid_r <= 1'b0;
	end
	else case (wst)
	WS_IDLE: begin
		bvalid_r <= 1'b0;
		if (awdly != 0) awdly <= awdly - 2'd1;
		if (wdly  != 0) wdly  <= wdly  - 2'd1;
		if (awvalid && awready) begin
			waddr_q <= awaddr;
			wprot_q <= awprot;
			aw_seen <= 1'b1;
			werr_q  <= (mode == M_SECURE_REFUSE && awprot[1] == 1'b0) ||
			           (inj_wr_en && (awaddr[15:0] == inj_wr_addr));
			if (inj_wr_en && (awaddr[15:0] == inj_wr_addr)) inj_wr_en <= 1'b0;
		end
		if (wvalid && wready) begin wdata_q <= wdata; w_seen <= 1'b1; end
		if ((aw_seen || (awvalid && awready)) && (w_seen || (wvalid && wready))) begin
			wst <= WS_LAT;
			wlat <= lat_pick();
			if (stall_w_en && (w_seen ? wdata_q : wdata) == stall_wd) begin
				wlat <= stall_n; stall_w_en <= 1'b0; stall_on <= 1'b1;
			end
			awdly <= rdy_pick();
			wdly  <= rdy_pick();
			aw_seen <= 1'b0;
			w_seen  <= 1'b0;
		end
	end
	WS_LAT: begin
		if (wlat == 0) begin
			wst <= WS_RSP;
			bvalid_r <= 1'b1;
			bresp_r  <= werr_q ? 2'b10 : 2'b00;
		end
		else begin wlat <= wlat - 16'd1; bvalid_r <= 1'b0; end
	end
	WS_RSP: begin
		if (bready) begin                  // the handshake ends it; a stale valid would be taken
			bvalid_r <= 1'b0;
			if (bresp_r == 2'b00) mem[waddr_q[15:2]] <= wdata_q;
			if (tl_on && tl_n < 4096) begin
				tl_we[tl_n] = 1'b1; tl_off[tl_n] = waddr_q[15:0]; tl_dat[tl_n] = wdata_q; tl_n = tl_n + 1;
			end
			stall_on <= 1'b0;
			wst <= WS_IDLE;
		end
		else bvalid_r <= 1'b1;             // hold with its payload until BREADY
	end
	default: wst <= WS_IDLE;
	endcase
end

// read channel
localparam [1:0] RS_IDLE = 0, RS_LAT = 1, RS_RSP = 2;
reg  [1:0]  rst = RS_IDLE;
reg  [1:0]  ardy = 0;
reg  [15:0] rlat = 0;
reg  [31:0] raddr_q = 0, rdata_r = 0;
reg         rerr_q = 0;
reg         rvalid_r = 0;
reg  [1:0]  rresp_r = 0;

assign arready = (mode != M_NEVER) && (rst == RS_IDLE) && (ardy == 0);
assign rid     = 8'd0;
assign rdata   = rdata_r;
assign rresp   = rresp_r;
assign rlast   = 1'b1;
assign rvalid  = rvalid_r;

always @(posedge clk) begin
	if (!hps_rst_n) begin                 // the bridge resets with the HPS: its channels go idle
		rst <= RS_IDLE; ardy <= 2'd0; rvalid_r <= 1'b0;
	end
	else case (rst)
	RS_IDLE: if (ardy != 0) begin ardy <= ardy - 2'd1; rvalid_r <= 1'b0; end
	else if (arvalid && arready) begin
		raddr_q <= araddr;
		rdata_r <= mem[araddr[15:2]];
		rerr_q  <= (mode == M_SECURE_REFUSE && arprot[1] == 1'b0) ||
		           (inj_rd_en && (araddr[15:0] == inj_rd_addr));
		if (inj_rd_en && (araddr[15:0] == inj_rd_addr)) inj_rd_en <= 1'b0;
		rlat <= lat_pick();
		if (stall_r_en && araddr[15:0] == stall_ra) begin rlat <= stall_n; stall_r_en <= 1'b0; stall_on <= 1'b1; end
		ardy <= rdy_pick();
		rst  <= RS_LAT;
	end
	RS_LAT: begin
		if (rlat == 0) begin rst <= RS_RSP; rvalid_r <= 1'b1; rresp_r <= rerr_q ? 2'b10 : 2'b00; end
		else begin rlat <= rlat - 16'd1; rvalid_r <= 1'b0; end
	end
	RS_RSP: begin
		if (rready) begin
			rvalid_r <= 1'b0; rst <= RS_IDLE; stall_on <= 1'b0;
			if (tl_on && tl_n < 4096) begin
				tl_we[tl_n] = 1'b0; tl_off[tl_n] = raddr_q[15:0]; tl_dat[tl_n] = rdata_r; tl_n = tl_n + 1;
			end
		end
		else rvalid_r <= 1'b1;
	end
	default: rst <= RS_IDLE;
	endcase
end

///////////////////////////////  the host  //////////////////////////////////

// The host's strongly-ordered peeks and pokes, and its GPO stores 60-600 ns apart on its own clock.
reg [15:0] host_tail = 16'd0;
integer    exp_ns = 0, exp_flood = 0;

task host_wait;
	begin
		repeat (4 + (($random(seed) & 32'h7FFF_FFFF) % 37)) @(posedge clk_host);
	end
endtask
task gpo_store(input [31:0] v);
	begin host_wait; gpo_reg = v; end
endtask
task poke(input [15:0] off, input [31:0] v);
	begin mem[off >> 2] = v; end
endtask
// The host's own tail store: the array and the monitor's idea of the host's tail move together.
task store_tail(input [15:0] t);
	begin mem[H_IDX3 >> 2] = {16'd0, t}; host_tail = t; end
endtask
task peek(input [15:0] off, output [31:0] v);
	begin v = mem[off >> 2]; end
endtask
function [31:0] evt_word(input [15:0] idx);
	evt_word = mem[(EVT_BASE + ((idx & (EVT_WORDS - 1)) << 2)) >> 2];
endfunction
function [31:0] v_of(input [31:0] g);
	v_of = g ^ {g[15:0], g[31:16]} ^ 32'h9E37_79B9;
endfunction

task zero_page;
	integer i;
	begin
		for (i = 0; i < 64; i = i + 1) mem[i] = 32'd0;
		host_tail = 16'd0;
	end
endtask

task reset_floor;
	begin
		mode = M_NORMAL;
		fast_lat = 1'b0;
		inj_rd_en = 0; inj_wr_en = 0;
		gpo_reg = GPO_IO;
		repeat (8) @(posedge clk);
		hps_rst_n = 0;
		repeat (12) @(posedge clk);
		hps_rst_n = 1;
		repeat (16) @(posedge clk);
		zero_page;
		exp_ns = 0; exp_flood = 0;
	end
endtask

task run_edge(input integer ns, input integer flood);
	reg [31:0] b;
	begin
		b = GPO_IO;
		if (ns)    b = b | 32'h0800_0000;
		if (flood) b = b | 32'h0400_0000;
		exp_ns = ns; exp_flood = flood;
		gpo_store(b);
		gpo_store(b | 32'h1000_0000);
	end
endtask

task wait_word(input [15:0] off, input [31:0] want, input integer us, output integer ok);
	integer n;
	reg [31:0] v;
	begin
		ok = 0;
		n = 0;
		while (!ok && n < us * 100) begin       // 1 us = 100 clk_100m cycles
			peek(off, v);
			if (v === want) ok = 1;
			else begin @(posedge clk); n = n + 1; end
		end
	end
endtask

task wait_armed(input integer us, output integer ok);
	begin wait_word(C_HELLO, C_HELLO_ARMED, us, ok); end
endtask
task wait_echo(input [15:0] nonce, input integer us, output integer ok);
	begin wait_word(C_HELLO, {16'h4F43, nonce}, us, ok); end
endtask
task wait_stopped(input integer us, output integer ok);
	begin wait_word(C_HELLO, C_HELLO_STOP, us, ok); end
endtask

task start_session(input integer ns, input integer flood, input integer line_en, input [15:0] nonce,
                   output integer ok);
	integer ok1, ok2;
	begin
		zero_page;
		poke(H_LINE, line_en ? 32'd8 : 32'd0);
		poke(H_IDX3, 32'd0);
		run_edge(ns, flood);
		wait_armed(1000, ok1);
		poke(H_HELLO, {16'h4F43, nonce});
		wait_echo(nonce, 1000, ok2);
		ok = ok1 && ok2;
		check($sformatf("session %04h: ARMED then the echo", nonce), ok, 1);
	end
endtask

////////////////////////////  the OSD models  ///////////////////////////////

reg [7:0] rowdata [0:4095];

task make_row(input integer row);
	integer j;
	begin
		for (j = 0; j < 256; j = j + 1)
			rowdata[row*256 + j] = ($random(seed) & 32'h7FFF_FFFF);
	end
endtask

task spi_word(input [15:0] w);
	begin
		@(negedge clk_sys); io_din_a = w; io_strobe_a = 1'b1;
		@(negedge clk_sys); io_strobe_a = 1'b0;
	end
endtask

// The SPI model's OSDCMDWRITE: 0x20 | row, then the row's 256 bytes, as osd.v's command protocol takes them.
task spi_row(input integer row);
	integer j;
	begin
		@(negedge clk_sys); io_osd_a = 1'b1;
		spi_word(16'h0020 | row[15:0]);
		for (j = 0; j < 256; j = j + 1) spi_word({8'd0, rowdata[row*256 + j]});
		@(negedge clk_sys); io_osd_a = 1'b0;
		@(negedge clk_sys);
	end
endtask

task osd_clear;
	integer j;
	begin
		for (j = 0; j < ROWS*256; j = j + 1) begin
			osd_spi.osd_buffer[j] = 8'd0;
			osd_ocr.osd_buffer[j] = 8'd0;
		end
	end
endtask

task osd_compare(output integer bad);
	integer j;
	begin
		bad = 0;
		for (j = 0; j < ROWS*256; j = j + 1)
			if (osd_spi.osd_buffer[j] !== osd_ocr.osd_buffer[j]) begin
				bad = bad + 1;
				if (bad <= 4)
					$display("FAIL osd compare at %0d: SPI %02h, OCRAM %02h", j,
					         osd_spi.osd_buffer[j], osd_ocr.osd_buffer[j]);
			end
	end
endtask

// The host's OSD ring: records at the free-running word index osd_head, one head store publishes them.
integer osd_head = 0;
task osd_ring_put(input [15:0] wi, input [31:0] v);
	begin poke(OSD_BASE + ((wi & (OSD_WORDS - 1)) << 2), v); end
endtask
task osd_hdr(input [7:0] kind, input [7:0] arg, input [15:0] len);
	begin
		osd_ring_put(osd_head[15:0], {kind, arg, len});
		osd_head = osd_head + 1;
	end
endtask
task osd_word(input [31:0] v);
	begin osd_ring_put(osd_head[15:0], v); osd_head = osd_head + 1; end
endtask
task osd_publish;
	begin poke(H_IDX2, {16'd0, osd_head[15:0]}); end
endtask
task osd_publish_row(input integer row);
	integer j;
	reg [31:0] w;
	begin
		osd_hdr(8'h01, row[7:0], 16'd64);
		for (j = 0; j < 64; j = j + 1) begin
			w = {rowdata[row*256 + 4*j + 3], rowdata[row*256 + 4*j + 2],
			     rowdata[row*256 + 4*j + 1], rowdata[row*256 + 4*j]};
			osd_word(w);
		end
	end
endtask

//////////////////////////////  the monitor  ////////////////////////////////

// Every invariant of §3 on every cycle, and the bench's own copy of the counters and the records.
integer cyc = 0;
integer mon_err = 0;

task merr(input string what);
	begin
		mon_err = mon_err + 1;
		if (mon_err <= 8) $display("   protocol monitor at %0t: %s", $time, what);
	end
endtask

function in_evt(input [15:0] a);
	in_evt = (a >= EVT_BASE) && (a <= EVT_BASE + EVT_WORDS * 4 - 1);
endfunction
function in_osd(input [15:0] a);
	in_osd = (a >= OSD_BASE) && (a <= OSD_BASE + OSD_WORDS * 4 - 1);
endfunction
function in_ts(input [15:0] a);
	in_ts = (a >= TS_BASE) && (a <= TS_BASE + TS_WORDS * 4 - 1);
endfunction

reg [4:0]  rst_grace = 5'd0;
reg        awv_d = 0, wv_d = 0, arv_d = 0, bv_d = 0, rv_d = 0;
reg        awr_d = 1, wr_d = 1, arr_d = 1;
reg [31:0] awaddr_s = 0, wdata_s = 0, araddr_s = 0;
reg  [3:0] awlen_s = 0, awcache_s = 0, wstrb_s = 0, arlen_s = 0, arcache_s = 0;
reg  [2:0] awsize_s = 0, awprot_s = 0, arsize_s = 0, arprot_s = 0;
reg  [1:0] awburst_s = 0, awlock_s = 0, arburst_s = 0, arlock_s = 0;
reg  [4:0] awuser_s = 0, aruser_s = 0;
reg  [7:0] awid_s = 0, wid_s = 0, arid_s = 0;
reg        wlast_s = 0;

reg        w_out = 0, r_out = 0;
reg        w_counted = 0, r_counted = 0;
reg [15:0] w_off_q = 0, r_off_q = 0;
reg [31:0] w_data_q = 0;
integer    w_t0 = 0, r_t0 = 0;

integer txn_starts = 0, txn_done = 0;
// Case 27's bound: the design note's "at most one transaction for each requester above". A step-4 EVT tail
// re-read launched while a consumer waits is a transaction that consumer sits behind; line_yield_q lets at most
// one of those through between two consumer grants. A no-consumer-request cycle clears the yield (and is no
// delay to anyone), so the counter lapses there too. line_rr_all proves the re-reads did happen.
integer rr_wait_since = 0;   // such re-reads since the last grant, in a run of continuously waiting consumers
integer rr_wait_max = 0;     // the longest such run between two consumer grants
integer grants_all = 0;      // the consumer grants seen (either port)
integer line_rr_all = 0;     // every step-4 EVT tail re-read
reg     first_arm = 0, first_seen = 0;
integer first_off = 0;
reg     first_we = 0;

// the monitor's counters (cleared at each hello, as the DIAG module's are)
integer m_rd = 0, m_wr = 0, m_rd_sum = 0, m_wr_sum = 0;
integer m_rd_max = 0, m_wr_max = 0, m_rd_min = 16'hFFFF, m_wr_min = 16'hFFFF;
integer m_rd_err = 0, m_wr_err = 0;
integer m_rd_hist [0:15];
integer m_wr_hist [0:15];
integer m_hello = 0;
integer m_term = 0;
integer m_hidx_reads = 0, m_hello_reads = 0;
integer m_deepest = 0;
integer rq_published = 0;    // REQUEST records whose head store was seen
integer tail_seen = 0;        // the master's last-read EVT tail, not the host's last store

// the EVT records
integer rec_writes = 0;       // words of the current record, the header included
integer rec_len_m = 0;
integer rec_kind_m = 0;
integer pub_words = 0;        // every EVT word published since the hello
integer pub_head = 0;         // the last C_IDX[3] store
integer mark_started = 0, mark_published = 0, verdict_started = 0, verdict_published = 0;
integer last_mark_start = 0, gap_min = 1000000, gap_max = 0, gap_n = 0;

// the hello's stores, in the order the master made them
integer hseq_n = 0;
reg [15:0] hseq_addr [0:31];
reg [31:0] hseq_data [0:31];
reg in_hello = 0;

// the snapshot and the records the host has consumed
reg     snap_arm = 0, snap_taken = 0, snap_done = 0;
integer snap_seq = 0, mark_consumed = 0;
integer s_rd = 0, s_wr = 0, s_rd_sum = 0, s_wr_sum = 0, s_hcnt = 0, s_deepest = 0;
integer s_rd_max = 0, s_wr_max = 0, s_rd_min = 0, s_wr_min = 0, s_rd_err = 0, s_wr_err = 0;
integer s_ns = 0, s_flood = 0;
integer s_rd_hist [0:15];
integer s_wr_hist [0:15];
reg [31:0] vd_last [0:7];
integer    rq_n = 0;              // REQUEST records the host has taken
reg [31:0] rq_lba_log [0:255];
integer vd_seen = 0;

function integer bin_of(input integer lat);
	begin
		if (lat < 64)        bin_of = lat / 8;
		else if (lat < 128)  bin_of = 8;
		else if (lat < 256)  bin_of = 9;
		else if (lat < 512)  bin_of = 10;
		else if (lat < 1024) bin_of = 11;
		else if (lat < 2048) bin_of = 12;
		else if (lat < 4096) bin_of = 13;
		else if (lat < 65536)bin_of = 14;
		else                 bin_of = 15;
	end
endfunction

function [31:0] snap_word(input integer i);
	begin
		snap_word = 32'd0;
		case (i)
		0:  snap_word = (s_hcnt << 24) | (s_flood << 17) | (s_ns << 16) | s_deepest;
		1:  snap_word = s_rd;
		2:  snap_word = s_wr;
		3:  snap_word = (s_rd_max << 16) | s_rd_min;
		4:  snap_word = (s_wr_max << 16) | s_wr_min;
		5:  snap_word = s_rd_sum;
		6:  snap_word = s_wr_sum;
		7:  snap_word = (s_rd_err << 16) | s_wr_err;
		default: begin
			if (i < 24) snap_word = s_rd_hist[i - 8];
			else        snap_word = s_wr_hist[i - 24];
		end
		endcase
	end
endfunction

task mon_stat_clear;
	begin
		gap_min = 1000000; gap_max = 0; gap_n = 0;
	end
endtask

always @(posedge clk) begin
	integer lat, b, gap, hi;
	cyc = cyc + 1;
	if (!hps_rst_n) begin
		rst_grace = 5'd16;
		w_out = 0; r_out = 0;
	end
	else if (rst_grace != 0) rst_grace = rst_grace - 1;

	// valid stability: while a valid is held without its ready, it and its payloads hold
	if (rst_grace == 0) begin
		if (awv_d && !awr_d && (!awvalid || awaddr !== awaddr_s || awlen !== awlen_s ||
		    awsize !== awsize_s || awburst !== awburst_s || awlock !== awlock_s ||
		    awcache !== awcache_s || awprot !== awprot_s || awuser !== awuser_s || awid !== awid_s))
			merr("AWVALID or its payload changed before AWREADY");
		if (wv_d && !wr_d && (!wvalid || wdata !== wdata_s || wstrb !== wstrb_s ||
		    wlast !== wlast_s || wid !== wid_s))
			merr("WVALID or its payload changed before WREADY");
		if (arv_d && !arr_d && (!arvalid || araddr !== araddr_s || arlen !== arlen_s ||
		    arsize !== arsize_s || arburst !== arburst_s || arlock !== arlock_s ||
		    arcache !== arcache_s || arprot !== arprot_s || aruser !== aruser_s || arid !== arid_s))
			merr("ARVALID or its payload changed before ARREADY");
		if (bvalid && !bready) merr("BVALID without BREADY");
		if (rvalid && !rready) merr("RVALID without RREADY");
	end
	if (awvalid && !awready) begin
		awaddr_s = awaddr; awlen_s = awlen; awsize_s = awsize; awburst_s = awburst;
		awlock_s = awlock; awcache_s = awcache; awprot_s = awprot; awuser_s = awuser; awid_s = awid;
	end
	if (wvalid && !wready) begin wdata_s = wdata; wstrb_s = wstrb; wlast_s = wlast; wid_s = wid; end
	if (arvalid && !arready) begin
		araddr_s = araddr; arlen_s = arlen; arsize_s = arsize; arburst_s = arburst;
		arlock_s = arlock; arcache_s = arcache; arprot_s = arprot; aruser_s = aruser; arid_s = arid;
	end
	// the constants of §3, on every cycle a valid is asserted
	if (awvalid) begin
		if (awlen !== 4'd0 || awsize !== 3'b010 || awburst !== 2'b01 || awlock !== 2'b00 ||
		    awcache !== 4'b0000 || awuser !== 5'd0 || awid !== 8'd0) merr("a write address constant");
		if (awaddr[31:16] !== 16'hFFFF) merr("AWADDR's upper half");
		if (!((awaddr[15:0] >= 16'h0080 && awaddr[15:0] <= 16'h00FF) || in_evt(awaddr[15:0])))
			merr("AWADDR outside the fabric half and the EVT span");
		if (awprot !== {1'b0, exp_ns[0], 1'b0}) merr("AWPROT");
	end
	if (wvalid) begin
		if (wstrb !== 4'hF || wlast !== 1'b1 || wid !== 8'd0) merr("a write data constant");
	end
	if (arvalid) begin
		if (arlen !== 4'd0 || arsize !== 3'b010 || arburst !== 2'b01 || arlock !== 2'b00 ||
		    arcache !== 4'b0000 || aruser !== 5'd0 || arid !== 8'd0) merr("a read address constant");
		if (araddr[31:16] !== 16'hFFFF) merr("ARADDR's upper half");
		if (!(araddr[15:0] === H_HELLO || araddr[15:0] === H_BIND || araddr[15:0] === H_LINE ||
		      araddr[15:0] === H_IDX2 || araddr[15:0] === H_IDX3 || araddr[15:0] === H_IDX4 ||
		      (SEC_B && araddr[15:0] === H_SECTOR) ||
		      in_osd(araddr[15:0]) || in_ts(araddr[15:0])))
			merr("ARADDR outside the control page's host half and the ring spans");
		if (arprot !== {1'b0, exp_ns[0], 1'b0}) merr("ARPROT");
	end

	// transaction starts and completions
	if (awvalid && !awv_d) begin
		if (w_out || r_out) merr("a write starts while a transaction is in flight");
		txn_starts = txn_starts + 1;
		w_out = 1;
		w_off_q = awaddr[15:0];
		w_data_q = wdata;
		w_t0 = cyc;
		w_counted = !in_evt(awaddr[15:0]) &&
		            !((awaddr[15:0] == C_HELLO) && (wdata == C_HELLO_STOP));
		if (first_arm && !first_seen) begin
			first_off = awaddr[15:0]; first_we = 1; first_seen = 1;
		end
	end
	if (arvalid && !arv_d) begin
		if (w_out || r_out) merr("a read starts while a transaction is in flight");
		txn_starts = txn_starts + 1;
		r_out = 1;
		r_off_q = araddr[15:0];
		r_t0 = cyc;
		r_counted = 1;
		if (first_arm && !first_seen) begin
			first_off = araddr[15:0]; first_we = 0; first_seen = 1;
		end
	end
	if (bvalid && bready && w_out) begin
		w_out = 0;
		txn_done = txn_done + 1;
		if (w_counted) begin
			lat = cyc - w_t0;
			m_wr = m_wr + 1;
			m_wr_sum = m_wr_sum + lat;
			b = (lat > 65535) ? 65535 : lat;
			if (b > m_wr_max) m_wr_max = b;
			if (b < m_wr_min) m_wr_min = b;
			if (bresp != 2'b00 && m_wr_err < 65535) m_wr_err = m_wr_err + 1;
			m_wr_hist[bin_of(lat)] = m_wr_hist[bin_of(lat)] + 1;
		end
		if (w_off_q == C_HELLO && w_data_q == C_HELLO_STOP) m_term = m_term + 1;
		if (w_off_q == C_HELLO && w_data_q[31:16] == 16'h4F43 && w_data_q[15:0] != 16'd0)
			m_hello = m_hello + 1;
	end
	if (rvalid && rready && r_out) begin
		r_out = 0;
		txn_done = txn_done + 1;
		if (r_off_q == H_IDX3) begin
			m_hidx_reads = m_hidx_reads + 1;
			tail_seen = rdata[15:0];
			b = (pub_head - tail_seen) & 16'hFFFF;
			if (b <= EVT_WORDS && b > m_deepest) m_deepest = b;
		end
		if (r_off_q == H_HELLO) m_hello_reads = m_hello_reads + 1;
		if (r_counted) begin
			lat = cyc - r_t0;
			m_rd = m_rd + 1;
			m_rd_sum = m_rd_sum + lat;
			b = (lat > 65535) ? 65535 : lat;
			if (b > m_rd_max) m_rd_max = b;
			if (b < m_rd_min) m_rd_min = b;
			if (rresp != 2'b00 && m_rd_err < 65535) m_rd_err = m_rd_err + 1;
			m_rd_hist[bin_of(lat)] = m_rd_hist[bin_of(lat)] + 1;
		end
	end

	// the hello's stores, in order: from the C_VERSION store to the C_HELLO echo
	if (awvalid && !awv_d && awaddr[15:0] >= 16'h0080 && awaddr[15:0] <= 16'h00FF) begin
		if (!in_hello && awaddr[15:0] == C_VERSION && wdata == C_VERSION_1_0) begin
			in_hello = 1; hseq_n = 0;
		end
		if (in_hello) begin
			hseq_addr[hseq_n] = awaddr[15:0];
			hseq_data[hseq_n] = wdata;
			if (hseq_n < 31) hseq_n = hseq_n + 1;
			if (awaddr[15:0] == C_HELLO) in_hello = 0;
		end
	end

	// the DIAG counters clear at a hello: the C_VERSION store is its first transaction
	if (awvalid && !awv_d && awaddr[15:0] == C_VERSION && wdata == C_VERSION_1_0) begin
		m_rd = 0; m_wr = 0; m_rd_sum = 0; m_wr_sum = 0;
		m_rd_max = 0; m_wr_max = 0; m_rd_min = 16'hFFFF; m_wr_min = 16'hFFFF;
		m_rd_err = 0; m_wr_err = 0;
		for (b = 0; b < 16; b = b + 1) begin m_rd_hist[b] = 0; m_wr_hist[b] = 0; end
		m_deepest = 0; tail_seen = 0;
		pub_words = 0; pub_head = 0; rec_writes = 0;
		mark_started = 0; mark_published = 0; verdict_started = 0; verdict_published = 0;
		mark_consumed = 0;
	end

	// the event-ring accounting from the stores the master makes
	if (awvalid && !awv_d) begin
		if (in_evt(awaddr[15:0])) begin
			// this record's next 16-bit index; every EVT write is in the record being written, in its place,
			// and never over a word the host has not consumed
			b = (pub_words + rec_writes) & 16'hFFFF;
			if (rec_writes == 0) begin
				rec_len_m = wdata[15:0];
				rec_kind_m = wdata[31:24];
				if (rec_kind_m == 8'h23) begin
					if (wdata !== MARK_HEADER) merr("a MARK header word");
					if (mark_started > 0) begin
						gap = cyc - last_mark_start;
						if (gap < gap_min) gap_min = gap;
						if (gap > gap_max) gap_max = gap;
						gap_n = gap_n + 1;
					end
					last_mark_start = cyc;
					if (snap_arm) begin
						s_rd = m_rd; s_wr = m_wr; s_rd_sum = m_rd_sum; s_wr_sum = m_wr_sum;
						s_rd_max = m_rd_max; s_wr_max = m_wr_max;
						s_rd_min = m_rd_min; s_wr_min = m_wr_min;
						s_rd_err = m_rd_err; s_wr_err = m_wr_err;
						for (hi = 0; hi < 16; hi = hi + 1) begin
							s_rd_hist[hi] = m_rd_hist[hi]; s_wr_hist[hi] = m_wr_hist[hi];
						end
						s_hcnt = m_hello; s_deepest = m_deepest;
						s_ns = exp_ns[0]; s_flood = exp_flood[0];
						snap_seq = mark_started;
						snap_taken = 1;
						snap_arm = 0;
					end
					mark_started = mark_started + 1;
				end else if (rec_kind_m == 8'h25) begin
					if (wdata !== VERDICT_HEADER) merr("a VERDICT header word");
					verdict_started = verdict_started + 1;
				end else if (SEC_B && rec_kind_m == 8'h21) begin
					if (wdata !== REQ_HEADER) merr("a REQUEST header word");
				end else merr("an EVT header of an unknown kind");
			end else if (rec_writes > rec_len_m) begin
				merr("an EVT record longer than its header's length");
			end
			if (((awaddr[15:2] - (EVT_BASE >> 2)) & (EVT_WORDS - 1)) !== (b & (EVT_WORDS - 1)))
				merr("an EVT word outside its record's place");
			if (((b - host_tail) & 16'hFFFF) < ((pub_words - host_tail) & 16'hFFFF) &&
			    ((pub_words - host_tail) & 16'hFFFF) <= EVT_WORDS)
				merr("an EVT word stored over one the host has not consumed");
			rec_writes = rec_writes + 1;
		end
		if (awaddr[15:0] == C_IDX3) begin
			if (rec_writes > 0) begin
				if (rec_writes !== 1 + rec_len_m) merr("the head stored before the record's last word");
				if (wdata[15:0] !== ((pub_words + rec_writes) & 16'hFFFF)) merr("the published head");
				pub_head = wdata[15:0];
				pub_words = pub_words + rec_writes;
				if (rec_kind_m == 8'h23) mark_published = mark_published + 1;
				else if (rec_kind_m == 8'h21) rq_published = rq_published + 1;
				else verdict_published = verdict_published + 1;
				b = (pub_head - (tail_seen & 16'hFFFF)) & 16'hFFFF;
				if (b <= EVT_WORDS && b > m_deepest) m_deepest = b;
				rec_writes = 0;
			end else if (wdata == 32'd0) begin
				pub_words = 0; pub_head = 0;       // the hello's C_IDX[3] = 0
			end else merr("a C_IDX3 store with a half-written record");
		end
	end

	// the line's back-to-back tail re-read and the consumers' turns: between two consumer grants at most one
	// step-4 re-read is launched (line_yield_q), so a consumer waits at most one transaction for the line
	if (arvalid && !arv_d && (araddr[15:0] === H_IDX3) && (dut.poll_step == 3'd4)) begin
		line_rr_all = line_rr_all + 1;
		if (dut.consumer_req) rr_wait_since = rr_wait_since + 1;
	end
	if (dut.c0_go || dut.c1_go) begin
		if (grants_all > 0 && rr_wait_since > rr_wait_max) rr_wait_max = rr_wait_since;
		rr_wait_since = 0;
		grants_all = grants_all + 1;
	end
	else if (!dut.consumer_req) rr_wait_since = 0;

	// the edge detectors, for the next cycle's rise and stability checks
	awv_d = awvalid; awr_d = awready; wv_d = wvalid; wr_d = wready;
	arv_d = arvalid; arr_d = arready; bv_d = bvalid; rv_d = rvalid;
end

// The host's EVT drain: one whole record if one is published, its header checked, a VERDICT's payload kept in
// vd_last, a MARK's snapshot compared against the monitor's numbers.
task evt_take(output integer got);
	integer i, h, len, kind;
	reg [31:0] v, hdr;
	begin
		got = 0;
		peek(C_IDX3, v);
		h = v[15:0];
		if (((h - host_tail) & 16'hFFFF) >= 1) begin
			hdr = evt_word(host_tail);
			len = hdr[15:0];
			kind = hdr[31:24];
			if (((h - host_tail) & 16'hFFFF) >= 1 + len) begin
				if (kind == 8'h23) begin
					check("MARK record: header", hdr, MARK_HEADER);
					if (snap_taken && (mark_consumed == snap_seq)) begin
						for (i = 1; i <= 40; i = i + 1)
							check($sformatf("MARK record: payload word %0d against the monitor", i - 1),
							      evt_word(host_tail + i[15:0]), snap_word(i - 1));
						snap_taken = 0; snap_done = 1;
					end
					mark_consumed = mark_consumed + 1;
				end else if (kind == 8'h25) begin
					check("VERDICT record: header", hdr, VERDICT_HEADER);
					for (i = 0; i < 8; i = i + 1)
						vd_last[i] = evt_word(host_tail + 1 + i[15:0]);
					vd_seen = vd_seen + 1;
				end else if (SEC_B && kind == 8'h21) begin
					check("REQUEST record: header", hdr, REQ_HEADER);
					if (rq_n < 256) rq_lba_log[rq_n] = evt_word(host_tail + 16'd1);
					rq_n = rq_n + 1;
				end else begin
					check("EVT record: a known kind", kind, 8'h23);
				end
				host_tail = host_tail + 1 + len;
				poke(H_IDX3, {16'd0, host_tail});
				got = 1;
			end
		end
	end
endtask

task consume_record(output integer ok);
	integer t;
	begin
		ok = 0; t = 0;
		while (!ok && t < 200) begin
			evt_take(ok);
			if (!ok) begin repeat (5) @(posedge clk); t = t + 1; end
		end
	end
endtask

task wait_verdict(input integer recs, input integer orows, input integer us, output integer ok);
	integer t, v0, got;
	begin
		ok = 0; t = 0; v0 = vd_seen;
		while (!ok && t < us * 100) begin
			evt_take(got);
			if (got) begin
				if (vd_seen != v0 && vd_last[0] == recs && vd_last[7][31:16] == orows) ok = 1;
				v0 = vd_seen;
			end else begin @(posedge clk); t = t + 1; end
		end
	end
endtask

///////////////////////////  the torture model  /////////////////////////////

// The host's torture ring: records at the free-running word index ts_head. The expected counters are the host's
// own, advanced as it stores; the verdict's snapshot must match them once the traffic and the drain settle.
integer    ts_head = 0;
integer    ts_g = 0;
integer    ts_recs = 0;
integer    ts_words_exp = 0;
reg [31:0] ts_sum_exp = 32'd0;

task ts_reset;
	begin
		ts_head = 0; ts_g = 0; ts_recs = 0; ts_words_exp = 0; ts_sum_exp = 32'd0;
	end
endtask
task ts_put(input [31:0] v);
	begin
		poke(TS_BASE + ((ts_head & (TS_WORDS - 1)) << 2), v);
		ts_head = ts_head + 1;
	end
endtask

// One TSEQ record: {0x50, seq, len}, then len words v(g); bad_idx selects a word to store as bad_val (or -1),
// wrong_seq stores the next record number (the check still advances the expected one).
task ts_record(input integer len, input integer bad_idx, input [31:0] bad_val, input integer wrong_seq);
	integer j;
	reg [31:0] w;
	reg  [7:0] seq;
	begin
		seq = ts_recs[7:0];
		if (wrong_seq) seq = seq + 8'd1;
		ts_put({8'h50, seq, len[15:0]});
		for (j = 0; j < len; j = j + 1) begin
			if (j == bad_idx) w = bad_val;
			else              w = v_of(ts_g[31:0]);
			ts_put(w);
			ts_sum_exp = ts_sum_exp + w;
			ts_words_exp = ts_words_exp + 1;
			ts_g = ts_g + 1;
		end
		ts_recs = ts_recs + 1;
		poke(H_IDX4, {16'd0, ts_head & 16'hFFFF});
	end
endtask

task ts_wait_room(input integer need);
	integer t, got;
	reg [31:0] v;
	begin
		t = 0;
		peek(C_IDX4, v);
		while ((((ts_head - v[15:0]) & 16'hFFFF) > (TS_WORDS - need)) && t < 2000000) begin
			evt_take(got);                       // keep the MARK and VERDICT records drained
			@(posedge clk); t = t + 1;
			peek(C_IDX4, v);
		end
	end
endtask

task wait_ts_drain(input integer us, output integer ok);
	integer t, got;
	reg [31:0] v;
	begin
		ok = 0; t = 0;
		while (!ok && t < us * 100) begin
			peek(C_IDX4, v);
			if (v[15:0] == (ts_head & 16'hFFFF)) ok = 1;
			else begin
				evt_take(got);                   // the host keeps its EVT ring drained
				@(posedge clk); t = t + 1;
			end
		end
	end
endtask

// Case 27's host leaves the EVT ring undrained: these wait on the torture ring only.
task ts_wait_room_nd(input integer need);
	integer t;
	reg [31:0] v;
	begin
		t = 0;
		peek(C_IDX4, v);
		while ((((ts_head - v[15:0]) & 16'hFFFF) > (TS_WORDS - need)) && t < 2000000) begin
			@(posedge clk); t = t + 1;
			peek(C_IDX4, v);
		end
	end
endtask

task wait_ts_drain_nd(input integer us, output integer ok);
	integer t;
	reg [31:0] v;
	begin
		ok = 0; t = 0;
		while (!ok && t < us * 100) begin
			peek(C_IDX4, v);
			if (v[15:0] == (ts_head & 16'hFFFF)) ok = 1;
			else begin @(posedge clk); t = t + 1; end
		end
	end
endtask

///////////////////////////////  the cases  /////////////////////////////////

integer txn0, term0, w0, p0, h0, v0, ok, ok1, ok2, i, n, bad;
integer r, j, len, recs0;
reg [31:0] v;

task wait_until_pub(input integer n, output integer ok);
	integer c;
	begin
		ok = 0; c = 0;
		while (!ok && c < 500000) begin          // 5 ms
			if ((mark_published - p0) >= n) ok = 1;
			else begin @(posedge clk); c = c + 1; end
		end
	end
endtask
task wait_until_start(input integer n, output integer ok);
	integer c;
	begin
		ok = 0; c = 0;
		while (!ok && c < 500000) begin          // 5 ms
			if ((mark_started - h0) >= n) ok = 1;
			else begin @(posedge clk); c = c + 1; end
		end
	end
endtask

// 1. stock host
task case1;
	begin
		$display("-- stock host: random GPO traffic on every bit but 28, long: zero transactions");
		reset_floor;
		txn0 = txn_starts;
		for (i = 0; i < 200; i = i + 1) gpo_store(($random(seed) & 32'h7FFF_FFFF) & 32'hEFFF_FFFF);
		repeat (200) @(posedge clk);
		check("stock host: transactions", txn_starts - txn0, 0);
		check("stock host: present stays 1", present_wire, 1);
	end
endtask

// 2. the hello
task case2;
	begin
		$display("-- the hello: ARMED, the echo and every C_ word; the count is 1, then 2 in a new run");
		reset_floor;
		first_arm = 1; first_seen = 0;
		run_edge(0, 0);
		wait_armed(1000, ok);
		check("ARMED within 1 ms of the edge", ok, 1);
		check("the run's first transaction reads H_HELLO", (first_seen && first_off == H_HELLO && !first_we), 1);
		first_arm = 0;
		peek(C_HELLO, v);   check("C_HELLO ARMED", v, C_HELLO_ARMED);
		peek(C_VERSION, v); check("no C_VERSION before the hello", v, 32'd0);
		peek(C_CAPS, v);    check("no C_CAPS before the hello", v, 32'd0);
		poke(H_HELLO, {16'h4F43, 16'h1234});
		wait_echo(16'h1234, 1000, ok);
		check("the echo within 1 ms", ok, 1);
		peek(C_HELLO, v);   check("C_HELLO is the echo", v, {16'h4F43, 16'h1234});
		peek(C_VERSION, v); check("C_VERSION 1.0", v, C_VERSION_1_0);
		peek(C_CAPS, v);    check("C_CAPS: the built rings", v, C_CAPS_EXP);
		peek(C_STATE, v);   check("C_STATE: one hello, no fault", v, 32'd1);
		peek(C_IDX3, v);    check("C_IDX[3] = 0", v, 32'd0);
		peek(C_BIND, v);    check("C_BIND = 0", v, 32'd0);
		peek(C_RING3, v);   check("C_RING[3] descriptor", v, C_RING3_EVT);
`ifdef MISTER_OCRAM_OSD
		peek(C_IDX2, v);    check("C_IDX[2] = 0", v, 32'd0);
		peek(C_RING2, v);   check("C_RING[2] descriptor", v, C_RING2_OSD);
		peek(C_K8, v);      check("C_K[8] descriptor", v, C_K8_EXP);
`endif
`ifdef MISTER_OCRAM_DIAG
		peek(C_IDX4, v);    check("C_IDX[4] = 0", v, 32'd0);
		peek(C_RING4, v);   check("C_RING[4] descriptor", v, C_RING4_TS);
`endif
`ifdef MISTER_OCRAM_SECTOR_SLOT
		peek(C_IDX4, v);    check("C_IDX[4] = 0", v, 32'd0);
		peek(C_RING4, v);   check("C_RING[4] descriptor (SECTOR)", v, C_RING4_SEC);
		peek(16'h00D0, v);  check("C_K[0] {0, gap}", v, C_K0_EXP);
		peek(16'h00D4, v);  check("C_K[1] {BLKSZ, wide}", v, C_K1_EXP);
		peek(16'h00D8, v);  check("C_K[2] one descriptor", v, 32'd1);
		peek(16'h00E0, v);  check("C_K[4] {RD, slot, frame}", v, C_K4_EXP);
		peek(16'h00E4, v);  check("C_K[5] 32 LBA bits", v, 32'd32);
		peek(16'h00E8, v);  check("C_K[6] not stored", v, 32'd0);
`else
		peek(16'h00D0, v);  check("C_K[0] not stored", v, 32'd0);
		peek(16'h00E4, v);  check("the other C_RING words not stored", v, 32'd0);
`endif
		gpo_store(GPO_IO);
		wait_stopped(1000, ok);
		check("a stop stores the terminal value", ok, 1);
		reset_floor;
		zero_page;
		poke(H_LINE, 32'd0); poke(H_IDX3, 32'd0);
		run_edge(0, 0); wait_armed(1000, ok);
		poke(H_HELLO, {16'h4F43, 16'h2345});
		wait_echo(16'h2345, 1000, ok);
		check("a second hello in a new run", ok, 1);
		peek(C_STATE, v); check("C_STATE: two hellos", v, 32'd2);
	end
endtask

// 3. a stale page
task case3;
	begin
		$display("-- a stale hello on the page: WAITING, never an echo, until H_HELLO reads 0");
		reset_floor;
		poke(H_HELLO, {16'h4F43, 16'h0077});
		run_edge(0, 0);
		wait_word(C_HELLO, C_HELLO_WAIT, 1000, ok);
		check("a stale page stores WAITING", ok, 1);
		repeat (3 << POLLC) @(posedge clk);
		peek(C_HELLO, v);   check("never an echo: C_HELLO", v, C_HELLO_WAIT);
		peek(C_VERSION, v); check("never a hello: C_VERSION", v, 32'd0);
		poke(H_HELLO, 32'd0);
		wait_armed(1000, ok);
		check("H_HELLO = 0 then ARMED", ok, 1);
	end
endtask

// 4. an edge, not a level
task case4;
	begin
		$display("-- edge, not level: RUN held across a bit-31 fall and return; a core-reset store");
		reset_floor;
		start_session(0, 0, 0, 16'h0101, ok);
		gpo_store(32'h1000_0000);                 // bit 31 low, RUN still 1
		wait_stopped(1000, ok);
		check("the bit-31 fall stores the terminal value", ok, 1);
		txn0 = txn_starts;
		gpo_store(GPO_RUN);                       // bit 31 back, RUN still 1
		repeat (4000) @(posedge clk);
		check("bit 31's return starts no transaction", txn_starts - txn0, 0);
		peek(C_HELLO, v); check("C_HELLO stays stopped", v, C_HELLO_STOP);
		reset_floor;
		start_session(0, 0, 0, 16'h0102, ok);
		gpo_store(32'h0000_0000);                 // a core-reset store: bits 31 and 28 both 0
		wait_stopped(1000, ok);
		check("the core-reset store stores the terminal value", ok, 1);
	end
endtask

`ifdef MISTER_OCRAM_DIAG
// 5. the stop
task case5;
	integer mid;
	begin
		$display("-- the stop: the in-flight transaction completes, one terminal store, no head, bit 19 next");
		reset_floor;
		start_session(0, 1, 8, 16'h0201, ok);
		p0 = pub_words;  h0 = mark_published;
		term0 = m_term;
		// stop with a record part-written
		mid = 0;
		i = 0;
		while (!mid && i < 40000) begin
			@(posedge clk); i = i + 1;
			if (rec_writes > 0 && rec_writes < 1 + rec_len_m) mid = 1;
		end
		check("a record is part-written when RUN falls", mid, 1);
		gpo_store(GPO_IO);
		wait_stopped(1000, ok);
		check("the stop stores the terminal value", ok, 1);
		check("exactly one terminal store", m_term - term0, 1);
		peek(C_IDX3, v);
		check("the abandoned record's head was not stored", v[15:0], (pub_head & 16'hFFFF));
		check("the published head is the complete records'", pub_head & 16'hFFFF, (pub_words & 16'hFFFF));
		txn0 = txn_starts;
		repeat (4000) @(posedge clk);
		check("nothing after the terminal store", txn_starts - txn0, 0);
		reset_floor;
		start_session(0, 0, 0, 16'h0202, ok);
		peek(C_STATE, v);
		check("the next session's C_STATE bit 19", v[19], 1);
	end
endtask
`endif

// 6. the HPS reset
task case6;
	begin
		$display("-- HPS reset: no terminal, no start while it is seen, an unanswered transaction dropped");
		reset_floor;
		start_session(0, 1, 0, 16'h0301, ok);
		term0 = m_term;
		repeat (30) @(posedge clk);
		hps_rst_n = 0;
		wait (dut.hrst_q === 1'b1);
		repeat (8) @(posedge clk);
		txn0 = txn_starts;
		repeat (60) @(posedge clk);
		check("nothing starts while the HPS reset is seen", txn_starts - txn0, 0);
		check("no terminal store while it is high", m_term - term0, 0);
		hps_rst_n = 1;
		repeat (16) @(posedge clk);
		check("no terminal store after it", m_term - term0, 0);
		reset_floor;
		zero_page;
		poke(H_LINE, 32'd0); poke(H_IDX3, 32'd0);
		run_edge(0, 0); wait_armed(1000, ok);
		check("a new RUN edge after the reset works", ok, 1);
		$display("-- HPS reset with a transaction never answered: it is dropped at the reset's fall");
		reset_floor;
		mode = M_NEVER;
		zero_page;
		run_edge(0, 0);
		ok = 0; i = 0;
		while (!ok && i < 10000) begin @(posedge clk); i = i + 1; if (dut.arvalid === 1'b1) ok = 1; end
		check("the first read hangs", ok, 1);
		term0 = m_term;
		hps_rst_n = 0;
		wait (dut.hrst_q === 1'b1);
		repeat (8) @(posedge clk);
		hps_rst_n = 1;
		repeat (16) @(posedge clk);
		check("the reset drops the unanswered read", dut.arvalid, 0);
		check("no terminal store after the reset", m_term - term0, 0);
		mode = M_NORMAL;
	end
endtask

// 7. a 60 ns 0 store
task case7;
	begin
		$display("-- a 0 lasting one host store (60 ns): the latch sets");
		reset_floor;
		zero_page;
		poke(H_LINE, 32'd0); poke(H_IDX3, 32'd0);
		run_edge(0, 0); wait_armed(1000, ok);
		term0 = m_term;
		#60; gpo_reg = GPO_IO;       // RUN 0 for 60 ns
		#60; gpo_reg = GPO_RUN;      // RUN 1: the edge
		ok = 0; i = 0;
		while (!ok && i < 100000) begin            // 1 ms
			@(posedge clk); i = i + 1;
			if ((m_term - term0) == 1) ok = 1;
		end
		check("the 60 ns 0 store was seen: the terminal store", ok, 1);
		wait_armed(1000, ok);
		check("the new RUN edge arms again", ok, 1);
	end
endtask

`ifdef MISTER_OCRAM_DIAG
// 8. the EVT ring: flood off, then flood on
task case8;
	integer wraps0, t0;
	begin
		$display("-- EVT: well-formed MARK records, published after their payload, never over the tail");
		$display("   flood off: one MARK per period");
		reset_floor;
		start_session(0, 0, 0, 16'h0401, ok);
		h0 = mark_started;
		mon_stat_clear;
		wait_until_start(3, ok);
		check("three MARK records start", ok, 1);
		check("flood off: MARK start-to-start gaps at least 2^MARK cycles",
		      (gap_n >= 2 && gap_min >= (1 << MARKC) - 2), 1);
		$display("-- EVT: flood on, the host drains; two wraps of the 2,048-word ring");
		reset_floor;
		start_session(0, 1, 0, 16'h0402, ok);
		wraps0 = pub_words;
		mon_stat_clear;
		ok = 0; i = 0;
		while (!ok && i < 500000) begin          // 5 ms
			consume_record(ok1);
			i = i + 1;
			if ((pub_words - wraps0) >= 2 * EVT_WORDS) ok = 1;
		end
		check("the ring wraps at least twice", ok, 1);
		check("flood on: the MARK records run back to back", (gap_n > 3 && gap_min < (1 << MARKC)), 1);
		h0 = mark_started;                        // every MARK started up to this instant is published
		ok = 0; i = 0;
		while (!ok && i < 200000) begin
			consume_record(ok1);
			i = i + 1;
			if (mark_published >= h0) ok = 1;
		end
		check("every started MARK is published", ok, 1);
		check("no word was stored over an unconsumed one", mon_err, 0);
	end
endtask

// 9. the line
task case9;
	integer c0, b;
	begin
		$display("-- the line: H_LINE bit 3 high, records unread: the tail is re-read back to back");
		reset_floor;
		start_session(0, 0, 8, 16'h0501, ok);
		p0 = mark_published;
		wait_until_pub(1, ok);
		ok1 = 0; i = 0;
		while (!ok1 && i < 100000) begin @(posedge clk); i = i + 1; if (line_out === 1'b1) ok1 = 1; end
		check("the line rises with records unread", ok1, 1);
		// 16 poll periods: one 200-2000-cycle transaction of the model's 5 % tail is not enough to stall the count
		h0 = m_hidx_reads; v0 = m_hello_reads;
		repeat (16 << POLLC) @(posedge clk);
		check("the tail is re-read more often than it is polled", (m_hidx_reads - h0) > (m_hello_reads - v0), 1);
		repeat (40) @(posedge clk);
		peek(C_IDX3, v);
		c0 = txn_done;
		store_tail(v[15:0]);
		@(negedge clk); c0 = txn_starts;          // transactions issued after the store
		b = 0;
		while (line_out === 1'b1 && b < 200) begin @(negedge clk); b = txn_starts - c0; end
		check("the line falls within 4 transactions of the tail store", (line_out === 1'b0 && b <= 4), 1);
		$display("-- the line: H_LINE 0: never high");
		reset_floor;
		start_session(0, 1, 0, 16'h0502, ok);
		p0 = mark_published;
		wait_until_pub(1, ok);
		bad = 0;
		for (i = 0; i < 2000; i = i + 1) begin @(posedge clk); if (line_out === 1'b1) bad = bad + 1; end
		check("H_LINE 0: cycles with the line high", bad, 0);
	end
endtask

// 10. a host tail beyond the ring
task case10;
	begin
		$display("-- a tail beyond the ring: C_STATE bit 11, no further record, the line low, until a hello");
		reset_floor;
		start_session(0, 1, 8, 16'h0601, ok);
		p0 = mark_published;
		wait_until_pub(2, ok);
		repeat (40) @(posedge clk);
		peek(C_IDX3, v);
		store_tail(v[15:0] - 16'd2049);
		ok = 0; i = 0;
		while (!ok && i < 20000) begin
			@(posedge clk); i = i + 1;
			peek(C_STATE, v);
			if (v[11] === 1'b1) ok = 1;
		end
		check("C_STATE bit 11 is stored", ok, 1);
		check("the line is low", line_out, 0);
		w0 = pub_words;
		repeat (4 << POLLC) @(posedge clk);
		check("no further record is published", pub_words - w0, 0);
		// a new hello clears the fault
		poke(H_HELLO, 32'd0);
		wait_armed(1000, ok);
		store_tail(16'd0);
		poke(H_LINE, 32'd8);
		poke(H_HELLO, {16'h4F43, 16'h0602});
		wait_echo(16'h0602, 1000, ok);
		peek(C_STATE, v);
		check("the new hello clears the fault bit", v[11], 0);
		p0 = mark_published;
		wait_until_pub(1, ok);
		check("records resume after the new hello", ok, 1);
	end
endtask

// 12. the counters against the monitor
task case12;
	integer snap_seen, t0;
	begin
		$display("-- the counters: the next MARK's 40 words equal the monitor's numbers exactly");
		reset_floor;
		zero_page;
		poke(H_LINE, 32'd8); poke(H_IDX3, 32'd0);
		inj_wr_addr = C_CAPS; inj_wr_en = 1;
		inj_rd_addr = H_LINE; inj_rd_en = 1;
		run_edge(1, 0);
		wait_armed(1000, ok);
		poke(H_HELLO, {16'h4F43, 16'h0801});
		wait_echo(16'h0801, 1000, ok);
		ok = 0; i = 0;
		while (!ok && i < 200000) begin
			@(posedge clk); i = i + 1;
			if (m_rd_err >= 1 && m_wr_err >= 1) ok = 1;
		end
		check("the injected read and write errors are counted", ok, 1);
		snap_arm = 1; snap_taken = 0; snap_done = 0;
		ok = 0; i = 0;
		while (!ok && i < 20000) begin
			consume_record(ok1);
			i = i + 1;
			if (snap_done) ok = 1;
		end
		check("the snapshot record is published and its 40 words compared", ok, 1);
	end
endtask

// 13. AxPROT and the test knob
task case13;
	integer d0, mr0;
	begin
		$display("-- AxPROT: GPO 27 = 0 is secure; with secure refused it never arms, and the errors count");
		reset_floor;
		mode = M_SECURE_REFUSE;
		zero_page;
		poke(H_LINE, 32'd0); poke(H_IDX3, 32'd0);
		d0 = dut.diag.rd_err; mr0 = m_rd_err;
		run_edge(0, 0);
		repeat (4 << POLLC) @(posedge clk);
		peek(C_HELLO, v); check("secure refused: never ARMED", v, 32'd0);
		check("secure refused: the monitor saw read errors", m_rd_err - mr0 > 0, 1);
		check("secure refused: the diag's read errors match", dut.diag.rd_err - d0, m_rd_err - mr0);
		mode = M_NORMAL;
		reset_floor;
		mode = M_SECURE_REFUSE;
		zero_page;
		poke(H_LINE, 32'd0); poke(H_IDX3, 32'd0);
		run_edge(1, 0);
		wait_armed(1000, ok);
		check("GPO 27 = 1: non-secure, and it arms", ok, 1);
		check("every AxPROT was 3'b010", mon_err, 0);
		mode = M_NORMAL;
	end
endtask
`endif

// 11. the session ends when H_HELLO changes
task case11;
	integer cnt0;
	begin
		$display("-- the session's end by H_HELLO: 0 then ARMED, another value then WAITING");
		reset_floor;
		start_session(0, 0, 0, 16'h0701, ok);
		peek(C_STATE, v); cnt0 = v[7:0];
		poke(H_HELLO, 32'd0);
		wait_armed(1000, ok);
		check("H_HELLO = 0: ARMED", ok, 1);
		poke(H_HELLO, {16'h4F43, 16'h0702});
		wait_echo(16'h0702, 1000, ok);
		check("a new nonce after 0: a new session", ok, 1);
		peek(C_STATE, v); check("the hello count advanced by one", v[7:0], (cnt0 + 1) & 8'hFF);
		cnt0 = v[7:0];
		poke(H_HELLO, 32'h1234_5678);
		wait_word(C_HELLO, C_HELLO_WAIT, 1000, ok);
		check("any other value: WAITING", ok, 1);
		poke(H_HELLO, 32'd0);
		wait_armed(1000, ok);
		check("0 then ARMED again", ok, 1);
		poke(H_HELLO, {16'h4F43, 16'h0703});
		wait_echo(16'h0703, 1000, ok);
		check("a third session", ok, 1);
		peek(C_STATE, v); check("the hello count advanced again", v[7:0], (cnt0 + 1) & 8'hFF);
	end
endtask

// 14. never answered
task case14;
	begin
		$display("-- never answered: the hung read holds its valid until an HPS reset drops it");
		reset_floor;
		mode = M_NEVER;
		zero_page;
		run_edge(0, 0);
		ok = 0; i = 0;
		while (!ok && i < 20000) begin @(posedge clk); i = i + 1; if (dut.arvalid === 1'b1) ok = 1; end
		repeat (2) @(posedge clk);
		check("the first read hangs", ok, 1);
		term0 = m_term; txn0 = txn_starts;
		gpo_store(GPO_IO);
		repeat (2000) @(posedge clk);
		check("the hung read keeps ARVALID", dut.arvalid, 1);
		check("no terminal store can complete", m_term - term0, 0);
		check("the master issues nothing new", txn_starts - txn0, 0);
		hps_rst_n = 0;
		wait (dut.hrst_q === 1'b1);
		repeat (8) @(posedge clk);
		hps_rst_n = 1;
		repeat (16) @(posedge clk);
		check("the HPS reset drops the hung read", dut.arvalid, 0);
		check("no terminal store after the reset", m_term - term0, 0);
		mode = M_NORMAL;
	end
endtask

////////////////////////////  the OSD tier cases  ///////////////////////////

task check_hello_word(input integer i, input [15:0] a, input [31:0] d);
	begin
		check($sformatf("hello store %0d: the address", i), hseq_addr[i], a);
		check($sformatf("hello store %0d: the word", i), hseq_data[i], d);
	end
endtask

task osd_bind_now(input [15:0] nonce, output integer ok);
	begin
		poke(H_BIND, 32'h0000_0800);              // bit 11: the OSD bind
		ok = 0; i = 0;
		while (!ok && i < 100000) begin
			@(posedge clk); i = i + 1;
			peek(C_BIND, v);
			if (v === 32'h0000_0800) ok = 1;
		end
		check("C_BIND bit 11 is stored after H_BIND bit 11", ok, 1);
	end
endtask

task osd_wait_tail(input [15:0] want, input integer us, output integer ok);
	begin wait_word(C_IDX2, {16'd0, want}, us, ok); end
endtask

// 15. the hello with the OSD tier
task case15;
	integer k, cnt0;
	reg [31:0] state_exp;
	begin
		$display("-- hello with the OSD tier: every store of §2.2 in order, the values, C_BIND 0");
		reset_floor;
		cnt0 = dut.hello_cnt;                  // the hello count is cumulative since configuration
		state_exp = (cnt0 + 8'd1) & 8'hFF;
		if (dut.prev_mid_q)     state_exp = state_exp | 32'h0008_0000;
		if (dut.prev_mid_osd_q) state_exp = state_exp | 32'h0004_0000;
		if (dut.prev_mid_ts_q)  state_exp = state_exp | 32'h0010_0000;
		start_session(0, 0, 0, 16'h0A01, ok);
		k = 0;
		check_hello_word(k, C_VERSION, C_VERSION_1_0); k = k + 1;
		check_hello_word(k, C_CAPS, C_CAPS_EXP); k = k + 1;
		check_hello_word(k, C_STATE, state_exp); k = k + 1;
`ifdef MISTER_OCRAM_OSD
		check_hello_word(k, C_IDX2, 32'd0); k = k + 1;
`endif
		check_hello_word(k, C_IDX3, 32'd0); k = k + 1;
`ifdef MISTER_OCRAM_DIAG
		check_hello_word(k, C_IDX4, 32'd0); k = k + 1;
`endif
`ifdef MISTER_OCRAM_SECTOR_SLOT
		check_hello_word(k, C_IDX4, 32'd0); k = k + 1;
`endif
		check_hello_word(k, C_BIND, 32'd0); k = k + 1;
`ifdef MISTER_OCRAM_OSD
		check_hello_word(k, C_RING2, C_RING2_OSD); k = k + 1;
`endif
		check_hello_word(k, C_RING3, C_RING3_EVT); k = k + 1;
`ifdef MISTER_OCRAM_DIAG
		check_hello_word(k, C_RING4, C_RING4_TS); k = k + 1;
`endif
`ifdef MISTER_OCRAM_SECTOR_SLOT
		check_hello_word(k, C_RING4, C_RING4_SEC); k = k + 1;
`endif
`ifdef MISTER_OCRAM_OSD
		check_hello_word(k, C_K8, C_K8_EXP); k = k + 1;
`endif
`ifdef MISTER_OCRAM_SECTOR_SLOT
		check_hello_word(k, 16'h00D0, C_K0_EXP); k = k + 1;
		check_hello_word(k, 16'h00D4, C_K1_EXP); k = k + 1;
		check_hello_word(k, 16'h00D8, 32'd1); k = k + 1;
		check_hello_word(k, 16'h00E0, C_K4_EXP); k = k + 1;
		check_hello_word(k, 16'h00E4, 32'd32); k = k + 1;
`endif
		check_hello_word(k, C_HELLO, {16'h4F43, 16'h0A01}); k = k + 1;
		check("the hello's store count", hseq_n, k);
		peek(C_BIND, v); check("C_BIND is 0 at the hello", v, 32'd0);
	end
endtask

// 16. the bind
task case16;
	begin
		$display("-- bind: rows published before H_BIND[11] are not drawn; after it C_BIND[11] = 1, then they draw");
		reset_floor;
		osd_clear;
		start_session(0, 0, 0, 16'h0A02, ok);
		make_row(0);
		spi_row(0);
		osd_head = 0;
		osd_publish_row(0);
		osd_publish;
		repeat (6 << POLLC) @(posedge clk);
		peek(C_IDX2, v); check("no row is drawn before the bind", v, 32'd0);
		peek(C_BIND, v); check("C_BIND stays 0 before the bind", v, 32'd0);
		poke(H_BIND, 32'h0000_0800);
		ok = 0; i = 0;
		while (!ok && i < 100000) begin
			@(posedge clk); i = i + 1;
			peek(C_BIND, v);
			if (v === 32'h0000_0800) ok = 1;
		end
		check("C_BIND bit 11 is stored", ok, 1);
		osd_wait_tail(65, 1000, ok);
		check("the row published before the bind draws after it", ok, 1);
		osd_compare(bad);
		check("osd B's buffer equals osd A's", bad, 0);
	end
endtask

// 17. twenty (or sixteen) random rows, one head store
task case17;
	integer r;
	begin
		$display("-- %0d random rows in one head store: osd B equals osd A byte for byte; C_IDX[2] ends at the head",
		         ROWS);
		reset_floor;
		osd_clear;
		start_session(0, 0, 0, 16'h0A03, ok);
		osd_bind_now(16'h0A03, ok);
		osd_head = 0;
		for (r = 0; r < ROWS; r = r + 1) begin
			make_row(r);
			spi_row(r);
			osd_publish_row(r);
		end
		osd_publish;
		osd_wait_tail(ROWS*65, 10000, ok);
		check("C_IDX[2] ends at the head", ok, 1);
		peek(C_IDX2, v); check("C_IDX[2] = the head", v[15:0], (ROWS*65) & 16'hFFFF);
		osd_compare(bad);
		check("osd B's buffer equals osd A's, byte for byte", bad, 0);
		check("highres set by rows 8-15 in osd A", osd_spi.osd_h, 22'd128);
		check("highres set by rows 8-15 in osd B", osd_ocr.osd_h, 22'd128);
		peek(C_STATE, v); check("no OSD fault bit", {v[10], v[18]}, 2'b00);
	end
endtask

// 18. the guard
task case18;
	integer wc;
	begin
		$display("-- the guard: an SPI select held high, and the ID view, each hold the writer, and the two cycles after");
		reset_floor;
		osd_clear;
		start_session(0, 0, 0, 16'h0A04, ok);
		osd_bind_now(16'h0A04, ok);
		make_row(0); make_row(1);
		spi_row(0);  spi_row(1);
		osd_head = 0;
		osd_publish_row(0);
		osd_publish;
		// wait until a byte of the row has landed, then hold one osd selected mid-row
		ok = 0; i = 0;
		while (!ok && i < 20000) begin
			@(posedge clk_sys); #1; i = i + 1;
			if (ocram_we === 1'b1) ok = 1;
		end
		check("the writer starts the row", ok, 1);
		sel_m = 2'b01;                        // one osd selected: not quiet
		wc = 0;
		repeat (20) begin @(posedge clk_sys); #1; if (ocram_we === 1'b1) wc = wc + 1; end
		check("no byte lands while a select is high", wc, 0);
		@(negedge clk_sys); sel_m = 2'b00;
		#1; check("the release cycle: no byte", ocram_we, 0);
		@(posedge clk_sys); #1; check("the cycle after the release: no byte", ocram_we, 0);
		ok = 0; i = 0;
		while (!ok && i < 20) begin @(posedge clk_sys); #1; i = i + 1; if (ocram_we === 1'b1) ok = 1; end
		check("the writer resumes on the third quiet cycle", ok, 1);
		// now the ID view, on the next row
		osd_wait_tail(65, 1000, ok);
		osd_head = 65;
		osd_publish_row(1);
		osd_publish;
		ok = 0; i = 0;
		while (!ok && i < 20000) begin
			@(posedge clk_sys); #1; i = i + 1;
			if (ocram_we === 1'b1) ok = 1;
		end
		check("the writer starts the second row", ok, 1);
		view_m = 1'b0;                        // the ID view: not quiet
		wc = 0;
		repeat (20) begin @(posedge clk_sys); #1; if (ocram_we === 1'b1) wc = wc + 1; end
		check("no byte lands in the ID view", wc, 0);
		@(negedge clk_sys); view_m = 1'b1;
		#1; check("the view's return cycle: no byte", ocram_we, 0);
		@(posedge clk_sys); #1; check("the cycle after the view returns: no byte", ocram_we, 0);
		ok = 0; i = 0;
		while (!ok && i < 20) begin @(posedge clk_sys); #1; i = i + 1; if (ocram_we === 1'b1) ok = 1; end
		check("the writer resumes on the third quiet cycle after the view returns", ok, 1);
		osd_wait_tail(130, 10000, ok);
		check("both rows finish after the guard returns", ok, 1);
		osd_compare(bad);
		check("osd B's buffer equals osd A's", bad, 0);
	end
endtask

// 19. skip
task case19;
	begin
		$display("-- skip: an unknown kind, a ROW of len 63 and a ROW with arg >= ROWS are skipped by length");
		reset_floor;
		osd_clear;
		start_session(0, 0, 0, 16'h0A05, ok);
		osd_bind_now(16'h0A05, ok);
		make_row(0); make_row(1);
		spi_row(0);  spi_row(1);
		osd_head = 0;
		osd_publish_row(0);
		osd_hdr(8'h77, 8'd5, 16'd3);  osd_word(32'h1111_1111); osd_word(32'h2222_2222);
		osd_word(32'h3333_3333);                     // an unknown kind: 1 + 3 words
		osd_hdr(8'h01, 8'd2, 16'd63);                // a ROW of another length: 1 + 63 words
		for (j = 0; j < 63; j = j + 1) osd_word(32'd0);
		osd_hdr(8'h01, ROWS16, 16'd64);              // a row outside the OSD: 1 + 64 words
		for (j = 0; j < 64; j = j + 1) osd_word(32'd0);
		osd_publish_row(1);
		osd_publish;
		osd_wait_tail(65 + 4 + 64 + 65 + 65, 10000, ok);
		check("the rows around the skipped records land", ok, 1);
		peek(C_IDX2, v); check("C_IDX[2] = the head after the skips", v[15:0], (65+4+64+65+65) & 16'hFFFF);
		peek(C_STATE, v); check("a skip is not a fault", {v[10], v[18]}, 2'b00);
		osd_compare(bad);
		check("osd B's buffer equals osd A's", bad, 0);
	end
endtask

// 20. fault
task case20;
	begin
		$display("-- fault: a len past the head stops the ring, C_STATE bit 10 is stored at once, no more rows draw");
		reset_floor;
		osd_clear;
		start_session(0, 0, 0, 16'h0A06, ok);
		osd_bind_now(16'h0A06, ok);
		make_row(0); make_row(1);
		spi_row(0);  spi_row(1);
		osd_head = 0;
		osd_publish_row(0);
		osd_publish;
		osd_wait_tail(65, 1000, ok);
		check("the first row lands", ok, 1);
		// a header whose len runs past the head: one payload word behind it
		osd_hdr(8'h01, 8'd1, 16'd64);
		osd_word(32'd0);
		osd_publish;
		ok = 0; i = 0;
		while (!ok && i < 100000) begin
			@(posedge clk); i = i + 1;
			peek(C_STATE, v);
			if (v[10] === 1'b1) ok = 1;
		end
		check("C_STATE bit 10 is stored", ok, 1);
		repeat (2000) @(posedge clk);
		peek(C_IDX2, v); check("the bad row's tail is not stored", v[15:0], 16'd65);
		// the next hello clears it and rows draw again
		poke(H_HELLO, 32'd0);
		wait_armed(1000, ok);
		osd_clear;
		osd_head = 0;
		poke(H_IDX2, 32'd0);
		poke(H_BIND, 32'h0000_0800);
		poke(H_HELLO, {16'h4F43, 16'h0A07});
		wait_echo(16'h0A07, 1000, ok);
		peek(C_STATE, v); check("the new hello clears the fault bit", v[10], 0);
		peek(C_IDX2, v); check("the new hello's C_IDX[2] = 0", v, 32'd0);
		spi_row(1);
		osd_publish_row(1);
		osd_publish;
		osd_wait_tail(65, 10000, ok);
		check("a row draws after the new hello", ok, 1);
		osd_compare(bad);
		check("osd B's buffer equals osd A's after the fault", bad, 0);
	end
endtask

// 21. the session ends mid-row
task case21;
	begin
		$display("-- the session ends mid-row (RUN falls): the rest is not drawn, C_IDX[2] not stored, bit 18 next");
		reset_floor;
		osd_clear;
		start_session(0, 0, 0, 16'h0A08, ok);
		osd_bind_now(16'h0A08, ok);
		make_row(0);
		osd_head = 0;
		osd_publish_row(0);
		osd_publish;
		ok = 0; i = 0;
		while (!ok && i < 20000) begin
			@(posedge clk_sys); #1; i = i + 1;
			if (ocram_we === 1'b1) ok = 1;
		end
		check("the row is being drawn when RUN falls", ok, 1);
		gpo_store(GPO_IO);
		wait_stopped(1000, ok);
		check("the terminal store lands", ok, 1);
		repeat (1000) @(posedge clk_sys);
		peek(C_IDX2, v); check("the row's tail is not stored", v, 32'd0);
		bad = 0;
		repeat (200) begin @(posedge clk_sys); #1; if (ocram_we === 1'b1) bad = bad + 1; end
		check("no byte lands after the link drains", bad, 0);
		reset_floor;
		zero_page;
		poke(H_IDX2, 32'd0);
		osd_head = 0;
		run_edge(0, 0); wait_armed(1000, ok);
		poke(H_BIND, 32'h0000_0800);
		poke(H_HELLO, {16'h4F43, 16'h0A09});
		wait_echo(16'h0A09, 1000, ok);
		peek(C_STATE, v); check("the next hello's C_STATE bit 18", v[18], 1);
		$display("-- the session ends mid-row (H_HELLO changes): the rest is not drawn, C_IDX[2] not stored, bit 18 next");
		reset_floor;
		osd_clear;
		start_session(0, 0, 0, 16'h0A0A, ok);
		osd_bind_now(16'h0A0A, ok);
		make_row(1);
		osd_head = 0;
		osd_publish_row(1);
		osd_publish;
		ok = 0; i = 0;
		while (!ok && i < 20000) begin
			@(posedge clk_sys); #1; i = i + 1;
			if (ocram_we === 1'b1) ok = 1;
		end
		check("the row is being drawn when H_HELLO changes", ok, 1);
		poke(H_HELLO, 32'd0);
		wait_armed(1000, ok);
		check("H_HELLO = 0 ends the session: ARMED", ok, 1);
		repeat (1000) @(posedge clk_sys);
		peek(C_IDX2, v); check("the row's tail is not stored", v, 32'd0);
		zero_page;
		poke(H_IDX2, 32'd0);
		osd_head = 0;
		poke(H_BIND, 32'h0000_0800);
		poke(H_HELLO, {16'h4F43, 16'h0A0B});
		wait_echo(16'h0A0B, 1000, ok);
		check("a new session after the change", ok, 1);
		peek(C_STATE, v); check("the next hello's C_STATE bit 18", v[18], 1);
	end
endtask

// 22. a frozen clk_sys
`ifdef MISTER_OCRAM_DIAG
task case22;
	begin
		$display("-- frozen clk_sys (GPO 25 = 1 mid-row): the walker waits, polls and the stop run, no echo until thaw");
		reset_floor;
		osd_clear;
		start_session(0, 0, 0, 16'h0A0C, ok);
		osd_bind_now(16'h0A0C, ok);
		make_row(0); make_row(1);
		spi_row(0);
		osd_head = 0;
		osd_publish_row(0);
		osd_publish;
		ok = 0; i = 0;
		while (!ok && i < 20000) begin
			@(posedge clk_sys); #1; i = i + 1;
			if (ocram_we === 1'b1) ok = 1;
		end
		check("the row is being drawn when GPO 25 rises", ok, 1);
		gpo_store(GPO_RUN | GPO_FRZ);          // RUN held, the freeze knob set
		repeat (300) @(posedge clk_sys);
		bad = 0;
		repeat (200) begin @(posedge clk_sys); #1; if (ocram_we === 1'b1) bad = bad + 1; end
		check("no byte lands while frozen", bad, 0);
		peek(C_IDX2, v); check("the row's tail is not stored while frozen", v, 32'd0);
		h0 = m_hello_reads;
		repeat (4 << POLLC) @(posedge clk);
		check("the master still polls while frozen", m_hello_reads - h0 > 0, 1);
		// the stop proof lands while frozen
		gpo_store(GPO_IO | GPO_FRZ);           // RUN 0, the freeze held
		wait_stopped(1000, ok);
		check("the stop proof lands while frozen", ok, 1);
		// a new RUN edge and hello get ARMED but no echo while frozen
		reset_floor;
		osd_clear;
		zero_page;
		osd_head = 0;
		poke(H_IDX2, 32'd0);
		gpo_store(GPO_IO | GPO_FRZ);
		gpo_store(GPO_RUN | GPO_FRZ);          // the new RUN edge, still frozen
		wait_armed(1000, ok);
		check("a new RUN edge arms while frozen", ok, 1);
		repeat (200) @(posedge clk_sys);      // let the freeze cross to clk_sys
		check("the frozen half reports not at rest", c0_rest, 0);
		poke(H_HELLO, {16'h4F43, 16'h0A0D});
		wait_echo(16'h0A0D, 1200, ok);
		check("no echo while frozen (the hello waits for rest)", ok, 0);
		peek(C_VERSION, v); check("no hello store while frozen", v, 32'd0);
		gpo_store(GPO_RUN);                    // thaw
		wait_echo(16'h0A0D, 1000, ok);
		check("the echo comes on thaw", ok, 1);
		osd_clear;
		make_row(1);
		spi_row(1);
		osd_head = 0;
		poke(H_BIND, 32'h0000_0800);
		poke(H_IDX2, 32'd0);
		poke(H_HELLO, {16'h4F43, 16'h0A0D});
		osd_publish_row(1);
		osd_publish;
		osd_wait_tail(65, 10000, ok);
		check("a fresh row draws after the thaw", ok, 1);
		osd_compare(bad);
		check("osd B's buffer equals osd A's after the thaw", bad, 0);
	end
endtask
`endif

// 23. the torture sink
`ifdef MISTER_OCRAM_DIAG
task case23;
	begin
		$display("-- the torture sink: 2,000 records of random lengths 1..1,500, crossing the ring's end many times");
		reset_floor;
		fast_lat = 1;
		ts_reset;
		start_session(0, 0, 0, 16'h0B01, ok);
		for (r = 0; r < 2000; r = r + 1) begin
			j = ($random(seed) & 32'h7FFF_FFFF);
			if (j % 16 == 0) len = 201 + (j % 1300);  // one in sixteen is long, up to 1,500
			else             len = 1 + (j % 100);
			ts_wait_room(1 + len);
			ts_record(len, -1, 32'd0, 0);
			evt_take(ok1);
			if (r % 64 == 0) check($sformatf("torture record %0d stored", r), ts_recs, r + 1);
		end
		wait_ts_drain(20000, ok);
		check("the sink drains the ring", ok, 1);
		wait_verdict(2000, 0, 20000, ok);
		check("a VERDICT with 2,000 records is posted", ok, 1);
		check("VERDICT: records", vd_last[0], 2000);
		check("VERDICT: words", vd_last[1], ts_words_exp);
		check("VERDICT: errors", vd_last[2], 0);
		check("VERDICT: first_g", vd_last[3], 0);
		check("VERDICT: first_got", vd_last[4], 0);
		check("VERDICT: sum", vd_last[5], ts_sum_exp);
		check("VERDICT: skipped", vd_last[6][31:16], 0);
		check("VERDICT: deepest is at most the ring", vd_last[6][15:0] <= TS_WORDS, 1);
		check("VERDICT: the OSD statistics are 0 with no rows", vd_last[7], 0);
		check("no protocol error over the torture traffic", mon_err, 0);
	end
endtask

// 24. torture errors
task case24;
	reg [31:0] badv;
	begin
		$display("-- torture errors: one corrupted payload word and one wrong seq are each counted");
		reset_floor;
		fast_lat = 1;
		ts_reset;
		start_session(0, 0, 0, 16'h0B02, ok);
		badv = 32'hDEAD_BEEF;
		ts_record(10, 3, badv, 0);             // the bad word is global g = 3
		ts_record(5, -1, 32'd0, 1);            // a wrong seq
		wait_ts_drain(20000, ok);
		check("the sink drains the ring", ok, 1);
		wait_verdict(2, 0, 20000, ok);
		check("a VERDICT with 2 records is posted", ok, 1);
		check("VERDICT: records", vd_last[0], 2);
		check("VERDICT: words", vd_last[1], 15);
		check("VERDICT: errors (a bad word and a bad seq)", vd_last[2], 2);
		check("VERDICT: first_g", vd_last[3], 3);
		check("VERDICT: first_got", vd_last[4], badv);
		check("VERDICT: sum", vd_last[5], ts_sum_exp);
	end
endtask

// 25. concurrency
task case25;
	integer rows_done;
	begin
		$display("-- concurrency: OSD rows, torture records and MARKs at once");
		reset_floor;
		fast_lat = 1;
		osd_clear;
		ts_reset;
		start_session(0, 0, 8, 16'h0B03, ok);
		osd_bind_now(16'h0B03, ok);
		make_row(0); make_row(1);
		spi_row(0);  spi_row(1);
		osd_head = 0;
		osd_publish_row(0);
		osd_publish;
		for (r = 0; r < 40; r = r + 1) begin
			len = 1 + (($random(seed) & 32'h7FFF_FFFF) % 200);
			ts_wait_room(1 + len);
			ts_record(len, -1, 32'd0, 0);
			evt_take(ok1);
		end
		// the second row follows the first in the ring: osd_head is already the head the first published
		osd_publish_row(1);
		osd_publish;
		wait_ts_drain(20000, ok);
		check("the sink drains the ring", ok, 1);
		osd_wait_tail(130, 20000, ok);
		check("every OSD row lands", ok, 1);
		osd_compare(bad);
		check("osd B's buffer equals osd A's", bad, 0);
		wait_verdict(40, 2, 20000, ok);
		check("a VERDICT with 40 records is posted", ok, 1);
		check("VERDICT: records", vd_last[0], 40);
		check("VERDICT: words", vd_last[1], ts_words_exp);
		check("VERDICT: errors", vd_last[2], 0);
		check("VERDICT: sum", vd_last[5], ts_sum_exp);
		check("VERDICT: the OSD row count", vd_last[7][31:16], 2);
		check("no protocol error with every consumer busy", mon_err, 0);
		check("MARKs were posted beside the consumers", mark_published > 0, 1);
	end
endtask
`endif

// 27. the line held high with the EVT ring undrained
`ifdef MISTER_OCRAM_DIAG
task case27;
	integer r, jj, l;
	begin
		$display("-- the line held high, EVT undrained: 8 OSD rows and 200 torture records still land, one re-read per turn");
		reset_floor;
		fast_lat = 1;
		osd_clear;
		ts_reset;
		start_session(0, 0, 8, 16'h0D01, ok);       // H_LINE bit 3; the host never stores the EVT tail
		osd_bind_now(16'h0D01, ok);
		ok = 0; i = 0;
		while (!ok && i < 200000) begin @(posedge clk); i = i + 1; if (line_out === 1'b1) ok = 1; end
		check("the line is high before the traffic", ok, 1);
		// the bound is between consumer grants, so the counters start with the traffic
		rr_wait_since = 0; rr_wait_max = 0; grants_all = 0;
		// a torture backlog first, so the sink keeps a request pending through the OSD rows
		for (r = 0; r < 200; r = r + 1) begin
			jj = ($random(seed) & 32'h7FFF_FFFF);
			l = 1 + (jj % 200);
			ts_wait_room_nd(1 + l);
			ts_record(l, -1, 32'd0, 0);
		end
		// then the 8 rows through the OCRAM writer, their twins over SPI, in one head store
		osd_head = 0;
		for (r = 0; r < 8; r = r + 1) begin
			make_row(r);
			spi_row(r);
			osd_publish_row(r);
		end
		osd_publish;
		osd_wait_tail(8*65, 20000, ok);
		check("every row lands while the line is high", ok, 1);
		wait_ts_drain_nd(20000, ok);
		check("the sink drains its ring while the line is high", ok, 1);
		check("the line is still high when the traffic ends", line_out, 1);
		osd_compare(bad);
		check("every row is byte-equal to osd A", bad, 0);
		// the host drains now: the verdict with both counters, then the arbitration bound
		wait_verdict(200, 8, 20000, ok);
		check("a VERDICT with 200 records and 8 rows is posted", ok, 1);
		check("VERDICT: records", vd_last[0], 200);
		check("VERDICT: words", vd_last[1], ts_words_exp);
		check("VERDICT: errors", vd_last[2], 0);
		check("VERDICT: sum", vd_last[5], ts_sum_exp);
		check("VERDICT: the OSD row count", vd_last[7][31:16], 8);
		$display("   %0d tail re-reads; the longest a waiting consumer sat behind between two grants: %0d",
		         line_rr_all, rr_wait_max);
		check("at most one EVT tail re-read a waiting consumer sits behind, between two grants", rr_wait_max <= 1, 1);
		check("the consumers were granted repeatedly", grants_all > 2000, 1);
		check("re-reads occurred during the traffic", line_rr_all > 100, 1);
	end
endtask
`endif

// The SECTOR tier (MISTER_OCRAM_SECTOR_SLOT): the core's block reader on clk_sys, the host's SECTOR ring.
`ifdef MISTER_OCRAM_SECTOR_SLOT
localparam [15:0] SEC_BASE  = 16'h7000;
localparam integer SEC_WORDS = 8192;
localparam integer SREC = SFRAME + 2;       // a FRAME record's words: header, LBA, payload

// the core: one request at a time; it falls with the ack's rise; the strobes land in cbuf; a frame ends at the fall
reg  [15:0] cbuf [0:8191];
integer     c_strobes = 0, c_frames = 0;
reg         c_ack_d = 1'b0;
always @(posedge clk_sys) begin
	c_ack_d <= x_ack;
	if (x_ack && !c_ack_d) x_req <= 1'b0;
	if (x_wr) begin cbuf[x_adr] <= x_dat; c_strobes = c_strobes + 1; end
	if (!x_ack && c_ack_d) c_frames = c_frames + 1;
end
task core_ask(input [31:0] lba);
	integer j;
	begin
		for (j = 0; j < 2 * SFRAME; j = j + 1) cbuf[j] = 16'hDEAD;
		@(negedge clk_sys); x_lba = lba; x_req = 1'b1;
	end
endtask
task core_drop;
	begin @(negedge clk_sys); x_req = 1'b0; end
endtask
task core_wait(input integer f0, input integer us, output integer ok);
	integer t;
	begin
		ok = 0; t = 0;
		while (!ok && t < us * 100) begin
			if (c_frames > f0) ok = 1;
			else begin @(posedge clk); t = t + 1; end
		end
	end
endtask
// a frame's payload word j: the LBA, a disc generation and the index
function [31:0] sword(input [31:0] lba, input integer gen, input integer j);
	sword = v_of({lba[11:0], gen[3:0], j[15:0]});
endfunction
// the frame as the core holds it: zeros from word z on (z = SFRAME: none)
task core_check(input string what, input [31:0] lba, input integer gen, input integer z);
	integer j, bad;
	reg [31:0] w;
	begin
		bad = 0;
		for (j = 0; j < SFRAME; j = j + 1) begin
			w = (j >= z) ? 32'd0 : sword(lba, gen, j);
			if ({cbuf[2 * j + 1], cbuf[2 * j]} !== w) begin
				if (bad < 4) $display("   %s: word %0d: got %04h%04h exp %08h", what, j, cbuf[2*j+1], cbuf[2*j], w);
				bad = bad + 1;
			end
		end
		check(what, bad, 0);
	end
endtask

// the host's SECTOR ring: records at the free-running word index sec_head, one H_IDX[4] store publishes them
integer sec_head = 0;
task sec_put(input [31:0] v);
	begin poke(SEC_BASE + ((sec_head[15:0] & (SEC_WORDS - 1)) << 2), v); sec_head = sec_head + 1; end
endtask
task sec_frame(input [31:0] lba, input integer gen);
	integer j;
	begin
		sec_put({8'h10, 8'd0, SFRAME[15:0] + 16'd1});
		sec_put(lba);
		for (j = 0; j < SFRAME; j = j + 1) sec_put(sword(lba, gen, j));
	end
endtask
task sec_publish;
	begin poke(H_IDX4, {16'd0, sec_head[15:0]}); end
endtask
function [15:0] sec_tail(input dummy);
	sec_tail = mem[C_IDX4 >> 2][15:0];
endfunction
task rq_drain;
	integer got;
	begin got = 1; while (got) evt_take(got); end
endtask
task rq_wait(input integer n0, input integer us, output integer ok);
	integer t;
	begin
		ok = 0; t = 0;
		while (!ok && t < us * 20) begin
			rq_drain;
			if (rq_n > n0) ok = 1;
			else begin repeat (5) @(posedge clk); t = t + 1; end
		end
	end
endtask
task sec_session(input [15:0] nonce, input integer osd);
	begin
		reset_floor;
		sec_head = 0; rq_n = 0; osd_head = 0;
		x_req = 1'b0; x_reset = 1'b0;
		start_session(0, 0, 0, nonce, ok);        // its zero_page clears H_SECTOR and H_IDX[4]
		poke(H_SECTOR, 32'd1);
		if (osd) poke(H_BIND, 32'h0000_0800);
		wait_word(C_BIND, osd ? 32'h0000_0801 : 32'h0000_0001, 2000, ok);
		check("C_BIND bit 0 is stored once the slot's mux is on the mailbox", ok, 1);
		check("the slot's mux is on the mailbox (en_sys)", x_en, 1);
	end
endtask
task wait_bind0(input integer us, output integer ok);
	integer t;
	begin
		ok = 0; t = 0;
		while (!ok && t < us * 100) begin
			if (mem[C_BIND >> 2][0] === 1'b0) ok = 1;
			else begin @(posedge clk); t = t + 1; end
		end
	end
endtask
// the index of the first log entry that writes off (with data d if dm), from k on; -1 if none
function integer tl_find(input integer k, input [15:0] off, input [31:0] d, input dm, input we);
	integer q;
	begin
		tl_find = -1;
		for (q = tl_n - 1; q >= k; q = q - 1)
			if (tl_we[q] == we && tl_off[q] == off && (!dm || tl_dat[q] == d)) tl_find = q;
	end
endfunction

// S1. the bind and its transitions
task sec_case1;
	begin
		$display("-- SECTOR: H_SECTOR bit 0 binds the slot: C_BIND bit 0 follows en_sys, 1, 0, 1; bit 11 apart");
		sec_session(16'h0C01, 0);
		poke(H_SECTOR, 32'd0);
		wait_word(C_BIND, 32'd0, 2000, ok);
		check("H_SECTOR bit 0 cleared: C_BIND bit 0 = 0", ok, 1);
		check("the mux is back on SPI", x_en, 0);
		poke(H_SECTOR, 32'd1);
		wait_word(C_BIND, 32'd1, 2000, ok);
		check("set again: C_BIND bit 0 = 1", ok, 1);
		poke(H_BIND, 32'h0000_0800);
		wait_word(C_BIND, 32'h0000_0801, 2000, ok);
		check("the OSD bind beside it: C_BIND = 0x801", ok, 1);
		check("C_CAPS has bit 4 (ring 4)", mem[C_CAPS >> 2][4], 1);
	end
endtask

// S2. play, REQUEST post and re-post
task sec_case2;
	integer f0, n0;
	begin
		$display("-- SECTOR: a request with the ring empty posts one REQUEST; the frame plays; a new request re-posts");
		sec_session(16'h0C02, 0);
		n0 = rq_n;
		core_ask(32'd1000);
		rq_wait(n0, 2000, ok);
		check("a REQUEST is posted", ok, 1);
		check("its LBA", rq_lba_log[n0], 32'd1000);
		repeat (40 << POLLC) @(posedge clk);
		rq_drain;
		check("one REQUEST while the request stands unanswered", rq_n - n0, 1);
		f0 = c_frames;
		sec_frame(32'd1000, 0); sec_publish;
		core_wait(f0, 5000, ok);
		check("the frame plays", ok, 1);
		core_check("the frame's payload, as the core holds it", 32'd1000, 0, SFRAME);
		wait_word(C_IDX4, {16'd0, sec_head[15:0]}, 1000, ok);
		check("C_IDX[4] moves past the record", ok, 1);
		n0 = rq_n;
		core_ask(32'd1001);
		rq_wait(n0, 2000, ok);
		check("the next request posts its own REQUEST", ok && rq_lba_log[n0] == 32'd1001, 1);
		f0 = c_frames;
		sec_frame(32'd1001, 0); sec_publish;
		core_wait(f0, 5000, ok);
		core_check("the second frame's payload", 32'd1001, 0, SFRAME);
		check("REQUEST records seen by the monitor", rq_published, rq_n);
	end
endtask

// S3. prefetch several frames ahead
task sec_case3;
	integer f0, n0, k;
	begin
		$display("-- SECTOR: six frames prefetched ahead of the requests play in turn, with no REQUEST");
		sec_session(16'h0C03, 0);
		for (k = 0; k < 6; k = k + 1) sec_frame(32'd2000 + k, 0);
		sec_publish;
		n0 = rq_n;
		for (k = 0; k < 6; k = k + 1) begin
			f0 = c_frames;
			core_ask(32'd2000 + k);
			core_wait(f0, 5000, ok);
			core_check($sformatf("prefetched frame %0d", k), 32'd2000 + k, 0, SFRAME);
		end
		rq_drain;
		check("no REQUEST for a prefetched frame", rq_n - n0, 0);
		wait_word(C_IDX4, {16'd0, sec_head[15:0]}, 200, ok);
		check("C_IDX[4] at the head once the last word's done is back", ok, 1);
	end
endtask

// S4. mismatch discard, S5. no request keeps the frame
task sec_case4;
	integer f0, n0;
	begin
		$display("-- SECTOR: frames for other LBAs are discarded up to the requested one; with no request a frame waits");
		sec_session(16'h0C04, 0);
		sec_frame(32'd3000, 0); sec_frame(32'd3001, 0); sec_frame(32'd3002, 0); sec_publish;
		repeat (20 << POLLC) @(posedge clk);
		check("no request: the frames wait, C_IDX[4] = 0", sec_tail(1'b0), 16'd0);
		check("no request: no strobe", c_strobes > 0 && x_ack, 0);
		f0 = c_frames;
		core_ask(32'd3002);
		core_wait(f0, 5000, ok);
		check("the requested frame plays", ok, 1);
		core_check("its payload (3000 and 3001 discarded)", 32'd3002, 0, SFRAME);
		wait_word(C_IDX4, {16'd0, sec_head[15:0]}, 200, ok);
		check("C_IDX[4] at the head once the last word's done is back", ok, 1);
		n0 = rq_n;
		core_ask(32'd3000);
		rq_wait(n0, 2000, ok);
		check("the discarded LBA is asked for again", ok && rq_lba_log[n0] == 32'd3000, 1);
		f0 = c_frames;
		sec_frame(32'd3000, 1); sec_publish;
		core_wait(f0, 5000, ok);
		core_check("the re-sent frame plays", 32'd3000, 1, SFRAME);
	end
endtask

// S6. a disc swap: H_SECTOR bit 0 falls with 12 frames queued; none of them plays after the rebind
task sec_case6;
	integer f0, k, a, b;
	begin
		$display("-- SECTOR: a disc swap with 12 frames queued: the tail jumps to the head before C_BIND bit 0 = 0");
		sec_session(16'h0C06, 0);
		for (k = 0; k < 12; k = k + 1) sec_frame(32'd6000 + k, 0);
		sec_publish;
		repeat (8 << POLLC) @(posedge clk);
		check("queued, unplayed: C_IDX[4] = 0", sec_tail(1'b0), 16'd0);
		tl_n = 0; tl_on = 1;
		poke(H_SECTOR, 32'd0);
		wait_bind0(3000, ok);
		tl_on = 0;
		check("C_BIND bit 0 = 0", ok, 1);
		check("every queued frame discarded: C_IDX[4] = the head", sec_tail(1'b0), sec_head[15:0]);
		a = tl_find(0, C_IDX4, {16'd0, sec_head[15:0]}, 1, 1);
		b = tl_find(0, C_BIND, 32'd0, 1, 1);
		check("the C_IDX[4] store precedes the C_BIND bit-0 store", a >= 0 && b > a, 1);
		check("the head was read for it (H_IDX[4] before the store)", tl_find(0, H_IDX4, 0, 0, 0) >= 0 &&
		      tl_find(0, H_IDX4, 0, 0, 0) < a, 1);
		for (k = 0; k < 12; k = k + 1) sec_frame(32'd6000 + k, 1);
		sec_publish;
		poke(H_SECTOR, 32'd1);
		wait_word(C_BIND, 32'd1, 3000, ok);
		check("the rebind", ok, 1);
		for (k = 0; k < 12; k = k + 1) begin
			f0 = c_frames;
			core_ask(32'd6000 + k);
			core_wait(f0, 5000, ok);
			core_check($sformatf("after the swap, LBA %0d is the new disc's", 6000 + k), 32'd6000 + k, 1, SFRAME);
		end
	end
endtask

// S7. H_SECTOR dropped mid-frame: the frame finishes, the frame behind it is discarded
task sec_case7;
	integer f0, s0;
	begin
		$display("-- SECTOR: H_SECTOR bit 0 cleared mid-frame: that frame plays out whole, the next is discarded");
		sec_session(16'h0C07, 0);
		sec_frame(32'd8000, 0); sec_frame(32'd8001, 0); sec_publish;
		f0 = c_frames; s0 = c_strobes;
		core_ask(32'd8000);
		i = 0; while (c_strobes < s0 + 40 && i < 400000) begin @(posedge clk); i = i + 1; end
		poke(H_SECTOR, 32'd0);
		core_wait(f0, 5000, ok);
		check("the frame completes", ok, 1);
		core_check("its payload is whole", 32'd8000, 0, SFRAME);
		wait_bind0(3000, ok);
		check("C_BIND bit 0 = 0 after it", ok, 1);
		check("the frame behind it discarded: C_IDX[4] = the head", sec_tail(1'b0), sec_head[15:0]);
		repeat (100) @(posedge clk_sys);
		check("the mux is back on SPI", x_en, 0);
	end
endtask

// S8. unbind while a REQUEST is part-written: the header and LBA stored, the head store pending
task sec_case8;
	integer n0, b, c, d, q, late;
	begin
		$display("-- SECTOR: H_SECTOR cleared while a REQUEST's head store is owed: the record completes first");
		sec_session(16'h0C08, 0);
		n0 = rq_n;
		tl_n = 0; tl_on = 1;
		stall_wd = 32'd7000; stall_n = 3000; stall_w_en = 1;    // the LBA's store: longer than a poll period
		core_ask(32'd7000);
		i = 0; while (!stall_on && i < 200000) begin @(posedge clk); i = i + 1; end
		check("the REQUEST's LBA store is in flight", stall_on, 1);
		poke(H_SECTOR, 32'd0);
		wait_bind0(3000, ok);
		repeat (20 << POLLC) @(posedge clk);
		tl_on = 0;
		check("C_BIND bit 0 = 0", ok, 1);
		b = -1;
		for (q = 0; q < tl_n; q = q + 1) if (b < 0 && tl_we[q] && tl_dat[q] == 32'd7000) b = q;
		c = tl_find(b < 0 ? 0 : b, H_SECTOR, 32'd0, 1, 0);
		d = tl_find(b < 0 ? 0 : b, C_IDX3, 0, 0, 1);
		check("the LBA stored, then H_SECTOR read as 0, then the head stored", b >= 0 && c > b && d > c, 1);
		check("C_BIND bit 0 = 0 only after the head store", tl_find(0, C_BIND, 32'd0, 1, 1) > d, 1);
		late = 0;
		for (q = tl_find(0, C_BIND, 32'd0, 1, 1); q >= 0 && q < tl_n; q = q + 1)
			if (tl_we[q] && tl_dat[q] == REQ_HEADER) late = late + 1;
		check("no REQUEST header after the C_BIND bit-0 store", late, 0);
		rq_drain;
		check("the one REQUEST is whole, for LBA 7000", rq_n - n0 == 1 && rq_lba_log[n0] == 32'd7000, 1);
		core_drop;
	end
endtask

// S9. RUN falls mid-frame; S10. the consumer's reset mid-frame
task sec_case9;
	integer f0, s0, z;
	begin
		$display("-- SECTOR: RUN falls mid-frame: the feeder zero-fills, the ack completes; C_STATE bit 20 next");
		sec_session(16'h0C09, 0);
		sec_frame(32'd8100, 0); sec_publish;
		f0 = c_frames; s0 = c_strobes;
		core_ask(32'd8100);
		i = 0; while (c_strobes < s0 + 100 && i < 400000) begin @(posedge clk); i = i + 1; end
		gpo_store(GPO_IO);
		core_wait(f0, 5000, ok);
		check("the core's ack completes", ok, 1);
		z = 0; while (z < SFRAME && {cbuf[2*z+1], cbuf[2*z]} === sword(32'd8100, 0, z)) z = z + 1;
		check("the words before the cut are the frame's", z >= 40, 1);
		core_check("then zeros to the end", 32'd8100, 0, z);
		wait_stopped(1000, ok);
		reset_floor;
		poke(H_SECTOR, 32'd1);
		run_edge(0, 0); wait_armed(1000, ok);
		poke(H_HELLO, {16'h4F43, 16'h0C0A});
		wait_echo(16'h0C0A, 1000, ok);
		peek(C_STATE, v); check("the next hello's C_STATE bit 20 (a SECTOR record abandoned)", v[20], 1);

		$display("-- SECTOR: the consumer's reset mid-frame zero-fills it; the rest is consumed; the next frame plays");
		sec_session(16'h0C0B, 0);
		sec_frame(32'd8200, 0); sec_frame(32'd8201, 0); sec_publish;
		f0 = c_frames; s0 = c_strobes;
		core_ask(32'd8200);
		i = 0; while (c_strobes < s0 + 60 && i < 400000) begin @(posedge clk); i = i + 1; end
		@(negedge clk_sys); x_reset = 1'b1; @(negedge clk_sys); x_reset = 1'b0;
		core_wait(f0, 5000, ok);
		check("the ack completes", ok, 1);
		z = 0; while (z < SFRAME && {cbuf[2*z+1], cbuf[2*z]} === sword(32'd8200, 0, z)) z = z + 1;
		core_check("zeros from the next word", 32'd8200, 0, z);
		f0 = c_frames;
		core_ask(32'd8201);
		core_wait(f0, 5000, ok);
		core_check("the frame after it plays whole", 32'd8201, 0, SFRAME);
		wait_word(C_IDX4, {16'd0, sec_head[15:0]}, 200, ok);
		check("C_IDX[4] at the head once the last word's done is back", ok, 1);
	end
endtask

// S11. a frozen clk_sys mid-frame; S12. the request withdrawn while SEC_HEAD crosses
task sec_case11;
	integer f0, s0, h1;
	begin
		$display("-- SECTOR: clk_sys stopped mid-frame: the master keeps polling; on thaw the frame completes whole");
		sec_session(16'h0C0C, 0);
		sec_frame(32'd8300, 0); sec_publish;
		f0 = c_frames; s0 = c_strobes;
		core_ask(32'd8300);
		i = 0; while (c_strobes < s0 + 80 && i < 400000) begin @(posedge clk); i = i + 1; end
		sys_run = 1'b0;
		h1 = m_hello_reads;
		repeat (20 << POLLC) @(posedge clk);
		check("the master polls while clk_sys is stopped", m_hello_reads - h1 > 4, 1);
		check("C_IDX[4] not stored while frozen", sec_tail(1'b0), 16'd0);
		sys_run = 1'b1;
		core_wait(f0, 5000, ok);
		check("on thaw the frame completes", ok, 1);
		core_check("its payload is whole", 32'd8300, 0, SFRAME);

		$display("-- SECTOR: the request withdrawn while the frame's head crosses: answered 'no request', the frame waits");
		sec_session(16'h0C0D, 0);
		f0 = c_frames;
		sec_frame(32'd8400, 0);
		stall_ra = SEC_BASE + 16'd4; stall_n = 2000; stall_r_en = 1;   // the frame's LBA word
		core_ask(32'd8400);
		repeat (4) @(posedge clk_sys);
		sec_publish;
		i = 0; while (!stall_on && i < 400000) begin @(posedge clk); i = i + 1; end
		check("the walker reads the frame's LBA for a standing request", stall_on, 1);
		core_drop;
		repeat (60 << POLLC) @(posedge clk);
		check("no strobe for the withdrawn request", c_frames, f0);
		check("the frame waits: C_IDX[4] = 0", sec_tail(1'b0), 16'd0);
		core_ask(32'd8400);
		core_wait(f0, 5000, ok);
		check("asked again, it plays", ok, 1);
		core_check("its payload", 32'd8400, 0, SFRAME);
	end
endtask

// S13. OSD and SECTOR rings interleaved; S14. faults on rings 3 and 4 drop the bind
task sec_case13;
	integer f0, k, r0;
	begin
		$display("-- SECTOR: OSD rows and SECTOR frames interleaved: both rings drain, each payload whole");
		sec_session(16'h0C0E, 1);
		osd_clear;
		for (r0 = 0; r0 < 8; r0 = r0 + 1) begin make_row(r0); spi_row(r0); end
		for (k = 0; k < 4; k = k + 1) sec_frame(32'd8500 + k, 0);
		sec_publish;
		for (r0 = 0; r0 < 8; r0 = r0 + 1) osd_publish_row(r0);
		osd_publish;
		for (k = 0; k < 4; k = k + 1) begin
			f0 = c_frames;
			core_ask(32'd8500 + k);
			core_wait(f0, 5000, ok);
			core_check($sformatf("frame %0d among the rows", k), 32'd8500 + k, 0, SFRAME);
		end
		osd_wait_tail(8 * 65, 10000, ok);
		check("every row drawn", ok, 1);
		osd_compare(bad);
		check("osd B's buffer equals osd A's", bad, 0);

		$display("-- SECTOR: a FRAME length past the head stops ring 4 (C_STATE bit 12) and drops the bind");
		sec_session(16'h0C0F, 0);
		sec_put({8'h10, 8'd0, SFRAME[15:0] + 16'd1}); sec_put(32'd8600);
		sec_publish;                               // two words published of a 590-word record
		core_ask(32'd8600);
		wait_bind0(3000, ok);
		check("C_BIND bit 0 = 0", ok, 1);
		peek(C_STATE, v); check("C_STATE bit 12", v[12], 1);
		repeat (100) @(posedge clk_sys);
		check("the mux is back on SPI", x_en, 0);
		core_drop;

		$display("-- SECTOR: a host EVT tail beyond the ring stops ring 3 (C_STATE bit 11) and drops the bind");
		sec_session(16'h0C10, 0);
		poke(H_IDX3, 32'h0000_7000);
		wait_bind0(3000, ok);
		check("C_BIND bit 0 = 0", ok, 1);
		peek(C_STATE, v); check("C_STATE bit 11", v[11], 1);
	end
endtask

// S15. the ring held part-full (never full) for many records, order and payload per record
task sec_case15;
	integer f0, k, nxt, ahead, bad15, lim;
	begin
		$display("-- SECTOR: 24 frames through a ring kept 3-10 frames ahead, never full: order and payload each");
		sec_session(16'h0C11, 0);
		nxt = 0; bad15 = 0;
		for (k = 0; k < 24; k = k + 1) begin
			lim = 3 + (($random(seed) & 32'h7FFF_FFFF) % 8);
			while (nxt < 24 && (nxt - k) < lim) begin
				ahead = ((sec_head - sec_tail(1'b0)) & 16'hFFFF);
				if (ahead + SREC > SEC_WORDS - SREC) bad15 = bad15 + 1;    // would fill: never let it
				sec_frame(32'd9000 + nxt, 2); nxt = nxt + 1;
			end
			sec_publish;
			f0 = c_frames;
			core_ask(32'd9000 + k);
			core_wait(f0, 5000, ok);
			core_check($sformatf("record %0d, in order", k), 32'd9000 + k, 2, SFRAME);
		end
		check("the ring never held more than 12 records", bad15, 0);
		wait_word(C_IDX4, {16'd0, sec_head[15:0]}, 200, ok);
		check("C_IDX[4] at the head once the last word's done is back", ok, 1);
		check("no REQUEST: every frame was ahead", rq_n, 0);
	end
endtask
`endif

// 26. an HPS reset in the middle of a row
task case26;
	begin
		$display("-- an HPS reset in the middle of a row: no store after it, the next hello works, no old byte after it");
		reset_floor;
		osd_clear;
		start_session(0, 0, 0, 16'h0C01, ok);
		osd_bind_now(16'h0C01, ok);
		make_row(0); make_row(1);
		spi_row(0);  spi_row(1);
		osd_head = 0;
		osd_publish_row(0);
		osd_publish;
		ok = 0; i = 0;
		while (!ok && i < 20000) begin
			@(posedge clk_sys); #1; i = i + 1;
			if (ocram_we === 1'b1) ok = 1;
		end
		check("the row is being drawn when the HPS reset rises", ok, 1);
		term0 = m_term;
		hps_rst_n = 0;
		wait (dut.hrst_q === 1'b1);
		repeat (8) @(posedge clk);
		txn0 = txn_starts;
		repeat (60) @(posedge clk);
		check("nothing starts while the HPS reset is seen", txn_starts - txn0, 0);
		hps_rst_n = 1;
		repeat (16) @(posedge clk);
		check("no terminal store after the reset", m_term - term0, 0);
		repeat (500) @(posedge clk_sys);
		peek(C_IDX2, v); check("the row's tail is not stored", v, 32'd0);
		// the next run's hello works, and nothing of the old row lands after its echo
		zero_page;
		osd_head = 0;
		poke(H_IDX2, 32'd0);
		run_edge(0, 0); wait_armed(1000, ok);
		poke(H_BIND, 32'h0000_0800);
		poke(H_HELLO, {16'h4F43, 16'h0C02});
		wait_echo(16'h0C02, 1000, ok);
		check("the next run's hello works", ok, 1);
		peek(C_STATE, v); check("the next hello's C_STATE bit 18", v[18], 1);
		bad = 0;
		repeat (500) begin @(posedge clk_sys); #1; if (ocram_we === 1'b1) bad = bad + 1; end
		check("no byte from the old row lands after the next echo", bad, 0);
	end
endtask

///////////////////////////////  the run  ///////////////////////////////////

// OCRAM_TB_RATE: a rate run, not a check. The bridge model answers at a fixed 10 cycles, which reproduces the rig's
// torture-sink and OSD rates; each consumer's words/s is timed from the head store to C_IDX = head, ring pre-filled.
`ifdef OCRAM_TB_RATE
real rt0, rt1, rto0, rto1, rts0, rts1;
task rate_ts;
	integer k, ok;
	begin
`ifdef MISTER_OCRAM_DIAG
		reset_floor; fast_lat = 1'b0;
		start_session(0, 0, 0, 16'h0E01, ok);
		ts_reset;
		rt0 = $realtime;
		for (k = 0; k < 16; k = k + 1) begin ts_wait_room(1001); ts_record(1000, -1, 32'd0, 0); end
		wait_ts_drain(20000, ok);
		rt1 = $realtime;
		$display("RATE tsink: %0d words in %0.1f us: %0.2f M words/s (ok %0d)", 16000, (rt1 - rt0) / 1000.0,
		         16000.0 * 1000.0 / (rt1 - rt0), ok);
`endif
	end
endtask
task rate_osd_fill;
	integer r;
	begin osd_head = 0; for (r = 0; r < 16; r = r + 1) begin make_row(r); osd_publish_row(r); end end
endtask
task rate_osd;
	integer ok;
	begin
		reset_floor; fast_lat = 1'b0;
		start_session(0, 0, 0, 16'h0E02, ok);
		osd_bind_now(16'h0E02, ok);
		rate_osd_fill;
		rto0 = $realtime; osd_publish;
		osd_wait_tail(16 * 65, 20000, ok);
		rto1 = $realtime;
		$display("RATE osd alone: 16 rows (1,024 payload words) in %0.1f us: %0.2f us a row, %0.0f ns a word, %0.2f M words/s (ok %0d)",
		         (rto1 - rto0) / 1000.0, (rto1 - rto0) / 16000.0, (rto1 - rto0) / 1024.0, 1024.0 * 1000.0 / (rto1 - rto0), ok);
	end
endtask
`ifdef MISTER_OCRAM_SECTOR_SLOT
localparam integer RNF = 8192 / (SFRAME + 2);
task rate_sec_core;
	integer k, f0, ok;
	begin
		for (k = 0; k < RNF; k = k + 1) begin f0 = c_frames; core_ask(32'd3000 + k); core_wait(f0, 20000, ok); end
		rts1 = $realtime;
	end
endtask
task rate_sec(input integer both);
	integer k, ok;
	begin
		sec_session(both ? 16'h0E04 : 16'h0E03, both);
		for (k = 0; k < RNF; k = k + 1) sec_frame(32'd3000 + k, 0);
		sec_publish;
		if (both) rate_osd_fill;
		rts0 = $realtime; rto0 = rts0;
		fork
			rate_sec_core;
			if (both) begin osd_publish; osd_wait_tail(16 * 65, 20000, ok); rto1 = $realtime; end
		join
		$display("RATE sector %0s: %0d frames (%0d payload words) in %0.1f us: %0.0f ns a word, %0.2f M words/s",
		         both ? "beside osd" : "alone", RNF, RNF * SFRAME, (rts1 - rts0) / 1000.0, (rts1 - rts0) / (RNF * SFRAME),
		         RNF * SFRAME * 1000.0 / (rts1 - rts0));
		if (both) begin
			$display("RATE osd beside sector: 1,024 payload words in %0.1f us: %0.2f us a row, %0.2f M words/s",
			         (rto1 - rto0) / 1000.0, (rto1 - rto0) / 16000.0, 1024.0 * 1000.0 / (rto1 - rto0));
			$display("RATE aggregate while both ran: %0.2f M words/s over the osd's %0.1f us",
			         (1024.0 + SFRAME * (rto1 - rto0) * 1.0 * RNF / (rts1 - rts0)) * 1000.0 / (rto1 - rto0), (rto1 - rto0) / 1000.0);
		end
	end
endtask
`endif
task rate_run;
	begin
		$display("RATE clk_sys half period %0.3f ns", SYS_HP);
		rate_ts;
`ifdef MISTER_OCRAM_OSD
		rate_osd;
`endif
`ifdef MISTER_OCRAM_SECTOR_SLOT
		rate_sec(0);
		rate_sec(1);
`endif
		$finish;
	end
endtask
`endif

initial begin
	if (!$value$plusargs("lw_seed=%d", seed)) seed = 1;
	$display("ocram_tb: seed %0d, POLL %0d, MARK %0d, DIAG %0d, OSD %0d, ROWS %0d",
	         seed, POLLC, MARKC, TS_B, OSD_B, ROWS);
	hps_rst_n = 0;
	gpo_reg = 32'd0;
	mode = M_NORMAL;
	repeat (20) @(posedge clk);
	hps_rst_n = 1;
	repeat (20) @(posedge clk);
	gpo_reg = GPO_IO;

	$display("-- present: the floor drives GPI bit 26");
	check("present is the constant 1", present_wire, 1);
`ifdef OCRAM_TB_RATE
	rate_run;
`endif

	case1;
	case2;
	case3;
	case4;
`ifdef MISTER_OCRAM_DIAG
	case5;
`endif
	case6;
	case7;
`ifdef MISTER_OCRAM_DIAG
	case8;
	case9;
	case10;
`endif
	case11;
`ifdef MISTER_OCRAM_DIAG
	case12;
	case13;
`endif
	case14;
	case15;
	case16;
	case17;
	case18;
	case19;
	case20;
	case21;
`ifdef MISTER_OCRAM_DIAG
	case22;
	case23;
	case24;
	case25;
	case27;
`endif
	case26;
`ifdef MISTER_OCRAM_SECTOR_SLOT
	sec_case1;
	sec_case2;
	sec_case3;
	sec_case4;
	sec_case6;
	sec_case7;
	sec_case8;
	sec_case9;
	sec_case11;
	sec_case13;
	sec_case15;
`endif

	$display("-- the protocol monitor over the whole suite");
	check("protocol invariant failures", mon_err, 0);
	check("present at the end", present_wire, 1);

	if (fails) begin
		$display("FAILED: %0d checks, %0d failures", checks, fails);
		$fatal(1, "ocram_tb: failures");
	end
	else begin
		$display("PASSED: %0d checks, 0 failures", checks);
		$finish;
	end
end

initial begin #100000000 $display("FAILED: timeout"); $finish; end

endmodule

`default_nettype wire
