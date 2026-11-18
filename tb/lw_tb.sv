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
// LW write path and both osds' osd_buffer, driven through a stand-in for the HPS LW bridge atom (lw_stub_atom.sv).
// The tier macros choose the blocks (MISTER_LW_PAD, _EVT, _SECTOR_SLOT, _LANE, _OSD): a case that needs a block
// runs only where it is built, and the directory, the decode and the lines are checked against the set built.
// HPS_BUS[383:46] is the window's one port with a weak pull-down, as synthesis ties a net with no source low.
// LW_TB_TIER runs the short tier run (arm T, and arm B with a core side that leaves pieces out: LW_TB_BUS46, a
// 46-bit bus at the core; LW_TB_NOMUX, no SECTOR mux; LW_TB_NOSAVE1, save slot 1's mux missing; LW_TB_NOMARK,
// hps_io's commit mark missing).
// The reference configuration is a CD-style core: a 588-word primary frame on hps_io slot 1 and two 1 KB save
// slots on hps_io slots 2 and 3, each with its image in DDR. In the cases' text "slot k" is the block's slot index
// (0 the primary, the CD; 1 and 2 the save slots, sv1 and sv2; "save slot 1" is MISTER_LW_SAVE1's, slot index 2).
// Without MISTER_LW_SECTOR_SLOT the window has no SECTOR block and the consumers meet hps_io directly (the
// Template's own configuration); the directory, the block's addresses and the doorbell must then be silent.
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

// The checker bound to each enable link (lw_fence): its level rises only on an acknowledge of rest, and falls only
// after the acknowledge has read 0 since its rise; the acknowledge never reads 0 in IDLE (off: a case that
// scrambles or forces a link); no store strobe reaches the block outside OPEN. Sampled at every clk_100m negedge.
module fence_chk(input clk, input off, input [2:0] st, input lvl, input ack_s, input stb);
integer n_rise = 0, n_fall = 0, n_bad = 0, n_stb = 0;
reg     lvl_p = 0, ack_p = 0, seen0 = 0, init = 0;
task bad(input string what);
	begin
		n_bad = n_bad + 1;
		if (n_bad <= 6) $display("   enable link %m at %0t: %s", $time, what);
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

// A save-slot consumer (the usual HPS save handshake): rd or wr with the LBA, both dropped on ack; the
// transfer ends when ack falls. The buffer (dpram port B) is written on write & ack at addr, one halfword or byte
// (din[7:0]) per address as hps_io's WIDE says. hold (the cases set it): 1 keeps the request raised through the
// ack, 2 drops it in the cycle after the ack falls. acks counts falls; amin is the fewest clk_sys cycles seen
// between two strobes, or from an ack's rise to its first strobe (alead).
module sv_model(input clk, input reset, output reg rd = 0, output reg wr = 0, output reg [31:0] lba = 0,
                input ack, input write, input [13:0] addr, input [15:0] din, input [13:0] haddr, output reg [15:0] dout);
reg  [15:0] ram [0:16383];
integer     since = 1000, amin = 1000, alead = 1000, n_ack = 0;
reg         first = 0;
integer     strobes = 0, done = 0, acks = 0, hold = 0;
reg         busy = 0, seen = 0, ack_d = 0;
always @(posedge clk) begin
	dout <= ram[haddr];
	ack_d <= ack;
	if (ack_d && !ack) acks <= acks + 1;
	since <= since + 1;
	if (ack && !ack_d) begin since <= 1; first <= 1; n_ack <= n_ack + 1; end
	if (write && ack) begin
		ram[addr] <= din;
		strobes   <= strobes + 1;
		since     <= 1;
		first     <= 0;
		if (first) begin if (since < alead) alead <= since; end
		else if (since < amin) amin <= since;
	end
	if (hold == 0 ? ack : hold == 2 && ack_d && !ack) begin rd <= 0; wr <= 0; end
	if (reset) begin rd <= 0; wr <= 0; busy <= 0; seen <= 0; end
	else if (busy) begin
		if (ack) seen <= 1;
		else if (seen) begin busy <= 0; seen <= 0; done <= done + 1; end
	end
end
task go(input is_wr, input [31:0] l);
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
wire  [2:0] lines;           // line ordinal k = lines[k-1]
wire        doorbell = lines[0];   // ordinal 1: SECTOR's entry
wire        sys_irq  = lines[1];   // ordinal 2: SYS's entry
wire        osd_we;          // the OSDBUF writer's byte to both osds, on clk_sys
wire [12:0] osd_addr;
wire  [7:0] osd_byte;

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

// The register bus's store pulses, the dropped burst beats, the OSD image stores queued and the bytes the OSD
// writer wrote, counted.
integer n_lwwr = 0, n_lwwb = 0, n_osdq = 0, n_osdwe = 0, n_padwe = 0;
always @(posedge clk_sys) begin
	if (lw_pad[48]) n_padwe = n_padwe + 1;
	if (osd_we)     n_osdwe = n_osdwe + 1;
end
always @(posedge clk) begin
	if (lw_write)  n_lwwr  = n_lwwr + 1;
	if (lw_wburst) n_lwwb  = n_lwwb + 1;
`ifdef MISTER_LW_OSD
	if (win.osd.osd.st && !win.osd.osd.full) n_osdq = n_osdq + 1;
`endif
end

// The tiers built (_B), and listed: a built tier whose core-side piece is present.
`ifdef MISTER_LW_PAD
localparam TB_PADB = 1;
`else
localparam TB_PADB = 0;
`endif
`ifdef MISTER_LW_EVT
localparam TB_EVTB = 1;
`else
localparam TB_EVTB = 0;
`endif
`ifdef MISTER_LW_SECTOR_SLOT
localparam TB_SECB = 1;
`else
localparam TB_SECB = 0;
`endif
`ifdef MISTER_LW_OSD
localparam TB_OSD = 1;
`else
localparam TB_OSD = 0;
`endif
`ifdef LW_TB_BUS46
localparam TB_BUS46 = 1;
`else
localparam TB_BUS46 = 0;
`endif
`ifdef LW_TB_NOMUX
localparam TB_NOMUX = 1;
`else
localparam TB_NOMUX = 0;
`endif
`ifdef LW_TB_NOSAVE1
localparam TB_NOSAVE1 = 1;
`else
localparam TB_NOSAVE1 = 0;
`endif
`ifdef LW_TB_NOMARK
localparam TB_NOMARK = 1;
`else
localparam TB_NOMARK = 0;
`endif
localparam TB_PAD = TB_PADB && !TB_BUS46;
localparam TB_EVT = TB_EVTB && !TB_BUS46;
localparam TB_CMT = TB_SECB && !TB_BUS46 && !TB_NOMARK;    // hps_io's commit mark reaches the window
localparam TB_SEC = TB_CMT && !TB_NOMUX;
localparam TB_W1  = TB_SEC && !TB_NOSAVE1;    // save slot 1 wired
// The geometry. The primary's frame, and save slot 1's (slot index 2): its own length, shorter than save slot 0's
// 256 words, so each save slot is served at its own length (both 256 at the default geometry). TB_WIDE is hps_io's
// WIDE; the primary and the save slots sit on hps_io slots TB_PS, TB_S1H and TB_S2H of TB_VD; TB_LBAB is the save
// slots' LBA width. LW_TB_GEOMW (arm W): byte strobes, a 4096-word primary on hps_io slot 9, save slots of 510 and
// 256 words with 32-bit LBAs on slots 12 and 15, strobes 5 cycles apart.
`ifdef LW_TB_DEFGEOM
localparam TB_FRAME = 128;
localparam [12:0] TB_XFRAME0 = 13'd256, TB_XFRAME1 = 13'd256;
localparam TB_WIDE = 1, TB_VD = 4, TB_PS = 1, TB_S1H = 2, TB_S2H = 3, TB_GAP = 2, TB_LBAB = 32;
`elsif LW_TB_GEOMW
localparam TB_FRAME = 4096;
localparam [12:0] TB_XFRAME0 = 13'd510, TB_XFRAME1 = 13'd256;
localparam TB_WIDE = 0, TB_VD = 16, TB_PS = 9, TB_S1H = 12, TB_S2H = 15, TB_GAP = 5, TB_LBAB = 32;
`else
localparam TB_FRAME = 588;
localparam [12:0] TB_XFRAME0 = 13'd256, TB_XFRAME1 = 13'd192;
localparam TB_WIDE = 1, TB_VD = 4, TB_PS = 1, TB_S1H = 2, TB_S2H = 3, TB_GAP = 2, TB_LBAB = 7;
`endif
`ifdef LW_TB_GEOMW
localparam TB_GEOMW = 1;
`else
localparam TB_GEOMW = 0;
`endif
localparam TB_DW = TB_WIDE ? 15 : 7, TB_AW = TB_WIDE ? 12 : 13;   // hps_io's sd_buff_dout and sd_buff_addr
localparam [31:0] TB_LBAM = (TB_LBAB >= 32) ? 32'hFFFFFFFF : (32'd1 << TB_LBAB) - 32'd1;   // a save slot's LBA bits
`ifdef MISTER_LW_LANE
localparam TB_LANE = 1;
`else
localparam TB_LANE = 0;
`endif
localparam TB_COUNT = TB_PAD + TB_SEC + TB_OSD + TB_EVT + TB_LANE;
// SECTOR at 0x48000, register map 2: the common words, then slot k's at 0x100 + 0x20k, DATA from 0x4000
localparam [20:0] SEC = 21'h48000;
localparam [20:0] S_CAUSE = 21'h000, S_REQ = 21'h004, S_CTRL = 21'h008, S_STATUS = 21'h00C, S_ACCEPT = 21'h010,
                  S_ID = 21'h014, S_DROPS = 21'h018, S_COUNTS = 21'h01C, S_RINGS = 21'h020, S_TSREQ = 21'h024,
                  S_TSDONE = 21'h028, S_CYCLES = 21'h02C, S_SUM = 21'h030, S_DATA = 21'h4000;
localparam [20:0] S_RSTAT0 = 21'h100, S_LBA0 = 21'h104, S_BEGIN0 = 21'h108,
                  S_RSTAT1 = 21'h120, S_LBA1 = 21'h124, S_BEGIN1 = 21'h128, S_WACK1 = 21'h12C,
                  S_RSTAT2 = 21'h140, S_LBA2 = 21'h144, S_BEGIN2 = 21'h148, S_WACK2 = 21'h14C;
localparam [31:0] C_EN0 = 32'h001, C_EN1 = 32'h002, C_EN2 = 32'h004, C_ABORT = 32'h100, C_TEST = 32'h200,
                  C_CLR = 32'h400;                               // CTRL: EN by slot index k, ABORT, TEST, CLR
`ifdef MENU_CORE
localparam TB_MENU = 1;          // osd_buffer has the header rows 16-19
`else
localparam TB_MENU = 0;
`endif

// HPS_BUS as sys_top wires it: the window's hps_lw is [383:46]; the SPI side is driven by the tb (io_*). A bit no
// piece drives reads 0 (the pull-down), as synthesis ties it.
reg         io_enable = 0, io_strobe = 0;
reg  [15:0] io_din = 0;
wire [383:0] hps_bus;
assign (weak0, weak1) hps_bus[383:46] = 338'd0;
assign hps_bus[45:38] = 0;
assign hps_bus[35:33] = {m_sel[1], m_sel[0], m_strobe};
assign hps_bus[31:16] = m_din;
// The to-core fields the cases watch, from the bus: the PAD store, the primary's and the save slots' lines.
wire [48:0] lw_pad  = hps_bus[94:46];
wire [18:0] blk_dn  = {hps_bus[95], hps_bus[96], hps_bus[97], hps_bus[113:98]};             // {en_sys, ack, wr, data}
wire [12:0] save_dn = {hps_bus[131], hps_bus[130], hps_bus[129], hps_bus[128], hps_bus[122:114]};
                                                                     // {ack1, en_sys1, ack0, en_sys0, addr[8:0]}

lw_sys_window #(
	.PAD_EN(TB_PADB), .EVT_EN(TB_EVTB), .OSD_EN(TB_OSD),
	.SEC_EN(TB_SECB), .SEC_SLOT(TB_PS), .SEC_FRAME(TB_FRAME), .SEC_GAP(TB_GAP), .BLKSZ(3'd3),
	.SAVE0_EN(1), .SAVE0_SLOT(TB_S1H), .SAVE0_WSRC(32'h30100000), .SAVE0_EXT(32'h00020000), .SAVE0_FRAME(TB_XFRAME0),
	.SAVE0_LBA_BITS(TB_LBAB),
	.SAVE1_EN(1), .SAVE1_SLOT(TB_S2H), .SAVE1_WSRC(32'h30200000), .SAVE1_EXT(32'h00020000), .SAVE1_FRAME(TB_XFRAME1),
	.SAVE1_LBA_BITS(TB_LBAB),
	.LANE_EN(TB_LANE)
) win
(
	.clk(clk),
	.lw_raddr(lw_raddr), .lw_rdata(lw_rdata),
	.lw_waddr(lw_waddr), .lw_write(lw_write), .lw_wdata(lw_wdata), .lw_be(lw_be), .lw_wburst(lw_wburst),
	.lw_hrst(lw_hrst), .lw_freed(lw_freed), .lw_idv(lw_idv), .held(lw_held),
	.lines(lines),
	.clk_sys(clk_sys), .hps_lw(hps_bus[383:46]), .io_wide(hps_bus[32]),
	.osd_sel(m_sel[3:2]), .osd_view(gpo31_s), .osd_we(osd_we), .osd_addr(osd_addr), .osd_byte(osd_byte),
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

// Both osd instances as sys_top wires them: one SPI select (the HPS drives ss0 and ss2 together), one LW write port.
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
	.dout(ho_d), .de_out(ho_de), .vs_out(ho_vs), .hs_out(ho_hs)
`ifdef MISTER_LW_OSD
	, .lw_we(osd_we), .lw_addr(osd_addr), .lw_byte(osd_byte)
`endif
);

osd vga_osd
(
	.clk_sys(clk_sys), .io_osd(m_sel[3]), .io_strobe(m_strobe), .io_din(m_din), .osd_status(osd_status),
	.clk_video(clk_vid), .din(24'd0), .de_in(v_de), .vs_in(v_vs), .hs_in(v_hs),
	.dout(vo_d), .de_out(vo_de), .vs_out(vo_vs), .hs_out(vo_hs)
`ifdef MISTER_LW_OSD
	, .lw_we(osd_we), .lw_addr(osd_addr), .lw_byte(osd_byte)
`endif
);

osd_cap cap_h(.clk(clk_vid), .d(ho_d), .de(ho_de), .vs(ho_vs));
osd_cap cap_v(.clk(clk_vid), .d(vo_d), .de(vo_de), .vs(vo_vs));
// The pixel path's interlace flag has no initial value in osd.v (a register powers up 0 on the device); the
// simulation gives it one through the block name LW_TB_PROBES adds, or no frame ever shows the OSD.
`ifdef LW_TB_PROBES
initial begin hdmi_osd.pix.f1 = 1'b0; vga_osd.pix.f1 = 1'b0; end
`endif

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
// 25 the firewall's present, 24 its idle, 22 the window's held. Bit 22 is compared with en_q in every cycle.
wire [31:0] tb_gpi = !gpo31 ? 32'h5CA623A4 : {6'd0, fw_present, fw_idle, 1'b0, lw_held, 22'd0};
integer gpi22_n = 0, gpi22_bad = 0, gpi22_on = 0;
always @(negedge clk) begin
	gpi22_n = gpi22_n + 1;
	if (gpo31 && tb_gpi[22] !== win.en_q) gpi22_bad = gpi22_bad + 1;
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


localparam CONF_STR = {"CORE;", `SYS_LW_CAPS "SS3E000000:400000;", "V,v1;"};

wire [31:0] joy0, joy1, joy2, joy3, joy4, joy5;
wire [95:0] lstk, rstk;                 // joystick_l_analog_0-5, joystick_r_analog_0-5, 16 bits each
wire [47:0] pdl;                        // paddle_0-5
wire [53:0] spn;                        // spinner_0-5, {toggle, data}
wire [15:0] ana0 = lstk[15:0], ana3 = lstk[63:48], rana0 = rstk[15:0], rana3 = rstk[63:48];
wire  [7:0] pdl1 = pdl[15:8];
reg         cd_reset = 0;
reg         cd_req = 0;
reg  [31:0] cd_lba = 32'hFFFFFFFF;
wire        cd_ack, cd_wr;
wire [15:0] cd_data;
wire [TB_VD-1:0] sd_rd, sd_wr, sd_ack;
reg         blk0_rd = 0;                // a consumer on hps_io slot 0 that holds its request (the EVT cases)
wire [TB_DW:0] sd_buff_dout;
wire        sd_buff_wr;
wire [TB_AW:0] sd_buff_addr;
wire [31:0] sd_lba [TB_VD];
wire [TB_DW:0] sd_buff_din [TB_VD];
wire [13:0] sd_addr14 = sd_buff_addr;   // hps_io's buffer address and data as the muxes take them
wire [15:0] sd_dout16 = sd_buff_dout;
assign sd_rd[0] = blk0_rd;
assign sd_wr[0] = 0;
assign sd_lba[0] = 0;
assign sd_buff_din[0] = 0;
// the primary on hps_io slot TB_PS (read only), the save slots on TB_S1H and TB_S2H; every other slot idle
assign sd_wr[TB_PS] = 0;
assign sd_lba[TB_PS] = TB_GEOMW ? pm.lba : cd_lba;
assign sd_lba[TB_S1H] = sv1.lba;
assign sd_lba[TB_S2H] = sv2.lba;
assign sd_buff_din[TB_PS] = 0;
assign sd_buff_din[TB_S1H] = sv1.dout[TB_DW:0];
assign sd_buff_din[TB_S2H] = sv2.dout[TB_DW:0];
genvar gs;
generate for (gs = 1; gs < TB_VD; gs = gs + 1) begin : idle_slot
	if (gs != TB_PS && gs != TB_S1H && gs != TB_S2H) begin : idle
		assign sd_rd[gs] = 0, sd_wr[gs] = 0, sd_lba[gs] = 0, sd_buff_din[gs] = 0;
	end
end
endgenerate

// The core's status request: a rising edge of status_set latches status_in.
reg [127:0] st_in = 0;
reg         st_set = 0;
// hps_io's status flag: at module scope with the events tier, in uio_block without it.
`ifdef MISTER_LW_EVT
`define TB_STFLG hps_io.stflg
`else
`define TB_STFLG hps_io.uio_block.stflg
`endif
// The core's info and upload requests: a rising edge of each (hps_io's 0x36 and 0x3C).
reg         info_rq = 0, up_rq = 0;
reg   [7:0] info_v = 8'h00;

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

// The core side's view of HPS_BUS: the whole bus, or with LW_TB_BUS46 a core whose emu port is still 46 bits, so
// its bits above 45 go nowhere.
`ifdef LW_TB_BUS46
wire [383:46] core_hi;
`define TB_CORE_BUS {core_hi, hps_bus[45:0]}
`else
`define TB_CORE_BUS hps_bus
`endif
// hps_io's own view: with LW_TB_NOMARK an hps_io whose commit mark [381:380] goes nowhere.
`ifdef LW_TB_NOMARK
wire   [1:0] mark_cut;
`define TB_IO_BUS {hps_bus[383:382], mark_cut, hps_bus[379:0]}
`else
`define TB_IO_BUS `TB_CORE_BUS
`endif

hps_io #(.CONF_STR(CONF_STR), .WIDE(TB_WIDE), .VDNUM(TB_VD), .BLKSZ(3)) hps_io
(
	.clk_sys(clk_sys),
	.HPS_BUS(`TB_IO_BUS),
`ifdef MISTER_LW_LANE
	.ioctl_wait(ioctl_wait), .status(hst), .RTC(rtc), .TIMESTAMP(tstamp), .img_mounted(img_m), .img_readonly(img_ro),
	.img_size(img_sz), .gamma_bus(gbus), .sdram_sz(sdsz), .uart_mode(umode), .uart_speed(uspeed), .EXT_BUS(ebus),
`endif
	.status_in(st_in), .status_set(st_set), .info_req(info_rq), .info(info_v), .ioctl_upload_req(up_rq),
	.ioctl_upload_index(8'h5C),
	.joystick_0(joy0), .joystick_1(joy1), .joystick_2(joy2), .joystick_3(joy3), .joystick_4(joy4), .joystick_5(joy5),
	.joystick_l_analog_0(lstk[15:0]), .joystick_l_analog_1(lstk[31:16]), .joystick_l_analog_2(lstk[47:32]),
	.joystick_l_analog_3(lstk[63:48]), .joystick_l_analog_4(lstk[79:64]), .joystick_l_analog_5(lstk[95:80]),
	.joystick_r_analog_0(rstk[15:0]), .joystick_r_analog_1(rstk[31:16]), .joystick_r_analog_2(rstk[47:32]),
	.joystick_r_analog_3(rstk[63:48]), .joystick_r_analog_4(rstk[79:64]), .joystick_r_analog_5(rstk[95:80]),
	.paddle_0(pdl[7:0]), .paddle_1(pdl[15:8]), .paddle_2(pdl[23:16]), .paddle_3(pdl[31:24]), .paddle_4(pdl[39:32]),
	.paddle_5(pdl[47:40]),
	.spinner_0(spn[8:0]), .spinner_1(spn[17:9]), .spinner_2(spn[26:18]), .spinner_3(spn[35:27]),
	.spinner_4(spn[44:36]), .spinner_5(spn[53:45]),
	.sd_lba(sd_lba), .sd_rd(sd_rd), .sd_wr(sd_wr), .sd_ack(sd_ack),
	.sd_buff_dout(sd_buff_dout), .sd_buff_din(sd_buff_din), .sd_buff_wr(sd_buff_wr), .sd_buff_addr(sd_buff_addr)
);

// Two save slots on hps_io slots TB_S1H and TB_S2H, each behind an lw_save_mux when the SECTOR block is built (no
// mux with LW_TB_NOMUX; save slot 1's missing with LW_TB_NOSAVE1). With LW_TB_GEOMW the primary is pm, a consumer of
// the same kind that keeps every strobe at its address, read only; otherwise the CD below.
wire        sv1_ack, sv1_write, sv2_ack, sv2_write, pm_ack, pm_write;
wire [13:0] sv1_addr, sv2_addr, pm_addr, cd_addr;
wire [15:0] sv1_din, sv2_din, pm_din;
sv_model sv1(.clk(clk_sys), .reset(cd_reset), .ack(sv1_ack), .write(sv1_write), .addr(sv1_addr), .din(sv1_din),
             .haddr(sd_addr14));
sv_model sv2(.clk(clk_sys), .reset(cd_reset), .ack(sv2_ack), .write(sv2_write), .addr(sv2_addr), .din(sv2_din),
             .haddr(sd_addr14));
sv_model pm(.clk(clk_sys), .reset(cd_reset), .ack(pm_ack), .write(pm_write), .addr(pm_addr), .din(pm_din),
            .haddr(sd_addr14));
wire        p_req   = TB_GEOMW ? pm.rd : cd_req;      // the primary consumer's request, ack, strobe and LBA
wire [31:0] p_lba   = TB_GEOMW ? pm.lba : cd_lba;
wire        p_ack, p_wr;
wire [13:0] p_addr;
wire [15:0] p_data;
assign pm_ack = TB_GEOMW ? p_ack : 1'b0, pm_write = TB_GEOMW ? p_wr : 1'b0, pm_addr = p_addr, pm_din = p_data;
assign cd_ack = TB_GEOMW ? 1'b0 : p_ack, cd_wr = TB_GEOMW ? 1'b0 : p_wr, cd_addr = p_addr, cd_data = p_data;
`ifdef MISTER_LW_SECTOR_SLOT
`ifndef LW_TB_NOMUX
`define TB_MUX
`endif
`endif
`ifdef TB_MUX
lw_save_mux #(.SAVE(0)) sv1_lw
(
	.HPS_BUS(`TB_CORE_BUS),
	.core_rd(sv1.rd), .core_wr(sv1.wr), .core_lba(sv1.lba), .core_ack(sv1_ack), .core_write(sv1_write),
	.core_addr(sv1_addr), .core_data(sv1_din),
	.hps_rd(sd_rd[TB_S1H]), .hps_wr(sd_wr[TB_S1H]), .hps_ack(sd_ack[TB_S1H]), .hps_write(sd_buff_wr),
	.hps_addr(sd_addr14), .hps_data(sd_dout16)
);
`ifdef LW_TB_NOSAVE1
assign sd_rd[TB_S2H] = sv2.rd, sd_wr[TB_S2H] = sv2.wr, sv2_ack = sd_ack[TB_S2H], sv2_write = sd_buff_wr;
assign sv2_addr = sd_addr14, sv2_din = sd_dout16;
`else
lw_save_mux #(.SAVE(1)) sv2_lw
(
	.HPS_BUS(`TB_CORE_BUS),
	.core_rd(sv2.rd), .core_wr(sv2.wr), .core_lba(sv2.lba), .core_ack(sv2_ack), .core_write(sv2_write),
	.core_addr(sv2_addr), .core_data(sv2_din),
	.hps_rd(sd_rd[TB_S2H]), .hps_wr(sd_wr[TB_S2H]), .hps_ack(sd_ack[TB_S2H]), .hps_write(sd_buff_wr),
	.hps_addr(sd_addr14), .hps_data(sd_dout16)
);
`endif

// The primary (the CD, or pm) on hps_io slot TB_PS, behind an lw_blk_mux when the SECTOR block is built.
lw_blk_mux cd_lw
(
	.HPS_BUS(`TB_CORE_BUS), .reset(cd_reset),
	.core_req(p_req), .core_lba(p_lba), .core_ack(p_ack), .core_wr(p_wr), .core_addr(p_addr), .core_data(p_data),
	.hps_req(sd_rd[TB_PS]), .hps_ack(sd_ack[TB_PS]), .hps_wr(sd_buff_wr), .hps_addr(sd_addr14), .hps_data(sd_dout16)
);
`undef TB_MUX
`else
// No SECTOR block, or no mux: each consumer meets hps_io directly, as in a core that declares none.
assign sd_rd[TB_S1H] = sv1.rd, sd_wr[TB_S1H] = sv1.wr, sv1_ack = sd_ack[TB_S1H], sv1_write = sd_buff_wr;
assign sv1_addr = sd_addr14, sv1_din = sd_dout16;
assign sd_rd[TB_S2H] = sv2.rd, sd_wr[TB_S2H] = sv2.wr, sv2_ack = sd_ack[TB_S2H], sv2_write = sd_buff_wr;
assign sv2_addr = sd_addr14, sv2_din = sd_dout16;
assign sd_rd[TB_PS] = p_req, p_ack = sd_ack[TB_PS], p_wr = sd_buff_wr, p_addr = sd_addr14, p_data = sd_dout16;
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

// Save slot 0 (slot index 1) on hps_io slot TB_S1H, which gates the shared strobe with its own ack.
integer sv_words = 0, sv_sum = 0;
always @(posedge clk_sys) if (sd_buff_wr && sd_ack[TB_S1H]) begin
	sv_words <= sv_words + 1;
	sv_sum   <= sv_sum + sd_buff_dout;
end

// One request, one path: a request of the CD, slot 1 or slot 2 begins at a rise of the core's request, or at a fall
// of the core's ack that leaves it raised. ap_both counts requests that both hps_io (sd_ack) and the feeder (the
// slot's LW ack) acknowledged; ap_spi and ap_lw count each path's acks. Throughout the suite, except from a
// scramble of the clk_sys registers (ap_scr, which sets the acks themselves at random) until its case ends (fc_off).
reg       fc_off = 0;                   // a case scrambles or forces a link
integer   ap_both = 0, ap_spi = 0, ap_lw = 0;
reg       ap_scr = 0;
reg [2:0] ap_up_d = 0, ap_ack_d = 0, ap_sa_d = 0, ap_la_d = 0, ap_s = 0, ap_l = 0;
wire [2:0] ap_up  = {sv2.rd | sv2.wr, sv1.rd | sv1.wr, p_req};
wire [2:0] ap_ack = {sv2_ack, sv1_ack, p_ack};
wire [2:0] ap_sa  = {sd_ack[TB_S2H], sd_ack[TB_S1H], sd_ack[TB_PS]};
wire [2:0] ap_la  = {hps_bus[131], hps_bus[129], hps_bus[96]};
always @(posedge clk_sys) begin : ap_mon
	integer k;
	if (!fc_off) ap_scr = 0;
	for (k = 0; k < 3; k = k + 1) begin
		if (ap_scr) begin ap_s[k] = 0; ap_l[k] = 0; end
		if ((ap_up[k] === 1'b1 && !ap_up_d[k]) || (ap_ack_d[k] && ap_ack[k] !== 1'b1 && ap_up[k] === 1'b1)) begin
			ap_s[k] = 0; ap_l[k] = 0;
		end
		if (ap_sa[k] === 1'b1 && !ap_sa_d[k]) begin
			ap_spi = ap_spi + 1;
			if (ap_l[k] && !ap_s[k]) begin
				ap_both = ap_both + 1; $display("     slot %0d's request acknowledged over LW, then over SPI, at %0t", k + 1, $time);
			end
			ap_s[k] = 1;
		end
		if (ap_la[k] === 1'b1 && !ap_la_d[k]) begin
			ap_lw = ap_lw + 1;
			if (ap_s[k] && !ap_l[k]) begin
				ap_both = ap_both + 1; $display("     slot %0d's request acknowledged over SPI, then over LW, at %0t", k + 1, $time);
			end
			ap_l[k] = 1;
		end
		ap_up_d[k] = ap_up[k] === 1'b1; ap_ack_d[k] = ap_ack[k] === 1'b1;
		ap_sa_d[k] = ap_sa[k] === 1'b1; ap_la_d[k] = ap_la[k] === 1'b1;
	end
end

// Reads have no side effect: the state a read could move, by hierarchical reference. With clk_sys stopped nothing
// else moves it, so it must be identical before and after any reads.
function [511:0] pure_win;
	reg [95:0] p;
	reg [167:0] e;
	reg [33:0] o;
	begin
		p = 0; e = 0; o = 0;
`ifdef MISTER_LW_OSD
		o = {win.osd.osd.wptr, win.osd.osd.osd_w_lwgray, win.osd.osd.drops};
`endif
`ifdef MISTER_LW_PAD
		// (not the scan pointer: it moves in every cycle the handshake is idle)
		p = {win.pad.pad.sdrop, win.pad.pad.dirty, win.pad.pad.awp, win.pad.pad.arp, win.pad.pad.ds, win.pad.pad.psel};
`endif
`ifdef MISTER_LW_EVT
		e = {win.evt.evt.seq, win.evt.evt.cause, win.evt.evt.s_q, win.evt.evt.e_q};
`endif
		pure_win = {180'd0, win.scratch, o, doorbell, sys_irq, p, e};
	end
endfunction
function [511:0] pure_sec;
`ifdef MISTER_LW_SECTOR_SLOT
	pure_sec = {169'd0, win.sec.sector.cause, win.sec.sector.rings, win.sec.sector.ans, win.sec.sector.btag,
	           win.sec.sector.rq_off0, win.sec.sector.rq_rd0, win.sec.sector.rq_lba, win.sec.sector.rq_xoff,
	           win.sec.sector.rq_xrd,
	           win.sec.sector.rq_xwr, win.sec.sector.rq_xlba, win.sec.sector.abort_pend, win.sec.sector.wspec,
	           win.sec.sector.wcom, win.sec.sector.wvis, win.sec.sector.wsl,
	           win.sec.sector.exp_w,
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
	pure_lane = {134'd0, win.lane.lane.wspec, win.lane.lane.wcom, win.lane.lane.wvis, win.lane.lane.open, win.lane.lane.ovf,
	            win.lane.lane.played, win.lane.lane.drops, win.lane.lane.rrp, win.lane.lane.rhead,
	            win.lane.lane.rdrops, win.lane.lane.pt_d, win.lane.lane.rdt_d, win.lane.lane.rwg_d, win.lane.lane.cause};
`else
	pure_lane = 0;
`endif
endfunction

// The readable view: every word of the directory and the blocks (0x40000-0x47FFC), every word of SECTOR's registers
// (0x48000-0x481FC), then one word per KB of the rest of the window. SECTOR's CYCLES counts clk_100m cycles and is
// left out.
localparam PURE_N = 8192 + 128 + 480;
reg [31:0] pure_v [0:PURE_N-1];
function [20:0] pure_addr(input integer i);
	integer j, r;
	begin
		r = i - 8192;
		j = i - 8192 - 128;
		pure_addr = (i < 8192) ? 21'h40000 + {i[18:0], 2'b00} : (i < 8192 + 128) ? 21'h48000 + {r[6:0], 2'b00} :
		            21'h48000 + {j[10:0], 10'd0};
	end
endfunction
task pure_view(input do_cmp, output integer nbad);
	integer m, nb;
	reg [31:0] v, vl;
	begin
		nbad = 0;
		for (m = 0; m < PURE_N; m = m + 1) begin
			axi_read(pure_addr(m), 0, v, vl, nb);
			if (pure_addr(m) == SEC + S_CYCLES || pure_addr(m) == ENA) v = 0;
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

// ---- The host enable, and the bench's own host ----
// ENABLE store: one full word, the key and bit 0. Nothing but an ID view, a core reset through GPO or an HPS reset
// clears it: the bench's host ends its enable with an ID view (host_end), as its successor's start would.
localparam [20:0] SYSB = 21'h43000;               // the SYS block
localparam [20:0] ENA = 21'h40018, OWNEDA = 21'h40024, OPENA = 21'h4002C, LINEN = 21'h40030, RESETSA = 21'h40034;
localparam [31:0] ENA_W = 32'h4C570001;
reg bus_busy = 0;
task bus_get; bus_busy = 1; endtask
// A program's identification of the core: the ID view for n clk_100m cycles, then the I/O view again.
task identify(input integer n); begin @(negedge clk); gpo31 = 0; repeat (n) @(negedge clk); gpo31 = 1; repeat (6) @(negedge clk); end endtask
// The end of a host's enable: the next program's ID view.
task host_end; identify(4); endtask

// Drive at negedge, sample at negedge: cycles from VALID asserted to BVALID/RVALID seen.
task axi_write(input [20:0] a, input [31:0] d, input [3:0] s);
	integer n;
	reg aw_hs, w_hs;
	begin
		bus_get;
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

localparam [20:0] PADB = 21'h80000;              // the PAD block: STAT, then ID
function [20:0] pad_addr(input [7:0] op, input [7:0] idx);
	pad_addr = 21'h40000 + 21'h40000 + {op, 10'd0} + {idx, 2'b00};
endfunction

reg [31:0] d, dl;
integer beats, i;

task rd(input string what, input [20:0] a, input [31:0] exp);
	begin axi_read(a, 0, d, dl, beats); check(what, d, exp); end
endtask
// the bits of mask only
task rdm(input string what, input [20:0] a, input [31:0] mask, input [31:0] exp);
	begin axi_read(a, 0, d, dl, beats); check(what, d & mask, exp); end
endtask

task settle; repeat (slow(100)) @(negedge clk); endtask

`ifdef MISTER_LW_PAD
// T-PAD1: hps_io's spinner registers have no initial value (the FPGA's is 0), so a toggle would stay x; spinner_2's
// toggles are counted.
integer n_spn2 = 0;
initial begin
	hps_io.spinner_0 = 0; hps_io.spinner_1 = 0; hps_io.spinner_2 = 0;
	hps_io.spinner_3 = 0; hps_io.spinner_4 = 0; hps_io.spinner_5 = 0;
end
always @(spn[26]) if ($realtime > 0) n_spn2 = n_spn2 + 1;

// A level store's word, for group g (0 digital, 1 left stick, 2 right stick, 3 paddle), player k, round r: every
// register's last word differs from its previous one and from every other register's in the bits it takes.
function [31:0] pv(input [1:0] g, input [2:0] k, input integer r);
	reg [15:0] lo;
	begin lo = r + 41 * k + 67 * g; pv = {4'hA, 2'd0, g, 5'd0, k, lo}; end
endfunction

function [7:0] dop(input [2:0] k);
	dop = (k < 2) ? 8'h02 + k : 8'h0E + k;               // 0x02, 0x03, 0x10-0x13
endfunction

// The value of each level register: g, k as above.
function [31:0] pad_reg(input [1:0] g, input [2:0] k);
	case (g)
		2'd0: pad_reg = (k == 0) ? joy0 : (k == 1) ? joy1 : (k == 2) ? joy2 : (k == 3) ? joy3 : (k == 4) ? joy4 : joy5;
		2'd1: pad_reg = {16'd0, lstk[16 * k +: 16]};
		2'd2: pad_reg = {16'd0, rstk[16 * k +: 16]};
		default: pad_reg = {24'd0, pdl[8 * k +: 8]};
	endcase
endfunction

function [31:0] pad_exp(input [1:0] g, input [31:0] v);
	pad_exp = (g == 0) ? v : (g == 3) ? {24'd0, v[7:0]} : {16'd0, v[15:0]};
endfunction

// T-PAD1 (SPEC 9): with clk_sys stopped, 1,000 stores to each of the 24 level slots (the six pads, both sticks
// and the paddle of each player) and 20 to one spinner. After the restart every register holds its last store,
// each slot crosses once (one more word: the first store, launched before the stop), and the spinner takes its
// 16 queued events while the 4 past the FIFO are dropped and counted in STAT, in every build.
task pad_hold_cases;
	reg  [31:0] s0, s1, v, d0, dl0;
	reg [767:0] r0;
	integer r, g, k, n, t0, t, nb, bad;
	begin
		$display("-- T-PAD1: 1,000 stores to every level and 20 to a spinner with clk_sys stopped");
		axi_read(PADB, 0, s0, dl0, nb);
		sys_run = 0; repeat (20) @(negedge clk);
		r0 = {joy0, joy1, joy2, joy3, joy4, joy5, lstk, rstk, pdl, spn};
		n = n_padwe; t0 = n_spn2;
		for (r = 0; r < 1000; r = r + 1)
			for (k = 0; k < 6; k = k + 1) begin
				axi_write(pad_addr(dop(k), 8'd0), pv(0, k, r), 4'hF);
				axi_write(pad_addr(8'h1A, k), pv(1, k, r), 4'hF);
				axi_write(pad_addr(8'h3D, k), pv(2, k, r), 4'hF);
				axi_write(pad_addr(8'h1A, {1'b0, k[2:0], 4'hF}), pv(3, k, r), 4'hF);
			end
		for (r = 0; r < 20; r = r + 1) axi_write(pad_addr(8'h1A, 8'hAF), 32'h00000040 + r, 4'hF);   // spinner_2
		axi_read(PADB, 0, s1, dl0, nb);
		check("T-PAD1: STAT counts the 4 spinner stores past the FIFO", s1, s0 + 32'd4);
		check("T-PAD1: no register moves while clk_sys is stopped",
		      {joy0, joy1, joy2, joy3, joy4, joy5, lstk, rstk, pdl, spn} === r0, 1);
		sys_run = 1;
		t = 0;
		while (n_padwe - n < 41 && t < slow(20000)) begin @(negedge clk); t = t + 1; end
		settle;
		check("T-PAD1: words crossed: one in flight, 24 slots once each, 16 spinner events", n_padwe - n, 41);
		bad = 0;
		for (g = 0; g < 4; g = g + 1)
			for (k = 0; k < 6; k = k + 1)
				if (pad_reg(g, k) !== pad_exp(g, pv(g, k, 999))) begin
					if (bad < 4) $display("     group %0d player %0d: %08h, expected %08h", g, k, pad_reg(g, k),
					                      pad_exp(g, pv(g, k, 999)));
					bad = bad + 1;
				end
		check("T-PAD1: every pad, stick and paddle holds its last store", bad, 0);
		check("T-PAD1: spinner_2 holds the 16th store", {24'd0, spn[25:18]}, 32'h0000004F);
		check("T-PAD1: spinner_2 toggled once per queued store", n_spn2 - t0, 16);
		check("T-PAD1: the other spinners unchanged", {spn[53:27], spn[17:0]} === {r0[53:27], r0[17:0]}, 1);
		axi_read(PADB, 0, v, dl0, nb);
		check("T-PAD1: STAT holds its count after the restart", v, s0 + 32'd4);
	end
endtask
`endif

// The host's first two drain steps on a CAUSE word: read it, store the value back (write-one-to-clear).
task cause_drain(input [20:0] a);
	reg [31:0] c, cl;
	integer nb;
	begin axi_read(a, 0, c, cl, nb); axi_write(a, c, 4'hF); end
endtask
// The same on SYS's CAUSE word, where the block is listed.
task sys_drain; if (TB_EVT) cause_drain(SYSB); endtask

// The entries with an enable link (SECTOR, OSDBUF, LANE): a store to one needs its OPEN bit.
localparam TB_E_SEC = TB_PAD, TB_E_OSD = TB_E_SEC + TB_SEC, TB_E_SYS = TB_E_OSD + TB_OSD, TB_E_LANE = TB_E_SYS + TB_EVT;
localparam [31:0] TB_LINKED = (TB_SEC ? 32'd1 << TB_E_SEC : 32'd0) | (TB_OSD ? 32'd1 << TB_E_OSD : 32'd0) |
                              (TB_LANE ? 32'd1 << TB_E_LANE : 32'd0);
// The host's step before an ENABLE: ENABLE read until quiet, for a bound (a stopped clk_sys never answers).
task wait_quiet;
	reg [31:0] v;
	integer n;
	begin
		n = 0;
		axi_read(ENA, 0, v, dl, beats);
		while (!v[30] && n < slow(4000)) begin
			repeat (6) @(negedge clk); n = n + 10;
			axi_read(ENA, 0, v, dl, beats);
		end
	end
endtask
// An ENABLE stored once the enable links let it through.
task ena_w; begin wait_quiet; axi_write(ENA, ENA_W, 4'hF); end endtask
// The host's step after an ENABLE: OPEN read until the linked entries' bits are set, for a bound.
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
// The bench's host enables (ending any enable first with an ID view, then waiting for quiet) and waits for its
// linked entries to open, with no line.
task ena_take;
	reg [31:0] v;
	begin
		axi_read(ENA, 0, v, dl, beats);
		if (v[31]) host_end;
		ena_w;
		axi_read(ENA, 0, v, dl, beats);
		check("the bench's host holds the enable", v[31], 1);
		wait_open;
	end
endtask
// The same, then every line enabled.
task ena_short; begin ena_take; axi_write(LINEN, 32'hFFFFFFFF, 4'hF); end endtask
// The same, then the status read once (the ENABLE raised STATUS).
task ena_up; begin ena_short; sys_drain; end endtask
// The bench's host's enable ended by the next program's ID view.
task ena_off;
	reg [31:0] v;
	begin
		host_end;
		axi_read(ENA, 0, v, dl, beats);
		check("the ID view ended the bench's enable", v[31], 0);
	end
endtask
task ena_rd(output [31:0] v); axi_read(ENA, 0, v, dl, beats); endtask

// ---- The enable links: the checker on each, the bench's own rest predicate, ENABLEs against it ----
`ifdef MISTER_LW_SECTOR_SLOT
fence_chk fc_sec(.clk(clk), .off(fc_off), .st(win.sec.sector.fence.st), .lvl(win.sec.sector.fence.hen_lwx),
                 .ack_s(win.sec.sector.fence.ack_s), .stb(win.sec.sector.wsel & win.sec.sector.write));
`endif
`ifdef MISTER_LW_OSD
fence_chk fc_osd(.clk(clk), .off(fc_off), .st(win.osd.osd.fence.st), .lvl(win.osd.osd.fence.hen_lwx),
                 .ack_s(win.osd.osd.fence.ack_s), .stb(win.osd.osd.wsel & win.osd.osd.write));
`endif
`ifdef MISTER_LW_LANE
fence_chk fc_lane(.clk(clk), .off(fc_off), .st(win.lane.lane.fence.st), .lvl(win.lane.lane.fence.hen_lwx),
                  .ack_s(win.lane.lane.fence.ack_s), .stb(win.lane.lane.wsel & win.lane.lane.write));
`endif
function [10:0] tb_g2b(input [10:0] g);
	integer n;
	begin tb_g2b[10] = g[10]; for (n = 9; n >= 0; n = n - 1) tb_g2b[n] = tb_g2b[n + 1] ^ g[n]; end
endfunction
// Each linked half at rest, as the bench reads its registers: the enable level low, the machine idle with idle
// outputs, the queue empty against the synchronised write pointer, the Gray sources current, every EN and mux
// (SECTOR) on SPI, no OSD word part written. rh_*: the predicate at the last four clk_sys edges.
wire tr_sec, tr_osd, tr_lane;
`ifdef MISTER_LW_SECTOR_SLOT
assign tr_sec = !win.sec.sector.hen_s && win.sec.sector.cs1 == 0 && !win.sec.sector.en_sys &&
                win.sec.sector.x_en_sys == 0 && win.sec.sector.fidle && win.sec.sector.offd == 0 &&
                win.sec.sector.dirty == 0 && !win.sec.sector.rc_busy &&
                !win.sec.sector.qv && !win.sec.sector.em && win.sec.sector.fack == 0 && !win.sec.sector.o_wr &&
                {1'b0, win.sec.sector.rptr} == tb_g2b({1'b0, win.sec.sector.wg_s}) &&
                win.sec.sector.sector_r_lwgray == (win.sec.sector.rptr ^ (win.sec.sector.rptr >> 1));
`else
assign tr_sec = 1'b1;
`endif
`ifdef MISTER_LW_OSD
assign tr_osd = !win.osd.osd.hen_s && !win.osd.osd.hd &&
                win.osd.osd.rptr == tb_g2b({2'b00, win.osd.osd.wg_s}) &&
                win.osd.osd.osd_r_lwgray == (win.osd.osd.rptr ^ (win.osd.osd.rptr >> 1));
`else
assign tr_osd = 1'b1;
`endif
`ifdef MISTER_LW_LANE
assign tr_lane = !win.lane.lane.hen_s && win.lane.lane.ps == 0 && !win.lane.lane.own && !win.lane.lane.hold &&
                 !win.lane.lane.strobe && win.lane.lane.sel == 0 && win.lane.lane.rptr == tb_g2b(win.lane.lane.wg_s) &&
                 win.lane.lane.lane_r_lwgray == (win.lane.lane.rptr ^ (win.lane.lane.rptr >> 1)) &&
                 win.lane.lane.lane_rw_lwgray == (win.lane.lane.rwp ^ (win.lane.lane.rwp >> 1));
`else
assign tr_lane = 1'b1;
`endif
reg [3:0] rh_sec = 0, rh_osd = 0, rh_lane = 0;
always @(posedge clk_sys) begin
	rh_sec <= {rh_sec[2:0], tr_sec}; rh_osd <= {rh_osd[2:0], tr_osd}; rh_lane <= {rh_lane[2:0], tr_lane};
end
integer n_claims = 0, n_offrest = 0;
always @(posedge clk) if (win.e_take) begin
	n_claims = n_claims + 1;
	if (!(tr_sec && tr_osd && tr_lane && &rh_sec && &rh_osd && &rh_lane)) begin
		n_offrest = n_offrest + 1;
		if (n_offrest <= 4) $display("   an ENABLE taken at %0t with a half off rest: SECTOR %b %b, OSD %b %b, LANE %b %b",
		                             $time, tr_sec, rh_sec, tr_osd, rh_osd, tr_lane, rh_lane);
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
	if (lw_waddr == SEC) t_wsec = $realtime;
	if (lw_waddr == 21'h43000) t_wsys = $realtime;
end

// ENABLE moves with the bench's own stores (`burst`, `stale`): word comparisons leave it out.
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
reg [31:0] rings0, d0;
realtime   t0_en;
integer n0, s0, overlap = 0;

// a save-slot strobe while the LW feeder holds the CD ack: the overlap the mux exists for
always @(posedge clk_sys) if (sd_buff_wr && sd_ack[TB_S1H] && blk_dn[17] && !blk_dn[16]) overlap = 1;

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
task sec_data(input integer i, input [31:0] v); axi_write(SEC + S_DATA + 4*i, v, 4'hF); endtask

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
`ifdef MISTER_LW_EVT
always @(posedge clk) if (win.evt.evt.s_v) t_cap = $realtime;
`endif

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

// One SPI command word and the reply word hps_io gives for it (0x36 info, 0x3C upload, 0x0N17 a slot's read).
task spi_get(input [15:0] cmd, output [15:0] r);
	begin
		@(negedge clk_sys); io_enable = 1;
		io_din = cmd; io_strobe = 1; @(negedge clk_sys); io_strobe = 0; @(negedge clk_sys); r = hps_io.io_dout;
		io_enable = 0; @(negedge clk_sys);
	end
endtask

`ifdef MISTER_LW_EVT
// T-EVT1's checker: each pulse hps_io sends on HPS_BUS[239:237] ({upload, info, blk}) is owed a raise of its
// CAUSE bit after it; a raise is the event cell's valid pulse with the bit set. ev_nl and ev_nb count pulses met
// in a launch cycle of the event handshake and while it was busy.
realtime ev_tp [0:2], ev_tr [0:2];
integer  ev_np [0:2], ev_nr [0:2], ev_nl = 0, ev_nb = 0;
initial for (i = 0; i < 3; i = i + 1) begin ev_tp[i] = 0; ev_tr[i] = 0; ev_np[i] = 0; ev_nr[i] = 0; end
always @(posedge clk_sys) begin : ev_mon
	integer b;
	for (b = 0; b < 3; b = b + 1) if (hps_bus[237 + b] === 1'b1) begin ev_tp[b] = $realtime; ev_np[b] = ev_np[b] + 1; end
	if (|hps_bus[239:237] === 1'b1) begin
		if (win.evt.evt.e_go && !win.evt.evt.e_busy) ev_nl = ev_nl + 1;
		if (win.evt.evt.e_busy) ev_nb = ev_nb + 1;
	end
end
always @(posedge clk) begin : ev_rmon
	integer b;
	if (win.evt.evt.e_v)
		for (b = 0; b < 3; b = b + 1) if (win.evt.evt.e_q[b]) begin ev_tr[b] = $realtime; ev_nr[b] = ev_nr[b] + 1; end
end
// bits whose last pulse has no raise after it
function integer ev_owed;
	integer b;
	begin ev_owed = 0; for (b = 0; b < 3; b = b + 1) if (ev_tp[b] > ev_tr[b]) ev_owed = ev_owed + 1; end
endfunction

localparam [20:0] EVPEND = SYSB + 21'h18;

// T-EVT1: BLK, INFO and UPLOAD, one pulse each, then pulses at every spacing to sixteen clk_sys cycles.
task evt1_cases;
	reg [15:0] r;
	integer k, n0, n1, nr0, lost, j;
	begin
		$display("-- EVT: BLK, INFO, UPLOAD: one pulse raises its cause once; PEND follows the levels; serving raises nothing");
		sys_drain;
		rd("PEND: nothing stands", EVPEND, 32'd0);
		nr0 = ev_nr[1];
		info_v = 8'h21; @(negedge clk_sys) info_rq = 1; settle;
		rd("an info_req rise: CAUSE INFO", SYSB, 32'h4);
		check("one raise of INFO", ev_nr[1] - nr0, 1);
		rd("PEND: info pending", EVPEND, 32'h4);
		axi_write(SYSB, 32'h4, 4'hF); settle;
		rd("info_req held: no second INFO", SYSB, 32'd0);
		spi_get(16'h0036, r);
		check("0x36 answers the info byte", r, 16'h0021);
		settle;
		rd("0x36 served it: PEND clear", EVPEND, 32'd0);
		rd("serving raised nothing", SYSB, 32'd0);
		@(negedge clk_sys) info_rq = 0; settle;
		rd("info_req falls: nothing raised", SYSB, 32'd0);

		nr0 = ev_nr[2];
		@(negedge clk_sys) up_rq = 1; settle;
		rd("an ioctl_upload_req rise: CAUSE UPLOAD", SYSB, 32'h8);
		check("one raise of UPLOAD", ev_nr[2] - nr0, 1);
		rd("PEND: upload pending", EVPEND, 32'h8);
		axi_write(SYSB, 32'h8, 4'hF);
		spi_get(16'h003C, r);
		check("0x3C answers {index, 1}", r, 16'h5C01);
		settle;
		rd("0x3C served it: PEND clear, nothing raised", EVPEND, 32'd0);
		rd("CAUSE clear", SYSB, 32'd0);
		@(negedge clk_sys) up_rq = 0;

		nr0 = ev_nr[0];
		@(negedge clk_sys) blk0_rd = 1; settle;
		rd("a slot's request rises: CAUSE BLK", SYSB, 32'h2);
		check("one raise of BLK", ev_nr[0] - nr0, 1);
		rd("PEND: a block request stands", EVPEND, 32'h2);
		axi_write(SYSB, 32'h2, 4'hF);
		spi_get(16'h0017, r);                            // slot 0's read: the ack rises, and falls at the command's end
		settle;
		rd("its SPI ack fell with the request still raised: BLK again", SYSB, 32'h2);
		check("a second raise of BLK", ev_nr[0] - nr0, 2);
		axi_write(SYSB, 32'h2, 4'hF);
		@(negedge clk_sys) blk0_rd = 0; settle;
		rd("the request falls: nothing raised", SYSB, 32'd0);
		rd("PEND clear", EVPEND, 32'd0);
		nr0 = ev_nr[0];
		cd_request(32'h00005A00); settle;
		if (!TB_SEC || !blk_dn[18]) rd("the CD's request on SPI: CAUSE BLK", SYSB, 32'h2);
		axi_write(SYSB, 32'h2, 4'hF);
		n1 = cd_frames; spi_frame(cd_lba); wait_frame(n1); settle;
		rd("served over SPI, the request dropped on the ack: no second BLK", SYSB, 32'd0);
		check("one raise of BLK for the CD's request", ev_nr[0] - nr0, 1);
		if (TB_SEC) begin
			$display("-- EVT: a slot the SECTOR mux has claimed raises no BLK; back on SPI its standing request does");
			sec_wr(S_CTRL, C_EN0); settle;
			nr0 = ev_nr[0];
			cd_request(32'h00005B00); settle;
			if (win.sec_lvl) cause_drain(SEC);
			check("SECTOR's EN: the CD's request raises no BLK", ev_nr[0] - nr0, 0);
			rd("PEND: no block request at hps_io", EVPEND, 32'd0);
			sec_wr(S_CTRL, 32'h0); settle;
			rd("EN cleared: the request reaches hps_io, BLK", SYSB, 32'h2);
			if (win.sec_lvl) cause_drain(SEC);
			sec_spi_clear; settle;
			sys_drain;
		end

		$display("-- EVT: pulses one cycle apart, in the launch cycle and during a handshake are not lost");
		lost = 0; n0 = ev_nl; n1 = ev_nb;
		for (k = 0; k < 17; k = k + 1) for (j = 0; j < 2; j = j + 1) begin
			fork
				begin @(negedge clk_sys); info_rq = 1; @(negedge clk_sys); info_rq = 0;
				      if (j) begin repeat (k) @(negedge clk_sys); info_rq = 1; @(negedge clk_sys); info_rq = 0; end end
				begin repeat (k + 1) @(negedge clk_sys); up_rq = 1; @(negedge clk_sys); up_rq = 0; end
				begin repeat (k + 2) @(negedge clk_sys); blk0_rd = 1; end
			join
			settle;
			lost = lost + ev_owed();
			if (TB_EVT) rd($sformatf("pulses %0d apart (%0d): all three causes", k, j), SYSB, 32'hE);
			@(negedge clk_sys) blk0_rd = 0;
			spi_get(16'h0036, r); spi_get(16'h003C, r); settle;
			sys_drain;
		end
		$display("     %0d pulses in a launch cycle, %0d during a handshake", ev_nl - n0, ev_nb - n1);
		check("no pulse lost", lost, 0);
		check("pulses met a launch cycle and a busy handshake", {ev_nl > n0, ev_nb > n1}, 2'b11);
		rd("PEND clear after the sweep", EVPEND, 32'd0);
		rd("CAUSE clear after the sweep", SYSB, 32'd0);
	end
endtask

// T-EVT2: status_set rising two clk_sys cycles apart, read with brackets of four shapes at 64 phase offsets.
// st_fl[x] is the flag hps_io gave st_pat(3000 + x).
reg [3:0] st_fl [0:1023];
// a status word's value index (the inverse of st_pat for word j), or -1
function integer st_idx(input integer j, input [31:0] w);
	integer x;
	reg [127:0] v;
	begin
		case (j)
			0: x = w[31:8];
			1: x = w[15:0];
			2: x = w - 32'h5A000000;
			default: x = w ^ 32'hA5000000;
		endcase
		v = st_pat(x);
		st_idx = (x >= 3000 && x < 3000 + 1024 && v[32 * j +: 32] == w) ? x - 3000 : -1;
	end
endfunction
task evt2_cases;
	reg [31:0] f0, f1, f, vl;
	reg [31:0] w [0:3];
	reg [3:0] got;
	integer k, sh, j, m, x, x0, ok, acc, rej, bad, nb, last, done;
	begin
		$display("-- EVT: status_set two cycles apart at 64 phase offsets; every accepted bracket is one value, in full");
		acc = 0; rej = 0; bad = 0;
		st_req(st_pat(3000)); st_fl[0] = `TB_STFLG; last = 0; settle;
		for (k = 0; k < 64; k = k + 1) begin
			sh = k % 4; done = 0;
			fork
				begin
					for (m = 1; m <= 12; m = m + 1) begin
						st_req(st_pat(3000 + 12 * k + m)); st_fl[12 * k + m] = `TB_STFLG;
					end
					last = 12 * k + 12;
					repeat (8) @(negedge clk_sys); done = 1;
				end
				begin
					#(k * 1.25);
					while (!done) begin
						got = 0;
						if (sh == 3) begin
							axi_read(SYSB + 21'h04, 4'd4, f, vl, nb);
							f0 = rb_d[0]; for (j = 0; j < 4; j = j + 1) w[j] = rb_d[1 + j]; got = 4'hF;
						end
						else begin
							axi_read(SYSB + 21'h04, 0, f0, vl, nb);
							if (sh == 2) begin
								axi_read(SYSB + 21'h08, 4'd3, f, vl, nb);
								for (j = 0; j < 4; j = j + 1) w[j] = rb_d[j]; got = 4'hF;
							end
							else if (sh == 1) begin
								j = (k / 4) % 4; axi_read(SYSB + 21'h08 + 4 * j, 0, w[j], vl, nb); got = 4'd1 << j;
							end
							else begin
								for (j = 0; j < 4; j = j + 1) axi_read(SYSB + 21'h08 + 4 * j, 0, w[j], vl, nb);
								got = 4'hF;
							end
						end
						axi_read(SYSB + 21'h04, 0, f1, vl, nb);
						if (f0 != f1) rej = rej + 1;
						else begin
							acc = acc + 1; x0 = -2; ok = 1;
							for (j = 0; j < 4; j = j + 1) if (got[j]) begin
								x = st_idx(j, w[j]);
								if (x < 0 || (x0 != -2 && x != x0)) ok = 0;
								x0 = x;
							end
							if (ok && st_fl[x0] !== f0[3:0]) ok = 0;
							if (!ok) begin
								bad = bad + 1;
								if (bad <= 4) $display("     shape %0d offset %0d: STFLG %08h, words %08h %08h %08h %08h (read %b)", sh, k, f0,
								                       w[0], w[1], w[2], w[3], got);
							end
						end
					end
				end
			join
			settle;
			st_read(f0, f1, stw);
			if (f0 !== f1 || stw !== st_pat(3000 + last) || f0[3:0] !== st_fl[last]) begin
				bad = bad + 1; $display("     offset %0d: the last value did not arrive", k);
			end
		end
		$display("     %0d brackets accepted, %0d rejected", acc, rej);
		check("every accepted bracket equals one value status_in had, with its flag", bad, 0);
		check("brackets accepted and brackets rejected", {acc > 0, rej > 0}, 2'b11);
		sys_drain;
	end
endtask
`endif

localparam [20:0] OSD   = 21'h44000;
localparam [20:0] OSDST = OSD + 21'h1800;      // STATUS {level, drops}
// The image both osd_buffers must hold, whichever path wrote it (X where nothing has been written).
reg  [7:0] ob [0:5119];
integer    ob_i;
initial for (ob_i = 0; ob_i < 5120; ob_i = ob_i + 1) ob[ob_i] = 8'bx;
localparam OB_ROWS = TB_MENU ? 20 : 16;

function [7:0] lwp(input integer i, input integer k);
	lwp = (i * 37) ^ (i >> 4) ^ (k * 8'h5B) ^ 8'hA5;
endfunction
function [7:0] spp(input integer i, input integer k);
	spp = (i * 113) ^ (i >> 6) ^ (k * 8'hC7) ^ 8'h3C;
endfunction
// pattern f (0: lwp, 1: spp) of image word w, generation k
function [31:0] pw(input integer w, input integer k, input integer f);
	integer b;
	for (b = 0; b < 4; b = b + 1) pw[8*b +: 8] = f ? spp(4*w + b, k) : lwp(4*w + b, k);
endfunction
// image word w as each osd_buffer holds it
function [31:0] bw_h(input integer w);
	bw_h = {hdmi_osd.osd_buffer[4*w+3], hdmi_osd.osd_buffer[4*w+2], hdmi_osd.osd_buffer[4*w+1], hdmi_osd.osd_buffer[4*w]};
endfunction
function [31:0] bw_v(input integer w);
	bw_v = {vga_osd.osd_buffer[4*w+3], vga_osd.osd_buffer[4*w+2], vga_osd.osd_buffer[4*w+1], vga_osd.osd_buffer[4*w]};
endfunction

// An image store that, taken, lands: the model follows it.
task osd_word(input integer w, input [31:0] v, input [3:0] be);
	integer b;
	begin
		axi_write(OSD + 4*w, v, be);
		for (b = 0; b < 4; b = b + 1) if (be[b]) ob[4*w + b] = v[8*b +: 8];
	end
endtask
// The model follows a whole-word store the case has seen land.
task ob_word(input integer w, input [31:0] v);
	integer b;
	for (b = 0; b < 4; b = b + 1) ob[4*w + b] = v[8*b +: 8];
endtask
task osd_st(output [31:0] v); axi_read(OSDST, 0, v, dl, beats); endtask
// The host's pacing: STATUS read until the FIFO has room for n more entries, for a bound.
task osd_room(input integer n);
	reg [31:0] v;
	integer m;
	begin
		m = 0;
		osd_st(v);
		while (v[24:16] > 256 - n && m < slow(20000)) begin repeat (10) @(negedge clk); m = m + 10; osd_st(v); end
	end
endtask
// STATUS read until level is 0: every byte stored is in osd_buffer.
task osd_drain; osd_room(256); endtask

// One row over LW, pattern f, generation k, after waiting for room for it.
task lw_row_f(input integer r, input integer k, input integer f);
	integer w;
	begin
		osd_room(64);
		for (w = r*64; w < r*64 + 64; w = w + 1) osd_word(w, pw(w, k, f), 4'hF);
	end
endtask
task lw_row(input integer r, input integer k); lw_row_f(r, k, 0); endtask

task spi_w(input [15:0] v);
	begin io_din = v; io_strobe = 1; @(negedge clk_sys); io_strobe = 0; @(negedge clk_sys); end
endtask
task spi_osd_begin(input [7:0] cmd);
	begin
		@(negedge clk_sys); io_osd = 1;
		spi_w({8'h00, cmd}); @(negedge clk_sys);
	end
endtask
task spi_osd_end; begin io_osd = 0; repeat (2) @(negedge clk_sys); end endtask
task spi_osd_cmd(input [7:0] cmd); begin spi_osd_begin(cmd); spi_osd_end; end endtask
// OSDCMDENABLE with its five words: the info box (inf) at (x, y), w x h cells of 8 pixels, rotation ro
task spi_osd_en(input inf, input [11:0] x, input [11:0] y, input [5:0] w, input [5:0] h, input [1:0] ro);
	begin
		spi_osd_begin(inf ? 8'h45 : 8'h41);
		spi_w({4'd0, x}); spi_w({4'd0, y}); spi_w({10'd0, w}); spi_w({10'd0, h}); spi_w({14'd0, ro});
		spi_osd_end;
	end
endtask

// A known OSD state: disabled (highres clear), then enabled with no info box and no rotation: 64 lines.
task osd_lo; begin spi_osd_cmd(8'h40); spi_osd_en(0, 12'd0, 12'd0, 6'd0, 6'd0, 2'd0); end endtask
// The model takes both osd_buffers as they are (cases the model does not follow, such as the link's and the lane's
// OSD frames, write them too), once they agree.
task ob_sync(input string what);
	integer i, n;
	begin
		n = 0;
		for (i = 0; i < 5120; i = i + 1) begin
			if (hdmi_osd.osd_buffer[i] !== vga_osd.osd_buffer[i]) n = n + 1;
			ob[i] = hdmi_osd.osd_buffer[i];
		end
		check({what, ": both osd_buffers alike"}, n, 0);
	end
endtask

// OSDCMDWRITE row r: 0x20|r, so bit 3 (rows 8-15) re-arms the SPI highres (osd.v:80)
task spi_osd_row(input integer r, input integer k);
	integer m;
	begin
		spi_osd_begin(8'h20 | r[4:0]);
		for (m = 0; m < 256; m = m + 1) begin
			ob[r*256 + m] = spp(r*256 + m, k);
			spi_w({8'h00, ob[r*256 + m]});
		end
		spi_osd_end;
	end
endtask

// Both osd_buffers against the model, rows r0 to r1 - 1: the bytes that differ.
task ob_cmp(input string what, input integer r0, input integer r1);
	integer i, n;
	begin
		n = 0;
		for (i = r0 * 256; i < r1 * 256; i = i + 1)
			if (hdmi_osd.osd_buffer[i] !== ob[i] || vga_osd.osd_buffer[i] !== ob[i]) n = n + 1;
		check({what, ": bytes of both osd_buffers that differ from the image"}, n, 0);
	end
endtask

// After `nf` frames, both outputs drew an OSD box `exp_lines` high and 256 wide, bit (y, x) from the image.
task osd_check(input string what, input integer exp_lines, input integer nf);
	integer f, y, x, bh, bv;
	reg [7:0] b;
	begin
		f = cap_h.frames;
		wait (cap_h.frames >= f + nf);
		@(negedge clk_vid);
		bh = 0; bv = 0;
		for (y = 0; y < exp_lines && y < 128; y = y + 1)
			for (x = 0; x < 256; x = x + 1) begin
				b = ob[(y >> 3)*256 + x];
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

// The pixel path's output, whatever it draws: both captures and their box heights after `nf` frames, kept
// (cap_snap) or compared with the kept one (cap_diff: the pixels that differ).
reg     cs_h [0:32767];
reg     cs_v [0:32767];
integer cs_lh, cs_lv;
task cap_wait(input integer nf);
	integer f;
	begin f = cap_h.frames; wait (cap_h.frames >= f + nf); @(negedge clk_vid); end
endtask
task cap_snap(input integer nf);
	integer i;
	begin
		cap_wait(nf);
		for (i = 0; i < 32768; i = i + 1) begin cs_h[i] = cap_h.cap[i]; cs_v[i] = cap_v.cap[i]; end
		cs_lh = cap_h.lines; cs_lv = cap_v.lines;
	end
endtask
task cap_diff(input integer nf, output integer n);
	integer i;
	begin
		cap_wait(nf);
		n = (cap_h.lines != cs_lh) + (cap_v.lines != cs_lv);
		for (i = 0; i < 32768; i = i + 1) n = n + (cap_h.cap[i] !== cs_h[i]) + (cap_v.cap[i] !== cs_v[i]);
	end
endtask
// The same buffer drawn the same, LW or SPI: an image over SPI (generation k, rows 0 to nr - 1), its pixels kept;
// another image over LW must draw otherwise; the SPI image's bytes over LW must then draw exactly as over SPI.
task osd_same(input string what, input integer nr, input integer k);
	integer r, n;
	begin
		for (r = 0; r < nr; r = r + 1) spi_osd_row(r, k);
		cap_snap(4);
		check({what, ": the SPI image is drawn"}, cs_lh > 0 && cs_lv > 0, 1);
		for (r = 0; r < nr; r = r + 1) lw_row(r, k + 1);
		osd_drain;
		cap_diff(3, n);
		check({what, ": another image over LW draws otherwise"}, n > 0, 1);
		for (r = 0; r < nr; r = r + 1) lw_row_f(r, k, 1);
		osd_drain;
		ob_cmp({what, ": the SPI image's bytes over LW"}, 0, nr);
		cap_diff(3, n);
		check({what, ": its pixels, as over SPI"}, n, 0);
	end
endtask

integer r, bad;
string  caps;

// The OSD writer's byte comes only while both osd selects are low and the I/O view selected, in its cycle and
// the two before it (sampled on clk_sys, as the writer is); osd_held counts the cycles it had a word and waited.
integer osd_clash = 0, osd_held = 0;
reg [1:0] osd_qh = 0;
always @(posedge clk_sys) begin
	if (osd_we && !(m_sel[3:2] == 2'b00 && gpo31_s && osd_qh == 2'b11)) osd_clash = osd_clash + 1;
`ifdef MISTER_LW_OSD
	if (win.osd.osd.hen_s && win.osd.osd.avail && m_sel[3:2] != 2'b00) osd_held = osd_held + 1;
`endif
	osd_qh <= {osd_qh[0], m_sel[3:2] == 2'b00 && gpo31_s};
end

`ifdef MISTER_LW_LANE
// LANE: frames stored over LW against the same frames sent as link words (the HPS's handshake on io_clk/io_ack).
localparam [20:0] LANE = 21'h46000;
localparam [20:0] LCAUSE = 21'h46024;   // CAUSE {DROP, REPLY, PLAYED}
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

// T-LN1, the lane's cause word and its line. The clk_100m edge of the lane's last RPOP taken, of its last CAUSE
// store taken and of the last advance of the synchronised reply write pointer (the edge REPLY is raised at).
realtime t_lpop = 0, t_lclr = 0, t_lrw = 0;
integer  n_poprw = 0;                 // RPOPs taken at the edge a reply word landed
always @(posedge clk) begin
	if (win.lane.lane.w_pop) t_lpop = $realtime;
	if (win.lane.lane.w_cause) t_lclr = $realtime;
	if (win.lane.lane.rwg_s != win.lane.lane.rwg_d) t_lrw = $realtime;
	if (win.lane.lane.w_pop && win.lane.lane.rwg_s != win.lane.lane.rwg_d) n_poprw = n_poprw + 1;
end
// The host model's sleep: a reply word waiting, with the line low, for more than the two edges the cause and the
// line take after the word lands, is a word stranded.
`ifndef LW_TB_LN1_N
`define LW_TB_LN1_N 10000
`endif
reg     ln_mon = 0, ln_asleep = 0;
integer ln_run = 0, ln_strand = 0;
always @(posedge clk) if (ln_mon) begin
	if (ln_asleep && win.lane.lane.rcount != 0 && !lines[2]) begin
		ln_run = ln_run + 1;
		if (ln_run == 3) ln_strand = ln_strand + 1;
	end
	else ln_run = 0;
end
// One word waits and the cause is clear. A one-strobe capture frame (one reply) is stored and, d clk_100m cycles
// after its END's response, an RPOP (mode 0) or a store-back of CAUSE (mode 1). same: the store took effect at the
// edge the reply landed; lat: cycles from the END's response to that edge, eff: from the store's issue to its edge.
task ln_trial(input integer mode, input integer d, output integer same, output integer lat, output integer eff);
	reg  [31:0] c, rs, p, dr;
	realtime    t0, ti;
	begin
		cause_drain(LCAUSE);
		lane_count(p, dr);
		f_set(0, 16'h0029, 0);
		lane_send_c(1, 1'b1);
		t0 = $realtime;
		repeat (d) @(negedge clk);
		ti = $realtime;
		if (mode == 0) axi_write(LANE + 21'h0C, 32'd0, 4'hF);
		else axi_write(LCAUSE, 32'h7, 4'hF);
		lane_wait(p + 1);
		lat  = $rtoi((t_lrw - t0) / 10.0 + 0.5);
		eff  = $rtoi(((mode == 0 ? t_lpop : t_lclr) - ti) / 10.0 + 0.5);
		same = (mode == 0 ? t_lpop : t_lclr) == t_lrw;
		axi_read(LCAUSE, 0, c, dl, beats);
		lane_rstat(rs);
		if (mode == 0) begin
			if (!c[1]) $display("     RPOP %0d cycles after END: REPLY not raised (pop at %0t, reply at %0t)", d, t_lpop, t_lrw);
			check($sformatf("REPLY with an RPOP %0d cycles after END: raised", d), c[1], 1);
			check($sformatf("REPLY with an RPOP %0d cycles after END: one word waits", d), rs[15:0], 1);
		end
		else begin
			check($sformatf("REPLY with a store-back %0d cycles after END: set exactly when it landed at or after the clear", d),
			      c[1], t_lrw >= t_lclr);
			check($sformatf("REPLY with a store-back %0d cycles after END: two words wait", d), rs[15:0], 2);
			axi_write(LANE + 21'h0C, 32'd0, 4'hF);
		end
	end
endtask
task lane_cause_cases;
	reg  [31:0] p, dr, c, rs, v;
	reg  [15:0] w;
	reg  [15:0] r0;
	integer     k, m, hits, lat, eff, ctr, same, seed, prod, popped, wakes, len, idle, nr0, nf;
	begin
		$display("-- LANE 17: CAUSE: PLAYED per frame; the line follows; W1C clears only the bits written; reads never clear");
		axi_write(LINEN, 32'hFFFFFFFF, 4'hF);
		cause_drain(LCAUSE); settle;
		rd("lane 17: CAUSE clear", LCAUSE, 32'd0);
		check("lane 17: line 3 low", lines[2], 0);
		lane_count(p, dr);
		f_set(0, 16'h001E, 2); fw[0] = 16'h1701; fw[1] = 16'h1702;
		lane_send(1); lane_wait(p + 1);
		rd("lane 17: a frame played: PLAYED", LCAUSE, 32'h1);
		rd("lane 17: read again: unchanged", LCAUSE, 32'h1);
		rdm("lane 17: PENDING has the lane's bit", DIRB + 21'h20, 32'd1 << TB_E_LANE, 32'd1 << TB_E_LANE);
		check("lane 17: line 3 high", lines[2], 1);
		axi_write(LCAUSE, 32'h1, 4'h1);
		rd("lane 17: a partial store clears nothing", LCAUSE, 32'h1);
		axi_write(LCAUSE, 32'h6, 4'hF);
		rd("lane 17: a store of the other bits clears nothing", LCAUSE, 32'h1);
		axi_write(LINEN, 32'hFFFFFFFF & ~(32'd1 << TB_E_LANE), 4'hF); repeat (2) @(negedge clk);
		check("lane 17: its LINE_EN bit clear: line 3 low, the cause standing", {lines[2], win.lane_lvl}, 2'b01);
		axi_write(LINEN, 32'hFFFFFFFF, 4'hF); repeat (2) @(negedge clk);
		check("lane 17: its LINE_EN bit set again: line 3 high", lines[2], 1);
		axi_write(LCAUSE, 32'h1, 4'hF); repeat (2) @(negedge clk);
		check("lane 17: the store-back: line 3 low within two cycles", lines[2], 0);
		rd("lane 17: CAUSE clear", LCAUSE, 32'd0);

		$display("-- LANE 18: CAUSE: REPLY for a captured frame beside PLAYED; each bit cleared alone");
		lane_count(p, dr);
		f_set(0, 16'h0029, 0);
		lane_send_c(1, 1'b1); lane_wait(p + 1);
		rd("lane 18: a capture frame: PLAYED and REPLY", LCAUSE, 32'h3);
		axi_write(LCAUSE, 32'h1, 4'hF);
		rd("lane 18: PLAYED cleared, REPLY kept", LCAUSE, 32'h2);
		check("lane 18: line 3 still high", lines[2], 1);
		axi_write(LCAUSE, 32'h2, 4'hF);
		rd("lane 18: REPLY cleared", LCAUSE, 32'h0);
		lane_rstat(rs);
		check("lane 18: the reply still waits (the level)", rs[15:0], 1);

		$display("-- LANE 19: CAUSE: a reply landing while one word waits raises REPLY");
		lane_send_c(1, 1'b1); lane_wait(p + 2);
		axi_read(LCAUSE, 0, c, dl, beats);
		check("lane 19: REPLY raised by a second word", c[1], 1);
		lane_rstat(rs);
		check("lane 19: two words wait", rs[15:0], 2);
		cause_drain(LCAUSE);
		lane_pop(w);
		rd("lane 19: an RPOP raises nothing", LCAUSE, 32'd0);

		$display("-- LANE 20: CAUSE: a reply landing at the edge the last waiting word is popped raises REPLY");
		ln_trial(0, 400, same, lat, eff);
		ctr = lat - eff;
		$display("     the reply lands %0d cycles after END's response, an RPOP takes effect %0d after its issue", lat, eff);
		hits = 0;
		for (k = 0; k < 4 && hits < 2; k = k + 1)
			for (m = -8; m <= 8; m = m + 1) begin
				ln_trial(0, (ctr + m > 0) ? ctr + m : 0, same, lat, eff);
				hits = hits + same;
			end
		$display("     %0d RPOPs taken at the edge the reply landed", hits);
		check("lane 20: an RPOP met a reply's landing at its edge", hits > 0, 1);

		$display("-- LANE 21: CAUSE: a reply raised at the edge of a store-back stays set");
		ln_trial(1, 400, same, lat, eff);
		ctr = lat - eff;
		hits = 0;
		for (k = 0; k < 4 && hits < 2; k = k + 1)
			for (m = -8; m <= 8; m = m + 1) begin
				ln_trial(1, (ctr + m > 0) ? ctr + m : 0, same, lat, eff);
				hits = hits + same;
			end
		$display("     %0d store-backs taken at the edge the reply landed", hits);
		check("lane 21: a store-back met a reply's landing at its edge", hits > 0, 1);
		lane_pop(w);
		cause_drain(LCAUSE);
		lane_rstat(rs);
		check("lane 21: drained", rs[15:0], 0);

		$display("-- LANE 22: a host that sleeps on line 3 and drains by the rule strands no reply (%0d replies)", `LW_TB_LN1_N);
		axi_write(LINEN, 32'd1 << TB_E_LANE, 4'hF);
		cause_drain(LCAUSE);
		lane_rstat(rs); r0 = rs[31:16];
		nr0 = n_poprw;
		seed = 29; prod = 0; popped = 0; wakes = 0; idle = 0; nf = 0;
		ln_strand = 0; ln_run = 0; ln_asleep = 1; ln_mon = 1;
		while ((prod < `LW_TB_LN1_N || popped < prod) && idle < slow(5000)) begin
			if (lines[2]) begin
				ln_asleep = 0; wakes = wakes + 1; idle = 0;
				repeat ($unsigned($random(seed)) % 8) @(negedge clk);
				axi_read(LCAUSE, 0, c, dl, beats);
				axi_write(LCAUSE, c, 4'hF);
				lane_rstat(rs);
				while (rs[15:0] != 0) begin
					axi_write(LANE + 21'h0C, 32'd0, 4'hF); popped = popped + 1;
					lane_rstat(rs);
				end
				ln_asleep = 1;
			end
			else if (prod < `LW_TB_LN1_N && prod - popped <= 200 && ($random(seed) & 3) != 0) begin
				len = 1 + $unsigned($random(seed)) % 4;
				if (prod + len > `LW_TB_LN1_N) len = `LW_TB_LN1_N - prod;
				f_set(0, 16'h0029, len - 1);
				for (m = 0; m < len - 1; m = m + 1) fw[m] = 16'h0000;
				lane_send_c(1, 1'b1);
				prod = prod + len; nf = nf + 1;
			end
			else begin
				repeat (1 + $unsigned($random(seed)) % 16) @(negedge clk);
				idle = idle + 1;
			end
		end
		ln_mon = 0; ln_asleep = 0;
		$display("     %0d replies in %0d frames, %0d popped, %0d wakes, %0d RPOPs taken at a landing's edge", prod,
		         nf, popped, wakes, n_poprw - nr0);
		check("lane 22: every reply popped", popped, prod);
		check("lane 22: no word waited with line 3 low while the host slept", ln_strand, 0);
		lane_rstat(rs);
		check("lane 22: none waiting, no reply dropped", rs, {r0, 16'd0});
		settle;
		cause_drain(LCAUSE);
		axi_write(LINEN, 32'hFFFFFFFF, 4'hF);
	end
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
// The store target: LINE_EN, which reads 0 with no enable and holds the listed entries' bits (FW_MASK: the bits
// of the two stored that it can hold); the directory magic the read target.
localparam [20:0] FW_CTRL  = LINEN;
localparam [31:0] FW_MASK  = ((32'd1 << TB_COUNT) - 32'd1) & 32'h3;
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

// One reset case. A transaction of `kind` (0 a read of the magic, 1 a store to LINE_EN whose data beat waits,
// 2 a store to LINE_EN whose response waits) is issued `off` cycles before hps_rst_n falls (after it when
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
		ena_take;                                    // the reset ends it
		axi_read(FW_CTRL, 0, oc, dl, beats);
		v  = {30'd0, ~oc[1:0]} & FW_MASK;
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
		// the reset ended the enable, so the store had no effect: LINE_EN reads 0 without one
		ena_rd(lv);
		check({w, ": the enable ended, by the HPS reset"}, lv[31], 0);
		axi_read(FW_CTRL, 0, rv, dl, beats);
		check({w, ": LINE_EN, with no enable"}, rv, 0);
		// the next accesses are answered in the fixed latency (axi_read and axi_write check it)
		rd({w, ": a read after"}, 21'h40000, FW_MAGIC);
		ena_take;
		axi_write(FW_CTRL, v, 4'hF);
		rd({w, ": a store after, under a new enable"}, FW_CTRL, v);
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
		ena_take;
		axi_read(FW_CTRL, 0, oc, dl, beats); v = {30'd0, ~oc[1:0]} & FW_MASK;
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
		rd("the store had no effect: the reset ended the enable", FW_CTRL, 32'd0);

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
		check("beats that differ from a plain 16-beat read (data but ENABLE, OWNED, OPEN, LINE_EN and RESETS, which the reset moves; ID, RLAST)", n, 0);
		check("nothing freed", n_fr - fr0, 0);
		rd("a read after", 21'h40000, FW_MAGIC);

		$display("-- firewall, HPS reset held low while reads and stores run: every access answers, nothing is freed");
		ena_take;
		axi_read(FW_CTRL, 0, oc, dl, beats); v = {30'd0, ~oc[1:0]} & FW_MASK;
		fr0 = n_fr; fw0 = n_fw;
		hps_rst_n = 0; fw_wait_high;
		for (j = 0; j < 4; j = j + 1) begin
			rd("magic, reset held", 21'h40000, FW_MAGIC);
			axi_write(FW_CTRL, v, 4'hF);
			rd("a store, reset held: dropped, no enable", FW_CTRL, 32'd0);
			axi_write(ENA, ENA_W, 4'hF);
			axi_read(ENA, 0, oc, dl, beats);
			check("an ENABLE, reset held: not taken, hrst shown", {oc[31], oc[15]}, 2'b01);
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
		ena_take;
		axi_read(FW_CTRL, 0, oc, dl, beats); v = {30'd0, ~oc[1:0]} & FW_MASK;
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
		fw_w1(oc);                                     // its next store's data beat: lands on LINE_EN's address
		fw_i_mon = 0;
		repeat (4) @(negedge clk);
		check("idle read 0 from the orphans to the last data beat", fw_i_ones, 0);
		check("records: the next beat completes the accepted AW (ID)", fb_id, 12'h0B2);
		check("idle after", fw_idle, 1);
		check("the next beat's store on the bus: LINE_EN's address, its data", {fw_wa == FW_CTRL, fw_wd == oc}, 2'b11);
		rd("LINE_EN: no enable after the reset", FW_CTRL, 32'd0);

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
		if (TB_PAD) begin
			dm_entry(e, 1, 32'h00010001, 32'h40000, 32'h40000, 0, 0, 0);           // PAD 1.1
			dm[16 + 32 * e + 8] = 32'h040F000C; dm[16 + 32 * e + 9] = 32'h20000000;   // K0-K7: the opcode bitmap
			dm[16 + 32 * e + 16] = 32'h00100018;                                       // K8 {spinner FIFO, slots}
			e = e + 1;
		end
		if (TB_SEC) begin
			dm_entry(e, 2, 32'h00020000, 32'h08000, 32'h08000, 32'h08000, 1, TB_W1 ? 7 : 3);   // 2.0: line 1, a cause per slot
			b = 16 + 32 * e + 8;
			dm[b]     = 32'h02000000 | TB_GAP;                                         // K0 {FIFO entries, 0, gap}
			dm[b + 1] = 32'h00000300 | TB_WIDE;                                        // K1 {BLKSZ, wide}
			dm[b + 2] = 32'd3;                                                         // K2: three descriptors
			dm[b + 4] = 32'h01000000 | (TB_PS << 16) | TB_FRAME;                       // K4-K7: the primary, RD
			dm[b + 5] = 32'd32;
			dm[b + 8] = 32'h03000000 | (TB_S1H << 16) | TB_XFRAME0;                    // K8-K11: slot 1, RD and WR
			dm[b + 9] = TB_LBAB; dm[b + 10] = 32'h30100000; dm[b + 11] = 32'h00020000;
			dm[b + 12] = (TB_W1 ? 32'h03000000 : 32'h0) | (TB_S2H << 16) | TB_XFRAME1; // K12-K15: slot 2, flags 0
			dm[b + 13] = TB_LBAB; dm[b + 14] = 32'h30200000; dm[b + 15] = 32'h00020000; //   unwired
			e = e + 1;
		end
		if (TB_OSD) begin
			dm_entry(e, 3, 32'h00020000, 32'h04000, 32'h02000, 0, 0, 0);           // OSDBUF 2.0
			dm[16 + 32 * e + 8] = TB_MENU ? 32'h00140100 : 32'h00100100;              // K0 {rows, bytes per row}
			dm[16 + 32 * e + 9] = 32'h01000000;                                        // K1 {FIFO entries, 0}
			e = e + 1;
		end
		if (TB_EVT) begin
			dm_entry(e, 4, 32'h00020000, 32'h03000, 32'h01000, 32'h03000, 2, 32'hF);   // EVT 2.0: line 2, four causes
			dm[16 + 32 * e + 8] = 32'd4;                                               // K0: the status words
			e = e + 1;
		end
		if (TB_LANE) begin
			dm_entry(e, 5, 32'h00010002, 32'h06000, 32'h01000, 32'h06024, 3, 32'h7);   // LANE 1.2: line 3, three causes
			b = 16 + 32 * e + 8;
			dm[b] = 32'h70000002; dm[b + 1] = 32'h080E0014;                            // K0-K7: the replay bitmap
			dm[b + 8] = 32'h00000003; dm[b + 9] = 32'h04000004; dm[b + 10] = 32'h01000000;
			e = e + 1;
		end
		em_n = e;
		dm[0] = 32'h5244574C; dm[1] = 32'h00020000; dm[2] = 32'h00800040; dm[3] = e;
		dm[4] = 32'h00040000; dm[5] = 32'h00080000; dm[6] = 32'h40000000; dm[7] = 32'd0;
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
	kind_major = (k >= 16'd2 && k <= 16'd4) ? 2 : (k == 16'd1 || k == 16'd5) ? 1 : 0;
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
		rd("ENABLE: not set, quiet",     DIRB + 21'h18, 32'h40000000);
		rd("0x1C reserved: 0",           DIRB + 21'h1C, 32'd0);
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
		check("entries with a line", nl, TB_SEC + TB_EVT + TB_LANE);
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
		syn_add(n + 4, 32'h0003, 32'h00020000, 32'h7F000, 32'h2000, 0, 32'd0, 0);                // OSD, past the end
		syn_add(n + 5, 32'h0004, 32'h00020000, 32'h03000, 32'h1000, 32'h05000, 32'd0, 0);        // SYS, CAUSE outside
		dir_walk(1, tb_off, tb_size); walk_check("six entries to skip", 0, 0);
		syn[16 + 32 * (n + 1) + 5] = 32'd2;                         // the unknown kind names the SYS entry's ordinal
		dir_walk(1, tb_off, tb_size); walk_check("an ordinal two entries name: no line for either", 4, 0);
		syn[16 + 32 * (n + 1) + 5] = 32'd4;
		if (em_n > 0) begin
			syn[16 + 5] = 32'd20;                                  // the first entry names a private ordinal
			dir_walk(1, tb_off, tb_size); walk_check("a framework ordinal out of 1-15: no line", em_kind[0], 0);
		end

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
	if (win.r_dir + win.r_sec + win.r_evt + win.r_osd + win.r_lane + win.r_pad + win.r_none != 1 ||
	    win.w_dir + win.w_sec + win.w_evt + win.w_osd + win.w_lane + win.w_pad > 1) sel_bad = sel_bad + 1;
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
		if (TB_SEC) begin sec_wr(S_CTRL, 32'h0); settle; end
		for (i = 0; i < wk_n; i = i + 1) if (wk_cause[i] != 0) begin
			for (j = 0; j < wk_n; j = j + 1) if (wk_cause[j] != 0) cause_drain(DIRB + wk_cause[j]);
			rd($sformatf("kind %0d: PENDING clear before", wk_kind[i]), DIRB + 21'h20, 32'd0);
			check($sformatf("kind %0d: lines low before", wk_kind[i]), lines, 0);
			// SECTOR's request with its EN set: the mux claims the slot, so hps_io shows it nowhere (no EVT BLK)
			if (wk_kind[i] == 2) begin sec_wr(S_CTRL, C_EN0); settle; cd_request(32'h00007700); end
`ifdef MISTER_LW_LANE
			else if (wk_kind[i] == 5) begin                 // LANE: a frame played
				lane_count(v, vl); f_set(0, 16'h001E, 2); fw[0] = 16'h7701; fw[1] = 16'h7702;
				lane_send(1); lane_wait(v + 1);
			end
`endif
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
				sec_wr(S_CTRL, 32'h0); settle;
			end
		end
		check("ordinal 3: its line rose exactly when the lane is built", line3_high != 0, TB_LANE != 0);
		$display("     %0d cycles with the block selects compared", sel_cycles);
		check("cycles with no read select, two read selects or two store selects", sel_bad, 0);
	end
endtask

// Every 4 KB page of the 2 MB against the window's map: a block's page reads that block (never the default), every
// other page reads 0xDEADBEEF at its first, middle and last word and drops a store; each block's ID word is at its
// offset.
function integer blk_at(input [20:0] a);
	if (a >= 21'h40000 && a < 21'h41000) blk_at = 1;                    // the directory
	else if (TB_SEC && a >= 21'h48000 && a < 21'h50000) blk_at = 2;     // SECTOR
	else if (TB_EVT && a >= 21'h43000 && a < 21'h44000) blk_at = 3;     // SYS
	else if (TB_OSD && a >= 21'h44000 && a < 21'h46000) blk_at = 4;     // OSDBUF
	else if (TB_LANE && a >= 21'h46000 && a < 21'h47000) blk_at = 5;    // LANE
	else if (TB_PAD && a >= 21'h80000 && a < 21'hC0000) blk_at = 6;     // PAD
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
		check("pages in blocks", nblk, 1 + 8 * TB_SEC + TB_EVT + 2 * TB_OSD + TB_LANE + 64 * TB_PAD);
		check("every page reads as the map says", bad, 0);
		rd("the directory's MAGIC at its offset", DIRB, FW_MAGIC);
		if (TB_SEC) rd("SECTOR's ID at its offset", SEC + S_ID, 32'h33434553);
		if (TB_EVT) rd("SYS's ID at its offset", SYSB + 21'h1C, 32'h32545645);
		if (TB_OSD) rd("OSDBUF's ID at its offset", OSD + 21'h1804, 32'h3244534F);
		if (TB_LANE) rd("LANE's ID at its offset", 21'h46018, 32'h314E414C);
		if (TB_PAD) rd("PAD's ID at its offset", PADB + 21'h4, 32'h31444150);
		if (TB_PAD) rd("PAD reads 0", pad_addr(8'h02, 0), 32'd0);
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

// ---- The host enable: the gate, the store, the ends, the lines and the start order ----
// en_q's edges, and the lines against their definition in every cycle: line k is the register of en_q & its
// entry's LINE_EN bit & its entry's CAUSE word non-zero (SECTOR ordinal 1, SYS 2, LANE 3).
realtime t_lrise = 0, t_lfall = 0;
integer  n_lrise = 0, n_lfall = 0;
always @(win.en_q) if ($realtime > 0) begin
	if (win.en_q) begin t_lrise = $realtime; n_lrise = n_lrise + 1; end
	else begin t_lfall = $realtime; n_lfall = n_lfall + 1; end
end
reg  [2:0] lm_q = 0;
integer    lm_n = 0, lm_bad = 0, lm_fall_c = 0, lm_fall_e = 0, lm_fall_l = 0, lm_rises = 0, lm_falls = 0;
always @(posedge clk) begin : line_model
	reg [2:0] en, lv, nx;
	en = {TB_LANE != 0 && win.line_en[TB_E_LANE], win.line_en[TB_E_SYS], win.line_en[TB_E_SEC]};
	lv = {win.lane_lvl, win.evt_lvl, win.sec_lvl};
	nx = {3{win.en_q}} & en & lv;
	for (i_lm = 0; i_lm < 3; i_lm = i_lm + 1) if (lm_q[i_lm] && !nx[i_lm]) begin
		if (!win.en_q) lm_fall_l = lm_fall_l + 1;
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

// A second host's look before it touches the link: the GPO value as left (the bench's gpo31), then one GPI sample.
// 1: an LW host holds the enable (store nothing); 0: go on.
function look_held;
	look_held = gpo31 && tb_gpi[31] == 1'b0 && tb_gpi[25] && tb_gpi[22];
endfunction

task ena_cases;
	reg [31:0] v, v0;
	integer k, n, j, nw0;
	begin
		$display("-- enable: without it every store but ENABLE is dropped and sets stale");
		ena_w;                                                        // an ENABLE clears stale and burst
		host_end;
		wait_quiet;
		ena_rd(v);
		check("ended: not enabled, quiet, nothing stale", v, 32'h40000000);
		rd("OPEN: no block takes stores", OPENA, 32'd0);
		rd("OWNED: nothing owned", OWNEDA, 32'd0);
		check("GPI[22] reads 0", tb_gpi[22], 0);
		nw0 = n_lwwr;
		v0 = win.scratch;   // what earlier cases left (0 without LW_DIAG)
		axi_write(SCR, 32'h12345678, 4'hF);
		rd("no enable: SCRATCH unchanged", SCR, v0);
		axi_write(LINEN, 32'hFFFFFFFF, 4'hF);
		rd("no enable: LINE_EN unchanged", LINEN, 32'd0);
		n = n_padwe; axi_write(pad_addr(8'h02, 0), 32'h00005555, 4'hF); settle;
		check("no enable: a pad store is dropped, nothing reaches hps_io", n_padwe - n, 0);
		n = n_osdq; j = n_osdwe;
		axi_write(OSD + 21'h4, 32'hFFFFFFFF, 4'hF); axi_write(OSD, 32'hFFFFFFFF, 4'hF); settle;
		check("no enable: an image store takes no entry and writes no byte", (n_osdq - n) + (n_osdwe - j), 0);
		st_req(st_pat(900)); settle;
		axi_write(SYSB, 32'h1, 4'hF);
		if (TB_EVT) rd("no enable: a W1C store on SYS CAUSE is dropped, the cause stands", SYSB, 32'h1);
		if (TB_SEC) begin
			sec_wr(S_CTRL, C_EN0);
			rd("no enable: SECTOR CTRL unchanged", SEC + S_CTRL, 32'd0);
		end
		if (TB_LANE) begin
			axi_read(21'h46010, 0, v0, dl, beats);
			axi_write(21'h46000, 32'h0000001E, 4'hF); axi_write(21'h46008, 32'h0, 4'hF);
			repeat (slow(200)) @(negedge clk);
			rd("no enable: no frame committed, none played (PLAYED)", 21'h46010, v0);
			check("no enable: the lane's write pointers unmoved", win.lane.lane.wcom == win.lane.lane.wspec, 1);
		end
		check("no enable: every store reached the bus", n_lwwr - nw0, 6 + TB_SEC + 2 * TB_LANE);
		ena_rd(v);
		check("no enable: stale set, nothing else", v, 32'h60000000);

		$display("-- enable: a wrong key, a wrong bit, any other word and partial strobes do nothing");
		axi_write(ENA, 32'h4C560001, 4'hF);
		axi_write(ENA, 32'h4C570000, 4'hF);
		axi_write(ENA, 32'h4C570003, 4'hF);
		axi_write(ENA, 32'h4C575551, 4'hF);                          // a CLAIM of the withdrawn lease
		axi_write(ENA, 32'hFFFFFFFF, 4'hF);
		axi_write(ENA, ENA_W, 4'h7);
		axi_write(ENA, ENA_W, 4'hE);
		ena_rd(v);
		check("refused stores: ENABLE unchanged (stale set before)", v, 32'h60000000);
		axi_wburst(SCR, 4'd1, 2'b01, n);
		ena_rd(v);
		check("a burst write without the enable: burst set", v[28], 1);

		$display("-- enable: ENABLE sets it, clears stale and burst; stores take effect");
		axi_write(ENA, ENA_W, 4'hF);
		ena_rd(v);
		check("ENABLE: {en, stale, burst}", {v[31], v[29:28]}, 3'b100);
		check("ENABLE: quiet only with no enable link", v[30], TB_LINKED == 0);
		check("ENABLE: no other bit", v & 32'h0FFFFFFF, 0);
		check("GPI[22] shows the enable", tb_gpi[22], 1);
		axi_write(ENA, ENA_W, 4'hF);
		ena_rd(v);
		check("a second ENABLE: changes nothing", v[31], 1);
		wait_open;
		rd("OPEN: every listed block", OPENA, (32'd1 << em_n) - 1);
		rd("OWNED: none", OWNEDA, 32'd0);
		axi_write(SCR, 32'h12345678, 4'hF);
		rd("enabled: SCRATCH", SCR, dg(32'h12345678));
		axi_write(LINEN, 32'hFFFFFFFF, 4'hF);
		rd("enabled: LINE_EN holds the listed entries' bits", LINEN, (32'd1 << em_n) - 1);
		axi_write(pad_addr(8'h02, 0), 32'h00005555, 4'hF); settle;
		if (TB_PAD) check("enabled: the pad store lands", joy0, 32'h00005555);
		axi_write(pad_addr(8'h02, 0), 32'h00000000, 4'hF); settle;
		if (TB_OSD) begin
			osd_word(15*64 + 63, 32'h600DF00D, 4'hF); osd_drain;
			check("enabled: an image store lands in both osd_buffers",
			      {bw_h(15*64 + 63) == 32'h600DF00D, bw_v(15*64 + 63) == 32'h600DF00D}, 2'b11);
		end
		axi_write(SYSB, 32'h1, 4'hF);
		if (TB_EVT) rd("enabled: the W1C clears SYS CAUSE", SYSB, 32'd0);
		axi_wburst(SCR, 4'd3, 2'b01, n);
		ena_rd(v);
		check("a burst write while enabled: burst set, stale clear", v[29:28], 2'b01);
		axi_write(SCR, 32'h0, 4'hF);
		$display("-- enable: no timer: it holds for as long as the bench waits, with no store");
		repeat (slow(20000)) @(negedge clk);
		ena_rd(v);
		check("still enabled after 20,000 cycles and more with no store", v[31], 1);
		if (TB_LINKED != 0) begin
			$display("-- enable: a store to a linked block before its OPEN bit is dropped and sets stale");
			host_end;
			wait_quiet;
			sys_run = 0; repeat (20) @(negedge clk);       // the far halves cannot answer the rise
			axi_write(ENA, ENA_W, 4'hF);
			ena_rd(v);
			check("clk_sys stopped: the ENABLE is taken, not quiet, nothing stale", {v[31:29]}, 3'b100);
			rd("clk_sys stopped: OPEN shows the unlinked entries only", OPENA, ((32'd1 << em_n) - 1) & ~TB_LINKED);
			rd("clk_sys stopped: OWNED none (the links rise under a held enable)", OWNEDA, 32'd0);
			n = n_lwwr; j = n_osdq;
			if (TB_SEC) sec_wr(S_CTRL, C_EN0);
			if (TB_LANE) axi_write(LANE, 32'h0000001E, 4'hF);
			if (TB_OSD) axi_write(OSD + 4*(15*64 + 62), 32'hBAD0BAD0, 4'hF);
			ena_rd(v);
			check("a store to a linked block, not open: stale set", v[29], 1);
			if (TB_OSD) check("not open: the OSD store takes no entry", n_osdq - j, 0);
			if (TB_SEC) check("not open: SECTOR's EN unchanged", win.sec.sector.en_lwx, 0);
			if (TB_LANE) check("not open: the lane's frame not begun", win.lane.lane.open, 0);
			axi_write(pad_addr(8'h02, 0), 32'h0BADF00D, 4'hF);   // PAD is not linked: kept for the restart
			repeat (20) @(negedge clk);
			if (TB_PAD) check("clk_sys stopped: the pad word not yet delivered", joy0 == 32'h0BADF00D, 0);
			sys_run = 1;
			settle;
			if (TB_PAD) check("clk_sys running again: the PAD store kept and delivered", joy0, 32'h0BADF00D);
			wait_open;
			rd("clk_sys running again: OPEN every listed block", OPENA, (32'd1 << em_n) - 1);
			if (TB_SEC) begin
				sec_wr(S_CTRL, C_EN0);
				rd("open: SECTOR's EN store lands", SEC + S_CTRL, 32'h1);
				sec_wr(S_CTRL, 32'h0);
			end
			if (TB_OSD) begin
				osd_word(15*64 + 62, 32'h0FF1CE00, 4'hF); osd_drain;
				check("open: the OSD store lands", {bw_h(15*64 + 62) == 32'h0FF1CE00, bw_v(15*64 + 62) == 32'h0FF1CE00}, 2'b11);
			end
		end

		$display("-- enable: its ends: an HPS reset, GPO[31] low for one cycle and long; none taken in the ID view");
		ena_take;
		@(negedge clk); hps_rst_n = 0; repeat (3) @(negedge clk); hps_rst_n = 1;
		fw_wait_fall; repeat (2) @(negedge clk);
		ena_rd(v);
		check("an HPS reset: ended", v[31], 0);
		rd("no enable: LINE_EN reads 0", LINEN, 32'd0);
		rd("no enable: OPEN reads 0", OPENA, 32'd0);
		check("GPI[22] clear", tb_gpi[22], 0);
		ena_take;
		@(negedge clk); #2 gpo31 = 0; #10 gpo31 = 1;
		repeat (8) @(negedge clk);
		ena_rd(v);
		check("GPO[31] low for one clk_100m cycle: ended", v[31], 0);
		ena_take;
		@(negedge clk); gpo31 = 0; repeat (8) @(negedge clk);
		check("GPO[31] low: GPI[22] reads 0 in the ID view", tb_gpi[22], 0);
		for (j = 0; j < 3; j = j + 1) axi_write(ENA, ENA_W, 4'hF);
		ena_rd(v);
		check("GPO[31] held low: no ENABLE taken, idv shown", {v[31], v[11]}, 2'b01);
		repeat (200) @(negedge clk);
		axi_write(ENA, ENA_W, 4'hF);
		check("GPO[31] held low for 200 cycles more: still none", win.en_q, 0);
		gpo31 = 1; repeat (6) @(negedge clk);
		ena_take;
		check("GPO[31] high again: an ENABLE is taken", win.en_q, 1);
		rd("after an ENABLE: LINE_EN reads 0", LINEN, 32'd0);

		$display("-- lines: low without the enable whatever the causes; PENDING shows them either way; LINE_EN per entry");
		host_end;
		st_req(st_pat(902)); settle;
		if (TB_SEC) begin cd_request(32'h00007E00); settle; end
		axi_read(DIRB + 21'h20, 0, v, dl, beats);
		check("no enable: PENDING shows SYS (SECTOR's request has no EN, so no cause)", v,
		      TB_EVT ? 32'd1 << TB_E_SYS : 0);
		check("no enable: every line low", lines, 0);
		rd("OWNED reads 0", OWNEDA, 32'd0);
		ena_take;
		settle;
		check("enabled, LINE_EN 0: every line low", lines, 0);
		rd("PENDING when enabled: SYS's (SECTOR's word cleared at the ENABLE)", DIRB + 21'h20,
		   TB_EVT ? 32'd1 << TB_E_SYS : 32'd0);
		if (TB_SEC) begin
			sec_wr(S_CTRL, C_EN0); settle;
			rd("an EN rise raises SECTOR's cause for its standing request", DIRB + 21'h20,
			   (32'd1 << TB_E_SEC) | (TB_EVT ? 32'd1 << TB_E_SYS : 0));
		end
		axi_write(LINEN, 32'd1 << TB_E_SYS, 4'hF); settle;
		check("LINE_EN with SYS's bit: SYS's line alone", lines, TB_EVT ? 3'b010 : 3'b000);
		if (TB_SEC) begin
			axi_write(LINEN, 32'd1 << TB_E_SEC, 4'hF); settle;
			check("LINE_EN with SECTOR's bit: SECTOR's line alone", lines, 3'b001);
			axi_write(LINEN, (32'd1 << TB_E_SEC) | (32'd1 << TB_E_SYS), 4'hF); settle;
			check("both bits: both lines", lines, TB_EVT ? 3'b011 : 3'b001);
		end
		axi_write(LINEN, ~((32'd1 << TB_E_SEC) | (32'd1 << TB_E_SYS)), 4'hF); settle;
		check("the other bits: no line", lines, 0);
		axi_write(LINEN, 32'hFFFFFFFF, 4'hF); settle;
		host_end;
		rd("LINE_EN reads 0 after the ID view", LINEN, 32'd0);
		check("the enable ended: every line low", lines, 0);
		ena_take;
		rd("LINE_EN reads 0 after an ENABLE", LINEN, 32'd0);
		if (TB_SEC) begin
			$display("-- lines: a host that skips SECTOR is never woken by it; the block takes nothing from SPI");
			axi_write(LINEN, 32'd1 << TB_E_SYS, 4'hF);
			sys_drain; settle;
			check("after the ENABLE: SECTOR's cause clear, its request standing", {win.sec_lvl, sd_rd[1]}, 2'b01);
			@(negedge clk_sys) cd_reset = 1; repeat (4) @(negedge clk_sys); cd_reset = 0;
			cd_request(32'h00007E00); settle;
			check("a new request without EN: no SECTOR cause", win.sec_lvl, 0);
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
			if (TB_EVT) check("the host's other line works: SYS", lines, 3'b010);
			sys_drain; cause_drain(SEC);
		end
		else sys_drain;
		host_end;
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
	if (TB_EVT) begin
		$display("-- lines are levels: a raise swept across the store-back's cycle; where it wins, no low cycle");
		ena_up;
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
				check($sformatf("raise %0d in the clear's cycle: the cause stands", k), win.evt_lvl, 1);
				check($sformatf("raise %0d in the clear's cycle: the line had no low cycle", k), low, 0);
			end
			sys_drain; settle;
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
		check("quiescent: the cause clear, the line low", {win.evt_lvl, lines[1]}, 2'b00);
		ena_off;
	end
endtask

// Look before identifying. A second host model reads the GPO value as left and one GPI sample; an enable held
// makes it store nothing. A takeover on purpose is an identification; a stock start, GPO = 0 first, ends a live
// host's enable too.
task look_cases;
	reg [31:0] v, v0, p, dr;
	integer n, nw0;
	begin
		$display("-- the start order: a second host looks at GPO and GPI[22] first, sees the enable and stores nothing");
		ena_up;
		if (TB_SEC) sec_wr(S_CTRL, C_EN0);
		if (TB_OSD) begin osd_word(0, 32'h5A5AA5A5, 4'hF); osd_drain; end
`ifdef MISTER_LW_LANE
		lane_count(p, dr);
		io_enable = 1;                                      // the link busy: frames queue
		f_set(0, 16'h001E, 2); fw[0] = 16'h1601; fw[1] = 16'h1602;
		lane_send(1); lane_send(1);
		repeat (slow(100)) @(negedge clk);
`endif
		nw0 = n_lwwr;
		check("second host: the GPO value it reads is the I/O view, and GPI says an LW host is enabled", look_held(), 1);
		repeat (50) @(negedge clk);                         // it stops: no store, no GPO change
		check("second host: stored nothing", n_lwwr - nw0, 0);
		ena_rd(v);
		check("the first host's enable as it was", v[31], 1);
		if (TB_SEC) rd("the first host's SECTOR binding as it was", SEC + S_CTRL, 32'h1);
		if (TB_OSD) begin
			check("the first host's image word as it was", {bw_h(0) == 32'h5A5AA5A5, bw_v(0) == 32'h5A5AA5A5}, 2'b11);
			osd_word(1, 32'h1B1B1B1B, 4'hF); osd_drain;
			check("the first host's next image store lands", {bw_h(1) == 32'h1B1B1B1B, bw_v(1) == 32'h1B1B1B1B}, 2'b11);
		end
`ifdef MISTER_LW_LANE
		lane_count(v, dr);
		check("the first host's frames still queued", v, p);
		io_enable = 0;
		lane_wait(p + 2);
`endif
		axi_write(SCR, 32'h0B0B0B0B, 4'hF);
		rd("the first host's next store lands", SCR, dg(32'h0B0B0B0B));

		$display("-- the start order: a takeover on purpose: the second host identifies at once; the first host's stores drop");
		identify(4);
		ena_rd(v);
		check("the identification ended the enable", v[31], 0);
		axi_write(SCR, 32'h0A0A0A0A, 4'hF);
		ena_rd(v);
		check("the first host's next store: dropped, stale", {v[31], v[29]}, 2'b01);
		rd("SCRATCH not stored", SCR, dg(32'h0B0B0B0B));
		ena_w;
		check("the second host enables", win.en_q, 1);

		$display("-- the start order: a stock start (GPO = 0 first) beside a live host: its enable ends, its stores drop");
		wait_open;
		@(negedge clk); gpo31 = 0;                            // the stock first act: GPO = 0, then the ID view
		repeat (40) @(negedge clk);
		n = n_osdq;
		axi_write(OSD + 4*2, 32'h2C2C2C2C, 4'hF);            // the live host goes on storing
		gpo31 = 1; repeat (6) @(negedge clk);                 // the stock program speaks in the I/O view
		axi_write(SCR, 32'h0C0C0C0C, 4'hF);
		ena_rd(v);
		check("a stock start: the enable ended, stale set", {v[31], v[29]}, 2'b01);
		if (TB_OSD) check("a stock start: the live host's image store dropped", n_osdq - n, 0);
		rd("a stock start: SCRATCH not stored", SCR, dg(32'h0B0B0B0B));
		if (TB_SEC) check("a stock start: SECTOR's EN gone", blk_dn[18], 0);
		$display("     GPI[22] compared with en_q in %0d cycles, %0d of them held", gpi22_n, gpi22_on);
	end
endtask

// A new host starts from stock. The bench's host enables and binds every block: SECTOR EN with a frame begun and
// half stored, an OSD image word written and a repaint (rows 8-15 among it) queued behind a held osd select, lane
// frames queued behind a held select, every line. Its successor (any program: a stock Main, a swapped binary, the
// same one re-executed) starts with the core running: GPO = 0, the ID view for its identification, then the I/O
// view. With no store to the window, it then finds the fabric off and the core on stock SPI: ENABLE and GPI[22] 0,
// every line low with causes pending, no queued OSD byte written and highres as it was, the SECTOR frame completed
// and the slot SPI's, an SPI frame serving the next request, the successor's link words intact with no queued
// frame strobed, and an SPI OSD row shown.
task fresh_cases;
	reg [31:0] v, p, dr, lba;
	integer k, n1, pl0;
	begin
		$display("-- a new host starts from stock: every block bound, then the successor's ID view");
`ifdef MISTER_LW_LANE
		f_set(0, 16'h001E, 2); fw[0] = 16'h4E01; fw[1] = 16'h4E02;
		trk_clear; link_send; trk_save;                    // the successor's own link frame, the reference
		f_set(0, 16'h001E, 2); fw[0] = 16'h0000; fw[1] = 16'h0000; link_send;
`endif
		ena_up;
		if (TB_OSD) begin
			osd_lo;                                       // 64 lines
			ob_sync("bound");
			osd_word(64, 32'h5A5AA5A5, 4'hF); osd_drain;
			check("bound: an image word over LW in both osds", {bw_h(64) == 32'h5A5AA5A5, bw_v(64) == 32'h5A5AA5A5}, 2'b11);
			@(negedge clk_sys); io_osd = 1;              // the dying host's osd select: the repaint queues
			for (k = 9*64; k < 9*64 + 30; k = k + 1) axi_write(OSD + 4*k, ~pw(k, 31, 0), 4'hF);
			repeat (slow(50)) @(negedge clk);
			osd_st(v);
			check("bound: the repaint queued", v[24:16], 30);
		end
		if (TB_SEC) begin
			sec_wr(S_CTRL, C_EN0);
			sec_stand(32'h00004E00); lba = cd_lba; if (win.sec_lvl) cause_drain(SEC);
			n1 = cd_frames;
			sec_wr(S_BEGIN0, lba);
			frame_data(lba, 0, 20);                         // the frame begun, its words not all stored
`ifdef MISTER_LW_SECTOR_SLOT
			k = 0; while (win.sec.sector.fidle && k < slow(2000)) begin
				@(negedge clk); k = k + 1; end
`endif
			check("bound: SECTOR's slot on LW, its frame open", {blk_dn[18], k < slow(2000)}, 2'b11);
		end
`ifdef MISTER_LW_LANE
		@(negedge clk_sys); io_enable = 1;                 // the dying host's select: frames queue behind it
		f_set(0, 16'h001E, 2);
		for (k = 0; k < 3; k = k + 1) begin fw[0] = 16'h4D00 + k; fw[1] = 16'h4D80 + k; lane_send(1); end
		repeat (slow(100)) @(negedge clk);
		pl0 = n_play;
`endif
		st_req(st_pat(990)); settle;
		if (TB_EVT) check("bound: SYS's line high", lines[1], 1);
		// the successor: GPO = 0 (the select drops, the ID view), its identification, then the I/O view
		@(negedge clk_sys); gpo31 = 0;
		io_osd = 0;
`ifdef MISTER_LW_LANE
		io_enable = 0;
`endif
		repeat (slow(2000)) @(negedge clk);
		gpo31 = 1; repeat (2) @(negedge clk_sys); repeat (6) @(negedge clk);
		ena_rd(v);
		check("the successor finds ENABLE 0", v[31], 0);
		check("the successor finds GPI[22] 0", tb_gpi[22], 0);
		check("every line low, the SYS cause still pending", {lines, win.evt_lvl}, {3'b000, TB_EVT != 0});
		repeat (slow(50)) @(negedge clk);
		if (TB_OSD) begin
			osd_st(v);
			check("OSD: nothing waiting", v[24:16], 0);
			ob_cmp("OSD: no queued byte of the dead host's repaint written", 0, 16);
			check("OSD: highres as it was, 64 lines", {hdmi_osd.osd_h == 64, vga_osd.osd_h == 64}, 2'b11);
			spi_osd_row(3, 33);
			ob_cmp("OSD: an SPI row lands, and nothing else moved", 0, 16);
			osd_check("OSD: the SPI image drawn", 64, 6);   // the enable shows after up to 5 frames
		end
		if (TB_SEC) begin
			wait_frame(n1);
			check_buf("the frame begun completes, filled from its first missing word", lba, 20);
			k = 0; while ((blk_dn[18] || blk_dn[17]) && k < slow(5000)) begin @(negedge clk); k = k + 1; end
			check("SECTOR's slot back on SPI, its ack done", blk_dn[18:17], 2'b00);
			sec_stand(32'h00004E10);
			n1 = cd_frames; spi_frame(cd_lba); wait_frame(n1);
			check_buf("a stock SPI frame serves the next request", cd_lba, TB_FRAME);
		end
`ifdef MISTER_LW_LANE
		trk_clear; f_set(0, 16'h001E, 2); fw[0] = 16'h4E01; fw[1] = 16'h4E02; link_send;
		repeat (slow(400)) @(negedge clk);
		trk_cmp("the successor's link words intact, no queued frame strobed");
		check("no queued frame played", n_play - pl0, 0);
`endif
		wait_quiet;
		ena_rd(v);
		check("every block returned: quiet, OWNED clear", {v[30], v[31]}, 2'b10);
		rd("OWNED clear", OWNEDA, 32'd0);
	end
endtask

`ifdef MISTER_LW_SECTOR_SLOT
// SECTOR at the next program's ID view, in each state of the primary slot, with clk_sys running or stopped
// across it. The ack completes, en_sys drops, the slot is SPI's again, and an SPI frame serves it.
integer sx_fs = -1;     // the feeder's state in the cycle the enable fell
always @(negedge win.en_q) sx_fs = win.sec.sector.fs;
task sec_view(input integer sc, input stopped);
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
		$display("-- SECTOR at an ID view: %s", w);
		lba = 32'h00007A00 + {sc[3:0], stopped, 4'd0};
		ena_short;
		sec_wr(S_CTRL, C_EN0); settle;
		check({w, ": bound"}, blk_dn[18], 1);
		n0 = cd_frames; nst = 0;
		if (sc >= 1) begin cd_request(lba); settle; cause_drain(SEC); end
		if (sc >= 2) begin sec_wr(S_BEGIN0, lba); settle; end
		if (sc == 3) begin frame_data(lba, 0, 294); nst = 294; end
		if (sc == 4) begin
			frame_data(lba, 0, 20); nst = 20;
			repeat (12000) @(negedge clk);                  // the 20 words played, the frame waits for more
			sec_wr(S_CTRL, C_EN0 | C_ABORT);                // CTRL.ABORT: the fill runs across the ID view
			repeat (295) @(negedge clk);
		end
		if (sc == 5) begin
			sys_run = 0; repeat (20) @(negedge clk);
			frame_data(lba, 0, TB_FRAME);
			axi_read(SEC + S_ACCEPT, 0, v, dl, beats); nst = v[9:0];
			check({w, ": the FIFO filled, stores dropped"}, v[27:16] != 0, 1);
			if (!stopped) sys_run = 1;
			repeat (200) @(negedge clk);
		end
		if (stopped && sc != 5) begin sys_run = 0; repeat (20) @(negedge clk); end
		host_end;
		ena_rd(v);
		check({w, ": the ID view ended the enable"}, v[31], 0);
		if (sc == 4 && !stopped) check({w, ": the fill was running at the ID view"}, sx_fs, 3);
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
		ena_take; cause_drain(SEC); host_end;
	end
endtask
// The same for slot 1 (T-LS3 with a save slot): a frame not stored in full never reaches the core, and its request
// is served over SPI after the view; one stored in full and playing at the view plays whole.
task sec_view_save(input integer sc, input stopped);
	reg [31:0] v;
	reg  [6:0] l;
	reg        ok;
	integer a0, s0x, t;
	string  w;
	begin
		case (sc)
			0: w = "idle";
			1: w = "a request unanswered";
			2: w = "mid-BEGIN";
			3: w = "mid-DATA";
			default: w = "stored in full";
		endcase
		w = {w, stopped ? ", clk_sys stopped across it" : ", clk_sys running"};
		$display("-- SECTOR at an ID view, slot 1: %s", w);
		l = 7'h48 + {sc[2:0], stopped};
		for (t = 0; t < 512; t = t + 1) sv1.ram[t] = 16'hBEEF;
		ena_short;
		sec_wr(S_CTRL, C_EN1); settle;
		check({w, ": bound"}, save_dn[9], 1);
		a0 = sv1.acks; s0x = sv1.strobes;
		if (sc >= 1) begin sv1.go(0, l); settle; cause_drain(SEC); end
		if (sc >= 2) sec_wr(S_BEGIN1, l);
		if (sc == 3) sv_store(l, 0, TB_XFRAME0 / 2);
		if (stopped) begin @(negedge clk_sys); sys_run = 0; repeat (20) @(negedge clk); end
		if (sc == 4) begin
			sv_store(l, 0, TB_XFRAME0);
			if (!stopped) begin
				t = 0; while (!save_dn[10] && t < slow(5000)) begin @(negedge clk); t = t + 1; end
				check({w, ": the frame plays at the ID view"}, save_dn[10], 1);
			end
			else repeat (300) @(negedge clk);                   // every entry visible
		end
		host_end;
		ena_rd(v);
		check({w, ": the ID view ended the enable"}, v[31], 0);
		if (stopped) begin
			repeat (400) @(negedge clk);
			check({w, ": EN cleared on clk_100m while clk_sys is stopped"}, win.sec.sector.xen_lwx, 2'b00);
			sys_run = 1;
		end
		if (sc == 4) begin
			t = 0; while (sv1.acks == a0 && t < slow(8000)) begin @(negedge clk_sys); t = t + 1; end
			repeat (20) @(negedge clk_sys);
			if (!stopped || sv1.acks != a0) begin
				// (stopped: the delay models may let the feeder take the BEGIN before the enable's fall)
				check({w, ": the frame plays whole, one ack"}, sv1.acks - a0, 1);
				check({w, ": byte-exact"}, sv1_bad({25'd0, l}), 0);
				l = l + 7'h10;
				sv1.go(0, l);                                   // its request ended on the ack: the next one
			end
			else check({w, ": its BEGIN not taken before the fall: no ack, no strobe"}, sv1.strobes - s0x, 0);
		end
		else begin
			if (sc == 0) sv1.go(0, l);
			repeat (slow(1000)) @(negedge clk);
			check({w, ": no ack, no strobe: nothing of the frame reached the core"},
			      (sv1.acks - a0) + (sv1.strobes - s0x), 0);
		end
		sv1_spi_wait(ok);
		check({w, ": slot 1 back on SPI, its request seen by hps_io"}, ok, 1);
		sv1_spi_serve(w, 16'h3100 + 16'h0010 * sc[7:0] + stopped);
		ena_take; cause_drain(SEC); host_end;
	end
endtask
// A save-slot READ stored in full, so committed, queued behind a primary frame that plays: the enable falls (the
// next program's ID view) before the feeder reaches it. clk_sys is stopped from before the frames' last stores to
// after the fall, so the primary still has 200 words to play when the feeder sees the enable low. The primary's
// frame completes whole; slot 1's frame is discarded unplayed, its request stands, and SPI serves it.
task sec_view_queued;
	reg [31:0] v, lba;
	reg  [6:0] l;
	reg        ok;
	integer a0, s0x, t, lp;
	string  w;
	begin
		w = "a save-slot frame stored in full behind a playing primary frame";
		$display("-- SECTOR at an ID view: %s is discarded unplayed, its request standing for SPI", w);
		lba = 32'h00007C00; l = 7'h4C; lp = TB_FRAME - 200;
		ena_short;
		sec_wr(S_CTRL, C_EN0 | C_EN1); settle;
		check({w, ": both slots bound"}, {blk_dn[18], save_dn[9]}, 2'b11);
		a0 = sv1.acks; s0x = sv1.strobes; n0 = cd_frames;
		cd_request(lba); sv1.go(0, l); settle; cause_drain(SEC);
		sec_wr(S_BEGIN0, lba);
		frame_data(lba, 0, lp);
		repeat (slow(6000)) @(negedge clk);              // those words played: the frame waits for more
		check({w, ": the primary's frame plays"}, win.sec.sector.fs, 2);
		@(negedge clk_sys); sys_run = 0; repeat (20) @(negedge clk);
		frame_data(lba, lp, TB_FRAME);
		rd({w, ": the primary's frame stored in full"}, SEC + S_ACCEPT, {19'h40000, TB_FRAME[12:0]});
		sec_wr(S_BEGIN1, {25'd0, l});
		sv_store({25'd0, l}, 0, TB_XFRAME0);
		rd({w, ": slot 1's frame stored in full: ACCEPT ok"}, SEC + S_ACCEPT, {19'h40000, TB_XFRAME0});
		host_end;
		ena_rd(v);
		check({w, ": the ID view ended the enable"}, v[31], 0);
		repeat (200) @(negedge clk);
		sys_run = 1;
		wait_frame(n0);
		check_buf({w, ": the frame playing at the fall completes whole"}, lba, TB_FRAME);
		t = 0; while ((blk_dn[18] || blk_dn[17]) && t < slow(5000)) begin @(negedge clk); t = t + 1; end
		repeat (slow(2000)) @(negedge clk);
		check({w, ": no ack, no strobe: the committed frame was discarded unplayed"},
		      (sv1.acks - a0) + (sv1.strobes - s0x), 0);
		sv1_spi_wait(ok);
		check({w, ": slot 1 back on SPI, its request seen by hps_io"}, ok, 1);
		sv1_spi_serve(w, 16'h3C00);
		ena_take; cause_drain(SEC); host_end;
	end
endtask
task sec_view_cases;
	integer sc;
	begin
		for (sc = 0; sc < 6; sc = sc + 1) sec_view(sc, 1'b0);
		sec_view(1, 1'b1);
		sec_view(3, 1'b1);
		sec_view(5, 1'b1);
		for (sc = 0; sc < 5; sc = sc + 1) sec_view_save(sc, 1'b0);
		sec_view_save(1, 1'b1);
		sec_view_save(3, 1'b1);
		sec_view_save(4, 1'b1);
		sec_view_queued;
	end
endtask
`endif

`ifdef MISTER_LW_LANE
// The lane at the next program's ID view, with a frame open, three queued and one held mid-play by the link's
// wait: the one in flight completes, the queued never strobe, the open one is abandoned, a link word after is
// intact. A capture frame cut mid-play: its replies land after the enable fell and are discarded.
task lane_view_cases;
	reg [31:0] p, dr, rs;
	integer b0, k;
	begin
		$display("-- LANE at an ID view: one frame mid-play, three queued, one open");
		f_set(0, 16'h001E, 4);
		for (k = 0; k < 4; k = k + 1) fw[k] = 16'h7100 + k;
		trk_clear; link_send; trk_save;                    // the reference for the link frame sent after
		ena_short;
		lane_count(p, dr);
		ioctl_wait = 1;
		lane_send(1);
		repeat (slow(200)) @(negedge clk);
		check("ID view: the first frame holds after its first strobe", {lane_own, bus_n[7:0] - t_ref[0][7:0]}, {1'b1, 8'd1});
		for (k = 0; k < 3; k = k + 1) begin fw[0] = 16'h7200 + k; lane_send(1); end
		fw[0] = 16'h7300; lane_send(0);                    // open: BEGIN and its words, no END
		fw[0] = 16'h7100;
		b0 = bus_n;
		host_end;
		check("ID view: the open frame abandoned", win.lane.lane.wspec, win.lane.lane.wcom);
		@(negedge clk_sys); ioctl_wait = 0;
		lane_wait(p + 1);
		repeat (slow(400)) @(negedge clk);
		check("ID view: the frame in flight completed its strobes", bus_n - b0, 4);
		check("ID view: the queued frames were popped, none strobed", {lane_own, win.lane.lane.rptr == win.lane.lane.wcom},
		      2'b01);
		lane_count(v_l, dr);
		check("ID view: PLAYED counts the one frame", v_l, p + 1);
		check("ID view: the frame's PLAYED is pending, and no line rises without the enable", {win.lane_lvl, lines[2]}, 2'b10);
		trk_clear; link_send;
		trk_cmp("ID view: a link frame after");

		$display("-- LANE at an ID view: a capture frame cut mid-play; its replies are discarded");
		ena_short;
		lane_count(p, dr);
		lane_rstat(rs); rd0 = rs[31:16];
		f_set(0, 16'h0029, 8);
		for (k = 0; k < 8; k = k + 1) fw[k] = 16'h0000;
		ioctl_wait = 1;
		lane_send_c(1, 1'b1);
		repeat (slow(200)) @(negedge clk);
		host_end;
		rw0 = win.lane.lane.rwp;
		ena_rd(rs);
		check("cut capture, held by the link's wait: not quiet", rs[30], 0);
		axi_write(ENA, ENA_W, 4'hF);
		check("cut capture, held: an ENABLE is refused", win.en_q, 0);
		@(negedge clk_sys); ioctl_wait = 0;
		lane_wait(p + 1);
		repeat (slow(400)) @(negedge clk);
		check("cut capture: its replies landed after the enable fell", win.lane.lane.rwp != rw0, 1);
		lane_rstat(rs);
		check("cut capture: RSTAT shows none waiting", rs[15:0], 0);
		wait_quiet;
		ena_rd(rs);
		check("cut capture: quiet returns once the player is at rest", rs[30], 1);
		ena_up;
		check("cut capture: an ENABLE is then taken", win.en_q, 1);
		lane_rstat(rs);
		check("cut capture: RSTAT under the new enable: none waiting, no drop counted", rs, {rd0, 16'd0});
		rd("cut capture: CAUSE under the new enable: clear", LCAUSE, 32'd0);
		check("cut capture: line 3 low under the new enable", lines[2], 0);
	end
endtask
reg [31:0] v_l, dr1_l;
reg  [8:0] rw0;
reg [15:0] rd0;
`endif

// ---- The enable links: an ENABLE waits for every linked half's rest; stores wait for its acknowledge ----
// An ENABLE stored again and again, as fast as the bus allows, until one is taken or max stores were made.
task ena_hammer(input integer max, output integer tries);
	begin
		tries = 0;
		while (!win.en_q && tries < max) begin axi_write(ENA, ENA_W, 4'hF); tries = tries + 1; end
	end
endtask
// The link states as the bench reads them, 7 where a block is not built.
`ifdef MISTER_LW_SECTOR_SLOT
wire [2:0] lk_sec = win.sec.sector.fence.st;
`else
wire [2:0] lk_sec = 3'd7;
`endif
`ifdef MISTER_LW_OSD
wire [2:0] lk_osd = win.osd.osd.fence.st;
`else
wire [2:0] lk_osd = 3'd7;
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

// An ID view with a lane frame held mid-play and two queued, and a SECTOR frame begun with no data; then an
// ENABLE stored as fast as the bus allows. None is taken while a half is off rest (the ENABLE monitor, and quiet
// read 0 while the frame is held); the one taken finds REQ 0; the frame in flight completes, the queued ones
// never strobe, the SECTOR frame is zero-filled.
task ls6;
	reg [31:0] v, p, dr;
	integer tries, b0, c0, n1, k;
	begin
		$display("-- enable links: an ID view, then ENABLEs as fast as the bus allows: none before every half is at rest");
		ena_up;
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
			sec_wr(S_CTRL, C_EN0); sec_stand(32'h00006600); cause_drain(SEC);
			n1 = cd_frames;
			sec_wr(S_BEGIN0, cd_lba);
			// the ID view once the half has popped the BEGIN and opened its frame (it would discard one still queued)
`ifdef MISTER_LW_SECTOR_SLOT
			k = 0; while (win.sec.sector.fidle && k < slow(2000)) begin
				@(negedge clk); k = k + 1; end
			check("the BEGIN popped and its frame open before the ID view", k < slow(2000), 1);
`endif
		end
		host_end;
`ifdef MISTER_LW_LANE
		ena_hammer(60, tries);
		ena_rd(v);
		check("a lane frame held by the link's wait: 60 ENABLEs refused, quiet 0", {win.en_q, v[30]}, 2'b00);
		@(negedge clk_sys); ioctl_wait = 0;
`endif
		ena_hammer(slow(4000), tries);
		$display("     taken after %0d ENABLE stores", tries);
		check("an ENABLE is taken once every half is at rest", win.en_q, 1);
		check("taken ENABLEs, every half at rest", n_offrest, 0);
		if (TB_SEC) begin
			rd("the ENABLE finds REQ 0", SEC + S_REQ, 32'd0);
			wait_frame(n1);
			check_buf("the SECTOR frame begun with no data: zero-filled", cd_lba, 0);
		end
`ifdef MISTER_LW_LANE
		repeat (slow(400)) @(negedge clk);
		lane_count(v, dr);
		check("the frame in flight completed; the queued two were popped", v, p + 1);
		check("its four remaining strobes, and no other", bus_n - b0, 4);
		check("the player idle", {lane_own, lane_hold}, 2'b00);
		rd("the ENABLE finds the lane's CAUSE clear (its last frame played before it)", LCAUSE, 32'd0);
`endif
		wait_open;
		axi_write(LINEN, 32'hFFFFFFFF, 4'hF); sys_drain;
		if (TB_SEC) begin cause_drain(SEC); sec_wr(S_CTRL, 32'h0); sec_spi_clear; end
	end
endtask

// A predecessor read and cleared the causes and died with nothing served; the next ENABLE reloads them.
task ls10;
	reg [31:0] v;
	reg [15:0] r;
	begin
		$display("-- enable links: at an ENABLE EVT ORs in STATUS and every standing level; SECTOR's word is clear until EN rises over a request");
		ena_short;
		st_req(st_pat(950));
		sec_stand(32'h00006A00);                     // SECTOR's EN is 0: the request stands at hps_io
		info_v = 8'h3B; @(negedge clk_sys) info_rq = 1; @(negedge clk_sys) up_rq = 1; settle;
		if (TB_EVT) rd("before: EVT CAUSE shows all four", SYSB, 32'hF);
		if (TB_EVT) rd("before: PEND shows the three levels", SYSB + 21'h18, 32'hE);
`ifdef MISTER_LW_LANE
		begin : l10lane                          // the predecessor's frames played and a reply waits, its cause unread
			reg [31:0] p, dr;
			lane_count(p, dr);
			f_set(0, 16'h0029, 0);
			lane_send_c(1, 1'b1); lane_wait(p + 1);
			rd("before: the lane's CAUSE shows PLAYED and REPLY", LCAUSE, 32'h3);
		end
`endif
		sys_drain;
		if (TB_SEC) if (win.sec_lvl) cause_drain(SEC);
		host_end;
		ena_w; wait_open;
		if (TB_EVT) rd("after the ENABLE: EVT CAUSE shows STATUS, BLK, INFO and UPLOAD", SYSB, 32'hF);
`ifdef MISTER_LW_LANE
		rd("after the ENABLE: the lane's CAUSE clear", LCAUSE, 32'd0);
		rdm("after the ENABLE: the lane's reply stepped out, RSTAT none waiting", LANE + 21'h1C, 32'h0000FFFF, 32'd0);
		axi_write(LINEN, 32'd1 << TB_E_LANE, 4'hF); settle;
		check("the lane's LINE_EN bit: no line", lines, 0);
		axi_write(LINEN, 32'd0, 4'hF); settle;
`endif
		check("LINE_EN 0: no line", lines, 0);
		axi_write(LINEN, 32'd1 << TB_E_SYS, 4'hF); settle;
		check("SYS's LINE_EN bit: its line rises", lines[1], TB_EVT != 0);
		if (TB_SEC) begin
			rd("after the ENABLE: SECTOR CAUSE clear, its request standing", SEC, 32'd0);
			check("the request stands", sd_rd[1] | cd_req, 1);
			sec_wr(S_CTRL, C_EN0); settle;
			rd("EN rises over the standing request: SECTOR CAUSE", SEC, 32'h1);
			sec_wr(S_CTRL, C_EN0 | C_ABORT);       // ABORT: nothing begun, so nothing to fill
			cause_drain(SEC); sec_wr(S_CTRL, 32'h0);
		end
		spi_get(16'h0036, r); spi_get(16'h003C, r);
		@(negedge clk_sys) info_rq = 0; up_rq = 0;
		sec_spi_clear; settle;
		if (TB_EVT) rd("served over SPI: PEND clear", SYSB + 21'h18, 32'd0);
		sys_drain;
		host_end;
	end
endtask

`ifdef MISTER_LW_SECTOR_SLOT
// A BEGIN stored just before an ID view, still queued when the link's level falls (clk_sys stopped across both):
// the half discards it unplayed and the request stands for SPI. The next host enables and stores EN before anyone
// serves it: the EN rise raises SECTOR's cause, REQ shows the request, and a frame over the bridge serves it.
integer n_dis = 0;
always @(posedge clk_sys)
	if (win.sec.sector.qv && win.sec.sector.fidle && !win.sec.sector.hen_s && win.sec.sector.qt == 2'd1)
		n_dis = n_dis + 1;
task ls18;
	reg [31:0] v;
	integer d0, n1, t;
	begin
		$display("-- enable links: a BEGIN discarded with the enable gone; the next host's EN rise finds its request");
		ena_up;
		sec_wr(S_CTRL, C_EN0); sec_stand(32'h00006E00); if (win.sec_lvl) cause_drain(SEC);
		check("bound before the BEGIN", blk_dn[18], 1);
		@(negedge clk_sys); sys_run = 0; repeat (20) @(negedge clk);
		d0 = n_dis;
		sec_wr(S_BEGIN0, cd_lba);
		host_end;
		repeat (200) @(negedge clk);
		check("the ID view with clk_sys stopped: EN cleared, the BEGIN still queued",
		      {win.sec.sector.en_lwx, win.sec.sector.rptr != win.sec.sector.wvis}, 2'b01);
		sys_run = 1;
		t = 0; while ((n_dis == d0 || blk_dn[18:17] != 0) && t < slow(5000)) begin @(negedge clk); t = t + 1; end
		check("the BEGIN discarded unplayed", n_dis - d0, 1);
		check("the slot back on SPI", blk_dn[18:17], 2'b00);
		settle;
		check("the request stands for SPI", sd_rd[1], 1);
		ena_w; wait_open;
		ena_rd(v);
		check("the next host holds the enable", v[31], 1);
		axi_write(LINEN, 32'hFFFFFFFF, 4'hF); sys_drain;
		rd("after the ENABLE: SECTOR CAUSE clear", SEC, 32'd0);
		sec_wr(S_CTRL, C_EN0); settle;
		rd("EN rises over the standing request: SECTOR CAUSE", SEC, 32'h1);
		check("its line rises", lines[TB_E_SEC - 1], 1);
		axi_read(SEC + S_REQ, 0, v, dl, beats);
		check("REQ shows the request: a read on the primary", {v[31], v[0]}, 2'b11);
		rd("LBA is the request's", SEC + S_LBA0, cd_lba);
		cause_drain(SEC);
		n1 = cd_frames;
		sec_wr(S_BEGIN0, cd_lba);
		frame_data(cd_lba, 0, TB_FRAME);
		wait_frame(n1);
		check_buf("the request served over the bridge, byte-exact", cd_lba, TB_FRAME);
		cause_drain(SEC); sec_wr(S_CTRL, 32'h0); settle;
		sec_spi_clear;
	end
endtask
`endif

// An ENABLE followed at once by work, without waiting for OPEN, after an ID view and after an HPS reset. Every
// lane frame plays whole or was dropped at the gate (stale set); none is popped unplayed.
// A SECTOR EN store lands or is dropped with stale. A host that waits for OPEN loses nothing.
task ls11;
	reg [31:0] v, en;
	integer k, j, c0, p0, b0, s0, q0;
	string  w;
	begin
		for (k = 0; k < 2; k = k + 1) begin
			w = (k == 0) ? "after an ID view" : "after an HPS reset";
			$display("-- enable links: an ENABLE followed at once by work, %s", w);
			ena_rd(v); if (!v[31]) ena_w;
			if (k == 0) host_end;
			else begin
				@(negedge clk); hps_rst_n = 0; repeat (3) @(negedge clk); hps_rst_n = 1;
				fw_wait_fall; repeat (2) @(negedge clk);
			end
			wait_quiet;
`ifdef MISTER_LW_LANE
			c0 = n_commit; p0 = n_play; b0 = bus_n;
`endif
			q0 = n_osdq;
			axi_write(ENA, ENA_W, 4'hF);
			check({w, ": the ENABLE is taken"}, win.en_q, 1);
			if (TB_OSD) axi_write(OSD + 4*(2*64 + k), 32'h11223344 ^ k, 4'hF);
`ifdef MISTER_LW_LANE
			f_set(0, 16'h001E, 2);
			for (j = 0; j < 4; j = j + 1) begin fw[0] = 16'h1100 + j; fw[1] = 16'h1180 + j; lane_send(1); end
`endif
			if (TB_SEC) sec_wr(S_CTRL, C_EN0);
			wait_open;
			ena_rd(v);
			if (TB_OSD) begin
				osd_drain;
				$display("     %s: the OSD store %s", w, n_osdq - q0 == 1 ? "queued" : "dropped");
				if (n_osdq - q0 == 1) ob_word(2*64 + k, 32'h11223344 ^ k);
				check({w, ": the OSD store is in both osd_buffers, or was dropped with stale"},
				      (n_osdq - q0 == 1 && bw_h(2*64 + k) == (32'h11223344 ^ k) && bw_v(2*64 + k) == (32'h11223344 ^ k)) ||
				      (n_osdq - q0 == 0 && v[29]), 1);
				osd_word(2*64 + 8 + k, 32'h55667788 ^ k, 4'hF); osd_drain;
				check({w, ": an OSD store after OPEN lands"},
				      {bw_h(2*64 + 8 + k) == (32'h55667788 ^ k), bw_v(2*64 + 8 + k) == (32'h55667788 ^ k)}, 2'b11);
			end
			if (TB_SEC) begin
				axi_read(SEC + S_CTRL, 0, en, dl, beats);
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
			if (TB_SEC) sec_wr(S_CTRL, 32'h0);
		end
		host_end;
	end
endtask

// A host dies with a link select asserted, lane frames queued behind it and SECTOR bound; a stock
// successor starts (GPO = 0: the selects drop and the ID view is selected), identifies the core in the ID view
// and sends link words. v: 0 the selects and bit 31 together; 1 bit 31 a clk_sys cycle first; 2 the selects first;
// 3 with clk_sys stopped across the GPO store.
task ls12_view(input integer v);
	reg [31:0] p, dr, lv;
	integer k, n1, pl0, oq;
	string  w;
	begin
		w = (v == 0) ? "the selects and bit 31 together" : (v == 1) ? "bit 31 a cycle first" :
		    (v == 2) ? "the selects a cycle first" : "clk_sys stopped across the GPO store";
		$display("-- enable links, the view: a dead host's select and queued frames; a stock successor (%s)", w);
`ifdef MISTER_LW_LANE
		f_set(0, 16'h001E, 2); fw[0] = 16'h3E01; fw[1] = 16'h3E02;
		trk_clear; link_send; trk_save;                    // the successor's own link frame, the reference
		f_set(0, 16'h001E, 2); fw[0] = 16'h0000; fw[1] = 16'h0000; link_send;
`endif
		ena_up;
		if (TB_SEC) sec_wr(S_CTRL, C_EN0);
		if (TB_OSD) begin osd_lo; ob_sync(w); end                        // 64 lines
		@(negedge clk_sys);
		io_osd = TB_OSD != 0;                               // the dead host's osd select
`ifdef MISTER_LW_LANE
		io_enable = 1;                                      // and its UIO select
		f_set(0, 16'h001E, 2);
		for (k = 0; k < 3; k = k + 1) begin fw[0] = 16'h3D00 + k; fw[1] = 16'h3D80 + k; lane_send(1); end
`endif
		// its repaint queued behind the osd select, rows 8-15 among it
		for (k = 0; k < 24; k = k + 1) if (TB_OSD) axi_write(OSD + 4*(k[0] ? 10*64 + k : 64 + k), ~pw(k, 40 + v, 0), 4'hF);
		repeat (slow(100)) @(negedge clk);
`ifdef MISTER_LW_LANE
		pl0 = n_play;
`endif
		oq = n_osdwe;
		if (v == 3) begin
			@(negedge clk_sys); sys_run = 0; repeat (20) @(negedge clk);
			gpo31 = 0; io_enable = 0; io_osd = 0; repeat (40) @(negedge clk); sys_run = 1;
		end
		else begin
			@(negedge clk_sys);
			gpo31 = 0;
			if (v == 2) begin io_enable = 0; io_osd = 0; end
			else begin repeat (v == 0 ? 1 : 2) @(negedge clk_sys); io_enable = 0; io_osd = 0; end
		end
		repeat (slow(2000)) @(negedge clk);                // the ID view: start-up and identification
		gpo31 = 1; repeat (2) @(negedge clk_sys);
`ifdef MISTER_LW_LANE
		trk_clear; f_set(0, 16'h001E, 2); fw[0] = 16'h3E01; fw[1] = 16'h3E02; link_send;
		repeat (slow(400)) @(negedge clk);
		trk_cmp({w, ": the successor's link words intact, no queued frame strobed"});
		check({w, ": no queued frame played"}, n_play - pl0, 0);
`endif
		ena_rd(lv);
		check({w, ": the view ended the enable"}, lv[31], 0);
		if (TB_OSD) begin
			check({w, ": no queued OSD byte written"}, n_osdwe - oq, 0);
			ob_cmp({w, ": both osd_buffers as they were"}, 0, 16);
			check({w, ": highres unchanged"}, {hdmi_osd.osd_h == 64, vga_osd.osd_h == 64}, 2'b11);
			osd_st(lv);
			check({w, ": nothing left waiting in the OSD FIFO"}, lv[24:16], 0);
		end
		if (TB_SEC) begin
			check({w, ": SECTOR's slot back on SPI"}, blk_dn[18], 0);
			sec_stand(32'h00006C00 + v);
			n1 = cd_frames; spi_frame(cd_lba); wait_frame(n1);
			check_buf({w, ": an SPI frame serves the slot"}, cd_lba, TB_FRAME);
		end
	end
endtask
task ls12;
	integer v;
	begin
		for (v = 0; v < 4; v = v + 1) ls12_view(v);
	end
endtask

// An enable that ends inside the rise. It is ended by GPO[31] low and by an HPS reset, k clk_100m cycles after
// the ENABLE's store took effect, k from before the store's own cycle to twenty cycles past the last OPEN bit.
// Then ENABLEs as fast as the bus allows; the one taken is followed at once by lane frames and a
// SECTOR EN and BEGIN. The link checker, the ENABLE monitor, and: every lane frame plays or was dropped with stale;
// the BEGIN is served or its request stands.
integer l15_rise = 0, l15_open = 0;
reg     l15_on = 0;
always @(negedge win.en_q) if (l15_on) begin
	if (lk_sec == 3'd2 || lk_osd == 3'd2 || lk_lane == 3'd2) l15_rise = l15_rise + 1;
	if (lk_sec == 3'd3 || lk_osd == 3'd3 || lk_lane == 3'd3) l15_open = l15_open + 1;
end
task ls15;
	reg [31:0] v;
	integer kind, k, past, tries, c0, p0, n1, s0, bad_p, bad_s, bad_o, nit, r0, o0, q0;
	string  w;
	begin
		for (kind = 1; kind < 3; kind = kind + 1) begin
			w = (kind == 1) ? "GPO[31] low" : "an HPS reset";
			$display("-- enable links: an enable ended inside its rise by %s, in every cycle to twenty past OPEN", w);
			past = 0; bad_p = 0; bad_s = 0; bad_o = 0; nit = 0; r0 = l15_rise; o0 = l15_open;
			for (k = -3; past < 20 && k < slow(400); k = k + 1) begin
				ena_rd(v); if (v[31]) host_end;
				wait_quiet;
				l15_on = 1;
				fork
					axi_write(ENA, ENA_W, 4'hF);
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
				if (win.en_q) host_end;
				// the next ENABLE, and work at once
`ifdef MISTER_LW_LANE
				c0 = n_commit; p0 = n_play;
`endif
				if (TB_SEC) sec_stand(32'h00007500 + k[7:0]);
				q0 = n_osdq;
				ena_hammer(slow(4000), tries);
`ifdef MISTER_LW_LANE
				f_set(0, 16'h001E, 2); fw[0] = 16'h5100 + k; fw[1] = 16'h5180 + k; lane_send(1); lane_send(1);
`endif
				if (TB_OSD) axi_write(OSD + 4*(4*64 + (k & 63)), 32'h15150000 + k, 4'hF);
				if (TB_SEC) begin
					n1 = cd_frames;
					sec_wr(S_CTRL, C_EN0);
					sec_wr(S_BEGIN0, cd_lba);
					sec_wr(S_CTRL, C_EN0 | C_ABORT);        // ABORT: the frame, if served, is filled
				end
				wait_open;
`ifdef MISTER_LW_LANE
				s0 = 0; while (n_play - p0 < n_commit - c0 && s0 < slow(4000)) begin @(negedge clk); s0 = s0 + 1; end
				ena_rd(v);
				if (n_play - p0 != n_commit - c0 || (n_commit - c0 != 2 && !v[29])) bad_p = bad_p + 1;
`endif
				if (TB_OSD) begin
					ena_rd(v);
					osd_drain;
					if (!((n_osdq - q0 == 1 && bw_h(4*64 + (k & 63)) == 32'h15150000 + k &&
					       bw_v(4*64 + (k & 63)) == 32'h15150000 + k) || (n_osdq - q0 == 0 && v[29]))) bad_o = bad_o + 1;
					if (n_osdq - q0 == 1) ob_word(4*64 + (k & 63), 32'h15150000 + k);
				end
				if (TB_SEC) begin
					s0 = 0; while (cd_frames == n1 && s0 < slow(4000)) begin @(negedge clk); s0 = s0 + 1; end
					sec_wr(S_CTRL, 32'h0);
					host_end;
					s0 = 0; while ((blk_dn[18] || blk_dn[17]) && s0 < slow(4000)) begin @(negedge clk); s0 = s0 + 1; end
					settle;
					if (cd_frames == n1 && !sd_rd[1]) bad_s = bad_s + 1;   // neither served nor standing for SPI
					sec_spi_clear;
				end
				else host_end;
				nit = nit + 1;
			end
			$display("     %0d ends, %0d with a link in RISE and %0d in OPEN at the end", nit, l15_rise - r0, l15_open - o0);
			check({w, ": ends inside the rise were reached"}, l15_rise - r0 > 0, 1);
			check({w, ": ends in OPEN, twenty"}, past >= 20, 1);
			check({w, ": every lane frame played or dropped with stale"}, bad_p, 0);
			check({w, ": every OSD store in both osd_buffers, or dropped with stale"}, bad_o, 0);
			check({w, ": every BEGIN served, or its request standing for SPI"}, bad_s, 0);
		end
		check("enable links: ENABLEs taken with a half off rest", n_offrest, 0);

		$display("-- enable links: clk_sys stopped from before an ENABLE to after its end: the link waits in RISE, OWNED set");
		ena_rd(v); if (v[31]) host_end;
		wait_quiet;
		sys_run = 0; repeat (20) @(negedge clk);
		axi_write(ENA, ENA_W, 4'hF);
		repeat (20) @(negedge clk);
		host_end;
		repeat (40) @(negedge clk);
		if (TB_SEC) check("stopped: SECTOR's link in RISE", lk_sec, 3'd2);
		if (TB_OSD) check("stopped: OSD's link in RISE", lk_osd, 3'd2);
		if (TB_LANE) check("stopped: the lane's link in RISE", lk_lane, 3'd2);
		rd("stopped: OWNED shows the linked entries", OWNEDA, TB_LINKED);
		ena_rd(v);
		check("stopped: not quiet", v[30], TB_LINKED == 0);
		ena_hammer(8, tries);
		check("stopped: no ENABLE taken", win.en_q, 0);
		sys_run = 1;
		wait_quiet;
		rd("restarted: the return completes, OWNED clear", OWNEDA, 32'd0);
		ena_w;
		check("restarted: an ENABLE is taken", win.en_q, 1);
		host_end;
	end
endtask

// One link's acknowledge forced high across an ENABLE (a half that never answers): that entry never opens,
// the others do and work, its requests are served over SPI, and with the force gone a late bind works. Forced low
// after an enable (a half that cannot return): quiet stays 0, OWNED names it alone, no ENABLE is taken, SPI works.
// SECTOR's link, then OSD's.
task ls17;
	reg [31:0] v, p, dr;
	integer tries, n1, q0;
	begin
`ifdef MISTER_LW_SECTOR_SLOT
		$display("-- enable links: a SECTOR half that never answers costs only SECTOR");
		ena_rd(v); if (v[31]) host_end;
		wait_quiet;
		fc_off = 1;
		force win.sec.sector.fence.ack_lwx = 1'b1;
		ena_w;
		repeat (slow(600)) @(negedge clk);
		rd("OPEN: every entry but SECTOR's", OPENA, ((32'd1 << em_n) - 1) & ~(32'd1 << TB_E_SEC));
		sec_wr(S_CTRL, C_EN0);
		rd("a store to SECTOR is dropped", SEC + S_CTRL, 32'd0);
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
		rd("the force removed under the same enable: OPEN every entry", OPENA, (32'd1 << em_n) - 1);
		sec_wr(S_CTRL, C_EN0);
		rd("a late bind works: EN", SEC + S_CTRL, 32'h1);
		sec_wr(S_CTRL, 32'h0);
		host_end;

		$display("-- enable links: a SECTOR half that cannot return holds the shared gate (recorded)");
		repeat (slow(200)) @(negedge clk);
		force win.sec.sector.fence.ack_lwx = 1'b0;
		repeat (slow(200)) @(negedge clk);
		ena_rd(v);
		check("not quiet", v[30], 0);
		rd("OWNED names SECTOR alone", OWNEDA, 32'd1 << TB_E_SEC);
		ena_hammer(20, tries);
		check("no ENABLE is taken", win.en_q, 0);
		sec_stand(32'h00007701);
		n1 = cd_frames; spi_frame(cd_lba); wait_frame(n1);
		check_buf("SPI works", cd_lba, TB_FRAME);
		release win.sec.sector.fence.ack_lwx;
		ena_w;
		check("the force removed: an ENABLE is taken", win.en_q, 1);
		fc_off = 0;
`ifdef MISTER_LW_LANE
		$display("-- enable links: a host that skips SECTOR waits on the lane's OPEN bit alone");
		tries = 0;
		axi_read(OPENA, 0, v, dl, beats);
		while (!v[TB_E_LANE] && tries < 100) begin tries = tries + 1; axi_read(OPENA, 0, v, dl, beats); end
		lane_count(p, dr);
		f_set(0, 16'h001E, 2); fw[0] = 16'h1703; fw[1] = 16'h1704; lane_send(1);
		lane_wait(p + 1);
`endif
		host_end;
`endif
`ifdef MISTER_LW_OSD
		$display("-- enable links: an OSD half that never answers costs only OSD");
		ob_sync("before the force");
		ena_rd(v); if (v[31]) host_end;
		wait_quiet;
		fc_off = 1;
		force win.osd.osd.fence.ack_lwx = 1'b1;
		ena_w;
		repeat (slow(600)) @(negedge clk);
		rd("OPEN: every entry but OSD's", OPENA, ((32'd1 << em_n) - 1) & ~(32'd1 << TB_E_OSD));
		q0 = n_osdq;
		axi_write(OSD + 4*(6*64), 32'h17171717, 4'hF);
		ena_rd(v);
		check("a store to OSD is dropped, with stale", {n_osdq - q0 == 0, v[29]}, 2'b11);
		if (TB_SEC) begin
			sec_wr(S_CTRL, C_EN0);
			rd("SECTOR works: its EN store lands", SEC + S_CTRL, 32'h1);
			sec_wr(S_CTRL, 32'h0);
		end
		spi_osd_row(6, 34);
		ob_cmp("the unanswered block's image served over SPI", 0, 16);
		release win.osd.osd.fence.ack_lwx;
		wait_open;
		rd("the force removed under the same enable: OPEN every entry", OPENA, (32'd1 << em_n) - 1);
		osd_word(6*64, 32'h17171717, 4'hF); osd_drain;
		check("a late store lands", {bw_h(6*64) == 32'h17171717, bw_v(6*64) == 32'h17171717}, 2'b11);
		host_end;

		$display("-- enable links: an OSD half that cannot return holds the shared gate (recorded)");
		repeat (slow(200)) @(negedge clk);
		force win.osd.osd.fence.ack_lwx = 1'b0;
		repeat (slow(200)) @(negedge clk);
		ena_rd(v);
		check("not quiet", v[30], 0);
		rd("OWNED names OSD alone", OWNEDA, 32'd1 << TB_E_OSD);
		ena_hammer(20, tries);
		check("no ENABLE is taken", win.en_q, 0);
		spi_osd_row(6, 35);
		ob_cmp("SPI works", 0, 16);
		release win.osd.osd.fence.ack_lwx;
		ena_w;
		check("the force removed: an ENABLE is taken", win.en_q, 1);
		fc_off = 0;
		host_end;
`endif
	end
endtask

// Random values in every clk_sys register of the lane, SECTOR, PAD, SYS and OSD halves, and in each enable
// link's clk_sys registers. Pass 1 mid-enable, then an ID view and ENABLEs as fast as the bus allows: none taken
// before every half is at rest (the ENABLE monitor), a stock SPI transfer before the ENABLE, and a frame of each
// block correct under the new enable. Pass 2 during the return: the links converge and a later enable works.
task scramble;
	begin
		@(negedge clk_sys);
		ap_scr = 1;
`ifdef MISTER_LW_PAD
		win.pad.pad.pad_x.lwtg_a = $random; win.pad.pad.pad_x.d_v = $random;
		win.pad.pad.pad_x.cap.lwq_d = {$random, $random};
		win.pad.pad.pad_x.sync_r.lws0_q = $random; win.pad.pad.pad_x.sync_r.lws1_q = $random;
`endif
`ifdef MISTER_LW_EVT
		win.evt.evt.flg_d = $random; win.evt.evt.sdirty = $random; win.evt.evt.epend = $random;
		win.evt.evt.lvl_lwx = $random;
		win.evt.evt.st_x.lwsrc_d = {5{$random}}; win.evt.evt.st_x.lwtg_r = $random;
		win.evt.evt.st_x.sync_a.lws0_q = $random; win.evt.evt.st_x.sync_a.lws1_q = $random;
		win.evt.evt.ev_x.lwsrc_d = $random; win.evt.evt.ev_x.lwtg_r = $random;
		win.evt.evt.ev_x.sync_a.lws0_q = $random; win.evt.evt.ev_x.sync_a.lws1_q = $random;
`endif
`ifdef MISTER_LW_OSD
		win.osd.osd.rptr = $random; win.osd.osd.q = {$random, $random}; win.osd.osd.fresh = $random;
		win.osd.osd.left = $random; win.osd.osd.hd = $random; win.osd.osd.qn = $random;
		win.osd.osd.osd_r_lwgray = $random;
		win.osd.osd.wg_sync.lws0_q = $random; win.osd.osd.wg_sync.lws1_q = $random;
		win.osd.osd.fence.rest_n = $random; win.osd.osd.fence.ack_lwx = $random;
		win.osd.osd.fence.hen_sync.lws0_q = $random; win.osd.osd.fence.hen_sync.lws1_q = $random;
`endif
`ifdef MISTER_LW_SECTOR_SLOT
		win.sec.sector.q = {$random, $random}; win.sec.sector.rptr = $random; win.sec.sector.sector_r_lwgray = $random;
		win.sec.sector.fs = $random; win.sec.sector.fexp = $random; win.sec.sector.em = $random;
		win.sec.sector.qv = $random; win.sec.sector.tmode = $random; win.sec.sector.sn = $random;
		win.sec.sector.ackc = $random; win.sec.sector.wv = $random; win.sec.sector.fsum = $random;
		win.sec.sector.gc = win.sec.sector.wv[31:28];   // one draw for both: the later blocks' draws stay as they were
		win.sec.sector.evt = $random; win.sec.sector.dcnt = $random; win.sec.sector.dsent = $random;
		win.sec.sector.dsum = $random; win.sec.sector.fsel = $random; win.sec.sector.fwack = $random;
		win.sec.sector.offd = $random; win.sec.sector.opn = $random; win.sec.sector.rej = $random;
		win.sec.sector.rtag = $random; win.sec.sector.dirty = $random; win.sec.sector.rearm = $random;
		win.sec.sector.rlast = $random; win.sec.sector.rec_x.lwsrc_d = {$random, $random};
		win.sec.sector.rec_x.lwtg_r = $random;
		win.sec.sector.rec_x.sync_a.lws0_q = $random; win.sec.sector.rec_x.sync_a.lws1_q = $random;
		win.sec.sector.fack = $random; win.sec.sector.mt = $random; win.sec.sector.ml = $random;
		win.sec.sector.en_sys = $random; win.sec.sector.x_en_sys = $random; win.sec.sector.o_wr = $random;
		win.sec.sector.o_dat = $random; win.sec.sector.o_adr = $random; win.sec.sector.sys_lwx = $random;
		win.sec.sector.ctl_sync.lws0_q = $random; win.sec.sector.ctl_sync.lws1_q = $random;
		win.sec.sector.wg_sync.lws0_q = $random; win.sec.sector.wg_sync.lws1_q = $random;
		win.sec.sector.fence.rest_n = $random; win.sec.sector.fence.ack_lwx = $random;
		win.sec.sector.fence.hen_sync.lws0_q = $random; win.sec.sector.fence.hen_sync.lws1_q = $random;
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
		win.lane.lane.fence.hen_sync.lws0_q = $random; win.lane.lane.fence.hen_sync.lws1_q = $random;
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
			$display("-- enable links: every clk_sys register scrambled mid-enable (seed %0d); then an ID view and ENABLEs", pass);
			ena_up;
			if (TB_SEC) sec_wr(S_CTRL, C_EN0);
			fc_off = 1;
			for (j = 0; j < 3; j = j + 1) begin scramble; repeat (slow(30)) @(negedge clk); end
			repeat (slow(200)) @(negedge clk);
			host_end;
			if (pass[0]) begin
				// ENABLEs as fast as the bus allows from the ID view on: the ENABLE monitor checks the one taken
				ena_hammer(slow(20000), tries);
				$display("     taken after %0d ENABLE stores", tries);
				check("after the scramble: an ENABLE is taken", win.en_q, 1);
				check("after the scramble: ENABLEs taken with a half off rest", n_offrest, 0);
				fc_off = 0;
			end
			else begin
				t = 0; while (TB_LINKED != 0 && t < slow(20000)) begin
					axi_read(OWNEDA, 0, v, dl, beats); if (v == 0) t = slow(20000); else t = t + 10;
				end
				check("after the scramble: the return completes, OWNED clear", v, 0);
				repeat (slow(100)) @(negedge clk);
				fc_off = 0;
				check("after the return: every SECTOR ack idle", {blk_dn[17], save_dn[12], save_dn[10]}, 3'd0);
`ifdef MISTER_LW_LANE
				check("after the return: the player idle", {lane_own, lane_hold}, 2'd0);
`endif
				if (TB_SEC) begin
					sec_stand(32'h00007800 + pass);
					n1 = cd_frames; spi_frame(cd_lba); wait_frame(n1);
					check_buf("a stock SPI transfer before the ENABLE", cd_lba, TB_FRAME);
				end
				ena_hammer(slow(4000), tries);
				check("an ENABLE is taken", win.en_q, 1);
			end
			wait_open; 			axi_write(LINEN, 32'hFFFFFFFF, 4'hF); sys_drain;
			axi_write(pad_addr(8'h02, 0), 32'h00C0FFEE ^ pass, 4'hF); settle;
			if (TB_PAD) check("a pad store lands under the new enable", joy0, 32'h00C0FFEE ^ pass);
			st_req(st_pat(970 + pass)); settle;
			if (TB_EVT) check("a status capture raises SYS's line", lines[1], 1);
			sys_drain;
`ifdef MISTER_LW_LANE
			f_set(0, 16'h001E, 2); fw[0] = 16'h8800 + pass; fw[1] = 16'h8880 + pass;
			lane_vs_link("after the scramble: a lane frame");
`endif
			if (TB_OSD) begin
				osd_drain;
				axi_write(OSD + 4*(11*64 + pass), 32'h88880000 + pass, 4'hF); osd_drain;
				check("after the scramble: an OSD store lands",
				      {bw_h(11*64 + pass) == 32'h88880000 + pass, bw_v(11*64 + pass) == 32'h88880000 + pass}, 2'b11);
			end
			if (TB_SEC) begin
				sec_wr(S_CTRL, C_EN0);
				sec_stand(32'h00007900 + pass); if (win.sec_lvl) cause_drain(SEC);
				n1 = cd_frames;
				sec_wr(S_BEGIN0, cd_lba);
				frame_data(cd_lba, 0, TB_FRAME);
				wait_frame(n1);
				check_buf("after the scramble: a SECTOR frame byte-exact", cd_lba, TB_FRAME);
				cause_drain(SEC); sec_wr(S_CTRL, 32'h0);
			end
		end
		$display("-- enable links: the clk_sys registers scrambled during the return: the links converge, an enable works");
		for (pass = 0; pass < 8; pass = pass + 1) begin
			ena_up;
`ifdef MISTER_LW_LANE
			@(negedge clk_sys); io_enable = 1;
			f_set(0, 16'h001E, 2); fw[0] = 16'h8900; fw[1] = 16'h8901; lane_send(1); lane_send(1);
`endif
			fc_off = 1;
			fork
				host_end;
				begin repeat (2 + 3 * pass) @(negedge clk); scramble; end
			join
`ifdef MISTER_LW_LANE
			@(negedge clk_sys); io_enable = 0;
`endif
			// (a scrambled lane half may play up to a FIFO of stray entries, 2048, before its pointer meets the write's)
			t = 0; while (t < slow(200000)) begin
				axi_read(ENA, 0, v, dl, beats); if (v[30]) t = slow(200000); else t = t + 10;
			end
			check($sformatf("scrambled in the return (%0d): quiet again", pass), v[30], 1);
			repeat (slow(50)) @(negedge clk);
			fc_off = 0;
			ena_up;
`ifdef MISTER_LW_LANE
			lane_count(p, dr);
			f_set(0, 16'h001E, 2); fw[0] = 16'h8A00 + pass; fw[1] = 16'h8A80 + pass; lane_send(1);
			lane_wait(p + 1);
`endif
			if (TB_SEC) begin
				sec_wr(S_CTRL, C_EN0); rd("a later enable: SECTOR's EN lands", SEC + S_CTRL, 32'h1); sec_wr(S_CTRL, 32'h0);
				sec_spi_clear;
			end
			if (TB_OSD) begin
				osd_drain;
				axi_write(OSD + 4*(12*64 + pass), 32'h8A8A0000 + pass, 4'hF); osd_drain;
				check($sformatf("a later enable (%0d): an OSD store lands", pass),
				      {bw_h(12*64 + pass) == 32'h8A8A0000 + pass, bw_v(12*64 + pass) == 32'h8A8A0000 + pass}, 2'b11);
			end
		end
`ifdef MISTER_LW_LANE
		vs_auto = 0;
		@(negedge clk_sys); tb_vsync = 0;
`endif
		ena_off;
	end
endtask

`ifdef MISTER_LW_SECTOR_SLOT
// ---- SECTOR's request records (T-SEC1, T-SEC3, T-SEC5, T-SEC7, T-SEC9) ----
// The CD's request dropped for one clk_sys cycle and raised again with LBA l; the CD reader keeps waiting for its ack.
task cd_blip(input [31:0] l);
	begin @(negedge clk_sys); cd_req = 0; @(negedge clk_sys); cd_req = 1; cd_lba = l; end
endtask
// Slot 1's buffer against the pattern of LBA l
function integer sv1_bad(input [31:0] l);
	integer m;
	begin
		sv1_bad = 0;
		for (m = 0; m < 256; m = m + 1) if ({sv1.ram[2*m+1], sv1.ram[2*m]} !== pat(l, m)) sv1_bad = sv1_bad + 1;
	end
endfunction
// Slot 1's request served over LW at LBA l: the answer, its frame, the buffer.
task sv1_serve(input string what, input [31:0] l);
	integer a0;
	begin
		a0 = sv1.acks;
		sec_wr(S_BEGIN1, l);
		frame_data(l, 0, 256);
		repeat (1500) @(negedge clk_sys);
		check({what, ": slot 1's frame done"}, sv1.acks - a0, 1);
		check({what, ": slot 1's buffer byte-exact"}, sv1_bad(l), 0);
	end
endtask

// T-SEC5: clk_sys edges from slot 1's ack fall to its request's next offer.
integer rof_n = 0, rof_gap = -1;
reg     rof_off = 0;
// T-SEC9: changes of the primary's offered in its record's launch cycle and in a handshake, and slot 1's reject
// set in its record's launch cycle. Sampled at the clk_sys edge they meet.
integer s9_ol = 0, s9_ob = 0, s9_rl = 0;
always @(posedge clk_sys) begin
	if (win.sec.sector.offd[1] && !rof_off) rof_gap = rof_n;
	rof_off = win.sec.sector.offd[1];
	rof_n = save_dn[10] ? 0 : rof_n + 1;
	if ((win.sec.sector.off_d[0] ^ win.sec.sector.offd[0]) && win.sec.sector.lsel[0]) s9_ol = s9_ol + 1;
	if ((win.sec.sector.off_d[0] ^ win.sec.sector.offd[0]) && win.sec.sector.rc_busy) s9_ob = s9_ob + 1;
	if (win.sec.sector.rj_set[1] && win.sec.sector.lsel[1]) s9_rl = s9_rl + 1;
end

task sec_rec_cases;
	reg [31:0] l, v;
	integer t, a0, o0, b0, r0;
	string  w;
	begin
		$display("-- T-SEC1: a request dropped for one clk_sys cycle and raised again before its BEGIN: seen again, served once");
		for (t = 0; t < 2; t = t + 1) begin
			w = t ? "a new LBA" : "the same LBA";
			l = 32'h00005100 + 16 * t;
			n0 = cd_frames;
			cd_request(l); settle; cause_drain(SEC);
			cd_blip(l + t); l = l + t; settle;
			rd({w, ": the re-open raises the cause"}, SEC, 32'h1);
			rd({w, ": REQ shows it"}, SEC + S_REQ, 32'h800080C5);
			rd({w, ": LBA is the live one"}, SEC + S_LBA0, l);
			cause_drain(SEC);
			sec_wr(S_BEGIN0, l);
			frame_data(l, 0, TB_FRAME);
			wait_frame(n0);
			check_buf({w, ": the frame byte-exact"}, l, TB_FRAME);
			repeat (slow(400)) @(negedge clk);
			check({w, ": served once"}, cd_frames - n0, 1);
			rd({w, ": REQ idle after it"}, SEC + S_REQ, 32'd0);
			check({w, ": no ring left"}, doorbell, 0);
		end

		$display("-- T-SEC1: the same after its BEGIN, queued behind a save-slot frame: that BEGIN serves the same LBA; a new one rejects it and re-opens");
		for (t = 0; t < 2; t = t + 1) begin
			w = t ? "a new LBA after the BEGIN" : "the same LBA after the BEGIN";
			l = 32'h00005300 + 16 * t;
			n0 = cd_frames; a0 = sv1.acks;
			sv1.go(0, 7'h51 + t); cd_request(l); settle; cause_drain(SEC);
			sys_run = 0;                                        // stored with clk_sys stopped, at any rate:
			sec_wr(S_BEGIN1, 32'h51 + t);
			frame_data(32'h51 + t, 0, 256);                     // slot 1's frame holds the feeder ...
			sec_wr(S_BEGIN0, l);                                // ... with the CD's answer queued behind it
			sys_run = 1;
			cd_blip(l + t); settle;
			rd({w, ": the re-raise is seen"}, SEC, 32'h1);
			cause_drain(SEC);
			repeat (1500) @(negedge clk_sys);
			check({w, ": slot 1's frame done"}, sv1.acks - a0, 1);
			check({w, ": slot 1's buffer byte-exact"}, sv1_bad(32'h51 + t), 0);
			if (!t) begin
				rd({w, ": the queued BEGIN serves it: no cause"}, SEC, 32'd0);
				rd({w, ": REQ idle while it is served"}, SEC + S_REQ, 32'd0);
			end
			else begin
				rd({w, ": the queued BEGIN is rejected and the request re-opens"}, SEC, 32'h1);
				rd({w, ": REQ shows it"}, SEC + S_REQ, 32'h800080C5);
				rd({w, ": LBA is the live one"}, SEC + S_LBA0, l + 1);
				l = l + 1;
				cause_drain(SEC);
				sec_wr(S_BEGIN0, l);
			end
			frame_data(l, 0, TB_FRAME);
			wait_frame(n0);
			check_buf({w, ": the frame byte-exact"}, l, TB_FRAME);
			repeat (slow(400)) @(negedge clk);
			check({w, ": served once"}, cd_frames - n0, 1);
			rd({w, ": REQ idle after it"}, SEC + S_REQ, 32'd0);
			check({w, ": no ring left"}, doorbell, 0);
		end

		$display("-- T-SEC3: a BEGIN met while a frame streams zero-fills that frame and re-opens its own request");
		n0 = cd_frames;
		cd_request(32'h00005500); sv1.go(0, 7'h55); settle; cause_drain(SEC);
		axi_read(SEC + S_COUNTS, 0, d0, dl, beats);
		sec_wr(S_BEGIN0, 32'h00005500);
		frame_data(32'h00005500, 0, 100);
		repeat (slow(3000)) @(negedge clk);                      // the 100 words played; the frame waits for more
		sec_wr(S_BEGIN1, 32'h00000055);                        // slot 1's answer, stored whole, met mid-stream
		frame_data(32'h00000055, 0, 256); settle;
		wait_frame(n0);
		check_buf("the streaming frame: 100 words, then zeros", 32'h00005500, 100);
		rd("slot 1's answer rejected: its request re-opens", SEC, 32'h2);
		rd("REQ shows slot 1", SEC + S_REQ, 32'h800180C9);
		axi_read(SEC + S_COUNTS, 0, d, dl, beats);
		check("COUNTS: one reject and one abort more", {d[31:24] - d0[31:24], d[15:8] - d0[15:8]}, dg(16'h0101));
		cause_drain(SEC);
		sv1_serve("after the re-open", 32'h00000055);
		rd("REQ idle", SEC + S_REQ, 32'd0);
		check("no ring left", doorbell, 0);

		$display("-- T-SEC3: eight stale answers back to back: the request re-opens, at the live LBA");
		sv1.go(0, 7'h56); settle; cause_drain(SEC);
		axi_read(SEC + S_COUNTS, 0, d0, dl, beats);
		sys_run = 0;
		for (t = 0; t < 8; t = t + 1) sec_wr(S_WACK1, 32'h00000060 + t);   // WACKs on a read, each whole
		sys_run = 1; settle;
		rd("re-opened: slot 1's cause", SEC, 32'h2);
		rd("REQ shows slot 1", SEC + S_REQ, 32'h800180C9);
		rd("LBA_S[1] the live LBA", SEC + S_LBA1, 32'h00000056);
		axi_read(SEC + S_COUNTS, 0, d, dl, beats);
		check("COUNTS: eight rejects more", d[31:24] - d0[31:24], dg(8));
		cause_drain(SEC);
		sv1_serve("after eight rejects", 32'h00000056);
		rd("REQ idle", SEC + S_REQ, 32'd0);
		check("no ring left", doorbell, 0);

		$display("-- T-SEC5: a request held through its frame is offered again two cycles after its ack falls");
		sv1.hold = 1;
		sv1.go(0, 7'h57); settle; cause_drain(SEC);
		rof_gap = -1;
		sv1_serve("held", 32'h00000057);
		check("the request still raised after its frame", sv1.rd, 1);
		check("offered again at the third clk_sys edge after the ack's fall", rof_gap, 3);
		rd("offered again: slot 1's cause", SEC, 32'h2);
		rd("REQ shows slot 1 again", SEC + S_REQ, 32'h800180C9);
		cause_drain(SEC);
		sv1.hold = 0;
		sv1_serve("served again, the request dropped on its ack", 32'h00000057);
		rd("REQ idle", SEC + S_REQ, 32'd0);
		check("no ring left", doorbell, 0);
		sv1.hold = 2;                                          // dropped in the cycle after the ack falls
		sv1.go(0, 7'h58); settle; cause_drain(SEC);
		rof_gap = -1;
		sv1_serve("dropped a cycle after its ack's fall", 32'h00000058);
		check("dropped a cycle after its ack's fall: not offered again", rof_gap, -1);
		rd("REQ idle", SEC + S_REQ, 32'd0);
		check("no ring left", doorbell, 0);
		sv1.hold = 0;

		$display("-- T-SEC7: CTRL.ABORT into a full FIFO is held, stores behind it are refused, and it aborts");
		n0 = cd_frames;
		cd_request(32'h00005900); settle; cause_drain(SEC);
		sec_wr(S_BEGIN0, 32'h00005900); settle;
		sys_run = 0;
		frame_data(32'h00005900, 0, 520);                       // 512 taken, the FIFO full
		sec_wr(S_CTRL, C_EN0 | C_EN1 | C_EN2 | C_ABORT);         // EN as before, and ABORT: held
		sec_data(512, pat(32'h00005900, 512));                   // refused behind it
		rd("ACCEPT: 512 taken, 9 refused (the ABORT is held, not dropped)", SEC + S_ACCEPT, 32'h00090200);
		sys_run = 1;
		wait_frame(n0);
		check_buf("the 512 words, then zeros: the held ABORT aborted the frame", 32'h00005900, 512);
		rd("REQ idle", SEC + S_REQ, 32'd0);

		$display("-- T-SEC7: a BEGIN_S refused behind a held ABORT opens no frame: ACCEPT shows it closed, its DATA is ignored, the slot's cause rises; stored again, it serves the request");
		n0 = cd_frames; a0 = sv1.acks;
		cd_request(32'h00005980); settle;
		sv1.go(0, 7'h5A); settle; cause_drain(SEC);
		sec_wr(S_BEGIN0, 32'h00005980); settle;
		sys_run = 0;
		frame_data(32'h00005980, 0, 520);                       // 512 taken, the FIFO full
		sec_wr(S_CTRL, C_EN0 | C_EN1 | C_EN2 | C_ABORT);         // held
		sec_wr(S_BEGIN1, 32'h0000005A);                          // refused: the ABORT is held
		rd("refused BEGIN_S: ACCEPT closed, one refused, nothing open", SEC + S_ACCEPT, 32'h40010000);
		rd("refused BEGIN_S: slot 1's cause rises again", SEC + S_CAUSE, 32'h00000002);
		rdm("refused BEGIN_S: RSTAT_S[1] offered, not answered", SEC + S_RSTAT1, 32'hC0000000, 32'h80000000);
		sv_store(32'h0000005A, 0, 8);
		rd("refused BEGIN_S: its DATA ignored, uncounted", SEC + S_ACCEPT, 32'h40010000);
		sys_run = 1;
		wait_frame(n0);
		check_buf("refused BEGIN_S: the primary aborted by the held ABORT", 32'h00005980, 512);
		check("refused BEGIN_S: nothing reached slot 1", sv1.acks - a0, 0);
		cause_drain(SEC);
		sv1_serve("refused BEGIN_S, stored again", 32'h0000005A);
		rd("refused BEGIN_S, stored again: ACCEPT ok", SEC + S_ACCEPT, 32'h80000100);
		rd("REQ idle", SEC + S_REQ, 32'd0);

		$display("-- T-SEC7: EN cleared mid-frame with DATA outstanding zero-fills the primary's frame");
		n0 = cd_frames;
		cd_request(32'h00005A00); settle; cause_drain(SEC);
		sec_wr(S_BEGIN0, 32'h00005A00);
		frame_data(32'h00005A00, 0, 50);
		repeat (slow(1500)) @(negedge clk);
		sec_wr(S_CTRL, C_EN1 | C_EN2);
		wait_frame(n0);
		check_buf("50 words, then zeros", 32'h00005A00, 50);
		t = 0; while (blk_dn[18:17] != 0 && t < slow(5000)) begin @(negedge clk); t = t + 1; end
		check("the primary back on SPI, its ack done", blk_dn[18:17], 2'b00);
		sec_wr(S_CTRL, C_EN0 | C_EN1 | C_EN2); settle;

		$display("-- T-SEC9: offered changes in every phase of its record's launch: REQ ends on the final state, every open rings");
		o0 = s9_ol; b0 = s9_ob;
		for (t = 0; t < 32; t = t + 1) begin
			l = 32'h00005C00 + 16 * t;
			cause_drain(SEC);
			cd_request(l);
			repeat (t % 16) @(negedge clk_sys);
			@(negedge clk_sys); cd_req = 0;
			if (t < 16) begin @(negedge clk_sys); cd_req = 1; cd_lba = l + 1; end   // raised again, a new LBA
			settle;
			if (t < 16) begin
				rd($sformatf("dropped at %0d and raised again: the cause", t), SEC, 32'h1);
				rd($sformatf("dropped at %0d and raised again: REQ shows it", t), SEC + S_REQ, 32'h800080C5);
				rd($sformatf("dropped at %0d and raised again: LBA the final one", t), SEC + S_LBA0, l + 1);
			end
			else begin
				rd($sformatf("dropped at %0d: REQ idle", t - 16), SEC + S_REQ, 32'd0);
				rd($sformatf("dropped at %0d: LBA the final one", t - 16), SEC + S_LBA0, l);
			end
			@(negedge clk_sys) cd_reset = 1; repeat (4) @(negedge clk_sys); cd_reset = 0; settle;
			rd($sformatf("pass %0d: REQ idle after the reset", t), SEC + S_REQ, 32'd0);
		end
		$display("     changes in the record's launch cycle %0d, during its handshake %0d", s9_ol - o0, s9_ob - b0);
		check("some changes met the launch cycle", s9_ol > o0, 1);
		check("some changes met a handshake", s9_ob > b0, 1);
		cause_drain(SEC);

		$display("-- T-SEC9: a reject set in every phase of its slot's record launch re-opens the request");
		r0 = s9_rl;
		sv1.go(0, 7'h5D); cd_request(32'h00005E00); settle; cause_drain(SEC);
		for (t = 0; t < 48; t = t + 1) begin
			@(negedge clk_sys); sys_run = 0;
			for (a0 = 0; a0 < 12; a0 = a0 + 1) sec_wr(S_CTRL, C_EN0 | C_EN1 | C_EN2 | C_ABORT); // twelve idle ABORTs ahead (EN unchanged),
			for (a0 = 0; a0 < 6; a0 = a0 + 1)                   // then six stale answers for slot 1 (WACKs) ...
				sec_wr(S_WACK1, 32'h00000100 + 8 * t + a0);
			repeat (20) @(negedge clk);
			sys_run = 1;
			repeat (t) @(negedge clk_sys);
			cd_blip(32'h00005E00);                              // ... and the CD's records in flight around them
			repeat (64) @(negedge clk_sys); settle;             // the 18 entries popped, every record landed
			rd($sformatf("offset %0d: slot 1 re-opened by its latest reject, the CD's re-raise seen", t), SEC, 32'h3);
			cause_drain(SEC);
		end
		$display("     rejects set in their slot's launch cycle %0d", s9_rl - r0);
		check("some rejects met their slot's launch cycle", s9_rl > r0, 1);
		sec_wr(S_CTRL, C_EN1); settle;                          // slot 1's EN alone: REQ shows it
		rd("slot 1's request not hidden", SEC + S_REQ, 32'h800180C9);
		sec_wr(S_CTRL, C_EN0 | C_EN1 | C_EN2); settle;
		cause_drain(SEC);
		sv1_serve("after the sweep", 32'h0000005D);
		@(negedge clk_sys) cd_reset = 1; repeat (4) @(negedge clk_sys); cd_reset = 0; settle;
		cause_drain(SEC);
		rd("REQ idle after the sweeps", SEC + S_REQ, 32'd0);
	end
endtask

// ---- Save-slot frames are all or nothing (T-SEC8): slot 1's read, F words, against every way it can be cut ----
// Slot 1's buffer against the block spi_poll sends from seed (512 halfwords)
function integer sv1_spi_bad(input [15:0] seed);
	integer m;
	begin
		sv1_spi_bad = 0;
		for (m = 0; m < 512; m = m + 1) if (sv1.ram[m] !== seed + m[15:0]) sv1_spi_bad = sv1_spi_bad + 1;
	end
endfunction
// Slot 1's DATA words [from, to) of LBA l, back to back (a save-slot frame is visible only once whole)
task sv_store(input [31:0] l, input integer from, input integer to);
	integer m;
	for (m = from; m < to; m = m + 1) sec_data(m, pat(l, m));
endtask
// Slot 1 back on SPI: its en_sys low and its request seen by hps_io
task sv1_spi_wait(output ok);
	integer t;
	begin
		t = 0; while (save_dn[9] && t < slow(5000)) begin @(negedge clk); t = t + 1; end
		settle;
		ok = !save_dn[9] && sd_rd[TB_S1H] === 1'b1;
	end
endtask
// The standing request served over SPI from seed: the GET_SDSTAT shows it, the transfer ends its handshake once and
// lands byte for byte
task sv1_spi_serve(input string what, input [15:0] seed);
	reg [15:0] w;
	integer d0;
	begin
		d0 = sv1.done;
		spi_poll(1, seed, w);
		repeat (20) @(negedge clk_sys);
		check({what, ": the GET_SDSTAT shows slot 1's read"}, w[5:0], (TB_S1H << 2) | 1);
		check({what, ": the SPI transfer ends the handshake once"}, sv1.done - d0, 1);
		check({what, ": the SPI transfer delivers the real block"}, sv1_spi_bad(seed), 0);
	end
endtask

task sec8_cases;
	reg [31:0] v, ex, er;
	reg  [6:0] l;
	reg        ok;
	integer c, n, k, a0, s0x, t, nrq, ncs, nreq, nspi, nacc, f0;
	string  cn;
	begin
		$display("-- T-SEC8: a save-slot read cut after every store count by ABORT, by a BEGIN or WACK and by an ID view: the core sees nothing and the request stands; stored in full, it plays whole");
		sv1.hold = 0;
		sec_wr(S_CTRL, C_EN0 | C_EN1 | C_EN2); settle; cause_drain(SEC);
		for (c = 0; c < 3; c = c + 1) begin
			cn = c == 0 ? "ABORT" : c == 1 ? "a BEGIN or WACK" : "an ID view";
			l = 7'h60 + c[6:0];
			for (k = 0; k < 512; k = k + 1) sv1.ram[k] = 16'hBEEF;
			sv1.go(0, l); settle; cause_drain(SEC);
			a0 = sv1.acks; s0x = sv1.strobes;
			nrq = 0; ncs = 0; nreq = 0; nspi = 0; nacc = 0; f0 = -1;
			for (n = 0; n <= TB_XFRAME0; n = n + 1) begin
				sec_wr(S_BEGIN1, l);
				sv_store(l, 0, n);
				ex = 32'h2; er = 32'h800180C9;               // after the cut: slot 1's cause, REQ shows it
				case (c)
					0: sec_wr(S_CTRL, C_EN0 | C_EN1 | C_EN2 | C_ABORT);
					1: case (n % 3)
						0: sec_wr(S_BEGIN0, 32'h00007777);   // the primary, with no request: rejected
						1: sec_wr(S_WACK2, 32'h00000077);    // slot 2, with no request: rejected
						default: begin sec_wr(S_BEGIN1, l); ex = 0; er = 0; end   // slot 1 answered again
					endcase
					default: begin
						host_end;
						if (n < TB_XFRAME0) begin sv1_spi_wait(ok); if (!ok) nspi = nspi + 1; end
					end
				endcase
				if (n < TB_XFRAME0) begin
					if (c == 0 && n == TB_XFRAME0 - 1) begin
						axi_read(SEC + S_ACCEPT, 0, v, dl, beats);
						if (v[31]) nacc = nacc + 1;
					end
					if (c == 2) begin ena_up; sec_wr(S_CTRL, C_EN0 | C_EN1 | C_EN2); end
					settle;
					if (sv1.rd !== 1'b1 || sv1.acks != a0 || sv1.strobes != s0x) begin
						nrq = nrq + 1; if (f0 < 0) f0 = n; end
					axi_read(SEC + S_CAUSE, 0, v, dl, beats);
					if (v !== ex) begin ncs = ncs + 1; if (f0 < 0) f0 = n; end
					cause_drain(SEC);
					axi_read(SEC + S_REQ, 0, v, dl, beats);
					if (v !== er) begin nreq = nreq + 1; if (f0 < 0) f0 = n; end
				end
			end
			if (f0 >= 0) $display("     %s: the first count that failed %0d", cn, f0);
			check({cn, ": a frame cut at a count below its length reached the core, or its request fell"}, nrq, 0);
			check({cn, ": the cut left a cause other than slot 1's re-open"}, ncs, 0);
			check({cn, ": REQ after the cut not slot 1's standing request"}, nreq, 0);
			if (c == 0) check({cn, ": ACCEPT's ok set after the ABORT withdrew the frame"}, nacc, 0);
			if (c == 2) check({cn, ": the slot not back on SPI with its request after the view"}, nspi, 0);
			// stored in full, then the same cut: the frame plays whole
			t = 0; while (sv1.acks == a0 && t < slow(8000)) begin @(negedge clk_sys); t = t + 1; end
			repeat (20) @(negedge clk_sys);
			check({cn, " after the last word: one ack"}, sv1.acks - a0, 1);
			check({cn, " after the last word: two strobes a word"}, sv1.strobes - s0x, 2 * TB_XFRAME0);
			check({cn, " after the last word: byte-exact"}, sv1_bad({25'd0, l}), 0);
			if (c == 2) begin ena_up; sec_wr(S_CTRL, C_EN0 | C_EN1 | C_EN2); end
			settle; cause_drain(SEC);
			rd({cn, ": REQ idle"}, SEC + S_REQ, 32'd0);
		end

		$display("-- T-SEC8: a host that falls silent mid-frame: the core waits untouched; the next program's ID view returns the slot to SPI, whose transfer delivers the real block");
		for (k = 0; k < 4; k = k + 1) begin
			n = (k == 0) ? 0 : (k == 1) ? 1 : (k == 2) ? TB_XFRAME0 / 2 : TB_XFRAME0 - 1;
			cn = $sformatf("silent after %0d words", n);
			l = 7'h68 + k[6:0];
			for (t = 0; t < 512; t = t + 1) sv1.ram[t] = 16'hBEEF;
			sv1.go(0, l); settle; cause_drain(SEC);
			a0 = sv1.acks; s0x = sv1.strobes;
			sec_wr(S_BEGIN1, l); sv_store(l, 0, n);
			repeat (slow(3000)) @(negedge clk);
			rdm({cn, ": the feeder idle, slot 1's ack low"}, SEC + S_STATUS, 32'h00000120, 32'h0);
			rd({cn, ": RSTAT_S[1] offered and answered"}, SEC + S_RSTAT1, 32'hC00080C9);
			check({cn, ": no ack, no strobe, the request raised"},
			      (sv1.acks - a0) + (sv1.strobes - s0x) + (sv1.rd !== 1'b1), 0);
			host_end;
			sv1_spi_wait(ok);
			check({cn, ": after the ID view slot 1 is on SPI with its request"}, ok, 1);
			sv1_spi_serve(cn, 16'h4000 + 16'h0100 * k[7:0]);
			ena_up; sec_wr(S_CTRL, C_EN0 | C_EN1 | C_EN2); settle; cause_drain(SEC);
		end

		$display("-- T-SEC8: a frame whose BEGIN is popped after its EN fell is rejected whole; the request stands for SPI");
		l = 7'h6C;
		for (t = 0; t < 512; t = t + 1) sv1.ram[t] = 16'hBEEF;
		sv1.go(0, l); settle; cause_drain(SEC);
		a0 = sv1.acks; s0x = sv1.strobes;
		sec_wr(S_CTRL, C_EN0 | C_EN1 | C_EN2 | C_CLR);          // the DIAG counts from 0 (the rejects above saturate)
		sec_wr(S_BEGIN1, l); sv_store(l, 0, TB_XFRAME0 - 1);
		sec_wr(S_CTRL, C_EN0 | C_EN2);                          // slot 1's EN cleared, its frame one word short
		sv1_spi_wait(ok);
		check("EN cleared: slot 1 on SPI before its frame is whole", ok, 1);
		sec_data(TB_XFRAME0 - 1, pat(l, TB_XFRAME0 - 1));      // the last word commits it
		rd("ACCEPT ok: every word stored", SEC + S_ACCEPT, 32'h80000000 | TB_XFRAME0);
		repeat (1500) @(negedge clk_sys);
		check("the committed frame popped with EN low: no ack, no strobe", (sv1.acks - a0) + (sv1.strobes - s0x), 0);
		axi_read(SEC + S_COUNTS, 0, v, dl, beats);
		check("COUNTS: one reject", v[31:24], dg(1));
		rd("no cause: the request is not offered", SEC + S_CAUSE, 32'd0);
		sv1_spi_serve("the frame rejected whole", 16'h4800);
		sec_wr(S_CTRL, C_EN0 | C_EN1 | C_EN2); settle; cause_drain(SEC);

		$display("-- T-SEC8: in order only: a later word and an earlier word's resend are not taken, and next_index tells");
		l = 7'h6D;
		sv1.go(0, l); settle; cause_drain(SEC);
		a0 = sv1.acks; s0x = sv1.strobes;
		sec_wr(S_BEGIN1, l); sv_store(l, 0, 10);
		sec_data(11, pat(l, 11));
		rd("a later word not taken: next_index 10", SEC + S_ACCEPT, 32'h0000000A);
		sec_data(TB_XFRAME0 - 1, pat(l, TB_XFRAME0 - 1));
		rd("the last word out of order commits nothing: next_index 10", SEC + S_ACCEPT, 32'h0000000A);
		sec_data(4, ~pat(l, 4));
		rd("an earlier word's resend not taken: next_index 10", SEC + S_ACCEPT, 32'h0000000A);
		sv_store(l, 10, TB_XFRAME0 - 1);
		rd("every word but the last: next_index, not ok", SEC + S_ACCEPT, TB_XFRAME0 - 1);
		repeat (800) @(negedge clk_sys);
		check("one word short: no ack, no strobe", (sv1.acks - a0) + (sv1.strobes - s0x), 0);
		sec_data(TB_XFRAME0 - 1, pat(l, TB_XFRAME0 - 1));
		rd("the last word: ok, no store refused", SEC + S_ACCEPT, 32'h80000000 | TB_XFRAME0);
		repeat (1500) @(negedge clk_sys);
		check("in order: one ack", sv1.acks - a0, 1);
		check("in order: byte-exact", sv1_bad({25'd0, l}), 0);
		rd("REQ idle after T-SEC8", SEC + S_REQ, 32'd0);
		check("no ring left after T-SEC8", doorbell, 0);
	end
endtask

// ---- One request, one path (T-SEC10): the EN store against the stock host's GET_SDSTAT and its transfer ----
// t10_mon: clk_sys edges are counted (t10_n). The window opens at the edge of a GET_SDSTAT answer that shows slot 1's
// request (t10_a) and closes after the edge at which slot 1's SPI ack falls (t10_e, the last edge inside it).
// t10_rise counts rises of slot 1's en_sys at an edge inside a window; t10_xl is the edge at which slot 1's EN
// first read 1 on clk_sys (xen_s), so en_sys could rise at t10_xl + 1.
wire    sp_ans = hps_io.io_enable && hps_io.io_strobe && hps_io.io_din == 16'h0016 && hps_io.byte_cnt == 0;
integer t10_n = 0, t10_ws = 0, t10_rise = 0, t10_a = -1, t10_e = -1, t10_xl = -1, t10_f = 0;
reg     t10_inp = 0, t10_xp = 0, t10_sp = 0;
always @(posedge clk_sys) begin : t10_mon
	reg inw;
	inw = 0;
	if (t10_ws == 0 && sp_ans && hps_io.sdn == 4'd2 && (sd_rd[2] | sd_wr[2]) === 1'b1) begin
		t10_ws = 1; t10_a = t10_n;
	end
	if (t10_ws == 1) begin inw = 1; if (sd_ack[2] === 1'b1) t10_ws = 2; end
	else if (t10_ws == 2) begin
		if (sd_ack[2] === 1'b1) inw = 1;
		else begin t10_ws = 0; t10_e = t10_n - 1; end
	end
	if (win.sec.sector.x_en_sys[0] === 1'b1 && !t10_xp && t10_inp) t10_rise = t10_rise + 1;
	if (win.sec.sector.xen_s[0] === 1'b1 && !t10_sp && t10_xl < 0) t10_xl = t10_n - 1;
	t10_xp  = win.sec.sector.x_en_sys[0] === 1'b1;
	t10_sp  = win.sec.sector.xen_s[0] === 1'b1;
	t10_inp = inw;
	t10_n   = t10_n + 1;
end

// The stock host's poll step: GET_SDSTAT (its status word w), then, for the request it shows, the block transfer:
// a read is SECTOR_RD with 512 words from seed, a write SECTOR_WR, whose words land in t10_buf. xfer 0: no transfer.
reg [15:0] t10_buf [0:511];
task spi_poll(input xfer, input [15:0] seed, output [15:0] w);
	integer m;
	begin
		@(negedge clk_sys); io_enable = 1;
		io_din = 16'h0016; io_strobe = 1; @(negedge clk_sys); io_strobe = 0; @(negedge clk_sys); w = hps_io.io_dout;
		for (m = 1; m < 4; m = m + 1) begin
			io_din = 16'd0; io_strobe = 1; @(negedge clk_sys); io_strobe = 0; @(negedge clk_sys);
		end
		io_enable = 0; @(negedge clk_sys);
		if (xfer && w[1:0] != 2'b00) begin
			@(negedge clk_sys); io_enable = 1;
			io_din = {4'd0, w[5:2], w[0] ? 8'h17 : 8'h18}; io_strobe = 1; @(negedge clk_sys); io_strobe = 0;
			repeat (2) @(negedge clk_sys);
			for (m = 0; m < 512; m = m + 1) begin
				io_din = seed + m; io_strobe = 1; @(negedge clk_sys); io_strobe = 0;
				t10_buf[m] = hps_io.io_dout; repeat (2) @(negedge clk_sys);
			end
			io_enable = 0; @(negedge clk_sys);
		end
	end
endtask

// Every en_sys low, or one of them still high after n clk_sys edges
task t10_low(input integer n);
	integer t;
	begin
		t = 0;
		while ((blk_dn[18] || save_dn[9] || save_dn[11]) && t < n) begin @(negedge clk_sys); t = t + 1; end
	end
endtask

// One step: slot 1 raises a read (wr 0) or write (wr 1) request at LBA l; the stock host polls from clk_sys edge
// lead, and slot 1's EN is stored sub ns after edge at. The request is served by the path that showed it, and then the
// next request (a write at l ^ 0x40) over LW; EN is cleared. Counts: t10_cnt[0] data words wrong, [1] transfers
// that did not complete slot 1's handshake once, [2] requests the answer did not show and LW did not offer or
// serve, [3] next requests LW did not offer or serve, [4] steps whose answer showed the request, [5] steps after
// which en_sys did not rise.
integer t10_cnt [0:5];
task t10_step(input wr, input [6:0] l, input [15:0] seed, input integer lead, input integer at, input real sub,
              output integer rel_a, output integer rel_e, output shown);
	reg [15:0] w;
	reg [31:0] v;
	integer m, d0, t;
	begin
		for (m = 0; m < 512; m = m + 1) sv1.ram[m] = seed ^ 16'hA5A5 ^ m;
		d0 = sv1.done;
		sv1.go(wr, l); settle;
		@(negedge clk_sys); t10_ws = 0; t10_a = -1; t10_e = -1; t10_xl = -1; t10_f = t10_n;
		fork
			begin repeat (lead) @(negedge clk_sys); spi_poll(1, seed, w); end
			begin repeat (at) @(negedge clk_sys); #(sub); sec_wr(S_CTRL, C_EN1); end
		join
		t = 0; while (t10_xl < 0 && t < 64) begin @(negedge clk_sys); t = t + 1; end
		rel_a = (t10_a < 0 || t10_xl < 0) ? -1000 : t10_xl - t10_a;
		rel_e = (t10_e < 0 || t10_xl < 0) ? -1000 : t10_xl - t10_e;
		shown = w[1:0] != 2'b00;
		repeat (20) @(negedge clk_sys);
		if (shown) begin
			t10_cnt[4] = t10_cnt[4] + 1;
			for (m = 0; m < 512; m = m + 1)
				if ((wr ? t10_buf[m] : sv1.ram[m]) !== (wr ? (seed ^ 16'hA5A5 ^ m) : seed + m)) t10_cnt[0] = t10_cnt[0] + 1;
			if (sv1.done - d0 != 1) t10_cnt[1] = t10_cnt[1] + 1;
		end
		else begin
			axi_read(SEC + S_REQ, 0, v, dl, beats);
			if (v !== (wr ? 32'h800180CA : 32'h800180C9)) t10_cnt[2] = t10_cnt[2] + 1;
			if (wr) sec_wr(S_WACK1, {25'd0, l});
			else begin sec_wr(S_BEGIN1, {25'd0, l}); frame_data({25'd0, l}, 0, 256); end
			repeat (1500) @(negedge clk_sys);
			if (sv1.done - d0 != 1 || (!wr && sv1_bad({25'd0, l}) != 0)) t10_cnt[2] = t10_cnt[2] + 1;
		end
		t = 0; while (!save_dn[9] && t < slow(400)) begin @(negedge clk_sys); t = t + 1; end
		if (!save_dn[9]) t10_cnt[5] = t10_cnt[5] + 1;
		d0 = sv1.done;
		sv1.go(1, l ^ 7'h40); settle;
		axi_read(SEC + S_REQ, 0, v, dl, beats);
		if (v !== 32'h800180CA) t10_cnt[3] = t10_cnt[3] + 1;
		sec_wr(S_WACK1, {25'd0, l ^ 7'h40});
		repeat (100) @(negedge clk_sys);
		if (sv1.done - d0 != 1) t10_cnt[3] = t10_cnt[3] + 1;
		sec_wr(S_CTRL, 32'h0);
		t10_low(slow(400));
		cause_drain(SEC);
	end
endtask

task sec10_cases;
	integer i, wr, lat, t, ra, re, dlen, aoff, n_hid, n_crit, n_held, n_end, n_aft, nb, nx;
	integer tg [0:63];
	integer ntg, a0, r0;
	reg [15:0] w;
	reg        sh;
	string  dn;
	begin
		$display("-- T-SEC10: the EN store in every cycle around a GET_SDSTAT and its transfer: the mux never moves inside");
		sv1.hold = 0;
		sec_wr(S_CTRL, 32'h0); t10_low(slow(400)); cause_drain(SEC);
		// the EN store's latency, from the clk_sys edge it is issued at to slot 1's EN on clk_sys
		@(negedge clk_sys); lat = 0;
		fork
			sec_wr(S_CTRL, C_EN1);
			while (!win.sec.sector.xen_s[0]) begin @(negedge clk_sys); lat = lat + 1; end
		join
		sec_wr(S_CTRL, 32'h0); t10_low(slow(400));
		r0 = t10_rise;
		for (i = 0; i < 6; i = i + 1) t10_cnt[i] = 0;
		// a first step with the store long after the transfer measures the answer's edge from the poll's start
		// (aoff, the link front's delay included) and the window's length (dlen)
		t10_step(1, 7'h11, 16'h1000, lat + 48, lat + 48 + 3000, 0.0, ra, re, sh);
		aoff = t10_a - t10_f - (lat + 48);
		dlen = t10_e - t10_a;
		$display("     EN store latency %0d clk_sys edges; the answer %0d edges after the poll starts; the window, answer to ack fall, %0d edges",
		         lat, aoff, dlen);
		// the landing edges, relative to the answer edge: two before it, every edge from -4 to +12, a stride of
		// 96 across the transfer, every edge from 8 before its end to 8 after, and -3 to +1 twice more
		ntg = 0;
		tg[ntg] = -40; ntg = ntg + 1; tg[ntg] = -12; ntg = ntg + 1;
		for (t = -4; t <= 12; t = t + 1) begin tg[ntg] = t; ntg = ntg + 1; end
		for (t = 108; t < dlen - 8 && ntg < 40; t = t + 96) begin tg[ntg] = t; ntg = ntg + 1; end
		for (t = dlen - 8; t <= dlen + 8 && ntg < 64; t = t + 1) begin tg[ntg] = t; ntg = ntg + 1; end
		// the edges around the answer once more at two other phases of the poll's start
		for (t = -3; t <= 1 && ntg < 64; t = t + 1) begin tg[ntg] = t; ntg = ntg + 1; end
		for (t = -3; t <= 1 && ntg < 64; t = t + 1) begin tg[ntg] = t; ntg = ntg + 1; end
		for (wr = 0; wr < 2; wr = wr + 1) begin
			dn = wr ? "a write and SECTOR_WR" : "a read and SECTOR_RD";
			n_hid = 0; n_crit = 0; n_held = 0; n_end = 0; n_aft = 0;
			for (i = 0; i < 6; i = i + 1) t10_cnt[i] = 0;
			for (i = 0; i < ntg; i = i + 1) begin
				// the answer edge is aoff edges after the poll starts at a0; the store lands lat edges after it is issued
				a0 = lat + 48 + i % 3;
				t10_step(wr, 7'h20 + i[6:0], 16'h2000 + 16'h0100 * i[7:0], a0, a0 + aoff + tg[i] - lat, 1.25 * (i % 4),
				         ra, re, sh);
				if (!sh) n_hid = n_hid + 1;
				else if (ra == -1) n_crit = n_crit + 1;
				else if (re < 0) n_held = n_held + 1;
				else if (re == 0) n_end = n_end + 1;
				else n_aft = n_aft + 1;
			end
			// the store's landing jitters with the clocks' phase: until one has landed one edge before the answer,
			// up to 24 more steps around it, each at another sub-cycle offset
			nx = 0;
			while (n_crit == 0 && nx < 24) begin
				a0 = lat + 48 + nx % 3;
				t10_step(wr, 7'h60 + nx[6:0], 16'h6000 + 16'h0100 * nx[7:0], a0, a0 + aoff - 2 + nx % 3 - lat,
				         0.625 * (nx % 8), ra, re, sh);
				if (sh && ra == -1) n_crit = n_crit + 1;
				nx = nx + 1;
			end
			$display("     %s: %0d steps; EN read 1 before the answer %0d, one edge before it %0d, inside the window %0d, at its last edge %0d, after it %0d",
			         dn, ntg + nx, n_hid, n_crit, n_held, n_end, n_aft);
			check({dn, ": en_sys rises inside the window"}, t10_rise - r0, 0);
			check({dn, ": SPI transfers byte for byte (words wrong)"}, t10_cnt[0], 0);
			check({dn, ": SPI transfers that did not end slot 1's handshake once"}, t10_cnt[1], 0);
			check({dn, ": requests the answer did not show, not served over LW"}, t10_cnt[2], 0);
			check({dn, ": the next request not offered and served over LW"}, t10_cnt[3], 0);
			check({dn, ": en_sys did not rise after the transfer"}, t10_cnt[5], 0);
			check({dn, ": some answers showed the request"}, t10_cnt[4] > 0, 1);
			check({dn, ": some stores landed before the answer (the request hidden)"}, n_hid > 0, 1);
			check({dn, ": some stores landed one edge before the answer (held by the answer cycle)"}, n_crit > 0, 1);
			check({dn, ": some stores landed inside the window"}, n_held > 0, 1);
			check({dn, ": some stores landed after the window"}, n_aft > 0, 1);
			r0 = t10_rise;
		end

		$display("-- T-SEC10: a mark left by a GET_SDSTAT with no transfer holds every en_sys low until one shows none");
		n0 = sv1.done;
		sv1.go(0, 7'h33); settle;
		spi_poll(0, 16'h0, w);
		check("the GET_SDSTAT shows slot 1's read", w[5:0], {4'd2, 2'b01});
		check("the mark reads 1", hps_bus[380], TB_SEC);
		sec_wr(S_CTRL, C_EN0 | C_EN1 | C_EN2);
		nb = 0;
		repeat (slow(2000)) begin @(negedge clk_sys); if (blk_dn[18] || save_dn[9] || save_dn[11]) nb = nb + 1; end
		check("EN for every slot stored: no en_sys rose", nb, 0);
		rdm("STATUS: the commit mark, no en_sys", SEC + S_STATUS, 32'h00000707, 32'h00000400);
		spi_poll(0, 16'h0, w);
		check("the request still shows over SPI", w[5:0], {4'd2, 2'b01});
		repeat (slow(200)) begin @(negedge clk_sys); if (blk_dn[18] || save_dn[9] || save_dn[11]) nb = nb + 1; end
		check("after it: no en_sys rose", nb, 0);
		@(negedge clk_sys); sv1.rd = 0; sv1.busy = 0;            // the core drops its request
		settle;
		check("still none with the request gone", {blk_dn[18], save_dn[9], save_dn[11]}, 3'b000);
		spi_poll(0, 16'h0, w);
		check("a GET_SDSTAT shows no request", w[1:0], 2'b00);
		repeat (20) @(negedge clk_sys);
		check("the mark cleared", hps_bus[380], 0);
		check("every wired slot's en_sys rose", {blk_dn[18], save_dn[9], save_dn[11]}, {1'b1, 1'b1, TB_W1 != 0});
		rdm("STATUS: en_sys by slot index, the mark clear", SEC + S_STATUS, 32'h00000707, TB_W1 ? 32'h7 : 32'h3);
		check("slot 1 never acked", sv1.done - n0, 0);
		settle;
		cause_drain(SEC);
		rd("REQ idle after T-SEC10", SEC + S_REQ, 32'd0);
	end
endtask
`endif

// SECTOR's sweep: a primary frame stored in full, the enable ended by an ID view at each offset from the last store's
// response to after the frame's last strobe (every cycle for the first 24, then a stride): byte-exact every time.
task sec7_sweep;
	reg [31:0] lba;
	integer off, n1, n_bef, n_aft, stride;
	begin
		$display("-- SECTOR: a frame stored in full, the enable ended at each offset to its last strobe: byte-exact");
		n_bef = 0; n_aft = 0; stride = 1;
		for (off = 0; n_aft < 2 && off < slow(20000); off = off + stride) begin
			ena_up;
			sec_wr(S_CTRL, C_EN0);
			sec_stand(32'h00007C00 + off); lba = cd_lba; if (win.sec_lvl) cause_drain(SEC);
			n1 = cd_frames;
			sec_wr(S_BEGIN0, lba);
			frame_data(lba, 0, TB_FRAME);
			repeat (off) @(negedge clk);
			host_end;
			if (cd_frames == n1) n_bef = n_bef + 1; else n_aft = n_aft + 1;
			wait_frame(n1);
			check_buf($sformatf("ended %0d cycles after the last store: byte-exact", off), lba, TB_FRAME);
			sec_spi_clear;
			if (off >= 23) stride = slow(500);
		end
		$display("     %0d ends landed before the frame's last strobe, %0d after", n_bef, n_aft);
		check("ends landed after the frame's last strobe", n_aft > 0, 1);
		// from 100 MHz a frame can drain before the earliest ID view takes effect (it lands a few bridge cycles
		// after the last store's response, through GPO[31]'s synchroniser): recorded, not required
		if (`LW_TB_SYS_HP_PS > 5000) check("ends landed while the frame still played", n_bef > 0, 1);
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

// The enable's every-cycle checks, at the end of an arm: GPI[22]
// was en_q, the lines were their definition, and each of a line's three reasons to fall was seen.
task ena_summary(input cov_on);
	begin
		$display("-- enable: GPI[22] and the lines' definition over the whole run");
		$display("     %0d cycles GPI[22] compared (%0d enabled), %0d cycles the lines compared", gpi22_n, gpi22_on, lm_n);
		$display("     line falls: %0d by its cause, %0d by LINE_EN, %0d by the enable's end", lm_fall_c, lm_fall_e, lm_fall_l);
		check("cycles in which GPI[22] differs from en_q", gpi22_bad, 0);
		check("cycles in which a line differs from its definition", lm_bad, 0);
		$display("     enable links: %0d ENABLEs taken, %0d with a half off rest", n_claims, n_offrest);
		check("ENABLEs taken with a linked half off rest (the bench's own predicate, now and four clk_sys cycles before)",
		      n_offrest, 0);
`ifdef MISTER_LW_SECTOR_SLOT
		$display("     SECTOR's link: %0d rises, %0d falls, %0d store strobes", fc_sec.n_rise, fc_sec.n_fall, fc_sec.n_stb);
		check("SECTOR's link checker", fc_sec.n_bad, 0);
`endif
`ifdef MISTER_LW_OSD
		$display("     OSD's link: %0d rises, %0d falls, %0d store strobes", fc_osd.n_rise, fc_osd.n_fall, fc_osd.n_stb);
		check("OSD's link checker", fc_osd.n_bad, 0);
`endif
`ifdef MISTER_LW_LANE
		$display("     the lane's link: %0d rises, %0d falls, %0d store strobes", fc_lane.n_rise, fc_lane.n_fall,
		         fc_lane.n_stb);
		check("the lane's link checker", fc_lane.n_bad, 0);
`endif
		if (cov_on && (TB_SEC || TB_EVT)) check("a line fell by each of its three reasons", {lm_fall_c > 0, lm_fall_e > 0, lm_fall_l > 0}, 3'b111);
	end
endtask

`ifdef MENU_CORE
initial begin
	repeat (4) @(negedge clk);
	sys_rate;
	cdc_cells;
	dir_cases;
	ena_up;
	$display("-- MENU_CORE: the OSD lists 20 rows; the header rows 16-19 stored over LW land, and draw as over SPI");
	rd("OSD K0: 20 rows of 256 bytes", DIRB + 21'h40 + 21'h80 * TB_E_OSD + 21'h20, 32'h00140100);
	n0 = n_osdq;
	axi_write(OSD + 21'h1400, 32'h12345678, 4'hF);         // past row 19
	check("a store past row 19 takes no entry", n_osdq - n0, 0);
	for (r = 0; r < 20; r = r + 1) spi_osd_row(r, 3);
	spi_osd_cmd(8'h40);                        // clears the highres rows 8-15 re-armed: 64 + 24 header lines fit
	spi_osd_cmd(8'h41);
	ob_cmp("the SPI image, header rows included", 0, 20);
	cap_snap(4);
	$display("     the box: %0d lines", cs_lh);
	check("the box holds the header lines", cs_lh > 64 && cs_lv > 64, 1);
	for (r = 0; r < 20; r = r + 1) lw_row(r, 4);
	osd_drain;
	ob_cmp("another image over LW, header rows included", 0, 20);
	spi_osd_cmd(8'h40); spi_osd_cmd(8'h41);
	cap_diff(3, n0);
	check("another image over LW draws otherwise", n0 > 0, 1);
	for (r = 0; r < 16; r = r + 1) lw_row_f(r, 3, 1);
	osd_drain;
	spi_osd_cmd(8'h40); spi_osd_cmd(8'h41);
	cap_diff(3, n0);
	check("rows 0-15 as over SPI, the header rows still another image's: drawn otherwise", n0 > 0, 1);
	for (r = 16; r < 20; r = r + 1) lw_row_f(r, 3, 1);
	osd_drain;
	ob_cmp("the SPI image's bytes over LW", 0, 20);
	cap_diff(3, n0);
	check("drawn exactly as over SPI, header lines included", n0, 0);
	check("no LW byte written in a cycle with, or two after, an osd select or the ID view", osd_clash, 0);
	ena_summary(0);
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
	ena_up;
	$display("-- default geometry: a %0d-word primary, 256-word save slots", TB_FRAME);
	rd("SECTOR K4: the primary, RD, hps_io slot 1, 128 words", 21'h40070 + 21'h80 * TB_E_SEC, 32'h01010080);
	rd("SECTOR K8: slot 1, RD and WR, hps_io slot 2, 256 words", 21'h40080 + 21'h80 * TB_E_SEC, 32'h03020100);
	rd("SECTOR K12: slot 2, RD and WR, hps_io slot 3, 256 words", 21'h40090 + 21'h80 * TB_E_SEC,
	   TB_W1 ? 32'h03030100 : 32'h00030100);
	sec_wr(S_CTRL, C_EN0 | C_EN1 | C_EN2); settle;

	$display("-- a 128-word primary frame");
	n0 = cd_frames; s0 = cd_strobes;
	cd_request(32'h00000100); settle;
	cause_drain(SEC);
	sec_wr(S_BEGIN0, 32'h00000100);
	frame_data(32'h00000100, 0, 128);
	rd("ACCEPT ok, 128", SEC + S_ACCEPT, 32'h80000080);
	wait_frame(n0);
	check_buf("primary frame 0x100", 32'h00000100, 128);
	repeat (200) @(negedge clk_sys);
	check("256 strobes", cd_strobes - s0, 256);

	$display("-- a 256-word save-slot frame, longer than the primary's");
	r = sv1.done; s0 = sv1.strobes;
	sv1.go(0, 7'h12); settle;
	cause_drain(SEC);
	sec_wr(S_BEGIN1, 32'h00000012);
	frame_data(32'h00000012, 0, 256);
	rd("ACCEPT ok, 256: stores past the primary's length land", SEC + S_ACCEPT, 32'h80000100);
	repeat (1500) @(negedge clk_sys);
	check("slot 1 frame done", sv1.done - r, 1);
	check("slot 1 took 512 strobes", sv1.strobes - s0, 512);
	bad = 0; for (i = 0; i < 256; i = i + 1) if ({sv1.ram[2*i+1], sv1.ram[2*i]} !== pat(32'h12, i)) bad = bad + 1;
	check("slot 1 buffer byte-exact (1 KB)", bad, 0);
	axi_read(SEC + S_STATUS, 0, d, dl, beats);
	check("STATUS idle after the save-slot frame, busy and every ack low", {d[8], d[6:4]}, 4'd0);

	$display("-- the primary after the save slot: the shared feeder is free");
	n0 = cd_frames;
	cd_request(32'h00000101); settle;
	cause_drain(SEC);
	sec_wr(S_BEGIN0, 32'h00000101);
	frame_data(32'h00000101, 0, 128);
	rd("ACCEPT ok, 128 after the save slot", SEC + S_ACCEPT, 32'h80000080);
	wait_frame(n0);
	check_buf("primary frame 0x101 after the save slot", 32'h00000101, 128);
	rd("REQ idle", SEC + S_REQ, 32'd0);
	check("no ring left", doorbell, 0);
	ena_summary(0);

	$display("%s: %0d checks (%0d functional + %0d fixed-latency, one per transaction), %0d failures",
	         fails ? "FAILED" : "PASSED", checks, checks - lchecks, lchecks, fails);
	$finish;
end
`elsif LW_TB_GEOMW
// Arm W (T-SEC6): hps_io at WIDE = 0, so the feeder makes byte strobes; a 4096-word primary on hps_io slot 9, save
// slots on 12 and 15 with 32-bit LBAs, strobes 5 clk_sys cycles apart. Each frame lands byte for byte at its byte
// addresses, against hps_io's own SPI transfer of the same block for the primary; the directory reports it all.
reg [7:0] w_img [0:16383];

// The block's bytes over SPI: UIO_SECTOR_RD on hps_io slot `slot`, one byte per strobe, byte 0 of word 0 first, a
// strobe every 3 clk_sys cycles (hps_io steps its buffer address 3 cycles after a strobe).
task spi_bytes(input [3:0] slot, input [31:0] lba, input integer n);
	integer m;
	reg [31:0] w;
	begin
		@(negedge clk_sys); io_enable = 1;
		io_din = {4'd0, slot, 8'h17}; io_strobe = 1; @(negedge clk_sys); io_strobe = 0; @(negedge clk_sys);
		for (m = 0; m < n; m = m + 1) begin
			w = pat(lba, m >> 2);
			io_din = {8'd0, w[8 * (m % 4) +: 8]}; io_strobe = 1; @(negedge clk_sys); io_strobe = 0;
			repeat (2) @(negedge clk_sys);
		end
		io_enable = 0; @(negedge clk_sys);
	end
endtask

// A frame's DATA in chunks the FIFO holds, each given the time its strobes take, so nothing is refused.
task w_frame(input [31:0] lba, input integer n);
	integer c;
	for (c = 0; c < n; c = c + 256) begin
		frame_data(lba, c, (c + 256 < n) ? c + 256 : n);
		repeat (256 * 4 * TB_GAP) @(negedge clk_sys);
	end
endtask

initial begin : arm_w
	integer t, nb, bb;
	reg [31:0] v;
	repeat (4) @(negedge clk);
	sys_rate;
	cdc_cells;
	dir_cases;
	ena_up;
	$display({"-- geometry W: hps_io WIDE = 0, a %0d-word primary on hps_io slot %0d, save slots of %0d and %0d words ",
	          "on %0d and %0d, %0d-bit LBAs, gap %0d"}, TB_FRAME, TB_PS, TB_XFRAME0, TB_XFRAME1, TB_S1H, TB_S2H,
	         TB_LBAB, TB_GAP);
	check("hps_io's io_wide on HPS_BUS[32]: 0", hps_bus[32], 0);
	rd("SECTOR K0: 512 FIFO entries, gap 5", 21'h40060 + 21'h80 * TB_E_SEC, 32'h02000005);
	rd("SECTOR K1: BLKSZ 3, wide 0 (byte strobes)", 21'h40064 + 21'h80 * TB_E_SEC, 32'h00000300);
	rd("SECTOR K4: the primary, RD, hps_io slot 9, 4096 words", 21'h40070 + 21'h80 * TB_E_SEC, 32'h01091000);
	rd("SECTOR K5: the primary's LBA, 32 bits", 21'h40074 + 21'h80 * TB_E_SEC, 32'd32);
	rd("SECTOR K8: slot 1, RD and WR, hps_io slot 12, 510 words", 21'h40080 + 21'h80 * TB_E_SEC, 32'h030C01FE);
	rd("SECTOR K9: slot 1's LBA, 32 bits", 21'h40084 + 21'h80 * TB_E_SEC, 32'd32);
	rd("SECTOR K12: slot 2, RD and WR, hps_io slot 15, 256 words", 21'h40090 + 21'h80 * TB_E_SEC, 32'h030F0100);
	rd("SECTOR K13: slot 2's LBA, 32 bits", 21'h40094 + 21'h80 * TB_E_SEC, 32'd32);

	$display("-- W: a 4096-word block over SPI, byte strobes from hps_io (EN clear: the slot is SPI's)");
	n0 = pm.done; s0 = pm.strobes;
	for (i = 0; i < 16384; i = i + 1) pm.ram[i] = 16'hEEEE;
	pm.go(0, 32'h89ABCDEF); settle;
	check("the primary's request reaches hps_io", sd_rd[TB_PS], 1);
	spi_bytes(4'd9, 32'h89ABCDEF, 16384);
	repeat (50) @(negedge clk_sys);
	check("SPI: the block done", pm.done - n0, 1);
	check("SPI: 16384 byte strobes", pm.strobes - s0, 16384);
	bad = 0;
	for (i = 0; i < 16384; i = i + 1) begin
		v = pat(32'h89ABCDEF, i >> 2);
		w_img[i] = pm.ram[i][7:0];
		if (pm.ram[i][7:0] !== v[8 * (i % 4) +: 8]) bad = bad + 1;
	end
	check("SPI: the block at its byte addresses", bad, 0);

	$display("-- W: the same block over LW: 16384 byte strobes, byte 0 first, at the byte index, 5 cycles apart");
	sec_wr(S_CTRL, C_EN0 | C_EN1 | C_EN2); settle;
	for (i = 0; i < 16384; i = i + 1) pm.ram[i] = 16'hEEEE;
	pm.amin = 1000; pm.alead = 1000;
	n0 = pm.done; s0 = pm.strobes;
	pm.go(0, 32'h89ABCDEF); settle;
	check("the primary hidden from hps_io", sd_rd[TB_PS], 0);
	cause_drain(SEC);
	rd("REQ: the primary, hps_io slot 9", SEC + S_REQ, 32'h800080E5);
	rd("LBA_S[0]: the 32-bit LBA", SEC + S_LBA0, 32'h89ABCDEF);
	sec_wr(S_BEGIN0, 32'h89ABCDEF);
	w_frame(32'h89ABCDEF, 4096);
	rd("ACCEPT ok, 4096, no store refused", SEC + S_ACCEPT, 32'h80001000);
	t = 0; while (pm.done == n0 && t < 200000) begin @(negedge clk_sys); t = t + 1; end
	check("LW: the block done", pm.done - n0, 1);
	check("LW: 16384 byte strobes", pm.strobes - s0, 16384);
	bad = 0; for (i = 0; i < 16384; i = i + 1) if (pm.ram[i] !== {8'd0, w_img[i]}) bad = bad + 1;
	check("LW: byte for byte the SPI transfer's block, at the same addresses", bad, 0);
	check("LW: strobes 5 cycles apart", pm.amin, 5);
	check("LW: the ack leads the first strobe by 5", pm.alead, 5);
	rd("REQ idle", SEC + S_REQ, 32'd0);

	$display("-- W: slot 1, 510 words with a 32-bit LBA: the LBA round-trips, and all 32 bits are compared");
	sv1.amin = 1000; sv1.alead = 1000;
	n0 = sv1.done; s0 = sv1.strobes;
	sv1.go(0, 32'hFEDCBA98); settle;
	rd("CAUSE: slot 1's bit", SEC + S_CAUSE, 32'h00000002);
	cause_drain(SEC);
	rd("RSTAT_S[1]: offered, a read on hps_io slot 12", SEC + S_RSTAT1, 32'h800080F1);
	rd("LBA_S[1]: all 32 bits", SEC + S_LBA1, 32'hFEDCBA98);
	sec_wr(S_BEGIN1, 32'h7EDCBA98);
	frame_data(32'h7EDCBA98, 0, 510);
	repeat (1200) @(negedge clk_sys);                        // its DATA popped behind it
	settle; settle;
	rd("a BEGIN_S whose LBA differs in bit 31: rejected, the request re-opened", SEC + S_CAUSE, 32'h00000002);
	cause_drain(SEC);
	rd("RSTAT_S[1]: offered, not answered", SEC + S_RSTAT1, 32'h800080F1);
	check("no strobe for the rejected answer", sv1.strobes - s0, 0);
	sec_wr(S_BEGIN1, 32'hFEDCBA98);
	frame_data(32'hFEDCBA98, 0, 510);
	rd("ACCEPT ok, 510", SEC + S_ACCEPT, 32'h800001FE);
	t = 0; while (sv1.done == n0 && t < 50000) begin @(negedge clk_sys); t = t + 1; end
	check("slot 1: the frame done", sv1.done - n0, 1);
	check("slot 1: 2040 byte strobes", sv1.strobes - s0, 2040);
	bad = 0;
	for (i = 0; i < 2040; i = i + 1) begin
		v = pat(32'hFEDCBA98, i >> 2);
		if (sv1.ram[i] !== {8'd0, v[8 * (i % 4) +: 8]}) bad = bad + 1;
	end
	check("slot 1: byte-exact at its byte addresses", bad, 0);
	check("slot 1: strobes 5 cycles apart", sv1.amin, 5);
	check("slot 1: the ack leads the first strobe by 5", sv1.alead, 5);

	$display("-- W: slot 2's write answered by a WACK on all 32 bits of the LBA");
	n0 = sv2.acks; s0 = sv2.strobes;
	sv2.go(1, 32'h80000001); settle;
	cause_drain(SEC);
	rd("LBA_S[2]: all 32 bits", SEC + S_LBA2, 32'h80000001);
	rd("RSTAT_S[2]: offered, a write on hps_io slot 15", SEC + S_RSTAT2, 32'h800080FE);
	sec_wr(S_WACK2, 32'h00000001);
	settle; settle;
	rd("a WACK_S that differs in bit 31: rejected, the request re-opened", SEC + S_CAUSE, 32'h00000004);
	cause_drain(SEC);
	check("no ack for it", sv2.acks - n0, 0);
	sec_wr(S_WACK2, 32'h80000001);
	t = 0; while (sv2.acks == n0 && t < 2000) begin @(negedge clk_sys); t = t + 1; end
	check("slot 2: one ack, no strobe", {sv2.acks - n0, sv2.strobes - s0}, {32'd1, 32'd0});
	rd("REQ idle", SEC + S_REQ, 32'd0);
	check("no ring left", doorbell, 0);
	check("requests acknowledged by both paths", ap_both, 0);
	ena_summary(0);

	$display("%s: %0d checks (%0d functional + %0d fixed-latency, one per transaction), %0d failures",
	         fails ? "FAILED" : "PASSED", checks, checks - lchecks, lchecks, fails);
	$finish;
end
`elsif LW_TB_TIER
// Arm T (each of the 32 tier sets) and arm B (a core side that leaves pieces out): what is listed is what is built
// and wired; the directory, the decode and the lines follow it, and the linked blocks' OWNED and OPEN bits are
// theirs alone.
task tier_cases;
	reg [31:0] v, pm;
	begin
		$display("-- tiers: listed = built and present (PAD %0d of %0d, EVT %0d of %0d, SECTOR %0d of %0d, OSD %0d, LANE %0d)",
		         TB_PAD, TB_PADB, TB_EVT, TB_EVTB, TB_SEC, TB_SECB, TB_OSD, TB_LANE);
		check("PAD listed", win.has_pad, TB_PAD);
		check("EVT listed", win.has_evt, TB_EVT);
		check("SECTOR listed", win.has_sec, TB_SEC);
		check("hps_io's commit mark present", win.pres_cmt, TB_CMT);
		rd("COUNT", DIRB + 21'h0C, TB_COUNT);
		pm = (32'd1 << TB_COUNT) - 32'd1;
		if (TB_SEC) begin
			$display("-- tiers: a save slot's EN bit is set only while its mux is wired");
			sec_wr(S_CTRL, C_EN0 | C_EN1 | C_EN2);
			rd("SECTOR CTRL: EN for the primary and both save slots stored", SEC + S_CTRL, TB_W1 ? 32'h7 : 32'h3);
			sec_wr(S_CTRL, 32'h0);
		end
		$display("-- tiers: each linked block's return shows in its OWNED bit alone, its opening in its OPEN bit alone");
		ena_take;
		rd("enabled: OPEN every listed entry", OPENA, pm);
		rd("enabled: OWNED none", OWNEDA, 32'd0);
		sys_run = 0; repeat (20) @(negedge clk);       // the far halves cannot answer the fall
		host_end;
		rd("clk_sys stopped across the ID view: OWNED the linked entries alone", OWNEDA, TB_LINKED);
		rd("clk_sys stopped across the ID view: OPEN none", OPENA, 32'd0);
		sys_run = 1;
		wait_quiet;
		rd("clk_sys running: every link returned, OWNED none", OWNEDA, 32'd0);
		sys_run = 0; repeat (20) @(negedge clk);       // the far halves cannot answer the rise
		axi_write(ENA, ENA_W, 4'hF);
		ena_rd(v);
		check("clk_sys stopped: the ENABLE is taken", v[31], 1);
		rd("clk_sys stopped: OPEN the unlinked entries alone", OPENA, pm & ~TB_LINKED);
		sys_run = 1;
		wait_open;
		rd("clk_sys running: OPEN every listed entry", OPENA, pm);
		host_end;
	end
endtask

initial begin
	repeat (4) @(negedge clk);
	dir_cases;
	ena_up;
	decode_sweep;
	dir_causes;
	tier_cases;
	ena_summary(0);
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
	ena_up;

	$display("-- scratch");
	axi_write(SCR, 32'hA5A55A5A, 4'hF);
	rd("scratch", SCR, dg(32'hA5A55A5A));
	axi_write(SCR, 32'h00C30000, 4'h4);
	rd("scratch byte lane", SCR, dg(32'hA5C35A5A));

`ifdef MISTER_LW_PAD
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
	rd("PAD ID", PADB + 21'h4, 32'h31444150);
	axi_read(PADB, 0, d, dl, beats);
	check("PAD STAT[31:16] reads 0", d[31:16], 0);
	begin : pad_keys
	integer n;
	reg v;
	n = n_padwe;
	axi_write(pad_addr(8'h1A, 8'h10), 32'h00005A5A, 4'hF);   // a stick index with a high nibble
	axi_write(pad_addr(8'h3D, 8'h06), 32'h00005A5A, 4'hF);   // stick 6
	axi_write(pad_addr(8'h1A, 8'h6F), 32'h00005A5A, 4'hF);   // paddle 6
	axi_write(pad_addr(8'h1A, 8'hEF), 32'h00005A5A, 4'hF);   // spinner 14 - 8
	axi_write(pad_addr(8'h1A, 8'h1E), 32'h00005A5A, 4'hF);   // index {1, 14}
	settle;
	check("stores at no key are ignored: nothing crosses", n_padwe - n, 0);
	check("stores at no key: left analog_0 unchanged", {16'd0, ana0}, 32'h0000817F);
	v = spn[8];
	axi_write(pad_addr(8'h1A, 8'h8F), 32'h00000077, 4'hF); settle;
	check("spinner_0 (idx {8, 15}): data", {24'd0, spn[7:0]}, 32'h00000077);
	check("spinner_0: one store toggles bit 8", spn[8] ^ v, 1);
	axi_write(pad_addr(8'h1A, 8'hDF), 32'h00000099, 4'hF); settle;
	check("spinner_5 (idx {13, 15}): data", {24'd0, spn[52:45]}, 32'h00000099);
	end

`endif
	$display("-- default responder");
	rd("LW 0x000000", 21'h000000, 32'hDEADBEEF);
	rd("below window", 21'h03FFFC, 32'hDEADBEEF);
	rd("directory page past the entries", DIRB + 21'h40 + 21'h80 * em_n, 32'd0);
	rd("directory page end, below SECTOR", 21'h040FFC, 32'd0);
	rd("above SECTOR: SYS CAUSE idle", 21'h043000, TB_EVT ? 32'd0 : 32'hDEADBEEF);
	rd("SYS block end", 21'h043FFC, TB_EVT ? 32'd0 : 32'hDEADBEEF);
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
		nw0 = n_lwwr; nb0 = n_lwwb; no0 = n_osdq; nbeats = 0;
		for (b = 0; b < 13; b = b + 1) begin
			case (b)
				0:  a = SEC + S_CTRL;                  // SECTOR CTRL
				1:  a = SEC + S_BEGIN0;                // BEGIN_S[0]
				2:  a = SEC + S_BEGIN1;                // BEGIN_S[1]
				3:  a = SEC + S_CAUSE;                 // SECTOR CAUSE
				4:  a = SEC + S_DATA;                  // DATA[0]
				5:  a = pad_addr(8'h02, 0);            // PAD
				6:  a = pad_addr(8'h1A, 0);            // PAD, analog
				7:  a = OSD;                           // IMAGE
				8:  a = OSD + 21'h1800;                // OSD STATUS
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
		check("burst writes: no beat reached the OSD FIFO", n_osdq - no0, 0);
		check("burst writes: every beat reported dropped", n_lwwb - nb0, nbeats);
		pure_view(1, nbad);
		check("burst writes: the readable view is unchanged", nbad, 0);
		check("burst writes: the window's, PAD's and SYS's state is unchanged", pure_win() === sw0, 1);
		check("burst writes: SECTOR's state is unchanged", pure_sec() === ss0, 1);
		check("burst writes: the lane's state is unchanged", pure_lane() === sl0, 1);
		sys_run = 1;
		settle;
		// a single beat (AWLEN 0) is a store whatever its burst type
		axi_read(FW_CTRL, 0, oc, dl, beats);
		nw0 = n_lwwr; nb0 = n_lwwb;
		for (t = 0; t < 4; t = t + 1) begin
			awburst = t[1:0];
			axi_write(FW_CTRL, {30'd0, ~t[1:0]}, 4'hF);
			rd($sformatf("single store, burst type %0d: LINE_EN", t), FW_CTRL, {30'd0, ~t[1:0]} & FW_MASK);
			axi_write(SCR, 32'h5C000000 | t, 4'hF);
			rd($sformatf("single store, burst type %0d: SCRATCH", t), SCR, dg(32'h5C000000 | t));
		end
		awburst = 2'b01;
		check("single stores: one bus pulse each", n_lwwr - nw0, 8);
		check("single stores: none dropped", n_lwwb - nb0, 0);
		axi_write(FW_CTRL, oc, 4'hF);
		rd("LINE_EN restored", FW_CTRL, oc);
	end

	$display("-- clk_sys stopped (core held in reset)");
	sys_run = 0;
	rd("magic, clk_sys stopped", 21'h40000, FW_MAGIC);
	axi_write(SCR, 32'h0BADF00D, 4'hF);
	rd("scratch, clk_sys stopped", SCR, dg(32'h0BADF00D));
`ifdef MISTER_LW_PAD
	for (i = 1; i <= 5; i = i + 1) axi_write(pad_addr(8'h02, 0), i, 4'hF);
	axi_write(pad_addr(8'h03, 0), 32'h00000077, 4'hF);
	axi_write(pad_addr(8'h03, 0), 32'h00000000, 4'hF);
	for (i = 1; i <= 10; i = i + 1) axi_write(pad_addr(8'h1A, 0), i, 4'hF);
	for (i = 11; i <= 20; i = i + 1) axi_write(pad_addr(8'h3D, 0), i, 4'hF);
`endif
	rd("undecoded, clk_sys stopped", 21'h1FFFFC, 32'hDEADBEEF);
`ifdef MISTER_LW_PAD
	check("joystick_0 unchanged while stopped", joy0, 32'h0000A0A0);
	check("left analog_0 unchanged while stopped", {16'd0, ana0}, 32'h0000817F);
`endif
	sys_run = 1; repeat (slow(300)) @(negedge clk);
`ifdef MISTER_LW_PAD
	check("joystick_0 last of 5 lands on restart", joy0, 32'h00000005);
	check("joystick_1 release (last) lands on restart", joy1, 32'h00000000);
	check("left analog_0: the last of 10 lands on restart", {16'd0, ana0}, 32'h0000000A);
	check("right analog_0: the last of 10 lands on restart", {16'd0, rana0}, 32'h00000014);
	pad_hold_cases;
`endif

`ifndef MISTER_LW_SECTOR_SLOT
	$display("-- no SECTOR block: its addresses read 0xDEADBEEF, stores are ignored, no doorbell");
	rd("SECTOR CAUSE reads the default", SEC + S_CAUSE, 32'hDEADBEEF);
	rd("SECTOR ID reads the default", SEC + S_ID, 32'hDEADBEEF);
	axi_write(SEC + S_CTRL, 32'h1, 4'hF);
	axi_write(SEC + S_BEGIN0, 32'h0, 4'hF);
	rd("SECTOR CTRL still the default", SEC + S_CTRL, 32'hDEADBEEF);
	rd("SECTOR RSTAT_S[0] reads the default", SEC + S_RSTAT0, 32'hDEADBEEF);
	rd("SECTOR DATA reads the default", SEC + S_DATA, 32'hDEADBEEF);
	rd("SECTOR end reads the default", SEC + 21'h7FFC, 32'hDEADBEEF);
	check("no doorbell", doorbell, 0);
	check("blk_dn idle", blk_dn, 0);
	check("save_dn idle", save_dn, 0);
`else

	$display("-- SECTOR block: registers");
	rd("ID SEC3", SEC + S_ID, 32'h33434553);
	rd("CTRL reset", SEC + S_CTRL, 32'd0);
	rd("STATUS idle", SEC + S_STATUS, 32'd0);
	rd("REQ idle", SEC + S_REQ, 32'd0);
	rd("CAUSE idle", SEC + S_CAUSE, 32'd0);
	rd("BEGIN_S[0] reads 0", SEC + S_BEGIN0, 32'd0);
	// (rd and wr are the last record's, from earlier cases)
	rdm("RSTAT_S[0]: no request, the slot's GET_SDSTAT word", SEC + S_RSTAT0, 32'hFFFFFFFC, 32'h000080C4);
	rdm("RSTAT_S[1]: no request", SEC + S_RSTAT1, 32'hFFFFFFFC, 32'h000080C8);
	rdm("RSTAT_S[2]: no request", SEC + S_RSTAT2, 32'hFFFFFFFC, 32'h000080CC);
	rd("WACK_S[1] reads 0", SEC + S_WACK1, 32'd0);
	rd("register hole after SUM", SEC + 21'h034, 32'd0);
	rd("register hole before the slot words", SEC + 21'h0FC, 32'd0);
	rd("slot 0 has no WACK_S", SEC + 21'h10C, 32'd0);
	rd("no slot index 3", SEC + 21'h160, 32'd0);
	rd("register page end", SEC + 21'h1FC, 32'd0);
	rd("block hole", SEC + 21'h3FFC, 32'd0);
	rd("DATA[0] reads 0", SEC + S_DATA, 32'd0);
	rd("DATA[588] reads 0", SEC + S_DATA + 4*588, 32'd0);
	rd("block end", SEC + 21'h7FFC, 32'd0);
	check("doorbell low", doorbell, 0);

	$display("-- T-SEC4: a request standing before EN raises no cause; EN's rise raises it, once");
	axi_read(SEC + S_RINGS, 0, rings0, dl, beats);
	cd_request(32'h00001234);
	settle;
	check("no cause without EN", doorbell, 0);
	rd("CAUSE clear without EN", SEC + S_CAUSE, 32'd0);
	check("SPI slot 1 offered while EN=0", sd_rd[1], 1);
	rd("REQ hides a request whose EN is 0", SEC + S_REQ, 32'd0);
	rd("no record without EN: LBA_S[0] 0", SEC + S_LBA0, 32'd0);
	rdm("no record without EN: RSTAT_S[0] not offered", SEC + S_RSTAT0, 32'hFFFFFFFC, 32'h000080C4);
	rd("RINGS unmoved without EN", SEC + S_RINGS, dg(rings0));
	sec_wr(S_CTRL, C_EN0); t0_en = $realtime; settle;
	check("EN's rise raises the cause", doorbell, 1);
	check("the raise came after the EN store", t_fire > t0_en, 1);
	rd("LBA_S[0] from the record", SEC + S_LBA0, 32'h00001234);
	rd("RSTAT_S[0] from the record: offered, a read, not answered", SEC + S_RSTAT0, 32'h800080C5);
	rd("RINGS +1", SEC + S_RINGS, dg(rings0 + 1));
	rsize = 0; rd("CAUSE byte read: data", SEC + S_CAUSE, 32'h00000001); rsize = 2;
	check("byte read does not clear", doorbell, 1);
	rd("CAUSE word read: bit 0", SEC + S_CAUSE, 32'h00000001);
	check("word read does not clear", doorbell, 1);
	arburst = 2'b00; axi_read(SEC + S_CAUSE, 4'd15, d, dl, beats); arburst = 2'b01;
	check("a 16-beat FIXED burst on CAUSE: every beat the cause", {d, dl}, {32'h00000001, 32'h00000001});
	check("burst read does not clear", doorbell, 1);
	axi_write(SEC + S_CAUSE, 32'hFFFFFFFE, 4'hF);
	check("a store with bit 0 clear clears nothing", doorbell, 1);
	axi_write(SEC + S_CAUSE, 32'h00000001, 4'h7);
	check("a partial-width store clears nothing", doorbell, 1);
	rd("CAUSE kept", SEC + S_CAUSE, 32'h00000001);
	axi_write(SEC + S_CAUSE, 32'h00000001, 4'hF);
	check("a store of bit 0 clears the line", doorbell, 0);
	rd("CAUSE zero iff idle", SEC + S_CAUSE, 32'd0);
	sec_wr(S_CTRL, C_EN0); settle;
	check("EN stored again over the same request: no cause", doorbell, 0);
	rd("RINGS still +1", SEC + S_RINGS, dg(rings0 + 1));
	rd("REQ is a level (pending once EN)", SEC + S_REQ, 32'h800080C5);
	rd("LBA_S[0] still the request's", SEC + S_LBA0, 32'h00001234);
	rd("STATUS en_sys", SEC + S_STATUS, 32'h00000001);
	check("SPI slot 1 hidden while en_sys", sd_rd[1], 0);

	$display("-- full 588-store frame");
	n0 = cd_frames; s0 = cd_strobes;
	sec_wr(S_BEGIN0, 32'h00001234);
	frame_data(32'h00001234, 0, 588);
	rd("ACCEPT ok, 588", SEC + S_ACCEPT, 32'h8000024C);
	wait_frame(n0);
	check_buf("frame 0x1234", 32'h00001234, 588);
	repeat (200) @(negedge clk_sys);
	check("1176 strobes", cd_strobes - s0, 1176);
	check("the CD reader saw ack then dropped req", cd_req, 0);
	rd("REQ cleared by ack", SEC + S_REQ, 32'd0);
	axi_read(SEC + S_STATUS, 0, d, dl, beats);
	check("STATUS: 1 frame, idle, en_sys", {d[31:16], d[10:0]}, {DIAG ? 16'd1 : 16'd0, 11'h001});
	if (!DIAG) check("STATUS[31:16] lean", d[31:16], 0);
	axi_read(SEC + S_DROPS, 0, d, dl, beats);
	$display("     FIFO high-water %0d entries", d[23:16]);
	rd("SUM", SEC + S_SUM, dg(frame_sum(32'h00001234, 588)));
	axi_read(SEC + S_TSREQ, 0, d, dl, beats);
	axi_read(SEC + S_TSDONE, 0, dl, d0, beats);
	if (DIAG) check("TSTAMP_DONE after TSTAMP_REQ", dl > d, 1);
	else check("TSTAMP_REQ/DONE lean", {d, dl}, 0);
	if (!DIAG) begin
		rd("lean CYCLES reads 0", SEC + S_CYCLES, 32'd0);
		rd("lean RINGS reads 0", SEC + S_RINGS, 32'd0);
		rd("lean DROPS reads 0", SEC + S_DROPS, 32'd0);
		rd("lean COUNTS reads 0", SEC + S_COUNTS, 32'd0);
	end
	$display("     req -> done %0d clk_100m cycles", dl - d);

	$display("-- out-of-order and duplicated stores");
	n0 = cd_frames;
	cd_request(32'h00001235); settle;
	sec_wr(S_BEGIN0, 32'h00001235);
	frame_data(32'h00001235, 0, 10);
	sec_data(11, pat(32'h00001235, 11));
	rd("a later word is not taken: next_index 10", SEC + S_ACCEPT, 32'h0000000A);
	sec_data(10, pat(32'h00001235, 10));
	sec_data(10, pat(32'h00001235, 10));
	sec_data(4, pat(32'h00001235, 4));
	rd("an earlier word's resend is not taken: next_index 11", SEC + S_ACCEPT, 32'h0000000B);
	sec_data(11, pat(32'h00001235, 11));
	frame_data(32'h00001235, 12, 588);
	rd("ACCEPT ok", SEC + S_ACCEPT, 32'h8000024C);
	wait_frame(n0);
	check_buf("frame 0x1235 not shifted", 32'h00001235, 588);
	rd("COUNTS: the feeder met no word out of order", SEC + S_COUNTS, 32'h00000000);

	$display("-- lost store detected by ACCEPT, resent from first_drop");
	n0 = cd_frames;
	cd_request(32'h00001236); settle;
	sec_wr(S_BEGIN0, 32'h00001236);
	frame_data(32'h00001236, 0, 100);
	frame_data(32'h00001236, 101, 588);
	rd("ACCEPT !ok, first_drop 100", SEC + S_ACCEPT, 32'h00000064);
	repeat (slow(2000)) @(negedge clk);
	check("frame held open (the CD reader waits)", cd_frames, n0);
	frame_data(32'h00001236, 100, 588);
	rd("ACCEPT ok after resend", SEC + S_ACCEPT, 32'h8000024C);
	wait_frame(n0);
	check_buf("frame 0x1236 after resend", 32'h00001236, 588);

	$display("-- FIFO full with clk_sys stopped: no back-pressure, drops, fixed latency, resend");
	n0 = cd_frames;
	cd_request(32'h00001237); settle;
	sec_wr(S_BEGIN0, 32'h00001237);
	settle;
	sys_run = 0;
	frame_data(32'h00001237, 0, 588);
	rd("ACCEPT drops 76, first_drop 512", SEC + S_ACCEPT, 32'h004C0200);
	rd("DROPS: 76, the high-water saturated at 255", SEC + S_DROPS, dg(32'h00FF004C));
	rd("CAUSE pending, clk_sys stopped", SEC + S_CAUSE, 32'h00000001);
	check("CAUSE read keeps the line with clk_sys stopped", doorbell, 1);
	axi_write(SEC + S_CAUSE, 32'h00000001, 4'hF);
	check("CAUSE store clears with clk_sys stopped", doorbell, 0);
	rd("ID, clk_sys stopped", SEC + S_ID, 32'h33434553);
	rd("undecoded, clk_sys stopped", 21'h1FFFFC, 32'hDEADBEEF);
	sys_run = 1;
	repeat (slow(8000)) @(negedge clk);
	frame_data(32'h00001237, 512, 588);
	rd("ACCEPT ok after resend (drops kept)", SEC + S_ACCEPT, 32'h804C024C);
	wait_frame(n0);
	check_buf("frame 0x1237 after FIFO-full resend", 32'h00001237, 588);

	$display("-- ABORT mid-frame zero-fills");
	n0 = cd_frames;
	cd_request(32'h00001238); settle;
	sec_wr(S_BEGIN0, 32'h00001238);
	frame_data(32'h00001238, 0, 300);
	sec_wr(S_CTRL, C_EN0 | C_ABORT);
	wait_frame(n0);
	check_buf("frame 0x1238: 300 words then zeros", 32'h00001238, 300);
	axi_read(SEC + S_COUNTS, 0, d, dl, beats);
	check("COUNTS: rejected 0, aborted 1, fills 0", {d[31:24], d[15:0]}, dg(24'h000100));
	check("COUNTS: no word met out of order (the lost store's followers were not taken)", d[23:16], 0);
	rd("CTRL keeps EN", SEC + S_CTRL, 32'h1);

	$display("-- save-slot SECTOR_RD over SPI overlapping an LW frame");
	n0 = cd_frames; s0 = cd_strobes; overlap = 0;
	cd_request(32'h00001239); settle;
	sec_wr(S_BEGIN0, 32'h00001239);
	fork
		frame_data(32'h00001239, 0, 588);
		begin repeat (slow(100)) @(negedge clk); spi_sector(16'h0217, 256, 16'h7700, 3); end
	join
	rd("ACCEPT ok", SEC + S_ACCEPT, 32'h8000024C);
	wait_frame(n0);
	repeat (200) @(negedge clk_sys);
	check("save slot took its 256 words", sv_words, 256);
	check("save-slot strobes landed while the LW frame was open", overlap, 1);
	check("the CD reader saw only its 1176 strobes", cd_strobes - s0, 1176);
	check_buf("frame 0x1239 untouched by the save slot", 32'h00001239, 588);

	$display("-- stale BEGIN rejected");
	n0 = cd_frames;
	cd_request(32'h0000123A); settle;
	cause_drain(SEC);
	sec_wr(S_BEGIN0, 32'h00009999);
	frame_data(32'h00009999, 0, 8);
	settle;
	axi_read(SEC + S_STATUS, 0, d, dl, beats);
	check("STATUS: busy and every ack low, bits 3 and 7 reserved", {d[8:3]}, 6'd0);
	rd("the reject re-opens the request: its cause", SEC + S_CAUSE, 32'h00000001);
	rd("REQ shows it again", SEC + S_REQ, 32'h800080C5);
	cause_drain(SEC);
	axi_read(SEC + S_COUNTS, 0, d, dl, beats);
	check("COUNTS: rejected 1", {d[31:24], d[15:0]}, dg(24'h010100));
	sec_wr(S_BEGIN0, 32'h0000123A);
	frame_data(32'h0000123A, 0, 588);
	rd("ACCEPT ok", SEC + S_ACCEPT, 32'h8000024C);
	wait_frame(n0);
	check_buf("frame 0x123A after the reject", 32'h0000123A, 588);

	$display("-- core reset mid-frame zero-fills");
	n0 = cd_frames;
	cd_request(32'h0000123B); settle;
	sec_wr(S_BEGIN0, 32'h0000123B);
	frame_data(32'h0000123B, 0, 200);
	@(negedge clk_sys) cd_reset = 1;
	repeat (4) @(negedge clk_sys);
	cd_reset = 0;
	repeat (2400) @(negedge clk_sys);
	axi_read(SEC + S_STATUS, 0, d, dl, beats);
	check("STATUS idle after the fill, busy and every ack low", {d[8], d[6:4]}, 4'd0);
	axi_read(SEC + S_COUNTS, 0, d, dl, beats);
	check("COUNTS: reset_fills 1", {d[31:24], d[15:0]}, dg(24'h010101));
	check("the CD reader (in reset) completed no frame", cd_frames, n0);
	frame_data(32'h0000123B, 200, 588);
	cd_request(32'h0000123C); settle;
	sec_wr(S_BEGIN0, 32'h0000123C);
	frame_data(32'h0000123C, 0, 588);
	wait_frame(n0);
	check_buf("next frame after the reset", 32'h0000123C, 588);

	$display("-- TEST mode: SUM only, consumer untouched (LW_DIAG=0: TEST ignored, the BEGIN is rejected)");
	settle;                                     // the last frame's done event has crossed before FRAMES is read
	n0 = cd_frames; s0 = cd_strobes;
	axi_read(SEC + S_STATUS, 0, d0, dl, beats);
	sec_wr(S_CTRL, C_EN0 | C_TEST);
	settle;
	rd("CTRL TEST bit", SEC + S_CTRL, DIAG ? 32'h201 : 32'h1);
	sec_wr(S_BEGIN0, 32'h00ABCDEF);
	frame_data(32'h00ABCDEF, 0, 588);
	repeat (slow(8000)) @(negedge clk);
	rd("SUM", SEC + S_SUM, dg(frame_sum(32'h00ABCDEF, 588)));
	axi_read(SEC + S_STATUS, 0, d, dl, beats);
	check("TEST frame counted", d[31:16] - d0[31:16], dg(1));
	if (!DIAG) rd("lean: the BEGIN with no request is rejected, re-opening nothing", SEC + S_REQ, 32'd0);
	check("no strobes reached the CD reader", cd_strobes - s0, 0);
	check("no ack reached the CD reader", cd_frames, n0);
	sec_wr(S_CTRL, C_EN0);

	$display("-- CLR counters");
	sec_wr(S_CTRL, C_EN0 | C_CLR);
	rd("COUNTS cleared", SEC + S_COUNTS, 32'd0);
	rd("DROPS cleared", SEC + S_DROPS, 32'd0);
	axi_read(SEC + S_STATUS, 0, d, dl, beats);
	check("frames cleared (the high-water with DROPS)", d[31:16], 0);

	$display("-- EN off: the slot goes back to hps_io");
	sec_wr(S_CTRL, 32'h0); settle;
	rd("STATUS en_sys 0", SEC + S_STATUS, 32'd0);
	n0 = cd_frames;
	cause_drain(SEC);
	cd_request(32'h00002000); settle;
	check("SPI slot 1 offered", sd_rd[1], 1);
	check("EN 0: no cause", doorbell, 0);
	spi_frame(32'h00002000);
	wait_frame(n0);
	check_buf("frame 0x2000 over SPI through the mux", 32'h00002000, 588);

	$display("-- CAUSE: a request racing the store-back is never lost; a raise in the clear's cycle survives");
	begin : sec_race
		integer k, lost, kept, same, gone, stale, m;
		realtime t0, lat_f, lat_w, off, lead;
		// one raise per round: the request rises with EN set (nothing answers it), then a core reset drops it;
		// calibrated from the clk_sys edge each starts on; the store leads the request's edge by up to 48 ns
		sec_wr(S_CTRL, C_EN0); settle;
		cause_drain(SEC);
		fork
			begin @(negedge clk_sys) t0 = $realtime; end
			cd_request(32'h00002100);
		join
		settle; lat_f = t_fire - t0;
		@(negedge clk_sys) cd_reset = 1; repeat (4) @(negedge clk_sys); cd_reset = 0; settle;
		cause_drain(SEC);
		@(negedge clk_sys) t0 = $realtime; axi_write(SEC + S_CAUSE, 32'h00000000, 4'hF); lat_w = t_wsec - t0;
		m = $rtoi(64.0 / (2.0 * SYS_HP_NS)) + 2; lead = m * 2.0 * SYS_HP_NS;
		lost = 0; kept = 0; same = 0; gone = 0; stale = 0;
		// 128 offsets 0.75 ns apart: at 150 MHz clk_sys is nearly locked to clk_100m (3:2), so each offset's raise
		// and clear meet at a phase that drifts little, and 64 coarser steps can miss the clear's own cycle
		for (k = 0; k < 128; k = k + 1) begin
			off = lat_f - lat_w + (k - 64) * 0.75;
			fork
				begin @(negedge clk_sys); if (lead + off > 0) #(lead + off); axi_write(SEC + S_CAUSE, 32'h00000001, 4'hF); end
				begin repeat (m) @(negedge clk_sys); cd_request(32'h00002200 + k); end
			join
			settle;
			if (t_fire >= t_wsec) begin kept = kept + 1; if (!doorbell) lost = lost + 1; end
			else begin gone = gone + 1; if (doorbell) stale = stale + 1; end
			if (t_fire == t_wsec) same = same + 1;
			// the drain's level read after the store-back: the request the bit no longer shows is in LBA
			axi_read(SEC + S_LBA0, 0, d, dl, beats);
			if (!doorbell && d !== 32'h00002200 + k) lost = lost + 1;
			@(negedge clk_sys) cd_reset = 1; repeat (4) @(negedge clk_sys); cd_reset = 0; settle;
			if (doorbell) cause_drain(SEC);
		end
		$display("     %0d offsets: %0d raises before the clear, %0d in or after it, %0d in its cycle", 128, gone, kept,
		         same);
		check("no request lost across 128 offsets", lost, 0);
		check("some raises came before the clear", gone > 0, 1);
		check("the store cleared every raise before it", stale, 0);
		check("some raises landed in the clear's own cycle (the raise won)", same > 0, 1);
		check("no ring left after the sweep", doorbell, 0);
		sec_wr(S_CTRL, 32'h0); settle;
	end

	$display("-- save slots: EN=0 leaves both save slots on hps_io");
	cause_drain(SEC);                                 // clears the CD's pending cause
	n0 = sv1.done; s0 = sv1.strobes;
	sv1.go(0, 7'h05); settle;
	check("slot 1 rd offered to hps_io", sd_rd[2], 1);
	check("save-slot EN=0: no doorbell", doorbell, 0);
	rd("save-slot EN=0: REQ idle", SEC + S_REQ, 32'd0);
	spi_sector(16'h0217, 512, 16'h3100, 3);
	repeat (50) @(negedge clk_sys);
	check("slot 1 SPI read completed", sv1.done - n0, 1);
	check("slot 1 SPI took 512 strobes", sv1.strobes - s0, 512);
	bad = 0; for (i = 0; i < 512; i = i + 1) if (sv1.ram[i] !== 16'h3100 + i) bad = bad + 1;
	check("slot 1 SPI buffer", bad, 0);

	$display("-- save slots: CTRL enables by slot index (CD bit 0, slot 1 bit 1, slot 2 bit 2)");
	sec_wr(S_CTRL, C_EN0 | C_EN1 | C_EN2); settle;
	rd("CTRL readback EN mask", SEC + S_CTRL, 32'h00000007);
	rdm("STATUS: every slot's en_sys, by slot index", SEC + S_STATUS, 32'h00000777, 32'h00000007);

	$display("-- slot 1 READ over LW; its LBA's bits above the declared 7 are not kept, crossed or compared");
	n0 = sv1.done; s0 = sv1.strobes; r = cd_strobes;
	sv1.go(0, 32'h00000F92); settle;
	check("slot 1 hidden from hps_io", sd_rd[2], 0);
	check("save-slot rd rings", doorbell, 1);
	rd("CAUSE: slot 1's bit", SEC + S_CAUSE, 32'h00000002);
	cause_drain(SEC);
	rd("REQ slot 1 read", SEC + S_REQ, 32'h800180C9);
	rd("LBA_S[1]", SEC + S_LBA1, 32'h00000012);
	rd("RSTAT_S[1]: offered, a read", SEC + S_RSTAT1, 32'h800080C9);
	rd("no slot index 3: its LBA_S offset reads 0", SEC + 21'h164, 32'd0);
	sec_wr(S_BEGIN1, 32'h00000012);
	rd("REQ hides the answered read", SEC + S_REQ, 32'd0);
	rd("RSTAT_S[1]: offered and answered", SEC + S_RSTAT1, 32'hC00080C9);
	frame_data(32'h00000012, 0, 128);
	rdm("STATUS, half the frame stored: nothing of it visible, the feeder idle", SEC + S_STATUS, 32'h00000170, 32'h0);
	frame_data(32'h00000012, 128, 256);
	rd("ACCEPT ok, 256", SEC + S_ACCEPT, 32'h80000100);
	repeat (slow(50)) @(negedge clk);
	rdm("STATUS, the frame stored whole: the feeder busy, slot 1's ack alone", SEC + S_STATUS, 32'h00000170,
	    32'h00000120);
	repeat (1500) @(negedge clk_sys);
	check("slot 1 frame done", sv1.done - n0, 1);
	check("slot 1 took 512 strobes", sv1.strobes - s0, 512);
	bad = 0; for (i = 0; i < 256; i = i + 1) if ({sv1.ram[2*i+1], sv1.ram[2*i]} !== pat(32'h12, i)) bad = bad + 1;
	check("slot 1 buffer byte-exact (1 KB)", bad, 0);
	check("the CD reader saw no save-slot strobe", cd_strobes - r, 0);
	rd("REQ idle after the frame", SEC + S_REQ, 32'd0);

	$display("-- slot 2 WRITE answered by WACK");
	n0 = sv2.done; s0 = sv2.strobes; r = sv1.strobes;
	sv2.go(1, 7'h7F); settle;
	check("slot 2 wr hidden from hps_io", sd_wr[3], 0);
	rd("CAUSE: slot 2's bit", SEC + S_CAUSE, 32'h00000004);
	cause_drain(SEC);
	rd("REQ slot 2 write", SEC + S_REQ, 32'h800280CE);
	rd("RSTAT_S[2]: offered, a write", SEC + S_RSTAT2, 32'h800080CE);
	rd("LBA_S[2]", SEC + S_LBA2, 32'h0000007F);
	sec_wr(S_WACK2, 32'h0000007F);
	rd("ACCEPT ok after a WACK (length 0)", SEC + S_ACCEPT, 32'h80000000);
	repeat (100) @(negedge clk_sys);
	check("slot 2 write acked", sv2.done - n0, 1);
	check("WACK puts no strobe on slot 2", sv2.strobes - s0, 0);
	check("nor on slot 1", sv1.strobes - r, 0);
	rd("REQ idle after the WACK", SEC + S_REQ, 32'd0);

	$display("-- CD first: CD and slot 1 pending together");
	n0 = cd_frames; s0 = sv1.strobes; r = sv1.done;
	fork
		cd_request(32'h00003000);
		sv1.go(0, 7'h20);
	join
	settle;
	rd("REQ: the CD first", SEC + S_REQ, 32'h800080C5);
	rd("CAUSE: a bit for each", SEC + S_CAUSE, 32'h00000003);
	cause_drain(SEC);
	rd("RSTAT_S[0]: offered, not answered", SEC + S_RSTAT0, 32'h800080C5);
	rd("RSTAT_S[1]: offered, a read, not answered", SEC + S_RSTAT1, 32'h800080C9);
	rd("LBA_S[1] the live LBA", SEC + S_LBA1, 32'h00000020);
	sec_wr(21'h10C, 32'h00003000);                         // slot 0 has no WACK_S: nothing enters
	rd("a store at slot 0's WACK_S offset answers nothing", SEC + S_REQ, 32'h800080C5);
	sec_wr(S_BEGIN0, 32'h00003000);
	rd("REQ: answered CD hidden, slot 1 next", SEC + S_REQ, 32'h800180C9);
	rd("RSTAT_S[0]: offered and answered", SEC + S_RSTAT0, 32'hC00080C5);
	rd("RSTAT_S[1] unchanged", SEC + S_RSTAT1, 32'h800080C9);
	frame_data(32'h00003000, 0, 588);
	rd("ACCEPT ok, 588", SEC + S_ACCEPT, 32'h8000024C);
	wait_frame(n0);
	check_buf("CD frame 0x3000", 32'h00003000, 588);
	check("the CD frame put no strobe on slot 1", sv1.strobes - s0, 0);
	n0 = cd_strobes;
	sec_wr(S_BEGIN1, 32'h00000020);
	frame_data(32'h00000020, 0, 256);
	repeat (1500) @(negedge clk_sys);
	check("slot 1 frame after the CD", sv1.done - r, 1);
	bad = 0; for (i = 0; i < 256; i = i + 1) if ({sv1.ram[2*i+1], sv1.ram[2*i]} !== pat(32'h20, i)) bad = bad + 1;
	check("slot 1 buffer after the CD", bad, 0);
	check("the save-slot frame put no strobe on the CD reader", cd_strobes - n0, 0);

	$display("-- stale and mismatched save-slot answers rejected");
	r = sv1.done; s0 = sv1.strobes;
	sv1.go(0, 7'h21); settle;
	cause_drain(SEC);
	sec_wr(S_BEGIN1, 32'h00000099); frame_data(32'h00000099, 0, 256);
	repeat (1500) @(negedge clk_sys); settle;               // its DATA popped behind it
	rd("a stale BEGIN_S, stored whole, re-opens slot 1: its cause", SEC + S_CAUSE, 32'h00000002);
	rd("REQ shows slot 1 again", SEC + S_REQ, 32'h800180C9);
	cause_drain(SEC);
	sec_wr(S_WACK1, 32'h00000021); settle;                  // WACK for a READ request
	rd("a WACK on a read re-opens slot 1: its cause", SEC + S_CAUSE, 32'h00000002);
	rd("REQ shows slot 1 after the WACK", SEC + S_REQ, 32'h800180C9);
	cause_drain(SEC);
	sec_wr(S_WACK2, 32'h00000021); settle;                  // a slot with no request
	rd("an answer for a slot with no request re-opens nothing", SEC + S_CAUSE, 32'd0);
	check("no save-slot strobe, no ack", sv1.strobes - s0 + sv1.done - r, 0);
	sec_wr(S_BEGIN1, 32'h00000021);
	frame_data(32'h00000021, 0, 256);
	repeat (1500) @(negedge clk_sys);
	check("slot 1 frame after the rejects", sv1.done - r, 1);
	bad = 0; for (i = 0; i < 256; i = i + 1) if ({sv1.ram[2*i+1], sv1.ram[2*i]} !== pat(32'h21, i)) bad = bad + 1;
	check("slot 1 buffer after the rejects", bad, 0);

	$display("-- a core reset during a save-slot frame: one not stored in full never reaches the core, one stored in full plays out whole");
	s0 = sv1.strobes; r = sv1.acks;
	sv1.go(0, 7'h22); settle;
	cause_drain(SEC);
	sec_wr(S_BEGIN1, 32'h00000022);
	frame_data(32'h00000022, 0, 100);
	repeat (600) @(negedge clk_sys);
	@(negedge clk_sys) cd_reset = 1;
	repeat (4) @(negedge clk_sys);
	cd_reset = 0;
	repeat (1500) @(negedge clk_sys);
	check("100 words stored, the reset: no ack, no strobe", (sv1.acks - r) + (sv1.strobes - s0), 0);
	frame_data(32'h00000022, 100, 256);                     // the host stores the rest: the request is gone
	repeat (1500) @(negedge clk_sys);
	check("the frame committed after the request fell: rejected whole, no ack, no strobe",
	      (sv1.acks - r) + (sv1.strobes - s0), 0);
	rd("no request: nothing re-opens", SEC + S_CAUSE, 32'd0);
	sv1.go(0, 7'h23); settle;
	cause_drain(SEC);
	sec_wr(S_BEGIN1, 32'h00000023);
	frame_data(32'h00000023, 0, 256);
	repeat (300) @(negedge clk_sys);
	check("stored in full, the frame plays", sv1.acks - r == 0 && sv1.strobes - s0 > 0, 1);
	@(negedge clk_sys) cd_reset = 1;
	repeat (4) @(negedge clk_sys);
	cd_reset = 0;
	repeat (1500) @(negedge clk_sys);
	check("the reset mid-play: the frame's 512 strobes", sv1.strobes - s0, 512);
	check("the reset mid-play: one ack", sv1.acks - r, 1);
	check("slot 1: byte-exact, never zero-filled", sv1_bad(32'h23), 0);
	axi_read(SEC + S_STATUS, 0, d, dl, beats);
	check("STATUS idle after the save-slot frame", {d[8], d[6:4]}, 4'd0);

	$display("-- core reset during a save slot load: the stale BEGIN_S is rejected, the request re-opens");
	r = sv1.done; s0 = sv1.strobes;
	sv1.go(0, 7'h30); settle;
	cause_drain(SEC);                                    // the HPS drains the cause
	rd("REQ slot 1 read", SEC + S_REQ, 32'h800180C9);
	rd("LBA_S[1] 0x30", SEC + S_LBA1, 32'h00000030);       // the HPS fills LBA 0x30 ...
	@(negedge clk_sys) cd_reset = 1;
	repeat (4) @(negedge clk_sys);
	cd_reset = 0;
	sv1.go(0, 7'h00); settle;                              // ... the load restarts at LBA 0
	cause_drain(SEC);                                    // the restart's ring, drained early
	sec_wr(S_BEGIN1, 32'h00000030); frame_data(32'h00000030, 0, 256); settle;   // the stale answer
	check("the reject re-rings the doorbell", doorbell, 1);
	rd("REQ shows slot 1 again", SEC + S_REQ, 32'h800180C9);
	rd("LBA_S[1] 0", SEC + S_LBA1, 32'h00000000);
	cause_drain(SEC);
	sec_wr(S_BEGIN1, 32'h00000000);
	frame_data(32'h00000000, 0, 256);
	repeat (1500) @(negedge clk_sys);
	check("slot 1 frame at LBA 0 after the reject", sv1.done - r, 1);
	bad = 0; for (i = 0; i < 256; i = i + 1) if ({sv1.ram[2*i+1], sv1.ram[2*i]} !== pat(32'h00, i)) bad = bad + 1;
	check("slot 1 buffer LBA 0", bad, 0);
	rd("REQ idle after the re-served frame", SEC + S_REQ, 32'd0);
	check("no ring left", doorbell, 0);

	$display("-- a stale reject behind a newer answer re-opens nothing (the entry's tag)");
	r = sv1.done; n0 = cd_frames;
	sv1.go(0, 7'h31); settle;
	cd_request(32'h00004000); settle;
	cause_drain(SEC);
	sec_wr(S_BEGIN0, 32'h00004000);
	frame_data(32'h00004000, 0, 588);                       // the feeder is still streaming it ...
	sec_wr(S_WACK1, 32'h00000031);                          // ... when both save-slot answers queue: a WACK on a
	sec_wr(S_BEGIN1, 32'h00000031);                         // read (rejected), then the BEGIN_S
	wait_frame(n0);                                         // (its data after: FIFO depth 512)
	frame_data(32'h00000031, 0, 256);
	repeat (1500) @(negedge clk_sys);
	check("slot 1 frame at 0x31", sv1.done - r, 1);
	bad = 0; for (i = 0; i < 256; i = i + 1) if ({sv1.ram[2*i+1], sv1.ram[2*i]} !== pat(32'h31, i)) bad = bad + 1;
	check("slot 1 buffer 0x31", bad, 0);
	check("the older answer's reject rang nothing", doorbell, 0);
	rd("REQ idle", SEC + S_REQ, 32'd0);

	$display("-- the save slot's LBA moves while its request stays high: the reject re-latches it");
	r = sv1.done;
	sv1.go(0, 7'h40); settle;
	cause_drain(SEC);
	rd("LBA_S[1] 0x40", SEC + S_LBA1, 32'h00000040);
	@(negedge clk_sys) sv1.lba = 7'h41;                     // rd never fell
	settle;
	rd("LBA_S[1] still the latched 0x40", SEC + S_LBA1, 32'h00000040);
	sec_wr(S_BEGIN1, 32'h00000040);                         // answers the LBA it was shown, whole
	frame_data(32'h00000040, 0, 256); settle;
	check("the moved LBA's answer re-rings", doorbell, 1);
	rd("REQ shows slot 1 again", SEC + S_REQ, 32'h800180C9);
	rd("LBA_S[1] re-latched 0x41", SEC + S_LBA1, 32'h00000041);
	cause_drain(SEC);
	sec_wr(S_BEGIN1, 32'h00000041);
	frame_data(32'h00000041, 0, 256);
	repeat (1500) @(negedge clk_sys);
	check("slot 1 frame at 0x41", sv1.done - r, 1);
	bad = 0; for (i = 0; i < 256; i = i + 1) if ({sv1.ram[2*i+1], sv1.ram[2*i]} !== pat(32'h41, i)) bad = bad + 1;
	check("slot 1 buffer 0x41", bad, 0);
	rd("REQ idle after the moved LBA", SEC + S_REQ, 32'd0);
	check("no ring left after the moved LBA", doorbell, 0);

	$display("-- a save slot's reject re-opens that slot only: the CD answered behind it stays hidden");
	r = sv1.done; s0 = sv2.done; n0 = cd_frames;
	sv1.go(0, 7'h70);
	sv2.go(0, 7'h10);
	cd_request(32'h00006000); settle;
	cause_drain(SEC);
	sys_run = 0;                                            // every answer queues before the feeder runs
	sec_wr(S_BEGIN1, 32'h00000071);                         // slot 1, the wrong LBA, stored whole: rejected first
	frame_data(32'h00000071, 0, 256);
	sec_wr(S_BEGIN2, 32'h00000010);                         // slot 2, right: its frame holds the feeder
	frame_data(32'h00000010, 0, TB_XFRAME1);
	sec_wr(S_BEGIN0, 32'h00006000);                         // the CD, right, behind slot 2
	sys_run = 1; repeat (40) @(negedge clk_sys);            // the reject is in; slot 2 still streams
	rd("REQ shows slot 1, not the answered CD", SEC + S_REQ, 32'h800180C9);
	repeat (1500) @(negedge clk_sys);
	check("slot 2 frame at 0x10", sv2.done - s0, 1);
	frame_data(32'h00006000, 0, 588);
	wait_frame(n0);
	check_buf("CD buffer 0x6000 behind the save slot's reject", 32'h00006000, 588);
	cause_drain(SEC);
	sec_wr(S_BEGIN1, 32'h00000070);
	frame_data(32'h00000070, 0, 256);
	repeat (1500) @(negedge clk_sys);
	check("slot 1 frame at 0x70", sv1.done - r, 1);
	rd("REQ idle after all three", SEC + S_REQ, 32'd0);
	check("no ring left after all three", doorbell, 0);

	$display("-- adjacent rejects of two slots: both requests re-open");
	r = sv1.done; s0 = sv2.done;
	sv1.go(0, 7'h24);
	sv2.go(0, 7'h34); settle;
	cause_drain(SEC);
	sys_run = 0;                                            // both stale answers queue back to back: WACKs on
	sec_wr(S_WACK1, 32'h00000024);                          // read requests
	sec_wr(S_WACK2, 32'h00000034);
	sys_run = 1; settle;
	rd("REQ shows slot 1 re-opened", SEC + S_REQ, 32'h800180C9);
	cause_drain(SEC);
	sec_wr(S_BEGIN1, 32'h00000024);
	frame_data(32'h00000024, 0, 256);
	repeat (1500) @(negedge clk_sys);
	check("slot 1 frame at 0x24", sv1.done - r, 1);
	rd("REQ shows slot 2 re-opened", SEC + S_REQ, 32'h800280CD);
	cause_drain(SEC);
	r = sv2.strobes;
	sec_wr(S_BEGIN2, 32'h00000034);
	frame_data(32'h00000034, 0, TB_XFRAME1);
	repeat (1500) @(negedge clk_sys);
	check("slot 2 frame at 0x34", sv2.done - s0, 1);
	check("slot 2's frame at its own length: two strobes a word", sv2.strobes - r, 2 * TB_XFRAME1);
	bad = 0;
	for (i = 0; i < TB_XFRAME1; i = i + 1) if ({sv2.ram[2*i+1], sv2.ram[2*i]} !== pat(32'h34, i)) bad = bad + 1;
	check("slot 2 buffer byte-exact", bad, 0);
	rd("REQ idle after both re-opened", SEC + S_REQ, 32'd0);

	sec_rec_cases;
	sec8_cases;
	sec10_cases;

	$display("-- save slot EN off: back to hps_io, no doorbell");
	sec_wr(S_CTRL, C_EN0); settle;
	cause_drain(SEC);
	n0 = sv2.done; s0 = sv2.strobes;
	sv2.go(1, 7'h44); settle;
	check("slot 2 wr offered to hps_io", sd_wr[3], 1);
	check("no doorbell for a disabled save slot", doorbell, 0);
	rd("REQ idle for a disabled save slot", SEC + S_REQ, 32'd0);
	spi_sector(16'h0318, 512, 16'h0, 3);                   // the stock save drain, hps_io reads the buffer
	repeat (50) @(negedge clk_sys);
	check("slot 2 SPI write completed", sv2.done - n0, 1);
	check("SPI write: no strobe into slot 2", sv2.strobes - s0, 0);
	sec_wr(S_CTRL, 32'h0); settle;

	if (DIAG) begin
		$display("-- DIAG: SUM and FRAMES move on the same edge");
		$display("     %0d SUM changes, %0d without FRAMES", sum_moves, sum_alone);
		check("SUM changed during the SECTOR cases", sum_moves > 0, 1);
		check("SUM never changed without FRAMES", sum_alone, 0);
	end

	sec_view_cases;
	ena_up;
`endif

`ifdef MISTER_LW_EVT
	$display("-- SYS: the status back-channel (hps_io 0x29) as a cause, a line and a snapshot");
	rd("SYS ID EVT2", SYSB + 21'h1C, 32'h32545645);
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
			if (sys_irq) sys_drain;
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

	evt1_cases;
	evt2_cases;
	rd("SYS PEND: nothing stands", EVPEND, 32'd0);

`endif
`ifdef MISTER_LW_OSD
	$display("-- OSDBUF: registers");
	rd("OSD ID OSD2", OSD + 21'h1804, 32'h3244534F);
	osd_st(d);
	check("OSD STATUS: nothing waiting", d[24:16], 0);
	rd("OSD image reads 0", OSD + 21'h0000, 32'd0);
	rd("OSD image end reads 0", OSD + 21'h0FFC, 32'd0);
	rd("OSD register hole", OSD + 21'h1808, 32'd0);
	rd("OSD block end", OSD + 21'h1FFC, 32'd0);
	n0 = n_osdq;
	axi_write(OSD + 21'h1000, 32'h12345678, 4'hF);          // row 16: only with MENU_CORE
	axi_write(OSD + 21'h1800, 32'h12345678, 4'hF);          // STATUS
	axi_write(OSD + 21'h0000, 32'h12345678, 4'h0);          // no byte enabled
	check("a store past row 15, to STATUS or with no byte enabled takes no entry", n_osdq - n0, 0);

	$display("-- OSDBUF: an image over LW equals it over SPI, byte for byte, in both osds");
	osd_st(d); d0 = d;
	for (r = 0; r < 16; r = r + 1) spi_osd_row(r, 0);
	spi_osd_cmd(8'h40);                   // rows 8-15 re-armed highres; DISABLE clears it
	spi_osd_cmd(8'h41);
	check("osd_status after ENABLE", osd_status, 1);
	ob_cmp("the SPI image", 0, 16);
	for (r = 0; r < 16; r = r + 1) lw_row(r, 1);
	osd_drain;
	ob_cmp("another image over LW", 0, 16);
	for (r = 0; r < 16; r = r + 1) lw_row_f(r, 0, 1);
	osd_drain;
	bad = 0;
	for (i = 0; i < 4096; i = i + 1)
		if (hdmi_osd.osd_buffer[i] !== spp(i, 0) || vga_osd.osd_buffer[i] !== spp(i, 0)) bad = bad + 1;
	check("the SPI image's bytes over LW: both osd_buffers byte for byte as over SPI", bad, 0);
	osd_st(d);
	check("STATUS: nothing waiting, nothing dropped (the stores paced by level)", {d[24:16], d[15:0]}, {9'd0, d0[15:0]});

	$display("-- OSDBUF: byte-lane stores");
	osd_word(3*64 + 10, 32'h0000E700, 4'h2);
	osd_word(12*64 + 25, 32'hC3A55A3C, 4'hF);
	osd_word(7*64 + 1, 32'hAB0000CD, 4'h9);
	osd_word(9*64 + 2, 32'h00FE0000, 4'h4);
	osd_drain;
	ob_cmp("byte lanes: only the enabled bytes", 0, 16);
	check("the lane-1 byte (row 3, column 41)", hdmi_osd.osd_buffer[3*256 + 41], 8'hE7);
	check("its neighbours as they were", {hdmi_osd.osd_buffer[3*256 + 40], hdmi_osd.osd_buffer[3*256 + 42]},
	      {spp(3*256 + 40, 0), spp(3*256 + 42, 0)});

	$display("-- OSDBUF: an SPI row and an LW row in flight together: SPI goes first, nothing is lost");
	n0 = osd_held; d0 = d;
	fork
		spi_osd_row(2, 1);
		lw_row(5, 1);
	join
	osd_drain;
	ob_cmp("both rows landed", 0, 16);
	osd_st(d);
	check("nothing dropped", d[15:0], d0[15:0]);
	$display("     the writer held a word while a select was high in %0d cycles", osd_held - n0);
	check("the writer waited for the SPI row", osd_held - n0 > 0, 1);
	check("no LW byte written in a cycle with, or two after, an osd select or the ID view", osd_clash, 0);

	$display("-- OSDBUF: rows 8-15 over LW set highres as OSDCMDWRITE's bit 3 does; the disable clears it");
	osd_lo;
	check("after DISABLE and ENABLE: 64 lines in both osds", {hdmi_osd.osd_h == 64, vga_osd.osd_h == 64}, 2'b11);
	lw_row(0, 3); lw_row(7, 3); osd_drain;
	check("rows 0 and 7 over LW: still 64", {hdmi_osd.osd_h == 64, vga_osd.osd_h == 64}, 2'b11);
	osd_check("rows 0-15 drawn, 64 lines", 64, 6);
	osd_word(9*64 + 3, 32'h5A5A5A5A, 4'h8); osd_drain;
	check("a byte of row 9 over LW: 128 lines", {hdmi_osd.osd_h == 128, vga_osd.osd_h == 128}, 2'b11);
	osd_check("highres from an LW byte: 128 lines", 128, 3);
	spi_osd_cmd(8'h40);
	check("osd_status after DISABLE", osd_status, 0);
	check("the disable clears it: 64", {hdmi_osd.osd_h == 64, vga_osd.osd_h == 64}, 2'b11);
	osd_check("OSD disabled: no box", 0, 3);
	spi_osd_cmd(8'h41);

	$display("-- OSDBUF: the pixel path draws the same buffer the same, LW or SPI (1x, rotated, info)");
	osd_same("1x", 16, 20);
	spi_osd_en(0, 12'd0, 12'd0, 6'd0, 6'd0, 2'd1);
	osd_same("rotated", 16, 22);
	spi_osd_en(1, 12'd16, 12'd8, 6'd16, 6'd6, 2'd0);
	check("info box: osd_status low", osd_status, 0);
	osd_same("info box", 16, 24);
	spi_osd_en(0, 12'd0, 12'd0, 6'd0, 6'd0, 2'd0);
	spi_osd_cmd(8'h40); spi_osd_cmd(8'h41);

	$display("-- OSDBUF: with clk_sys stopped the FIFO fills and drops count; after the restart a re-store completes it");
	osd_st(d0);
	sys_run = 0; repeat (20) @(negedge clk);
	for (i = 0; i < 300; i = i + 1) axi_write(OSD + 4*i, pw(i, 30, 0), 4'hF);
	osd_st(d);
	check("stopped: level 256", d[24:16], 256);
	check("stopped: 44 stores dropped", d[15:0] - d0[15:0], 44);
	sys_run = 1;
	osd_drain;
	bad = 0;
	for (i = 0; i < 256; i = i + 1) if (bw_h(i) !== pw(i, 30, 0) || bw_v(i) !== pw(i, 30, 0)) bad = bad + 1;
	check("restarted: the 256 queued words landed", bad, 0);
	bad = 0;
	for (i = 256; i < 300; i = i + 1) if (bw_h(i) === pw(i, 30, 0) || bw_v(i) === pw(i, 30, 0)) bad = bad + 1;
	check("restarted: the 44 dropped did not", bad, 0);
	for (i = 0; i < 300; i = i + 1) begin
		if (i % 64 == 0) osd_room(64);
		osd_word(i, pw(i, 30, 0), 4'hF);
	end
	osd_drain;
	ob_cmp("the re-store completes the image", 0, 16);

	$display("-- OSDBUF: bytes queued behind an asserted osd select when the enable ends are discarded");
	osd_lo;
	@(negedge clk_sys); io_osd = 1;                    // a host's select left asserted, no strobe
	for (i = 9*64; i < 9*64 + 40; i = i + 1) axi_write(OSD + 4*i, ~pw(i, 30, 0), 4'hF);   // rows 8-15: highres
	for (i = 64; i < 64 + 40; i = i + 1) axi_write(OSD + 4*i, ~pw(i, 30, 0), 4'hF);
	osd_st(d);
	check("queued behind the select: 80 entries waiting", d[24:16], 80);
	host_end;
	wait_quiet;
	rd("after the ID view: OWNED clear", OWNEDA, 32'd0);
	osd_st(d);
	check("after the ID view: nothing waiting", d[24:16], 0);
	@(negedge clk_sys); io_osd = 0;
	repeat (slow(300)) @(negedge clk);
	ob_cmp("no queued byte was written", 0, 16);
	check("highres unchanged: 64", {hdmi_osd.osd_h == 64, vga_osd.osd_h == 64}, 2'b11);
	ena_take;
	osd_word(64 + 5, 32'h0F1E2D3C, 4'hF);
	osd_drain;
	check("a store after OPEN under the next enable lands",
	      {bw_h(64 + 5) == 32'h0F1E2D3C, bw_v(64 + 5) == 32'h0F1E2D3C}, 2'b11);
	ob_cmp("the image after it", 0, 16);
	ena_up;

`endif
`ifdef MISTER_LW_LANE
	$display("-- LANE: registers");
	rd("LANE ID LAN1", LANE + 21'h18, 32'h314E414C);
	rd("LANE PLAYED reset", LANE + 21'h10, 32'd0);
	rd("LANE DROPS reset", LANE + 21'h14, 32'd0);
	rd("LANE RSTAT reset", LANE + 21'h1C, 32'd0);
	rd("LANE RHEAD reads 0 while empty", LANE + 21'h20, 32'd0);
	rd("LANE BEGIN reads 0", LANE, 32'd0);
	rd("LANE CAUSE clear", LCAUSE, 32'd0);
	rd("LANE register hole", LANE + 21'h28, 32'd0);
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
		cause_drain(LCAUSE);
		@(negedge clk_sys); io_enable = 1;          // the link holds its select: the player cannot start
		f_set(0, 16'h001E, 8);
		for (k = 0; k < nfit + 2; k = k + 1) begin
			for (i = 0; i < 8; i = i + 1) fw[i] = k * 16 + i;
			lane_send(1);
		end
		lane_count(p1, dr1);
		check("lane 8: (DEPTH - 1) / 10 frames of 10 entries fit, two dropped", dr1 - dr, 2);
		check("lane 8: nothing played while the link is selected", p1, p);
		rd("lane 8: CAUSE: DROP alone", LCAUSE, 32'h4);
		check("lane 8: line 3 high", lines[2], 1);
		axi_write(LCAUSE, 32'h4, 4'hF);
		@(negedge clk_sys); io_enable = 0;
		lane_wait(p + nfit);
		check("lane 8: the last committed frame's status", hst[31:0], {16'((nfit - 1) * 16 + 1), 16'((nfit - 1) * 16)});
		check("lane 8: status word 7", hst[127:112], 16'((nfit - 1) * 16 + 7));
		lane_count(p1, dr1);
		check("lane 8: DROPS unchanged by the drain", dr1 - dr, 2);
		rd("lane 8: CAUSE after the drain: PLAYED alone", LCAUSE, 32'h1);
		cause_drain(LCAUSE);
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
		sys_drain;                         // drain the SYS cause the request raised
		// the link: the reply to each word is io_dout once its ack is seen (the HPS reads gp_in then)
		@(negedge clk_sys); io_enable = 1; repeat (2) @(negedge clk_sys);
		for (m = 0; m < 9; m = m + 1) begin hps_word((m == 0) ? 16'h0029 : 16'h0000); ws[m] = hps_bus[15:0]; end
		@(negedge clk_sys); io_enable = 0; repeat (3) @(negedge clk_sys);
		check("lane 13 link: the head {A, flag}", {ws[0][15:4], ws[0][3:0] == `TB_STFLG}, {12'h00A, 1'b1});
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
		cause_drain(LCAUSE);
		f_set(0, 16'h0033, nr + 44);                // nr + 45 replies: 45 past the depth
		for (i = 0; i < nr + 44; i = i + 1) fw[i] = {8'(i / 3), 8'(i * 11)};
		trk_clear;
		lane_send_c(1, 1'b1);
		lane_wait(p + 1);
		check("lane 15: the frame played whole", gam_n, nr + 44);
		lane_rstat(rs);
		check("lane 15: the reply FIFO holds its depth", rs[15:0], nr);
		check("lane 15: 45 replies dropped", rs[31:16], 45);
		rd("lane 15: CAUSE: PLAYED, REPLY and DROP", LCAUSE, 32'h7);
		lane_count(v_l, dr1_l);
		check("lane 15: DROP from the reply words: DROPS unchanged", dr1_l, dr);
		cause_drain(LCAUSE);
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

	lane_cause_cases;
	lane_view_cases;
`endif

	$display("-- a store is in effect at its response: a read issued after it sees it, one accepted before its beat may not");
	begin : store_order
		reg [31:0] v, oc, p, dr, rs, h;
		integer k, nbad;
		if (TB_EVT) begin
			st_req(st_pat(600)); settle;
			check("order: SYS cause pending", sys_irq, 1);
			store_read(SYSB, 32'h1, SYSB, v);
			check("order: a W1C store on SYS CAUSE, then CAUSE reads clear", v, 32'd0);
		end
		store_read(SCR, 32'h0D15EA5E, SCR, v);
		check("order: a store to SCRATCH, then SCRATCH", v, dg(32'h0D15EA5E));
		axi_read(FW_CTRL, 0, oc, dl, beats);
		store_read(FW_CTRL, 32'h2, FW_CTRL, v);
		check("order: a store to LINE_EN, then LINE_EN", v, 32'h2 & FW_MASK);
		axi_write(FW_CTRL, oc, 4'hF);
		// the read's address accepted two cycles and one before the W1C's data beat, in its cycle and after it
		for (k = -2; k <= 1 && TB_EVT; k = k + 1) begin
			st_req(st_pat(610 + k)); settle;
			race_store_read(SYSB, 32'h1, SYSB, k, v);
			check($sformatf("order: CAUSE read accepted %0d cycles from the W1C's data beat", k), v,
			      k < 0 ? 32'h00000001 : 32'd0);
			check("order: the W1C cleared it either way", sys_irq, 0);
		end
		if (TB_SEC) begin
			cause_drain(SEC);
			sec_wr(S_CTRL, 32'h0); settle;
			cd_request(32'h00007100); settle;
			check("order: no SECTOR cause before EN", doorbell, 0);
			store_read(SEC + S_CTRL, 32'h1, SEC + S_CTRL, v);
			check("order: an EN store, then CTRL reads it", v, 32'h1);
			settle;
			check("order: SECTOR cause pending", doorbell, 1);
			store_read(SEC, 32'h1, SEC, v);
			check("order: a W1C store on SECTOR CAUSE, then CAUSE reads clear", v, 32'd0);
			sec_wr(S_CTRL, 32'h0);
			@(negedge clk_sys) cd_reset = 1; repeat (4) @(negedge clk_sys); cd_reset = 0; settle;
			if (doorbell) cause_drain(SEC);
			rd("order: REQ idle again", SEC + S_REQ, 32'd0);
			sec_wr(S_CTRL, C_EN0); settle;
			n0 = cd_frames;
			cd_request(32'h00007200); settle;
			cause_drain(SEC);
			sec_wr(S_BEGIN0, 32'h00007200);
			frame_data(32'h00007200, 0, TB_FRAME - 1);
			store_read(SEC + S_DATA + 4*(TB_FRAME - 1), pat(32'h00007200, TB_FRAME - 1), SEC + S_ACCEPT, v);
			check("order: the last DATA store, then ACCEPT counts it", v, 32'h80000000 | TB_FRAME);
			wait_frame(n0);
			check_buf("order: that frame completes", 32'h00007200, TB_FRAME);
			cause_drain(SEC);
			sec_wr(S_CTRL, 32'h0); settle;
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
		if (TB_EVT) begin
			st_req(st_pat(500)); settle;
			check("pure reads: SYS cause pending", sys_irq, 1);
		end
		if (TB_SEC) begin
			sec_wr(S_CTRL, C_EN0); settle;
			cd_request(32'h00007000); settle;
			check("pure reads: SECTOR cause pending", doorbell, 1);
			n0 = cd_frames;
			sec_wr(S_BEGIN0, 32'h00007000);
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
		check("pure reads: both lines still high", {30'd0, TB_SEC ? doorbell : 1'b1, TB_EVT ? sys_irq : 1'b1}, 32'd3);
		sys_run = 1;
		if (TB_SEC) begin
			frame_data(32'h00007000, 294, 588);
			wait_frame(n0);
			check_buf("pure reads: the half-stored frame completes", 32'h00007000, 588);
			cause_drain(SEC);
			sec_wr(S_CTRL, 32'h0); settle;
		end
		if (TB_EVT) sys_drain;
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
	ena_up;

	if (!DIAG) begin
		$display("-- LW_DIAG=0: the removed registers read 0");
		rd("lean SCRATCH", SCR, 32'd0);
		if (TB_SEC) begin
			axi_read(SEC + S_STATUS, 0, d, dl, beats);
			check("lean STATUS[31:16]", d[31:16], 0);
			for (i = 6; i < 13; i = i + 1) rd("lean SECTOR reg", SEC + 4*i, 32'd0);
			rd("lean ID kept", SEC + S_ID, 32'h33434553);
		end
	end

`ifdef LW_TB_CDC
	// the models moved edges in this run: synchroniser bits, a Gray bus, and no bundle capture took a bit in flight
	if (!$value$plusargs("lw_seed=%d", i)) i = 1;
	$display("-- CDC models (seed %0d): edges moved: PAD handshake %0d, EVT handshakes and levels %0d, the cell bench's Gray counts %0d",
	         i, win.pad.pad.pad_x.sync_r.m_n + win.pad.pad.pad_x.sync_a.m_n,
	         win.evt.evt.st_x.sync_r.m_n + win.evt.evt.st_x.sync_a.m_n + win.evt.evt.ev_x.sync_r.m_n +
	         win.evt.evt.ev_x.sync_a.m_n + win.evt.evt.lvl_sync.m_n, cg_xa.m_n + cg_xb.m_n);
	check("models: the PAD handshake's toggles took extra cycles", win.pad.pad.pad_x.sync_r.m_n + win.pad.pad.pad_x.sync_a.m_n > 0, 1);
	check("models: the EVT handshakes' toggles took extra cycles",
	      win.evt.evt.st_x.sync_r.m_n + win.evt.evt.st_x.sync_a.m_n + win.evt.evt.ev_x.sync_r.m_n + win.evt.evt.ev_x.sync_a.m_n > 0, 1);
	check("models: a Gray bus took an extra cycle", cg_xa.m_n + cg_xb.m_n > 0, 1);
	check("models: captures that took a bit still in flight (PAD, EVT's status and events, the cell bench)",
	      win.pad.pad.pad_x.cap.m_n + win.evt.evt.st_x.cap.m_n + win.evt.evt.ev_x.cap.m_n + cx_a.cap.m_n + cx_b.cap.m_n, 0);
`ifdef MISTER_LW_SECTOR_SLOT
	check("models: SECTOR's request records: a capture never took a bit still in flight", win.sec.sector.rec_x.cap.m_n, 0);
	check("models: SECTOR's record handshake's toggles took extra cycles",
	      win.sec.sector.rec_x.sync_r.m_n + win.sec.sector.rec_x.sync_a.m_n > 0, 1);
`endif
`ifdef MISTER_LW_SECTOR_SLOT
	check("models: SECTOR's crossings took extra cycles",
	      win.sec.sector.sys_sync.m_n + win.sec.sector.ctl_sync.m_n > 0, 1);
`endif
`ifdef MISTER_LW_LANE
	check("models: the lane's pointers took extra cycles", win.lane.lane.wg_sync.m_n + win.lane.lane.rg_sync.m_n > 0, 1);
`endif
`endif

	ena_off;
	ena_cases;
	line_cases;
	look_cases;
	fresh_cases;
	ls10;
	if (TB_LINKED != 0) begin
		ls6;
		ls11;
		ls12;
		ls15;
		ls17;
		ls8;
`ifdef MISTER_LW_SECTOR_SLOT
		ls18;
`endif
		if (TB_SEC) sec7_sweep;
	end
	ena_summary(1);

	$display("-- one request, one path: no request acknowledged by both hps_io and the feeder (the whole suite)");
	$display("     %0d SPI acks, %0d LW acks", ap_spi, ap_lw);
	check("requests acknowledged by both paths", ap_both, 0);
	if (TB_SEC) check("acks met on both paths", ap_spi > 0 && ap_lw > 0, 1);

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
