//============================================================================
//  lw_lane.sv -- part of the LW bridge framework (docs/LW_BRIDGE.md)
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
`ifdef MISTER_LW_LANE
`define LW_LANE_BUILT
`define LW_LINK_BUILT
`endif
`endif
`ifdef MISTER_OCRAM
`ifdef MISTER_OCRAM_FIO
`ifndef LW_LINK_BUILT
`define LW_LINK_BUILT
`endif
`endif
`endif
`ifdef LW_LINK_BUILT
`default_nettype none

// LANE block (LW directory kind 5, minor 2, "LAN1"): the ordered frame lane, built with MISTER_LW_LANE.
// Stores on clk_100m queue whole frames (BEGIN, WORDs, END) in a dual-clock FIFO; the clk_sys player replays
// each committed frame onto the link nets through lw_link, so hps_io, hps_ext, both osd instances and sys_top's
// own arms each decode one ordinary link transaction. A frame opened with BEGIN's capture bit records the
// link's reply word (io_dout) after each of its strobes in a second FIFO, which the HPS reads at RHEAD and
// pops with a store to RPOP. Bridge side has no reset and answers every store in fixed cycles: a frame that
// does not fit is dropped whole at END and counted, a frame without END is never played, and a reply word
// that finds the reply FIFO full is dropped and counted. Contracts: docs/LW_BRIDGE.md, "LANE block".
// CAUSE (0x24), the lane's line: PLAYED (bit 0) per frame played, REPLY (1) in every cycle the synchronised reply
// write pointer differs from its value a cycle before, so a word landing while another waits, or in the cycle the
// last is popped, raises it too; DROP (2) per frame dropped at END and per reply word dropped. A full-word store
// clears each bit written as 1, a raise in the store's cycle wins; a taken ENABLE clears the word. Reads change
// nothing. Each bit comes from a crossing the block already has: the played toggle, the reply write pointer, the
// reply-drop toggle and the frame-drop count.
// The host enable (hen, the window's en_q; it falls at an ID view, a core reset through GPO or an HPS reset):
// while it is low the open frame is abandoned and the reply FIFO's read pointer steps toward its write pointer, so
// replies that land after the enable fell are discarded. The player learns of the enable from its enable link
// alone (lw_fence, lw_cdc.sv): stores reach this block only while the link is open, which is after the player has
// seen the enable; while its level is low the player completes the frame in
// flight and pops every queued entry unplayed, in every cycle it is idle. Its rest, which the link acknowledges,
// is the player idle with idle outputs and the frame FIFO empty; the link returns to IDLE only once the reply
// FIFO has been stepped to empty too, so a new enable starts with both FIFOs empty. The player's busy input
// includes the link's ID view (sys_top), so no frame starts while it is selected.
// Crossings, each through lw_sync (lw_cdc.sv): the four Gray pointers of the two FIFOs, each recomputed from its
// binary pointer in every cycle, the played and reply-drop toggles, and the enable link's two levels.
// lw_link and lw_link_seq, below, are built with this block or with the OCRAM download tier (MISTER_OCRAM_FIO), whose
// player in hps_ocram_rx.sv takes the same p_* side and the same sequencer; a build has one player.

`ifdef LW_LANE_BUILT
module lw_lane #(parameter AW = 10, parameter [2:0] GAP = 3'd4, parameter RAW = 8)   // GAP: 2..7
(
	input wire         clk,          // clk_100m

	input wire  [11:0] raddr,        // block-relative
	output reg  [31:0] rdata,
	input wire  [11:0] waddr,
	input wire         wsel,
	input wire         write,
	input wire  [31:0] wdata,
	input wire   [3:0] be,
	input wire         hen,          // the host enable
	input wire         take,         // an ENABLE store is taken in this cycle
	output wire        f_open,       // the enable link: stores are taken (OPEN)
	output wire        f_owned,      // OWNED
	output wire        f_idle,       // the link lets an ENABLE through
	output wire        irq,          // level, high while CAUSE != 0

	input wire         clk_sys,
	input wire         l_busy,       // any of the link's own selects, or its ID view
	input wire         l_wait,       // io_wait | vs_wait: the term that withholds the link's ack
	input wire  [15:0] l_dout,       // the link's reply word (io_dout | io_dout_sys)
	output wire        own,          // the player drives the link nets
	output wire        hold,         // own, plus one cycle: the link's ack path stays frozen
	output wire  [3:0] sel,          // {osd_vga, osd_hdmi, fpga, uio}
	output wire        strobe,
	output wire [15:0] din           // the head entry's data: held from the cycle before its strobe
);

localparam DEPTH  = 1 << AW;
localparam RDEPTH = 1 << RAW;
localparam [1:0] K_WORD = 2'd0, K_BEGIN = 2'd1, K_END = 2'd2, K_BEGINC = 2'd3;   // kind[0] = a BEGIN
localparam [AW:0] ROOM = {1'b0, {AW{1'b1}}}; // DEPTH - 1: a frame always keeps one entry for its END

function automatic [AW:0] g2b(input [AW:0] g);
	integer n;
	begin
		g2b[AW] = g[AW];
		for (n = AW - 1; n >= 0; n = n - 1) g2b[n] = g2b[n + 1] ^ g[n];
	end
endfunction

function automatic [RAW:0] g2r(input [RAW:0] g);
	integer n;
	begin
		g2r[RAW] = g[RAW];
		for (n = RAW - 1; n >= 0; n = n - 1) g2r[n] = g2r[n + 1] ^ g[n];
	end
endfunction

// {kind[1:0], sel[1:0], data[15:0]}
(* ramstyle = "M10K" *) reg [19:0] fram [0:DEPTH-1];
// reply words, written on clk_sys, read on clk_100m
(* ramstyle = "M10K" *) reg [15:0] rram [0:RDEPTH-1];

/////////////////////////////// clk_100m //////////////////////////////////

reg  [AW:0] wspec = 0;                      // the open frame's next entry
reg  [AW:0] wcom  = 0;                      // past the last committed END
reg  [AW:0] wvis  = 0;                      // steps one entry per clk toward wcom: the gray code moves one bit
(* preserve *) reg [AW:0] lane_w_lwgray = 0;
(* preserve *) reg [AW:0] lane_r_lwgray = 0;   // clk_sys
wire [AW:0] rg_s;
reg         open = 0, ovf = 0;
reg  [15:0] played = 0, drops = 0;          // free-running, 16-bit
(* preserve *) reg pt_lwx = 0;              // clk_sys: toggles once per played frame
wire        pt_s;
reg         pt_d = 0;

reg [RAW:0] rrp = 0;                        // reply FIFO read pointer
(* preserve *) reg [RAW:0] lane_rr_lwgray = 0;
(* preserve *) reg [RAW:0] lane_rw_lwgray = 0;   // clk_sys: reply FIFO write pointer
wire [RAW:0] rwg_s;
reg [RAW:0] rwg_d = 0;                      // rwg_s a cycle before: a difference is a reply word landed
reg  [15:0] rhead = 0;
reg  [15:0] rdrops = 0;                     // reply words dropped on a full reply FIFO, free-running
(* preserve *) reg rdt_lwx = 0;             // clk_sys: toggles once per dropped reply word
wire        rdt_s;
reg         rdt_d = 0;

lw_sync #(.W(AW + 1), .GRAY(1)) rg_sync (.clk(clk), .d(lane_r_lwgray), .q(rg_s));
lw_sync #(.W(RAW + 1), .GRAY(1)) rwg_sync (.clk(clk), .d(lane_rw_lwgray), .q(rwg_s));
lw_sync #(.W(1)) pt_sync (.clk(clk), .d(pt_lwx), .q(pt_s));
lw_sync #(.W(1)) rdt_sync (.clk(clk), .d(rdt_lwx), .q(rdt_s));

wire        st     = wsel & write & &be;
wire        w_beg  = st && waddr[11:2] == 10'd0;
wire        w_wrd  = st && waddr[11:2] == 10'd1 && open;
wire        w_end  = st && waddr[11:2] == 10'd2 && open;
wire [AW:0] rbin   = g2b(rg_s);
wire [AW:0] used   = (w_beg ? wcom : wspec) - rbin;   // entries in use below the one this store writes
wire        b_ok   = used < ROOM;           // BEGIN, and its END after it
wire        w_ok   = ~ovf && used < ROOM;
wire [RAW:0] rcount = g2r(rwg_s) - rrp;      // reply words waiting
wire        w_pop  = st && waddr[11:2] == 10'd3 && rcount != 0;
wire [RAW:0] rrnext = rrp + 1'd1;

// CAUSE: {DROP, REPLY, PLAYED}
localparam [2:0] C_PLAYED = 3'b001, C_REPLY = 3'b010, C_DROP = 3'b100;
reg   [2:0] cause  = 0;
wire        w_cause = st && waddr[11:2] == 10'd9;
wire  [2:0] clr    = w_cause ? wdata[2:0] : 3'd0;
wire  [2:0] raise  = ((pt_s ^ pt_d) ? C_PLAYED : 3'd0) | ((rwg_s != rwg_d) ? C_REPLY : 3'd0) |
                     (((hen && w_end && ovf) || (rdt_s ^ rdt_d)) ? C_DROP : 3'd0);
assign      irq    = |cause;

reg         we;
reg  [AW-1:0] wa;
reg  [19:0] wd;
always @(*) begin
	we = 0;
	wa = wspec[AW-1:0];
	wd = {K_WORD, 2'b00, wdata[15:0]};
	if (w_beg) begin
		we = b_ok;
		wa = wcom[AW-1:0];
		wd = {wdata[18] ? K_BEGINC : K_BEGIN, wdata[17:16], wdata[15:0]};
	end
	else if (w_wrd) we = w_ok;
	else if (w_end) begin
		we = ~ovf;
		wd = {K_END, 18'd0};
	end
end

always @(posedge clk) if (we) fram[wa] <= wd;
always @(posedge clk) rhead <= rram[rrp[RAW-1:0]];

always @(posedge clk) begin
	if (wvis != wcom) wvis <= wvis + 1'd1;
	lane_w_lwgray <= wvis ^ (wvis >> 1);

	pt_d <= pt_s;
	if (pt_s ^ pt_d) played <= played + 1'd1;

	if (w_pop || (!hen && rcount != 0)) rrp <= rrnext;
	lane_rr_lwgray <= rrp ^ (rrp >> 1);
	rdt_d <= rdt_s;
	if (rdt_s ^ rdt_d) rdrops <= rdrops + 1'd1;
	rwg_d <= rwg_s;
	cause <= raise | (cause & ~clr & ~{3{take}});   // a raise beats a clear in the same cycle; an ENABLE clears

	// BEGIN abandons any open frame: its entries past wcom are overwritten; so does the enable's fall
	if (!hen) begin
		open  <= 0;
		ovf   <= 0;
		wspec <= wcom;
	end
	else if (w_beg) begin
		open  <= 1;
		ovf   <= ~b_ok;
		wspec <= b_ok ? wcom + 1'd1 : wcom;
	end
	else if (w_wrd) begin
		if (w_ok) wspec <= wspec + 1'd1;
		else ovf <= 1;
	end
	else if (w_end) begin
		open <= 0;
		if (ovf) drops <= drops + 1'd1;
		else wcom <= wspec + 1'd1;
	end
end

// The enable link. Drain: no frame open, every committed entry visible to clk_sys. Last condition before IDLE:
// the reply FIFO stepped to empty.
wire        rest, hen_s;
wire        f_drain = !open && wspec == wcom && wvis == wcom && lane_w_lwgray == (wvis ^ (wvis >> 1));
lw_fence fence (.clk(clk), .hen(hen), .take(take), .drain(f_drain), .fin(rcount == 0),
                .open(f_open), .owned(f_owned), .idle(f_idle), .lvl(), .fall(),
                .clk_sys(clk_sys), .rest(rest), .hen_s(hen_s));

always @(*) begin
	case (raddr[11:2])
		10'd4:   rdata = {16'd0, played};
		10'd5:   rdata = {16'd0, drops};
		10'd6:   rdata = 32'h314E414C;    // "LAN1"
		10'd7:   rdata = {rdrops, {(15 - RAW){1'b0}}, rcount};
		10'd8:   rdata = (rcount != 0) ? {16'd0, rhead} : 32'd0;
		10'd9:   rdata = {29'd0, cause};
		default: rdata = 32'd0;
	endcase
end

/////////////////////////////// clk_sys ///////////////////////////////////

wire [AW:0]  wg_s;
wire [RAW:0] rrg_s;
lw_sync #(.W(AW + 1), .GRAY(1)) wg_sync (.clk(clk_sys), .d(lane_w_lwgray), .q(wg_s));
lw_sync #(.W(RAW + 1), .GRAY(1)) rrg_sync (.clk(clk_sys), .d(lane_rr_lwgray), .q(rrg_s));
reg  [AW:0] rptr  = 0;
reg  [19:0] q     = 0;
reg         fresh = 0;                      // q holds fram[rptr]
reg         capf  = 0;                      // the frame playing records its replies
reg [RAW:0] rwp   = 0;
reg   [1:0] ml    = 0;                      // cycles the FIFO has read empty with the enable level low, to 2

wire [AW:0] rnext = rptr + 1'd1;
wire        empty = rptr == g2b(wg_s);
wire        avail = fresh && lane_r_lwgray != wg_s;
wire  [1:0] qk    = q[19:18];
wire        s_idle, s_beg, s_stb, s_ack, s_nxt, s_rel;
// the link words: a frame begins at a BEGIN entry (its first link word) and ends at its END, or at a word that
// cannot come (the enable low, the FIFO empty for two cycles)
lw_link_seq #(.GAP(GAP)) seq
(
	.clk_sys(clk_sys), .l_busy(l_busy), .l_wait(l_wait),
	.start(avail && qk[0] && hen_s), .fsel(q[17:16]), .more(avail && qk == K_WORD),
	.stop(avail || (!hen_s && empty && ml == 2'd2)),
	.own(own), .hold(hold), .sel(sel), .strobe(strobe),
	.idle(s_idle), .beg(s_beg), .stb(s_stb), .ack(s_ack), .nxt(s_nxt), .rel(s_rel)
);
wire        t_odd = s_idle && avail && !qk[0] && hen_s;   // a stray entry
wire        t_dis = s_idle && !empty && !hen_s;            // without the enable: every entry, every cycle
wire        t_end = s_nxt && avail && qk == K_END;
wire        pop   = t_odd | t_dis | s_stb | t_end;   // q still holds the entry through its strobe cycle

assign rest = !hen_s && s_idle && !own && !hold && !strobe && sel == 4'd0 && empty &&
              lane_r_lwgray == (rptr ^ (rptr >> 1)) && lane_rw_lwgray == (rwp ^ (rwp >> 1));
wire        r_push = s_ack & capf;
wire [RAW:0] r_used = rwp - g2r(rrg_s);
wire        r_full = r_used[RAW];
wire [RAW:0] rwnext = rwp + 1'd1;

assign din = q[15:0];

always @(posedge clk_sys) if (r_push & ~r_full) rram[rwp[RAW-1:0]] <= l_dout;

always @(posedge clk_sys) begin
	q     <= fram[rptr[AW-1:0]];
	fresh <= ~pop;
	if (pop) rptr <= rnext;
	lane_r_lwgray <= rptr ^ (rptr >> 1);
	ml    <= (hen_s || !empty) ? 2'd0 : (ml == 2'd2) ? 2'd2 : ml + 2'd1;

	// the reply to the strobe just acknowledged; the gray pointer follows rwp a cycle later, after the word is in
	// the RAM
	if (r_push) begin
		if (r_full) rdt_lwx <= ~rdt_lwx;
		else rwp <= rwnext;
	end
	lane_rw_lwgray <= rwp ^ (rwp >> 1);

	if (s_beg) capf <= qk[1];
	if (s_rel) pt_lwx <= ~pt_lwx;
end

endmodule
`endif

// The link's ack path (sys_top's rack/io_ack, unchanged) and the mux that hands the link nets to the lane
// player: while it holds them the link's strobe and ack stay frozen, so a link word is delayed, never lost.
/* verilator lint_off DECLFILENAME */   // lw_link and lw_link_seq share lw_lane's file: either floor builds them
module lw_link
(
	input wire         clk_sys,
	input wire   [3:0] l_sel,        // the link's selects {osd_vga, osd_hdmi, fpga, uio}, as sys_top decodes them
	input wire         l_clk,
	input wire  [15:0] l_din,
	input wire         l_wait,       // io_wait | vs_wait

	input wire         p_own,
	input wire         p_hold,
	input wire   [3:0] p_sel,
	input wire         p_strobe,
	input wire  [15:0] p_din,

	output wire  [3:0] io_sel,
	output wire        io_strobe,
	output wire [15:0] io_din,
	output reg         io_ack = 0
);

reg  rack = 0;
wire l_strobe = ~rack & l_clk;

assign io_sel    = p_own ? p_sel : l_sel;
assign io_strobe = p_own ? p_strobe : ~p_hold & l_strobe;
assign io_din    = p_own ? p_din : l_din;

always @(posedge clk_sys) begin
	if (~p_hold & (~l_wait | l_strobe)) begin
		rack   <= l_clk;
		io_ack <= rack;
	end
end

endmodule

// The link-word sequencer of a player (lw_lane's, and the OCRAM download tier's hps_ocram_fplay): it takes the link
// nets between link transactions and strobes one link word at a time, the player's source supplying the words. A
// frame begins once start is up, both selects have been low for two cycles and the link is not busy: own and hold
// rise with the select (fsel). Each word is one strobe, then GAP consecutive cycles with l_wait low (ack); the next
// word's strobe follows if more, else the frame ends if stop: the select drops, own two cycles later, hold one after
// that (rel). The state strobes tell the player where it is; none of its inputs may read them.
module lw_link_seq #(parameter [2:0] GAP = 3'd4)   // 2..7
(
	input  wire       clk_sys,
	input  wire       l_busy,        // any of the link's own selects, or its ID view
	input  wire       l_wait,        // io_wait | vs_wait: the term that withholds the link's ack
	input  wire       start,         // a frame can begin (the enable in it)
	input  wire [1:0] fsel,          // its select, read with beg
	input  wire       more,          // read in nxt: the next link word is in hand
	input  wire       stop,          // read in nxt with more low: the frame ends
	output reg        own    = 1'b0, // the player drives the link nets
	output reg        hold   = 1'b0, // own, plus one cycle: the link's ack path stays frozen
	output reg  [3:0] sel    = 4'd0, // {osd_vga, osd_hdmi, fpga, uio}
	output reg        strobe = 1'b0,
	output wire       idle,          // no frame
	output wire       beg,           // one cycle: the frame begins
	output wire       stb,           // the strobe cycle
	output wire       ack,           // one cycle: the word is acknowledged
	output wire       nxt,           // the cycle after an ack
	output wire       rel            // the frame's last cycle
);

localparam [2:0] S_IDLE = 3'd0, S_STB = 3'd1, S_ACK = 3'd2, S_NEXT = 3'd3, S_GAP = 3'd4, S_REL = 3'd5;

reg   [2:0] ps    = S_IDLE;
reg   [2:0] cnt   = 3'd0;
reg   [1:0] quiet = 2'd0;            // cycles the link's selects have been low, saturating at 2

assign idle = ps == S_IDLE;
assign beg  = idle && start && quiet == 2'd2 && !l_busy;
assign stb  = ps == S_STB;
assign ack  = ps == S_ACK && !l_wait && cnt == GAP - 3'd1;
assign nxt  = ps == S_NEXT;
assign rel  = ps == S_REL;

always @(posedge clk_sys) begin
	quiet <= l_busy ? 2'd0 : (quiet == 2'd2) ? 2'd2 : quiet + 2'd1;

	case (ps)
		S_IDLE: if (beg) begin
			own  <= 1'b1;
			hold <= 1'b1;
			sel  <= 4'b0001 << fsel;
			ps   <= S_STB;
		end
		else begin
			own    <= 1'b0;
			hold   <= 1'b0;
			sel    <= 4'd0;
			strobe <= 1'b0;
		end
		S_STB: begin
			strobe <= 1'b1;
			cnt    <= 3'd0;
			ps     <= S_ACK;
		end
		S_ACK: begin
			strobe <= 1'b0;
			if (l_wait) cnt <= 3'd0;
			else if (ack) ps <= S_NEXT;
			else cnt <= cnt + 3'd1;
		end
		S_NEXT: if (more) ps <= S_STB;
		else if (stop) begin
			sel <= 4'd0;
			cnt <= 3'd0;
			ps  <= S_GAP;
		end
		S_GAP: if (cnt == 3'd1) begin
			own <= 1'b0;
			ps  <= S_REL;
		end
		else cnt <= cnt + 3'd1;
		S_REL: begin
			hold <= 1'b0;
			ps   <= S_IDLE;
		end
		default: begin
			own    <= 1'b0;
			hold   <= 1'b0;
			sel    <= 4'd0;
			strobe <= 1'b0;
			ps     <= S_IDLE;
		end
	endcase
end

endmodule
/* verilator lint_on DECLFILENAME */

`default_nettype wire
`endif
`undef LW_LANE_BUILT
`undef LW_LINK_BUILT
