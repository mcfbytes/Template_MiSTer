//============================================================================
//  lw_osd.sv -- part of the LW bridge framework (docs/LW_BRIDGE.md)
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
`ifdef MISTER_LW_OSD
`default_nettype none

// OSDBUF block (LW directory kind 3, version 2.0, "OSD2"), built with MISTER_LW_OSD: image stores written into
// each osd's own osd_buffer. A store to IMAGE (byte a = osd_buffer[a]: row a[12:8], column a[7:0], ROWS rows) takes
// one entry {be, word, data} of a 2^AW-entry dual-clock FIFO; a store that finds it full is dropped and counted.
// The writer on clk_sys gives each osd one enabled byte per cycle on {we, addr, dbyte}, and only while the link
// is quiet for it: both osd selects low and the I/O view selected (view, GPO[31] on clk_sys), in this cycle and
// the two before it. An SPI OSD write needs its select high, so the two never meet; osd.v takes the SPI byte if
// both come. The read pointer advances once a word's last enabled byte is written: STATUS's level 0 means every
// stored byte is in osd_buffer. Reads: STATUS {level, drops} and ID; every other word reads 0.
// The host enable (hen, the window's en_q) reaches the writer only through the block's enable link (lw_fence,
// lw_cdc.sv): stores are taken while the link is open, which is after the writer has seen the enable. While its
// level is low the writer pops an entry in every cycle and writes nothing, so a byte stored by a host whose
// enable has fallen never lands, whenever it reaches the head; its rest, which the link acknowledges, is the level
// low, the FIFO empty and no word part written. From any state the pointers converge: with the level low every
// entry drains, and a read pointer more than the FIFO's depth behind the write pointer takes it at once.
// Crossings, each through lw_sync (lw_cdc.sv): the FIFO's two Gray pointers, each recomputed from its binary pointer
// in every cycle, and the enable link's two levels. Bridge side has no reset and answers every store in fixed cycles.

module lw_osd #(parameter ROWS = 16, parameter AW = 8)   // ROWS: 16, or 20 with MENU_CORE's header rows
(
	input wire         clk,          // clk_100m

	input wire  [12:0] raddr,        // block-relative
	output reg  [31:0] rdata,
	input wire  [12:0] waddr,
	input wire         wsel,
	input wire         write,        // the window's gated store
	input wire  [31:0] wdata,
	input wire   [3:0] be,
	input wire         hen,          // the host enable
	input wire         take,         // an ENABLE store is taken in this cycle
	output wire        f_open,       // the enable link: stores are taken (OPEN)
	output wire        f_owned,      // OWNED
	output wire        f_idle,       // the link lets an ENABLE through

	input wire         clk_sys,
	input wire   [1:0] sel,          // both osd selects as each osd sees them, {vga, hdmi}
	input wire         view,         // GPO[31]: 1 = the link's I/O view, 0 = its ID view
	output wire        we,           // one byte to every osd's osd_buffer
	output wire [12:0] addr,
	output wire  [7:0] dbyte
);

localparam        DEPTH = 1 << AW;
localparam [10:0] WORDS = ROWS * 64;

function automatic [AW:0] g2b(input [AW:0] g);
	integer n;
	begin
		g2b[AW] = g[AW];
		for (n = AW - 1; n >= 0; n = n - 1) g2b[n] = g2b[n + 1] ^ g[n];
	end
endfunction

// {be[3:0], word[10:0], data[31:0]}, written on clk_100m, read on clk_sys
(* ramstyle = "M10K" *) reg [46:0] fram [0:DEPTH-1];

/////////////////////////////// clk_100m //////////////////////////////////

reg  [AW:0] wptr  = 0;
(* preserve *) reg [AW:0] osd_w_lwgray = 0;
(* preserve *) reg [AW:0] osd_r_lwgray = 0;   // clk_sys
wire [AW:0] rg_s;
reg  [15:0] drops = 0;                       // stores dropped on a full FIFO, free-running

lw_sync #(.W(AW + 1), .GRAY(1)) rg_sync (.clk(clk), .d(osd_r_lwgray), .q(rg_s));

wire [AW:0] level = wptr - g2b(rg_s);        // entries waiting: 0..DEPTH
wire        full  = level[AW];
wire        st    = wsel & write & |be && waddr[12:2] < WORDS;

always @(posedge clk) if (st && !full) fram[wptr[AW-1:0]] <= {be, waddr[12:2], wdata};

always @(posedge clk) begin
	if (st) begin
		if (full) drops <= drops + 1'd1;
		else wptr <= wptr + 1'd1;
	end
	osd_w_lwgray <= wptr ^ (wptr >> 1);
end

always @(*) begin
	case (raddr[12:2])
		11'h600: rdata = {{(15 - AW){1'b0}}, level, drops};   // STATUS: {level[24:16], drops[15:0]}
		11'h601: rdata = 32'h3244534F;                        // "OSD2"
		default: rdata = 32'd0;
	endcase
end

// The enable link. Drain: every entry stored is visible to clk_sys. No last condition.
wire        rest, hen_s;
wire        f_drain = osd_w_lwgray == (wptr ^ (wptr >> 1));
lw_fence fence (.clk(clk), .hen(hen), .take(take), .drain(f_drain), .fin(1'b1),
                .open(f_open), .owned(f_owned), .idle(f_idle), .lvl(), .fall(),
                .clk_sys(clk_sys), .rest(rest), .hen_s(hen_s));

/////////////////////////////// clk_sys ///////////////////////////////////

wire [AW:0] wg_s;
lw_sync #(.W(AW + 1), .GRAY(1)) wg_sync (.clk(clk_sys), .d(osd_w_lwgray), .q(wg_s));
reg  [AW:0] rptr  = 0;
reg  [46:0] q     = 0;                       // fram[rptr], the head entry
reg         fresh = 0;                       // q holds fram[rptr]
reg   [3:0] left  = 0;                       // the head word's enabled bytes not yet written
reg         hd    = 0;                       // left holds them: the head word is part written
reg   [1:0] qn    = 0;                       // cycles quiet before this one, saturating at 2

wire [AW:0] used  = g2b(wg_s) - rptr;
wire        empty = used == 0;
wire        over  = used > DEPTH;            // only after an upset: the read pointer takes the write pointer
wire        quiet = sel == 2'b00 && view;
wire  [3:0] m     = hd ? left : q[46:43];
wire  [1:0] b     = m[0] ? 2'd0 : m[1] ? 2'd1 : m[2] ? 2'd2 : 2'd3;
wire  [3:0] m_n   = m & ~(4'd1 << b);
wire        avail = fresh && !empty && !over;
wire        t_wr  = hen_s && avail && m != 4'd0 && quiet && qn == 2'd2;
wire        t_nil = hen_s && avail && m == 4'd0;  // an entry with no byte, which only an upset leaves
wire        t_dis = !hen_s && !empty && !over;     // without the enable: every entry, every cycle
wire        pop   = t_dis | t_nil | (t_wr && m_n == 4'd0);

assign we    = t_wr;
assign addr  = {q[42:32], b};
assign dbyte = q[{b, 3'd0} +: 8];

assign rest = !hen_s && empty && !hd && osd_r_lwgray == (rptr ^ (rptr >> 1));

always @(posedge clk_sys) begin
	q     <= fram[rptr[AW-1:0]];
	fresh <= ~pop & ~over;
	if (over) rptr <= g2b(wg_s);
	else if (pop) rptr <= rptr + 1'd1;
	osd_r_lwgray <= rptr ^ (rptr >> 1);
	qn <= !quiet ? 2'd0 : (qn == 2'd2) ? 2'd2 : qn + 2'd1;
	if (!hen_s || pop || over) hd <= 0;
	else if (t_wr) begin
		hd   <= 1;
		left <= m_n;
	end
end

endmodule

`default_nettype wire
`endif
`endif
