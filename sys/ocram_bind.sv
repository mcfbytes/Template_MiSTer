//============================================================================
//  ocram_bind.sv -- part of the OCRAM mailbox (docs/OCRAM_MAILBOX.md)
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

`ifdef MISTER_OCRAM
`ifdef MISTER_OCRAM_OSD
`define LW_OCRAM_BIND_BUILT
`endif
`ifdef MISTER_OCRAM_SECTOR_SLOT
`define LW_OCRAM_BIND_BUILT
`endif
`ifdef MISTER_OCRAM_FIO
`define LW_OCRAM_BIND_BUILT
`endif
`ifdef LW_OCRAM_BIND_BUILT
`default_nettype none

// One slot's bind, on clk_100m: the level to clk_sys rises while the host asks (want) and falls only between
// records (hold); bound (C_BIND bit k) rises once en_sys reads back 1 at the level. When the ask goes, drop begins
// (drop_go) with the level held low; bound and drop fall once en_sys reads back 0 and no flush is owed (the disc
// slot's: every queued frame discarded first). The level register is the crossing's (ocram_xing): lvl_d is its
// next value, lvl its value. A session's start, or its end with the walker idle (clr), clears it all.

module ocram_bind
(
	input  wire        clk,
	input  wire        s_hello,
	input  wire        clr,
	input  wire        want,         // the host asks for the bind
	input  wire        hold,         // a record of the slot's ring is being walked
	input  wire        flush,        // the unbind's flush is owed
	input  wire        lvl,          // the bind level
	input  wire        en,           // en_sys, back
	output wire        lvl_d,
	output reg         bound = 1'b0,
	output reg         drop = 1'b0,
	output wire        drop_go       // one cycle: drop begins
);

assign lvl_d   = s_hello ? 1'b0 : (want && !drop) ? 1'b1 : hold ? lvl : 1'b0;
assign drop_go = bound && !want && !drop && !hold;

always @(posedge clk) begin
	if (lvl && en && !drop) bound <= 1'b1;
	if (drop_go) drop <= 1'b1;
	if (drop && !flush && !en) begin drop <= 1'b0; bound <= 1'b0; end
	if (s_hello || clr) begin bound <= 1'b0; drop <= 1'b0; end
end

endmodule

`default_nettype wire
`endif
`undef LW_OCRAM_BIND_BUILT
`endif
