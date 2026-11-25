//============================================================================
//  ocram_osd_wr.sv -- part of the OCRAM mailbox (docs/OCRAM_MAILBOX.md)
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
`define LW_OCRAM_OSD_WR_BUILT
`endif
`ifdef MISTER_OCRAM_SECTOR_SLOT
`define LW_OCRAM_OSD_WR_BUILT
`endif
`ifdef MISTER_OCRAM_FIO
`define LW_OCRAM_OSD_WR_BUILT
`endif
`ifdef LW_OCRAM_OSD_WR_BUILT
`default_nettype none

// The OSD writer, on clk_sys, a record sink (w_v, w_head, w_d; back a_go, rest): a ROW's head names the row
// and is answered at once; each DATA word waits in the crossing's capture register (w_d) while its four bytes are
// written, little-endian, one a cycle, under the OSDBUF guard (both osd selects low and the I/O view, in that
// cycle and the two before), and is answered after the fourth. A word met with the enable low is answered
// unwritten. While frozen (DIAG) nothing is written or answered.

module ocram_osd_wr
(
	input  wire        clk_sys,
	input  wire        hen_s,        // the session's enable level
	input  wire        frz,          // the DIAG freeze
	input  wire        w_v,          // one cycle: an entry is in w_d
	input  wire        w_head,       // it is a ROW's head (the row in w_d[20:16])
	/* verilator lint_off UNUSEDSIGNAL */   // a head carries the row alone
	input  wire [31:0] w_d,          // held until the entry is answered
	/* verilator lint_on UNUSEDSIGNAL */
	output wire        a_go,         // one cycle: the entry's answer (took)
	output wire        rest,         // no entry held

	// The OSDBUF write port every osd takes (osd.v)
	input  wire  [1:0] sel,          // both osd selects as each osd sees them, {vga, hdmi}
	input  wire        view,         // GPO[31]: 1 = the link's I/O view
	output wire        we,
	output wire [12:0] addr,
	output wire  [7:0] dbyte
);

reg         o_have = 1'b0;             // a DATA word to write
reg         o_hans = 1'b0;             // a header to answer
reg   [4:0] o_row = 5'd0;
reg   [5:0] o_word = 6'd0;
reg   [1:0] o_b = 2'd0;
reg   [1:0] qn = 2'd0;
wire        quiet = (sel == 2'b00) && view;
wire        wnow  = hen_s && !frz && o_have && quiet && qn == 2'd2;
wire        hans  = o_hans && !frz;
wire        wdrop = o_have && !frz && !hen_s;   // with the enable low: discarded, still answered
assign we    = wnow;
assign addr  = {o_row, o_word, o_b};
assign dbyte = w_d[{o_b, 3'd0} +: 8];
assign a_go  = hans || wdrop || (wnow && o_b == 2'd3);
assign rest  = !o_have && !o_hans;

always @(posedge clk_sys) begin
	qn <= !quiet ? 2'd0 : (qn == 2'd2) ? 2'd2 : qn + 2'd1;
	if (w_v) begin
		if (w_head) begin o_row <= w_d[20:16]; o_word <= 6'd0; o_hans <= 1'b1; end
		else begin o_have <= 1'b1; o_b <= 2'd0; end
	end
	// one answer a cycle
	if (hans) o_hans <= 1'b0;
	else if (wdrop) o_have <= 1'b0;
	else if (wnow) begin
		o_b <= o_b + 2'd1;
		if (o_b == 2'd3) begin o_have <= 1'b0; o_word <= o_word + 6'd1; end
	end
end

endmodule

`default_nettype wire
`endif
`undef LW_OCRAM_OSD_WR_BUILT
`endif
