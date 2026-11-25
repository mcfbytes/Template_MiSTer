//============================================================================
//  ocram_mnt_wr.sv -- part of the OCRAM mailbox (docs/OCRAM_MAILBOX.md)
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
`define LW_OCRAM_MNT_WR_BUILT
`endif
`ifdef MISTER_OCRAM_SECTOR_SLOT
`define LW_OCRAM_MNT_WR_BUILT
`endif
`ifdef MISTER_OCRAM_FIO
`define LW_OCRAM_MNT_WR_BUILT
`endif
`ifdef LW_OCRAM_MNT_WR_BUILT
`default_nettype none

// A mount slot's WRITE, on clk_100m: core words pulled through the walker's one forward cell, handed to the master.
// It reads no ring, so it is the walker's part, not a walker state: the walker lends it the cell (W_MNT) from go to
// fin and keeps the one counter k (k_zero, k_step). Its head {SEC_HEAD, wr, slot, lba} crosses; accept raises the
// slot's ack and each of the mn words is fetched by one DATA token, held in m_wd (the answer's d) until the master
// takes it; after the head store (m_pub) the end token (d[0] = 1) drops the ack. A refusal, or the end token's
// answer, ends the WRITE for the slot's record (w_done) until the next record lands. The EVT ring's stop sends the
// end token at the next word. job, the ack up (post), the word held and the word outstanding are state decodes.

module ocram_mnt_wr
(
	input  wire        clk,
	input  wire        s_hello,
	input  wire        s_live,
	input  wire        go,           // one cycle: the walker gives the cell to the WRITE of slot
	input  wire  [1:0] slot,         // the slot, 1 or 2, held by the walker for the WRITE
	input  wire [31:0] lba,          // its request's LBA
	input  wire [10:0] mn,           // its block's words
	input  wire [13:0] k,            // the walker's counter: the words taken
	input  wire        fwd_busy,
	input  wire        done_v,
	input  wire  [1:0] done_c,
	input  wire        evt_stop,
	input  wire        m_start,
	input  wire        m_wtake,
	input  wire        m_pub,
	input  wire  [1:0] land,         // one cycle: slot {2, 1}'s request record lands
	output wire        x_go,
	output wire [36:0] x_d,
	output wire        x_out,        // a word launched, its done not back
	output wire        k_zero,       // the walker's k <= 0
	output wire        k_step,       // the walker's k <= k + 1
	output wire        fin,          // the walker is idle next
	output wire        job,          // the WRITE is the master's record source
	output wire        post,         // its ack is up and the master has not begun its record
	output wire        have,         // m_wd holds a word for the master
	output wire  [1:0] w_done        // that slot's WRITE is over for its record
);

localparam [1:0] TG_SHEAD = 2'd1, TG_DATA = 2'd2;
localparam [1:0] A_ACCEPT = 2'd1;
localparam [2:0] M_IDLE = 3'd0, M_HGO = 3'd1, M_ANS = 3'd2, M_TOK = 3'd3, M_WD = 3'd4, M_WH = 3'd5, M_WP = 3'd6;

reg   [2:0] ms = M_IDLE;
reg         w_started = 1'b0;          // the master has begun its WRITE record
reg         m_end = 1'b0;              // the next token is the end token
reg   [1:0] w_done_q = 2'd0;

wire        ans = ms == M_ANS && done_v;
assign job    = ms != M_IDLE;
assign post   = job && ms != M_HGO && ms != M_ANS && !w_started;
assign have   = ms == M_WH;
assign x_out  = ms == M_HGO || ms == M_ANS || ms == M_WD;
assign x_go   = (ms == M_HGO || ms == M_TOK) && !fwd_busy && s_live;
assign x_d    = (ms == M_TOK) ? {TG_DATA, 3'd0, 31'd0, m_end} : {TG_SHEAD, 1'b1, slot, lba};
assign k_zero = ans && done_c == A_ACCEPT;
assign k_step = ms == M_WH && m_wtake;
assign fin    = (ans && done_c != A_ACCEPT) || (ms == M_WD && done_v && m_end);
assign w_done = w_done_q;

always @(posedge clk) begin
	w_done_q <= w_done_q & ~land;
	if (m_start && job) w_started <= 1'b1;   // once: the slot's next record may land before the ack's fall
	if (s_hello) begin ms <= M_IDLE; m_end <= 1'b0; w_done_q <= 2'd0; end
	else if (!s_live) ms <= M_IDLE;           // the session ended: abandoned
	else begin
		if (go) begin ms <= M_HGO; w_started <= 1'b0; m_end <= 1'b0; end
		case (ms)
		M_IDLE: ;
		M_HGO: if (!fwd_busy) ms <= M_ANS;        // x_go launches the head
		M_ANS: if (done_v) begin
			// accepted: the ack is up and the words follow; else it is over for this record
			if (done_c == A_ACCEPT) ms <= M_TOK;
			else begin w_done_q[slot == 2'd2] <= 1'b1; ms <= M_IDLE; end
		end
		M_TOK: if (!fwd_busy) ms <= M_WD;         // x_go launches the token
		M_WD: if (done_v) begin
			if (m_end) begin w_done_q[slot == 2'd2] <= 1'b1; ms <= M_IDLE; end   // ack down
			else if (evt_stop) begin m_end <= 1'b1; ms <= M_TOK; end
			else ms <= M_WH;
		end
		M_WH: if (m_wtake) ms <= (k + 14'd1 == {3'd0, mn}) ? M_WP : M_TOK;
		else if (evt_stop) begin m_end <= 1'b1; ms <= M_TOK; end
		M_WP: if (m_pub || evt_stop) begin m_end <= 1'b1; ms <= M_TOK; end   // published, or never: the ack falls
		default: ms <= M_IDLE;
		endcase
	end
end

endmodule

`default_nettype wire
`endif
`undef LW_OCRAM_MNT_WR_BUILT
`endif
