//============================================================================
//  ocram_evt_wr.sv -- part of the OCRAM mailbox (docs/OCRAM_MAILBOX.md)
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
`default_nettype none

// The EVT ring's record writer (hps_ocram_master.sv): its head and tail, its free words, and one record at a time
// from N record sources, source 0 first. A record is its header, its len payload words and the head store
// C_IDX[3] = head + len + 1 that publishes it.
//
//  * A source asks with post and holds hdr, the header as stored {kind, arg, len}; it opens in a session with the
//    ring not stopped (live), none open, and room: the ring's waiting words (head - tail) are at most 2,048 and
//    its free words exceed len. start is the header store's launch. Each payload word is offered by wv with wd and
//    taken by wtake, the launch of its store; k is the word's index (1..len), first is k = 1. pub is the head
//    store's done: the record is published.
//  * drop abandons the record open: its head is not stored. A head store already in flight still lands and
//    publishes; kill (an HPS reset's rise, a run's start) and ended (a run ends this cycle) make a done
//    publish nothing.
//  * The store's data and the room are registers or a mux of them: the head store's value (head_nx_q), whether
//    the next store is the head (last_q) and the free words (free_q) are computed when rec_k steps or the head or
//    the tail moves, ahead of their first read. The offset is one 16-bit add of registers (head_q + rec_k), and the
//    pick is one compare per variable-length source (free_q > len).
//  * Every address is C_IDX[3] or EVT_BASE + ((index & (EVT_WORDS - 1)) << 2). tb/static.sh holds it.

module ocram_evt_wr #(parameter integer N = 7)
(
	input  wire            clk,          // clk_100m
	input  wire            clr,          // the hello: head and tail to 0, the record abandoned
	input  wire            live,         // a session, the ring not stopped
	input  wire            drop,         // the record open is abandoned
	input  wire            kill,         // a store in flight publishes nothing
	input  wire            ended,        // a store done this cycle publishes nothing
	input  wire            t_set,        // the session read H_IDX[3]: t_d is the tail
	input  wire [15:0]     t_d,
	output wire [15:0]     head,         // the head, last published
	output wire [15:0]     tail,         // the tail, last read
	output wire [15:0]     head_nx,      // the open record's head store value
	output wire [15:0]     free,         // 2,048 - (head - tail), modulo 2^16
	output wire            act,          // a record is open
	output wire [10:0]     k,            // its next store: 0 header, 1..len payload, len + 1 head
	output wire            first,        // k is 1
	output wire  [2:0]     src,          // its source
	output wire            wfly,         // a header or payload store of it is in flight
	output wire            hfly,         // its head store is in flight

	// the record sources
	input  wire [N-1:0]    post,
	input  wire [32*N-1:0] hdr,
	input  wire [N-1:0]    wv,
	input  wire [32*N-1:0] wd,
	output wire [N-1:0]    start,
	output wire [N-1:0]    wtake,
	output wire [N-1:0]    pub,

	// the bus request port (ocram_arb)
	output wire            req,
	output wire [15:0]     off,
	output wire [31:0]     wdata,
	output wire            counted,      // the head store (the DIAG counters time it)
	input  wire            grant,
	input  wire            done          // the engine's done
);

localparam [15:0] C_IDX3    = 16'h009C;    // fabric: the EVT ring's head
localparam [15:0] EVT_BASE  = 16'h4000;    // EVT word k at EVT_BASE + ((head + k) & 2047) * 4
localparam [15:0] EVT_WORDS = 16'd2048;

generate
	if (EVT_BASE + EVT_WORDS * 4 > 16'hF000) begin : g_evt_span
		ocram_evt_span_above_0xF000 guard();
	end
	if (C_IDX3 >= 16'h0100) begin : g_evt_ctrl
		ocram_evt_control_constant_above_0xFF guard();
	end
	if (N > 8) begin : g_evt_sources
		ocram_evt_more_than_8_sources guard();
	end
endgenerate

localparam [1:0] FL_NONE = 2'd0, FL_WORD = 2'd1, FL_HEAD = 2'd2;

reg  [15:0] head_q = 16'd0;
reg  [15:0] tail_q = 16'd0;
reg         rec_act = 1'b0;
reg  [10:0] rec_k = 11'd0;
reg  [10:0] rec_len_q = 11'd0;
reg   [2:0] src_q = 3'd0;
reg         first_q = 1'b0;            // rec_k == 1
reg         last_q = 1'b0;             // rec_k == rec_len_q + 1: the next store is the head
reg  [15:0] head_nx_q = 16'd0;         // head_q + rec_len_q + 1
reg  [15:0] free_q = EVT_WORDS;        // EVT_WORDS - (head_q - tail_q)
reg         room_q = 1'b1;             // head_q - tail_q <= EVT_WORDS
reg   [1:0] fly = FL_NONE;

wire [15:0] evt_index = head_q + {5'd0, rec_k};
wire [15:0] evt_off = EVT_BASE + ((evt_index & (EVT_WORDS - 1)) << 2);

// the open record's source's offer
reg         wv_s;
reg  [31:0] wd_s;
// the first source with a record and room for it
reg [N-1:0] nsel;
reg  [31:0] nhdr;
reg   [2:0] nsrc;
integer i;
always @(*) begin
	wv_s = 1'b0; wd_s = 32'd0;
	nsel = {N{1'b0}}; nhdr = 32'd0; nsrc = 3'd0;
	for (i = 0; i < N; i = i + 1)
		if (src_q == i[2:0]) begin wv_s = wv[i]; wd_s = wd[32*i +: 32]; end
	for (i = N - 1; i >= 0; i = i - 1)
		if (post[i] && room_q && free_q > hdr[32*i +: 16]) begin
			nsel = {{(N-1){1'b0}}, 1'b1} << i; nhdr = hdr[32*i +: 32]; nsrc = i[2:0];
		end
end

wire rec_go   = rec_act && (last_q || wv_s);
wire new_go   = live && !rec_act && nsel != {N{1'b0}};
wire take_rec = grant && rec_go;
wire take_new = grant && !rec_go;
wire pub_w    = done && !ended && fly == FL_HEAD;
wire [N-1:0] own = {{(N-1){1'b0}}, 1'b1} << src_q;

assign req     = rec_go || new_go;
assign off     = (rec_go && last_q) ? C_IDX3 : evt_off;
assign wdata   = rec_go ? (last_q ? {16'd0, head_nx_q} : wd_s) : nhdr;
assign counted = rec_go && last_q;
assign start   = take_new ? nsel : {N{1'b0}};
assign wtake   = (take_rec && !last_q) ? own : {N{1'b0}};
assign pub     = pub_w ? own : {N{1'b0}};
assign head    = head_q;
assign tail    = tail_q;
assign head_nx = head_nx_q;
assign free    = free_q;
assign act     = rec_act;
assign k       = rec_k;
assign first   = first_q;
assign src     = src_q;
assign wfly    = fly == FL_WORD;
assign hfly    = fly == FL_HEAD;

// the head and tail as they are after this edge: the room is ready the cycle either moves
wire [15:0] head_n = pub_w ? head_nx_q : clr ? 16'd0 : head_q;
wire [15:0] tail_n = t_set ? t_d : clr ? 16'd0 : tail_q;
wire [15:0] wait_n = head_n - tail_n;

always @(posedge clk) begin
	head_q <= head_n;
	tail_q <= tail_n;
	free_q <= EVT_WORDS - wait_n;
	room_q <= wait_n <= EVT_WORDS;

	if (done || kill) fly <= FL_NONE;
	if (grant) fly <= (rec_go && last_q) ? FL_HEAD : FL_WORD;

	if (take_new) begin
		rec_act   <= 1'b1;
		rec_k     <= 11'd1;
		rec_len_q <= nhdr[10:0];
		src_q     <= nsrc;
		first_q   <= 1'b1;
		last_q    <= nhdr[10:0] == 11'd0;
		head_nx_q <= head_q + {5'd0, nhdr[10:0]} + 16'd1;
	end
	else if (take_rec) begin
		rec_k   <= rec_k + 11'd1;
		first_q <= 1'b0;
		last_q  <= rec_k == rec_len_q;
	end
	if (pub_w || drop || clr) begin
		rec_act <= 1'b0;
		rec_k   <= 11'd0;
		first_q <= 1'b0;
		last_q  <= 1'b0;
	end
end

endmodule

`default_nettype wire
`endif
