//============================================================================
//  ocram_blk_req.sv -- part of the OCRAM mailbox (docs/OCRAM_MAILBOX.md)
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
`define LW_OCRAM_BLK_REQ_BUILT
`endif
`ifdef MISTER_OCRAM_SECTOR_SLOT
`define LW_OCRAM_BLK_REQ_BUILT
`endif
`ifdef MISTER_OCRAM_FIO
`define LW_OCRAM_BLK_REQ_BUILT
`endif
`ifdef LW_OCRAM_BLK_REQ_BUILT
`default_nettype none

// The block slots' request records, on clk_sys, by slot index k (0 the disc slot, 1 and 2 the mount slots), one
// rule for each: offered = the slot's mux here, its request raised, its ack low and two cycles past its fall (a
// mount slot's also a block of at most MFRAME words); each change of offered marks the slot's record to send, and a
// change in a launch cycle stays marked for the next. The disc slot's record {offered, lba} has its own channel;
// the mount slots' records {k, wr, blk_cnt, offered, lba} share one, in turn when both are marked, and none
// launches while a mount slot's ack is up.

module ocram_blk_req #(parameter SEC_EN = 0, parameter [1:0] MNT_EN = 2'd0, parameter [31:0] MFRAME = 32'd128,
                       parameter [2:0] BLKSZ = 3'd2)
(
	input  wire        clk_sys,
	input  wire        x_en,         // the disc slot: its mux here, the core's request, the LBA, the acks
	input  wire        x_req,
	input  wire [31:0] x_lba,
	input  wire        x_ack,
	input  wire        x_hps_ack,
	/* verilator lint_off UNUSEDSIGNAL */   // without MNT_EN the mount slots' inputs are not read
	input  wire  [1:0] xm_en,        // the mount slots, {k = 2, k = 1}
	input  wire  [1:0] xm_rd,
	input  wire  [1:0] xm_wr,
	input  wire [63:0] xm_lba,
	input  wire [11:0] xm_cnt,
	input  wire  [1:0] xm_ack,
	input  wire  [1:0] xm_hps_ack,
	/* verilator lint_on UNUSEDSIGNAL */
	output wire        rc_go,        // the disc slot's channel
	output wire [32:0] rc_d,
	input  wire        rc_busy,
	output wire        mrc_go,       // the mount slots' channel
	output wire [41:0] mrc_d,
	input  wire        mrc_busy,
	output wire        rest          // nothing offered, nothing to send
);

function automatic [18:0] mwords(input [5:0] cnt);
	mwords = ({13'd0, cnt} + 19'd1) << ({1'b0, BLKSZ} + 4'd5);
endfunction

wire  [2:0] has  = {MNT_EN, SEC_EN != 0};
wire  [2:0] en   = {xm_en, x_en};
wire  [2:0] up   = {xm_rd | xm_wr, x_req};
wire  [2:0] ack  = {xm_ack, x_ack};
wire  [2:0] hack = {xm_hps_ack, x_hps_ack};
wire  [2:0] fit  = {mwords(xm_cnt[11:6]) <= MFRAME[18:0], mwords(xm_cnt[5:0]) <= MFRAME[18:0], 1'b1};
reg   [2:0] offd  = 3'd0;              // offered, registered
reg   [2:0] dirty = 3'd0;              // the record is to send
reg   [5:0] rearm = 6'd0;              // the ack, one and two cycles back
reg         mlast = 1'b0;              // the mount slot launched last: 0 is k = 1, 1 is k = 2
wire  [2:0] calm = {rearm[5:4] == 2'd0, rearm[3:2] == 2'd0, rearm[1:0] == 2'd0};
wire  [2:0] off  = has & en & up & ~ack & calm & fit;
wire        mrs  = (dirty[2:1] == 2'b11) ? ~mlast : dirty[2];   // the mount slot to launch
assign rc_go  = dirty[0] && !rc_busy;
assign rc_d   = {offd[0], x_lba};
assign mrc_go = dirty[2:1] != 2'd0 && !mrc_busy && xm_ack == 2'd0;
assign mrc_d  = mrs ? {2'd2, xm_wr[1], xm_cnt[11:6], offd[2], xm_lba[63:32]} :
                      {2'd1, xm_wr[0], xm_cnt[5:0], offd[1], xm_lba[31:0]};
wire  [2:0] sent = {mrc_go & mrs, mrc_go & ~mrs, rc_go};
assign rest   = offd == 3'd0 && dirty == 3'd0;

always @(posedge clk_sys) begin
	offd  <= off;
	dirty <= (off ^ offd) | (dirty & ~sent);
	rearm <= {rearm[4], ack[2] | hack[2], rearm[2], ack[1] | hack[1], rearm[0], ack[0] | hack[0]};
	if (mrc_go) mlast <= mrs;
end

endmodule

`default_nettype wire
`endif
`undef LW_OCRAM_BLK_REQ_BUILT
`endif
