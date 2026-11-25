//============================================================================
//  lw_blk_mux.sv -- part of the LW bridge framework (docs/LW_BRIDGE.md)
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
`ifdef MISTER_LW_SECTOR_SLOT
`default_nettype none

// emu side of the LW SECTOR route (lw_sector.sv): one hps_io read slot's consumer is fed either by hps_io
// or, while sys_top's en_sys is set, by the LW feeder. The consumer's ack/write/address/data are muxed here, never
// the shared sd_buff_wr, so an SPI transfer on another slot cannot land in an LW frame. With en_sys set the
// slot's request is hidden from hps_io, so the slot has one writer. All signals are on hps_io's clk_sys. The feeder's
// strobes are hps_io's: with WIDE = 1 two 16-bit strobes per word at the halfword index, with WIDE = 0 four 8-bit
// strobes (data [7:0]) at the byte index.
// HPS_BUS bits (docs/LW_BRIDGE.md): to the core [95] en_sys, [96] ack, [97] the shared strobe, [113:98] its data,
// [127:114] its address; from the core [166:132] {lba[31:0], req, hps_ack, reset} and [377], this mux's presence
// bit, a constant 1.

module lw_blk_mux
(
	inout wire [383:0] HPS_BUS,

	input wire         reset,       // the consumer's reset: zero-fills an open LW frame
	input wire         core_req,
	input wire  [31:0] core_lba,
	output wire        core_ack,
	output wire        core_wr,
	output wire [13:0] core_addr,
	output wire [15:0] core_data,

	output wire        hps_req,     // -> hps_io sd_rd[slot]
	input wire         hps_ack,     // <- hps_io sd_ack[slot]
	input wire         hps_wr,      // <- hps_io sd_buff_wr
	input wire  [13:0] hps_addr,    // <- hps_io sd_buff_addr
	input wire  [15:0] hps_data     // <- hps_io sd_buff_dout
);

wire en = HPS_BUS[95];

assign HPS_BUS[166:132] = {core_lba, core_req, hps_ack, reset};
assign HPS_BUS[377]     = 1'b1;

assign hps_req   = core_req & ~en;
assign core_ack  = en ? HPS_BUS[96]     : hps_ack;
assign core_wr   = en ? HPS_BUS[97] & HPS_BUS[96] : hps_wr;   // never a save-slot frame's strobe
assign core_addr = en ? HPS_BUS[127:114] : hps_addr;
assign core_data = en ? HPS_BUS[113:98] : hps_data;

endmodule

// emu side of a save slot on the LW SECTOR route: SAVE 0 is the slot MISTER_LW_SAVE0_SLOT declares (the block's
// slot index 1), SAVE 1 the MISTER_LW_SAVE1_SLOT one (index 2). While the slot's en_sys is set its rd/wr are hidden
// from hps_io and the slot's ack/write/addr/data come from the feeder, the write gated by the slot's own ack. The
// core's buffer read data keeps feeding hps_io's sd_buff_din (the SPI save path). All on hps_io's clk_sys.
// HPS_BUS bits: to the core [128 + 2 SAVE] en_sys, [129 + 2 SAVE] ack, [97] the shared strobe, [113:98] its data,
// [127:114] its address; from the core [201:167] (SAVE 0) or [236:202] (SAVE 1) {lba[31:0], rd, wr, hps_ack},
// and [378 + SAVE], this mux's presence bit, a constant 1.

module lw_save_mux #(parameter SAVE = 0)
(
	inout wire [383:0] HPS_BUS,

	input wire         core_rd,
	input wire         core_wr,
	input wire  [31:0] core_lba,    // kept, crossed and compared on the slot's MISTER_LW_SAVE<n>_LBA_BITS
	output wire        core_ack,
	output wire        core_write,
	output wire [13:0] core_addr,
	output wire [15:0] core_data,

	output wire        hps_rd,      // -> hps_io sd_rd[slot]
	output wire        hps_wr,      // -> hps_io sd_wr[slot]
	input wire         hps_ack,     // <- hps_io sd_ack[slot]
	input wire         hps_write,   // <- hps_io sd_buff_wr
	input wire  [13:0] hps_addr,    // <- hps_io sd_buff_addr
	input wire  [15:0] hps_data     // <- hps_io sd_buff_dout
);

localparam U = SAVE ? 202 : 167;

wire en  = HPS_BUS[128 + 2 * SAVE];
wire ack = HPS_BUS[129 + 2 * SAVE];

assign HPS_BUS[U + 34:U]     = {core_lba, core_rd, core_wr, hps_ack};
assign HPS_BUS[378 + SAVE]   = 1'b1;

assign hps_rd     = core_rd & ~en;
assign hps_wr     = core_wr & ~en;
assign core_ack   = en ? ack                  : hps_ack;
assign core_write = en ? HPS_BUS[97] & ack    : hps_write;
assign core_addr  = en ? HPS_BUS[127:114]     : hps_addr;
assign core_data  = en ? HPS_BUS[113:98]      : hps_data;

endmodule

`default_nettype wire
`endif
`endif
