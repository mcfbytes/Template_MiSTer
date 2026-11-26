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
`define LW_BLK_MUX_BUILT
`endif
`ifdef MISTER_OCRAM_SECTOR_SLOT
`define LW_BLK_MUX_BUILT
`endif
`ifdef LW_BLK_MUX_BUILT
`default_nettype none

// emu side of the LW SECTOR route (lw_sector.sv, or the OCRAM mailbox's feeder in hps_ocram_rx.sv with
// MISTER_OCRAM_SECTOR_SLOT): one hps_io read slot's consumer is fed either by hps_io or, while sys_top's en_sys is
// set, by the LW feeder.

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

`ifdef MISTER_LW_SECTOR_SLOT
`define LW_SAVE_MUX_BUILT
`endif
`ifdef MISTER_OCRAM_MOUNT0_SLOT
`define LW_SAVE_MUX_BUILT
`endif
`ifdef MISTER_OCRAM_MOUNT1_SLOT
`ifndef LW_SAVE_MUX_BUILT
`define LW_SAVE_MUX_BUILT
`endif
`endif
// LW_SAVE_MUX_DIN: the OCRAM mount slots are built (a mount macro with MISTER_OCRAM and its SECTOR tier), so the
// LBA field carries core_din under the ack; a mount macro left in a build without them changes nothing here.
`ifdef MISTER_OCRAM
`ifdef MISTER_OCRAM_SECTOR_SLOT
`ifdef MISTER_OCRAM_MOUNT0_SLOT
`define LW_SAVE_MUX_DIN
`endif
`ifdef MISTER_OCRAM_MOUNT1_SLOT
`ifndef LW_SAVE_MUX_DIN
`define LW_SAVE_MUX_DIN
`endif
`endif
`endif
`endif
`ifdef LW_SAVE_MUX_BUILT
// emu side of a save slot on the LW SECTOR route (MISTER_LW_SECTOR_SLOT), or of a mount slot on the OCRAM one
// (MISTER_OCRAM_MOUNT0_SLOT, _MOUNT1_SLOT): SAVE 0 is the slot MISTER_LW_SAVE0_SLOT or MISTER_OCRAM_MOUNT0_SLOT
// declares (the slot index 1), SAVE 1 the MISTER_LW_SAVE1_SLOT or MISTER_OCRAM_MOUNT1_SLOT one (index 2).

module lw_save_mux #(parameter SAVE = 0)
(
	inout wire [383:0] HPS_BUS,

	input wire         core_rd,
	input wire         core_wr,
	input wire  [31:0] core_lba,    // kept, crossed and compared on the slot's MISTER_LW_SAVE<n>_LBA_BITS
	/* verilator lint_off UNUSEDSIGNAL */   // read on the OCRAM route only
	input wire  [15:0] core_din,    // the core's buffer word at core_addr (its sd_buff_din)
	/* verilator lint_on UNUSEDSIGNAL */
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

`ifdef LW_SAVE_MUX_DIN
assign HPS_BUS[U + 34:U]     = {(en & ack) ? {16'd0, core_din} : core_lba, core_rd, core_wr, hps_ack};
// The ack answers only the request that was up when it rose.
reg  fell = 1'b0;
always @(posedge HPS_BUS[36]) fell <= en & ack & (fell | ~(core_rd | core_wr));
wire cack = ack & ~(fell & (core_rd | core_wr));
`else
assign HPS_BUS[U + 34:U]     = {core_lba, core_rd, core_wr, hps_ack};
wire cack = ack;
`endif
assign HPS_BUS[378 + SAVE]   = 1'b1;

assign hps_rd     = core_rd & ~en;
assign hps_wr     = core_wr & ~en;
assign core_ack   = en ? cack                 : hps_ack;
assign core_write = en ? HPS_BUS[97] & cack   : hps_write;
assign core_addr  = en ? HPS_BUS[127:114]     : hps_addr;
assign core_data  = en ? HPS_BUS[113:98]      : hps_data;

endmodule
`endif
`ifdef LW_SAVE_MUX_DIN
`undef LW_SAVE_MUX_DIN
`endif
`ifdef LW_SAVE_MUX_BUILT
`undef LW_SAVE_MUX_BUILT
`endif

`default_nettype wire
`endif
`undef LW_BLK_MUX_BUILT
`endif
