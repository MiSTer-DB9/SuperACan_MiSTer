// Copyright (c) 2026 Jamie Blanks

// UM6618 register file: 256 sixteen-bit registers at $F00000.
//
// Every write is stored and every register reads back, including unmapped
// ones (software touches $F0019C, $F00110, $F00030, $F001C2 and $F001E0).
// The groups the renderers need are mirrored into flops and leave as one
// packed vector per block; the mirrors also answer reads so reset and DMA
// writeback stay visible to the CPU.
//
// $00 is vblank status, $02 the scanline, $04 the dot within the line. Boom
// Zoo and Super Taiwanese Baseball League read $04 as a varying quantity.
//
// No !UDS or !LDS pin, so every access is a whole word.

module um6618_regs (
	input  wire        clk_sys,
	input  wire        reset,

	input  wire        wr,
	input  wire [7:0]  index,      // byte offset / 2
	input  wire [15:0] din,
	output wire [15:0] dout,

	// From the video timing.
	input  wire        in_vblank,
	input  wire [8:0]  vpos,
	input  wire [9:0]  hpos,

	// Live mirrors for the rest of the chip.
	output reg  [15:0]  video_flags,   // $08
	output reg  [15:0]  raster_on,     // $0A
	output reg  [15:0]  raster_off,    // $0C
	output reg  [15:0]  pixel_mode,    // $1F0

	output reg  [511:0] tm_regs,       // $100/$120/$140/$160, eight words each
	output reg  [255:0] roz_regs,      // $180-$19E
	output reg  [127:0] spr_dma_regs,  // $10-$1E
	// The sprite DMA writes its final pointers back for software to read.
	input  wire         spr_dma_wb,
	input  wire [31:0]  spr_dma_dst_in,
	input  wire [31:0]  spr_dma_src_in,
	output reg  [63:0]  spr_regs,      // $20-$26
	output reg  [127:0] win_regs,      // $1D0-$1DE

	input  wire [63:0] ss_din,
	input  wire [9:0]  ss_addr,
	input  wire        ss_wren,
	input  wire        ss_rst,
	output wire [63:0] ss_dout
);
	`include "ss_map.svh"

	wire [15:0] store_q;
	wire [15:0] store_ss_q;
	wire ss_store_sel = (ss_addr >= 10'(SSW_6618_STORE))
	                  & (ss_addr < 10'(SSW_6618_STORE + 256));
	wire [7:0] ss_store_index = 8'(ss_addr - 10'(SSW_6618_STORE));

	// One low halfword per state word; the spare RAM port serves the state bus.
	wire [1151:0] ss_live = {win_regs, spr_regs, spr_dma_regs, roz_regs, tm_regs,
	                        pixel_mode, raster_off, raster_on, video_flags};
	wire [1151:0] ss_restore;
	wire [1151:0] ss_words;
	genvar word_i;
	generate
		for (word_i = 0; word_i < 18; word_i = word_i + 1) begin : state_words
			ss_reg #(.ADDR(SSW_6618_REGS + word_i)) word (
				.clk_i(clk_sys), .bus_din_i(ss_din), .bus_addr_i(ss_addr),
				.bus_wren_i(ss_wren), .bus_rst_i(ss_rst),
				.bus_dout_o(ss_words[word_i*64 +: 64]),
				.din_i(ss_live[word_i*64 +: 64]),
				.dout_o(ss_restore[word_i*64 +: 64])
			);
		end
	endgenerate
	reg [63:0] ss_scalar_dout;
	integer state_i;
	always @(*) begin
		ss_scalar_dout = 64'd0;
		for (state_i = 0; state_i < 18; state_i = state_i + 1)
			ss_scalar_dout = ss_scalar_dout | ss_words[state_i*64 +: 64];
	end
	assign ss_dout = ss_scalar_dout | (ss_store_sel ? {48'd0, store_ss_q} : 64'd0);

	// $00, $02 and $04 come from the beam. Mirrors answer wherever reset or
	// DMA can change a register without a CPU write.
	assign dout = (index == 8'h00)        ? {in_vblank, 15'd0}
	            : (index == 8'h01)        ? {7'd0, vpos}
	            : (index == 8'h02)        ? {6'd0, hpos}
	            : (index == 8'h04)        ? video_flags
	            : (index == 8'h05)        ? raster_on
	            : (index == 8'h06)        ? raster_off
	            : (index == 8'hF8)        ? pixel_mode
	            : is_tm                  ? tm_regs[{index[5:4], index[2:0], 4'd0} +: 16]
	            : is_roz                 ? roz_regs[{index[3:0], 4'd0} +: 16]
	            : is_win                 ? win_regs[{index[2:0], 4'd0} +: 16]
	            : is_spr                 ? spr_regs[{index[1:0], 4'd0} +: 16]
	            : is_sdm                 ? spr_dma_regs[{index[2:0], 4'd0} +: 16]
	                                      : store_q;


	cache_ram_dp_be #(.ADDR_WIDTH(8), .DATA_WIDTH(16)) store
	(
		.clk_i     (clk_sys),
		.addr_a_i  (index),
		.wren_a_i  (wr),
		.be_a_i    (2'b11),
		.wdata_a_i (din),
		.q_a_o     (store_q),
		.addr_b_i  (ss_store_index),
		.wren_b_i  (ss_store_sel & ss_wren),
		.be_b_i    (2'b11),
		.wdata_b_i (ss_din[15:0]),
		.q_b_o     (store_ss_q)
	);

	// Where each renderer's register block sits in the flat index space.
	//   $10-$1E  sprite DMA        index $08-$0F
	//   $20-$26  sprite            index $10-$13
	//   $100+    tilemap 0-3       index $80+layer*$10, first eight words
	//   $180+    ROZ               index $C0-$CF
	//   $1D0+    windows 0 and 1   index $E8-$EF
	wire is_tm  = (index[7:6] == 2'b10) & ~index[3];
	wire is_roz = (index[7:4] == 4'hC);
	wire is_win = (index[7:3] == 5'b11101);
	wire is_spr = (index[7:2] == 6'b000100);
	wire is_sdm = (index[7:3] == 5'b00001);

	always @(posedge clk_sys) begin
		if (reset) begin
			video_flags  <= ss_restore[15:0];
			raster_on    <= ss_restore[31:16];
			raster_off   <= ss_restore[47:32];
			pixel_mode   <= ss_restore[63:48];
			tm_regs      <= ss_restore[575:64];
			roz_regs     <= ss_restore[831:576];
			spr_dma_regs <= ss_restore[959:832];
			spr_regs     <= ss_restore[1023:960];
			win_regs     <= ss_restore[1151:1024];
		end else if (wr) begin
			case (index)
				8'h04:   video_flags <= din;   // $08
				8'h05:   raster_on   <= din;   // $0A
				8'h06:   raster_off  <= din;   // $0C
				8'hF8:   pixel_mode  <= din;   // $1F0
				default: ;
			endcase

			if (is_tm)  tm_regs     [{index[5:4], index[2:0], 4'd0} +: 16]  <= din;
			if (is_roz) roz_regs    [{index[3:0], 4'd0} +: 16]              <= din;
			if (is_win) win_regs    [{index[2:0], 4'd0} +: 16]              <= din;
			if (is_spr) spr_regs    [{index[1:0], 4'd0} +: 16]              <= din;
			if (is_sdm) spr_dma_regs[{index[2:0], 4'd0} +: 16]              <= din;
		end else if (spr_dma_wb) begin
			spr_dma_regs[31:16] <= spr_dma_dst_in[31:16];
			spr_dma_regs[47:32] <= spr_dma_dst_in[15:0];
			spr_dma_regs[79:64] <= spr_dma_src_in[31:16];
			spr_dma_regs[95:80] <= spr_dma_src_in[15:0];
		end
	end
endmodule
