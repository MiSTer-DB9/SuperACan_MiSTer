// Copyright (c) 2026 Jamie Blanks

// The 16 KiB of sample data inside the UM6619, copied into sound RAM at
// reset: the first 8 KiB to $0000 and the second to $2000. Byte wide for the
// 65C02; the low half of each loaded word is the even byte.

module um6619_smprom (
	input  wire        clk_sys,

	input  wire [13:0] addr,      // byte address inside the 16 KiB
	output wire [7:0]  q,

	input  wire        ld_wr,
	input  wire [12:0] ld_addr,   // word address
	input  wire [15:0] ld_data
);
	wire [7:0] q_lo, q_hi;

	assign q = addr[0] ? q_hi : q_lo;

	cache_ram #(.ADDR_WIDTH(13), .DATA_WIDTH(8)) even_bytes
	(
		.clk_i   (clk_sys),
		.addr_i  (ld_wr ? ld_addr : addr[13:1]),
		.wren_i  (ld_wr),
		.wdata_i (ld_data[7:0]),
		.q_o     (q_lo)
	);

	cache_ram #(.ADDR_WIDTH(13), .DATA_WIDTH(8)) odd_bytes
	(
		.clk_i   (clk_sys),
		.addr_i  (ld_wr ? ld_addr : addr[13:1]),
		.wren_i  (ld_wr),
		.wdata_i (ld_data[15:8]),
		.q_o     (q_hi)
	);
endmodule
