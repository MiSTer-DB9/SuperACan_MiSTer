// Copyright (c) 2026 Jamie Blanks

// The 4 KiB 68000 boot ROM inside the UM6619, overlaid at $000000 and
// $F80000 until the IPL switches each overlay off. Loaded from the HPS; the
// load port is only driven during a download.

module um6619_iplrom (
	input  wire        clk_sys,

	input  wire [10:0] addr,      // word address inside the 4 KiB
	output wire [15:0] q,

	input  wire        ld_wr,
	input  wire [10:0] ld_addr,
	input  wire [15:0] ld_data
);
	cache_ram #(.ADDR_WIDTH(11), .DATA_WIDTH(16)) mem
	(
		.clk_i   (clk_sys),
		.addr_i  (ld_wr ? ld_addr : addr),
		.wren_i  (ld_wr),
		.wdata_i (ld_data),
		.q_o     (q)
	);
endmodule
