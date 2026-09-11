// Copyright (c) 2026 Jamie Blanks

// U3 on the cartridge board, 8-bit comparator. !P=Q goes low when the inputs
// match and !G is low.

module ttl_74f688 (
	input  wire [7:0] P,
	input  wire [7:0] Q,
	input  wire       G_n,
	output wire       PEQ_n
);
	assign PEQ_n = G_n | (P != Q);
endmodule
