// Copyright (c) 2026 Jamie Blanks

// U21, hex Schmitt-trigger inverter. One input sits on the R52/C43 RC and
// acts as a delay on the board; the RC values are unknown, so that gate is
// treated as instant here.

module ttl_74f14 (
	input  wire [5:0] I,
	output wire [5:0] O
);
	assign O = ~I;
endmodule
