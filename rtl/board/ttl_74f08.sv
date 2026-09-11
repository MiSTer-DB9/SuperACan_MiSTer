// Copyright (c) 2026 Jamie Blanks

// U16, quad two-input AND gate.

module ttl_74f08 (
	input  wire [3:0] I0,
	input  wire [3:0] I1,
	output wire [3:0] O
);
	assign O = I0 & I1;
endmodule
