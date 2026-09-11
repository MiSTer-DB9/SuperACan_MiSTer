// Copyright (c) 2026 Jamie Blanks

// U17, quad two-input OR gate.

module ttl_74f32 (
	input  wire [3:0] I0,
	input  wire [3:0] I1,
	output wire [3:0] O
);
	assign O = I0 | I1;
endmodule
