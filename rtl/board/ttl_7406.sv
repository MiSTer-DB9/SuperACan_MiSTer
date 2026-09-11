// Copyright (c) 2026 Jamie Blanks

// U20, hex inverter with open-collector outputs. The gate only pulls low, so
// the output is split into a value and an enable that is active when the
// input is high. Several of these share one joypad line.

module ttl_7406 (
	input  wire [5:0] I,
	output wire [5:0] O,
	output wire [5:0] O_oe
);
	assign O    = 6'b000000;
	assign O_oe = I;
endmodule
