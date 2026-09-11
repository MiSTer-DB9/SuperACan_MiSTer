// Copyright (c) 2026 Jamie Blanks

// U4 on the cartridge board, hex inverter with open-collector outputs. The
// gate only pulls low, so the output is a value plus an enable. R1-R3 pull
// every used output up, so on this board they act as plain inverters.

module ttl_74ls05 (
	input  wire [5:0] I,
	output wire [5:0] O,
	output wire [5:0] O_oe
);
	assign O    = 6'b000000;
	assign O_oe = I;
endmodule
