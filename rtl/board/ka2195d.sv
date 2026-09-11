// Copyright (c) 2026 Jamie Blanks

// U4, KA2195D video encoder. The framework does its own analog encoding, so
// this is a pass-through that keeps the board netlist complete.

module ka2195d (
	input  wire [7:0] R_i,
	input  wire [7:0] G_i,
	input  wire [7:0] B_i,
	input  wire       CSYNC_i,

	output wire [7:0] R_o,
	output wire [7:0] G_o,
	output wire [7:0] B_o,
	output wire       CSYNC_o
);
	assign R_o     = R_i;
	assign G_o     = G_i;
	assign B_o     = B_i;
	assign CSYNC_o = CSYNC_i;
endmodule
