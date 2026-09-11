// Copyright (c) 2026 Jamie Blanks

// Super A'Can pad: a sixteen-bit parallel-to-serial shift register with an
// open-collector output, the same arrangement as a SNES pad. Pin 1 latches
// the buttons, pin 7 clocks one bit out on pin 3, MSB first, pressed = low.
//
//   bit 15 A   14 B   13 Start  12 Select  11 Up   10 Down  9 Left  8 Right
//        7 X    6 Y    5 L       4 R        3-0 unused, read high
//
// After sixteen bits, data stays low until the next latch.

module joypad (
	input  wire        clk_sys,

	input  wire        latch,     // pin 1
	input  wire        clock,     // pin 7
	output wire        data,      // pin 3

	// High when pressed, framework order:
	// right, left, down, up, A, B, X, Y, L, R, select, start.
	input  wire [11:0] buttons,

	// The shift register is often mid-poll, so it is part of the savestate.
	input  wire        ss_load,
	input  wire [15:0] ss_in,
	output wire [15:0] ss_out
);
	wire [15:0] state = ~{
		buttons[4],  buttons[5],  buttons[11], buttons[10],
		buttons[3],  buttons[2],  buttons[1],  buttons[0],
		buttons[6],  buttons[7],  buttons[8],  buttons[9],
		4'b0000
	};

	reg [15:0] sr = 16'hFFFF;

	assign data = sr[15];

	assign ss_out = sr;

	always @(posedge clk_sys) begin
		if (ss_load)    sr <= ss_in;
		else if (latch) sr <= state;
		else if (clock) sr <= {sr[14:0], 1'b0};
	end
endmodule
