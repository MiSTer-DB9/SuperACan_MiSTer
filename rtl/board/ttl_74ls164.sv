// Copyright (c) 2026 Jamie Blanks

// U18 and U19, 8-bit serial-in parallel-out shift registers. U18's QH feeds
// U19's A and B, so the pair forms sixteen stages on one clock. A and B are
// ANDed into the first stage and !CLR is asynchronous. The CLK pin is a joypad
// clock from the UM6619, so it arrives as a rising-edge enable.

module ttl_74ls164 (
	input  wire       clk_sys,
	input  wire       clk_rise,   // rising edge on the CLK pin
	input  wire       A,
	input  wire       B,
	input  wire       CLR_n,
	output wire [7:0] Q
);
	reg [7:0] q;

	assign Q = q;

	always @(posedge clk_sys) begin
		if (!CLR_n)         q <= 8'd0;
		else if (clk_rise)  q <= {q[6:0], A & B};
	end
endmodule
