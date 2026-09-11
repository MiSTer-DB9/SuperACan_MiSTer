// Copyright (c) 2026 Jamie Blanks

// Multitap on one pad connector, up to MAX_PADS pads.
//
// The sound driver latches once and keeps clocking, sixteen clocks per pad,
// until a frame reads all zero or fifteen frames are in. The tap answers pad
// 1's sixteen bits, then pad 2's, and drops the line after the last pad. The
// real tap's design is unknown; this one gives each pad a sixteen-clock window.
//
//   latch ─┬─────────┬─────────┬─────────┐
//          │         │         │         │
//   clock ─┼─[win 0]─┼─[win 1]─┼─[win 2]─┼─ ...    window = count[7:4]
//          │    │    │    │    │    │    │
//        pad 1  │  pad 2  │  pad 3  │  pad 4        every pad loads on latch
//          └────┴────┴────┴────┴────┴────┘
//   data  = the pad in the current window, 0 past the last connected pad

module multitap #(
	parameter int MAX_PADS = 4
) (
	input  wire                     clk_sys,

	input  wire                     latch,   // pin 1
	input  wire                     clock,   // pin 7
	output wire                     data,    // pin 3

	// Pads plugged in, 1..MAX_PADS. Pad n's buttons are bits [12n+11:12n].
	input  wire [3:0]               pads,
	input  wire [MAX_PADS*12-1:0]   buttons,

	// Savestate: clock counter plus every pad's shift register.
	input  wire                     ss_load,
	input  wire [MAX_PADS*16+7:0]   ss_in,
	output wire [MAX_PADS*16+7:0]   ss_out
);
	// Clocks since latch. Saturates so a long poll cannot wrap back to pad 1.
	reg [7:0] count = 8'd0;

	wire [3:0] window = count[7:4];

	always @(posedge clk_sys) begin
		if (ss_load)          count <= ss_in[MAX_PADS*16 +: 8];
		else if (latch)       count <= 8'd0;
		else if (clock && ~&count) count <= count + 8'd1;
	end

	assign ss_out[MAX_PADS*16 +: 8] = count;

	wire [MAX_PADS-1:0] pad_data;

	genvar n;
	generate
		for (n = 0; n < MAX_PADS; n = n + 1) begin : pad
			joypad pad (
				.clk_sys (clk_sys),
				.latch   (latch),
				.clock   (clock && window == n[3:0]),
				.data    (pad_data[n]),
				.buttons (buttons[n*12 +: 12]),
				.ss_load (ss_load),
				.ss_in   (ss_in[n*16 +: 16]),
				.ss_out  (ss_out[n*16 +: 16])
			);
		end
	endgenerate

	// Past the last connected pad nothing drives the line and it reads low.
	reg sel;
	integer i;
	always @* begin
		sel = 1'b0;
		for (i = 0; i < MAX_PADS; i = i + 1)
			if (window == i[3:0] && window < pads) sel = pad_data[i];
	end

	assign data = sel;
endmodule
