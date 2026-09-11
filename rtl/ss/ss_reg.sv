// Copyright (c) 2026 Jamie Blanks

// One 64-bit word of savestate scalar state. A module packs its live state
// into `din_i` and reads its restored state from `dout_o`. On bus_rst the
// word takes DEFAULT, the power-on value, so one reset branch serves a cold
// start and a restore. `bus_dout_o` is zero unless addressed, so the parent
// can OR every block's output together.

module ss_reg #(
	parameter integer ADDR    = 0,
	parameter [63:0]  DEFAULT = 64'd0
) (
	input  wire        clk_i,
	input  wire [63:0] bus_din_i,
	input  wire [9:0]  bus_addr_i,
	input  wire        bus_wren_i,
	input  wire        bus_rst_i,
	output wire [63:0] bus_dout_o,

	input  wire [63:0] din_i,        // live value
	output reg  [63:0] dout_o        // restored value
);
	wire sel = (bus_addr_i == ADDR[9:0]);

	initial dout_o = DEFAULT;

	always @(posedge clk_i) begin
		if (bus_rst_i)             dout_o <= DEFAULT;
		else if (sel && bus_wren_i) dout_o <= bus_din_i;
	end

	assign bus_dout_o = sel ? din_i : 64'd0;
endmodule
