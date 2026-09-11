// Copyright (c) 2026 Jamie Blanks

// $E9001C, the overlay and sound reset register.
//
//   bit 0  65C02 reset and halt
//   bit 1  low overlay off:  $000000 becomes cartridge
//   bit 2  unknown
//   bit 3  high overlay off: $F80000 stops being the IPL
//
// Bits 1 and 3 are one-way latches until reset. The register must read back:
// the IPL read-modify-writes it at $60A.
//
// BIOS_BYPASS brings both latches up set, so the 68000 boots from the
// cartridge and the authorisation check never runs.

module um6619_overlay #(
	parameter bit BIOS_BYPASS = 1'b0
) (
	input  wire        clk_sys,
	input  wire        reset,

	input  wire        wr,
	input  wire [15:0] din,
	output wire [15:0] dout,

	output wire        low_overlay,    // high while $000000 is the IPL
	output wire        high_overlay,   // high while $F80000 is the IPL
	output wire        snd_reset,

	// Savestate word.
	input  wire [63:0] ss_din,
	input  wire [9:0]  ss_addr,
	input  wire        ss_wren,
	input  wire        ss_rst,
	output wire [63:0] ss_dout
);
	`include "ss_map.svh"

	reg [3:0] bits;

	assign low_overlay  = ~bits[1];
	assign high_overlay = ~bits[3];
	assign snd_reset    = bits[0];
	assign dout         = {12'd0, bits};

	wire [63:0] ss_now;

	ss_reg #(
		.ADDR    (SSW_6619_BUS),
		.DEFAULT ({60'd0, BIOS_BYPASS ? 4'b1010 : 4'b0000})
	) word
	(
		.clk_i(clk_sys), .bus_din_i(ss_din), .bus_addr_i(ss_addr),
		.bus_wren_i(ss_wren), .bus_rst_i(ss_rst), .bus_dout_o(ss_dout),
		.din_i({60'd0, bits}), .dout_o(ss_now)
	);

	always @(posedge clk_sys) begin
		if (reset) begin
			bits <= ss_now[3:0];
		end else if (wr) begin
			// Bits 1 and 3 only ever go up.
			bits[0] <= din[0];
			bits[1] <= bits[1] | din[1];
			bits[2] <= din[2];
			bits[3] <= bits[3] | din[3];
		end
	end

	// Unused savestate bits.
	`ifdef VERILATOR
	/* verilator lint_off UNUSEDSIGNAL */
	wire unused = &{1'b0, din[15:4], ss_now[63:4]};
	/* verilator lint_on UNUSEDSIGNAL */
	`endif
endmodule
