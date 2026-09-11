// Copyright (c) 2026 Jamie Blanks

// The interrupt encoder: seven sources onto the 68000's three !IPL pins.
//
//   1  expansion port
//   2  cartridge
//   3  free running counter
//   4  UM6618 visible line       needs $E90010 bit 4
//   5  UM6618 line on/off
//   6  sound mailbox             the 65C02 writing $0300
//   7  UM6618 vertical retrace   needs $E90010 bit 7
//
// The mask at $E90010 is only known to gate levels 4 and 7. Every source is
// level held until its own acknowledge. !IPL idles at 3'b111.

module um6619_irq (
	input  wire        clk_sys,
	input  wire        reset,

	// $E90010.
	input  wire        mask_wr,
	input  wire [15:0] mask_din,
	output wire [15:0] mask_dout,

	// Level-held requests.
	input  wire        req_expansion,
	input  wire        req_cartridge,
	input  wire        req_frc,
	input  wire        req_line,
	input  wire        req_line_onoff,
	input  wire        req_mailbox,
	input  wire        req_vblank,

	output wire [2:0]  ipl_n,

	// Savestate word.
	input  wire [63:0] ss_din,
	input  wire [9:0]  ss_addr,
	input  wire        ss_wren,
	input  wire        ss_rst,
	output wire [63:0] ss_dout
);
	`include "ss_map.svh"

	reg [15:0] mask;
	wire [63:0] ss_now;

	assign mask_dout = mask;

	ss_reg #(.ADDR(SSW_6619_BUS + 1), .DEFAULT(64'd0)) word
	(
		.clk_i(clk_sys), .bus_din_i(ss_din), .bus_addr_i(ss_addr),
		.bus_wren_i(ss_wren), .bus_rst_i(ss_rst), .bus_dout_o(ss_dout),
		.din_i({48'd0, mask}), .dout_o(ss_now)
	);

	always @(posedge clk_sys) begin
		if (reset)        mask <= ss_now[15:0];
		else if (mask_wr) mask <= mask_din;
	end

	wire [7:1] req = {
		req_vblank & mask[7],
		req_mailbox,
		req_line_onoff,
		req_line & mask[4],
		req_frc,
		req_cartridge,
		req_expansion
	};

	// Highest pending level wins; the pins carry its complement.
	reg [2:0] level;
	always @(*) begin
		if      (req[7]) level = 3'd7;
		else if (req[6]) level = 3'd6;
		else if (req[5]) level = 3'd5;
		else if (req[4]) level = 3'd4;
		else if (req[3]) level = 3'd3;
		else if (req[2]) level = 3'd2;
		else if (req[1]) level = 3'd1;
		else             level = 3'd0;
	end

	assign ipl_n = ~level;

	`ifdef VERILATOR
	/* verilator lint_off UNUSEDSIGNAL */
	wire unused = &{1'b0, ss_now[63:16], mask[15:8], mask[6:5], mask[3:0]};
	/* verilator lint_on UNUSEDSIGNAL */
	`endif
endmodule
