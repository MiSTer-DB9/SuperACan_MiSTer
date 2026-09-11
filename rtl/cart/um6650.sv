// Copyright (c) 2026 Jamie Blanks

// U1 on the cartridge board, the UM6650 lockout chip. D0-D7, one address
// line, two selects from the UM6619. Behaviour follows what the IPL does
// with it.
//
//   A1 = 1   $EB0D03   address port
//   A1 = 0   $EB0D01   data port
//
//   $20-$2F  sixteen key bytes, read only. The IPL checks them against
//            "UMC 1994 (C)" plus four checksums of that string.
//   $40-$5F  32 bytes of RAM. The IPL writes each address its own value and
//            reads it back.
//   $09      written $FF after the checks pass; feeds cartridge pins B26/B27.
//   $0C      written $00.
//
// Everything else reads as zero.

module um6650 (
	input  wire       clk_sys,
	input  wire       reset,

	input  wire       A1,
	input  wire [7:0] D_i,
	output wire [7:0] D_o,
	output wire       D_oe,
	input  wire       RW,        // high = read
	input  wire       CART26_n,  // low while the mask ROMs are selected
	input  wire       CART27_n,  // low while this chip is selected

	// Key bytes from the BIOS image.
	input  wire       ld_wr,
	input  wire [2:0] ld_addr,   // word address inside the sixteen bytes
	input  wire [15:0] ld_data,

	output wire [7:0] pass_out,  // register $09

	// Savestate word.
	input  wire [63:0] ss_din,
	input  wire [9:0]  ss_addr,
	input  wire        ss_wren,
	input  wire        ss_rst,
	output wire [63:0] ss_dout
);
	`include "ss_map.svh"

	reg [6:0] addr;
	reg [7:0] ram [0:31];
	reg [7:0] key [0:15];
	reg [7:0] reg09;
	reg [7:0] reg0c;

	wire sel = ~CART27_n;
	wire wr  = sel & ~RW;
	wire rd  = sel &  RW;

	wire is_key = (addr[6:4] == 3'b010);            // $20-$2F
	wire is_ram = (addr[6:5] == 2'b10);             // $40-$5F

	wire [7:0] rdata = is_key ? key[addr[3:0]]
	                 : is_ram ? ram[addr[4:0]]
	                 : 8'd0;

	assign D_o     = A1 ? {1'b0, addr} : rdata;
	assign D_oe    = rd;
	assign pass_out = reg09;

	wire [63:0] ss_now;
	ss_reg #(.ADDR(SSW_CART_BASE), .DEFAULT(64'd0)) word
	(
		.clk_i(clk_sys), .bus_din_i(ss_din), .bus_addr_i(ss_addr),
		.bus_wren_i(ss_wren), .bus_rst_i(ss_rst), .bus_dout_o(ss_dout),
		.din_i({41'd0, addr, reg0c, reg09}), .dout_o(ss_now)
	);

	// A 68000 write holds the select for several clocks; act once per cycle.
	reg wr_q;
	wire wr_edge = wr & ~wr_q;

	always @(posedge clk_sys) begin
		wr_q <= wr;

		if (ld_wr) begin
			key[{ld_addr, 1'b0}] <= ld_data[7:0];
			key[{ld_addr, 1'b1}] <= ld_data[15:8];
		end

		if (reset) begin
			reg09 <= ss_now[7:0];
			reg0c <= ss_now[15:8];
			addr  <= ss_now[22:16];
			wr_q  <= 1'b0;
		end else if (wr_edge) begin
			if (A1) begin
				addr <= D_i[6:0];
			end else begin
				if (is_ram)             ram[addr[4:0]] <= D_i;
				if (addr == 7'h09)      reg09 <= D_i;
				if (addr == 7'h0C)      reg0c <= D_i;
			end
		end
	end

	`ifdef VERILATOR
	/* verilator lint_off UNUSEDSIGNAL */
	wire unused = &{1'b0, CART26_n, ss_now[63:23], D_i[7]};
	/* verilator lint_on UNUSEDSIGNAL */
	`endif
endmodule
