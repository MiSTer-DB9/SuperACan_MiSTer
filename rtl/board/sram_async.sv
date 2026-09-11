// Copyright (c) 2026 Jamie Blanks

// The board's asynchronous SRAMs: UM62256 for work and sound RAM, UM61512AK
// for VRAM. One byte-wide device per instance, matching the board, where each
// 68000 byte lane is its own chip with its own chip select.
//
//   read   !CS and !OE low, !WE high            data out one clk_sys later
//   write  !CS low and !WE low                  data in on that edge
//
// On this board !CS and !OE share a net and !WE is the CPU's R/W line, so a
// write drives all three low at once and the write has to win, as it does on
// a real 62256.
//
// The savestate port is only used while the machine is parked.
//
// BACKUP_PORT is for the battery-backed cartridge U2. The save file needs
// access while the machine runs, so the storage becomes a 16-bit true dual
// port: pins and savestate use a byte lane on port A, the save file gets
// port B at the width sd_buff wants. Byte 2k is the low half of word k, so
// .sav byte N is SRAM byte N.

module sram_async #(
	parameter int ADDR_WIDTH = 15,
	parameter int DATA_WIDTH = 8,
	parameter bit BACKUP_PORT = 1'b0
) (
	input  wire                     clk_sys,

	input  wire [ADDR_WIDTH-1:0]    A,
	input  wire                     CS_n,
	input  wire                     OE_n,
	input  wire                     WE_n,
	input  wire [DATA_WIDTH-1:0]    D_i,
	output wire [DATA_WIDTH-1:0]    D_o,
	output wire                     D_oe,

	// Savestate port, valid only while the machine is parked.
	input  wire                     ss_own,
	input  wire [ADDR_WIDTH-1:0]    ss_addr,
	input  wire                     ss_we,
	input  wire [DATA_WIDTH-1:0]    ss_din,
	output wire [DATA_WIDTH-1:0]    ss_dout,

	// Save-file port, BACKUP_PORT only.
	input  wire [ADDR_WIDTH-2:0]    bk_addr,
	input  wire                     bk_wren,
	input  wire [1:0]               bk_be,
	input  wire [15:0]              bk_wdata,
	output wire [15:0]              bk_q,
	output wire                     dirty        // pulses per pin-side write
);
	wire                  write = ~CS_n & ~WE_n;
	wire [ADDR_WIDTH-1:0] addr  = ss_own ? ss_addr : A;
	wire                  we    = ss_own ? ss_we   : write;
	wire [DATA_WIDTH-1:0] din   = ss_own ? ss_din  : D_i;

	wire [DATA_WIDTH-1:0] q;

	assign D_o     = q;
	assign D_oe    = ~CS_n & ~OE_n & WE_n;
	assign ss_dout = q;
	assign dirty   = write & ~ss_own;

	generate
	if (BACKUP_PORT) begin : g_backup
		// Read data lands a clock later, so the byte lane select rides with it.
		reg lane_q;
		always @(posedge clk_sys) lane_q <= addr[0];

		wire [15:0] q_a;
		assign q = lane_q ? q_a[15:8] : q_a[7:0];

		cache_ram_dp_be #(
			.ADDR_WIDTH (ADDR_WIDTH-1),
			.DATA_WIDTH (16)
		) mem
		(
			.clk_i     (clk_sys),
			.addr_a_i  (addr[ADDR_WIDTH-1:1]),
			.wren_a_i  (we),
			.be_a_i    (addr[0] ? 2'b10 : 2'b01),
			.wdata_a_i ({din, din}),
			.q_a_o     (q_a),
			.addr_b_i  (bk_addr),
			.wren_b_i  (bk_wren),
			.be_b_i    (bk_be),
			.wdata_b_i (bk_wdata),
			.q_b_o     (bk_q)
		);
	end else begin : g_plain
		assign bk_q = 16'd0;

		cache_ram #(
			.ADDR_WIDTH (ADDR_WIDTH),
			.DATA_WIDTH (DATA_WIDTH)
		) mem
		(
			.clk_i   (clk_sys),
			.addr_i  (addr),
			.wren_i  (we),
			.wdata_i (din),
			.q_o     (q)
		);

		`ifdef VERILATOR
		/* verilator lint_off UNUSEDSIGNAL */
		wire unused = &{1'b0, bk_addr, bk_wren, bk_be, bk_wdata};
		/* verilator lint_on UNUSEDSIGNAL */
		`endif
	end
	endgenerate
endmodule
