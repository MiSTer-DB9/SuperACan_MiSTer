// Copyright (c) 2026 Jamie Blanks

// The savestate's window on the board's DDR3, which the FPGA shares with the
// ARM. One 64-bit word per request, no bursts: the transfer engine asks for
// one word and waits, and at that rate the extra latency of a single-beat
// access costs nothing next to the eight clocks it spends per byte anyway.
//
// The framework's port addresses 64-bit words. The savestate region starts at
// byte 0x30000000, so the top four address bits are constant and the engine's
// dword address, which counts in fours of bytes, halves into a word index.
//
// Reads have no fixed latency here - the ARM is on the same memory - so
// nothing waits a set number of cycles: a command is presented until the
// controller stops saying busy, and a read then waits for its own data ready.

module ss_ddram (
	input  wire        clk,
	input  wire        reset,

	output wire        DDRAM_CLK,
	input  wire        DDRAM_BUSY,
	output wire [7:0]  DDRAM_BURSTCNT,
	output wire [28:0] DDRAM_ADDR,
	input  wire [63:0] DDRAM_DOUT,
	input  wire        DDRAM_DOUT_READY,
	output reg         DDRAM_RD,
	output wire [63:0] DDRAM_DIN,
	output wire [7:0]  DDRAM_BE,
	output reg         DDRAM_WE,

	// Dword offset from the base of the region, as savestates.sv counts.
	input  wire [25:0] addr_i,
	input  wire [63:0] din_i,
	output reg  [63:0] dout_o,
	input  wire [7:0]  be_i,
	input  wire        rnw_i,
	input  wire        ena_i,
	output reg         done_o
);
	localparam [3:0] DDR_BASE = 4'b0011;   // byte 0x30000000

	// Bit 0 is always zero: the engine steps two dwords, one 64-bit word.
	/* verilator lint_off UNUSEDSIGNAL */
	reg [25:0] addr_q;
	/* verilator lint_on UNUSEDSIGNAL */
	reg [63:0] din_q;
	reg [7:0]  be_q;
	reg        rnw_q;
	reg        busy_q;

	assign DDRAM_CLK      = clk;
	assign DDRAM_BURSTCNT = 8'd1;
	assign DDRAM_ADDR     = {DDR_BASE, addr_q[25:1]};
	assign DDRAM_DIN      = din_q;
	assign DDRAM_BE       = rnw_q ? 8'hFF : be_q;

	// The controller takes the command on the first cycle it is not busy.
	wire accepted = (DDRAM_RD | DDRAM_WE) & ~DDRAM_BUSY;

	always @(posedge clk) begin
		done_o <= 1'b0;

		if (reset) begin
			DDRAM_RD <= 1'b0;
			DDRAM_WE <= 1'b0;
			busy_q   <= 1'b0;
		end else begin
			if (accepted) begin
				DDRAM_RD <= 1'b0;
				DDRAM_WE <= 1'b0;
				// A write is finished the moment it is taken; a read still
				// has to bring its word back.
				if (!rnw_q) begin
					busy_q <= 1'b0;
					done_o <= 1'b1;
				end
			end

			if (busy_q && rnw_q && DDRAM_DOUT_READY) begin
				dout_o <= DDRAM_DOUT;
				busy_q <= 1'b0;
				done_o <= 1'b1;
			end

			if (ena_i && !busy_q) begin
				addr_q   <= addr_i;
				din_q    <= din_i;
				be_q     <= be_i;
				rnw_q    <= rnw_i;
				DDRAM_RD <= rnw_i;
				DDRAM_WE <= ~rnw_i;
				busy_q   <= 1'b1;
			end
		end
	end

endmodule
