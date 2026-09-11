// Copyright (c) 2026 Jamie Blanks

// U5 and U6 on the cartridge board, ICROS16002 mask ROMs, 1M x 16 each with
// !BYTE tied high. The image lives in SDRAM, so this is the part's pin
// contract over a memory bridge. `ready` lets the UM6619 hold !DTACK while
// SDRAM answers.
//
// A miss launches from `spec`, the address decode alone, half a CPU clock
// before the strobes:
//
//   68000   S0   S1   S2   S3   S4 ...
//   A       ---<===== address =====>
//   !AS     -------------\__________
//   spec    ____/‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾     miss launched here
//   sel     _____________/‾‾‾‾‾‾‾‾‾‾     ready needed by the end of S4

module mask_rom (
	input  wire        clk_sys,
	input  wire        reset,

	input  wire [20:1] A,
	input  wire        CE_n,
	input  wire        OE_n,
	input  wire        spec,      // A decodes to this part, strobes not yet
	output wire [15:0] D_o,
	output wire        D_oe,

	input  wire [25:0] base,      // SDRAM byte address of this image

	// Memory bridge: pulse req while busy is low, then wait for done.
	output wire        mem_req,
	output wire [25:0] mem_addr,
	input  wire [15:0] mem_dout,
	input  wire        mem_busy,
	input  wire        mem_done,

	output wire        ready      // D_o belongs to the current A
);
	reg [20:1] served;
	reg        served_v;
	reg [15:0] q;
	reg        pending;

	wire sel  = ~CE_n & ~OE_n;
	wire hit  = served_v & (served == A);

	assign D_o  = q;
	assign D_oe = sel & hit;

	// Must be combinational: a registered copy would still show the previous
	// access on the cycle the strobes assert and let the 68000 latch stale data.
	assign ready = sel & hit;

	// A speculative launch nobody reads just fills the cache, so no hazard.
	wire launch  = (sel | spec) & ~hit;
	assign mem_req = launch & ~pending & ~mem_busy & ~mem_done & ~reset;
	assign mem_addr = base + {5'd0, A, 1'b0};

	always @(posedge clk_sys) begin
		if (reset) begin
			served_v    <= 1'b0;
			pending     <= 1'b0;
			q           <= 16'd0;
		end else begin
			if (mem_req) begin
				pending  <= 1'b1;
				served   <= A;
				served_v <= 1'b0;
			end

			if (pending && mem_done) begin
				pending  <= 1'b0;
				q        <= mem_dout;
				served_v <= 1'b1;
			end
		end
	end
endmodule
