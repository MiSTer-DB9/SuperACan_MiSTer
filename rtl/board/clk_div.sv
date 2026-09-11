// Copyright (c) 2026 Jamie Blanks

// Clock dividers inside the UM6618. The 53.693175 MHz crystal reaches only
// that chip, so `clk_sys` is MCLK and every board clock is divided here.
//
//   MCLK ---+-- /5  --> 10.738635 MHz  CPU_CLK, out to U9, U10 and the slot
//           +-- /6  --> 8.948862 MHz   U10's free running counter
//           +-- /10 --> 5.369318 MHz   dot clock and U3's PCLK, 256-wide
//           +-- /8  --> 6.711647 MHz   dot clock and U3's PCLK, 320-wide
//
// The 65C02's 3.579545 MHz is CPU_CLK/3, made inside the UM6619. The free
// running counter tick is MCLK/6, which the UM6619 has no MCLK pin to make,
// so it is carried as an extra net. phi1 opens the emulated clock period,
// phi2 falls near its middle.
//
// A 320-pixel line is 427.5 MCLK/8 dots, so thirty blanking dots run at
// /10; the video timing raises `pix_stretch` the dot before each one.
//
// No reset: the crystal runs while the console is held in reset. The pause
// inputs are combinational so a freeze lands the cycle it is asserted; the
// savestate parks the CPUs first and lets the video run on to blanking.

module clk_div (
	input  wire clk_sys,
	input  wire pause_cpu,   // 68000, 65C02 and free running counter
	input  wire pause_vid,   // dot clock

	input  wire h320,        // U12 width mode, from $F00008 bit 8
	input  wire pix_stretch, // run the next dot at the H256 rate
	input  wire ss_load,
	input  wire [63:0] ss_din,
	output wire [63:0] ss_dout,
	output wire pix_ready,

	output wire ce_cpu_phi1,
	output wire ce_cpu_phi2,
	output wire ce_pix
);
	// MCLK ticks per emulated clock.
	localparam int CPU_DIV = 5;
	localparam int PIX_DIV_H256 = 10;
	localparam int PIX_DIV_H320 = 8;

	localparam int CPU_PHI2 = CPU_DIV / 2;

	localparam int CPU_W = $clog2(CPU_DIV);
	localparam int PIX_W = $clog2(PIX_DIV_H256 > PIX_DIV_H320 ?
	                              PIX_DIV_H256 : PIX_DIV_H320);

	// No reset, so power-up values are what make these deterministic.
	/* verilator lint_off PROCASSINIT */
	reg [CPU_W-1:0] cpu_cnt = '0;
	reg [PIX_W-1:0] pix_cnt = '0;
	reg [PIX_W-1:0] pix_last = PIX_W'(PIX_DIV_H256 - 1);
	/* verilator lint_on PROCASSINIT */

	// Length of the next dot, sampled on the last cycle of the current one.
	wire [PIX_W-1:0] pix_len_m1 = (h320 && !pix_stretch) ?
		PIX_W'(PIX_DIV_H320 - 1) : PIX_W'(PIX_DIV_H256 - 1);

	// The chips restore one cycle behind ss_load. An enable on that last
	// cycle would start a step that the load then overwrites.
	reg ss_load_q = 1'b0;
	always @(posedge clk_sys) ss_load_q <= ss_load;
	wire ss_restoring = ss_load | ss_load_q;

	assign pix_ready   = (pix_cnt == PIX_W'(0));
	assign ce_cpu_phi1 = ~ss_restoring & ~pause_cpu & (cpu_cnt == CPU_W'(0));
	assign ce_cpu_phi2 = ~ss_restoring & ~pause_cpu & (cpu_cnt == CPU_W'(CPU_PHI2));
	assign ce_pix      = ~ss_restoring & ~pause_vid & pix_ready;
	// Bits 47:32 once held a second divider; kept zero so the word keeps its shape.
	assign ss_dout     = {16'(cpu_cnt), 16'd0, 16'(pix_cnt), 16'(pix_last)};

	always @(posedge clk_sys) begin
		if (ss_load) begin
			cpu_cnt <= CPU_W'(ss_din[63:48]);
		end else if (ss_load_q) begin
			cpu_cnt <= cpu_cnt;
		end else if (!pause_cpu) begin
			cpu_cnt <= (cpu_cnt == CPU_W'(CPU_DIV - 1)) ? '0 : cpu_cnt + 1'd1;
		end

		if (ss_load) begin
			pix_cnt  <= PIX_W'(ss_din[31:16]);
			pix_last <= PIX_W'(ss_din[15:0]);
		end else if (ss_load_q) begin
			pix_cnt  <= pix_cnt;
			pix_last <= pix_last;
		end else if (!pause_vid) begin
			// A width change mid-dot can leave the count past the new length.
			if (pix_cnt >= pix_last) begin
				pix_cnt  <= '0;
				pix_last <= pix_len_m1;
			end else begin
				pix_cnt <= pix_cnt + 1'd1;
			end
		end
	end
endmodule
