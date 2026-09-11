// Copyright (c) 2026 Jamie Blanks

// U9, MC68000P10: fx68k behind the schematic's pin names. CLK becomes
// `clk_sys` plus the two phase enables; the data bus and open-drain pins
// split into in, out and enable.
//
// !BERR is tied high on this board, so an unanswered address hangs the CPU
// rather than trapping. !VPA is asserted by the UM6619 for autovectors.

module mc68000 (
	input  wire        clk_sys,
	input  wire        ce_phi1,     // CPU_CLK rising
	input  wire        ce_phi2,     // CPU_CLK falling

	input  wire        pwr_up,      // cold start only, with RESET_n low

	output wire [23:1] A,
	input  wire [15:0] D_i,
	output wire [15:0] D_o,
	output wire        D_oe,

	output wire        AS_n,
	output wire        UDS_n,
	output wire        LDS_n,
	output wire        RW,          // high = read
	input  wire        DTACK_n,
	input  wire        BERR_n,
	input  wire        VPA_n,
	output wire        VMA_n,
	output wire        E,

	input  wire        BR_n,
	output wire        BG_n,
	input  wire        BGACK_n,

	input  wire        IPL0_n,
	input  wire        IPL1_n,
	input  wire        IPL2_n,

	// Function codes, unused on the board.
	output wire        FC0,
	output wire        FC1,
	output wire        FC2,

	// Open-drain, driven by both the board and the CPU.
	input  wire        RESET_n_i,
	output wire        RESET_n_o,
	output wire        RESET_n_oe,
	input  wire        HALT_n_i,
	output wire        HALT_n_o,
	output wire        HALT_n_oe,

	output wire        ss_ready,

	// Savestate. ss_load is a restore window, separate from reset.
	input  wire        ss_load,
	input  wire [63:0] ss_din,
	input  wire [9:0]  ss_addr,
	input  wire        ss_wren,
	input  wire        ss_rst,
	output wire [63:0] ss_dout
);
	`include "ss_map.svh"

	wire o_reset_n, o_halted_n;

	// The real part starts driving half a cycle after !AS; nothing else is on
	// the bus then, so driving for the whole write cycle is equivalent.
	assign D_oe = ~RW & ~AS_n;

	// Open drain: pull low only.
	assign RESET_n_o  = 1'b0;
	assign RESET_n_oe = ~o_reset_n;
	assign HALT_n_o   = 1'b0;
	assign HALT_n_oe  = ~o_halted_n;

	// A savestate may stop the CPU mid-instruction, but never mid bus cycle.
	assign ss_ready = AS_n & UDS_n & LDS_n;

	// Whole CPU state, 1407 bits over 22 words.
	localparam int SS_CPU_BITS  = 1407;
	localparam int SS_CPU_WORDS = 22;

	wire [SS_CPU_BITS-1:0] ss_live;
	wire [SS_CPU_WORDS*64-1:0] ss_saved;
	wire [SS_CPU_WORDS*64-1:0] ss_live_padded =
		{ {(SS_CPU_WORDS*64 - SS_CPU_BITS){1'b0}}, ss_live };
	wire [63:0] ss_word_dout [SS_CPU_WORDS];

	genvar w;
	generate
		for( w = 0; w < SS_CPU_WORDS; w++) begin : g_ss_words
			ss_reg #(.ADDR(SSW_M68K_BASE + w)) word (
				.clk_i      (clk_sys),
				.bus_din_i  (ss_din),
				.bus_addr_i (ss_addr),
				.bus_wren_i (ss_wren),
				// Only read inside the restore window, so no reset needed.
				.bus_rst_i  (1'b0),
				.bus_dout_o (ss_word_dout[w]),
				.din_i      (ss_live_padded[w*64 +: 64]),
				.dout_o     (ss_saved[w*64 +: 64])
			);
		end
	endgenerate

	// Unaddressed words read as zero, so OR them together.
	logic [63:0] ss_dout_or;
	always_comb begin
		ss_dout_or = 64'd0;
		for( int i = 0; i < SS_CPU_WORDS; i++)
			ss_dout_or = ss_dout_or | ss_word_dout[i];
	end
	assign ss_dout = ss_dout_or;

	fx68k cpu
	(
		.clk       (clk_sys),
		.HALTn     (HALT_n_i),
		.extReset  (~RESET_n_i),
		.pwrUp     (pwr_up),
		.enPhi1    (ce_phi1),
		.enPhi2    (ce_phi2),

		.eRWn      (RW),
		.ASn       (AS_n),
		.LDSn      (LDS_n),
		.UDSn      (UDS_n),
		.E         (E),
		.VMAn      (VMA_n),

		.FC0       (FC0),
		.FC1       (FC1),
		.FC2       (FC2),
		.BGn       (BG_n),
		.oRESETn   (o_reset_n),
		.oHALTEDn  (o_halted_n),

		.DTACKn    (DTACK_n),
		.VPAn      (VPA_n),
		.BERRn     (BERR_n),
		.BRn       (BR_n),
		.BGACKn    (BGACK_n),
		.IPL0n     (IPL0_n),
		.IPL1n     (IPL1_n),
		.IPL2n     (IPL2_n),

		.iEdb      (D_i),
		.oEdb      (D_o),
		.eab       (A),

		.ssLoad    (ss_load),
		.ssIn      (ss_saved[SS_CPU_BITS-1:0]),
		.ssOut     (ss_live)
	);

	`ifdef VERILATOR
	/* verilator lint_off UNUSEDSIGNAL */
	wire ss_unused = &{1'b0, ss_rst, ss_saved[SS_CPU_WORDS*64-1:SS_CPU_BITS]};
	/* verilator lint_on UNUSEDSIGNAL */
	`endif
endmodule
