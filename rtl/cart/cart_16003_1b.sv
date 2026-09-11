// Copyright (c) 2026 Jamie Blanks

// PCCAR 16003-1B retail cartridge board, as a netlist with the schematic's
// net names.
//
//   U5, U6  ICROS16002  16 Mbit mask ROMs, 4 MB across the pair
//   U2      62256       32 KiB battery-backed SRAM
//   U1      UM6650      lockout chip
//   U3      74F688      8-bit comparator
//   U4      74LS05      hex open-collector inverter
//
// Both ROMs' !CE is cart net 26. U5's !OE is A21 and U6's is its inverse, so
// A21 is the whole bank switch. The comparator matches A23..A16 against $EC
// under !AS, putting the SRAM at $EC0000-$ECFFFF on the low byte lane only,
// which is why a save file is 32 KiB over a 64 KiB window. The UM6650 sits
// on cart nets 26 and 27, one address line and the low byte lane.

module cart_16003_1b (
	input  wire        clk_sys,
	input  wire        reset,

	// Address lines each ROM has, and whether U6 is fitted.
	input  wire [20:1] u5_amask,
	input  wire [20:1] u6_amask,
	input  wire        u6_fitted,

	// J1, the edge connector.
	input  wire [23:1] CPU_A,
	input  wire [15:0] CPU_D_i,
	output wire [15:0] CPU_D_o,
	output wire        CPU_D_oe,
	input  wire        CPU_AS_n,
	input  wire        CPU_RW,
	input  wire        CART_26_n,
	input  wire        CART_27_n,

	// The UM6619 holds !DTACK until the ROMs can answer.
	output wire        rom_ready,

	// Mask ROMs over the SDRAM bridge.
	output wire        u5_req,
	output wire [25:0] u5_addr,
	input  wire [15:0] u5_dout,
	input  wire        u5_busy,
	input  wire        u5_done,
	output wire        u6_req,
	output wire [25:0] u6_addr,
	input  wire [15:0] u6_dout,
	input  wire        u6_busy,
	input  wire        u6_done,

	// UM6650 key from the BIOS image.
	input  wire        ld_key_wr,
	input  wire [2:0]  ld_key_addr,
	input  wire [15:0] ld_key_data,

	// Battery SRAM savestate port.
	input  wire        ss_own,
	input  wire [14:0] ss_addr_b,
	input  wire        ss_we,
	input  wire [7:0]  ss_din_b,
	output wire [7:0]  ss_dout_b,

	// Battery SRAM save-file port; bk_dirty pulses per CPU write.
	input  wire [13:0] bk_addr,
	input  wire        bk_wren,
	input  wire [1:0]  bk_be,
	input  wire [15:0] bk_wdata,
	output wire [15:0] bk_q,
	output wire        bk_dirty,

	input  wire [63:0] ss_din,
	input  wire [9:0]  ss_addr,
	input  wire        ss_wren,
	input  wire        ss_rst,
	output wire [63:0] ss_dout
);
	// ---- U4, the open-collector inverters ---------------------------------

	// R1-R3 pull every used output up, so each gate acts as a plain inverter.
	wire [5:0] u4_i, u4_o, u4_oe;
	wire n2, n3, n5, n12;

	assign u4_i = {3'b111, CPU_A[21], n3, n2};
	assign n3  = ~u4_oe[0] | u4_o[0];
	assign n5  = ~u4_oe[1] | u4_o[1];
	assign n12 = ~u4_oe[2] | u4_o[2];

	ttl_74ls05 U4 (.I(u4_i), .O(u4_o), .O_oe(u4_oe));

	// ---- U3, the SRAM window comparator ------------------------------------

	// Q is strapped to $EC.
	ttl_74f688 U3
	(
		.P     (CPU_A[23:16]),
		.Q     (8'hEC),
		.G_n   (CPU_AS_n),
		.PEQ_n (n2)
	);

	// ---- U5 and U6, the mask ROMs ------------------------------------------

	// The 256 KiB window at $F80000 mirrors $000000, so U5 answers it with
	// the lines above A17 held low.
	wire hi_win = (CPU_A[23:18] == 6'b111110);
	// Address-only decode so a miss can launch before !AS.
	wire lo_win  = (CPU_A[23:22] == 2'b00);
	wire u5_spec = hi_win | (lo_win & ~CPU_A[21]);
	wire u6_spec = lo_win & CPU_A[21] & ~hi_win;

	wire [15:0] u5_d, u6_d;
	wire        u5_oe, u6_oe, u5_ready, u6_ready;

	mask_rom U5
	(
		.clk_sys     (clk_sys),
		.reset       (reset),
		.A           (hi_win ? {3'b000, CPU_A[17:1]}
		                     : (CPU_A[20:1] & u5_amask)),
		.CE_n        (CART_26_n),
		.OE_n        (hi_win ? 1'b0 : CPU_A[21]),
		.spec        (u5_spec),
		.D_o         (u5_d),
		.D_oe        (u5_oe),
		.base        (26'h000000),
		.mem_req     (u5_req),
		.mem_addr    (u5_addr),
		.mem_dout    (u5_dout),
		.mem_busy    (u5_busy),
		.mem_done    (u5_done),
		.ready       (u5_ready)
	);

	wire [15:0] u6_d_raw;
	wire        u6_oe_raw, u6_ready_raw;

	mask_rom U6
	(
		.clk_sys     (clk_sys),
		.reset       (reset),
		.A           (CPU_A[20:1] & u6_amask),
		.CE_n        (CART_26_n | ~u6_fitted),
		.OE_n        (hi_win ? 1'b1 : n12),
		.spec        (u6_spec & u6_fitted),
		.D_o         (u6_d_raw),
		.D_oe        (u6_oe_raw),
		.base        (26'h200000),
		.mem_req     (u6_req),
		.mem_addr    (u6_addr),
		.mem_dout    (u6_dout),
		.mem_busy    (u6_busy),
		.mem_done    (u6_done),
		.ready       (u6_ready_raw)
	);

	// Empty socket: nothing drives the bus and nothing waits.
	assign u6_d     = u6_fitted ? u6_d_raw     : 16'hFFFF;
	assign u6_oe    = u6_fitted ? u6_oe_raw    : 1'b0;
	assign u6_ready = u6_fitted ? u6_ready_raw : 1'b1;

	// Only the chip with !OE low can be waiting. The high window is always U5.
	assign rom_ready   = CART_26_n
	                   | ((CPU_A[21] & ~hi_win) ? u6_ready : u5_ready);

	// ---- U2, the battery SRAM ----------------------------------------------

	wire [7:0] u2_d;
	wire       u2_oe;

	sram_async #(.ADDR_WIDTH(15), .DATA_WIDTH(8), .BACKUP_PORT(1'b1)) U2
	(
		.clk_sys (clk_sys),
		.A       (CPU_A[15:1]),
		.CS_n    (n5),
		.OE_n    (n5),
		.WE_n    (CPU_RW),
		.D_i     (CPU_D_i[7:0]),
		.D_o     (u2_d),
		.D_oe    (u2_oe),
		.ss_own  (ss_own),
		.ss_addr (ss_addr_b),
		.ss_we   (ss_we),
		.ss_din  (ss_din_b),
		.ss_dout (ss_dout_b),
		.bk_addr (bk_addr),
		.bk_wren (bk_wren),
		.bk_be   (bk_be),
		.bk_wdata(bk_wdata),
		.bk_q    (bk_q),
		.dirty   (bk_dirty)
	);

	// ---- U1, the lockout chip ----------------------------------------------

	wire [7:0] u1_d;
	wire       u1_oe;
	/* verilator lint_off UNUSEDSIGNAL */
	wire [7:0] u1_pass;
	/* verilator lint_on UNUSEDSIGNAL */

	um6650 U1
	(
		.clk_sys  (clk_sys),
		.reset    (reset),
		.A1       (CPU_A[1]),
		.D_i      (CPU_D_i[7:0]),
		.D_o      (u1_d),
		.D_oe     (u1_oe),
		.RW       (CPU_RW),
		.CART26_n (CART_26_n),
		.CART27_n (CART_27_n),
		.ld_wr    (ld_key_wr),
		.ld_addr  (ld_key_addr),
		.ld_data  (ld_key_data),
		.pass_out (u1_pass),
		.ss_din   (ss_din),
		.ss_addr  (ss_addr),
		.ss_wren  (ss_wren),
		.ss_rst   (ss_rst),
		.ss_dout  (ss_dout)
	);

	// ---- the connector's data bus -------------------------------------------

	assign CPU_D_o = u5_oe ? u5_d
	               : u6_oe ? u6_d
	               : u2_oe ? {8'hFF, u2_d}
	               : u1_oe ? {8'hFF, u1_d}
	                       : 16'hFFFF;
	assign CPU_D_oe = u5_oe | u6_oe | u2_oe | u1_oe;

	`ifdef VERILATOR
	/* verilator lint_off UNUSEDSIGNAL */
	wire unused = &{1'b0, CPU_D_i[15:8], u4_o[5:3], u4_oe[5:3]};
	/* verilator lint_on UNUSEDSIGNAL */
	`endif
endmodule
