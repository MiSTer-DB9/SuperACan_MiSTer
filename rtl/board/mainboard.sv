// Copyright (c) 2026 Jamie Blanks

// PCGAM 16000-2A mainboard netlist: one module per chip, wired with the
// schematic's net names. The only logic here resolves the open-drain nets.
//
//   U9   MC68000P10   CPU
//   U10  UM6619F      system, sound and I/O
//   U12  UM6618F      video
//   U1   UM62256      work RAM, low byte
//   U2   UM62256      work RAM, high byte
//   U11  UM62256      sound RAM
//   U5   UM61512AK    VRAM, low byte
//   U6   UM61512AK    VRAM, high byte
//   U3   UM70C188     palette RAM and triple DAC
//   U4   KA2195D      RGB to NTSC encoder
//   U13  53.693175 MHz crystal
//   U16..U21          pad glue
//
// MCLK reaches U12 only; U12 drives CPU_CLK to U9, U10 and both connectors,
// and U10 divides that again for the 65C02.

module mainboard #(
	parameter bit BIOS_BYPASS = 1'b0
) (
	input  wire        clk_sys,
	input  wire        cold_reset,
	input  wire        reset_btn_n,

	// Savestate park, from ss_park.
	input  wire        pause_cpu,
	input  wire        pause_vid,
	output wire        ss_cpu_ready,
	output wire        ss_snd_ready,

	output wire        vid_overscan,

	input  wire        ss_sndram_own,
	input  wire [15:0] ss_sndram_addr,
	input  wire        ss_sndram_we,
	input  wire [7:0]  ss_sndram_wdata,
	output wire [7:0]  ss_sndram_q,
	output wire        ss_vid_ready,

	// Savestate scalar bus and memory ports.
	input  wire [63:0] ss_din,
	input  wire [9:0]  ss_addr,
	input  wire        ss_wren,
	input  wire        ss_rst,
	input  wire        ss_load,
	output wire [63:0] ss_dout,

	input  wire        ss_wram_own,
	input  wire [14:0] ss_wram_addr,
	input  wire        ss_wram_we,
	input  wire [1:0]  ss_wram_be,
	input  wire [15:0] ss_wram_wdata,
	output wire [15:0] ss_wram_q,

	input  wire        ss_vram_own,
	input  wire [15:0] ss_vram_addr,
	input  wire        ss_vram_we,
	input  wire [1:0]  ss_vram_be,
	input  wire [15:0] ss_vram_wdata,
	output wire [15:0] ss_vram_q,

	input  wire        ss_pal_own,
	input  wire [7:0]  ss_pal_addr,
	input  wire        ss_pal_we,
	input  wire [1:0]  ss_pal_be,
	input  wire [15:0] ss_pal_wdata,
	output wire [15:0] ss_pal_q,

	// MiSTer cheats. The engine sits between the board bus and the 68000's D
	// pins, so only what the CPU reads is patched.
	input  wire        cheat_load_begin,
	input  wire        cheat_invalidate,
	input  wire        cheat_commit_req,
	input  wire        cheat_paused,
	input  wire [128:0] cheat_code,
	output wire        cheat_commit_done,

	// BIOS load path into the UM6619 ROMs and the cartridge UM6650 key.
	input  wire        ld_wr,
	input  wire [14:0] ld_addr,
	input  wire [15:0] ld_data,

	// J6, the cartridge connector.
	output wire [23:1] CART_A,
	input  wire [15:0] CART_D_i,
	output wire [15:0] CART_D_o,
	output wire        CART_D_oe,
	output wire        CART_AS_n,
	output wire        CART_UDS_n,
	output wire        CART_LDS_n,
	output wire        CART_RW,
	output wire        CART_RESET_n,
	output wire        CART_MCLK,
	output wire        CART_CPUCLK,
	output wire        CART_26,
	output wire        CART_27,
	input  wire        CART_29,
	output wire        CART_66,
	input  wire        CARTIN_n,
	// Cartridge ROM lives in SDRAM, so !DTACK has to wait on it.
	input  wire        cart_ready,

	// P1 and P2, the joypad connectors.
	output wire        JOY1_1,
	input  wire        JOY1_3,
	output wire        JOY1_7,
	output wire        JOY2_1,
	input  wire        JOY2_3,
	output wire        JOY2_7,

	// U4 output.
	output wire [7:0]  VIDEO_R,
	output wire [7:0]  VIDEO_G,
	output wire [7:0]  VIDEO_B,
	output wire        VIDEO_HS,
	output wire        VIDEO_VS,
	output wire        VIDEO_HB,
	output wire        VIDEO_VB,
	output wire        VIDEO_DE,
	output wire        ce_pix,

	output wire [15:0] AUDIO_L,
	output wire [15:0] AUDIO_R
);
	// ---- clocks -----------------------------------------------------------

	wire ce_cpu_phi1, ce_cpu_phi2;

	assign CART_MCLK   = clk_sys;
	assign CART_CPUCLK = ce_cpu_phi1;

	// ---- the 68000 bus ----------------------------------------------------

	// While the UM6619 holds the bus these lines come from it.
	wire [23:1] CPU_A;
	wire        CPU_AS_n, CPU_UDS_n, CPU_LDS_n, CPU_RW;
	wire [23:1] cpu_a_68k, dma_a;
	wire        cpu_as_n_68k, cpu_uds_n_68k, cpu_lds_n_68k, cpu_rw_68k;
	wire        dma_as_n, dma_uds_n, dma_lds_n, dma_rw, dma_bus_oe;

	assign CPU_A      = dma_bus_oe ? dma_a      : cpu_a_68k;
	assign CPU_AS_n   = dma_bus_oe ? dma_as_n   : cpu_as_n_68k;
	assign CPU_UDS_n  = dma_bus_oe ? dma_uds_n  : cpu_uds_n_68k;
	assign CPU_LDS_n  = dma_bus_oe ? dma_lds_n  : cpu_lds_n_68k;
	assign CPU_RW     = dma_bus_oe ? dma_rw     : cpu_rw_68k;
	wire [15:0] CPU_D;
	wire [15:0] cpu_d_o, u10_d_o;
	wire        cpu_d_oe, u10_d_oe;

	wire        CPU_DTACK_n, CPU_VPA_n;
	wire        CPU_BR_n, CPU_BG_n, CPU_BGACK_n;
	wire        CPU_IPL_n_0, CPU_IPL_n_1, CPU_IPL_n_2;
	wire  [2:0] CPU_FC;
	wire        VMA_n, E;

	// !RESET and !HALT are open drain, pulled up, and shared between the CPU,
	// the UM6619 and both connectors. R63 pulls !AS up the same way.
	wire cpu_reset_n_o, cpu_reset_n_oe;
	wire cpu_halt_n_o, cpu_halt_n_oe;
	wire u10_reset_n_o, u10_reset_n_oe;
	wire u10_halt_n_o, u10_halt_n_oe;

	wire CPU_RESET_n = ~((cpu_reset_n_oe & ~cpu_reset_n_o)
	                   | (u10_reset_n_oe & ~u10_reset_n_o));
	wire CPU_HALT_n  = ~((cpu_halt_n_oe & ~cpu_halt_n_o)
	                   | (u10_halt_n_oe & ~u10_halt_n_o));

	wire [7:0] u1_d_o, u2_d_o;
	wire       u1_d_oe, u2_d_oe;
	wire       WRAM_LSEL_n, WRAM_HSEL_n;

	assign CPU_D[7:0]  = cpu_d_oe ? cpu_d_o[7:0]
	                   : u10_d_oe ? u10_d_o[7:0]
	                   : u12_d_oe ? u12_d_o[7:0]
	                   : u1_d_oe  ? u1_d_o
	                   : CART_D_i[7:0];
	assign CPU_D[15:8] = cpu_d_oe ? cpu_d_o[15:8]
	                   : u10_d_oe ? u10_d_o[15:8]
	                   : u12_d_oe ? u12_d_o[15:8]
	                   : u2_d_oe  ? u2_d_o
	                   : CART_D_i[15:8];

	// The cheat engine watches the 68000's own address and strobes so a DMA
	// cycle cannot disturb it. Only the 68000 sees its output.
	wire [15:0] CPU_D_cheat;

	acan_cheat_engine cheats
	(
		.clk_sys_i     (clk_sys),
		.load_begin_i  (cheat_load_begin),
		.invalidate_i  (cheat_invalidate),
		.commit_req_i  (cheat_commit_req),
		.paused_i      (cheat_paused),
		.code_i        (cheat_code),
		.commit_done_o (cheat_commit_done),

		.enable_i      (cpu_rw_68k & ~cpu_as_n_68k & ~dma_bus_oe),
		.addr_i        (cpu_a_68k),
		.be_i          ({~cpu_uds_n_68k, ~cpu_lds_n_68k}),
		.data_i        (CPU_D),
		.data_o        (CPU_D_cheat)
	);

	assign CART_A     = CPU_A;
	assign CART_D_o   = cpu_d_o;
	assign CART_D_oe  = cpu_d_oe;
	assign CART_AS_n  = CPU_AS_n;
	assign CART_UDS_n = CPU_UDS_n;
	assign CART_LDS_n = CPU_LDS_n;
	assign CART_RW    = CPU_RW;
	assign CART_RESET_n = CPU_RESET_n;

	// ---- U9, the CPU ------------------------------------------------------

	mc68000 U9
	(
		.clk_sys    (clk_sys),
		.ce_phi1    (ce_cpu_phi1),
		.ce_phi2    (ce_cpu_phi2),
		.pwr_up     (cold_reset),

		.A          (cpu_a_68k),
		.D_i        (CPU_D_cheat),
		.D_o        (cpu_d_o),
		.D_oe       (cpu_d_oe),

		.AS_n       (cpu_as_n_68k),
		.UDS_n      (cpu_uds_n_68k),
		.LDS_n      (cpu_lds_n_68k),
		.RW         (cpu_rw_68k),
		.DTACK_n    (CPU_DTACK_n),
		.BERR_n     (1'b1),           // tied to VCC on this board
		.VPA_n      (CPU_VPA_n),
		.VMA_n      (VMA_n),
		.E          (E),

		.BR_n       (CPU_BR_n),
		.BG_n       (CPU_BG_n),
		.BGACK_n    (CPU_BGACK_n),

		.IPL0_n     (CPU_IPL_n_0),
		.IPL1_n     (CPU_IPL_n_1),
		.IPL2_n     (CPU_IPL_n_2),

		.FC0        (CPU_FC[0]),
		.FC1        (CPU_FC[1]),
		.FC2        (CPU_FC[2]),

		.RESET_n_i  (CPU_RESET_n),
		.RESET_n_o  (cpu_reset_n_o),
		.RESET_n_oe (cpu_reset_n_oe),
		.HALT_n_i   (CPU_HALT_n),
		.HALT_n_o   (cpu_halt_n_o),
		.HALT_n_oe  (cpu_halt_n_oe),

		.ss_ready   (ss_cpu_ready),

		.ss_load    (ss_load),
		.ss_din     (ss_din),
		.ss_addr    (ss_addr),
		.ss_wren    (ss_wren),
		.ss_rst     (ss_rst),
		.ss_dout    (ss_dout_68k)
	);
	wire [63:0] ss_dout_68k;

	// ---- U10, the system ASIC ---------------------------------------------

	/* verilator lint_off UNUSEDSIGNAL */
	wire [63:0] ss_dout_6619;
	// 135: interrupt acknowledge from the UM6619. 136: sprite DMA token.
	// 134: acknowledge while one runs.
	wire        net_6618_135, net_6618_136;
	wire [15:0] SNDRAM_A;
	wire [7:0]  sndram_d_i;
	wire  [7:0] sndram_d_o;
	wire        sndram_d_oe, SNDRAM_CEOE_n, SNDRAM_WE_n;
	wire        net_6618_126, net_6618_127, net_6618_134;
	wire        net_6618_137;
	wire [63:0] ss_dout_70c188;
	/* verilator lint_on UNUSEDSIGNAL */

	// Register each chip's readback to keep its state mux local.
	reg [63:0] ss_dout_68k_q = 64'd0, ss_dout_6619_q = 64'd0;
	reg [63:0] ss_dout_6618_q = 64'd0, ss_dout_70c188_q = 64'd0;
	always @(posedge clk_sys) begin
		ss_dout_68k_q <= ss_dout_68k;
		ss_dout_6619_q <= ss_dout_6619;
		ss_dout_6618_q <= ss_dout_6618;
		ss_dout_70c188_q <= ss_dout_70c188;
	end
	assign ss_dout = ss_dout_68k_q | ss_dout_6619_q | ss_dout_6618_q | ss_dout_70c188_q;

	um6619 #(.BIOS_BYPASS(BIOS_BYPASS)) U10
	(
		.clk_sys        (clk_sys),
		.ce_phi1        (ce_cpu_phi1),
		.ce_phi2        (ce_cpu_phi2),

		.CPU_A          (CPU_A),
		.CPU_D_i        (CPU_D),
		.CPU_D_o        (u10_d_o),
		.CPU_D_oe       (u10_d_oe),
		.CPU_AS_n       (CPU_AS_n),
		.CPU_UDS_n      (CPU_UDS_n),
		.CPU_LDS_n      (CPU_LDS_n),
		.CPU_RW         (CPU_RW),
		.CPU_FC         (CPU_FC),

		.CPU_DTACK_n    (CPU_DTACK_n),
		.CPU_VPA_n      (CPU_VPA_n),
		.CPU_IPL_n_0    (CPU_IPL_n_0),
		.CPU_IPL_n_1    (CPU_IPL_n_1),
		.CPU_IPL_n_2    (CPU_IPL_n_2),
		.CPU_HALT_n     (u10_halt_n_o),
		.CPU_HALT_n_oe  (u10_halt_n_oe),
		.CPU_RESET_n_i  (CPU_RESET_n),
		.CPU_RESET_n    (u10_reset_n_o),
		.CPU_RESET_n_oe (u10_reset_n_oe),

		.CPU_BR_n       (CPU_BR_n),
		.CPU_BG_n       (CPU_BG_n),
		.CPU_BGACK_n    (CPU_BGACK_n),
		.CPU_A_o        (dma_a),
		.CPU_AS_n_o     (dma_as_n),
		.CPU_UDS_n_o    (dma_uds_n),
		.CPU_LDS_n_o    (dma_lds_n),
		.CPU_RW_o       (dma_rw),
		.CPU_BUS_oe     (dma_bus_oe),

		.WRAM_LSEL_n    (WRAM_LSEL_n),
		.WRAM_HSEL_n    (WRAM_HSEL_n),

		.SNDRAM_A       (SNDRAM_A),
		.SNDRAM_D_i     (sndram_d_i),
		.SNDRAM_D_o     (sndram_d_o),
		.SNDRAM_D_oe    (sndram_d_oe),
		.SNDRAM_CEOE_n  (SNDRAM_CEOE_n),
		.SNDRAM_WE_n    (SNDRAM_WE_n),

		.net_6618_126   (net_6618_126),
		.net_6618_127   (net_6618_127),
		.net_6618_134   (net_6618_134),
		.net_6618_135   (net_6618_135),
		.net_6618_136   (net_6618_136),

		.CART_26        (CART_26),
		.CART_27        (CART_27),
		.CART_29        (CART_29),
		.CART_66        (CART_66),
		.cart_ready     (cart_ready),

		.EXP_21         (1'b1),
		.EXP_22         (1'b1),
		.EXP_25         (1'b1),

		.JOY1_1         (JOY1_1),
		.JOY1_3         (JOY1_3),
		.JOY1_7         (JOY1_7),
		.JOY2_1         (JOY2_1),
		.JOY2_3         (JOY2_3),
		.JOY2_7         (JOY2_7),

		.net_6619_85    (1'b1),
		.net_6619_86    (),
		.net_6619_87    (),
		.net_6619_88    (),
		.net_6619_89    (1'b1),
		.net_6619_90    (1'b1),
		.net_6619_91    (),

		.RESET_BTN_n    (reset_btn_n),

		.AUDIO_L        (AUDIO_L),
		.AUDIO_R        (AUDIO_R),

		.cold_reset     (cold_reset),

		.ld_wr          (ld_wr),
		.ld_addr        (ld_addr),
		.ld_data        (ld_data),

		.ss_din         (ss_din),
		.ss_addr        (ss_addr),
		.ss_wren        (ss_wren),
		.ss_rst         (ss_rst),
		.ss_load        (ss_load),
		.ss_dout        (ss_dout_6619),

		.ss_ready       (ss_snd_ready)
	);

	// ---- U11, sound RAM ----------------------------------------------------

	// !CS and !OE share a net. The UM6619 drives all three pins itself since
	// two CPUs and the sampler share this part.
	sram_async #(.ADDR_WIDTH(16), .DATA_WIDTH(8)) U11
	(
		.clk_sys (clk_sys),
		.A       (SNDRAM_A),
		.CS_n    (SNDRAM_CEOE_n),
		.OE_n    (SNDRAM_CEOE_n),
		.WE_n    (SNDRAM_WE_n),
		.D_i     (sndram_d_oe ? sndram_d_o : 8'hFF),
		.D_o     (sndram_d_i),
		/* verilator lint_off PINCONNECTEMPTY */
		.D_oe    (),
		/* verilator lint_on PINCONNECTEMPTY */
		.ss_own  (ss_sndram_own),
		.ss_addr (ss_sndram_addr),
		.ss_we   (ss_sndram_we),
		.ss_din  (ss_sndram_wdata),
		.ss_dout (ss_sndram_q),
		.bk_addr ('0),
		.bk_wren (1'b0),
		.bk_be   (2'b00),
		.bk_wdata(16'd0),
		/* verilator lint_off PINCONNECTEMPTY */
		.bk_q    (),
		.dirty   ()
		/* verilator lint_on PINCONNECTEMPTY */
	);

	// ---- U1 and U2, work RAM ----------------------------------------------

	// !CS and !OE share a net and !WE is the CPU's R/W line.
	sram_async #(.ADDR_WIDTH(15), .DATA_WIDTH(8)) U1
	(
		.clk_sys (clk_sys),
		.A       (CPU_A[15:1]),
		.CS_n    (WRAM_LSEL_n),
		.OE_n    (WRAM_LSEL_n),
		.WE_n    (CPU_RW),
		.D_i     (CPU_D[7:0]),
		.D_o     (u1_d_o),
		.D_oe    (u1_d_oe),
		.ss_own  (ss_wram_own),
		.ss_addr (ss_wram_addr),
		.ss_we   (ss_wram_we & ss_wram_be[0]),
		.ss_din  (ss_wram_wdata[7:0]),
		.ss_dout (ss_wram_q[7:0]),
		.bk_addr ('0),
		.bk_wren (1'b0),
		.bk_be   (2'b00),
		.bk_wdata(16'd0),
		/* verilator lint_off PINCONNECTEMPTY */
		.bk_q    (),
		.dirty   ()
		/* verilator lint_on PINCONNECTEMPTY */
	);

	sram_async #(.ADDR_WIDTH(15), .DATA_WIDTH(8)) U2
	(
		.clk_sys (clk_sys),
		.A       (CPU_A[15:1]),
		.CS_n    (WRAM_HSEL_n),
		.OE_n    (WRAM_HSEL_n),
		.WE_n    (CPU_RW),
		.D_i     (CPU_D[15:8]),
		.D_o     (u2_d_o),
		.D_oe    (u2_d_oe),
		.ss_own  (ss_wram_own),
		.ss_addr (ss_wram_addr),
		.ss_we   (ss_wram_we & ss_wram_be[1]),
		.ss_din  (ss_wram_wdata[15:8]),
		.ss_dout (ss_wram_q[15:8]),
		.bk_addr ('0),
		.bk_wren (1'b0),
		.bk_be   (2'b00),
		.bk_wdata(16'd0),
		/* verilator lint_off PINCONNECTEMPTY */
		.bk_q    (),
		.dirty   ()
		/* verilator lint_on PINCONNECTEMPTY */
	);

	// ---- U12, the video ASIC, and the parts around it ----------------------

	wire [16:1] VRAM_A;
	wire [15:0] vram_d_o;
	wire        vram_d_oe, VRAM_OE_n, VRAM_WE_n;
	wire [7:0]  u5_d_o, u6_d_o;
	/* verilator lint_off UNUSEDSIGNAL */
	wire        u5_d_oe, u6_d_oe, VRAM_A17;
	wire        CSYNC, SUBCARR;
	wire [63:0] ss_dout_6618;
	/* verilator lint_on UNUSEDSIGNAL */
	wire [15:0] VRAM_D = vram_d_oe ? vram_d_o : {u6_d_o, u5_d_o};

	wire [7:0] DAC_P, dac_d_o, dac_d_i;
	wire [1:0] DAC_RS;
	wire       DAC_PCLK, DAC_RD_n, DAC_WR_n, DAC_BLANK_n;
	wire [7:0] dac_r, dac_g, dac_b;
	wire [15:0] u12_d_o;
	wire        u12_d_oe;

	um6618 U12
	(
		.clk_sys      (clk_sys),
		.ce_cpu_phi1  (ce_cpu_phi1),
		.ce_cpu_phi2  (ce_cpu_phi2),

		.CPU_A        (CPU_A),
		.CPU_D_i      (CPU_D),
		.CPU_D_o      (u12_d_o),
		.CPU_D_oe     (u12_d_oe),
		.CPU_AS_n     (CPU_AS_n),
		.CPU_RW       (CPU_RW),
		.CPU_RESET_n  (CPU_RESET_n),

		.VRAM_A       (VRAM_A),
		.VRAM_A17     (VRAM_A17),
		.VRAM_D_i     (VRAM_D),
		.VRAM_D_o     (vram_d_o),
		.VRAM_D_oe    (vram_d_oe),
		.VRAM_OE_n    (VRAM_OE_n),
		.VRAM_WE_n    (VRAM_WE_n),

		.DAC_P        (DAC_P),
		.DAC_PCLK     (DAC_PCLK),
		.DAC_D_o      (dac_d_o),
		.DAC_D_i      (dac_d_i),
		.DAC_RS       (DAC_RS),
		.DAC_RD_n     (DAC_RD_n),
		.DAC_WR_n     (DAC_WR_n),
		.DAC_BLANK_n  (DAC_BLANK_n),

		.CSYNC        (CSYNC),
		.SUBCARR      (SUBCARR),

		.net_6618_126 (net_6618_126),
		.net_6618_127 (net_6618_127),
		.net_6618_134 (net_6618_134),
		.net_6618_135 (net_6618_135),
		.net_6618_136 (net_6618_136),

		.CART_31      (1'b1),
		.CART_32      (1'b1),
		.CART_34      (1'b1),
		.CART_68      (1'b1),
		.net_6618_137 (net_6618_137),
		.JOY_2        (1'b1),
		.JP1          (1'b1),
		.JP2          (1'b1),

		.ce_pix       (ce_pix),
		.hsync        (VIDEO_HS),
		.vsync        (VIDEO_VS),
		.hblank       (VIDEO_HB),
		.vblank       (VIDEO_VB),
		.de           (VIDEO_DE),

		.pause_cpu    (pause_cpu),
		.pause_vid    (pause_vid),
		.ss_ready     (ss_vid_ready),

		.ss_din       (ss_din),
		.ss_addr      (ss_addr),
		.ss_wren      (ss_wren),
		.ss_rst       (ss_rst),
		.ss_dout      (ss_dout_6618),
		.overscan     (vid_overscan)
	);

	// U5 and U6, VRAM. The board carries UM61512AK (64K x 8) where the
	// schematic drew a UM611024 with a seventeenth address line.
	sram_async #(.ADDR_WIDTH(16), .DATA_WIDTH(8)) U5
	(
		.clk_sys (clk_sys),
		.A       (VRAM_A),
		.CS_n    (1'b0),
		.OE_n    (VRAM_OE_n),
		.WE_n    (VRAM_WE_n),
		.D_i     (VRAM_D[7:0]),
		.D_o     (u5_d_o),
		.D_oe    (u5_d_oe),
		.ss_own  (ss_vram_own),
		.ss_addr (ss_vram_addr),
		.ss_we   (ss_vram_we & ss_vram_be[0]),
		.ss_din  (ss_vram_wdata[7:0]),
		.ss_dout (ss_vram_q[7:0]),
		.bk_addr ('0),
		.bk_wren (1'b0),
		.bk_be   (2'b00),
		.bk_wdata(16'd0),
		/* verilator lint_off PINCONNECTEMPTY */
		.bk_q    (),
		.dirty   ()
		/* verilator lint_on PINCONNECTEMPTY */
	);

	sram_async #(.ADDR_WIDTH(16), .DATA_WIDTH(8)) U6
	(
		.clk_sys (clk_sys),
		.A       (VRAM_A),
		.CS_n    (1'b0),
		.OE_n    (VRAM_OE_n),
		.WE_n    (VRAM_WE_n),
		.D_i     (VRAM_D[15:8]),
		.D_o     (u6_d_o),
		.D_oe    (u6_d_oe),
		.ss_own  (ss_vram_own),
		.ss_addr (ss_vram_addr),
		.ss_we   (ss_vram_we & ss_vram_be[1]),
		.ss_din  (ss_vram_wdata[15:8]),
		.ss_dout (ss_vram_q[15:8]),
		.bk_addr ('0),
		.bk_wren (1'b0),
		.bk_be   (2'b00),
		.bk_wdata(16'd0),
		/* verilator lint_off PINCONNECTEMPTY */
		.bk_q    (),
		.dirty   ()
		/* verilator lint_on PINCONNECTEMPTY */
	);

	um70c188 U3
	(
		.clk_sys   (clk_sys),
		.reset     (~CPU_RESET_n),
		.D_i       (dac_d_o),
		.D_o       (dac_d_i),
		.RS        (DAC_RS),
		.RD_n      (DAC_RD_n),
		.WR_n      (DAC_WR_n),
		.PCLK      (DAC_PCLK),
		.P         (DAC_P),
		.BLANK_n   (DAC_BLANK_n),
		.ROUT      (dac_r),
		.GOUT      (dac_g),
		.BOUT      (dac_b),
		.ss_own    (ss_pal_own),
		.ss_addr_b (ss_pal_addr),
		.ss_we     (ss_pal_we),
		.ss_be     (ss_pal_be),
		.ss_wdata  (ss_pal_wdata),
		.ss_q      (ss_pal_q),
		.ss_din    (ss_din),
		.ss_addr   (ss_addr),
		.ss_wren   (ss_wren),
		.ss_rst    (ss_rst),
		.ss_dout   (ss_dout_70c188)
	);

	ka2195d U4
	(
		.R_i     (dac_r),
		.G_i     (dac_g),
		.B_i     (dac_b),
		.CSYNC_i (CSYNC),
		.R_o     (VIDEO_R),
		.G_o     (VIDEO_G),
		.B_o     (VIDEO_B),
		/* verilator lint_off PINCONNECTEMPTY */
		.CSYNC_o ()
		/* verilator lint_on PINCONNECTEMPTY */
	);

	`ifdef VERILATOR
	/* verilator lint_off UNUSEDSIGNAL */
	wire unused = &{1'b0, VMA_n, E, VRAM_A17, SUBCARR, u5_d_oe, u6_d_oe,
	                CARTIN_n, net_6618_136};
	/* verilator lint_on UNUSEDSIGNAL */
	`endif
endmodule
