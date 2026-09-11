// Copyright (c) 2026 Jamie Blanks

// U10, UM6619F: system, sound and I/O. No datasheet exists; this is a model
// of what the software demands, behind the schematic's pin names with _n
// for active low.
//
// It owns the 68000 address decode outside the UM6618's windows, !DTACK and
// !VPA, the work RAM selects, the interrupt encoder, the overlay latches,
// two internal ROMs, two DMA channels, the free running counter, the 65C02
// and its shared RAM, the I/O page, both joypad ports and the sampler.
//
// !BERR is tied high on this board, so an unanswered address hangs the
// machine rather than trapping.

module um6619 #(
	// Both overlay latches come up off, so the 68000 boots from the
	// cartridge and the authorisation check never runs.
	parameter bit BIOS_BYPASS = 1'b0
) (
	// CPU_CLK from the UM6618, as two phase enables.
	input  wire        clk_sys,
	input  wire        ce_phi1,
	input  wire        ce_phi2,

	// 68000 bus, slave side.
	input  wire [23:1] CPU_A,
	input  wire [15:0] CPU_D_i,
	output wire [15:0] CPU_D_o,
	output wire        CPU_D_oe,
	input  wire        CPU_AS_n,
	input  wire        CPU_UDS_n,
	input  wire        CPU_LDS_n,
	input  wire        CPU_RW,
	input  wire [2:0]  CPU_FC,

	// 68000 bus, driven lines.
	output wire        CPU_DTACK_n,
	output wire        CPU_VPA_n,
	output wire        CPU_IPL_n_0,
	output wire        CPU_IPL_n_1,
	output wire        CPU_IPL_n_2,
	output wire        CPU_HALT_n,
	output wire        CPU_HALT_n_oe,
	input  wire        CPU_RESET_n_i,
	output wire        CPU_RESET_n,
	output wire        CPU_RESET_n_oe,

	// Bus arbitration for the DMA channels.
	output wire        CPU_BR_n,
	input  wire        CPU_BG_n,
	output wire        CPU_BGACK_n,

	// The bus while this chip holds it; the same pins as the slave side with
	// the direction split out.
	output wire [23:1] CPU_A_o,
	output wire        CPU_AS_n_o,
	output wire        CPU_UDS_n_o,
	output wire        CPU_LDS_n_o,
	output wire        CPU_RW_o,
	output wire        CPU_BUS_oe,

	// Work RAM chip selects; !CS and !OE are tied together on the board.
	output wire        WRAM_LSEL_n,
	output wire        WRAM_HSEL_n,

	// Sound RAM, U11.
	output wire [15:0] SNDRAM_A,
	input  wire [7:0]  SNDRAM_D_i,
	output wire [7:0]  SNDRAM_D_o,
	output wire        SNDRAM_D_oe,
	output wire        SNDRAM_CEOE_n,
	output wire        SNDRAM_WE_n,

	// To the UM6618, roles inferred: video interrupt requests, an
	// acknowledge, and the sprite DMA pair. 136 goes out with each source
	// word; 134 comes back with each acknowledgement, borrowed from the
	// retrace request while a command runs.
	input  wire        net_6618_126,
	input  wire        net_6618_127,
	input  wire        net_6618_134,
	output wire        net_6618_135,
	output wire        net_6618_136,

	// To the cartridge. 26 enables the mask ROMs, 27 and 66 are the UM6650's
	// ports, 29 goes through the pad glue.
	output wire        CART_26,
	output wire        CART_27,
	input  wire        CART_29,
	output wire        CART_66,

	// Cartridge ROM lives in SDRAM, so !DTACK has to wait on it.
	input  wire        cart_ready,


	// Expansion edge.
	input  wire        EXP_21,
	input  wire        EXP_22,
	input  wire        EXP_25,

	// Joypads. 1 latch, 3 data in, 7 shift clock.
	output wire        JOY1_1,
	input  wire        JOY1_3,
	output wire        JOY1_7,
	output wire        JOY2_1,
	input  wire        JOY2_3,
	output wire        JOY2_7,

	// Pad glue chain, U16 to U21.
	input  wire        net_6619_85,
	output wire        net_6619_86,
	output wire        net_6619_87,
	output wire        net_6619_88,
	input  wire        net_6619_89,
	input  wire        net_6619_90,
	output wire        net_6619_91,

	// Front panel.
	input  wire        RESET_BTN_n,

	// Audio, signed.
	output wire [15:0] AUDIO_L,
	output wire [15:0] AUDIO_R,

	// Framework power-on reset, BIOS load path and savestate bus.
	input  wire        cold_reset,

	input  wire        ld_wr,
	input  wire [14:0] ld_addr,     // byte address inside the BIOS image
	input  wire [15:0] ld_data,

	input  wire [63:0] ss_din,
	input  wire [9:0]  ss_addr,
	input  wire        ss_wren,
	input  wire        ss_rst,
	input  wire        ss_load,
	output wire [63:0] ss_dout,

	// Savestate park condition.
	output wire        ss_ready
);
	`include "ss_map.svh"

	// ---- address decode ---------------------------------------------------

	wire valid = ~CPU_AS_n & (~CPU_UDS_n | ~CPU_LDS_n);

	// Cartridge ROM windows. The IPL overlays the first 4 KiB of each until
	// its latch is set.
	wire sel_cart_lo = (CPU_A[23:22] == 2'b00);            // $000000-$3FFFFF
	wire sel_cart_hi = (CPU_A[23:18] == 6'b111110);        // $F80000-$FBFFFF

	wire ipl_lo = sel_cart_lo & low_overlay  & (CPU_A[23:12] == 12'h000);
	wire ipl_hi = sel_cart_hi & high_overlay & (CPU_A[23:12] == 12'hF80);
	wire sel_ipl = ipl_lo | ipl_hi;

	// Work RAM at $FC0000, mirrored every 64 KiB to $FFFFFF; the IPL uses
	// $FC0000 and $FD000A as one memory.
	wire sel_wram = (CPU_A[23:18] == 6'b111111);

	wire sel_sndram = (CPU_A[23:16] == 8'hE8);

	wire sel_host = (CPU_A[23:16] == 8'hE9) & (CPU_A[15:5] == 11'd0);
	wire sel_dma  = (CPU_A[23:16] == 8'hE9) & (CPU_A[15:6] == 10'd0) & CPU_A[5];
	wire sel_egg  = (CPU_A[23:16] == 8'hE9) & (CPU_A[15:1] == 15'h059E);
	wire sel_lock = (CPU_A[23:16] == 8'hEB) & (CPU_A[15:2] == 14'h0340);

	// Video registers and palette. The UM6618 has no !DTACK pin, so this
	// chip answers for it.
	wire sel_vregs = (CPU_A[23:10] == 14'h3C00);          // $F00000-$F003FF

	// Interrupt acknowledge: FC 7 with the level on A1-A3 and every other
	// address line high. Answered with !VPA for an autovector, never !DTACK.
	// The function codes hold 7 while the bus idles after an acknowledge,
	// which is when a DMA takes it, so a cycle this chip drives is never one.
	wire iack       = ~CPU_AS_n & (CPU_FC == 3'b111) & ~CPU_BUS_oe;
	wire [2:0] iack_level = CPU_A[3:1];

	// Every cycle is answered, since !BERR is tied high and an unanswered
	// cycle hangs the machine; the windows only decide who drives the data.
	// Sango Fighter's match setup copies 32 words past the end of the
	// palette and expects those cycles to complete. Interrupt acknowledge is
	// the exception, answered with !VPA instead.
	wire bus_ack = ~iack;

	// An acknowledge cycle's all-ones address falls in the work RAM window.
	assign WRAM_LSEL_n = ~(valid & ~iack & sel_wram & ~CPU_LDS_n);
	assign WRAM_HSEL_n = ~(valid & ~iack & sel_wram & ~CPU_UDS_n);
	assign CART_26     = ~(valid & (sel_cart_lo | sel_cart_hi) & ~sel_ipl);

	// Waits: a cartridge ROM read, and a sound RAM access (two byte cycles on
	// a device the 65C02 may be using).
	wire rom_window = (sel_cart_lo | sel_cart_hi) & ~sel_ipl;
	// Registered to split the decode and sampling paths.
	reg dtack_n_q = 1'b1;
	always @(posedge clk_sys)
		dtack_n_q <= ~(valid & bus_ack & (~rom_window | cart_ready)
		                             & (~sel_sndram | snd_win_held));
	assign CPU_DTACK_n = dtack_n_q;

	wire wr = valid & ~CPU_RW;

	// ---- the host port, $E90000-$E9001F ------------------------------------

	wire host_wr = wr & sel_host;
	wire [4:1] host_reg = CPU_A[4:1];

	wire        low_overlay, high_overlay, snd_reset;
	wire [15:0] overlay_dout, irq_mask_dout;
	wire [15:0] frc_ctrl_dout, frc_reload_dout, frc_value_dout;
	wire        frc_irq;

	wire [63:0] ss_dout_overlay, ss_dout_irq, ss_dout_frc, ss_dout_misc;
	wire [63:0] ss_dout_snd_cpu, ss_dout_snd, ss_dout_dma;
	assign ss_dout = ss_dout_overlay | ss_dout_irq | ss_dout_frc | ss_dout_misc
	               | ss_dout_snd_cpu | ss_dout_snd | ss_dout_dma;

	um6619_overlay #(.BIOS_BYPASS(BIOS_BYPASS)) overlay
	(
		.clk_sys      (clk_sys),
		.reset        (~CPU_RESET_n_i),
		.wr           (host_wr & (host_reg == 4'hE)),
		.din          (CPU_D_i),
		.dout         (overlay_dout),
		.low_overlay  (low_overlay),
		.high_overlay (high_overlay),
		.snd_reset    (snd_reset),
		.ss_din       (ss_din),
		.ss_addr      (ss_addr),
		.ss_wren      (ss_wren),
		.ss_rst       (ss_rst),
		.ss_dout      (ss_dout_overlay)
	);

	um6619_irq irqs
	(
		.clk_sys        (clk_sys),
		.reset          (~CPU_RESET_n_i),
		.mask_wr        (host_wr & (host_reg == 4'h8)),
		.mask_din       (CPU_D_i),
		.mask_dout      (irq_mask_dout),
		.req_expansion  (1'b0),
		.req_cartridge  (1'b0),
		.req_frc        (frc_irq),
		.req_line       (net_6618_126),
		.req_line_onoff (net_6618_127),
		.req_mailbox    (mailbox_irq),
		.req_vblank     (vblank_level),
		.ipl_n          ({CPU_IPL_n_2, CPU_IPL_n_1, CPU_IPL_n_0}),
		.ss_din         (ss_din),
		.ss_addr        (ss_addr),
		.ss_wren        (ss_wren),
		.ss_rst         (ss_rst),
		.ss_dout        (ss_dout_irq)
	);

	um6619_frc frc
	(
		.clk_sys     (clk_sys),
		.reset       (~CPU_RESET_n_i),
		.ce_phi1     (ce_phi1),
		.vblank_tick (vblank_edge),
		.ctrl_wr     (host_wr & (host_reg == 4'hA)),
		.ctrl_din    (CPU_D_i),
		.ctrl_dout   (frc_ctrl_dout),
		.reload_wr   (host_wr & (host_reg == 4'hB)),
		.reload_din  (CPU_D_i),
		.reload_dout (frc_reload_dout),
		.value_dout  (frc_value_dout),
		.irq         (frc_irq),
		// Cleared by reading the current value or by the 68000 taking the
		// interrupt; Sonic Dragon's IRQ3 handler never reads the count.
		.irq_ack     ((valid & CPU_RW & sel_host & (host_reg == 4'hC))
		              | (iack & (iack_level == 3'd3))),
		.ss_din      (ss_din),
		.ss_addr     (ss_addr),
		.ss_wren     (ss_wren),
		.ss_rst      (ss_rst),
		.ss_dout     (ss_dout_frc)
	);

	// The IPL writes nine constants to $E90B3C during its security check
	// (UMC's Chinese company name) and reads it once at $41C. Decoded so the
	// read gets a defined bit 0.
	wire [63:0] ss_now_misc;
	reg [15:0] egg;
	always @(posedge clk_sys)
		if (~CPU_RESET_n_i) egg <= ss_now_misc[15:0];
		else if (wr & sel_egg) egg <= CPU_D_i;

	// ---- the internal ROMs -------------------------------------------------

	wire [15:0] ipl_q;
	wire [7:0]  smp_q;

	// The BIOS image: 4 KiB of 68000 IPL, two 8 KiB sound RAM images, then
	// the UM6650's sixteen key bytes.
	wire ld_ipl = ld_wr & (ld_addr[14:12] == 3'd0);
	wire ld_smp = ld_wr & (ld_addr[14:12] >= 3'd1) & (ld_addr[14:12] <= 3'd4);

	um6619_iplrom iplrom
	(
		.clk_sys (clk_sys),
		.addr    (CPU_A[11:1]),
		.q       (ipl_q),
		.ld_wr   (ld_ipl),
		.ld_addr (ld_addr[11:1]),
		.ld_data (ld_data)
	);

	um6619_smprom smprom
	(
		.clk_sys (clk_sys),
		.addr    (boot_addr),
		.q       (smp_q),
		.ld_wr   (ld_smp),
		.ld_addr (ld_addr[13:1] - 13'h0800),
		.ld_data (ld_data)
	);

	// ---- what this chip puts on the data bus -------------------------------

	reg [15:0] rdata;
	always @(*) begin
		if      (sel_ipl)  rdata = ipl_q;
		else if (sel_sndram) rdata = snd_rdata;
		else if (sel_dma)    rdata = dma_dout;
		else if (sel_host) begin
			case (host_reg)
				4'h2:    rdata = {io_040c, io_040d};
				// One byte in both halves; Super Taiwanese Baseball League
				// reads the odd byte alone.
				4'h6:    rdata = {io_040a, io_040a};
				4'h8:    rdata = irq_mask_dout;
				4'hA:    rdata = frc_ctrl_dout;
				4'hB:    rdata = frc_reload_dout;
				4'hC:    rdata = frc_value_dout;
				4'hE:    rdata = overlay_dout;
				default: rdata = 16'd0;
			endcase
		end
		else if (sel_egg) rdata = egg;
		else              rdata = 16'd0;
	end

	assign CPU_D_o  = CPU_BUS_oe ? dma_d_o : rdata;
	// The lockout window is answered by the cartridge's UM6650.
	assign CPU_D_oe = (CPU_BUS_oe & dma_d_oe)
	                | (valid & CPU_RW
	                   & (sel_ipl | sel_host | sel_dma | sel_egg | sel_sndram));

	// ---- reset -------------------------------------------------------------

	// !RESET is open drain, shared with the 68000's own RESET output.
	assign CPU_RESET_n    = 1'b0;
	assign CPU_RESET_n_oe = cold_reset | ~RESET_BTN_n;

	assign CPU_HALT_n    = 1'b0;
	assign CPU_HALT_n_oe = 1'b0;

	// ---- the sound CPU and everything on its side --------------------------

	// The 65C02 is held while $E9001C bit 0 is clear. Only the CPU stops; the
	// sound RAM arbiter keeps serving the 68000's window.
	wire c65_reset = ~CPU_RESET_n_i;
	wire c65_halt  = ~snd_reset;

	// One sound RAM access per 68000 strobe.
	reg  snd_win_held;
	wire snd_win_done;
	wire [15:0] snd_win_dout;
	reg  [15:0] snd_rdata;

	always @(posedge clk_sys) begin
		if (~CPU_RESET_n_i) begin
			snd_win_held <= ss_now_misc[32];
			snd_rdata    <= ss_now_misc[31:16];
		end else if (!(valid & sel_sndram)) begin
			snd_win_held <= 1'b0;
		end else if (snd_win_done) begin
			snd_win_held <= 1'b1;
			snd_rdata    <= snd_win_dout;
		end
	end

	wire [7:0] io_040a, io_040c, io_040d;
	wire       mailbox_pulse;
	reg        mailbox_irq;

	// Level held until the 68000 reads $E9000C.
	always @(posedge clk_sys) begin
		if (~CPU_RESET_n_i)                                          mailbox_irq <= ss_now_misc[33];
		else if (mailbox_pulse)                                      mailbox_irq <= 1'b1;
		else if (valid & CPU_RW & sel_host & (host_reg == 4'h6))     mailbox_irq <= 1'b0;
		else if (iack & (iack_level == 3'd6))                        mailbox_irq <= 1'b0;
	end

	um6619_cpu sound_side
	(
		.clk_sys       (clk_sys),
		.reset         (c65_reset),
		.c65_halt      (c65_halt),
		.ce_phi1       (ce_phi1),
		.ce_phi2       (ce_phi2),
		.SNDRAM_A      (SNDRAM_A),
		.SNDRAM_D_i    (SNDRAM_D_i),
		.SNDRAM_D_o    (SNDRAM_D_o),
		.SNDRAM_D_oe   (SNDRAM_D_oe),
		.SNDRAM_CEOE_n (SNDRAM_CEOE_n),
		.SNDRAM_WE_n   (SNDRAM_WE_n),
		.win_req       (valid & sel_sndram & ~snd_win_held),
		.win_we        (~CPU_RW),
		.win_addr      (CPU_A[15:1]),
		.win_be        ({~CPU_UDS_n, ~CPU_LDS_n}),
		.win_din       (CPU_D_i),
		.win_dout      (snd_win_dout),
		.win_done      (snd_win_done),
		.cmd_write     (host_wr & (host_reg == 4'h5)),
		.io_040a       (io_040a),
		.io_040c       (io_040c),
		.io_040d       (io_040d),
		.irq6          (mailbox_pulse),
		.vblank_nmi    (vblank_edge & irq_mask_dout[7]),

		.ss_load       (ss_load),
		.ss_din        (ss_din),
		.ss_addr       (ss_addr),
		.ss_wren       (ss_wren),
		.ss_rst        (ss_rst),
		.ss_dout       (ss_dout_snd_cpu),

		.JOY1_1        (JOY1_1),
		.JOY1_3        (JOY1_3),
		.JOY1_7        (JOY1_7),
		.JOY2_1        (JOY2_1),
		.JOY2_3        (JOY2_3),
		.JOY2_7        (JOY2_7),
		.snd_reg_addr  (snd_reg_addr),
		.snd_reg_wdata (snd_reg_wdata),
		.snd_reg_wr    (snd_reg_wr),
		.snd_reg_rd    (snd_reg_rd),
		.snd_reg_rdata (snd_reg_rdata),
		.snd_status    (snd_status),
		.snd_irq_timer (snd_irq_timer),
		.snd_irq_dma   (snd_irq_dma),
		.smp_req       (smp_req),
		.smp_addr      (smp_addr),
		.smp_dout      (smp_dout),
		.smp_done      (smp_done),
		.ce_snd        (ce_snd),
		.boot_addr     (boot_addr),
		.boot_data     (smp_q),
		.idle          (snd_idle)
	);

	wire       snd_idle;
	wire [13:0] boot_addr;
	wire [7:0] snd_reg_addr, snd_reg_wdata;
	wire       snd_reg_wr, snd_reg_rd;

	wire [7:0] snd_reg_rdata, snd_status;
	wire       snd_irq_timer, snd_irq_dma;
	wire       ce_snd, smp_req, smp_done;
	wire [15:0] smp_addr;
	wire [7:0] smp_dout;

	um6619_snd sampler
	(
		.clk_sys   (clk_sys),
		.reset     (c65_reset),
		.ce_snd    (ce_snd),
		.reg_addr  (snd_reg_addr),
		.reg_wdata (snd_reg_wdata),
		.reg_wr    (snd_reg_wr),
		.reg_rd    (snd_reg_rd),
		.reg_rdata (snd_reg_rdata),
		.status    (snd_status),
		.irq_timer (snd_irq_timer),
		.irq_dma   (snd_irq_dma),
		.ram_req   (smp_req),
		.ram_addr  (smp_addr),
		.ram_dout  (smp_dout),
		.ram_done  (smp_done),
		.audio_l   (AUDIO_L),
		.audio_r   (AUDIO_R),

		.ss_load   (ss_load),
		.ss_din    (ss_din),
		.ss_addr   (ss_addr),
		.ss_wren   (ss_wren),
		.ss_rst    (ss_rst),
		.ss_dout   (ss_dout_snd)
	);

	// Vertical retrace also pokes the 65C02. Pin 134 is only the retrace
	// while no sprite DMA command has it; the level is held for the length
	// of a command so a retrace inside one is presented afterwards.
	reg  vblank_level, vblank_q;
	wire vblank_edge = vblank_level & ~vblank_q;
	always @(posedge clk_sys) begin
		if (~CPU_RESET_n_i) begin
			vblank_q     <= ss_now_misc[34];
			vblank_level <= ss_now_misc[35];
		end else begin
			vblank_q <= vblank_level;
			if ((ce_phi1 | ce_phi2) && !dma_tok) vblank_level <= net_6618_134;
		end
	end

	// ---- the DMA channels and the bus they take ----------------------------

	wire [15:0] dma_dout;
	wire        dma_busy, dma_tok;
	// Sprite DMA block at $F00010-$F0001E, as it passes on the bus.
	wire        vid_wr = wr & sel_vregs & (CPU_A[9:4] == 6'b000001);

	um6619_dma dma
	(
		.clk_sys     (clk_sys),
		.reset       (~CPU_RESET_n_i),
		.ce_phi1     (ce_phi1),
		.ce_phi2     (ce_phi2),
		.reg_wr      (wr & sel_dma),
		.reg_addr    (CPU_A[4:1]),
		.reg_be      ({~CPU_UDS_n, ~CPU_LDS_n}),
		.reg_din     (CPU_D_i),
		.reg_dout    (dma_dout),
		.vid_wr      (vid_wr),
		.vid_index   (CPU_A[3:1]),
		.vid_din     (CPU_D_i),
		.fwd         (net_6618_136),
		.rev         (net_6618_134),
		.tok_active  (dma_tok),
		.BR_n        (CPU_BR_n),
		.BG_n        (CPU_BG_n),
		.BGACK_n     (CPU_BGACK_n),
		.A_o         (CPU_A_o),
		.AS_n_o      (CPU_AS_n_o),
		.UDS_n_o     (CPU_UDS_n_o),
		.LDS_n_o     (CPU_LDS_n_o),
		.RW_o        (CPU_RW_o),
		.bus_oe      (CPU_BUS_oe),
		.D_i         (CPU_D_i),
		.D_o         (dma_d_o),
		.D_oe        (dma_d_oe),
		.DTACK_n     (CPU_DTACK_n),
		.AS_n_i      (CPU_AS_n),
		.busy        (dma_busy),

		.ss_load     (ss_load),
		.ss_din      (ss_din),
		.ss_addr     (ss_addr),
		.ss_wren     (ss_wren),
		.ss_rst      (ss_rst),
		.ss_dout     (ss_dout_dma)
	);

	wire [15:0] dma_d_o;
	wire        dma_d_oe;

	assign CPU_VPA_n = ~iack;

	// Acknowledge for the UM6618's one-shot latches, raised only for the
	// three levels that chip owns.
	assign net_6618_135 = iack & ((iack_level == 3'd4) | (iack_level == 3'd5)
	                            | (iack_level == 3'd7));

	// UM6650 ports; the cartridge decodes them.
	assign CART_27 = ~(valid & sel_lock);
	assign CART_66 = 1'b1;

	assign net_6619_86 = 1'b1;
	assign net_6619_87 = 1'b1;
	assign net_6619_88 = 1'b1;
	assign net_6619_91 = 1'b1;

	// Savestate word for the pieces without a block of their own.
	ss_reg #(.ADDR(SSW_6619_BUS + 2), .DEFAULT(64'd0)) misc_word
	(
		.clk_i(clk_sys), .bus_din_i(ss_din), .bus_addr_i(ss_addr),
		.bus_wren_i(ss_wren), .bus_rst_i(ss_rst), .bus_dout_o(ss_dout_misc),
		.din_i({28'd0, vblank_level, vblank_q, mailbox_irq, snd_win_held, snd_rdata, egg}),
		.dout_o(ss_now_misc)
	);

	// Park waits for quiet sound RAM and no transfer holding the bus.
	assign ss_ready = snd_idle & ~dma_busy;

	`ifdef VERILATOR
	/* verilator lint_off UNUSEDSIGNAL */
	wire unused = &{1'b0, CART_29, EXP_21, EXP_22, EXP_25,
	                net_6619_85, net_6619_89, net_6619_90,
	                ss_now_misc[63:36], ld_addr[14], ld_addr[0],
	                CPU_A[21:16]};
	/* verilator lint_on UNUSEDSIGNAL */
	`endif
endmodule
