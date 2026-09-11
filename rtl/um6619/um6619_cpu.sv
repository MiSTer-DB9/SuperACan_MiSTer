// Copyright (c) 2026 Jamie Blanks

// UM6619 sound-CPU side: the 6502, its clock, the arbiter in front of the
// sound RAM, the I/O page, both joypad ports and the two mailboxes.
//
// The sound CPU runs at CPU_CLK/3, six CPU_CLK phases per cycle, over
// 64 KiB of sound RAM with the I/O page at $0400. The 68000 sees the same
// RAM through $E80000 as two byte cycles per word; one state machine owns
// the pins and serves the 6502 first.
//
// I/O page $0400-$04FF: six interrupt sources, level held until their own
// read; status $0411 clears nothing. The byte latches at $0404/$0405 read
// $CD when empty and are filled by 68000 writes through the RAM window.
// $0407 drives the joypads on the falling edge of each control bit: 1-0
// latch, 3-2 shift a bit in on JOYn_3, 5-4 clear the shift register.

module um6619_cpu (
	input  wire        clk_sys,
	input  wire        reset,       // board reset
	// $E9001C bit 0 clear holds the sound CPU; the rest keeps running so the
	// 68000 can write the driver into sound RAM first.
	input  wire        c65_halt,
	input  wire        ce_phi1,     // CPU_CLK phases
	input  wire        ce_phi2,

	// Sound RAM device, U11.
	output reg  [15:0] SNDRAM_A,
	input  wire [7:0]  SNDRAM_D_i,
	output reg  [7:0]  SNDRAM_D_o,
	output reg         SNDRAM_D_oe,
	output reg         SNDRAM_CEOE_n,
	output reg         SNDRAM_WE_n,

	// 68000 window at $E80000.
	input  wire        win_req,
	input  wire        win_we,
	input  wire [15:1] win_addr,
	input  wire [1:0]  win_be,      // {UDS, LDS}
	input  wire [15:0] win_din,
	output reg  [15:0] win_dout,
	output reg         win_done,

	// Host port side of the mailboxes.
	input  wire        cmd_write,   // write to $E9000A sets IRQ bit 5
	output wire [7:0]  io_040a,     // read back at $E9000C
	output wire [7:0]  io_040c,     // read back at $E90004
	output wire [7:0]  io_040d,     // and $E90005
	output reg         irq6,        // the 65C02 wrote $040A

	input  wire        vblank_nmi,

	// Savestate. ss_load is the restore window and outranks reset.
	input  wire        ss_load,
	input  wire [63:0] ss_din,
	input  wire [9:0]  ss_addr,
	input  wire        ss_wren,
	input  wire        ss_rst,
	output wire [63:0] ss_dout,

	// Joypad ports.
	output reg         JOY1_1,
	input  wire        JOY1_3,
	output reg         JOY1_7,
	output reg         JOY2_1,
	input  wire        JOY2_3,
	output reg         JOY2_7,

	// Sampler read port on the sound RAM.
	input  wire        smp_req,
	input  wire [15:0] smp_addr,
	output reg  [7:0]  smp_dout,
	output reg         smp_done,

	output wire        ce_snd,      // one pulse per sound-CPU cycle

	// Internal 16 KiB image, copied into sound RAM as the machine leaves reset.
	output wire [13:0] boot_addr,
	input  wire [7:0]  boot_data,

	// Sampler register file, addressed through $0420/$0422.
	output reg  [7:0]  snd_reg_addr,
	output reg  [7:0]  snd_reg_wdata,
	output reg         snd_reg_wr,
	output reg         snd_reg_rd,
	input  wire [7:0]  snd_reg_rdata,
	input  wire [7:0]  snd_status,
	input  wire        snd_irq_timer,   // I/O page IRQ bit 7, level held
	input  wire        snd_irq_dma,     // I/O page IRQ bit 6, level held

	output wire        idle          // nothing in flight, safe to park
);
	// ---- the sound CPU's clock ----------------------------------------------

	// Restore enable, one flop deep like every other block.
	reg ss_load_q = 1'b0;
	always @(posedge clk_sys) ss_load_q <= ss_load;

	// Three CPU clocks per 6502 cycle. The counter steps on CPU_CLK phase 1,
	// so the enables cannot drift against it.
	reg [1:0] phase;
	wire      c65_phi1 = ce_phi1 & (phase == 2'd0);
	wire      c65_phi2 = ce_phi2 & (phase == 2'd2);

	always @(posedge clk_sys) begin
		if (ss_load_q)      phase <= ss_saved[261:260];
		else if (reset)   phase <= 2'd0;
		else if (ce_phi1) phase <= (phase == 2'd2) ? 2'd0 : phase + 2'd1;
	end

	`include "ss_map.svh"

	// The 65C02, this block's state and both joypad shift registers, over
	// sixteen words.
	localparam int SS_SND_CPU_BITS  = 509;
	localparam int SS_SND_CPU_WORDS = 16;

	wire [259:0] ss_c65_out;
	wire [SS_SND_CPU_BITS-1:0] ss_live;
	wire [SS_SND_CPU_WORDS*64-1:0] ss_saved;
	wire [SS_SND_CPU_WORDS*64-1:0] ss_live_padded =
		{ {(SS_SND_CPU_WORDS*64 - SS_SND_CPU_BITS){1'b0}}, ss_live };
	wire [63:0] ss_word_dout [SS_SND_CPU_WORDS];

	genvar sw;
	generate
		for( sw = 0; sw < SS_SND_CPU_WORDS; sw++) begin : g_ss_words
			ss_reg #(.ADDR(SSW_6619_CPU + sw)) word (
				.clk_i      (clk_sys),
				.bus_din_i  (ss_din),
				.bus_addr_i (ss_addr),
				.bus_wren_i (ss_wren),
				// Only read inside the restore window, so no reset needed.
				.bus_rst_i  (1'b0),
				.bus_dout_o (ss_word_dout[sw]),
				.din_i      (ss_live_padded[sw*64 +: 64]),
				.dout_o     (ss_saved[sw*64 +: 64])
			);
		end
	endgenerate

	logic [63:0] ss_dout_or;
	always_comb begin
		ss_dout_or = 64'd0;
		for (int i = 0; i < SS_SND_CPU_WORDS; i++)
			ss_dout_or = ss_dout_or | ss_word_dout[i];
	end
	assign ss_dout = ss_dout_or;

	// 259:0 the 65C02, 508:260 this block.
	assign ss_live = { SNDRAM_D_o, SNDRAM_A, smp_dout, snd_reg_wdata, snd_reg_addr, shadow_040d, shadow_040c, shadow_040a, copying, copy_ph, copy_addr, c65_din, io_special, io_rdata, io_wdata, io_addr, io_stage, nmi_pending, win_dout, win_hi, win_taken, win_pending, c65_ram_wdata, c65_ram_addr, c65_ram_wr, c65_ram_rd, shift_ctrl, latch1_full, latch0_full, latch1, latch0, shift1, shift0, irq_status, irq_enable, arb, resv, wr_delay, rd_delay, phase,
		ss_c65_out };

	`ifdef VERILATOR
	/* verilator lint_off UNUSEDSIGNAL */
	wire ss_unused = &{1'b0, ss_rst, ss_saved[SS_SND_CPU_WORDS*64-1:SS_SND_CPU_BITS]};
	/* verilator lint_on UNUSEDSIGNAL */
	`endif

	// The 6502's address settles one clk_sys after phase 1 and its write
	// data one after phase 2. The core captures its read at phase 2.
	reg [1:0] rd_delay, wr_delay;
	reg [3:0] resv;
	wire      c65_rd_go = (rd_delay == 2'd1);
	wire      c65_wr_go = (wr_delay == 2'd1);

	always @(posedge clk_sys) begin
		if (ss_load_q) begin
			rd_delay <= ss_saved[263:262];
			wr_delay <= ss_saved[265:264];
			resv     <= ss_saved[269:266];
		end else if (reset) begin
			rd_delay <= 2'd0;
			wr_delay <= 2'd0;
			resv     <= 4'd0;
		end else begin
			rd_delay <= c65_phi1 ? 2'd1 : (rd_delay == 2'd0 ? 2'd0 : rd_delay - 2'd1);
			wr_delay <= c65_phi2 ? 2'd1 : (wr_delay == 2'd0 ? 2'd0 : wr_delay - 2'd1);
			// Reserve the device for the 6502 after each phase, or a 68000
			// poll loop can push its read past the phase 2 capture.
			resv <= (c65_phi1 | c65_phi2) ? 4'd4
			                              : (resv == 4'd0 ? 4'd0 : resv - 4'd1);
		end
	end

	// ---- the 6502 -----------------------------------------------------------

	wire [15:0] c65_addr;
	wire [7:0]  c65_dout;
	wire        c65_rw_n;
	/* verilator lint_off UNUSEDSIGNAL */
	wire        c65_jam;
	wire        c65_sync;
	/* verilator lint_on UNUSEDSIGNAL */
	reg  [7:0]  c65_din;

	// Bits 7 and 6 belong to the sampler and are merged in, not stored.
	wire [7:0] irq_live = {snd_irq_timer, snd_irq_dma, irq_status[5:0]};
	wire irq_asserted = (irq_live & irq_enable) != 8'd0;
	reg  nmi_pending;

	mos6502 #(.BCD_EN(1'b1)) sound_cpu
	(
		.clk_sys  (clk_sys),
		.phi1_en  (c65_phi1),
		.phi2_en  (c65_phi2),
		.res_n    (~(reset | c65_halt)),
		.rdy      (1'b1),
		.irq_n    (~irq_asserted),
		.nmi_n    (~nmi_pending),
		.so_n     (1'b1),
		.data_in  (c65_din),
		.data_out (c65_dout),
		/* verilator lint_off PINCONNECTEMPTY */
		.data_oe  (),
		.addr_out (c65_addr),
		.rw_n     (c65_rw_n),
		.sync     (c65_sync),
		.phi1_out (),
		.phi2_out (),
		.jammed   (c65_jam),
		.dbg_a (), .dbg_x (), .dbg_y (), .dbg_s (), .dbg_p (), .dbg_ir (),
		.dbg_pc (), .dbg_t (), .dbg_hold (), .dbg_int_active (),
		.dbg_take_int (), .dbg_res_active (), .dbg_tg (),
		/* verilator lint_on PINCONNECTEMPTY */
		.ss_load  (ss_load),
		.ss_in    (ss_saved[259:0]),
		.ss_out   (ss_c65_out)
	);

	// The I/O page is decoded on all sixteen bits; the RAM sees fifteen.
	wire c65_is_io = (c65_addr[15:8] == 8'h04);

	// ---- the I/O page's own storage -----------------------------------------

	// Sets and clears are collected as terms and applied once, so a set and
	// a clear on the same clock both land.
	reg  [7:0] irq_enable, irq_status;
	reg  [7:0] irq_set, irq_clr;

	reg  [7:0] shift0, shift1;
	reg  [7:0] latch0, latch1;
	reg        latch0_full, latch1_full;
	reg  [7:0] shift_ctrl;

	reg  [7:0] io_addr;
	reg        io_wren;
	reg  [7:0] io_wdata;
	wire [7:0] io_q;

	cache_ram #(.ADDR_WIDTH(8), .DATA_WIDTH(8)) io_mem
	(
		.clk_i   (clk_sys),
		.addr_i  (io_addr),
		.wren_i  (io_wren),
		.wdata_i (io_wdata),
		.q_o     (io_q)
	);

	// Copies of the three I/O page bytes the host port reads back, so it
	// need not take a turn on the RAM.
	reg [7:0] shadow_040a, shadow_040c, shadow_040d;
	assign io_040a = shadow_040a;
	assign io_040c = shadow_040c;
	assign io_040d = shadow_040d;

	// ---- the pins the sound RAM shares --------------------------------------

	localparam [2:0] A_IDLE = 3'd0, A_C65 = 3'd1, A_C65_D = 3'd2,
	                 A_WIN0 = 3'd3, A_WIN0_D = 3'd4, A_WIN1 = 3'd5,
	                 A_SMP = 3'd6, A_SMP_D = 3'd7;

	reg [2:0] arb;
	reg       c65_ram_rd, c65_ram_wr;
	reg [15:0] c65_ram_addr;
	reg [7:0]  c65_ram_wdata;
	reg        win_pending;
	reg [7:0]  win_hi;

	reg [15:0] copy_addr;
	reg        copying, copy_ph;
	assign boot_addr = copy_addr[13:0];

	assign idle = (arb == A_IDLE) & ~win_pending & ~c65_ram_rd & ~c65_ram_wr
	            & ~copying;
	assign ce_snd = c65_phi1;

	// ---- the machine --------------------------------------------------------

	// An I/O page read answers either from a side-effect register when
	// issued, or from the page's store a clock later.
	reg [7:0] io_rdata;
	reg       io_special;
	reg [1:0] io_stage;
	reg       win_taken;

	always @(posedge clk_sys) begin
		irq_set       = 8'd0;
		irq_clr       = 8'd0;
		io_wren       <= 1'b0;
		irq6          <= 1'b0;
		snd_reg_wr    <= 1'b0;
		snd_reg_rd    <= 1'b0;
		SNDRAM_CEOE_n <= 1'b1;
		SNDRAM_WE_n   <= 1'b1;
		SNDRAM_D_oe   <= 1'b0;
		win_done      <= 1'b0;
		smp_done      <= 1'b0;
		JOY1_1        <= 1'b0;
		JOY2_1        <= 1'b0;
		JOY1_7        <= 1'b0;
		JOY2_7        <= 1'b0;

		if (ss_load_q) begin
			{ SNDRAM_D_o, SNDRAM_A, smp_dout, snd_reg_wdata, snd_reg_addr,
		  shadow_040d, shadow_040c, shadow_040a, copying, copy_ph, copy_addr,
		  c65_din, io_special, io_rdata, io_wdata, io_addr, io_stage,
		  nmi_pending, win_dout, win_hi, win_taken, win_pending,
		  c65_ram_wdata, c65_ram_addr, c65_ram_wr, c65_ram_rd,
		  shift_ctrl, latch1_full, latch0_full, latch1, latch0, shift1, shift0,
		  irq_status, irq_enable, arb }
				<= ss_saved[508:270];
		end
		else if (reset) begin
			arb         <= A_IDLE;
			irq_enable  <= 8'd0;
			irq_status  <= 8'd0;
			shift0      <= 8'd0;
			shift1      <= 8'd0;
			latch0_full <= 1'b0;
			latch1_full <= 1'b0;
			shift_ctrl  <= 8'd0;
			c65_ram_rd  <= 1'b0;
			c65_ram_wr  <= 1'b0;
			win_pending <= 1'b0;
			win_taken   <= 1'b0;
			nmi_pending <= 1'b0;
			io_stage    <= 2'd0;
			c65_din     <= 8'd0;
			copy_addr   <= 16'd0;
			copy_ph     <= 1'b0;
			copying     <= 1'b1;
			shadow_040a <= 8'd0;
			shadow_040c <= 8'd0;
			shadow_040d <= 8'd0;
		end else begin
			if (cmd_write && irq_enable[5]) irq_set = irq_set | 8'h20;
			// The NMI latch clears with the CPU's reset. A vblank landing
			// during a driver swap would otherwise leave the line low with
			// nobody to acknowledge it, and no later vblank makes an edge;
			// Magical Pool polls the pads from its NMI handler.
			if (c65_halt)        nmi_pending <= 1'b0;
			else if (vblank_nmi) nmi_pending <= 1'b1;

			// One 68000 window access per assertion of its request.
			if (!win_req)                            win_taken <= 1'b0;
			else if (!win_taken && (arb == A_IDLE)) begin
				win_taken   <= 1'b1;
				win_pending <= 1'b1;
			end

			// ---- the 6502's read ------------------------------------------

			if (c65_rd_go && c65_rw_n && !c65_halt) begin
				if (c65_is_io) begin
					io_addr    <= c65_addr[7:0];
					io_stage   <= 2'd2;
					io_special <= 1'b1;
					case (c65_addr[7:0])
						8'h02: io_rdata <= shift0;
						8'h03: io_rdata <= shift1;
						8'h04: begin
							irq_clr     = irq_clr | 8'h08;
							io_rdata    <= latch0_full ? latch0 : 8'hCD;
							latch0_full <= 1'b0;
						end
						8'h05: begin
							irq_clr     = irq_clr | 8'h04;
							io_rdata    <= latch1_full ? latch1 : 8'hCD;
							latch1_full <= 1'b0;
						end
						8'h06: io_rdata <= 8'd0;
						8'h09: begin
							irq_clr    = irq_clr | 8'h10;
							io_rdata   <= 8'd0;
						end
						8'h0A: begin
							irq_clr    = irq_clr | 8'h20;
							io_rdata   <= shadow_040a;
						end
						8'h10: io_rdata <= irq_enable;
						8'h11: io_rdata <= irq_live;
						8'h12: begin
							nmi_pending <= 1'b0;
							io_rdata    <= 8'd0;
						end
						8'h20: io_rdata <= snd_status;
						8'h22: begin
							snd_reg_rd <= 1'b1;
							io_rdata   <= snd_reg_rdata;
						end
						default: io_special <= 1'b0;
					endcase
				end else begin
					c65_ram_addr <= c65_addr;
					c65_ram_rd   <= 1'b1;
				end
			end

			if (io_stage != 2'd0) begin
				io_stage <= io_stage - 2'd1;
				if (io_stage == 2'd1) c65_din <= io_special ? io_rdata : io_q;
			end

			// ---- the 6502's write -----------------------------------------

			if (c65_wr_go && !c65_rw_n && !c65_halt) begin
				if (c65_is_io) begin
					io_addr  <= c65_addr[7:0];
					io_wdata <= c65_dout;
					io_wren  <= 1'b1;
					case (c65_addr[7:0])
						8'h07: begin
							shift_ctrl <= c65_dout;
							// Falling edges: bits 1-0 latch, 3-2 shift a bit in,
							// 5-4 clear the shift register.
							if (shift_ctrl[0] & ~c65_dout[0]) JOY1_1 <= 1'b1;
							if (shift_ctrl[1] & ~c65_dout[1]) JOY2_1 <= 1'b1;
							if (shift_ctrl[2] & ~c65_dout[2]) begin
								JOY1_7 <= 1'b1;
								shift0 <= {shift0[6:0], JOY1_3};
							end
							if (shift_ctrl[3] & ~c65_dout[3]) begin
								JOY2_7 <= 1'b1;
								shift1 <= {shift1[6:0], JOY2_3};
							end
							// Bits 5 and 4 must not raise the $0404/$0405
							// interrupts: drivers pulse them at startup and
							// declare a pad absent if it answers.
							if (shift_ctrl[4] & ~c65_dout[4]) shift0 <= 8'd0;
							if (shift_ctrl[5] & ~c65_dout[5]) shift1 <= 8'd0;
						end
						8'h0A: begin
							shadow_040a <= c65_dout;
							irq6        <= 1'b1;
						end
						8'h0C:   shadow_040c  <= c65_dout;
						8'h0D:   shadow_040d  <= c65_dout;
						8'h10:   irq_enable   <= c65_dout;
						8'h20:   snd_reg_addr <= c65_dout;
						8'h22: begin
							snd_reg_wdata <= c65_dout;
							snd_reg_wr    <= 1'b1;
						end
						default: ;
					endcase
				end else begin
					c65_ram_addr  <= c65_addr;
					c65_ram_wdata <= c65_dout;
					c65_ram_wr    <= 1'b1;
				end
			end

			// ---- the one device, and who has it ---------------------------

			case (arb)
			A_IDLE:
				if (copying) begin
					// The internal image copy holds the device until done.
					if (!copy_ph) begin
						copy_ph <= 1'b1;
					end else begin
						SNDRAM_A      <= copy_addr;
						SNDRAM_CEOE_n <= 1'b0;
						SNDRAM_WE_n   <= 1'b0;
						SNDRAM_D_o    <= boot_data;
						SNDRAM_D_oe   <= 1'b1;
						copy_ph       <= 1'b0;
						copy_addr     <= copy_addr + 16'd1;
						if (copy_addr == 16'h3FFF) copying <= 1'b0;
					end
				end else if (c65_ram_wr) begin
					SNDRAM_A      <= c65_ram_addr;
					SNDRAM_CEOE_n <= 1'b0;
					SNDRAM_WE_n   <= 1'b0;
					SNDRAM_D_o    <= c65_ram_wdata;
					SNDRAM_D_oe   <= 1'b1;
					c65_ram_wr    <= 1'b0;
				end else if (c65_ram_rd) begin
					SNDRAM_A      <= c65_ram_addr;
					SNDRAM_CEOE_n <= 1'b0;
					arb           <= A_C65;
				end else if (win_pending && (resv == 4'd0)) begin
					SNDRAM_A      <= {win_addr[15:1], 1'b0};
					SNDRAM_CEOE_n <= ~win_be[1];
					SNDRAM_WE_n   <= ~(win_we & win_be[1]);
					SNDRAM_D_o    <= win_din[15:8];
					SNDRAM_D_oe   <= win_we & win_be[1];
					arb           <= A_WIN0;
				// smp_done is still high the clock this goes idle, so check
				// it or the request is served twice.
				end else if (smp_req && !smp_done && (resv == 4'd0)) begin
					SNDRAM_A      <= smp_addr;
					SNDRAM_CEOE_n <= 1'b0;
					arb           <= A_SMP;
				end

			// Address out one clock, answer back the next.
			A_C65: arb <= A_C65_D;

			A_C65_D: begin
				c65_din    <= SNDRAM_D_i;
				c65_ram_rd <= 1'b0;
				arb        <= A_IDLE;
			end

			A_SMP: arb <= A_SMP_D;

			A_SMP_D: begin
				smp_dout <= SNDRAM_D_i;
				smp_done <= 1'b1;
				arb      <= A_IDLE;
			end

			A_WIN0: begin
				// Odd byte's address goes out as the even byte's answer returns.
				SNDRAM_A      <= {win_addr[15:1], 1'b1};
				SNDRAM_CEOE_n <= ~win_be[0];
				SNDRAM_WE_n   <= ~(win_we & win_be[0]);
				SNDRAM_D_o    <= win_din[7:0];
				SNDRAM_D_oe   <= win_we & win_be[0];
				arb           <= A_WIN0_D;
			end

			A_WIN0_D: begin
				win_hi <= SNDRAM_D_i;
				arb    <= A_WIN1;
			end

			A_WIN1: begin
				win_dout    <= {win_hi, SNDRAM_D_i};
				win_done    <= 1'b1;
				win_pending <= 1'b0;
				arb         <= A_IDLE;
				// Both CPUs write $040A; the 68000 clears it through this
				// window, so keep the host readback copy in step.
				if (win_we && (win_addr[15:1] == 15'h0205) && win_be[1])
					shadow_040a <= win_din[15:8];

				// A 68000 write to $E80404/$E80405 fills a byte latch and
				// raises its interrupt.
				if (win_we && (win_addr[15:1] == 15'h0202)) begin
					if (win_be[1]) begin
						latch0      <= win_din[15:8];
						latch0_full <= 1'b1;
						irq_set     = irq_set | 8'h08;
					end
					if (win_be[0]) begin
						latch1      <= win_din[7:0];
						latch1_full <= 1'b1;
						irq_set     = irq_set | 8'h04;
					end
				end
			end

			default: arb <= A_IDLE;
			endcase

			irq_status <= (irq_status | irq_set) & ~irq_clr;
		end
	end

	`ifdef VERILATOR
	/* verilator lint_off UNUSEDSIGNAL */
	wire unused = &{1'b0, c65_addr[15]};
	/* verilator lint_on UNUSEDSIGNAL */
	`endif
endmodule
