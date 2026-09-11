// Copyright (c) 2026 Jamie Blanks

// U12, UM6618F video ASIC. No datasheet exists; this is a model of what the
// software demands, behind the schematic's pin list.
//
// The output to the DAC is an eight-bit palette index per pixel, so every
// layer mix, sprite priority, window fill and colour effect happens in index
// space, and translucency is an index add.
//
// MCLK reaches only this chip; it divides the crystal and hands CPU_CLK to
// the 68000, the UM6619 and both connectors.
//
// No !UDS or !LDS pin, so every register and palette access is a whole word.

module um6618 (
	// clk_sys is MCLK. CPU_CLK leaves as two phase enables.
	input  wire        clk_sys,
	output wire        ce_cpu_phi1,
	output wire        ce_cpu_phi2,

	// 68000 slave side.
	input  wire [23:1] CPU_A,
	input  wire [15:0] CPU_D_i,
	output wire [15:0] CPU_D_o,
	output wire        CPU_D_oe,
	input  wire        CPU_AS_n,
	input  wire        CPU_RW,
	input  wire        CPU_RESET_n,

	// VRAM master side. VRAM_A17 has nowhere to go on the 64K x 8 devices fitted.
	output wire [16:1] VRAM_A,
	output wire        VRAM_A17,
	input  wire [15:0] VRAM_D_i,
	output wire [15:0] VRAM_D_o,
	output wire        VRAM_D_oe,
	output wire        VRAM_OE_n,
	output wire        VRAM_WE_n,

	// Palette DAC.
	output wire [7:0]  DAC_P,
	output wire        DAC_PCLK,
	output wire [7:0]  DAC_D_o,
	input  wire [7:0]  DAC_D_i,
	output wire [1:0]  DAC_RS,
	output wire        DAC_RD_n,
	output wire        DAC_WR_n,
	output wire        DAC_BLANK_n,

	// Encoder.
	output wire        CSYNC,
	output wire        SUBCARR,

	// To the UM6619, roles inferred: three interrupt requests out, an
	// acknowledge back, and the sprite DMA pair. 134 carries the DMA
	// acknowledge while a command runs and the vertical retrace otherwise.
	output wire        net_6618_126,   // visible line, IRQ4
	output wire        net_6618_127,   // line on/off, IRQ5
	output wire        net_6618_134,   // vertical retrace, IRQ7 / DMA acknowledge
	input  wire        net_6618_135,   // interrupt acknowledge
	input  wire        net_6618_136,   // DMA token

	// Cartridge and pad glue.
	input  wire        CART_31,
	input  wire        CART_32,
	input  wire        CART_34,
	input  wire        CART_68,
	output wire        net_6618_137,
	input  wire        JOY_2,
	input  wire        JP1,
	input  wire        JP2,

	// Framework video output and savestate.
	output wire        ce_pix,
	output wire        hsync,
	output wire        vsync,
	output wire        hblank,
	output wire        vblank,
	output wire        de,

	input  wire        pause_cpu,
	input  wire        pause_vid,
	output wire        ss_ready,

	input  wire [63:0] ss_din,
	input  wire [9:0]  ss_addr,
	input  wire        ss_wren,
	input  wire        ss_rst,
	output wire [63:0] ss_dout,

	output wire        overscan        // $F00008 bit 9, for the framework's crop
);
	`include "ss_map.svh"
	wire reset = ~CPU_RESET_n;
	/* verilator lint_off PROCASSINIT */
	reg reset_q = 1'b0;
	reg ss_loaded = 1'b0;
	/* verilator lint_on PROCASSINIT */
	wire ss_load = reset_q && !reset && ss_loaded;
	always @(posedge clk_sys) begin
		reset_q <= reset;
		if (ss_rst) ss_loaded <= 1'b0;
		else if (ss_wren && ss_addr == 10'(SSW_6618_BASE)) ss_loaded <= 1'b1;
	end

	// ---- the board's clocks ------------------------------------------------

	wire pix_stretch;
	wire pix_ready;
	wire [63:0] ss_clocks_live, ss_clocks_saved, ss_clocks_dout;
	wire [63:0] ss_timing_live, ss_timing_saved, ss_timing_dout;
	wire [63:0] ss_top_saved, ss_top_dout, ss_regs_dout, ss_roz_dout, ss_sprite_dout, ss_dma_dout;

	clk_div clocks
	(
		.clk_sys     (clk_sys),
		.pause_cpu   (pause_cpu),
		.pause_vid   (pause_vid),
		.h320        (video_flags[8]),
		.pix_stretch (pix_stretch),
		.ss_load     (ss_load),
		.ss_din      (ss_clocks_saved),
		.ss_dout     (ss_clocks_live),
		.pix_ready   (pix_ready),
		.ce_cpu_phi1 (ce_cpu_phi1),
		.ce_cpu_phi2 (ce_cpu_phi2),
		.ce_pix      (ce_pix)
	);

	// ---- timing -------------------------------------------------------------

	wire [9:0] hpos;
	wire       line_start, csync_w;
	wire [8:0] vpos;

	wire [15:0] video_flags, raster_on, raster_off, pixel_mode;
	assign overscan = video_flags[9];

	// The DAC latches a dot's colour on the enable that advances the
	// counters, so sync and blanking are delayed one dot to match.
	wire hb_w, vb_w, hs_w, vs_w, de_w;

	always @(posedge clk_sys) begin
		if (ss_load) begin
			{hblank_q, vblank_q, hsync_q, vsync_q, de_q} <= ss_top_saved[40:36];
		end else if (ce_pix) begin
			hblank_q <= hb_w;
			vblank_q <= vb_w;
			hsync_q  <= hs_w;
			vsync_q  <= vs_w;
			de_q     <= de_w;
		end
	end

	reg hblank_q, vblank_q, hsync_q, vsync_q, de_q;

	assign hblank = hblank_q;
	assign vblank = vblank_q;
	assign hsync  = hsync_q;
	assign vsync  = vsync_q;
	assign de     = de_q;

	um6618_vtiming timing
	(
		.clk_sys     (clk_sys),
		.ce_pix      (ce_pix),
		.ss_load     (ss_load),
		.ss_din      (ss_timing_saved),
		.ss_dout     (ss_timing_live),
		.h320        (video_flags[8]),
		.overscan    (video_flags[9]),
		.pix_stretch (pix_stretch),
		.hpos        (hpos),
		.vpos        (vpos),
		.hblank      (hb_w),
		.vblank      (vb_w),
		.hsync       (hs_w),
		.vsync       (vs_w),
		.csync       (csync_w),
		.blank_n     (DAC_BLANK_n),
		.de          (de_w),
		.line_start  (line_start)
	);

	assign CSYNC   = csync_w;
	// Colour subcarrier for the encoder; the framework makes its own.
	assign SUBCARR = 1'b0;

	// ---- the 68000 slave ----------------------------------------------------

	wire valid = ~CPU_AS_n;
	wire sel_regs = valid & (CPU_A[23:9] == 15'b111100000000000);   // $F00000-$F001FF
	wire sel_pal  = valid & (CPU_A[23:9] == 15'b111100000000001);   // $F00200-$F003FF
	wire sel_vram = valid & (CPU_A[23:17] == 7'b1111010);           // $F40000-$F5FFFF

	wire reg_wr = sel_regs & ~CPU_RW;

	wire [15:0] reg_dout;
	wire [511:0] tm_regs;
	wire [255:0] roz_regs;
	wire [127:0] spr_dma_regs;
	wire [63:0]  spr_regs;
	wire [127:0] win_regs;

	um6618_regs regs
	(
		.clk_sys     (clk_sys),
		.reset       (reset),
		.wr          (reg_wr & ~reg_wr_q),
		.index       (CPU_A[8:1]),
		.din         (CPU_D_i),
		.dout        (reg_dout),
		.in_vblank   (vb_w),
		.vpos        (vpos),
		.hpos        (hpos),
		.video_flags  (video_flags),
		.raster_on    (raster_on),
		.raster_off   (raster_off),
		.pixel_mode   (pixel_mode),
		.tm_regs      (tm_regs),
		.roz_regs     (roz_regs),
		.spr_dma_regs   (spr_dma_regs),
		.spr_dma_wb     (spr_dma_done),
		.spr_dma_dst_in (spr_dma_dst_out),
		.spr_dma_src_in (spr_dma_src_out),
		.spr_regs     (spr_regs),
		.win_regs     (win_regs),
		.ss_din       (ss_din),
		.ss_addr      (ss_addr),
		.ss_wren      (ss_wren),
		.ss_rst       (ss_rst),
		.ss_dout      (ss_regs_dout)
	);

	// A 68000 strobe lasts several clocks; the register file wants an edge.
	reg reg_wr_q, pal_wr_q, pal_rd_q;
	always @(posedge clk_sys) begin
		reg_wr_q <= reg_wr;
		pal_wr_q <= sel_pal & ~CPU_RW;
		pal_rd_q <= sel_pal &  CPU_RW;
	end

	// ---- the CPU's VRAM window ---------------------------------------------

	// The 68000 holds its address for at least sixteen clk_sys cycles, so the
	// one slot a dot gives it always arrives in time and no wait is needed.
	reg         cpu_vram_req;
	reg  [15:0] cpu_vram_dout;
	reg         vram_held;

	always @(posedge clk_sys) begin
		if (reset) begin
			cpu_vram_req <= 1'b0;
			vram_held    <= 1'b0;
			cpu_vram_dout <= ss_top_saved[31:16];
		end else begin
			if (sel_vram && !vram_held) begin
				cpu_vram_req <= 1'b1;
				vram_held    <= 1'b1;
			end
			if (arb_gnt[C_CPU]) cpu_vram_req <= 1'b0;
			if (!sel_vram)      vram_held    <= 1'b0;
		end
		if (!reset && arb_ack[C_CPU]) cpu_vram_dout <= arb_q;
	end

	// ---- the VRAM arbiter and its clients ----------------------------------

	localparam int NCL   = 9;
	localparam int C_CPU = 0;
	localparam int C_DMA = 1;
	localparam int C_TM0 = 2;
	localparam int C_ROZ = 6;
	localparam int C_SPR = 7;
	localparam int C_WIN = 8;

	wire [NCL-1:0]    arb_req, arb_we, arb_gnt, arb_ack;
	wire [NCL*16-1:0] arb_addr, arb_wdata;
	wire [15:0]       arb_q;

	um6618_vram_arb #(.NCLIENTS(NCL)) arb
	(
		.clk_sys   (clk_sys),
		.ce_pix    (ce_pix),
		.req       (arb_req),
		.we        (arb_we),
		.addr      (arb_addr),
		.wdata     (arb_wdata),
		.gnt       (arb_gnt),
		.ack       (arb_ack),
		.q         (arb_q),
		.VRAM_A    (VRAM_A),
		.VRAM_D_i  (VRAM_D_i),
		.VRAM_D_o  (VRAM_D_o),
		.VRAM_D_oe (VRAM_D_oe),
		.VRAM_OE_n (VRAM_OE_n),
		.VRAM_WE_n (VRAM_WE_n)
	);

	// ---- the renderers ------------------------------------------------------

	// Pixel depth per layer: 0 eight bits, 1 four, 2 two. Layers 0 and 1
	// follow the GFX mode field of $F001F0; layers 2 and 3 are always two.
	wire [2:0] gfx_mode = pixel_mode[2:0];

	reg [1:0] region0, region1;
	always @(*) begin
		case (gfx_mode)
			3'd0:    begin region0 = 2'd2; region1 = 2'd2; end
			3'd1:    begin region0 = 2'd1; region1 = 2'd1; end
			3'd2:    begin region0 = 2'd0; region1 = 2'd1; end
			3'd3:    begin region0 = 2'd1; region1 = 2'd1; end
			default: begin region0 = 2'd0; region1 = 2'd2; end
		endcase
	end

	// Line buffers build vpos+1 while vpos is on screen; banks swap once a line.
	wire [8:0] render_y  = (vpos == 9'd261) ? 9'd0 : vpos + 9'd1;
	wire       render_en = (render_y < 9'd240);
	wire       lb_wbank  = vpos[0];
	wire       lb_rbank  = ~vpos[0];
	wire [8:0] read_x    = hpos[8:0];

	wire [31:0] tm_index;
	wire [3:0]  tm_opaque;
	wire [3:0]  tm_tie, tm_add;
	wire [11:0] tm_priority;
	wire [3:0]  tm_req, tm_gnt, tm_ack;
	wire [63:0] tm_addr;

	genvar g;
	generate
		for (g = 0; g < 4; g = g + 1) begin : layers
			um6618_tilemap layer
			(
				.clk_sys      (clk_sys),
				.reset        (reset),
				.regs         (tm_regs[g*128 +: 128]),
				.region       (g == 0 ? region0 : g == 1 ? region1 : 2'd2),
				.enable       (video_flags[7 - g]),
				.prep         (line_start),
				.render_y     (render_y),
				.render_en    (render_en),
				.h320         (video_flags[8]),
				.write_bank   (lb_wbank),
				.read_x       (read_x),
				.read_bank    (lb_rbank),
				.pix_index    (tm_index[g*8 +: 8]),
				.pix_opaque   (tm_opaque[g]),
				.pix_priority (tm_priority[g*3 +: 3]),
				.pix_tie      (tm_tie[g]),
				.pix_add      (tm_add[g]),
				.vram_req     (tm_req[g]),
				.vram_addr    (tm_addr[g*16 +: 16]),
				.vram_gnt     (tm_gnt[g]),
				.vram_ack     (tm_ack[g]),
				.vram_q       (arb_q)
			);
			assign tm_gnt[g] = arb_gnt[C_TM0 + g];
			assign tm_ack[g] = arb_ack[C_TM0 + g];
		end
	endgenerate

	// ---- the ROZ layer ------------------------------------------------------

	wire [7:0]  roz_index;
	wire        roz_opaque, roz_req, roz_gnt, roz_ack;
	wire [2:0]  roz_priority;
	wire        roz_tie, roz_add;
	wire [15:0] roz_addr;
	um6618_roz roz
	(
		.clk_sys      (clk_sys),
		.reset        (reset),
		.regs         (roz_regs),
		.pixel_mode   (pixel_mode),
		.enable       (video_flags[2]),
		.prep         (line_start),
		.render_y     (render_y),
		.render_en    (render_en),
		.h320         (video_flags[8]),
		.write_bank   (lb_wbank),
		.read_x       (read_x),
		.read_bank    (lb_rbank),
		.pix_index    (roz_index),
		.pix_opaque   (roz_opaque),
		.pix_priority (roz_priority),
		.pix_tie      (roz_tie),
		.pix_add      (roz_add),
		.vram_req     (roz_req),
		.vram_addr    (roz_addr),
		.vram_gnt     (roz_gnt),
		.vram_ack     (roz_ack),
		.vram_q       (arb_q),
		.ss_din       (ss_din),
		.ss_addr      (ss_addr),
		.ss_wren      (ss_wren),
		.ss_rst       (ss_rst),
		.ss_dout      (ss_roz_dout)
	);

	assign roz_gnt = arb_gnt[C_ROZ];
	assign roz_ack = arb_ack[C_ROZ];

	// ---- the sprites --------------------------------------------------------

	wire [7:0]  spr_index, spr_backup;
	wire [1:0]  spr_priority, spr_flags;
	wire        spr_masked, spr_req, spr_gnt, spr_ack;
	wire [15:0] spr_addr;

	// A write to the table base or count ($20, $22) re-reads the table, like
	// the sprite DMA finishing. Journey to the Laugh rewrites the count every
	// frame on the vblank line the table is read on. Delayed one clock so the
	// register file has captured the new value before the engine reads it.
	wire spr_regs_wr = reg_wr & ~reg_wr_q & (CPU_A[8:2] == 7'h08);
	reg  spr_regs_wr_q;
	always @(posedge clk_sys) spr_regs_wr_q <= spr_regs_wr;

	// A CPU write inside the live table also re-arms the read. African
	// Adventure rebuilds every entry across the vblank boundary and only then
	// writes the count.
	wire [15:0] spr_table_lo  = {spr_regs[14:0], 1'b0};
	wire [16:0] spr_table_hi  = {1'b0, spr_table_lo} + ({spr_regs[30:16], 2'b00} + 17'd4);
	wire        spr_table_wr  = arb_gnt[C_CPU] & ~CPU_RW
	                          & (CPU_A[16:1] >= spr_table_lo)
	                          & ({1'b0, CPU_A[16:1]} < spr_table_hi);

	wire spr_eval_idle;
	um6618_sprite sprites
	(
		.clk_sys      (clk_sys),
		.reset        (reset),
		.regs         (spr_regs),
		.enable       (video_flags[3]),
		.prep         (line_start),
		.table_updated(spr_dma_done | spr_regs_wr_q | spr_table_wr),
		.eval_idle    (spr_eval_idle),
		.render_y     (render_y),
		.render_en    (render_en),
		.h320         (video_flags[8]),
		.write_bank   (lb_wbank),
		.read_x       (read_x),
		.read_bank    (lb_rbank),
		.pix_index    (spr_index),
		.pix_priority (spr_priority),
		.pix_flags    (spr_flags),
		.pix_backup   (spr_backup),
		.line_masked  (spr_masked),
		.vram_req     (spr_req),
		.vram_addr    (spr_addr),
		.vram_gnt     (spr_gnt),
		.vram_ack     (spr_ack),
		.vram_q       (arb_q),
		.ss_din       (ss_din),
		.ss_addr      (ss_addr),
		.ss_wren      (ss_wren),
		.ss_rst       (ss_rst),
		.ss_dout      (ss_sprite_dout)
	);

	assign spr_gnt = arb_gnt[C_SPR];
	assign spr_ack = arb_ack[C_SPR];

	// ---- the two window clippers --------------------------------------------

	wire [15:0] win_pen;
	wire [1:0]  win_in, win_active;
	wire [5:0]  win_priority;
	wire        win_req, win_gnt, win_ack;
	wire [15:0] win_addr;

	um6618_window windows
	(
		.clk_sys    (clk_sys),
		.reset      (reset),
		.regs       (win_regs),
		.enable     (video_flags[1:0]),
		.prep       (line_start),
		.render_y   (render_y),
		.render_en  (render_en),
		.read_x     (read_x),
		.pen        (win_pen),
		.in_span    (win_in),
		.priority_o (win_priority),
		.active     (win_active),
		.vram_req   (win_req),
		.vram_addr  (win_addr),
		.vram_gnt   (win_gnt),
		.vram_ack   (win_ack),
		.vram_q     (arb_q)
	);

	assign win_gnt = arb_gnt[C_WIN];
	assign win_ack = arb_ack[C_WIN];

	assign arb_req   = {win_req, spr_req, roz_req, tm_req, spr_vram_req, cpu_vram_req};
	assign arb_we    = {7'd0, 1'b1, ~CPU_RW & cpu_vram_req};
	assign arb_addr  = {win_addr, spr_addr, roz_addr, tm_addr, spr_vram_addr, CPU_A[16:1]};
	assign arb_wdata = {112'd0, spr_vram_data, CPU_D_i};

	assign VRAM_A17 = 1'b0;

	// ---- the sprite DMA -----------------------------------------------------

	// Writing $F0001E with bit 15 set starts a transfer. The UM6619 fetches
	// the source and this side writes each word into VRAM; on close the final
	// pointers go back into $F00012/$F00014 and $F00018/$F0001A.
	wire        spr_dma_done, spr_dma_busy;
	wire [31:0] spr_dma_dst_out, spr_dma_src_out;
	wire        spr_vram_req;
	wire [15:0] spr_vram_addr, spr_vram_data;
	wire        dma_rev, dma_tok;

	um6618_dma dma
	(
		.clk_sys      (clk_sys),
		.reset        (reset),
		.regs         (spr_dma_regs),
		.trigger      (reg_wr & ~reg_wr_q & (CPU_A[8:1] == 8'h0F) & CPU_D_i[15]),
		.trigger_ctrl (CPU_D_i),
		.CPU_D_i      (CPU_D_i),
		.fwd          (net_6618_136),
		.rev          (dma_rev),
		.tok_active   (dma_tok),
		.idle_level   (irq_vblank),
		.hold         (pause_cpu),
		.vram_req     (spr_vram_req),
		.vram_addr    (spr_vram_addr),
		.vram_data    (spr_vram_data),
		.vram_gnt     (arb_gnt[C_DMA]),
		.vram_ack     (arb_ack[C_DMA]),
		.done         (spr_dma_done),
		.dst_out      (spr_dma_dst_out),
		.src_out      (spr_dma_src_out),
		.busy         (spr_dma_busy),
		.ss_din       (ss_din),
		.ss_addr      (ss_addr),
		.ss_wren      (ss_wren),
		.ss_rst       (ss_rst),
		.ss_dout      (ss_dma_dout)
	);

	// ---- the palette window, forwarded to the DAC ---------------------------

	// The DAC is eight bits wide, so one 68000 word becomes three accesses:
	// the entry, then its two halves, each strobe followed by an idle cycle
	// so the DAC sees three edges.
	localparam [2:0] DP_IDLE = 3'd0, DP_ADDR = 3'd1, DP_ADDR_G = 3'd2,
	                 DP_LO = 3'd3, DP_LO_G = 3'd4, DP_HI = 3'd5, DP_HI_G = 3'd6;

	reg [2:0]  dp_state;
	reg [7:0]  dp_byte;
	reg [1:0]  dp_rs;
	reg        dp_wr, dp_rd;
	reg [15:0] dp_read;
	reg        dp_is_read;

	wire pal_wr = (sel_pal & ~CPU_RW) & ~pal_wr_q;
	wire pal_rd = (sel_pal &  CPU_RW) & ~pal_rd_q;

	assign DAC_D_o  = dp_byte;
	assign DAC_RS   = dp_rs;
	assign DAC_WR_n = ~dp_wr;
	assign DAC_RD_n = ~dp_rd;

	always @(posedge clk_sys) begin
		dp_wr <= 1'b0;
		dp_rd <= 1'b0;

		if (reset) begin
			dp_state   <= DP_IDLE;
			dp_rs      <= 2'b00;
			dp_read    <= ss_top_saved[15:0];
			dp_is_read <= 1'b0;
		end else begin
			case (dp_state)
			DP_IDLE:
				if (pal_wr | pal_rd) begin
					// Entry number first.
					dp_rs      <= 2'b00;
					dp_byte    <= CPU_A[8:1];
					dp_wr      <= 1'b1;
					dp_is_read <= pal_rd;
					dp_state   <= DP_ADDR;
				end

			DP_ADDR:  dp_state <= DP_ADDR_G;

			DP_ADDR_G: begin
				dp_rs    <= 2'b01;
				dp_byte  <= CPU_D_i[7:0];
				dp_wr    <= ~dp_is_read;
				dp_rd    <=  dp_is_read;
				dp_state <= DP_LO;
			end

			DP_LO:    dp_state <= DP_LO_G;

			DP_LO_G: begin
				dp_read[7:0] <= DAC_D_i;
				dp_byte      <= CPU_D_i[15:8];
				dp_wr        <= ~dp_is_read;
				dp_rd        <=  dp_is_read;
				dp_state     <= DP_HI;
			end

			DP_HI:    dp_state <= DP_HI_G;

			default: begin
				dp_read[15:8] <= DAC_D_i;
				dp_state      <= DP_IDLE;
			end
			endcase
		end
	end

	// ---- what this chip drives on the 68000's bus ---------------------------

	assign CPU_D_o  = sel_regs ? reg_dout
	                : sel_pal  ? dp_read
	                           : cpu_vram_dout;
	assign CPU_D_oe = valid & CPU_RW & (sel_regs | sel_pal | sel_vram);

	// ---- the picture --------------------------------------------------------

	// Line-buffer reads and two composition stages fit inside an eight-clock dot.
	um6618_mixer mixer
	(
		.clk_sys      (clk_sys),
		.tm_index     (tm_index),
		.tm_opaque    (tm_opaque),
		.tm_priority  (tm_priority),
		.tm_tie       (tm_tie),
		.tm_add       (tm_add),
		.tm_enable    ({video_flags[4], video_flags[5],
		                video_flags[6], video_flags[7]}),
		.roz_index    (roz_index),
		.roz_opaque   (roz_opaque),
		.roz_priority (roz_priority),
		.roz_enable   (video_flags[2]),
		.roz_tie      (roz_tie),
		.roz_add      (roz_add),
		.win_pen      (win_pen),
		.win_inside   (win_in),
		.win_priority (win_priority),
		.win_enable   (win_active),
		.spr_index    (spr_index),
		.spr_priority (spr_priority),
		.spr_flags    (spr_flags),
		.spr_backup   (spr_backup),
		.spr_enable   (video_flags[3]),
		.line_masked  (spr_masked),
		.index        (DAC_P)
	);

	assign DAC_PCLK = ce_pix;

	// ---- interrupts to the UM6619 -------------------------------------------

	// Three one-shot latches: vertical retrace at the top of blanking, the
	// visible-line source on every displayed line, and the line comparators.
	// Software never clears them; the 68000 taking the interrupt does, via
	// net_6618_135.
	reg irq_vblank, irq_line, irq_line_on;
	reg irq_ack_q;

	// Pin 134 is the retrace request until a sprite command takes it over.
	assign net_6618_134 = dma_tok ? dma_rev : irq_vblank;
	assign net_6618_127 = irq_line_on;
	assign net_6618_126 = irq_line;

	always @(posedge clk_sys) begin
		irq_ack_q <= net_6618_135;
		if (reset) begin
			irq_ack_q   <= ss_top_saved[35];
			{irq_vblank, irq_line_on, irq_line} <= ss_top_saved[34:32];
		end else begin
			// IACK spans a bus cycle; consume it once. The address names the
			// serviced level. New events below win over the acknowledge.
			if (net_6618_135 && !irq_ack_q) begin
				case (CPU_A[3:1])
					3'd7: irq_vblank  <= 1'b0;
					3'd5: irq_line_on <= 1'b0;
					3'd4: irq_line    <= 1'b0;
					default: ;
				endcase
			end

			if (line_start && (vpos == 9'd240))                    irq_vblank <= 1'b1;
			if (line_start && (vpos != 9'd0) && (vpos < 9'd240))   irq_line   <= 1'b1;

			if (line_start && raster_on[15] && (vpos[7:0] == raster_on[7:0]))
				irq_line_on <= 1'b1;
			if (line_start && raster_off[15] && (vpos[7:0] == raster_off[7:0]))
				irq_line_on <= 1'b0;

			// Reading the status register does not release the interrupt;
			// Boom Zoo polls it thousands of times a frame.
		end
	end

	// Park before sprite evaluation so line pipelines rebuild before display.
	assign ss_ready = pix_ready && (hpos == 10'd0) && (vpos == 9'd255)
	                  && spr_eval_idle && !spr_dma_busy && !cpu_vram_req && !vram_held
	                  && (dp_state == DP_IDLE);

	ss_reg #(.ADDR(SSW_6618_BASE)) state_top (
		.clk_i(clk_sys), .bus_din_i(ss_din), .bus_addr_i(ss_addr),
		.bus_wren_i(ss_wren), .bus_rst_i(ss_rst), .bus_dout_o(ss_top_dout),
		.din_i({23'd0, hblank_q, vblank_q, hsync_q, vsync_q,
		        de_q, irq_ack_q, irq_vblank, irq_line_on, irq_line,
		        cpu_vram_dout, dp_read}), .dout_o(ss_top_saved)
	);
	ss_reg #(.ADDR(SSW_6618_CLOCKS)) state_clocks (
		.clk_i(clk_sys), .bus_din_i(ss_din), .bus_addr_i(ss_addr),
		.bus_wren_i(ss_wren), .bus_rst_i(ss_rst), .bus_dout_o(ss_clocks_dout),
		.din_i(ss_clocks_live), .dout_o(ss_clocks_saved)
	);
	ss_reg #(.ADDR(SSW_6618_TIMING)) state_timing (
		.clk_i(clk_sys), .bus_din_i(ss_din), .bus_addr_i(ss_addr),
		.bus_wren_i(ss_wren), .bus_rst_i(ss_rst), .bus_dout_o(ss_timing_dout),
		.din_i(ss_timing_live), .dout_o(ss_timing_saved)
	);
	assign ss_dout = ss_top_dout | ss_clocks_dout | ss_timing_dout
	               | ss_regs_dout | ss_roz_dout | ss_sprite_dout | ss_dma_dout;

	assign net_6618_137 = 1'b1;

	`ifdef VERILATOR
	/* verilator lint_off UNUSEDSIGNAL */
	wire unused = &{1'b0, CART_31, CART_32,
	                CART_34, CART_68, JOY_2, JP1, JP2, pixel_mode[15:3],
	                ss_top_saved[63:41],
	                video_flags[15:10], video_flags[7:0],
	                raster_on[14:8], raster_off[14:8],
	                CPU_A[22:17]};
	/* verilator lint_on UNUSEDSIGNAL */
	`endif
endmodule
