// Copyright (c) 2026 Jamie Blanks

// UM6619 sampler: sixteen channels of eight-bit unsigned PCM from sound RAM,
// plus the timer that paces the driver. The 65C02 reaches it indirectly:
// $0420 is the register number, $0422 the data.
//
// One output sample every eighty sound clocks (44744.3125 Hz); each channel
// has a 16.16 phase accumulator with increment pitch << 6. Key-on at $17
// points the channel at start << 6 with end = start + length ($50). At the
// end a channel with non-zero $90 is a DMA stream and raises the sample
// interrupt and restarts; a one-shot stops; anything else loops. A streaming
// one-shot stops on a zero sample, since the driver counts whole buffers.
//
// The timer alternates phases every 5 x (0x10000 - n) sound clocks. Reading
// $14 returns the phase in bit 6 and acknowledges its interrupt; reading $16
// acknowledges the sample interrupt and reports whether any stream runs.
//
// Volume scales the sample by nibble/16 (boot-chime notes at $44 and $66
// measure 0.672 on the console). One channel is mixed at a time.

module um6619_snd (
	input  wire        clk_sys,
	input  wire        reset,
	input  wire        ce_snd,        // one pulse per sound-CPU cycle

	// Indirect register file.
	input  wire [7:0]  reg_addr,
	input  wire [7:0]  reg_wdata,
	input  wire        reg_wr,
	input  wire        reg_rd,
	output wire [7:0]  reg_rdata,
	output wire [7:0]  status,

	// Interrupt sources, level held until their register is read.
	output reg         irq_timer,
	output reg         irq_dma,

	// Sound RAM read port.
	output reg         ram_req,
	output reg  [15:0] ram_addr,
	input  wire [7:0]  ram_dout,
	input  wire        ram_done,

	output reg signed [15:0] audio_l,
	output reg signed [15:0] audio_r,

	// Savestate. ss_load is the restore window and outranks reset.
	input  wire        ss_load,
	input  wire [63:0] ss_din,
	input  wire [9:0]  ss_addr,
	input  wire        ss_wren,
	input  wire        ss_rst,
	output wire [63:0] ss_dout
);
	`include "ss_map.svh"

	// The restore enable fans out to more flops here than anywhere else, so
	// it is copied one flop deep: one per quarter of the register store, one
	// per channel, one for the rest. Every block delays it the same way.
	reg  [3:0] ss_load_regs_q = 4'd0;
	reg [15:0] ss_load_ch_q   = 16'd0;
	reg        ss_load_misc_q = 1'b0;
	always @(posedge clk_sys) begin
		ss_load_regs_q <= {4{ss_load}};
		ss_load_ch_q   <= {16{ss_load}};
		ss_load_misc_q <= ss_load;
	end

	localparam int CYCLES_PER_SAMPLE = 80;

	// ---- the register file ---------------------------------------------------

	reg [7:0] regs [0:255];

	// ---- per-channel state ---------------------------------------------------

	reg [15:0] pitch    [0:15];
	reg [21:0] incr     [0:15];
	reg [15:0] start    [0:15];
	reg [15:0] length   [0:15];
	reg [15:0] current  [0:15];
	reg [15:0] end_addr [0:15];
	reg [15:0] fraction [0:15];
	reg [7:0]  dma_flag [0:15];
	reg [7:0]  vol_l    [0:15];
	reg [7:0]  vol_r    [0:15];
	reg        one_shot [0:15];
	reg [15:0] active;
	reg [15:0] releasing;
	reg [15:0] attacking, decaying;
	reg [31:0] release_level [0:15];

	// Volume nibble in the top four bits of a byte, so the multiplier stays
	// one shape.
	function automatic [7:0] expand_vol(input [3:0] n);
		expand_vol = {n, 4'd0};
	endfunction

	// ---- the savestate words -------------------------------------------------

	//    2047:0     regs[0..255]        3935:3680  fraction
	//    2303:2048  pitch               4063:3936  dma_flag
	//    2655:2304  incr                4191:4064  vol_l
	//    2911:2656  start               4319:4192  vol_r
	//    3167:2912  length              4335:4320  one_shot
	//    3423:3168  current             4847:4336  release_level
	//    3679:3424  end_addr            5148:4848  everything not per channel
	localparam int SS_SND_BITS  = 5149;
	localparam int SS_SND_WORDS = 81;

	wire [SS_SND_BITS-1:0] ss_live;
	wire [SS_SND_WORDS*64-1:0] ss_saved;
	wire [SS_SND_WORDS*64-1:0] ss_live_padded =
		{ {(SS_SND_WORDS*64 - SS_SND_BITS){1'b0}}, ss_live };
	wire [63:0] ss_word_dout [SS_SND_WORDS];

	genvar gw;
	generate
		for (gw = 0; gw < SS_SND_WORDS; gw++) begin : g_ss_words
			ss_reg #(.ADDR(SSW_6619_SND + gw)) word (
				.clk_i      (clk_sys),
				.bus_din_i  (ss_din),
				.bus_addr_i (ss_addr),
				.bus_wren_i (ss_wren),
				// Only read inside the restore window, so no reset needed.
				.bus_rst_i  (1'b0),
				.bus_dout_o (ss_word_dout[gw]),
				.din_i      (ss_live_padded[gw*64 +: 64]),
				.dout_o     (ss_saved[gw*64 +: 64])
			);
		end
	endgenerate

	logic [63:0] ss_dout_or;
	always_comb begin
		ss_dout_or = 64'd0;
		for (int k = 0; k < SS_SND_WORDS; k++)
			ss_dout_or = ss_dout_or | ss_word_dout[k];
	end
	assign ss_dout = ss_dout_or;

	genvar gc;
	generate
		for (gc = 0; gc < 256; gc++) begin : g_ss_regs
			assign ss_live[gc*8 +: 8] = regs[gc];
		end
		for (gc = 0; gc < 16; gc++) begin : g_ss_chan
			assign ss_live[2048 + gc*16 +: 16] = pitch[gc];
			assign ss_live[2304 + gc*22 +: 22] = incr[gc];
			assign ss_live[2656 + gc*16 +: 16] = start[gc];
			assign ss_live[2912 + gc*16 +: 16] = length[gc];
			assign ss_live[3168 + gc*16 +: 16] = current[gc];
			assign ss_live[3424 + gc*16 +: 16] = end_addr[gc];
			assign ss_live[3680 + gc*16 +: 16] = fraction[gc];
			assign ss_live[3936 + gc*8  +: 8]  = dma_flag[gc];
			assign ss_live[4064 + gc*8  +: 8]  = vol_l[gc];
			assign ss_live[4192 + gc*8  +: 8]  = vol_r[gc];
			assign ss_live[4320 + gc]          = one_shot[gc];
			assign ss_live[4336 + gc*32 +: 32] = release_level[gc];
		end
	endgenerate

	assign ss_live[5148:4848] = { timer_phase, audio_r, audio_l, ram_addr, ram_req, irq_dma, irq_timer, held_valid, held_step, effective_r, effective_l, fetched, acc_r, acc_l, sample_cnt, mch, mstate, reg_16, reg_14, reg_12, reg_11, timer_running, timer_left, decaying, attacking, releasing, active };

	`ifdef VERILATOR
	/* verilator lint_off UNUSEDSIGNAL */
	wire ss_unused = &{1'b0, ss_rst, ss_saved[SS_SND_WORDS*64-1:SS_SND_BITS]};
	/* verilator lint_on UNUSEDSIGNAL */
	`endif

	// ---- the timer -----------------------------------------------------------

	reg [31:0] timer_left;
	reg        timer_running;
	reg        timer_phase;

	// Own copies of the four readback registers keep the flat store to one
	// asynchronous read port.
	reg [7:0] reg_11, reg_12, reg_14, reg_16;

	wire [15:0] timer_n      = {reg_12, reg_11};
	wire [16:0] timer_count  = 17'h10000 - {1'b0, timer_n};
	wire [18:0] timer_ticks  = {timer_count, 2'd0} + {2'd0, timer_count};
	wire [31:0] timer_period = {13'd0, timer_ticks};

	// ---- reading the register file -------------------------------------------

	reg [15:0] dma_active_mask;
	integer m;
	wire dma_busy = |(active & dma_active_mask);
	always @(*) begin
		dma_active_mask = 16'd0;
		for (m = 0; m < 16; m = m + 1)
			dma_active_mask[m] = (dma_flag[m] != 8'd0);
	end

	assign status    = 8'd0;
	assign reg_rdata = (reg_addr == 8'h16)
	                 ? ((reg_16 & ~8'h40) | (dma_busy ? 8'h40 : 8'h00))
	                 : (reg_addr == 8'h14) ? {reg_14[7], timer_phase, reg_14[5:0]}
	                 : regs[reg_addr];

	// ---- the mixing sequencer ------------------------------------------------

	localparam [2:0] M_IDLE = 3'd0, M_PICK = 3'd1, M_FETCH = 3'd2,
	                 M_MIX = 3'd3, M_STEP = 3'd4, M_DONE = 3'd5, M_ENV = 3'd6;

	reg [2:0]  mstate;
	reg [4:0]  mch;
	reg [6:0]  sample_cnt;
	reg signed [23:0] acc_l, acc_r;
	reg [7:0]  fetched;
	reg [7:0] effective_l, effective_r;

	wire [3:0] c = mch[3:0];
	wire key_event_c = reg_wr && reg_addr == 8'h17 && reg_wdata[3:0] == c;
	// Control byte of the channel a key write names.
	wire [7:0] ctrl_w_idx = {4'h4, reg_wdata[3:0]};
	wire [7:0] ctrl_w = regs[ctrl_w_idx];
	wire key_on_c = key_event_c && reg_wdata[7:4] != 4'd0;
	wire envelope_write_c = reg_wr && reg_addr[3:0] == c
	                        && (reg_addr[7:4] == 4'h4 || reg_addr[7:4] == 4'h9
	                            || (reg_addr[7:4] >= 4'ha && reg_addr[7:4] <= 4'hd));

	// Reuse the two multipliers to apply the envelope before mixing a voice.
	wire signed [8:0]  pcm      = $signed({1'b0, fetched}) - 9'sd128;
	wire signed [8:0] mul_l = mstate == M_ENV
	                           ? $signed({1'b0, vol_l[c]})
	                           : pcm;
	wire signed [8:0] mul_r = mstate == M_ENV
	                           ? $signed({1'b0, vol_r[c]})
	                           : pcm;
	wire signed [8:0] gain_l = mstate == M_ENV
	                           ? $signed({1'b0, release_level[c][31:24]})
	                           : $signed({1'b0, effective_l});
	wire signed [8:0] gain_r = mstate == M_ENV
	                           ? $signed({1'b0, release_level[c][31:24]})
	                           : $signed({1'b0, effective_r});
	wire signed [17:0] mix_l = mul_l * gain_l;
	wire signed [17:0] mix_r = mul_r * gain_r;
	// Adding volume makes the maximum release level exactly unity.
	wire [17:0] scaled_l = $unsigned(mix_l) + {10'd0, vol_l[c]};
	wire [17:0] scaled_r = $unsigned(mix_r) + {10'd0, vol_r[c]};
	// Linear ramp, slope doubling per code; full-scale RR7 lasts about
	// 0.875 s. Code 0 stops the voice at once rather than ramping for two
	// minutes, since drivers key off code-0 voices with the next note.
	wire [3:0]  release_rate = regs[{4'hd, c}][3:0];
	wire [31:0] release_step = release_rate == 4'd0 ? 32'hffffffff
	                                               : 32'd857 << release_rate;
	// Attack and first decay approach a target; second decay falls to zero.
	// Rate codes use a power-of-two approximation.
	wire [7:0] ctrl_c_idx = {4'h4, c};
	wire [7:0] ctrl_c = regs[ctrl_c_idx];
	wire envelope = ctrl_c[1] && dma_flag[c] == 8'd0;
	wire [4:0] attack_rate = regs[{4'ha, c}][4:0];
	wire [4:0] decay_rate = regs[{4'hb, c}][4:0];
	wire [4:0] sustain_rate = regs[{4'hc, c}][4:0];
	wire [31:0] sustain_level = 32'hffffffff - {regs[{4'hd, c}][7:4], 28'd0};
	wire [4:0] envelope_shift = attacking[c]
	                           ? 5'd18 - {1'b0, attack_rate[4:1]}
	                           : 5'd20 - {1'b0, decay_rate[4:1]};
	wire [31:0] envelope_gap = attacking[c] ? ~release_level[c]
	                          : release_level[c] > sustain_level
	                          ? release_level[c] - sustain_level : 32'd0;
	wire [31:0] sustain_step = sustain_rate == 5'd0 ? 32'd0
	                          : 32'd857 << sustain_rate[4:1];
	reg [31:0] held_step;
	reg held_valid;

	wire [21:0] step_sum  = {6'd0, fraction[c]} + incr[c];
	wire [15:0] frac_next = step_sum[15:0];
	wire [15:0] curr_next = current[c] + {10'd0, step_sum[21:16]};

	// Where a channel goes at its end.
	wire hit_end = (curr_next >= end_addr[c]);

	function automatic signed [15:0] sat16(input signed [23:0] v);
		if (v > 24'sd32767)       sat16 = 16'sd32767;
		else if (v < -24'sd32768) sat16 = 16'sh8000;
		else                      sat16 = v[15:0];
	endfunction

	integer i;

	// The RAM request is a level: the arbiter only takes a client when idle
	// and outside the 6502's reserved clocks, so a pulse would be missed.
	always @(posedge clk_sys) begin
		for (i = 0; i < 256; i = i + 1)
			if (ss_load_regs_q[i[7:6]]) regs[i] <= ss_saved[i*8 +: 8];

		for (i = 0; i < 16; i = i + 1)
			if (ss_load_ch_q[i]) begin
				pitch[i]         <= ss_saved[2048 + i*16 +: 16];
				incr[i]          <= ss_saved[2304 + i*22 +: 22];
				start[i]         <= ss_saved[2656 + i*16 +: 16];
				length[i]        <= ss_saved[2912 + i*16 +: 16];
				current[i]       <= ss_saved[3168 + i*16 +: 16];
				end_addr[i]      <= ss_saved[3424 + i*16 +: 16];
				fraction[i]      <= ss_saved[3680 + i*16 +: 16];
				dma_flag[i]      <= ss_saved[3936 + i*8  +: 8];
				vol_l[i]         <= ss_saved[4064 + i*8  +: 8];
				vol_r[i]         <= ss_saved[4192 + i*8  +: 8];
				one_shot[i]      <= ss_saved[4320 + i];
				release_level[i] <= ss_saved[4336 + i*32 +: 32];
			end

		if (ss_load_misc_q) begin
			{ timer_phase, audio_r, audio_l, ram_addr, ram_req, irq_dma, irq_timer, held_valid, held_step, effective_r, effective_l, fetched, acc_r, acc_l, sample_cnt, mch, mstate, reg_16, reg_14, reg_12, reg_11, timer_running, timer_left, decaying, attacking, releasing, active } <= ss_saved[5148:4848];
		end
		else begin
		if (ram_done) ram_req <= 1'b0;

		if (reset) begin
			active        <= 16'd0;
			releasing     <= 16'd0;
			attacking    <= 16'd0;
			decaying     <= 16'd0;
			held_valid   <= 1'b0;
			effective_l   <= 8'd0;
			effective_r   <= 8'd0;
			mstate        <= M_IDLE;
			sample_cnt    <= 7'd0;
			timer_running <= 1'b0;
			timer_phase   <= 1'b0;
			timer_left    <= 32'd0;
			irq_timer     <= 1'b0;
			irq_dma       <= 1'b0;
			ram_req       <= 1'b0;
			ram_addr      <= 16'd0;
			fetched       <= 8'd0;
			mch           <= 5'd0;
			acc_l         <= 24'sd0;
			acc_r         <= 24'sd0;
			reg_11        <= 8'd0;
			reg_12        <= 8'd0;
			reg_14        <= 8'd0;
			reg_16        <= 8'd0;
			audio_l       <= 16'sd0;
			audio_r       <= 16'sd0;
			for (i = 0; i < 16; i = i + 1) begin
				release_level[i] <= 32'd0;
				pitch[i]    <= 16'd0;
				incr[i]     <= 22'd0;
				start[i]    <= 16'd0;
				length[i]   <= 16'd0;
				current[i]  <= 16'd0;
				end_addr[i] <= 16'd0;
				fraction[i] <= 16'd0;
				dma_flag[i] <= 8'd0;
				vol_l[i]    <= 8'd0;
				vol_r[i]    <= 8'd0;
				one_shot[i] <= 1'b0;
			end
			for (i = 0; i < 256; i = i + 1) regs[i] <= 8'd0;
			held_step  <= 32'd0;
		end else begin
			// ---- the 65C02's register access ----------------------------

			if (reg_rd) begin
				if (reg_addr == 8'h14) irq_timer <= 1'b0;
				if (reg_addr == 8'h16) irq_dma   <= 1'b0;
			end

			if (reg_wr) begin
				regs[reg_addr] <= reg_wdata;
				case (reg_addr)
					8'h11:   reg_11 <= reg_wdata;
					8'h12:   reg_12 <= reg_wdata;
					8'h14:   reg_14 <= reg_wdata;
					8'h16:   reg_16 <= reg_wdata;
					default: ;
				endcase
				case (reg_addr[7:4])
				4'h1:
					if (reg_addr == 8'h14) begin
						if (reg_wdata[7:6] == 2'b00) begin
							timer_running <= 1'b0;
							irq_timer     <= 1'b0;
						end else if (!timer_running) begin
							timer_running <= 1'b1;
							timer_left    <= timer_period;
						end
					end else if (reg_addr == 8'h17) begin
						if (reg_wdata[7:4] != 4'd0) begin
							// Key on.
							if (!ctrl_w[1]
							    || dma_flag[reg_wdata[3:0]] != 8'd0
							    || regs[{4'ha, reg_wdata[3:0]}][4:0] == 5'd31)
								release_level[reg_wdata[3:0]] <= 32'hffffffff;
							else if (!active[reg_wdata[3:0]])
								release_level[reg_wdata[3:0]] <= 32'd0;
							attacking[reg_wdata[3:0]] <= regs[{4'ha, reg_wdata[3:0]}][4:0] != 5'd31;
							decaying[reg_wdata[3:0]] <= 1'b1;
							releasing[reg_wdata[3:0]] <= 1'b0;
							current[reg_wdata[3:0]]  <= start[reg_wdata[3:0]] << 6;
							end_addr[reg_wdata[3:0]] <= (start[reg_wdata[3:0]] << 6)
							                          + length[reg_wdata[3:0]];
							fraction[reg_wdata[3:0]] <= 16'd0;
							active[reg_wdata[3:0]]   <= 1'b1;
						end else if (ctrl_w[1]
						             && dma_flag[reg_wdata[3:0]] == 8'd0) begin
							// Key-off releases the loop; streams keep their stop path.
							releasing[reg_wdata[3:0]] <= 1'b1;
						end else begin
							active[reg_wdata[3:0]]   <= 1'b0;
						end
					end
				4'h2: begin
					pitch[reg_addr[3:0]][7:0] <= reg_wdata;
					incr[reg_addr[3:0]]       <= {pitch[reg_addr[3:0]][15:8],
					                              reg_wdata, 6'd0};
				end
				4'h3: begin
					pitch[reg_addr[3:0]][15:8] <= reg_wdata;
					incr[reg_addr[3:0]]        <= {reg_wdata,
					                               pitch[reg_addr[3:0]][7:0], 6'd0};
				end
				4'h5: begin
					length[reg_addr[3:0]]   <= 16'h40 << reg_wdata[3:1];
					one_shot[reg_addr[3:0]] <= reg_wdata[0];
				end
				4'h6: start[reg_addr[3:0]][15:8] <= reg_wdata;
				4'h7: start[reg_addr[3:0]][7:0]  <= reg_wdata;
				4'h9: dma_flag[reg_addr[3:0]]    <= reg_wdata;
				4'hE: begin
					vol_l[reg_addr[3:0]] <= expand_vol(reg_wdata[7:4]);
					vol_r[reg_addr[3:0]] <= expand_vol(reg_wdata[3:0]);
				end
				default: ;
				endcase
			end

			// ---- the timer -----------------------------------------------

			if (ce_snd && timer_running
			    && !(reg_wr && reg_addr == 8'h14 && reg_wdata[7:6] == 2'b00)) begin
				if (timer_left <= 32'd1) begin
					timer_left <= timer_period;
					timer_phase <= ~timer_phase;
					irq_timer   <= 1'b1;
				end else begin
					timer_left <= timer_left - 32'd1;
				end
			end

			// ---- one output sample every eighty sound clocks ---------------

			if (ce_snd) begin
				if (sample_cnt == 7'(CYCLES_PER_SAMPLE - 1)) begin
					sample_cnt <= 7'd0;
					if (mstate == M_IDLE) begin
						mch    <= 5'd0;
						acc_l  <= 24'sd0;
						acc_r  <= 24'sd0;
						mstate <= M_PICK;
					end
				end else begin
					sample_cnt <= sample_cnt + 7'd1;
				end
			end

			case (mstate)
			M_PICK:
				if (mch == 5'd16) begin
					mstate <= M_DONE;
				end else if (!active[c]) begin
					mch <= mch + 5'd1;
				end else begin
					// Full sixteen bits; the arbiter decides whether A15 exists.
					ram_addr <= current[c];
					ram_req  <= 1'b1;
					mstate   <= M_FETCH;
				end

			M_FETCH:
				if (ram_done) begin
					held_valid <= 1'b0;
					if (ram_dout == 8'd0 && dma_flag[c] != 8'd0 && one_shot[c]) begin
						// A streamed sound ends on a zero byte, which is never
						// a real sample value. Sango Fighter's announcer ends
						// this way a hundred bytes into a 512-byte buffer.
						// Tested before the mix so that byte is not heard.
						active[c] <= 1'b0;
						irq_dma   <= 1'b1;
						mch       <= mch + 5'd1;
						mstate    <= M_PICK;
					end else begin
						fetched <= ram_dout;
						effective_l <= vol_l[c];
						effective_r <= vol_r[c];
						mstate <= (releasing[c] || envelope) ? M_ENV : M_MIX;
					end
				end

			M_ENV: begin
				effective_l <= scaled_l[15:8];
				effective_r <= scaled_r[15:8];
				held_step <= envelope_gap >> envelope_shift;
				held_valid <= !key_event_c && !envelope_write_c;
				mstate <= M_MIX;
			end

			M_MIX: begin
				// A command can replace the envelope after its step was computed.
				if (key_event_c || envelope_write_c) held_valid <= 1'b0;
				acc_l  <= acc_l + {{6{mix_l[17]}}, mix_l};
				acc_r  <= acc_r + {{6{mix_r[17]}}, mix_r};
				mstate <= M_STEP;
			end

			M_STEP: begin
				if (releasing[c] && !key_event_c) begin
					if (release_level[c] <= release_step) begin
						release_level[c] <= 32'd0;
						active[c] <= 1'b0;
					end else begin
						release_level[c] <= release_level[c] - release_step;
					end
				end else if (envelope && held_valid && !key_event_c && !envelope_write_c) begin
					if (attacking[c]) begin
						if (attack_rate != 5'd0) begin
							if (attack_rate == 5'd31 || envelope_gap <= 32'h01ffffff) begin
								release_level[c] <= 32'hffffffff;
								attacking[c] <= 1'b0;
							end else release_level[c] <= release_level[c] + held_step;
						end
					end else if (decaying[c]) begin
						if (decay_rate != 5'd0) begin
							if (envelope_gap <= 32'h00ffffff) begin
								release_level[c] <= sustain_level;
								decaying[c] <= 1'b0;
							end else release_level[c] <= release_level[c] - held_step;
						end
					end else if (sustain_step != 32'd0) begin
						if (release_level[c] <= sustain_step) begin
							release_level[c] <= 32'd0;
							active[c] <= 1'b0;
						end else release_level[c] <= release_level[c] - sustain_step;
					end
				end
				// A new note owns its phase even when the old note ends here.
				if (!key_on_c) begin
					fraction[c] <= frac_next;
					current[c]  <= curr_next;
					if (hit_end) begin
						if (dma_flag[c] != 8'd0) begin
							// Stream: ask for the next buffer and restart. The
							// sub-sample phase carries over so the double buffer
							// is seamless.
							irq_dma     <= 1'b1;
							current[c]  <= start[c] << 6;
							end_addr[c] <= (start[c] << 6) + length[c];
						end else if (one_shot[c]) begin
							active[c] <= 1'b0;
						end else if (length[c] != 16'd0) begin
							current[c] <= curr_next - length[c];
						end
					end
				end
				mch    <= mch + 5'd1;
				mstate <= M_PICK;
			end

			M_DONE: begin
				// 16 voices x 127 x 15 = 30480 fits sixteen bits; the >>> 4
				// undoes the volume nibble's shift. Saturation guards later
				// changes.
				audio_l <= sat16(acc_l >>> 4);
				audio_r <= sat16(acc_r >>> 4);
				mstate  <= M_IDLE;
			end

			default: ;
			endcase
		end
		end
	end

	`ifdef VERILATOR
	/* verilator lint_off UNUSEDSIGNAL */
	wire unused = &{1'b0, step_sum[15:0], mch[4]};
	/* verilator lint_on UNUSEDSIGNAL */
	`endif
endmodule
