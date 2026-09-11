// Copyright (c) 2026 Jamie Blanks

// Sprite engine, register block $F00020-$F00026.
//
// Evaluation turns four attribute words into geometry and Bresenham steps,
// running after the frame boundary and sprite-table DMA in the time left
// after each rendered row. Rendering clears the line buffers, walks the
// descriptors and draws each covered row with adds; vertical accumulators
// carry across rows.
//
// Three line buffers share the destination address:
//   S  index, priority, and whether a mask sprite drew this dot
//   B  what was under a mask sprite when it drew
//   K  whether a mask-only sprite covered this dot
//
// The walk stops at SPRITE_MAX entries.

module um6618_sprite (
	input  wire         clk_sys,
	input  wire         reset,

	input  wire [63:0]  regs,        // $20-$26
	input  wire         enable,

	input  wire         prep,
	input  wire         table_updated,
	output wire         eval_idle,
	input  wire [8:0]   render_y,
	input  wire         render_en,
	input  wire         h320,
	input  wire         write_bank,

	input  wire [8:0]   read_x,
	input  wire         read_bank,
	output wire [7:0]   pix_index,
	output wire [1:0]   pix_priority,
	output wire [1:0]   pix_flags,
	output wire [7:0]   pix_backup,
	output reg          line_masked,

	output reg          vram_req,
	output reg  [15:0]  vram_addr,
	input  wire         vram_gnt,
	input  wire         vram_ack,
	input  wire [15:0]  vram_q,

	input  wire [63:0]  ss_din,
	input  wire [9:0]   ss_addr,
	input  wire         ss_wren,
	input  wire         ss_rst,
	output wire [63:0]  ss_dout
);
	`include "ss_map.svh"
	localparam int SPRITE_MAX = 128;
	localparam int IDXW = $clog2(SPRITE_MAX);

	wire [15:0] base_reg  = regs[15:0];    // $20
	wire [15:0] count_reg = regs[31:16];   // $22
	wire [15:0] flag_reg  = regs[63:48];   // $26

	wire       region = ~flag_reg[0];      // 1 = four bits, 0 = eight
	wire [9:0] width_screen = h320 ? 10'd320 : 10'd256;

	// Heights the size field can name, in tiles.
	function automatic [4:0] ysize_of(input [3:0] code);
		case (code)
			4'd0:  ysize_of = 5'd1;   4'd1:  ysize_of = 5'd2;
			4'd2:  ysize_of = 5'd3;   4'd3:  ysize_of = 5'd4;
			4'd4:  ysize_of = 5'd5;   4'd5:  ysize_of = 5'd6;
			4'd6:  ysize_of = 5'd7;   4'd7:  ysize_of = 5'd8;
			4'd8:  ysize_of = 5'd9;   4'd9:  ysize_of = 5'd10;
			4'd10: ysize_of = 5'd12;  4'd11: ysize_of = 5'd16;
			4'd12: ysize_of = 5'd20;  4'd13: ysize_of = 5'd22;
			4'd14: ysize_of = 5'd24;  default: ysize_of = 5'd26;
		endcase
	endfunction

	// ---- the three line buffers ---------------------------------------------

	reg  [8:0]  lb_addr;
	reg         s_wren, b_wren, k_wren;
	reg  [10:0] s_wdata;
	reg  [7:0]  b_wdata;
	reg         k_wdata;
	wire [10:0] s_qa, s_qb;
	wire [7:0]  b_qb;
	wire        k_qb;

	cache_ram_dp #(.ADDR_WIDTH(10), .DATA_WIDTH(11)) buf_s
	(
		.clk_i (clk_sys),
		.addr_a_i ({write_bank, lb_addr}), .wren_a_i (s_wren),
		.wdata_a_i (s_wdata), .q_a_o (s_qa),
		.addr_b_i ({read_bank, read_x}), .wren_b_i (1'b0),
		.wdata_b_i (11'd0), .q_b_o (s_qb)
	);

	/* verilator lint_off PINCONNECTEMPTY */
	cache_ram_dp #(.ADDR_WIDTH(10), .DATA_WIDTH(8)) buf_b
	(
		.clk_i (clk_sys),
		.addr_a_i ({write_bank, lb_addr}), .wren_a_i (b_wren),
		.wdata_a_i (b_wdata), .q_a_o (),
		.addr_b_i ({read_bank, read_x}), .wren_b_i (1'b0),
		.wdata_b_i (8'd0), .q_b_o (b_qb)
	);

	cache_ram_dp #(.ADDR_WIDTH(10), .DATA_WIDTH(1)) buf_k
	(
		.clk_i (clk_sys),
		.addr_a_i ({write_bank, lb_addr}), .wren_a_i (k_wren),
		.wdata_a_i (k_wdata), .q_a_o (),
		.addr_b_i ({read_bank, read_x}), .wren_b_i (1'b0),
		.wdata_b_i (1'b0), .q_b_o (k_qb)
	);
	/* verilator lint_on PINCONNECTEMPTY */

	assign pix_index    = s_qb[7:0];
	assign pix_priority = s_qb[9:8];
	assign pix_flags    = {s_qb[10], k_qb};
	assign pix_backup   = b_qb;

	// ---- the descriptor and the per-sprite vertical state --------------------

	reg  [IDXW-1:0] d_addr;
	reg             d_wren;
	reg  [119:0]    d_wdata;
	wire [119:0]    d_q;

	reg  [IDXW-1:0] v_addr;
	reg             v_wren;
	reg  [31:0]     v_wdata;
	wire [31:0]     v_q;

	// Evaluation can miss its deadline and leave old entries in either cache,
	// so both are saved through their spare ports.
	wire ss_desc_sel = (ss_addr >= 10'(SSW_6618_DESCRIPTORS))
	                 && (ss_addr < 10'(SSW_6618_DESCRIPTORS + 2*SPRITE_MAX));
	wire ss_vstate_sel = (ss_addr >= 10'(SSW_6618_VSTATE))
	                   && (ss_addr < 10'(SSW_6618_VSTATE + SPRITE_MAX));
	wire [9:0] ss_desc_word = ss_addr - 10'(SSW_6618_DESCRIPTORS);
	wire [9:0] ss_vstate_word = ss_addr - 10'(SSW_6618_VSTATE);
	wire [119:0] ss_desc_q;
	wire [31:0] ss_vstate_q;
	wire [63:0] ss_mask_dout;

	cache_ram_dp_be #(.ADDR_WIDTH(IDXW), .DATA_WIDTH(120)) descriptors
	(
		.clk_i (clk_sys),
		.addr_a_i (d_addr), .wren_a_i (d_wren && !reset), .be_a_i (15'h7fff),
		.wdata_a_i (d_wdata), .q_a_o (d_q),
		.addr_b_i (IDXW'(ss_desc_word[9:1])), .wren_b_i (ss_desc_sel && ss_wren),
		.be_b_i (ss_desc_word[0] ? 15'h7f00 : 15'h00ff),
		.wdata_b_i (ss_desc_word[0] ? {ss_din[55:0], 64'd0} : {56'd0, ss_din}),
		.q_b_o (ss_desc_q)
	);

	cache_ram_dp #(.ADDR_WIDTH(IDXW), .DATA_WIDTH(32)) vstate
	(
		.clk_i (clk_sys),
		.addr_a_i (v_addr), .wren_a_i (v_wren && !reset), .wdata_a_i (v_wdata), .q_a_o (v_q),
		.addr_b_i (IDXW'(ss_vstate_word)), .wren_b_i (ss_vstate_sel && ss_wren),
		.wdata_b_i (ss_din[31:0]), .q_b_o (ss_vstate_q)
	);

	assign ss_dout = ss_mask_dout
	               | (ss_desc_sel ? (ss_desc_word[0] ? {8'd0, ss_desc_q[119:64]} : ss_desc_q[63:0]) : 64'd0)
	               | (ss_vstate_sel ? {32'd0, ss_vstate_q} : 64'd0);

	// The descriptor, unpacked.
	wire [15:0] q_w3     = d_q[119:104];
	wire [9:0]  q_x      = d_q[103:94];
	wire [9:0]  q_y      = d_q[93:84];
	wire [8:0]  q_width  = d_q[83:75];
	wire [8:0]  q_height = d_q[74:66];
	wire [3:0]  q_stepx  = d_q[65:62];
	wire [8:0]  q_remx   = d_q[61:53];
	wire [3:0]  q_stepy  = d_q[52:49];
	wire [8:0]  q_remy   = d_q[48:40];
	wire [10:0] q_nw     = d_q[39:29];
	wire [8:0]  q_nh     = d_q[28:20];
	wire [2:0]  q_xslog  = d_q[19:17];
	wire [3:0]  q_bank   = d_q[16:13];
	wire [1:0]  q_mask   = d_q[12:11];
	wire [1:0]  q_prio   = d_q[10:9];
	wire        q_flipx  = d_q[8];
	wire        q_flipy  = d_q[7];
	wire        q_direct = d_q[6];
	wire [2:0]  q_mosaic = d_q[4:2];
	wire [9:0]  q_mosaic_sizes = mosaic_sizes(q_mosaic);
	wire        q_valid  = d_q[1];

	// ---- the divider ---------------------------------------------------------

	// Shift-and-subtract divider, twenty steps.
	reg  [19:0] div_num, div_quot;
	reg  [12:0] div_rem;
	reg  [11:0] div_den;
	reg  [5:0]  div_cnt;
	reg         div_busy;
	reg         div_go;
	reg  [19:0] div_n_in;
	reg  [11:0] div_d_in;

	wire [12:0] div_try = {div_rem[11:0], div_num[19]};

	// div_go pulses for one clock from the pass machine.
	always @(posedge clk_sys) begin
		if (div_go) begin
			div_num  <= div_n_in;
			div_den  <= div_d_in;
			div_rem  <= 13'd0;
			div_quot <= 20'd0;
			div_cnt  <= 6'd20;
			div_busy <= 1'b1;
		end else if (div_busy) begin
			div_num <= {div_num[18:0], 1'b0};
			if (div_try >= {1'b0, div_den}) begin
				div_rem  <= div_try - {1'b0, div_den};
				div_quot <= {div_quot[18:0], 1'b1};
			end else begin
				div_rem  <= div_try;
				div_quot <= {div_quot[18:0], 1'b0};
			end
			div_cnt <= div_cnt - 6'd1;
			if (div_cnt == 6'd1) div_busy <= 1'b0;
		end
	end

	// ---- the pass machine ----------------------------------------------------

	localparam [4:0]
		S_IDLE   = 5'd0,
		E_W0     = 5'd1,  E_W1     = 5'd2,  E_W2   = 5'd3,  E_W3    = 5'd4,
		E_WIDTH  = 5'd5,  E_HEIGHT = 5'd6,  E_STEPX = 5'd7, E_STEPY = 5'd8,
		E_Q0     = 5'd9,  E_ACC    = 5'd10, E_BLK  = 5'd11, E_WRITE = 5'd12,
		E_NEXT   = 5'd13,
		R_CLEAR  = 5'd14, R_DESC   = 5'd15, R_DESC2 = 5'd16, R_ROW  = 5'd17,
		R_PIX    = 5'd18, R_ENTRY  = 5'd19, R_DATA = 5'd20, R_BACKUP = 5'd21,
		R_WRITE  = 5'd22, R_ADVANCE = 5'd23, R_STATE = 5'd24, R_NEXT = 5'd25, E_INIT = 5'd26;

	reg [4:0]  state;
	reg        in_eval;
	reg [4:0]  eval_state;
	reg        eval_pending, eval_active, eval_request, render_pending;
	reg [15:0] eval_i, eval_n;
	reg [8:0]  eval_y;
	wire       eval_read = state >= E_W0 && state <= E_W3;
	wire       row_pending = (prep && render_en) || render_pending;
	wire [8:0] next_eval_y = row_pending ? render_y
	                         : (render_en && render_y < 9'd239) ? render_y + 9'd1 : 9'd0;
	assign eval_idle = !eval_active && !eval_pending && !in_eval && !render_pending;
	reg [15:0] sprite_i, sprite_n, table_ptr;

	reg [15:0] w0, w1, w2, w3;
	reg [10:0] e_nw;
	reg [8:0]  e_nh;
	wire [12:0] e_nh3 = {4'd0, e_nh} + {3'd0, e_nh, 1'b0};   // e_nh * 3
	reg [8:0]  e_width, e_height;
	reg [3:0]  e_stepx, e_stepy;
	reg [8:0]  e_remx, e_remy;
	reg [8:0]  e_q0, e_d0;
	reg [8:0]  e_acc, e_blk, e_racc;
	reg [3:0]  e_cnt0;
	reg        e_direct, e_offset;
	reg [2:0]  e_xslog;

	reg signed [10:0] d_x, d_y;
	reg [8:0]  r_width, r_height;
	reg [3:0]  r_stepx, r_stepy;
	reg [8:0]  r_remx, r_remy;
	reg [10:0] r_nw;
	reg [8:0]  r_nh;
	reg [2:0]  r_xslog;
	reg [3:0]  r_bank;
	reg [1:0]  r_mask, r_prio;
	reg        r_flipx, r_flipy, r_direct, r_valid;
	reg [4:0]  r_mosaic;
	reg [4:0]  r_mosaic_x;
	reg [15:0] r_w3;

	reg [9:0]  destx;
	reg [10:0] sx_acc, sx_blk;
	reg [8:0]  sx_rem;
	reg [3:0]  mx_cnt;
	reg [8:0]  src_y;

	reg [15:0] ent_key, ent_val;
	reg        ent_hit;
	reg [15:0] dat_key, dat_val;
	reg        dat_hit;
	reg        masked_next;

	// Mask presence is published once per frame.
	reg        frame_masked, masked_acc;

	wire [63:0] ss_restore;
	ss_reg #(.ADDR(SSW_6618_SPRITE)) mask_state
	(
		.clk_i(clk_sys), .bus_din_i(ss_din), .bus_addr_i(ss_addr),
		.bus_wren_i(ss_wren), .bus_rst_i(ss_rst), .bus_dout_o(ss_mask_dout),
		.din_i({60'd0, masked_acc, frame_masked, masked_next, line_masked}),
		.dout_o(ss_restore)
	);

	// ---- where the current source pixel comes from and lands -----------------

	wire signed [10:0] screen_x = d_x + $signed({1'b0, destx});
	wire on_screen = (screen_x >= 11'sd0) & (screen_x < $signed({1'b0, width_screen}));

	wire [10:0] src_x = r_flipx ? (r_nw - 11'd1 - sx_blk) : sx_blk;
	wire [8:0] use_y = r_flipy ? (r_nh - 9'd1 - src_y) : src_y;

	wire [15:0] ent_addr = {r_w3[14:0], 1'b0}
	                     + (({10'd0, use_y[8:3]} << r_xslog) + {8'd0, src_x[10:3]});
	wire [15:0] entry    = r_direct ? r_w3 : ent_val;

	wire [12:0] tile_no = {3'd0, entry[9:0]}
	                    + ({9'd0, r_bank} << (region ? 4'd9 : 4'd8));
	wire [2:0]  tile_px = src_x[2:0] ^ {3{entry[11]}};
	wire [2:0]  tile_py = use_y[2:0] ^ {3{entry[10]}};
	wire [18:0] tile_byte = ({6'd0, tile_no} << (region ? 4'd5 : 4'd6))
	                      + ({16'd0, tile_py} << (region ? 3'd2 : 3'd3))
	                      + (region ? {17'd0, tile_px[2:1]} : {16'd0, tile_px});
	wire [15:0] dat_addr = tile_byte[16:1];

	wire [7:0] dat_b     = tile_byte[0] ? dat_val[7:0] : dat_val[15:8];
	wire [3:0] pix4      = tile_px[0] ? dat_b[3:0] : dat_b[7:4];
	wire       pix_on    = region ? (pix4 != 4'd0) : (dat_b != 8'd0);
	wire [7:0] out_index = region ? {entry[15:12], pix4} : dat_b;

	// Horizontal and vertical block sizes grow separately.
	function automatic [9:0] mosaic_sizes(input [2:0] code);
		case (code)
			3'd0: mosaic_sizes = {5'd1, 5'd1};
			3'd1: mosaic_sizes = {5'd2, 5'd2};
			3'd2: mosaic_sizes = {5'd4, 5'd2};
			3'd3: mosaic_sizes = {5'd4, 5'd4};
			3'd4: mosaic_sizes = {5'd4, 5'd8};
			3'd5: mosaic_sizes = {5'd8, 5'd8};
			3'd6: mosaic_sizes = {5'd16, 5'd8};
			3'd7: mosaic_sizes = {5'd16, 5'd16};
		endcase
	endfunction

	// ---- what the evaluation pass computes -----------------------------------

	wire [4:0]  e_ysize  = ysize_of(w0[12:9]);
	wire [7:0]  e_xsize  = 8'd1 << w1[2:0];
	wire [13:0] e_nw_raw = {3'd0, e_xsize, 3'd0};
	wire        e_is1x1  = (e_xsize == 8'd1) & (e_ysize == 5'd1);
	wire [4:0]  e_vscale = {2'd0, w0[15:13]};
	wire [5:0]  e_hscale = {1'b0, w2[15:11]};

	wire signed [10:0] e_sx = (w2[8:0] >= 9'h180)
	                        ? ($signed({2'b00, w2[8:0]}) - 11'sd512)
	                        : $signed({2'b00, w2[8:0]});
	wire signed [10:0] e_sy = (w0[8:0] >= 9'h180)
	                        ? ($signed({2'b00, w0[8:0]}) - 11'sd512)
	                        : $signed({2'b00, w0[8:0]});
	wire signed [10:0] e_row_offset = $signed({2'd0, next_eval_y}) - e_sy;
	wire [9:0] e_mosaic_sizes = mosaic_sizes(w1[5:3]);
	wire [4:0] e_mosaic = e_mosaic_sizes[4:0];

	// Destination row times sprite height in pixels.
	wire [17:0] mul_acc = e_d0 * e_nh;
	wire [17:0] mul_blk = e_q0 * e_nh;

	wire [8:0] cap_quot = (div_quot > 20'd256) ? 9'd256
	                    : (div_quot == 20'd0)  ? 9'd1 : div_quot[8:0];

	// Next line's vertical accumulator for the sprite being drawn.
	wire       vy_carry    = ({1'b0, v_q[26:18]} + {1'b0, r_remy}) >= {1'b0, r_height};
	wire [8:0] vy_rem_next = vy_carry ? (v_q[26:18] + r_remy - r_height)
	                                  : (v_q[26:18] + r_remy);
	wire [8:0] vy_acc_next = v_q[8:0] + {5'd0, r_stepy} + (vy_carry ? 9'd1 : 9'd0);
	wire       vy_wrap     = ({1'b0, v_q[30:27]} == r_mosaic - 5'd1);
	wire [3:0] vy_cnt_next = vy_wrap ? 4'd0 : v_q[30:27] + 4'd1;
	wire [8:0] vy_blk_next = vy_wrap ? vy_acc_next : v_q[17:9];

	// Bresenham step across the row.
	wire       sx_carry    = ({1'b0, sx_rem} + {1'b0, r_remx}) >= {1'b0, r_width};
	wire [8:0]  sx_rem_next = sx_carry ? (sx_rem + r_remx - r_width) : (sx_rem + r_remx);
	wire [10:0] sx_acc_next = sx_acc + {7'd0, r_stepx} + (sx_carry ? 11'd1 : 11'd0);

	// ---- the machine ---------------------------------------------------------

	always @(posedge clk_sys) begin
		reg [4:0] next_state;
		next_state = state;
		s_wren <= 1'b0;
		b_wren <= 1'b0;
		k_wren <= 1'b0;
		d_wren <= 1'b0;
		v_wren <= 1'b0;
		div_go <= 1'b0;
		if (vram_gnt) vram_req <= 1'b0;

		if (reset) begin
			next_state = S_IDLE;
			vram_req     <= 1'b0;
			line_masked  <= ss_restore[0];
			masked_next  <= ss_restore[1];
			frame_masked <= ss_restore[2];
			masked_acc   <= ss_restore[3];
			sprite_n    <= 16'd0;
			in_eval     <= 1'b0;
			eval_pending <= 1'b0;
			eval_active <= 1'b0;
			eval_request <= 1'b0;
			render_pending <= 1'b0;
			eval_state <= S_IDLE;
			eval_i <= 16'd0;
			eval_n <= 16'd0;
		end else begin
			if (prep) begin
				line_masked <= frame_masked;
				if (masked_next) masked_acc <= 1'b1;
				masked_next <= 1'b0;
			end

			// Publish the completed frame's mask history.
			if (prep && (render_y == 9'd256)) begin
				frame_masked <= masked_acc | masked_next;
				masked_acc   <= 1'b0;
			end

			if (table_updated || (prep && render_y == 9'd256))
				eval_pending <= 1'b1;

			if (prep && render_en && !in_eval) begin
				sprite_i <= 16'd0;
				lb_addr <= 9'd0;
				vram_req <= 1'b0;
				next_state = R_CLEAR;
			end else if (prep && !render_en && !in_eval && state != S_IDLE) begin
				vram_req <= 1'b0;
				next_state = S_IDLE;
			end else begin
				case (state)

				S_IDLE: begin
					if (eval_active) begin
						in_eval <= 1'b1;
						next_state = eval_state;
						if (eval_request) begin
							vram_addr <= table_ptr + 16'(eval_state - E_W0);
							vram_req <= 1'b1;
						end
						eval_request <= 1'b0;
					end else if (eval_pending || table_updated || (prep && render_y == 9'd256)) begin
						eval_pending <= 1'b0;
						eval_i <= 16'd0;
						table_ptr <= {base_reg[14:0], 1'b0};
						if (enable) begin
							eval_n <= ({1'b0, count_reg[14:0]} + 16'd1 > 16'(SPRITE_MAX))
							          ? 16'(SPRITE_MAX) : {1'b0, count_reg[14:0]} + 16'd1;
							sprite_n <= ({1'b0, count_reg[14:0]} + 16'd1 > 16'(SPRITE_MAX))
							            ? 16'(SPRITE_MAX) : {1'b0, count_reg[14:0]} + 16'd1;
							vram_addr <= {base_reg[14:0], 1'b0};
							vram_req <= 1'b1;
							eval_active <= 1'b1;
							in_eval <= 1'b1;
							next_state = E_W0;
						end else begin
							sprite_n <= 16'd0;
						end
					end
				end

				// ---- evaluation ------------------------------------------

				E_W0: if (vram_ack) begin
					w0        <= vram_q;
					vram_addr <= table_ptr + 16'd1;
					vram_req  <= 1'b1;
					next_state = E_W1;
				end

				E_W1: if (vram_ack) begin
					w1        <= vram_q;
					vram_addr <= table_ptr + 16'd2;
					vram_req  <= 1'b1;
					next_state = E_W2;
				end

				E_W2: if (vram_ack) begin
					w2        <= vram_q;
					vram_addr <= table_ptr + 16'd3;
					vram_req  <= 1'b1;
					next_state = E_W3;
				end

				E_W3: if (vram_ack) begin
					w3 <= vram_q;
					if (vram_q == 16'd0) begin
						// An empty entry still takes its descriptor slot.
						d_addr  <= eval_i[IDXW-1:0];
						d_wdata <= 120'd0;
						d_wren  <= 1'b1;
						next_state = E_NEXT;
					end else begin
						e_direct <= vram_q[15] | e_is1x1;

						e_xslog  <= (vram_q[15] | e_is1x1) ? 3'd0 : w1[2:0];
						e_nw     <= (vram_q[15] | e_is1x1) ? 11'd8 : e_nw_raw[10:0];
						e_nh     <= (vram_q[15] | e_is1x1) ? 9'd8
						          : {1'd0, e_ysize, 3'd0};
						// width = (hscale + 6*nw) / (hscale + 1)
						div_n_in <= (vram_q[15] | e_is1x1)
						          ? (20'd48 + {14'd0, e_hscale})
						          : ({6'd0, e_nw_raw} << 2) + ({6'd0, e_nw_raw} << 1)
						            + {14'd0, e_hscale};
						div_d_in <= {6'd0, e_hscale} + 12'd1;
						div_go   <= 1'b1;
						next_state = E_WIDTH;
					end
				end

				E_WIDTH: if (!div_busy && !div_go) begin
					e_width <= cap_quot;
					if (e_vscale == 5'd0) begin
						e_height <= (e_nh3 > 13'd256) ? 9'd256 : e_nh3[8:0];
						next_state = E_STEPX;
					end else begin
						// height = (vscale + 2*nh - 1) / vscale
						div_n_in <= {15'd0, e_vscale} + {10'd0, e_nh, 1'b0} - 20'd1;
						div_d_in <= {7'd0, e_vscale};
						div_go   <= 1'b1;
						next_state = E_HEIGHT;
					end
				end

				E_HEIGHT: if (!div_busy && !div_go) begin
					e_height <= cap_quot;
					next_state = E_STEPX;
				end

				E_STEPX: begin
					div_n_in <= {9'd0, e_nw};
					div_d_in <= {3'd0, e_width};
					div_go   <= 1'b1;
					next_state = E_STEPY;
				end

				E_STEPY: if (!div_busy && !div_go) begin
					e_stepx  <= (div_quot > 20'd15) ? 4'd15 : div_quot[3:0];
					e_remx   <= div_rem[8:0];
					div_n_in <= {11'd0, e_nh};
					div_d_in <= {3'd0, e_height};
					div_go   <= 1'b1;
					next_state = E_Q0;
				end

				E_Q0: if (!div_busy && !div_go) begin
					e_stepy <= (div_quot > 20'd15) ? 4'd15 : div_quot[3:0];
					e_remy  <= div_rem[8:0];
					next_state = E_INIT;
				end

				E_INIT: begin
					eval_y <= next_eval_y;
					e_offset <= e_row_offset > 11'sd0;
					if (e_row_offset > 11'sd0) begin
						// A late table update starts at the next unbuilt row.
						e_d0     <= 9'(e_row_offset);
						div_n_in <= {11'd0, 9'(e_row_offset)};
						div_d_in <= {7'd0, e_mosaic};
						div_go   <= 1'b1;
						next_state = E_ACC;
					end else begin
						e_acc  <= 9'd0;
						e_blk  <= 9'd0;
						e_racc <= 9'd0;
						e_cnt0 <= 4'd0;
						next_state = E_WRITE;
					end
				end

				E_ACC: if (!div_busy && !div_go) begin
					// q0 is the mosaic block origin of that first drawn row.
					e_q0     <= 9'(div_quot[8:0] * e_mosaic);
					e_cnt0   <= div_rem[3:0];
					div_n_in <= {2'd0, mul_acc};
					div_d_in <= {3'd0, e_height};
					div_go   <= 1'b1;
					next_state = E_BLK;
				end

				E_BLK: if (!div_busy && !div_go) begin
					e_acc    <= div_quot[8:0];
					e_racc   <= div_rem[8:0];
					div_n_in <= {2'd0, mul_blk};
					div_d_in <= {3'd0, e_height};
					div_go   <= 1'b1;
					next_state = E_WRITE;
				end

				E_WRITE: if (!div_busy && !div_go) begin
					if (eval_y != next_eval_y) begin
						next_state = E_INIT;
					end else begin
						d_addr  <= eval_i[IDXW-1:0];
						d_wdata <= {w3, e_sx[9:0], e_sy[9:0], e_width, e_height,
						            e_stepx, e_remx, e_stepy, e_remy, e_nw, e_nh,
						            e_xslog, w1[15:12], w1[9:8], w2[10:9],
						            w1[11], w1[10], e_direct, 1'b0, w1[5:3], 1'b1, 1'b0};
						d_wren  <= 1'b1;
						v_addr  <= eval_i[IDXW-1:0];
						v_wdata <= {1'b0, e_cnt0, e_racc,
						            e_offset ? div_quot[8:0] : e_blk, e_acc};
						v_wren  <= 1'b1;
						next_state = E_NEXT;
					end
				end

				E_NEXT: begin
					eval_i  <= eval_i + 16'd1;
					table_ptr <= table_ptr + 16'd4;
					if (eval_i + 16'd1 >= eval_n) begin
						in_eval <= 1'b0;
						eval_active <= 1'b0;
						next_state = S_IDLE;
					end else begin
						vram_addr <= table_ptr + 16'd4;
						vram_req  <= 1'b1;
						next_state = E_W0;
					end
				end

				// ---- the render pass -------------------------------------

				R_CLEAR: begin
					lb_addr <= lb_addr + 9'd1;
					s_wren  <= 1'b1;
					b_wren  <= 1'b1;
					k_wren  <= 1'b1;
					s_wdata <= 11'd0;
					b_wdata <= 8'd0;
					k_wdata <= 1'b0;
						if ({1'b0, lb_addr} + 10'd1 >= width_screen) begin
						lb_addr <= 9'd0;
						if (enable && sprite_n != 16'd0) begin
							d_addr <= {IDXW{1'b0}};
							v_addr <= {IDXW{1'b0}};
							next_state = R_DESC;
						end else begin
							next_state = S_IDLE;
						end
					end
				end

				R_DESC: next_state = R_DESC2;

				R_DESC2: begin
					d_x      <= {q_x[9], q_x};
					d_y      <= {q_y[9], q_y};
					r_width  <= q_width;
					r_height <= q_height;
					r_stepx  <= q_stepx;
					r_remx   <= q_remx;
					r_stepy  <= q_stepy;
					r_remy   <= q_remy;
					r_nw     <= q_nw;
					r_nh     <= q_nh;
					r_xslog  <= q_xslog;
					r_bank   <= q_bank;
					r_mask   <= q_mask;
					r_prio   <= q_prio;
					r_flipx  <= q_flipx;
					r_flipy  <= q_flipy;
					r_direct <= q_direct;
					r_mosaic <= q_mosaic_sizes[4:0];
					r_mosaic_x <= q_mosaic_sizes[9:5];
					r_w3     <= q_w3;
					r_valid  <= q_valid;
					next_state = R_ROW;
				end

				R_ROW:
					if (!r_valid
					    || ($signed({2'b00, render_y}) < d_y)
					    || ($signed({2'b00, render_y})
					        >= d_y + $signed({2'd0, r_height}))) begin
						next_state = R_NEXT;
					end else begin
						src_y   <= v_q[17:9];
						destx   <= 10'd0;
						lb_addr <= d_x[8:0];
						sx_acc  <= 11'd0;
						sx_blk  <= 11'd0;
						sx_rem  <= 9'd0;
						mx_cnt  <= 4'd0;
						ent_hit <= 1'b0;
						dat_hit <= 1'b0;
						next_state = R_PIX;
					end

				R_PIX:
					if ({1'b0, destx} >= {1'b0, r_width}) begin
						next_state = R_STATE;
					end else if (!on_screen) begin
						next_state = R_ADVANCE;
					end else if (!r_direct && (!ent_hit || ent_key != ent_addr)) begin
						vram_addr <= ent_addr;
						vram_req  <= 1'b1;
						next_state = R_ENTRY;
					end else if (entry == 16'd0 && (region || r_bank == 4'd0)) begin
						next_state = R_ADVANCE;
					end else if (!dat_hit || dat_key != dat_addr) begin
						vram_addr <= dat_addr;
						vram_req  <= 1'b1;
						next_state = R_DATA;
					end else if (!pix_on) begin
						next_state = R_ADVANCE;
					end else begin
						// Modes 1 and 3 preserve the covered colour, either
						// as a backup or while replacing only its priority.
						next_state = (r_mask == 2'd1 || r_mask == 2'd3) ? R_BACKUP : R_WRITE;
					end

				R_ENTRY: if (vram_ack) begin
					ent_val <= vram_q;
					ent_key <= ent_addr;
					ent_hit <= 1'b1;
					next_state = R_PIX;
				end

				R_DATA: if (vram_ack) begin
					dat_val <= vram_q;
					dat_key <= dat_addr;
					dat_hit <= 1'b1;
					next_state = R_PIX;
				end

				R_BACKUP: next_state = R_WRITE;

				R_WRITE: begin
					if (r_mask > 2'd1) begin
						if (r_mask == 2'd3) begin
							// The foreground mask moves the covered sprite
							// ahead of a layer without replacing its colour.
							s_wdata <= {s_qa[10], r_prio, s_qa[7:0]};
							s_wren  <= 1'b1;
						end
						// Mask only: it marks the dot and draws nothing.
						k_wdata     <= 1'b1;
						k_wren      <= 1'b1;
						masked_next <= 1'b1;
					end else if (s_qa[7:0] == 8'd0 || r_prio <= s_qa[9:8]) begin
						s_wdata <= {r_mask == 2'd1, r_prio, out_index};
						s_wren  <= 1'b1;
						if (r_mask == 2'd1) begin
							b_wdata <= s_qa[7:0];
							b_wren  <= 1'b1;
						end
					end
					next_state = R_ADVANCE;
				end

				R_ADVANCE: begin
					destx  <= destx + 10'd1;
					// The pending write uses this dot; prefetch the next one.
					lb_addr <= screen_x[8:0] + 9'd1;
					sx_rem <= sx_rem_next;
					sx_acc <= sx_acc_next;
					if ({1'b0, mx_cnt} == r_mosaic_x - 5'd1) begin
						mx_cnt <= 4'd0;
						sx_blk <= sx_acc_next;
					end else begin
						mx_cnt <= mx_cnt + 4'd1;
					end
					next_state = R_PIX;
				end

				R_STATE: begin
					v_addr  <= sprite_i[IDXW-1:0];
					v_wdata <= {1'b0, vy_cnt_next, vy_rem_next,
					            vy_blk_next, vy_acc_next};
					v_wren  <= 1'b1;
					next_state = R_NEXT;
				end

				R_NEXT: begin
					sprite_i <= sprite_i + 16'd1;
					if (sprite_i + 16'd1 >= sprite_n) begin
						next_state = S_IDLE;
					end else begin
						d_addr <= sprite_i[IDXW-1:0] + {{(IDXW-1){1'b0}}, 1'b1};
						v_addr <= sprite_i[IDXW-1:0] + {{(IDXW-1){1'b0}}, 1'b1};
						next_state = R_DESC;
					end
				end

				default: ;   // S_IDLE
				endcase
			end

			// Finish an issued header read before lending the port to a row.
			if (row_pending && in_eval) begin
				render_pending <= 1'b1;
				if (!eval_read || vram_ack) begin
					eval_state <= next_state;
					eval_request <= next_state >= E_W0 && next_state <= E_W3;
					in_eval <= 1'b0;
					vram_req <= 1'b0;
					render_pending <= 1'b0;
					sprite_i <= 16'd0;
					lb_addr <= 9'd0;
					next_state = R_CLEAR;
				end
			end
		end
		state <= next_state;
	end

	`ifdef VERILATOR
	/* verilator lint_off UNUSEDSIGNAL */
	wire unused = &{1'b0, regs[47:32], base_reg[15], count_reg[15],
	                flag_reg[15:1], d_q[0], v_q[31], tile_byte[18:17],
	                div_quot[19:9], screen_x[10:9], e_nw_raw[13:11],
	                w0[8:0], w1[7:6], w2[8:0], ss_restore[63:4]};
	/* verilator lint_on UNUSEDSIGNAL */
	`endif
endmodule
