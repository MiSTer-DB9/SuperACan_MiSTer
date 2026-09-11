// Copyright (c) 2026 Jamie Blanks

// UM6618 rotate-and-zoom layer, register block $F00180-$F0019E.
//
// The affine transform is accumulated, not multiplied:
//
//   sourceX = (scrollX + y*B + x*A) >> 8
//   sourceY = (scrollY + y*D + x*C) >> 8
//
// with 8.8 coefficients, adding A and C per dot and B and D per line. Mosaic
// quantises each screen axis to a latched block origin first.
//
// Per-line tables replace A and the y*D term. Tile-control bit 15 enables
// the Y table, bit 6 the $F00198 scale table, bit 8 two X words per line
// instead of one offset for all. $F0019C is a fourth table, fetched but unused.
//
//   mode[1:0]  region                 pixel source
//   ---------  ---------------------  ---------------------------------------
//      0       1 bpp boot logo        fixed tile layout with swapped
//                                     address bits
//      1       two bits a pixel       map entry, then tile data
//      2       four bits a pixel      map entry, then tile data
//      3       eight bits a pixel     map entry, then tile data, unless
//                                     $F001F0 selects the linear bitmap
//
// One map entry and one data word are held and refetched when the source
// leaves them. If a line runs out of time only its completed prefix shows.

module um6618_roz (
	input  wire         clk_sys,
	input  wire         reset,

	input  wire [255:0] regs,        // $180-$19E, sixteen words
	input  wire [15:0]  pixel_mode,  // $1F0
	input  wire         enable,

	input  wire         prep,
	input  wire [8:0]   render_y,
	input  wire         render_en,
	input  wire         h320,
	input  wire         write_bank,

	input  wire [8:0]   read_x,
	input  wire         read_bank,
	output wire [7:0]   pix_index,
	output wire         pix_opaque,
	output wire [2:0]   pix_priority,
	output wire         pix_tie,
	output wire         pix_add,

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
	// ---- the register block -------------------------------------------------

	wire [15:0] mode      = regs[15:0];      // $180
	wire [15:0] tile_mode = regs[31:16];     // $182
	wire [15:0] scroll_xh = regs[47:32];     // $184
	wire [15:0] scroll_xl = regs[63:48];     // $186
	wire [15:0] scroll_yh = regs[79:64];     // $188
	wire [15:0] scroll_yl = regs[95:80];     // $18A
	wire [15:0] coef_a    = regs[111:96];    // $18C
	wire [15:0] coef_b    = regs[127:112];   // $18E
	wire [15:0] coef_c    = regs[143:128];   // $190
	wire [15:0] coef_d    = regs[159:144];   // $192
	wire [15:0] map_base  = regs[175:160];   // $194
	wire [15:0] bank_reg  = regs[191:176];   // $196
	wire [15:0] tab0_base = regs[207:192];   // $198
	wire [15:0] tabx_base = regs[223:208];   // $19A
	wire [15:0] tab3_base = regs[239:224];   // $19C, fourth table
	wire [15:0] taby_base = regs[255:240];   // $19E

	reg [2:0] xs_log, ys_log;
	always @(*) begin
		case (mode[11:8])
			4'h2:    begin xs_log = 3'd4; ys_log = 3'd4; end
			4'h4:    begin xs_log = 3'd5; ys_log = 3'd5; end
			4'h6:    begin xs_log = 3'd6; ys_log = 3'd5; end
			4'hA:    begin xs_log = 3'd7; ys_log = 3'd5; end
			4'hC:    begin xs_log = 3'd6; ys_log = 3'd6; end
			default: begin xs_log = 3'd5; ys_log = 3'd5; end
		endcase
	end

	wire [15:0] x_limit = 16'd1 << ({1'b0, xs_log} + 4'd3);
	wire [15:0] y_limit = 16'd1 << ({1'b0, ys_log} + 4'd3);

	// Region 4 is the boot logo's 1 bpp layout, which region 0 selects.
	reg [2:0] region;
	always @(*) begin
		case (mode[1:0])
			2'd0:    region = 3'd4;
			2'd1:    region = 3'd2;
			2'd2:    region = 3'd1;
			default: region = 3'd0;
		endcase
	end

	wire bitmap  = (region == 3'd0) & (pixel_mode[4:3] == 2'b01);
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

	wire [9:0] mosaic_xy = mosaic_sizes(mode[4:2]);
	wire [4:0] mosaic_x = mosaic_xy[9:5];
	wire [4:0] mosaic_y = mosaic_xy[4:0];
	wire wrap = mode[5];

	// Only tile control selects the line tables; Journey's water ripples
	// through them with mode bit 9 set.
	wire use_lines = tile_mode[14];
	wire use_y_lines = use_lines & tile_mode[15];
	wire x_per_line = tile_mode[8] | bitmap | (mode[1:0] == 2'd0);

	assign pix_priority = mode[15:13];

	wire [9:0] width = h320 ? 10'd320 : 10'd256;

	// ---- the line buffer ----------------------------------------------------

	reg  [8:0] lb_waddr;
	reg        lb_wren;
	reg  [9:0] lb_wdata;
	wire [9:0] lb_q;
	reg        build_bank;
	reg  [9:0] filled[0:1];
	reg  [8:0] read_x_q;
	reg        read_bank_q;
	wire       lb_commit = lb_wren && !reset && !prep && (build_bank == write_bank);

	cache_ram_dp #(.ADDR_WIDTH(10), .DATA_WIDTH(10)) linebuf
	(
		.clk_i     (clk_sys),
		.addr_a_i  ({build_bank, lb_waddr}),
		.wren_a_i  (lb_commit),
		.wdata_a_i (lb_wdata),
		/* verilator lint_off PINCONNECTEMPTY */
		.q_a_o     (),
		/* verilator lint_on PINCONNECTEMPTY */
		.addr_b_i  ({read_bank, read_x}),
		.wren_b_i  (1'b0),
		.wdata_b_i (10'd0),
		.q_b_o     (lb_q)
	);

	// Match the RAM's registered read address; hides unwritten old pixels.
	always @(posedge clk_sys) begin
		read_x_q <= read_x;
		read_bank_q <= read_bank;
	end
	wire read_valid = {1'b0, read_x_q} < filled[read_bank_q];
	assign pix_index  = read_valid ? lb_q[7:0] : 8'd0;
	assign pix_opaque = read_valid && lb_q[8];
	assign pix_tie    = read_valid && lb_q[9];
	assign pix_add    = mode[6];

	// ---- the vertical accumulators ------------------------------------------

	// y*B and y*D, quantised to the mosaic block origin.
	reg signed [31:0] yacc_b, yacc_d, yblk_b, yblk_d;

	wire signed [31:0] b_ext = {{16{coef_b[15]}}, coef_b};
	wire signed [31:0] d_ext = {{16{coef_d[15]}}, coef_d};

	wire first_line = (render_y == 9'd0);
	// Blocks start at multiples of the size, as in the tilemaps.
	wire [3:0] mosaic_x_mask = mosaic_x[3:0] - 4'd1;
	wire [3:0] mosaic_y_mask = mosaic_y[3:0] - 4'd1;
	wire mq_wrap    = ((render_y[3:0] & mosaic_y_mask) == 4'd0);

	wire signed [31:0] yacc_b_next = first_line ? 32'sd0 : yacc_b + b_ext;
	wire signed [31:0] yacc_d_next = first_line ? 32'sd0 : yacc_d + d_ext;
	wire signed [31:0] yblk_b_next = first_line ? 32'sd0 : (mq_wrap ? yacc_b_next : yblk_b);
	wire signed [31:0] yblk_d_next = first_line ? 32'sd0 : (mq_wrap ? yacc_d_next : yblk_d);

	// ---- per-line and per-pixel state ---------------------------------------

	reg signed [31:0] accx, accy, blkx, blky;
	reg signed [15:0] line_a;
	reg        [9:0]  px;
	wire       [9:0]  px_next = px + 10'd1;
	reg               blank_line;

	reg signed [31:0] scroll_x_line, scroll_y_line;
	// With line tables on, global X is held from the start of the frame
	// (Sonic Dragon rewrites it mid-frame). Without tables it is read every
	// row (War of Light Saga writes it after line 0).
	reg signed [31:0] frame_x;
	reg frame_x_valid;

	reg        [15:0] dx_hi, dy_hi;
	reg        [15:0] map_key,  map_val;
	reg               map_hit;
	reg        [15:0] data_key, data_val;
	reg               data_hit;

	localparam [3:0] S_T0 = 4'd0, S_TX0 = 4'd1, S_TX1 = 4'd2, S_T3 = 4'd3,
	                 S_TY0 = 4'd4, S_TY1 = 4'd5, S_START = 4'd6, S_PIX = 4'd7,
	                 S_MAP = 4'd8, S_DATA = 4'd9, S_DONE = 4'd10, S_DRAIN = 4'd11;

	reg [3:0] state;
	/* verilator lint_off PROCASSINIT */
	reg pending = 1'b0;
	/* verilator lint_on PROCASSINIT */

	// ---- the source coordinate ----------------------------------------------

	wire signed [23:0] src_x = blkx[31:8];
	wire signed [23:0] src_y = blky[31:8];

	wire outside = ~wrap & ((src_x < 24'sd0) | (src_x >= $signed({8'd0, x_limit}))
	                      | (src_y < 24'sd0) | (src_y >= $signed({8'd0, y_limit})));

	wire [15:0] mx = src_x[15:0] & (x_limit - 16'd1);
	wire [15:0] my = src_y[15:0] & (y_limit - 16'd1);

	// ---- the map entry ------------------------------------------------------

	wire [12:0] map_index = (my[15:3] << xs_log) | mx[15:3];
	wire [15:0] map_addr  = {map_base[14:0], 1'b0} + {3'd0, map_index};
	wire        need_map  = (region != 3'd4) & ~bitmap;

	// A bank is 16 KiB at any depth: 256, 512 or 1024 tiles.
	wire [12:0] tile_bank = (region == 3'd0) ? {1'b0, bank_reg[15:12], 8'd0}
	                      : (region == 3'd1) ? {bank_reg[15:12], 9'd0}
	                      :                    {bank_reg[14:12], 10'd0};
	// A map reply launches its data fetch on the same edge.
	wire [15:0] map_entry = ((state == S_MAP) && vram_ack) ? vram_q : map_val;
	wire [12:0] tile_no = {3'd0, map_entry[9:0]} + tile_bank;
	wire [2:0]  tile_px = mx[2:0] ^ {3{map_entry[11]}};
	wire [2:0]  tile_py = my[2:0] ^ {3{map_entry[10]}};
	wire [3:0]  tile_pal = map_entry[15:12] | {tile_mode[9], 3'b000};

	// Byte within the tile row: one, two or four pixels per byte.
	reg [2:0] byte_sel;
	always @(*) begin
		case (region)
			3'd0:    byte_sel = tile_px;
			3'd1:    byte_sel = {1'b0, tile_px[2:1]};
			default: byte_sel = {2'b00, tile_px[2]};
		endcase
	end

	wire [19:0] tile_byte = ({7'd0, tile_no} << (4'd6 - {1'b0, region}))
	                      + ({17'd0, tile_py} << (3'd3 - region))
	                      + {17'd0, byte_sel};

	// Boot logo: fixed tile layout, low seven address bits rotated, map
	// thirty-two tiles wide whatever the size field says.
	wire [12:0] lcount = {my[10:8], my[7:3], 5'd0} + mx[15:3];
	reg  [12:0] logo_tile;
	always @(*) begin
		logo_tile = 13'h880 + {9'd0, lcount[2:0], 1'b0};
		if (lcount[5]) logo_tile = logo_tile ^ 13'd1;
		logo_tile = logo_tile | {7'd0, lcount[7:6], 4'd0};
	end
	wire [19:0] logo_byte_raw = {4'd0, logo_tile, 3'd0} + {17'd0, my[2:0]};
	wire [19:0] logo_byte = {logo_byte_raw[19:7], logo_byte_raw[3:0],
	                         logo_byte_raw[6:4]};

	// Linear bitmap: one byte a pixel.
	wire [19:0] bitmap_byte = {2'd0, bank_reg, 2'd0}
	                        + ({4'd0, my} << ({1'b0, xs_log} + 4'd3))
	                        + {4'd0, mx};

	wire [19:0] data_byte = (region == 3'd4) ? logo_byte
	                      : bitmap           ? bitmap_byte
	                                         : tile_byte;
	wire [15:0] data_addr = data_byte[16:1];

	// ---- the pixel ----------------------------------------------------------

	wire [7:0] data_b = data_byte[0] ? data_val[7:0] : data_val[15:8];

	reg [7:0] raw_pixel;
	always @(*) begin
		case (region)
			3'd4:    raw_pixel = {7'd0, data_b[3'd7 - mx[2:0]]};
			3'd0:    raw_pixel = data_b;
			3'd1:    raw_pixel = {4'd0, tile_px[0] ? data_b[3:0] : data_b[7:4]};
			// High pair first, as in the tilemaps; Gambling Lord's logo wipe
			// combs the other way round otherwise.
			default: raw_pixel = {6'd0, data_b[{~tile_px[1:0], 1'b0} +: 2]};
		endcase
	end

	// Boot logo and eight-bit forms have no palette bank; the index is the pixel.
	wire [7:0] pixel_value = (region == 3'd0) | (region == 3'd4)
	                       ? raw_pixel : {tile_pal, raw_pixel[3:0]};
	reg        pixel_op;
	always @(*) begin
		case (region)
			3'd4:    pixel_op = raw_pixel[0];
			3'd0:    pixel_op = (raw_pixel != 8'd0);
			3'd1:    pixel_op = (raw_pixel[3:0] != 4'd0);
			default: pixel_op = (raw_pixel[1:0] != 2'd0);
		endcase
	end

	// The bitmap path has no map entry, so no palette bank.
	wire [7:0] out_value = bitmap ? raw_pixel : pixel_value;
	wire pixel_tie = need_map && tile_mode[9] && !map_entry[15];

	// ---- the per-line table addresses ---------------------------------------

	wire [15:0] t0_addr = {tab0_base[14:0], 1'b0} + {7'd0, render_y};
	wire [15:0] tx_addr = {tabx_base[14:0], 1'b0}
	                    + (x_per_line ? {6'd0, render_y, 1'b0} : 16'd0);
	wire [15:0] t3_addr = {tab3_base[14:0], 1'b0} + {7'd0, render_y};
	wire [15:0] ty_addr = {taby_base[14:0], 1'b0} + {6'd0, render_y, 1'b0};

	// Origins have 24 signed bits, including eight fractional bits.
	wire signed [31:0] scroll_x_reg = {{8{scroll_xh[7]}}, scroll_xh[7:0], scroll_xl};
	wire signed [31:0] scroll_y_reg = {{8{scroll_yh[7]}}, scroll_yh[7:0], scroll_yl};
	wire signed [15:0] a_ext = coef_a;
	wire signed [31:0] row_y = use_y_lines ? scroll_y_line : scroll_y_line + yblk_d;

	wire [63:0] ss_yacc, ss_yblk;
	wire [63:0] ss_q0, ss_q1;
	ss_reg #(.ADDR(SSW_6618_ROZ)) saved_yacc
	(
		.clk_i(clk_sys), .bus_din_i(ss_din), .bus_addr_i(ss_addr),
		.bus_wren_i(ss_wren), .bus_rst_i(ss_rst), .bus_dout_o(ss_q0),
		.din_i({yacc_d, yacc_b}), .dout_o(ss_yacc)
	);
	ss_reg #(.ADDR(SSW_6618_ROZ + 1)) saved_yblk
	(
		.clk_i(clk_sys), .bus_din_i(ss_din), .bus_addr_i(ss_addr),
		.bus_wren_i(ss_wren), .bus_rst_i(ss_rst), .bus_dout_o(ss_q1),
		.din_i({yblk_d, yblk_b}), .dout_o(ss_yblk)
	);
	assign ss_dout = ss_q0 | ss_q1;

	// ---- the machine --------------------------------------------------------

	always @(posedge clk_sys) begin
		lb_wren <= 1'b0;
		if (vram_gnt) vram_req <= 1'b0;
		if (vram_gnt) pending <= 1'b1;
		else if (vram_ack) pending <= 1'b0;
		if (lb_commit) filled[build_bank] <= {1'b0, lb_waddr} + 10'd1;

		if (reset) begin
			frame_x_valid <= 1'b0;
			frame_x <= scroll_x_reg;
			state    <= S_DONE;
			vram_req <= 1'b0;
			yacc_b   <= ss_yacc[31:0];
			yacc_d   <= ss_yacc[63:32];
			yblk_b   <= ss_yblk[31:0];
			yblk_d   <= ss_yblk[63:32];
			build_bank <= write_bank;
			filled[0] <= 10'd0;
			filled[1] <= 10'd0;
		end else if (prep) begin
			px         <= 10'd0;
			map_hit    <= 1'b0;
			data_hit   <= 1'b0;
			vram_req   <= 1'b0;
			blank_line <= ~enable | ~render_en;
			build_bank <= write_bank;
			filled[write_bank] <= 10'd0;

			if (render_en) begin
				yacc_b <= yacc_b_next;
				yacc_d <= yacc_d_next;
				yblk_b <= yblk_b_next;
				yblk_d <= yblk_d_next;
			end

			line_a        <= a_ext;
			if (first_line || !frame_x_valid) begin
				frame_x <= scroll_x_reg;
				frame_x_valid <= 1'b1;
			end
			scroll_x_line <= (first_line || !frame_x_valid || !use_lines) ? scroll_x_reg : frame_x;
			scroll_y_line <= scroll_y_reg;

			state <= S_DRAIN;
		end else if (build_bank != write_bank) begin
			// The display takes this bank before the next prep.
			vram_req <= 1'b0;
			state <= S_DONE;
		end else begin
			case (state)
			S_DRAIN:
				if (!pending || vram_ack) begin
					// An abandoned read must return before ACK can be reused.
					if (render_en & enable & use_lines) begin
						vram_addr <= tile_mode[6] ? t0_addr : tx_addr;
						vram_req <= 1'b1;
						state <= tile_mode[6] ? S_T0 : S_TX0;
					end else begin
						state <= S_START;
					end
				end

			S_T0:
				if (vram_ack) begin
					// A zero in the first table turns the line off.
					if (vram_q == 16'd0) begin
						blank_line <= 1'b1;
						state      <= S_START;
					end else begin
						line_a    <= $signed(vram_q);
						vram_addr <= tx_addr;
						vram_req  <= 1'b1;
						state     <= S_TX0;
					end
				end

			S_TX0:
				if (vram_ack) begin
					if (x_per_line) begin
						dx_hi     <= vram_q;
						vram_addr <= tx_addr + 16'd1;
						state     <= S_TX1;
					end else begin
						// Integer pixels over the origin's eight fractional bits.
						scroll_x_line <= scroll_x_line + $signed({{8{vram_q[15]}}, vram_q, 8'd0});
						vram_addr <= t3_addr;
						state     <= S_T3;
					end
					vram_req  <= 1'b1;
				end

			S_TX1:
				if (vram_ack) begin
					scroll_x_line <= scroll_x_line + {dx_hi, vram_q};
					vram_addr     <= t3_addr;
					vram_req      <= 1'b1;
					state         <= S_T3;
				end

			S_T3:
				if (vram_ack) begin
					// Meaning unknown; fetched, then discarded.
					if (use_y_lines) begin
						vram_addr <= ty_addr;
						vram_req  <= 1'b1;
						state     <= S_TY0;
					end else begin
						state <= S_START;
					end
				end

			S_TY0:
				if (vram_ack) begin
					dy_hi     <= vram_q;
					vram_addr <= ty_addr + 16'd1;
					vram_req  <= 1'b1;
					state     <= S_TY1;
				end

			S_TY1:
				if (vram_ack) begin
					scroll_y_line <= scroll_y_line + {dy_hi, vram_q};
					state         <= S_START;
				end

			S_START: begin
				accx  <= scroll_x_line + yblk_b;
				accy  <= row_y;
				blkx  <= scroll_x_line + yblk_b;
				blky  <= row_y;
				state <= S_PIX;
			end

			S_PIX:
				if (px >= width) begin
					state <= S_DONE;
				end else if (!blank_line && !outside && need_map
				             && (!map_hit || (map_key != map_addr))) begin
					vram_addr <= map_addr;
					vram_req  <= 1'b1;
					state     <= S_MAP;
				end else if (!blank_line && !outside
				             && (!data_hit || (data_key != data_addr))) begin
					vram_addr <= data_addr;
					vram_req  <= 1'b1;
					state     <= S_DATA;
				end else begin
					lb_waddr <= px[8:0];
					lb_wdata <= (blank_line | outside) ? 10'd0
					                                   : {pixel_tie, pixel_op, out_value};
					lb_wren  <= 1'b1;
					px       <= px_next;
					accx     <= accx + {{16{line_a[15]}}, line_a};
					accy     <= accy + {{16{coef_c[15]}}, coef_c};
					if ((px_next[3:0] & mosaic_x_mask) == 4'd0) begin
						blkx   <= accx + {{16{line_a[15]}}, line_a};
						blky   <= accy + {{16{coef_c[15]}}, coef_c};
					end
				end

			S_MAP:
				if (vram_ack) begin
					map_val <= vram_q;
					map_key <= map_addr;
					map_hit <= 1'b1;
					if (!data_hit || (data_key != data_addr)) begin
						vram_addr <= data_addr;
						vram_req <= 1'b1;
						state <= S_DATA;
					end else begin
						state <= S_PIX;
					end
				end

			S_DATA:
				if (vram_ack) begin
					data_val <= vram_q;
					data_key <= data_addr;
					data_hit <= 1'b1;
					state    <= S_PIX;
				end

			default: ;   // S_DONE
			endcase
		end
	end

	`ifdef VERILATOR
	/* verilator lint_off UNUSEDSIGNAL */
	wire unused = &{1'b0, mode[12], mode[7], tile_mode[15], tile_mode[13:10], tile_mode[7], tile_mode[5:0],
	                bank_reg[11:0], map_base[15], tab0_base[15], tabx_base[15],
	                tab3_base[15], taby_base[15], pixel_mode[15:5],
	                scroll_xh[15:8], scroll_yh[15:8],
	                pixel_mode[2:0], lcount[12:8],
	                data_byte[19:17], tile_byte[19:17], logo_byte_raw[19:7],
	                raw_pixel[7:4], src_x[23:16], src_y[23:16]};
	/* verilator lint_on UNUSEDSIGNAL */
	`endif
endmodule
