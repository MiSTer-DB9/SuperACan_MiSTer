// Copyright (c) 2026 Jamie Blanks

// One of the UM6618's four tilemap layers, register blocks at $F00100,
// $F00120, $F00140 and $F00160.
//
// Renders a line ahead into a double-banked line buffer: while line L-1 is
// on screen this builds line L into bank vpos[0]. One VRAM slot per dot is
// enough since a 320-pixel line needs at most 200 fetches.
//
// The pixel loop walks the destination because mosaic advances the source in
// block-size steps. One cell of tile data is held in a register and
// refetched when the source crosses into the next cell.
//
// Layers 2 and 3 have their pixel format fixed to two bits by the wiring.

module um6618_tilemap (
	input  wire         clk_sys,
	input  wire         reset,

	// Eight registers: flags, tile mode, scroll X, scroll Y, map base, mode,
	// line-scroll table, line-select table.
	input  wire [127:0] regs,
	input  wire [1:0]   region,      // 0 = 8bpp, 1 = 4bpp, 2 = 2bpp
	input  wire         enable,

	// Line to build, and the pulse that starts it.
	input  wire         prep,
	input  wire [8:0]   render_y,
	input  wire         render_en,
	input  wire         h320,
	input  wire         write_bank,

	// Mixer read side; the buffer answers one clock after read_x.
	input  wire [8:0]   read_x,
	input  wire         read_bank,
	output wire [7:0]   pix_index,
	output wire         pix_opaque,
	output wire [2:0]   pix_priority,
	output wire         pix_tie,
	output wire         pix_add,

	// VRAM client port.
	output reg          vram_req,
	output reg  [15:0]  vram_addr,
	input  wire         vram_gnt,
	input  wire         vram_ack,
	input  wire [15:0]  vram_q
);
	// ---- the register block -------------------------------------------------

	wire [15:0] flags     = regs[15:0];
	wire [15:0] tile_mode = regs[31:16];
	wire [15:0] scroll_x  = regs[47:32];
	wire [15:0] scroll_y  = regs[63:48];
	wire [15:0] map_base  = regs[79:64];
	wire [15:0] mode      = regs[95:80];
	wire [15:0] lsx_base  = regs[111:96];
	wire [15:0] lsel_base = regs[127:112];

	// Bits 11-8 select the map size. All sizes are powers of two, so the map
	// index is a concatenation.
	reg [2:0] xs_log, ys_log;      // log2 of the map size in tiles
	always @(*) begin
		case (flags[11:8])
			4'h2:    begin xs_log = 3'd4; ys_log = 3'd4; end   // 16 x 16
			4'h4:    begin xs_log = 3'd5; ys_log = 3'd5; end   // 32 x 32
			4'h6:    begin xs_log = 3'd6; ys_log = 3'd5; end   // 64 x 32
			4'hA:    begin xs_log = 3'd7; ys_log = 3'd5; end   // 128 x 32
			4'hC:    begin xs_log = 3'd6; ys_log = 3'd6; end   // 64 x 64
			default: begin xs_log = 3'd5; ys_log = 3'd5; end
		endcase
	end

	wire [15:0] x_mask = (16'd1 << ({1'b0, xs_log} + 4'd3)) - 16'd1;
	wire [15:0] y_mask = (16'd1 << ({1'b0, ys_log} + 4'd3)) - 16'd1;

	// X scroll is eleven bits, sign in bit 10: Magic Cue Pool's title runs
	// this register through $070C with wrap off and the console places it at
	// -244. Y keeps twelve bits.
	//
	// A whole-layer flip is an exclusive-or on the scroll value.
	wire signed [15:0] scroll_x_s = $signed({{5{scroll_x[10]}}, scroll_x[10:0]})
	                              ^ (flags[1] ? $signed(x_mask) : 16'sd0);
	wire signed [15:0] scroll_y_s = $signed({{4{scroll_y[11]}}, scroll_y[11:0]})
	                              ^ (flags[0] ? $signed(y_mask) : 16'sd0);

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

	wire [9:0] mosaic_xy = mosaic_sizes(flags[4:2]);
	wire [4:0] mosaic_x = mosaic_xy[9:5];
	wire [4:0] mosaic_y = mosaic_xy[4:0];
	wire       wrap    = flags[5];
	wire       pal_hi  = tile_mode[9];
	wire       use_sel = tile_mode[11];
	wire       use_lsx = tile_mode[14];

	assign pix_priority = flags[15:13];

	wire [9:0] width = h320 ? 10'd320 : 10'd256;

	// ---- the line buffer ----------------------------------------------------

	reg  [8:0]  lb_waddr;
	reg         lb_wren;
	reg  [9:0]  lb_wdata;
	wire [9:0]  lb_q;

	cache_ram_dp #(.ADDR_WIDTH(10), .DATA_WIDTH(10)) linebuf
	(
		.clk_i     (clk_sys),
		.addr_a_i  ({write_bank, lb_waddr}),
		.wren_a_i  (lb_wren),
		.wdata_a_i (lb_wdata),
		/* verilator lint_off PINCONNECTEMPTY */
		.q_a_o     (),
		/* verilator lint_on PINCONNECTEMPTY */
		.addr_b_i  ({read_bank, read_x}),
		.wren_b_i  (1'b0),
		.wdata_b_i (10'd0),
		.q_b_o     (lb_q)
	);

	assign pix_index  = lb_q[7:0];
	assign pix_opaque = lb_q[8];
	assign pix_tie    = lb_q[9];
	assign pix_add    = flags[6];
	reg row_tie;

	// ---- the mosaic origins -------------------------------------------------

	// A block starts where the screen position is a multiple of the block
	// size, so a mid-frame size change takes hold at the next boundary of the
	// new size. Sonic Dragon changes the size around line 40 of a fade.
	reg [8:0] mq_val;

	wire [3:0] mosaic_x_mask = mosaic_x[3:0] - 4'd1;
	wire [3:0] mosaic_y_mask = mosaic_y[3:0] - 4'd1;
	wire       mq_wrap     = ((render_y[3:0] & mosaic_y_mask) == 4'd0);
	wire [8:0] mq_val_next = (render_y == 9'd0) ? 9'd0 : (mq_wrap ? render_y : mq_val);

	// ---- per-line state -----------------------------------------------------

	reg signed [15:0] line_scroll;
	reg        [15:0] real_y;
	reg               blank_line;
	reg        [9:0]  px;
	wire       [9:0]  px_next = px + 10'd1;
	reg        [9:0]  mx_val;

	reg        [12:0] cell_id;
	reg               cell_valid;
	reg        [63:0] row_data;
	reg        [2:0]  row_px_flip;
	reg        [3:0]  row_pal;
	reg        [1:0]  word_cnt;
	reg        [15:0] data_base;

	localparam [2:0] S_YSEL = 3'd0, S_XSCR = 3'd1, S_PIX = 3'd2,
	                 S_MAP  = 3'd3, S_DATA = 3'd4, S_DONE = 3'd5;

	reg [2:0] state;

	// ---- the source coordinate of the pixel being built ----------------------

	wire signed [15:0] real_x_full = $signed({6'd0, mx_val}) + line_scroll;
	wire        [15:0] real_x      = real_x_full & x_mask;
	wire signed [15:0] px_pos      = $signed({6'd0, px}) + line_scroll;
	wire               px_skip     = ~wrap & ((px_pos < 16'sd0) | (px_pos > $signed(x_mask)));

	wire [12:0] want_cell = real_x[15:3];
	wire [12:0] map_index = (real_y[15:3] << xs_log) | want_cell;

	// ---- the pixel the cell register holds ---------------------------------

	wire [2:0] sx = real_x[2:0] ^ row_px_flip;
	wire [7:0] two_bpp_byte = sx[2] ? row_data[55:48] : row_data[63:56];

	reg [7:0] cell_pixel;
	always @(*) begin
		case (region)
			2'd0:    cell_pixel = row_data[(7'd56 - {sx, 3'd0}) +: 8];
			2'd1:    cell_pixel = {4'd0, row_data[(7'd60 - {sx, 2'd0}) +: 4]};
			default: cell_pixel = {6'd0, two_bpp_byte[{~sx[1:0], 1'b0} +: 2]};
		endcase
	end

	wire [7:0] pixel_value = (region == 2'd0) ? cell_pixel
	                                          : {row_pal, cell_pixel[3:0]};
	wire       pixel_op    = (region == 2'd0) ? (cell_pixel      != 8'd0)
	                       : (region == 2'd1) ? (cell_pixel[3:0] != 4'd0)
	                                          : (cell_pixel[1:0] != 2'd0);

	// ---- the fetch addresses ------------------------------------------------

	wire [15:0] map_addr  = {map_base[14:0], 1'b0}  + {3'd0, map_index};
	wire [15:0] ysel_addr = {lsel_base[14:0], 1'b0} + {7'd0, render_y};
	// Tile-mode bit 8: line-scroll table has one entry per line. With it
	// clear the first entry serves every line, which is how Sonic Dragon
	// shakes a whole layer.
	wire [15:0] xscr_addr = {lsx_base[14:0], 1'b0}
	                      + (tile_mode[8] ? {7'd0, render_y} : 16'd0);

	// Tile and row strides follow the colour depth: 64, 32 or 16 bytes a
	// tile and 8, 4 or 2 bytes a row.
	wire [12:0] tile_no  = {3'd0, vram_q[9:0]}
	                     + (13'({1'b0, mode[14:12]}) << (4'd8 + {2'd0, region}));
	wire [2:0]  tile_py  = real_y[2:0] ^ {3{vram_q[10]}};
	wire [19:0] row_byte = (20'({7'd0, tile_no}) << (4'd6 - {2'd0, region}))
	                     + (20'({17'd0, tile_py}) << (3'd3 - region));

	// Line-select and wrap.
	wire [15:0] y_from_line = ($signed(vram_q) + scroll_y_s) & y_mask;
	wire [15:0] y_from_mos  = ($signed({7'd0, mq_val_next}) + scroll_y_s) & y_mask;
	wire signed [15:0] y_check = $signed({7'd0, render_y}) + scroll_y_s;

	// ---- the machine ---------------------------------------------------------

	always @(posedge clk_sys) begin
		lb_wren <= 1'b0;
		if (vram_gnt) vram_req <= 1'b0;

		if (reset) begin
			state    <= S_DONE;
			vram_req <= 1'b0;
			mq_val   <= 9'd0;
		end else if (prep) begin
			// A new line aborts whatever the last one was still doing.
			px          <= 10'd0;
			mx_val      <= 10'd0;
			cell_valid  <= 1'b0;
			line_scroll <= scroll_x_s;
			real_y      <= y_from_mos;
			blank_line  <= ~enable | ~render_en
			             | (~wrap & ((y_check < 16'sd0) | (y_check > $signed(y_mask))));
			vram_req    <= 1'b0;

			if (render_en) mq_val <= mq_val_next;

			if (render_en & enable & use_sel) begin
				vram_addr <= ysel_addr;
				vram_req  <= 1'b1;
				state     <= S_YSEL;
			end else if (render_en & enable & use_lsx) begin
				vram_addr <= xscr_addr;
				vram_req  <= 1'b1;
				state     <= S_XSCR;
			end else begin
				state <= S_PIX;
			end
		end else begin
			case (state)
			S_YSEL:
				if (vram_ack) begin
					real_y <= y_from_line;
					if (use_lsx) begin
						vram_addr <= xscr_addr;
						vram_req  <= 1'b1;
						state     <= S_XSCR;
					end else begin
						state <= S_PIX;
					end
				end

			S_XSCR:
				if (vram_ack) begin
					line_scroll <= line_scroll + $signed(vram_q);
					state       <= S_PIX;
				end

			S_PIX:
					if (px >= width) begin
					state <= S_DONE;
				end else if (!blank_line && !px_skip
				             && (!cell_valid || (cell_id != want_cell))) begin
					vram_addr <= map_addr;
					vram_req  <= 1'b1;
					state     <= S_MAP;
				end else begin
					lb_waddr <= px[8:0];
					lb_wdata <= (blank_line | px_skip) ? 10'd0
					                                   : {row_tie, pixel_op, pixel_value};
					lb_wren  <= 1'b1;
					px       <= px + 10'd1;
					if ((px_next[3:0] & mosaic_x_mask) == 4'd0) mx_val <= px_next;
				end

			S_MAP:
				if (vram_ack) begin
					row_px_flip <= {3{vram_q[11]}};
					row_pal     <= vram_q[15:12] | {pal_hi, 3'b000};
					row_tie     <= pal_hi && !vram_q[15];
					data_base   <= row_byte[16:1];
					cell_id     <= want_cell;
					cell_valid  <= 1'b1;
					word_cnt    <= 2'd0;
					vram_addr   <= row_byte[16:1];
					vram_req    <= 1'b1;
					state       <= S_DATA;
				end

			S_DATA:
				if (vram_ack) begin
					row_data[(6'd48 - {word_cnt, 4'd0}) +: 16] <= vram_q;
					if (word_cnt == (2'd3 >> region)) begin
						state <= S_PIX;
					end else begin
						word_cnt  <= word_cnt + 2'd1;
						vram_addr <= data_base + {14'd0, word_cnt} + 16'd1;
						vram_req  <= 1'b1;
					end
				end

			default: ;   // S_DONE: wait for the next prep
			endcase
		end
	end

	`ifdef VERILATOR
	/* verilator lint_off UNUSEDSIGNAL */
	wire unused = &{1'b0, flags[12], flags[7], tile_mode[15:12],
	                tile_mode[10], tile_mode[8:0], mode[15], mode[11:0],
	                map_base[15], lsx_base[15], lsel_base[15],
	                real_x_full[15:14], row_byte[19:17], row_byte[0],
	                cell_pixel[7:4]};
	/* verilator lint_on UNUSEDSIGNAL */
	`endif
endmodule
