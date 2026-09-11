// Copyright (c) 2026 Jamie Blanks

// The two window clippers, register blocks $F001D0 and $F001D8.
//
// A window is a horizontal span filled with one palette index. Its edges
// come from a two-word table in VRAM, one pair per frame or per line, offset
// by the window's scroll. Bit 11 inverts the test.
//
// The edges live in double-buffered registers: the pair fetched during line
// L-1 clips line L.

module um6618_window (
	input  wire         clk_sys,
	input  wire         reset,

	input  wire [127:0] regs,        // $1D0-$1DE, eight words
	input  wire [1:0]   enable,      // video flags bit1 (window 0), bit0 (1)

	input  wire         prep,
	input  wire [8:0]   render_y,
	input  wire         render_en,

	input  wire [8:0]   read_x,
	output wire [15:0]  pen,         // two bytes, window 0 low
	output wire [1:0]   in_span,
	output wire [5:0]   priority_o,  // two three-bit fields
	output wire [1:0]   active,

	output reg          vram_req,
	output reg  [15:0]  vram_addr,
	input  wire         vram_gnt,
	input  wire         vram_ack,
	input  wire [15:0]  vram_q
);
	wire [15:0] ctrl0  = regs[15:0];      // $1D0
	wire [15:0] base0  = regs[31:16];     // $1D2
	wire [15:0] scrx0  = regs[47:32];     // $1D4
	wire [15:0] ctrl1  = regs[79:64];     // $1D8
	wire [15:0] base1  = regs[95:80];     // $1DA
	wire [15:0] scrx1  = regs[111:96];    // $1DC

	// Window 0 is enabled by video flags bit 1 and window 1 by bit 0. Window
	// 1's control register doubles as a presence bit.
	assign active     = {enable[0] & (ctrl1 != 16'd0), enable[1]};
	assign pen        = {ctrl1[7:0], ctrl0[7:0]};
	assign priority_o = {1'b0, ctrl1[14:13], 1'b0, ctrl0[14:13]};

	function automatic [15:0] scroll_of(input [15:0] reg_v);
		scroll_of = {{6{reg_v[9]}}, reg_v[9:0]};
	endfunction

	wire signed [15:0] sx0 = scroll_of(scrx0);
	wire signed [15:0] sx1 = scroll_of(scrx1);

	// Control bit 8: one pair per line instead of per frame.
	wire [15:0] addr0 = {base0[14:0], 1'b0}
	                  + (ctrl0[8] ? {6'd0, render_y, 1'b0} : 16'd0);
	wire [15:0] addr1 = {base1[14:0], 1'b0}
	                  + (ctrl1[8] ? {6'd0, render_y, 1'b0} : 16'd0);

	reg signed [15:0] next_min0, next_max0, next_min1, next_max1;
	reg signed [15:0] cur_min0, cur_max0, cur_min1, cur_max1;

	localparam [2:0] W_MIN0 = 3'd0, W_MAX0 = 3'd1, W_MIN1 = 3'd2,
	                 W_MAX1 = 3'd3, W_IDLE = 3'd4;
	reg [2:0] state;

	wire signed [15:0] x = {7'd0, read_x};

	wire in0 = ((x >= cur_min0) & (x < cur_max0)) ^ ctrl0[11];
	wire in1 = ((x >= cur_min1) & (x < cur_max1)) ^ ctrl1[11];

	assign in_span = {in1, in0};

	always @(posedge clk_sys) begin
		if (vram_gnt) vram_req <= 1'b0;

		if (reset) begin
			state    <= W_IDLE;
			vram_req <= 1'b0;
			// Empty span until a table has been read.
			next_min0 <= 16'sd0;
			next_max0 <= 16'sd0;
			next_min1 <= 16'sd0;
			next_max1 <= 16'sd0;
			cur_min0  <= 16'sd0;
			cur_max0  <= 16'sd0;
			cur_min1  <= 16'sd0;
			cur_max1  <= 16'sd0;
		end else if (prep) begin
			cur_min0 <= next_min0;
			cur_max0 <= next_max0;
			cur_min1 <= next_min1;
			cur_max1 <= next_max1;
			vram_req <= 1'b0;
			if (render_en & active[0]) begin
				vram_addr <= addr0;
				vram_req  <= 1'b1;
				state     <= W_MIN0;
			end else if (render_en & active[1]) begin
				vram_addr <= addr1;
				vram_req  <= 1'b1;
				state     <= W_MIN1;
			end else begin
				state <= W_IDLE;
			end
		end else begin
			case (state)
			W_MIN0:
				if (vram_ack) begin
					next_min0 <= $signed(vram_q) + sx0;
					vram_addr <= addr0 + 16'd1;
					vram_req  <= 1'b1;
					state     <= W_MAX0;
				end

			W_MAX0:
				if (vram_ack) begin
					next_max0 <= $signed(vram_q) + sx0;
					if (active[1]) begin
						vram_addr <= addr1;
						vram_req  <= 1'b1;
						state     <= W_MIN1;
					end else begin
						state <= W_IDLE;
					end
				end

			W_MIN1:
				if (vram_ack) begin
					next_min1 <= $signed(vram_q) + sx1;
					vram_addr <= addr1 + 16'd1;
					vram_req  <= 1'b1;
					state     <= W_MAX1;
				end

			W_MAX1:
				if (vram_ack) begin
					next_max1 <= $signed(vram_q) + sx1;
					state     <= W_IDLE;
				end

			default: ;
			endcase
		end
	end

	/* verilator lint_off UNUSEDSIGNAL */
	// $1D6 and $1DE are the windows' vertical scroll; effect unknown.
	`ifdef VERILATOR
	wire unused = &{1'b0, regs[63:48], regs[127:112],
	                ctrl0[15], ctrl0[12:9], ctrl0[10:8],
	                ctrl1[15], ctrl1[12:9], ctrl1[10:8],
	                base0[15], base1[15], scrx0[15:10], scrx1[15:10]};
	/* verilator lint_on UNUSEDSIGNAL */
	`endif
endmodule
