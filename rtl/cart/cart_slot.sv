// Copyright (c) 2026 Jamie Blanks

// The cartridge slot. One board type, PCCAR 16003-1B; whether U6 is fitted
// follows from the image size (over 2 MB needs it).
//
// The two mask ROMs share one memory bridge client. A21 selects between them,
// so at most one is waiting on SDRAM at a time.
//
// An empty slot reads as all ones, from the connector's pull-ups.

module cart_slot (
	input  wire        clk_sys,
	input  wire        reset,

	input  wire        present,
	input  wire [24:0] rom_size,     // bytes, from the loader

	// J1, the edge connector.
	input  wire [23:1] CPU_A,
	input  wire [15:0] CPU_D_i,
	output wire [15:0] CPU_D_o,
	output wire        CPU_D_oe,
	input  wire        CPU_AS_n,
	input  wire        CPU_RW,
	input  wire        CART_26_n,
	input  wire        CART_27_n,

	output wire        rom_ready,

	// SDRAM bridge client shared by the mask ROMs.
	output wire        mem_req,
	output wire [25:0] mem_addr,
	input  wire [15:0] mem_dout,
	input  wire        mem_busy,
	input  wire        mem_done,

	input  wire        ld_key_wr,
	input  wire [2:0]  ld_key_addr,
	input  wire [15:0] ld_key_data,

	input  wire        ss_own,
	input  wire [14:0] ss_addr_b,
	input  wire        ss_we,
	input  wire [7:0]  ss_din_b,
	output wire [7:0]  ss_dout_b,

	// Battery SRAM save file.
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
	wire        u5_req, u6_req;
	wire [25:0] u5_addr, u6_addr;

	// Address mask for a ROM the size of its image. A 512 KB game has no top
	// address lines, so it repeats across U5's 2 MB window, and an absent U6
	// reads as pull-ups instead of whatever else is in SDRAM.
	function automatic [20:1] amask(input [24:0] bytes);
		reg [24:0] m;
		begin
			m = bytes - 25'd1;
			m = m | (m >> 1);
			m = m | (m >> 2);
			m = m | (m >> 4);
			m = m | (m >> 8);
			m = m | (m >> 16);
			amask = m[20:1];
		end
	endfunction

	wire        big      = (rom_size[24:21] != 4'd0);        // more than 2 MB
	wire [24:0] u5_bytes = big ? 25'h200000 : rom_size;
	wire [24:0] u6_bytes = big ? (rom_size - 25'h200000) : 25'd0;

	wire [20:1] u5_amask  = amask(u5_bytes);
	wire [20:1] u6_amask  = amask(u6_bytes);
	wire        u6_fitted = (u6_bytes != 25'd0);
	wire        rom_busy = mem_busy | mem_done;
	wire        board_ready;
	wire [15:0] board_d;
	wire        board_d_oe;
	wire        board_dirty;

	// U6 only ever asks while A21 is high, and U5 only while it is low.
	assign mem_req  = u5_req | u6_req;
	assign mem_addr = u6_req ? u6_addr : u5_addr;

	cart_16003_1b board
	(
		.clk_sys     (clk_sys),
		.reset       (reset),
		.u5_amask    (u5_amask),
		.u6_amask    (u6_amask),
		.u6_fitted   (u6_fitted),
		.CPU_A       (CPU_A),
		.CPU_D_i     (CPU_D_i),
		.CPU_D_o     (board_d),
		.CPU_D_oe    (board_d_oe),
		.CPU_AS_n    (CPU_AS_n),
		.CPU_RW      (CPU_RW),
		.CART_26_n   (CART_26_n),
		.CART_27_n   (CART_27_n),
		.rom_ready   (board_ready),
		.u5_req      (u5_req),
		.u5_addr     (u5_addr),
		.u5_dout     (mem_dout),
		.u5_busy     (rom_busy),
		.u5_done     (mem_done & ~u6_pending),
		.u6_req      (u6_req),
		.u6_addr     (u6_addr),
		.u6_dout     (mem_dout),
		.u6_busy     (rom_busy),
		.u6_done     (mem_done & u6_pending),
		.ld_key_wr   (ld_key_wr),
		.ld_key_addr (ld_key_addr),
		.ld_key_data (ld_key_data),
		.ss_own      (ss_own),
		.ss_addr_b   (ss_addr_b),
		.ss_we       (ss_we),
		.ss_din_b    (ss_din_b),
		.ss_dout_b   (ss_dout_b),
		.bk_addr     (bk_addr),
		.bk_wren     (bk_wren),
		.bk_be       (bk_be),
		.bk_wdata    (bk_wdata),
		.bk_q        (bk_q),
		.bk_dirty    (board_dirty),
		.ss_din      (ss_din),
		.ss_addr     (ss_addr),
		.ss_wren     (ss_wren),
		.ss_rst      (ss_rst),
		.ss_dout     (ss_dout)
	);

	// Route the answer back to whichever ROM asked.
	reg u6_pending;
	always @(posedge clk_sys) begin
		if (reset)       u6_pending <= 1'b0;
		else if (u6_req) u6_pending <= 1'b1;
		else if (u5_req) u6_pending <= 1'b0;
	end

	// An empty slot is pull-ups.
	assign CPU_D_o  = present ? board_d : 16'hFFFF;
	assign CPU_D_oe = present & board_d_oe;
	assign rom_ready = present ? board_ready : 1'b1;
	assign bk_dirty  = present & board_dirty;

	/* verilator lint_off UNUSEDSIGNAL */
	/* verilator lint_on UNUSEDSIGNAL */
endmodule
