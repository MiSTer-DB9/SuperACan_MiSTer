// Copyright (c) 2026 Jamie Blanks

// Savestate bulk memory walk, one byte at a time.
//
//   0  work RAM      64 KiB  U1 and U2, 16 bits wide
//   1  VRAM         128 KiB  U5 and U6
//   2  sound RAM     64 KiB  U11, 8 bits wide
//   3  cartridge RAM 32 KiB  cartridge 62256
//   4  palette          512 B inside the UM70C188
//
// All block RAM on clk_sys: address out, wait one clock, take the answer.
// 16-bit regions walk in 68000 byte order, even byte = high half.
//
// The transfer engine holds rd or wr high across the eight bytes of a group
// and only moves the address, so a new access is a change of address, type
// or direction, and `ready` means rdata_o belongs to the current address.
// Runs only while the machine is parked.

module ss_bulk (
	input  wire        clk,
	input  wire        reset,

	// ---- the transfer engine ---------------------------------------------
	input  wire [24:0] addr_i,
	input  wire [2:0]  type_i,
	input  wire        rd_i,
	input  wire        wr_i,
	input  wire [7:0]  wdata_i,
	output reg  [7:0]  rdata_o,
	output wire        ready_o,

	// ---- work RAM, U1 and U2, port B --------------------------------------
	output wire        wram_own_o,
	output wire [14:0] wram_addr_o,
	output wire        wram_we_o,
	output wire [1:0]  wram_be_o,
	output wire [15:0] wram_wdata_o,
	input  wire [15:0] wram_q_i,

	// ---- VRAM, U5 and U6, port B ------------------------------------------
	output wire        vram_own_o,
	output wire [15:0] vram_addr_o,
	output wire        vram_we_o,
	output wire [1:0]  vram_be_o,
	output wire [15:0] vram_wdata_o,
	input  wire [15:0] vram_q_i,

	// ---- sound RAM, U11, port B -------------------------------------------
	output wire        sndram_own_o,
	output wire [15:0] sndram_addr_o,
	output wire        sndram_we_o,
	output wire [7:0]  sndram_wdata_o,
	input  wire [7:0]  sndram_q_i,

	// ---- cartridge SRAM, port B -------------------------------------------
	output wire        cartram_own_o,
	output wire [14:0] cartram_addr_o,
	output wire        cartram_we_o,
	output wire [7:0]  cartram_wdata_o,
	input  wire [7:0]  cartram_q_i,

	// ---- palette RAM inside the UM70C188, port B --------------------------
	output wire        pal_own_o,
	output wire [7:0]  pal_addr_o,
	output wire        pal_we_o,
	output wire [1:0]  pal_be_o,
	output wire [15:0] pal_wdata_o,
	input  wire [15:0] pal_q_i
);

	`include "ss_map.svh"

	localparam [1:0] S_IDLE  = 2'd0;
	localparam [1:0] S_READ  = 2'd1;   // block RAM answers the cycle after
	localparam [1:0] S_DONE  = 2'd2;

	reg [1:0]  st;
	reg [24:0] served_addr;
	reg [2:0]  served_type;
	reg        served_we;
	reg        served_v;

	wire req = rd_i | wr_i;
	wire served = served_v && (served_addr == addr_i) && (served_type == type_i)
	              && (served_we == wr_i);

	wire go = req && !served && (st == S_IDLE);
	assign ready_o = served;

	wire is_wram    = (type_i == SS_MEM_WRAM);
	wire is_vram    = (type_i == SS_MEM_VRAM);
	wire is_sndram  = (type_i == SS_MEM_SNDRAM);
	wire is_cartram = (type_i == SS_MEM_CARTRAM);
	wire is_pal     = (type_i == SS_MEM_PALETTE);

	// The engine leaves type_i behind after a walk, so ownership follows the
	// strobes rather than the type alone.
	wire active = rd_i | wr_i | (st != S_IDLE);
	wire write_now = wr_i && go;

	// Even byte address is the high half of its word.
	wire [1:0] be = addr_i[0] ? 2'b01 : 2'b10;

	assign wram_own_o    = is_wram && active;
	assign wram_addr_o   = addr_i[15:1];
	assign wram_we_o     = is_wram && write_now;
	assign wram_be_o     = be;
	assign wram_wdata_o  = {2{wdata_i}};

	assign vram_own_o    = is_vram && active;
	assign vram_addr_o   = addr_i[16:1];
	assign vram_we_o     = is_vram && write_now;
	assign vram_be_o     = be;
	assign vram_wdata_o  = {2{wdata_i}};

	assign sndram_own_o   = is_sndram && active;
	assign sndram_addr_o  = addr_i[15:0];
	assign sndram_we_o    = is_sndram && write_now;
	assign sndram_wdata_o = wdata_i;

	assign cartram_own_o   = is_cartram && active;
	assign cartram_addr_o  = addr_i[14:0];
	assign cartram_we_o    = is_cartram && write_now;
	assign cartram_wdata_o = wdata_i;

	assign pal_own_o    = is_pal && active;
	assign pal_addr_o   = addr_i[8:1];
	assign pal_we_o     = is_pal && write_now;
	assign pal_be_o     = be;
	assign pal_wdata_o  = {2{wdata_i}};

	wire [15:0] word_q = is_wram ? wram_q_i : is_vram ? vram_q_i : pal_q_i;
	wire [7:0]  byte_q = is_sndram ? sndram_q_i : cartram_q_i;
	wire        is_word = is_wram || is_vram || is_pal;

	always @(posedge clk) begin
		if (reset) begin
			st          <= S_IDLE;
			served_v    <= 1'b0;
			served_addr <= 25'd0;
			served_type <= 3'd0;
			served_we   <= 1'b0;
			rdata_o     <= 8'd0;
		end else begin
			case (st)
			S_IDLE:
				if (go) begin
					served_addr <= addr_i;
					served_type <= type_i;
					served_we   <= wr_i;
					st <= S_READ;
				end

			S_READ: begin
				rdata_o <= is_word ? (addr_i[0] ? word_q[7:0] : word_q[15:8])
				                   : byte_q;
				served_v <= 1'b1;
				st <= S_DONE;
			end

			// One idle cycle so an address change on the edge the answer is
			// taken still gets a clean sequence.
			S_DONE:
				st <= S_IDLE;

			default: st <= S_IDLE;
			endcase

			if (served_v && !served) served_v <= 1'b0;
		end
	end

	`ifdef VERILATOR
	/* verilator lint_off UNUSEDSIGNAL */
	wire unused = &{1'b0, addr_i[24:17]};
	/* verilator lint_on UNUSEDSIGNAL */
	`endif

endmodule
