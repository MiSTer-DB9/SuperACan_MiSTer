// Copyright (c) 2026 Jamie Blanks

// VRAM arbiter. The two SRAMs are one port, so each consumer takes a turn on
// a fixed eight-slot schedule per dot. An H256 dot is ten cycles; the two
// extra go to the last slot.
//
//   slot  owner                        what it needs
//   ----  ---------------------------  ------------------------------------
//     0   tilemap layer 0              one map word and up to four data
//     1   tilemap layer 1              words per eight-pixel cell
//     2   tilemap layer 2
//     3   tilemap layer 3
//     4   ROZ                          a map word and a data word per pixel
//     5   ROZ
//     6   sprite engine                one line's walk, spread over the line
//     7   68000 window, and the sprite DMA when it holds the bus
//
// An unused slot falls through to the first waiting client in a fixed order,
// CPU first since it is the only one that stalls the machine.
//
// `gnt` is combinational so a client can drop its request on the edge that
// takes its address. `ack` marks the read data three cycles later.

module um6618_vram_arb #(
	parameter int NCLIENTS = 9
) (
	input  wire                    clk_sys,
	input  wire                    ce_pix,

	input  wire [NCLIENTS-1:0]     req,
	input  wire [NCLIENTS-1:0]     we,
	input  wire [NCLIENTS*16-1:0]  addr,     // word addresses, VRAM_A[16:1]
	input  wire [NCLIENTS*16-1:0]  wdata,
	output wire [NCLIENTS-1:0]     gnt,      // the address goes out at this edge
	output reg  [NCLIENTS-1:0]     ack,      // q is this client's answer
	output reg  [15:0]             q,

	output reg  [16:1]             VRAM_A,
	input  wire [15:0]             VRAM_D_i,
	output reg  [15:0]             VRAM_D_o,
	output reg                     VRAM_D_oe,
	output reg                     VRAM_OE_n,
	output reg                     VRAM_WE_n
);
	// Client numbers. Tilemap layers are consecutive for a generate loop.
	localparam int C_CPU  = 0;
	localparam int C_DMA  = 1;
	localparam int C_TM0  = 2;
	localparam int C_ROZ  = 6;
	localparam int C_SPR  = 7;
	localparam int C_WIN  = 8;

	// Slot owners.
	localparam [8*4-1:0] SLOT_OWNER = {
		4'(C_CPU), 4'(C_SPR), 4'(C_ROZ), 4'(C_ROZ),
		4'(C_TM0 + 3), 4'(C_TM0 + 2), 4'(C_TM0 + 1), 4'(C_TM0)
	};

	// Fallthrough order, most urgent first.
	localparam [9*4-1:0] FALLBACK = {
		4'(C_TM0 + 3), 4'(C_TM0 + 2), 4'(C_TM0 + 1), 4'(C_TM0),
		4'(C_ROZ), 4'(C_SPR), 4'(C_WIN), 4'(C_DMA), 4'(C_CPU)
	};

	reg [2:0] slot;

	// Restarts every dot; sticks at seven in H256 so the extra two cycles go
	// to the 68000.
	always @(posedge clk_sys) begin
		if (ce_pix) slot <= 3'd0;
		else if (slot != 3'd7) slot <= slot + 3'd1;
	end

	wire [3:0] owner = SLOT_OWNER[slot*4 +: 4];

	reg [3:0] winner;
	reg       any;
	reg       write_reserved = 1'b0;
	reg [3:0] reserved_writer;

	always @(*) begin
		winner = owner;
		any    = req[owner];
		if (!any) begin
			for (int i = 0; i < NCLIENTS; i = i + 1) begin
				if (!any && req[FALLBACK[i*4 +: 4]]) begin
					winner = FALLBACK[i*4 +: 4];
					any    = 1'b1;
				end
			end
		end
		if (write_reserved && req[reserved_writer]) begin
			winner = reserved_writer;
			any    = 1'b1;
		end
	end

	// Read data returns after the next address could issue, so leave the bus
	// undriven for it before a write, and hold the writer's slot so readers
	// cannot starve it.
	wire turnaround = any && we[winner] && !VRAM_OE_n && VRAM_WE_n;
	wire issue = any && !turnaround;
	assign gnt = issue ? (NCLIENTS'(1) << winner) : {NCLIENTS{1'b0}};

	reg [NCLIENTS-1:0] gnt_q1, gnt_q2;

	always @(posedge clk_sys) begin
		write_reserved <= turnaround;
		if (turnaround) reserved_writer <= winner;
		VRAM_D_oe <= 1'b0;
		VRAM_OE_n <= 1'b1;
		VRAM_WE_n <= 1'b1;

		if (issue) begin
			VRAM_A    <= addr[winner*16 +: 16];
			VRAM_D_o  <= wdata[winner*16 +: 16];
			VRAM_D_oe <= we[winner];
			VRAM_OE_n <= ~we[winner] ? 1'b0 : 1'b1;
			VRAM_WE_n <=  we[winner] ? 1'b0 : 1'b1;
		end

		gnt_q1 <= gnt;
		gnt_q2 <= gnt_q1;
		ack    <= gnt_q2;
		q      <= VRAM_D_i;
	end
endmodule
