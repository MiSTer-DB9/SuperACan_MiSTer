// Copyright (c) 2026 Jamie Blanks

// Parks the machine where a savestate may stop it, in two steps.
//
//   1. CPUs and bus: 68000 strobes negated, 65C02 sound-RAM arbiter idle,
//      both UM6619 DMA channels and the sprite DMA between transfers, no
//      cartridge ROM read outstanding. `pause_cpu` holds that.
//   2. Video: runs on to a quiet point in blanking before sprite evaluation.
//      `pause_vid` holds that.
//
//   req       ___/‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾\____
//   pause_cpu _______/‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾\____
//   pause_vid __________________/‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾\____
//   parked    __________________/‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾\____
//
// Both conditions may never coincide on one cycle since every clock divides
// the same crystal, hence two steps. The CPU goes first because a 68000
// access to the UM6618 waits on !DTACK; freezing the video mid-cycle would
// hang it. Worst case is one frame before a save or load starts.

module ss_park (
	input  wire clk_sys,
	input  wire reset,

	input  wire req,        // sleep_savestate from the transfer engine

	input  wire cpu_ready,  // 68000 between instructions, bus idle
	input  wire snd_ready,  // 65C02 between instructions
	input  wire mem_idle,   // no cartridge ROM read outstanding
	input  wire vid_ready,  // UM6618 at its quiescent blanking boundary

	output wire pause_cpu,
	output wire pause_vid,
	output reg  parked
);
	localparam [1:0] S_RUN    = 2'd0;
	localparam [1:0] S_CPU    = 2'd1;
	localparam [1:0] S_VIDEO  = 2'd2;
	localparam [1:0] S_PARKED = 2'd3;

	reg [1:0] st;
	reg       pause_cpu_q, pause_vid_q;

	// The pause lands on the edge the condition held; a registered version
	// would let the machine take one more step. So no ready term may depend
	// on its own clock enable.
	wire ready_cpu = cpu_ready & snd_ready & mem_idle;

	assign pause_cpu = pause_cpu_q | ((st == S_CPU) & req & ready_cpu);
	assign pause_vid = pause_vid_q | ((st == S_VIDEO) & req & vid_ready);

	always @(posedge clk_sys) begin
		if (reset) begin
			st          <= S_RUN;
			pause_cpu_q <= 1'b0;
			pause_vid_q <= 1'b0;
			parked      <= 1'b0;
		end else begin
			case (st)
			S_RUN: begin
				pause_cpu_q <= 1'b0;
				pause_vid_q <= 1'b0;
				parked      <= 1'b0;
				if (req) st <= S_CPU;
			end

			S_CPU:
				if (!req) begin
					st <= S_RUN;
				end else if (ready_cpu) begin
					pause_cpu_q <= 1'b1;
					st <= S_VIDEO;
				end

			S_VIDEO:
				if (!req) begin
					pause_cpu_q <= 1'b0;
					st <= S_RUN;
				end else if (vid_ready) begin
					pause_vid_q <= 1'b1;
					parked      <= 1'b1;
					st          <= S_PARKED;
				end

			default:
				if (!req) begin
					pause_cpu_q <= 1'b0;
					pause_vid_q <= 1'b0;
					parked      <= 1'b0;
					st          <= S_RUN;
				end
			endcase
		end
	end
endmodule
