// Copyright (c) 2026 Jamie Blanks
//
// Turns save/load requests and rewind into addressed transfer requests for
// the savestate engine. Manual saves use the slot in savestate_number; rewind
// keeps a REWIND_COUNT ring below Softmap_Rewind_ADDR, capturing every
// TIME_CAPTURE clocks and stepping back every TIME_REWIND clocks while held.
// sleep_rewind stalls the core until the picture is back after a rewind load.

module statemanager
#(
	parameter integer Softmap_SaveState_ADDR = 0,
	parameter integer Softmap_Rewind_ADDR = 0,
	parameter integer SAVESTATE_SHIFT = 17,
	// clk_sys cycles between rewind captures.
	parameter integer TIME_CAPTURE = 10_000_000
)
(
	input  wire clk,
	input  wire reset,

	input  wire rewind_on,
	input  wire rewind_active,

	input  integer savestate_number,
	input  wire save,
	input  wire load,

	output reg  sleep_rewind = 1'b0,
	input  wire vsync,

	output reg  request_savestate = 1'b0,
	output reg  request_loadstate = 1'b0,
	// High with the request pulse when it targets the rewind ring. The engine
	// latches sizes on acceptance, so the parent must mux them off this
	// combinationally.
	output reg  request_is_rewind = 1'b0,
	output integer request_address,
	input  wire request_busy
);

	localparam integer REWIND_COUNT = 4;

	localparam integer TIME_REWIND = 5000000;

	reg save_q = 1'b0;
	reg load_q = 1'b0;
	reg save_pending_q = 1'b0;
	reg load_pending_q = 1'b0;
	reg rewind_enabled_q = 1'b0;
	reg rewind_load_pending_q = 1'b0;
	integer capture_timer_q = 0;
	integer rewind_timer_q = 0;
	integer rewind_count_q = 0;
	integer rewind_position_q = 0;
	integer vsync_count_q = 0;
	reg vsync_q;

	function integer savestate_slot_addr;
		input integer base_addr_i;
		input integer slot_i;
		begin
			savestate_slot_addr = base_addr_i + (slot_i << SAVESTATE_SHIFT);
		end
	endfunction

	always @(posedge clk) begin
		request_savestate <= 1'b0;
		request_loadstate <= 1'b0;
		request_is_rewind <= 1'b0;
		vsync_q <= vsync;

		save_q <= save;
		if (save && !save_q) begin
			save_pending_q <= 1'b1;
		end
		load_q <= load;
		if (load && !load_q) begin
			load_pending_q <= 1'b1;
		end

		// rewind_load_pending_q is sticky, so drop it whenever the ring is
		// re-primed or a stale position would be restored.
		if (!rewind_on || reset) begin
			rewind_enabled_q <= 1'b0;
			rewind_load_pending_q <= 1'b0;
		end

		if (!rewind_active) begin
			rewind_timer_q <= 0;
		end else if (rewind_timer_q < TIME_REWIND) begin
			rewind_timer_q <= rewind_timer_q + 1;
		end

		if (rewind_active) begin
			capture_timer_q <= 0;
		end else if (capture_timer_q < TIME_CAPTURE) begin
			capture_timer_q <= capture_timer_q + 1;
		end

		if ((vsync_count_q < 2) && vsync && !vsync_q) begin
			vsync_count_q <= vsync_count_q + 1;
		end

		sleep_rewind <= 1'b0;
		if ((vsync_count_q == 2) && rewind_active) begin
			sleep_rewind <= 1'b1;
		end

		if (!reset && !request_busy) begin
			if (save_pending_q) begin
				request_address <= savestate_slot_addr(Softmap_SaveState_ADDR, savestate_number);
				request_savestate <= 1'b1;
				save_pending_q <= 1'b0;
			end else if (load_pending_q) begin
				request_address <= savestate_slot_addr(Softmap_SaveState_ADDR, savestate_number);
				request_loadstate <= 1'b1;
				load_pending_q <= 1'b0;
			end else if (!rewind_enabled_q && rewind_on) begin
				request_address <= Softmap_Rewind_ADDR;
				request_savestate <= 1'b1;
				request_is_rewind <= 1'b1;
				rewind_enabled_q <= 1'b1;
				capture_timer_q <= 0;
				rewind_count_q <= 1;
				rewind_position_q <= 1;
			end else if (rewind_enabled_q && (capture_timer_q == TIME_CAPTURE)) begin
				request_address <= savestate_slot_addr(Softmap_Rewind_ADDR, rewind_position_q);
				request_savestate <= 1'b1;
				request_is_rewind <= 1'b1;
				capture_timer_q <= 0;
				if (rewind_count_q < REWIND_COUNT) begin
					rewind_count_q <= rewind_count_q + 1;
				end
				if (rewind_position_q < (REWIND_COUNT - 1)) begin
					rewind_position_q <= rewind_position_q + 1;
				end else begin
					rewind_position_q <= 0;
				end
			end else if (rewind_enabled_q && (rewind_timer_q == TIME_REWIND)) begin
				if (rewind_count_q > 1) begin
					rewind_count_q <= rewind_count_q - 1;
					if (rewind_position_q > 0) begin
						rewind_position_q <= rewind_position_q - 1;
					end else begin
						rewind_position_q <= REWIND_COUNT - 1;
					end
					rewind_load_pending_q <= 1'b1;
				end
				rewind_timer_q <= 0;
			end else if (rewind_load_pending_q) begin
				// Consumed only here, so a busy engine cannot lose a step.
				request_address <= savestate_slot_addr(Softmap_Rewind_ADDR, rewind_position_q);
				request_loadstate <= 1'b1;
				request_is_rewind <= 1'b1;
				rewind_load_pending_q <= 1'b0;
				vsync_count_q <= 0;
			end
		end
	end

endmodule
