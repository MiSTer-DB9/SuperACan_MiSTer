// Copyright (c) 2026 Jamie Blanks

// Builds a 16-byte MiSTer cheat record from IOCTL halfword writes and runs an
// atomic live reload. wait_o holds the HPS until the machine has parked, and
// pause_req_o stays high until the engine acknowledges the commit, so no
// instruction can mix old and newly patched bytes.
//
// Records arrive little endian as method/width, address, compare, value.
// Bit 128 strobes for one clock when a record completes.
module acan_cheat_loader
(
	input  wire         clk_sys_i,
	input  wire         cheat_download_i,
	input  wire         cart_download_i,
	input  wire         paused_i,
	input  wire         commit_done_i,
	input  wire         ioctl_wr_i,
	input  wire  [3:0]  ioctl_addr_i,
	input  wire [15:0]  ioctl_dout_i,
	output wire         wait_o,
	output reg          pause_req_o,
	output reg          load_begin_o,
	output reg          invalidate_o,
	output reg          commit_req_o,
	output reg  [128:0] code_o
);

	reg cheat_download_q;
	reg cart_download_q;

	// No payload halfword is accepted until the machine has stopped.
	assign wait_o = cheat_download_i && !paused_i;

	// Power-up state only; a console reset must not remove cheats.
	initial begin
		pause_req_o = 1'b0;
		load_begin_o = 1'b0;
		invalidate_o = 1'b0;
		commit_req_o = 1'b0;
		code_o = 129'd0;
		cheat_download_q = 1'b0;
		cart_download_q = 1'b0;
	end

	always @(posedge clk_sys_i) begin
		code_o[128] <= 1'b0;
		load_begin_o <= 1'b0;
		invalidate_o <= 1'b0;
		cheat_download_q <= cheat_download_i;
		cart_download_q <= cart_download_i;

		// The engine acknowledges once the last record has expanded.
		if (commit_done_i) begin
			pause_req_o <= 1'b0;
			commit_req_o <= 1'b0;
		end

		// A new transfer starts a fresh inactive generation; the active table
		// stays untouched.
		if (cheat_download_i && !cheat_download_q) begin
			pause_req_o <= 1'b1;
			load_begin_o <= 1'b1;
			commit_req_o <= 1'b0;
		end

		// Held as a level until commit_done_i; a pulse could outrun a pending
		// expansion in the engine.
		if (!cheat_download_i && cheat_download_q) begin
			commit_req_o <= 1'b1;
		end

		// A new cartridge drops the old cheats.
		if (cart_download_i && !cart_download_q) begin
			pause_req_o <= 1'b0;
			commit_req_o <= 1'b0;
			invalidate_o <= 1'b1;
		end

		if (cheat_download_i && paused_i && ioctl_wr_i) begin
			case (ioctl_addr_i)
				4'd0:  code_o[111:96]  <= ioctl_dout_i;
				4'd2:  code_o[127:112] <= ioctl_dout_i;
				4'd4:  code_o[79:64]   <= ioctl_dout_i;
				4'd6:  code_o[95:80]   <= ioctl_dout_i;
				4'd8:  code_o[47:32]   <= ioctl_dout_i;
				4'd10: code_o[63:48]   <= ioctl_dout_i;
				4'd12: code_o[15:0]    <= ioctl_dout_i;
				4'd14: begin
					code_o[31:16] <= ioctl_dout_i;
					code_o[128]   <= 1'b1;
				end
				default: ;
			endcase
		end
	end

endmodule
