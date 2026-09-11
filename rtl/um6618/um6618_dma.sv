// Copyright (c) 2026 Jamie Blanks

// Destination half of the sprite DMA at $F00010-$F0001E.
//
// This chip has no bus arbitration pins, so the UM6619 fetches the source
// words and the two share the work over the bus plus two wires:
//
//   fwd   UM6619 -> here.  Toggles once per unit while the source word is on
//         CPU_D (or once per unit of a fill), then once more to close.
//   rev   here -> UM6619.  Toggles once per committed unit. Outside a command
//         the same pin carries the vertical retrace interrupt; tok_active
//         says which.
//
//   write $1E with bit 15  ---> both chips snapshot the eight words
//   fwd toggle             ---> take CPU_D (or zero), write it to VRAM
//   rev toggle             <--- committed, pointers stepped
//   ... count+1 times, up to two units in flight ...
//   fwd toggle             ---> closed: final pointers written back
//
// A destination with bit 13 or 14 set, or inside $F40000-$F5FFFF, is written
// here through the VRAM port; anything else the UM6619 writes over the bus
// while this side steps its pointers. Acknowledgements are spaced three
// clocks apart so the UM6619 sees each toggle on its own clock edges.

module um6618_dma (
	input  wire         clk_sys,
	input  wire         reset,

	// Live copy of $10-$1E, and the trigger write.
	input  wire [127:0] regs,
	input  wire         trigger,       // write to $1E with bit 15 set
	input  wire [15:0]  trigger_ctrl,  // data of that write

	input  wire [15:0]  CPU_D_i,
	input  wire         fwd,
	output reg          rev,
	output reg          tok_active,
	input  wire         idle_level,    // shared pin level when idle
	input  wire         hold,          // framework pause: hold tokens back

	// VRAM arbiter client.
	output wire         vram_req,
	output wire [15:0]  vram_addr,
	output wire [15:0]  vram_data,
	input  wire         vram_gnt,
	input  wire         vram_ack,

	// Command closed: final pointers go back to the register file and the
	// sprite engine re-reads its table.
	output reg          done,
	output wire [31:0]  dst_out,
	output wire [31:0]  src_out,
	output wire         busy,

	// Savestate.
	input  wire [63:0]  ss_din,
	input  wire [9:0]   ss_addr,
	input  wire         ss_wren,
	input  wire         ss_rst,
	output wire [63:0]  ss_dout
);
	`include "ss_map.svh"

	localparam [1:0] R_IDLE = 2'd0, R_RUN = 2'd1, R_CLOSE = 2'd2;

	reg [1:0]  rstate;
	reg        pend;         // triggered, not yet closed
	reg        pend_again;   // triggered again while a command was running
	reg [31:0] w_dst, w_src;
	reg [15:0] w_dst_inc, w_src_inc, w_left;
	reg        w_fill, w_local;
	reg        last_popped;
	reg        fwd_q;
	reg [15:0] q_data [0:1];
	reg [1:0]  q_count;
	reg        q_rd, q_wr;
	reg [1:0]  acks_owed;
	reg [1:0]  rev_hold;
	reg        write_out;    // a word is at the arbiter, not yet acknowledged
	reg        step_wait;    // bus-side unit commits one clock after its token

	// Control bits 13 and 14 force the destination into VRAM.
	function automatic [31:0] effective(input [1:0] sel, input [31:0] d);
		effective = (|sel) ? (d | 32'h00F40000) : d;
	endfunction
	wire [31:0] regs_dst  = {regs[31:16], regs[47:32]};
	wire [31:0] trig_dst  = effective(trigger_ctrl[14:13], regs_dst);
	wire [31:0] again_dst = effective(regs[126:125], regs_dst);

	wire fwd_edge  = (fwd != fwd_q);
	wire push      = fwd_edge && ((rstate == R_RUN) || (rstate == R_IDLE && pend));
	wire pop_local = vram_gnt;
	wire pop_bus   = (rstate == R_RUN) && !w_local && (q_count != 2'd0) && !step_wait;
	wire pop       = pop_local | pop_bus;
	wire commit    = vram_ack | step_wait;
	// Acknowledgements wait out a pause so the stopped UM6619 sees them all.
	wire emit      = !hold && (rev_hold == 2'd0) && (acks_owed != 2'd0);

	assign vram_req  = (rstate == R_RUN) && w_local && (q_count != 2'd0) && !write_out;
	assign vram_addr = w_dst[16:1];
	assign vram_data = q_data[q_rd];
	assign dst_out   = w_dst;
	assign src_out   = w_src;
	assign busy      = pend | (rstate != R_IDLE);

	task automatic snapshot(input fill, input [31:0] d);
		w_dst     <= d;
		w_dst_inc <= regs[63:48];
		w_src     <= {regs[79:64], regs[95:80]};
		w_src_inc <= regs[111:96];
		w_left    <= regs[15:0];
		w_fill    <= fill;
		w_local   <= (d[23:17] == 7'b1111010);
	endtask

	always @(posedge clk_sys) begin
		done  <= 1'b0;
		fwd_q <= fwd;
		if (reset) begin
			rstate      <= ss_saved[1:0];
			pend        <= ss_saved[2];
			pend_again  <= ss_saved[3];
			w_fill      <= ss_saved[4];
			w_local     <= ss_saved[5];
			fwd_q       <= ss_saved[6];
			rev         <= ss_saved[7];
			tok_active  <= ss_saved[8];
			q_count     <= ss_saved[10:9];
			q_rd        <= ss_saved[11];
			q_wr        <= ss_saved[12];
			acks_owed   <= ss_saved[14:13];
			rev_hold    <= ss_saved[16:15];
			write_out   <= ss_saved[17];
			step_wait   <= ss_saved[18];
			last_popped <= ss_saved[19];
			w_left      <= ss_saved[47:32];
			w_dst_inc   <= ss_saved[63:48];
			w_dst       <= ss_saved[95:64];
			w_src       <= ss_saved[127:96];
			w_src_inc   <= ss_saved[143:128];
			q_data[0]   <= ss_saved[159:144];
			q_data[1]   <= ss_saved[175:160];
		end else begin
			// A trigger during a running command is remembered and takes the
			// registers when the command closes.
			if (trigger) begin
				if (rstate == R_IDLE) begin
					pend <= 1'b1;
					snapshot(trigger_ctrl[8], trig_dst);
				end else begin
					pend_again <= 1'b1;
				end
			end

			// First token opens the command and takes over the retrace pin at
			// its current level.
			if (fwd_edge && rstate == R_IDLE && pend) begin
				rstate      <= R_RUN;
				tok_active  <= 1'b1;
				rev         <= idle_level;
				last_popped <= 1'b0;
			end

			if (push) begin
				q_data[q_wr] <= w_fill ? 16'd0 : CPU_D_i;
				q_wr         <= ~q_wr;
			end

			if (pop) begin
				q_rd  <= ~q_rd;
				w_dst <= w_dst + {15'd0, w_dst_inc, 1'b0};
				if (!w_fill) w_src <= w_src + {15'd0, w_src_inc, 1'b0};
				if (w_left == 16'd0) last_popped <= 1'b1;
				else                 w_left      <= w_left - 16'd1;
				write_out <= pop_local;
				step_wait <= pop_bus;
			end
			if (commit) begin
				write_out <= 1'b0;
				step_wait <= 1'b0;
			end

			q_count   <= q_count + {1'b0, push} - {1'b0, pop};
			acks_owed <= acks_owed + {1'b0, commit} - {1'b0, emit};

			if (rev_hold != 2'd0) rev_hold <= rev_hold - 2'd1;
			if (emit) begin
				rev      <= ~rev;
				rev_hold <= 2'd3;
			end

			if (rstate == R_RUN && last_popped && q_count == 2'd0 && !write_out && !step_wait)
				rstate <= R_CLOSE;

			// Closing token: pointers written back, pin handed back.
			if (fwd_edge && rstate == R_CLOSE) begin
				done       <= 1'b1;
				tok_active <= 1'b0;
				rstate     <= R_IDLE;
				pend       <= pend_again;
				pend_again <= 1'b0;
				if (pend_again) snapshot(regs[120], again_dst);
			end
		end
	end

	// ---- savestate ---------------------------------------------------------

	wire [63:0]  ss_saved_w [0:2];
	wire [63:0]  ss_dout_w  [0:2];
	wire [191:0] ss_saved = {ss_saved_w[2], ss_saved_w[1], ss_saved_w[0]};
	wire [191:0] ss_live = {16'd0, q_data[1], q_data[0], w_src_inc, w_src, w_dst, w_dst_inc, w_left,
	                        12'd0, last_popped, step_wait, write_out, rev_hold, acks_owed, q_wr, q_rd,
	                        q_count, tok_active, rev, fwd_q, w_local, w_fill, pend_again, pend, rstate};

	genvar gw;
	generate
		for (gw = 0; gw < 3; gw++) begin : g_ss
			ss_reg #(.ADDR(SSW_6618_DMA + gw)) word (
				.clk_i(clk_sys), .bus_din_i(ss_din), .bus_addr_i(ss_addr),
				.bus_wren_i(ss_wren), .bus_rst_i(ss_rst), .bus_dout_o(ss_dout_w[gw]),
				.din_i(ss_live[gw*64 +: 64]), .dout_o(ss_saved_w[gw])
			);
		end
	endgenerate
	assign ss_dout = ss_dout_w[0] | ss_dout_w[1] | ss_dout_w[2];

	`ifdef VERILATOR
	/* verilator lint_off UNUSEDSIGNAL */
	wire unused = &{1'b0, ss_saved[191:176], ss_saved[31:20],
	                regs[127], regs[124:121], regs[119:112],
	                trigger_ctrl[15], trigger_ctrl[12:9], trigger_ctrl[7:0]};
	/* verilator lint_on UNUSEDSIGNAL */
	`endif
endmodule
