// Copyright (c) 2026 Jamie Blanks

// The two host DMA channels at $E90020-$E9003F, and the bus master they
// share with the UM6618's sprite DMA. Transfers arbitrate through !BR, !BG
// and !BGACK and run real four-clock bus cycles, so the CPU stalls for the
// length of the transfer.
//
//   !BR low  ->  !BG low  ->  bus quiet  ->  !BGACK low, !BR high
//                                            run the units
//                                            !BGACK high
//
// Host transfers move one word a unit. Control bit 12 clear rewrites the
// same source word every unit (how games clear RAM with $8800/$A800), bit 8
// pulls the destination back every sixteen bytes for eight-word sprite
// records, bits 10 and 9 count destination and source down.
//
// The sprite DMA is programmed through the UM6618 at $F00010, but that chip
// has no arbitration pins, so this one watches those writes, fetches each
// source word and toggles pin 136 with it on the bus. The UM6618 writes it
// to VRAM and toggles pin 134 back, which is otherwise the retrace
// interrupt. A last toggle closes the command; up to two units are in
// flight. A sprite command triggered from inside a host transfer runs under
// the same grant. Everything moves on the two edges of the CPU clock.

module um6619_dma (
	input  wire        clk_sys,
	input  wire        reset,
	input  wire        ce_phi1,
	input  wire        ce_phi2,

	// Channel register files at $E90020-$E9003F.
	input  wire        reg_wr,
	input  wire [4:1]  reg_addr,     // bit4 picks the channel
	input  wire [1:0]  reg_be,       // {UDS, LDS}
	input  wire [15:0] reg_din,
	output reg  [15:0] reg_dout,

	// Writes to the UM6618's $F00010-$F0001E, seen on the bus.
	input  wire        vid_wr,
	input  wire [3:1]  vid_index,
	input  wire [15:0] vid_din,

	// Sprite DMA wires to the UM6618.
	output reg         fwd,          // pin 136
	input  wire        rev,          // pin 134
	output reg         tok_active,   // pin 134 is a token, not an interrupt

	// Bus arbitration.
	output reg         BR_n,
	input  wire        BG_n,
	output reg         BGACK_n,

	// The bus while this chip holds it.
	output reg  [23:1] A_o,
	output reg         AS_n_o,
	output reg         UDS_n_o,
	output reg         LDS_n_o,
	output reg         RW_o,
	output wire        bus_oe,
	input  wire [15:0] D_i,
	output reg  [15:0] D_o,
	output reg         D_oe,
	input  wire        DTACK_n,

	// The bus while the CPU holds it.
	input  wire        AS_n_i,

	output wire        busy,

	// Savestate. ss_load is the restore window.
	input  wire        ss_load,
	input  wire [63:0] ss_din,
	input  wire [9:0]  ss_addr,
	input  wire        ss_wren,
	input  wire        ss_rst,
	output wire [63:0] ss_dout
);
	`include "ss_map.svh"

	wire ph = ce_phi1 | ce_phi2;

	// ---- the two register files ---------------------------------------------

	reg [31:0] src   [0:1];
	reg [31:0] dst   [0:1];
	reg [15:0] count [0:1];
	reg [15:0] ctrl  [0:1];

	wire       ch    = reg_addr[4];
	wire [2:0] index = reg_addr[3:1];

	always @(*) begin
		case (index)
			3'd0:    reg_dout = src[ch][31:16];
			3'd1:    reg_dout = src[ch][15:0];
			3'd2:    reg_dout = dst[ch][31:16];
			3'd3:    reg_dout = dst[ch][15:0];
			3'd4:    reg_dout = count[ch];
			3'd5:    reg_dout = ctrl[ch];
			default: reg_dout = 16'd0;
		endcase
	end

	// Only a write touching the low byte of the control register triggers,
	// so one word write cannot start two bursts.
	wire [15:0] merged = {reg_be[1] ? reg_din[15:8] : reg_dout[15:8],
	                      reg_be[0] ? reg_din[7:0]  : reg_dout[7:0]};
	wire trigger_now = reg_wr & (index == 3'd5) & reg_be[0]
	                 & ((merged & 16'h8800) != 16'd0);

	// ---- this chip's copy of the sprite DMA block --------------------------

	// The UM6618's eight words, kept by watching the bus. A write to $1E
	// with bit 15 set queues a command.
	reg [127:0] vregs;
	reg         vid_wr_q;
	reg         vid_pending;
	wire        vid_trigger = vid_wr & ~vid_wr_q & (vid_index == 3'd7) & vid_din[15];

	function automatic [31:0] effective(input [1:0] sel, input [31:0] d);
		effective = (|sel) ? (d | 32'h00F40000) : d;
	endfunction

	// ---- the bus cycle engine ------------------------------------------------

	// Address out on a rising edge, strobes on the next falling edge, !DTACK
	// sampled from the third falling edge, strobes off on the rising edge
	// after that. A held read keeps its strobes and data up until the UM6618
	// has taken the word.
	localparam [2:0] B_IDLE = 3'd0, B_S0 = 3'd1, B_S2 = 3'd2, B_S4 = 3'd3,
	                 B_S6 = 3'd4, B_HOLD = 3'd5;

	reg [2:0]  bus_state;
	reg        req_t, ack_t;           // cycle wanted / started
	reg        consume_t, consumed_t;  // held read may be released / was
	reg        bus_write, bus_hold;
	reg [23:1] bus_addr;
	reg [15:0] bus_wdata;
	reg        holding;

	wire bus_pending    = (req_t != ack_t);
	wire release_wanted = (consume_t != consumed_t);
	wire sample_now     = (bus_state == B_S4) && ce_phi2 && !DTACK_n;
	// The next address goes out on the edge that ends the held read.
	wire src_is_wram    = (A_o[23:18] == 6'b111111);

	assign bus_oe = holding;

	task automatic launch;
		A_o    <= bus_addr;
		RW_o   <= ~bus_write;
		ack_t  <= req_t;
		bus_state <= B_S0;
	endtask

	task automatic drop_strobes;
		AS_n_o  <= 1'b1;
		UDS_n_o <= 1'b1;
		LDS_n_o <= 1'b1;
		RW_o    <= 1'b1;
		D_oe    <= 1'b0;
	endtask

	always @(posedge clk_sys) begin
		if (ss_load) begin
			{bus_state, ack_t, consumed_t, AS_n_o, UDS_n_o, LDS_n_o, RW_o, D_o, D_oe, A_o}
				<= ss_saved[SS_BUS_LO +: SS_BUS_BITS];
		end else if (reset) begin
			bus_state  <= B_IDLE;
			ack_t      <= 1'b0;
			consumed_t <= 1'b0;
			AS_n_o     <= 1'b1;
			UDS_n_o    <= 1'b1;
			LDS_n_o    <= 1'b1;
			RW_o       <= 1'b1;
			D_oe       <= 1'b0;
		end else begin
			case (bus_state)
			B_IDLE:
				if (bus_pending && ce_phi1) launch();

			B_S0:
				if (ce_phi2) begin
					AS_n_o <= 1'b0;
					if (!bus_write) begin
						UDS_n_o <= 1'b0;
						LDS_n_o <= 1'b0;
					end else begin
						D_o  <= bus_wdata;
						D_oe <= 1'b1;
					end
					bus_state <= B_S2;
				end

			B_S2:
				if (ce_phi2) begin
					if (bus_write) begin
						UDS_n_o <= 1'b0;
						LDS_n_o <= 1'b0;
					end
					bus_state <= B_S4;
				end

			B_S4:
				if (ce_phi2 && !DTACK_n)
					bus_state <= bus_hold ? B_HOLD : B_S6;

			B_S6:
				if (ce_phi1) begin
					drop_strobes();
					bus_state <= B_IDLE;
				end

			B_HOLD:
				if (release_wanted && ce_phi1) begin
					drop_strobes();
					consumed_t <= consume_t;
					if (bus_pending && src_is_wram) launch();
					else bus_state <= B_IDLE;
				end

			default: bus_state <= B_IDLE;
			endcase
		end
	end

	// ---- the transfer engine -------------------------------------------------

	localparam [3:0] T_IDLE = 4'd0, T_REQ = 4'd1, T_WAIT_BG = 4'd2, T_TAKE = 4'd3,
	                 T_UNIT = 4'd4, T_READ = 4'd5, T_SEND = 4'd6, T_WRITE = 4'd7,
	                 T_CLOSE = 4'd8, T_RELEASE = 4'd9;

	reg [3:0]  tstate;
	reg        which;          // host channel 0 or 1
	reg        is_video;
	reg        host_resume;    // a host transfer waits under this grant
	reg [15:0] host_left;
	reg [31:0] t_src, t_dst;
	reg [15:0] t_left, t_ctrl, t_src_inc, t_dst_inc;
	reg [1:0]  in_flight;      // units sent and not yet acknowledged
	reg        rev_q;          // the last token level seen
	reg        sync_pending;   // first token out; learn the pin's level
	reg [1:0]  tok_guard;      // phases before pin 134 is an interrupt again

	assign busy = (tstate != T_IDLE) | vid_pending | tok_active;

	wire v_fill    = is_video & t_ctrl[8];
	wire v_local   = is_video & (t_dst[23:17] == 7'b1111010);
	wire src_fixed = !is_video & !t_ctrl[12];
	wire dst_down  = !is_video & t_ctrl[10];
	wire src_down  = !is_video & t_ctrl[9];
	wire wrap16    = !is_video & t_ctrl[8];
	wire last_unit = (t_left == 16'd0);

	// Where each pointer goes after a unit.
	wire [31:0] src_next = is_video
	                     ? (v_fill ? t_src : t_src + {15'd0, t_src_inc, 1'b0})
	                     : src_fixed ? t_src
	                     : src_down  ? (t_src - 32'd2)
	                                 : (t_src + 32'd2);

	wire [31:0] dst_stepped = is_video ? (t_dst + {15'd0, t_dst_inc, 1'b0})
	                        : dst_down ? (t_dst - 32'd2)
	                                   : (t_dst + 32'd2);

	wire [31:0] dst_next = (wrap16 && ((dst_stepped & 32'h0000000F) == 32'd0))
	                     ? (dst_stepped - 32'h10) : dst_stepped;

	// An acknowledgement is a level change on pin 134 once the UM6618 owns
	// it. The base level is learned the phase after the first token, and
	// nothing counts while nothing is in flight.
	wire ack_seen = tok_active & ~sync_pending & (in_flight != 2'd0) & (rev != rev_q);
	wire can_send = (in_flight - {1'b0, ack_seen}) != 2'd2;
	// A sprite unit goes out with its token.
	wire sent = (tstate == T_READ  && sample_now && v_local && can_send)
	          | (tstate == T_SEND  && ph && can_send)
	          | (tstate == T_WRITE && sample_now && is_video);

	task automatic load_video;
		is_video    <= 1'b1;
		t_src       <= {vregs[79:64], vregs[95:80]};
		t_dst       <= effective(vregs[126:125], {vregs[31:16], vregs[47:32]});
		t_left      <= vregs[15:0];
		t_ctrl      <= vregs[127:112];
		t_src_inc   <= vregs[111:96];
		t_dst_inc   <= vregs[63:48];
		vid_pending <= 1'b0;
	endtask

	task automatic start_read(input [23:1] a);
		bus_addr  <= a;
		bus_write <= 1'b0;
		bus_hold  <= is_video & v_local;
		req_t     <= ~req_t;
		tstate    <= T_READ;
	endtask

	// One unit is ready: tell the UM6618, step, then close or fetch the next.
	task automatic send_unit;
		fwd       <= ~fwd;
		if (!tok_active) begin
			tok_active   <= 1'b1;
			sync_pending <= 1'b1;
		end
		if (v_local && !v_fill) consume_t <= ~consume_t;
		t_src <= src_next;
		t_dst <= dst_next;
		if (last_unit) begin
			tstate <= T_CLOSE;
		end else begin
			t_left <= t_left - 16'd1;
			if (v_fill) tstate <= T_SEND;
			else        start_read(src_next[23:1]);
		end
	endtask

	always @(posedge clk_sys) begin
		vid_wr_q <= vid_wr;

		if (ss_load) begin
			{tstate, which, is_video, host_resume, host_left, t_src, t_dst, t_left, t_ctrl,
			 t_src_inc, t_dst_inc, in_flight, rev_q, sync_pending, tok_guard, fwd, tok_active,
			 vid_pending, vid_wr_q, req_t, consume_t, bus_write, bus_hold, bus_addr, bus_wdata,
			 BR_n, BGACK_n, holding} <= ss_saved[SS_XFER_LO +: SS_XFER_BITS];
			{ctrl[1], ctrl[0], count[1], count[0],
			 dst[1], dst[0], src[1], src[0]} <= ss_saved[191:0];
			vregs <= vregs_saved;
		end else if (reset) begin
			tstate       <= T_IDLE;
			is_video     <= 1'b0;
			host_resume  <= 1'b0;
			in_flight    <= 2'd0;
			sync_pending <= 1'b0;
			tok_guard    <= 2'd0;
			fwd          <= 1'b0;
			tok_active   <= 1'b0;
			vid_pending  <= 1'b0;
			req_t        <= 1'b0;
			consume_t    <= 1'b0;
			bus_hold     <= 1'b0;
			BR_n         <= 1'b1;
			BGACK_n      <= 1'b1;
			holding      <= 1'b0;
			{ctrl[1], ctrl[0], count[1], count[0],
			 dst[1], dst[0], src[1], src[0]} <= ss_saved[191:0];
			vregs    <= vregs_saved;
		end else begin
			// Register files take every write, triggering or not.
			if (reg_wr) begin
				case (index)
					3'd0: src[ch][31:16]   <= merged;
					3'd1: src[ch][15:0]    <= merged;
					3'd2: dst[ch][31:16]   <= merged;
					3'd3: dst[ch][15:0]    <= merged;
					3'd4: count[ch]        <= merged;
					3'd5: ctrl[ch]         <= merged;
					default: ;
				endcase
			end
			if (vid_wr) vregs[{vid_index, 4'd0} +: 16] <= vid_din;
			if (vid_trigger) vid_pending <= 1'b1;

			// Acknowledgements from the UM6618.
			if (ph) begin
				if (sync_pending) begin
					rev_q        <= rev;
					sync_pending <= 1'b0;
				end else if (ack_seen) begin
					rev_q <= rev;
				end
				in_flight <= in_flight + {1'b0, sent} - {1'b0, ack_seen};
				if (tok_guard != 2'd0) begin
					tok_guard <= tok_guard - 2'd1;
					if (tok_guard == 2'd1) tok_active <= 1'b0;
				end
			end

			case (tstate)
			T_IDLE:
				if (ph) begin
					if (trigger_now) begin
						which    <= ch;
						is_video <= 1'b0;
						t_src    <= src[ch];
						t_dst    <= dst[ch];
						t_left   <= count[ch];
						t_ctrl   <= merged;
						tstate   <= T_REQ;
					end else if (vid_pending) begin
						load_video();
						tstate <= T_REQ;
					end
				end

			T_REQ:
				if (ph) begin
					BR_n   <= 1'b0;
					tstate <= T_WAIT_BG;
				end

			T_WAIT_BG:
				// Wait for the grant and a quiet bus.
				if (ph && !BG_n && AS_n_i && DTACK_n) begin
					BGACK_n <= 1'b0;
					BR_n    <= 1'b1;
					holding <= 1'b1;
					tstate  <= T_TAKE;
				end

			T_TAKE:
				if (ph) tstate <= T_UNIT;

			T_UNIT:
				if (ph) begin
					if (!is_video && vid_pending) begin
						// Run the sprite command the host just wrote before the
						// next host unit can overwrite its registers.
						src[which]  <= t_src;
						dst[which]  <= t_dst;
						host_left   <= t_left;
						host_resume <= 1'b1;
						load_video();
					end else if (v_fill && v_local) begin
						tstate <= T_SEND;
					end else begin
						start_read(t_src[23:1]);
					end
				end

			T_READ:
				if (sample_now) begin
					if (v_local) begin
						// Hand the word over, or hold the read until the UM6618
						// has room.
						if (can_send) send_unit();
						else          tstate <= T_SEND;
					end else begin
						bus_wdata <= D_i;
						bus_addr  <= t_dst[23:1];
						bus_write <= 1'b1;
						bus_hold  <= 1'b0;
						req_t     <= ~req_t;
						tstate    <= T_WRITE;
					end
				end

			T_SEND:
				if (ph && can_send) send_unit();

			T_WRITE:
				if (sample_now) begin
					if (is_video) begin
						// Bus-side sprite destination: the UM6618 only steps.
						send_unit();
					end else begin
						t_src <= src_next;
						t_dst <= dst_next;
						if (last_unit) begin
							tstate <= T_RELEASE;
						end else begin
							t_left <= t_left - 16'd1;
							if (vid_pending) tstate <= T_UNIT;
							else             start_read(src_next[23:1]);
						end
					end
				end

			T_CLOSE:
				// All units acknowledged: one more token closes the command and
				// the pin is an interrupt again two phases on.
				if (ph && ((in_flight - {1'b0, ack_seen}) == 2'd0)) begin
					fwd       <= ~fwd;
					tok_guard <= 2'd2;
					vregs[95:80] <= t_src[15:0];
					vregs[79:64] <= t_src[31:16];
					vregs[47:32] <= t_dst[15:0];
					vregs[31:16] <= t_dst[31:16];
					if (host_resume) begin
						// Keep the grant while the suspended host stream resumes.
						is_video    <= 1'b0;
						host_resume <= 1'b0;
						t_src       <= src[which];
						t_dst       <= dst[which];
						t_left      <= host_left;
						t_ctrl      <= ctrl[which];
						tstate      <= T_UNIT;
					end else begin
						tstate <= T_RELEASE;
					end
				end

			T_RELEASE:
				if (ph) begin
					if (!is_video) begin
						src[which] <= t_src;
						dst[which] <= t_dst;
					end
					BGACK_n <= 1'b1;
					holding <= 1'b0;
					tstate  <= T_IDLE;
				end

			default: tstate <= T_IDLE;
			endcase
		end
	end

	// ---- savestate ---------------------------------------------------------

	// Nine words.
	//
	//   191:0    src, dst, count and ctrl for both channels
	//   240:192  bus machine and the pins it drives
	//   447:241  transfer machine
	localparam int SS_DMA_WORDS = 9;
	localparam int SS_BUS_LO    = 192;
	localparam int SS_BUS_BITS  = 49;
	localparam int SS_XFER_LO   = SS_BUS_LO + SS_BUS_BITS;
	localparam int SS_XFER_BITS = 207;
	localparam int SS_PAD_BITS  = SS_DMA_WORDS*64 - SS_XFER_LO - SS_XFER_BITS;

	wire [SS_BUS_BITS-1:0] ss_bus_live =
		{bus_state, ack_t, consumed_t, AS_n_o, UDS_n_o, LDS_n_o, RW_o, D_o, D_oe, A_o};
	wire [SS_XFER_BITS-1:0] ss_xfer_live =
		{tstate, which, is_video, host_resume, host_left, t_src, t_dst, t_left, t_ctrl,
		 t_src_inc, t_dst_inc, in_flight, rev_q, sync_pending, tok_guard, fwd, tok_active,
		 vid_pending, vid_wr_q, req_t, consume_t, bus_write, bus_hold, bus_addr, bus_wdata,
		 BR_n, BGACK_n, holding};

	wire [SS_DMA_WORDS*64-1:0] ss_saved;
	wire [SS_DMA_WORDS*64-1:0] ss_live = {
		{SS_PAD_BITS{1'b0}}, ss_xfer_live, ss_bus_live,
		ctrl[1], ctrl[0], count[1], count[0], dst[1], dst[0], src[1], src[0] };
	wire [63:0] ss_word_dout [SS_DMA_WORDS];

	genvar gw;
	generate
		for (gw = 0; gw < SS_DMA_WORDS; gw++) begin : g_ss_words
			ss_reg #(.ADDR(SSW_6619_DMA + gw)) word (
				.clk_i      (clk_sys),
				.bus_din_i  (ss_din),
				.bus_addr_i (ss_addr),
				.bus_wren_i (ss_wren),
				.bus_rst_i  (1'b0),
				.bus_dout_o (ss_word_dout[gw]),
				.din_i      (ss_live[gw*64 +: 64]),
				.dout_o     (ss_saved[gw*64 +: 64])
			);
		end
	endgenerate

	logic [63:0] ss_dout_or;
	always_comb begin
		ss_dout_or = 64'd0;
		for (int k = 0; k < SS_DMA_WORDS; k++)
			ss_dout_or = ss_dout_or | ss_word_dout[k];
	end
	assign ss_dout = ss_dout_or;

	// The sprite block from the UM6618's register words 13 and 14, taken as
	// they pass on the state bus.
	/* verilator lint_off PROCASSINIT */
	reg [127:0] vregs_saved = 128'd0;
	/* verilator lint_on PROCASSINIT */
	always @(posedge clk_sys) begin
		if (ss_rst) vregs_saved <= 128'd0;
		else if (ss_wren && ss_addr == 10'(SSW_6618_REGS + 13)) vregs_saved[63:0]   <= ss_din;
		else if (ss_wren && ss_addr == 10'(SSW_6618_REGS + 14)) vregs_saved[127:64] <= ss_din;
	end

	`ifdef VERILATOR
	/* verilator lint_off UNUSEDSIGNAL */
	wire unused = &{1'b0, ss_saved[SS_DMA_WORDS*64-1:SS_XFER_LO+SS_XFER_BITS]};
	/* verilator lint_on UNUSEDSIGNAL */
	`endif
endmodule
