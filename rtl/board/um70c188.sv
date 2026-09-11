// Copyright (c) 2026 Jamie Blanks

// U3, UM70C188: 256-entry palette RAM plus triple video DAC. No datasheet
// exists; the register model follows the pin-compatible UM70C171. The
// UM6618 feeds it an 8-bit pixel index on P0-P7 with PCLK and !BLANK, and
// talks to it over D0-D7 with two register selects.
//
//   RS  write                     read
//   00  set the entry to access   -
//   01  colour, low then high     colour, low then high
//   10  pixel mask                pixel mask
//   11  -                         entry being accessed
//
// Entries are xBGR555. Each channel expands to eight bits as {c, c[4:2]}.

module um70c188 (
	input  wire        clk_sys,
	input  wire        reset,

	// Host side, driven by the UM6618.
	input  wire [7:0]  D_i,
	output wire [7:0]  D_o,
	input  wire [1:0]  RS,
	input  wire        RD_n,
	input  wire        WR_n,

	// Pixel side.
	input  wire        PCLK,       // one pulse per dot
	input  wire [7:0]  P,
	input  wire        BLANK_n,

	output wire [7:0]  ROUT,
	output wire [7:0]  GOUT,
	output wire [7:0]  BOUT,

	// Savestate port on the palette, plus its scalar word.
	input  wire        ss_own,
	input  wire [7:0]  ss_addr_b,
	input  wire        ss_we,
	input  wire [1:0]  ss_be,
	input  wire [15:0] ss_wdata,
	output wire [15:0] ss_q,

	input  wire [63:0] ss_din,
	input  wire [9:0]  ss_addr,
	input  wire        ss_wren,
	input  wire        ss_rst,
	output wire [63:0] ss_dout
);
	`include "ss_map.svh"

	reg [7:0] entry;
	reg       phase;      // which byte of the entry the colour port is on
	reg [7:0] mask;
	reg [7:0] rd_hold;

	wire wr = ~WR_n;
	wire rd = ~RD_n;

	// Port A is the host, port B the pixel lookup. The savestate borrows
	// port B while the machine is parked.
	wire [15:0] host_q, pix_q;
	wire [7:0]  pix_index = P & mask;

	wire [7:0]  b_addr  = ss_own ? ss_addr_b : pix_index;
	wire        b_we    = ss_own & ss_we;
	wire [1:0]  b_be    = ss_own ? ss_be : 2'b00;
	wire [15:0] b_wdata = ss_wdata;

	assign ss_q = pix_q;

	// Same-entry write and lookup on one clock is undefined in the block RAM,
	// so the byte going in is forwarded around it. The RAM answers a cycle
	// late, so the collision is latched and merged a cycle late too.
	reg       fw_v;
	reg [1:0] fw_be;
	reg [7:0] fw_data;

	always @(posedge clk_sys) begin
		fw_v    <= a_wren & (entry == b_addr);
		fw_be   <= a_be;
		fw_data <= D_i;
	end

	wire [15:0] pix_fwd = {(fw_v & fw_be[1]) ? fw_data : pix_q[15:8],
	                       (fw_v & fw_be[0]) ? fw_data : pix_q[7:0]};

	wire       a_wren = wr & (RS == 2'b01);
	wire [1:0] a_be   = phase ? 2'b10 : 2'b01;

	cache_ram_dp_be #(.ADDR_WIDTH(8), .DATA_WIDTH(16)) palette
	(
		.clk_i     (clk_sys),
		.addr_a_i  (entry),
		.wren_a_i  (a_wren),
		.be_a_i    (a_be),
		.wdata_a_i ({D_i, D_i}),
		.q_a_o     (host_q),
		.addr_b_i  (b_addr),
		.wren_b_i  (b_we),
		.be_b_i    (b_be),
		.wdata_b_i (b_wdata),
		.q_b_o     (pix_q)
	);

	assign D_o = (RS == 2'b11) ? entry
	           : (RS == 2'b10) ? mask
	                           : rd_hold;

	// Latched colour, black while !BLANK is low.
	reg [14:0] shown;

	wire [4:0] r5 = shown[4:0];
	wire [4:0] g5 = shown[9:5];
	wire [4:0] b5 = shown[14:10];

	assign ROUT = {r5, r5[4:2]};
	assign GOUT = {g5, g5[4:2]};
	assign BOUT = {b5, b5[4:2]};

	wire [63:0] ss_now;
	ss_reg #(.ADDR(SSW_70C188_BASE), .DEFAULT(64'h0000_0000_0000_FF00)) word
	(
		.clk_i(clk_sys), .bus_din_i(ss_din), .bus_addr_i(ss_addr),
		.bus_wren_i(ss_wren), .bus_rst_i(ss_rst), .bus_dout_o(ss_dout),
		.din_i({32'd0, shown, phase, mask, entry}), .dout_o(ss_now)
	);

	reg wr_q, rd_q;
	always @(posedge clk_sys) begin
		wr_q <= wr;
		rd_q <= rd;

		if (PCLK) shown <= BLANK_n ? pix_fwd[14:0] : 15'd0;

		if (reset) begin
			entry <= ss_now[7:0];
			mask  <= ss_now[15:8];
			phase <= ss_now[16];
			shown <= ss_now[31:17];
			rd_hold <= 8'd0;
		end else begin
			if (wr & ~wr_q) begin
				case (RS)
					2'b00: begin entry <= D_i; phase <= 1'b0; end
					2'b01: begin
						phase <= ~phase;
						if (phase) entry <= entry + 8'd1;
					end
					2'b10: mask <= D_i;
					default: ;
				endcase
			end

			if (rd & ~rd_q && (RS == 2'b01)) begin
				rd_hold <= phase ? host_q[15:8] : host_q[7:0];
				phase   <= ~phase;
				if (phase) entry <= entry + 8'd1;
			end
		end
	end

	`ifdef VERILATOR
	/* verilator lint_off UNUSEDSIGNAL */
	wire unused = &{1'b0, ss_now[63:32]};
	/* verilator lint_on UNUSEDSIGNAL */
	`endif
endmodule
