// Copyright (c) 2026 Jamie Blanks

// Free running counter, the source of interrupt level 3.
//
//   $E90014  control    $A2xx selects a timer mode; $A300 counts video frames
//   $E90016  reload     the period, as n
//   $E90018  current    read back while it runs
//
// Modes: 0 fires once a second and ignores the period, 1 counts
// 1024 x (n+1) input ticks, $F counts 9040 x (n+1). $A300 counts n+1 video
// ticks (the REBEL text crawl). Other A3 modes are unknown and disabled.
//
// The counter ticks on CPU_CLK, 10.738635 MHz, this chip's only clock.

module um6619_frc (
	input  wire        clk_sys,
	input  wire        reset,
	input  wire        ce_phi1,
	input  wire        vblank_tick,

	input  wire        ctrl_wr,
	input  wire [15:0] ctrl_din,
	output wire [15:0] ctrl_dout,

	input  wire        reload_wr,
	input  wire [15:0] reload_din,
	output wire [15:0] reload_dout,

	output wire [15:0] value_dout,

	output reg         irq,          // level held until acknowledged
	input  wire        irq_ack,

	// Savestate words.
	input  wire [63:0] ss_din,
	input  wire [9:0]  ss_addr,
	input  wire        ss_wren,
	input  wire        ss_rst,
	output wire [63:0] ss_dout
);
	`include "ss_map.svh"

	localparam int TICKS_PER_SECOND = 10_738_635;  // CPU_CLK, MCLK / 5
	localparam int MODE1_TICKS      = 1024;
	localparam int MODEF_TICKS      = 9040;

	reg [15:0] ctrl;
	reg [15:0] reload;
	reg [15:0] value;
	reg [23:0] sub;      // ticks inside the current unit of the chosen mode

	wire [3:0] mode = ctrl[3:0];
	wire mode_ok = (mode == 4'd0) | (mode == 4'd1) | (mode == 4'hF);
	wire frame_mode = (ctrl == 16'hA300);
	wire enabled = ((ctrl[15:8] == 8'hA2) & mode_ok) | frame_mode;

	// Mode 0 is one second regardless of the period register.
	wire [23:0] sub_limit = (mode == 4'd0) ? 24'(TICKS_PER_SECOND - 1)
	                      : (mode == 4'hF) ? 24'(MODEF_TICKS - 1)
	                                       : 24'(MODE1_TICKS - 1);

	assign ctrl_dout   = ctrl;
	assign reload_dout = reload;
	assign value_dout  = value;

	wire [63:0] ss_now0, ss_now1;
	wire [63:0] ss_dout0, ss_dout1;
	assign ss_dout = ss_dout0 | ss_dout1;

	ss_reg #(.ADDR(SSW_6619_FRC), .DEFAULT(64'd0)) w0
	(
		.clk_i(clk_sys), .bus_din_i(ss_din), .bus_addr_i(ss_addr),
		.bus_wren_i(ss_wren), .bus_rst_i(ss_rst), .bus_dout_o(ss_dout0),
		.din_i({16'd0, value, reload, ctrl}), .dout_o(ss_now0)
	);

	ss_reg #(.ADDR(SSW_6619_FRC + 1), .DEFAULT(64'd0)) w1
	(
		.clk_i(clk_sys), .bus_din_i(ss_din), .bus_addr_i(ss_addr),
		.bus_wren_i(ss_wren), .bus_rst_i(ss_rst), .bus_dout_o(ss_dout1),
		.din_i({39'd0, irq, sub}), .dout_o(ss_now1)
	);

	always @(posedge clk_sys) begin
		if (reset) begin
			ctrl   <= ss_now0[15:0];
			reload <= ss_now0[31:16];
			value  <= ss_now0[47:32];
			sub    <= ss_now1[23:0];
			irq    <= ss_now1[24];
		end else begin
			if (ctrl_wr) begin
				ctrl <= ctrl_din;
				// Starting the counter loads it from the period register.
				if ((ctrl_din[15:8] == 8'hA2) || (ctrl_din == 16'hA300)) begin
					value <= reload;
					sub   <= 24'd0;
				end
			end

			if (reload_wr) reload <= reload_din;

			if (irq_ack) irq <= 1'b0;

			if (enabled && !ctrl_wr && (frame_mode ? vblank_tick : ce_phi1)) begin
				if (frame_mode || sub == sub_limit) begin
					sub <= 24'd0;
					if (!frame_mode && mode == 4'd0) begin
						irq <= 1'b1;
					end else if (value == 16'd0) begin
						value <= reload;
						irq   <= 1'b1;
					end else begin
						value <= value - 16'd1;
					end
				end else begin
					sub <= sub + 24'd1;
				end
			end
		end
	end

	`ifdef VERILATOR
	/* verilator lint_off UNUSEDSIGNAL */
	wire unused = &{1'b0, ss_now0[63:48], ss_now1[63:25], ctrl[7:4]};
	/* verilator lint_on UNUSEDSIGNAL */
	`endif
endmodule
