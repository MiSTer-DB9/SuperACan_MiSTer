// Copyright (c) 2026 Jamie Blanks

// The console: PCGAM 16000-2A mainboard, cartridge slot, pads and reset
// button. Everything above this is MiSTer framework; everything below it is
// a part with a number printed on it.

module superacan_console #(
	parameter bit BIOS_BYPASS = 1'b0
) (
	input  wire        clk_sys,
	input  wire        cold_reset,
	input  wire        reset_btn_n,

	// Pads, buttons high when pressed. multitap: 0 = one pad per port,
	// 1 = four pads on port 1, 2 = four on port 1 and two on port 2.
	// Pad n of a port sits at bits [12n+11:12n].
	input  wire [1:0]  multitap,
	input  wire [47:0] pad1,
	input  wire [23:0] pad2,

	// Savestate park and state bus.
	input  wire        pause_cpu,
	input  wire        pause_vid,
	output wire        ss_cpu_ready,
	output wire        ss_snd_ready,

	output wire        vid_overscan,

	input  wire        ss_sndram_own,
	input  wire [15:0] ss_sndram_addr,
	input  wire        ss_sndram_we,
	input  wire [7:0]  ss_sndram_wdata,
	output wire [7:0]  ss_sndram_q,
	output wire        ss_mem_idle,
	output wire        ss_vid_ready,

	input  wire [63:0] ss_din,
	input  wire [9:0]  ss_addr,
	input  wire        ss_wren,
	input  wire        ss_rst,
	// Restore window; the CPUs load their state on this rather than on reset.
	input  wire        ss_load,
	output wire [63:0] ss_dout,

	input  wire        ss_wram_own,
	input  wire [14:0] ss_wram_addr,
	input  wire        ss_wram_we,
	input  wire [1:0]  ss_wram_be,
	input  wire [15:0] ss_wram_wdata,
	output wire [15:0] ss_wram_q,

	input  wire        ss_vram_own,
	input  wire [15:0] ss_vram_addr,
	input  wire        ss_vram_we,
	input  wire [1:0]  ss_vram_be,
	input  wire [15:0] ss_vram_wdata,
	output wire [15:0] ss_vram_q,

	input  wire        ss_pal_own,
	input  wire [7:0]  ss_pal_addr,
	input  wire        ss_pal_we,
	input  wire [1:0]  ss_pal_be,
	input  wire [15:0] ss_pal_wdata,
	output wire [15:0] ss_pal_q,

	input  wire        ss_cartram_own,
	input  wire [14:0] ss_cartram_addr,
	input  wire        ss_cartram_we,
	input  wire  [7:0] ss_cartram_wdata,
	output wire  [7:0] ss_cartram_q,

	// Cartridge battery SRAM save file.
	input  wire [13:0] bk_addr,
	input  wire        bk_wren,
	input  wire  [1:0] bk_be,
	input  wire [15:0] bk_wdata,
	output wire [15:0] bk_q,
	output wire        bk_dirty,

	// MiSTer cheats.
	input  wire        cheat_load_begin,
	input  wire        cheat_invalidate,
	input  wire        cheat_commit_req,
	input  wire        cheat_paused,
	input  wire [128:0] cheat_code,
	output wire        cheat_commit_done,

	// BIOS load path to the UM6619 ROMs and the cartridge UM6650.
	input  wire        ld_wr,
	input  wire [14:0] ld_addr,
	input  wire [15:0] ld_data,

	// Cartridge slot SDRAM client.
	output wire        cart_mem_req,
	output wire [25:0] cart_mem_addr,
	input  wire [15:0] cart_mem_dout,
	input  wire        cart_mem_busy,
	input  wire        cart_mem_done,

	input  wire        cart_present,
	input  wire [24:0] rom_size,

	// Video, in place of the KA2195D analog output.
	output wire        ce_pix,
	output wire [7:0]  vid_r,
	output wire [7:0]  vid_g,
	output wire [7:0]  vid_b,
	output wire        vid_hs,
	output wire        vid_vs,
	output wire        vid_hb,
	output wire        vid_vb,
	output wire        vid_de,

	// Signed stereo from the UM6619.
	output wire [15:0] audio_l,
	output wire [15:0] audio_r
);
	// J6, the cartridge connector.
	wire [23:1] cart_a;
	wire [15:0] cart_bus_d_o, cart_board_d_o;
	wire        cart_bus_d_oe, cart_board_d_oe;
	wire        cart_as_n, cart_rw, cart_26, cart_27;
	wire        cart_ready;
	/* verilator lint_off UNUSEDSIGNAL */
	wire        cart_uds_n, cart_lds_n, cart_reset_n, cart_mclk, cart_cpuclk;
	wire        cart_66;
	wire        joy1_1, joy1_7, joy2_1, joy2_7;
	wire        joy1_3, joy2_3;

	wire [71:0] ss_pad1_out;
	wire [39:0] ss_pad2_out;
	wire [63:0] ss_pad_saved, ss_tap_saved, ss_dout_pad, ss_dout_tap;

	`include "ss_map.svh"

	wire [3:0] port1_pads = (multitap == 2'd0) ? 4'd1 : 4'd4;
	wire [3:0] port2_pads = (multitap == 2'd2) ? 4'd2 : 4'd1;

	multitap #(.MAX_PADS(4)) pad_one
	(
		.clk_sys (clk_sys),
		.latch   (joy1_1),
		.clock   (joy1_7),
		.data    (joy1_3),
		.pads    (port1_pads),
		.buttons (pad1),
		.ss_load (ss_load),
		// Word 176: pad 1 of each port, then pad 2 of each. Word 177: port
		// 1's pads 3 and 4 plus both clock counts.
		.ss_in   ({ss_tap_saved[39:32], ss_tap_saved[31:0],
		           ss_pad_saved[47:32], ss_pad_saved[15:0]}),
		.ss_out  (ss_pad1_out)
	);

	multitap #(.MAX_PADS(2)) pad_two
	(
		.clk_sys (clk_sys),
		.latch   (joy2_1),
		.clock   (joy2_7),
		.data    (joy2_3),
		.pads    (port2_pads),
		.buttons (pad2),
		.ss_load (ss_load),
		.ss_in   ({ss_tap_saved[47:40], ss_pad_saved[63:48], ss_pad_saved[31:16]}),
		.ss_out  (ss_pad2_out)
	);

	// Pad shift registers power up all ones, tap clock counts at zero.
	ss_reg #(.ADDR(SSW_TOP_BASE), .DEFAULT(64'hFFFF_FFFF_FFFF_FFFF)) state_pads (
		.clk_i      (clk_sys),
		.bus_din_i  (ss_din),
		.bus_addr_i (ss_addr),
		.bus_wren_i (ss_wren),
		.bus_rst_i  (ss_rst),
		.bus_dout_o (ss_dout_pad),
		.din_i      ({ss_pad2_out[31:16], ss_pad1_out[31:16],
		              ss_pad2_out[15:0],  ss_pad1_out[15:0]}),
		.dout_o     (ss_pad_saved)
	);

	ss_reg #(.ADDR(SSW_TOP_BASE + 1), .DEFAULT(64'h0000_0000_FFFF_FFFF)) state_taps (
		.clk_i      (clk_sys),
		.bus_din_i  (ss_din),
		.bus_addr_i (ss_addr),
		.bus_wren_i (ss_wren),
		.bus_rst_i  (ss_rst),
		.bus_dout_o (ss_dout_tap),
		.din_i      ({16'd0, ss_pad2_out[39:32], ss_pad1_out[71:64],
		              ss_pad1_out[63:32]}),
		.dout_o     (ss_tap_saved)
	);
	/* verilator lint_on UNUSEDSIGNAL */

	wire [15:0] cart_d = cart_bus_d_oe   ? cart_bus_d_o
	                   : cart_board_d_oe ? cart_board_d_o
	                                     : 16'hFFFF;

	// Match the mainboard's readback latency.
	wire [63:0] ss_dout_board, ss_dout_cart;
	reg [63:0] ss_dout_cart_q = 64'd0, ss_dout_pad_q = 64'd0;
	always @(posedge clk_sys) begin
		ss_dout_cart_q <= ss_dout_cart;
		ss_dout_pad_q <= ss_dout_pad | ss_dout_tap;
	end
	assign ss_dout = ss_dout_board | ss_dout_cart_q | ss_dout_pad_q;

	mainboard #(.BIOS_BYPASS(BIOS_BYPASS)) board
	(
		.clk_sys       (clk_sys),
		.cold_reset    (cold_reset),
		.reset_btn_n   (reset_btn_n),

		.pause_cpu     (pause_cpu),
		.pause_vid     (pause_vid),

		.cheat_load_begin  (cheat_load_begin),
		.cheat_invalidate  (cheat_invalidate),
		.cheat_commit_req  (cheat_commit_req),
		.cheat_paused      (cheat_paused),
		.cheat_code        (cheat_code),
		.cheat_commit_done (cheat_commit_done),
		.ss_cpu_ready  (ss_cpu_ready),
		.ss_snd_ready  (ss_snd_ready),
		.vid_overscan  (vid_overscan),
		.ss_sndram_own   (ss_sndram_own),
		.ss_sndram_addr  (ss_sndram_addr),
		.ss_sndram_we    (ss_sndram_we),
		.ss_sndram_wdata (ss_sndram_wdata),
		.ss_sndram_q     (ss_sndram_q),
		.ss_vid_ready  (ss_vid_ready),

		.ss_din        (ss_din),
		.ss_addr       (ss_addr),
		.ss_wren       (ss_wren),
		.ss_rst        (ss_rst),
		.ss_load       (ss_load),
		.ss_dout       (ss_dout_board),

		.ld_wr         (ld_wr),
		.ld_addr       (ld_addr),
		.ld_data       (ld_data),

		.ss_wram_own   (ss_wram_own),
		.ss_wram_addr  (ss_wram_addr),
		.ss_wram_we    (ss_wram_we),
		.ss_wram_be    (ss_wram_be),
		.ss_wram_wdata (ss_wram_wdata),
		.ss_wram_q     (ss_wram_q),

		.ss_vram_own   (ss_vram_own),
		.ss_vram_addr  (ss_vram_addr),
		.ss_vram_we    (ss_vram_we),
		.ss_vram_be    (ss_vram_be),
		.ss_vram_wdata (ss_vram_wdata),
		.ss_vram_q     (ss_vram_q),

		.ss_pal_own    (ss_pal_own),
		.ss_pal_addr   (ss_pal_addr),
		.ss_pal_we     (ss_pal_we),
		.ss_pal_be     (ss_pal_be),
		.ss_pal_wdata  (ss_pal_wdata),
		.ss_pal_q      (ss_pal_q),

		.CART_A        (cart_a),
		.CART_D_i      (cart_d),
		.CART_D_o      (cart_bus_d_o),
		.CART_D_oe     (cart_bus_d_oe),
		.CART_AS_n     (cart_as_n),
		.CART_UDS_n    (cart_uds_n),
		.CART_LDS_n    (cart_lds_n),
		.CART_RW       (cart_rw),
		.CART_RESET_n  (cart_reset_n),
		.CART_MCLK     (cart_mclk),
		.CART_CPUCLK   (cart_cpuclk),
		.CART_26       (cart_26),
		.CART_27       (cart_27),
		.CART_29       (1'b1),
		.CART_66       (cart_66),
		.CARTIN_n      (~cart_present),
		.cart_ready    (cart_ready),

		.JOY1_1        (joy1_1),
		.JOY1_3        (joy1_3),
		.JOY1_7        (joy1_7),
		.JOY2_1        (joy2_1),
		.JOY2_3        (joy2_3),
		.JOY2_7        (joy2_7),

		.VIDEO_R       (vid_r),
		.VIDEO_G       (vid_g),
		.VIDEO_B       (vid_b),
		.VIDEO_HS      (vid_hs),
		.VIDEO_VS      (vid_vs),
		.VIDEO_HB      (vid_hb),
		.VIDEO_VB      (vid_vb),
		.VIDEO_DE      (vid_de),
		.ce_pix        (ce_pix),

		.AUDIO_L       (audio_l),
		.AUDIO_R       (audio_r)
	);

	cart_slot slot
	(
		.clk_sys     (clk_sys),
		.reset       (~reset_btn_n | cold_reset),
		.present     (cart_present),
		.rom_size    (rom_size),
		.CPU_A       (cart_a),
		.CPU_D_i     (cart_d),
		.CPU_D_o     (cart_board_d_o),
		.CPU_D_oe    (cart_board_d_oe),
		.CPU_AS_n    (cart_as_n),
		.CPU_RW      (cart_rw),
		.CART_26_n   (cart_26),
		.CART_27_n   (cart_27),
		.rom_ready   (cart_ready),
		.mem_req     (cart_mem_req),
		.mem_addr    (cart_mem_addr),
		.mem_dout    (cart_mem_dout),
		.mem_busy    (cart_mem_busy),
		.mem_done    (cart_mem_done),
		// The key is the last sixteen bytes of the BIOS image, at $5000.
		.ld_key_wr   (ld_wr & (ld_addr[14:4] == 11'h500)),
		.ld_key_addr (ld_addr[3:1]),
		.ld_key_data (ld_data),
		.ss_own      (ss_cartram_own),
		.ss_addr_b   (ss_cartram_addr),
		.ss_we       (ss_cartram_we),
		.ss_din_b    (ss_cartram_wdata),
		.ss_dout_b   (ss_cartram_q),
		.bk_addr     (bk_addr),
		.bk_wren     (bk_wren),
		.bk_be       (bk_be),
		.bk_wdata    (bk_wdata),
		.bk_q        (bk_q),
		.bk_dirty    (bk_dirty),
		.ss_din      (ss_din),
		.ss_addr     (ss_addr),
		.ss_wren     (ss_wren),
		.ss_rst      (ss_rst),
		.ss_dout     (ss_dout_cart)
	);

	// Cartridge reads are the only SDRAM traffic inside the console.
	assign ss_mem_idle = ~cart_mem_busy;

endmodule
