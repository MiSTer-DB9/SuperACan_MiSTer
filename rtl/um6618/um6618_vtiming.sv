// Copyright (c) 2026 Jamie Blanks

// UM6618 horizontal and vertical counters and what derives from them:
// blanking, sync, the readable scanline, the vblank flag and the raster
// comparators.
//
// A line is 3420 master clocks in both width modes, 2560 of them active, so
// both modes run at 15699.76 Hz and 59.9226 Hz.
//
//   H256   |<------ 2560 mclk active ------>|<-- 860 -->|   342 dots
//   H320   |<------ 2560 mclk active ------>|<-- 860 -->|   420 dots
//
// 3420 does not divide by 8, so seventy of the 320-dot line's blanking dots
// run at /8 and thirty at /10. Which dots stretch is not observable, so they
// sit at the end of the back porch.
//
// Sync position inside the blanking is inferred, NTSC-shaped; only the
// blanking length is known. 262 lines, visible region ending at 240. The
// overscan bit crops the displayed height to 224 without changing timing.

module um6618_vtiming (
	input  wire        clk_sys,
	input  wire        ce_pix,
	input  wire        ss_load,
	input  wire [63:0] ss_din,
	output wire [63:0] ss_dout,

	input  wire        h320,        // $F00008 bit 8
	input  wire        overscan,    // $F00008 bit 9: centered 224-line aperture

	output wire        pix_stretch, // next dot runs at the slower rate

	output reg  [9:0]  hpos,        // in dots
	output reg  [8:0]  vpos,        // in lines

	output wire        hblank,
	output wire        vblank,
	output wire        hsync,
	output wire        vsync,
	output wire        csync,
	output wire        blank_n,     // DAC_!BLANK
	output wire        de,          // inside the displayed area

	output wire        line_start   // ce_pix at the first dot of a line
);
	// Horizontal, H256: 342 dots of MCLK/10.
	localparam int H256_TOTAL   = 342;
	localparam int H256_ACTIVE  = 256;
	localparam int H256_SYNC    = 278;
	localparam int H256_SYNC_E  = 304;

	// Horizontal, H320: 420 dots, the last thirty of them stretched.
	localparam int H320_TOTAL   = 420;
	localparam int H320_ACTIVE  = 320;
	localparam int H320_SYNC    = 347;
	localparam int H320_SYNC_E  = 379;
	localparam int H320_STRETCH = 390;

	localparam int V_TOTAL   = 262;
	localparam int V_VISIBLE = 240;
	localparam int V_CROP    = 8;      // 224 lines, centred, when overscan is off
	localparam int V_SYNC    = 243;
	localparam int V_SYNC_E  = 246;

	wire [9:0] h_total  = h320 ? 10'(H320_TOTAL)  : 10'(H256_TOTAL);
	wire [9:0] h_active = h320 ? 10'(H320_ACTIVE) : 10'(H256_ACTIVE);
	wire [9:0] h_sync   = h320 ? 10'(H320_SYNC)   : 10'(H256_SYNC);
	wire [9:0] h_sync_e = h320 ? 10'(H320_SYNC_E) : 10'(H256_SYNC_E);

	// A live width change can put the beam past the new line's last dot.
	wire h_last = (hpos >= h_total - 10'd1);
	wire v_last = (vpos == 9'(V_TOTAL - 1));

	assign line_start  = ce_pix & (hpos == 10'd0);
	assign ss_dout = {45'd0, vpos, hpos};

	// The divider samples this on the dot before, so it is raised and
	// dropped one dot early.
	assign pix_stretch = h320 & (hpos >= 10'(H320_STRETCH - 1))
	                          & (hpos <  10'(H320_TOTAL - 1));

	assign hblank  = (hpos >= h_active);
	assign vblank  = (vpos >= 9'(V_VISIBLE));
	assign hsync   = (hpos >= h_sync) & (hpos < h_sync_e);
	assign vsync   = (vpos >= 9'(V_SYNC)) & (vpos < 9'(V_SYNC_E));

	// No equalising or serration pulses; the framework makes its own.
	assign csync   = hsync ^ vsync;
	assign blank_n = ~(hblank | vblank);

	wire v_shown = ~overscan ? ~vblank
	                        : (vpos >= 9'(V_CROP))
	                        & (vpos < 9'(V_VISIBLE - V_CROP));
	assign de = ~hblank & v_shown;

	always @(posedge clk_sys) begin
		if (ss_load) begin
			hpos      <= ss_din[9:0];
			vpos      <= ss_din[18:10];
		end else if (ce_pix) begin
			if (h_last) begin
				hpos <= 10'd0;
				if (v_last) begin
					vpos      <= 9'd0;
				end else begin
					vpos <= vpos + 9'd1;
				end
			end else begin
				hpos <= hpos + 10'd1;
			end
		end
	end
endmodule
