//============================================================================
//
//  Funtech Super A'Can for MiSTer.
//
//  This program is free software; you can redistribute it and/or modify it
//  under the terms of the GNU General Public License as published by the Free
//  Software Foundation; either version 2 of the License, or (at your option)
//  any later version.
//
//  This program is distributed in the hope that it will be useful, but WITHOUT
//  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
//  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
//  more details.
//
//  You should have received a copy of the GNU General Public License along
//  with this program; if not, write to the Free Software Foundation, Inc.,
//  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
//
//============================================================================
//
// Framework glue. The console is `superacan_console`; everything here is the
// MiSTer side: HPS, PLL, load paths, SDRAM controller, video and audio.
//
//============================================================================

module emu
(
	//Master input clock
	input         CLK_50M,

	//Async reset from top-level module.
	//Can be used as initial reset.
	input         RESET,

	//Must be passed to hps_io module
	inout  [45:0] HPS_BUS,

	//Base video clock. Usually equals to CLK_SYS.
	output        CLK_VIDEO,

	//Multiple resolutions are supported using different CE_PIXEL rates.
	//Must be based on CLK_VIDEO
	output        CE_PIXEL,

	//Video aspect ratio for HDMI. Most retro systems have ratio 4:3.
	//if VIDEO_ARX[12] or VIDEO_ARY[12] is set then [11:0] contains scaled size instead of aspect ratio.
	output [12:0] VIDEO_ARX,
	output [12:0] VIDEO_ARY,

	output  [7:0] VGA_R,
	output  [7:0] VGA_G,
	output  [7:0] VGA_B,
	output        VGA_HS,
	output        VGA_VS,
	output        VGA_DE,    // = ~(VBlank | HBlank)
	output        VGA_F1,
	output [1:0]  VGA_SL,
	output        VGA_SCALER, // Force VGA scaler
	output        VGA_DISABLE, // analog out is off

	input  [11:0] HDMI_WIDTH,
	input  [11:0] HDMI_HEIGHT,
	output        HDMI_FREEZE,
	output        HDMI_BLACKOUT,
	output        HDMI_BOB_DEINT,

`ifdef MISTER_FB
	// Use framebuffer in DDRAM
	// FB_FORMAT:
	//    [2:0] : 011=8bpp(palette) 100=16bpp 101=24bpp 110=32bpp
	//    [3]   : 0=16bits 565 1=16bits 1555
	//    [4]   : 0=RGB  1=BGR (for 16/24/32 modes)
	//
	// FB_STRIDE either 0 (rounded to 256 bytes) or multiple of pixel size (in bytes)
	output        FB_EN,
	output  [4:0] FB_FORMAT,
	output [11:0] FB_WIDTH,
	output [11:0] FB_HEIGHT,
	output [31:0] FB_BASE,
	output [13:0] FB_STRIDE,
	input         FB_VBL,
	input         FB_LL,
	output        FB_FORCE_BLANK,

`ifdef MISTER_FB_PALETTE
	// Palette control for 8bit modes.
	// Ignored for other video modes.
	output        FB_PAL_CLK,
	output  [7:0] FB_PAL_ADDR,
	output [23:0] FB_PAL_DOUT,
	input  [23:0] FB_PAL_DIN,
	output        FB_PAL_WR,
`endif
`endif

	output        LED_USER,  // 1 - ON, 0 - OFF.

	// b[1]: 0 - LED status is system status OR'd with b[0]
	//       1 - LED status is controled solely by b[0]
	// hint: supply 2'b00 to let the system control the LED.
	output  [1:0] LED_POWER,
	output  [1:0] LED_DISK,

	// I/O board button press simulation (active high)
	// b[1]: user button
	// b[0]: osd button
	output  [1:0] BUTTONS,

	input         CLK_AUDIO, // 24.576 MHz
	output [15:0] AUDIO_L,
	output [15:0] AUDIO_R,
	output        AUDIO_S,   // 1 - signed audio samples, 0 - unsigned
	output  [1:0] AUDIO_MIX, // 0 - no mix, 1 - 25%, 2 - 50%, 3 - 100% (mono)

	//ADC
	inout   [3:0] ADC_BUS,

	//SD-SPI
	output        SD_SCK,
	output        SD_MOSI,
	input         SD_MISO,
	output        SD_CS,
	input         SD_CD,

	//High latency DDR3 RAM interface
	//Use for non-critical time purposes
	output        DDRAM_CLK,
	input         DDRAM_BUSY,
	output  [7:0] DDRAM_BURSTCNT,
	output [28:0] DDRAM_ADDR,
	input  [63:0] DDRAM_DOUT,
	input         DDRAM_DOUT_READY,
	output        DDRAM_RD,
	output [63:0] DDRAM_DIN,
	output  [7:0] DDRAM_BE,
	output        DDRAM_WE,

	//SDRAM interface with lower latency
	output        SDRAM_CLK,
	output        SDRAM_CKE,
	output [12:0] SDRAM_A,
	output  [1:0] SDRAM_BA,
	inout  [15:0] SDRAM_DQ,
	output        SDRAM_DQML,
	output        SDRAM_DQMH,
	output        SDRAM_nCS,
	output        SDRAM_nCAS,
	output        SDRAM_nRAS,
	output        SDRAM_nWE,

`ifdef MISTER_DUAL_SDRAM
	//Secondary SDRAM
	//Set all output SDRAM_* signals to Z ASAP if SDRAM2_EN is 0
	input         SDRAM2_EN,
	output        SDRAM2_CLK,
	output [12:0] SDRAM2_A,
	output  [1:0] SDRAM2_BA,
	inout  [15:0] SDRAM2_DQ,
	output        SDRAM2_nCS,
	output        SDRAM2_nCAS,
	output        SDRAM2_nRAS,
	output        SDRAM2_nWE,
`endif

	input         UART_CTS,
	output        UART_RTS,
	input         UART_RXD,
	output        UART_TXD,
	output        UART_DTR,
	input         UART_DSR,

	// Open-drain User port.
	// 0 - D+/RX
	// 1 - D-/TX
	// 2..6 - USR2..USR6
	// Set USER_OUT to 1 to read from USER_IN.
	// [MiSTer-DB9 BEGIN] - DB9/SNAC8 support: USER_OSD + per-pin push-pull mask, USER_IO widened to 8 bits
	output        USER_OSD,
	output  [7:0] USER_PP,
	input   [7:0] USER_IN,
	output  [7:0] USER_OUT,
	// [MiSTer-DB9 END]

	input         OSD_STATUS
);

///////// Ports this core does not use /////////

assign ADC_BUS  = 'Z;
// [MiSTer-DB9 BEGIN] - DB9/SNAC8 support: USER_PP driven by wrapper; USER_OUT driven by joydb (USER_OUT_DRIVE) below
assign USER_PP = USER_PP_DRIVE;
// [MiSTer-DB9 END]
assign {UART_RTS, UART_TXD, UART_DTR} = 0;
assign {SD_SCK, SD_MOSI, SD_CS} = 'Z;

assign VGA_F1 = 0;
assign VGA_SCALER  = 0;
assign VGA_DISABLE = 0;
assign HDMI_FREEZE = 0;
// Black frames while the scaler re-fits a new picture size, which happens
// when a 320-wide game takes over from the 256-wide BIOS.
assign HDMI_BLACKOUT = 1;
assign HDMI_BOB_DEINT = 0;

assign AUDIO_MIX = 0;

assign LED_POWER = 0;

//////////////////////////////////////////////////////////////////

wire [1:0] ar = status[122:121];

`include "build_id.v"
localparam CONF_STR = {
	"SuperAcan;SS3E000000:100000;",
	"FS1,BIN,Load Cartridge;",
	"-;",
	"C,Cheats;",
	"-;",
	"d0O[19:18],Savestate slot,1,2,3,4;",
	"d0R[20],Save state (Alt-F1);",
	"d0R[21],Restore state (F1);",
	"-;",
	"P1,Audio & Video;",
	"P1-;",
	"P1O[122:121],Aspect ratio,Original,Full Screen,[ARC1],[ARC2];",
	"P1O[16:15],Crop,224,240,Auto;",
	"P1O[27:25],Scale,Normal,V-Integer,Narrower HV-Integer,Wider HV-Integer;",
	"P1O[24:22],Scandoubler Fx,None,HQ2x,CRT 25%,CRT 50%,CRT 75%;",
	"P2,Advanced;",
	"P2-;",
	"P2O[12],Autosave,On,Off;",
	"h1P2T[13],Save Backup RAM;",
	"h1P2T[14],Restore Backup RAM;",
	"P2-;",
	"P2O[31:30],Cartridge byte order,Auto,Word-swapped,Big-endian;",
	"-;",
	"O[29:28],Multitap,Off,Port 1,Both;",
	"-;",
	// [MiSTer-DB9-Pro BEGIN] - Saturn-first joy_type (canonical bit notation)
	"O[127:126],UserIO Joystick,Off,Saturn,DB9MD,DB15;",
	"O[125],UserIO Players, 1 Player,2 Players;",
	// [MiSTer-DB9-Pro END]
	"R[0],Reset;",
	"-;",
	"J1,A,B,X,Y,L,R,Select,Start,Savestates;",
	"jn,A,B,X,Y,L,R,Select,Start;",
	"I,",
	"-,",
	"-,",
	"-,",
	"-,",
	"Slot=L/R Load=U Save=D,",
	"Active Slot 1,",
	"Active Slot 2,",
	"Active Slot 3,",
	"Active Slot 4,",
	"Save to state 1,",
	"Restore state 1,",
	"Save to state 2,",
	"Restore state 2,",
	"Save to state 3,",
	"Restore state 3,",
	"Save to state 4,",
	"Restore state 4;",
	"v,0;",
	"V,v",`BUILD_DATE
};

wire        forced_scandoubler;
wire [21:0] gamma_bus;
wire  [1:0] buttons;
wire [127:0] status;
wire [31:0] joystick_0, joystick_1, joystick_2, joystick_3, joystick_4, joystick_5;
wire [10:0] ps2_key;

wire        ioctl_download;
wire        ioctl_wr;
wire [26:0] ioctl_addr;
wire [15:0] ioctl_dout;
wire [15:0] ioctl_index;
wire        ioctl_wait;
wire        cart_ioctl_wait;
wire        cheat_wait;

// Save file, moved 512 bytes at a time through the 16-bit sd_buff.
wire        img_mounted, img_readonly;
wire [63:0] img_size;
wire [31:0] sd_lba;
wire        sd_rd, sd_wr, sd_ack;
wire [12:0] sd_buff_addr;
wire [15:0] sd_buff_dout;
wire        sd_buff_wr;
wire        bk_writable, bk_pending, bk_busy, bk_dirty;
wire        bk_loading;
wire [15:0] bk_q;

// [MiSTer-DB9 BEGIN] - DB9/SNAC8 support: joydb wrapper
wire         CLK_JOY = CLK_50M;                 // Assign clock between 40-50Mhz
wire   [1:0] joy_type_raw    = status[127:126]; // 0=Off, 1=Saturn, 2=DB9MD, 3=DB15
wire         joy_2p          = status[125];
// SNAC cores: replace 1'b0 with the core's SNAC enable expression so SNAC
// preempts the joydb wrapper on shared USER_IO pins. Default 1'b0 is no-op.
wire         snac_active     = 1'b0;
// MT32-pi cores on primary USER_IO: replace 1'b0 with the core's MT32-active
// expression (e.g. `mt32_use` under `ifndef SECOND_MT32`, `~mt32_disable` for
// TRS-80's inverted polarity). Suppresses the OSD-open autodetect probe so it
// doesn't read the RPi's I2C master traffic as a ghost Saturn signature.
wire         mt32_primary_active = 1'b0;
wire   [1:0] joy_type        = snac_active ? 2'd0 : joy_type_raw;
wire         joy_db9md_en    = (joy_type == 2'd2);
wire         joy_db15_en     = (joy_type == 2'd3);
wire         joy_any_en      = |joy_type;
// [MiSTer-DB9 END]

// [MiSTer-DB9-Pro BEGIN] - Saturn key gate
wire         saturn_unlocked;                   // driven by hps_io UIO_DB9_KEY (0xFE)
// [MiSTer-DB9-Pro END]

// [MiSTer-DB9 BEGIN] - DB9/SNAC8 support: joydb wrapper wires + instance
wire   [7:0] USER_OUT_DRIVE;
wire   [7:0] USER_PP_DRIVE;
wire  [15:0] joydb_1, joydb_2;
wire         joydb_1ena, joydb_2ena;
wire  [15:0] joy_raw_payload;
// Programmable-remap matrix (joydb_remap inside joydb): clk_sys carries the
// 0xFD selector load (HPS-bus domain). joydb_*_mapped are the MiSTer-standard
// joystick words the core consumes at its joy0_USB merge point. db9_remap_*
// are driven by the hps_io instance (see the .db9_remap_* bindings there).
// Until a core consumes joydb_*_mapped (Layer B), the matrix stays at identity
// and synthesis prunes it -- binding these is dormant + fleet-safe.
wire  [15:0] joydb_1_mapped, joydb_2_mapped;
wire         db9_remap_cmd;
wire   [5:0] db9_remap_byte_cnt;
wire  [15:0] db9_remap_din;

// [MiSTer-DB9 BEGIN] - DB9 remap factory default (used until Main_MiSTer streams UIO 0xFD)
// Derived from CONF_STR J1, same rule as db9_map.cpp; lets the core work on a stock MiSTer binary.
// DB15:  A=A, B=B, X=C, Y=D, L=E, R=F, Select=SELECT, Start=START, Savestates=-
// DB9MD: A=A, B=B, X=X, Y=Y, L=-, R=-, Select=MODE, Start=START, Savestates=C
wire  [35:0] db9_remap_default_db15  = 36'hFAB987654;
wire  [35:0] db9_remap_default_db9md = 36'h6ABFF8754;
// [MiSTer-DB9 END]
joydb joydb (
  .clk             ( CLK_JOY         ),
  .clk_sys         ( clk_sys            ),
  .USER_IN         ( USER_IN         ),
  .OSD_STATUS          ( OSD_STATUS          ),
  .snac_active         ( snac_active         ),
  .mt32_primary_active ( mt32_primary_active ),
  .joy_type        ( joy_type        ),
  .joy_2p          ( joy_2p          ),
  .saturn_unlocked ( saturn_unlocked ),
  .USER_OUT_DRIVE  ( USER_OUT_DRIVE  ),
  .USER_PP_DRIVE   ( USER_PP_DRIVE   ),
  .USER_OSD        ( USER_OSD        ),
  .joydb_1         ( joydb_1         ),
  .joydb_2         ( joydb_2         ),
  .joydb_1ena      ( joydb_1ena      ),
  .joydb_2ena      ( joydb_2ena      ),
  .remap_cmd       ( db9_remap_cmd      ),
  .remap_byte_cnt  ( db9_remap_byte_cnt ),
  .remap_din       ( db9_remap_din      ),
  .remap_default_db15  ( db9_remap_default_db15  ),
  .remap_default_db9md ( db9_remap_default_db9md ),
  .joydb_1_mapped  ( joydb_1_mapped     ),
  .joydb_2_mapped  ( joydb_2_mapped     ),
  .joy_raw         ( joy_raw_payload )
);

assign USER_OUT = USER_OUT_DRIVE;
// [MiSTer-DB9 END]

// [MiSTer-DB9 BEGIN] - DB9/SNAC8 support: rename USB joystick wires
wire [31:0] joystick_0_USB, joystick_1_USB;
// [MiSTer-DB9 END]

hps_io #(.CONF_STR(CONF_STR), .WIDE(1)) hps_io
(
	.clk_sys(clk_sys),
	.HPS_BUS(HPS_BUS),
	.EXT_BUS(),
	.gamma_bus(gamma_bus),

	.forced_scandoubler(forced_scandoubler),

	.buttons(buttons),
	.status(status),
	.status_in({status[127:20], ss_slot, status[17:0]}),
	.status_set(ss_status_update),
	// Active high: the entry shows when its bit is set.
	.status_menumask({14'd0, bk_writable, ss_menu_enabled}),

	.info_req(ss_info_req),
	.info(ss_info),

	.joystick_0(joystick_0_USB),
	.joystick_1(joystick_1_USB),
	.joystick_2(joystick_2),
	.joystick_3(joystick_3),
	.joystick_4(joystick_4),
	.joystick_5(joystick_5),
	.ps2_key(ps2_key),

	.ioctl_download(ioctl_download),
	.ioctl_index(ioctl_index),
	.ioctl_wr(ioctl_wr),
	.ioctl_addr(ioctl_addr),
	.ioctl_dout(ioctl_dout),
	.ioctl_wait(ioctl_wait),

	.img_mounted(img_mounted),
	.img_readonly(img_readonly),
	.img_size(img_size),
	.sd_lba('{sd_lba}),
	.sd_rd(sd_rd),
	.sd_wr(sd_wr),
	.sd_ack(sd_ack),
	.sd_buff_addr(sd_buff_addr),
	.sd_buff_dout(sd_buff_dout),
	.sd_buff_din('{bk_q}),
	.sd_buff_wr(sd_buff_wr),
	// [MiSTer-DB9 BEGIN] - DB9/SNAC8 support: joy_raw
	.joy_raw(joy_raw_payload),
	// programmable remap matrix selector load (UIO_DB9_MAP 0xFD)
	.db9_remap_cmd(db9_remap_cmd),
	.db9_remap_byte_cnt(db9_remap_byte_cnt),
	.db9_remap_din(db9_remap_din),
	// [MiSTer-DB9 END]
	// [MiSTer-DB9-Pro BEGIN] - Saturn key gate
	.saturn_unlocked(saturn_unlocked)
	// [MiSTer-DB9-Pro END]
);

// [MiSTer-DB9-Pro BEGIN] - DB controllers muted while OSD is open
// joydb_*_mapped is already in MiSTer-standard J1 order, identical to the USB
// joystick word, so it replaces the USB input as-is.
assign joystick_0 = joydb_1ena ? (OSD_STATUS ? 16'b0 : joydb_1_mapped) : joystick_0_USB;
assign joystick_1 = joydb_2ena ? (OSD_STATUS ? 16'b0 : joydb_2_mapped) : joydb_1ena ? joystick_0_USB : joystick_1_USB;
// [MiSTer-DB9-Pro END]

///////////////////////   CLOCKS   ///////////////////////////////

wire clk_sys;   // 53.693175 MHz, U13's MCLK
wire clk_ram;   // 107.386350 MHz, the SDRAM controller only
wire pll_locked;

pll pll
(
	.refclk   (CLK_50M),
	.rst      (1'b0),
	.outclk_0 (clk_sys),
	.outclk_1 (clk_ram),
	.locked   (pll_locked)
);

// Two resets. Cold is the framework's: new core, new file, PLL unlocked.
// The console's is the front panel, and it also holds the machine while
// content arrives. Reset is asserted asynchronously from `cold_reset`
// because a clocked chain reads "not in reset" until two edges have come.
//
// Index 1 is the cartridge; index 255 is the cheat blob, which must stay
// out of every reset term since a cheat toggle parks and resumes instead.
wire cart_dl  = ioctl_download && (ioctl_index[7:0] == 8'd1);
wire cheat_dl = ioctl_download && (ioctl_index[7:0] == 8'd255);
wire content_dl = ioctl_download && !cheat_dl;

// Cartridge byte order override. A change restarts the machine, since the
// setting only reaches the console through the words it reads.
wire [1:0]  cart_order = status[31:30];
wire        cart_swapped;

reg  [1:0]  cart_order_q;
reg [15:0]  cart_order_rst;
always @(posedge clk_sys) begin
	cart_order_q <= cart_order;
	if (cart_order != cart_order_q) cart_order_rst <= 16'hFFFF;
	else if (|cart_order_rst)       cart_order_rst <= cart_order_rst - 16'd1;
end

wire cold_reset = RESET | ~pll_locked;
// The .sav load also holds the console so the game cannot start on a
// half-written SRAM.
wire soft_reset = status[0] | buttons[1] | content_dl
                  | ss_restore_hold | bk_loading | (|cart_order_rst);

reg [1:0] rst_sys_q = 2'b11, rst_ram_q = 2'b11, rst_con_q = 2'b11;

always @(posedge clk_sys or posedge cold_reset)
	if (cold_reset) rst_sys_q <= 2'b11;
	else            rst_sys_q <= {rst_sys_q[0], 1'b0};

always @(posedge clk_ram or posedge cold_reset)
	if (cold_reset) rst_ram_q <= 2'b11;
	else            rst_ram_q <= {rst_ram_q[0], 1'b0};

always @(posedge clk_sys or posedge cold_reset)
	if (cold_reset) rst_con_q <= 2'b11;
	else            rst_con_q <= {rst_con_q[0], soft_reset};

wire reset_sys = rst_sys_q[1];   // memory path, cold start only
wire reset_ram = rst_ram_q[1];
wire reset_con = rst_con_q[1];   // console

///////////////////////   LOADING   //////////////////////////////

wire        bios_wr;
wire [14:0] bios_addr;
wire [15:0] bios_data;

bios_loader bios
(
	.ioctl_download (ioctl_download),
	.ioctl_index    (ioctl_index),
	.ioctl_wr       (ioctl_wr),
	.ioctl_addr     (ioctl_addr),
	.ioctl_dout     (ioctl_dout),
	.ld_wr          (bios_wr),
	.ld_addr        (bios_addr),
	.ld_data        (bios_data)
);

wire        ld_req, ld_we;
wire [25:0] ld_addr;
wire [15:0] ld_din, ld_dout;
wire        ld_busy, ld_done;

wire [24:0] rom_size;
wire        cart_present;

cart_loader cart
(
	.clk_sys        (clk_sys),
	.reset          (reset_sys),
	.ioctl_download (ioctl_download),
	.ioctl_index    (ioctl_index),
	.ioctl_wr       (ioctl_wr),
	.ioctl_addr     (ioctl_addr),
	.ioctl_dout     (ioctl_dout),
	.ioctl_wait     (cart_ioctl_wait),
	.order_sel      (cart_order),
	.rom_swapped    (cart_swapped),
	.mem_req        (ld_req),
	.mem_we         (ld_we),
	.mem_addr       (ld_addr),
	.mem_din        (ld_din),
	.mem_busy       (ld_busy),
	.mem_done       (ld_done),
	.rom_size       (rom_size),
	.rom_present    (cart_present)
);

// With no cartridge, open the OSD: after ~1.25 s idle the OSD button is
// held ~0.3 s and released, since Main opens the menu on release. The
// counter parks so it fires once.
reg [26:0] boot_osd_cnt;
wire       boot_osd_hold = boot_osd_cnt[26] & ~boot_osd_cnt[24];
always @(posedge clk_sys) begin
	if (reset_sys | content_dl | cart_present) boot_osd_cnt <= 27'd0;
	else if (~(boot_osd_cnt[26] & boot_osd_cnt[24])) boot_osd_cnt <= boot_osd_cnt + 27'd1;
end

assign BUTTONS = {1'b0, boot_osd_hold};

///////////////////////   CHEATS   //////////////////////////////

// Main's standard 16-byte records. They stream in while the machine is parked
// and become visible in one step at the end, so a multi-record patch is never
// half applied. Nothing here is savestated: the table belongs to the .cht file
// the user picked, not to the machine.
wire         cheat_pause_req;
wire         cheat_load_begin;
wire         cheat_invalidate;
wire         cheat_commit_req;
wire         cheat_commit_done;
wire [128:0] cheat_code;

acan_cheat_loader cheat_loader
(
	.clk_sys_i        (clk_sys),
	.cheat_download_i (cheat_dl),
	.cart_download_i  (cart_dl),
	.paused_i         (cheat_safe_stopped),
	.commit_done_i    (cheat_commit_done),
	.ioctl_wr_i       (ioctl_wr),
	.ioctl_addr_i     (ioctl_addr[3:0]),
	.ioctl_dout_i     (ioctl_dout),
	.wait_o           (cheat_wait),
	.pause_req_o      (cheat_pause_req),
	.load_begin_o     (cheat_load_begin),
	.invalidate_o     (cheat_invalidate),
	.commit_req_o     (cheat_commit_req),
	.code_o           (cheat_code)
);

// Cartridge and cheat loaders share the one back-pressure pin.
assign ioctl_wait = cart_ioctl_wait | cheat_wait;

// A cheat load normally parks within a frame. A machine wedged on a bus
// cycle never parks, and holding ioctl_wait forever would hang the HPS, so
// the stop is granted anyway after about 78 ms.
reg [22:0] cheat_park_timer = 23'd0;
always @(posedge clk_sys)
	if (!cheat_pause_req)             cheat_park_timer <= 23'd0;
	else if (!cheat_park_timer[22])   cheat_park_timer <= cheat_park_timer + 23'd1;

wire cheat_safe_stopped = reset_con | ss_parked | cheat_park_timer[22];

///////////////////////   SDRAM   ////////////////////////////////

wire        p0_req, p0_we;
wire [25:0] p0_addr;
wire [63:0] p0_din, p0_dout;
wire  [7:0] p0_byte_en;
wire        p0_busy, p0_ready;
wire        p1_req;
wire [25:0] p1_addr;
wire [63:0] p1_dout;
wire        p1_busy, p1_ready;

sdram_bridge mem_bridge
(
	.clk_sys    (clk_sys),
	.reset_sys  (reset_sys),
	.clk_ram    (clk_ram),
	.reset_ram  (reset_ram),

	.c0_req     (ld_req),
	.c0_we      (ld_we),
	.c0_addr    (ld_addr),
	.c0_din     (ld_din),
	.c0_be      (2'b11),
	.c0_dout    (ld_dout),
	.c0_busy    (ld_busy),
	.c0_done    (ld_done),

	.c1_req     (cart_mem_req),
	.c1_addr    (cart_mem_addr),
	.c1_dout    (cart_mem_dout),
	.c1_busy    (cart_mem_busy),
	.c1_done    (cart_mem_done),

	.p0_req     (p0_req),
	.p0_we      (p0_we),
	.p0_addr    (p0_addr),
	.p0_din     (p0_din),
	.p0_byte_en (p0_byte_en),
	.p0_dout    (p0_dout),
	.p0_busy    (p0_busy),
	.p0_ready   (p0_ready),

	.p1_req     (p1_req),
	.p1_addr    (p1_addr),
	.p1_dout    (p1_dout),
	.p1_busy    (p1_busy),
	.p1_ready   (p1_ready)
);

sdram #(
	.CLK_FREQ_HZ         (107_386_350),
	.DQ_CAPTURE_PIPELINE (3),
	.PORT0_SIZE          (1),   // 16-bit
	.PORT1_SIZE          (1),
	.PORT2_SIZE          (1)
) sdram_i
(
	.clk         (clk_ram),
	.reset       (reset_ram),
	.refresh     (1'b0),
	.dq_pipe_sel (3'd0),

	.p0_req      (p0_req),
	.p0_we       (p0_we),
	.p0_addr     (p0_addr),
	.p0_din      (p0_din),
	.p0_byte_en  (p0_byte_en),
	.p0_dout     (p0_dout),
	.p0_busy     (p0_busy),
	.p0_ready    (p0_ready),

	.p1_req      (p1_req),
	.p1_we       (1'b0),
	.p1_addr     (p1_addr),
	.p1_din      (64'd0),
	.p1_byte_en  (8'd0),
	.p1_dout     (p1_dout),
	.p1_busy     (p1_busy),
	.p1_ready    (p1_ready),

	.p2_req      (1'b0),
	.p2_we       (1'b0),
	.p2_addr     (26'd0),
	.p2_din      (64'd0),
	.p2_byte_en  (8'd0),
	.p2_dout     (),
	.p2_busy     (),
	.p2_ready    (),

	.SDRAM_CLK   (SDRAM_CLK),
	.SDRAM_CKE   (SDRAM_CKE),
	.SDRAM_A     (SDRAM_A),
	.SDRAM_BA    (SDRAM_BA),
	.SDRAM_DQ    (SDRAM_DQ),
	.SDRAM_DQML  (SDRAM_DQML),
	.SDRAM_DQMH  (SDRAM_DQMH),
	.SDRAM_nCS   (SDRAM_nCS),
	.SDRAM_nCAS  (SDRAM_nCAS),
	.SDRAM_nRAS  (SDRAM_nRAS),
	.SDRAM_nWE   (SDRAM_nWE)
);

///////////////////////   CONSOLE   //////////////////////////////

wire        ce_pix;
wire  [7:0] vid_r, vid_g, vid_b;
wire        vid_hs, vid_vs, vid_hb, vid_vb, vid_de;
wire [15:0] audio_l, audio_r;

wire ss_chord = joystick_0[12];   // Savestates button

// While the savestate chord is held the other pad 1 buttons pick the slot
// and save or load, so the game must not see them.
wire [11:0] joy0_game = ss_chord ? 12'd0 : joystick_0[11:0];

superacan_console console
(
	.clk_sys        (clk_sys),
	// Multitap off: joysticks 1 and 2 on the two ports. On: 1-4 on port 1,
	// 5 (and 6 with both ports tapped) on port 2.
	.multitap       (status[29:28]),
	.pad1           ({joystick_3[11:0], joystick_2[11:0], joystick_1[11:0], joy0_game}),
	.pad2           (status[29:28] == 2'd0 ? {joystick_5[11:0], joystick_1[11:0]}
	                                       : {joystick_5[11:0], joystick_4[11:0]}),
	.cold_reset     (cold_reset),
	// OSD reset, loading hold and restore hold all act as the front panel button.
	.reset_btn_n    (~reset_con),

	.ss_cpu_ready   (ss_cpu_ready),
	.ss_snd_ready   (ss_snd_ready),
	.vid_overscan   (vid_overscan),
	.ss_mem_idle    (ss_mem_idle),
	.ss_vid_ready   (ss_vid_ready),

	.ss_din         (ss_bus_din),
	.ss_addr        (ss_bus_addr),
	.ss_wren        (ss_bus_wren),
	.ss_rst         (ss_bus_rst),
	.ss_load        (ss_cpu_load),
	.ss_dout        (ss_bus_dout),

	.ld_wr          (bios_wr),
	.ld_addr        (bios_addr),
	.ld_data        (bios_data),

	.ss_sndram_own   (ss_sndram_own),
	.ss_sndram_addr  (ss_sndram_addr),
	.ss_sndram_we    (ss_sndram_we),
	.ss_sndram_wdata (ss_sndram_wdata),
	.ss_sndram_q     (ss_sndram_q),
	.ss_wram_own    (ss_wram_own),
	.ss_wram_addr   (ss_wram_addr),
	.ss_wram_we     (ss_wram_we),
	.ss_wram_be     (ss_wram_be),
	.ss_wram_wdata  (ss_wram_wdata),
	.ss_wram_q      (ss_wram_q),

	.ss_vram_own    (ss_vram_own),
	.ss_vram_addr   (ss_vram_addr),
	.ss_vram_we     (ss_vram_we),
	.ss_vram_be     (ss_vram_be),
	.ss_vram_wdata  (ss_vram_wdata),
	.ss_vram_q      (ss_vram_q),

	.ss_pal_own     (ss_pal_own),
	.ss_pal_addr    (ss_pal_addr),
	.ss_pal_we      (ss_pal_we),
	.ss_pal_be      (ss_pal_be),
	.ss_pal_wdata   (ss_pal_wdata),
	.ss_pal_q       (ss_pal_q),

	.ss_cartram_own   (ss_cartram_own),
	.ss_cartram_addr  (ss_cartram_addr),
	.ss_cartram_we    (ss_cartram_we),
	.ss_cartram_wdata (ss_cartram_wdata),
	.ss_cartram_q     (ss_cartram_q),

	.bk_addr          (bk_addr),
	.bk_wren          (bk_wren),
	.bk_be            (2'b11),
	.bk_wdata         (sd_buff_dout),
	.bk_q             (bk_q),
	.bk_dirty         (bk_dirty),

	.cart_mem_req     (cart_mem_req),
	.cart_mem_addr    (cart_mem_addr),
	.cart_mem_dout    (cart_swapped ? {cart_mem_dout[7:0], cart_mem_dout[15:8]}
	                                : cart_mem_dout),
	.cart_mem_busy    (cart_mem_busy),
	.cart_mem_done    (cart_mem_done),
	.rom_size         (rom_size),

	.cart_present   (cart_present),

	.pause_cpu      (ss_pause_cpu),
	.pause_vid      (ss_pause_vid),

	.cheat_load_begin  (cheat_load_begin),
	.cheat_invalidate  (cheat_invalidate),
	.cheat_commit_req  (cheat_commit_req),
	.cheat_paused      (cheat_safe_stopped),
	.cheat_code        (cheat_code),
	.cheat_commit_done (cheat_commit_done),

	.ce_pix         (ce_pix),
	.vid_r          (vid_r),
	.vid_g          (vid_g),
	.vid_b          (vid_b),
	.vid_hs         (vid_hs),
	.vid_vs         (vid_vs),
	.vid_hb         (vid_hb),
	.vid_vb         (vid_vb),
	.vid_de         (vid_de),

	.audio_l        (audio_l),
	.audio_r        (audio_r)
);

///////////////////////   BACKUP RAM   ///////////////////////////

// The cartridge's battery SRAM as a .sav file. `save_slot` owns the block
// protocol; only the address arithmetic and write strobe are here. sd_lba
// counts 512-byte blocks and sd_buff_addr the 256 words inside one.
wire [13:0] bk_addr = {sd_lba[5:0], sd_buff_addr[7:0]};
wire        bk_wren = sd_buff_wr & sd_ack;

// The OSD entries are toggles; save_slot wants a one-cycle pulse.
reg  bk_save_q, bk_load_q;
always @(posedge clk_sys) begin
	bk_save_q <= status[13];
	bk_load_q <= status[14];
end
wire bk_save_req = status[13] ^ bk_save_q;
wire bk_load_req = status[14] ^ bk_load_q;

// A new cartridge must not inherit the last one's save. The download starts
// before the new .sav mounts, so clearing here is safe.
reg  bk_dl_q;
always @(posedge clk_sys) bk_dl_q <= cart_dl;
wire bk_invalidate = cart_dl & ~bk_dl_q;

save_slot #(.MAX_BLOCKS (64)) backup
(
	.clk_sys          (clk_sys),
	.invalidate_pulse (bk_invalidate),
	.mount_pulse      (img_mounted),
	.mount_readonly   (img_readonly),
	.mount_size       (img_size),
	.block_limit      (32'd64),
	.load_req         (bk_load_req),
	.save_req         (bk_save_req),
	.autosave_disable (status[12]),
	.osd_status       (OSD_STATUS),
	.dirty_pulse      (bk_dirty),
	// The SRAM's port B belongs to the save slot alone.
	.transfer_allowed (1'b1),
	.mounted_writable (bk_writable),
	.pending          (bk_pending),
	.busy             (bk_busy),
	.load_active      (bk_loading),
	.sd_lba           (sd_lba),
	.sd_rd            (sd_rd),
	.sd_wr            (sd_wr),
	.sd_ack           (sd_ack)
);

// The framework's disk light: lit while a transfer runs or one is due.
assign LED_DISK = {1'b1, bk_pending | bk_busy};

///////////////////////   SAVESTATES   ///////////////////////////////

// Four 1 MiB slots in DDR3 at 0x3E000000 (58,720,256 dwords from the DDR
// base). The sizes must move with ss_map.svh: SS_STATESIZE is what stops a
// state file from an older build being loaded.
localparam integer SS_INTERNALS = 922;
localparam integer SS_WRAM      = 65536;
localparam integer SS_VRAM      = 131072;
localparam integer SS_SNDRAM    = 65536;
localparam integer SS_CARTRAM   = 32768;
localparam integer SS_PALETTE   = 512;
localparam integer SS_STATESIZE = SS_INTERNALS * 2
	+ (SS_WRAM + SS_VRAM + SS_SNDRAM + SS_CARTRAM + SS_PALETTE) / 4;

wire [63:0] ss_bus_din;
wire [9:0]  ss_bus_addr;
wire        ss_bus_wren, ss_bus_rst_pulse;
wire [24:0] ss_ram_addr;
wire [2:0]  ss_ram_type;
wire        ss_ram_rd, ss_ram_wr, ss_ram_ready;
wire [7:0]  ss_ram_wdata, ss_ram_rdata;
wire        ss_sleep, ss_parked, ss_pause_cpu, ss_pause_vid;
wire        ss_restore_begin, ss_load_done;
wire        ss_save, ss_load, ss_status_update, ss_info_req;
wire        ss_cpu_ready, ss_snd_ready, ss_mem_idle, ss_vid_ready;
wire        ss_wram_own, ss_wram_we;
wire        ss_sndram_own, ss_sndram_we;
wire [15:0] ss_sndram_addr;
wire  [7:0] ss_sndram_wdata, ss_sndram_q;
wire        ss_vram_own, ss_vram_we, ss_pal_own, ss_pal_we;
wire [15:0] ss_vram_addr, ss_vram_wdata, ss_vram_q;
wire  [7:0] ss_pal_addr;
wire  [1:0] ss_vram_be, ss_pal_be;
wire [15:0] ss_pal_wdata, ss_pal_q;
wire        ss_cartram_own, ss_cartram_we;
wire [14:0] ss_cartram_addr;
wire  [7:0] ss_cartram_wdata, ss_cartram_q;
wire        cart_mem_req, cart_mem_busy, cart_mem_done;
wire [25:0] cart_mem_addr;
wire [15:0] cart_mem_dout;
wire [14:0] ss_wram_addr;
wire  [1:0] ss_wram_be;
wire [15:0] ss_wram_wdata, ss_wram_q;
wire  [7:0] ss_info;
wire  [1:0] ss_slot;
wire [25:0] ss_ddr_addr;
wire [63:0] ss_ddr_din, ss_ddr_dout;
wire  [7:0] ss_ddr_be;
wire        ss_ddr_rnw, ss_ddr_ena, ss_ddr_done;
wire        ss_req_save, ss_req_load;
integer     ss_req_addr;
wire        ss_busy;

wire ss_menu_enabled = cart_present & ~content_dl & ~ss_busy;

// The CPUs load their state on this window, so it must outlast the last
// word write by as much as reset_con does.
reg [2:0] ss_cpu_load_q = 3'd0;
always @(posedge clk_sys) ss_cpu_load_q <= {ss_cpu_load_q[1:0], ss_restore_hold};
wire ss_cpu_load = ss_restore_hold | (|ss_cpu_load_q);

// Every chip's words are ORed onto this.
wire [63:0] ss_bus_dout;

// The machine is held in reset for the whole restore; blocks reload from
// their saved words on the last reset cycle.
reg  ss_restore_hold = 1'b0;
reg  [3:0] ss_after_q = 4'd0;
always @(posedge clk_sys) begin
	if (cold_reset) begin
		ss_restore_hold <= 1'b0;
		ss_after_q      <= 4'd0;
	end else begin
		if (ss_restore_begin)  ss_restore_hold <= 1'b1;
		else if (ss_load_done) ss_restore_hold <= 1'b0;

		// A few cycles after the hold lets go, restore every word to its
		// power-on default so a later reset does not reload the saved state.
		if (ss_restore_hold)         ss_after_q <= 4'd1;
		else if (ss_after_q != 4'd0 && ss_after_q != 4'd9)
			ss_after_q <= ss_after_q + 4'd1;
	end
end
wire ss_defaults_now = (ss_after_q == 4'd8);

// The reset synchroniser lags the hold by two flops; masking on the hold
// alone would let reset clear the saved words just before the blocks read
// them.
wire ss_restore_window = (ss_after_q != 4'd0) && (ss_after_q < 4'd8);

// Not the restore hold: that is a machine reset and must leave the saved
// words alone.
wire ss_bus_rst = ss_bus_rst_pulse | ss_defaults_now
                  | (reset_con & ~ss_restore_window);

savestate_ui ss_ui
(
	.clk           (clk_sys),
	.ps2_key       (ps2_key),
	.allow_ss      (ss_menu_enabled),
	.joySS         (ss_chord),
	.joyRight      (joystick_0[0]),
	.joyLeft       (joystick_0[1]),
	.joyDown       (joystick_0[2]),
	.joyUp         (joystick_0[3]),
	.joyStart      (1'b0),
	.joySaveState  (ss_chord),
	.status_slot   (status[19:18]),
	.OSD_saveload  (status[21:20]),
	.ss_save       (ss_save),
	.ss_load       (ss_load),
	.ss_info_req   (ss_info_req),
	.ss_info       (ss_info),
	.statusUpdate  (ss_status_update),
	.selected_slot (ss_slot)
);

// Rewind is not wired up; a capture stops the machine for the whole transfer.
statemanager #(
	.Softmap_SaveState_ADDR (58720256),
	.Softmap_Rewind_ADDR    (0),
	.SAVESTATE_SHIFT        (18)
) ss_manager
(
	.clk               (clk_sys),
	.reset             (cold_reset),
	.rewind_on         (1'b0),
	.rewind_active     (1'b0),
	.savestate_number  ({30'd0, ss_slot}),
	.save              (ss_save),
	.load              (ss_load),
	/* verilator lint_off PINCONNECTEMPTY */
	.sleep_rewind      (),
	/* verilator lint_on PINCONNECTEMPTY */
	.vsync             (vid_vs),
	.request_savestate (ss_req_save),
	.request_loadstate (ss_req_load),
	/* verilator lint_off PINCONNECTEMPTY */
	.request_is_rewind (),
	/* verilator lint_on PINCONNECTEMPTY */
	.request_address   (ss_req_addr),
	.request_busy      (ss_busy)
);

savestates #(
	.STATESIZE_PARAM      (SS_STATESIZE),
	.INTERNALSCOUNT_PARAM (SS_INTERNALS),
	.SAVETYPESCOUNT_PARAM (5),
	.SAVETYPE0_SIZE       (SS_WRAM),
	.SAVETYPE1_SIZE       (SS_VRAM),
	.SAVETYPE2_SIZE       (SS_SNDRAM),
	.SAVETYPE3_SIZE       (SS_CARTRAM),
	.SAVETYPE4_SIZE       (SS_PALETTE)
) ss_engine
(
	.clk                      (clk_sys),
	.reset_in                 (cold_reset),
	.reset_ss                 (),
	/* verilator lint_off PINCONNECTEMPTY */
	.reset_delay              (),
	/* verilator lint_on PINCONNECTEMPTY */
	.restore_begin            (ss_restore_begin),
	.load_done                (ss_load_done),
	// No reset-held preparation is needed.
	.restore_prepare_ready_i  (1'b1),
	.restore_prepare_failed_i (1'b0),

	.increaseSSHeaderCount    (1'b1),
	.save                     (ss_req_save),
	.load                     (ss_req_load),
	.state_size_i             (SS_STATESIZE[31:0]),
	.savetype3_size_i         (SS_CARTRAM[24:0]),
	.is_rewind_i              (1'b0),
	.savestate_address        (ss_req_addr),
	.savestate_busy           (ss_busy),
	.paused                   (ss_parked),

	.BUS_Din                  (ss_bus_din),
	.BUS_Adr                  (ss_bus_addr),
	.BUS_wren                 (ss_bus_wren),
	.BUS_rst                  (ss_bus_rst_pulse),
	.BUS_Dout                 (ss_bus_dout),

	/* verilator lint_off PINCONNECTEMPTY */
	.loading_savestate        (),
	.saving_savestate         (),
	/* verilator lint_on PINCONNECTEMPTY */
	.sleep_savestate          (ss_sleep),

	.Save_RAMAddr             (ss_ram_addr),
	.Save_RAMRdEn             (ss_ram_rd),
	.Save_RAMWrEn             (ss_ram_wr),
	.Save_RAMWriteData        (ss_ram_wdata),
	.Save_RAMReadData         (ss_ram_rdata),
	.Save_RAMReady            (ss_ram_ready),
	.Save_RAMType             (ss_ram_type),

	.bus_out_Din              (ss_ddr_din),
	.bus_out_Dout             (ss_ddr_dout),
	.bus_out_Adr              (ss_ddr_addr),
	.bus_out_rnw              (ss_ddr_rnw),
	.bus_out_ena              (ss_ddr_ena),
	.bus_out_be               (ss_ddr_be),
	.bus_out_done             (ss_ddr_done)
);

ss_park park
(
	.clk_sys   (clk_sys),
	.reset     (cold_reset),
	.req       (ss_sleep | cheat_pause_req),
	.cpu_ready (ss_cpu_ready),
	.snd_ready (ss_snd_ready),
	.mem_idle  (ss_mem_idle),
	.vid_ready (ss_vid_ready),
	.pause_cpu (ss_pause_cpu),
	.pause_vid (ss_pause_vid),
	.parked    (ss_parked)
);

ss_bulk ss_mem
(
	.clk     (clk_sys),
	.reset   (cold_reset),
	.addr_i  (ss_ram_addr),
	.type_i  (ss_ram_type),
	.rd_i    (ss_ram_rd),
	.wr_i    (ss_ram_wr),
	.wdata_i (ss_ram_wdata),
	.rdata_o (ss_ram_rdata),
	.ready_o (ss_ram_ready),

	.wram_own_o (ss_wram_own), .wram_addr_o (ss_wram_addr),
	.wram_we_o (ss_wram_we), .wram_be_o (ss_wram_be),
	.wram_wdata_o (ss_wram_wdata), .wram_q_i (ss_wram_q),

	/* verilator lint_off PINCONNECTEMPTY */
	.vram_own_o (ss_vram_own), .vram_addr_o (ss_vram_addr),
	.vram_we_o (ss_vram_we), .vram_be_o (ss_vram_be),
	.vram_wdata_o (ss_vram_wdata), .vram_q_i (ss_vram_q),
	.sndram_own_o (ss_sndram_own), .sndram_addr_o (ss_sndram_addr),
	.sndram_we_o (ss_sndram_we),
	.sndram_wdata_o (ss_sndram_wdata), .sndram_q_i (ss_sndram_q),
	.cartram_own_o (ss_cartram_own), .cartram_addr_o (ss_cartram_addr),
	.cartram_we_o (ss_cartram_we), .cartram_wdata_o (ss_cartram_wdata),
	.cartram_q_i (ss_cartram_q),
	/* verilator lint_on PINCONNECTEMPTY */

	.pal_own_o (ss_pal_own), .pal_addr_o (ss_pal_addr),
	.pal_we_o (ss_pal_we), .pal_be_o (ss_pal_be),
	.pal_wdata_o (ss_pal_wdata), .pal_q_i (ss_pal_q)
);

ss_ddram ss_ddr
(
	.clk              (clk_sys),
	.reset            (cold_reset),
	.DDRAM_CLK        (DDRAM_CLK),
	.DDRAM_BUSY       (DDRAM_BUSY),
	.DDRAM_BURSTCNT   (DDRAM_BURSTCNT),
	.DDRAM_ADDR       (DDRAM_ADDR),
	.DDRAM_DOUT       (DDRAM_DOUT),
	.DDRAM_DOUT_READY (DDRAM_DOUT_READY),
	.DDRAM_RD         (DDRAM_RD),
	.DDRAM_DIN        (DDRAM_DIN),
	.DDRAM_BE         (DDRAM_BE),
	.DDRAM_WE         (DDRAM_WE),
	.addr_i           (ss_ddr_addr),
	.din_i            (ss_ddr_din),
	.dout_o           (ss_ddr_dout),
	.be_i             (ss_ddr_be),
	.rnw_i            (ss_ddr_rnw),
	.ena_i            (ss_ddr_ena),
	.done_o           (ss_ddr_done)
);

///////////////////////   VIDEO   ////////////////////////////////

wire [2:0] sfx = status[24:22];
wire [2:0] sl  = |sfx ? sfx - 3'd1 : 3'd0;
assign VGA_SL = sl[1:0];

assign CLK_VIDEO = clk_sys;

wire vga_de_mixed;
wire vid_overscan;

// Height crop. $F00008 bit 9 asks for a centred 224-line aperture, but
// software moves it mid-run (Son of Evil switches from 224 to 240 as it
// takes over) and every step makes the scaler relock. Auto follows the
// chip; the other settings pin the height. 224 is the default, matching a
// television.
wire [1:0] crop = status[16:15];

// Line inside the visible region, counted at the end of each line so it
// holds the line's own number through its active dots.
reg [7:0] vis_line;
reg       vid_hb_q;
always @(posedge CLK_VIDEO) if (ce_pix) begin
	vid_hb_q <= vid_hb;
	if (vid_vb)                  vis_line <= 8'd0;
	else if (~vid_hb_q & vid_hb) vis_line <= vis_line + 8'd1;
end

// The chip's centred window, for when the chip is not cropping.
wire pin224       = (vis_line < 8'd8) | (vis_line >= 8'd232);
wire chip_crop    = ~vid_de & ~vid_hb;
wire line_cropped = (crop == 2'd0) ? pin224
                  : (crop == 2'd1) ? 1'b0
                                   : chip_crop;

// Displayed ratio as two constants. Both width modes have 2560 of 3420
// master clocks active, so only the height moves the ratio:
//
//   DAR = 4/3 * 2560/3420 * 262/lines * 64/53 * 32/35
//
//   240 lines  1.20290  ->  83:69
//   224 lines  1.28882  ->  415:322
//
// Measuring the raster instead took ten frames to settle after each height
// change and read the 320-dot line 1.8% wide, since thirty of its blanking
// dots are stretched.
wire        show224 = (crop == 2'd0) | ((crop == 2'd2) & vid_overscan);
wire [11:0] crt_arx = show224 ? 12'd415 : 12'd83;
wire [11:0] crt_ary = show224 ? 12'd322 : 12'd69;

video_mixer #(.LINE_LENGTH(360), .HALF_DEPTH(0), .GAMMA(1)) video_mixer
(
	.CLK_VIDEO   (CLK_VIDEO),
	.CE_PIXEL    (CE_PIXEL),
	.ce_pix      (ce_pix),

	.scandoubler (|sfx | forced_scandoubler),
	.hq2x        (sfx == 3'd1),

	.gamma_bus   (gamma_bus),

	.R           (vid_r),
	.G           (vid_g),
	.B           (vid_b),

	.HSync       (vid_hs),
	.VSync       (vid_vs),
	.HBlank      (vid_hb | line_cropped),
	.VBlank      (vid_vb),

	.HDMI_FREEZE (HDMI_FREEZE),
	.freeze_sync (),

	.VGA_R       (VGA_R),
	.VGA_G       (VGA_G),
	.VGA_B       (VGA_B),
	.VGA_VS      (VGA_VS),
	.VGA_HS      (VGA_HS),
	.VGA_DE      (vga_de_mixed)
);

// video_freak owns VIDEO_ARX/ARY; integer scaling sets their [12] flag.
video_freak video_freak
(
	.CLK_VIDEO   (CLK_VIDEO),
	.CE_PIXEL    (CE_PIXEL),
	.VGA_VS      (VGA_VS),
	.HDMI_WIDTH  (HDMI_WIDTH),
	.HDMI_HEIGHT (HDMI_HEIGHT),
	.VGA_DE      (VGA_DE),
	.VIDEO_ARX   (VIDEO_ARX),
	.VIDEO_ARY   (VIDEO_ARY),

	.VGA_DE_IN   (vga_de_mixed),
	.ARX         ((ar == 2'd0) ? crt_arx : {10'd0, ar - 2'd1}),
	.ARY         ((ar == 2'd0) ? crt_ary : 12'd0),
	.CROP_SIZE   (12'd0),
	.CROP_OFF    (5'd0),
	.SCALE       (status[27:25])
);

///////////////////////   AUDIO   ////////////////////////////////

assign AUDIO_S = 1;
assign AUDIO_L = audio_l;
assign AUDIO_R = audio_r;

assign LED_USER = ioctl_download;

`ifdef VERILATOR
/* verilator lint_off UNUSEDSIGNAL */
wire unused = &{1'b0,
                buttons[0], ioctl_index[15:8], joystick_1[31:12],
                joystick_0[31:13],
                joystick_2[31:12], joystick_3[31:12], joystick_4[31:12],
                joystick_5[31:12],
                status[127:123], status[120:32], status[17], status[7:1],
                sd_buff_addr[12:8]};
/* verilator lint_on UNUSEDSIGNAL */
`endif

endmodule
