// Copyright (c) 2026 Jamie Blanks

// BIOS load path. Four internal images, loadable as one file or four.
//
//   0x0000  4096  68000 IPL, inside the UM6619
//   0x1000  8192  sound RAM image $0000-$1FFF
//   0x3000  8192  sound RAM image $2000-$3FFF
//   0x5000    16  UM6650 key
//                 -----
//                 20496
//
// One file: the concatenation as `boot.rom`. Four files: `boot0.rom` to
// `boot3.rom`, all on slot 0 with the number in the top two bits of the
// index (0x00, 0x40, 0x80, 0xC0), which picks the offset.
//
// Dumps carry each 68000 word byte-swapped and the HPS sends words low byte
// first, so `ioctl_dout` is already big-endian.

module bios_loader #(
	parameter [15:0] BIOS_INDEX = 16'd0
) (
	input  wire        ioctl_download,
	input  wire [15:0] ioctl_index,
	input  wire        ioctl_wr,
	input  wire [26:0] ioctl_addr,
	input  wire [15:0] ioctl_dout,

	output wire        ld_wr,
	output wire [14:0] ld_addr,   // byte address inside boot.rom
	output wire [15:0] ld_data
);
	wire bios_stream = ioctl_download && (ioctl_index[5:0] == BIOS_INDEX[5:0]);

	reg [14:0] base;
	always @(*) begin
		case (ioctl_index[7:6])
			2'd0:    base = 15'h0000;             // boot0.rom - 68000 IPL
			2'd1:    base = 15'h1000;             // boot1.rom - 6502 low
			2'd2:    base = 15'h3000;             // boot2.rom - 6502 high
			default: base = 15'h5000;             // boot3.rom - UM6650 key
		endcase
	end

	// Drop anything past the concatenation instead of writing past the ROMs.
	assign ld_wr   = bios_stream && ioctl_wr && !(|ioctl_addr[26:15]);
	assign ld_addr = base + ioctl_addr[14:0];
	assign ld_data = ioctl_dout;
endmodule
