// Savestate word allocation and memory types.
//
// SS_STATESIZE in SuperAcan.sv must move with this layout; it is what stops
// a state file from an older build being loaded.
//
// Included inside a module body, so every name is a module-scoped localparam.
// No include guard: one would leave every module after the first without them.
//
//    0- 23  U9  MC68000    fx68k state, 22 words in use
//   24- 31  spare
//   32- 47  U10 UM6619     bus side: decode, DTACK, IPL, overlay latches
//   48- 55  spare
//   56- 63  U10 UM6619     free running counter, mailboxes, I/O page
//   64- 79  U10 UM6619     65C02, sound CPU block, joypad shift registers
//   80-111  spare
//  112-159  U12 UM6618     registers, timing, layers, ROZ, sprites, window,
//                          mixer, sprite DMA destination side (137-139)
//  160-167  U3  UM70C188   DAC address and mask latches
//  168-175  cartridge      UM6650 lockout state
//  176-183  top level      176: pad 1 and 2 of each port; 177: port 1's
//                          pads 3 and 4 and both multitap clock counts
//  184-191  spare
//  192-447  U12 UM6618     register readback RAM, one 16-bit entry per word
//  448-703  U12 UM6618     128 sprite descriptors, two words per entry
//  704-831  U12 UM6618     128 sprite vertical-state entries
//  832-912  U10 UM6619     16-channel sampler: register store, per-channel
//                          arrays and the mixer
//  913-921  U10 UM6619     both DMA channel register files, bus machine and
//                          transfer machine

/* verilator lint_off UNUSEDPARAM */

localparam int SS_INTERNAL_WORDS = 922;

localparam int SSW_M68K_BASE   = 0;
localparam int SSW_M68K_REGS   = 24;
localparam int SSW_6619_BUS    = 32;
localparam int SSW_6619_DMA    = 913;
localparam int SSW_6619_FRC    = 56;
localparam int SSW_6619_CPU    = 64;
localparam int SSW_6619_SND    = 832;
localparam int SSW_6618_BASE   = 112;
localparam int SSW_6618_CLOCKS = 113;
localparam int SSW_6618_TIMING = 114;
localparam int SSW_6618_SPRITE = 115;
localparam int SSW_6618_REGS   = 116;
localparam int SSW_6618_ROZ    = 134;
localparam int SSW_6618_DMA    = 137;
localparam int SSW_6618_STORE  = 192;
localparam int SSW_6618_DESCRIPTORS = 448;
localparam int SSW_6618_VSTATE = 704;
localparam int SSW_70C188_BASE = 160;
localparam int SSW_CART_BASE   = 168;
localparam int SSW_TOP_BASE    = 176;

// Bulk memory regions, in the order the transfer engine walks them.
localparam [2:0] SS_MEM_WRAM    = 3'd0;   //  64 KiB  U1, U2
localparam [2:0] SS_MEM_VRAM    = 3'd1;   // 128 KiB  U5, U6
localparam [2:0] SS_MEM_SNDRAM  = 3'd2;   //  64 KiB  U11
localparam [2:0] SS_MEM_CARTRAM = 3'd3;   //  32 KiB  cartridge 62256
localparam [2:0] SS_MEM_PALETTE = 3'd4;   //   512 B  inside U3

localparam int SS_WRAM_BYTES    = 65536;
localparam int SS_VRAM_BYTES    = 131072;
localparam int SS_SNDRAM_BYTES  = 65536;
localparam int SS_CARTRAM_BYTES = 32768;
localparam int SS_PALETTE_BYTES = 512;

/* verilator lint_on UNUSEDPARAM */
