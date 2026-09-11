// Copyright (c) 2026 Jamie Blanks

// Cartridge image path: the HPS streams the file in, this writes it to SDRAM.
//
// Dumps come in two byte orders. Most have each 68000 word byte-swapped,
// which cancels the HPS sending words low byte first; the rest are native
// order and arrive backwards. The first 64 longwords are the vector table and
// every entry is an even 24-bit address, so counting the entries that fit
// each way round tells them apart. The image is stored as it arrived and
// fixed on the way out, so the whole table is seen before deciding.

module cart_loader #(
	parameter [15:0] CART_INDEX = 16'd1
) (
	input  wire        clk_sys,
	input  wire        reset,

	input  wire        ioctl_download,
	input  wire [15:0] ioctl_index,
	input  wire        ioctl_wr,
	input  wire [26:0] ioctl_addr,
	input  wire [15:0] ioctl_dout,
	output wire        ioctl_wait,

	// 0 auto, 1 word-swapped, 2 big-endian.
	input  wire [1:0]  order_sel,
	// Words read back from SDRAM need their bytes exchanged.
	output wire        rom_swapped,

	// SDRAM bridge client, write only: pulse req while busy is low, wait for done.
	output reg         mem_req,
	output reg         mem_we,
	output reg  [25:0] mem_addr,
	output reg  [15:0] mem_din,
	input  wire        mem_busy,
	input  wire        mem_done,

	output reg  [24:0] rom_size,    // bytes, zero until an image is loaded
	output reg         rom_present
);
	wire cart_stream = ioctl_download && (ioctl_index == CART_INDEX);

	reg        pending;
	reg        req_sent;
	reg        cart_stream_q;

	// Anything past 32 MiB is not a cartridge.
	wire load_wr = ioctl_wr && cart_stream && !pending && !(|ioctl_addr[26:25]);

	// Score the vector table a longword at a time; hdr_hi holds the high
	// word until the low word completes the address.
	//
	//   file bytes  fc 00 fe ff
	//   direct      $00FCFFFE   even, top byte clear: an address
	//   swapped     $FC00FEFF   odd and past the map: not one
	wire        hdr_word  = load_wr && !(|ioctl_addr[24:8]);
	wire        hdr_lo    = ioctl_addr[1];        // low word of the longword
	wire        hdr_ssp   = !(|ioctl_addr[7:2]);  // longword 0, the stack pointer
	reg  [15:0] hdr_hi;
	wire        vec_direct = (hdr_hi[15:8] == 8'h00) && !ioctl_dout[0];
	wire        vec_swap   = (hdr_hi[7:0]  == 8'h00) && !ioctl_dout[8];
	wire        vec_nz     = |{hdr_hi, ioctl_dout};

	reg        ssp_direct, ssp_swap;
	reg  [5:0] hand_direct, hand_swap;   // how many of the 63 vectors fit
	reg        auto_swap;

	// A usable stack pointer outranks any number of plausible handlers. A tie
	// leaves the file alone.
	wire swap_wins = (ssp_swap & ~ssp_direct)
	              |  (ssp_swap &  ssp_direct & (hand_swap > hand_direct));

	assign rom_swapped = (order_sel == 2'd0) ? auto_swap : order_sel[1];

	// Only a download write may stall the HPS.
	assign ioctl_wait = pending;

	always @(posedge clk_sys) begin
		mem_req       <= 1'b0;
		cart_stream_q <= cart_stream;

		if (reset) begin
			mem_req       <= 1'b0;
			mem_we        <= 1'b1;
			mem_addr      <= '0;
			mem_din       <= '0;
			pending       <= 1'b0;
			req_sent      <= 1'b0;
			rom_size      <= '0;
			rom_present   <= 1'b0;
			cart_stream_q <= 1'b0;
			hdr_hi        <= '0;
			ssp_direct    <= 1'b0;
			ssp_swap      <= 1'b0;
			hand_direct   <= 6'd0;
			hand_swap     <= 6'd0;
			auto_swap     <= 1'b0;
		end else begin
			// A new download replaces whatever was there.
			if (cart_stream && !cart_stream_q) begin
				rom_size    <= '0;
				rom_present <= 1'b0;
				ssp_direct  <= 1'b0;
				ssp_swap    <= 1'b0;
				hand_direct <= 6'd0;
				hand_swap   <= 6'd0;
			end

			if (hdr_word) begin
				if (!hdr_lo) hdr_hi <= ioctl_dout;
				else if (hdr_ssp) begin
					ssp_direct <= vec_direct & vec_nz;
					ssp_swap   <= vec_swap   & vec_nz;
				end else begin
					if (vec_direct) hand_direct <= hand_direct + 6'd1;
					if (vec_swap)   hand_swap   <= hand_swap   + 6'd1;
				end
			end

			if (load_wr) begin
				pending  <= 1'b1;
				req_sent <= 1'b0;
				mem_we   <= 1'b1;
				mem_addr <= {1'b0, ioctl_addr[24:0]};
				mem_din  <= ioctl_dout;
				rom_size <= ioctl_addr[24:0] + 25'd2;
			end

			// Byte order is decided once the whole file is in.
			if (!cart_stream && cart_stream_q && (rom_size != 0)) begin
				rom_present <= 1'b1;
				auto_swap   <= swap_wins;
			end

			// One request per pending access; a second one would return the
			// previous address's data.
			if (pending && !req_sent && !mem_busy && !mem_done) begin
				mem_req  <= 1'b1;
				req_sent <= 1'b1;
			end

			if (mem_done) pending <= 1'b0;
		end
	end
endmodule
