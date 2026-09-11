// Copyright (c) 2026 Jamie Blanks

// The cartridge's mask ROMs live in the DE10-nano's SDRAM. Everything that
// reads or writes them runs on clk_sys and the controller runs on clk_ram,
// which is exactly twice it off the same PLL, so each port crosses once
// through `sync_xfer`: a four-phase handshake on two levels with no
// synchroniser in it, because related clocks have no metastability to protect
// against and the paths between them are timed rather than cut.
//
// Client contract, on clk_sys: pulse `req` for one cycle while `busy` is low;
// `busy` stays high until the transfer finishes; `done` pulses once with
// `dout` valid.
//
// Port 0 carries the loader and its readback; port 1 carries the cartridge's
// mask ROMs, which are read only. The loader and the console never run at the
// same time, so the two never contend.
//
// The controller side is what the controller's own contract demands: `req` is
// a level held until `busy` rises, and `busy` rising is the only
// acknowledgement that the request was taken. A request that misses its
// window is simply dropped, and `ready` never falls, so a client that waited
// on `ready` alone would read stale data and call it a result.

module sdram_bridge (
	input  wire        clk_sys,
	input  wire        reset_sys,
	input  wire        clk_ram,
	input  wire        reset_ram,

	// Port 0 client: the loader while content arrives, and its readback.
	input  wire        c0_req,
	input  wire        c0_we,
	input  wire [25:0] c0_addr,
	input  wire [15:0] c0_din,
	input  wire [1:0]  c0_be,
	output wire [15:0] c0_dout,
	output wire        c0_busy,
	output wire        c0_done,

	// Port 1 client: the cartridge's mask ROMs. Read only.
	input  wire        c1_req,
	input  wire [25:0] c1_addr,
	output wire [15:0] c1_dout,
	output wire        c1_busy,
	output wire        c1_done,

	// To the SDRAM controller, on clk_ram.
	output wire        p0_req,
	output wire        p0_we,
	output wire [25:0] p0_addr,
	output wire [63:0] p0_din,
	output wire [7:0]  p0_byte_en,
	input  wire [63:0] p0_dout,
	input  wire        p0_busy,
	input  wire        p0_ready,

	output wire        p1_req,
	output wire [25:0] p1_addr,
	input  wire [63:0] p1_dout,
	input  wire        p1_busy,
	input  wire        p1_ready
);
	wire        c0_src_busy, c0_src_done;
	// The controller zero-extends a narrow read, so only the low word carries
	// anything here.
	/* verilator lint_off UNUSEDSIGNAL */
	wire [63:0] c0_src_resp;
	/* verilator lint_on UNUSEDSIGNAL */
	wire        p0_dst_valid, p0_dst_ack;
	wire [44:0] p0_dst_data;

	sync_xfer #(.WIDTH(45), .RESP_WIDTH(64)) xfer_p0
	(
		.src_clk(clk_sys), .src_reset(reset_sys),
		.src_req(c0_req),
		.src_data({c0_we, c0_addr, c0_din, c0_be}),
		.src_busy(c0_src_busy), .src_done(c0_src_done), .src_resp(c0_src_resp),
		.dst_clk(clk_ram), .dst_reset(reset_ram),
		.dst_valid(p0_dst_valid), .dst_data(p0_dst_data),
		.dst_ack(p0_dst_ack), .dst_resp(p0_dout)
	);

	assign c0_busy = c0_src_busy;
	assign c0_done = c0_src_done;
	assign c0_dout = c0_src_resp[15:0];

	assign p0_we      = p0_dst_data[44];
	assign p0_addr    = p0_dst_data[43:18];
	assign p0_din     = {48'd0, p0_dst_data[17:2]};
	assign p0_byte_en = {6'd0, p0_dst_data[1:0]};

	// Port 1 is read only, so only the address crosses.
	/* verilator lint_off UNUSEDSIGNAL */
	wire [63:0] c1_src_resp;
	/* verilator lint_on UNUSEDSIGNAL */
	wire        p1_dst_valid, p1_dst_ack;

	sync_xfer #(.WIDTH(26), .RESP_WIDTH(64)) xfer_p1
	(
		.src_clk(clk_sys), .src_reset(reset_sys),
		.src_req(c1_req),
		.src_data(c1_addr),
		.src_busy(c1_busy), .src_done(c1_done), .src_resp(c1_src_resp),
		.dst_clk(clk_ram), .dst_reset(reset_ram),
		.dst_valid(p1_dst_valid), .dst_data(p1_addr),
		.dst_ack(p1_dst_ack), .dst_resp(p1_dout)
	);

	assign c1_dout = c1_src_resp[15:0];

	// The request lines are registered because they reach the controller's
	// arbitration and from there its address mux and the SDRAM_A pins, which is
	// the longest path in the clk_ram domain. One cycle later costs nothing:
	// the request is a level and the client is waiting anyway.
	reg p0_in_flight, p1_in_flight;
	reg p0_req_q, p1_req_q;

	assign p0_req = p0_req_q;
	assign p1_req = p1_req_q;
	assign p0_dst_ack = p0_in_flight & p0_ready;
	assign p1_dst_ack = p1_in_flight & p1_ready;

	always @(posedge clk_ram) begin
		if (reset_ram) begin
			p0_in_flight <= 1'b0;
			p1_in_flight <= 1'b0;
			p0_req_q     <= 1'b0;
			p1_req_q     <= 1'b0;
		end else begin
			p0_req_q <= p0_dst_valid & ~p0_in_flight;
			p1_req_q <= p1_dst_valid & ~p1_in_flight;

			if (p0_req && !p0_busy)                  p0_in_flight <= 1'b1;
			else if (p0_in_flight && p0_ready)       p0_in_flight <= 1'b0;

			if (p1_req && !p1_busy)                  p1_in_flight <= 1'b1;
			else if (p1_in_flight && p1_ready)       p1_in_flight <= 1'b0;
		end
	end
endmodule
