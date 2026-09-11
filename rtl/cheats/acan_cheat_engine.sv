// Copyright (c) 2026 Jamie Blanks

// MiSTer cheat engine on the 68000's read seam. The record is the standard
// 16-byte MiSTer one:
//
//   128       record-valid strobe
//   105:104   replacement method (0 replace, 1 OR, 2 AND)
//   102:100   width (000/001 byte, 010 16-bit, 100 32-bit)
//   96        compare enable
//   95:64     physical byte address, 24 bits used
//   63:32     compare value
//   31:0      replacement value
//
// Multi-byte records expand big endian. Entries are matched on address only,
// inside the 68000's address window, leaving one byte compare and one
// replace/OR/AND per lane on the data path:
//
//   S0 S1  S2 S3  S4 S5  S6 S7      68000 read cycle, 5 clk_sys per state pair
//   |      |                    |
//   |      +- !UDS/!LDS assert  +- CPU latches the bus here
//   +- address valid
//
//   scan   ....xxxxx.............   1 clock of M10K latency + 4 groups of 8
//
// Only the newest address match survives. Both byte lanes share one
// comparator per entry, address bit 0 picking the lane. Four mirrored 64x43
// M10K tables give eight read ports over the active 32-entry bank while the
// next generation is written into the other; bank and count switch together
// once the machine is parked.
module acan_cheat_engine
(
	input  wire         clk_sys_i,

	// Loader handshake.
	input  wire         load_begin_i,
	input  wire         invalidate_i,
	input  wire         commit_req_i,
	input  wire         paused_i,
	input  wire [128:0] code_i,
	output reg          commit_done_o,

	// The 68000's own bus, so a DMA cycle cannot disturb the scan key.
	input  wire         enable_i,   // read cycle in progress
	input  wire [23:1]  addr_i,
	input  wire  [1:0]  be_i,       // {!UDS, !LDS}, active high
	input  wire [15:0]  data_i,
	output wire [15:0]  data_o
);

	localparam [5:0] MAX_CODES_COUNT = 6'd32;
	localparam [1:0] METHOD_REPLACE = 2'd0;
	localparam [1:0] METHOD_OR      = 2'd1;
	localparam [1:0] METHOD_AND     = 2'd2;

	wire        code_valid_w          = code_i[128];
	wire  [1:0] code_method_w         = code_i[105:104];
	wire  [2:0] code_width_w          = code_i[102:100];
	wire        code_compare_enable_w = code_i[96];
	wire  [7:0] code_addr_high_w      = code_i[95:88];
	wire [23:0] code_addr_w           = code_i[87:64];
	wire [31:0] code_compare_w        = code_i[63:32];
	wire [31:0] code_value_w          = code_i[31:0];

	// ---- the staging bank -------------------------------------------------

	reg        active_bank_q;
	reg        load_active_q;
	reg  [5:0] active_count_q;
	reg  [5:0] staging_count_q;
	reg  [2:0] pending_count_q;
	reg [23:0] pending_addr_q;
	reg [31:0] pending_value_q;
	reg [31:0] pending_compare_q;
	reg        pending_compare_enable_q;
	reg  [1:0] pending_method_q;

	// Entry layout: address[42:19], compare[18:11], value[10:3], compare
	// enable[2], method[1:0].
	wire        table_write_w = load_active_q && (pending_count_q != 3'd0);
	wire  [5:0] table_write_addr_w = {~active_bank_q, staging_count_q[4:0]};
	wire [42:0] table_write_data_w = {
		pending_addr_q,
		pending_compare_q[31:24],
		pending_value_q[31:24],
		pending_compare_enable_q,
		pending_method_q
	};

	// commit_req_i is a level from the loader, acknowledged only once the
	// last wide record has finished expanding.
	wire commit_w = commit_req_i && paused_i && load_active_q &&
		!code_valid_w && (pending_count_q == 3'd0);

	// ---- the scan ---------------------------------------------------------

	reg [23:1] key_addr_q;
	reg  [1:0] key_be_q;
	reg  [5:0] key_count_q;
	reg        key_bank_q;
	reg        scan_active_q;
	reg  [1:0] scan_group_q;

	// The key includes the generation, so a commit rescans immediately.
	wire scan_restart_w = (addr_i != key_addr_q) || (be_i != key_be_q) ||
		(active_count_q != key_count_q) || (active_bank_q != key_bank_q);

	wire [1:0] next_group_w = scan_group_q + 2'd1;
	wire [4:0] scan_base_w = scan_restart_w ? 5'd0 : {next_group_w, 3'b000};

	// Port A doubles as the staging write port; staging only runs while parked.
	wire [343:0] entries_w;

	cache_ram_dp #(.ADDR_WIDTH(6), .DATA_WIDTH(43)) table0
	(
		.clk_i     (clk_sys_i),
		.addr_a_i  (table_write_w ? table_write_addr_w
		                          : {active_bank_q, scan_base_w + 5'd0}),
		.wren_a_i  (table_write_w),
		.wdata_a_i (table_write_data_w),
		.q_a_o     (entries_w[0 * 43 +: 43]),
		.addr_b_i  ({active_bank_q, scan_base_w + 5'd1}),
		.wren_b_i  (1'b0),
		.wdata_b_i (43'd0),
		.q_b_o     (entries_w[1 * 43 +: 43])
	);

	cache_ram_dp #(.ADDR_WIDTH(6), .DATA_WIDTH(43)) table1
	(
		.clk_i     (clk_sys_i),
		.addr_a_i  (table_write_w ? table_write_addr_w
		                          : {active_bank_q, scan_base_w + 5'd2}),
		.wren_a_i  (table_write_w),
		.wdata_a_i (table_write_data_w),
		.q_a_o     (entries_w[2 * 43 +: 43]),
		.addr_b_i  ({active_bank_q, scan_base_w + 5'd3}),
		.wren_b_i  (1'b0),
		.wdata_b_i (43'd0),
		.q_b_o     (entries_w[3 * 43 +: 43])
	);

	cache_ram_dp #(.ADDR_WIDTH(6), .DATA_WIDTH(43)) table2
	(
		.clk_i     (clk_sys_i),
		.addr_a_i  (table_write_w ? table_write_addr_w
		                          : {active_bank_q, scan_base_w + 5'd4}),
		.wren_a_i  (table_write_w),
		.wdata_a_i (table_write_data_w),
		.q_a_o     (entries_w[4 * 43 +: 43]),
		.addr_b_i  ({active_bank_q, scan_base_w + 5'd5}),
		.wren_b_i  (1'b0),
		.wdata_b_i (43'd0),
		.q_b_o     (entries_w[5 * 43 +: 43])
	);

	cache_ram_dp #(.ADDR_WIDTH(6), .DATA_WIDTH(43)) table3
	(
		.clk_i     (clk_sys_i),
		.addr_a_i  (table_write_w ? table_write_addr_w
		                          : {active_bank_q, scan_base_w + 5'd6}),
		.wren_a_i  (table_write_w),
		.wdata_a_i (table_write_data_w),
		.q_a_o     (entries_w[6 * 43 +: 43]),
		.addr_b_i  ({active_bank_q, scan_base_w + 5'd7}),
		.wren_b_i  (1'b0),
		.wdata_b_i (43'd0),
		.q_b_o     (entries_w[7 * 43 +: 43])
	);

	// ---- one comparator per entry -----------------------------------------

	// !UDS carries the even byte on D[15:8], !LDS the odd byte on D[7:0].
	// The entry's address bit 0 picks the lane.
	wire [7:0] entry_hit_w;
	wire [7:0] entry_odd_w;
	wire [7:0] entry_live_w;

	genvar ei;
	generate
		for (ei = 0; ei < 8; ei = ei + 1) begin : g_entry
			localparam [2:0] IDX = 3'(ei);
			assign entry_hit_w[ei]  = (entries_w[ei * 43 + 20 +: 23] == addr_i);
			assign entry_odd_w[ei]  = entries_w[ei * 43 + 19];
			assign entry_live_w[ei] =
				({1'b0, scan_group_q, IDX} < active_count_q);
		end
	endgenerate

	// ---- per byte lane ----------------------------------------------------

	function [7:0] apply_method_fn;
		input [1:0] method_i;
		input [7:0] read_byte_i;
		input [7:0] value_i;
		begin
			case (method_i)
				METHOD_OR:  apply_method_fn = read_byte_i | value_i;
				METHOD_AND: apply_method_fn = read_byte_i & value_i;
				default:    apply_method_fn = value_i;
			endcase
		end
	endfunction

	wire [15:0] patched_w;

	genvar l;
	generate
		for (l = 0; l < 2; l = l + 1) begin : g_lane
			localparam [0:0] LANE_ODD = (l == 0) ? 1'b1 : 1'b0;

			wire [7:0] raw_byte = data_i[l * 8 +: 8];

			// One group's eight entries, in index order so the newest wins.
			reg       grp_hit;
			reg [1:0] grp_method;
			reg [7:0] grp_value;
			reg       grp_cmp_en;
			reg [7:0] grp_cmp;

			always @* begin : group_select
				integer e;

				grp_hit    = 1'b0;
				grp_method = METHOD_REPLACE;
				grp_value  = 8'd0;
				grp_cmp_en = 1'b0;
				grp_cmp    = 8'd0;

				for (e = 0; e < 8; e = e + 1) begin
					if (be_i[l] && entry_live_w[e] && entry_hit_w[e] &&
						(entry_odd_w[e] == LANE_ODD)) begin
						grp_hit    = 1'b1;
						grp_method = entries_w[e * 43 +: 2];
						grp_value  = entries_w[e * 43 + 3 +: 8];
						grp_cmp_en = entries_w[e * 43 + 2];
						grp_cmp    = entries_w[e * 43 + 11 +: 8];
					end
				end
			end

			// Carried across the four groups, then published to the data path.
			reg       acc_hit;
			reg [1:0] acc_method;
			reg [7:0] acc_value;
			reg       acc_cmp_en;
			reg [7:0] acc_cmp;

			reg       res_hit;
			reg [1:0] res_method;
			reg [7:0] res_value;
			reg       res_cmp_en;
			reg [7:0] res_cmp;

			// Power-up state only; a console reset must not remove cheats.
			initial begin
				acc_hit = 1'b0;
				acc_method = METHOD_REPLACE;
				acc_value = 8'd0;
				acc_cmp_en = 1'b0;
				acc_cmp = 8'd0;
				res_hit = 1'b0;
				res_method = METHOD_REPLACE;
				res_value = 8'd0;
				res_cmp_en = 1'b0;
				res_cmp = 8'd0;
			end

			always @(posedge clk_sys_i) begin
				if (invalidate_i || load_begin_i || commit_w || table_write_w) begin
					res_hit <= 1'b0;
				end else if (scan_restart_w) begin
					acc_hit <= 1'b0;
					res_hit <= 1'b0;
				end else if (scan_active_q) begin
					if (grp_hit) begin
						acc_hit    <= 1'b1;
						acc_method <= grp_method;
						acc_value  <= grp_value;
						acc_cmp_en <= grp_cmp_en;
						acc_cmp    <= grp_cmp;
					end

					// The last group is the newest, so it overwrites whatever the
					// three earlier ones selected.
					if (scan_group_q == 2'd3) begin
						res_hit    <= grp_hit ? 1'b1       : acc_hit;
						res_method <= grp_hit ? grp_method : acc_method;
						res_value  <= grp_hit ? grp_value  : acc_value;
						res_cmp_en <= grp_hit ? grp_cmp_en : acc_cmp_en;
						res_cmp    <= grp_hit ? grp_cmp    : acc_cmp;
					end
				end
			end

			// Only the compare waits for read data.
			wire pass = res_hit && (!res_cmp_en || (res_cmp == raw_byte));

			assign patched_w[l * 8 +: 8] = (enable_i && be_i[l] && pass)
				? apply_method_fn(res_method, raw_byte, res_value) : raw_byte;
		end
	endgenerate

	assign data_o = patched_w;

	// ---- record acceptance ------------------------------------------------

	reg [2:0] record_byte_count;
	reg       record_aligned;

	// Only byte/16/32-bit widths at natural alignment, and only while every
	// expanded byte still fits in the table.
	always @* begin
		case (code_width_w)
			3'b000, 3'b001: begin
				record_byte_count = 3'd1;
				record_aligned = 1'b1;
			end
			3'b010: begin
				record_byte_count = 3'd2;
				record_aligned = !code_addr_w[0];
			end
			3'b100: begin
				record_byte_count = 3'd4;
				record_aligned = !(|code_addr_w[1:0]);
			end
			default: begin
				record_byte_count = 3'd0;
				record_aligned = 1'b0;
			end
		endcase
	end

	// Big endian expansion: the byte for the record's own address sits in
	// [31:24] and the rest shift up behind it.
	reg [31:0] load_value;
	reg [31:0] load_compare;
	always @* begin
		case (record_byte_count)
			3'd1: begin
				load_value   = {code_value_w[7:0], 24'd0};
				load_compare = {code_compare_w[7:0], 24'd0};
			end
			3'd2: begin
				load_value   = {code_value_w[15:0], 16'd0};
				load_compare = {code_compare_w[15:0], 16'd0};
			end
			default: begin
				load_value   = code_value_w;
				load_compare = code_compare_w;
			end
		endcase
	end

	initial begin
		active_bank_q = 1'b0;
		load_active_q = 1'b0;
		active_count_q = 6'd0;
		staging_count_q = 6'd0;
		pending_count_q = 3'd0;
		pending_addr_q = 24'd0;
		pending_value_q = 32'd0;
		pending_compare_q = 32'd0;
		pending_compare_enable_q = 1'b0;
		pending_method_q = METHOD_REPLACE;
		key_addr_q = 23'd0;
		key_be_q = 2'd0;
		key_count_q = 6'd0;
		key_bank_q = 1'b0;
		scan_active_q = 1'b0;
		scan_group_q = 2'd0;
		commit_done_o = 1'b0;
	end

	// Reserved record bits.
	`ifdef VERILATOR
	/* verilator lint_off UNUSED */
	wire unused_ok = &{1'b0, code_i[127:106], code_i[103], code_i[99:97], 1'b0};
	/* verilator lint_on UNUSED */
	`endif

	always @(posedge clk_sys_i) begin
		commit_done_o <= 1'b0;

		if (invalidate_i) begin
			active_count_q <= 6'd0;
			staging_count_q <= 6'd0;
			pending_count_q <= 3'd0;
			load_active_q <= 1'b0;
			scan_active_q <= 1'b0;
		end else if (load_begin_i) begin
			// The active generation stays visible until commit.
			staging_count_q <= 6'd0;
			pending_count_q <= 3'd0;
			load_active_q <= 1'b1;
			// The table read runs a group ahead of the index, and this branch
			// does not advance the index, so a running scan would go wrong.
			scan_active_q <= 1'b0;
		end else begin
			if (commit_w) begin
				active_bank_q <= ~active_bank_q;
				active_count_q <= staging_count_q;
				load_active_q <= 1'b0;
				commit_done_o <= 1'b1;
				scan_active_q <= 1'b0;
			end else if (table_write_w) begin
				staging_count_q <= staging_count_q + 6'd1;
				pending_addr_q <= pending_addr_q + 24'd1;
				pending_value_q <= {pending_value_q[23:0], 8'd0};
				pending_compare_q <= {pending_compare_q[23:0], 8'd0};
				pending_count_q <= pending_count_q - 3'd1;
				scan_active_q <= 1'b0;
			end else if (scan_restart_w) begin
				key_addr_q <= addr_i;
				key_be_q <= be_i;
				key_count_q <= active_count_q;
				key_bank_q <= active_bank_q;
				scan_active_q <= 1'b1;
				scan_group_q <= 2'd0;
			end else if (scan_active_q) begin
				if (scan_group_q == 2'd3) scan_active_q <= 1'b0;
				else                      scan_group_q <= scan_group_q + 2'd1;
			end

			if (load_active_q && !commit_w && code_valid_w && !table_write_w &&
				!(|code_addr_high_w) && record_aligned &&
				(record_byte_count != 3'd0) &&
				(({1'b0, staging_count_q} + {4'd0, record_byte_count}) <=
					{1'b0, MAX_CODES_COUNT})) begin
				pending_addr_q <= code_addr_w;
				pending_value_q <= load_value;
				pending_compare_q <= load_compare;
				pending_compare_enable_q <= code_compare_enable_w;
				pending_method_q <= code_method_w;
				pending_count_q <= record_byte_count;
			end
		end
	end

endmodule
