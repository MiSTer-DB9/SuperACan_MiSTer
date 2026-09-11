// Copyright (c) 2026 Jamie Blanks

// Priority and colour composition in palette-index space. Lower priorities
// are in front. A descriptor's tie bit can move it ahead of an equal-priority
// normal layer or sprite; ROZ keeps precedence over equal normal layers.

module um6618_mixer (
	input  wire        clk_sys,
	input  wire [31:0] tm_index,
	input  wire [3:0]  tm_opaque,
	input  wire [11:0] tm_priority,
	input  wire [3:0]  tm_enable,
	input  wire [3:0]  tm_tie,
	input  wire [3:0]  tm_add,
	input  wire [7:0]  roz_index,
	input  wire        roz_opaque,
	input  wire [2:0]  roz_priority,
	input  wire        roz_enable,
	input  wire        roz_tie,
	input  wire        roz_add,
	input  wire [15:0] win_pen,
	input  wire [1:0]  win_inside,
	input  wire [5:0]  win_priority,
	input  wire [1:0]  win_enable,
	input  wire [7:0]  spr_index,
	input  wire [1:0]  spr_priority,
	input  wire [1:0]  spr_flags,
	input  wire [7:0]  spr_backup,
	input  wire        spr_enable,
	input  wire        line_masked,
	output wire [7:0]  index
);
	localparam int N = 7;

	// Equal-priority visits are BG0..3, ROZ, then windows0..1. An ordinary
	// layer replaces an earlier equal-priority layer only when it raises T.
	wire [N-1:0] valid = {win_enable & win_inside, roz_enable & roz_opaque,
	                      tm_enable & tm_opaque};
	wire [N*8-1:0] colour = {win_pen, roz_index, tm_index};
	wire [N*3-1:0] priority_i = {win_priority, roz_priority, tm_priority};
	wire [N-1:0] tie_i = {2'b11, roz_tie, tm_tie};
	wire [N-1:0] add_i = {2'b00, roz_add, tm_add};
	wire [N-1:0] eligible;
	wire [N*N-1:0] nearer;

	genvar i, j;
	generate
		for (i = 0; i < N; i = i + 1) begin : candidates
			wire [N-1:0] blocked;
			for (j = 0; j < N; j = j + 1) begin : peers
				assign nearer[i*N+j] = (priority_i[j*3 +: 3] < priority_i[i*3 +: 3])
				                          | ((j > i) && (priority_i[j*3 +: 3] == priority_i[i*3 +: 3]));
				// On a tie a window never stands behind a background or ROZ.
				// Sango Fighter's character select closes with a window whose
				// priority equals the layer it covers.
				assign blocked[j] = (j < i) && valid[j]
				                     && (priority_i[j*3 +: 3] == priority_i[i*3 +: 3])
				                     && ((i < 4) ? (tie_i[j] || !tie_i[i])
				                                 : ((i > 4) && (j > 4)));
			end
			assign eligible[i] = valid[i] && !(|blocked);
		end
	endgenerate

	// These two pipeline stages run inside a dot. They do not delay the pixel
	// enable: the line-buffer output settles well before the next dot latch.
	reg [N-1:0] eligible_q, tie_q, add_q;
	reg [N*N-1:0] nearer_q;
	reg [N*8-1:0] colour_q;
	reg [N*3-1:0] priority_q;
	reg [7:0] sprite_q, backup_q;
	reg [1:0] sprite_priority_q, flags_q;
	reg sprite_valid_q, masked_q;

	always @(posedge clk_sys) begin
		eligible_q <= eligible;
		nearer_q <= nearer;
		colour_q <= colour;
		priority_q <= priority_i;
		tie_q <= tie_i;
		add_q <= add_i;
		sprite_q <= spr_index;
		backup_q <= spr_backup;
		sprite_priority_q <= spr_priority;
		flags_q <= spr_flags;
		sprite_valid_q <= spr_enable && (spr_index != 8'd0);
		masked_q <= line_masked;
	end

	// A sprite can supply the base colour once, at the first additive layer
	// ahead of it. An equal-priority background with T set covers that sprite.
	wire [N-1:0] ahead, behind, first_ahead, sprite_covered, inject;
	wire [N-1:0] top, barrier, contributes;
	wire sprite_base = sprite_valid_q && !(|sprite_covered);
	generate
		for (i = 0; i < N; i = i + 1) begin : composition
			wire [N-1:0] farther;
			for (j = 0; j < N; j = j + 1) begin : reverse_order
				assign farther[j] = nearer_q[j*N+i];
			end
			assign ahead[i] = eligible_q[i] && (priority_q[i*3 +: 3] < {1'b0, sprite_priority_q});
			assign behind[i] = eligible_q[i] && !ahead[i];
			assign first_ahead[i] = ahead[i] && !(|(farther & ahead));
			assign sprite_covered[i] = behind[i] && !(|(nearer_q[i*N +: N] & behind))
			                           && (priority_q[i*3 +: 3] == {1'b0, sprite_priority_q}) && tie_q[i];
			assign inject[i] = first_ahead[i] && add_q[i] && sprite_base;
			assign barrier[i] = (eligible_q[i] && !add_q[i]) || inject[i];
			assign contributes[i] = eligible_q[i] && !(|(nearer_q[i*N +: N] & barrier));
			assign top[i] = eligible_q[i] && !(|(nearer_q[i*N +: N] & eligible_q));
		end
	endgenerate

	reg [3:0] bg_priority;
	reg bg_tie;
	integer k;
	always @(*) begin
		bg_priority = 4'd15;
		bg_tie = 1'b0;
		for (k = 0; k < N; k = k + 1) begin
			if (top[k]) begin
				bg_priority = {1'b0, priority_q[k*3 +: 3]};
				bg_tie = tie_q[k];
			end
		end
	end

	// An opaque write stops contributions from every farther layer. Additive
	// writes keep them, so a balanced sum replaces a long chain of pixel adds.
	reg [63:0] terms_q;
	reg [3:0] bg_priority_q;
	reg bg_tie_q;
	reg [7:0] sprite_final_q, backup_final_q;
	reg [1:0] sprite_priority_final_q, flags_final_q;
	reg sprite_valid_final_q, masked_final_q;
	always @(posedge clk_sys) begin
		for (integer n = 0; n < N; n = n + 1)
			terms_q[n*8 +: 8] <= contributes[n] ? colour_q[n*8 +: 8] : 8'd0;
		terms_q[63:56] <= (|(inject & contributes)) ? sprite_q : 8'd0;
		bg_priority_q <= bg_priority;
		bg_tie_q <= bg_tie;
		sprite_final_q <= sprite_q;
		backup_final_q <= backup_q;
		sprite_priority_final_q <= sprite_priority_q;
		flags_final_q <= flags_q;
		sprite_valid_final_q <= sprite_valid_q;
		masked_final_q <= masked_q;
	end

	wire [7:0] sum01 = 8'(terms_q[7:0] + terms_q[15:8]);
	wire [7:0] sum23 = 8'(terms_q[23:16] + terms_q[31:24]);
	wire [7:0] sum45 = 8'(terms_q[39:32] + terms_q[47:40]);
	wire [7:0] sum67 = 8'(terms_q[55:48] + terms_q[63:56]);
	wire [7:0] sum03 = 8'(sum01 + sum23);
	wire [7:0] sum47 = 8'(sum45 + sum67);
	wire [7:0] bg_index = 8'(sum03 + sum47);
	wire sprite_hit = sprite_valid_final_q
	                   && (({2'd0, sprite_priority_final_q} < bg_priority_q)
	                       || (({2'd0, sprite_priority_final_q} == bg_priority_q) && !bg_tie_q));
	wire [3:0] under = (backup_final_q != 8'd0) ? backup_final_q[3:0] : bg_index[3:0];

	// A blend sprite keeps its own palette row and adds only the pens, with
	// no carry: African Adventure's logo shadow (row 0, pen 6) over background
	// entries $11-$13 shows entries $07-$09 on the console, not $17-$19.
	wire [3:0] blend_pen = 4'(sprite_final_q[3:0] + under);
	assign index = !sprite_hit ? bg_index
	             : !flags_final_q[1] ? sprite_final_q
	             : masked_final_q ? (flags_final_q[0] ? sprite_final_q : bg_index)
	             : {sprite_final_q[7:4], blend_pen};
endmodule
