// The divider: OP_DIV, OP_DIVU, OP_REM and OP_REMU of cpu_exec in
// emu/src/rv32.c, with their special cases (divide by zero, and
// 0x80000000 / -1), one quotient bit a clock.
//
// It takes 17 fabric clocks (two bits a clock) with board time stopped, so its own speed is
// not seen: the C charges DIV_LATENCY in board time, and chg_cpu does that.

module chg_cpu_div
(
	input             clk,
	input             start,        // one clock, with a, b, kind
	input      [31:0] a,
	input      [31:0] b,
	input       [1:0] kind,         // 0 DIV, 1 DIVU, 2 REM, 3 REMU
	output reg        done,         // one clock, result valid from then on
	output reg [31:0] result
);

reg  [5:0] n;
reg        busy = 0;
reg  [1:0] pre;                        // after start: 1 finds the dividend's leading zero pairs, 2 skips them
reg  [3:0] lzp_r;
reg [31:0] q, rem, dv;
reg        neg_q, neg_r, want_rem;

wire        sgn = !kind[0];
wire [31:0] ua = (sgn && a[31]) ? -a : a;
wire [31:0] ub = (sgn && b[31]) ? -b : b;
// leading zero pairs of the dividend (0..15): the iterations they would take find nothing
function [3:0] lzpairs(input [31:0] v);
	integer i;
	reg found;
	begin
		lzpairs = 4'd0;
		found = 1'b0;
		for (i = 15; i >= 1; i = i - 1)
			if (!found) begin
				if (v[2*i +: 2] != 2'b00) found = 1'b1;
				else lzpairs = lzpairs + 4'd1;
			end
	end
endfunction
wire [3:0] lzp = lzpairs(q);

// two steps of restoring division a clock: shift a dividend bit into the
// remainder, subtract the divisor if it goes
wire [32:0] shifted = {rem, q[31]};                    // rem << 1 | the next dividend bit
wire [33:0] diff = {1'b0, shifted} - {2'b0, dv};
wire [31:0] rem1 = diff[33] ? shifted[31:0] : diff[31:0];
wire [32:0] shifted2 = {rem1, q[30]};
wire [33:0] diff2 = {1'b0, shifted2} - {2'b0, dv};
wire [31:0] rem2 = diff2[33] ? shifted2[31:0] : diff2[31:0];

always @(posedge clk) begin
	done <= 0;
	if (start) begin
		want_rem <= kind[1];
		if (b == 0) begin
			// DIV, DIVU: 0xffffffff; REM, REMU: the dividend
			result <= kind[1] ? a : 32'hffffffff;
			done <= 1;
			busy <= 0;
		end else if (sgn && a == 32'h80000000 && b == 32'hffffffff) begin
			result <= kind[1] ? 32'd0 : a;
			done <= 1;
			busy <= 0;
		end else begin
			q <= ua;
			rem <= 0;
			dv <= ub;
			neg_q <= sgn && (a[31] ^ b[31]);
			neg_r <= sgn && a[31];
			n <= 6'd16;
			pre <= 2'd1;
			busy <= 1;
		end
	end else if (busy && pre == 2'd1) begin
		lzp_r <= lzp;
		pre <= 2'd2;
	end else if (busy && pre == 2'd2) begin
		// the quotient only starts at the dividend's first set pair: 16 - lzp steps, not 16
		q <= q << {lzp_r, 1'b0};
		n <= 6'd16 - {2'b00, lzp_r};
		pre <= 2'd0;
	end else if (busy) begin
		if (n != 0) begin
			rem <= rem2;
			q <= {q[29:0], !diff[33], !diff2[33]};
			n <= n - 1'd1;
		end else begin
			result <= want_rem ? (neg_r ? -rem : rem) : (neg_q ? -q : q);
			done <= 1;
			busy <= 0;
		end
	end
end

endmodule
