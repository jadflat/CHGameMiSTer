// RCC at 0x4002_1000: eleven plain registers, and the reset-cause flags.
//
// Translates the RCC cases of bus_read_slow / bus_write_slow, bus_reset's
// RCC part and the reset-flag lines of chg_reset in emu/src (bus.c,
// machine.c). Nothing here does anything to the clocks: the C does not.
//
//   CTLR (0)     reads with HSION and HSIRDY set; resets to 0x83
//   CFGR0 (1)    SWS (bits 3:2) reads as SW (bits 1:0)
//   RSTSCKR (9)  bits 23:0 are a register (LSIRDY reads as LSION); bits
//                31:25 are the reset cause, m->reset_flags, which survive a
//                software reset: a power-on sets PINRSTF, PORRSTF and
//                SFTRSTF (0x1C00_0000, the board's factory boot code
//                enters user flash with a software reset), a software reset
//                adds SFTRSTF, a write with RMVF (bit 24) clears them
//   others < 11  read back what was written; offsets from 0x2C read 0

module chg_rcc
(
	input             clk,
	input             power_on_reset,   // chg_reset(m, true)
	input             soft_reset,       // chg_reset(m, false): PFIC SYSRESET
	input             ce,               // not used

	input             req,
	input             we,
	input      [31:0] addr,
	input       [2:0] size,
	input      [31:0] wdata,
	output reg [31:0] rdata,
	output reg        ack
);

reg [31:0] rcc[0:10];
reg [31:0] flags;               // m->reset_flags

wire [4:0]  sh  = {addr[1:0], 3'b000};
wire [31:0] wv  = wdata << sh;
wire [7:0]  r   = addr[9:2];

function [31:0] lane_r(input [31:0] x0, input [4:0] s, input [2:0] sz);
	reg [31:0] x;
	begin
		x = x0 >> s;
		if (sz == 3'd1) x = x & 32'h000000ff;
		else if (sz == 3'd2) x = x & 32'h0000ffff;
		lane_r = x;
	end
endfunction

reg [31:0] rv;
always @* begin
	rv = (r < 8'd11) ? rcc[r[3:0]] : 32'd0;
	if (r == 8'd0) rv = rv | 32'h3;
	if (r == 8'd1) rv = (rv & ~32'hc) | {28'd0, rv[1:0], 2'b00};
	if (r == 8'd9) begin
		rv = (rv & 32'h00ffffff) | flags;
		if (rv[0]) rv = rv | 32'h2;
	end
end

integer k;
always @(posedge clk) begin
	ack <= 0;
	if (power_on_reset || soft_reset) begin
		for (k = 0; k < 11; k = k + 1) rcc[k] <= 0;
		rcc[0] <= 32'h00000083;
		if (power_on_reset) flags <= 32'h1C000000;
		else flags <= flags | 32'h10000000;
	end else if (req) begin
		ack <= 1;
		if (!we)
			rdata <= lane_r(rv, sh, size);
		else if (r == 8'd9) begin
			if (wv[24]) flags <= 0;
			rcc[9] <= wv & 32'h00ffffff;
		end else if (r < 8'd11)
			rcc[r[3:0]] <= wv;
	end
end

endmodule
