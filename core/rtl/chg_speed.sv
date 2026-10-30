// The board's speed as a percent of real time, for the speed display: board
// cycles (ce) counted over 0.5 s of wall time (48,000,000 clocks of 96 MHz);
// 24,000,000 is 100%. Out every half second as 3 BCD digits, rounded, 999 at
// most. The division is a subtraction of 240,000 (1%) a clock, so it is done
// within a thousand clocks.

module chg_speed
(
	input             clk,      // 96 MHz
	input             ce,
	output reg [11:0] bcd       // {hundreds, tens, units}
);
reg [25:0] wall = 0, cnt = 0, rem = 0;
reg        busy = 0;
reg [11:0] tmp = 0;
wire       tick = wall == 26'd47999999;
wire       more = rem >= 26'd240000;
wire       full = tmp == 12'h999;
wire [3:0] u1 = tmp[3:0] == 4'd9 ? 4'd0 : tmp[3:0] + 1'd1;
wire       cu = tmp[3:0] == 4'd9;
wire [3:0] t1 = cu ? (tmp[7:4] == 4'd9 ? 4'd0 : tmp[7:4] + 1'd1) : tmp[7:4];
wire       ct = cu && tmp[7:4] == 4'd9;
wire [3:0] h1 = ct ? tmp[11:8] + 1'd1 : tmp[11:8];

always @(posedge clk) begin
	wall <= tick ? 26'd0 : wall + 1'd1;
	if (tick) begin
		rem  <= cnt + (ce ? 26'd120001 : 26'd120000);     // + half a percent: rounds
		cnt  <= 0;
		tmp  <= 0;
		busy <= 1;
	end
	else begin
		cnt <= cnt + ce;
		if (busy) begin
			if (more && !full) begin
				rem <= rem - 26'd240000;
				tmp <= {h1, t1, u1};
			end
			else begin
				bcd  <= tmp;
				busy <= 0;
			end
		end
	end
end
endmodule
