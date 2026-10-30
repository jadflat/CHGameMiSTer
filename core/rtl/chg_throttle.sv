// Board time. The board may run a cycle on any clock it does not need for
// something else (CONTRACT.md 2); this keeps it at 48 MHz on average: a
// cycle is owed every other clock, and the board runs while any are owed.
// It may catch up at most 1/60 s it fell behind.
//
// ff (fast forward): the throttle stops limiting; run stays high and the board
// goes as fast as the CPU can. owed is cleared while it is on, so on going
// back to normal the board does not race to catch up. With ff low this is
// exactly the throttle CHGame.sv had before (bit for bit).
//
// pause (the OSD is open and the option is on): run goes low, so ce stops and
// board time stands still; owed is held at 0, so on resuming the board does
// not race to catch up, it carries on at real speed at once. Pause wins over
// ff. With pause low this is the throttle above, bit for bit.

// rate (the OSD's Board clock): the board is owed cycles at 48 MHz x the chosen factor, from a
// fractional accumulator (rate / 40 of a cycle a clock, so exactly on average, no drift):
// 0 100% (20/40: a carry every other clock, the old phase bit), 1 110% (22/40), 2 125% (25/40),
// 3 150% (30/40), 4 175% (35/40), 5 200% (40/40, more than the CPU can do: it runs flat out like
// ff, the speed display says how fast). Where the CPU cannot keep up the board just goes as fast
// as it can. A change of rate clears owed (no burst). Rate 0 is the throttle above, bit for bit:
// the carry replaces the phase bit, and a carry with a cycle at once cancels, as phase did.
//
module chg_throttle
(
	input      clk,         // 96 MHz
	input [2:0] rate,
	input      ff,
	input      pause,
	input      ce,          // a board cycle happened on this clock
	output reg run
);
localparam OWED_MAX = 21'd800000;
reg        ff_r, pause_r;
reg  [2:0] rate_r;
reg  [5:0] acc;
reg [20:0] owed;
wire [5:0] num = rate_r == 3'd1 ? 6'd22 : rate_r == 3'd2 ? 6'd25 : rate_r == 3'd3 ? 6'd30 :
                 rate_r == 3'd4 ? 6'd35 : rate_r >= 3'd5 ? 6'd40 : 6'd20;
wire [6:0] sum = {1'b0, acc} + {1'b0, num};
wire       carry = sum >= 7'd40;
wire       owed_z = owed == 0, owed_1 = owed == 21'd1, owed_max = owed == OWED_MAX;
wire       owed_inc = carry && !ce && !owed_max;
wire       owed_dec = !carry && ce && !owed_z;
always @(posedge clk) begin
	ff_r  <= ff;
	pause_r <= pause;
	rate_r <= rate;
	acc <= carry ? sum - 7'd40 : sum[5:0];
	if (ff_r || pause_r || rate_r != rate) owed <= 21'd0;
	else if (owed_inc) owed <= owed + 1'd1;
	else if (owed_dec) owed <= owed - 1'd1;
	run <= pause_r ? 1'b0 : ff_r ? 1'b1 : owed_inc ? 1'b1 : owed_dec ? !owed_1 : !owed_z;
end
endmodule
