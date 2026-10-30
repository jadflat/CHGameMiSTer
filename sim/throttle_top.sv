// chg_throttle and chg_speed with a stand-in board, for sim/tb_throttle.cpp: the board runs a cycle
// on a clock when run is high and the pattern allows (stall: no cycle on that clock). Also the throttle
// CHGame.sv had before fast forward (old_run), to compare with ff low.
module throttle_top
(
	input         clk,
	input         ff,
	input         pause,
	input   [2:0] rate,
	input         stall,
	output        ce,
	output        run,
	output        old_run,
	output [11:0] bcd
);
assign ce = run && !stall;
chg_throttle thr(.clk(clk), .ff(ff), .rate(rate), .pause(pause), .ce(ce), .run(run));
chg_speed speed(.clk(clk), .ce(ce), .bcd(bcd));

// the old inline throttle, with its own ce (it must see the same one)
localparam OWED_MAX = 21'd800000;
reg        phase;
reg [20:0] owed;
reg        orun;
wire       owed_z = owed == 0, owed_1 = owed == 21'd1, owed_max = owed == OWED_MAX;
wire       owed_inc = phase && !ce && !owed_max;
wire       owed_dec = !phase && ce && !owed_z;
always @(posedge clk) begin
	phase <= ~phase;
	if (owed_inc) owed <= owed + 1'd1;
	else if (owed_dec) owed <= owed - 1'd1;
	orun <= owed_inc ? 1'b1 : owed_dec ? !owed_1 : !owed_z;
end
assign old_run = orun;
endmodule
