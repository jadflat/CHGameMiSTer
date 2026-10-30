// The buzzer pin (PB10) turned into 48 kHz samples, as emu/src/audio.c does.
//
// 48 MHz / 48 kHz is exactly 1000 board cycles a sample, so the C's
// closed-form average of the pin over a sample is here a count of the cycles
// the pin is high (0..1000). Then, as chg_audio_render:
//   - a DC blocker (first order high pass at 20 Hz): dc += (level - dc) * k
//   - optionally the piezo's response, a 256-tap FIR fitted to a recording
//     (piezo_filter.c; coefficients in piezo_fir.hex, tools/gen_audio.py)
//   - times the volume, to int16, truncated toward zero as the libretro
//     core's (int16_t) cast does.
// Fixed point instead of the C's float: sim/tb_audio.cpp gives the
// difference.
//
// The samples come out in board time. Nothing here feeds back into the
// board, so the FIR may take fabric clocks without holding board time.

module chg_audio
(
	input                clk,
	input                reset,
	input                ce,
	input                pin,
	input                piezo,       // 1: the piezo filter, 0: the bare pin (the emulator's default)
	input         [11:0] vol,         // the volume, 1024 = 1.0 (the emulator's default is 0.5: 512)
	output reg signed [15:0] sample,
	output reg           strobe       // one clock when sample is new
);

// ---------------------------------------------------------------------------
// the pin averaged over each sample's 1000 cycles
// ---------------------------------------------------------------------------

reg  [9:0] cyc;
reg  [9:0] high;
reg  [9:0] level;
reg        go;

always @(posedge clk) begin
	go <= 0;
	if (reset) begin
		cyc <= 0; high <= 0;
	end
	else if (ce) begin
		if (cyc == 10'd999) begin
			level <= high + pin;
			go    <= 1;
			cyc   <= 0;
			high  <= 0;
		end
		else begin
			cyc  <= cyc + 1'd1;
			high <= high + pin;
		end
	end
end

// ---------------------------------------------------------------------------
// DC blocker, in units of 1/1000 of full scale with 24 fraction bits:
// k = 2 pi 20 / 48000 = 0.00261799, here 43922/2^24 = 0.00261795
// ---------------------------------------------------------------------------

localparam signed [16:0] K = 17'sd43922;

reg  signed [39:0] dc;
wire signed [39:0] lv = {6'd0, level, 24'd0};

// the FIR: 256 taps, one multiply a clock
reg  signed [17:0] hist[256];          // v >> 9: units of 1/128000 of full scale
reg  signed [17:0] coef[256];
// Quartus reads paths from the project folder (core/), the testbenches run from CHGameMiSTer/
`ifdef VERILATOR
initial $readmemh("core/rtl/piezo_fir.hex", coef);
`else
initial $readmemh("rtl/piezo_fir.hex", coef);
`endif

reg  [7:0] hpos;                        // where the newest sample is
reg  [8:0] tap;                         // 256 while idle
reg  [7:0] haddr;
reg  signed [17:0] hq, cq;
reg  signed [47:0] acc;
reg  signed [31:0] v;

always @(posedge clk) begin
	hq <= hist[haddr];
	cq <= coef[tap[7:0]];
end

function signed [15:0] to16(input signed [63:0] x);
	to16 = x > 64'sd32767 ? 16'sd32767 : x < -64'sd32768 ? -16'sd32768 : x[15:0];
endfunction

// truncation toward zero of x / 2^s, as a float-to-int cast does
function signed [63:0] tdiv(input signed [63:0] x, input integer s);
	tdiv = (x < 0) ? -((-x) >>> s) : (x >>> s);
endfunction

// The arithmetic a step a clock (a sample has 1000 board cycles), so that no
// path holds more than one multiply or add: 96 MHz. A wide multiply is made
// in two halves (each fits a DSP block) and added the clock after.
reg   [3:0] st;                         // the steps after go; 0 idle
reg   [2:0] fst;                        // the steps after the FIR
reg  signed [39:0] diff, dnew;
reg  signed [63:0] prod, m1, p, q, aq;
reg  signed [37:0] ph1, pl1;            // diff * K, the halves
reg  signed [33:0] mh, ml;              // (vq or fin) * vol, the halves
reg  signed [45:0] ph, pl;              // m1 * scale, the halves
reg         fin16;
wire signed [12:0] vs = $signed({1'b0, vol});
wire signed [18:0] scale = bare ? 19'sd134214 : 19'sd16777;
reg         qneg;
reg  signed [31:0] vq;
reg  signed [39:0] fin;                 // acc >>> 8, the FIR's sum to scale
reg         bare;                       // the sample in m1/p is the bare pin's (not the FIR's)
reg  signed [35:0] pr;                  // a tap's product
reg   [2:0] pipe3;


always @(posedge clk) begin
	strobe <= 0;
	if (reset) begin
		dc <= 0; hpos <= 0; tap <= 9'd256; pipe3 <= 0; sample <= 0; st <= 0; fst <= 0; fin16 <= 0;
	end
	else begin
		// dc += (level - dc) * k, then v = level - dc
		case (st)
			4'd0: if (go) begin diff <= lv - dc; st <= 4'd1; end
			4'd1: begin
				ph1 <= 38'($signed(diff[39:20]) * K);
				pl1 <= 38'($signed({1'b0, diff[19:0]}) * K);
				st <= 4'd2;
			end
			4'd2: begin prod <= (64'(ph1) <<< 20) + 64'(pl1); st <= 4'd3; end
			4'd3: begin dnew <= dc + 40'(prod >>> 24); st <= 4'd4; end
			4'd4: begin
				dc <= dnew;
				vq <= 32'((lv - dnew) >>> 8);           // Q16
				st <= 4'd5;
			end
			4'd5: begin
				v <= vq;
				hist[hpos + 1'd1] <= 18'(vq >>> 9);
				hpos <= hpos + 1'd1;
				if (piezo) begin tap <= 0; haddr <= hpos + 1'd1; acc <= 0; pipe3 <= 0; st <= 4'd0; end
				// out = v / 1000 * vol/1024 * 32767 = v_q16 * vol * (32767/1000 * 2^12) / 2^38
				else begin
					mh <= 34'($signed(vq[31:16]) * vs);
					ml <= 34'($signed({1'b0, vq[15:0]}) * vs);
					bare <= 1; fst <= 3'd1; st <= 4'd0;
				end
			end
			default: st <= 4'd0;
		endcase

		if (tap != 9'd256 || pipe3 != 0) begin
			// the address goes out on one clock, the product two later, the sum three later
			if (tap != 9'd256) begin
				tap   <= tap + 1'd1;
				haddr <= haddr - 1'd1;
			end
			// hq and cq hold the tap issued the clock before
			pipe3 <= {pipe3[1:0], tap != 9'd256};
			if (pipe3[0]) pr <= hq * cq;
			if (pipe3[1]) acc <= acc + 48'(pr);
			// acc: Q17 coefficients times units of 1/128000. out = acc / 2^17 / 128000 * vol/1024 * 32767
			//    = (acc >> 8) * vol * (32767/128000 * 2^16) / 2^35
			if (pipe3 == 3'b100 && tap == 9'd256) begin fin <= 40'(acc >>> 8); bare <= 0; fst <= 3'd6; end
		end

		// the volume and the scale, then to int16
		fin16 <= 0;
		case (fst)
			3'd6: begin
				mh <= 34'($signed(fin[39:20]) * vs);
				ml <= 34'($signed({1'b0, fin[19:0]}) * vs);
				fst <= 3'd7;
			end
			3'd7: begin m1 <= (64'(mh) <<< 20) + 64'(ml); fst <= 3'd2; end
			3'd1: begin m1 <= (64'(mh) <<< 16) + 64'(ml); fst <= 3'd2; end
			3'd2: begin
				// m1 has under 53 bits: the high half fits 27
				ph <= 46'($signed(m1[52:26]) * scale);
				pl <= 46'($signed({1'b0, m1[25:0]}) * scale);
				fst <= 3'd3;
			end
			3'd3: begin p <= (64'(ph) <<< 26) + 64'(pl); fst <= 3'd4; end
			// truncation toward zero, as tdiv
			3'd4: begin qneg <= p < 0; aq <= (p < 0) ? -p : p; fst <= 3'd5; end
			3'd5: begin
				q <= qneg ? -(aq >>> (bare ? 38 : 35)) : (aq >>> (bare ? 38 : 35));
				fst <= 3'd0;
				fin16 <= 1;                     // to16 the clock after
			end
			default: ;
		endcase
		if (fin16) begin sample <= to16(q); strobe <= 1; end
	end
end

endmodule
