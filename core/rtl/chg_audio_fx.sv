// Sound models, a 7 band graphic equaliser and a spectrum analyser on the
// 48 kHz samples chg_audio makes, between its `sample` and AUDIO_L/R.
//
// One engine, one 27 x 27 multiplier, time-shared: a sample is run through up
// to 17 biquads (direct form I, 27 bit data = 16 bit sample with 8 fraction
// bits and 3 bits of headroom, coefficients Q3.24 from audiofx_coefs.hex,
// made by tools/eq_coefs.py), a multiply-add every 5 clocks:
//
//   slot 0       the EQ's pre-gain (-3 .. -12 dB, so the boosts do not clip)
//   slots 1-2    the sound model (model 2-7: two biquads of a transducer)
//   slots 3-9    the EQ's seven bands (63 160 400 1k 2.5k 6.3k 16k Hz)
//   slots 10-16  the analyser's band pass filters, on the result, each
//                followed by an envelope follower and a log scale
//
// A slot that does nothing (flat band, no pre-gain, model 0/1) is skipped, and
// with the sound model 0 or 1, the EQ off, `out` is `sample` itself, delayed
// one clock: the same stream. Model 1 is chg_audio's own piezo filter (the
// FIR in chg_audio.sv), which is not touched here.
//
// The rounding of each stage's result is carried to the next sample (first order
// noise shaping): a low band's poles are so near 1 that plain rounding noise
// would come out 80 dB louder at 63 Hz.
//
// A sample takes under 800 clocks; the next one arrives 1000 (fast forward)
// or 2000 clocks later. Every result is saturated: the data to 27 bits in
// each stage and the output to 16 bits.
//
// model   0 Bare pin   1 Piezo filter (chg_audio)   2 Small piezo
//         3 Large buzzer   4 Speaker   5 Warm   6 Handheld   7 Bright
// preset  0 Off  1 Bass boost  2 Treble boost  3 Loudness  4 Vocal  5 Custom (bands)
// bands   seven 4 bit fields, band 0 in bits 3:0; 0 = 0 dB, 1-4 = +3..+12 dB,
//         5-8 = -12..-3 dB (so that an untouched OSD is flat)
// spec    0 off, else the analyser runs (the peaks are always made)
// bars/peaks: seven 4 bit heights (0-15, 3 dB a step from about -48 dBFS)

module chg_audio_fx
(
	input                    clk,
	input                    reset,
	input  signed     [15:0] sample,
	input                    strobe,     // sample is new
	input              [2:0] model,
	input              [2:0] preset,
	input             [27:0] bands,
	input                    spec,
	output reg signed [15:0] out,
	output reg        [27:0] bars,
	output reg        [27:0] peaks
);

// ---------------------------------------------------------------------------
// the settings, as gain indexes 0-8 = -12 .. +12 dB (4 = flat)
// ---------------------------------------------------------------------------

function [3:0] gidx(input [3:0] v);
	gidx = (v <= 4'd4) ? 4'd4 + v : (v <= 4'd8) ? v - 4'd5 : 4'd4;
endfunction

function [3:0] pgi(input [2:0] p, input [2:0] b);
	case (p)
		3'd1: pgi = (b == 0) ? 4'd7 : (b == 1) ? 4'd6 : (b == 2) ? 4'd5 : 4'd4;                  // bass boost
		3'd2: pgi = (b == 4) ? 4'd5 : (b == 5) ? 4'd6 : (b == 6) ? 4'd7 : 4'd4;                  // treble boost
		3'd3: pgi = (b == 0) ? 4'd7 : (b == 1) ? 4'd6 : (b == 5) ? 4'd5 : (b == 6) ? 4'd6 : 4'd4; // loudness
		3'd4: pgi = (b == 0) ? 4'd2 : (b == 1) ? 4'd3 : (b == 3) ? 4'd5 : (b == 4) ? 4'd6 : (b == 5) ? 4'd5 : (b == 6) ? 4'd3 : 4'd4; // vocal
		default: pgi = 4'd4;
	endcase
endfunction

// the biggest boost, in 3 dB steps
function [2:0] maxboost(input [27:0] g);
	integer i;
	reg [2:0] r;
	begin
		r = 0;
		for (i = 0; i < 7; i = i + 1)
			if (g[4*i +: 4] > 4'd4 && 3'(g[4*i +: 4] - 4'd4) > r) r = 3'(g[4*i +: 4] - 4'd4);
		maxboost = r;
	end
endfunction

reg [27:0] gip;                 // seven gain indexes
reg  [2:0] pg, pg0;             // pre-gain: 0 none, 1-4 = -3 .. -12 dB
reg        eq_on, active;
integer    k1;
always @(posedge clk) begin
	eq_on  <= preset >= 3'd1 && preset <= 3'd5;
	active <= model >= 3'd2 || (preset >= 3'd1 && preset <= 3'd5);
	for (k1 = 0; k1 < 7; k1 = k1 + 1)
		gip[4*k1 +: 4] <= (preset == 3'd5) ? gidx(bands[4*k1 +: 4]) : pgi(preset, 3'(k1));
	pg0 <= maxboost(gip);
	pg  <= pg0;
end

// ---------------------------------------------------------------------------
// the coefficient ROM and the state memory
// ---------------------------------------------------------------------------

reg  signed [26:0] rom[512];
`ifdef VERILATOR
initial $readmemh("core/rtl/audiofx_coefs.hex", rom);
`else
initial $readmemh("rtl/audiofx_coefs.hex", rom);
`endif

reg  signed [26:0] stmem[136];          // per slot: x1 x2 y1 y2 and the rounding error left over
integer j;
initial for (j = 0; j < 136; j = j + 1) stmem[j] = 0;

reg  [8:0]  cadr;
reg  [7:0]  sadr;
reg         wen;
reg  [7:0]  waddr;
reg  signed [26:0] wdata, crom, rdq;
always @(posedge clk) begin
	crom <= rom[cadr];
	if (wen) stmem[waddr] <= wdata;
	rdq  <= stmem[sadr];
end

// ---------------------------------------------------------------------------
// the engine
// ---------------------------------------------------------------------------

localparam [2:0] IDLE = 0, NEXT = 1, TERM = 2, FIN = 3, ENV = 4;

reg  [2:0]  s;
reg  [4:0]  slot;
reg  [2:0]  term;
reg  [3:0]  ph;
reg  [1:0]  l10;
reg  signed [26:0] xv, ain;
reg  signed [27:0] xr;
wire signed [26:0] xin = (slot >= 5'd10) ? ain : xv;
wire signed [26:0] svw = (term == 0) ? xin : rdq;
reg  [6:0]  idx;
reg  signed [26:0] coef_r, sv_r, x1t, y1t, yv;
reg  signed [53:0] prod;
reg  signed [57:0] acc, accr, rsd;
reg         ysat;
reg  signed [15:0] fx_out;

wire [3:0] gsel = gip[4*(slot - 5'd3) +: 4];

// does this slot do anything
wire run = (slot == 5'd0) ? (eq_on && pg != 0) :
           (slot <= 5'd2) ? (model >= 3'd2) :
           (slot <= 5'd9) ? (eq_on && gsel != 4'd4) :
           (slot <= 5'd16) ? (spec != 0) : 1'b0;

// the coefficient set of this slot
function [6:0] eqbase(input [4:0] sl);
	case (sl)
		5'd3: eqbase = 7'd4;   5'd4: eqbase = 7'd13;  5'd5: eqbase = 7'd22;  5'd6: eqbase = 7'd31;
		5'd7: eqbase = 7'd40;  5'd8: eqbase = 7'd49;  default: eqbase = 7'd58;
	endcase
endfunction
function [6:0] mbase(input [2:0] m);
	case (m)
		3'd2: mbase = 7'd67; 3'd3: mbase = 7'd69; 3'd4: mbase = 7'd71;
		3'd5: mbase = 7'd73; 3'd6: mbase = 7'd75; default: mbase = 7'd77;
	endcase
endfunction

function signed [26:0] sat27(input signed [33:0] v);
	sat27 = (v > 34'sd67108863) ? 27'sd67108863 : (v < -34'sd67108864) ? 27'sh4000000 : v[26:0];
endfunction

function signed [15:0] sat16(input signed [27:0] t);     // t is already v/256 rounded
	sat16 = (t > 28'sd32767) ? 16'sd32767 : (t < -28'sd32768) ? 16'sh8000 : t[15:0];
endfunction

function [4:0] msb(input [26:0] v);
	integer i;
	begin
		msb = 0;
		for (i = 0; i < 27; i = i + 1) if (v[i]) msb = i[4:0];
	end
endfunction

// the analyser's release: slower for the low bands
function [3:0] rsh(input [2:0] b);
	case (b)
		3'd0: rsh = 4'd11; 3'd1: rsh = 4'd10; 3'd2: rsh = 4'd10; default: rsh = 4'd9;
	endcase
endfunction

reg  [26:0] e, a, df, stp, en;
reg         gt;
reg  [4:0]  m;
reg         nx;
reg  [3:0]  hh;
reg  [3:0]  pk[7];
reg  [26:0] envm[7];
integer     k2;
initial for (j = 0; j < 7; j = j + 1) begin envm[j] = 0; pk[j] = 0; end
reg  [10:0] pkc;
reg  [33:0] cfg_q;
reg         clr;
reg  [7:0]  clra;
reg         dec;
wire [2:0]  eb = 3'(slot - 5'd10);
wire [5:0]  lg = {m, 1'b0} + 6'(nx);        // 2 log2 e: 3 dB a step

always @(posedge clk) begin
	wen <= 0;
	xr  <= {xv[26], xv} + 28'sd128;
	out <= active ? fx_out : sample;

	if (spec == 0) begin bars <= 0; peaks <= 0; for (k2 = 0; k2 < 7; k2 = k2 + 1) pk[k2] <= 0; end

	if (reset) begin s <= IDLE; fx_out <= 0; pkc <= 0; dec <= 0; end
	else case (s)
		IDLE: if (strobe && !clr && (active || spec != 0)) begin
			xv <= {{3{sample[15]}}, sample, 8'd0};
			slot <= 0; l10 <= 0; s <= NEXT;
			pkc <= (pkc == 11'd1999) ? 11'd0 : pkc + 1'd1;
			if (pkc == 11'd1999) dec <= 1;
		end

		NEXT: begin
			if (slot == 5'd17) begin s <= IDLE; dec <= 0; end
			else if (slot == 5'd10 && l10 != 2'd2) begin
				// the result so far, once the last slot's value has settled
				l10 <= l10 + 1'd1;
				if (l10 == 2'd1) begin fx_out <= sat16(xr >>> 8); ain <= xv; end
			end
			else if (slot >= 5'd10 && spec == 0) s <= IDLE;
			else if (run) begin term <= 0; ph <= 4'd15; s <= TERM; end
			else slot <= slot + 1'd1;
		end

		TERM: begin
			ph <= ph + 1'd1;
			case (ph)
				4'd15: begin
					ph <= 0;
					idx <= (slot == 5'd0) ? 7'(pg - 1'd1) :
					       (slot <= 5'd2) ? mbase(model) + 7'(slot - 5'd1) :
					       (slot <= 5'd9) ? eqbase(slot) + 7'(gsel) :
					                        7'd79 + 7'(slot - 5'd10);
				end
				4'd0: begin
					cadr <= {idx, 2'd0} + 9'(idx) + 9'(term);
					sadr <= {slot, 3'(term - 1'd1)};
				end
				4'd2: begin
					// term 5 adds the rounding error the last sample left (times 1)
					coef_r <= (term == 3'd5) ? 27'sd1 : crom;
					sv_r   <= svw;
					if (term == 1) x1t <= svw;
					if (term == 3) y1t <= svw;
				end
				4'd3: prod <= coef_r * sv_r;
				4'd4: begin
					acc <= ((term == 0) ? 58'sd0 : acc) + 58'(prod);
					ph  <= 0;
					if (term == 3'd5) s <= FIN; else term <= term + 1'd1;
				end
				default: ;
			endcase
		end

		FIN: begin
			ph <= ph + 1'd1;
			case (ph)
				4'd0: accr <= acc + 58'sd8388608;
				4'd1: begin yv <= sat27(34'(accr >>> 24)); ysat <= (accr >>> 24) != 58'(sat27(34'(accr >>> 24))); end
				// what rounding to y left over, carried to the next sample (noise shaping: the
				// low bands' poles would otherwise amplify the rounding noise enormously)
				4'd2: rsd <= ysat ? 58'sd0 : acc - (58'(yv) <<< 24);
				4'd3: begin wen <= 1; waddr <= {slot, 3'd0}; wdata <= xin; end
				4'd4: begin wen <= 1; waddr <= {slot, 3'd1}; wdata <= x1t; end
				4'd5: begin wen <= 1; waddr <= {slot, 3'd2}; wdata <= yv; end
				4'd6: begin wen <= 1; waddr <= {slot, 3'd3}; wdata <= y1t; end
				4'd7: begin wen <= 1; waddr <= {slot, 3'd4}; wdata <= rsd[26:0]; end
				4'd8: begin
					ph <= 0;
					if (slot < 5'd10) begin xv <= yv; slot <= slot + 1'd1; s <= NEXT; end
					else s <= ENV;
				end
				default: ;
			endcase
		end

		ENV: begin
			ph <= ph + 1'd1;
			case (ph)
				4'd0: begin e <= envm[eb]; a <= yv[26] ? 27'(-yv) : 27'(yv); end
				4'd1: begin gt <= a > e; df <= (a > e) ? a - e : e - a; end
				4'd2: stp <= gt ? (df + 27'd3) >> 2 :
				             (((df >> rsh(eb)) == 0 && df != 0) ? 27'd1 : (df >> rsh(eb)));
				4'd3: en <= gt ? e + stp : e - stp;
				4'd4: begin envm[eb] <= en; m <= msb(en); end
				4'd5: nx <= (m != 0) ? en[m - 1'd1] : 1'b0;
				4'd6: hh <= (lg < 6'd30) ? 4'd0 : (lg > 6'd45) ? 4'd15 : 4'(lg - 6'd30);
				4'd7: begin
					bars[4*eb +: 4] <= hh;
					if (hh >= pk[eb]) pk[eb] <= hh;
					else if (dec && pk[eb] != 0) pk[eb] <= pk[eb] - 1'd1;
					ph <= 0;
					slot <= slot + 1'd1;
					s <= NEXT;
				end
				default: ;
			endcase
		end

		default: s <= IDLE;
	endcase

	if (spec != 0) for (k2 = 0; k2 < 7; k2 = k2 + 1) peaks[4*k2 +: 4] <= pk[k2];

	// a new model, preset or band: the filters start from silence (no ringing of the old
	// setting's states in the new one), cleared between two samples
	if (reset) begin clr <= 0; cfg_q <= {model, preset, bands}; end
	else if (clr) begin
		wen <= 1; waddr <= clra; wdata <= 27'sd0;
		clra <= clra + 1'd1;
		if (clra == 8'd135) clr <= 0;
	end
	else if ({model, preset, bands} != cfg_q && s == IDLE) begin clr <= 1; clra <= 0; cfg_q <= {model, preset, bands}; end
end

endmodule
