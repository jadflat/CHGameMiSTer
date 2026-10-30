// chg_audio and chg_audio_fx joined as in CHGame.sv, for sim/tb_audiofx.cpp;
// inj_en feeds the fx with samples from the testbench instead of chg_audio's
module audiofx_top
(
	input         clk,
	input         reset,
	input         ce,
	input         pin,
	input  [2:0]  model,
	input  [2:0]  preset,
	input  [27:0] bands,
	input  [1:0]  spec,
	input         inj_en,
	input  signed [15:0] inj_sample,
	input         inj_strobe,
	output signed [15:0] raw,          // chg_audio's sample
	output signed [15:0] audio,        // AUDIO_L
	output [27:0] bars,
	output [27:0] peaks,
	output        raw_strobe
);
wire signed [15:0] sample;
wire strobe;
chg_audio audio_i(.clk(clk), .reset(reset), .ce(ce), .pin(pin), .piezo(model == 3'd1), .vol(12'd1024), .sample(sample), .strobe(strobe));
assign raw = sample;
assign raw_strobe = strobe;
chg_audio_fx fx(.clk(clk), .reset(reset), .sample(inj_en ? inj_sample : sample), .strobe(inj_en ? inj_strobe : strobe),
                .model(model), .preset(preset), .bands(bands), .spec(spec != 0), .out(audio), .bars(bars), .peaks(peaks));
endmodule
