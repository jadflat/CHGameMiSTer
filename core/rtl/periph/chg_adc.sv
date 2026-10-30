// The ADC at 0x4001_2400. Nothing is wired to it on the CHGame; a
// conversion reads noise.
//
// Translates adc_read and adc_write in emu/src/bus.c: 24 plain 32-bit
// registers, with
//   STATR (0)     write: bits cleared where the value has 0 (adc[0] &= v)
//   CTLR2 (2)     bits 3:2 (calibration) clear at once; SWSTART (bit 22)
//                 clears, and does a conversion: the noise generator steps,
//                 RDATAR gets 1800 + (noise >> 22) % 400, STATR gets EOC
//                 and STRT, and the access takes 50 board cycles more
//   RDATAR (19)   a read clears EOC (any size, any byte of it)
// The noise generator (m->adc_noise) is
//   noise = (noise * 1664525 + 1013904223) ^ (uint32_t)m->cycles
// with m->cycles the board time of the access. It is zeroed by chg_init
// only (not by a reset), so it and the board-time count here are cleared by
// 'init' alone: chg_soc pulses that when board time starts again from 0.
// The 50 cycles come out as 'stall' with the ack, as chg_flashctl's do.

module chg_adc
(
	input             clk,
	input             init,         // chg_init: board time 0, the noise generator 0
	input             reset,        // chg_reset: bus_reset
	input             ce,

	input             req,
	input             we,
	input      [31:0] addr,
	input       [2:0] size,
	input      [31:0] wdata,
	output reg [31:0] rdata,
	output reg        ack,
	output reg [16:0] stall
);

reg [31:0] adc[0:23];
reg [31:0] noise;
reg [31:0] t;                   // board time, low 32 bits (m->cycles)

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

// the noise step, over two clocks (the multiply has a clock of its own)
reg        conv;
reg [31:0] prod;
wire [31:0] n_noise = (prod + 32'd1013904223) ^ t;
wire  [9:0] nb = n_noise[31:22];
wire  [9:0] nm = (nb >= 10'd800) ? nb - 10'd800 : (nb >= 10'd400) ? nb - 10'd400 : nb;

integer k;
always @(posedge clk) begin
	ack <= 0;
	if (init) begin
		noise <= 0;
		t <= 0;
	end else if (ce)
		t <= t + 32'd1;
	if (reset) begin
		for (k = 0; k < 24; k = k + 1) adc[k] <= 0;
		conv <= 0;
	end else if (conv) begin
		conv <= 0;
		noise <= n_noise;
		adc[19] <= 32'd1800 + {22'd0, nm};
		adc[0] <= adc[0] | 32'h12;
		stall <= 17'd50;
		ack <= 1;
	end else if (req) begin
		stall <= 0;
		if (!we) begin
			rdata <= lane_r((r < 8'd24) ? adc[r[4:0]] : 32'd0, sh, size);
			if (r == 8'd19) adc[0] <= adc[0] & ~32'h2;
			ack <= 1;
		end else if (r == 8'd0) begin
			adc[0] <= adc[0] & wv;
			ack <= 1;
		end else if (r < 8'd24) begin
			if (r == 8'd2) begin
				adc[2] <= wv & ~32'h0040000c;
				if (wv[22]) begin
					prod <= noise * 32'd1664525;
					conv <= 1;
				end else
					ack <= 1;
			end else begin
				adc[r[4:0]] <= wv;
				ack <= 1;
			end
		end else
			ack <= 1;
	end
end

endmodule
