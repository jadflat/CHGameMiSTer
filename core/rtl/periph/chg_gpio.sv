// GPIO ports A, B, C (0x4001_0800, 0x4001_0C00, 0x4001_1000), AFIO
// (0x4001_0000), the buttons, and the pin levels the rest of the board sees.
//
// Translates gpio_read, gpio_write, gpio_pin, af_level, chg_set_buttons,
// the AFIO cases of bus_read_slow / bus_write_slow, and the pin side of
// buzzer_update in emu/src/bus.c.
//
// A pin (gpio_pin): an output (MODE bits != 0) gives OUTDR, or its alternate
// function when CNF bit 1 (cfg bit 3) is set, and a button holding it low
// wins either way. An input reads 0 when a button holds it low, OUTDR's bit
// with pull up/down (CNF 2), else 0 (floating or analog). The alternate
// functions (af_level): PA4 is SPI1's NSS (the SPI side gives the level),
// PB10 is TIM1 CH2, anything else 0.
//
// PB10 is the buzzer. The C keeps what drives it in m->buzzer.cur, worked
// out by buzzer_update at every GPIO, AFIO, SPI or TIM1 write, and the pin
// (and the audio) follows that:
// - alternate-function output with the TIM1 partial remap (PCFR1 bits
//   17:15 = 1): TIM1 CH2 (chg_tim's oc2, the same at every cycle as the C's
//   steady level or PWM wave)
// - alternate-function output without the remap: buzzer_update takes
//   gpio_pin(PB10), which is af_level, which is what buzzer.cur was doing
//   at that moment. So the pin freezes at the level it had just before the
//   write that took the remap away (or made it AF): bz_hold
// - otherwise the pin as gpio_pin gives it
// Buttons (CHG_BTN_* order, 1 = held) pull their pins low: UP PB4, DOWN
// PC14, LEFT PB3, RIGHT PC15, A PB1, B PB6, SELECT PB7, START PB8.
//
// gpio_changed's other effects (the LCD reset on PB12, the LED on PB9) are
// the pin outputs here; whoever watches a pin watches it every clock.

module chg_gpio
(
	input             clk,
	input             reset,        // chg_reset: bus_reset (every pin a floating input)
	input             ce,           // not used: nothing here counts board time

	input             req,
	input             we,
	input      [31:0] addr,
	input       [2:0] size,
	input      [31:0] wdata,
	output reg [31:0] rdata,
	output reg        ack,

	input       [7:0] buttons,
	input             spi_nss,      // PA4's alternate function: !(SSOE && SPE)
	input             tim1_oc2,     // TIM1 CH2 output (chg_tim oc2)

	output            pa4,          // LCD CS
	output            pb0,          // LCD D/C
	output            pb9,          // LED
	output            pb10,         // buzzer
	output            pb11,         // SD CS
	output            pb12,         // LCD RESET
	output     [31:0] afio_pcfr1
);

reg [31:0] cfg0[0:2], cfg1[0:2], cfg2[0:2], lckr[0:2];
reg [23:0] outdr[0:2];
reg [31:0] pcfr1, exticr0, exticr1, actlr;
reg        bz_hold;

assign afio_pcfr1 = pcfr1;

// chg_set_buttons
wire [23:0] ext_low[0:2];
assign ext_low[0] = 24'd0;
assign ext_low[1] = {15'd0, buttons[7], buttons[6], buttons[5], 1'b0, buttons[0], buttons[2], 1'b0, buttons[4], 1'b0};
assign ext_low[2] = {8'd0, buttons[3], buttons[1], 14'd0};

function [3:0] pin_cfg(input [31:0] c0, input [31:0] c1, input [31:0] c2, input [4:0] pin);
	reg [31:0] c;
	begin
		c = (pin < 5'd8) ? c0 : (pin < 5'd16) ? c1 : c2;
		pin_cfg = c[{pin[2:0], 2'b00} +: 4];
	end
endfunction

// gpio_pin with the alternate-function level given
function pin_level(input [3:0] cfg, input od, input el, input af);
	begin
		if (cfg[1:0] != 2'd0)
			pin_level = (cfg[3] ? af : od) & !el;
		else if (el)
			pin_level = 1'b0;
		else
			pin_level = (cfg[3:2] == 2'd2) ? od : 1'b0;
	end
endfunction

// PB10 (buzzer_update)
wire [3:0] b10cfg  = pin_cfg(cfg0[1], cfg1[1], cfg2[1], 5'd10);
wire       b10_af  = (b10cfg[1:0] != 2'd0) && b10cfg[3];
wire       remap   = pcfr1[17:15] == 3'd1;
wire       b10_frozen = b10_af && !remap;
wire       bz_cur  = b10_af ? (remap ? tim1_oc2 : bz_hold)
                            : pin_level(b10cfg, outdr[1][10], ext_low[1][10], 1'b0);

// INDR of each port: every pin's level
reg [23:0] indr[0:2];
integer p, i;
always @* begin
	for (p = 0; p < 3; p = p + 1)
		for (i = 0; i < 24; i = i + 1)
			indr[p][i] = pin_level(pin_cfg(cfg0[p], cfg1[p], cfg2[p], i[4:0]), outdr[p][i], ext_low[p][i],
			                       (p == 0 && i == 4) ? spi_nss : (p == 1 && i == 10) ? bz_cur : 1'b0);
end

assign pa4  = indr[0][4];
assign pb0  = indr[1][0];
assign pb9  = indr[1][9];
assign pb10 = bz_cur;
assign pb11 = indr[1][11];
assign pb12 = indr[1][12];

// ---------------------------------------------------------------------------
// the bus
// ---------------------------------------------------------------------------

wire [4:0]  sh  = {addr[1:0], 3'b000};
wire [31:0] wv  = wdata << sh;
wire [9:0]  off = {addr[9:2], 2'b00};
wire [6:0]  win = addr[16:10];          // (addr - 0x4000_0000) >> 10, low bits
wire        is_afio = win == 7'h40;
wire [1:0]  port = (win == 7'h42) ? 2'd0 : (win == 7'h43) ? 2'd1 : 2'd2;

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
	rv = 32'd0;
	if (is_afio)
		case (off)
		10'h04: rv = pcfr1;
		10'h08: rv = exticr0;
		10'h0c: rv = exticr1;
		10'h18: rv = actlr;
		default: rv = 32'd0;
		endcase
	else
		case (off)
		10'h00: rv = cfg0[port];
		10'h04: rv = cfg1[port];
		10'h08: rv = {8'd0, indr[port]};
		10'h0c: rv = {8'd0, outdr[port]};
		10'h18: rv = lckr[port];
		10'h1c: rv = cfg2[port];
		default: rv = 32'd0;
		endcase
end

integer k;
always @(posedge clk) begin
	ack <= 0;
	if (!b10_frozen) bz_hold <= bz_cur;
	if (reset) begin
		for (k = 0; k < 3; k = k + 1) begin
			cfg0[k] <= 32'h44444444; cfg1[k] <= 32'h44444444; cfg2[k] <= 32'h44444444;
			outdr[k] <= 0; lckr[k] <= 0;
		end
		pcfr1 <= 0; exticr0 <= 0; exticr1 <= 0; actlr <= 0;
		bz_hold <= 0;
	end else if (req) begin
		ack <= 1;
		if (!we)
			rdata <= lane_r(rv, sh, size);
		else if (is_afio)
			case (off)
			10'h04: pcfr1 <= wv;
			10'h08: exticr0 <= wv;
			10'h0c: exticr1 <= wv;
			10'h18: actlr <= wv;
			default: ;
			endcase
		else
			case (off)
			10'h00: cfg0[port] <= wv;
			10'h04: cfg1[port] <= wv;
			10'h0c: outdr[port] <= wv[23:0];
			10'h10: outdr[port] <= (outdr[port] | {8'd0, wv[15:0]}) & ~{8'd0, wv[31:16]};
			10'h14: outdr[port] <= outdr[port] & ~wv[23:0];
			10'h18: lckr[port] <= wv;
			10'h1c: cfg2[port] <= wv;
			10'h20: outdr[port] <= (outdr[port] | {wv[7:0], 16'd0}) & ~{wv[23:16], 16'd0};
			default: ;
			endcase
	end
end

endmodule
