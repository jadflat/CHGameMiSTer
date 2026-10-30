// Picture options between chg_video and MiSTer's video path: an "LCD look"
// that imitates the handheld's ST7735 panel, and a dot for the board's LED.
// With every option off the picture is chg_video's exactly, delayed by two
// pixel periods (ce_pix steps) in all of r, g, b, hs, vs, hblank and vblank
// together, so their alignment is unchanged; ce_pix itself is not delayed.
//
// OSD options (CHGame.sv):
//   grid_str, grid_sty  the LCD grid: strength and style (2x only)
//   lcd_col  the panel's colours (an approximation, see below)
//   led_en   a dot for the LED in the picture's bottom left corner
//
// The fine modes (chg_video's 3 and 4: 1.75x and 240x240, a glass pixel 14 or 15
// ticks wide) are worked on once a glass pixel: ce_px, chg_video's strobe, is
// ce_pix in the other modes and the pixel's first tick in these, and the
// pixel's place comes from chg_video (p_gx and the others) instead of being
// counted from the blanking. The pipeline then runs on ce_px ("a pixel period"
// is a glass pixel); the timing (hs, vs, hblank, vblank), which has to keep its
// place to the tick (the H position), goes through a delay of the same 2
// pixel periods (2T ticks). No grid in these modes (a glass pixel is not a
// whole number of lines): colours and the LED dot are as in the others.
//
// Where in the picture a pixel is (its glass column gx, row gy and which of
// the S x S repeats it is) is counted here from hblank, vblank, vs and the
// mode, the way chg_video lays it out: 1x has the window on the picture (S =
// 1); 2x has the window starting at glass row 4 less the V position
// (voff_in, latched at vs as chg_video does); 3x starts on row 0. No grid in
// 1x, where a glass pixel is one output pixel: the grid needs S >= 2.
//
//  - Grid: the last column and/or line of each S x S block is dimmed to 90, 75,
//    50 or 0% (style lines: both, a corner twice, e.g. 56%; rows; columns; dots:
//    the corner pixel only, once).
//  - LCD colours: 75% saturation (toward the luma (r + 2g + b) / 4), 7/8
//    contrast with black lifted to 12 (a backlit panel's black), red x 15/16
//    and blue x 33/32 (cooler white: 255 gives 222, 236, 243). An approximation of
//    the ST7735 panel's look, applied inside the picture only, never to the
//    blanking or the black around a moved picture.
//  - LED dot: a black 5 x 5 glass pixel box at columns 1-5, rows 110-114,
//    with the dot 3 x 3 inside it: red (f02020) lit, dark grey (383838) off.
//
// Pipeline (all registers on clk, "e0" the clock edge that has ce_pix high;
// a pixel period is at least 5 clocks): e0 the pixel and its position are
// taken in; e1 luma; e2 saturation; e3
// contrast and tint; e4 pass on; e5 = the next e0: grid and LED
// into the output registers.

module chg_postfx
(
	input             clk,          // 96 MHz
	input             ce_pix,
	input             ce_px,        // the pixel strobe: ce_pix, or in the fine modes once a glass pixel
	input       [2:0] mode,         // chg_video's mode being shown
	input       [4:0] voff_in,      // the OSD's V position, as chg_video's
	input       [2:0] grid_str,     // LCD grid: 0 off, 1 light, 2 medium, 3 dark, 4 black
	input       [1:0] grid_sty,     // 0 lines, 1 rows, 2 columns, 3 dots
	input             lcd_col,
	input             led_en,
	input             led,

	input       [7:0] r_in, g_in, b_in,
	input             hs_in, vs_in, hblank_in, vblank_in,
	input       [6:0] p_gx, p_gy,   // the fine modes: the pixel's place (chg_video's)
	input             p_act, p_yl, p_wl,

	output reg  [7:0] r, g, b,
	output            hs, vs, hblank, vblank,
	output reg        px_act        // with r, g, b: the pixel is in the picture's columns (chg_hfine)
);

wire       fine = mode >= 3'd3;
wire [1:0] s1 = mode == 3'd2 ? 2'd2 : mode == 3'd1 ? 2'd1 : 2'd0;     // S - 1

// ---- e0: the pixel and where it is ----
reg  [7:0] a_r, a_g, a_b;
reg        a_hs, a_vs, a_hb, a_vb, a_led, a_pact, a_pyl, a_pwl;
reg        hb_p, vs_p;
reg signed [5:0] voff_l;
reg  [6:0] gx;
reg  [1:0] xs;
reg signed [8:0] gy;
reg  [1:0] ys;
reg  [8:0] lc;                  // the line in the window, 0 on its first

wire signed [6:0] rp0 = 7'sd8 - 7'(voff_l);             // 2x: the glass line (x2) on the window's first
wire signed [8:0] gy0 = mode == 3'd1 ? 9'(rp0 >>> 1) : 9'sd0;
wire        [1:0] ys0 = mode == 3'd1 ? {1'b0, rp0[0]} : 2'd0;

always @(posedge clk) if (ce_px) begin
	a_r <= r_in; a_g <= g_in; a_b <= b_in;
	a_hs <= hs_in; a_vs <= vs_in; a_hb <= hblank_in; a_vb <= vblank_in;
	a_led <= led;
	a_pact <= p_act; a_pyl <= p_yl; a_pwl <= p_wl;
	hb_p <= hblank_in;
	vs_p <= vs_in;
	if (vs_in && !vs_p)
		voff_l <= voff_in[4] ? 6'($signed({1'b0, voff_in}) - 7'sd31) : $signed({1'b0, voff_in});
	if (fine) begin
		// the fine modes: the place is given
		gx <= p_gx; xs <= 2'd0; gy <= {2'b00, p_gy};
	end
	else begin
		// across: gx counts glass columns, xs the repeat; wraps to 0 on the line's first picture pixel
		if (hblank_in) begin gx <= 7'h7f; xs <= s1; end
		else if (xs >= s1) begin xs <= 2'd0; gx <= gx + 1'd1; end
		else xs <= xs + 1'd1;
		// down: on the end of a line the next line's row and repeat (the first picture line during vblank)
		if (hblank_in && !hb_p) begin
			if (vblank_in) begin gy <= gy0; ys <= ys0; lc <= 9'd0; end
			else if (ys >= s1) begin ys <= 2'd0; gy <= gy + 1'd1; end
			else ys <= ys + 1'd1;
			if (!vblank_in) lc <= lc + 1'd1;
		end
	end
end

wire a_blank = a_hb | a_vb;
wire a_act   = fine ? a_pact : !a_blank && !gy[8] && !gy[7];   // in the glass picture (0-127)
wire a_xl    = xs == s1;                                // last repeat across / down
wire a_yl    = fine ? a_pyl : ys == s1;

// ---- e1: luma ----
reg  [7:0] y1;
always @(posedge clk) y1 <= 8'((10'(a_r) + {1'b0, a_g, 1'b0} + 10'(a_b)) >> 2);

// ---- e2: 75% saturation ----
reg  [7:0] c1r, c1g, c1b;
always @(posedge clk) begin
	c1r <= 8'((10'(a_r) * 3 + 10'(y1)) >> 2);
	c1g <= 8'((10'(a_g) * 3 + 10'(y1)) >> 2);
	c1b <= 8'((10'(a_b) * 3 + 10'(y1)) >> 2);
end

// ---- e3: contrast and tint (or the pixel as it is) ----
wire [7:0] c2r = 8'd12 + c1r - (c1r >> 3);
wire [7:0] c2g = 8'd12 + c1g - (c1g >> 3);
wire [7:0] c2b = 8'd12 + c1b - (c1b >> 3);
wire [8:0] c3b = 9'(c2b) + 9'(c2b >> 5);
reg  [7:0] t_r, t_g, t_b;
always @(posedge clk) begin
	if (lcd_col && a_act) begin
		t_r <= c2r - (c2r >> 4);
		t_g <= c2g;
		t_b <= c3b[8] ? 8'd255 : c3b[7:0];
	end
	else begin t_r <= a_r; t_g <= a_g; t_b <= a_b; end
end

// the grid: the column dim is done as the pixel enters f (e4), the row dim in e5,
// so each clock has one dm
// strength 1 light 90% (x * 29 / 32), 2 medium 75% (x - x/4), 3 dark 50%, 4 black
function [7:0] dm(input [7:0] x, input [2:0] st);
	case (st)
		3'd1:    dm = 8'((13'(x) * 13'd29) >> 5);
		3'd2:    dm = x - (x >> 2);
		3'd3:    dm = x >> 1;
		default: dm = 8'd0;
	endcase
endfunction
// style 0 lines (a corner dims twice), 1 rows only, 2 columns only, 3 dots (the corner pixel only)
wire grid = grid_str != 3'd0 && s1 != 2'd0 && a_act;
wire d1   = grid_sty == 2'd3 ? a_xl && a_yl : grid_sty == 2'd1 ? 1'b0 : a_xl;
wire d2   = (grid_sty == 2'd0 || grid_sty == 2'd1) && a_yl;
wire [7:0] gr1 = grid && d1 ? dm(t_r, grid_str) : t_r;
wire [7:0] gg1 = grid && d1 ? dm(t_g, grid_str) : t_g;
wire [7:0] gb1 = grid && d1 ? dm(t_b, grid_str) : t_b;
// ---- e4: the pixel passes on (st1-st4 keep the 5 clock stage timing) ----
reg         st1, st2, st3, st4;
reg   [7:0] f_r, f_g, f_b;
always @(posedge clk) begin
	st1 <= ce_px; st2 <= st1; st3 <= st2; st4 <= st3;
	if (st4) begin f_r <= gr1; f_g <= gg1; f_b <= gb1; end
end

// ---- e5: the row dim and the LED dot, out with the timing ----
wire [7:0] gr2 = grid && d2 ? dm(f_r, grid_str) : f_r;
wire [7:0] gg2 = grid && d2 ? dm(f_g, grid_str) : f_g;
wire [7:0] gb2 = grid && d2 ? dm(f_b, grid_str) : f_b;

wire in_box = led_en && a_act && gx >= 7'd1 && gx <= 7'd5 && gy >= 9'sd110 && gy <= 9'sd114;
wire in_dot = in_box && gx >= 7'd2 && gx <= 7'd4 && gy >= 9'sd111 && gy <= 9'sd113;
wire [23:0] dot = a_led ? 24'hf02020 : 24'h383838;

reg o_hs, o_vs, o_hb, o_vb;
always @(posedge clk) if (ce_px) begin
	{r, g, b} <= in_dot ? dot : in_box ? 24'd0 : {gr2, gg2, gb2};
	o_hs <= a_hs; o_vs <= a_vs; o_hb <= a_hb; o_vb <= a_vb;
	px_act <= !a_hb;
end

// the fine modes' timing: ce_pix ticks, delayed by what the colour takes, 2 pixel periods = 2T ticks
wire [3:0] td_q;
chg_tdelay #(.W(4)) td (.clk(clk), .ce(ce_pix), .d(mode == 3'd4 ? 6'd30 : 6'd28), .din({hs_in, vs_in, hblank_in, vblank_in}), .q(td_q));
assign hs     = fine ? td_q[3] : o_hs;
assign vs     = fine ? td_q[2] : o_vs;
assign hblank = fine ? td_q[1] : o_hb;
assign vblank = fine ? td_q[0] : o_vb;

endmodule
