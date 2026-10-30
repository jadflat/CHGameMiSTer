// Video timing for the 128x128 glass, in five modes (the OSD's "Analogue
// mode": 1x, 1.75x, 240x240, 2x; the 31 kHz one is not offered), all 60 Hz. The picture made here goes to the analogue output as it
// is and to HDMI through MiSTer's scaler, which takes only the area marked
// visible (DE). Each mode is a whole number of 96 MHz clocks a pixel, and the
// picture is each glass pixel repeated S times across and S lines down.
//
//  mode  name            pixel    S  line (pixels)                  frame (lines)
//   (modes 3 and 4 are below the table: "fine" modes)
//   0    15 kHz 1x       6 MHz    1  384: 128 picture, 94 border,   262: 3 sync, 72 to the picture,
//        (the default,                   9 fp, 28 sync, 34 bp,          128 picture, 59 below.
//        original mode)                    91 border.                     15.625 kHz, 59.64 Hz
//   1    15 kHz 2x       6 MHz    2  384: 256 picture, 29 border,   262: 3 sync, 16 bp, 240 picture
//                                        9 fp, 28 sync, 34 bp,          (glass rows 4-123), 3 fp.
//                                        28 border.                     15.625 kHz, 59.64 Hz
//   2    31 kHz 3x       19.2 MHz 3  610: 384 picture, 52 border,   525: 2 sync, 33 bp, 48 border,
//                                        12 fp, 73 sync (3.8 us),       384 picture, 48 border, 10 fp.
//                                        37 bp, 52 border.              31.475 kHz, 59.95 Hz
//
// At 6 MHz a pixel is as wide as a line is tall on a 4:3 tube, and the
// 31 kHz mode keeps VGA's proportions, so the picture is square. Sync is
// active high, as before.
//
// The picture can be moved up to 15 lines up or down (voff_in), in the 15 kHz
// modes. In 1x the visible window moves with it (72 lines above it and 59
// below leave room); in 2x the window and the sync stay where they are and
// the picture moves inside it, cropped at the window's edge with black lines
// coming in at the other side. A new mode or offset starts at the end of a
// frame.
//
// The picture can also be moved up to 20 pixels left or right (hoff_in), in
// the 15 kHz modes, by moving the horizontal sync the other way within the
// line's blanking; the window and the line's length stay the same.
//
// FINE MODES (3: 15 kHz 1.75x, 4: 15 kHz 240x240; the OSD's other two choices).
// A glass pixel is not a whole number of 6 MHz pixels there, so ce_pix runs at
// 48 MHz (every 2 clocks, a "tick": 1/8 of a 6 MHz pixel) in them, 3072 ticks a
// line, and a glass pixel is T ticks (1.75x: 14 = 28 clocks, 240x240: 15 = 30
// clocks). The line is still 6144 clocks (15.625 kHz), the frame 262 lines.
//
//  mode  name       T   picture (ticks)  hsync (ticks, hoff 0)   window (lines)     glass rows : lines
//   3    1.75x     14   128 x 14 = 1792   2224-2448 (4.67 us)     27-250 (224)       4 : 7 (2,2,2,1)
//   4    240x240   15   128 x 15 = 1920   2288-2512 (4.67 us)     19-258 (240)       8 : 15 (2 x7, 1)
//
// hsync ends 78 (70) 6 MHz pixels before the picture starts, which centres it as the
// 1x and 2x pictures are (the centre is 190 pixels after hsync's end in all). The
// H position moves hsync by 8 ticks (16 clocks, a 6 MHz pixel) a step, as in 1x/2x.
// The V position moves the picture inside the fixed window, as in 2x. ce_pix
// is the clock enable of everything that follows (the mixer, HDMI: 1792 or 1920
// samples a line); only the colour changes on every T-th tick (ce_px, the
// glass pixel's strobe, which chg_postfx works on), the sync and blanking on
// any tick. The line buffer: the Scale filter ("filt") makes some lines a
// mix of two glass rows, the weights w/8 (see ftype below), the row before
// kept in a 128 x 24 bit memory (lbuf) while the next is read from the st7735.
// The Soft filter's horizontal blend is chg_hfine's (after chg_postfx).
//
// The row to show is worked out ahead (vline, at the end of the sync of the
// line before), and a pixel's colour comes back from the st7735 three clocks
// after vx, inside the shortest pixel (5 clocks), so it goes out on the next
// pixel together with that pixel's timing. In the fine modes the column
// runs 4 ticks ahead (see sx), and the colour is ready 8 clocks after.
//
// The panel on the board refreshes on its own clock and the game is not
// synchronised to it; here the picture is read from the frame memory as it
// is scanned out, so a game that draws during the scan tears as on the board
// (where exactly it tears differs).

module chg_video
(
	input             clk,          // 96 MHz
	input       [2:0] mode_in,      // 0-4 as above (more is mode 0)
	input       [1:0] filt_in,      // Scale filter (fine modes): 0 off, 1 even rows, 2 sharp bilinear, 3 soft
	input       [4:0] voff_in,      // the picture 0-15 lines down (0-15) or 15-1 up (16-30)
	input       [5:0] hoff_in,      // the picture 0-20 pixels right (0-20) or 20-1 left (21-40)
	input             ovl_en,       // speed display on
	input      [11:0] ovl_bcd,      // the percent, 3 BCD digits
	input       [1:0] spec_opt,     // spectrum analyser: 0 off, 1 while the OSD is open, 2 always
	input             spec_pk,      // with peak marks
	input             osd_open,     // the OSD menu is open (clk synchronous)
	input      [27:0] spec_bars,    // seven 4 bit heights (chg_audio_fx)
	input      [27:0] spec_peaks,   // and their peaks
	output reg  [2:0] mode,         // the mode being shown
	output reg  [1:0] filt,         // the filter being shown
	output reg        ce_pix,
	output reg        ce_px,        // the glass pixel's strobe: ce_pix, or in fine modes every T-th ce_pix
	// for chg_postfx in the fine modes, together with r, g, b (valid at ce_px): the glass pixel
	output reg  [6:0] p_gx, p_gy,
	output reg        p_act,        // in the glass picture
	output reg        p_yl,         // the last of the row's lines
	output reg        p_wl,         // the window's last line

	// to the st7735: which pixel, and the colour it gives back
	output      [6:0] vx,
	output      [6:0] vy,
	output reg        vline,
	input      [23:0] vrgb,

	output reg  [7:0] r, g, b,
	output reg        hs, vs, hblank, vblank
);

// the mode's numbers, registered (the mode and the offset change only between
// frames). Lines wstart to wend are the visible window (blanking outside it);
// glass row 0 starts on line pst, and the picture is ppl lines tall; the
// window shows what of the picture falls inside it, black elsewhere.
reg signed [5:0] voff;          // the picture's vertical offset, lines down
reg signed [5:0] hoff;          // the picture's horizontal offset, pixels right
reg        fine, m240;          // a fine mode (3, 4); mode 4
reg [3:0] pm1;                  // clocks a tick, less 1
reg [11:0] htot1, hact, hs0, hs1;
reg [9:0] vtot1, vs1, wstart, wend, ppl;
reg signed [10:0] pst;
reg [1:0] s1;                   // S - 1
reg [3:0] xw;                   // ticks a glass pixel, less 1 (S - 1 in modes 0-2)
reg [2:0] gm;                   // fine: glass rows a group, less 1 (3 or 7); the group's last row has one line
wire signed [11:0] hoff8 = 12'(hoff) <<< 3;       // a 6 MHz pixel is 8 ticks in the fine modes
always @(posedge clk) begin
	fine <= mode >= 3'd3;
	m240 <= mode == 3'd4;
	case (mode)
		3'd1:    {pm1, htot1, hact, hs0, hs1, vtot1, vs1, s1, wstart, wend, pst, ppl} <=
		         {4'd15, 12'd383, 12'd256, 12'(12'sd294 - 12'(hoff)), 12'(12'sd322 - 12'(hoff)), 10'd261, 10'd3, 2'd1,
		          10'd19, 10'd259, 11'sd11 + 11'(voff), 10'd256};
		3'd2:    {pm1, htot1, hact, hs0, hs1, vtot1, vs1, s1, wstart, wend, pst, ppl} <=
		         {4'd4,  12'd609, 12'd384, 12'd448, 12'd521, 10'd524, 10'd2, 2'd2,
		          10'd83, 10'd467, 11'sd83, 10'd384};
		3'd3:    {pm1, htot1, hact, hs0, hs1, vtot1, vs1, s1, wstart, wend, pst, ppl} <=
		         {4'd1,  12'd3071, 12'd1792, 12'(13'sd2224 - 13'(hoff8)), 12'(13'sd2448 - 13'(hoff8)), 10'd261, 10'd3, 2'd0,
		          10'd27, 10'd251, 11'sd27 + 11'(voff), 10'd224};
		3'd4:    {pm1, htot1, hact, hs0, hs1, vtot1, vs1, s1, wstart, wend, pst, ppl} <=
		         {4'd1,  12'd3071, 12'd1920, 12'(13'sd2288 - 13'(hoff8)), 12'(13'sd2512 - 13'(hoff8)), 10'd261, 10'd3, 2'd0,
		          10'd19, 10'd259, 11'sd19 + 11'(voff), 10'd240};
		default: {pm1, htot1, hact, hs0, hs1, vtot1, vs1, s1, wstart, wend, pst, ppl} <=
		         {4'd15, 12'd383, 12'd128, 12'(12'sd231 - 12'(hoff)), 12'(12'sd259 - 12'(hoff)), 10'd261, 10'd3, 2'd0,
		          10'(10'sd75 + 10'(voff)), 10'(10'sd203 + 10'(voff)), 11'sd75 + 11'(voff), 10'd128};
	endcase
	xw <= mode == 3'd3 ? 4'd13 : mode == 3'd4 ? 4'd14 : {2'b00, mode == 3'd1 ? 2'd1 : mode == 3'd2 ? 2'd2 : 2'd0};
	gm <= mode == 3'd4 ? 3'd7 : 3'd3;
end

// from those: the lines with picture, l0 to pend, and the glass row and
// repeat on line l0 (the picture may start above the window)
reg  [9:0] l0, pend;
reg  [6:0] syi;
reg  [1:0] yri;
wire signed [10:0] rel = $signed({1'b0, wstart}) - pst;
wire signed [10:0] pe = pst + $signed({1'b0, ppl});
// fine modes: the picture may start above the window; then line l0 is the line
// before the window (it is not shown), the glass row of the line "rx" of the
// pattern (2,2,2,1 or 2,2,2,2,2,2,2,1) is the first, so that a line of the
// window that mixes in the row before (the line before it) has that row in lbuf
reg  signed [10:0] rel_r;       // the steps to the first line's place in the pattern are registered: they change only between frames
reg         [9:0] rx;
reg         [1:0] fq;
reg         [3:0] frm;
always @(posedge clk) begin
	rel_r <= rel;
	rx    <= rel_r > 0 ? 10'(rel_r - 11'sd1) : 10'd0;
	fq    <= m240 ? {1'b0, rx >= 10'd15} : (rx >= 10'd14 ? 2'd2 : rx >= 10'd7 ? 2'd1 : 2'd0);
	frm   <= 4'(m240 ? rx - (rx >= 10'd15 ? 10'd15 : 10'd0) : rx - (rx >= 10'd14 ? 10'd14 : rx >= 10'd7 ? 10'd7 : 10'd0));
end
wire        flast = m240 ? frm == 4'd14 : frm == 4'd6;
wire  [2:0] frow  = flast ? (m240 ? 3'd7 : 3'd3) : frm[3:1];
wire  [6:0] fsyi  = (m240 ? {3'b000, fq[0], 3'b000} : {3'b000, fq, 2'b00}) + {4'b0000, frow};
always @(posedge clk) begin
	l0   <= fine ? (rel > 0 ? wstart - 10'd1 : rel == 0 ? wstart : 10'(pst)) : (rel >= 0 ? wstart : 10'(pst));
	pend <= pe < $signed({1'b0, wend}) ? 10'(pe) : wend;
	if (fine) {syi, yri} <= {fsyi, 1'b0, flast ? 1'b0 : frm[0]};
	else if (rel < 0) {syi, yri} <= 0;
	else if (s1 == 2'd1) {syi, yri} <= {rel[7:1], 1'b0, rel[0]};
	else {syi, yri} <= {rel[6:0], 2'd0};        // 1x; 3x has no offset (rel 0)
end

reg [3:0] div;
reg [11:0] h;
reg [9:0] v;
reg [6:0] sx, sy, nq;           // glass column and row of this pixel (in the fine modes the column being fetched, 4 ticks ahead); the row to read for the next line
reg [3:0] xr;                   // ticks done of sx
reg [1:0] yr;                   // lines done of sy
reg       nact;                 // the next line has picture

wire       hend = h >= htot1;
wire       vlast = v >= vtot1;
wire [9:0] vn = vlast ? 10'd0 : v + 1'd1;
wire       hact_w = h < hact;
wire       vact_w = v >= wstart && v < wend;
wire       pact_w = v >= l0 && v < pend;

assign vx = sx;
assign vy = nq;

// the lines of the glass row: S in modes 0-2, in the fine modes 2, or 1 for the
// group's last row. The line after this one: its glass row and which line of it,
// and what the filter does with it (ftype: 0 the row as it is, 1 mix of the row
// before (lbuf) and the row after, weight w/8 for the row after: the second line
// of a row, w = (the row's place in its group + 1) x 8 / the group's rows, the
// exact share of the line's area; 2 Even rows' mix of the row before and this one
// on the group's last row's line, 4/8 each)
wire [1:0] yrw    = fine ? (((sy[2:0] & gm) == gm) ? 2'd0 : 2'd1) : s1;
wire       n_new  = vn == l0;
wire       n_adv  = yr == yrw;
wire [6:0] n_row  = n_new ? syi : n_adv ? sy + 1'd1 : sy;
wire [1:0] n_rep  = n_new ? yri : n_adv ? 2'd0 : yr + 1'd1;
wire [2:0] n_k    = n_row[2:0] & gm;
wire       n_isN  = fine && filt[1] && n_rep == 2'd1;
wire       n_isP  = fine && filt == 2'd1 && n_k == gm;
wire [2:0] n_wN   = gm == 3'd3 ? {n_k[1:0] + 2'd1, 1'b0} : n_k + 3'd1;
reg  [1:0] ntype, ftype;
reg  [2:0] nw, fw;
reg        fst;                 // the line being fetched is read from the st7735 (and so written to lbuf)

// The speed display: white 3x5 digits and a % on a black box, glass columns
// 111-127 and rows 13-19 (inside the 2x mode's window too), so it is scaled
// with the picture. Worked out from sx and sy in two registered steps, which
// settle inside a pixel (5 clocks at least) like the colour from the st7735.
function [2:0] glyph(input [3:0] c, input [2:0] row);
	case ({c, row})
		{4'd0, 3'd0}: glyph = 3'b111; {4'd0, 3'd1}: glyph = 3'b101; {4'd0, 3'd2}: glyph = 3'b101; {4'd0, 3'd3}: glyph = 3'b101; {4'd0, 3'd4}: glyph = 3'b111;
		{4'd1, 3'd0}: glyph = 3'b010; {4'd1, 3'd1}: glyph = 3'b110; {4'd1, 3'd2}: glyph = 3'b010; {4'd1, 3'd3}: glyph = 3'b010; {4'd1, 3'd4}: glyph = 3'b111;
		{4'd2, 3'd0}: glyph = 3'b111; {4'd2, 3'd1}: glyph = 3'b001; {4'd2, 3'd2}: glyph = 3'b111; {4'd2, 3'd3}: glyph = 3'b100; {4'd2, 3'd4}: glyph = 3'b111;
		{4'd3, 3'd0}: glyph = 3'b111; {4'd3, 3'd1}: glyph = 3'b001; {4'd3, 3'd2}: glyph = 3'b111; {4'd3, 3'd3}: glyph = 3'b001; {4'd3, 3'd4}: glyph = 3'b111;
		{4'd4, 3'd0}: glyph = 3'b101; {4'd4, 3'd1}: glyph = 3'b101; {4'd4, 3'd2}: glyph = 3'b111; {4'd4, 3'd3}: glyph = 3'b001; {4'd4, 3'd4}: glyph = 3'b001;
		{4'd5, 3'd0}: glyph = 3'b111; {4'd5, 3'd1}: glyph = 3'b100; {4'd5, 3'd2}: glyph = 3'b111; {4'd5, 3'd3}: glyph = 3'b001; {4'd5, 3'd4}: glyph = 3'b111;
		{4'd6, 3'd0}: glyph = 3'b111; {4'd6, 3'd1}: glyph = 3'b100; {4'd6, 3'd2}: glyph = 3'b111; {4'd6, 3'd3}: glyph = 3'b101; {4'd6, 3'd4}: glyph = 3'b111;
		{4'd7, 3'd0}: glyph = 3'b111; {4'd7, 3'd1}: glyph = 3'b001; {4'd7, 3'd2}: glyph = 3'b001; {4'd7, 3'd3}: glyph = 3'b001; {4'd7, 3'd4}: glyph = 3'b001;
		{4'd8, 3'd0}: glyph = 3'b111; {4'd8, 3'd1}: glyph = 3'b101; {4'd8, 3'd2}: glyph = 3'b111; {4'd8, 3'd3}: glyph = 3'b101; {4'd8, 3'd4}: glyph = 3'b111;
		{4'd9, 3'd0}: glyph = 3'b111; {4'd9, 3'd1}: glyph = 3'b101; {4'd9, 3'd2}: glyph = 3'b111; {4'd9, 3'd3}: glyph = 3'b001; {4'd9, 3'd4}: glyph = 3'b111;
		{4'd10, 3'd0}: glyph = 3'b101; {4'd10, 3'd1}: glyph = 3'b001; {4'd10, 3'd2}: glyph = 3'b010; {4'd10, 3'd3}: glyph = 3'b100; {4'd10, 3'd4}: glyph = 3'b101;
		default: glyph = 3'b000;
	endcase
endfunction

wire [6:0] ox = sx - 7'd112;
reg  [3:0] o_code;
reg  [1:0] o_col;
reg  [2:0] o_row;
reg        o_box1, o_txt1, o_box, o_px;
wire [2:0] o_gl = glyph(o_code, o_row);
always @(posedge clk) begin
	o_box1 <= ovl_en && sx >= 7'd111 && sy >= 7'd13 && sy <= 7'd19;
	o_txt1 <= ovl_en && sx >= 7'd112 && sy >= 7'd14 && sy <= 7'd18;
	o_col  <= ox[1:0];
	o_row  <= 3'(sy - 7'd14);
	case (ox[3:2])
		2'd0:    o_code <= ovl_bcd[11:8] == 0 ? 4'd15 : ovl_bcd[11:8];
		2'd1:    o_code <= (ovl_bcd[11:4] == 0) ? 4'd15 : ovl_bcd[7:4];
		2'd2:    o_code <= ovl_bcd[3:0];
		default: o_code <= 4'd10;
	endcase
	o_box <= o_box1;
	o_px  <= o_txt1 && o_col != 2'd3 && o_gl[3'd2 - {1'b0, o_col}];
end

// The spectrum analyser: seven bars of 3 glass pixels with a gap, up to 15 rows
// tall (green, yellow above 8 rows, red above 12), and a white peak mark, on a
// black box, glass rows 89-106, columns 49-77: centred under MiSTer's OSD menu,
// which the framework draws centred on the screen, 32 glass rows tall at the
// top 48 in 1x and 2x (its bottom edge is at row 80 there, and at about row 80
// on HDMI at 720p/1080p, row 85 in 3x). Shown with spec_opt 2, or 1 while the
// OSD is open. The rows are inside the 2x mode's window. Two registered steps.
wire       spec_vis = spec_opt == 2'd2 || (spec_opt == 2'd1 && osd_open);
wire [1:0] spec_mode = spec_vis ? (spec_pk ? 2'd2 : 2'd1) : 2'd0;
wire [6:0] sx0  = 7'd50;                            // the first bar's column
reg        x_box1, x_box, x_lit, x_pk;
reg        x_cok1;
reg  [6:0] x_dx;
reg  [7:0] x_r;
reg  [3:0] x_h, x_p;
reg  [2:0] x_col;                                   // 0 green, 1 yellow, 2 red
always @(posedge clk) begin
	x_box1 <= spec_mode != 0 && sx >= sx0 - 7'd1 && sx <= sx0 + 7'd27 && sy >= 7'd89 && sy <= 7'd106;
	x_dx   <= sx - sx0;
	x_r    <= 8'd105 - {1'b0, sy};
	x_box  <= x_box1;
	// the bar this column is in, and whether it is a bar column or a gap
	x_cok1 <= x_dx < 7'd27 && x_dx[1:0] != 2'd3;
	case (x_dx[4:2])
		3'd0:    begin x_h <= spec_bars[3:0];   x_p <= spec_peaks[3:0];   end
		3'd1:    begin x_h <= spec_bars[7:4];   x_p <= spec_peaks[7:4];   end
		3'd2:    begin x_h <= spec_bars[11:8];  x_p <= spec_peaks[11:8];  end
		3'd3:    begin x_h <= spec_bars[15:12]; x_p <= spec_peaks[15:12]; end
		3'd4:    begin x_h <= spec_bars[19:16]; x_p <= spec_peaks[19:16]; end
		3'd5:    begin x_h <= spec_bars[23:20]; x_p <= spec_peaks[23:20]; end
		default: begin x_h <= spec_bars[27:24]; x_p <= spec_peaks[27:24]; end
	endcase
	x_lit  <= x_cok1 && x_r[7:4] == 4'd0 && x_r[3:0] < x_h;
	x_pk   <= x_cok1 && spec_mode == 2'd2 && x_r[7:4] == 4'd0 && x_p > x_h && x_r[3:0] == x_p - 4'd1;
	x_col  <= x_r[3:0] >= 4'd12 ? 3'd2 : x_r[3:0] >= 4'd8 ? 3'd1 : 3'd0;
end
wire [23:0] x_rgb = x_pk ? 24'hffffff : !x_lit ? 24'd0 : x_col == 3'd2 ? 24'hff3020 : x_col == 3'd1 ? 24'hf0d020 : 24'h20d030;

// ---- the fine modes' glass pixel: read from the st7735, mixed with the row before ----
// sx runs 4 ticks (8 clocks) ahead of the pixel shown: it is set to 0 at 4 ticks
// before the line's end, T ticks later to 1, and so on. A pixel's colour comes
// back from the st7735 3 clocks after sx moves (vrgb), and lbuf's 1 clock after
// (bq). Clocks after sx moves (C0): sa = vrgb, sb = bq (C4); sa2, the products
// (C5); mrgb, the mix (C6); the pixel is shown from C8 (the 4th tick). lbuf
// takes the row just read (wd = sa a clock later) at C6, after bq was last used
// for this pixel (C4) and before sx moves again (28 clocks at least).
(* ramstyle = "M10K, no_rw_check" *) reg [23:0] lbuf [0:127];
reg  [23:0] bq, sa, sb, sa2, wd, mrgb;
reg  [11:0] pa_r, pa_g, pa_b, pb_r, pb_g, pb_b;
reg  [5:0]  zs;
reg         fok;                // sx is in 0..127 (a glass pixel of the picture)
wire [2:0]  wq = ftype == 2'd2 ? 3'd4 : fw;
always @(posedge clk) begin
	bq  <= lbuf[sx];
	sa  <= vrgb;
	sb  <= bq;
	sa2 <= sa;
	wd  <= sa;
	pa_r <= (4'd8 - {1'b0, wq}) * sb[23:16]; pb_r <= wq * sa[23:16];
	pa_g <= (4'd8 - {1'b0, wq}) * sb[15:8];  pb_g <= wq * sa[15:8];
	pa_b <= (4'd8 - {1'b0, wq}) * sb[7:0];   pb_b <= wq * sa[7:0];
	mrgb <= ftype == 2'd0 ? sa2 : {8'((pa_r + pb_r + 12'd4) >> 3), 8'((pa_g + pb_g + 12'd4) >> 3), 8'((pa_b + pb_b + 12'd4) >> 3)};
	if (zs[5] && fst) lbuf[sx] <= wd;
end

always @(posedge clk) begin
	div    <= (div >= pm1) ? 4'd0 : div + 1'd1;
	ce_pix <= div == pm1 - 1'd1;    // high on the clock with div == pm1, the last of the pixel
	ce_px  <= div == pm1 - 1'd1 && (!fine || xr == 4'd3);
	nq     <= fine && n_isN ? n_row + 1'd1 : n_row;
	ntype  <= n_isN ? 2'd1 : n_isP ? 2'd2 : 2'd0;
	nw     <= n_isN ? n_wN : 3'd4;
	nact   <= vn >= l0 && vn < pend;
	vline  <= div == 4'd0 && h == hs1 && nact;
	zs     <= {zs[4:0], 1'b0};

	if (ce_pix) begin
		// what this pixel period looked up, out together with its timing (in the fine modes the colour only on
		// every T-th tick, when sx is on the pixel to show: xr is 3)
		if (!fine || xr == 4'd3) begin
			{r, g, b} <= (hact_w && vact_w && pact_w) ? (o_box ? {24{o_px}} : x_box ? x_rgb : fine ? mrgb : vrgb) : 24'd0;
			p_gx <= sx; p_gy <= sy;
			p_act <= hact_w && vact_w && pact_w;
			p_yl  <= yr == yrw;
			p_wl  <= v == wend - 1'd1;
		end
		hblank    <= !hact_w;
		vblank    <= !vact_w;
		hs        <= h >= hs0 && h < hs1;
		vs        <= v < vs1;

		// the column: whole line in modes 0-2; in the fine modes it starts 3 ticks before the line's end, T ticks a pixel
		if (fine) begin
			if (h == htot1 - 12'd3) begin
				sx <= 0; xr <= 0; fok <= 1; zs[0] <= 1;
				ftype <= ntype; fw <= nw; fst <= nact;
			end
			else if (xr == xw) begin
				xr <= 0; sx <= sx + 1'd1;
				if (sx == 7'd127) fok <= 0; else if (fok) zs[0] <= 1;
			end
			else xr <= xr + 1'd1;
		end
		else if (hend) begin sx <= 0; xr <= 0; end
		else if (xr == xw) begin sx <= sx + 1'd1; xr <= 0; end
		else xr <= xr + 1'd1;

		if (hend) begin
			h  <= 0;
			v  <= vn;
			if (vn == l0) begin sy <= syi; yr <= yri; end
			else if (yr == yrw) begin sy <= sy + 1'd1; yr <= 0; end
			else yr <= yr + 1'd1;
			if (vlast) begin
				mode <= (mode_in > 3'd4) ? 3'd0 : mode_in;
				filt <= filt_in;
				// 0-15 down, 16-30 are 15-1 up, 31 is 0; none in 3x
				voff <= (mode_in == 3'd2) ? 6'sd0 : voff_in[4] ? 6'($signed({1'b0, voff_in}) - 7'sd31) : $signed({1'b0, voff_in});
				// 0-20 right, 21-40 are 20-1 left, more is 0; none in 3x
				hoff <= (mode_in == 3'd2 || hoff_in > 6'd40) ? 6'sd0 : 6'(hoff_in > 6'd20 ? $signed({1'b0, hoff_in}) - 7'sd41 : $signed({1'b0, hoff_in}));
			end
		end
		else h <= h + 1'd1;
	end
end

endmodule

// A delay of d ticks (ce ticks, 2 to 63) in a small memory: q after tick n is din as
// it was after tick n - d (d = 2: a register stage and one more).
module chg_tdelay #(parameter W = 4)
(
	input              clk,
	input              ce,
	input        [5:0] d,
	input      [W-1:0] din,
	output reg [W-1:0] q
);
(* ramstyle = "MLAB, no_rw_check" *) reg [W-1:0] mem [0:63];
reg [5:0] wp;
always @(posedge clk) if (ce) begin
	mem[wp] <= din;
	q       <= mem[wp - (d - 6'd1)];
	wp      <= wp + 6'd1;
end
endmodule

// The fine modes' last stage, after chg_postfx: shows each glass pixel (which
// arrives once, at ce_px) for its T ticks, and with soft (the Scale filter's
// Soft) blends the 4 first ticks of a pixel 3 : 1 with the pixel before, and
// the 4 last 3 : 1 with the pixel after (25% of the neighbour, rounded; not
// across the picture's edge). It needs the pixel after, so it shows a pixel
// at 2T + 1 ticks after it arrived: the timing is delayed by as much. In modes
// 0-2 it is a wire.
module chg_hfine
(
	input             clk,
	input             ce_pix,
	input             ce_px,
	input       [2:0] mode,
	input             sfilt,
	input       [7:0] r_in, g_in, b_in,
	input             act_in,       // the pixel is in the picture's columns
	input             hs_in, vs_in, hblank_in, vblank_in,
	output      [7:0] r, g, b,
	output            hs, vs, hblank, vblank
);
wire       fine = mode >= 3'd3;
wire [3:0] tm1  = mode == 3'd4 ? 4'd14 : 4'd13;          // T - 1
wire [5:0] dly  = mode == 3'd4 ? 6'd31 : 6'd29;          // 2T + 1

reg  [23:0] prv, cur, nxt;
reg         pa, ca, na;
reg   [3:0] pos;                // ticks since the pixel arrived
reg   [7:0] fr, fg, fb;
function [7:0] bl(input [7:0] c, input [7:0] n);
	bl = 8'((10'(c) * 10'd3 + 10'(n) + 10'd2) >> 2);
endfunction
wire lft = sfilt && ca && pa && pos < 4'd4;
wire rgt = sfilt && ca && na && {1'b0, pos} + 5'd4 > {1'b0, tm1};
always @(posedge clk) if (ce_pix) begin
	if (ce_px) begin
		prv <= cur; cur <= nxt; nxt <= {r_in, g_in, b_in};
		pa  <= ca;  ca  <= na;  na  <= act_in;
		pos <= 4'd0;
	end
	else if (pos != 4'd15) pos <= pos + 4'd1;
	fr <= lft ? bl(cur[23:16], prv[23:16]) : rgt ? bl(cur[23:16], nxt[23:16]) : cur[23:16];
	fg <= lft ? bl(cur[15:8],  prv[15:8])  : rgt ? bl(cur[15:8],  nxt[15:8])  : cur[15:8];
	fb <= lft ? bl(cur[7:0],   prv[7:0])   : rgt ? bl(cur[7:0],   nxt[7:0])   : cur[7:0];
end

wire [3:0] td_q;
chg_tdelay #(.W(4)) td (.clk(clk), .ce(ce_pix), .d(dly), .din({hs_in, vs_in, hblank_in, vblank_in}), .q(td_q));

assign r = fine ? fr : r_in;
assign g = fine ? fg : g_in;
assign b = fine ? fb : b_in;
assign hs     = fine ? td_q[3] : hs_in;
assign vs     = fine ? td_q[2] : vs_in;
assign hblank = fine ? td_q[1] : hblank_in;
assign vblank = fine ? td_q[0] : vblank_in;
endmodule
