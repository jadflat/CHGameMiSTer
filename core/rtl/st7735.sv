// The CHGame's display controller, an ST7735S, and the 128x128 glass behind it.
//
// A line-by-line translation of joyrider3774's emu/src/st7735.c (MIT), which
// was measured against the real panel: read that file for the why of every
// rule here. The SPI side hands in one byte at a time with its D/C level;
// the video side asks for the pixel at glass position (x, y) and gets its
// 24-bit colour three clocks later.
//
// Differences from st7735.c, on purpose:
// - the frame memory's power-on pattern (st7735_power_on) is loaded when the
//   FPGA is configured, not again for each game loaded
// - the read commands are not modelled (neither are they there)
//

module st7735
(
	input             clk,
	input             reset,        // RESX low (PB12): MADCTL and COLMOD back to defaults
	input             rst_level,    // RESX as the pin is now; low shows a blank glass

	// a byte from SPI1 while CS (PA4) is low
	input             byte_stb,
	input             dc,           // PB0: 0 command, 1 parameter or pixel data
	input       [7:0] byte_in,

	// the glass, read by the video side; x and y 0..127
	input       [6:0] vx,
	input       [6:0] vy,
	input             vline,        // pulse at the start of each line: latches the row for vy
	output reg [23:0] vrgb
);

localparam COLS  = 132;
localparam ROWS  = 162;
localparam LINES = 132;

localparam MAD_MY = 7, MAD_MX = 6, MAD_MV = 5, MAD_ML = 4, MAD_BGR = 3;

// ---------------------------------------------------------------------------
// frame memory: 162 rows of 132 18-bit pixels, R 17:12, G 11:6, B 5:0
// ---------------------------------------------------------------------------

reg [17:0] gram[ROWS*COLS];
// at power-on: st7735_power_on's pattern (tools/gen_rom.py); a game that
// turns the display on before drawing shows it, as on the board
`ifdef VERILATOR
initial $readmemh("core/rtl/gram_init.hex", gram);
`else
initial $readmemh("rtl/gram_init.hex", gram);
`endif

reg        gwe;
reg [14:0] gwaddr;
reg [17:0] gwdata;
always @(posedge clk) if (gwe) gram[gwaddr] <= gwdata;

reg [14:0] graddr;
reg [17:0] grdata;
always @(posedge clk) grdata <= gram[graddr];

// ---------------------------------------------------------------------------
// controller state
// ---------------------------------------------------------------------------

reg  [7:0] cmd;
reg  [3:0] nparam;              // parameters seen, saturating
reg  [7:0] p0, p2, p4;          // the high bytes of 16-bit parameters
reg [15:0] xs, xe, ys, ye, cx, cy;
reg        writing;
reg  [7:0] pix0, pix1;
reg  [1:0] npix;
reg  [7:0] madctl, colmod;
reg        sleeping, display_on, inverted, idle, partial, scrolling;
reg [15:0] psl, pel, tfa, vsa, ssa;

function [5:0] c5(input [4:0] v); c5 = {v, &v}; endfunction
function [5:0] c4(input [3:0] v); c4 = {v, v[3:2]}; endfunction

// one pixel at the address counter, then the counter moves on (put_pixel)
task automatic put_pixel(input [17:0] c);
	reg [15:0] col, row, t;
	begin
		col = cx; row = cy;
		if (madctl[MAD_MV]) begin t = col; col = row; row = t; end
		if (madctl[MAD_MX]) col = COLS - 1 - col;
		if (madctl[MAD_MY]) row = LINES - 1 - row;
		if (col < COLS && row < ROWS) begin
			gwe    <= 1;
			gwaddr <= 15'(row * COLS + col);
			gwdata <= c;
		end
		if (cx + 1'd1 > xe) begin
			cx <= xs;
			cy <= (cy + 1'd1 > ye) ? ys : cy + 1'd1;
		end
		else cx <= cx + 1'd1;
	end
endtask

task automatic common_reset;
	begin
		cmd <= 0; nparam <= 0; writing <= 0; npix <= 0;
		sleeping <= 1; display_on <= 0; inverted <= 0; idle <= 0;
		partial <= 0; scrolling <= 0;
		psl <= 0; pel <= LINES - 1; tfa <= 0; vsa <= LINES; ssa <= 0;
		xs <= 0; xe <= LINES - 1; ys <= 0; ye <= LINES - 1;
		cx <= 0; cy <= 0;
	end
endtask

always @(posedge clk) begin
	gwe <= 0;
	if (reset) begin
		madctl <= 0;
		colmod <= 8'h06;
		common_reset();
	end
	else if (byte_stb && !dc) begin
		// a command (command())
		cmd <= byte_in; nparam <= 0; writing <= 0; npix <= 0;
		case (byte_in)
			8'h01: common_reset();
			8'h10: sleeping <= 1;
			8'h11: sleeping <= 0;
			8'h12: begin partial <= 1; scrolling <= 0; end
			8'h13: begin partial <= 0; scrolling <= 0; end
			8'h20: inverted <= 0;
			8'h21: inverted <= 1;
			8'h28: display_on <= 0;
			8'h29: display_on <= 1;
			8'h38: idle <= 0;
			8'h39: if (!partial) idle <= 1;
			8'h2C: begin writing <= 1; cx <= xs; cy <= ys; end
			default: ;
		endcase
	end
	else if (byte_stb && writing) begin
		// pixel data (pixel_byte)
		case (colmod[2:0])
			3'd5: if (npix == 0) begin pix0 <= byte_in; npix <= 1; end
			      else begin
				      put_pixel({c5(pix0[7:3]), pix0[2:0], byte_in[7:5], c5(byte_in[4:0])});
				      npix <= 0;
			      end
			3'd3: if (npix == 0) begin pix0 <= byte_in; npix <= 1; end
			      else if (npix == 1) begin
				      put_pixel({c4(pix0[7:4]), c4(pix0[3:0]), c4(byte_in[7:4])});
				      pix1 <= byte_in; npix <= 2;
			      end
			      else begin
				      put_pixel({c4(pix1[3:0]), c4(byte_in[7:4]), c4(byte_in[3:0])});
				      npix <= 0;
			      end
			default:
			      if (npix == 0) begin pix0 <= byte_in; npix <= 1; end
			      else if (npix == 1) begin pix1 <= byte_in; npix <= 2; end
			      else begin
				      put_pixel({pix0[7:2], pix1[7:2], byte_in[7:2]});
				      npix <= 0;
			      end
		endcase
	end
	else if (byte_stb) begin
		// a parameter (parameter()); n is the count including this byte
		automatic reg [3:0] n = nparam + 1'd1;
		if (nparam != 4'hF) nparam <= n;
		case (n)
			1: p0 <= byte_in;
			3: p2 <= byte_in;
			5: p4 <= byte_in;
			default: ;
		endcase
		case (cmd)
			8'h2A: begin if (n == 2) xs <= {p0, byte_in}; if (n == 4) xe <= {p2, byte_in}; end
			8'h2B: begin if (n == 2) ys <= {p0, byte_in}; if (n == 4) ye <= {p2, byte_in}; end
			8'h30: begin if (n == 2) psl <= {p0, byte_in}; if (n == 4) pel <= {p2, byte_in}; end
			8'h33: begin
				if (n == 2) tfa <= {p0, byte_in};
				if (n == 4) vsa <= {p2, byte_in};
				// bfa (n == 6) is kept by st7735.c but changes nothing shown
			end
			8'h37: if (n == 2 && !partial) begin ssa <= {p0, byte_in}; scrolling <= 1; end
			8'h36: if (n == 1) madctl <= byte_in;
			8'h3A: if (n == 1 && !partial) colmod <= byte_in;
			default: ;
		endcase
	end
end

// ---------------------------------------------------------------------------
// the glass (st7735_render): screen pixel (x, y) is memory column 129-x on
// panel line 128-y, through scrolling, partial mode, inversion and idle
// ---------------------------------------------------------------------------

// memory_row() for the line about to be shown, worked out once a line in
// three registered steps after vline (the line's first pixel is read a dozen
// clocks later): g and s; the scrolling area; the row and whether it is shown
reg        row_ok;
reg  [7:0] row;
reg  [2:0] rstep;
reg  signed [9:0] rs;                   // s
reg  [8:0] rg, rend;                    // g, the end of the scrolling area
reg        rin;                         // s is inside the scrolling area
reg  signed [9:0] rr;                   // the row, -1 for none
always @(posedge clk) begin
	rstep <= {rstep[1:0], vline};
	if (vline) begin
		automatic reg [8:0] g = 9'd128 - vy;
		automatic reg [16:0] e = tfa + vsa;
		rg   <= g;
		rs   <= madctl[MAD_ML] ? 10'(LINES - 1) - 10'(g) : 10'(g);
		rend <= e > LINES ? 9'(LINES) : 9'(e);
	end
	if (rstep[0]) begin
		rin <= scrolling && rs >= $signed({1'b0, tfa}) && rs < $signed({1'b0, rend});
	end
	if (rstep[1]) begin
		// 18 bits: SSA is any 16-bit value, and the C's s is an int (a big SSA ends up off the glass: white)
		automatic reg signed [17:0] t = 18'($signed({2'b0, ssa})) + 18'(rs) - 18'($signed({2'b0, tfa}));
		automatic reg signed [17:0] u = (t >= $signed({9'b0, rend})) ? t - 18'(rend - tfa) : t;
		automatic reg signed [17:0] sf = rin ? u : 18'(rs);
		if (!scrolling) rr <= 10'(rg);
		else if (sf < 0 || sf >= LINES) rr <= -10'sd1;
		else rr <= madctl[MAD_ML] ? 10'(LINES - 1) - 10'(sf) : 10'(sf);
	end
	if (rstep[2]) begin
		row_ok <= rr >= 0 && (!partial || (psl <= pel ? (rr >= $signed({1'b0, psl}) && rr <= $signed({1'b0, pel}))
		                                              : (rr >= $signed({1'b0, psl}) || rr <= $signed({1'b0, pel}))));
		row <= 8'(rr);
	end
end

wire blank_all = !display_on || sleeping || !rst_level;

always @(posedge clk) graddr <= 15'(row * COLS + (129 - vx));

function [7:0] c8(input [5:0] v); c8 = {v, v[5:4]}; endfunction

reg row_ok1, blank1;
always @(posedge clk) begin
	reg [17:0] v;
	reg  [5:0] r, g, b;
	{row_ok1, blank1} <= {row_ok, blank_all};       // line up with grdata
	v = grdata;
	if (inverted) v = v ^ 18'h3FFFF;
	if (idle) v = {{6{v[17]}}, {6{v[11]}}, {6{v[5]}}};
	{r, g, b} = madctl[MAD_BGR] ? {v[17:12], v[11:6], v[5:0]} : {v[5:0], v[11:6], v[17:12]};
	vrgb <= (blank1 || !row_ok1) ? 24'hFFFFFF : {c8(r), c8(g), c8(b)};
end

endmodule
