// st7735, chg_video and chg_postfx joined as in CHGame.sv, for sim/tb_postfx.cpp:
// v_* is chg_video's picture, the others chg_postfx's
module postfx_top
(
	input        clk,
	input        reset,
	input  [2:0] mode_in,
	input  [1:0] filt_in,
	input  [4:0] voff_in,
	input  [5:0] hoff_in,
	input  [2:0] grid_str,
	input  [1:0] grid_sty,
	input        lcd_col,
	input        led_en,
	input        led,
	output [2:0] mode,
	output       ce_px,
	input        byte_stb,
	input        dc,
	input  [7:0] byte_in,
	output       ce_pix,
	output [7:0] v_r, v_g, v_b,
	output       v_hs, v_vs, v_hblank, v_vblank,
	output [7:0] r, g, b,
	output       hs, vs, hblank, vblank,
	output [7:0] pr, pg, pb,
	output       phs, pvs, phblank, pvblank, p_hact
);
wire  [6:0] vx, vy;
wire        vline;
wire [23:0] vrgb;
st7735 lcd(.clk(clk), .reset(reset), .rst_level(1'b1), .byte_stb(byte_stb), .dc(dc), .byte_in(byte_in),
           .vx(vx), .vy(vy), .vline(vline), .vrgb(vrgb));
wire [6:0] p_gx, p_gy;
wire       p_act, p_yl, p_wl;
wire [1:0] filt;
chg_video video(.clk(clk), .mode_in(mode_in), .filt_in(filt_in), .voff_in(voff_in), .hoff_in(hoff_in), .ovl_en(1'b0), .ovl_bcd(12'd0), .spec_opt(2'd0), .spec_pk(1'b0), .osd_open(1'b0), .spec_bars(28'd0), .spec_peaks(28'd0),
                .mode(mode), .filt(filt), .ce_pix(ce_pix), .ce_px(ce_px), .p_gx(p_gx), .p_gy(p_gy), .p_act(p_act), .p_yl(p_yl), .p_wl(p_wl), .vx(vx), .vy(vy), .vline(vline), .vrgb(vrgb),
                .r(v_r), .g(v_g), .b(v_b), .hs(v_hs), .vs(v_vs), .hblank(v_hblank), .vblank(v_vblank));
chg_postfx postfx(.clk(clk), .ce_pix(ce_pix), .ce_px(ce_px), .mode(mode), .voff_in(voff_in), .grid_str(grid_str), .grid_sty(grid_sty), .lcd_col(lcd_col), .led_en(led_en), .led(led),
                  .r_in(v_r), .g_in(v_g), .b_in(v_b), .hs_in(v_hs), .vs_in(v_vs), .hblank_in(v_hblank), .vblank_in(v_vblank),
                  .p_gx(p_gx), .p_gy(p_gy), .p_act(p_act), .p_yl(p_yl), .p_wl(p_wl),
                  .r(pr), .g(pg), .b(pb), .hs(phs), .vs(pvs), .hblank(phblank), .vblank(pvblank), .px_act(p_hact));
// the whole chain, as CHGame.sv: r, g, b, hs ... are chg_hfine's; pr ... chg_postfx's
chg_hfine hfine(.clk(clk), .ce_pix(ce_pix), .ce_px(ce_px), .mode(mode), .sfilt(filt == 2'd3), .r_in(pr), .g_in(pg), .b_in(pb), .act_in(p_hact),
                .hs_in(phs), .vs_in(pvs), .hblank_in(phblank), .vblank_in(pvblank),
                .r(r), .g(g), .b(b), .hs(hs), .vs(vs), .hblank(hblank), .vblank(vblank));
endmodule
