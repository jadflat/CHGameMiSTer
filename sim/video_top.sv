// st7735, chg_video and chg_hfine joined as in CHGame.sv (without chg_postfx), for sim/tb_video.cpp
module video_top
(
	input        clk,
	input        reset,
	input  [2:0] mode_in,
	input  [1:0] filt_in,
	input  [4:0] voff_in,
	input  [5:0] hoff_in,
	input        ovl_en,
	input [11:0] ovl_bcd,
	input  [1:0] spec_opt,
	input        spec_pk,
	input        osd_open,
	input [27:0] spec_bars,
	input [27:0] spec_peaks,
	output [2:0] mode,
	input        byte_stb,
	input        dc,
	input  [7:0] byte_in,
	output       ce_pix,
	output [7:0] r, g, b,
	output       hs, vs, hblank, vblank
);
wire  [6:0] vx, vy;
wire        vline;
wire [23:0] vrgb;
wire  [7:0] vr, vg, vb;
wire        vhs, vvs, vhblank, vvblank, ce_px, p_act, p_yl, p_wl;
wire  [6:0] p_gx, p_gy;
wire  [1:0] filt;
st7735 lcd(.clk(clk), .reset(reset), .rst_level(1'b1), .byte_stb(byte_stb), .dc(dc), .byte_in(byte_in),
           .vx(vx), .vy(vy), .vline(vline), .vrgb(vrgb));
chg_video video(.clk(clk), .mode_in(mode_in), .filt_in(filt_in), .voff_in(voff_in), .hoff_in(hoff_in), .ovl_en(ovl_en), .ovl_bcd(ovl_bcd), .spec_opt(spec_opt), .spec_pk(spec_pk), .osd_open(osd_open), .spec_bars(spec_bars), .spec_peaks(spec_peaks), .mode(mode), .filt(filt), .ce_pix(ce_pix), .ce_px(ce_px),
                .p_gx(p_gx), .p_gy(p_gy), .p_act(p_act), .p_yl(p_yl), .p_wl(p_wl), .vx(vx), .vy(vy), .vline(vline), .vrgb(vrgb),
                .r(vr), .g(vg), .b(vb), .hs(vhs), .vs(vvs), .hblank(vhblank), .vblank(vvblank));
// as CHGame.sv has it, minus chg_postfx: every pixel is "in the picture's columns" when not blanked
chg_hfine hfine(.clk(clk), .ce_pix(ce_pix), .ce_px(ce_px), .mode(mode), .sfilt(filt == 2'd3), .r_in(vr), .g_in(vg), .b_in(vb), .act_in(!vhblank),
                .hs_in(vhs), .vs_in(vvs), .hblank_in(vhblank), .vblank_in(vvblank),
                .r(r), .g(g), .b(b), .hs(hs), .vs(vs), .hblank(hblank), .vblank(vblank));
endmodule
