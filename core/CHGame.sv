//============================================================================
//
//  CHGame for MiSTer: the CHGame handheld (WCH CH32X035, 128x128 ST7735,
//  piezo buzzer, microSD) in the FPGA, playing .CHG games unchanged.
//
//  The machine is modelled on joyrider3774's chgame_emulator (MIT); the
//  framework around it is MiSTer-devel's Template_MiSTer (GPL). CHGame is by
//  Kevin Bates (Apache-2.0; its bootloader is MIT).
//
//  Top level of the core (the emu module of the MiSTer framework).
//
//============================================================================

module emu
(
	`include "sys/emu_ports.vh"
);

///////// Default values for ports not used in this core /////////

assign ADC_BUS  = 'Z;
assign USER_OUT = '1;
assign {UART_RTS, UART_TXD, UART_DTR} = 0;
assign {SD_SCK, SD_MOSI, SD_CS} = 'Z;
assign {SDRAM_DQ, SDRAM_A, SDRAM_BA, SDRAM_CLK, SDRAM_CKE, SDRAM_DQML, SDRAM_DQMH, SDRAM_nWE, SDRAM_nCAS, SDRAM_nRAS, SDRAM_nCS} = 'Z;
// DDR3 holds a .DAT loaded from the OSD (chg_card), on clk_sys: no crossing
assign DDRAM_CLK = clk_sys;

assign VGA_F1 = 0;
assign VGA_SCALER  = 0;
assign VGA_DISABLE = 0;
assign HDMI_FREEZE = 0;
assign HDMI_BLACKOUT = 0;
assign HDMI_BOB_DEINT = 0;

assign AUDIO_MIX = 0;
assign LED_DISK = 0;
assign LED_POWER = 0;
assign BUTTONS = 0;

//////////////////////////////////////////////////////////////////


`include "build_id.v"
localparam CONF_STR = {
	"CHGame;;",
	"-;",
	"FS1,CHG,Load game;",
	"F2,DAT,Load data file,38000000;",
	"R[65],Clear data files;",
	"S1,IMG,Mount microSD image;",
	"-;",
	"R[8],Save now;",
	"R[90],Reload save;",
	"O[7],Autosave,On,Off;",
	"-;",
	"O[102:100],Board clock,100% (real),110%,125%,150%,175%,200% (max);",
	"O[1],Speed,Normal,Fast forward;",
	"O[2],Speed display,Off,On;",
	"O[92:91],Fast card loading,Auto,Always on,Off,Off;",
	"O[64],Pause when OSD open,Off,On;",
	"-;",
	"P1,Video;",
	"P2,Audio;",
	"P3,Equaliser;",
	"-;",
	"T[0],Reset;",
	"R[0],Reset and close OSD;",
	"P1-;",
	"P1O[97:96],Analogue mode,15kHz 1x,15kHz 1.75x,15kHz 240x240,15kHz 2x;",
	"P1O[99:98],Scale filter,Off,Even rows,Sharp bilinear,Soft;",
	"P1O[20:16],V position,0,+1 down,+2 down,+3 down,+4 down,+5 down,+6 down,+7 down,+8 down,+9 down,+10 down,+11 down,+12 down,+13 down,+14 down,+15 down,-15 up,-14 up,-13 up,-12 up,-11 up,-10 up,-9 up,-8 up,-7 up,-6 up,-5 up,-4 up,-3 up,-2 up,-1 up;",
	"P1O[26:21],H position,0,+1 right,+2 right,+3 right,+4 right,+5 right,+6 right,+7 right,+8 right,+9 right,+10 right,+11 right,+12 right,+13 right,+14 right,+15 right,+16 right,+17 right,+18 right,+19 right,+20 right,-20 left,-19 left,-18 left,-17 left,-16 left,-15 left,-14 left,-13 left,-12 left,-11 left,-10 left,-9 left,-8 left,-7 left,-6 left,-5 left,-4 left,-3 left,-2 left,-1 left;",
	"P1O[89],Per-game position,On,Off;",
	"P1-;",
	"P1O[122:121],Aspect ratio,Original,Full Screen,[ARC1],[ARC2];",
	"P1O[14:12],Scale,Normal,V-Integer,Narrower HV-Integer,Wider HV-Integer;",
	"P1O[11:9],Scandoubler Fx,None,HQ2x,CRT 25%,CRT 50%,CRT 75%;",
	"P1-;",
	"P1O[86:84],LCD grid (2x only),Off,Light,Medium,Dark,Black;",
	"P1O[88:87],Grid style,Lines,Rows only,Columns only,Dots;",
	"P1O[82],LCD colours,Off,On;",
	"P1O[83],LED dot,Off,On;",
	"P2-;",
	"P2O[29:27],Sound,Bare pin,Piezo filter,Small piezo,Large buzzer,Speaker,Warm,Handheld,Bright;",
	"P2O[6:4],Volume,50%,75%,100%,150%,25%,Off;",
	"P2-;",
	"P2O[32:30],EQ preset,Off,Bass boost,Treble boost,Loudness,Vocal,Custom;",
	"P2-;",
	"P2O[35:34],Spectrum,Off,While in menu,Always;",
	"P2O[33],Spectrum peaks,Off,On;",
	"P3-,Used when EQ preset is Custom;",
	"P3-;",
	"P3O[39:36],63 Hz,0 dB,+3 dB,+6 dB,+9 dB,+12 dB,-12 dB,-9 dB,-6 dB,-3 dB;",
	"P3O[43:40],160 Hz,0 dB,+3 dB,+6 dB,+9 dB,+12 dB,-12 dB,-9 dB,-6 dB,-3 dB;",
	"P3O[47:44],400 Hz,0 dB,+3 dB,+6 dB,+9 dB,+12 dB,-12 dB,-9 dB,-6 dB,-3 dB;",
	"P3O[51:48],1 kHz,0 dB,+3 dB,+6 dB,+9 dB,+12 dB,-12 dB,-9 dB,-6 dB,-3 dB;",
	"P3O[55:52],2.5 kHz,0 dB,+3 dB,+6 dB,+9 dB,+12 dB,-12 dB,-9 dB,-6 dB,-3 dB;",
	"P3O[59:56],6.3 kHz,0 dB,+3 dB,+6 dB,+9 dB,+12 dB,-12 dB,-9 dB,-6 dB,-3 dB;",
	"P3O[63:60],16 kHz,0 dB,+3 dB,+6 dB,+9 dB,+12 dB,-12 dB,-9 dB,-6 dB,-3 dB;",
	"J1,B,A,Start,Select,FF,Up;",
	"jn,A,B,Select,Start,R,X;",
	"v,0;",
	"V,v0.5-",`BUILD_DATE
};

wire         forced_scandoubler;
wire  [21:0] gamma_bus;
wire   [1:0] buttons;
wire [127:0] status;
wire         pos_set;                  // chg_save: a game's stored picture position goes into the OSD
wire   [4:0] pos_set_v;
wire   [5:0] pos_set_h;
wire [127:0] status_in = {status[127:27], pos_set_h, pos_set_v, status[15:0]};
wire  [31:0] joystick_0;

wire         ioctl_download;
wire  [15:0] ioctl_index;
wire         ioctl_wr;
wire  [26:0] ioctl_addr;
wire   [7:0] ioctl_dout;
wire         ioctl_wait;

// virtual disk 0: the game's .sav, which "FS" makes MiSTer mount beside the
// game; virtual disk 1: the microSD card image
wire   [1:0] img_mounted;
wire         img_readonly;
wire  [63:0] img_size;
wire  [31:0] sd_lba[2];
wire   [1:0] sd_rd, sd_wr, sd_ack;
wire  [13:0] sd_buff_addr;           // hps_io allows blocks up to 16 KB; ours are 512 bytes
wire   [7:0] sd_buff_dout;
wire   [7:0] sd_buff_din[2];
wire         sd_buff_wr;

hps_io #(.CONF_STR(CONF_STR), .VDNUM(2)) hps_io
(
	.clk_sys(clk_sys),
	.HPS_BUS(HPS_BUS),
	.EXT_BUS(),
	.gamma_bus(gamma_bus),

	.forced_scandoubler(forced_scandoubler),
	.buttons(buttons),
	.status(status),
	.status_in(status_in),
	.status_set(pos_set),
	.status_menumask(16'd0),
	.joystick_0(joystick_0),

	.ioctl_download(ioctl_download),
	.ioctl_index(ioctl_index),
	.ioctl_wr(ioctl_wr),
	.ioctl_addr(ioctl_addr),
	.ioctl_dout(ioctl_dout),
	.ioctl_wait(ioctl_wait),

	.img_mounted(img_mounted),
	.img_readonly(img_readonly),
	.img_size(img_size),
	.sd_lba(sd_lba),
	.sd_rd(sd_rd),
	.sd_wr(sd_wr),
	.sd_ack(sd_ack),
	.sd_buff_addr(sd_buff_addr),
	.sd_buff_dout(sd_buff_dout),
	.sd_buff_din(sd_buff_din),
	.sd_buff_wr(sd_buff_wr)
);

///////////////////////   CLOCKS   ///////////////////////////////

wire clk_sys;           // 96 MHz: two clocks a board cycle
pll pll
(
	.refclk(CLK_50M),
	.rst(0),
	.outclk_0(clk_sys)
);

// Board time (chg_throttle.sv): 48 MHz on average, or as fast as the CPU can
// while fast-forwarding (the OSD's Speed, or the pad's FF button held).
wire       ce;                          // a board cycle happened on this clock
wire       run;                         // (96 MHz: ce goes everywhere, so run is a register)
wire       card_sel;                    // the card's CS is low (chg_soc)
// Fast card loading (OSD, status[92:91]): 0 Auto (chg_soc's loading detector), 1 Always on, 2/3 Off (cycle exact).
// card_sel is high only while fast loading is active and the card is selected.
wire       ff = status[1] | joystick_0[8] | card_sel;
// Pause when OSD open: the OSD's open flag comes from sys_top's clock, so it goes through two registers.
// The board stops (chg_throttle) and the sound is muted; picture and OSD carry on.
reg  [1:0] osd_sync = 2'b00;
always @(posedge clk_sys) osd_sync <= {osd_sync[0], OSD_STATUS};
wire       osd_status  = osd_sync[1];     // the OSD menu is open (synchronised)
wire       pause_block = status[35:34] == 2'd1;           // high: the pause does not apply (for the spectrum analyser's "While in menu")
wire       pause = status[64] & osd_status & ~pause_block;
chg_throttle throttle(.clk(clk_sys), .ff(ff), .rate(status[102:100]), .pause(pause), .ce(ce), .run(run));

// the speed display: board cycles done in the last half second, as a percent
wire [11:0] speed_bcd;
chg_speed speed(.clk(clk_sys), .ce(ce), .bcd(speed_bcd));

/////////////////////////  LOADING AND SAVES  ////////////////////

wire        ld_we, ld_busy, ld_bad, ld_wait, card_wait;
wire [15:0] ld_addr;
assign ioctl_wait = ld_wait | card_wait;
wire  [7:0] ld_data;

chg_loader loader
(
	.clk(clk_sys),
	.reset(RESET),
	.download_in(ioctl_download && ioctl_index[5:0] == 6'd1),
	.wr_in(ioctl_wr),
	.addr_in(ioctl_addr),
	.din_in(ioctl_dout),
	.wait_req(ld_wait),
	.fl_we(ld_we),
	.fl_addr(ld_addr),
	.fl_data(ld_data),
	.busy(ld_busy),
	.bad(ld_bad)
);

reg ld_busy_d;
always @(posedge clk_sys) ld_busy_d <= ld_busy;
wire loaded = ld_busy_d && !ld_busy;
// black until the first game has loaded: the panel would show white (lit, never set up)
reg  have_game = 0;
always @(posedge clk_sys) if (loaded) have_game <= 1;

wire        sv_we, sv_busy, sv_pending;
wire [15:0] sv_addr;
wire  [7:0] sv_data;

// the board's own writes to its flash (the flash controller), for the save
wire        gfw_we;
wire [15:0] gfw_addr;
wire  [2:0] gfw_size;
wire [31:0] gfw_data;

// into the flash while the board is held in reset: the loader, then the save
wire        fl_we   = ld_we | sv_we;
wire [15:0] fl_addr = ld_we ? ld_addr : sv_addr;
wire  [7:0] fl_data = ld_we ? ld_data : sv_data;

chg_save save
(
	.clk(clk_sys),
	.reset(RESET),
	.ce(ce),
	.fw_we(fl_we | gfw_we),
	.fw_addr(fl_we ? fl_addr : gfw_addr),
	.fw_size(fl_we ? 3'd1 : gfw_size),
	.fw_data(fl_we ? {24'd0, fl_data} : gfw_data),
	.fw_game(!fl_we),
	.loaded(loaded),
	.busy(sv_busy),
	.fl_we(sv_we),
	.fl_addr(sv_addr),
	.fl_data(sv_data),
	.save_now(status[8]),
	.autosave(!status[7]),
	.reload(status[90]),
	.img_mounted(img_mounted[0]),
	.img_size(img_size),
	.img_readonly(img_readonly),
	.sd_lba(sd_lba[0]),
	.sd_rd(sd_rd[0]),
	.sd_wr(sd_wr[0]),
	.sd_ack(sd_ack[0]),
	.sd_buff_addr(sd_buff_addr[8:0]),
	.sd_buff_dout(sd_buff_dout),
	.sd_buff_din(sd_buff_din[0]),
	.sd_buff_wr(sd_buff_wr),
	.pending(sv_pending),
	.pos_en(!status[89]), .pos_v(status[20:16]), .pos_h(status[26:21]),
	.pos_set(pos_set), .pos_set_v(pos_set_v), .pos_set_h(pos_set_h)
);

// the microSD slot: a .DAT from "Load data file" (in DDR3, a card made in
// logic) or the image mounted on virtual disk 1, whichever went in last
wire        card_busy, card_hold, c_mounted, c_rd, c_wr, c_ack, c_buff_wr;
wire [63:0] c_size;
wire [31:0] c_lba;
wire  [8:0] c_buff_addr;
wire  [7:0] c_buff_dout, c_buff_din;

chg_card card
(
	.clk(clk_sys),
	.clear_files(status[65]),
	.ioctl_download(ioctl_download), .ioctl_index(ioctl_index), .ioctl_wr(ioctl_wr),
	.ioctl_addr(ioctl_addr), .ioctl_dout(ioctl_dout), .ioctl_wait(card_wait), .busy(card_busy), .hold(card_hold),
	.img_mounted(img_mounted[1]), .img_size(img_size),
	.hps_lba(sd_lba[1]), .hps_rd(sd_rd[1]), .hps_wr(sd_wr[1]), .hps_ack(sd_ack[1]),
	.hps_buff_addr(sd_buff_addr[8:0]), .hps_buff_dout(sd_buff_dout), .hps_buff_din(sd_buff_din[1]),
	.hps_buff_wr(sd_buff_wr),
	.sd_mounted(c_mounted), .sd_size(c_size), .sd_lba(c_lba), .sd_rd(c_rd), .sd_wr(c_wr), .sd_ack(c_ack),
	.sd_buff_addr(c_buff_addr), .sd_buff_dout(c_buff_dout), .sd_buff_din(c_buff_din), .sd_buff_wr(c_buff_wr),
	.ddr_busy(DDRAM_BUSY), .ddr_burstcnt(DDRAM_BURSTCNT), .ddr_addr(DDRAM_ADDR), .ddr_dout(DDRAM_DOUT),
	.ddr_dout_ready(DDRAM_DOUT_READY), .ddr_rd(DDRAM_RD), .ddr_din(DDRAM_DIN), .ddr_be(DDRAM_BE), .ddr_we(DDRAM_WE)
);

// the board is powered off while a game loads or its save is restored (and
// stays off after a file that is not a CHGame package), and while a .DAT loads
wire reset = RESET | status[0] | buttons[1] | ld_busy | sv_busy | card_busy;

/////////////////////////  THE BOARD  ////////////////////////////

// an NES pad's buttons with each pair the other way round: the pad's A is the
// board's B, B is A, Select is START and Start is SELECT
wire [7:0] chg_buttons = { joystick_0[6], joystick_0[7], joystick_0[4], joystick_0[5],
                           joystick_0[0], joystick_0[1], joystick_0[2], joystick_0[3] | joystick_0[9] };

wire        buzzer, led;
wire  [6:0] vx, vy;
wire        vline;
wire [23:0] vrgb;

chg_soc soc
(
	.clk(clk_sys),
	.reset(reset),
	.run(run),
	.ce(ce),
	.fast_card(status[92:91] != 2'd2 && status[92:91] != 2'd3), .fast_always(status[92:91] == 2'd1), .card_sel(card_sel),
	.hold_mode(card_hold),      // the .DAT's card holds board time for its blocks; the mounted image does not

	.fl_we(fl_we),
	.fl_addr(fl_addr),
	.fl_data(fl_data),
	.gfw_we(gfw_we),
	.gfw_addr(gfw_addr),
	.gfw_size(gfw_size),
	.gfw_data(gfw_data),

	.buttons(chg_buttons),

	.vx(vx),
	.vy(vy),
	.vline(vline),
	.vrgb(vrgb),

	.buzzer(buzzer),
	.led(led),

	.sd_mounted(c_mounted),
	.sd_size(c_size),
	.sd_lba(c_lba),
	.sd_rd(c_rd),
	.sd_wr(c_wr),
	.sd_ack(c_ack),
	.sd_buff_addr(c_buff_addr),
	.sd_buff_dout(c_buff_dout),
	.sd_buff_din(c_buff_din),
	.sd_buff_wr(c_buff_wr),

	.dbg_boundary(),
	.retire(),
	.retire_pc(),
	.retire_instret(),
	.retire_cycles(),
	.dbg_x(),
	.faulted()
);

/////////////////////////  THE GLASS  ////////////////////////////

// the ST7735 and its frame memory are inside chg_soc
wire ce_pix, hs, vs, hblank, vblank;
wire [7:0] r, g, b;
wire [2:0] vmode;                       // the Analogue mode being shown (chg_video: 0 1x, 1 2x, 3 1.75x, 4 240x240)
wire       ce_px;                       // the glass pixel strobe (chg_video)
wire [6:0] p_gx, p_gy;
wire       p_act, p_yl, p_wl;
wire [1:0] vfilt;
// the OSD's Analogue mode (status 97:96: 1x, 1.75x, 240x240, 2x) as chg_video's mode number
wire [2:0] mode_sel = status[97:96] == 2'd1 ? 3'd3 : status[97:96] == 2'd2 ? 3'd4 : status[97:96] == 2'd3 ? 3'd1 : 3'd0;

chg_video video
(
	.clk(clk_sys),
	.mode_in(mode_sel), .filt_in(status[99:98]),
	.voff_in(status[20:16]), .hoff_in(status[26:21]),
	.ovl_en(status[2]),
	.ovl_bcd(speed_bcd),
	.spec_opt(status[35:34]),
	.spec_pk(status[33]),
	.osd_open(osd_status),
	.spec_bars(spec_bars),
	.spec_peaks(spec_peaks),
	.mode(vmode), .filt(vfilt),
	.ce_pix(ce_pix), .ce_px(ce_px),
	.p_gx(p_gx), .p_gy(p_gy), .p_act(p_act), .p_yl(p_yl), .p_wl(p_wl),
	.vx(vx),
	.vy(vy),
	.vline(vline),
	.vrgb(vrgb),
	.r(vr), .g(vg), .b(vb),
	.hs(vhs), .vs(vvs), .hblank(vhblank), .vblank(vvblank)
);

// the LCD look and the LED dot (chg_postfx.sv: status 82-88). All off, the picture
// is chg_video's, two pixel periods late with its timing.
wire [7:0] vr, vg, vb;
wire       vhs, vvs, vhblank, vvblank;
wire [7:0] pr, pg, pb;                  // postfx's picture
wire       phs, pvs, phblank, pvblank, p_hact;
chg_postfx postfx
(
	.clk(clk_sys), .ce_pix(ce_pix), .ce_px(ce_px), .mode(vmode), .voff_in(status[20:16]),
	.grid_str(status[86:84]), .grid_sty(status[88:87]), .lcd_col(status[82]), .led_en(status[83]), .led(led),
	.r_in(vr & {8{have_game}}), .g_in(vg & {8{have_game}}), .b_in(vb & {8{have_game}}), .hs_in(vhs), .vs_in(vvs), .hblank_in(vhblank), .vblank_in(vvblank),
	.p_gx(p_gx), .p_gy(p_gy), .p_act(p_act), .p_yl(p_yl), .p_wl(p_wl),
	.r(pr), .g(pg), .b(pb), .hs(phs), .vs(pvs), .hblank(phblank), .vblank(pvblank), .px_act(p_hact)
);

// the fine modes' (1.75x, 240x240) last stage: each glass pixel for its ticks, and the Soft filter's
// horizontal blend. A wire in the other modes.
chg_hfine hfine
(
	.clk(clk_sys), .ce_pix(ce_pix), .ce_px(ce_px), .mode(vmode), .sfilt(vfilt == 2'd3),
	.r_in(pr), .g_in(pg), .b_in(pb), .act_in(p_hact),
	.hs_in(phs), .vs_in(pvs), .hblank_in(phblank), .vblank_in(pvblank),
	.r(r), .g(g), .b(b), .hs(hs), .vs(vs), .hblank(hblank), .vblank(vblank)
);

// MiSTer's usual video path. A 15 kHz mode goes to an RGB CRT as it is (or
// line doubled, with HQ2x or scanlines, when the scandoubler is on or
// MiSTer.ini forces it for a VGA monitor); the 31 kHz mode (3x) is already
// VGA rate, so the scandoubler and its effects are off there. HDMI goes
// through the scaler, with the aspect ratio and integer scaling from
// video_freak. The picture is square: "Original" is 1:1, or 16:15 in the 2x
// mode, which shows 120 of the 128 rows (240 lines).
wire       vga31 = vmode == 3'd2;
wire       vfine = vmode >= 3'd3;       // the 48 MHz tick modes: lines too long for the scandoubler
wire [2:0] fx = (vga31 || vfine) ? 3'd0 : status[11:9];
wire [2:0] sl = fx ? fx - 1'd1 : 3'd0;
assign VGA_SL = sl[1:0];

wire vga_de;
video_mixer #(.LINE_LENGTH(400), .GAMMA(1)) video_mixer
(
	.CLK_VIDEO(clk_sys),
	.CE_PIXEL(CE_PIXEL),
	.ce_pix(ce_pix),
	.scandoubler(!vga31 && !vfine && (fx != 0 || forced_scandoubler)),
	.hq2x(fx == 1),
	.gamma_bus(gamma_bus),
	.R(r), .G(g), .B(b),
	.HSync(hs), .VSync(vs), .HBlank(hblank), .VBlank(vblank),
	.HDMI_FREEZE(1'b0),
	.freeze_sync(),
	.VGA_R(VGA_R), .VGA_G(VGA_G), .VGA_B(VGA_B),
	.VGA_VS(VGA_VS), .VGA_HS(VGA_HS), .VGA_DE(vga_de)
);

wire [1:0] ar = status[122:121];
video_freak video_freak
(
	.CLK_VIDEO(clk_sys),
	.CE_PIXEL(CE_PIXEL),
	.VGA_VS(VGA_VS),
	.HDMI_WIDTH(HDMI_WIDTH),
	.HDMI_HEIGHT(HDMI_HEIGHT),
	.VGA_DE(VGA_DE),
	.VIDEO_ARX(VIDEO_ARX),
	.VIDEO_ARY(VIDEO_ARY),
	.VGA_DE_IN(vga_de),
	.ARX((!ar) ? ((vmode == 3'd1) ? 12'd16 : 12'd1) : 12'(ar - 1'd1)),
	.ARY((!ar) ? ((vmode == 3'd1) ? 12'd15 : 12'd1) : 12'd0),
	.CROP_SIZE(12'd0),
	.CROP_OFF(5'd0),
	.SCALE(status[14:12])
);

assign CLK_VIDEO = clk_sys;

/////////////////////////  THE BUZZER  ///////////////////////////

wire signed [15:0] sample;
wire               sample_stb;
reg         [11:0] vol;
always @(posedge clk_sys) if (ff | pause) vol <= 12'd0;      // muted while fast-forwarding or paused
else case (status[6:4])
	3'd0: vol <= 12'd512;       // 50%, the emulator's default
	3'd1: vol <= 12'd768;
	3'd2: vol <= 12'd1024;
	3'd3: vol <= 12'd1536;
	3'd4: vol <= 12'd256;
	default: vol <= 12'd0;
endcase

chg_audio audio
(
	.clk(clk_sys),
	.reset(reset),
	.ce(ce),
	.pin(buzzer),
	.piezo(status[29:27] == 3'd1),     // Sound: Piezo filter
	.vol(vol),
	.sample(sample),
	.strobe(sample_stb)
);

// the sound models, the equaliser and the spectrum analyser (the defaults pass the sample through)
wire signed [15:0] sample_fx;
// Spectrum = While in menu: the pause (status[64]) must not stop the game then, or the bars are flat
wire               spec_menu = status[35:34] == 2'd1;
wire        [27:0] spec_bars, spec_peaks;
chg_audio_fx audio_fx
(
	.clk(clk_sys),
	.reset(reset),
	.sample(sample),
	.strobe(sample_stb),
	.model(status[29:27]),
	.preset(status[32:30]),
	.bands(status[63:36]),
	.spec(|status[35:34]),
	.out(sample_fx),
	.bars(spec_bars),
	.peaks(spec_peaks)
);

assign AUDIO_S = 1;
assign AUDIO_L = pause ? 16'sd0 : sample_fx;
assign AUDIO_R = pause ? 16'sd0 : sample_fx;

// the board's LED, and the save's pages not yet written out
assign LED_USER = led | sv_pending;

endmodule
