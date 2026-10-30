// The MiSTer side of the core for sim/tb_mister.cpp: sys_top's HPS I/O glue
// (gp_out's two registers, io_strobe, io_ack and the ioctl_wait hold, as in
// core/sys/sys_top.v), the real hps_io (core/sys/hps_io.sv) with
// CHGame.sv's CONF_STR and VDNUM, and CHGame.sv's loader, save, card and
// board wired as CHGame.sv wires them (keep the two the same). The testbench
// is the ARM (Main_MiSTer: gp_out, gp_in) and the DDR3.
// The video and the sound are left out (the throttle is chg_throttle.sv, used when thr_sel is 1): the testbench reads the
// glass through vx/vy and gives run itself.
//


module mister_top
(
	input             clk,
	input             RESET,
	input             run,
	input             thr_sel,          // the testbench: board time from the real throttle (chg_throttle.sv) instead of run
	input             ff,               // fast forward, for the throttle
	input       [2:0] rate,             // the OSD's Board clock, for the throttle
	output            ce,
	output     [11:0] speed_bcd,
	input       [7:0] buttons_in,       // the board's, CHG_BTN_* order
	input             force_hold,       // the testbench: hold_mode 1 for the image too (lockstep with the C)
	input             OSD_STATUS,       // the OSD is open (the testbench), for Pause when OSD open
	output            paused,           // the board is held by it

	// the ARM's side of h2f_gp
	input      [15:0] gp_din,
	input             gp_clk,           // io_clk (gp_out[17])
	input             gp_ss0,           // gp_out[18]
	input             gp_ss1,           // gp_out[19]
	input             gp_ss2,           // gp_out[20]
	output     [15:0] gp_dout,
	output reg        gp_ack,

	// DDR3
	input             DDRAM_BUSY,
	output      [7:0] DDRAM_BURSTCNT,
	output     [28:0] DDRAM_ADDR,
	input      [63:0] DDRAM_DOUT,
	input             DDRAM_DOUT_READY,
	output            DDRAM_RD,
	output     [63:0] DDRAM_DIN,
	output      [7:0] DDRAM_BE,
	output            DDRAM_WE,

	// for the testbench
	output            board_reset,
	output            dbg_boundary,
	output     [63:0] retire_instret,
	output   [1023:0] dbg_x,
	input       [6:0] vx,
	input       [6:0] vy,
	input             vline,
	output     [23:0] vrgb,
	output      [1:0] sd_ack_o,
	output            sd_rd1,
	output            card_sel_dat,
	output     [127:0] status_o,
	output            sv_pending_o,     // the testbench: pages written, not saved yet
	input      [13:0] dbg_fa,           // the testbench: a word of the board's flash
	output     [31:0] dbg_fq
);

// ---- sys_top.v's HPS I/O ----
reg [31:0] gp_outr, gp_outd;
wire [31:0] gp_out = {11'd0, gp_ss2, gp_ss1, gp_ss0, gp_clk, 1'b0, gp_din};
always @(posedge clk) begin gp_outr <= gp_outd; gp_outd <= gp_out; end
wire [15:0] io_din = gp_outr[15:0];
wire        io_clk = gp_outr[17];
wire        io_ss0 = gp_outr[18];
wire        io_ss1 = gp_outr[19];
wire        io_ss2 = gp_outr[20];
wire        io_fpga = ~io_ss1 & io_ss0;
wire        io_uio  = ~io_ss1 & io_ss2;
reg         rack;
wire        io_strobe = ~rack & io_clk;
wire        io_wait;
initial begin rack = 0; gp_ack = 0; end
always @(posedge clk) begin
	if (~io_wait | io_strobe) begin
		rack <= io_clk;
		gp_ack <= rack;
	end
end

wire [45:0] HPS_BUS;
wire [35:0] EXT_BUS;                // nothing on it: undriven in CHGame.sv, which Quartus ties low
assign EXT_BUS[32] = 0;
assign EXT_BUS[15:0] = 0;
assign HPS_BUS[45:38] = 0;
assign HPS_BUS[35] = io_fpga;
assign HPS_BUS[34] = io_uio;
assign HPS_BUS[33] = io_strobe;
assign HPS_BUS[31:16] = io_din;
assign io_wait = HPS_BUS[37];
assign gp_dout = HPS_BUS[15:0];

// ---- CHGame.sv ----
localparam CONF_STR = {
	"CHGame;;",
	"-;",
	"FS1,CHG,Load game;",
	"F2,DAT,Load data file,38000000;",
	"R[65],Clear data files;",
	"S1,IMG,Mount microSD image;",
	"-;",
	"O[92:91],Fast card loading,Auto,Always on,Off,Off;",
	"O[64],Pause when OSD open,Off,On;",
	"O[3],Sound,Bare pin,Piezo filter;",
	"O[7],Autosave,On,Off;",
	"R[8],Save now;",
	"T[0],Reset;",
	"V,v0.4"
};

wire  [1:0] buttons;
wire [127:0] status;
wire         pos_set;                  // chg_save: a game's stored picture position goes into the OSD
wire   [4:0] pos_set_v;
wire   [5:0] pos_set_h;
wire [127:0] status_in = {status[127:27], pos_set_h, pos_set_v, status[15:0]};
wire         ioctl_download;
wire  [15:0] ioctl_index;
wire         ioctl_wr;
wire  [26:0] ioctl_addr;
wire   [7:0] ioctl_dout;
wire         ioctl_wait;
wire   [1:0] img_mounted;
wire         img_readonly;
wire  [63:0] img_size;
wire  [31:0] sd_lba[2];
wire   [1:0] sd_rd, sd_wr, sd_ack;
wire  [13:0] sd_buff_addr;
wire   [7:0] sd_buff_dout;
wire   [7:0] sd_buff_din[2];
wire         sd_buff_wr;
wire  [21:0] gamma_bus;
wire  [5:0]  sd_blk_cnt[2];
assign sd_blk_cnt[0] = 0;
assign sd_blk_cnt[1] = 0;

hps_io #(.CONF_STR(CONF_STR), .VDNUM(2)) hps_io
(
	.clk_sys(clk),
	.HPS_BUS(HPS_BUS),
	.EXT_BUS(EXT_BUS),
	.gamma_bus(gamma_bus),
	.buttons(buttons),
	.status(status),
	.status_in(status_in),
	.status_set(pos_set),
	.status_menumask(16'd0),
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
	.sd_blk_cnt(sd_blk_cnt),
	.sd_rd(sd_rd),
	.sd_wr(sd_wr),
	.sd_ack(sd_ack),
	.sd_buff_addr(sd_buff_addr),
	.sd_buff_dout(sd_buff_dout),
	.sd_buff_din(sd_buff_din),
	.sd_buff_wr(sd_buff_wr)
);

wire        ld_we, ld_busy, ld_bad, ld_wait, card_wait;
wire [15:0] ld_addr;
wire  [7:0] ld_data;
assign ioctl_wait = ld_wait | card_wait;

chg_loader loader
(
	.clk(clk), .reset(RESET),
	.download_in(ioctl_download && ioctl_index[5:0] == 6'd1),
	.wr_in(ioctl_wr), .addr_in(ioctl_addr), .din_in(ioctl_dout), .wait_req(ld_wait),
	.fl_we(ld_we), .fl_addr(ld_addr), .fl_data(ld_data), .busy(ld_busy), .bad(ld_bad)
);

reg ld_busy_d;
always @(posedge clk) ld_busy_d <= ld_busy;
wire loaded = ld_busy_d && !ld_busy;

wire        sv_we, sv_busy, sv_pending;
wire [15:0] sv_addr;
wire  [7:0] sv_data;
wire        gfw_we;
wire [15:0] gfw_addr;
wire  [2:0] gfw_size;
wire [31:0] gfw_data;
wire        fl_we   = ld_we | sv_we;
wire [15:0] fl_addr = ld_we ? ld_addr : sv_addr;
wire  [7:0] fl_data = ld_we ? ld_data : sv_data;

chg_save save
(
	.clk(clk), .reset(RESET), .ce(ce),
	.fw_we(fl_we | gfw_we), .fw_addr(fl_we ? fl_addr : gfw_addr), .fw_size(fl_we ? 3'd1 : gfw_size),
	.fw_data(fl_we ? {24'd0, fl_data} : gfw_data), .fw_game(!fl_we),
	.loaded(loaded), .busy(sv_busy), .fl_we(sv_we), .fl_addr(sv_addr), .fl_data(sv_data),
	.save_now(status[8]), .autosave(!status[7]), .reload(status[90]),
	.img_mounted(img_mounted[0]), .img_size(img_size), .img_readonly(img_readonly),
	.sd_lba(sd_lba[0]), .sd_rd(sd_rd[0]), .sd_wr(sd_wr[0]), .sd_ack(sd_ack[0]),
	.sd_buff_addr(sd_buff_addr[8:0]), .sd_buff_dout(sd_buff_dout), .sd_buff_din(sd_buff_din[0]),
	.sd_buff_wr(sd_buff_wr), .pending(sv_pending),
	.pos_en(!status[89]), .pos_v(status[20:16]), .pos_h(status[26:21]),
	.pos_set(pos_set), .pos_set_v(pos_set_v), .pos_set_h(pos_set_h)
);

wire        card_busy, card_hold;
wire        c_mounted;
wire [63:0] c_size;
wire [31:0] c_lba;
wire        c_rd, c_wr, c_ack, c_buff_wr;
wire  [8:0] c_buff_addr;
wire  [7:0] c_buff_dout, c_buff_din;

chg_card card
(
	.clk(clk),
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

wire reset = RESET | status[0] | buttons[1] | ld_busy | sv_busy | card_busy;
assign board_reset = reset;
assign sd_ack_o = sd_ack;
assign sd_rd1 = sd_rd[1];
assign card_sel_dat = card.sel;
assign sv_pending_o = sv_pending;
assign status_o = status;
assign dbg_fq = soc.flash[dbg_fa];

wire thr_run;
// Pause when OSD open, as in CHGame.sv
reg  [1:0] osd_sync = 2'b00;
always @(posedge clk) osd_sync <= {osd_sync[0], OSD_STATUS};
wire       pause_block = 1'b0;
wire       pause = status[64] & osd_sync[1] & ~pause_block;
assign paused = pause;
wire card_sel;
chg_throttle throttle(.clk(clk), .ff(ff | card_sel), .rate(rate), .pause(pause), .ce(ce), .run(thr_run));
chg_speed speed(.clk(clk), .ce(ce), .bcd(speed_bcd));

chg_soc soc
(
	.clk(clk), .reset(reset), .run(thr_sel ? thr_run : run), .ce(ce), .fast_card(status[92:91] < 2'd2), .fast_always(status[92:91] == 2'd1), .card_sel(card_sel), .hold_mode(card_hold | force_hold),
	.fl_we(fl_we), .fl_addr(fl_addr), .fl_data(fl_data),
	.gfw_we(gfw_we), .gfw_addr(gfw_addr), .gfw_size(gfw_size), .gfw_data(gfw_data),
	.buttons(buttons_in),
	.vx(vx), .vy(vy), .vline(vline), .vrgb(vrgb),
	.buzzer(), .led(),
	.sd_mounted(c_mounted), .sd_size(c_size), .sd_lba(c_lba), .sd_rd(c_rd), .sd_wr(c_wr), .sd_ack(c_ack),
	.sd_buff_addr(c_buff_addr), .sd_buff_dout(c_buff_dout), .sd_buff_din(c_buff_din), .sd_buff_wr(c_buff_wr),
	.dbg_boundary(dbg_boundary), .retire(), .retire_pc(), .retire_instret(retire_instret), .retire_cycles(),
	.dbg_x(dbg_x), .faulted()
);

endmodule
