// The card in the board's microSD slot (chg_soc's sd_* port, served by
// chg_sdspi): one of two, the one inserted last.
//   - the .DAT files loaded from the OSD ("Load data file", ioctl index 2): kept
//     in DDR3, served as a 128 MB FAT32 card holding those files (up to eight,
//     each load adds one; chg_datcard). Loading one powers the board off until
//     it is in, and the game then starts again, as a card put in a real CHGame
//     would need. "Clear data files" (clear_files, a pulse or a rising edge)
//     drops them all, takes the card out and resets the board the same way.
//   - a card image mounted from the OSD ("Mount microSD image", hps_io's
//     virtual disk 1), served by the ARM block by block.
// Ejecting the image (an empty mount) falls back to the .DAT if one is held.
// The .DAT files stay loaded across game loads and resets until cleared (or the
// core is loaded again).
//
// The .DAT's card holds board time while a block comes (see hold below).
//
// Every signal between the two sources and chg_sdspi goes through one
// register (96 MHz), the buffer's address, data, strobe and ack together, so
// the order of things is as each source makes it.

module chg_card
(
	input             clk,
	input             clear_files,      // OSD "Clear data files" (status bit, rising edge)

	// the OSD's file loads
	input             ioctl_download,
	input      [15:0] ioctl_index,
	input             ioctl_wr,
	input      [26:0] ioctl_addr,
	input       [7:0] ioctl_dout,
	output            ioctl_wait,
	output            busy,             // a .DAT is loading: hold the board off
	output            hold,             // chg_soc's hold_mode: the .DAT's card holds board time for its blocks

	// hps_io's virtual disk 1
	input             img_mounted,
	input      [63:0] img_size,
	output reg [31:0] hps_lba,
	output reg        hps_rd,
	output reg        hps_wr,
	input             hps_ack,
	input       [8:0] hps_buff_addr,
	input       [7:0] hps_buff_dout,
	output reg  [7:0] hps_buff_din,
	input             hps_buff_wr,

	// the card in the slot, for chg_soc
	output reg        sd_mounted,
	output reg [63:0] sd_size,
	input      [31:0] sd_lba,
	input             sd_rd,
	input             sd_wr,
	output reg        sd_ack,
	output reg  [8:0] sd_buff_addr,
	output reg  [7:0] sd_buff_dout,
	input       [7:0] sd_buff_din,
	output reg        sd_buff_wr,

	// DDR3 (DDRAM_*, on clk)
	input             ddr_busy,
	output      [7:0] ddr_burstcnt,
	output     [28:0] ddr_addr,
	input      [63:0] ddr_dout,
	input             ddr_dout_ready,
	output            ddr_rd,
	output     [63:0] ddr_din,
	output      [7:0] ddr_be,
	output            ddr_we
);

localparam [63:0] DAT_CARD = 64'd134217728;        // chg_datcard's card: 128 MB

// the .DAT's download, registered once on its way in
reg        dl, dl_wr;
reg [26:0] dl_addr;
reg  [7:0] dl_data;
always @(posedge clk) begin
	dl <= ioctl_download && ioctl_index[5:0] == 6'd2;
	dl_wr <= ioctl_wr;
	dl_addr <= ioctl_addr;
	dl_data <= ioctl_dout;
end

wire        d_ack, d_wr, d_wait, d_loaded, d_valid, d_loading, d_cleared;
reg         clr_d, clr_r;
reg   [7:0] cbusy;
wire        clr_p = clear_files && !clr_d;
wire  [8:0] d_addr;
wire  [7:0] d_dout;
reg  [31:0] c_lba;
reg         c_rd, c_wr;
reg   [7:0] c_din;

chg_datcard dat
(
	.clk(clk),
	.clr(clr_r), .cleared(d_cleared),
	.dl(dl), .dl_wr(dl_wr), .dl_addr(dl_addr), .dl_data(dl_data), .dl_wait(d_wait),
	.loaded(d_loaded), .valid(d_valid), .loading(d_loading),
	.sd_lba(c_lba), .sd_rd(c_rd), .sd_wr(c_wr), .sd_ack(d_ack),
	.sd_buff_addr(d_addr), .sd_buff_dout(d_dout), .sd_buff_din(c_din), .sd_buff_wr(d_wr),
	.ddr_busy(ddr_busy), .ddr_burstcnt(ddr_burstcnt), .ddr_addr(ddr_addr), .ddr_dout(ddr_dout),
	.ddr_dout_ready(ddr_dout_ready), .ddr_rd(ddr_rd), .ddr_din(ddr_din), .ddr_be(ddr_be), .ddr_we(ddr_we)
);

// the wait goes straight to hps_io (as chg_loader's): the ARM holds its next word
assign ioctl_wait = d_wait;
assign busy = dl || d_loading || cbusy != 0;
// The .DAT's card holds board time while a block comes from DDR3 (hold_mode 1:
// the game sees the emulator's card, which never makes it wait). The image
// keeps the original way (hold_mode 0: the card answers "not yet" while the ARM
// works, as a real card does), which runs well on the MiSTer.
assign hold = sel;

// which card is in: 1 the .DAT, 0 the image (or none)
reg sel;
initial begin sel = 0; sd_mounted = 0; sd_size = 0; clr_d = 0; clr_r = 0; cbusy = 0; end
reg dl_d;
always @(posedge clk) begin
	sd_mounted <= 0;
	dl_d <= dl;
	// Clear data files: the board off for a moment (while the card is in use), the files dropped, the card out
	clr_d <= clear_files;
	clr_r <= clr_p && !dl && !d_loading;
	if (clr_p && sel && !dl && !d_loading) cbusy <= 8'd255;
	else if (cbusy != 0) cbusy <= cbusy - 1'd1;
	if (img_mounted) begin
		if (img_size != 0 || !d_valid) begin sel <= 0; sd_size <= img_size; end
		else begin sel <= 1; sd_size <= DAT_CARD; end
		sd_mounted <= 1;
	end
	else if (dl && !dl_d && sel) begin
		// the .DAT's card comes out while the next goes in
		sd_size <= 0;
		sd_mounted <= 1;
	end
	else if (d_loaded) begin
		sel <= 1;
		sd_size <= DAT_CARD;
		sd_mounted <= 1;
	end
	else if (d_cleared && sel) begin
		sd_size <= 0;
		sd_mounted <= 1;
	end
end

// the block traffic, one register each way
always @(posedge clk) begin
	hps_lba <= sd_lba;
	c_lba <= sd_lba;
	hps_rd <= sd_rd && !sel;
	hps_wr <= sd_wr && !sel;
	c_rd <= sd_rd && sel;
	c_wr <= sd_wr && sel;
	sd_ack       <= sel ? d_ack  : hps_ack;
	sd_buff_addr <= sel ? d_addr : hps_buff_addr;
	sd_buff_dout <= sel ? d_dout : hps_buff_dout;
	sd_buff_wr   <= sel ? d_wr   : hps_buff_wr;
	hps_buff_din <= sd_buff_din;
	c_din <= sd_buff_din;
end

endmodule
