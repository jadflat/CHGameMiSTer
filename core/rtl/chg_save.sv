// A game's saves: the flash pages it writes, kept in a .sav that MiSTer
// mounts beside the game ("FS" in the OSD string makes the ARM mount
// saves/CHGame/<game>.sav as virtual disk 0).
//
// What is saved is what emu/src/loader.c's chg_write_save saves: every 256-byte
// page below 0xF800 the game has written since it was loaded, with its
// contents. The file is fixed size, 125 blocks of 512 bytes (64,000 bytes):
//   block 0      "CHGS", version 1, page count 248, then at byte 16 a
//                bitmap of the pages saved (bit p of byte p/8); at byte 48
//                0xA5 if the picture position follows (49: V, 5 bits, 50: H,
//                6 bits, as the OSD holds them), else 0
//   blocks 1-124 pages 0-247 (0x0000-0xF7FF), two a block; pages not in
//                the bitmap are whatever the flash held
// tools/savconv.py converts to and from the emulator's .sav.
//
// A shadow copy of the flash (block RAM, 62 KB) follows every write to it
// (loader, restore, the game's flash controller), so a save is written from
// the shadow without touching the board's memories or its time.
// It is written when the OSD asks, and (autosave) once the game has not
// written the flash for a second of board time, as the libretro core does.

module chg_save
(
	input             clk,
	input             reset,
	input             ce,

	// every write to the flash, in the form of CONTRACT.md's port B (byte
	// address, 1/2/4 bytes, value right aligned), and whether it is the
	// game's (a save) or the loader's
	input             fw_we,
	input      [15:0] fw_addr,
	input       [2:0] fw_size,
	input      [31:0] fw_data,
	input             fw_game,

	input             loaded,       // pulse: a game was just loaded (the dirty map starts over, then restore)
	output reg        busy,         // restoring: hold the board in reset
	output reg        fl_we,        // restored pages into the flash
	output reg [15:0] fl_addr,
	output reg  [7:0] fl_data,

	input             save_now,     // pulse, from the OSD
	input             autosave,
	input             reload,       // OSD "Reload save" (level; a rising edge restores the mounted save and restarts the game)

	// hps_io virtual disk 0
	input             img_mounted,
	input      [63:0] img_size,
	input             img_readonly,
	output reg [31:0] sd_lba,
	output reg        sd_rd,
	output reg        sd_wr,
	input             sd_ack,
	input       [8:0] sd_buff_addr,
	input       [7:0] sd_buff_dout,
	output      [7:0] sd_buff_din,
	input             sd_buff_wr,

	output reg        pending,      // pages written that are not saved yet (for the LED)

	// per-game picture position (OSD V and H position), kept in block 0 at bytes 48-50
	input             pos_en,       // OSD "Per-game position" On
	input       [4:0] pos_v,
	input       [5:0] pos_h,
	output reg        pos_set,      // pulse: the restored position is in pos_set_v/h, to go into the OSD's status
	output reg  [4:0] pos_set_v,
	output reg  [5:0] pos_set_h
);

localparam PAGES = 248, BLOCKS = 125;

// ---------------------------------------------------------------------------
// the shadow and the dirty map
// ---------------------------------------------------------------------------

reg  [PAGES-1:0] dirty;
reg  [7:0]       sh_q;
reg  [6:0]       blk;
// block b holds bytes (b-1)*512 + o: the shadow follows hps_io's address with one clock of latency
wire [15:0]      page_byte = {blk - 7'd1, sd_buff_addr};
wire [31:0]      sh_w;
reg  [1:0]       sh_lane;
always @(posedge clk) sh_lane <= page_byte[1:0];
always @* sh_q = sh_w[8*sh_lane +: 8];

// a write as byte enables and lanes (aligned: the C splits misaligned ones into
// bytes), registered on the way in (96 MHz)
reg         sh_we, sh_game;
reg  [15:0] sh_addr;
reg   [3:0] sh_be;
reg  [31:0] sh_d;
always @(posedge clk) begin
	sh_we   <= fw_we && fw_addr < 16'hF800;
	sh_game <= fw_game;
	sh_addr <= fw_addr;
	sh_be   <= fw_size == 3'd4 ? 4'b1111 : fw_size == 3'd2 ? 4'b0011 << fw_addr[1:0] : 4'b0001 << fw_addr[1:0];
	sh_d    <= fw_data << (8 * fw_addr[1:0]);
end

// The shadow: words of 0x0000-0xF7FF, written on port A, read for a save on
// port B. It needs no initial contents: only pages the game writes are
// saved, and the loader sets every page it can write before that.
chg_dpram #(.AW(14), .BYTES(4), .WORDS(16'h3E00)) shadow
(
	.clk(clk),
	.a_addr(sh_addr[15:2]), .a_we(sh_we ? sh_be : 4'd0), .a_d(sh_d), .a_q(),
	.b_addr(page_byte[15:2]), .b_we(4'd0), .b_d(32'd0), .b_q(sh_w)
);

// ---------------------------------------------------------------------------
// the file
// ---------------------------------------------------------------------------

reg        mounted;             // a writable .sav is mounted: saves are written to it
reg        full;                // ... and it is a whole save (64,000 bytes or more): restore from it
reg  [7:0] hdr[32];             // block 0 as read: magic, version, count, bitmap
wire       hdr_ok = {hdr[3], hdr[2], hdr[1], hdr[0]} == 32'h53474843 && hdr[4] == 1 && {hdr[9], hdr[8]} == PAGES;

// A game's first save goes into the new, empty file MiSTer makes for it (size 0),
// which grows as the 125 blocks are written: so any writable file is written to.
// Main_MiSTer mounts the game's .sav during or after the .CHG download, so the
// mount can come before or after `loaded`. `fresh` is a mount that no load has
// used yet: a load takes it (waiting up to half a second for one that has not
// come), and the board is held in reset until the restore has run.
reg        fresh;
reg        reload_d, mnt_d;
reg [25:0] tmo;

// the position as a save in progress writes it (taken when the save starts)
reg        sv_en;
reg  [4:0] sv_v;
reg  [5:0] sv_h;
reg  [7:0] r_mk;                // the position bytes as the restore read them
reg  [4:0] r_v;
reg  [5:0] r_h;
reg  [4:0] last_v;              // the position the OSD held last, to see it change
reg  [5:0] last_h;
reg        waiting;             // a restored position was sent to the OSD and has not come back yet
reg [25:0] wait_t;

// a byte of block b at offset o, as the save holds it
function [7:0] hdr_byte(input [8:0] o);
	case (o)
		9'd0: hdr_byte = "C"; 9'd1: hdr_byte = "H"; 9'd2: hdr_byte = "G"; 9'd3: hdr_byte = "S";
		9'd4: hdr_byte = 8'd1;
		9'd8: hdr_byte = 8'(PAGES); 9'd9: hdr_byte = 8'(PAGES >> 8);
		default: hdr_byte = 0;
	endcase
endfunction

// what goes out for a byte of a save: the header and bitmap, or the shadow
wire [7:0] pos_byte = sd_buff_addr == 9'd48 ? (sv_en ? 8'hA5 : 8'd0) : sd_buff_addr == 9'd49 ? {3'd0, sv_v} : sd_buff_addr == 9'd50 ? {2'd0, sv_h} : 8'd0;
assign sd_buff_din = (blk == 0) ? (sd_buff_addr >= 9'd48 && sd_buff_addr <= 9'd50 ? pos_byte : sd_buff_addr >= 9'd16 && sd_buff_addr < 9'd16 + 9'd31 ?
                                   8'(dirty >> (8 * (sd_buff_addr - 9'd16))) : hdr_byte(sd_buff_addr))
                                : sh_q;

typedef enum logic [2:0] { IDLE, RD_REQ, RD_WAIT, WR_REQ, WR_WAIT, WAIT_M } state_t;
state_t     st;
reg         writing;            // the transfer in progress is a save
reg         ack_d;
reg  [25:0] quiet;              // board cycles since the game last wrote the flash
reg         want_save;

wire        reload_p = reload && !reload_d;
wire        page_dirty_rd = dirty[page_byte[15:8]];

always @(posedge clk) begin
	fl_we <= 0;
	ack_d <= sd_ack;
	reload_d <= reload;
	mnt_d <= img_mounted;

	// the game's own writes: the page is a save now
	if (sh_we && sh_game) begin
		dirty[sh_addr[15:8]] <= 1;     // a write never crosses a page: it is aligned
		pending <= 1;
		quiet <= 0;
	end
	else if (ce && quiet != 26'h3FFFFFF) quiet <= quiet + 1'd1;

	// the OSD's position changed while the game runs: the next save is due
	pos_set <= 0;
	if (waiting) begin
		wait_t <= wait_t + 1'd1;
		if ((pos_v == pos_set_v && pos_h == pos_set_h) || &wait_t) begin
			waiting <= 0; last_v <= pos_v; last_h <= pos_h;
		end
	end
	else if (pos_v != last_v || pos_h != last_h) begin
		last_v <= pos_v; last_h <= pos_h;
		if (pos_en) pending <= 1;
	end

	if (save_now || (autosave && pending && quiet == 26'd48_000_000)) want_save <= 1;

	if (reset) begin
		waiting <= 0; last_v <= pos_v; last_h <= pos_h;
		st <= IDLE; busy <= 0; pending <= 0; dirty <= 0; want_save <= 0; sd_rd <= 0; sd_wr <= 0;
	end
	else case (st)
		IDLE:
			if (loaded || (reload_p && mounted && full)) begin
				dirty <= 0; pending <= 0; want_save <= 0; waiting <= 0; last_v <= pos_v; last_h <= pos_h;
				if (!loaded || fresh) begin
					fresh <= 0;
					if (mounted && full) begin busy <= 1; blk <= 0; writing <= 0; st <= RD_REQ; end
				end
				else begin busy <= 1; tmo <= 0; st <= WAIT_M; end
			end
			else if (want_save && mounted) begin
				sv_en <= pos_en; sv_v <= pos_v; sv_h <= pos_h;
				want_save <= 0; pending <= 0; blk <= 0; writing <= 1; st <= WR_REQ;
			end
		WAIT_M: begin
			tmo <= tmo + 1'd1;
			if (fresh) begin
				fresh <= 0;
				if (mounted && full) begin blk <= 0; writing <= 0; st <= RD_REQ; end
				else begin busy <= 0; st <= IDLE; end
			end
			else if (tmo == 26'd48_000_000) begin busy <= 0; st <= IDLE; end
		end
		RD_REQ: begin sd_lba <= 32'(blk); sd_rd <= 1; st <= RD_WAIT; end
		RD_WAIT: begin
			if (sd_ack) sd_rd <= 0;
			if (sd_buff_wr && sd_ack) begin
				if (blk == 0) begin
					if (sd_buff_addr < 9'd32) hdr[sd_buff_addr[4:0]] <= sd_buff_dout;
					if (sd_buff_addr == 9'd48) r_mk <= sd_buff_dout;
					if (sd_buff_addr == 9'd49) r_v <= sd_buff_dout[4:0];
					if (sd_buff_addr == 9'd50) r_h <= sd_buff_dout[5:0];
					if (sd_buff_addr >= 9'd16 && sd_buff_addr < 9'd16 + 9'd31)
						dirty[8 * (sd_buff_addr - 9'd16) +: 8] <= sd_buff_dout;
				end
				else if (page_dirty_rd) begin
					fl_we <= 1; fl_addr <= page_byte; fl_data <= sd_buff_dout;
				end
			end
			if (ack_d && !sd_ack) begin
				// block 0 not a CHGame save (a new, empty file): nothing to restore
				if (blk == 0 && !hdr_ok) begin dirty <= 0; busy <= 0; st <= IDLE; end
				else if (blk == BLOCKS - 1) begin
					busy <= 0; st <= IDLE;
					if (pos_en && r_mk == 8'hA5) begin
						pos_set <= 1; pos_set_v <= r_v; pos_set_h <= r_h; waiting <= 1; wait_t <= 0;
					end
				end
				else begin blk <= blk + 1'd1; st <= RD_REQ; end
			end
		end
		WR_REQ: begin sd_lba <= 32'(blk); sd_wr <= 1; st <= WR_WAIT; end
		WR_WAIT: begin
			if (sd_ack) sd_wr <= 0;
			if (ack_d && !sd_ack) begin
				if (blk == BLOCKS - 1) st <= IDLE;
				else begin blk <= blk + 1'd1; st <= WR_REQ; end
			end
		end
		default: st <= IDLE;
	endcase

	if (reset) begin mounted <= 0; full <= 0; fresh <= 0; end
	else if (img_mounted && !mnt_d) begin
		mounted <= !img_readonly;
		full    <= img_size >= 64'(BLOCKS * 512);
		fresh   <= 1;
	end
end

endmodule
