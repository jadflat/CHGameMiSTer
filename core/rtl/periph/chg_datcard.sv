// A microSD card made in logic around up to eight .DAT files kept in the HPS's DDR3.
//
// The OSD's "Load data file" ("F2,DAT,Load data file,38000000" in CONF_STR) makes Main_MiSTer
// copy the .DAT straight into DDR3 at 0x3800_0000 (the staging area) through shared memory,
// at the SD card's speed. hps_io shows only ioctl_download (high for the whole copy) with
// ioctl_addr = the file's size at its start, and no ioctl_wr. When the download ends the
// core reads the file's first word from the staging area (its name), then moves the file with
// 8-word bursts to its place on the card. Each file is ADDED to
// the card: it is kept in DDR3 from 0x3000_0000 on, after the files already held, each
// starting on a 512-byte boundary (a cluster: the card has one sector a cluster), up to 64 MB
// in all. The card then answers block reads as the emulator's folder card holding exactly
// those files would (emu/src/sdcard.c: sdcard_open on a folder, card_size_for, format,
// import_dir), byte for byte: a 128 MB card (every folder of files up to 76 MB gets that
// size), MBR, a FAT32 partition from sector 2048 with one sector a cluster, boot sector and
// FSInfo (and their backups at +6, +7), two FATs of 2032 sectors, the root directory from
// cluster 2 (sector 6144) holding the volume label CHGAME and the files' 8.3 entries in
// strcmp order of their names (import_dir sorts the folder with qsort/strcmp), and the files
// from the first cluster after the root in that same order, each in one run (alloc_chain
// hands clusters out in order). The root is one cluster, two with eight files (sdcard.c sizes
// it at 2 slots a file, 32 bytes each, plus the label and an end mark: 8 files need 1056
// bytes). Only the files' sectors come from DDR3; the rest is worked out per byte. Bytes past
// a file's end and every other sector are 0.
//
// The files' names. hps_io gives the core only the file's contents and its extension, never
// its name, so the name comes from the file itself: CHGame .DAT files begin with their name
// (STARLIFT, DANKTOMB, ANOTHERW; RASTAN padded with zeros). The name is the first bytes up
// to 8 that may stand in an 8.3 name (letters made capitals); the extension is DAT. Ethan's
// Critters' card format begins with its magic "ECRTCARD" instead; that file is CRITTERS.DAT.
// A file with no name at its start is CARD.DAT.
//
// Adding a file whose name is already on the card REPLACES that file (the card then holds the
// new contents). If the old one was the last stored in DDR3 the new one goes into its place
// (so loading the same big file again always fits); otherwise it goes after the others and
// the old one's space is lost until "Clear data files". A file that does not fit (a ninth
// name, or no DDR3 left) is ignored; one that overflows the 64 MB is cut at the end of
// DDR3, as before. clr (Clear data files) drops all files.
//
// The table: the files in card order (sorted by name), each with its DDR3 sector, length
// and its first/last cluster on the card, worked out after every load (a clock a step, it
// is not in a hurry). A block read looks the sector up in it, a file at a time.
//
// The card side is hps_io's virtual disk protocol (sd_lba, sd_rd, sd_wr, sd_ack,
// sd_buff_*), so chg_sdspi serves it as it serves a mounted image. Writes go to DDR3 when
// they hit a file's sectors (kept until the file is replaced or cleared or until the core is
// reloaded, never written back to the file) and are dropped elsewhere; CHGame's SdSpi.cpp
// has no write command, so no game writes.
//
// DDR3: the template's DDRAM_* port on clk (CHGame.sv runs DDRAM_CLK on clk_sys, so there is
// no clock crossing): one 64-bit word an access.
//
// With one file loaded the card is the one sdcard.c makes, sector for
// sector.

module chg_datcard
(
	input             clk,

	// the .DAT's direct load (the caller has matched the OSD index)
	input             dl,           // downloading (Main copies the file into DDR3 meanwhile)
	input             dl_wr,        // unused: a direct load has no writes
	input      [26:0] dl_addr,      // at the download's start: the file's size
	input       [7:0] dl_data,      // unused
	output            dl_wait,      // always 0
	input             clr,          // pulse: drop all files ("Clear data files")
	output reg        cleared,      // pulse: they are dropped
	output reg        loaded,       // pulse: the file is in, the card is ready
	output reg        valid,        // a file is held
	output            loading,      // from the download's start until the card is ready

	// the card, as a virtual disk
	input      [31:0] sd_lba,
	input             sd_rd,
	input             sd_wr,
	output reg        sd_ack,
	output reg  [8:0] sd_buff_addr,
	output reg  [7:0] sd_buff_dout,
	input       [7:0] sd_buff_din,
	output reg        sd_buff_wr,

	// DDR3
	input             ddr_busy,
	output      [7:0] ddr_burstcnt,
	output     [28:0] ddr_addr,
	input      [63:0] ddr_dout,
	input             ddr_dout_ready,
	output reg        ddr_rd,
	output reg [63:0] ddr_din,
	output reg  [7:0] ddr_be,
	output reg        ddr_we
);

localparam [26:0] MAXLEN = 27'h400_0000;            // 64 MB
localparam [17:0] MAXSEC = 18'd131072;              // in sectors
localparam [31:0] BOOT = 2048, FSI = 2049, BOOT_B = 2054, FSI_B = 2055;
localparam [31:0] FAT1 = 2080, FAT2 = 4112, ROOT = 6144;   // the root is cluster 2: cluster c is sector 6142 + c
localparam [10:0] FATSZ = 11'd2032;

reg [22:0] daddr;                                   // the 64-bit word
assign ddr_addr = {4'b0011, stg, 1'b0, daddr};        // 0x3000_0000 on; the staging area 0x3800_0000 on
reg  [7:0] bc;
assign ddr_burstcnt = bc;
reg        stg;                                     // the address is in the staging area (0x3800_0000 on)

initial begin bc = 8'd1; stg = 0; valid = 0; loaded = 0; cleared = 0; sd_ack = 0; ddr_rd = 0; ddr_we = 0; sd_buff_wr = 0; end

// ---------------------------------------------------------------------------
// the files, in card order (sorted by name)
// ---------------------------------------------------------------------------

reg [63:0] t_name [0:7];                            // 8 characters, byte 0 first, padded with spaces
reg [26:0] t_len  [0:7];
reg [16:0] t_off  [0:7];                            // the file's first sector in DDR3 (from 0x3000_0000)
reg [18:0] t_cs   [0:7];                            // its first cluster on the card
reg [18:0] t_ce   [0:7];                            // one past its last cluster (0 for an unused entry)
reg  [3:0] cnt;
reg        rc2;                                     // the root directory takes two clusters (8 files)
reg [18:0] tot;                                     // one past the last file's last cluster
reg [17:0] dend;                                    // the next free sector in DDR3
initial begin
	cnt = 0; rc2 = 0; tot = 19'd3; dend = 0;
	for (int k = 0; k < 8; k++) begin t_cs[k] = 0; t_ce[k] = 0; end
end

// ---------------------------------------------------------------------------
// loading
// ---------------------------------------------------------------------------

reg        dl_d, fin;
reg [26:0] sz;                                      // the download's size (ioctl_addr at its start)
reg [63:0] hdr;
reg [26:0] lim;                                     // bytes of the download that fit in DDR3
reg [17:0] base;                                    // where it starts in DDR3, in sectors
reg        mdone, mgo, reject, reuse, mfound;
reg  [2:0] mk;
reg [63:0] nm;                                      // the download's name
reg [63:0] ta;
reg [16:0] o_k;
reg [18:0] n_k;
reg  [3:0] ij;
reg        we_card;                                 // the write on DDR3 is the card's
reg        clr_q;
// the mover: the header read, then the file from the staging area to its place
reg        hrd, xstart;
reg  [2:0] xst;
reg [22:0] sw, dw;                                  // source and destination, in words
reg [20:0] nb;                                      // bursts left
reg [63:0] sb [0:7];                                // the burst, a shift register
reg  [3:0] bk;
localparam X_IDLE = 3'd0, X_HWT = 3'd1, X_RD = 3'd2, X_RWT = 3'd3, X_W0 = 3'd4, X_WR = 3'd5, X_NEXT = 3'd6;

assign dl_wait = 1'b0;
localparam M_IDLE = 4'd0, M_NM = 4'd1, M_L = 4'd2, M_C = 4'd3, M_E = 4'd4, M_F = 4'd5, M_G = 4'd6, M_H = 4'd7;
localparam I_IDLE = 4'd0, I_S1 = 4'd1, I_S2 = 4'd2, I_INS = 4'd3, I_CS0 = 4'd4, I_CS1 = 4'd5, I_CS2 = 4'd6,
           I_CS3 = 4'd7, I_FIN = 4'd8, I_END = 4'd9, I_LEN = 4'd10, I_CP0 = 4'd11, I_CP1 = 4'd12;
reg  [3:0] mst, ist;
initial begin mst = M_IDLE; ist = I_IDLE; end
assign loading = dl || dl_d || fin || hrd || xstart || xst != X_IDLE || mst != M_IDLE || ist != I_IDLE;

function [63:0] put_byte(input [63:0] w, input [2:0] lane, input [7:0] b);
	put_byte = w;
	put_byte[8 * lane +: 8] = b;
endfunction

// a character an 8.3 name may hold (sdcard.c's sfn_char), as a capital; 0 if none
function [7:0] sfn(input [7:0] c);
	if (c >= "a" && c <= "z") sfn = c - 8'd32;
	else if ((c >= "A" && c <= "Z") || (c >= "0" && c <= "9")) sfn = c;
	else case (c)
		"$", "%", "'", "-", "_", "@", "~", "`", "!", "(", ")", "{", "}", "^", "#", "&": sfn = c;
		default: sfn = 8'd0;
	endcase
endfunction

function [63:0] name_of(input [63:0] h);
	reg go;
	reg [7:0] c;
	name_of = {8{8'h20}};
	go = 1;
	for (int i = 0; i < 8; i++) begin
		c = sfn(h[8 * i +: 8]);
		if (c == 0) go = 0;
		if (go) name_of[8 * i +: 8] = c;
	end
	if (h == "DRACTRCE") name_of = "SRETTIRC";       // "ECRTCARD" byte 0 first: CRITTERS
	else if (sfn(h[7:0]) == 0) name_of = "    DRAC";  // CARD
endfunction

// strcmp of "A.DAT" and "B.DAT" (sdcard.c sorts the folder's names so): {equal, a < b}. The name's end (the
// padding space) stands for the '.', which sorts before letters and digits but after some of the other characters.
function [1:0] cmpn(input [63:0] a, input [63:0] b);
	reg dec;
	reg [7:0] ca, cb;
	cmpn = 2'b10;
	dec = 0;
	for (int k = 0; k < 8; k++) begin
		ca = a[8 * k +: 8];
		cb = b[8 * k +: 8];
		if (ca == 8'h20) ca = 8'h2E;
		if (cb == 8'h20) cb = 8'h2E;
		if (!dec && ca != cb) begin dec = 1; cmpn = {1'b0, ca < cb}; end
	end
endfunction

// the card's state (its always block is below)
localparam C_IDLE = 4'd0, C_CL1 = 4'd1, C_CL2 = 4'd2, C_CL3 = 4'd3, C_GEN = 4'd4, C_RD = 4'd5,
           C_RDW = 4'd6, C_EMIT = 4'd7, C_WRB = 4'd8, C_WRD = 4'd9, C_WRW = 4'd10, C_END = 4'd11,
           C_FND = 4'd12, C_CL4 = 4'd13;
localparam K_ZERO = 3'd0, K_MBR = 3'd1, K_BOOT = 3'd2, K_FSI = 3'd3, K_FAT = 3'd4, K_ROOT = 3'd5, K_DATA = 3'd6;

reg  [3:0] cst;
initial cst = C_IDLE;

reg [26:0] fin_len;
reg [17:0] nsec_n;                                  // the new file's sectors
reg  [3:0] p;                                       // its place in the table
reg        eqf;
reg  [2:0] pe;
reg [26:0] lj;
reg [17:0] nj;
reg [18:0] cum;
reg        cwe_set, crd_set;
reg [22:0] cw_addr, cr_addr;
reg [63:0] cw_data;
wire [1:0] cres = cmpn(ta, nm);                     // the table entry against the new name: {equal, entry < new}

always @(posedge clk) begin
	loaded <= 0;
	cleared <= 0;
	dl_d <= dl;
	if (clr) clr_q <= 1;
	if (mst == M_IDLE && mgo) mgo <= 0;
	if (dl && !dl_d) begin
		valid <= 0; sz <= dl_addr; hdr <= 0; fin <= 0;
		mdone <= 0; mgo <= 0; reject <= 0; lim <= MAXLEN;
	end
	if (!dl && dl_d) begin
		// the end: the name from the file's first word, then the card
		fin <= 1;
		if (sz == 0) mgo <= 1; else hrd <= 1;
	end

	// which file this is (by name), and so where its data goes in DDR3
	case (mst)
	M_IDLE: if (mgo) mst <= M_NM;
	M_NM: begin nm <= name_of(hdr); ij <= 0; mfound <= 0; mst <= M_L; end
	M_L: begin ta <= t_name[ij[2:0]]; mst <= M_C; end
	M_C: begin
		if (ij < cnt && cres[1]) begin mfound <= 1; mk <= ij[2:0]; end
		ij <= ij + 1'd1;
		mst <= (ij == 4'd7) ? M_E : M_L;
	end
	M_E: begin o_k <= t_off[mk]; n_k <= t_ce[mk] - t_cs[mk]; mst <= M_F; end
	M_F: begin reuse <= mfound && ({2'b0, o_k} + n_k == {1'b0, dend}); mst <= M_G; end
	M_G: begin
		base <= reuse ? {1'b0, o_k} : dend;
		reject <= !mfound && cnt == 4'd8;
		mst <= M_H;
	end
	M_H: begin
		lim <= {MAXSEC - base, 9'd0};
		if (base[17]) reject <= 1;
		mdone <= 1;
		mst <= M_IDLE;
	end
	default: mst <= M_IDLE;
	endcase

	// the file is in: put it in the table (in its place by name, or over the same name), then work out the clusters
	case (ist)
	I_IDLE:
			if (fin && mdone && mst == M_IDLE) begin
				fin <= 0;
				fin_len <= (sz > lim) ? lim : sz;    // cut at the end of DDR3
				ist <= I_LEN;
			end
			else if (clr_q && !dl && !dl_d && !fin && !hrd && cst == C_IDLE) begin
			clr_q <= 0;
			cnt <= 0; dend <= 0; rc2 <= 0; tot <= 19'd3; valid <= 0;
			for (int k = 0; k < 8; k++) begin t_cs[k] <= 0; t_ce[k] <= 0; end
			cleared <= 1;
		end
	I_LEN: begin
		nsec_n <= 18'((fin_len + 27'd511) >> 9);
		if (fin_len == 0 || reject) ist <= I_END;
		else ist <= I_CP0;
	end
	I_CP0: begin
		nb <= {nsec_n, 3'd0};
		sw <= 0;
		dw <= {base[16:0], 6'd0};
		xstart <= 1;
		ist <= I_CP1;
	end
	I_CP1:
		if (xst == X_IDLE && !xstart) begin ij <= 0; p <= 0; eqf <= 0; ist <= I_S1; end
	I_S1: begin ta <= t_name[ij[2:0]]; ist <= I_S2; end
	I_S2: begin
		if (ij < cnt) begin
			if (cres[1]) begin eqf <= 1; pe <= ij[2:0]; end
			else if (cres[0]) p <= ij + 1'd1;
		end
		ij <= ij + 1'd1;
		ist <= (ij == 4'd7) ? I_INS : I_S1;
	end
	I_INS: begin
		if (eqf) begin
			t_len[pe] <= fin_len;
			t_off[pe] <= base[16:0];
		end
		else begin
			for (int k = 7; k > 0; k--)
				if (k > p) begin t_name[k] <= t_name[k - 1]; t_len[k] <= t_len[k - 1]; t_off[k] <= t_off[k - 1]; end
			t_name[p[2:0]] <= nm;
			t_len[p[2:0]] <= fin_len;
			t_off[p[2:0]] <= base[16:0];
			cnt <= cnt + 1'd1;
		end
		ist <= I_CS0;
	end
	I_CS0: begin ij <= 0; cum <= (cnt >= 4'd8) ? 19'd4 : 19'd3; ist <= I_CS1; end
	I_CS1: begin lj <= (ij < cnt) ? t_len[ij[2:0]] : 27'd0; ist <= I_CS2; end
	I_CS2: begin nj <= 18'((lj + 27'd511) >> 9); ist <= I_CS3; end
	I_CS3: begin
		t_cs[ij[2:0]] <= (ij < cnt) ? cum : 19'd0;
		t_ce[ij[2:0]] <= (ij < cnt) ? cum + 19'(nj) : 19'd0;
		cum <= cum + 19'(nj);
		ij <= ij + 1'd1;
		ist <= (ij == 4'd7) ? I_FIN : I_CS1;
	end
	I_FIN: begin
		dend <= base + nsec_n;
		tot <= cum;
		rc2 <= cnt >= 4'd8;
		valid <= 1;
		loaded <= 1;
		ist <= I_IDLE;
	end
	I_END: begin
		// nothing to add (empty, or it does not fit): the card is as it was
		if (cnt != 0) begin valid <= 1; loaded <= 1; end
		ist <= I_IDLE;
	end
	default: ist <= I_IDLE;
	endcase

	// DDR3: the mover's header read and bursts, the card's writes and reads (never two at once:
	// the mover starts only with the card idle, the card starts only when the mover is idle)
	case (xst)
	X_IDLE:
		if (hrd && cst == C_IDLE && !ddr_we && !ddr_rd) begin
			ddr_rd <= 1; stg <= 1; bc <= 8'd1; daddr <= 0; xst <= X_HWT;
		end
		else if (xstart && !hrd) begin xstart <= 0; xst <= X_RD; end
	X_HWT:
		if (ddr_dout_ready) begin
			for (int k = 0; k < 8; k++) hdr[8 * k +: 8] <= (sz > k) ? ddr_dout[8 * k +: 8] : 8'h00;   // a short file: zeros past its end
			hrd <= 0; mgo <= 1; xst <= X_IDLE;
		end
	X_RD: begin
		ddr_rd <= 1; stg <= 1; bc <= 8'd8; daddr <= sw; bk <= 0; xst <= X_RWT;
	end
	X_RWT:
		if (ddr_dout_ready) begin
			for (int k = 0; k < 7; k++) sb[k] <= sb[k + 1];
			sb[7] <= ddr_dout;
			bk <= bk + 1'd1;
			if (bk == 4'd7) xst <= X_W0;
		end
	X_W0: begin
		ddr_we <= 1; ddr_be <= 8'hFF; stg <= 0; bc <= 8'd8; daddr <= dw; bk <= 0;
		ddr_din <= sb[0];
		for (int k = 0; k < 7; k++) sb[k] <= sb[k + 1];
		xst <= X_WR;
	end
	X_WR:
		if (ddr_we && !ddr_busy) begin
			for (int k = 0; k < 7; k++) sb[k] <= sb[k + 1];
			ddr_din <= sb[0];
			bk <= bk + 1'd1;
			if (bk == 4'd7) begin ddr_we <= 0; xst <= X_NEXT; end
		end
	X_NEXT: begin
		sw <= sw + 23'd8; dw <= dw + 23'd8; nb <= nb - 1'd1;
		xst <= (nb == 21'd1) ? X_IDLE : X_RD;
	end
	default: xst <= X_IDLE;
	endcase

	if (cwe_set) begin
		ddr_we <= 1; we_card <= 1; stg <= 0; bc <= 8'd1; daddr <= cw_addr; ddr_din <= cw_data; ddr_be <= 8'hFF;
	end
	else if (ddr_we && !ddr_busy && xst != X_WR) begin
		ddr_we <= 0;
		we_card <= 0;
	end
	if (crd_set) begin ddr_rd <= 1; stg <= 0; bc <= 8'd1; daddr <= cr_addr; end
	else if (ddr_rd && !ddr_busy) ddr_rd <= 0;
end
initial begin fin = 0; dl_d = 0; we_card = 0; sz = 0; mgo = 0; clr_q = 0; mdone = 0; lim = MAXLEN; hrd = 0; xstart = 0; xst = X_IDLE; end

// ---------------------------------------------------------------------------
// the card
// ---------------------------------------------------------------------------


localparam [15:0] DATE = 16'h5C21;                  // FAT_DATE: 2026-01-01
localparam [87:0] LABEL = "CHGAME     ";            // SDCARD_LABEL, 11 characters

// A byte of a worked-out sector. (Everything it reads comes in as an argument: Quartus 17 saw name as never read
// otherwise.) rn, rl, rcl: the root entry's file (name, length, first cluster); rok: there is one at this offset.
function [7:0] meta(input [2:0] k, input [8:0] o, input [31:0] fv, input [63:0] rn, input [26:0] rl,
                    input [18:0] rcl, input rok);
	meta = 8'h00;
	case (k)
	K_MBR: case (o)
		9'h1BF, 9'h1C3: meta = 8'hFE;
		9'h1C0, 9'h1C1, 9'h1C4, 9'h1C5: meta = 8'hFF;
		9'h1C2: meta = 8'h0C;
		9'h1C7: meta = 8'h08;                       // 2048
		9'h1CB: meta = 8'hF8; 9'h1CC: meta = 8'h03; // 260096 sectors
		9'h1FE: meta = 8'h55; 9'h1FF: meta = 8'hAA;
		default: ;
	endcase
	K_BOOT: case (o)
		0: meta = 8'hEB; 1: meta = 8'h58; 2: meta = 8'h90;
		3: meta = "M"; 4: meta = "S"; 5: meta = "W"; 6: meta = "I"; 7: meta = "N"; 8: meta = "4"; 9: meta = "."; 10: meta = "1";
		12: meta = 8'h02; 13: meta = 8'h01; 14: meta = 8'h20; 16: meta = 8'h02; 21: meta = 8'hF8;
		24: meta = 8'h3F; 26: meta = 8'hFF; 29: meta = 8'h08;
		33: meta = 8'hF8; 34: meta = 8'h03;
		36: meta = 8'hF0; 37: meta = 8'h07;
		44: meta = 8'h02; 48: meta = 8'h01; 50: meta = 8'h06;
		64: meta = 8'h80; 66: meta = 8'h29;
		67: meta = 8'h31; 68: meta = 8'h41; 69: meta = 8'h4B; 70: meta = 8'h41;
		71: meta = "C"; 72: meta = "H"; 73: meta = "G"; 74: meta = "A"; 75: meta = "M"; 76: meta = "E";
		77, 78, 79, 80, 81: meta = " ";
		82: meta = "F"; 83: meta = "A"; 84: meta = "T"; 85: meta = "3"; 86: meta = "2";
		87, 88, 89: meta = " ";
		510: meta = 8'h55; 511: meta = 8'hAA;
		default: ;
	endcase
	K_FSI: case (o)
		0, 1: meta = 8'h52; 2: meta = 8'h61; 3: meta = 8'h41;
		484, 485: meta = 8'h72; 486: meta = 8'h41; 487: meta = 8'h61;
		488, 489, 490, 491, 492, 493, 494, 495: meta = 8'hFF;
		510: meta = 8'h55; 511: meta = 8'hAA;
		default: ;
	endcase
	K_FAT: meta = fv[8 * o[1:0] +: 8];
	K_ROOT:
		if (o < 11) meta = LABEL[8 * (10 - o) +: 8];
		else if (o == 11) meta = 8'h08;
		else if (o >= 32 && rok) case (o[4:0])
			0, 1, 2, 3, 4, 5, 6, 7: meta = rn[8 * o[2:0] +: 8];
			8: meta = "D"; 9: meta = "A"; 10: meta = "T";
			11: meta = 8'h20;
			16, 18, 24: meta = DATE[7:0];
			17, 19, 25: meta = DATE[15:8];
			20: meta = {5'd0, rcl[18:16]};
			26: meta = rcl[7:0]; 27: meta = rcl[15:8];
			28: meta = rl[7:0]; 29: meta = rl[15:8]; 30: meta = rl[23:16]; 31: meta = {5'd0, rl[26:24]};
			default: ;
		endcase
	default: ;
	endcase
endfunction

reg [31:0] lba;
reg        cwr;
reg  [2:0] kind;
reg        f_mbr, f_boot, f_fsi, f_fat1, f_fat2, f_root, f_data, f_cand;
reg [18:0] cl;                  // the sector's cluster
reg [10:0] fs;                  // the FAT's sector
reg [17:0] d;                   // the file's sector in DDR3
reg  [9:0] vb;                  // bytes of it that are the file's
reg [26:0] rem;
reg  [9:0] i;                   // byte (GEN), or word * 8 + byte (EMIT, WR)
reg  [2:0] ph;
reg [63:0] word;
// the lookup of a data sector among the files
reg  [3:0] fj;
reg  [2:0] fph;
reg [18:0] x_cs, x_ce;
reg        hit;
reg  [2:0] kf;
reg [16:0] y_off;
reg [26:0] y_len;
reg [17:0] fd;

// GEN's stages: P1 issues the byte and its FAT entry, P2 looks at the table, A works out the FAT entry,
// then the byte
reg        p1_v, p2_v, a_v;
reg  [8:0] p1_i, p2_i, a_i;
reg [18:0] p1_e, p2_e;
reg        ehit;
reg [63:0] p2_rn, a_rn;
reg [26:0] p2_rl, a_rl;
reg [18:0] p2_rcl, a_rcl;
reg        p2_rok, a_rok;
reg [31:0] a_fat;
wire [18:0] e = {fs, 7'd0} + 19'(i[8:2]);
wire [18:0] p1_e1 = p1_e + 19'd1;
wire [18:0] p2_e1 = p2_e + 19'd1;
wire  [7:0] ehv = { t_ce[7] == p1_e1, t_ce[6] == p1_e1, t_ce[5] == p1_e1, t_ce[4] == p1_e1,
                    t_ce[3] == p1_e1, t_ce[2] == p1_e1, t_ce[1] == p1_e1, t_ce[0] == p1_e1 };
wire  [3:0] rsel = p1_i[8:5] - 4'd1;             // the root entry this offset is in (offset 32 on)

always @(posedge clk) begin
	sd_buff_wr <= 0;
	cwe_set <= 0;
	crd_set <= 0;
	case (cst)
	C_IDLE:
		if ((sd_rd || sd_wr) && !sd_ack && !dl && !dl_d && !hrd && !ddr_we && !fin && ist == I_IDLE && xst == X_IDLE) begin
			lba <= sd_lba;
			cwr <= sd_wr;
			sd_ack <= 1;
			cst <= C_CL1;
		end
	C_CL1: begin
		f_mbr  <= lba == 0;
		f_boot <= lba == BOOT || lba == BOOT_B;
		f_fsi  <= lba == FSI || lba == FSI_B;
		f_fat1 <= lba >= FAT1 && lba < FAT2;
		f_fat2 <= lba >= FAT2 && lba < ROOT;
		f_root <= lba == ROOT;
		f_data <= 0;
		f_cand <= valid && lba >= ROOT + 32'd1 + {31'd0, rc2};
		cl     <= 19'(lba - 32'd6142);
		cst <= C_CL2;
	end
	C_CL2: begin
		fj <= 0;
		fph <= 0;
		cst <= f_cand ? C_FND : C_CL3;
	end

	// which file a data sector is in (a file a few clocks)
	C_FND: case (fph)
		3'd0: begin x_cs <= t_cs[fj[2:0]]; x_ce <= t_ce[fj[2:0]]; fph <= 3'd1; end
		3'd1: begin hit <= cl >= x_cs && cl < x_ce; fph <= 3'd2; end
		3'd2:
			if (hit) begin kf <= fj[2:0]; fph <= 3'd3; end
			else if (fj == 4'd7) cst <= C_CL3;
			else begin fj <= fj + 1'd1; fph <= 3'd0; end
		3'd3: begin y_off <= t_off[kf]; y_len <= t_len[kf]; fd <= 18'(cl - x_cs); fph <= 3'd4; end
		default: begin
			d <= {1'b0, y_off} + fd;
			rem <= y_len - {fd, 9'd0};
			f_data <= 1;
			cst <= C_CL3;
		end
	endcase

	C_CL3: begin
		kind <= f_mbr ? K_MBR : f_boot ? K_BOOT : f_fsi ? K_FSI : (f_fat1 || f_fat2) ? K_FAT :
		        f_root ? K_ROOT : f_data ? K_DATA : K_ZERO;
		fs   <= f_fat1 ? 11'(lba - FAT1) : 11'(lba - FAT2);
		cst <= C_CL4;
	end
	C_CL4: begin
		vb <= (rem >= 27'd512) ? 10'd512 : 10'(rem);
		i <= 0;
		ph <= 0;
		p1_v <= 0; p2_v <= 0; a_v <= 0;
		if (cwr) cst <= C_WRB;
		else if (kind == K_DATA) cst <= C_RD;
		else cst <= C_GEN;
	end

	// worked-out sectors: a byte a clock
	C_GEN: begin
		if (!i[9]) begin
			p1_v <= 1;
			p1_i <= i[8:0];
			p1_e <= e;
			i <= i + 1'd1;
		end
		else p1_v <= 0;
		p2_v <= p1_v;
		p2_i <= p1_i;
		p2_e <= p1_e;
		ehit <= |ehv;
		p2_rok <= p1_i[8:5] != 0 && rsel < cnt;
		p2_rn  <= t_name[rsel[2:0]];
		p2_rl  <= t_len[rsel[2:0]];
		p2_rcl <= t_cs[rsel[2:0]];
		a_v <= p2_v;
		a_i <= p2_i;
		a_rn <= p2_rn; a_rl <= p2_rl; a_rcl <= p2_rcl; a_rok <= p2_rok;
		a_fat <= (p2_e == 0) ? 32'h0FFF_FFF8 : (p2_e == 1) ? 32'h0FFF_FFFF :
		         (p2_e == 2) ? (rc2 ? 32'd3 : 32'h0FFF_FFFF) :
		         (p2_e == 3 && rc2) ? 32'h0FFF_FFFF :
		         (valid && p2_e >= (rc2 ? 19'd4 : 19'd3) && p2_e < tot) ? (ehit ? 32'h0FFF_FFFF : 32'(p2_e1)) : 32'd0;
		if (a_v) begin
			sd_buff_wr <= 1;
			sd_buff_addr <= a_i;
			sd_buff_dout <= (kind == K_ZERO || (kind == K_ROOT && !valid)) ? 8'h00 : meta(kind, a_i, a_fat, a_rn, a_rl, a_rcl, a_rok);
			if (a_i == 9'd511) cst <= C_END;
		end
	end

	// the file's sectors: a word from DDR3, then its 8 bytes
	C_RD: begin
		crd_set <= 1;
		cr_addr <= {d[16:0], i[8:3]};
		cst <= C_RDW;
	end
	C_RDW:
		if (ddr_dout_ready) begin
			word <= ddr_dout;
			ph <= 0;
			cst <= C_EMIT;
		end
	C_EMIT: begin
		sd_buff_wr <= 1;
		sd_buff_addr <= i[8:0];
		sd_buff_dout <= (i < vb) ? word[8 * i[2:0] +: 8] : 8'h00;
		i <= i + 1'd1;
		if (i[2:0] == 3'd7) cst <= (i[8:3] == 6'd63) ? C_END : C_RD;
	end

	// a block written: its bytes from the buffer (the address, five clocks for
	// chg_card's registers and the buffer, the byte)
	C_WRB: begin
		sd_buff_addr <= i[8:0];
		ph <= ph + 1'd1;
		if (ph == 3'd5) begin
			ph <= 0;
			word[8 * i[2:0] +: 8] <= sd_buff_din;
			i <= i + 1'd1;
			if (i[2:0] == 3'd7) cst <= C_WRD;
		end
	end
	C_WRD: begin
		if (kind == K_DATA) begin
			cwe_set <= 1;
			cw_addr <= {d[16:0], i[8:3] - 6'd1};
			cw_data <= word;
			cst <= C_WRW;
		end
		else cst <= (i[9]) ? C_END : C_WRB;
	end
	C_WRW:
		if (!cwe_set && !ddr_we) cst <= (i[9]) ? C_END : C_WRB;

	C_END: begin
		sd_ack <= 0;
		cst <= C_IDLE;
	end
	default: cst <= C_IDLE;
	endcase
end

endmodule
