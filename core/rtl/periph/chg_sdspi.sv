// The CHGame's microSD card in SPI mode, as emu/src/sdspi.c has it: an SDHC
// card, block addressed, CRCs not checked, answering one byte per byte the
// SPI sends while its CS (PB11) is low. The 512-byte blocks come from a
// MiSTer hps_io virtual disk (sd_lba, sd_rd, sd_wr, sd_ack, sd_buff_*).
//
// Translates sdspi_reset, sdspi_deselect, sdspi_xfer and command(). What the
// card says next (sdspi.c's out[] with nout and pout) is kept here as a short
// prefix of up to 6 bytes, then optionally a data part: a filler 0xFF, the
// start token 0xFE, dlen data bytes and two 0xFF CRC bytes (put_block). The
// data are a block buffer (CMD17/18), 0xFF (a block past the end of the
// card, read_block failing) or a register (CSD, CID, SD status, SCR).
//
// Block timing. The C card reads a block the moment CMD17/18 completes (and
// CMD18's next block the moment the last one is out) and writes one the
// moment its last CRC byte is in; it never makes the host wait. hps_io
// answers milliseconds later. Two 512-byte buffers let CMD18 fetch the next
// block while the present one is sent. When a block is still missing at the
// byte that would be its start token, or a write is still running at the
// last busy byte of the data response:
//   hold_mode = 1 (simulation, lockstep with the C): xfer_done is held back,
//     so the SPI holds board time until the block is there. Board time sees
//     nothing, and every byte is the C's.
//   hold_mode = 0 (the MiSTer): the card answers 0xFF (no token yet) or 0x00
//     (still busy) and stays where it is, as a real card does. CHGame's
//     SdSpi.cpp waits up to 1 s for a token and for busy to end.

module chg_sdspi
(
	input             clk,
	input             reset,        // the card's power (sdspi_reset): with the board's power-on only
	input             hold_mode,

	// one byte each way while the card is selected (sdspi_xfer)
	input             xfer_stb,
	input       [7:0] mosi,
	output reg        xfer_done,
	output reg  [7:0] miso,
	input             desel_stb,    // a frame went by with CS high (sdspi_deselect)

	// the card in the slot: hps_io's image, block 0 = byte 0
	input             img_mounted,
	input      [63:0] img_size,

	output reg [31:0] sd_lba,
	output reg        sd_rd,
	output reg        sd_wr,
	input             sd_ack,
	input       [8:0] sd_buff_addr,
	input       [7:0] sd_buff_dout,
	output      [7:0] sd_buff_din,
	input             sd_buff_wr
);

// ---------------------------------------------------------------------------
// the card in the slot (struct SdCard *card; NULL for none)
// ---------------------------------------------------------------------------

reg        present;
reg [63:0] size;
wire [54:0] sectors = size[63:9];
always @(posedge clk) begin
	if (img_mounted) begin
		present <= img_size != 0;
		size    <= img_size;
	end
end
initial begin present = 0; size = 0; end

// CSD (csd()): all constant but C_SIZE, sectors / 1024 - 1
wire [44:0] kblocks = size[63:19];
wire [21:0] c_size  = (kblocks != 0) ? 22'(kblocks - 1'd1) : 22'd0;

function [7:0] csd_byte(input [5:0] i, input [21:0] cs);
	case (i[3:0])
		0: csd_byte = 8'h40;  1: csd_byte = 8'h0E;  2: csd_byte = 8'h00;  3: csd_byte = 8'h32;
		4: csd_byte = 8'h5B;  5: csd_byte = 8'h59;  6: csd_byte = 8'h00;  7: csd_byte = {2'b00, cs[21:16]};
		8: csd_byte = cs[15:8]; 9: csd_byte = cs[7:0]; 10: csd_byte = 8'h7F; 11: csd_byte = 8'h80;
		12: csd_byte = 8'h0A; 13: csd_byte = 8'h40; 14: csd_byte = 8'h00; default: csd_byte = 8'h01;
	endcase
endfunction

// CID (cid())
function [7:0] cid_byte(input [5:0] i);
	case (i[3:0])
		0: cid_byte = 8'h43;  1: cid_byte = "C";  2: cid_byte = "H";  3: cid_byte = "C";
		4: cid_byte = "H";    5: cid_byte = "G";  6: cid_byte = "S";  7: cid_byte = "D";
		8: cid_byte = 8'h10;  9: cid_byte = 8'h20; 10: cid_byte = 8'h26; 11: cid_byte = 8'h09;
		12: cid_byte = 8'h30; 13: cid_byte = 8'h01; 14: cid_byte = 8'hA9; default: cid_byte = 8'h01;
	endcase
endfunction

// ---------------------------------------------------------------------------
// two block buffers: port 1 is the card's, port 2 hps_io's
// ---------------------------------------------------------------------------

reg        b1_we;
reg  [9:0] b1_addr;
reg  [7:0] b1_wdata;
wire [7:0] b1_q;

reg        ebuf;                    // the buffer hps_io is working on
reg        ebusy, ewr, eacked;      // a transfer is under way, a write, hps_io has taken it
// a block RAM Quartus can build (chg_dpram)
chg_dpram #(.AW(10), .BYTES(1)) buff
(
	.clk(clk),
	.a_addr(b1_addr), .a_we(b1_we), .a_d(b1_wdata), .a_q(b1_q),
	.b_addr({ebuf, sd_buff_addr}), .b_we(sd_buff_wr && sd_ack), .b_d(sd_buff_dout), .b_q(sd_buff_din)
);

// what each buffer holds
localparam B_FREE = 2'd0, B_QUEUED = 2'd1, B_LOADING = 2'd2, B_LOADED = 2'd3;
reg  [1:0] bstate[2];
reg [32:0] blba[2];
reg        bdrop[2];                // a write to its block came in while it was loading

// the write waiting for hps_io (card side: the block in wbuf)
reg        wr_pend;                 // queued
reg        wr_busy;                 // queued or being written
reg        wbuf;
reg [32:0] wlba;

// ---------------------------------------------------------------------------
// the card's state (ChgSdSpi)
// ---------------------------------------------------------------------------

reg        idle, app;
reg  [7:0] cmd0, cmd1, cmd2, cmd3, cmd4;
reg  [2:0] ncmd;
reg  [7:0] pre[6];                  // out[]: the prefix
reg  [2:0] npre;
reg  [2:0] dk;                      // the data part after it
localparam D_NONE = 3'd0, D_BLOCK = 3'd1, D_FFBLOCK = 3'd2, D_CSD = 3'd3, D_CID = 3'd4,
           D_SDSTAT = 3'd5, D_SCR = 3'd6;
reg  [9:0] pout;
reg        busy_resp;               // the prefix ends in a write's busy bytes
reg        cur;                     // the buffer D_BLOCK is sent from
reg        reading;                 // CMD18
reg  [1:0] wstate;
reg        multi_write;
reg  [9:0] nblk;
reg [32:0] lba;

wire [9:0] dlen = (dk == D_BLOCK || dk == D_FFBLOCK) ? 10'd512 : (dk == D_CSD || dk == D_CID) ? 10'd16 :
                  (dk == D_SDSTAT) ? 10'd64 : 10'd8;
wire [9:0] nout = 10'(npre) + ((dk != D_NONE) ? dlen + 10'd4 : 10'd0);
wire       cur_ok = bstate[cur] == B_LOADED;

// a block wanted for sending: into a buffer that has it or is free to take it.
// Returns the buffer; the caller marks it.
function pick_read(input [32:0] want, input c);
	if (bstate[c] != B_FREE && blba[c] == want && !bdrop[c]) pick_read = c;
	else if (bstate[!c] != B_FREE && blba[!c] == want && !bdrop[!c]) pick_read = !c;
	else if (!bbusy(c)) pick_read = c;
	else pick_read = !c;
endfunction

// a buffer hps_io has, or will have, for a write
function wbusy(input b);
	wbusy = (ebusy && ewr && ebuf == b) || (wr_pend && wbuf == b);
endfunction
// a buffer that must not be written into: hps_io is filling it or will read it
function bbusy(input b);
	bbusy = bstate[b] == B_LOADING || wbusy(b);
endfunction

// ---------------------------------------------------------------------------
// one byte (sdspi_xfer)
// ---------------------------------------------------------------------------

localparam X_IDLE = 2'd0, X_OUT = 2'd1, X_DATA = 2'd2, X_IN = 2'd3;
reg  [1:0] xs;
reg  [7:0] xmosi;
reg  [7:0] xout;
reg        xfrom_buf;

// The streaming fast path: a data byte of a block being read, whose address was
// put on the buffer a byte ahead (b1_addr, in X_IN), goes straight from X_IDLE to
// X_IN, as X_OUT and X_DATA would have taken it (two clocks less a byte).
// rd_fast is registered from the state of the clock before (xs idle, no desel).
wire [9:0] qn = pout - 10'(npre);
reg        rd_fast;
always @(posedge clk)
	rd_fast <= xs == X_IDLE && !desel_stb && !xfer_stb && dk == D_BLOCK && wstate == 2'd0 &&
	           pout >= 10'(npre) && qn >= 10'd2 && qn < 10'd514 && b1_addr == {cur, 9'(qn - 10'd2)};

// queue a read of block 'want' into buffer b for sending (D_BLOCK), or note
// that it is past the end (D_FFBLOCK)
// (b and whether it already has the block are worked out by the caller,
// from the two buffers' compares made side by side: 96 MHz)
task automatic want_block(input [32:0] want, input b, input has);
	if (55'(want) < sectors) begin            // read_block fails past the end
		dk  <= D_BLOCK;
		cur <= b;
		if (!has) begin
			bstate[b] <= B_QUEUED;
			blba[b]   <= want;
			bdrop[b]  <= 0;
		end
	end
	else dk <= D_FFBLOCK;
endtask

task automatic set_pre(input [2:0] n, input [7:0] a, input [7:0] b, input [7:0] c,
                       input [7:0] d, input [7:0] e, input [7:0] f);
	npre <= n; pout <= 0;
	pre[0] <= a; pre[1] <= b; pre[2] <= c; pre[3] <= d; pre[4] <= e; pre[5] <= f;
endtask

// the sixth byte of a command is in (command())
task automatic command;
	reg  [5:0] idx;
	reg [31:0] arg;
	reg  [7:0] r1;
	reg        was_app;
	reg        b;
	begin
		idx = cmd0[5:0];
		arg = {cmd1, cmd2, cmd3, cmd4};
		was_app = app;
		app <= 0;
		r1 = idle ? 8'h01 : 8'h00;
		reading <= 0;
		dk <= D_NONE;
		busy_resp <= 0;
		set_pre(1, 8'hFF, 0, 0, 0, 0, 0);       // start_response: NCR, one byte
		if (!present) set_pre(0, 0, 0, 0, 0, 0, 0);
		else if (was_app && idx == 41) begin   // SD_SEND_OP_COND: ready at once
			idle <= 0;
			set_pre(2, 8'hFF, 8'h00, 0, 0, 0, 0);
		end
		else if (was_app && idx == 13) begin   // SD_STATUS
			set_pre(3, 8'hFF, r1, 8'h00, 0, 0, 0);
			dk <= D_SDSTAT;
		end
		else if (was_app && idx == 51) begin   // SCR
			set_pre(2, 8'hFF, r1, 0, 0, 0, 0);
			dk <= D_SCR;
		end
		else if (was_app && (idx == 22 || idx == 23 || idx == 42))
			set_pre(2, 8'hFF, r1, 0, 0, 0, 0);
		else case (idx)
			0: begin                        // GO_IDLE_STATE
				idle <= 1;
				wstate <= 0;
				set_pre(2, 8'hFF, 8'h01, 0, 0, 0, 0);
			end
			1: begin                        // SEND_OP_COND
				idle <= 0;
				set_pre(2, 8'hFF, 8'h00, 0, 0, 0, 0);
			end
			8: set_pre(6, 8'hFF, r1, 8'h00, 8'h00, {4'h0, arg[11:8]}, arg[7:0]);
			55: begin app <= 1; set_pre(2, 8'hFF, r1, 0, 0, 0, 0); end
			58: set_pre(6, 8'hFF, r1, 8'hC0, 8'hFF, 8'h80, 8'h00);
			9:  begin set_pre(2, 8'hFF, r1, 0, 0, 0, 0); dk <= D_CSD; end
			10: begin set_pre(2, 8'hFF, r1, 0, 0, 0, 0); dk <= D_CID; end
			12: set_pre(4, 8'hFF, 8'hFF, r1, 8'h00, 0, 0);
			13: set_pre(3, 8'hFF, r1, 8'h00, 0, 0, 0);
			16, 59, 23, 32, 33, 38: set_pre(2, 8'hFF, r1, 0, 0, 0, 0);
			17, 18: begin                   // READ_SINGLE / MULTIPLE_BLOCK
				set_pre(2, 8'hFF, r1, 0, 0, 0, 0);
				begin
					automatic reg h0 = bstate[0] != B_FREE && blba[0] == {1'b0, arg} && !bdrop[0];
					automatic reg h1 = bstate[1] != B_FREE && blba[1] == {1'b0, arg} && !bdrop[1];
					// pick_read(arg, cur)
					automatic reg hc = cur ? h1 : h0, ho = cur ? h0 : h1;
					b = hc ? cur : ho ? !cur : !bbusy(cur) ? cur : !cur;
					want_block({1'b0, arg}, b, h0 || h1);
				end
				lba <= {1'b0, arg} + 1'd1;
				reading <= idx == 18;
			end
			24, 25: begin                   // WRITE_BLOCK / MULTIPLE_BLOCK
				set_pre(2, 8'hFF, r1, 0, 0, 0, 0);
				lba <= {1'b0, arg};
				wstate <= 1;
				multi_write <= idx == 25;
			end
			default: set_pre(2, 8'hFF, r1 | 8'h04, 0, 0, 0, 0);
		endcase
	end
endtask

wire [9:0] q = pout - 10'(npre);     // position in the data part

always @(posedge clk) begin
	xfer_done <= 0;
	b1_we <= 0;
	if (reset) begin
		// sdspi_reset
		idle <= 1; app <= 0; ncmd <= 0; npre <= 0; dk <= D_NONE; pout <= 0; busy_resp <= 0;
		reading <= 0; wstate <= 0; multi_write <= 0; nblk <= 0; lba <= 0; cur <= 0;
		xs <= X_IDLE;
		bstate[0] <= B_FREE; bstate[1] <= B_FREE; bdrop[0] <= 0; bdrop[1] <= 0;
		blba[0] <= 0; blba[1] <= 0;
		wr_pend <= 0; wr_busy <= 0; wbuf <= 0; wlba <= 0;
		ebusy <= 0; ewr <= 0; sd_rd <= 0; sd_wr <= 0; sd_lba <= 0; ebuf <= 0; eacked <= 0;
	end
	else begin
		// ---- the card side ----
		case (xs)
		X_IDLE:
			if (desel_stb) begin
				// sdspi_deselect
				ncmd <= 0; npre <= 0; dk <= D_NONE; pout <= 0; busy_resp <= 0;
				reading <= 0;
			end
			else if (xfer_stb) begin
				xmosi <= mosi;
				if (rd_fast) begin
					xfrom_buf <= 1;
					pout <= pout + 1'd1;
					xs <= X_IN;
				end
				else begin
					xfrom_buf <= 0;
					xout <= 8'hFF;
					xs <= X_OUT;
				end
			end
		X_OUT:
			// what goes out was decided before this byte came in. Staying
			// here holds the byte back (hold_mode)
			if (wstate == 2 && nblk == 513 && wr_pend) ;
				// the last byte of a block while the one before still waits
				// for hps_io (the host did not wait out its busy bytes): wait
			else if (wstate == 1 && (xmosi == 8'hFE || (multi_write && xmosi == 8'hFC)) && bbusy(0) && bbusy(1)) ;
				// a block coming with no buffer to put it in: wait
			else begin
				if (pout < nout) begin
					if (pout < 10'(npre)) begin
						if (busy_resp && pout == 10'(npre) - 1'd1 && wr_busy) begin
							// a write still going: busy, as long as it takes
							if (!hold_mode) begin xout <= 8'h00; xs <= X_DATA; end
						end
						else begin
							xout <= pre[pout[2:0]];
							pout <= pout + 1'd1;
							xs <= X_DATA;
						end
					end
					else if (q == 1) begin
						// the start token, once the block is there
						if (dk != D_BLOCK || cur_ok) begin
							xout <= 8'hFE;
							pout <= pout + 1'd1;
							xs <= X_DATA;
						end
						else if (!hold_mode) xs <= X_DATA;      // 0xFF: not yet
					end
					else begin
						if (q >= 2 && q < dlen + 10'd2) begin
							case (dk)
								D_BLOCK:  begin xfrom_buf <= 1; b1_addr <= {cur, 9'(q - 10'd2)}; end
								D_FFBLOCK: xout <= 8'hFF;
								D_CSD:    xout <= csd_byte(6'(q - 10'd2), c_size);
								D_CID:    xout <= cid_byte(6'(q - 10'd2));
								D_SDSTAT: xout <= (q == 2) ? 8'h80 : 8'h00;
								default:  xout <= (q == 2) ? 8'h02 : (q == 3) ? 8'h35 : (q == 4) ? 8'h80 : 8'h00;
							endcase
						end
						pout <= pout + 1'd1;
						xs <= X_DATA;
					end
				end
				else if (reading && present) begin
					// CMD18: the next block, after the gap
					automatic reg h0 = bstate[0] != B_FREE && blba[0] == lba && !bdrop[0];
					automatic reg h1 = bstate[1] != B_FREE && blba[1] == lba && !bdrop[1];
					automatic reg hc = cur ? h1 : h0, ho = cur ? h0 : h1;
					// the other buffer if it has it, else pick_read(lba, !cur)
					automatic reg b = ho ? !cur : hc ? cur : !bbusy(!cur) ? !cur : cur;
					npre <= 0;
					busy_resp <= 0;
					want_block(lba, b, h0 || h1);
					lba <= lba + 1'd1;
					pout <= 1;
					xs <= X_DATA;
				end
				else xs <= X_DATA;
			end
		X_DATA: begin
			// the buffer's byte has been read
			xs <= X_IN;
		end
		X_IN: begin
			automatic reg [7:0] o = xfrom_buf ? b1_q : xout;
			miso <= o;
			xfer_done <= 1;
			xs <= X_IDLE;
			// the next block byte, put on the buffer now (rd_fast)
			if (dk == D_BLOCK && qn >= 10'd2 && qn < 10'd514) b1_addr <= {cur, 9'(qn - 10'd2)};
			// a block the host is writing
			if (wstate == 1 && (xmosi == 8'hFE || (multi_write && xmosi == 8'hFC))) begin
				wstate <= 2;
				nblk <= 0;
				// the buffer it goes into: not one hps_io is filling or writing
				wbuf <= !bbusy(!cur) ? !cur : cur;
			end
			else if (wstate == 1 && multi_write && xmosi == 8'hFD) begin
				// stop token: busy a moment, then done
				wstate <= 0;
				dk <= D_NONE;
				set_pre(3, 8'hFF, 8'h00, 8'h00, 0, 0, 0);
				busy_resp <= 1;
			end
			else if (wstate == 1 && xmosi == 8'hFF) ;    // still waiting for the token
			else if (wstate == 2) begin
				if (nblk < 512) begin
					b1_we <= 1;
					b1_addr <= {wbuf, nblk[8:0]};
					b1_wdata <= xmosi;
					if (bstate[wbuf] != B_LOADING) bstate[wbuf] <= B_FREE;
				end
				nblk <= nblk + 1'd1;
				if (nblk == 513) begin
					if (present && 55'(lba) < sectors) begin   // sdcard_write fails past the end
						wr_pend <= 1;
						wr_busy <= 1;
						wlba <= lba;
						// what the buffers hold of that block is old now
						for (int i = 0; i < 2; i++)
							if (1'(i) != wbuf && bstate[i] != B_FREE && blba[i] == lba) begin
								if (bstate[i] == B_LOADING) bdrop[i] <= 1;
								else bstate[i] <= B_FREE;
							end
					end
					lba <= lba + 1'd1;
					dk <= D_NONE;
					set_pre(3, 8'h05, 8'h00, 8'h00, 0, 0, 0);    // data accepted, busy programming
					busy_resp <= 1;
					wstate <= multi_write ? 2'd1 : 2'd0;
				end
			end
			else begin
				if (wstate == 1) wstate <= 0;               // a command instead
				// commands
				if (ncmd == 0) begin
					if (xmosi[7:6] == 2'b01) begin cmd0 <= xmosi; ncmd <= 1; end
				end
				else begin
					case (ncmd)
						1: cmd1 <= xmosi;
						2: cmd2 <= xmosi;
						3: cmd3 <= xmosi;
						4: cmd4 <= xmosi;
						default: ;
					endcase
					ncmd <= ncmd + 1'd1;
					if (ncmd == 5) begin
						ncmd <= 0;
						command();
					end
				end
			end
		end
		default: xs <= X_IDLE;
		endcase

		// ---- the hps_io side: one block at a time, writes first ----
		if (!ebusy) begin
			if (wr_pend) begin
				ebusy <= 1; ewr <= 1; ebuf <= wbuf; eacked <= 0;
				sd_lba <= wlba[31:0]; sd_wr <= 1;
				wr_pend <= 0;
			end
			else if (bstate[cur] == B_QUEUED || bstate[!cur] == B_QUEUED) begin
				automatic reg b = (bstate[cur] == B_QUEUED) ? cur : !cur;
				ebusy <= 1; ewr <= 0; ebuf <= b; eacked <= 0;
				sd_lba <= blba[b][31:0]; sd_rd <= 1;
				bstate[b] <= B_LOADING;
			end
		end
		else if (!eacked) begin
			if (sd_ack) begin eacked <= 1; sd_rd <= 0; sd_wr <= 0; end
		end
		else if (!sd_ack) begin
			ebusy <= 0;
			if (ewr) wr_busy <= wr_pend;
			else begin
				bstate[ebuf] <= bdrop[ebuf] ? B_FREE : B_LOADED;
				bdrop[ebuf] <= 0;
			end
		end

		// ---- CMD18: fetch the block after the one being sent ----
		if (reading && dk == D_BLOCK && !ebusy && !wr_pend && bstate[cur] == B_LOADED &&
		    bstate[!cur] == B_FREE && 55'(lba) < sectors && xs == X_IDLE && !xfer_stb) begin
			bstate[!cur] <= B_QUEUED;
			blba[!cur] <= lba;
			bdrop[!cur] <= 0;
		end
	end
end

endmodule
