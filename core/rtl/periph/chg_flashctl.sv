// The flash controller at 0x4002_2000, and the program's stores into the
// flash address space (0x0000_0000 / 0x0800_0000 + 64 KB), which chg_soc
// forwards on the same bus port.
//
// Translates flash_ctl_read, flash_ctl_write, flash_mem_write, flash_erase
// and flash_touched in emu/src/bus.c, and the split of a misaligned store
// into bytes that bus_write_slow does below 0x4000_0000 (each byte is a
// flash_mem_write of its own, with its own stall).
//
//   KEYR (0x04)      0x45670123 then 0xCDEF89AB unlocks; anything else
//                    starts the sequence again
//   MODEKEYR (0x24)  the same for fast (page) programming
//   CTLR (0x10)      kept without STRT, BUF_LOAD, BUF_RST, LOCK, FLOCK;
//                    LOCK and FLOCK read as the lock state. LOCK locks both,
//                    FLOCK the fast mode. Unlocked: BUF_RST (fast unlocked)
//                    fills the page buffer with 0xFF; STRT does, in order of
//                    precedence, a 256-byte page erase (PAGE_ER, fast
//                    unlocked), a 256-byte page program from the buffer
//                    (PAGE_PG, fast unlocked: flash &= buffer), a 1 KB erase
//                    (PER), the whole user flash (MER, 0..0xF7FF); then EOP
//   STATR (0x0C)     EOP (bit 5) cleared by writing 1
//   stores           locked, or at 0xF800 and above: nothing. PAGE_PG with
//                    fast unlocked: into the page buffer at (addr & 255).
//                    PG: flash &= the bytes, EOP
//
// Board time the C adds (m->cycles += ...) comes out as 'stall' with the
// ack: chg_soc must run that many ce clocks before the instruction goes on
// (see CONTRACT.md, section 2). STRT: 96,000 cycles
// with PAGE_ER, PER or MER in the value written, else 48,000, whatever was
// done; a PG store: 1,440 a byte operation (a halfword store is one, a
// misaligned halfword two).
//
// The flash itself is written through memory port B (CONTRACT.md section 6),
// a word at a time; the ack waits until all of it is done. fl_page_stb
// pulses once for every 256-byte page flash_touched marks dirty, with the
// page number in fl_page: the save side keeps the set.
//
// bus_reset clears the page buffer to 0x00, BUF_RST fills it with 0xFF: a
// buffer word not written since either reads as that byte four times
// (pdef, pdval), so the buffer can be a plain block RAM.

module chg_flashctl
(
	input             clk,
	input             reset,        // chg_reset: bus_reset
	input             ce,           // not used

	input             req,
	input             we,
	input      [31:0] addr,         // 0x4002_2xxx: registers; else a flash address
	input       [2:0] size,
	input      [31:0] wdata,
	output reg [31:0] rdata,
	output reg        ack,
	output reg [16:0] stall,        // board cycles the access takes, valid with ack

	// memory port B
	output reg        mreq,
	output reg        mwe,
	output reg [31:0] maddr,
	output      [2:0] msize,
	output reg [31:0] mwdata,
	input      [31:0] mrdata,
	input             mack,

	output reg        fl_page_stb,
	output reg  [7:0] fl_page
);

assign msize = 3'd4;

localparam FL_PG = 0, FL_PER = 1, FL_MER = 2, FL_STRT = 6, FL_LOCK = 7, FL_FLOCK = 15,
           FL_PAGE_PG = 16, FL_PAGE_ER = 17, FL_BUF_LOAD = 18, FL_BUF_RST = 19;
localparam [15:0] USER = 16'hF800;

reg [31:0] actlr, ctlr, faddr;
reg        eop;
reg        key1, mkey1;         // key_stage, mode_key_stage
reg        locked, fast_locked;

// the page buffer
reg [31:0] pbuf[0:63];
reg [63:0] pdef;
reg  [7:0] pdval;
reg  [5:0] pb_ra;
reg [31:0] pb_rd;
reg        pb_we;
reg  [5:0] pb_wa;
reg [31:0] pb_wd;
always @(posedge clk) begin
	if (pb_we) pbuf[pb_wa] <= pb_wd;
	pb_rd <= pbuf[pb_ra];
end
reg [5:0] pb_ra_q;
always @(posedge clk) pb_ra_q <= pb_ra;
wire [31:0] pb_word = pdef[pb_ra_q] ? {4{pdval}} : pb_rd;

// ---------------------------------------------------------------------------
// registers
// ---------------------------------------------------------------------------

wire [4:0]  sh  = {addr[1:0], 3'b000};
wire [31:0] wv  = wdata << sh;
wire [9:0]  off = {addr[9:2], 2'b00};

function [31:0] lane_r(input [31:0] x0, input [4:0] s, input [2:0] sz);
	reg [31:0] x;
	begin
		x = x0 >> s;
		if (sz == 3'd1) x = x & 32'h000000ff;
		else if (sz == 3'd2) x = x & 32'h0000ffff;
		lane_r = x;
	end
endfunction

reg [31:0] rv;
always @* begin
	case (off)
	10'h00:  rv = actlr;
	10'h0c:  rv = {26'd0, eop, 5'd0};
	10'h10:  rv = ctlr | (locked ? 32'h80 : 32'h0) | (fast_locked ? 32'h8000 : 32'h0);
	10'h14:  rv = faddr;
	10'h1c:  rv = 32'h03fffffc;
	10'h20:  rv = 32'hffffffff;
	default: rv = 32'd0;
	endcase
end

// a CTLR write: the lock state after it
wire n_locked = locked | wv[FL_LOCK];
wire n_fast   = fast_locked | wv[FL_LOCK] | wv[FL_FLOCK];

// ---------------------------------------------------------------------------
// the operations
// ---------------------------------------------------------------------------

localparam S_IDLE = 0, S_UNIT = 1, S_BUF_RD = 2, S_BUF_WR = 3, S_PG_WAIT = 5,
           S_PG_WAIT2 = 7, S_ER = 8, S_ER_WAIT = 9, S_PP = 10, S_PP_BUF = 11,
           S_PP_WAIT = 12, S_PP_WAIT2 = 14, S_DONE = 15;
reg  [3:0] st;

// a store: its units (one, or a byte each when misaligned)
reg [31:0] s_addr, s_data;
reg  [2:0] s_size;
reg        s_split;
reg  [2:0] s_i;
// the unit in hand
reg [23:0] u_a;
reg  [3:0] u_mask;              // byte lanes of the word
reg [31:0] u_data;              // in their lanes
// erase / page program
reg [15:0] e_cur, e_end;
reg  [5:0] p_w;
reg [15:0] p_base;

// the unit s_i of the store in hand
wire [31:0] ua_full = s_addr + {29'd0, s_i};
wire [23:0] ua      = s_split ? ua_full[23:0] : s_addr[23:0];
wire  [7:0] ubyte   = s_data[{s_i[1:0], 3'b000} +: 8];
wire  [3:0] umask   = s_split ? (4'b0001 << ua[1:0])
                    : (s_size == 3'd1) ? (4'b0001 << ua[1:0])
                    : (s_size == 3'd2) ? (4'b0011 << ua[1:0]) : 4'b1111;
wire [31:0] udata   = s_split ? ({24'd0, ubyte} << {ua[1:0], 3'b000}) : (s_data << {ua[1:0], 3'b000});
wire [31:0] lanes   = {{8{u_mask[3]}}, {8{u_mask[2]}}, {8{u_mask[1]}}, {8{u_mask[0]}}};

always @(posedge clk) begin
	ack <= 0;
	mreq <= 0;
	pb_we <= 0;
	fl_page_stb <= 0;
	if (reset) begin
		actlr <= 0; ctlr <= 0; faddr <= 0; eop <= 0;
		key1 <= 0; mkey1 <= 0; locked <= 1; fast_locked <= 1;
		pdef <= {64{1'b1}}; pdval <= 8'h00;
		st <= S_IDLE;
	end else case (st)
	S_IDLE: if (req) begin
		stall <= 0;
		if (addr[30]) begin
			// a register
			if (!we) begin
				rdata <= lane_r(rv, sh, size);
				ack <= 1;
			end else begin
				ack <= 1;
				case (off)
				10'h00: actlr <= wv;
				10'h04: begin
					if (!key1 && wv == 32'h45670123) key1 <= 1;
					else if (key1 && wv == 32'hCDEF89AB) begin key1 <= 0; locked <= 0; end
					else key1 <= 0;
				end
				10'h0c: if (wv[5]) eop <= 0;
				10'h10: begin
					locked <= n_locked;
					fast_locked <= n_fast;
					ctlr <= wv & ~32'h000C80C0;     // ~(STRT | BUF_LOAD | BUF_RST | LOCK | FLOCK)
					if (!n_locked) begin
						if (wv[FL_BUF_RST] && !n_fast) begin
							pdef <= {64{1'b1}};
							pdval <= 8'hff;
						end
						if (wv[FL_STRT]) begin
							eop <= 1;
							stall <= (wv[FL_PAGE_ER] | wv[FL_PER] | wv[FL_MER]) ? 17'd96000 : 17'd48000;
							if (wv[FL_PAGE_ER] && !n_fast) begin
								e_cur <= {faddr[15:8], 8'h00};
								e_end <= {faddr[15:8], 8'h00} + 16'd256;
								if (faddr[23:0] < {8'd0, USER}) begin ack <= 0; st <= S_ER; end
							end else if (wv[FL_PAGE_PG] && !n_fast) begin
								p_base <= {faddr[15:8], 8'h00};
								p_w <= 0;
								if (faddr[23:0] < {8'd0, USER}) begin ack <= 0; st <= S_PP; end
							end else if (wv[FL_PER]) begin
								e_cur <= {faddr[15:10], 10'h000};
								e_end <= {faddr[15:10], 10'h000} + 16'd1024;
								if (faddr[23:0] < {8'd0, USER}) begin ack <= 0; st <= S_ER; end
							end else if (wv[FL_MER]) begin
								e_cur <= 16'h0000;
								e_end <= USER;
								ack <= 0; st <= S_ER;
							end
						end
					end
				end
				10'h14: faddr <= wv;
				10'h24: begin
					if (!mkey1 && wv == 32'h45670123) mkey1 <= 1;
					else if (mkey1 && wv == 32'hCDEF89AB) begin mkey1 <= 0; fast_locked <= 0; end
					else mkey1 <= 0;
				end
				default: ;
				endcase
			end
		end else if (!we) begin
			rdata <= 0;             // flash reads are chg_soc's; not expected here
			ack <= 1;
		end else begin
			s_addr <= addr; s_data <= wdata; s_size <= size; s_i <= 0;
			s_split <= (size == 3'd2 && addr[0]) || (size == 3'd4 && addr[1:0] != 2'd0);
			st <= S_UNIT;
		end
	end

	// one unit of a store (flash_mem_write)
	S_UNIT: begin
		u_a <= ua; u_mask <= umask; u_data <= udata;
		if (locked || ua >= {8'd0, USER}) st <= S_DONE;
		else if (ctlr[FL_PAGE_PG] && !fast_locked) begin
			pb_ra <= ua[7:2];
			st <= S_BUF_RD;
		end else if (ctlr[FL_PG]) begin
			mreq <= 1; mwe <= 0; maddr <= {16'd0, ua[15:2], 2'b00};
			st <= S_PG_WAIT;
		end else st <= S_DONE;
	end
	S_BUF_RD: st <= S_BUF_WR;           // pb_rd arrives
	S_BUF_WR: begin
		pb_we <= 1; pb_wa <= pb_ra_q;
		pb_wd <= (pb_word & ~lanes) | (u_data & lanes);
		pdef[pb_ra_q] <= 0;
		st <= S_DONE;
	end
	S_PG_WAIT: if (mack) begin
		mreq <= 1; mwe <= 1; maddr <= {16'd0, u_a[15:2], 2'b00};
		mwdata <= mrdata & (u_data | ~lanes);
		st <= S_PG_WAIT2;
	end
	S_PG_WAIT2: if (mack) begin
		fl_page_stb <= 1; fl_page <= u_a[15:8];
		stall <= stall + 17'd1440;
		eop <= 1;
		st <= S_DONE;
	end

	// erase e_cur..e_end (flash_erase)
	S_ER: begin
		mreq <= 1; mwe <= 1; maddr <= {16'd0, e_cur}; mwdata <= 32'hffffffff;
		st <= S_ER_WAIT;
	end
	S_ER_WAIT: if (mack) begin
		if (e_cur[7:0] == 8'hfc) begin fl_page_stb <= 1; fl_page <= e_cur[15:8]; end
		e_cur <= e_cur + 16'd4;
		if (e_cur + 16'd4 == e_end) begin ack <= 1; st <= S_IDLE; end
		else st <= S_ER;
	end

	// program a page from the buffer
	S_PP: begin
		pb_ra <= p_w;                      // the buffer word, a clock to read
		st <= S_PP_BUF;
	end
	S_PP_BUF: begin
		mreq <= 1; mwe <= 0; maddr <= {16'd0, p_base[15:8], p_w, 2'b00};
		st <= S_PP_WAIT;
	end
	S_PP_WAIT: if (mack) begin
		mreq <= 1; mwe <= 1; maddr <= {16'd0, p_base[15:8], p_w, 2'b00};
		mwdata <= mrdata & pb_word;
		st <= S_PP_WAIT2;
	end
	S_PP_WAIT2: if (mack) begin
		p_w <= p_w + 6'd1;
		if (p_w == 6'd63) begin
			fl_page_stb <= 1; fl_page <= p_base[15:8];
			ack <= 1; st <= S_IDLE;
		end else st <= S_PP;
	end

	S_DONE: begin
		if (s_split && s_i + 3'd1 < s_size) begin
			s_i <= s_i + 3'd1;
			st <= S_UNIT;
		end else begin
			ack <= 1;
			st <= S_IDLE;
		end
	end
	default: st <= S_IDLE;
	endcase
end

endmodule
