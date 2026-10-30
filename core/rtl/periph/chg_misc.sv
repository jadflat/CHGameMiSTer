// Everything on the bus that no other module answers: the system area
// (ESIG, chip id, option bytes), the USARTs, the USB block's backing, the
// generic io[] backing, and plain nothing (reads 0, writes dropped).
//
// Translates sys_read, the USART, USB (0x4002_3400) and default cases of
// bus_read_slow / bus_write_slow in emu/src/bus.c, and their split of a
// misaligned access below 0x4000_0000 into bytes.
//
// chg_soc sends here every access that is not flash, SRAM, the PFIC
// (0xE000_E000..EFFF), SysTick, or one of the 1 KB windows of GPIO A/B/C,
// AFIO, RCC, SPI1, DMA1, TIM1/2/3, the flash controller or the ADC.
//
//   0x1FFF_0000..0x1FFF_FFFF  sys_read: ESIG flash size 62 KB, the unique
//                id, option bytes (erased pairs), the chip id; 0xFFFFFFFF
//                elsewhere. A misaligned access is a byte at a time, each
//                from its own word (bus_read_slow's split). Writes: nothing
//   USART1/2/3/4 (0x4001_3800, 0x4000_4400, 0x4800, 0x4C00): STATR reads
//                0xC0 (always ready), the rest 0; a write to DATAR gives
//                serial_stb with the byte (the C collects lines for printf)
//   USB (0x4002_3400): m->usb, 1 KB of backing: a block RAM here
//   io[]         m->io: 0x30000 bytes of backing for 0x4000_0000..
//                0x4002_FFFF that nothing models. CHix and STARLIFT touch
//                none of it in practice. Kept here as IO_WORDS
//                words, each taken by the first write to its address: a
//                program that writes more distinct words than that loses
//                the writes to the later ones (they read 0)
//   anything else  reads 0, writes do nothing
//
// The C writes a store's bytes at (word offset + (addr & 3)) onwards, so a
// misaligned store runs on into the next word, and reads take the word and
// shift it; the same here, a byte a clock. Bytes the C would write past the
// end of m->usb (into the PFIC's state: a 4-byte store at 0x4002_37FD..FF)
// are dropped here.

module chg_misc
#(
	parameter IO_WORDS = 16
)
(
	input             clk,
	input             reset,        // chg_reset: bus_reset clears io[] and usb[]
	input             ce,           // not used

	input             req,
	input             we,
	input      [31:0] addr,
	input       [2:0] size,
	input      [31:0] wdata,
	output reg [31:0] rdata,
	output reg        ack,

	output reg        serial_stb,
	output reg  [7:0] serial_byte
);

// ---------------------------------------------------------------------------
// sys_read
// ---------------------------------------------------------------------------

function [31:0] sys_word(input [31:0] w);
	begin
		if (w[31:16] != 16'h1FFF) sys_word = 32'd0;        // below 0x4000_0000, not memory: 0
		else if (w == 32'h1FFFF7E0) sys_word = 32'hFFFF003E;
		else if (w == 32'h1FFFF7E8) sys_word = 32'h43484731;
		else if (w == 32'h1FFFF7EC) sys_word = 32'h0035E3A1;
		else if (w == 32'h1FFFF7F0) sys_word = 32'h20260930;
		else if (w >= 32'h1FFFF800 && w < 32'h1FFFF840) sys_word = 32'h00FF00FF;
		else if (w == 32'h1FFFF704) sys_word = 32'h03510601;
		else sys_word = 32'hFFFFFFFF;
	end
endfunction

function [7:0] sys_byte(input [31:0] a);
	reg [31:0] w;
	begin
		w = sys_word({a[31:2], 2'b00});
		sys_byte = w[{a[1:0], 3'b000} +: 8];
	end
endfunction

wire [31:0] a1 = addr + 32'd1, a2 = addr + 32'd2, a3 = addr + 32'd3;
wire [31:0] low_v = {(size == 3'd4) ? sys_byte(a3) : 8'd0, (size == 3'd4) ? sys_byte(a2) : 8'd0,
                     (size != 3'd1) ? sys_byte(a1) : 8'd0, sys_byte(addr)};

// ---------------------------------------------------------------------------
// windows
// ---------------------------------------------------------------------------

wire        in_per = addr[31:18] == 14'h1000 && addr[17:16] != 2'b11;   // 0x4000_0000..0x4002_FFFF
wire [21:0] win    = {addr[31:10]};
wire        is_usart = in_per && (win == 22'h10004E || win == 22'h100011 || win == 22'h100012 ||
                                  win == 22'h100013);
wire        is_usb   = in_per && win == 22'h10008D;
wire        is_io    = in_per && !is_usart && !is_usb;
wire  [4:0] sh = {addr[1:0], 3'b000};

function [31:0] lane_r(input [31:0] x0, input [4:0] s, input [2:0] sz);
	reg [31:0] x;
	begin
		x = x0 >> s;
		if (sz == 3'd1) x = x & 32'h000000ff;
		else if (sz == 3'd2) x = x & 32'h0000ffff;
		lane_r = x;
	end
endfunction

// USB backing: 256 words, a valid bit each (reset leaves it all zero)
reg [31:0] usb[0:255];
reg [255:0] usb_ok;
reg  [7:0] ub_a;
reg [31:0] ub_q;
reg        ub_we;
reg [31:0] ub_d;
always @(posedge clk) begin
	if (ub_we) usb[ub_a] <= ub_d;
	ub_q <= usb[ub_a];
end
reg [7:0] ub_a_q;
always @(posedge clk) ub_a_q <= ub_a;
wire [31:0] ub_word = usb_ok[ub_a_q] ? ub_q : 32'd0;

// io[] backing: IO_WORDS words, each with the word address it holds
reg [15:0] io_tag[0:IO_WORDS-1];
reg [31:0] io_dat[0:IO_WORDS-1];
reg [IO_WORDS-1:0] io_ok;

// the word at io index w: hit, entry, a free entry
function integer io_find(input [15:0] w);
	integer j;
	begin
		io_find = -1;
		for (j = 0; j < IO_WORDS; j = j + 1)
			if (io_ok[j] && io_tag[j] == w) io_find = j;
	end
endfunction
function integer io_free(input integer dummy);
	integer j;
	begin
		io_free = -1;
		for (j = IO_WORDS - 1; j >= 0; j = j - 1)
			if (!io_ok[j]) io_free = j;
	end
endfunction

wire [15:0] io_w = addr[17:2];
integer     io_hit;
always @* io_hit = io_find(io_w);
wire [31:0] io_rword = (io_hit >= 0) ? io_dat[io_hit[3:0]] : 32'd0;

// ---------------------------------------------------------------------------
// the bus
// ---------------------------------------------------------------------------

localparam S_IDLE = 0, S_URD = 1, S_UBYTE = 2, S_UBRD = 3, S_UBWR = 4, S_IOBYTE = 5, S_IORD = 6, S_IOB2 = 7;
reg  [2:0] st;
reg [17:0] b_off;               // the byte being written, as an offset from 0x4000_0000
reg  [2:0] b_n;                 // bytes left
reg [31:0] b_data;
reg  [2:0] r_size;
reg  [4:0] r_sh;
reg        io_hv, e_v, f_v;
reg  [3:0] io_hi, e_i, f_i;

// the io[] entry of the byte being stored, or a free one
wire [15:0] bw = b_off[17:2];
integer e, f;
always @* begin
	e = io_find(bw);
	f = io_free(0);
end

always @(posedge clk) begin
	ack <= 0;
	ub_we <= 0;
	serial_stb <= 0;
	if (reset) begin
		usb_ok <= 0;
		io_ok <= 0;
		st <= S_IDLE;
	end else case (st)
	S_IDLE: if (req) begin
		r_size <= size; r_sh <= sh;
		if (!we) begin
			if (addr[31:30] == 2'b00) begin rdata <= low_v; ack <= 1; end
			else if (is_usart) begin rdata <= lane_r((addr[9:2] == 8'd0) ? 32'hC0 : 32'd0, sh, size); ack <= 1; end
			else if (is_usb) begin ub_a <= addr[9:2]; b_n <= 0; st <= S_URD; end
			else if (is_io) begin io_hv <= io_hit >= 0; io_hi <= io_hit[3:0]; st <= S_IORD; end
			else begin rdata <= 0; ack <= 1; end
		end else begin
			b_off <= addr[17:0];
			b_n <= size;
			b_data <= wdata;
			if (is_usart) begin
				if (addr[9:2] == 8'd1) begin serial_stb <= 1; serial_byte <= wdata[7:0]; end
				ack <= 1;
			end else if (is_usb) st <= S_UBYTE;
			else if (is_io) st <= S_IOBYTE;
			else ack <= 1;
		end
	end
	// an io[] read: the entry was found the clock before (96 MHz)
	S_IORD: begin
		rdata <= lane_r(io_hv ? io_dat[io_hi] : 32'd0, r_sh, r_size);
		ack <= 1;
		st <= S_IDLE;
	end
	S_URD: st <= S_UBRD;                // wait for the word; S_UBRD shared below
	S_UBRD: begin
		if (b_n == 3'd0) begin
			// a read: the word arrived
			rdata <= lane_r(ub_word, r_sh, r_size);
			ack <= 1;
			st <= S_IDLE;
		end else
			st <= S_UBWR;
	end
	// USB store, a byte at a time: read the word, put the byte in
	S_UBYTE: begin
		if (b_n == 3'd0) begin ack <= 1; st <= S_IDLE; end
		else if (b_off[17:10] != 8'h8D) begin
			// run past the end of usb[]: dropped
			ack <= 1; st <= S_IDLE;
		end else begin
			ub_a <= b_off[9:2];
			st <= S_URD;
		end
	end
	S_UBWR: begin
		ub_we <= 1;
		ub_d <= ub_word;
		ub_d[{b_off[1:0], 3'b000} +: 8] <= b_data[7:0];
		usb_ok[ub_a_q] <= 1;
		b_off <= b_off + 18'd1;
		b_data <= b_data >> 8;
		b_n <= b_n - 3'd1;
		st <= S_UBYTE;
	end
	// io[] store, a byte every two clocks: its entry (or a free one) found, then written
	S_IOBYTE: begin
		if (b_n == 3'd0 || b_off[17:16] == 2'b11) begin ack <= 1; st <= S_IDLE; end   // past io[]: dropped
		else begin
			e_v <= e >= 0; e_i <= e[3:0];
			f_v <= f >= 0; f_i <= f[3:0];
			st <= S_IOB2;
		end
	end
	S_IOB2: begin
		if (e_v)
			io_dat[e_i][{b_off[1:0], 3'b000} +: 8] <= b_data[7:0];
		else if (f_v) begin
			io_ok[f_i] <= 1;
			io_tag[f_i] <= bw;
			io_dat[f_i] <= {24'd0, b_data[7:0]} << {b_off[1:0], 3'b000};
		end
		b_off <= b_off + 18'd1;
		b_data <= b_data >> 8;
		b_n <= b_n - 3'd1;
		st <= S_IOBYTE;
	end
	default: st <= S_IDLE;
	endcase
end

endmodule
