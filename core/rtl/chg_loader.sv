// Puts a .CHG game into the flash as chg_load_program (emu/src/loader.c)
// does for a package: the program flash erased to 0xFF from 0x3000, the
// payload at 0x3000, and the metadata page at 0xF700 that the bootloader
// checks (magic "CHGM", version 1, length, CRC-32). The bootloader itself is
// the flash's initial contents (flash_init.hex) and is never written here.
//
// The package's header already carries the payload's length (offset 16) and
// CRC-32 (offset 20), and the metadata page wants exactly those, so nothing is
// computed. The header's own checks are a subset of chg_package's: magic
// "CHG1", format 1 with a 512-byte header, target "CX35" at 0x3000/0xF700, a
// length that fits. Its header CRC and the payload CRC are not checked (the
// bootloader checks the payload's CRC itself on every start).
//
// The board is held in reset from the start of the download until 'done'.

module chg_loader
(
	input             clk,
	input             reset,

	// MiSTer's hps_io download, already filtered to this file's index
	input             download_in,
	input             wr_in,
	input      [26:0] addr_in,
	input       [7:0] din_in,
	output            wait_req,     // to ioctl_wait: hold the ARM while erasing

	// byte writes into the flash
	output reg        fl_we,
	output reg [15:0] fl_addr,
	output reg  [7:0] fl_data,

	output reg        busy,         // hold the board in reset
	output reg        bad           // the file was not a CHGame package; the program flash is left erased
);

localparam APP = 16'h3000, META = 16'hF700;

// hps_io's signals, a clock late (96 MHz); its writes are many clocks apart
reg        download, wr;
reg [26:0] addr;
reg  [7:0] din;
always @(posedge clk) begin download <= download_in; wr <= wr_in; addr <= addr_in; din <= din_in; end

typedef enum logic [1:0] { IDLE, ERASE, LOAD, META_WR } state_t;
state_t     st;
reg  [16:0] pos;
reg [31:0]  len, crc;
reg [31:0]  w0, w2, w3;         // header words 0, 2, 3 (magic, target); w1 is format and size
reg [15:0]  fmt, hsize;
reg         was_dl;

// also from the first clock of a download, which the registers above see a clock late
assign wait_req = st == ERASE || (st == IDLE && (download_in || download));

wire hdr_ok = w0 == 32'h31474843 && fmt == 16'd1 && hsize == 16'd512 && w2 == 32'h35335843 &&
              w3 == 32'h003000F7 && len != 0 && len[1:0] == 0 && len <= 32'(META - APP);

// the metadata page's first 32 bytes: magic, version, length, crc, then zeros
function [7:0] meta_byte(input [4:0] i);
	reg [31:0] w;
	case (i[4:2])
		3'd0: w = 32'h4D474843;
		3'd1: w = 32'd1;
		3'd2: w = len;
		3'd3: w = crc;
		default: w = 0;
	endcase
	meta_byte = w[8*i[1:0] +: 8];
endfunction

always @(posedge clk) begin
	fl_we  <= 0;
	was_dl <= download;
	if (reset) begin
		st <= IDLE; busy <= 0; bad <= 0;
	end
	else case (st)
		IDLE: if (download && !was_dl) begin
			st <= ERASE; pos <= APP; busy <= 1; bad <= 0;
		end
		ERASE: begin
			// 0x3000-0xFFFF to 0xFF, a byte a clock (53 K clocks, under a millisecond)
			fl_we <= 1; fl_addr <= pos[15:0]; fl_data <= 8'hFF;
			if (pos == 17'hFFFF) st <= LOAD;
			pos <= pos + 1'd1;
		end
		LOAD: begin
			if (wr) begin
				if (addr < 27'd512) begin
					case (addr[8:0])
						9'd0, 9'd1, 9'd2, 9'd3:     w0[8*addr[1:0] +: 8] <= din;
						9'd4, 9'd5:                 fmt[8*addr[0] +: 8] <= din;
						9'd6, 9'd7:                 hsize[8*addr[0] +: 8] <= din;
						9'd8, 9'd9, 9'd10, 9'd11:   w2[8*addr[1:0] +: 8] <= din;
						9'd12, 9'd13, 9'd14, 9'd15: w3[8*addr[1:0] +: 8] <= din;
						9'd16, 9'd17, 9'd18, 9'd19: len[8*addr[1:0] +: 8] <= din;
						9'd20, 9'd21, 9'd22, 9'd23: crc[8*addr[1:0] +: 8] <= din;
						default: ;
					endcase
				end
				else if (hdr_ok && addr - 27'd512 < 27'(len)) begin
					fl_we <= 1; fl_addr <= 16'(addr - 27'd512 + APP); fl_data <= din;
				end
			end
			if (!download) begin
				if (hdr_ok) begin st <= META_WR; pos <= 0; end
				else begin st <= IDLE; busy <= 1; bad <= 1; end      // stays in reset
			end
		end
		META_WR: begin
			fl_we <= 1; fl_addr <= META + pos[4:0]; fl_data <= meta_byte(pos[4:0]);
			if (pos == 17'd31) begin st <= IDLE; busy <= 0; end
			pos <= pos + 1'd1;
		end
	endcase
end

endmodule
