// One instruction decoded, as cpu_decode, decode32 and decode16 in
// emu/src/rv32.c do it: the 16-bit instructions (RVC and WCH's XW byte and
// halfword loads and stores) are expanded into the 32-bit operations they
// stand for, and every field is set exactly as the C sets it, unused ones
// included (rs1 and rs2 of an instruction that has none are 0; a CSRRxI
// keeps its 5-bit immediate in rs1), because the divider check in cpu_exec
// compares rs1 and rs2 whatever the instruction is.
//
// Combinational. raw is the instruction's first 32 bits (only the low 16
// count for a 16-bit one); pc is used for wend, the 32-bit word the
// instruction's last byte is in (the prefetcher waits for it).

module chg_cpu_decode
(
	input      [15:0] pc,       // pc & 0xffff
	input      [31:0] raw,
	output reg  [6:0] op,
	output reg  [5:0] rd,       // 32 when the instruction writes x0 (a sink, as x[32] in the C)
	output reg  [4:0] rs1,
	output reg  [4:0] rs2,
	output reg [31:0] imm,
	output            len4,     // 4 bytes long, else 2
	output reg  [2:0] cost,     // cycles once fetched: 1, or LOAD/STORE_CYCLES
	output     [14:0] wend      // ((pc & 0xffff) + len - 1) >> 2
);

`include "chg_cpu_ops.svh"

// set() of rv32.c: {op, rd, rs1, rs2, imm}, rd 0 becoming 32
function [54:0] S(input [6:0] o, input [4:0] d, input [4:0] a, input [4:0] b, input [31:0] i);
	S = {o, (d == 5'd0) ? 6'd32 : {1'b0, d}, a, b, i};
endfunction

assign len4 = (raw[1:0] == 2'b11);
wire [16:0] wsum = {1'b0, pc} + (len4 ? 17'd3 : 17'd1);
assign wend = wsum[16:2];

// decode32
wire [31:0] w = raw;
wire  [4:0] f_rd = w[11:7], f_rs1 = w[19:15], f_rs2 = w[24:20];
wire  [2:0] f3 = w[14:12];
wire  [6:0] f7 = w[31:25];
wire [31:0] immI = {{20{w[31]}}, w[31:20]};
wire [31:0] immS = {{20{w[31]}}, w[31:25], w[11:7]};
wire [31:0] immB = {{19{w[31]}}, w[31], w[7], w[30:25], w[11:8], 1'b0};
wire [31:0] immJ = {{11{w[31]}}, w[31], w[19:12], w[20], w[30:21], 1'b0};

// decode16
wire [15:0] h = raw[15:0];
wire  [2:0] c3 = h[15:13];
wire  [4:0] rdp = {2'b01, h[4:2]};      // rd'/rs2'
wire  [4:0] rs1p = {2'b01, h[9:7]};     // rs1'/rd'
wire  [4:0] r = h[11:7];
wire  [4:0] r2 = h[6:2];
wire [31:0] imm6 = {{26{h[12]}}, h[12], h[6:2]};
wire [31:0] addi4 = {22'b0, h[10:7], h[12:11], h[5], h[6], 2'b0};
wire [31:0] off_bu = {27'b0, h[11:10], h[6], h[5], h[12]};          // c.lbu / c.sb
wire [31:0] off_w = {25'b0, h[5], h[12:10], h[6], 2'b0};              // c.lw / c.sw
wire [31:0] off_hu = {26'b0, h[12], h[11:10], h[6], h[5], 1'b0};      // c.lhu / c.sh
wire [31:0] boff = {28'b0, h[10:7]};
wire [31:0] hoff = {27'b0, h[7], h[10:8], 1'b0};
wire [31:0] offj = {{20{h[12]}}, h[12], h[8], h[10:9], h[6], h[7], h[2], h[11], h[5:3], 1'b0};
wire [31:0] off16sp = {{22{h[12]}}, h[12], h[4:3], h[5], h[2], h[6], 4'b0};
wire [31:0] offb = {{23{h[12]}}, h[12], h[6:5], h[2], h[11:10], h[4:3], 1'b0};
wire  [5:0] sh6 = {h[12], h[6:2]};
wire [31:0] off_lwsp = {24'b0, h[3:2], h[12], h[6:4], 2'b0};
wire [31:0] off_swsp = {24'b0, h[8:7], h[12:9], 2'b0};

reg [54:0] sd;

always @* begin
	if (len4) begin
		sd = S(OP_ILLEGAL, 0, 0, 0, w);
		case (w[6:0])
		7'h37: sd = S(OP_LUI, f_rd, 0, 0, {w[31:12], 12'b0});
		7'h17: sd = S(OP_AUIPC, f_rd, 0, 0, {w[31:12], 12'b0});
		7'h6f: sd = S(OP_JAL, f_rd, 0, 0, immJ);
		7'h67: if (f3 == 0) sd = S(OP_JALR, f_rd, f_rs1, 0, immI);
		7'h63:
			case (f3)
			3'd0: sd = S(OP_BEQ, 0, f_rs1, f_rs2, immB);
			3'd1: sd = S(OP_BNE, 0, f_rs1, f_rs2, immB);
			3'd4: sd = S(OP_BLT, 0, f_rs1, f_rs2, immB);
			3'd5: sd = S(OP_BGE, 0, f_rs1, f_rs2, immB);
			3'd6: sd = S(OP_BLTU, 0, f_rs1, f_rs2, immB);
			3'd7: sd = S(OP_BGEU, 0, f_rs1, f_rs2, immB);
			default: ;
			endcase
		7'h03:
			case (f3)
			3'd0: sd = S(OP_LB, f_rd, f_rs1, 0, immI);
			3'd1: sd = S(OP_LH, f_rd, f_rs1, 0, immI);
			3'd2: sd = S(OP_LW, f_rd, f_rs1, 0, immI);
			3'd4: sd = S(OP_LBU, f_rd, f_rs1, 0, immI);
			3'd5: sd = S(OP_LHU, f_rd, f_rs1, 0, immI);
			default: ;
			endcase
		7'h23:
			case (f3)
			3'd0: sd = S(OP_SB, 0, f_rs1, f_rs2, immS);
			3'd1: sd = S(OP_SH, 0, f_rs1, f_rs2, immS);
			3'd2: sd = S(OP_SW, 0, f_rs1, f_rs2, immS);
			default: ;
			endcase
		7'h13:
			case (f3)
			3'd0: sd = S(OP_ADDI, f_rd, f_rs1, 0, immI);
			3'd2: sd = S(OP_SLTI, f_rd, f_rs1, 0, immI);
			3'd3: sd = S(OP_SLTIU, f_rd, f_rs1, 0, immI);
			3'd4: sd = S(OP_XORI, f_rd, f_rs1, 0, immI);
			3'd6: sd = S(OP_ORI, f_rd, f_rs1, 0, immI);
			3'd7: sd = S(OP_ANDI, f_rd, f_rs1, 0, immI);
			3'd1: if (f7 == 0) sd = S(OP_SLLI, f_rd, f_rs1, 0, {27'b0, f_rs2});
			3'd5:
				if (f7 == 0) sd = S(OP_SRLI, f_rd, f_rs1, 0, {27'b0, f_rs2});
				else if (f7 == 7'h20) sd = S(OP_SRAI, f_rd, f_rs1, 0, {27'b0, f_rs2});
			endcase
		7'h33:
			if (f7 == 0) begin
				case (f3)
				3'd0: sd = S(OP_ADD, f_rd, f_rs1, f_rs2, 0);
				3'd1: sd = S(OP_SLL, f_rd, f_rs1, f_rs2, 0);
				3'd2: sd = S(OP_SLT, f_rd, f_rs1, f_rs2, 0);
				3'd3: sd = S(OP_SLTU, f_rd, f_rs1, f_rs2, 0);
				3'd4: sd = S(OP_XOR, f_rd, f_rs1, f_rs2, 0);
				3'd5: sd = S(OP_SRL, f_rd, f_rs1, f_rs2, 0);
				3'd6: sd = S(OP_OR, f_rd, f_rs1, f_rs2, 0);
				3'd7: sd = S(OP_AND, f_rd, f_rs1, f_rs2, 0);
				endcase
			end else if (f7 == 7'h20) begin
				if (f3 == 0) sd = S(OP_SUB, f_rd, f_rs1, f_rs2, 0);
				else if (f3 == 5) sd = S(OP_SRA, f_rd, f_rs1, f_rs2, 0);
			end else if (f7 == 7'h01) begin
				case (f3)
				3'd0: sd = S(OP_MUL, f_rd, f_rs1, f_rs2, 0);
				3'd1: sd = S(OP_MULH, f_rd, f_rs1, f_rs2, 0);
				3'd2: sd = S(OP_MULHSU, f_rd, f_rs1, f_rs2, 0);
				3'd3: sd = S(OP_MULHU, f_rd, f_rs1, f_rs2, 0);
				3'd4: sd = S(OP_DIV, f_rd, f_rs1, f_rs2, 0);
				3'd5: sd = S(OP_DIVU, f_rd, f_rs1, f_rs2, 0);
				3'd6: sd = S(OP_REM, f_rd, f_rs1, f_rs2, 0);
				3'd7: sd = S(OP_REMU, f_rd, f_rs1, f_rs2, 0);
				endcase
			end
		7'h2f:
			if (f3 == 2)
				case (f7[6:2])
				5'h02: if (f_rs2 == 0) sd = S(OP_LR, f_rd, f_rs1, 0, 0);
				5'h03: sd = S(OP_SC, f_rd, f_rs1, f_rs2, 0);
				5'h01: sd = S(OP_AMOSWAP, f_rd, f_rs1, f_rs2, 0);
				5'h00: sd = S(OP_AMOADD, f_rd, f_rs1, f_rs2, 0);
				5'h04: sd = S(OP_AMOXOR, f_rd, f_rs1, f_rs2, 0);
				5'h0c: sd = S(OP_AMOAND, f_rd, f_rs1, f_rs2, 0);
				5'h08: sd = S(OP_AMOOR, f_rd, f_rs1, f_rs2, 0);
				5'h10: sd = S(OP_AMOMIN, f_rd, f_rs1, f_rs2, 0);
				5'h14: sd = S(OP_AMOMAX, f_rd, f_rs1, f_rs2, 0);
				5'h18: sd = S(OP_AMOMINU, f_rd, f_rs1, f_rs2, 0);
				5'h1c: sd = S(OP_AMOMAXU, f_rd, f_rs1, f_rs2, 0);
				default: ;
				endcase
		7'h0f: sd = S(OP_FENCE, 0, 0, 0, 0);
		7'h73:
			if (f3 == 0) begin
				if (w == 32'h00000073) sd = S(OP_ECALL, 0, 0, 0, 0);
				else if (w == 32'h00100073) sd = S(OP_EBREAK, 0, 0, 0, 0);
				else if (w == 32'h30200073) sd = S(OP_MRET, 0, 0, 0, 0);
				else if (w == 32'h10500073) sd = S(OP_WFI, 0, 0, 0, 0);
			end else if (f3 != 4) begin
				case (f3)
				3'd1: sd = S(OP_CSRRW, f_rd, f_rs1, 0, {20'b0, w[31:20]});
				3'd2: sd = S(OP_CSRRS, f_rd, f_rs1, 0, {20'b0, w[31:20]});
				3'd3: sd = S(OP_CSRRC, f_rd, f_rs1, 0, {20'b0, w[31:20]});
				3'd5: sd = S(OP_CSRRWI, f_rd, f_rs1, 0, {20'b0, w[31:20]});
				3'd6: sd = S(OP_CSRRSI, f_rd, f_rs1, 0, {20'b0, w[31:20]});
				3'd7: sd = S(OP_CSRRCI, f_rd, f_rs1, 0, {20'b0, w[31:20]});
				default: ;
				endcase
			end
		default: ;
		endcase
	end else begin
		sd = S(OP_ILLEGAL, 0, 0, 0, {16'b0, h});
		case (h[1:0])
		2'd0:
			case (c3)
			3'd0: if (addi4 != 0) sd = S(OP_ADDI, rdp, 2, 0, addi4);            // c.addi4spn
			3'd1: sd = S(OP_LBU, rdp, rs1p, 0, off_bu);                          // XW c.lbu
			3'd2: sd = S(OP_LW, rdp, rs1p, 0, off_w);                            // c.lw
			3'd4:                                                               // XW sp relative
				if (h[12:11] == 0)
					case (h[6:5])
					2'd0: sd = S(OP_LBU, rdp, 2, 0, boff);                       // c.lbusp
					2'd1: sd = S(OP_LHU, rdp, 2, 0, hoff);                       // c.lhusp
					2'd2: sd = S(OP_SB, 0, 2, rdp, boff);                        // c.sbsp
					2'd3: sd = S(OP_SH, 0, 2, rdp, hoff);                        // c.shsp
					endcase
			3'd5: sd = S(OP_SB, 0, rs1p, rdp, off_bu);                           // XW c.sb
			3'd6: sd = S(OP_SW, 0, rs1p, rdp, off_w);                            // c.sw
			default: ;
			endcase
		2'd1:
			case (c3)
			3'd0: sd = S(OP_ADDI, r, r, 0, imm6);                                // c.addi, c.nop
			3'd1: sd = S(OP_JAL, 1, 0, 0, offj);                                 // c.jal
			3'd5: sd = S(OP_JAL, 0, 0, 0, offj);                                 // c.j
			3'd2: sd = S(OP_ADDI, r, 0, 0, imm6);                                // c.li
			3'd3:
				if (r == 2) begin                                               // c.addi16sp
					if (off16sp != 0) sd = S(OP_ADDI, 2, 2, 0, off16sp);
				end else if (imm6 != 0) begin                                   // c.lui
					sd = S(OP_LUI, r, 0, 0, {imm6[19:0], 12'b0});
				end
			3'd4:
				case (h[11:10])
				2'd0: if (sh6 < 32) sd = S(OP_SRLI, rs1p, rs1p, 0, {26'b0, sh6});
				2'd1: if (sh6 < 32) sd = S(OP_SRAI, rs1p, rs1p, 0, {26'b0, sh6});
				2'd2: sd = S(OP_ANDI, rs1p, rs1p, 0, imm6);
				2'd3:
					if (h[12] == 0)
						case (h[6:5])
						2'd0: sd = S(OP_SUB, rs1p, rs1p, rdp, 0);
						2'd1: sd = S(OP_XOR, rs1p, rs1p, rdp, 0);
						2'd2: sd = S(OP_OR, rs1p, rs1p, rdp, 0);
						2'd3: sd = S(OP_AND, rs1p, rs1p, rdp, 0);
						endcase
				endcase
			3'd6: sd = S(OP_BEQ, 0, rs1p, 0, offb);                              // c.beqz
			3'd7: sd = S(OP_BNE, 0, rs1p, 0, offb);                              // c.bnez
			endcase
		2'd2:
			case (c3)
			3'd0: if (sh6 < 32) sd = S(OP_SLLI, r, r, 0, {26'b0, sh6});          // c.slli
			3'd1: sd = S(OP_LHU, rdp, rs1p, 0, off_hu);                          // XW c.lhu
			3'd2: if (r != 0) sd = S(OP_LW, r, 2, 0, off_lwsp);                  // c.lwsp
			3'd4:
				if (h[12] == 0) begin
					if (r2 == 0) begin
						if (r != 0) sd = S(OP_JALR, 0, r, 0, 0);                 // c.jr
					end else sd = S(OP_ADD, r, 0, r2, 0);                        // c.mv
				end else begin
					if (r == 0 && r2 == 0) sd = S(OP_EBREAK, 0, 0, 0, 0);        // c.ebreak
					else if (r2 == 0) sd = S(OP_JALR, 1, r, 0, 0);               // c.jalr
					else sd = S(OP_ADD, r, r, r2, 0);                            // c.add
				end
			3'd5: sd = S(OP_SH, 0, rs1p, rdp, off_hu);                           // XW c.sh
			3'd6: sd = S(OP_SW, 0, 2, r2, off_swsp);                             // c.swsp
			default: ;
			endcase
		default: ;
		endcase
	end
	{op, rd, rs1, rs2, imm} = sd;
	case (op)
	OP_LB, OP_LH, OP_LW, OP_LBU, OP_LHU, OP_LR: cost = 3'd2;                  // LOAD_CYCLES
	OP_SB, OP_SH, OP_SW: cost = 3'd2;                                         // STORE_CYCLES
	OP_AMOSWAP, OP_AMOADD, OP_AMOXOR, OP_AMOAND, OP_AMOOR,
	OP_AMOMIN, OP_AMOMAX, OP_AMOMINU, OP_AMOMAXU, OP_SC: cost = 3'd4;
	default: cost = 3'd1;
	endcase
end

endmodule
