// The operation numbers of emu/src/rv32.c (enum OP_*), in the same order, so
// that a trace from either side names the same thing. Included inside the
// modules that need them.

localparam [6:0] OP_NONE = 7'd0,
	OP_LUI = 7'd1, OP_AUIPC = 7'd2, OP_JAL = 7'd3, OP_JALR = 7'd4,
	OP_BEQ = 7'd5, OP_BNE = 7'd6, OP_BLT = 7'd7, OP_BGE = 7'd8, OP_BLTU = 7'd9, OP_BGEU = 7'd10,
	OP_LB = 7'd11, OP_LH = 7'd12, OP_LW = 7'd13, OP_LBU = 7'd14, OP_LHU = 7'd15,
	OP_SB = 7'd16, OP_SH = 7'd17, OP_SW = 7'd18,
	OP_ADDI = 7'd19, OP_SLTI = 7'd20, OP_SLTIU = 7'd21, OP_XORI = 7'd22, OP_ORI = 7'd23,
	OP_ANDI = 7'd24, OP_SLLI = 7'd25, OP_SRLI = 7'd26, OP_SRAI = 7'd27,
	OP_ADD = 7'd28, OP_SUB = 7'd29, OP_SLL = 7'd30, OP_SLT = 7'd31, OP_SLTU = 7'd32,
	OP_XOR = 7'd33, OP_SRL = 7'd34, OP_SRA = 7'd35, OP_OR = 7'd36, OP_AND = 7'd37,
	OP_MUL = 7'd38, OP_MULH = 7'd39, OP_MULHSU = 7'd40, OP_MULHU = 7'd41,
	OP_DIV = 7'd42, OP_DIVU = 7'd43, OP_REM = 7'd44, OP_REMU = 7'd45,
	OP_LR = 7'd46, OP_SC = 7'd47, OP_AMOSWAP = 7'd48, OP_AMOADD = 7'd49, OP_AMOXOR = 7'd50,
	OP_AMOAND = 7'd51, OP_AMOOR = 7'd52, OP_AMOMIN = 7'd53, OP_AMOMAX = 7'd54,
	OP_AMOMINU = 7'd55, OP_AMOMAXU = 7'd56,
	OP_FENCE = 7'd57, OP_ECALL = 7'd58, OP_EBREAK = 7'd59, OP_MRET = 7'd60, OP_WFI = 7'd61,
	OP_CSRRW = 7'd62, OP_CSRRS = 7'd63, OP_CSRRC = 7'd64, OP_CSRRWI = 7'd65, OP_CSRRSI = 7'd66,
	OP_CSRRCI = 7'd67,
	OP_ILLEGAL = 7'd68;
