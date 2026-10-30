// Random programs for tb_cpu.cpp (included by it): what the games do not reach.
//
//   cpu.exe --random seed [seconds]
//
// The program, with no bootloader (entry 0x3000):
//   0x3000  init: pointer registers (x2 stack 0x20003000, x8 data 0x20002000,
//           x9 flash 0x3000, x4 the PFIC, x3 SysTick), mtvec = the handler
//   0x3080  the trap handler: an exception returns past the instruction (2 or 4
//           bytes, from its low bits); an interrupt is disabled in the PFIC (and its
//           pending bit cleared) and returned from
//   0x3100  block F, random instructions in flash, ending with a jump to block R
//   0x20000400  block R, random instructions in SRAM, ending with a jump to F
// Random instructions: every RV32IMAC + XW operation the decoder knows, random
// 16-bit words (whatever cpu_decode makes of them, illegal ones included),
// misaligned loads and stores, loads from flash (from RAM code too), AMOs, LR/SC,
// CSR reads and writes (MIE, INTSYSCR: HPE and nesting), ECALL, EBREAK, illegal
// words, PFIC and SysTick registers poked through x4/x3 (interrupts, priorities,
// the threshold, SWIE), and accesses through random registers to anywhere.
// Branches and jumps only go forward, to an instruction boundary, so there is
// no endless loop; registers x2, x3, x4, x8, x9 are never written by them.


static uint32_t rs_state = 1;
static uint32_t rr() { rs_state ^= rs_state << 13; rs_state ^= rs_state >> 17; rs_state ^= rs_state << 5; return rs_state; }
static uint32_t rn(uint32_t n) { return rr() % n; }

static uint32_t encR(int f7, int rs2, int rs1, int f3, int rd, int opc) { return (uint32_t)f7 << 25 | rs2 << 20 | rs1 << 15 | f3 << 12 | rd << 7 | opc; }
static uint32_t encI(int imm, int rs1, int f3, int rd, int opc) { return (uint32_t)(imm & 0xfff) << 20 | rs1 << 15 | f3 << 12 | rd << 7 | opc; }
static uint32_t encS(int imm, int rs2, int rs1, int f3, int opc) { return (uint32_t)((imm >> 5) & 0x7f) << 25 | rs2 << 20 | rs1 << 15 | f3 << 12 | (imm & 31) << 7 | opc; }
static uint32_t encB(int imm, int rs2, int rs1, int f3) {
    return (uint32_t)((imm >> 12) & 1) << 31 | ((imm >> 5) & 0x3f) << 25 | rs2 << 20 | rs1 << 15 | f3 << 12 |
           ((imm >> 1) & 0xf) << 8 | ((imm >> 11) & 1) << 7 | 0x63;
}
static uint32_t encJ(int imm, int rd) {
    return (uint32_t)((imm >> 20) & 1) << 31 | ((imm >> 1) & 0x3ff) << 21 | ((imm >> 11) & 1) << 20 |
           ((imm >> 12) & 0xff) << 12 | rd << 7 | 0x6f;
}
static uint32_t encU(uint32_t imm20, int rd, int opc) { return imm20 << 12 | rd << 7 | opc; }

struct RInsn { uint32_t w; int len; int kind; int target; };   // kind 1: branch/jal to fill in
enum { K_PLAIN = 0, K_BRANCH = 1, K_JAL = 2 };

static int wreg()
{
    static const int ok[] = { 0, 1, 5, 6, 7, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24,
                              25, 26, 27, 28, 29, 30, 31 };
    return ok[rn(sizeof ok / sizeof ok[0])];
}
static int anyreg() { return (int)rn(32); }

// a base register and offset for a load or store of 'size' bytes
static void mem_base(int *rs1, int *imm, bool store)
{
    const uint32_t r = rn(100);
    if (r < 50) { *rs1 = 8; *imm = (int)rn(129) - 64; }
    else if (r < 62) { *rs1 = 2; *imm = (int)rn(129) - 64; }
    else if (r < 74) { *rs1 = 9; *imm = (int)rn(2048); }
    else if (r < 88) {
        static const int pf_rd[] = { 0, 4, 0x20, 0x24, 0x40, 0x48, 0x4c, 0x50, 0x60, 0x64, 0x100, 0x104, 0x300, 0x304, 0x400, 0x404,
                                     0x408, 0x40c, 0x410, 0x438, 0x430, 0x4fc };
        static const int pf_wr[] = { 0x100, 0x104, 0x100, 0x180, 0x184, 0x200, 0x204, 0x280, 0x40, 0x400, 0x404, 0x408,
                                     0x40c, 0x410, 0x430, 0x438, 0x4fc, 0x50, 0x60, 0x6c };
        *rs1 = 4;
        *imm = store ? pf_wr[rn(sizeof pf_wr / sizeof pf_wr[0])] : pf_rd[rn(sizeof pf_rd / sizeof pf_rd[0])];
        *imm += (int)rn(4) == 0 ? (int)rn(4) : 0;
    } else if (r < 94) { *rs1 = 3; *imm = (int)rn(6) * 4 + ((int)rn(4) == 0 ? (int)rn(4) : 0); }
    else { *rs1 = anyreg(); *imm = (int)rn(4096) - 2048; }
}

static RInsn gen_one(bool in_ram)
{
    RInsn o = { 0, 4, K_PLAIN, 0 };
    const uint32_t r = rn(1000);
    if (r < 200) {                                  // reg-imm ALU
        static const int f3s[] = { 0, 2, 3, 4, 6, 7, 1, 5 };
        const int f3 = f3s[rn(8)];
        int imm = (int)rn(4096) - 2048;
        if (f3 == 1) imm = (int)rn(32);
        if (f3 == 5) imm = (int)rn(32) | (rn(2) ? 0x400 : 0);
        o.w = encI(imm, anyreg(), f3, wreg(), 0x13);
    } else if (r < 330) {                           // reg-reg, M
        const uint32_t k = rn(10);
        int f7 = 0;
        int f3 = (int)rn(8);
        if (k < 4) f7 = 1;
        else if (k < 5 && (f3 == 0 || f3 == 5)) f7 = 0x20;
        o.w = encR(f7, anyreg(), anyreg(), f3, wreg(), 0x33);
    } else if (r < 360) {
        o.w = encU(rr() & 0xfffff, wreg(), rn(2) ? 0x37 : 0x17);
    } else if (r < 480) {                           // loads
        int rs1, imm;
        mem_base(&rs1, &imm, false);
        static const int f3s[] = { 0, 1, 2, 4, 5 };
        o.w = encI(imm, rs1, f3s[rn(5)], wreg(), 0x03);
    } else if (r < 580) {                           // stores
        int rs1, imm;
        mem_base(&rs1, &imm, true);
        o.w = encS(imm, anyreg(), rs1, (int)rn(3), 0x23);
    } else if (r < 610) {                           // AMO, LR, SC on x8 / x2 (aligned)
        static const int f5s[] = { 0x02, 0x03, 0x01, 0x00, 0x04, 0x0c, 0x08, 0x10, 0x14, 0x18, 0x1c };
        const int f5 = f5s[rn(11)];
        o.w = encR(f5 << 2 | (int)rn(4), f5 == 2 ? 0 : anyreg(), rn(2) ? 8 : 2, 2, wreg(), 0x2f);
    } else if (r < 690) {                           // branches
        static const int f3s[] = { 0, 1, 4, 5, 6, 7 };
        o.w = encB(0, anyreg(), anyreg(), f3s[rn(6)]);
        o.kind = K_BRANCH;
    } else if (r < 705) {
        o.w = encJ(0, wreg());
        o.kind = K_JAL;
    } else if (r < 760) {                           // CSRs
        static const int rw[] = { 0x340, 0x342, 0x343, 0x800, 0x804, 0xBC0, 0x7C0, 0x7C1, 0x7A0, 0x301, 0xF11, 0xB00 };
        static const int ro[] = { 0x300, 0x301, 0x305, 0x340, 0x341, 0x342, 0x343, 0x800, 0x804, 0xBC0, 0xF11, 0xF12,
                                  0xF13, 0xF14, 0xB00, 0xC00, 0xB80, 0xC80, 0x7A5, 0x7C0, 0x7C1, 0x123 };
        const uint32_t k = rn(10);
        if (k < 4) o.w = encI(ro[rn(sizeof ro / sizeof ro[0])], 0, 2, wreg(), 0x73);            // csrr
        else if (k < 6) {
            // mstatus / 0x800: set or clear MIE, MPIE
            const int csr = rn(2) ? 0x300 : 0x800;
            o.w = encI(csr, (int)(rn(2) ? 8 : 0) | (int)rn(2), rn(2) ? 6 : 7, wreg(), 0x73);   // csrrsi/csrrci
        } else {
            static const int f3s[] = { 1, 2, 3, 5, 6, 7 };
            o.w = encI(rw[rn(sizeof rw / sizeof rw[0])], anyreg(), f3s[rn(6)], wreg(), 0x73);
        }
    } else if (r < 770) {
        static const uint32_t sys[] = { 0x00000073, 0x00100073, 0x0000000f, 0x0ff0000f, 0xffffffff, 0x00000000,
                                        0x0000507f, 0x02000033 | (7u << 12) };
        o.w = sys[rn(sizeof sys / sizeof sys[0])];
        if ((o.w & 3) != 3) o.len = 2;
    } else {                                        // a 16-bit word, whatever it decodes to
        for (;;) {
            uint32_t h = rr() & 0xffff;
            if ((h & 3) == 3) continue;
            ChgInsn d;
            cpu_decode(0x3100, h, &d, true);
            const int op = d.op;
            if (op == 3 || op == 4 || (op >= 5 && op <= 10)) continue;      // control flow: no
            if (op == 59) continue;                                         // c.ebreak: rare enough below
            const int rd = d.rd;
            if (rd == 2 || rd == 3 || rd == 4 || rd == 8 || rd == 9) continue;
            if (((op >= 11 && op <= 18)) && d.rs1 != 2 && d.rs1 != 8 && d.rs1 != 9 && rn(10)) continue;
            o.w = h;
            o.len = 2;
            break;
        }
        if (rn(200) == 0) o.w = 0x9002;     // c.ebreak
    }
    (void)in_ram;
    return o;
}

// lays out n random instructions at 'base' into out[], fixing branch targets
static void li_v(std::vector<RInsn> &v, int rd, uint32_t val)
{
    const uint32_t lo = val & 0xfff, hi = (val + (lo & 0x800 ? 0x1000 : 0)) >> 12;
    v.push_back({ encU(hi & 0xfffff, rd, 0x37), 4, K_PLAIN, 0 });
    v.push_back({ encI((int)lo, rd, 0, rd, 0x13), 4, K_PLAIN, 0 });
}

// sequences that make interrupts happen
static void gen_seq(std::vector<RInsn> &v)
{
    const int t = 5 + (int)rn(3);       // a scratch register (the handler's too)
    const uint32_t k = rn(100);
    if (k < 30) {
        // a PFIC register, a fresh value
        static const int offs[] = { 0x100, 0x104, 0x200, 0x204, 0x180, 0x184, 0x280, 0x284, 0x40 };
        const int off = offs[rn(sizeof offs / sizeof offs[0])];
        uint32_t val = rn(2) ? (1u << rn(32)) : rr();
        if (off == 0x40) val = rn(3) ? 0 : rn(256);
        if (off == 0x100 || off == 0x200) val |= (rn(2) ? 1u << 12 : 0) | (rn(2) ? 1u << 14 : 0);
        li_v(v, t, val);
        v.push_back({ encS(off, t, 4, 2, 0x23), 4, K_PLAIN, 0 });
    } else if (k < 50) {
        // a priority byte (of 0..63, mostly SysTick and SW)
        const int n = rn(2) ? (rn(2) ? 12 : 14) : (int)rn(64);
        li_v(v, t, rr() & 0xe0);
        v.push_back({ encS(0x400 + n, t, 4, 0, 0x23), 4, K_PLAIN, 0 });
    } else if (k < 75) {
        // SysTick: a compare value, then counting with the interrupt on
        li_v(v, t, 20 + rn(600));
        v.push_back({ encS(0x10, t, 3, 2, 0x23), 4, K_PLAIN, 0 });
        v.push_back({ encS(0x14, 0, 3, 2, 0x23), 4, K_PLAIN, 0 });
        v.push_back({ encS(0x04, 0, 3, 2, 0x23), 4, K_PLAIN, 0 });     // SR = 0
        li_v(v, t, 0x1 | 0x2 | (rn(2) ? 0x4 : 0) | 0x8 | 0x20 | (rn(4) == 0 ? 0x80000000u : 0));
        v.push_back({ encS(0x00, t, 3, 2, 0x23), 4, K_PLAIN, 0 });
    } else if (k < 88) {
        // INTSYSCR: HPE and nesting
        li_v(v, t, rn(4));
        v.push_back({ encI(0x804, t, 1, 0, 0x73), 4, K_PLAIN, 0 });
    } else if (k < 95) {
        v.push_back({ encI(0x300, 8, 6, 0, 0x73), 4, K_PLAIN, 0 });   // csrsi mstatus, MIE
    } else {
        // sleep until SysTick (or anything) wakes it
        li_v(v, t, 1u << 12);
        v.push_back({ encS(0x100, t, 4, 2, 0x23), 4, K_PLAIN, 0 });
        li_v(v, t, 30 + rn(300));
        v.push_back({ encS(0x10, t, 3, 2, 0x23), 4, K_PLAIN, 0 });
        v.push_back({ encS(0x04, 0, 3, 2, 0x23), 4, K_PLAIN, 0 });
        li_v(v, t, 0x1 | 0x2 | 0x4 | 0x8 | 0x20);
        v.push_back({ encS(0x00, t, 3, 2, 0x23), 4, K_PLAIN, 0 });
        v.push_back({ 0x10500073, 4, K_PLAIN, 0 });                     // wfi
    }
}

// per seed: flash programming and erasing, a software reset, running block F from the alias
static bool r_flash, r_reset, r_alias;

// the rare things: the flash controller (programming a halfword costs 1,440 cycles in the
// C, an erase 96,000), an ADC conversion (50), NVIC_SystemReset
static void gen_rare(std::vector<RInsn> &v, bool in_ram)
{
    const uint32_t k = rn(100);
    if (k < 50 && r_flash) {
        li_v(v, 5, 0x40022000);
        li_v(v, 6, 0x45670123);
        v.push_back({ encS(4, 6, 5, 2, 0x23), 4, K_PLAIN, 0 });
        li_v(v, 6, 0xCDEF89AB);
        v.push_back({ encS(4, 6, 5, 2, 0x23), 4, K_PLAIN, 0 });
        if (rn(3)) {
            li_v(v, 6, 1);                                              // PG
            v.push_back({ encS(0x10, 6, 5, 2, 0x23), 4, K_PLAIN, 0 });
            li_v(v, 7, 0xE000 + 2 * rn(1024));
            li_v(v, 6, rr());
            v.push_back({ encS(0, 6, 7, 1, 0x23), 4, K_PLAIN, 0 });       // sh: one halfword
            if (rn(2)) v.push_back({ encS(1, 6, 7, 2, 0x23), 4, K_PLAIN, 0 });   // sw misaligned: 4 bytes
        } else {
            li_v(v, 6, 2);                                              // PER
            v.push_back({ encS(0x10, 6, 5, 2, 0x23), 4, K_PLAIN, 0 });
            li_v(v, 6, 0xE000 + 0x400 * rn(5));
            v.push_back({ encS(0x14, 6, 5, 2, 0x23), 4, K_PLAIN, 0 });    // ADDR
            li_v(v, 6, 0x42);                                           // PER | STRT
            v.push_back({ encS(0x10, 6, 5, 2, 0x23), 4, K_PLAIN, 0 });
        }
        li_v(v, 6, 0x80);                                               // LOCK
        v.push_back({ encS(0x10, 6, 5, 2, 0x23), 4, K_PLAIN, 0 });
        v.push_back({ encI(0, 9, 2, 6, 0x03), 4, K_PLAIN, 0 });           // lw x6, 0(x9): flash as it is
    } else if (k < 85) {
        li_v(v, 5, 0x40012400);
        li_v(v, 6, 0x00400000 | (rr() & 0x000e0000));
        v.push_back({ encS(8, 6, 5, 2, 0x23), 4, K_PLAIN, 0 });           // CTLR2 SWSTART
        v.push_back({ encI(0x4c, 5, 2, 6, 0x03), 4, K_PLAIN, 0 });        // lw x6, RDATAR
    } else if (in_ram && r_reset) {
        li_v(v, 6, 0xBEEF0080);
        v.push_back({ encS(0x48, 6, 4, 2, 0x23), 4, K_PLAIN, 0 });        // KEY3 | SYSRESET
    }
}

static uint32_t gen_block(uint8_t *out, uint32_t base, int n, bool in_ram, uint32_t *size)
{
    std::vector<RInsn> v;
    while ((int)v.size() < n) {
        const uint32_t k = rn(1000);
        if (k < 40) gen_seq(v);
        else if (k < 42) gen_rare(v, in_ram);
        else v.push_back(gen_one(in_ram));
    }
    n = (int)v.size();
    std::vector<uint32_t> at(n + 1);
    uint32_t a = 0;
    for (int i = 0; i < n; i++) { at[i] = a; a += v[i].len; }
    at[n] = a;
    for (int i = 0; i < n; i++) {
        if (v[i].kind == K_PLAIN) continue;
        const int t = i + 1 + (int)rn(v[i].kind == K_BRANCH ? 24 : 60);
        const int ti = t > n ? n : t;
        const int off = (int)(at[ti] - at[i]);
        if (v[i].kind == K_BRANCH) v[i].w = (v[i].w & 0x01fff07fu) | (encB(off, 0, 0, 0) & 0xfe000f80u);
        else v[i].w = (v[i].w & 0xfffu) | (encJ(off, 0) & 0xfffff000u);
    }
    for (int i = 0; i < n; i++) {
        out[at[i]] = (uint8_t)v[i].w;
        out[at[i] + 1] = (uint8_t)(v[i].w >> 8);
        if (v[i].len == 4) { out[at[i] + 2] = (uint8_t)(v[i].w >> 16); out[at[i] + 3] = (uint8_t)(v[i].w >> 24); }
    }
    *size = a;
    (void)base;
    return a;
}

static void put32w(uint8_t *p, uint32_t w) { p[0] = (uint8_t)w; p[1] = (uint8_t)(w >> 8); p[2] = (uint8_t)(w >> 16); p[3] = (uint8_t)(w >> 24); }

// lui+addi of a constant into rd: 8 bytes
static void li(uint8_t *p, int rd, uint32_t v)
{
    const uint32_t lo = v & 0xfff, hi = (v + (lo & 0x800 ? 0x1000 : 0)) >> 12;
    put32w(p, encU(hi & 0xfffff, rd, 0x37));
    put32w(p + 4, encI((int)lo, rd, 0, rd, 0x13));
}

static ChgMachine *random_machine(uint32_t seed)
{
    ChgMachine *mm = (ChgMachine *)calloc(1, sizeof(ChgMachine));
    chg_init(mm);
    mm->entry = CHG_APP_START;
    chg_reset(mm, true);
    rs_state = seed ? seed : 1;
    r_flash = seed % 2 == 0;
    r_reset = seed % 3 == 0;
    r_alias = seed % 4 == 1;
    uint8_t *f = mm->flash;
    uint32_t p = 0x3000;
    li(f + p, 2, 0x20003000); p += 8;
    li(f + p, 8, 0x20002000); p += 8;
    li(f + p, 9, 0x00003000); p += 8;
    li(f + p, 4, 0xE000E000); p += 8;
    li(f + p, 3, 0xE000F000); p += 8;
    li(f + p, 5, 0x3080); p += 8;
    put32w(f + p, encI(0x305, 5, 1, 0, 0x73)); p += 4;             // csrw mtvec, x5
    li(f + p, 5, 0x3100); p += 8;
    put32w(f + p, encI(0, 5, 0, 0, 0x67)); p += 4;                 // jr x5
    // the handler at 0x3080
    p = 0x3080;
    const uint32_t h[] = {
        encI(0x342, 0, 2, 5, 0x73),             // csrr x5, mcause
        encB(36, 0, 5, 4),                      // blt x5, x0, irq (+36)
        encI(0x341, 0, 2, 5, 0x73),             // csrr x5, mepc
        encI(0, 5, 5, 6, 0x03),                 // lhu x6, 0(x5)
        encI(3, 6, 7, 6, 0x13),                 // andi x6, x6, 3
        encI(3, 0, 0, 7, 0x13),                 // addi x7, x0, 3
        encI(2, 5, 0, 5, 0x13),                 // addi x5, x5, 2
        encB(8, 7, 6, 1),                       // bne x6, x7, +8
        encI(2, 5, 0, 5, 0x13),                 // addi x5, x5, 2
        encI(0x341, 5, 1, 0, 0x73),             // csrw mepc, x5         (+36 from the blt is here? no: irq below)
        0x30200073,                             // mret
        // irq: SysTick has its flag cleared and stays on; anything else is disabled
        encI(63, 5, 7, 6, 0x13),                // andi x6, x5, 63
        encI(12, 0, 0, 7, 0x13),                // addi x7, x0, 12
        encB(12, 7, 6, 1),                      // bne x6, x7, +12
        encS(4, 0, 3, 2, 0x23),                 // sw x0, 4(x3)
        0x30200073,                             // mret
        encI(31, 5, 7, 6, 0x13),                // andi x6, x5, 31
        encI(1, 0, 0, 7, 0x13),                 // addi x7, x0, 1
        encR(0, 6, 7, 1, 7, 0x33),              // sll x7, x7, x6
        encI(5, 5, 5, 6, 0x13),                 // srli x6, x5, 5
        encI(1, 6, 7, 6, 0x13),                 // andi x6, x6, 1
        encI(2, 6, 1, 6, 0x13),                 // slli x6, x6, 2
        encR(0, 4, 6, 0, 6, 0x33),              // add x6, x6, x4
        encS(0x180, 7, 6, 2, 0x23),             // sw x7, 0x180(x6)
        encS(0x280, 7, 6, 2, 0x23),             // sw x7, 0x280(x6)
        0x30200073,                             // mret
    };
    for (uint32_t k = 0; k < sizeof h / sizeof h[0]; k++) put32w(f + p + 4 * k, h[k]);
    // blt offset: from index 1 to index 11 (irq) = 40 bytes
    put32w(f + p + 4, encB(40, 0, 5, 4));
    // block F
    uint32_t sz;
    gen_block(f + 0x3100, 0x3100, 2500, false, &sz);
    p = 0x3100 + sz;
    li(f + p, 5, 0x20000400); p += 8;
    put32w(f + p, encI(0, 5, 0, 0, 0x67)); p += 4;
    if (p > 0xF000) { fprintf(stderr, "block F too big\n"); exit(1); }
    // block R in SRAM
    gen_block(mm->ram + 0x400, 0x20000400, 1200, true, &sz);
    p = 0x400 + sz;
    li(mm->ram + p, 5, r_alias ? 0x08003100 : 0x3100); p += 8;     // block F again, maybe from the alias
    put32w(mm->ram + p, encI(0, 5, 0, 0, 0x67)); p += 4;
    if (p > 0x1F00) { fprintf(stderr, "block R too big\n"); exit(1); }
    return mm;
}
