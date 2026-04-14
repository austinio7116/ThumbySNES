/*
 * ThumbySNES — test suite for cpu_fast.c opcode handlers.
 *
 * For each of the 10 fast-path opcodes, tests:
 *   - Basic operation (8-bit mode, mf=1)
 *   - 16-bit mode (mf=0) where applicable
 *   - Flag computation (Z, N, C)
 *   - Edge cases (zero, negative, wrap)
 *
 * Uses a mock memory backend (flat 64 KB array) so tests run without
 * the full SNES bus. Build: `cc -o cpu_fast_test cpu_fast_test.c cpu_fast.c -I../vendor/lakesnes/snes`
 *
 * Run: `./cpu_fast_test` — prints PASS/FAIL per test, exits 0 if all pass.
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "cpu.h"
#include "cpu_fast.h"

/* ---- Mock memory ---- */
static uint8_t mock_mem[0x20000]; /* 128 KB — covers bank 0 + bank 1 */

static uint8_t mock_read(void* mem, uint32_t adr) {
    (void)mem;
    return mock_mem[adr & 0x1ffff];
}
static void mock_write(void* mem, uint32_t adr, uint8_t val) {
    (void)mem;
    mock_mem[adr & 0x1ffff] = val;
}
static void mock_idle(void* mem, bool waiting) {
    (void)mem; (void)waiting;
}

/* ---- Test harness ---- */
static int tests_run = 0;
static int tests_passed = 0;

#define ASSERT_EQ(name, got, expected) do { \
    if ((got) != (expected)) { \
        printf("  FAIL: %s: got 0x%x, expected 0x%x\n", name, (unsigned)(got), (unsigned)(expected)); \
        return 0; \
    } \
} while(0)

#define ASSERT_TRUE(name, cond) do { \
    if (!(cond)) { \
        printf("  FAIL: %s\n", name); \
        return 0; \
    } \
} while(0)

static Cpu* make_cpu(void) {
    static Cpu cpu;
    memset(&cpu, 0, sizeof(cpu));
    cpu.mem = NULL;
    cpu.read = mock_read;
    cpu.write = mock_write;
    cpu.idle = mock_idle;
    cpu.mf = true;   /* 8-bit mode default */
    cpu.xf = true;
    cpu.e = false;    /* native mode */
    cpu.sp = 0x01ff;
    cpu.pc = 0x8000;
    cpu.k = 0;
    cpu.db = 0;
    cpu.dp = 0;
    memset(mock_mem, 0, sizeof(mock_mem));
    return &cpu;
}

#define RUN_TEST(fn) do { \
    tests_run++; \
    printf("%-40s ", #fn); \
    if (fn()) { tests_passed++; printf("PASS\n"); } \
    else printf("\n"); \
} while(0)

/* ======== LDA dp (0xA5) ======== */

static int test_lda_dp_8bit(void) {
    Cpu *cpu = make_cpu();
    cpu->pc = 0x8000; cpu->dp = 0x0000; cpu->mf = true;
    mock_mem[0x8000] = 0x10;  /* operand: dp offset */
    mock_mem[0x0010] = 0x42;  /* data at dp+0x10 */
    int r = cpu_doOpcodeFast(cpu, 0xa5);
    ASSERT_EQ("handled", r, 1);
    ASSERT_EQ("a", cpu->a & 0xff, 0x42);
    ASSERT_EQ("pc", cpu->pc, 0x8001);
    ASSERT_TRUE("z=0", !cpu->z);
    ASSERT_TRUE("n=0", !cpu->n);
    return 1;
}

static int test_lda_dp_8bit_zero(void) {
    Cpu *cpu = make_cpu();
    cpu->pc = 0x8000; cpu->mf = true;
    mock_mem[0x8000] = 0x20;
    mock_mem[0x0020] = 0x00;
    cpu_doOpcodeFast(cpu, 0xa5);
    ASSERT_TRUE("z=1", cpu->z);
    ASSERT_TRUE("n=0", !cpu->n);
    return 1;
}

static int test_lda_dp_8bit_neg(void) {
    Cpu *cpu = make_cpu();
    cpu->pc = 0x8000; cpu->mf = true;
    mock_mem[0x8000] = 0x30;
    mock_mem[0x0030] = 0x80;
    cpu_doOpcodeFast(cpu, 0xa5);
    ASSERT_EQ("a", cpu->a & 0xff, 0x80);
    ASSERT_TRUE("n=1", cpu->n);
    ASSERT_TRUE("z=0", !cpu->z);
    return 1;
}

static int test_lda_dp_16bit(void) {
    Cpu *cpu = make_cpu();
    cpu->pc = 0x8000; cpu->mf = false;
    mock_mem[0x8000] = 0x10;
    mock_mem[0x0010] = 0x34;
    mock_mem[0x0011] = 0x12;
    cpu_doOpcodeFast(cpu, 0xa5);
    ASSERT_EQ("a", cpu->a, 0x1234);
    ASSERT_TRUE("z=0", !cpu->z);
    ASSERT_TRUE("n=0", !cpu->n);
    return 1;
}

static int test_lda_dp_preserves_high(void) {
    Cpu *cpu = make_cpu();
    cpu->pc = 0x8000; cpu->mf = true;
    cpu->a = 0xAB00;
    mock_mem[0x8000] = 0x05;
    mock_mem[0x0005] = 0xCD;
    cpu_doOpcodeFast(cpu, 0xa5);
    ASSERT_EQ("a", cpu->a, 0xABCD);
    return 1;
}

/* ======== STA dp (0x85) ======== */

static int test_sta_dp_8bit(void) {
    Cpu *cpu = make_cpu();
    cpu->pc = 0x8000; cpu->mf = true; cpu->a = 0x0055;
    mock_mem[0x8000] = 0x40;
    cpu_doOpcodeFast(cpu, 0x85);
    ASSERT_EQ("mem", mock_mem[0x0040], 0x55);
    ASSERT_EQ("pc", cpu->pc, 0x8001);
    return 1;
}

static int test_sta_dp_16bit(void) {
    Cpu *cpu = make_cpu();
    cpu->pc = 0x8000; cpu->mf = false; cpu->a = 0xBEEF;
    mock_mem[0x8000] = 0x40;
    cpu_doOpcodeFast(cpu, 0x85);
    ASSERT_EQ("lo", mock_mem[0x0040], 0xEF);
    ASSERT_EQ("hi", mock_mem[0x0041], 0xBE);
    return 1;
}

/* ======== BNE rel (0xD0) ======== */

static int test_bne_taken(void) {
    Cpu *cpu = make_cpu();
    cpu->pc = 0x8000; cpu->z = false;
    mock_mem[0x8000] = 0x10; /* +16 */
    cpu_doOpcodeFast(cpu, 0xd0);
    ASSERT_EQ("pc", cpu->pc, 0x8001 + 0x10);
    return 1;
}

static int test_bne_not_taken(void) {
    Cpu *cpu = make_cpu();
    cpu->pc = 0x8000; cpu->z = true;
    mock_mem[0x8000] = 0x10;
    cpu_doOpcodeFast(cpu, 0xd0);
    ASSERT_EQ("pc", cpu->pc, 0x8001);
    return 1;
}

static int test_bne_backward(void) {
    Cpu *cpu = make_cpu();
    cpu->pc = 0x8020; cpu->z = false;
    mock_mem[0x8020] = 0xFE; /* -2 */
    cpu_doOpcodeFast(cpu, 0xd0);
    ASSERT_EQ("pc", cpu->pc, 0x8021 - 2);
    return 1;
}

/* ======== BEQ rel (0xF0) ======== */

static int test_beq_taken(void) {
    Cpu *cpu = make_cpu();
    cpu->pc = 0x8000; cpu->z = true;
    mock_mem[0x8000] = 0x05;
    cpu_doOpcodeFast(cpu, 0xf0);
    ASSERT_EQ("pc", cpu->pc, 0x8001 + 5);
    return 1;
}

static int test_beq_not_taken(void) {
    Cpu *cpu = make_cpu();
    cpu->pc = 0x8000; cpu->z = false;
    mock_mem[0x8000] = 0x05;
    cpu_doOpcodeFast(cpu, 0xf0);
    ASSERT_EQ("pc", cpu->pc, 0x8001);
    return 1;
}

/* ======== LDA imm (0xA9) ======== */

static int test_lda_imm_8bit(void) {
    Cpu *cpu = make_cpu();
    cpu->pc = 0x8000; cpu->mf = true;
    mock_mem[0x8000] = 0x77;
    cpu_doOpcodeFast(cpu, 0xa9);
    ASSERT_EQ("a", cpu->a & 0xff, 0x77);
    ASSERT_EQ("pc", cpu->pc, 0x8001);
    return 1;
}

static int test_lda_imm_16bit(void) {
    Cpu *cpu = make_cpu();
    cpu->pc = 0x8000; cpu->mf = false;
    mock_mem[0x8000] = 0xCD;
    mock_mem[0x8001] = 0xAB;
    cpu_doOpcodeFast(cpu, 0xa9);
    ASSERT_EQ("a", cpu->a, 0xABCD);
    ASSERT_EQ("pc", cpu->pc, 0x8002);
    return 1;
}

/* ======== CMP imm (0xC9) ======== */

static int test_cmp_imm_equal(void) {
    Cpu *cpu = make_cpu();
    cpu->pc = 0x8000; cpu->mf = true; cpu->a = 0x0042;
    mock_mem[0x8000] = 0x42;
    cpu_doOpcodeFast(cpu, 0xc9);
    ASSERT_TRUE("z=1", cpu->z);
    ASSERT_TRUE("c=1", cpu->c);
    ASSERT_TRUE("n=0", !cpu->n);
    return 1;
}

static int test_cmp_imm_greater(void) {
    Cpu *cpu = make_cpu();
    cpu->pc = 0x8000; cpu->mf = true; cpu->a = 0x0050;
    mock_mem[0x8000] = 0x30;
    cpu_doOpcodeFast(cpu, 0xc9);
    ASSERT_TRUE("z=0", !cpu->z);
    ASSERT_TRUE("c=1", cpu->c);  /* A >= operand */
    return 1;
}

static int test_cmp_imm_less(void) {
    Cpu *cpu = make_cpu();
    cpu->pc = 0x8000; cpu->mf = true; cpu->a = 0x0010;
    mock_mem[0x8000] = 0x20;
    cpu_doOpcodeFast(cpu, 0xc9);
    ASSERT_TRUE("z=0", !cpu->z);
    ASSERT_TRUE("c=0", !cpu->c);  /* A < operand */
    ASSERT_TRUE("n=1", cpu->n);   /* result is negative */
    return 1;
}

static int test_cmp_imm_16bit(void) {
    Cpu *cpu = make_cpu();
    cpu->pc = 0x8000; cpu->mf = false; cpu->a = 0x1234;
    mock_mem[0x8000] = 0x34;
    mock_mem[0x8001] = 0x12;
    cpu_doOpcodeFast(cpu, 0xc9);
    ASSERT_TRUE("z=1", cpu->z);
    ASSERT_TRUE("c=1", cpu->c);
    return 1;
}

/* ======== LDA abs (0xAD) ======== */

static int test_lda_abs_8bit(void) {
    Cpu *cpu = make_cpu();
    cpu->pc = 0x8000; cpu->mf = true; cpu->db = 0;
    mock_mem[0x8000] = 0x00;  /* addr lo */
    mock_mem[0x8001] = 0x10;  /* addr hi → $1000 */
    mock_mem[0x1000] = 0x99;
    cpu_doOpcodeFast(cpu, 0xad);
    ASSERT_EQ("a", cpu->a & 0xff, 0x99);
    ASSERT_EQ("pc", cpu->pc, 0x8002);
    return 1;
}

static int test_lda_abs_16bit(void) {
    Cpu *cpu = make_cpu();
    cpu->pc = 0x8000; cpu->mf = false; cpu->db = 0;
    mock_mem[0x8000] = 0x00;
    mock_mem[0x8001] = 0x10;
    mock_mem[0x1000] = 0xEF;
    mock_mem[0x1001] = 0xBE;
    cpu_doOpcodeFast(cpu, 0xad);
    ASSERT_EQ("a", cpu->a, 0xBEEF);
    return 1;
}

/* ======== STA abs (0x8D) ======== */

static int test_sta_abs_8bit(void) {
    Cpu *cpu = make_cpu();
    cpu->pc = 0x8000; cpu->mf = true; cpu->a = 0x0011; cpu->db = 0;
    mock_mem[0x8000] = 0x00;
    mock_mem[0x8001] = 0x20;
    cpu_doOpcodeFast(cpu, 0x8d);
    ASSERT_EQ("mem", mock_mem[0x2000], 0x11);
    return 1;
}

static int test_sta_abs_16bit(void) {
    Cpu *cpu = make_cpu();
    cpu->pc = 0x8000; cpu->mf = false; cpu->a = 0xCAFE; cpu->db = 0;
    mock_mem[0x8000] = 0x00;
    mock_mem[0x8001] = 0x20;
    cpu_doOpcodeFast(cpu, 0x8d);
    ASSERT_EQ("lo", mock_mem[0x2000], 0xFE);
    ASSERT_EQ("hi", mock_mem[0x2001], 0xCA);
    return 1;
}

/* ======== JSR abs (0x20) ======== */

static int test_jsr(void) {
    Cpu *cpu = make_cpu();
    cpu->pc = 0x8000; cpu->sp = 0x01ff;
    mock_mem[0x8000] = 0x00;  /* target lo */
    mock_mem[0x8001] = 0x90;  /* target hi → $9000 */
    cpu_doOpcodeFast(cpu, 0x20);
    ASSERT_EQ("pc", cpu->pc, 0x9000);
    ASSERT_EQ("sp", cpu->sp, 0x01fd);
    /* Stack should have return address (0x8002 - 1 = 0x8001): hi then lo */
    ASSERT_EQ("stack_hi", mock_mem[0x01ff], 0x80);
    ASSERT_EQ("stack_lo", mock_mem[0x01fe], 0x01);
    return 1;
}

/* ======== RTS (0x60) ======== */

static int test_rts(void) {
    Cpu *cpu = make_cpu();
    cpu->sp = 0x01fd;
    /* Stack has return address 0x8001 (which RTS adds 1 to → 0x8002) */
    mock_mem[0x01fe] = 0x01;  /* lo */
    mock_mem[0x01ff] = 0x80;  /* hi */
    cpu_doOpcodeFast(cpu, 0x60);
    ASSERT_EQ("pc", cpu->pc, 0x8002);
    ASSERT_EQ("sp", cpu->sp, 0x01ff);
    return 1;
}

static int test_jsr_rts_roundtrip(void) {
    Cpu *cpu = make_cpu();
    cpu->pc = 0x8000; cpu->sp = 0x01ff;
    mock_mem[0x8000] = 0x00;
    mock_mem[0x8001] = 0x90;
    cpu_doOpcodeFast(cpu, 0x20); /* JSR $9000 */
    ASSERT_EQ("after_jsr_pc", cpu->pc, 0x9000);
    ASSERT_EQ("after_jsr_sp", cpu->sp, 0x01fd);
    cpu_doOpcodeFast(cpu, 0x60); /* RTS */
    ASSERT_EQ("after_rts_pc", cpu->pc, 0x8002);
    ASSERT_EQ("after_rts_sp", cpu->sp, 0x01ff);
    return 1;
}

/* ======== Fallback ======== */

static int test_unhandled_returns_zero(void) {
    Cpu *cpu = make_cpu();
    int r = cpu_doOpcodeFast(cpu, 0x00); /* BRK — not in fast path */
    ASSERT_EQ("handled", r, 0);
    return 1;
}

/* ======== Main ======== */

int main(void) {
    printf("cpu_fast opcode tests\n");
    printf("=====================\n\n");

    /* LDA dp */
    RUN_TEST(test_lda_dp_8bit);
    RUN_TEST(test_lda_dp_8bit_zero);
    RUN_TEST(test_lda_dp_8bit_neg);
    RUN_TEST(test_lda_dp_16bit);
    RUN_TEST(test_lda_dp_preserves_high);
    /* STA dp */
    RUN_TEST(test_sta_dp_8bit);
    RUN_TEST(test_sta_dp_16bit);
    /* BNE */
    RUN_TEST(test_bne_taken);
    RUN_TEST(test_bne_not_taken);
    RUN_TEST(test_bne_backward);
    /* BEQ */
    RUN_TEST(test_beq_taken);
    RUN_TEST(test_beq_not_taken);
    /* LDA imm */
    RUN_TEST(test_lda_imm_8bit);
    RUN_TEST(test_lda_imm_16bit);
    /* CMP imm */
    RUN_TEST(test_cmp_imm_equal);
    RUN_TEST(test_cmp_imm_greater);
    RUN_TEST(test_cmp_imm_less);
    RUN_TEST(test_cmp_imm_16bit);
    /* LDA abs */
    RUN_TEST(test_lda_abs_8bit);
    RUN_TEST(test_lda_abs_16bit);
    /* STA abs */
    RUN_TEST(test_sta_abs_8bit);
    RUN_TEST(test_sta_abs_16bit);
    /* JSR */
    RUN_TEST(test_jsr);
    /* RTS */
    RUN_TEST(test_rts);
    RUN_TEST(test_jsr_rts_roundtrip);
    /* Fallback */
    RUN_TEST(test_unhandled_returns_zero);

    printf("\n%d/%d tests passed.\n", tests_passed, tests_run);
    return (tests_passed == tests_run) ? 0 : 1;
}
