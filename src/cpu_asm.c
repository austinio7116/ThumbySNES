/*
 * ThumbySNES — ARM Thumb-2 assembly 65816 CPU dispatcher.
 *
 * Keeps 65816 registers in ARM callee-saved registers R4-R11 across
 * a batch of opcodes. Bus access (snes_cpuRead/Write/Idle) via BLX;
 * R4-R11 survive per AAPCS.
 *
 * Register map (inside asm block):
 *   R4  = 65816 A  (16-bit in 32-bit reg)
 *   R5  = 65816 X  (index register)
 *   R6  = 65816 Y  (index register)
 *   R7  = 65816 SP (stack pointer)
 *   R8  = 65816 PC (program counter)
 *   R9  = packed flags: b0=C b1=Z b2=I b3=D b4=XF b5=MF b6=V b7=N
 *   R10 = Cpu struct pointer
 *   R11 = Snes struct pointer (== cpu->mem)
 *
 * CRITICAL DESIGN: SP is set once in the prologue and NEVER changes
 * during opcode execution. All temporaries use named scratch slots on
 * the stack frame. Subroutines access the frame at fixed offsets.
 * Subroutines that call external functions save/restore LR via a
 * dedicated frame slot (FRM_LR_SUB), not via push/pop.
 *
 * Implements 30 opcodes in ASM, all others fall back to C.
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "cpu.h"
#include "snes.h"
#include "dma.h"
#include "perf.h"

extern uint8_t snes_cpuRead(void *mem, uint32_t adr);
extern void    snes_cpuWrite(void *mem, uint32_t adr, uint8_t val);
extern void    snes_cpuIdle(void *mem, bool waiting);
extern void    snes_flushCycles(Snes *snes);
extern void    cpu_runOpcode(Cpu *cpu);

#if defined(__arm__) || defined(__thumb__)

/* Cpu struct field offsets — verified by _Static_assert below. */
#define CPU_MEM    0
#define CPU_A      16
#define CPU_X      18
#define CPU_Y      20
#define CPU_SP     22
#define CPU_PC     24
#define CPU_DP     26
#define CPU_K      28
#define CPU_DB     29
#define CPU_C      30
#define CPU_Z      31
#define CPU_V      32
#define CPU_N      33
#define CPU_I      34
#define CPU_D      35
#define CPU_XF     36
#define CPU_MF     37
#define CPU_E      38
#define CPU_WAITING    39
#define CPU_STOPPED    40
#define CPU_IRQWANTED  41
#define CPU_NMIWANTED  42
#define CPU_INTWANTED  43
#define CPU_RESETWANTED 44

_Static_assert(offsetof(Cpu, mem) == CPU_MEM, "");
_Static_assert(offsetof(Cpu, a)   == CPU_A,   "");
_Static_assert(offsetof(Cpu, x)   == CPU_X,   "");
_Static_assert(offsetof(Cpu, y)   == CPU_Y,   "");
_Static_assert(offsetof(Cpu, sp)  == CPU_SP,  "");
_Static_assert(offsetof(Cpu, pc)  == CPU_PC,  "");
_Static_assert(offsetof(Cpu, dp)  == CPU_DP,  "");
_Static_assert(offsetof(Cpu, k)   == CPU_K,   "");
_Static_assert(offsetof(Cpu, db)  == CPU_DB,  "");
_Static_assert(offsetof(Cpu, c)   == CPU_C,   "");
_Static_assert(offsetof(Cpu, z)   == CPU_Z,   "");
_Static_assert(offsetof(Cpu, v)   == CPU_V,   "");
_Static_assert(offsetof(Cpu, n)   == CPU_N,   "");
_Static_assert(offsetof(Cpu, i)   == CPU_I,   "");
_Static_assert(offsetof(Cpu, d)   == CPU_D,   "");
_Static_assert(offsetof(Cpu, xf)  == CPU_XF,  "");
_Static_assert(offsetof(Cpu, mf)  == CPU_MF,  "");
_Static_assert(offsetof(Cpu, e)   == CPU_E,   "");
_Static_assert(offsetof(Cpu, waiting)     == CPU_WAITING,     "");
_Static_assert(offsetof(Cpu, stopped)     == CPU_STOPPED,     "");
_Static_assert(offsetof(Cpu, irqWanted)   == CPU_IRQWANTED,   "");
_Static_assert(offsetof(Cpu, nmiWanted)   == CPU_NMIWANTED,   "");
_Static_assert(offsetof(Cpu, intWanted)   == CPU_INTWANTED,   "");
_Static_assert(offsetof(Cpu, resetWanted) == CPU_RESETWANTED, "");

/*
 * Stack frame. SP is constant after prologue SUB.
 *
 * [SP+0]  K          [SP+4]  DB         [SP+8]  DP
 * [SP+12] E          [SP+16] maxOp      [SP+20] count
 * [SP+24] fn_read    [SP+28] fn_write   [SP+32] fn_idle
 * [SP+36] fn_flush   [SP+40] fn_runop
 * [SP+44] lr_sub     — LR save slot for non-leaf subroutines
 * [SP+48] s0         [SP+52] s1         [SP+56] s2
 * [SP+60] s3         [SP+64] s4
 * Total: 68 bytes, padded to 72 for 8-byte alignment.
 */
#define FK    0
#define FDB   4
#define FDP   8
#define FE    12
#define FMAX  16
#define FCNT  20
#define FRD   24
#define FWR   28
#define FID   32
#define FFL   36
#define FRO   40
#define FLR   44
#define S0    48
#define S1    52
#define S2    56
#define S3    60
#define S4    64
#define FSZ   72

#define _S(x) #x
#define S(x) _S(x)

__attribute__((naked, section(".time_critical.snes")))
int cpu_runBatchAsm(Cpu *cpu, int maxOpcodes)
{
    __asm__ volatile (
    ".syntax unified\n .thumb\n"

    /* === PROLOGUE === */
    "push {r4-r11, lr}\n"
    "sub  sp, sp, #" S(FSZ) "\n"
    "mov  r10, r0\n"                       /* R10 = Cpu* */
    "ldr  r11, [r0, #" S(CPU_MEM) "]\n"   /* R11 = Snes* */
    "str  r1, [sp, #" S(FMAX) "]\n"
    "movs r1, #0\n"
    "str  r1, [sp, #" S(FCNT) "]\n"

    /* Function pointers */
    "ldr r2, =snes_cpuRead\n   str r2, [sp, #" S(FRD) "]\n"
    "ldr r2, =snes_cpuWrite\n  str r2, [sp, #" S(FWR) "]\n"
    "ldr r2, =snes_cpuIdle\n   str r2, [sp, #" S(FID) "]\n"
    "ldr r2, =snes_flushCycles\n str r2, [sp, #" S(FFL) "]\n"
    "ldr r2, =cpu_runOpcode\n  str r2, [sp, #" S(FRO) "]\n"

    /* Load 65816 regs */
    "ldrh r4, [r10, #" S(CPU_A)  "]\n"
    "ldrh r5, [r10, #" S(CPU_X)  "]\n"
    "ldrh r6, [r10, #" S(CPU_Y)  "]\n"
    "ldrh r7, [r10, #" S(CPU_SP) "]\n"
    "ldrh r8, [r10, #" S(CPU_PC) "]\n"

    /* Pack flags */
    "ldrb r0, [r10, #" S(CPU_C) "]\n"
    "ldrb r1, [r10, #" S(CPU_Z) "]\n orr r0, r0, r1, lsl #1\n"
    "ldrb r1, [r10, #" S(CPU_I) "]\n orr r0, r0, r1, lsl #2\n"
    "ldrb r1, [r10, #" S(CPU_D) "]\n orr r0, r0, r1, lsl #3\n"
    "ldrb r1, [r10, #" S(CPU_XF) "]\n orr r0, r0, r1, lsl #4\n"
    "ldrb r1, [r10, #" S(CPU_MF) "]\n orr r0, r0, r1, lsl #5\n"
    "ldrb r1, [r10, #" S(CPU_V) "]\n orr r0, r0, r1, lsl #6\n"
    "ldrb r1, [r10, #" S(CPU_N) "]\n orr r9, r0, r1, lsl #7\n"

    /* Frame state */
    "ldrb r0, [r10, #" S(CPU_K) "]\n  str r0, [sp, #" S(FK) "]\n"
    "ldrb r0, [r10, #" S(CPU_DB) "]\n str r0, [sp, #" S(FDB) "]\n"
    "ldrh r0, [r10, #" S(CPU_DP) "]\n str r0, [sp, #" S(FDP) "]\n"
    "ldrb r0, [r10, #" S(CPU_E) "]\n  str r0, [sp, #" S(FE) "]\n"

    /* Initial flush */
    "mov r0, r11\n ldr r3, [sp, #" S(FFL) "]\n blx r3\n"

    /* === MAIN LOOP === */
    ".Lloop:\n"
    "ldr r0, [sp, #" S(FCNT) "]\n"
    "ldr r1, [sp, #" S(FMAX) "]\n"
    "cmp r0, r1\n bge .Ldone\n"

    /* Check special states */
    "ldrb r0, [r10, #" S(CPU_RESETWANTED) "]\n cbnz r0, .Lspecial\n"
    "ldrb r0, [r10, #" S(CPU_STOPPED) "]\n     cbnz r0, .Lspecial\n"
    "ldrb r0, [r10, #" S(CPU_WAITING) "]\n      cbnz r0, .Lspecial\n"
    "ldrb r0, [r10, #" S(CPU_INTWANTED) "]\n    cbnz r0, .Lspecial\n"

    /* Flush pending cycles */
    "mov r0, r11\n ldr r3, [sp, #" S(FFL) "]\n blx r3\n"

    /* Fetch opcode at (K<<16)|PC */
    "ldr r1, [sp, #" S(FK) "]\n"
    "lsl r1, r1, #16\n orr r1, r1, r8\n"
    "add r8, r8, #1\n uxth r8, r8\n"
    "mov r0, r11\n ldr r3, [sp, #" S(FRD) "]\n blx r3\n"
    /* R0 = opcode */

    /* --- Dispatch --- */
    "cmp r0, #0xD0\n beq .Lo_D0\n"
    "cmp r0, #0xF0\n beq .Lo_F0\n"
    "cmp r0, #0x10\n beq .Lo_10\n"
    "cmp r0, #0x30\n beq .Lo_30\n"
    "cmp r0, #0x90\n beq .Lo_90\n"
    "cmp r0, #0xB0\n beq .Lo_B0\n"
    "cmp r0, #0xA9\n beq .Lo_A9\n"
    "cmp r0, #0xA5\n beq .Lo_A5\n"
    "cmp r0, #0xAD\n beq .Lo_AD\n"
    "cmp r0, #0xB5\n beq .Lo_B5\n"
    "cmp r0, #0xB9\n beq .Lo_B9\n"
    "cmp r0, #0xBD\n beq .Lo_BD\n"
    "cmp r0, #0x85\n beq .Lo_85\n"
    "cmp r0, #0x8D\n beq .Lo_8D\n"
    "cmp r0, #0x95\n beq .Lo_95\n"
    "cmp r0, #0x99\n beq .Lo_99\n"
    "cmp r0, #0x9D\n beq .Lo_9D\n"
    "cmp r0, #0xA2\n beq .Lo_A2\n"
    "cmp r0, #0xA6\n beq .Lo_A6\n"
    "cmp r0, #0xAE\n beq .Lo_AE\n"
    "cmp r0, #0xA0\n beq .Lo_A0\n"
    "cmp r0, #0xA4\n beq .Lo_A4\n"
    "cmp r0, #0xAC\n beq .Lo_AC\n"
    "cmp r0, #0xC9\n beq .Lo_C9\n"
    "cmp r0, #0xC5\n beq .Lo_C5\n"
    "cmp r0, #0xCD\n beq .Lo_CD\n"
    "cmp r0, #0x69\n beq .Lo_69\n"
    "cmp r0, #0x65\n beq .Lo_65\n"
    "cmp r0, #0x20\n beq .Lo_20\n"
    "cmp r0, #0x60\n beq .Lo_60\n"

    /* === FALLBACK === */
    ".Lfb:\n"
    "sub r8, r8, #1\n uxth r8, r8\n"
    ".Lspecial:\n"
    "bl .Lstore\n"
    "mov r0, r10\n ldr r3, [sp, #" S(FRO) "]\n blx r3\n"
    "bl .Lload\n"
    "b  .Lnext\n"

    /* === .Lnext — increment count, loop === */
    ".Lnext:\n"
    "ldr r0, [sp, #" S(FCNT) "]\n adds r0, #1\n str r0, [sp, #" S(FCNT) "]\n"
    "b .Lloop\n"

    /* ========================================== */
    /* SUBROUTINES (SP never changes!)            */
    /* ========================================== */

    /* .Lstore — write ARM regs to Cpu struct. Leaf, uses bx lr. */
    ".Lstore:\n"
    "strh r4, [r10, #" S(CPU_A) "]\n"
    "strh r5, [r10, #" S(CPU_X) "]\n"
    "strh r6, [r10, #" S(CPU_Y) "]\n"
    "strh r7, [r10, #" S(CPU_SP) "]\n"
    "strh r8, [r10, #" S(CPU_PC) "]\n"
    "and  r0, r9, #1\n        strb r0, [r10, #" S(CPU_C) "]\n"
    "ubfx r0, r9, #1, #1\n    strb r0, [r10, #" S(CPU_Z) "]\n"
    "ubfx r0, r9, #2, #1\n    strb r0, [r10, #" S(CPU_I) "]\n"
    "ubfx r0, r9, #3, #1\n    strb r0, [r10, #" S(CPU_D) "]\n"
    "ubfx r0, r9, #4, #1\n    strb r0, [r10, #" S(CPU_XF) "]\n"
    "ubfx r0, r9, #5, #1\n    strb r0, [r10, #" S(CPU_MF) "]\n"
    "ubfx r0, r9, #6, #1\n    strb r0, [r10, #" S(CPU_V) "]\n"
    "ubfx r0, r9, #7, #1\n    strb r0, [r10, #" S(CPU_N) "]\n"
    "ldr r0, [sp, #" S(FK) "]\n  strb r0, [r10, #" S(CPU_K) "]\n"
    "ldr r0, [sp, #" S(FDB) "]\n strb r0, [r10, #" S(CPU_DB) "]\n"
    "ldr r0, [sp, #" S(FDP) "]\n strh r0, [r10, #" S(CPU_DP) "]\n"
    "ldr r0, [sp, #" S(FE) "]\n  strb r0, [r10, #" S(CPU_E) "]\n"
    "bx lr\n"

    /* .Lload — read Cpu struct to ARM regs. Leaf. */
    ".Lload:\n"
    "ldrh r4, [r10, #" S(CPU_A) "]\n"
    "ldrh r5, [r10, #" S(CPU_X) "]\n"
    "ldrh r6, [r10, #" S(CPU_Y) "]\n"
    "ldrh r7, [r10, #" S(CPU_SP) "]\n"
    "ldrh r8, [r10, #" S(CPU_PC) "]\n"
    "ldr  r11, [r10, #" S(CPU_MEM) "]\n"
    "ldrb r0, [r10, #" S(CPU_C) "]\n"
    "ldrb r1, [r10, #" S(CPU_Z) "]\n orr r0, r0, r1, lsl #1\n"
    "ldrb r1, [r10, #" S(CPU_I) "]\n orr r0, r0, r1, lsl #2\n"
    "ldrb r1, [r10, #" S(CPU_D) "]\n orr r0, r0, r1, lsl #3\n"
    "ldrb r1, [r10, #" S(CPU_XF) "]\n orr r0, r0, r1, lsl #4\n"
    "ldrb r1, [r10, #" S(CPU_MF) "]\n orr r0, r0, r1, lsl #5\n"
    "ldrb r1, [r10, #" S(CPU_V) "]\n orr r0, r0, r1, lsl #6\n"
    "ldrb r1, [r10, #" S(CPU_N) "]\n orr r9, r0, r1, lsl #7\n"
    "ldrb r0, [r10, #" S(CPU_K) "]\n  str r0, [sp, #" S(FK) "]\n"
    "ldrb r0, [r10, #" S(CPU_DB) "]\n str r0, [sp, #" S(FDB) "]\n"
    "ldrh r0, [r10, #" S(CPU_DP) "]\n str r0, [sp, #" S(FDP) "]\n"
    "ldrb r0, [r10, #" S(CPU_E) "]\n  str r0, [sp, #" S(FE) "]\n"
    "bx lr\n"

    /* .Lint — check interrupt. Leaf. Clobbers r0, r1. */
    ".Lint:\n"
    "ldrb r0, [r10, #" S(CPU_NMIWANTED) "]\n"
    "cbnz r0, 1f\n"
    "ldrb r0, [r10, #" S(CPU_IRQWANTED) "]\n"
    "cbz  r0, 2f\n"
    "tst  r9, #4\n bne 2f\n"  /* I flag set -> no IRQ */
    "1: movs r0, #1\n strb r0, [r10, #" S(CPU_INTWANTED) "]\n bx lr\n"
    "2: movs r0, #0\n strb r0, [r10, #" S(CPU_INTWANTED) "]\n bx lr\n"

    /* .Lzn8 — set Z,N from R0 (8-bit). Leaf. */
    ".Lzn8:\n"
    "bic r9, r9, #0x82\n"
    "tst r0, #0xFF\n bne 1f\n orr r9, r9, #0x02\n"
    "1: tst r0, #0x80\n beq 1f\n orr r9, r9, #0x80\n"
    "1: bx lr\n"

    /* .Lzn16 — set Z,N from R0 (16-bit). Leaf. */
    ".Lzn16:\n"
    "bic r9, r9, #0x82\n"
    "uxth r1, r0\n cbnz r1, 1f\n orr r9, r9, #0x02\n"
    "1: tst r0, #0x8000\n beq 1f\n orr r9, r9, #0x80\n"
    "1: bx lr\n"

    /*
     * .Lfetch — read byte at (K<<16)|PC, advance PC.
     * Non-leaf (calls snes_cpuRead). Saves LR to FLR slot.
     * Output: R0 = byte. Clobbers R0-R3, R12.
     */
    ".Lfetch:\n"
    "str  lr, [sp, #" S(FLR) "]\n"
    "ldr  r1, [sp, #" S(FK) "]\n"
    "lsl  r1, r1, #16\n orr r1, r1, r8\n"
    "add  r8, r8, #1\n uxth r8, r8\n"
    "mov  r0, r11\n"
    "ldr  r3, [sp, #" S(FRD) "]\n blx r3\n"
    "ldr  lr, [sp, #" S(FLR) "]\n bx lr\n"

    /*
     * .Lrd — read byte at address in R1. Non-leaf.
     * Output: R0 = byte.
     */
    ".Lrd:\n"
    "str  lr, [sp, #" S(FLR) "]\n"
    "mov  r0, r11\n"
    "ldr  r3, [sp, #" S(FRD) "]\n blx r3\n"
    "ldr  lr, [sp, #" S(FLR) "]\n bx lr\n"

    /*
     * .Lwr — write byte R2 at address R1. Non-leaf.
     */
    ".Lwr:\n"
    "str  lr, [sp, #" S(FLR) "]\n"
    "mov  r0, r11\n"
    "ldr  r3, [sp, #" S(FWR) "]\n blx r3\n"
    "ldr  lr, [sp, #" S(FLR) "]\n bx lr\n"

    /*
     * .Lidl — idle cycle. Non-leaf.
     */
    ".Lidl:\n"
    "str  lr, [sp, #" S(FLR) "]\n"
    "mov  r0, r11\n movs r1, #0\n"
    "ldr  r3, [sp, #" S(FID) "]\n blx r3\n"
    "ldr  lr, [sp, #" S(FLR) "]\n bx lr\n"

    /* ========================================== */
    /* OPCODES                                    */
    /*                                            */
    /* Temporaries use S0-S4 via str/ldr.         */
    /* SP never changes.                          */
    /* ========================================== */

    /* ---- LDA imm (A9) ---- */
    ".Lo_A9:\n"
    "tst r9, #0x20\n beq .La9_16\n"
    /* 8-bit */
    "ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n"
    "add r8, r8, #1\n uxth r8, r8\n"
    "bl .Lint\n bl .Lrd\n"
    "bfi r4, r0, #0, #8\n bl .Lzn8\n b .Lnext\n"
    ".La9_16:\n"
    "ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n"
    "add r8, r8, #1\n uxth r8, r8\n"
    "bl .Lrd\n str r0, [sp, #" S(S0) "]\n"  /* S0=low */
    "ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n"
    "add r8, r8, #1\n uxth r8, r8\n"
    "bl .Lint\n bl .Lrd\n"
    "lsl r0, r0, #8\n ldr r1, [sp, #" S(S0) "]\n orr r4, r1, r0\n"
    "mov r0, r4\n bl .Lzn16\n b .Lnext\n"

    /* ---- LDA dp (A5) ---- */
    ".Lo_A5:\n"
    "bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"  /* S0=dp offset */
    "ldr r0, [sp, #" S(FDP) "]\n tst r0, #0xFF\n beq 1f\n bl .Lidl\n"
    "1: ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
    "add r1, r1, r0\n uxth r1, r1\n"
    "tst r9, #0x20\n beq .La5_16\n"
    "bl .Lint\n bl .Lrd\n bfi r4, r0, #0, #8\n bl .Lzn8\n b .Lnext\n"
    ".La5_16:\n"
    "bl .Lrd\n str r0, [sp, #" S(S1) "]\n"  /* S1=low byte */
    "ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
    "add r1, r1, r0\n adds r1, #1\n uxth r1, r1\n"
    "bl .Lint\n bl .Lrd\n"
    "lsl r0, r0, #8\n ldr r1, [sp, #" S(S1) "]\n orr r4, r1, r0\n"
    "mov r0, r4\n bl .Lzn16\n b .Lnext\n"

    /* ---- LDA abs (AD) ---- */
    ".Lo_AD:\n"
    "bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"  /* S0=lo adr */
    "bl .Lfetch\n"  /* r0=hi adr */
    "ldr r1, [sp, #" S(S0) "]\n orr r1, r1, r0, lsl #8\n"
    "ldr r0, [sp, #" S(FDB) "]\n add r1, r1, r0, lsl #16\n"  /* ea */
    "tst r9, #0x20\n beq .Lad_16\n"
    "bl .Lint\n bl .Lrd\n bfi r4, r0, #0, #8\n bl .Lzn8\n b .Lnext\n"
    ".Lad_16:\n"
    "str r1, [sp, #" S(S0) "]\n bl .Lrd\n str r0, [sp, #" S(S1) "]\n"
    "ldr r1, [sp, #" S(S0) "]\n adds r1, #1\n bic r1, r1, #0xFF000000\n"
    "bl .Lint\n bl .Lrd\n"
    "lsl r0, r0, #8\n ldr r1, [sp, #" S(S1) "]\n orr r4, r1, r0\n"
    "mov r0, r4\n bl .Lzn16\n b .Lnext\n"

    /* ---- LDA dp,X (B5) ---- */
    ".Lo_B5:\n"
    "bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
    "ldr r0, [sp, #" S(FDP) "]\n tst r0, #0xFF\n beq 1f\n bl .Lidl\n"
    "1: bl .Lidl\n"  /* indexed */
    "ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
    "add r1, r1, r0\n add r1, r1, r5\n uxth r1, r1\n"
    "tst r9, #0x20\n beq .Lb5_16\n"
    "bl .Lint\n bl .Lrd\n bfi r4, r0, #0, #8\n bl .Lzn8\n b .Lnext\n"
    ".Lb5_16:\n"
    "bl .Lrd\n str r0, [sp, #" S(S1) "]\n"
    "ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
    "add r1, r1, r0\n add r1, r1, r5\n adds r1, #1\n uxth r1, r1\n"
    "bl .Lint\n bl .Lrd\n"
    "lsl r0, r0, #8\n ldr r1, [sp, #" S(S1) "]\n orr r4, r1, r0\n"
    "mov r0, r4\n bl .Lzn16\n b .Lnext\n"

    /* ---- LDA abs,Y (B9) / LDA abs,X (BD) ---- */
    ".Lo_B9:\n mov r12, r6\n b .Labxy\n"  /* r12 = Y */
    ".Lo_BD:\n mov r12, r5\n"             /* r12 = X */
    ".Labxy:\n"
    "str r12, [sp, #" S(S2) "]\n"  /* S2 = index value */
    "bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
    "bl .Lfetch\n ldr r1, [sp, #" S(S0) "]\n orr r1, r1, r0, lsl #8\n"
    "str r1, [sp, #" S(S0) "]\n"  /* S0 = base */
    /* page cross check */
    "tst r9, #0x10\n beq 1f\n"  /* XF=0 -> idle */
    "ldr r0, [sp, #" S(S2) "]\n"
    "add r2, r1, r0\n lsr r2, r2, #8\n lsr r1, r1, #8\n cmp r1, r2\n beq 2f\n"
    "1: bl .Lidl\n"
    "2: ldr r0, [sp, #" S(FDB) "]\n lsl r0, r0, #16\n"
    "ldr r1, [sp, #" S(S0) "]\n add r0, r0, r1\n"
    "ldr r1, [sp, #" S(S2) "]\n add r0, r0, r1\n"
    "bic r1, r0, #0xFF000000\n"  /* ea */
    "tst r9, #0x20\n beq .Labxy16\n"
    "bl .Lint\n bl .Lrd\n bfi r4, r0, #0, #8\n bl .Lzn8\n b .Lnext\n"
    ".Labxy16:\n"
    "str r1, [sp, #" S(S0) "]\n bl .Lrd\n str r0, [sp, #" S(S1) "]\n"
    "ldr r1, [sp, #" S(S0) "]\n adds r1, #1\n bic r1, r1, #0xFF000000\n"
    "bl .Lint\n bl .Lrd\n"
    "lsl r0, r0, #8\n ldr r1, [sp, #" S(S1) "]\n orr r4, r1, r0\n"
    "mov r0, r4\n bl .Lzn16\n b .Lnext\n"

    /* ---- STA dp (85) ---- */
    ".Lo_85:\n"
    "bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
    "ldr r0, [sp, #" S(FDP) "]\n tst r0, #0xFF\n beq 1f\n bl .Lidl\n"
    "1: ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
    "add r1, r1, r0\n uxth r1, r1\n"
    "tst r9, #0x20\n beq .L85_16\n"
    "uxtb r2, r4\n bl .Lint\n bl .Lwr\n b .Lnext\n"
    ".L85_16:\n"
    "str r1, [sp, #" S(S1) "]\n"  /* S1 = ea_lo */
    "uxtb r2, r4\n bl .Lwr\n"
    "ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
    "add r1, r1, r0\n adds r1, #1\n uxth r1, r1\n"
    "lsr r2, r4, #8\n uxtb r2, r2\n bl .Lint\n bl .Lwr\n b .Lnext\n"

    /* ---- STA abs (8D) ---- */
    ".Lo_8D:\n"
    "bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
    "bl .Lfetch\n ldr r1, [sp, #" S(S0) "]\n orr r1, r1, r0, lsl #8\n"
    "ldr r0, [sp, #" S(FDB) "]\n add r1, r1, r0, lsl #16\n"
    "tst r9, #0x20\n beq .L8d_16\n"
    "uxtb r2, r4\n bl .Lint\n bl .Lwr\n b .Lnext\n"
    ".L8d_16:\n"
    "str r1, [sp, #" S(S0) "]\n uxtb r2, r4\n bl .Lwr\n"
    "ldr r1, [sp, #" S(S0) "]\n adds r1, #1\n bic r1, r1, #0xFF000000\n"
    "lsr r2, r4, #8\n uxtb r2, r2\n bl .Lint\n bl .Lwr\n b .Lnext\n"

    /* ---- STA dp,X (95) ---- */
    ".Lo_95:\n"
    "bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
    "ldr r0, [sp, #" S(FDP) "]\n tst r0, #0xFF\n beq 1f\n bl .Lidl\n"
    "1: bl .Lidl\n"
    "ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
    "add r1, r1, r0\n add r1, r1, r5\n uxth r1, r1\n"
    "tst r9, #0x20\n beq .L95_16\n"
    "uxtb r2, r4\n bl .Lint\n bl .Lwr\n b .Lnext\n"
    ".L95_16:\n"
    "str r1, [sp, #" S(S1) "]\n uxtb r2, r4\n bl .Lwr\n"
    "ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
    "add r1, r1, r0\n add r1, r1, r5\n adds r1, #1\n uxth r1, r1\n"
    "lsr r2, r4, #8\n uxtb r2, r2\n bl .Lint\n bl .Lwr\n b .Lnext\n"

    /* ---- STA abs,Y (99) / STA abs,X (9D) ---- */
    ".Lo_99:\n mov r12, r6\n b .Lstabxy\n"
    ".Lo_9D:\n mov r12, r5\n"
    ".Lstabxy:\n"
    "str r12, [sp, #" S(S2) "]\n"
    "bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
    "bl .Lfetch\n ldr r1, [sp, #" S(S0) "]\n orr r1, r1, r0, lsl #8\n"
    "str r1, [sp, #" S(S0) "]\n"
    "bl .Lidl\n"  /* write: always extra */
    "ldr r0, [sp, #" S(FDB) "]\n lsl r0, r0, #16\n"
    "ldr r1, [sp, #" S(S0) "]\n add r0, r0, r1\n"
    "ldr r1, [sp, #" S(S2) "]\n add r0, r0, r1\n"
    "bic r1, r0, #0xFF000000\n"
    "tst r9, #0x20\n beq .Lstab16\n"
    "uxtb r2, r4\n bl .Lint\n bl .Lwr\n b .Lnext\n"
    ".Lstab16:\n"
    "str r1, [sp, #" S(S0) "]\n uxtb r2, r4\n bl .Lwr\n"
    "ldr r1, [sp, #" S(S0) "]\n adds r1, #1\n bic r1, r1, #0xFF000000\n"
    "lsr r2, r4, #8\n uxtb r2, r2\n bl .Lint\n bl .Lwr\n b .Lnext\n"

    /* ---- LDX imm (A2) ---- */
    ".Lo_A2:\n"
    "tst r9, #0x10\n beq .La2_16\n"
    "ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n"
    "add r8, r8, #1\n uxth r8, r8\n"
    "bl .Lint\n bl .Lrd\n mov r5, r0\n bl .Lzn8\n b .Lnext\n"
    ".La2_16:\n"
    "ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n"
    "add r8, r8, #1\n uxth r8, r8\n bl .Lrd\n str r0, [sp, #" S(S0) "]\n"
    "ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n"
    "add r8, r8, #1\n uxth r8, r8\n bl .Lint\n bl .Lrd\n"
    "lsl r0, r0, #8\n ldr r1, [sp, #" S(S0) "]\n orr r5, r1, r0\n"
    "mov r0, r5\n bl .Lzn16\n b .Lnext\n"

    /* ---- LDX dp (A6) ---- */
    ".Lo_A6:\n"
    "bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
    "ldr r0, [sp, #" S(FDP) "]\n tst r0, #0xFF\n beq 1f\n bl .Lidl\n"
    "1: ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
    "add r1, r1, r0\n uxth r1, r1\n"
    "tst r9, #0x10\n beq .La6_16\n"
    "bl .Lint\n bl .Lrd\n mov r5, r0\n bl .Lzn8\n b .Lnext\n"
    ".La6_16:\n"
    "bl .Lrd\n str r0, [sp, #" S(S1) "]\n"
    "ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
    "add r1, r1, r0\n adds r1, #1\n uxth r1, r1\n"
    "bl .Lint\n bl .Lrd\n"
    "lsl r0, r0, #8\n ldr r1, [sp, #" S(S1) "]\n orr r5, r1, r0\n"
    "mov r0, r5\n bl .Lzn16\n b .Lnext\n"

    /* ---- LDX abs (AE) ---- */
    ".Lo_AE:\n"
    "bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
    "bl .Lfetch\n ldr r1, [sp, #" S(S0) "]\n orr r1, r1, r0, lsl #8\n"
    "ldr r0, [sp, #" S(FDB) "]\n add r1, r1, r0, lsl #16\n"
    "tst r9, #0x10\n beq .Lae_16\n"
    "bl .Lint\n bl .Lrd\n mov r5, r0\n bl .Lzn8\n b .Lnext\n"
    ".Lae_16:\n"
    "str r1, [sp, #" S(S0) "]\n bl .Lrd\n str r0, [sp, #" S(S1) "]\n"
    "ldr r1, [sp, #" S(S0) "]\n adds r1, #1\n bic r1, r1, #0xFF000000\n"
    "bl .Lint\n bl .Lrd\n"
    "lsl r0, r0, #8\n ldr r1, [sp, #" S(S1) "]\n orr r5, r1, r0\n"
    "mov r0, r5\n bl .Lzn16\n b .Lnext\n"

    /* ---- LDY imm (A0) ---- */
    ".Lo_A0:\n"
    "tst r9, #0x10\n beq .La0_16\n"
    "ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n"
    "add r8, r8, #1\n uxth r8, r8\n"
    "bl .Lint\n bl .Lrd\n mov r6, r0\n bl .Lzn8\n b .Lnext\n"
    ".La0_16:\n"
    "ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n"
    "add r8, r8, #1\n uxth r8, r8\n bl .Lrd\n str r0, [sp, #" S(S0) "]\n"
    "ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n"
    "add r8, r8, #1\n uxth r8, r8\n bl .Lint\n bl .Lrd\n"
    "lsl r0, r0, #8\n ldr r1, [sp, #" S(S0) "]\n orr r6, r1, r0\n"
    "mov r0, r6\n bl .Lzn16\n b .Lnext\n"

    /* ---- LDY dp (A4) ---- */
    ".Lo_A4:\n"
    "bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
    "ldr r0, [sp, #" S(FDP) "]\n tst r0, #0xFF\n beq 1f\n bl .Lidl\n"
    "1: ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
    "add r1, r1, r0\n uxth r1, r1\n"
    "tst r9, #0x10\n beq .La4_16\n"
    "bl .Lint\n bl .Lrd\n mov r6, r0\n bl .Lzn8\n b .Lnext\n"
    ".La4_16:\n"
    "bl .Lrd\n str r0, [sp, #" S(S1) "]\n"
    "ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
    "add r1, r1, r0\n adds r1, #1\n uxth r1, r1\n"
    "bl .Lint\n bl .Lrd\n"
    "lsl r0, r0, #8\n ldr r1, [sp, #" S(S1) "]\n orr r6, r1, r0\n"
    "mov r0, r6\n bl .Lzn16\n b .Lnext\n"

    /* ---- LDY abs (AC) ---- */
    ".Lo_AC:\n"
    "bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
    "bl .Lfetch\n ldr r1, [sp, #" S(S0) "]\n orr r1, r1, r0, lsl #8\n"
    "ldr r0, [sp, #" S(FDB) "]\n add r1, r1, r0, lsl #16\n"
    "tst r9, #0x10\n beq .Lac_16\n"
    "bl .Lint\n bl .Lrd\n mov r6, r0\n bl .Lzn8\n b .Lnext\n"
    ".Lac_16:\n"
    "str r1, [sp, #" S(S0) "]\n bl .Lrd\n str r0, [sp, #" S(S1) "]\n"
    "ldr r1, [sp, #" S(S0) "]\n adds r1, #1\n bic r1, r1, #0xFF000000\n"
    "bl .Lint\n bl .Lrd\n"
    "lsl r0, r0, #8\n ldr r1, [sp, #" S(S1) "]\n orr r6, r1, r0\n"
    "mov r0, r6\n bl .Lzn16\n b .Lnext\n"

    /* ---- BRANCHES ---- */
    ".Lo_D0:\n tst r9, #0x02\n beq .Lbtk\n b .Lbnt\n"  /* BNE */
    ".Lo_F0:\n tst r9, #0x02\n bne .Lbtk\n b .Lbnt\n"  /* BEQ */
    ".Lo_10:\n tst r9, #0x80\n beq .Lbtk\n b .Lbnt\n"  /* BPL */
    ".Lo_30:\n tst r9, #0x80\n bne .Lbtk\n b .Lbnt\n"  /* BMI */
    ".Lo_90:\n tst r9, #0x01\n beq .Lbtk\n b .Lbnt\n"  /* BCC */
    ".Lo_B0:\n tst r9, #0x01\n bne .Lbtk\n b .Lbnt\n"  /* BCS */
    ".Lbnt:\n bl .Lint\n bl .Lfetch\n b .Lnext\n"
    ".Lbtk:\n"
    "bl .Lfetch\n sxtb r0, r0\n str r0, [sp, #" S(S0) "]\n"
    "bl .Lint\n bl .Lidl\n"
    "ldr r0, [sp, #" S(S0) "]\n add r8, r8, r0\n uxth r8, r8\n b .Lnext\n"

    /* ---- CMP imm (C9) ---- */
    ".Lo_C9:\n"
    "tst r9, #0x20\n beq .Lc9_16\n"
    "ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n"
    "add r8, r8, #1\n uxth r8, r8\n"
    "bl .Lint\n bl .Lrd\n b .Lcmp8\n"
    ".Lc9_16:\n"
    "ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n"
    "add r8, r8, #1\n uxth r8, r8\n bl .Lrd\n str r0, [sp, #" S(S0) "]\n"
    "ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n"
    "add r8, r8, #1\n uxth r8, r8\n bl .Lint\n bl .Lrd\n"
    "lsl r0, r0, #8\n ldr r1, [sp, #" S(S0) "]\n orr r0, r1, r0\n"
    "b .Lcmp16\n"

    /* ---- CMP dp (C5) ---- */
    ".Lo_C5:\n"
    "bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
    "ldr r0, [sp, #" S(FDP) "]\n tst r0, #0xFF\n beq 1f\n bl .Lidl\n"
    "1: ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
    "add r1, r1, r0\n uxth r1, r1\n"
    "tst r9, #0x20\n beq .Lc5_16\n"
    "bl .Lint\n bl .Lrd\n b .Lcmp8\n"
    ".Lc5_16:\n"
    "bl .Lrd\n str r0, [sp, #" S(S1) "]\n"
    "ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
    "add r1, r1, r0\n adds r1, #1\n uxth r1, r1\n"
    "bl .Lint\n bl .Lrd\n"
    "lsl r0, r0, #8\n ldr r1, [sp, #" S(S1) "]\n orr r0, r1, r0\n"
    "b .Lcmp16\n"

    /* ---- CMP abs (CD) ---- */
    ".Lo_CD:\n"
    "bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
    "bl .Lfetch\n ldr r1, [sp, #" S(S0) "]\n orr r1, r1, r0, lsl #8\n"
    "ldr r0, [sp, #" S(FDB) "]\n add r1, r1, r0, lsl #16\n"
    "tst r9, #0x20\n beq .Lcd_16\n"
    "bl .Lint\n bl .Lrd\n b .Lcmp8\n"
    ".Lcd_16:\n"
    "str r1, [sp, #" S(S0) "]\n bl .Lrd\n str r0, [sp, #" S(S1) "]\n"
    "ldr r1, [sp, #" S(S0) "]\n adds r1, #1\n bic r1, r1, #0xFF000000\n"
    "bl .Lint\n bl .Lrd\n"
    "lsl r0, r0, #8\n ldr r1, [sp, #" S(S1) "]\n orr r0, r1, r0\n"
    "b .Lcmp16\n"

    /* CMP finish — R0 = operand */
    ".Lcmp8:\n"
    "eor r0, r0, #0xFF\n and r1, r4, #0xFF\n adds r0, r1, r0\n adds r0, #1\n"
    "bic r9, r9, #0x83\n"  /* clear C,Z,N */
    "cmp r0, #0x100\n blo 1f\n orr r9, r9, #0x01\n"
    "1: tst r0, #0xFF\n  bne 1f\n orr r9, r9, #0x02\n"
    "1: tst r0, #0x80\n  beq 1f\n orr r9, r9, #0x80\n"
    "1: b .Lnext\n"

    ".Lcmp16:\n"
    "movw r1, #0xFFFF\n eor r0, r0, r1\n uxth r1, r4\n"
    "adds r0, r1, r0\n adds r0, #1\n"
    "bic r9, r9, #0x83\n"
    "movw r1, #0\n movt r1, #1\n cmp r0, r1\n blo 1f\n orr r9, r9, #0x01\n"
    "1: uxth r1, r0\n cbnz r1, 1f\n orr r9, r9, #0x02\n"
    "1: tst r0, #0x8000\n beq 1f\n orr r9, r9, #0x80\n"
    "1: b .Lnext\n"

    /* ---- ADC imm (69) — binary only ---- */
    ".Lo_69:\n"
    "tst r9, #0x08\n bne .Lfb\n"  /* decimal -> fallback */
    "tst r9, #0x20\n beq .L69_16\n"
    /* 8-bit */
    "ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n"
    "add r8, r8, #1\n uxth r8, r8\n"
    "bl .Lint\n bl .Lrd\n"  /* r0=val */
    "and  r1, r4, #0xFF\n"  /* r1=A lo */
    "and  r2, r9, #1\n"     /* r2=C */
    "add  r3, r1, r0\n add r3, r3, r2\n"  /* r3=result */
    /* V = ~(A^val) & (A^res) & 0x80 */
    "eor  r2, r1, r0\n mvn r2, r2\n eor r12, r1, r3\n and r2, r2, r12\n"
    "bic  r9, r9, #0xC3\n"  /* clear C,Z,V,N */
    "tst  r2, #0x80\n beq 1f\n orr r9, r9, #0x40\n"
    "1: cmp r3, #0x100\n blo 1f\n orr r9, r9, #0x01\n"
    "1: tst r3, #0xFF\n  bne 1f\n orr r9, r9, #0x02\n"
    "1: tst r3, #0x80\n  beq 1f\n orr r9, r9, #0x80\n"
    "1: bfi r4, r3, #0, #8\n b .Lnext\n"

    ".L69_16:\n"
    "ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n"
    "add r8, r8, #1\n uxth r8, r8\n bl .Lrd\n str r0, [sp, #" S(S0) "]\n"
    "ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n"
    "add r8, r8, #1\n uxth r8, r8\n bl .Lint\n bl .Lrd\n"
    "lsl r0, r0, #8\n ldr r1, [sp, #" S(S0) "]\n orr r0, r1, r0\n"  /* r0=16-bit val */
    "uxth r1, r4\n and r2, r9, #1\n"
    "add  r3, r1, r0\n add r3, r3, r2\n"
    "eor  r2, r1, r0\n mvn r2, r2\n eor r12, r1, r3\n and r2, r2, r12\n"
    "bic  r9, r9, #0xC3\n"
    "tst  r2, #0x8000\n beq 1f\n orr r9, r9, #0x40\n"
    "1: movw r2, #0\n movt r2, #1\n cmp r3, r2\n blo 1f\n orr r9, r9, #0x01\n"
    "1: uxth r2, r3\n cbnz r2, 1f\n orr r9, r9, #0x02\n"
    "1: tst r3, #0x8000\n beq 1f\n orr r9, r9, #0x80\n"
    "1: uxth r4, r3\n b .Lnext\n"

    /* ---- ADC dp (65) — binary only ---- */
    ".Lo_65:\n"
    "tst r9, #0x08\n bne .Lfb\n"
    "bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
    "ldr r0, [sp, #" S(FDP) "]\n tst r0, #0xFF\n beq 1f\n bl .Lidl\n"
    "1: ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
    "add r1, r1, r0\n uxth r1, r1\n"
    "tst r9, #0x20\n beq .L65_16\n"
    /* 8-bit */
    "bl .Lint\n bl .Lrd\n"
    "and  r1, r4, #0xFF\n and r2, r9, #1\n"
    "add  r3, r1, r0\n add r3, r3, r2\n"
    "eor  r2, r1, r0\n mvn r2, r2\n eor r12, r1, r3\n and r2, r2, r12\n"
    "bic  r9, r9, #0xC3\n"
    "tst  r2, #0x80\n beq 1f\n orr r9, r9, #0x40\n"
    "1: cmp r3, #0x100\n blo 1f\n orr r9, r9, #0x01\n"
    "1: tst r3, #0xFF\n  bne 1f\n orr r9, r9, #0x02\n"
    "1: tst r3, #0x80\n  beq 1f\n orr r9, r9, #0x80\n"
    "1: bfi r4, r3, #0, #8\n b .Lnext\n"
    ".L65_16:\n"
    "bl .Lrd\n str r0, [sp, #" S(S1) "]\n"
    "ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
    "add r1, r1, r0\n adds r1, #1\n uxth r1, r1\n"
    "bl .Lint\n bl .Lrd\n"
    "lsl r0, r0, #8\n ldr r1, [sp, #" S(S1) "]\n orr r0, r1, r0\n"
    "uxth r1, r4\n and r2, r9, #1\n"
    "add  r3, r1, r0\n add r3, r3, r2\n"
    "eor  r2, r1, r0\n mvn r2, r2\n eor r12, r1, r3\n and r2, r2, r12\n"
    "bic  r9, r9, #0xC3\n"
    "tst  r2, #0x8000\n beq 1f\n orr r9, r9, #0x40\n"
    "1: movw r2, #0\n movt r2, #1\n cmp r3, r2\n blo 1f\n orr r9, r9, #0x01\n"
    "1: uxth r2, r3\n cbnz r2, 1f\n orr r9, r9, #0x02\n"
    "1: tst r3, #0x8000\n beq 1f\n orr r9, r9, #0x80\n"
    "1: uxth r4, r3\n b .Lnext\n"

    /* ---- JSR abs (20) ---- */
    ".Lo_20:\n"
    "bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"  /* S0=lo adr */
    "bl .Lfetch\n str r0, [sp, #" S(S1) "]\n"  /* S1=hi adr */
    "bl .Lidl\n"
    /* ret = PC-1 */
    "sub r0, r8, #1\n uxth r0, r0\n str r0, [sp, #" S(S2) "]\n"
    /* Push high byte of ret */
    "lsr r2, r0, #8\n uxtb r2, r2\n mov r1, r7\n bl .Lwr\n"
    "subs r7, #1\n uxth r7, r7\n"
    "ldr r0, [sp, #" S(FE) "]\n cbz r0, 1f\n"
    "and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
    "1: bl .Lint\n"
    /* Push low byte of ret */
    "ldr r0, [sp, #" S(S2) "]\n uxtb r2, r0\n mov r1, r7\n bl .Lwr\n"
    "subs r7, #1\n uxth r7, r7\n"
    "ldr r0, [sp, #" S(FE) "]\n cbz r0, 1f\n"
    "and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
    "1: ldr r0, [sp, #" S(S0) "]\n ldr r1, [sp, #" S(S1) "]\n"
    "orr r8, r0, r1, lsl #8\n uxth r8, r8\n b .Lnext\n"

    /* ---- RTS (60) ---- */
    ".Lo_60:\n"
    "bl .Lidl\n bl .Lidl\n"
    /* Pull low byte */
    "adds r7, #1\n uxth r7, r7\n"
    "ldr r0, [sp, #" S(FE) "]\n cbz r0, 1f\n"
    "and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
    "1: mov r1, r7\n bl .Lrd\n str r0, [sp, #" S(S0) "]\n"
    /* Pull high byte */
    "adds r7, #1\n uxth r7, r7\n"
    "ldr r0, [sp, #" S(FE) "]\n cbz r0, 1f\n"
    "and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
    "1: mov r1, r7\n bl .Lrd\n"
    "lsl r0, r0, #8\n ldr r1, [sp, #" S(S0) "]\n orr r8, r1, r0\n"
    "adds r8, #1\n uxth r8, r8\n"
    "bl .Lint\n bl .Lidl\n b .Lnext\n"

    /* === EPILOGUE === */
    ".Ldone:\n"
    "bl .Lstore\n"
    "ldr r0, [sp, #" S(FCNT) "]\n"
    "add sp, sp, #" S(FSZ) "\n"
    "pop {r4-r11, pc}\n"

    ".ltorg\n"
    );
}

#else /* !ARM */

extern void cpu_runOpcode(Cpu* cpu);

int cpu_runBatchAsm(Cpu *cpu, int maxOpcodes) {
    int count;
    for (count = 0; count < maxOpcodes; count++) {
        cpu_runOpcode(cpu);
        if (cpu->stopped) break;
    }
    return count;
}

#endif
