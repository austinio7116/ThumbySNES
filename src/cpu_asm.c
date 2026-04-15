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
extern void    snes_flushCycles_extern(Snes *snes);
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
    "ldr r2, =snes_flushCycles_extern\n str r2, [sp, #" S(FFL) "]\n"
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
    "cmp r0, r1\n bge.w .Ldone\n"

    /* Check special states */
    "ldrb r0, [r10, #" S(CPU_RESETWANTED) "]\n cmp r0, #0\n bne.w .Lspecial\n"
    "ldrb r0, [r10, #" S(CPU_STOPPED) "]\n     cmp r0, #0\n bne.w .Lspecial\n"
    "ldrb r0, [r10, #" S(CPU_WAITING) "]\n      cmp r0, #0\n bne.w .Lspecial\n"
    "ldrb r0, [r10, #" S(CPU_INTWANTED) "]\n    cmp r0, #0\n bne.w .Lspecial\n"

    /* Flush pending cycles */
    "mov r0, r11\n ldr r3, [sp, #" S(FFL) "]\n blx r3\n"

    /* Fetch opcode at (K<<16)|PC */
    "ldr r1, [sp, #" S(FK) "]\n"
    "lsl r1, r1, #16\n orr r1, r1, r8\n"
    "add r8, r8, #1\n uxth r8, r8\n"
    "mov r0, r11\n ldr r3, [sp, #" S(FRD) "]\n blx r3\n"
    /* R0 = opcode */

    /* --- Jump table dispatch (256 entries, zero fallback) --- */
    "ldr  r12, =.Ljt\n"
    "ldr  r0, [r12, r0, lsl #2]\n"
    "bx   r0\n"

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
    ".ltorg\n"
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
    "cmp r0, #0\n bne 1f\n"
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
    "uxth r1, r0\n cmp r1, #0\n bne 1f\n orr r9, r9, #0x02\n"
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
    "tst r9, #0x20\n beq.w .La9_16\n"
    /* 8-bit */
    "ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n"
    "add r8, r8, #1\n uxth r8, r8\n"
    "bl .Lint\n bl .Lrd\n"
    "bfi r4, r0, #0, #8\n bl .Lzn8\n b.w .Lnext\n"
    ".La9_16:\n"
    "ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n"
    "add r8, r8, #1\n uxth r8, r8\n"
    "bl .Lrd\n str r0, [sp, #" S(S0) "]\n"  /* S0=low */
    "ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n"
    "add r8, r8, #1\n uxth r8, r8\n"
    "bl .Lint\n bl .Lrd\n"
    "lsl r0, r0, #8\n ldr r1, [sp, #" S(S0) "]\n orr r4, r1, r0\n"
    "mov r0, r4\n bl .Lzn16\n b.w .Lnext\n"

    /* ---- LDA dp (A5) ---- */
    ".Lo_A5:\n"
    "bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"  /* S0=dp offset */
    "ldr r0, [sp, #" S(FDP) "]\n tst r0, #0xFF\n beq 1f\n bl .Lidl\n"
    "1: ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
    "add r1, r1, r0\n uxth r1, r1\n"
    "tst r9, #0x20\n beq.w .La5_16\n"
    "bl .Lint\n bl .Lrd\n bfi r4, r0, #0, #8\n bl .Lzn8\n b.w .Lnext\n"
    ".La5_16:\n"
    "bl .Lrd\n str r0, [sp, #" S(S1) "]\n"  /* S1=low byte */
    "ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
    "add r1, r1, r0\n adds r1, #1\n uxth r1, r1\n"
    "bl .Lint\n bl .Lrd\n"
    "lsl r0, r0, #8\n ldr r1, [sp, #" S(S1) "]\n orr r4, r1, r0\n"
    "mov r0, r4\n bl .Lzn16\n b.w .Lnext\n"

    /* ---- LDA abs (AD) ---- */
    ".Lo_AD:\n"
    "bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"  /* S0=lo adr */
    "bl .Lfetch\n"  /* r0=hi adr */
    "ldr r1, [sp, #" S(S0) "]\n orr r1, r1, r0, lsl #8\n"
    "ldr r0, [sp, #" S(FDB) "]\n add r1, r1, r0, lsl #16\n"  /* ea */
    "tst r9, #0x20\n beq.w .Lad_16\n"
    "bl .Lint\n bl .Lrd\n bfi r4, r0, #0, #8\n bl .Lzn8\n b.w .Lnext\n"
    ".Lad_16:\n"
    "str r1, [sp, #" S(S0) "]\n bl .Lrd\n str r0, [sp, #" S(S1) "]\n"
    "ldr r1, [sp, #" S(S0) "]\n adds r1, #1\n bic r1, r1, #0xFF000000\n"
    "bl .Lint\n bl .Lrd\n"
    "lsl r0, r0, #8\n ldr r1, [sp, #" S(S1) "]\n orr r4, r1, r0\n"
    "mov r0, r4\n bl .Lzn16\n b.w .Lnext\n"

    /* ---- LDA dp,X (B5) ---- */
    ".Lo_B5:\n"
    "bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
    "ldr r0, [sp, #" S(FDP) "]\n tst r0, #0xFF\n beq 1f\n bl .Lidl\n"
    "1: bl .Lidl\n"  /* indexed */
    "ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
    "add r1, r1, r0\n add r1, r1, r5\n uxth r1, r1\n"
    "tst r9, #0x20\n beq.w .Lb5_16\n"
    "bl .Lint\n bl .Lrd\n bfi r4, r0, #0, #8\n bl .Lzn8\n b.w .Lnext\n"
    ".Lb5_16:\n"
    "bl .Lrd\n str r0, [sp, #" S(S1) "]\n"
    "ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
    "add r1, r1, r0\n add r1, r1, r5\n adds r1, #1\n uxth r1, r1\n"
    "bl .Lint\n bl .Lrd\n"
    "lsl r0, r0, #8\n ldr r1, [sp, #" S(S1) "]\n orr r4, r1, r0\n"
    "mov r0, r4\n bl .Lzn16\n b.w .Lnext\n"

    /* ---- LDA abs,Y (B9) / LDA abs,X (BD) ---- */
    ".Lo_B9:\n mov r12, r6\n b.w .Labxy\n"  /* r12 = Y */
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
    "tst r9, #0x20\n beq.w .Labxy16\n"
    "bl .Lint\n bl .Lrd\n bfi r4, r0, #0, #8\n bl .Lzn8\n b.w .Lnext\n"
    ".Labxy16:\n"
    "str r1, [sp, #" S(S0) "]\n bl .Lrd\n str r0, [sp, #" S(S1) "]\n"
    "ldr r1, [sp, #" S(S0) "]\n adds r1, #1\n bic r1, r1, #0xFF000000\n"
    "bl .Lint\n bl .Lrd\n"
    "lsl r0, r0, #8\n ldr r1, [sp, #" S(S1) "]\n orr r4, r1, r0\n"
    "mov r0, r4\n bl .Lzn16\n b.w .Lnext\n"

    /* ---- STA dp (85) ---- */
    ".Lo_85:\n"
    "bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
    "ldr r0, [sp, #" S(FDP) "]\n tst r0, #0xFF\n beq 1f\n bl .Lidl\n"
    "1: ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
    "add r1, r1, r0\n uxth r1, r1\n"
    "tst r9, #0x20\n beq.w .L85_16\n"
    "uxtb r2, r4\n bl .Lint\n bl .Lwr\n b.w .Lnext\n"
    ".L85_16:\n"
    "str r1, [sp, #" S(S1) "]\n"  /* S1 = ea_lo */
    "uxtb r2, r4\n bl .Lwr\n"
    "ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
    "add r1, r1, r0\n adds r1, #1\n uxth r1, r1\n"
    "lsr r2, r4, #8\n uxtb r2, r2\n bl .Lint\n bl .Lwr\n b.w .Lnext\n"

    /* ---- STA abs (8D) ---- */
    ".Lo_8D:\n"
    "bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
    "bl .Lfetch\n ldr r1, [sp, #" S(S0) "]\n orr r1, r1, r0, lsl #8\n"
    "ldr r0, [sp, #" S(FDB) "]\n add r1, r1, r0, lsl #16\n"
    "tst r9, #0x20\n beq.w .L8d_16\n"
    "uxtb r2, r4\n bl .Lint\n bl .Lwr\n b.w .Lnext\n"
    ".L8d_16:\n"
    "str r1, [sp, #" S(S0) "]\n uxtb r2, r4\n bl .Lwr\n"
    "ldr r1, [sp, #" S(S0) "]\n adds r1, #1\n bic r1, r1, #0xFF000000\n"
    "lsr r2, r4, #8\n uxtb r2, r2\n bl .Lint\n bl .Lwr\n b.w .Lnext\n"

    /* ---- STA dp,X (95) ---- */
    ".Lo_95:\n"
    "bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
    "ldr r0, [sp, #" S(FDP) "]\n tst r0, #0xFF\n beq 1f\n bl .Lidl\n"
    "1: bl .Lidl\n"
    "ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
    "add r1, r1, r0\n add r1, r1, r5\n uxth r1, r1\n"
    "tst r9, #0x20\n beq.w .L95_16\n"
    "uxtb r2, r4\n bl .Lint\n bl .Lwr\n b.w .Lnext\n"
    ".L95_16:\n"
    "str r1, [sp, #" S(S1) "]\n uxtb r2, r4\n bl .Lwr\n"
    "ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
    "add r1, r1, r0\n add r1, r1, r5\n adds r1, #1\n uxth r1, r1\n"
    "lsr r2, r4, #8\n uxtb r2, r2\n bl .Lint\n bl .Lwr\n b.w .Lnext\n"

    /* ---- STA abs,Y (99) / STA abs,X (9D) ---- */
    ".Lo_99:\n mov r12, r6\n b.w .Lstabxy\n"
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
    "tst r9, #0x20\n beq.w .Lstab16\n"
    "uxtb r2, r4\n bl .Lint\n bl .Lwr\n b.w .Lnext\n"
    ".Lstab16:\n"
    "str r1, [sp, #" S(S0) "]\n uxtb r2, r4\n bl .Lwr\n"
    "ldr r1, [sp, #" S(S0) "]\n adds r1, #1\n bic r1, r1, #0xFF000000\n"
    "lsr r2, r4, #8\n uxtb r2, r2\n bl .Lint\n bl .Lwr\n b.w .Lnext\n"

    /* ---- LDX imm (A2) ---- */
    ".Lo_A2:\n"
    "tst r9, #0x10\n beq.w .La2_16\n"
    "ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n"
    "add r8, r8, #1\n uxth r8, r8\n"
    "bl .Lint\n bl .Lrd\n mov r5, r0\n bl .Lzn8\n b.w .Lnext\n"
    ".La2_16:\n"
    "ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n"
    "add r8, r8, #1\n uxth r8, r8\n bl .Lrd\n str r0, [sp, #" S(S0) "]\n"
    "ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n"
    "add r8, r8, #1\n uxth r8, r8\n bl .Lint\n bl .Lrd\n"
    "lsl r0, r0, #8\n ldr r1, [sp, #" S(S0) "]\n orr r5, r1, r0\n"
    "mov r0, r5\n bl .Lzn16\n b.w .Lnext\n"

    /* ---- LDX dp (A6) ---- */
    ".Lo_A6:\n"
    "bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
    "ldr r0, [sp, #" S(FDP) "]\n tst r0, #0xFF\n beq 1f\n bl .Lidl\n"
    "1: ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
    "add r1, r1, r0\n uxth r1, r1\n"
    "tst r9, #0x10\n beq.w .La6_16\n"
    "bl .Lint\n bl .Lrd\n mov r5, r0\n bl .Lzn8\n b.w .Lnext\n"
    ".La6_16:\n"
    "bl .Lrd\n str r0, [sp, #" S(S1) "]\n"
    "ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
    "add r1, r1, r0\n adds r1, #1\n uxth r1, r1\n"
    "bl .Lint\n bl .Lrd\n"
    "lsl r0, r0, #8\n ldr r1, [sp, #" S(S1) "]\n orr r5, r1, r0\n"
    "mov r0, r5\n bl .Lzn16\n b.w .Lnext\n"

    /* ---- LDX abs (AE) ---- */
    ".Lo_AE:\n"
    "bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
    "bl .Lfetch\n ldr r1, [sp, #" S(S0) "]\n orr r1, r1, r0, lsl #8\n"
    "ldr r0, [sp, #" S(FDB) "]\n add r1, r1, r0, lsl #16\n"
    "tst r9, #0x10\n beq.w .Lae_16\n"
    "bl .Lint\n bl .Lrd\n mov r5, r0\n bl .Lzn8\n b.w .Lnext\n"
    ".Lae_16:\n"
    "str r1, [sp, #" S(S0) "]\n bl .Lrd\n str r0, [sp, #" S(S1) "]\n"
    "ldr r1, [sp, #" S(S0) "]\n adds r1, #1\n bic r1, r1, #0xFF000000\n"
    "bl .Lint\n bl .Lrd\n"
    "lsl r0, r0, #8\n ldr r1, [sp, #" S(S1) "]\n orr r5, r1, r0\n"
    "mov r0, r5\n bl .Lzn16\n b.w .Lnext\n"

    /* ---- LDY imm (A0) ---- */
    ".Lo_A0:\n"
    "tst r9, #0x10\n beq.w .La0_16\n"
    "ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n"
    "add r8, r8, #1\n uxth r8, r8\n"
    "bl .Lint\n bl .Lrd\n mov r6, r0\n bl .Lzn8\n b.w .Lnext\n"
    ".La0_16:\n"
    "ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n"
    "add r8, r8, #1\n uxth r8, r8\n bl .Lrd\n str r0, [sp, #" S(S0) "]\n"
    "ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n"
    "add r8, r8, #1\n uxth r8, r8\n bl .Lint\n bl .Lrd\n"
    "lsl r0, r0, #8\n ldr r1, [sp, #" S(S0) "]\n orr r6, r1, r0\n"
    "mov r0, r6\n bl .Lzn16\n b.w .Lnext\n"

    /* ---- LDY dp (A4) ---- */
    ".Lo_A4:\n"
    "bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
    "ldr r0, [sp, #" S(FDP) "]\n tst r0, #0xFF\n beq 1f\n bl .Lidl\n"
    "1: ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
    "add r1, r1, r0\n uxth r1, r1\n"
    "tst r9, #0x10\n beq.w .La4_16\n"
    "bl .Lint\n bl .Lrd\n mov r6, r0\n bl .Lzn8\n b.w .Lnext\n"
    ".La4_16:\n"
    "bl .Lrd\n str r0, [sp, #" S(S1) "]\n"
    "ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
    "add r1, r1, r0\n adds r1, #1\n uxth r1, r1\n"
    "bl .Lint\n bl .Lrd\n"
    "lsl r0, r0, #8\n ldr r1, [sp, #" S(S1) "]\n orr r6, r1, r0\n"
    "mov r0, r6\n bl .Lzn16\n b.w .Lnext\n"

    /* ---- LDY abs (AC) ---- */
    ".Lo_AC:\n"
    "bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
    "bl .Lfetch\n ldr r1, [sp, #" S(S0) "]\n orr r1, r1, r0, lsl #8\n"
    "ldr r0, [sp, #" S(FDB) "]\n add r1, r1, r0, lsl #16\n"
    "tst r9, #0x10\n beq.w .Lac_16\n"
    "bl .Lint\n bl .Lrd\n mov r6, r0\n bl .Lzn8\n b.w .Lnext\n"
    ".Lac_16:\n"
    "str r1, [sp, #" S(S0) "]\n bl .Lrd\n str r0, [sp, #" S(S1) "]\n"
    "ldr r1, [sp, #" S(S0) "]\n adds r1, #1\n bic r1, r1, #0xFF000000\n"
    "bl .Lint\n bl .Lrd\n"
    "lsl r0, r0, #8\n ldr r1, [sp, #" S(S1) "]\n orr r6, r1, r0\n"
    "mov r0, r6\n bl .Lzn16\n b.w .Lnext\n"

    /* ---- BRANCHES ---- */
    ".Lo_D0:\n tst r9, #0x02\n beq.w .Lbtk\n b.w .Lbnt\n"  /* BNE */
    ".Lo_F0:\n tst r9, #0x02\n bne.w .Lbtk\n b.w .Lbnt\n"  /* BEQ */
    ".Lo_10:\n tst r9, #0x80\n beq.w .Lbtk\n b.w .Lbnt\n"  /* BPL */
    ".Lo_30:\n tst r9, #0x80\n bne.w .Lbtk\n b.w .Lbnt\n"  /* BMI */
    ".Lo_90:\n tst r9, #0x01\n beq.w .Lbtk\n b.w .Lbnt\n"  /* BCC */
    ".Lo_B0:\n tst r9, #0x01\n bne.w .Lbtk\n b.w .Lbnt\n"  /* BCS */
    ".Lbnt:\n bl .Lint\n bl .Lfetch\n b.w .Lnext\n"
    ".Lbtk:\n"
    "bl .Lfetch\n sxtb r0, r0\n str r0, [sp, #" S(S0) "]\n"
    "bl .Lint\n bl .Lidl\n"
    "ldr r0, [sp, #" S(S0) "]\n add r8, r8, r0\n uxth r8, r8\n b.w .Lnext\n"

    /* ---- CMP imm (C9) ---- */
    ".Lo_C9:\n"
    "tst r9, #0x20\n beq.w .Lc9_16\n"
    "ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n"
    "add r8, r8, #1\n uxth r8, r8\n"
    "bl .Lint\n bl .Lrd\n b.w .Lcmp8\n"
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
    "tst r9, #0x20\n beq.w .Lc5_16\n"
    "bl .Lint\n bl .Lrd\n b.w .Lcmp8\n"
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
    "tst r9, #0x20\n beq.w .Lcd_16\n"
    "bl .Lint\n bl .Lrd\n b.w .Lcmp8\n"
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
    "1: b.w .Lnext\n"

    ".Lcmp16:\n"
    "movw r1, #0xFFFF\n eor r0, r0, r1\n uxth r1, r4\n"
    "adds r0, r1, r0\n adds r0, #1\n"
    "bic r9, r9, #0x83\n"
    "movw r1, #0\n movt r1, #1\n cmp r0, r1\n blo 1f\n orr r9, r9, #0x01\n"
    "1: uxth r1, r0\n cmp r1, #0\n bne 1f\n orr r9, r9, #0x02\n"
    "1: tst r0, #0x8000\n beq 1f\n orr r9, r9, #0x80\n"
    "1: b.w .Lnext\n"

    /* ---- ADC imm (69) — binary only ---- */
    ".Lo_69:\n"
    "tst r9, #0x08\n bne.w .Lfb\n"  /* decimal -> fallback */
    "tst r9, #0x20\n beq.w .L69_16\n"
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
    "1: bfi r4, r3, #0, #8\n b.w .Lnext\n"

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
    "1: uxth r2, r3\n cmp r2, #0\n bne 1f\n orr r9, r9, #0x02\n"
    "1: tst r3, #0x8000\n beq 1f\n orr r9, r9, #0x80\n"
    "1: uxth r4, r3\n b.w .Lnext\n"

    /* ---- ADC dp (65) — binary only ---- */
    ".Lo_65:\n"
    "tst r9, #0x08\n bne.w .Lfb\n"
    "bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
    "ldr r0, [sp, #" S(FDP) "]\n tst r0, #0xFF\n beq 1f\n bl .Lidl\n"
    "1: ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
    "add r1, r1, r0\n uxth r1, r1\n"
    "tst r9, #0x20\n beq.w .L65_16\n"
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
    "1: bfi r4, r3, #0, #8\n b.w .Lnext\n"
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
    "1: uxth r2, r3\n cmp r2, #0\n bne 1f\n orr r9, r9, #0x02\n"
    "1: tst r3, #0x8000\n beq 1f\n orr r9, r9, #0x80\n"
    "1: uxth r4, r3\n b.w .Lnext\n"

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
    "ldr r0, [sp, #" S(FE) "]\n cmp r0, #0\n beq 1f\n"
    "and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
    "1: bl .Lint\n"
    /* Push low byte of ret */
    "ldr r0, [sp, #" S(S2) "]\n uxtb r2, r0\n mov r1, r7\n bl .Lwr\n"
    "subs r7, #1\n uxth r7, r7\n"
    "ldr r0, [sp, #" S(FE) "]\n cmp r0, #0\n beq 1f\n"
    "and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
    "1: ldr r0, [sp, #" S(S0) "]\n ldr r1, [sp, #" S(S1) "]\n"
    "orr r8, r0, r1, lsl #8\n uxth r8, r8\n b.w .Lnext\n"

    /* ---- RTS (60) ---- */
    ".Lo_60:\n"
    "bl .Lidl\n bl .Lidl\n"
    /* Pull low byte */
    "adds r7, #1\n uxth r7, r7\n"
    "ldr r0, [sp, #" S(FE) "]\n cmp r0, #0\n beq 1f\n"
    "and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
    "1: mov r1, r7\n bl .Lrd\n str r0, [sp, #" S(S0) "]\n"
    /* Pull high byte */
    "adds r7, #1\n uxth r7, r7\n"
    "ldr r0, [sp, #" S(FE) "]\n cmp r0, #0\n beq 1f\n"
    "and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
    "1: mov r1, r7\n bl .Lrd\n"
    "lsl r0, r0, #8\n ldr r1, [sp, #" S(S0) "]\n orr r8, r1, r0\n"
    "adds r8, #1\n uxth r8, r8\n"
    "bl .Lint\n bl .Lidl\n b.w .Lnext\n"

    /* === EPILOGUE === */

    ".ltorg\n"
    /* ======== Group A opcodes ======== */
".Lo_09:\n"
"tst r9, #0x20\n beq.w .L09_16\n"
"ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n"
"add r8, r8, #1\n uxth r8, r8\n"
"bl .Lint\n bl .Lrd\n"
"uxtb r0, r0\n uxtb r1, r4\n orr r0, r1, r0\n bfi r4, r0, #0, #8\n"
"bl .Lzn8\n b.w .Lnext\n"
".L09_16:\n"
"ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n"
"add r8, r8, #1\n uxth r8, r8\n bl .Lrd\n str r0, [sp, #" S(S0) "]\n"
"ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n"
"add r8, r8, #1\n uxth r8, r8\n bl .Lint\n bl .Lrd\n"
"lsl r0, r0, #8\n ldr r1, [sp, #" S(S0) "]\n orr r0, r1, r0\n"
"uxth r1, r4\n orr r4, r1, r0\n uxth r4, r4\n"
"mov r0, r4\n bl .Lzn16\n b.w .Lnext\n"
".Lo_05:\n"
"bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
"ldr r0, [sp, #" S(FDP) "]\n tst r0, #0xFF\n beq 1f\n bl .Lidl\n"
"1: ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
"add r1, r1, r0\n uxth r1, r1\n"
"tst r9, #0x20\n beq.w .L05_16\n"
"bl .Lint\n bl .Lrd\n"
"uxtb r0, r0\n uxtb r1, r4\n orr r0, r1, r0\n bfi r4, r0, #0, #8\n"
"bl .Lzn8\n b.w .Lnext\n"
".L05_16:\n"
"bl .Lrd\n str r0, [sp, #" S(S1) "]\n"
"ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
"add r1, r1, r0\n adds r1, #1\n uxth r1, r1\n"
"bl .Lint\n bl .Lrd\n"
"lsl r0, r0, #8\n ldr r1, [sp, #" S(S1) "]\n orr r0, r1, r0\n"
"uxth r1, r4\n orr r4, r1, r0\n uxth r4, r4\n"
"mov r0, r4\n bl .Lzn16\n b.w .Lnext\n"
".Lo_0D:\n"
"bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
"bl .Lfetch\n ldr r1, [sp, #" S(S0) "]\n orr r1, r1, r0, lsl #8\n"
"ldr r0, [sp, #" S(FDB) "]\n add r1, r1, r0, lsl #16\n"
"tst r9, #0x20\n beq.w .L0D_16\n"
"bl .Lint\n bl .Lrd\n"
"uxtb r0, r0\n uxtb r1, r4\n orr r0, r1, r0\n bfi r4, r0, #0, #8\n"
"bl .Lzn8\n b.w .Lnext\n"
".L0D_16:\n"
"str r1, [sp, #" S(S0) "]\n bl .Lrd\n str r0, [sp, #" S(S1) "]\n"
"ldr r1, [sp, #" S(S0) "]\n adds r1, #1\n bic r1, r1, #0xFF000000\n"
"bl .Lint\n bl .Lrd\n"
"lsl r0, r0, #8\n ldr r1, [sp, #" S(S1) "]\n orr r0, r1, r0\n"
"uxth r1, r4\n orr r4, r1, r0\n uxth r4, r4\n"
"mov r0, r4\n bl .Lzn16\n b.w .Lnext\n"
".Lo_15:\n"
"bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
"ldr r0, [sp, #" S(FDP) "]\n tst r0, #0xFF\n beq 1f\n bl .Lidl\n"
"1: bl .Lidl\n"
"ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
"add r1, r1, r0\n add r1, r1, r5\n uxth r1, r1\n"
"tst r9, #0x20\n beq.w .L15_16\n"
"bl .Lint\n bl .Lrd\n"
"uxtb r0, r0\n uxtb r1, r4\n orr r0, r1, r0\n bfi r4, r0, #0, #8\n"
"bl .Lzn8\n b.w .Lnext\n"
".L15_16:\n"
"bl .Lrd\n str r0, [sp, #" S(S1) "]\n"
"ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
"add r1, r1, r0\n add r1, r1, r5\n adds r1, #1\n uxth r1, r1\n"
"bl .Lint\n bl .Lrd\n"
"lsl r0, r0, #8\n ldr r1, [sp, #" S(S1) "]\n orr r0, r1, r0\n"
"uxth r1, r4\n orr r4, r1, r0\n uxth r4, r4\n"
"mov r0, r4\n bl .Lzn16\n b.w .Lnext\n"
".Lo_19:\n mov r12, r6\n b.w .Loraxy\n"
".Lo_1D:\n mov r12, r5\n"
".Loraxy:\n"
"str r12, [sp, #" S(S2) "]\n"
"bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
"bl .Lfetch\n ldr r1, [sp, #" S(S0) "]\n orr r1, r1, r0, lsl #8\n"
"str r1, [sp, #" S(S0) "]\n"
"tst r9, #0x10\n beq 1f\n"
"ldr r0, [sp, #" S(S2) "]\n"
"add r2, r1, r0\n lsr r2, r2, #8\n lsr r1, r1, #8\n cmp r1, r2\n beq 2f\n"
"1: bl .Lidl\n"
"2: ldr r0, [sp, #" S(FDB) "]\n lsl r0, r0, #16\n"
"ldr r1, [sp, #" S(S0) "]\n add r0, r0, r1\n"
"ldr r1, [sp, #" S(S2) "]\n add r0, r0, r1\n"
"bic r1, r0, #0xFF000000\n"
"tst r9, #0x20\n beq.w .Loraxy16\n"
"bl .Lint\n bl .Lrd\n"
"uxtb r0, r0\n uxtb r1, r4\n orr r0, r1, r0\n bfi r4, r0, #0, #8\n"
"bl .Lzn8\n b.w .Lnext\n"
".Loraxy16:\n"
"str r1, [sp, #" S(S0) "]\n bl .Lrd\n str r0, [sp, #" S(S1) "]\n"
"ldr r1, [sp, #" S(S0) "]\n adds r1, #1\n bic r1, r1, #0xFF000000\n"
"bl .Lint\n bl .Lrd\n"
"lsl r0, r0, #8\n ldr r1, [sp, #" S(S1) "]\n orr r0, r1, r0\n"
"uxth r1, r4\n orr r4, r1, r0\n uxth r4, r4\n"
"mov r0, r4\n bl .Lzn16\n b.w .Lnext\n"
".Lo_29:\n"
"tst r9, #0x20\n beq.w .L29_16\n"
"ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n"
"add r8, r8, #1\n uxth r8, r8\n"
"bl .Lint\n bl .Lrd\n"
"uxtb r0, r0\n uxtb r1, r4\n and r0, r1, r0\n bfi r4, r0, #0, #8\n"
"bl .Lzn8\n b.w .Lnext\n"
".L29_16:\n"
"ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n"
"add r8, r8, #1\n uxth r8, r8\n bl .Lrd\n str r0, [sp, #" S(S0) "]\n"
"ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n"
"add r8, r8, #1\n uxth r8, r8\n bl .Lint\n bl .Lrd\n"
"lsl r0, r0, #8\n ldr r1, [sp, #" S(S0) "]\n orr r0, r1, r0\n"
"uxth r1, r4\n and r4, r1, r0\n uxth r4, r4\n"
"mov r0, r4\n bl .Lzn16\n b.w .Lnext\n"
".Lo_25:\n"
"bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
"ldr r0, [sp, #" S(FDP) "]\n tst r0, #0xFF\n beq 1f\n bl .Lidl\n"
"1: ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
"add r1, r1, r0\n uxth r1, r1\n"
"tst r9, #0x20\n beq.w .L25_16\n"
"bl .Lint\n bl .Lrd\n"
"uxtb r0, r0\n uxtb r1, r4\n and r0, r1, r0\n bfi r4, r0, #0, #8\n"
"bl .Lzn8\n b.w .Lnext\n"
".L25_16:\n"
"bl .Lrd\n str r0, [sp, #" S(S1) "]\n"
"ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
"add r1, r1, r0\n adds r1, #1\n uxth r1, r1\n"
"bl .Lint\n bl .Lrd\n"
"lsl r0, r0, #8\n ldr r1, [sp, #" S(S1) "]\n orr r0, r1, r0\n"
"uxth r1, r4\n and r4, r1, r0\n uxth r4, r4\n"
"mov r0, r4\n bl .Lzn16\n b.w .Lnext\n"
".Lo_2D:\n"
"bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
"bl .Lfetch\n ldr r1, [sp, #" S(S0) "]\n orr r1, r1, r0, lsl #8\n"
"ldr r0, [sp, #" S(FDB) "]\n add r1, r1, r0, lsl #16\n"
"tst r9, #0x20\n beq.w .L2D_16\n"
"bl .Lint\n bl .Lrd\n"
"uxtb r0, r0\n uxtb r1, r4\n and r0, r1, r0\n bfi r4, r0, #0, #8\n"
"bl .Lzn8\n b.w .Lnext\n"
".L2D_16:\n"
"str r1, [sp, #" S(S0) "]\n bl .Lrd\n str r0, [sp, #" S(S1) "]\n"
"ldr r1, [sp, #" S(S0) "]\n adds r1, #1\n bic r1, r1, #0xFF000000\n"
"bl .Lint\n bl .Lrd\n"
"lsl r0, r0, #8\n ldr r1, [sp, #" S(S1) "]\n orr r0, r1, r0\n"
"uxth r1, r4\n and r4, r1, r0\n uxth r4, r4\n"
"mov r0, r4\n bl .Lzn16\n b.w .Lnext\n"
".Lo_35:\n"
"bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
"ldr r0, [sp, #" S(FDP) "]\n tst r0, #0xFF\n beq 1f\n bl .Lidl\n"
"1: bl .Lidl\n"
"ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
"add r1, r1, r0\n add r1, r1, r5\n uxth r1, r1\n"
"tst r9, #0x20\n beq.w .L35_16\n"
"bl .Lint\n bl .Lrd\n"
"uxtb r0, r0\n uxtb r1, r4\n and r0, r1, r0\n bfi r4, r0, #0, #8\n"
"bl .Lzn8\n b.w .Lnext\n"
".L35_16:\n"
"bl .Lrd\n str r0, [sp, #" S(S1) "]\n"
"ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
"add r1, r1, r0\n add r1, r1, r5\n adds r1, #1\n uxth r1, r1\n"
"bl .Lint\n bl .Lrd\n"
"lsl r0, r0, #8\n ldr r1, [sp, #" S(S1) "]\n orr r0, r1, r0\n"
"uxth r1, r4\n and r4, r1, r0\n uxth r4, r4\n"
"mov r0, r4\n bl .Lzn16\n b.w .Lnext\n"
".Lo_39:\n mov r12, r6\n b.w .Landxy\n"
".Lo_3D:\n mov r12, r5\n"
".Landxy:\n"
"str r12, [sp, #" S(S2) "]\n"
"bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
"bl .Lfetch\n ldr r1, [sp, #" S(S0) "]\n orr r1, r1, r0, lsl #8\n"
"str r1, [sp, #" S(S0) "]\n"
"tst r9, #0x10\n beq 1f\n"
"ldr r0, [sp, #" S(S2) "]\n"
"add r2, r1, r0\n lsr r2, r2, #8\n lsr r1, r1, #8\n cmp r1, r2\n beq 2f\n"
"1: bl .Lidl\n"
"2: ldr r0, [sp, #" S(FDB) "]\n lsl r0, r0, #16\n"
"ldr r1, [sp, #" S(S0) "]\n add r0, r0, r1\n"
"ldr r1, [sp, #" S(S2) "]\n add r0, r0, r1\n"
"bic r1, r0, #0xFF000000\n"
"tst r9, #0x20\n beq.w .Landxy16\n"
"bl .Lint\n bl .Lrd\n"
"uxtb r0, r0\n uxtb r1, r4\n and r0, r1, r0\n bfi r4, r0, #0, #8\n"
"bl .Lzn8\n b.w .Lnext\n"
".Landxy16:\n"
"str r1, [sp, #" S(S0) "]\n bl .Lrd\n str r0, [sp, #" S(S1) "]\n"
"ldr r1, [sp, #" S(S0) "]\n adds r1, #1\n bic r1, r1, #0xFF000000\n"
"bl .Lint\n bl .Lrd\n"
"lsl r0, r0, #8\n ldr r1, [sp, #" S(S1) "]\n orr r0, r1, r0\n"
"uxth r1, r4\n and r4, r1, r0\n uxth r4, r4\n"
"mov r0, r4\n bl .Lzn16\n b.w .Lnext\n"
".Lo_49:\n"
"tst r9, #0x20\n beq.w .L49_16\n"
"ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n"
"add r8, r8, #1\n uxth r8, r8\n"
"bl .Lint\n bl .Lrd\n"
"uxtb r0, r0\n uxtb r1, r4\n eor r0, r1, r0\n bfi r4, r0, #0, #8\n"
"bl .Lzn8\n b.w .Lnext\n"
".L49_16:\n"
"ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n"
"add r8, r8, #1\n uxth r8, r8\n bl .Lrd\n str r0, [sp, #" S(S0) "]\n"
"ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n"
"add r8, r8, #1\n uxth r8, r8\n bl .Lint\n bl .Lrd\n"
"lsl r0, r0, #8\n ldr r1, [sp, #" S(S0) "]\n orr r0, r1, r0\n"
"uxth r1, r4\n eor r4, r1, r0\n uxth r4, r4\n"
"mov r0, r4\n bl .Lzn16\n b.w .Lnext\n"
".Lo_45:\n"
"bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
"ldr r0, [sp, #" S(FDP) "]\n tst r0, #0xFF\n beq 1f\n bl .Lidl\n"
"1: ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
"add r1, r1, r0\n uxth r1, r1\n"
"tst r9, #0x20\n beq.w .L45_16\n"
"bl .Lint\n bl .Lrd\n"
"uxtb r0, r0\n uxtb r1, r4\n eor r0, r1, r0\n bfi r4, r0, #0, #8\n"
"bl .Lzn8\n b.w .Lnext\n"
".L45_16:\n"
"bl .Lrd\n str r0, [sp, #" S(S1) "]\n"
"ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
"add r1, r1, r0\n adds r1, #1\n uxth r1, r1\n"
"bl .Lint\n bl .Lrd\n"
"lsl r0, r0, #8\n ldr r1, [sp, #" S(S1) "]\n orr r0, r1, r0\n"
"uxth r1, r4\n eor r4, r1, r0\n uxth r4, r4\n"
"mov r0, r4\n bl .Lzn16\n b.w .Lnext\n"
".Lo_4D:\n"
"bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
"bl .Lfetch\n ldr r1, [sp, #" S(S0) "]\n orr r1, r1, r0, lsl #8\n"
"ldr r0, [sp, #" S(FDB) "]\n add r1, r1, r0, lsl #16\n"
"tst r9, #0x20\n beq.w .L4D_16\n"
"bl .Lint\n bl .Lrd\n"
"uxtb r0, r0\n uxtb r1, r4\n eor r0, r1, r0\n bfi r4, r0, #0, #8\n"
"bl .Lzn8\n b.w .Lnext\n"
".L4D_16:\n"
"str r1, [sp, #" S(S0) "]\n bl .Lrd\n str r0, [sp, #" S(S1) "]\n"
"ldr r1, [sp, #" S(S0) "]\n adds r1, #1\n bic r1, r1, #0xFF000000\n"
"bl .Lint\n bl .Lrd\n"
"lsl r0, r0, #8\n ldr r1, [sp, #" S(S1) "]\n orr r0, r1, r0\n"
"uxth r1, r4\n eor r4, r1, r0\n uxth r4, r4\n"
"mov r0, r4\n bl .Lzn16\n b.w .Lnext\n"
".Lo_55:\n"
"bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
"ldr r0, [sp, #" S(FDP) "]\n tst r0, #0xFF\n beq 1f\n bl .Lidl\n"
"1: bl .Lidl\n"
"ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
"add r1, r1, r0\n add r1, r1, r5\n uxth r1, r1\n"
"tst r9, #0x20\n beq.w .L55_16\n"
"bl .Lint\n bl .Lrd\n"
"uxtb r0, r0\n uxtb r1, r4\n eor r0, r1, r0\n bfi r4, r0, #0, #8\n"
"bl .Lzn8\n b.w .Lnext\n"
".L55_16:\n"
"bl .Lrd\n str r0, [sp, #" S(S1) "]\n"
"ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
"add r1, r1, r0\n add r1, r1, r5\n adds r1, #1\n uxth r1, r1\n"
"bl .Lint\n bl .Lrd\n"
"lsl r0, r0, #8\n ldr r1, [sp, #" S(S1) "]\n orr r0, r1, r0\n"
"uxth r1, r4\n eor r4, r1, r0\n uxth r4, r4\n"
"mov r0, r4\n bl .Lzn16\n b.w .Lnext\n"
".Lo_59:\n mov r12, r6\n b.w .Leorxy\n"
".Lo_5D:\n mov r12, r5\n"
".Leorxy:\n"
"str r12, [sp, #" S(S2) "]\n"
"bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
"bl .Lfetch\n ldr r1, [sp, #" S(S0) "]\n orr r1, r1, r0, lsl #8\n"
"str r1, [sp, #" S(S0) "]\n"
"tst r9, #0x10\n beq 1f\n"
"ldr r0, [sp, #" S(S2) "]\n"
"add r2, r1, r0\n lsr r2, r2, #8\n lsr r1, r1, #8\n cmp r1, r2\n beq 2f\n"
"1: bl .Lidl\n"
"2: ldr r0, [sp, #" S(FDB) "]\n lsl r0, r0, #16\n"
"ldr r1, [sp, #" S(S0) "]\n add r0, r0, r1\n"
"ldr r1, [sp, #" S(S2) "]\n add r0, r0, r1\n"
"bic r1, r0, #0xFF000000\n"
"tst r9, #0x20\n beq.w .Leorxy16\n"
"bl .Lint\n bl .Lrd\n"
"uxtb r0, r0\n uxtb r1, r4\n eor r0, r1, r0\n bfi r4, r0, #0, #8\n"
"bl .Lzn8\n b.w .Lnext\n"
".Leorxy16:\n"
"str r1, [sp, #" S(S0) "]\n bl .Lrd\n str r0, [sp, #" S(S1) "]\n"
"ldr r1, [sp, #" S(S0) "]\n adds r1, #1\n bic r1, r1, #0xFF000000\n"
"bl .Lint\n bl .Lrd\n"
"lsl r0, r0, #8\n ldr r1, [sp, #" S(S1) "]\n orr r0, r1, r0\n"
"uxth r1, r4\n eor r4, r1, r0\n uxth r4, r4\n"
"mov r0, r4\n bl .Lzn16\n b.w .Lnext\n"
".Lo_18:\n"
"bl .Lint\n"
"ldrb r0, [r10, #" S(CPU_INTWANTED) "]\n"
"cmp r0, #0\n bne 1f\n bl .Lidl\n b 2f\n"
"1: ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n bl .Lrd\n"
"2: bic r9, r9, #0x01\n b.w .Lnext\n"
".Lo_38:\n"
"bl .Lint\n"
"ldrb r0, [r10, #" S(CPU_INTWANTED) "]\n"
"cmp r0, #0\n bne 1f\n bl .Lidl\n b 2f\n"
"1: ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n bl .Lrd\n"
"2: orr r9, r9, #0x01\n b.w .Lnext\n"
".Lo_58:\n"
"bl .Lint\n"
"ldrb r0, [r10, #" S(CPU_INTWANTED) "]\n"
"cmp r0, #0\n bne 1f\n bl .Lidl\n b 2f\n"
"1: ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n bl .Lrd\n"
"2: bic r9, r9, #0x04\n b.w .Lnext\n"
".Lo_78:\n"
"bl .Lint\n"
"ldrb r0, [r10, #" S(CPU_INTWANTED) "]\n"
"cmp r0, #0\n bne 1f\n bl .Lidl\n b 2f\n"
"1: ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n bl .Lrd\n"
"2: orr r9, r9, #0x04\n b.w .Lnext\n"
".Lo_D8:\n"
"bl .Lint\n"
"ldrb r0, [r10, #" S(CPU_INTWANTED) "]\n"
"cmp r0, #0\n bne 1f\n bl .Lidl\n b 2f\n"
"1: ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n bl .Lrd\n"
"2: bic r9, r9, #0x08\n b.w .Lnext\n"
".Lo_F8:\n"
"bl .Lint\n"
"ldrb r0, [r10, #" S(CPU_INTWANTED) "]\n"
"cmp r0, #0\n bne 1f\n bl .Lidl\n b 2f\n"
"1: ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n bl .Lrd\n"
"2: orr r9, r9, #0x08\n b.w .Lnext\n"
".Lo_B8:\n"
"bl .Lint\n"
"ldrb r0, [r10, #" S(CPU_INTWANTED) "]\n"
"cmp r0, #0\n bne 1f\n bl .Lidl\n b 2f\n"
"1: ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n bl .Lrd\n"
"2: bic r9, r9, #0x40\n b.w .Lnext\n"
".Lo_EA:\n"
"bl .Lint\n"
"ldrb r0, [r10, #" S(CPU_INTWANTED) "]\n"
"cmp r0, #0\n bne 1f\n bl .Lidl\n b.w .Lnext\n"
"1: ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n bl .Lrd\n"
"b.w .Lnext\n"
".Lo_AA:\n"
"bl .Lint\n"
"ldrb r0, [r10, #" S(CPU_INTWANTED) "]\n"
"cmp r0, #0\n bne 1f\n bl .Lidl\n b 2f\n"
"1: ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n bl .Lrd\n"
"2: tst r9, #0x10\n beq.w .Ltax16\n"
"uxtb r5, r4\n mov r0, r5\n bl .Lzn8\n b.w .Lnext\n"
".Ltax16:\n"
"uxth r5, r4\n mov r0, r5\n bl .Lzn16\n b.w .Lnext\n"
".Lo_A8:\n"
"bl .Lint\n"
"ldrb r0, [r10, #" S(CPU_INTWANTED) "]\n"
"cmp r0, #0\n bne 1f\n bl .Lidl\n b 2f\n"
"1: ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n bl .Lrd\n"
"2: tst r9, #0x10\n beq.w .Ltay16\n"
"uxtb r6, r4\n mov r0, r6\n bl .Lzn8\n b.w .Lnext\n"
".Ltay16:\n"
"uxth r6, r4\n mov r0, r6\n bl .Lzn16\n b.w .Lnext\n"
".Lo_8A:\n"
"bl .Lint\n"
"ldrb r0, [r10, #" S(CPU_INTWANTED) "]\n"
"cmp r0, #0\n bne 1f\n bl .Lidl\n b 2f\n"
"1: ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n bl .Lrd\n"
"2: tst r9, #0x20\n beq.w .Ltxa16\n"
"uxtb r0, r5\n bfi r4, r0, #0, #8\n bl .Lzn8\n b.w .Lnext\n"
".Ltxa16:\n"
"uxth r4, r5\n mov r0, r4\n bl .Lzn16\n b.w .Lnext\n"
".Lo_98:\n"
"bl .Lint\n"
"ldrb r0, [r10, #" S(CPU_INTWANTED) "]\n"
"cmp r0, #0\n bne 1f\n bl .Lidl\n b 2f\n"
"1: ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n bl .Lrd\n"
"2: tst r9, #0x20\n beq.w .Ltya16\n"
"uxtb r0, r6\n bfi r4, r0, #0, #8\n bl .Lzn8\n b.w .Lnext\n"
".Ltya16:\n"
"uxth r4, r6\n mov r0, r4\n bl .Lzn16\n b.w .Lnext\n"
".Lo_9A:\n"
"bl .Lint\n"
"ldrb r0, [r10, #" S(CPU_INTWANTED) "]\n"
"cmp r0, #0\n bne 1f\n bl .Lidl\n b 2f\n"
"1: ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n bl .Lrd\n"
"2: mov r7, r5\n uxth r7, r7\n"
"ldr r0, [sp, #" S(FE) "]\n cmp r0, #0\n beq 3f\n"
"and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
"3: b.w .Lnext\n"
".Lo_BA:\n"
"bl .Lint\n"
"ldrb r0, [r10, #" S(CPU_INTWANTED) "]\n"
"cmp r0, #0\n bne 1f\n bl .Lidl\n b 2f\n"
"1: ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n bl .Lrd\n"
"2: tst r9, #0x10\n beq.w .Ltsx16\n"
"uxtb r5, r7\n mov r0, r5\n bl .Lzn8\n b.w .Lnext\n"
".Ltsx16:\n"
"uxth r5, r7\n mov r0, r5\n bl .Lzn16\n b.w .Lnext\n"
".Lo_9B:\n"
"bl .Lint\n"
"ldrb r0, [r10, #" S(CPU_INTWANTED) "]\n"
"cmp r0, #0\n bne 1f\n bl .Lidl\n b 2f\n"
"1: ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n bl .Lrd\n"
"2: tst r9, #0x10\n beq.w .Ltxy16\n"
"uxtb r6, r5\n mov r0, r6\n bl .Lzn8\n b.w .Lnext\n"
".Ltxy16:\n"
"uxth r6, r5\n mov r0, r6\n bl .Lzn16\n b.w .Lnext\n"
".Lo_BB:\n"
"bl .Lint\n"
"ldrb r0, [r10, #" S(CPU_INTWANTED) "]\n"
"cmp r0, #0\n bne 1f\n bl .Lidl\n b 2f\n"
"1: ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n bl .Lrd\n"
"2: tst r9, #0x10\n beq.w .Ltyx16\n"
"uxtb r5, r6\n mov r0, r5\n bl .Lzn8\n b.w .Lnext\n"
".Ltyx16:\n"
"uxth r5, r6\n mov r0, r5\n bl .Lzn16\n b.w .Lnext\n"
".Lo_1A:\n"
"bl .Lint\n"
"ldrb r0, [r10, #" S(CPU_INTWANTED) "]\n"
"cmp r0, #0\n bne 1f\n bl .Lidl\n b 2f\n"
"1: ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n bl .Lrd\n"
"2: tst r9, #0x20\n beq.w .L1A_16\n"
"uxtb r0, r4\n adds r0, #1\n uxtb r0, r0\n bfi r4, r0, #0, #8\n"
"bl .Lzn8\n b.w .Lnext\n"
".L1A_16:\n"
"uxth r4, r4\n adds r4, #1\n uxth r4, r4\n"
"mov r0, r4\n bl .Lzn16\n b.w .Lnext\n"
".Lo_3A:\n"
"bl .Lint\n"
"ldrb r0, [r10, #" S(CPU_INTWANTED) "]\n"
"cmp r0, #0\n bne 1f\n bl .Lidl\n b 2f\n"
"1: ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n bl .Lrd\n"
"2: tst r9, #0x20\n beq.w .L3A_16\n"
"uxtb r0, r4\n subs r0, #1\n uxtb r0, r0\n bfi r4, r0, #0, #8\n"
"bl .Lzn8\n b.w .Lnext\n"
".L3A_16:\n"
"uxth r4, r4\n subs r4, #1\n uxth r4, r4\n"
"mov r0, r4\n bl .Lzn16\n b.w .Lnext\n"
".Lo_E8:\n"
"bl .Lint\n"
"ldrb r0, [r10, #" S(CPU_INTWANTED) "]\n"
"cmp r0, #0\n bne 1f\n bl .Lidl\n b 2f\n"
"1: ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n bl .Lrd\n"
"2: tst r9, #0x10\n beq.w .LE8_16\n"
"uxtb r0, r5\n adds r0, #1\n uxtb r5, r0\n"
"bl .Lzn8\n b.w .Lnext\n"
".LE8_16:\n"
"uxth r5, r5\n adds r5, #1\n uxth r5, r5\n"
"mov r0, r5\n bl .Lzn16\n b.w .Lnext\n"
".Lo_CA:\n"
"bl .Lint\n"
"ldrb r0, [r10, #" S(CPU_INTWANTED) "]\n"
"cmp r0, #0\n bne 1f\n bl .Lidl\n b 2f\n"
"1: ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n bl .Lrd\n"
"2: tst r9, #0x10\n beq.w .LCA_16\n"
"uxtb r0, r5\n subs r0, #1\n uxtb r5, r0\n"
"bl .Lzn8\n b.w .Lnext\n"
".LCA_16:\n"
"uxth r5, r5\n subs r5, #1\n uxth r5, r5\n"
"mov r0, r5\n bl .Lzn16\n b.w .Lnext\n"
".Lo_C8:\n"
"bl .Lint\n"
"ldrb r0, [r10, #" S(CPU_INTWANTED) "]\n"
"cmp r0, #0\n bne 1f\n bl .Lidl\n b 2f\n"
"1: ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n bl .Lrd\n"
"2: tst r9, #0x10\n beq.w .LC8_16\n"
"uxtb r0, r6\n adds r0, #1\n uxtb r6, r0\n"
"bl .Lzn8\n b.w .Lnext\n"
".LC8_16:\n"
"uxth r6, r6\n adds r6, #1\n uxth r6, r6\n"
"mov r0, r6\n bl .Lzn16\n b.w .Lnext\n"
".Lo_88:\n"
"bl .Lint\n"
"ldrb r0, [r10, #" S(CPU_INTWANTED) "]\n"
"cmp r0, #0\n bne 1f\n bl .Lidl\n b 2f\n"
"1: ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n bl .Lrd\n"
"2: tst r9, #0x10\n beq.w .L88_16\n"
"uxtb r0, r6\n subs r0, #1\n uxtb r6, r0\n"
"bl .Lzn8\n b.w .Lnext\n"
".L88_16:\n"
"uxth r6, r6\n subs r6, #1\n uxth r6, r6\n"
"mov r0, r6\n bl .Lzn16\n b.w .Lnext\n"

    ".ltorg\n"
    /* ======== Group B opcodes ======== */
    ".Lo_E9:\n"
    "tst r9, #0x08\n bne.w .Lfb\n"          /* decimal -> fallback */
    "tst r9, #0x20\n beq.w .Le9_16\n"
    "ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n"
    "add r8, r8, #1\n uxth r8, r8\n"
    "bl .Lint\n bl .Lrd\n"                  /* r0 = operand */
    "b .Lsbc8\n"
    ".Le9_16:\n"
    "ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n"
    "add r8, r8, #1\n uxth r8, r8\n bl .Lrd\n str r0, [sp, #" S(S0) "]\n"
    "ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n"
    "add r8, r8, #1\n uxth r8, r8\n bl .Lint\n bl .Lrd\n"
    "lsl r0, r0, #8\n ldr r1, [sp, #" S(S0) "]\n orr r0, r1, r0\n"
    "b .Lsbc16\n"
    ".Lo_E5:\n"
    "tst r9, #0x08\n bne.w .Lfb\n"
    "bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
    "ldr r0, [sp, #" S(FDP) "]\n tst r0, #0xFF\n beq 1f\n bl .Lidl\n"
    "1: ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
    "add r1, r1, r0\n uxth r1, r1\n"
    "tst r9, #0x20\n beq.w .Le5_16\n"
    "bl .Lint\n bl .Lrd\n b.w .Lsbc8\n"
    ".Le5_16:\n"
    "bl .Lrd\n str r0, [sp, #" S(S1) "]\n"
    "ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
    "add r1, r1, r0\n adds r1, #1\n uxth r1, r1\n"
    "bl .Lint\n bl .Lrd\n"
    "lsl r0, r0, #8\n ldr r1, [sp, #" S(S1) "]\n orr r0, r1, r0\n"
    "b .Lsbc16\n"
    ".Lo_ED:\n"
    "tst r9, #0x08\n bne.w .Lfb\n"
    "bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
    "bl .Lfetch\n ldr r1, [sp, #" S(S0) "]\n orr r1, r1, r0, lsl #8\n"
    "ldr r0, [sp, #" S(FDB) "]\n add r1, r1, r0, lsl #16\n"
    "tst r9, #0x20\n beq.w .Led_16\n"
    "bl .Lint\n bl .Lrd\n b.w .Lsbc8\n"
    ".Led_16:\n"
    "str r1, [sp, #" S(S0) "]\n bl .Lrd\n str r0, [sp, #" S(S1) "]\n"
    "ldr r1, [sp, #" S(S0) "]\n adds r1, #1\n bic r1, r1, #0xFF000000\n"
    "bl .Lint\n bl .Lrd\n"
    "lsl r0, r0, #8\n ldr r1, [sp, #" S(S1) "]\n orr r0, r1, r0\n"
    "b .Lsbc16\n"
    ".Lo_F5:\n"
    "tst r9, #0x08\n bne.w .Lfb\n"
    "bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
    "ldr r0, [sp, #" S(FDP) "]\n tst r0, #0xFF\n beq 1f\n bl .Lidl\n"
    "1: bl .Lidl\n"                          /* indexed penalty */
    "ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
    "add r1, r1, r0\n add r1, r1, r5\n uxth r1, r1\n"
    "tst r9, #0x20\n beq.w .Lf5_16\n"
    "bl .Lint\n bl .Lrd\n b.w .Lsbc8\n"
    ".Lf5_16:\n"
    "bl .Lrd\n str r0, [sp, #" S(S1) "]\n"
    "ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
    "add r1, r1, r0\n add r1, r1, r5\n adds r1, #1\n uxth r1, r1\n"
    "bl .Lint\n bl .Lrd\n"
    "lsl r0, r0, #8\n ldr r1, [sp, #" S(S1) "]\n orr r0, r1, r0\n"
    "b .Lsbc16\n"
    ".Lo_F9:\n mov r12, r6\n b.w .Lsbcabxy\n"
    ".Lo_FD:\n mov r12, r5\n"
    ".Lsbcabxy:\n"
    "tst r9, #0x08\n bne.w .Lfb\n"
    "str r12, [sp, #" S(S2) "]\n"
    "bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
    "bl .Lfetch\n ldr r1, [sp, #" S(S0) "]\n orr r1, r1, r0, lsl #8\n"
    "str r1, [sp, #" S(S0) "]\n"
    "tst r9, #0x10\n beq 1f\n"              /* XF=0 -> idle */
    "ldr r0, [sp, #" S(S2) "]\n"
    "add r2, r1, r0\n lsr r2, r2, #8\n lsr r1, r1, #8\n cmp r1, r2\n beq 2f\n"
    "1: bl .Lidl\n"
    "2: ldr r0, [sp, #" S(FDB) "]\n lsl r0, r0, #16\n"
    "ldr r1, [sp, #" S(S0) "]\n add r0, r0, r1\n"
    "ldr r1, [sp, #" S(S2) "]\n add r0, r0, r1\n"
    "bic r1, r0, #0xFF000000\n"             /* r1 = ea */
    "tst r9, #0x20\n beq.w .Lsbcabxy16\n"
    "bl .Lint\n bl .Lrd\n b.w .Lsbc8\n"
    ".Lsbcabxy16:\n"
    "str r1, [sp, #" S(S0) "]\n bl .Lrd\n str r0, [sp, #" S(S1) "]\n"
    "ldr r1, [sp, #" S(S0) "]\n adds r1, #1\n bic r1, r1, #0xFF000000\n"
    "bl .Lint\n bl .Lrd\n"
    "lsl r0, r0, #8\n ldr r1, [sp, #" S(S1) "]\n orr r0, r1, r0\n"
    "b .Lsbc16\n"
    ".Lsbc8:\n"
    "and  r1, r4, #0xFF\n"                  /* r1 = A low byte */
    "str  r0, [sp, #" S(S3) "]\n"           /* S3 = original operand (for V) */
    "mvn  r2, r0\n uxtb r2, r2\n"          /* r2 = ~operand & 0xFF */
    "and  r3, r9, #1\n"                     /* r3 = C flag */
    "add  r0, r1, r2\n add r0, r0, r3\n"   /* r0 = A + ~op + C (result) */
    "bic  r9, r9, #0xC3\n"                 /* clear C,Z,V,N */
    "cmp  r0, #0x100\n blo 1f\n orr r9, r9, #0x01\n"
    "1: ldr r2, [sp, #" S(S3) "]\n"        /* r2 = original operand */
    "eor  r3, r1, r0\n"                    /* A ^ result */
    "eor  r12, r1, r2\n"                   /* A ^ operand */
    "and  r3, r3, r12\n"
    "tst  r3, #0x80\n beq 1f\n orr r9, r9, #0x40\n"
    "1: tst r0, #0xFF\n  bne 1f\n orr r9, r9, #0x02\n"
    "1: tst r0, #0x80\n  beq 1f\n orr r9, r9, #0x80\n"
    "1: bfi r4, r0, #0, #8\n b.w .Lnext\n"
    ".Lsbc16:\n"
    "uxth r1, r4\n"                         /* r1 = A low 16 bits */
    "str  r0, [sp, #" S(S3) "]\n"           /* S3 = original operand */
    "movw r2, #0xFFFF\n eor r2, r0, r2\n"  /* r2 = ~operand & 0xFFFF */
    "and  r3, r9, #1\n"                     /* r3 = C */
    "add  r0, r1, r2\n add r0, r0, r3\n"   /* r0 = result */
    "bic  r9, r9, #0xC3\n"
    "movw r2, #0\n movt r2, #1\n cmp r0, r2\n blo 1f\n orr r9, r9, #0x01\n"
    "1: ldr r2, [sp, #" S(S3) "]\n"
    "eor  r3, r1, r0\n"
    "eor  r12, r1, r2\n"
    "and  r3, r3, r12\n"
    "tst  r3, #0x8000\n beq 1f\n orr r9, r9, #0x40\n"
    "1: uxth r2, r0\n cmp r2, #0\n bne 1f\n orr r9, r9, #0x02\n"
    "1: tst r0, #0x8000\n beq 1f\n orr r9, r9, #0x80\n"
    "1: uxth r4, r0\n b.w .Lnext\n"
    ".Lo_0A:\n"
    "bl .Lint\n bl .Lidl\n"
    "tst r9, #0x20\n beq.w .L0a_16\n"
    "uxtb r0, r4\n"
    "lsr  r1, r0, #7\n bfi r9, r1, #0, #1\n"   /* old bit7 -> C */
    "lsl  r0, r0, #1\n uxtb r0, r0\n"
    "bfi  r4, r0, #0, #8\n bl .Lzn8\n b.w .Lnext\n"
    ".L0a_16:\n"
    "uxth r0, r4\n"
    "lsr  r1, r0, #15\n bfi r9, r1, #0, #1\n"
    "lsl  r0, r0, #1\n uxth r4, r0\n"
    "mov  r0, r4\n bl .Lzn16\n b.w .Lnext\n"
    ".Lo_4A:\n"
    "bl .Lint\n bl .Lidl\n"
    "tst r9, #0x20\n beq.w .L4a_16\n"
    "uxtb r0, r4\n"
    "bfi  r9, r0, #0, #1\n"                /* bit0 -> C */
    "lsr  r0, r0, #1\n"
    "bfi  r4, r0, #0, #8\n bl .Lzn8\n b.w .Lnext\n"
    ".L4a_16:\n"
    "uxth r0, r4\n"
    "bfi  r9, r0, #0, #1\n"
    "lsr  r0, r0, #1\n uxth r4, r0\n"
    "mov  r0, r4\n bl .Lzn16\n b.w .Lnext\n"
    ".Lo_2A:\n"
    "bl .Lint\n bl .Lidl\n"
    "tst r9, #0x20\n beq.w .L2a_16\n"
    "uxtb r0, r4\n"
    "and  r1, r9, #1\n"                    /* old C */
    "lsr  r2, r0, #7\n"                    /* new C = old bit7 */
    "lsl  r0, r0, #1\n uxtb r0, r0\n"
    "orr  r0, r0, r1\n"                    /* insert old C at bit0 */
    "bfi  r9, r2, #0, #1\n"               /* update C */
    "bfi  r4, r0, #0, #8\n bl .Lzn8\n b.w .Lnext\n"
    ".L2a_16:\n"
    "uxth r0, r4\n"
    "and  r1, r9, #1\n"
    "lsr  r2, r0, #15\n"
    "lsl  r0, r0, #1\n uxth r0, r0\n"
    "orr  r0, r0, r1\n"
    "bfi  r9, r2, #0, #1\n"
    "uxth r4, r0\n mov r0, r4\n bl .Lzn16\n b.w .Lnext\n"
    ".Lo_6A:\n"
    "bl .Lint\n bl .Lidl\n"
    "tst r9, #0x20\n beq.w .L6a_16\n"
    "uxtb r0, r4\n"
    "and  r1, r9, #1\n"                    /* old C */
    "and  r2, r0, #1\n"                    /* new C = old bit0 */
    "lsr  r0, r0, #1\n"
    "orr  r0, r0, r1, lsl #7\n"           /* insert old C at bit7 */
    "bfi  r9, r2, #0, #1\n"
    "bfi  r4, r0, #0, #8\n bl .Lzn8\n b.w .Lnext\n"
    ".L6a_16:\n"
    "uxth r0, r4\n"
    "and  r1, r9, #1\n"
    "and  r2, r0, #1\n"
    "lsr  r0, r0, #1\n"
    "orr  r0, r0, r1, lsl #15\n"
    "bfi  r9, r2, #0, #1\n"
    "uxth r4, r0\n mov r0, r4\n bl .Lzn16\n b.w .Lnext\n"
    ".Lasl_rmw8:\n"
    "lsr  r1, r0, #7\n bfi r9, r1, #0, #1\n"
    "lsl  r0, r0, #1\n uxtb r0, r0\n bx lr\n"
    ".Lasl_rmw16:\n"
    "lsr  r1, r0, #15\n bfi r9, r1, #0, #1\n"
    "lsl  r0, r0, #1\n uxth r0, r0\n bx lr\n"
    ".Llsr_rmw8:\n"
    "bfi  r9, r0, #0, #1\n"               /* bit0 -> C */
    "lsr  r0, r0, #1\n uxtb r0, r0\n bx lr\n"
    ".Llsr_rmw16:\n"
    "bfi  r9, r0, #0, #1\n"
    "lsr  r0, r0, #1\n uxth r0, r0\n bx lr\n"
    ".Lrol_rmw8:\n"
    "and  r1, r9, #1\n"
    "lsr  r2, r0, #7\n"
    "lsl  r0, r0, #1\n uxtb r0, r0\n"
    "orr  r0, r0, r1\n"
    "bfi  r9, r2, #0, #1\n bx lr\n"
    ".Lrol_rmw16:\n"
    "and  r1, r9, #1\n"
    "lsr  r2, r0, #15\n"
    "lsl  r0, r0, #1\n uxth r0, r0\n"
    "orr  r0, r0, r1\n"
    "bfi  r9, r2, #0, #1\n bx lr\n"
    ".Lror_rmw8:\n"
    "and  r1, r9, #1\n"
    "and  r2, r0, #1\n"
    "lsr  r0, r0, #1\n"
    "orr  r0, r0, r1, lsl #7\n"
    "bfi  r9, r2, #0, #1\n bx lr\n"
    ".Lror_rmw16:\n"
    "and  r1, r9, #1\n"
    "and  r2, r0, #1\n"
    "lsr  r0, r0, #1\n"
    "orr  r0, r0, r1, lsl #15\n"
    "bfi  r9, r2, #0, #1\n bx lr\n"
    ".Lrmw8_body:\n"
    "bl .Lrd\n"                            /* r0 = value */
    "bl .Lidl\n"                           /* modify cycle */
    "str lr, [sp, #" S(FLR) "]\n"
    "ldr r3, [sp, #" S(S4) "]\n blx r3\n"
    "ldr lr, [sp, #" S(FLR) "]\n"
    "ldr r1, [sp, #" S(S0) "]\n"
    "mov r2, r0\n bl .Lwr\n"
    "bl .Lzn8\n b.w .Lnext\n"
    ".Lrmw16_body:\n"
    "bl .Lrd\n str r0, [sp, #" S(S1) "]\n"
    "ldr r1, [sp, #" S(S0) "]\n adds r1, #1\n bic r1, r1, #0xFF000000\n"
    "bl .Lrd\n"
    "lsl r0, r0, #8\n ldr r1, [sp, #" S(S1) "]\n orr r0, r1, r0\n"  /* 16-bit value */
    "bl .Lidl\n"                           /* modify cycle */
    "str lr, [sp, #" S(FLR) "]\n"
    "ldr r3, [sp, #" S(S4) "]\n blx r3\n"
    "ldr lr, [sp, #" S(FLR) "]\n"
    "str r0, [sp, #" S(S1) "]\n"
    "ldr r1, [sp, #" S(S0) "]\n adds r1, #1\n bic r1, r1, #0xFF000000\n"
    "lsr r2, r0, #8\n uxtb r2, r2\n bl .Lwr\n"
    "ldr r1, [sp, #" S(S0) "]\n"
    "ldr r0, [sp, #" S(S1) "]\n uxtb r2, r0\n bl .Lwr\n"
    "ldr r0, [sp, #" S(S1) "]\n"
    "bl .Lzn16\n b.w .Lnext\n"
    ".Lo_06:\n"
    "bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
    "ldr r0, [sp, #" S(FDP) "]\n tst r0, #0xFF\n beq 1f\n bl .Lidl\n"
    "1: ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
    "add r1, r1, r0\n uxth r1, r1\n str r1, [sp, #" S(S0) "]\n"
    "tst r9, #0x20\n beq.w 2f\n"
    "ldr r3, =.Lasl_rmw8\n str r3, [sp, #" S(S4) "]\n b.w .Lrmw8_body\n"
    "2: ldr r3, =.Lasl_rmw16\n str r3, [sp, #" S(S4) "]\n b.w .Lrmw16_body\n"
    ".Lo_0E:\n"
    "bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
    "bl .Lfetch\n ldr r1, [sp, #" S(S0) "]\n orr r1, r1, r0, lsl #8\n"
    "ldr r0, [sp, #" S(FDB) "]\n add r1, r1, r0, lsl #16\n"
    "bic r1, r1, #0xFF000000\n str r1, [sp, #" S(S0) "]\n"
    "tst r9, #0x20\n beq.w 2f\n"
    "ldr r3, =.Lasl_rmw8\n str r3, [sp, #" S(S4) "]\n b.w .Lrmw8_body\n"
    "2: ldr r3, =.Lasl_rmw16\n str r3, [sp, #" S(S4) "]\n b.w .Lrmw16_body\n"
    ".Lo_16:\n"
    "bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
    "ldr r0, [sp, #" S(FDP) "]\n tst r0, #0xFF\n beq 1f\n bl .Lidl\n"
    "1: bl .Lidl\n"
    "ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
    "add r1, r1, r0\n add r1, r1, r5\n uxth r1, r1\n str r1, [sp, #" S(S0) "]\n"
    "tst r9, #0x20\n beq.w 2f\n"
    "ldr r3, =.Lasl_rmw8\n str r3, [sp, #" S(S4) "]\n b.w .Lrmw8_body\n"
    "2: ldr r3, =.Lasl_rmw16\n str r3, [sp, #" S(S4) "]\n b.w .Lrmw16_body\n"
    ".Lo_1E:\n"
    "bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
    "bl .Lfetch\n ldr r1, [sp, #" S(S0) "]\n orr r1, r1, r0, lsl #8\n"
    "str r1, [sp, #" S(S0) "]\n"
    "bl .Lidl\n"                           /* index penalty (always for write) */
    "ldr r0, [sp, #" S(FDB) "]\n lsl r0, r0, #16\n"
    "ldr r1, [sp, #" S(S0) "]\n add r0, r0, r1\n add r0, r0, r5\n"
    "bic r1, r0, #0xFF000000\n str r1, [sp, #" S(S0) "]\n"
    "tst r9, #0x20\n beq.w 2f\n"
    "ldr r3, =.Lasl_rmw8\n str r3, [sp, #" S(S4) "]\n b.w .Lrmw8_body\n"
    "2: ldr r3, =.Lasl_rmw16\n str r3, [sp, #" S(S4) "]\n b.w .Lrmw16_body\n"
    ".Lo_46:\n"
    "bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
    "ldr r0, [sp, #" S(FDP) "]\n tst r0, #0xFF\n beq 1f\n bl .Lidl\n"
    "1: ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
    "add r1, r1, r0\n uxth r1, r1\n str r1, [sp, #" S(S0) "]\n"
    "tst r9, #0x20\n beq.w 2f\n"
    "ldr r3, =.Llsr_rmw8\n str r3, [sp, #" S(S4) "]\n b.w .Lrmw8_body\n"
    "2: ldr r3, =.Llsr_rmw16\n str r3, [sp, #" S(S4) "]\n b.w .Lrmw16_body\n"
    ".Lo_4E:\n"
    "bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
    "bl .Lfetch\n ldr r1, [sp, #" S(S0) "]\n orr r1, r1, r0, lsl #8\n"
    "ldr r0, [sp, #" S(FDB) "]\n add r1, r1, r0, lsl #16\n"
    "bic r1, r1, #0xFF000000\n str r1, [sp, #" S(S0) "]\n"
    "tst r9, #0x20\n beq.w 2f\n"
    "ldr r3, =.Llsr_rmw8\n str r3, [sp, #" S(S4) "]\n b.w .Lrmw8_body\n"
    "2: ldr r3, =.Llsr_rmw16\n str r3, [sp, #" S(S4) "]\n b.w .Lrmw16_body\n"
    ".Lo_56:\n"
    "bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
    "ldr r0, [sp, #" S(FDP) "]\n tst r0, #0xFF\n beq 1f\n bl .Lidl\n"
    "1: bl .Lidl\n"
    "ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
    "add r1, r1, r0\n add r1, r1, r5\n uxth r1, r1\n str r1, [sp, #" S(S0) "]\n"
    "tst r9, #0x20\n beq.w 2f\n"
    "ldr r3, =.Llsr_rmw8\n str r3, [sp, #" S(S4) "]\n b.w .Lrmw8_body\n"
    "2: ldr r3, =.Llsr_rmw16\n str r3, [sp, #" S(S4) "]\n b.w .Lrmw16_body\n"
    ".Lo_5E:\n"
    "bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
    "bl .Lfetch\n ldr r1, [sp, #" S(S0) "]\n orr r1, r1, r0, lsl #8\n"
    "str r1, [sp, #" S(S0) "]\n"
    "bl .Lidl\n"
    "ldr r0, [sp, #" S(FDB) "]\n lsl r0, r0, #16\n"
    "ldr r1, [sp, #" S(S0) "]\n add r0, r0, r1\n add r0, r0, r5\n"
    "bic r1, r0, #0xFF000000\n str r1, [sp, #" S(S0) "]\n"
    "tst r9, #0x20\n beq.w 2f\n"
    "ldr r3, =.Llsr_rmw8\n str r3, [sp, #" S(S4) "]\n b.w .Lrmw8_body\n"
    "2: ldr r3, =.Llsr_rmw16\n str r3, [sp, #" S(S4) "]\n b.w .Lrmw16_body\n"
    ".Lo_26:\n"
    "bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
    "ldr r0, [sp, #" S(FDP) "]\n tst r0, #0xFF\n beq 1f\n bl .Lidl\n"
    "1: ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
    "add r1, r1, r0\n uxth r1, r1\n str r1, [sp, #" S(S0) "]\n"
    "tst r9, #0x20\n beq.w 2f\n"
    "ldr r3, =.Lrol_rmw8\n str r3, [sp, #" S(S4) "]\n b.w .Lrmw8_body\n"
    "2: ldr r3, =.Lrol_rmw16\n str r3, [sp, #" S(S4) "]\n b.w .Lrmw16_body\n"
    ".Lo_2E:\n"
    "bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
    "bl .Lfetch\n ldr r1, [sp, #" S(S0) "]\n orr r1, r1, r0, lsl #8\n"
    "ldr r0, [sp, #" S(FDB) "]\n add r1, r1, r0, lsl #16\n"
    "bic r1, r1, #0xFF000000\n str r1, [sp, #" S(S0) "]\n"
    "tst r9, #0x20\n beq.w 2f\n"
    "ldr r3, =.Lrol_rmw8\n str r3, [sp, #" S(S4) "]\n b.w .Lrmw8_body\n"
    "2: ldr r3, =.Lrol_rmw16\n str r3, [sp, #" S(S4) "]\n b.w .Lrmw16_body\n"
    ".Lo_36:\n"
    "bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
    "ldr r0, [sp, #" S(FDP) "]\n tst r0, #0xFF\n beq 1f\n bl .Lidl\n"
    "1: bl .Lidl\n"
    "ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
    "add r1, r1, r0\n add r1, r1, r5\n uxth r1, r1\n str r1, [sp, #" S(S0) "]\n"
    "tst r9, #0x20\n beq.w 2f\n"
    "ldr r3, =.Lrol_rmw8\n str r3, [sp, #" S(S4) "]\n b.w .Lrmw8_body\n"
    "2: ldr r3, =.Lrol_rmw16\n str r3, [sp, #" S(S4) "]\n b.w .Lrmw16_body\n"
    ".Lo_3E:\n"
    "bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
    "bl .Lfetch\n ldr r1, [sp, #" S(S0) "]\n orr r1, r1, r0, lsl #8\n"
    "str r1, [sp, #" S(S0) "]\n"
    "bl .Lidl\n"
    "ldr r0, [sp, #" S(FDB) "]\n lsl r0, r0, #16\n"
    "ldr r1, [sp, #" S(S0) "]\n add r0, r0, r1\n add r0, r0, r5\n"
    "bic r1, r0, #0xFF000000\n str r1, [sp, #" S(S0) "]\n"
    "tst r9, #0x20\n beq.w 2f\n"
    "ldr r3, =.Lrol_rmw8\n str r3, [sp, #" S(S4) "]\n b.w .Lrmw8_body\n"
    "2: ldr r3, =.Lrol_rmw16\n str r3, [sp, #" S(S4) "]\n b.w .Lrmw16_body\n"
    ".Lo_66:\n"
    "bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
    "ldr r0, [sp, #" S(FDP) "]\n tst r0, #0xFF\n beq 1f\n bl .Lidl\n"
    "1: ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
    "add r1, r1, r0\n uxth r1, r1\n str r1, [sp, #" S(S0) "]\n"
    "tst r9, #0x20\n beq.w 2f\n"
    "ldr r3, =.Lror_rmw8\n str r3, [sp, #" S(S4) "]\n b.w .Lrmw8_body\n"
    "2: ldr r3, =.Lror_rmw16\n str r3, [sp, #" S(S4) "]\n b.w .Lrmw16_body\n"
    ".Lo_6E:\n"
    "bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
    "bl .Lfetch\n ldr r1, [sp, #" S(S0) "]\n orr r1, r1, r0, lsl #8\n"
    "ldr r0, [sp, #" S(FDB) "]\n add r1, r1, r0, lsl #16\n"
    "bic r1, r1, #0xFF000000\n str r1, [sp, #" S(S0) "]\n"
    "tst r9, #0x20\n beq.w 2f\n"
    "ldr r3, =.Lror_rmw8\n str r3, [sp, #" S(S4) "]\n b.w .Lrmw8_body\n"
    "2: ldr r3, =.Lror_rmw16\n str r3, [sp, #" S(S4) "]\n b.w .Lrmw16_body\n"
    ".Lo_76:\n"
    "bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
    "ldr r0, [sp, #" S(FDP) "]\n tst r0, #0xFF\n beq 1f\n bl .Lidl\n"
    "1: bl .Lidl\n"
    "ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
    "add r1, r1, r0\n add r1, r1, r5\n uxth r1, r1\n str r1, [sp, #" S(S0) "]\n"
    "tst r9, #0x20\n beq.w 2f\n"
    "ldr r3, =.Lror_rmw8\n str r3, [sp, #" S(S4) "]\n b.w .Lrmw8_body\n"
    "2: ldr r3, =.Lror_rmw16\n str r3, [sp, #" S(S4) "]\n b.w .Lrmw16_body\n"
    ".Lo_7E:\n"
    "bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
    "bl .Lfetch\n ldr r1, [sp, #" S(S0) "]\n orr r1, r1, r0, lsl #8\n"
    "str r1, [sp, #" S(S0) "]\n"
    "bl .Lidl\n"
    "ldr r0, [sp, #" S(FDB) "]\n lsl r0, r0, #16\n"
    "ldr r1, [sp, #" S(S0) "]\n add r0, r0, r1\n add r0, r0, r5\n"
    "bic r1, r0, #0xFF000000\n str r1, [sp, #" S(S0) "]\n"
    "tst r9, #0x20\n beq.w 2f\n"
    "ldr r3, =.Lror_rmw8\n str r3, [sp, #" S(S4) "]\n b.w .Lrmw8_body\n"
    "2: ldr r3, =.Lror_rmw16\n str r3, [sp, #" S(S4) "]\n b.w .Lrmw16_body\n"
    ".Linc_rmw8:\n"
    "uxtb r0, r0\n add r0, r0, #1\n uxtb r0, r0\n bx lr\n"
    ".Linc_rmw16:\n"
    "uxth r0, r0\n add r0, r0, #1\n uxth r0, r0\n bx lr\n"
    ".Ldec_rmw8:\n"
    "uxtb r0, r0\n sub r0, r0, #1\n uxtb r0, r0\n bx lr\n"
    ".Ldec_rmw16:\n"
    "uxth r0, r0\n sub r0, r0, #1\n uxth r0, r0\n bx lr\n"
    ".Lo_E6:\n"
    "bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
    "ldr r0, [sp, #" S(FDP) "]\n tst r0, #0xFF\n beq 1f\n bl .Lidl\n"
    "1: ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
    "add r1, r1, r0\n uxth r1, r1\n str r1, [sp, #" S(S0) "]\n"
    "tst r9, #0x20\n beq.w 2f\n"
    "ldr r3, =.Linc_rmw8\n str r3, [sp, #" S(S4) "]\n b.w .Lrmw8_body\n"
    "2: ldr r3, =.Linc_rmw16\n str r3, [sp, #" S(S4) "]\n b.w .Lrmw16_body\n"
    ".Lo_EE:\n"
    "bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
    "bl .Lfetch\n ldr r1, [sp, #" S(S0) "]\n orr r1, r1, r0, lsl #8\n"
    "ldr r0, [sp, #" S(FDB) "]\n add r1, r1, r0, lsl #16\n"
    "bic r1, r1, #0xFF000000\n str r1, [sp, #" S(S0) "]\n"
    "tst r9, #0x20\n beq.w 2f\n"
    "ldr r3, =.Linc_rmw8\n str r3, [sp, #" S(S4) "]\n b.w .Lrmw8_body\n"
    "2: ldr r3, =.Linc_rmw16\n str r3, [sp, #" S(S4) "]\n b.w .Lrmw16_body\n"
    ".Lo_F6:\n"
    "bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
    "ldr r0, [sp, #" S(FDP) "]\n tst r0, #0xFF\n beq 1f\n bl .Lidl\n"
    "1: bl .Lidl\n"
    "ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
    "add r1, r1, r0\n add r1, r1, r5\n uxth r1, r1\n str r1, [sp, #" S(S0) "]\n"
    "tst r9, #0x20\n beq.w 2f\n"
    "ldr r3, =.Linc_rmw8\n str r3, [sp, #" S(S4) "]\n b.w .Lrmw8_body\n"
    "2: ldr r3, =.Linc_rmw16\n str r3, [sp, #" S(S4) "]\n b.w .Lrmw16_body\n"
    ".Lo_FE:\n"
    "bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
    "bl .Lfetch\n ldr r1, [sp, #" S(S0) "]\n orr r1, r1, r0, lsl #8\n"
    "str r1, [sp, #" S(S0) "]\n"
    "bl .Lidl\n"
    "ldr r0, [sp, #" S(FDB) "]\n lsl r0, r0, #16\n"
    "ldr r1, [sp, #" S(S0) "]\n add r0, r0, r1\n add r0, r0, r5\n"
    "bic r1, r0, #0xFF000000\n str r1, [sp, #" S(S0) "]\n"
    "tst r9, #0x20\n beq.w 2f\n"
    "ldr r3, =.Linc_rmw8\n str r3, [sp, #" S(S4) "]\n b.w .Lrmw8_body\n"
    "2: ldr r3, =.Linc_rmw16\n str r3, [sp, #" S(S4) "]\n b.w .Lrmw16_body\n"
    ".Lo_C6:\n"
    "bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
    "ldr r0, [sp, #" S(FDP) "]\n tst r0, #0xFF\n beq 1f\n bl .Lidl\n"
    "1: ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
    "add r1, r1, r0\n uxth r1, r1\n str r1, [sp, #" S(S0) "]\n"
    "tst r9, #0x20\n beq.w 2f\n"
    "ldr r3, =.Ldec_rmw8\n str r3, [sp, #" S(S4) "]\n b.w .Lrmw8_body\n"
    "2: ldr r3, =.Ldec_rmw16\n str r3, [sp, #" S(S4) "]\n b.w .Lrmw16_body\n"
    ".Lo_CE:\n"
    "bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
    "bl .Lfetch\n ldr r1, [sp, #" S(S0) "]\n orr r1, r1, r0, lsl #8\n"
    "ldr r0, [sp, #" S(FDB) "]\n add r1, r1, r0, lsl #16\n"
    "bic r1, r1, #0xFF000000\n str r1, [sp, #" S(S0) "]\n"
    "tst r9, #0x20\n beq.w 2f\n"
    "ldr r3, =.Ldec_rmw8\n str r3, [sp, #" S(S4) "]\n b.w .Lrmw8_body\n"
    "2: ldr r3, =.Ldec_rmw16\n str r3, [sp, #" S(S4) "]\n b.w .Lrmw16_body\n"
    ".Lo_D6:\n"
    "bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
    "ldr r0, [sp, #" S(FDP) "]\n tst r0, #0xFF\n beq 1f\n bl .Lidl\n"
    "1: bl .Lidl\n"
    "ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
    "add r1, r1, r0\n add r1, r1, r5\n uxth r1, r1\n str r1, [sp, #" S(S0) "]\n"
    "tst r9, #0x20\n beq.w 2f\n"
    "ldr r3, =.Ldec_rmw8\n str r3, [sp, #" S(S4) "]\n b.w .Lrmw8_body\n"
    "2: ldr r3, =.Ldec_rmw16\n str r3, [sp, #" S(S4) "]\n b.w .Lrmw16_body\n"
    ".Lo_DE:\n"
    "bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
    "bl .Lfetch\n ldr r1, [sp, #" S(S0) "]\n orr r1, r1, r0, lsl #8\n"
    "str r1, [sp, #" S(S0) "]\n"
    "bl .Lidl\n"
    "ldr r0, [sp, #" S(FDB) "]\n lsl r0, r0, #16\n"
    "ldr r1, [sp, #" S(S0) "]\n add r0, r0, r1\n add r0, r0, r5\n"
    "bic r1, r0, #0xFF000000\n str r1, [sp, #" S(S0) "]\n"
    "tst r9, #0x20\n beq.w 2f\n"
    "ldr r3, =.Ldec_rmw8\n str r3, [sp, #" S(S4) "]\n b.w .Lrmw8_body\n"
    "2: ldr r3, =.Ldec_rmw16\n str r3, [sp, #" S(S4) "]\n b.w .Lrmw16_body\n"
    ".Lcmpx8:\n"
    "uxtb r1, r1\n"                        /* index low byte */
    "eor  r0, r0, #0xFF\n and r1, r1, #0xFF\n"
    "adds r0, r1, r0\n adds r0, #1\n"
    "bic  r9, r9, #0x83\n"                 /* clear C,Z,N */
    "cmp  r0, #0x100\n blo 1f\n orr r9, r9, #0x01\n"
    "1: tst r0, #0xFF\n  bne 1f\n orr r9, r9, #0x02\n"
    "1: tst r0, #0x80\n  beq 1f\n orr r9, r9, #0x80\n"
    "1: b.w .Lnext\n"
    ".Lcmpx16:\n"
    "uxth r1, r1\n"
    "movw r2, #0xFFFF\n eor r0, r0, r2\n"
    "adds r0, r1, r0\n adds r0, #1\n"
    "bic  r9, r9, #0x83\n"
    "movw r2, #0\n movt r2, #1\n cmp r0, r2\n blo 1f\n orr r9, r9, #0x01\n"
    "1: uxth r2, r0\n cmp r2, #0\n bne 1f\n orr r9, r9, #0x02\n"
    "1: tst r0, #0x8000\n beq 1f\n orr r9, r9, #0x80\n"
    "1: b.w .Lnext\n"
    ".Lo_E0:\n"
    "tst r9, #0x10\n beq.w .Le0_16\n"
    "ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n"
    "add r8, r8, #1\n uxth r8, r8\n"
    "bl .Lint\n bl .Lrd\n mov r1, r5\n b.w .Lcmpx8\n"
    ".Le0_16:\n"
    "ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n"
    "add r8, r8, #1\n uxth r8, r8\n bl .Lrd\n str r0, [sp, #" S(S0) "]\n"
    "ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n"
    "add r8, r8, #1\n uxth r8, r8\n bl .Lint\n bl .Lrd\n"
    "lsl r0, r0, #8\n ldr r1, [sp, #" S(S0) "]\n orr r0, r1, r0\n"
    "mov r1, r5\n b.w .Lcmpx16\n"
    ".Lo_E4:\n"
    "bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
    "ldr r0, [sp, #" S(FDP) "]\n tst r0, #0xFF\n beq 1f\n bl .Lidl\n"
    "1: ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
    "add r1, r1, r0\n uxth r1, r1\n"
    "tst r9, #0x10\n beq.w .Le4_16\n"
    "bl .Lint\n bl .Lrd\n mov r1, r5\n b.w .Lcmpx8\n"
    ".Le4_16:\n"
    "bl .Lrd\n str r0, [sp, #" S(S1) "]\n"
    "ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
    "add r1, r1, r0\n adds r1, #1\n uxth r1, r1\n"
    "bl .Lint\n bl .Lrd\n"
    "lsl r0, r0, #8\n ldr r1, [sp, #" S(S1) "]\n orr r0, r1, r0\n"
    "mov r1, r5\n b.w .Lcmpx16\n"
    ".Lo_EC:\n"
    "bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
    "bl .Lfetch\n ldr r1, [sp, #" S(S0) "]\n orr r1, r1, r0, lsl #8\n"
    "ldr r0, [sp, #" S(FDB) "]\n add r1, r1, r0, lsl #16\n"
    "tst r9, #0x10\n beq.w .Lec_16\n"
    "bl .Lint\n bl .Lrd\n mov r1, r5\n b.w .Lcmpx8\n"
    ".Lec_16:\n"
    "str r1, [sp, #" S(S0) "]\n bl .Lrd\n str r0, [sp, #" S(S1) "]\n"
    "ldr r1, [sp, #" S(S0) "]\n adds r1, #1\n bic r1, r1, #0xFF000000\n"
    "bl .Lint\n bl .Lrd\n"
    "lsl r0, r0, #8\n ldr r1, [sp, #" S(S1) "]\n orr r0, r1, r0\n"
    "mov r1, r5\n b.w .Lcmpx16\n"
    ".Lo_C0:\n"
    "tst r9, #0x10\n beq.w .Lc0_16\n"
    "ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n"
    "add r8, r8, #1\n uxth r8, r8\n"
    "bl .Lint\n bl .Lrd\n mov r1, r6\n b.w .Lcmpx8\n"
    ".Lc0_16:\n"
    "ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n"
    "add r8, r8, #1\n uxth r8, r8\n bl .Lrd\n str r0, [sp, #" S(S0) "]\n"
    "ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n"
    "add r8, r8, #1\n uxth r8, r8\n bl .Lint\n bl .Lrd\n"
    "lsl r0, r0, #8\n ldr r1, [sp, #" S(S0) "]\n orr r0, r1, r0\n"
    "mov r1, r6\n b.w .Lcmpx16\n"
    ".Lo_C4:\n"
    "bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
    "ldr r0, [sp, #" S(FDP) "]\n tst r0, #0xFF\n beq 1f\n bl .Lidl\n"
    "1: ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
    "add r1, r1, r0\n uxth r1, r1\n"
    "tst r9, #0x10\n beq.w .Lc4_16\n"
    "bl .Lint\n bl .Lrd\n mov r1, r6\n b.w .Lcmpx8\n"
    ".Lc4_16:\n"
    "bl .Lrd\n str r0, [sp, #" S(S1) "]\n"
    "ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
    "add r1, r1, r0\n adds r1, #1\n uxth r1, r1\n"
    "bl .Lint\n bl .Lrd\n"
    "lsl r0, r0, #8\n ldr r1, [sp, #" S(S1) "]\n orr r0, r1, r0\n"
    "mov r1, r6\n b.w .Lcmpx16\n"
    ".Lo_CC:\n"
    "bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
    "bl .Lfetch\n ldr r1, [sp, #" S(S0) "]\n orr r1, r1, r0, lsl #8\n"
    "ldr r0, [sp, #" S(FDB) "]\n add r1, r1, r0, lsl #16\n"
    "tst r9, #0x10\n beq.w .Lcc_16\n"
    "bl .Lint\n bl .Lrd\n mov r1, r6\n b.w .Lcmpx8\n"
    ".Lcc_16:\n"
    "str r1, [sp, #" S(S0) "]\n bl .Lrd\n str r0, [sp, #" S(S1) "]\n"
    "ldr r1, [sp, #" S(S0) "]\n adds r1, #1\n bic r1, r1, #0xFF000000\n"
    "bl .Lint\n bl .Lrd\n"
    "lsl r0, r0, #8\n ldr r1, [sp, #" S(S1) "]\n orr r0, r1, r0\n"
    "mov r1, r6\n b.w .Lcmpx16\n"

    ".ltorg\n"
    /* ======== Group C opcodes ======== */
".Lo_48:\n"
"bl .Lint\n"
"ldrb r0, [r10, #" S(CPU_INTWANTED) "]\n cmp r0, #0\n bne 1f\n"
"bl .Lidl\n b 2f\n"
"1: ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n bl .Lrd\n"
"2:\n"
"tst r9, #0x20\n beq.w .L48_16\n"
"mov r1, r7\n uxtb r2, r4\n bl .Lwr\n"
"sub r7, r7, #1\n uxth r7, r7\n"
"ldr r0, [sp, #" S(FE) "]\n cmp r0, #0\n beq 1f\n and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
"1: b.w .Lnext\n"
".L48_16:\n"
"mov r1, r7\n lsr r2, r4, #8\n uxtb r2, r2\n bl .Lwr\n"
"sub r7, r7, #1\n uxth r7, r7\n"
"ldr r0, [sp, #" S(FE) "]\n cmp r0, #0\n beq 1f\n and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
"1: bl .Lint\n"
"mov r1, r7\n uxtb r2, r4\n bl .Lwr\n"
"sub r7, r7, #1\n uxth r7, r7\n"
"ldr r0, [sp, #" S(FE) "]\n cmp r0, #0\n beq 1f\n and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
"1: b.w .Lnext\n"
".Lo_DA:\n"
"bl .Lint\n"
"ldrb r0, [r10, #" S(CPU_INTWANTED) "]\n cmp r0, #0\n bne 1f\n"
"bl .Lidl\n b 2f\n"
"1: ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n bl .Lrd\n"
"2:\n"
"tst r9, #0x10\n beq.w .LDA_16\n"
"mov r1, r7\n uxtb r2, r5\n bl .Lwr\n"
"sub r7, r7, #1\n uxth r7, r7\n"
"ldr r0, [sp, #" S(FE) "]\n cmp r0, #0\n beq 1f\n and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
"1: b.w .Lnext\n"
".LDA_16:\n"
"mov r1, r7\n lsr r2, r5, #8\n uxtb r2, r2\n bl .Lwr\n"
"sub r7, r7, #1\n uxth r7, r7\n"
"ldr r0, [sp, #" S(FE) "]\n cmp r0, #0\n beq 1f\n and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
"1: bl .Lint\n"
"mov r1, r7\n uxtb r2, r5\n bl .Lwr\n"
"sub r7, r7, #1\n uxth r7, r7\n"
"ldr r0, [sp, #" S(FE) "]\n cmp r0, #0\n beq 1f\n and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
"1: b.w .Lnext\n"
".Lo_5A:\n"
"bl .Lint\n"
"ldrb r0, [r10, #" S(CPU_INTWANTED) "]\n cmp r0, #0\n bne 1f\n"
"bl .Lidl\n b 2f\n"
"1: ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n bl .Lrd\n"
"2:\n"
"tst r9, #0x10\n beq.w .L5A_16\n"
"mov r1, r7\n uxtb r2, r6\n bl .Lwr\n"
"sub r7, r7, #1\n uxth r7, r7\n"
"ldr r0, [sp, #" S(FE) "]\n cmp r0, #0\n beq 1f\n and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
"1: b.w .Lnext\n"
".L5A_16:\n"
"mov r1, r7\n lsr r2, r6, #8\n uxtb r2, r2\n bl .Lwr\n"
"sub r7, r7, #1\n uxth r7, r7\n"
"ldr r0, [sp, #" S(FE) "]\n cmp r0, #0\n beq 1f\n and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
"1: bl .Lint\n"
"mov r1, r7\n uxtb r2, r6\n bl .Lwr\n"
"sub r7, r7, #1\n uxth r7, r7\n"
"ldr r0, [sp, #" S(FE) "]\n cmp r0, #0\n beq 1f\n and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
"1: b.w .Lnext\n"
".Lo_08:\n"
"bl .Lint\n"
"ldrb r0, [r10, #" S(CPU_INTWANTED) "]\n cmp r0, #0\n bne 1f\n"
"bl .Lidl\n b 2f\n"
"1: ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n bl .Lrd\n"
"2:\n"
"uxtb r2, r9\n"
"ldr r0, [sp, #" S(FE) "]\n cmp r0, #0\n beq 1f\n orr r2, r2, #0x10\n"
"1: mov r1, r7\n bl .Lwr\n"
"sub r7, r7, #1\n uxth r7, r7\n"
"ldr r0, [sp, #" S(FE) "]\n cmp r0, #0\n beq 1f\n and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
"1: b.w .Lnext\n"
".Lo_0B:\n"
"bl .Lint\n"
"ldrb r0, [r10, #" S(CPU_INTWANTED) "]\n cmp r0, #0\n bne 1f\n"
"bl .Lidl\n b 2f\n"
"1: ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n bl .Lrd\n"
"2:\n"
"ldr r0, [sp, #" S(FDP) "]\n"          /* r0 = DP value */
"mov r1, r7\n lsr r2, r0, #8\n uxtb r2, r2\n bl .Lwr\n"   /* push hi */
"sub r7, r7, #1\n uxth r7, r7\n"
"ldr r1, [sp, #" S(FE) "]\n cmp r1, #0\n beq 1f\n and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
"1: bl .Lint\n"
"ldr r0, [sp, #" S(FDP) "]\n"
"mov r1, r7\n uxtb r2, r0\n bl .Lwr\n"                    /* push lo */
"sub r7, r7, #1\n uxth r7, r7\n"
"ldr r1, [sp, #" S(FE) "]\n cmp r1, #0\n beq 1f\n and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
"1: b.w .Lnext\n"
".Lo_4B:\n"
"bl .Lint\n"
"ldrb r0, [r10, #" S(CPU_INTWANTED) "]\n cmp r0, #0\n bne 1f\n"
"bl .Lidl\n b 2f\n"
"1: ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n bl .Lrd\n"
"2:\n"
"ldr r0, [sp, #" S(FK) "]\n"           /* r0 = K */
"mov r1, r7\n uxtb r2, r0\n bl .Lwr\n"
"sub r7, r7, #1\n uxth r7, r7\n"
"ldr r0, [sp, #" S(FE) "]\n cmp r0, #0\n beq 1f\n and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
"1: b.w .Lnext\n"
".Lo_8B:\n"
"bl .Lint\n"
"ldrb r0, [r10, #" S(CPU_INTWANTED) "]\n cmp r0, #0\n bne 1f\n"
"bl .Lidl\n b 2f\n"
"1: ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n bl .Lrd\n"
"2:\n"
"ldr r0, [sp, #" S(FDB) "]\n"          /* r0 = DB */
"mov r1, r7\n uxtb r2, r0\n bl .Lwr\n"
"sub r7, r7, #1\n uxth r7, r7\n"
"ldr r0, [sp, #" S(FE) "]\n cmp r0, #0\n beq 1f\n and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
"1: b.w .Lnext\n"
".Lo_68:\n"
"bl .Lidl\n bl .Lidl\n"
"add r7, r7, #1\n uxth r7, r7\n"
"ldr r0, [sp, #" S(FE) "]\n cmp r0, #0\n beq 1f\n and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
"1: tst r9, #0x20\n beq.w .L68_16\n"
"mov r1, r7\n bl .Lint\n bl .Lrd\n"
"bfi r4, r0, #0, #8\n bl .Lzn8\n b.w .Lnext\n"
".L68_16:\n"
"mov r1, r7\n bl .Lrd\n str r0, [sp, #" S(S0) "]\n"
"add r7, r7, #1\n uxth r7, r7\n"
"ldr r0, [sp, #" S(FE) "]\n cmp r0, #0\n beq 1f\n and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
"1: mov r1, r7\n bl .Lint\n bl .Lrd\n"
"lsl r0, r0, #8\n ldr r1, [sp, #" S(S0) "]\n orr r4, r1, r0\n"
"uxth r4, r4\n mov r0, r4\n bl .Lzn16\n b.w .Lnext\n"
".Lo_FA:\n"
"bl .Lidl\n bl .Lidl\n"
"add r7, r7, #1\n uxth r7, r7\n"
"ldr r0, [sp, #" S(FE) "]\n cmp r0, #0\n beq 1f\n and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
"1: tst r9, #0x10\n beq.w .LFA_16\n"
"mov r1, r7\n bl .Lint\n bl .Lrd\n"
"uxtb r5, r0\n mov r0, r5\n bl .Lzn8\n b.w .Lnext\n"
".LFA_16:\n"
"mov r1, r7\n bl .Lrd\n str r0, [sp, #" S(S0) "]\n"
"add r7, r7, #1\n uxth r7, r7\n"
"ldr r0, [sp, #" S(FE) "]\n cmp r0, #0\n beq 1f\n and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
"1: mov r1, r7\n bl .Lint\n bl .Lrd\n"
"lsl r0, r0, #8\n ldr r1, [sp, #" S(S0) "]\n orr r5, r1, r0\n"
"uxth r5, r5\n mov r0, r5\n bl .Lzn16\n b.w .Lnext\n"
".Lo_7A:\n"
"bl .Lidl\n bl .Lidl\n"
"add r7, r7, #1\n uxth r7, r7\n"
"ldr r0, [sp, #" S(FE) "]\n cmp r0, #0\n beq 1f\n and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
"1: tst r9, #0x10\n beq.w .L7A_16\n"
"mov r1, r7\n bl .Lint\n bl .Lrd\n"
"uxtb r6, r0\n mov r0, r6\n bl .Lzn8\n b.w .Lnext\n"
".L7A_16:\n"
"mov r1, r7\n bl .Lrd\n str r0, [sp, #" S(S0) "]\n"
"add r7, r7, #1\n uxth r7, r7\n"
"ldr r0, [sp, #" S(FE) "]\n cmp r0, #0\n beq 1f\n and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
"1: mov r1, r7\n bl .Lint\n bl .Lrd\n"
"lsl r0, r0, #8\n ldr r1, [sp, #" S(S0) "]\n orr r6, r1, r0\n"
"uxth r6, r6\n mov r0, r6\n bl .Lzn16\n b.w .Lnext\n"
".Lo_28:\n"
"bl .Lidl\n bl .Lidl\n"
"add r7, r7, #1\n uxth r7, r7\n"
"ldr r0, [sp, #" S(FE) "]\n cmp r0, #0\n beq 1f\n and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
"1: mov r1, r7\n bl .Lint\n bl .Lrd\n"
"uxtb r9, r0\n"
"ldr r0, [sp, #" S(FE) "]\n cmp r0, #0\n beq 1f\n"
"orr r9, r9, #0x30\n"
"and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
"1:\n"
"tst r9, #0x10\n beq.w 1f\n"
"uxtb r5, r5\n uxtb r6, r6\n"
"1: b.w .Lnext\n"
".Lo_2B:\n"
"bl .Lidl\n bl .Lidl\n"
"add r7, r7, #1\n uxth r7, r7\n"
"ldr r0, [sp, #" S(FE) "]\n cmp r0, #0\n beq 1f\n and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
"1: mov r1, r7\n bl .Lrd\n str r0, [sp, #" S(S0) "]\n"
"add r7, r7, #1\n uxth r7, r7\n"
"ldr r0, [sp, #" S(FE) "]\n cmp r0, #0\n beq 1f\n and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
"1: mov r1, r7\n bl .Lint\n bl .Lrd\n"
"lsl r0, r0, #8\n ldr r1, [sp, #" S(S0) "]\n orr r0, r1, r0\n"
"uxth r0, r0\n str r0, [sp, #" S(FDP) "]\n"
"bl .Lzn16\n b.w .Lnext\n"
".Lo_4C:\n"
"bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"   /* lo */
"bl .Lint\n bl .Lfetch\n"                     /* hi */
"ldr r1, [sp, #" S(S0) "]\n orr r8, r1, r0, lsl #8\n uxth r8, r8\n"
"b.w .Lnext\n"
".Lo_6C:\n"
"bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"   /* ptr_lo */
"bl .Lfetch\n"                                /* ptr_hi */
"ldr r1, [sp, #" S(S0) "]\n orr r1, r1, r0, lsl #8\n"  /* r1 = pointer address (bank 0) */
"str r1, [sp, #" S(S1) "]\n"
"bl .Lint\n bl .Lrd\n str r0, [sp, #" S(S0) "]\n"
"ldr r1, [sp, #" S(S1) "]\n adds r1, #1\n uxth r1, r1\n"
"bl .Lrd\n"
"lsl r0, r0, #8\n ldr r1, [sp, #" S(S0) "]\n orr r8, r1, r0\n uxth r8, r8\n"
"b.w .Lnext\n"
".Lo_7C:\n"
"bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"   /* ptr_lo */
"bl .Lfetch\n"                                /* ptr_hi */
"ldr r1, [sp, #" S(S0) "]\n orr r0, r1, r0, lsl #8\n"  /* r0 = operand */
"add r0, r0, r5\n uxth r0, r0\n"             /* + X (16-bit, stays in bank K) */
"ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r0, r1\n"  /* ea = K:addr */
"str r1, [sp, #" S(S0) "]\n"
"bl .Lint\n bl .Lidl\n"                       /* extra cycle for index */
"ldr r1, [sp, #" S(S0) "]\n bl .Lrd\n str r0, [sp, #" S(S1) "]\n"
"ldr r1, [sp, #" S(S0) "]\n adds r1, #1\n bic r1, r1, #0xFF000000\n"
"bl .Lrd\n"
"lsl r0, r0, #8\n ldr r1, [sp, #" S(S1) "]\n orr r8, r1, r0\n uxth r8, r8\n"
"b.w .Lnext\n"
".Lo_5C:\n"
"bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"   /* lo */
"bl .Lfetch\n str r0, [sp, #" S(S1) "]\n"   /* hi */
"bl .Lint\n bl .Lfetch\n"                     /* bank */
"str r0, [sp, #" S(FK) "]\n"                  /* K = new bank */
"ldr r1, [sp, #" S(S0) "]\n ldr r2, [sp, #" S(S1) "]\n"
"orr r8, r1, r2, lsl #8\n uxth r8, r8\n"
"b.w .Lnext\n"
".Lo_DC:\n"
"bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"   /* ptr_lo */
"bl .Lfetch\n"                                /* ptr_hi */
"ldr r1, [sp, #" S(S0) "]\n orr r1, r1, r0, lsl #8\n"  /* r1 = pointer (bank 0) */
"str r1, [sp, #" S(S1) "]\n"
"bl .Lint\n bl .Lrd\n str r0, [sp, #" S(S0) "]\n"
"ldr r1, [sp, #" S(S1) "]\n adds r1, #1\n uxth r1, r1\n"
"bl .Lrd\n str r0, [sp, #" S(S2) "]\n"
"ldr r1, [sp, #" S(S1) "]\n adds r1, #2\n uxth r1, r1\n"
"bl .Lrd\n"
"str r0, [sp, #" S(FK) "]\n"                  /* new K */
"ldr r1, [sp, #" S(S0) "]\n ldr r2, [sp, #" S(S2) "]\n"
"orr r8, r1, r2, lsl #8\n uxth r8, r8\n"
"b.w .Lnext\n"
".Lo_22:\n"
"bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"   /* target lo */
"bl .Lfetch\n str r0, [sp, #" S(S1) "]\n"   /* target hi */
"bl .Lfetch\n str r0, [sp, #" S(S2) "]\n"   /* target bank */
"sub r0, r8, #1\n uxth r0, r0\n str r0, [sp, #" S(S3) "]\n"
"ldr r0, [sp, #" S(FK) "]\n"
"mov r1, r7\n uxtb r2, r0\n bl .Lwr\n"
"sub r7, r7, #1\n uxth r7, r7\n"
"ldr r0, [sp, #" S(FE) "]\n cmp r0, #0\n beq 1f\n and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
"1: bl .Lint\n bl .Lidl\n"
"ldr r0, [sp, #" S(S3) "]\n"
"mov r1, r7\n lsr r2, r0, #8\n uxtb r2, r2\n bl .Lwr\n"
"sub r7, r7, #1\n uxth r7, r7\n"
"ldr r0, [sp, #" S(FE) "]\n cmp r0, #0\n beq 1f\n and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
"1: ldr r0, [sp, #" S(S3) "]\n"
"mov r1, r7\n uxtb r2, r0\n bl .Lwr\n"
"sub r7, r7, #1\n uxth r7, r7\n"
"ldr r0, [sp, #" S(FE) "]\n cmp r0, #0\n beq 1f\n and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
"1: ldr r0, [sp, #" S(S2) "]\n str r0, [sp, #" S(FK) "]\n"
"ldr r1, [sp, #" S(S0) "]\n ldr r2, [sp, #" S(S1) "]\n"
"orr r8, r1, r2, lsl #8\n uxth r8, r8\n"
"b.w .Lnext\n"
".Lo_6B:\n"
"bl .Lidl\n bl .Lidl\n"
"add r7, r7, #1\n uxth r7, r7\n"
"ldr r0, [sp, #" S(FE) "]\n cmp r0, #0\n beq 1f\n and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
"1: mov r1, r7\n bl .Lrd\n str r0, [sp, #" S(S0) "]\n"
"add r7, r7, #1\n uxth r7, r7\n"
"ldr r0, [sp, #" S(FE) "]\n cmp r0, #0\n beq 1f\n and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
"1: mov r1, r7\n bl .Lrd\n str r0, [sp, #" S(S1) "]\n"
"add r7, r7, #1\n uxth r7, r7\n"
"ldr r0, [sp, #" S(FE) "]\n cmp r0, #0\n beq 1f\n and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
"1: mov r1, r7\n bl .Lint\n bl .Lrd\n"
"str r0, [sp, #" S(FK) "]\n"                  /* new K */
"ldr r1, [sp, #" S(S0) "]\n ldr r2, [sp, #" S(S1) "]\n"
"orr r8, r1, r2, lsl #8\n"
"adds r8, #1\n uxth r8, r8\n"
"b.w .Lnext\n"
".Lo_40:\n"
"bl .Lidl\n bl .Lidl\n"
"add r7, r7, #1\n uxth r7, r7\n"
"ldr r0, [sp, #" S(FE) "]\n cmp r0, #0\n beq 1f\n and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
"1: mov r1, r7\n bl .Lrd\n"
"uxtb r9, r0\n"
"ldr r0, [sp, #" S(FE) "]\n cmp r0, #0\n beq 1f\n"
"orr r9, r9, #0x30\n"
"and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
"1:\n"
"tst r9, #0x10\n beq.w 1f\n uxtb r5, r5\n uxtb r6, r6\n"
"1:\n"
"add r7, r7, #1\n uxth r7, r7\n"
"ldr r0, [sp, #" S(FE) "]\n cmp r0, #0\n beq 1f\n and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
"1: mov r1, r7\n bl .Lrd\n str r0, [sp, #" S(S0) "]\n"
"add r7, r7, #1\n uxth r7, r7\n"
"ldr r0, [sp, #" S(FE) "]\n cmp r0, #0\n beq 1f\n and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
"1: mov r1, r7\n bl .Lrd\n str r0, [sp, #" S(S1) "]\n"
"ldr r0, [sp, #" S(FE) "]\n cmp r0, #0\n bne.w .Lrti_e\n"
"add r7, r7, #1\n uxth r7, r7\n"
"mov r1, r7\n bl .Lint\n bl .Lrd\n"
"str r0, [sp, #" S(FK) "]\n"
"ldr r1, [sp, #" S(S0) "]\n ldr r2, [sp, #" S(S1) "]\n"
"orr r8, r1, r2, lsl #8\n uxth r8, r8\n"
"b.w .Lnext\n"
".Lrti_e:\n"
"ldr r1, [sp, #" S(S0) "]\n ldr r2, [sp, #" S(S1) "]\n"
"orr r8, r1, r2, lsl #8\n uxth r8, r8\n"
"b.w .Lnext\n"
".Lo_EB:\n"
"bl .Lidl\n"
"uxtb r0, r4\n"                /* r0 = old low byte (becomes new high) */
"lsr r1, r4, #8\n uxtb r1, r1\n"  /* r1 = old high byte (becomes new low) */
"orr r4, r1, r0, lsl #8\n"    /* r4 = (old_low<<8) | old_high */
"bl .Lint\n"
"ldrb r0, [r10, #" S(CPU_INTWANTED) "]\n cmp r0, #0\n bne 1f\n"
"bl .Lidl\n b 2f\n"
"1: ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n bl .Lrd\n"
"2:\n"
"uxtb r0, r4\n bl .Lzn8\n"
"b.w .Lnext\n"
".Lo_FB:\n"
"and r0, r9, #1\n"             /* r0 = old C */
"ldr r1, [sp, #" S(FE) "]\n"  /* r1 = old E */
"bfi r9, r1, #0, #1\n"        /* new C = old E */
"str r0, [sp, #" S(FE) "]\n"  /* new E = old C */
"ldr r0, [sp, #" S(FE) "]\n cmp r0, #0\n beq 1f\n"
"orr r9, r9, #0x30\n"
"and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
"1:\n"
"tst r9, #0x10\n beq.w 1f\n uxtb r5, r5\n uxtb r6, r6\n"
"1:\n"
"bl .Lint\n"
"ldrb r0, [r10, #" S(CPU_INTWANTED) "]\n cmp r0, #0\n bne 1f\n"
"bl .Lidl\n b.w .Lnext\n"
"1: ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n bl .Lrd\n"
"b.w .Lnext\n"
".Lo_42:\n"
"bl .Lint\n bl .Lfetch\n"
"b.w .Lnext\n"
".Lo_CB:\n b.w .Lfb\n"
".Lo_DB:\n b.w .Lfb\n"
".Lo_00:\n b.w .Lfb\n"
".Lo_02:\n b.w .Lfb\n"
".Lo_24:\n"
"bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
"ldr r0, [sp, #" S(FDP) "]\n tst r0, #0xFF\n beq 1f\n bl .Lidl\n"
"1: ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
"add r1, r1, r0\n uxth r1, r1\n"
"tst r9, #0x20\n beq.w .L24_16\n"
"bl .Lint\n bl .Lrd\n b.w .Lbit8\n"
".L24_16:\n"
"bl .Lrd\n str r0, [sp, #" S(S1) "]\n"
"ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
"add r1, r1, r0\n adds r1, #1\n uxth r1, r1\n"
"bl .Lint\n bl .Lrd\n"
"lsl r0, r0, #8\n ldr r1, [sp, #" S(S1) "]\n orr r0, r1, r0\n"
"b .Lbit16\n"
".Lo_2C:\n"
"bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
"bl .Lfetch\n ldr r1, [sp, #" S(S0) "]\n orr r1, r1, r0, lsl #8\n"
"ldr r0, [sp, #" S(FDB) "]\n add r1, r1, r0, lsl #16\n"
"tst r9, #0x20\n beq.w .L2C_16\n"
"bl .Lint\n bl .Lrd\n b.w .Lbit8\n"
".L2C_16:\n"
"str r1, [sp, #" S(S0) "]\n bl .Lrd\n str r0, [sp, #" S(S1) "]\n"
"ldr r1, [sp, #" S(S0) "]\n adds r1, #1\n bic r1, r1, #0xFF000000\n"
"bl .Lint\n bl .Lrd\n"
"lsl r0, r0, #8\n ldr r1, [sp, #" S(S1) "]\n orr r0, r1, r0\n"
"b .Lbit16\n"
".Lo_34:\n"
"bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
"ldr r0, [sp, #" S(FDP) "]\n tst r0, #0xFF\n beq 1f\n bl .Lidl\n"
"1: bl .Lidl\n"  /* indexed penalty */
"ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
"add r1, r1, r0\n add r1, r1, r5\n uxth r1, r1\n"
"tst r9, #0x20\n beq.w .L34_16\n"
"bl .Lint\n bl .Lrd\n b.w .Lbit8\n"
".L34_16:\n"
"bl .Lrd\n str r0, [sp, #" S(S1) "]\n"
"ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
"add r1, r1, r0\n add r1, r1, r5\n adds r1, #1\n uxth r1, r1\n"
"bl .Lint\n bl .Lrd\n"
"lsl r0, r0, #8\n ldr r1, [sp, #" S(S1) "]\n orr r0, r1, r0\n"
"b .Lbit16\n"
".Lo_3C:\n"
"str r5, [sp, #" S(S2) "]\n"   /* S2 = X */
"bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
"bl .Lfetch\n ldr r1, [sp, #" S(S0) "]\n orr r1, r1, r0, lsl #8\n"
"str r1, [sp, #" S(S0) "]\n"   /* S0 = base */
"tst r9, #0x10\n beq 1f\n"
"ldr r0, [sp, #" S(S2) "]\n"
"add r2, r1, r0\n lsr r2, r2, #8\n lsr r1, r1, #8\n cmp r1, r2\n beq.w 2f\n"
"1: bl .Lidl\n"
"2: ldr r0, [sp, #" S(FDB) "]\n lsl r0, r0, #16\n"
"ldr r1, [sp, #" S(S0) "]\n add r0, r0, r1\n"
"ldr r1, [sp, #" S(S2) "]\n add r0, r0, r1\n"
"bic r1, r0, #0xFF000000\n"    /* r1 = ea */
"tst r9, #0x20\n beq.w .L3C_16\n"
"bl .Lint\n bl .Lrd\n b.w .Lbit8\n"
".L3C_16:\n"
"str r1, [sp, #" S(S0) "]\n bl .Lrd\n str r0, [sp, #" S(S1) "]\n"
"ldr r1, [sp, #" S(S0) "]\n adds r1, #1\n bic r1, r1, #0xFF000000\n"
"bl .Lint\n bl .Lrd\n"
"lsl r0, r0, #8\n ldr r1, [sp, #" S(S1) "]\n orr r0, r1, r0\n"
"b .Lbit16\n"
".Lo_89:\n"
"tst r9, #0x20\n beq.w .L89_16\n"
"ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n"
"add r8, r8, #1\n uxth r8, r8\n"
"bl .Lint\n bl .Lrd\n"
"and r0, r4, r0\n"             /* A & imm */
"bic r9, r9, #0x02\n"          /* clear Z only */
"tst r0, #0xFF\n bne.w 1f\n orr r9, r9, #0x02\n"
"1: b.w .Lnext\n"
".L89_16:\n"
"ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n"
"add r8, r8, #1\n uxth r8, r8\n bl .Lrd\n str r0, [sp, #" S(S0) "]\n"
"ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n"
"add r8, r8, #1\n uxth r8, r8\n bl .Lint\n bl .Lrd\n"
"lsl r0, r0, #8\n ldr r1, [sp, #" S(S0) "]\n orr r0, r1, r0\n"
"uxth r1, r4\n and r0, r1, r0\n"  /* A16 & imm16 */
"bic r9, r9, #0x02\n"
"uxth r0, r0\n cmp r0, #0\n bne 1f\n orr r9, r9, #0x02\n"
"1: b.w .Lnext\n"
".Lbit8:\n"
"and r1, r4, r0\n"              /* A & operand (8-bit) */
"bic r9, r9, #0xC2\n"          /* clear N(7), V(6), Z(1) */
"tst r1, #0xFF\n bne.w 1f\n orr r9, r9, #0x02\n"   /* Z */
"1: tst r0, #0x80\n beq.w 1f\n orr r9, r9, #0x80\n" /* N from operand */
"1: tst r0, #0x40\n beq.w 1f\n orr r9, r9, #0x40\n" /* V from operand */
"1: b.w .Lnext\n"
".Lbit16:\n"
"uxth r1, r4\n and r1, r1, r0\n"  /* A16 & operand16 */
"bic r9, r9, #0xC2\n"
"uxth r1, r1\n cmp r1, #0\n bne 1f\n orr r9, r9, #0x02\n"     /* Z */
"1: tst r0, #0x8000\n beq.w 1f\n orr r9, r9, #0x80\n" /* N from bit15 */
"1: tst r0, #0x4000\n beq.w 1f\n orr r9, r9, #0x40\n" /* V from bit14 */
"1: b.w .Lnext\n"
".Lo_A1:\n"
"bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"       /* dp byte */
"ldr r0, [sp, #" S(FDP) "]\n tst r0, #0xFF\n beq 1f\n bl .Lidl\n"
"1: bl .Lidl\n"                                   /* index penalty */
"ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
"add r1, r1, r0\n add r1, r1, r5\n uxth r1, r1\n"  /* ptr_addr = DP+dp+X */
"bl .Lrd\n str r0, [sp, #" S(S1) "]\n"           /* ptr_lo */
"ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
"add r1, r1, r0\n add r1, r1, r5\n adds r1, #1\n uxth r1, r1\n"
"bl .Lrd\n"                                       /* ptr_hi */
"lsl r0, r0, #8\n ldr r1, [sp, #" S(S1) "]\n orr r1, r1, r0\n"
"ldr r0, [sp, #" S(FDB) "]\n add r1, r1, r0, lsl #16\n"  /* ea = DB:ptr */
"bic r1, r1, #0xFF000000\n"
"tst r9, #0x20\n beq.w .La1_16\n"
"bl .Lint\n bl .Lrd\n bfi r4, r0, #0, #8\n bl .Lzn8\n b.w .Lnext\n"
".La1_16:\n"
"str r1, [sp, #" S(S2) "]\n bl .Lrd\n str r0, [sp, #" S(S1) "]\n"
"ldr r1, [sp, #" S(S2) "]\n adds r1, #1\n bic r1, r1, #0xFF000000\n"
"bl .Lint\n bl .Lrd\n"
"lsl r0, r0, #8\n ldr r1, [sp, #" S(S1) "]\n orr r4, r1, r0\n"
"uxth r4, r4\n mov r0, r4\n bl .Lzn16\n b.w .Lnext\n"
".Lo_B1:\n"
"bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"       /* dp byte */
"ldr r0, [sp, #" S(FDP) "]\n tst r0, #0xFF\n beq 1f\n bl .Lidl\n"
"1: ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
"add r1, r1, r0\n uxth r1, r1\n"
"bl .Lrd\n str r0, [sp, #" S(S1) "]\n"           /* ptr_lo */
"ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
"add r1, r1, r0\n adds r1, #1\n uxth r1, r1\n"
"bl .Lrd\n"                                       /* ptr_hi */
"lsl r0, r0, #8\n ldr r1, [sp, #" S(S1) "]\n orr r1, r1, r0\n"
"str r1, [sp, #" S(S1) "]\n"                      /* S1 = base ptr */
"tst r9, #0x20\n beq 1f\n"    /* MF=0: always idle */
"add r0, r1, r6\n lsr r0, r0, #8\n lsr r2, r1, #8\n cmp r0, r2\n beq.w 2f\n"
"1: bl .Lidl\n"
"2: ldr r0, [sp, #" S(FDB) "]\n lsl r0, r0, #16\n"
"ldr r1, [sp, #" S(S1) "]\n add r0, r0, r1\n add r0, r0, r6\n"
"bic r1, r0, #0xFF000000\n"   /* ea */
"tst r9, #0x20\n beq.w .Lb1_16\n"
"bl .Lint\n bl .Lrd\n bfi r4, r0, #0, #8\n bl .Lzn8\n b.w .Lnext\n"
".Lb1_16:\n"
"str r1, [sp, #" S(S2) "]\n bl .Lrd\n str r0, [sp, #" S(S1) "]\n"
"ldr r1, [sp, #" S(S2) "]\n adds r1, #1\n bic r1, r1, #0xFF000000\n"
"bl .Lint\n bl .Lrd\n"
"lsl r0, r0, #8\n ldr r1, [sp, #" S(S1) "]\n orr r4, r1, r0\n"
"uxth r4, r4\n mov r0, r4\n bl .Lzn16\n b.w .Lnext\n"
".Lo_B2:\n"
"bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
"ldr r0, [sp, #" S(FDP) "]\n tst r0, #0xFF\n beq 1f\n bl .Lidl\n"
"1: ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
"add r1, r1, r0\n uxth r1, r1\n"
"bl .Lrd\n str r0, [sp, #" S(S1) "]\n"
"ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
"add r1, r1, r0\n adds r1, #1\n uxth r1, r1\n"
"bl .Lrd\n"
"lsl r0, r0, #8\n ldr r1, [sp, #" S(S1) "]\n orr r1, r1, r0\n"
"ldr r0, [sp, #" S(FDB) "]\n add r1, r1, r0, lsl #16\n"
"bic r1, r1, #0xFF000000\n"    /* ea = DB:ptr, no Y */
"tst r9, #0x20\n beq.w .Lb2_16\n"
"bl .Lint\n bl .Lrd\n bfi r4, r0, #0, #8\n bl .Lzn8\n b.w .Lnext\n"
".Lb2_16:\n"
"str r1, [sp, #" S(S2) "]\n bl .Lrd\n str r0, [sp, #" S(S1) "]\n"
"ldr r1, [sp, #" S(S2) "]\n adds r1, #1\n bic r1, r1, #0xFF000000\n"
"bl .Lint\n bl .Lrd\n"
"lsl r0, r0, #8\n ldr r1, [sp, #" S(S1) "]\n orr r4, r1, r0\n"
"uxth r4, r4\n mov r0, r4\n bl .Lzn16\n b.w .Lnext\n"
".Lo_A3:\n"
"bl .Lfetch\n"                                    /* r0 = offset */
"add r1, r7, r0\n uxth r1, r1\n"                 /* ea = SP+offset, bank 0 */
"str r1, [sp, #" S(S0) "]\n"
"bl .Lidl\n"                                       /* stack-relative idle */
"tst r9, #0x20\n beq.w .La3_16\n"
"ldr r1, [sp, #" S(S0) "]\n bl .Lint\n bl .Lrd\n"
"bfi r4, r0, #0, #8\n bl .Lzn8\n b.w .Lnext\n"
".La3_16:\n"
"ldr r1, [sp, #" S(S0) "]\n bl .Lrd\n str r0, [sp, #" S(S1) "]\n"
"ldr r1, [sp, #" S(S0) "]\n adds r1, #1\n uxth r1, r1\n"
"bl .Lint\n bl .Lrd\n"
"lsl r0, r0, #8\n ldr r1, [sp, #" S(S1) "]\n orr r4, r1, r0\n"
"uxth r4, r4\n mov r0, r4\n bl .Lzn16\n b.w .Lnext\n"
".Lo_B3:\n"
"bl .Lfetch\n"                                    /* r0 = offset */
"add r1, r7, r0\n uxth r1, r1\n"                 /* ptr_addr = SP+offset */
"str r1, [sp, #" S(S0) "]\n"
"bl .Lidl\n"                                       /* stack-relative idle */
"ldr r1, [sp, #" S(S0) "]\n bl .Lrd\n str r0, [sp, #" S(S1) "]\n"  /* ptr_lo */
"ldr r1, [sp, #" S(S0) "]\n adds r1, #1\n uxth r1, r1\n"
"bl .Lrd\n"                                       /* ptr_hi */
"lsl r0, r0, #8\n ldr r1, [sp, #" S(S1) "]\n orr r1, r1, r0\n"
"str r1, [sp, #" S(S1) "]\n"                      /* S1 = base ptr */
"bl .Lidl\n"                                       /* Y-index idle */
"ldr r0, [sp, #" S(FDB) "]\n lsl r0, r0, #16\n"
"ldr r1, [sp, #" S(S1) "]\n add r0, r0, r1\n add r0, r0, r6\n"
"bic r1, r0, #0xFF000000\n"   /* ea = DB:ptr+Y */
"tst r9, #0x20\n beq.w .Lb3_16\n"
"bl .Lint\n bl .Lrd\n bfi r4, r0, #0, #8\n bl .Lzn8\n b.w .Lnext\n"
".Lb3_16:\n"
"str r1, [sp, #" S(S2) "]\n bl .Lrd\n str r0, [sp, #" S(S1) "]\n"
"ldr r1, [sp, #" S(S2) "]\n adds r1, #1\n bic r1, r1, #0xFF000000\n"
"bl .Lint\n bl .Lrd\n"
"lsl r0, r0, #8\n ldr r1, [sp, #" S(S1) "]\n orr r4, r1, r0\n"
"uxth r4, r4\n mov r0, r4\n bl .Lzn16\n b.w .Lnext\n"

    ".ltorg\n"
    /* ======== Group D opcodes ======== */
".Ladr_idx:\n"
"str lr, [sp, #" S(S2) "]\n"
"bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"  /* S0 = dp offset */
"ldr r0, [sp, #" S(FDP) "]\n tst r0, #0xFF\n beq.w 1f\n bl .Lidl\n"
"1: bl .Lidl\n"  /* indexed extra cycle */
"ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
"add r1, r1, r0\n add r1, r1, r5\n uxth r1, r1\n"
"str r1, [sp, #" S(S1) "]\n"  /* S1 = base addr for pointer read */
"bl .Lrd\n str r0, [sp, #" S(S0) "]\n"  /* S0 = ptr_lo */
"ldr r1, [sp, #" S(S1) "]\n adds r1, #1\n uxth r1, r1\n"
"bl .Lrd\n"  /* r0 = ptr_hi */
"ldr r1, [sp, #" S(S0) "]\n orr r1, r1, r0, lsl #8\n"  /* r1 = 16-bit pointer */
"ldr r0, [sp, #" S(FDB) "]\n add r1, r1, r0, lsl #16\n"  /* + DB<<16 */
"str r1, [sp, #" S(S3) "]\n"
"adds r1, #1\n bic r1, r1, #0xFF000000\n str r1, [sp, #" S(S4) "]\n"
"ldr lr, [sp, #" S(S2) "]\n bx lr\n"
".Ladr_idy:\n"
"str lr, [sp, #" S(S2) "]\n"
"bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"  /* S0 = dp offset */
"ldr r0, [sp, #" S(FDP) "]\n tst r0, #0xFF\n beq.w 1f\n bl .Lidl\n"
"1:\n"
"ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
"add r1, r1, r0\n uxth r1, r1\n str r1, [sp, #" S(S1) "]\n"  /* S1 = base */
"bl .Lrd\n str r0, [sp, #" S(S0) "]\n"  /* S0 = ptr_lo */
"ldr r1, [sp, #" S(S1) "]\n adds r1, #1\n uxth r1, r1\n"
"bl .Lrd\n"  /* r0 = ptr_hi */
"ldr r1, [sp, #" S(S0) "]\n orr r1, r1, r0, lsl #8\n"  /* r1 = 16-bit pointer */
"str r1, [sp, #" S(S1) "]\n"  /* S1 = pointer */
"ldr r0, [sp, #" S(S4) "]\n"  /* write flag from caller */
"cmp r0, #0\n bne.w 2f\n"     /* write -> always idle */
"tst r9, #0x10\n beq.w 2f\n"  /* XF=0 -> idle */
"ldr r1, [sp, #" S(S1) "]\n"
"add r0, r1, r6\n lsr r0, r0, #8\n lsr r3, r1, #8\n cmp r0, r3\n beq.w 3f\n"
"2: bl .Lidl\n"
"3:\n"
"ldr r0, [sp, #" S(FDB) "]\n lsl r0, r0, #16\n"
"ldr r1, [sp, #" S(S1) "]\n add r0, r0, r1\n add r0, r0, r6\n"
"bic r0, r0, #0xFF000000\n str r0, [sp, #" S(S3) "]\n"
"adds r0, #1\n bic r0, r0, #0xFF000000\n str r0, [sp, #" S(S4) "]\n"
"ldr lr, [sp, #" S(S2) "]\n bx lr\n"
".Ladr_idp:\n"
"str lr, [sp, #" S(S2) "]\n"
"bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
"ldr r0, [sp, #" S(FDP) "]\n tst r0, #0xFF\n beq.w 1f\n bl .Lidl\n"
"1:\n"
"ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
"add r1, r1, r0\n uxth r1, r1\n str r1, [sp, #" S(S1) "]\n"
"bl .Lrd\n str r0, [sp, #" S(S0) "]\n"
"ldr r1, [sp, #" S(S1) "]\n adds r1, #1\n uxth r1, r1\n"
"bl .Lrd\n"
"ldr r1, [sp, #" S(S0) "]\n orr r1, r1, r0, lsl #8\n"
"ldr r0, [sp, #" S(FDB) "]\n add r1, r1, r0, lsl #16\n"
"str r1, [sp, #" S(S3) "]\n"
"adds r1, #1\n bic r1, r1, #0xFF000000\n str r1, [sp, #" S(S4) "]\n"
"ldr lr, [sp, #" S(S2) "]\n bx lr\n"
".Ladr_idl:\n"
"str lr, [sp, #" S(S2) "]\n"
"bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"  /* S0 = dp offset */
"ldr r0, [sp, #" S(FDP) "]\n tst r0, #0xFF\n beq.w 1f\n bl .Lidl\n"
"1:\n"
"ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
"add r1, r1, r0\n uxth r1, r1\n str r1, [sp, #" S(S1) "]\n"  /* S1 = base */
"bl .Lrd\n str r0, [sp, #" S(S0) "]\n"  /* S0 = ptr byte 0 */
"ldr r1, [sp, #" S(S1) "]\n adds r1, #1\n uxth r1, r1\n"
"bl .Lrd\n"  /* r0 = ptr byte 1 */
"ldr r1, [sp, #" S(S0) "]\n orr r1, r1, r0, lsl #8\n str r1, [sp, #" S(S0) "]\n"
"ldr r1, [sp, #" S(S1) "]\n adds r1, #2\n uxth r1, r1\n"
"bl .Lrd\n"  /* r0 = ptr byte 2 (bank) */
"ldr r1, [sp, #" S(S0) "]\n orr r1, r1, r0, lsl #16\n"  /* 24-bit pointer */
"str r1, [sp, #" S(S3) "]\n"
"adds r1, #1\n bic r1, r1, #0xFF000000\n str r1, [sp, #" S(S4) "]\n"
"ldr lr, [sp, #" S(S2) "]\n bx lr\n"
".Ladr_ily:\n"
"str lr, [sp, #" S(S2) "]\n"
"bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
"ldr r0, [sp, #" S(FDP) "]\n tst r0, #0xFF\n beq.w 1f\n bl .Lidl\n"
"1:\n"
"ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
"add r1, r1, r0\n uxth r1, r1\n str r1, [sp, #" S(S1) "]\n"
"bl .Lrd\n str r0, [sp, #" S(S0) "]\n"
"ldr r1, [sp, #" S(S1) "]\n adds r1, #1\n uxth r1, r1\n"
"bl .Lrd\n"
"ldr r1, [sp, #" S(S0) "]\n orr r1, r1, r0, lsl #8\n str r1, [sp, #" S(S0) "]\n"
"ldr r1, [sp, #" S(S1) "]\n adds r1, #2\n uxth r1, r1\n"
"bl .Lrd\n"
"ldr r1, [sp, #" S(S0) "]\n orr r1, r1, r0, lsl #16\n"
"add r1, r1, r6\n bic r1, r1, #0xFF000000\n"
"str r1, [sp, #" S(S3) "]\n"
"adds r1, #1\n bic r1, r1, #0xFF000000\n str r1, [sp, #" S(S4) "]\n"
"ldr lr, [sp, #" S(S2) "]\n bx lr\n"
".Ladr_sr:\n"
"str lr, [sp, #" S(S2) "]\n"
"bl .Lfetch\n"  /* r0 = offset */
"str r0, [sp, #" S(S0) "]\n"
"bl .Lidl\n"
"ldr r0, [sp, #" S(S0) "]\n add r1, r7, r0\n uxth r1, r1\n"
"str r1, [sp, #" S(S3) "]\n"
"adds r1, #1\n uxth r1, r1\n str r1, [sp, #" S(S4) "]\n"
"ldr lr, [sp, #" S(S2) "]\n bx lr\n"
".Ladr_isy:\n"
"str lr, [sp, #" S(S2) "]\n"
"bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
"bl .Lidl\n"
"ldr r0, [sp, #" S(S0) "]\n add r1, r7, r0\n uxth r1, r1\n"
"str r1, [sp, #" S(S1) "]\n"  /* S1 = addr */
"bl .Lrd\n str r0, [sp, #" S(S0) "]\n"  /* S0 = ptr_lo */
"ldr r1, [sp, #" S(S1) "]\n adds r1, #1\n uxth r1, r1\n"
"bl .Lrd\n"  /* r0 = ptr_hi */
"ldr r1, [sp, #" S(S0) "]\n orr r1, r1, r0, lsl #8\n"  /* r1 = 16-bit pointer */
"str r1, [sp, #" S(S1) "]\n"  /* save pointer */
"bl .Lidl\n"  /* extra idle cycle */
"ldr r0, [sp, #" S(FDB) "]\n lsl r0, r0, #16\n"
"ldr r1, [sp, #" S(S1) "]\n add r0, r0, r1\n add r0, r0, r6\n"
"bic r0, r0, #0xFF000000\n str r0, [sp, #" S(S3) "]\n"
"adds r0, #1\n bic r0, r0, #0xFF000000\n str r0, [sp, #" S(S4) "]\n"
"ldr lr, [sp, #" S(S2) "]\n bx lr\n"
".Ladr_abl:\n"
"str lr, [sp, #" S(S2) "]\n"
"bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"   /* byte 0 (low) */
"bl .Lfetch\n str r0, [sp, #" S(S1) "]\n"   /* byte 1 (mid) */
"bl .Lfetch\n"                                /* r0 = byte 2 (bank) */
"ldr r1, [sp, #" S(S0) "]\n ldr r2, [sp, #" S(S1) "]\n"
"orr r1, r1, r2, lsl #8\n orr r1, r1, r0, lsl #16\n"
"str r1, [sp, #" S(S3) "]\n"
"adds r1, #1\n bic r1, r1, #0xFF000000\n str r1, [sp, #" S(S4) "]\n"
"ldr lr, [sp, #" S(S2) "]\n bx lr\n"
".Ladr_alx:\n"
"str lr, [sp, #" S(S2) "]\n"
"bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
"bl .Lfetch\n str r0, [sp, #" S(S1) "]\n"
"bl .Lfetch\n"
"ldr r1, [sp, #" S(S0) "]\n ldr r2, [sp, #" S(S1) "]\n"
"orr r1, r1, r2, lsl #8\n orr r1, r1, r0, lsl #16\n"
"add r1, r1, r5\n bic r1, r1, #0xFF000000\n"
"str r1, [sp, #" S(S3) "]\n"
"adds r1, #1\n bic r1, r1, #0xFF000000\n str r1, [sp, #" S(S4) "]\n"
"ldr lr, [sp, #" S(S2) "]\n bx lr\n"
".ltorg\n"
".Lalu_ora:\n"
"tst r9, #0x20\n beq.w .Lalu_ora16\n"
"ldr r1, [sp, #" S(S3) "]\n bl .Lint\n bl .Lrd\n"
"uxtb r0, r0\n uxtb r1, r4\n orr r0, r1, r0\n bfi r4, r0, #0, #8\n"
"bl .Lzn8\n b.w .Lnext\n"
".Lalu_ora16:\n"
"ldr r1, [sp, #" S(S3) "]\n bl .Lrd\n str r0, [sp, #" S(S0) "]\n"
"ldr r1, [sp, #" S(S4) "]\n bl .Lint\n bl .Lrd\n"
"lsl r0, r0, #8\n ldr r1, [sp, #" S(S0) "]\n orr r0, r1, r0\n"
"uxth r1, r4\n orr r4, r1, r0\n uxth r4, r4\n"
"mov r0, r4\n bl .Lzn16\n b.w .Lnext\n"
".Lalu_and:\n"
"tst r9, #0x20\n beq.w .Lalu_and16\n"
"ldr r1, [sp, #" S(S3) "]\n bl .Lint\n bl .Lrd\n"
"uxtb r0, r0\n uxtb r1, r4\n and r0, r1, r0\n bfi r4, r0, #0, #8\n"
"bl .Lzn8\n b.w .Lnext\n"
".Lalu_and16:\n"
"ldr r1, [sp, #" S(S3) "]\n bl .Lrd\n str r0, [sp, #" S(S0) "]\n"
"ldr r1, [sp, #" S(S4) "]\n bl .Lint\n bl .Lrd\n"
"lsl r0, r0, #8\n ldr r1, [sp, #" S(S0) "]\n orr r0, r1, r0\n"
"uxth r1, r4\n and r4, r1, r0\n uxth r4, r4\n"
"mov r0, r4\n bl .Lzn16\n b.w .Lnext\n"
".Lalu_eor:\n"
"tst r9, #0x20\n beq.w .Lalu_eor16\n"
"ldr r1, [sp, #" S(S3) "]\n bl .Lint\n bl .Lrd\n"
"uxtb r0, r0\n uxtb r1, r4\n eor r0, r1, r0\n bfi r4, r0, #0, #8\n"
"bl .Lzn8\n b.w .Lnext\n"
".Lalu_eor16:\n"
"ldr r1, [sp, #" S(S3) "]\n bl .Lrd\n str r0, [sp, #" S(S0) "]\n"
"ldr r1, [sp, #" S(S4) "]\n bl .Lint\n bl .Lrd\n"
"lsl r0, r0, #8\n ldr r1, [sp, #" S(S0) "]\n orr r0, r1, r0\n"
"uxth r1, r4\n eor r4, r1, r0\n uxth r4, r4\n"
"mov r0, r4\n bl .Lzn16\n b.w .Lnext\n"
".Lalu_lda:\n"
"tst r9, #0x20\n beq.w .Lalu_lda16\n"
"ldr r1, [sp, #" S(S3) "]\n bl .Lint\n bl .Lrd\n"
"bfi r4, r0, #0, #8\n bl .Lzn8\n b.w .Lnext\n"
".Lalu_lda16:\n"
"ldr r1, [sp, #" S(S3) "]\n bl .Lrd\n str r0, [sp, #" S(S0) "]\n"
"ldr r1, [sp, #" S(S4) "]\n bl .Lint\n bl .Lrd\n"
"lsl r0, r0, #8\n ldr r1, [sp, #" S(S0) "]\n orr r4, r1, r0\n"
"mov r0, r4\n bl .Lzn16\n b.w .Lnext\n"
".Lalu_sta:\n"
"tst r9, #0x20\n beq.w .Lalu_sta16\n"
"ldr r1, [sp, #" S(S3) "]\n uxtb r2, r4\n bl .Lint\n bl .Lwr\n b.w .Lnext\n"
".Lalu_sta16:\n"
"ldr r1, [sp, #" S(S3) "]\n uxtb r2, r4\n bl .Lwr\n"
"ldr r1, [sp, #" S(S4) "]\n lsr r2, r4, #8\n uxtb r2, r2\n bl .Lint\n bl .Lwr\n b.w .Lnext\n"
".Lalu_cmp:\n"
"tst r9, #0x20\n beq.w .Lalu_cmp16\n"
"ldr r1, [sp, #" S(S3) "]\n bl .Lint\n bl .Lrd\n"
"b.w .Lcmp8\n"
".Lalu_cmp16:\n"
"ldr r1, [sp, #" S(S3) "]\n bl .Lrd\n str r0, [sp, #" S(S0) "]\n"
"ldr r1, [sp, #" S(S4) "]\n bl .Lint\n bl .Lrd\n"
"lsl r0, r0, #8\n ldr r1, [sp, #" S(S0) "]\n orr r0, r1, r0\n"
"b.w .Lcmp16\n"
".Lalu_adc:\n"
"tst r9, #0x20\n beq.w .Lalu_adc16\n"
"ldr r1, [sp, #" S(S3) "]\n bl .Lint\n bl .Lrd\n"
"and  r1, r4, #0xFF\n and r2, r9, #1\n"
"add  r3, r1, r0\n add r3, r3, r2\n"
"eor  r2, r1, r0\n mvn r2, r2\n eor r12, r1, r3\n and r2, r2, r12\n"
"bic  r9, r9, #0xC3\n"
"tst  r2, #0x80\n beq 1f\n orr r9, r9, #0x40\n"
"1: cmp r3, #0x100\n blo 1f\n orr r9, r9, #0x01\n"
"1: tst r3, #0xFF\n  bne 1f\n orr r9, r9, #0x02\n"
"1: tst r3, #0x80\n  beq 1f\n orr r9, r9, #0x80\n"
"1: bfi r4, r3, #0, #8\n b.w .Lnext\n"
".Lalu_adc16:\n"
"ldr r1, [sp, #" S(S3) "]\n bl .Lrd\n str r0, [sp, #" S(S0) "]\n"
"ldr r1, [sp, #" S(S4) "]\n bl .Lint\n bl .Lrd\n"
"lsl r0, r0, #8\n ldr r1, [sp, #" S(S0) "]\n orr r0, r1, r0\n"
"uxth r1, r4\n and r2, r9, #1\n"
"add  r3, r1, r0\n add r3, r3, r2\n"
"eor  r2, r1, r0\n mvn r2, r2\n eor r12, r1, r3\n and r2, r2, r12\n"
"bic  r9, r9, #0xC3\n"
"tst  r2, #0x8000\n beq 1f\n orr r9, r9, #0x40\n"
"1: movw r2, #0\n movt r2, #1\n cmp r3, r2\n blo 1f\n orr r9, r9, #0x01\n"
"1: uxth r2, r3\n cmp r2, #0\n bne 1f\n orr r9, r9, #0x02\n"
"1: tst r3, #0x8000\n beq 1f\n orr r9, r9, #0x80\n"
"1: uxth r4, r3\n b.w .Lnext\n"
".Lalu_sbc:\n"
"tst r9, #0x20\n beq.w .Lalu_sbc16\n"
"ldr r1, [sp, #" S(S3) "]\n bl .Lint\n bl .Lrd\n"
"eor r0, r0, #0xFF\n"
"and  r1, r4, #0xFF\n and r2, r9, #1\n"
"add  r3, r1, r0\n add r3, r3, r2\n"
"eor  r2, r1, r0\n mvn r2, r2\n eor r12, r1, r3\n and r2, r2, r12\n"
"bic  r9, r9, #0xC3\n"
"tst  r2, #0x80\n beq 1f\n orr r9, r9, #0x40\n"
"1: cmp r3, #0x100\n blo 1f\n orr r9, r9, #0x01\n"
"1: tst r3, #0xFF\n  bne 1f\n orr r9, r9, #0x02\n"
"1: tst r3, #0x80\n  beq 1f\n orr r9, r9, #0x80\n"
"1: bfi r4, r3, #0, #8\n b.w .Lnext\n"
".Lalu_sbc16:\n"
"ldr r1, [sp, #" S(S3) "]\n bl .Lrd\n str r0, [sp, #" S(S0) "]\n"
"ldr r1, [sp, #" S(S4) "]\n bl .Lint\n bl .Lrd\n"
"lsl r0, r0, #8\n ldr r1, [sp, #" S(S0) "]\n orr r0, r1, r0\n"
"movw r1, #0xFFFF\n eor r0, r0, r1\n"
"uxth r1, r4\n and r2, r9, #1\n"
"add  r3, r1, r0\n add r3, r3, r2\n"
"eor  r2, r1, r0\n mvn r2, r2\n eor r12, r1, r3\n and r2, r2, r12\n"
"bic  r9, r9, #0xC3\n"
"tst  r2, #0x8000\n beq 1f\n orr r9, r9, #0x40\n"
"1: movw r2, #0\n movt r2, #1\n cmp r3, r2\n blo 1f\n orr r9, r9, #0x01\n"
"1: uxth r2, r3\n cmp r2, #0\n bne 1f\n orr r9, r9, #0x02\n"
"1: tst r3, #0x8000\n beq 1f\n orr r9, r9, #0x80\n"
"1: uxth r4, r3\n b.w .Lnext\n"
".ltorg\n"
".Lo_44:\n"
"bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"   /* S0 = dest bank */
"bl .Lfetch\n str r0, [sp, #" S(S1) "]\n"   /* S1 = src bank */
"ldr r0, [sp, #" S(S0) "]\n str r0, [sp, #" S(FDB) "]\n"
"ldr r0, [sp, #" S(S1) "]\n lsl r1, r0, #16\n orr r1, r1, r5\n"
"bl .Lrd\n str r0, [sp, #" S(S3) "]\n"  /* S3 = data byte */
"ldr r0, [sp, #" S(S0) "]\n lsl r1, r0, #16\n orr r1, r1, r6\n"
"ldr r2, [sp, #" S(S3) "]\n uxtb r2, r2\n bl .Lwr\n"
"subs r4, #1\n uxth r4, r4\n"
"subs r5, #1\n uxth r5, r5\n"
"subs r6, #1\n uxth r6, r6\n"
"movw r0, #0xFFFF\n cmp r4, r0\n beq.w 1f\n"
"subs r8, #3\n uxth r8, r8\n"
"1:\n"
"tst r9, #0x10\n beq.w 1f\n"
"uxtb r5, r5\n uxtb r6, r6\n"
"1:\n"
"bl .Lidl\n bl .Lint\n bl .Lidl\n"
"b.w .Lnext\n"
".Lo_54:\n"
"bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
"bl .Lfetch\n str r0, [sp, #" S(S1) "]\n"
"ldr r0, [sp, #" S(S0) "]\n str r0, [sp, #" S(FDB) "]\n"
"ldr r0, [sp, #" S(S1) "]\n lsl r1, r0, #16\n orr r1, r1, r5\n"
"bl .Lrd\n str r0, [sp, #" S(S3) "]\n"
"ldr r0, [sp, #" S(S0) "]\n lsl r1, r0, #16\n orr r1, r1, r6\n"
"ldr r2, [sp, #" S(S3) "]\n uxtb r2, r2\n bl .Lwr\n"
"subs r4, #1\n uxth r4, r4\n"
"adds r5, #1\n uxth r5, r5\n"
"adds r6, #1\n uxth r6, r6\n"
"movw r0, #0xFFFF\n cmp r4, r0\n beq.w 1f\n"
"subs r8, #3\n uxth r8, r8\n"
"1:\n"
"tst r9, #0x10\n beq.w 1f\n"
"uxtb r5, r5\n uxtb r6, r6\n"
"1:\n"
"bl .Lidl\n bl .Lint\n bl .Lidl\n"
"b.w .Lnext\n"
".ltorg\n"
".Lo_01:\n bl .Ladr_idx\n b.w .Lalu_ora\n"
".Lo_21:\n bl .Ladr_idx\n b.w .Lalu_and\n"
".Lo_41:\n bl .Ladr_idx\n b.w .Lalu_eor\n"
".Lo_61:\n tst r9, #0x08\n bne.w .Lfb\n bl .Ladr_idx\n b.w .Lalu_adc\n"
".Lo_81:\n bl .Ladr_idx\n b.w .Lalu_sta\n"
".Lo_C1:\n bl .Ladr_idx\n b.w .Lalu_cmp\n"
".Lo_E1:\n tst r9, #0x08\n bne.w .Lfb\n bl .Ladr_idx\n b.w .Lalu_sbc\n"
".Lo_11:\n movs r0, #0\n str r0, [sp, #" S(S4) "]\n bl .Ladr_idy\n b.w .Lalu_ora\n"
".Lo_31:\n movs r0, #0\n str r0, [sp, #" S(S4) "]\n bl .Ladr_idy\n b.w .Lalu_and\n"
".Lo_51:\n movs r0, #0\n str r0, [sp, #" S(S4) "]\n bl .Ladr_idy\n b.w .Lalu_eor\n"
".Lo_71:\n tst r9, #0x08\n bne.w .Lfb\n movs r0, #0\n str r0, [sp, #" S(S4) "]\n bl .Ladr_idy\n b.w .Lalu_adc\n"
".Lo_91:\n movs r0, #1\n str r0, [sp, #" S(S4) "]\n bl .Ladr_idy\n b.w .Lalu_sta\n"
".Lo_D1:\n movs r0, #0\n str r0, [sp, #" S(S4) "]\n bl .Ladr_idy\n b.w .Lalu_cmp\n"
".Lo_F1:\n tst r9, #0x08\n bne.w .Lfb\n movs r0, #0\n str r0, [sp, #" S(S4) "]\n bl .Ladr_idy\n b.w .Lalu_sbc\n"
".Lo_12:\n bl .Ladr_idp\n b.w .Lalu_ora\n"
".Lo_32:\n bl .Ladr_idp\n b.w .Lalu_and\n"
".Lo_52:\n bl .Ladr_idp\n b.w .Lalu_eor\n"
".Lo_72:\n tst r9, #0x08\n bne.w .Lfb\n bl .Ladr_idp\n b.w .Lalu_adc\n"
".Lo_92:\n bl .Ladr_idp\n b.w .Lalu_sta\n"
".Lo_D2:\n bl .Ladr_idp\n b.w .Lalu_cmp\n"
".Lo_F2:\n tst r9, #0x08\n bne.w .Lfb\n bl .Ladr_idp\n b.w .Lalu_sbc\n"
".Lo_07:\n bl .Ladr_idl\n b.w .Lalu_ora\n"
".Lo_27:\n bl .Ladr_idl\n b.w .Lalu_and\n"
".Lo_47:\n bl .Ladr_idl\n b.w .Lalu_eor\n"
".Lo_67:\n tst r9, #0x08\n bne.w .Lfb\n bl .Ladr_idl\n b.w .Lalu_adc\n"
".Lo_87:\n bl .Ladr_idl\n b.w .Lalu_sta\n"
".Lo_A7:\n bl .Ladr_idl\n b.w .Lalu_lda\n"
".Lo_C7:\n bl .Ladr_idl\n b.w .Lalu_cmp\n"
".Lo_E7:\n tst r9, #0x08\n bne.w .Lfb\n bl .Ladr_idl\n b.w .Lalu_sbc\n"
".Lo_17:\n bl .Ladr_ily\n b.w .Lalu_ora\n"
".Lo_37:\n bl .Ladr_ily\n b.w .Lalu_and\n"
".Lo_57:\n bl .Ladr_ily\n b.w .Lalu_eor\n"
".Lo_77:\n tst r9, #0x08\n bne.w .Lfb\n bl .Ladr_ily\n b.w .Lalu_adc\n"
".Lo_97:\n bl .Ladr_ily\n b.w .Lalu_sta\n"
".Lo_B7:\n bl .Ladr_ily\n b.w .Lalu_lda\n"
".Lo_D7:\n bl .Ladr_ily\n b.w .Lalu_cmp\n"
".Lo_F7:\n tst r9, #0x08\n bne.w .Lfb\n bl .Ladr_ily\n b.w .Lalu_sbc\n"
".Lo_03:\n bl .Ladr_sr\n b.w .Lalu_ora\n"
".Lo_23:\n bl .Ladr_sr\n b.w .Lalu_and\n"
".Lo_43:\n bl .Ladr_sr\n b.w .Lalu_eor\n"
".Lo_63:\n tst r9, #0x08\n bne.w .Lfb\n bl .Ladr_sr\n b.w .Lalu_adc\n"
".Lo_83:\n bl .Ladr_sr\n b.w .Lalu_sta\n"
".Lo_C3:\n bl .Ladr_sr\n b.w .Lalu_cmp\n"
".Lo_E3:\n tst r9, #0x08\n bne.w .Lfb\n bl .Ladr_sr\n b.w .Lalu_sbc\n"
".Lo_13:\n bl .Ladr_isy\n b.w .Lalu_ora\n"
".Lo_33:\n bl .Ladr_isy\n b.w .Lalu_and\n"
".Lo_53:\n bl .Ladr_isy\n b.w .Lalu_eor\n"
".Lo_73:\n tst r9, #0x08\n bne.w .Lfb\n bl .Ladr_isy\n b.w .Lalu_adc\n"
".Lo_93:\n bl .Ladr_isy\n b.w .Lalu_sta\n"
".Lo_D3:\n bl .Ladr_isy\n b.w .Lalu_cmp\n"
".Lo_F3:\n tst r9, #0x08\n bne.w .Lfb\n bl .Ladr_isy\n b.w .Lalu_sbc\n"
".Lo_0F:\n bl .Ladr_abl\n b.w .Lalu_ora\n"
".Lo_2F:\n bl .Ladr_abl\n b.w .Lalu_and\n"
".Lo_4F:\n bl .Ladr_abl\n b.w .Lalu_eor\n"
".Lo_6F:\n tst r9, #0x08\n bne.w .Lfb\n bl .Ladr_abl\n b.w .Lalu_adc\n"
".Lo_8F:\n bl .Ladr_abl\n b.w .Lalu_sta\n"
".Lo_AF:\n bl .Ladr_abl\n b.w .Lalu_lda\n"
".Lo_CF:\n bl .Ladr_abl\n b.w .Lalu_cmp\n"
".Lo_EF:\n tst r9, #0x08\n bne.w .Lfb\n bl .Ladr_abl\n b.w .Lalu_sbc\n"
".Lo_1F:\n bl .Ladr_alx\n b.w .Lalu_ora\n"
".Lo_3F:\n bl .Ladr_alx\n b.w .Lalu_and\n"
".Lo_5F:\n bl .Ladr_alx\n b.w .Lalu_eor\n"
".Lo_7F:\n tst r9, #0x08\n bne.w .Lfb\n bl .Ladr_alx\n b.w .Lalu_adc\n"
".Lo_9F:\n bl .Ladr_alx\n b.w .Lalu_sta\n"
".Lo_BF:\n bl .Ladr_alx\n b.w .Lalu_lda\n"
".Lo_DF:\n bl .Ladr_alx\n b.w .Lalu_cmp\n"
".Lo_FF:\n tst r9, #0x08\n bne.w .Lfb\n bl .Ladr_alx\n b.w .Lalu_sbc\n"
".ltorg\n"
".Lo_80:\n"
"bl .Lfetch\n sxtb r0, r0\n str r0, [sp, #" S(S0) "]\n"
"bl .Lint\n bl .Lidl\n"
"ldr r0, [sp, #" S(S0) "]\n add r8, r8, r0\n uxth r8, r8\n b.w .Lnext\n"
".Lo_82:\n"
"bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
"bl .Lfetch\n"
"ldr r1, [sp, #" S(S0) "]\n orr r0, r1, r0, lsl #8\n"
"sxth r0, r0\n"
"add r8, r8, r0\n uxth r8, r8\n"
"bl .Lint\n bl .Lidl\n"
"b.w .Lnext\n"
".Lo_50:\n tst r9, #0x40\n beq.w .Lbtk\n b.w .Lbnt\n"
".Lo_70:\n tst r9, #0x40\n bne.w .Lbtk\n b.w .Lbnt\n"
".Lo_FC:\n"
"bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"  /* S0 = adrl */
"mov r1, r7\n lsr r2, r8, #8\n uxtb r2, r2\n bl .Lwr\n"
"subs r7, #1\n uxth r7, r7\n"
"ldr r0, [sp, #" S(FE) "]\n cmp r0, #0\n beq 1f\n and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
"1:\n"
"mov r1, r7\n uxtb r2, r8\n bl .Lwr\n"
"subs r7, #1\n uxth r7, r7\n"
"ldr r0, [sp, #" S(FE) "]\n cmp r0, #0\n beq 1f\n and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
"1:\n"
"bl .Lfetch\n"
"ldr r1, [sp, #" S(S0) "]\n orr r1, r1, r0, lsl #8\n"
"add r1, r1, r5\n uxth r1, r1\n str r1, [sp, #" S(S0) "]\n"
"bl .Lidl\n"
"ldr r0, [sp, #" S(FK) "]\n lsl r0, r0, #16\n"
"ldr r1, [sp, #" S(S0) "]\n orr r1, r0, r1\n"
"bl .Lrd\n str r0, [sp, #" S(S1) "]\n"
"ldr r0, [sp, #" S(FK) "]\n lsl r0, r0, #16\n"
"ldr r1, [sp, #" S(S0) "]\n adds r1, #1\n uxth r1, r1\n orr r1, r0, r1\n"
"bl .Lint\n bl .Lrd\n"
"ldr r1, [sp, #" S(S1) "]\n orr r8, r1, r0, lsl #8\n uxth r8, r8\n"
"b.w .Lnext\n"
".Lo_F4:\n"
"bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
"bl .Lfetch\n ldr r1, [sp, #" S(S0) "]\n orr r0, r1, r0, lsl #8\n"
"str r0, [sp, #" S(S0) "]\n"
"lsr r2, r0, #8\n uxtb r2, r2\n mov r1, r7\n bl .Lwr\n"
"subs r7, #1\n uxth r7, r7\n"
"ldr r0, [sp, #" S(FE) "]\n cmp r0, #0\n beq 1f\n and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
"1: bl .Lint\n"
"ldr r0, [sp, #" S(S0) "]\n uxtb r2, r0\n mov r1, r7\n bl .Lwr\n"
"subs r7, #1\n uxth r7, r7\n"
"ldr r0, [sp, #" S(FE) "]\n cmp r0, #0\n beq 1f\n and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
"1:\n"
"b.w .Lnext\n"
".Lo_D4:\n"
"bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
"ldr r0, [sp, #" S(FDP) "]\n tst r0, #0xFF\n beq.w 1f\n bl .Lidl\n"
"1: ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
"add r1, r1, r0\n uxth r1, r1\n str r1, [sp, #" S(S1) "]\n"
"bl .Lrd\n str r0, [sp, #" S(S3) "]\n"
"ldr r1, [sp, #" S(S1) "]\n adds r1, #1\n uxth r1, r1\n"
"bl .Lrd\n"
"ldr r1, [sp, #" S(S3) "]\n orr r0, r1, r0, lsl #8\n str r0, [sp, #" S(S0) "]\n"
"lsr r2, r0, #8\n uxtb r2, r2\n mov r1, r7\n bl .Lwr\n"
"subs r7, #1\n uxth r7, r7\n"
"ldr r0, [sp, #" S(FE) "]\n cmp r0, #0\n beq 1f\n and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
"1: bl .Lint\n"
"ldr r0, [sp, #" S(S0) "]\n uxtb r2, r0\n mov r1, r7\n bl .Lwr\n"
"subs r7, #1\n uxth r7, r7\n"
"ldr r0, [sp, #" S(FE) "]\n cmp r0, #0\n beq 1f\n and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
"1:\n"
"b.w .Lnext\n"
".Lo_62:\n"
"bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
"bl .Lfetch\n ldr r1, [sp, #" S(S0) "]\n orr r0, r1, r0, lsl #8\n"
"sxth r0, r0\n"
"add r0, r8, r0\n uxth r0, r0\n str r0, [sp, #" S(S0) "]\n"
"bl .Lidl\n"
"ldr r0, [sp, #" S(S0) "]\n lsr r2, r0, #8\n uxtb r2, r2\n mov r1, r7\n bl .Lwr\n"
"subs r7, #1\n uxth r7, r7\n"
"ldr r0, [sp, #" S(FE) "]\n cmp r0, #0\n beq 1f\n and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
"1: bl .Lint\n"
"ldr r0, [sp, #" S(S0) "]\n uxtb r2, r0\n mov r1, r7\n bl .Lwr\n"
"subs r7, #1\n uxth r7, r7\n"
"ldr r0, [sp, #" S(FE) "]\n cmp r0, #0\n beq 1f\n and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
"1:\n"
"b.w .Lnext\n"
".Lo_C2:\n"
"bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
"bl .Lint\n"
"ldr r0, [sp, #" S(S0) "]\n bic r0, r9, r0\n"
"ldr r1, [sp, #" S(FE) "]\n cmp r1, #0\n beq 1f\n"
"orr r0, r0, #0x30\n"
"1:\n"
"mov r9, r0\n"
"tst r9, #0x10\n beq.w 1f\n"
"uxtb r5, r5\n uxtb r6, r6\n"
"1: bl .Lidl\n"
"b.w .Lnext\n"
".Lo_E2:\n"
"bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
"bl .Lint\n"
"ldr r0, [sp, #" S(S0) "]\n orr r0, r9, r0\n"
"ldr r1, [sp, #" S(FE) "]\n cmp r1, #0\n beq 1f\n"
"orr r0, r0, #0x30\n"
"1:\n"
"mov r9, r0\n"
"tst r9, #0x10\n beq.w 1f\n"
"uxtb r5, r5\n uxtb r6, r6\n"
"1: bl .Lidl\n"
"b.w .Lnext\n"
".Lo_04:\n"
"bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
"ldr r0, [sp, #" S(FDP) "]\n tst r0, #0xFF\n beq.w 1f\n bl .Lidl\n"
"1: ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
"add r1, r1, r0\n uxth r1, r1\n str r1, [sp, #" S(S1) "]\n"
"tst r9, #0x20\n beq.w .Ltsb04_16\n"
"bl .Lrd\n str r0, [sp, #" S(S3) "]\n"
"bl .Lidl\n"
"ldr r0, [sp, #" S(S3) "]\n uxtb r1, r4\n and r1, r1, r0\n"
"bic r9, r9, #0x02\n cmp r1, #0\n bne.w 1f\n orr r9, r9, #0x02\n"
"1: bl .Lint\n"
"ldr r1, [sp, #" S(S1) "]\n ldr r0, [sp, #" S(S3) "]\n uxtb r2, r4\n orr r2, r0, r2\n"
"bl .Lwr\n b.w .Lnext\n"
".Ltsb04_16:\n"
"ldr r1, [sp, #" S(S1) "]\n bl .Lrd\n str r0, [sp, #" S(S3) "]\n"
"ldr r1, [sp, #" S(S1) "]\n adds r1, #1\n uxth r1, r1\n"
"bl .Lrd\n ldr r1, [sp, #" S(S3) "]\n orr r0, r1, r0, lsl #8\n str r0, [sp, #" S(S3) "]\n"
"bl .Lidl\n"
"ldr r0, [sp, #" S(S3) "]\n uxth r1, r4\n and r1, r1, r0\n"
"bic r9, r9, #0x02\n cmp r1, #0\n bne.w 1f\n orr r9, r9, #0x02\n"
"1:\n"
"ldr r0, [sp, #" S(S3) "]\n uxth r1, r4\n orr r0, r0, r1\n uxth r0, r0\n str r0, [sp, #" S(S3) "]\n"
"ldr r1, [sp, #" S(S1) "]\n adds r1, #1\n uxth r1, r1\n"
"ldr r0, [sp, #" S(S3) "]\n lsr r2, r0, #8\n uxtb r2, r2\n bl .Lwr\n"
"bl .Lint\n"
"ldr r1, [sp, #" S(S1) "]\n ldr r0, [sp, #" S(S3) "]\n uxtb r2, r0\n bl .Lwr\n"
"b.w .Lnext\n"
".Lo_0C:\n"
"bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
"bl .Lfetch\n ldr r1, [sp, #" S(S0) "]\n orr r1, r1, r0, lsl #8\n"
"ldr r0, [sp, #" S(FDB) "]\n add r1, r1, r0, lsl #16\n"
"str r1, [sp, #" S(S1) "]\n"
"tst r9, #0x20\n beq.w .Ltsb0c_16\n"
"bl .Lrd\n str r0, [sp, #" S(S3) "]\n"
"bl .Lidl\n"
"ldr r0, [sp, #" S(S3) "]\n uxtb r1, r4\n and r1, r1, r0\n"
"bic r9, r9, #0x02\n cmp r1, #0\n bne.w 1f\n orr r9, r9, #0x02\n"
"1: bl .Lint\n"
"ldr r1, [sp, #" S(S1) "]\n ldr r0, [sp, #" S(S3) "]\n uxtb r2, r4\n orr r2, r0, r2\n"
"bl .Lwr\n b.w .Lnext\n"
".Ltsb0c_16:\n"
"ldr r1, [sp, #" S(S1) "]\n bl .Lrd\n str r0, [sp, #" S(S3) "]\n"
"ldr r1, [sp, #" S(S1) "]\n adds r1, #1\n bic r1, r1, #0xFF000000\n"
"bl .Lrd\n ldr r1, [sp, #" S(S3) "]\n orr r0, r1, r0, lsl #8\n str r0, [sp, #" S(S3) "]\n"
"bl .Lidl\n"
"ldr r0, [sp, #" S(S3) "]\n uxth r1, r4\n and r1, r1, r0\n"
"bic r9, r9, #0x02\n cmp r1, #0\n bne.w 1f\n orr r9, r9, #0x02\n"
"1:\n"
"ldr r0, [sp, #" S(S3) "]\n uxth r1, r4\n orr r0, r0, r1\n uxth r0, r0\n str r0, [sp, #" S(S3) "]\n"
"ldr r1, [sp, #" S(S1) "]\n adds r1, #1\n bic r1, r1, #0xFF000000\n"
"ldr r0, [sp, #" S(S3) "]\n lsr r2, r0, #8\n uxtb r2, r2\n bl .Lwr\n"
"bl .Lint\n"
"ldr r1, [sp, #" S(S1) "]\n ldr r0, [sp, #" S(S3) "]\n uxtb r2, r0\n bl .Lwr\n"
"b.w .Lnext\n"
".Lo_14:\n"
"bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
"ldr r0, [sp, #" S(FDP) "]\n tst r0, #0xFF\n beq.w 1f\n bl .Lidl\n"
"1: ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
"add r1, r1, r0\n uxth r1, r1\n str r1, [sp, #" S(S1) "]\n"
"tst r9, #0x20\n beq.w .Ltrb14_16\n"
"bl .Lrd\n str r0, [sp, #" S(S3) "]\n"
"bl .Lidl\n"
"ldr r0, [sp, #" S(S3) "]\n uxtb r1, r4\n and r1, r1, r0\n"
"bic r9, r9, #0x02\n cmp r1, #0\n bne.w 1f\n orr r9, r9, #0x02\n"
"1: bl .Lint\n"
"ldr r1, [sp, #" S(S1) "]\n ldr r0, [sp, #" S(S3) "]\n uxtb r2, r4\n bic r2, r0, r2\n"
"bl .Lwr\n b.w .Lnext\n"
".Ltrb14_16:\n"
"ldr r1, [sp, #" S(S1) "]\n bl .Lrd\n str r0, [sp, #" S(S3) "]\n"
"ldr r1, [sp, #" S(S1) "]\n adds r1, #1\n uxth r1, r1\n"
"bl .Lrd\n ldr r1, [sp, #" S(S3) "]\n orr r0, r1, r0, lsl #8\n str r0, [sp, #" S(S3) "]\n"
"bl .Lidl\n"
"ldr r0, [sp, #" S(S3) "]\n uxth r1, r4\n and r1, r1, r0\n"
"bic r9, r9, #0x02\n cmp r1, #0\n bne.w 1f\n orr r9, r9, #0x02\n"
"1:\n"
"ldr r0, [sp, #" S(S3) "]\n uxth r1, r4\n bic r0, r0, r1\n uxth r0, r0\n str r0, [sp, #" S(S3) "]\n"
"ldr r1, [sp, #" S(S1) "]\n adds r1, #1\n uxth r1, r1\n"
"ldr r0, [sp, #" S(S3) "]\n lsr r2, r0, #8\n uxtb r2, r2\n bl .Lwr\n"
"bl .Lint\n"
"ldr r1, [sp, #" S(S1) "]\n ldr r0, [sp, #" S(S3) "]\n uxtb r2, r0\n bl .Lwr\n"
"b.w .Lnext\n"
".Lo_1C:\n"
"bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
"bl .Lfetch\n ldr r1, [sp, #" S(S0) "]\n orr r1, r1, r0, lsl #8\n"
"ldr r0, [sp, #" S(FDB) "]\n add r1, r1, r0, lsl #16\n"
"str r1, [sp, #" S(S1) "]\n"
"tst r9, #0x20\n beq.w .Ltrb1c_16\n"
"bl .Lrd\n str r0, [sp, #" S(S3) "]\n"
"bl .Lidl\n"
"ldr r0, [sp, #" S(S3) "]\n uxtb r1, r4\n and r1, r1, r0\n"
"bic r9, r9, #0x02\n cmp r1, #0\n bne.w 1f\n orr r9, r9, #0x02\n"
"1: bl .Lint\n"
"ldr r1, [sp, #" S(S1) "]\n ldr r0, [sp, #" S(S3) "]\n uxtb r2, r4\n bic r2, r0, r2\n"
"bl .Lwr\n b.w .Lnext\n"
".Ltrb1c_16:\n"
"ldr r1, [sp, #" S(S1) "]\n bl .Lrd\n str r0, [sp, #" S(S3) "]\n"
"ldr r1, [sp, #" S(S1) "]\n adds r1, #1\n bic r1, r1, #0xFF000000\n"
"bl .Lrd\n ldr r1, [sp, #" S(S3) "]\n orr r0, r1, r0, lsl #8\n str r0, [sp, #" S(S3) "]\n"
"bl .Lidl\n"
"ldr r0, [sp, #" S(S3) "]\n uxth r1, r4\n and r1, r1, r0\n"
"bic r9, r9, #0x02\n cmp r1, #0\n bne.w 1f\n orr r9, r9, #0x02\n"
"1:\n"
"ldr r0, [sp, #" S(S3) "]\n uxth r1, r4\n bic r0, r0, r1\n uxth r0, r0\n str r0, [sp, #" S(S3) "]\n"
"ldr r1, [sp, #" S(S1) "]\n adds r1, #1\n bic r1, r1, #0xFF000000\n"
"ldr r0, [sp, #" S(S3) "]\n lsr r2, r0, #8\n uxtb r2, r2\n bl .Lwr\n"
"bl .Lint\n"
"ldr r1, [sp, #" S(S1) "]\n ldr r0, [sp, #" S(S3) "]\n uxtb r2, r0\n bl .Lwr\n"
"b.w .Lnext\n"
".ltorg\n"
".Lo_1B:\n"
"bl .Lint\n bl .Lidl\n"
"mov r7, r4\n uxth r7, r7\n"
"b.w .Lnext\n"
".Lo_3B:\n"
"bl .Lint\n bl .Lidl\n"
"mov r4, r7\n uxth r4, r4\n"
"mov r0, r4\n bl .Lzn16\n b.w .Lnext\n"
".Lo_5B:\n"
"bl .Lint\n bl .Lidl\n"
"uxth r0, r4\n str r0, [sp, #" S(FDP) "]\n"
"bl .Lzn16\n b.w .Lnext\n"
".Lo_7B:\n"
"bl .Lint\n bl .Lidl\n"
"ldr r4, [sp, #" S(FDP) "]\n uxth r4, r4\n"
"mov r0, r4\n bl .Lzn16\n b.w .Lnext\n"

    ".ltorg\n"
    /* ======== Group E opcodes ======== */
".Lo_64:\n"
"bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
"ldr r0, [sp, #" S(FDP) "]\n tst r0, #0xFF\n beq 1f\n bl .Lidl\n"
"1: ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
"add r1, r1, r0\n uxth r1, r1\n"
"tst r9, #0x20\n beq.w .L64_16\n"
"movs r2, #0\n bl .Lint\n bl .Lwr\n b.w .Lnext\n"
".L64_16:\n"
"str r1, [sp, #" S(S1) "]\n"
"movs r2, #0\n bl .Lwr\n"
"ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
"add r1, r1, r0\n adds r1, #1\n uxth r1, r1\n"
"movs r2, #0\n bl .Lint\n bl .Lwr\n b.w .Lnext\n"
".Lo_74:\n"
"bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
"ldr r0, [sp, #" S(FDP) "]\n tst r0, #0xFF\n beq 1f\n bl .Lidl\n"
"1: bl .Lidl\n"
"ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
"add r1, r1, r0\n add r1, r1, r5\n uxth r1, r1\n"
"tst r9, #0x20\n beq.w .L74_16\n"
"movs r2, #0\n bl .Lint\n bl .Lwr\n b.w .Lnext\n"
".L74_16:\n"
"str r1, [sp, #" S(S1) "]\n"
"movs r2, #0\n bl .Lwr\n"
"ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
"add r1, r1, r0\n add r1, r1, r5\n adds r1, #1\n uxth r1, r1\n"
"movs r2, #0\n bl .Lint\n bl .Lwr\n b.w .Lnext\n"
".Lo_9C:\n"
"bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
"bl .Lfetch\n ldr r1, [sp, #" S(S0) "]\n orr r1, r1, r0, lsl #8\n"
"ldr r0, [sp, #" S(FDB) "]\n add r1, r1, r0, lsl #16\n"
"tst r9, #0x20\n beq.w .L9c_16\n"
"movs r2, #0\n bl .Lint\n bl .Lwr\n b.w .Lnext\n"
".L9c_16:\n"
"str r1, [sp, #" S(S0) "]\n movs r2, #0\n bl .Lwr\n"
"ldr r1, [sp, #" S(S0) "]\n adds r1, #1\n bic r1, r1, #0xFF000000\n"
"movs r2, #0\n bl .Lint\n bl .Lwr\n b.w .Lnext\n"
".Lo_9E:\n"
"bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
"bl .Lfetch\n ldr r1, [sp, #" S(S0) "]\n orr r1, r1, r0, lsl #8\n"
"str r1, [sp, #" S(S0) "]\n"
"bl .Lidl\n"  /* write: always extra idle */
"ldr r0, [sp, #" S(FDB) "]\n lsl r0, r0, #16\n"
"ldr r1, [sp, #" S(S0) "]\n add r0, r0, r1\n"
"add r0, r0, r5\n"  /* + X */
"bic r1, r0, #0xFF000000\n"
"tst r9, #0x20\n beq.w .L9e_16\n"
"movs r2, #0\n bl .Lint\n bl .Lwr\n b.w .Lnext\n"
".L9e_16:\n"
"str r1, [sp, #" S(S0) "]\n movs r2, #0\n bl .Lwr\n"
"ldr r1, [sp, #" S(S0) "]\n adds r1, #1\n bic r1, r1, #0xFF000000\n"
"movs r2, #0\n bl .Lint\n bl .Lwr\n b.w .Lnext\n"
".Lo_84:\n"
"bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
"ldr r0, [sp, #" S(FDP) "]\n tst r0, #0xFF\n beq 1f\n bl .Lidl\n"
"1: ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
"add r1, r1, r0\n uxth r1, r1\n"
"tst r9, #0x10\n beq.w .L84_16\n"
"uxtb r2, r6\n bl .Lint\n bl .Lwr\n b.w .Lnext\n"
".L84_16:\n"
"str r1, [sp, #" S(S1) "]\n"
"uxtb r2, r6\n bl .Lwr\n"
"ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
"add r1, r1, r0\n adds r1, #1\n uxth r1, r1\n"
"lsr r2, r6, #8\n uxtb r2, r2\n bl .Lint\n bl .Lwr\n b.w .Lnext\n"
".Lo_8C:\n"
"bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
"bl .Lfetch\n ldr r1, [sp, #" S(S0) "]\n orr r1, r1, r0, lsl #8\n"
"ldr r0, [sp, #" S(FDB) "]\n add r1, r1, r0, lsl #16\n"
"tst r9, #0x10\n beq.w .L8c_16\n"
"uxtb r2, r6\n bl .Lint\n bl .Lwr\n b.w .Lnext\n"
".L8c_16:\n"
"str r1, [sp, #" S(S0) "]\n uxtb r2, r6\n bl .Lwr\n"
"ldr r1, [sp, #" S(S0) "]\n adds r1, #1\n bic r1, r1, #0xFF000000\n"
"lsr r2, r6, #8\n uxtb r2, r2\n bl .Lint\n bl .Lwr\n b.w .Lnext\n"
".Lo_94:\n"
"bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
"ldr r0, [sp, #" S(FDP) "]\n tst r0, #0xFF\n beq 1f\n bl .Lidl\n"
"1: bl .Lidl\n"
"ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
"add r1, r1, r0\n add r1, r1, r5\n uxth r1, r1\n"
"tst r9, #0x10\n beq.w .L94_16\n"
"uxtb r2, r6\n bl .Lint\n bl .Lwr\n b.w .Lnext\n"
".L94_16:\n"
"str r1, [sp, #" S(S1) "]\n"
"uxtb r2, r6\n bl .Lwr\n"
"ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
"add r1, r1, r0\n add r1, r1, r5\n adds r1, #1\n uxth r1, r1\n"
"lsr r2, r6, #8\n uxtb r2, r2\n bl .Lint\n bl .Lwr\n b.w .Lnext\n"
".Lo_86:\n"
"bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
"ldr r0, [sp, #" S(FDP) "]\n tst r0, #0xFF\n beq 1f\n bl .Lidl\n"
"1: ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
"add r1, r1, r0\n uxth r1, r1\n"
"tst r9, #0x10\n beq.w .L86_16\n"
"uxtb r2, r5\n bl .Lint\n bl .Lwr\n b.w .Lnext\n"
".L86_16:\n"
"str r1, [sp, #" S(S1) "]\n"
"uxtb r2, r5\n bl .Lwr\n"
"ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
"add r1, r1, r0\n adds r1, #1\n uxth r1, r1\n"
"lsr r2, r5, #8\n uxtb r2, r2\n bl .Lint\n bl .Lwr\n b.w .Lnext\n"
".Lo_8E:\n"
"bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
"bl .Lfetch\n ldr r1, [sp, #" S(S0) "]\n orr r1, r1, r0, lsl #8\n"
"ldr r0, [sp, #" S(FDB) "]\n add r1, r1, r0, lsl #16\n"
"tst r9, #0x10\n beq.w .L8e_16\n"
"uxtb r2, r5\n bl .Lint\n bl .Lwr\n b.w .Lnext\n"
".L8e_16:\n"
"str r1, [sp, #" S(S0) "]\n uxtb r2, r5\n bl .Lwr\n"
"ldr r1, [sp, #" S(S0) "]\n adds r1, #1\n bic r1, r1, #0xFF000000\n"
"lsr r2, r5, #8\n uxtb r2, r2\n bl .Lint\n bl .Lwr\n b.w .Lnext\n"
".Lo_96:\n"
"bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
"ldr r0, [sp, #" S(FDP) "]\n tst r0, #0xFF\n beq 1f\n bl .Lidl\n"
"1: bl .Lidl\n"
"ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
"add r1, r1, r0\n add r1, r1, r6\n uxth r1, r1\n"  /* +Y not +X */
"tst r9, #0x10\n beq.w .L96_16\n"
"uxtb r2, r5\n bl .Lint\n bl .Lwr\n b.w .Lnext\n"
".L96_16:\n"
"str r1, [sp, #" S(S1) "]\n"
"uxtb r2, r5\n bl .Lwr\n"
"ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
"add r1, r1, r0\n add r1, r1, r6\n adds r1, #1\n uxth r1, r1\n"  /* +Y */
"lsr r2, r5, #8\n uxtb r2, r2\n bl .Lint\n bl .Lwr\n b.w .Lnext\n"
".Lo_B4:\n"
"bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
"ldr r0, [sp, #" S(FDP) "]\n tst r0, #0xFF\n beq 1f\n bl .Lidl\n"
"1: bl .Lidl\n"
"ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
"add r1, r1, r0\n add r1, r1, r5\n uxth r1, r1\n"
"tst r9, #0x10\n beq.w .Lb4_16\n"
"bl .Lint\n bl .Lrd\n mov r6, r0\n bl .Lzn8\n b.w .Lnext\n"
".Lb4_16:\n"
"bl .Lrd\n str r0, [sp, #" S(S1) "]\n"
"ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
"add r1, r1, r0\n add r1, r1, r5\n adds r1, #1\n uxth r1, r1\n"
"bl .Lint\n bl .Lrd\n"
"lsl r0, r0, #8\n ldr r1, [sp, #" S(S1) "]\n orr r6, r1, r0\n"
"mov r0, r6\n bl .Lzn16\n b.w .Lnext\n"
".Lo_BC:\n"
"bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
"bl .Lfetch\n ldr r1, [sp, #" S(S0) "]\n orr r1, r1, r0, lsl #8\n"
"str r1, [sp, #" S(S0) "]\n"
"tst r9, #0x10\n beq 1f\n"
"add r2, r1, r5\n lsr r2, r2, #8\n lsr r1, r1, #8\n cmp r1, r2\n beq 2f\n"
"1: bl .Lidl\n"
"2: ldr r0, [sp, #" S(FDB) "]\n lsl r0, r0, #16\n"
"ldr r1, [sp, #" S(S0) "]\n add r0, r0, r1\n"
"add r0, r0, r5\n"  /* + X */
"bic r1, r0, #0xFF000000\n"
"tst r9, #0x10\n beq.w .Lbc_16\n"
"bl .Lint\n bl .Lrd\n mov r6, r0\n bl .Lzn8\n b.w .Lnext\n"
".Lbc_16:\n"
"str r1, [sp, #" S(S0) "]\n bl .Lrd\n str r0, [sp, #" S(S1) "]\n"
"ldr r1, [sp, #" S(S0) "]\n adds r1, #1\n bic r1, r1, #0xFF000000\n"
"bl .Lint\n bl .Lrd\n"
"lsl r0, r0, #8\n ldr r1, [sp, #" S(S1) "]\n orr r6, r1, r0\n"
"mov r0, r6\n bl .Lzn16\n b.w .Lnext\n"
".Lo_B6:\n"
"bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
"ldr r0, [sp, #" S(FDP) "]\n tst r0, #0xFF\n beq 1f\n bl .Lidl\n"
"1: bl .Lidl\n"
"ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
"add r1, r1, r0\n add r1, r1, r6\n uxth r1, r1\n"  /* +Y */
"tst r9, #0x10\n beq.w .Lb6_16\n"
"bl .Lint\n bl .Lrd\n mov r5, r0\n bl .Lzn8\n b.w .Lnext\n"
".Lb6_16:\n"
"bl .Lrd\n str r0, [sp, #" S(S1) "]\n"
"ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
"add r1, r1, r0\n add r1, r1, r6\n adds r1, #1\n uxth r1, r1\n"  /* +Y */
"bl .Lint\n bl .Lrd\n"
"lsl r0, r0, #8\n ldr r1, [sp, #" S(S1) "]\n orr r5, r1, r0\n"
"mov r0, r5\n bl .Lzn16\n b.w .Lnext\n"
".Lo_BE:\n"
"bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
"bl .Lfetch\n ldr r1, [sp, #" S(S0) "]\n orr r1, r1, r0, lsl #8\n"
"str r1, [sp, #" S(S0) "]\n"
"tst r9, #0x10\n beq 1f\n"
"add r2, r1, r6\n lsr r2, r2, #8\n lsr r1, r1, #8\n cmp r1, r2\n beq 2f\n"
"1: bl .Lidl\n"
"2: ldr r0, [sp, #" S(FDB) "]\n lsl r0, r0, #16\n"
"ldr r1, [sp, #" S(S0) "]\n add r0, r0, r1\n"
"add r0, r0, r6\n"  /* + Y */
"bic r1, r0, #0xFF000000\n"
"tst r9, #0x10\n beq.w .Lbe_16\n"
"bl .Lint\n bl .Lrd\n mov r5, r0\n bl .Lzn8\n b.w .Lnext\n"
".Lbe_16:\n"
"str r1, [sp, #" S(S0) "]\n bl .Lrd\n str r0, [sp, #" S(S1) "]\n"
"ldr r1, [sp, #" S(S0) "]\n adds r1, #1\n bic r1, r1, #0xFF000000\n"
"bl .Lint\n bl .Lrd\n"
"lsl r0, r0, #8\n ldr r1, [sp, #" S(S1) "]\n orr r5, r1, r0\n"
"mov r0, r5\n bl .Lzn16\n b.w .Lnext\n"
".Lo_6D:\n"
"tst r9, #0x08\n bne.w .Lfb\n"
"bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
"bl .Lfetch\n ldr r1, [sp, #" S(S0) "]\n orr r1, r1, r0, lsl #8\n"
"ldr r0, [sp, #" S(FDB) "]\n add r1, r1, r0, lsl #16\n"
"tst r9, #0x20\n beq.w .L6d_16\n"
"bl .Lint\n bl .Lrd\n"
"and  r1, r4, #0xFF\n and r2, r9, #1\n"
"add  r3, r1, r0\n add r3, r3, r2\n"
"eor  r2, r1, r0\n mvn r2, r2\n eor r12, r1, r3\n and r2, r2, r12\n"
"bic  r9, r9, #0xC3\n"
"tst  r2, #0x80\n beq 1f\n orr r9, r9, #0x40\n"
"1: cmp r3, #0x100\n blo 1f\n orr r9, r9, #0x01\n"
"1: tst r3, #0xFF\n  bne 1f\n orr r9, r9, #0x02\n"
"1: tst r3, #0x80\n  beq 1f\n orr r9, r9, #0x80\n"
"1: bfi r4, r3, #0, #8\n b.w .Lnext\n"
".L6d_16:\n"
"str r1, [sp, #" S(S0) "]\n bl .Lrd\n str r0, [sp, #" S(S1) "]\n"
"ldr r1, [sp, #" S(S0) "]\n adds r1, #1\n bic r1, r1, #0xFF000000\n"
"bl .Lint\n bl .Lrd\n"
"lsl r0, r0, #8\n ldr r1, [sp, #" S(S1) "]\n orr r0, r1, r0\n"
"uxth r1, r4\n and r2, r9, #1\n"
"add  r3, r1, r0\n add r3, r3, r2\n"
"eor  r2, r1, r0\n mvn r2, r2\n eor r12, r1, r3\n and r2, r2, r12\n"
"bic  r9, r9, #0xC3\n"
"tst  r2, #0x8000\n beq 1f\n orr r9, r9, #0x40\n"
"1: movw r2, #0\n movt r2, #1\n cmp r3, r2\n blo 1f\n orr r9, r9, #0x01\n"
"1: uxth r2, r3\n cmp r2, #0\n bne 1f\n orr r9, r9, #0x02\n"
"1: tst r3, #0x8000\n beq 1f\n orr r9, r9, #0x80\n"
"1: uxth r4, r3\n b.w .Lnext\n"
".Lo_75:\n"
"tst r9, #0x08\n bne.w .Lfb\n"
"bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
"ldr r0, [sp, #" S(FDP) "]\n tst r0, #0xFF\n beq 1f\n bl .Lidl\n"
"1: bl .Lidl\n"
"ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
"add r1, r1, r0\n add r1, r1, r5\n uxth r1, r1\n"
"tst r9, #0x20\n beq.w .L75_16\n"
"bl .Lint\n bl .Lrd\n"
"and  r1, r4, #0xFF\n and r2, r9, #1\n"
"add  r3, r1, r0\n add r3, r3, r2\n"
"eor  r2, r1, r0\n mvn r2, r2\n eor r12, r1, r3\n and r2, r2, r12\n"
"bic  r9, r9, #0xC3\n"
"tst  r2, #0x80\n beq 1f\n orr r9, r9, #0x40\n"
"1: cmp r3, #0x100\n blo 1f\n orr r9, r9, #0x01\n"
"1: tst r3, #0xFF\n  bne 1f\n orr r9, r9, #0x02\n"
"1: tst r3, #0x80\n  beq 1f\n orr r9, r9, #0x80\n"
"1: bfi r4, r3, #0, #8\n b.w .Lnext\n"
".L75_16:\n"
"bl .Lrd\n str r0, [sp, #" S(S1) "]\n"
"ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
"add r1, r1, r0\n add r1, r1, r5\n adds r1, #1\n uxth r1, r1\n"
"bl .Lint\n bl .Lrd\n"
"lsl r0, r0, #8\n ldr r1, [sp, #" S(S1) "]\n orr r0, r1, r0\n"
"uxth r1, r4\n and r2, r9, #1\n"
"add  r3, r1, r0\n add r3, r3, r2\n"
"eor  r2, r1, r0\n mvn r2, r2\n eor r12, r1, r3\n and r2, r2, r12\n"
"bic  r9, r9, #0xC3\n"
"tst  r2, #0x8000\n beq 1f\n orr r9, r9, #0x40\n"
"1: movw r2, #0\n movt r2, #1\n cmp r3, r2\n blo 1f\n orr r9, r9, #0x01\n"
"1: uxth r2, r3\n cmp r2, #0\n bne 1f\n orr r9, r9, #0x02\n"
"1: tst r3, #0x8000\n beq 1f\n orr r9, r9, #0x80\n"
"1: uxth r4, r3\n b.w .Lnext\n"
".Lo_79:\n mov r12, r6\n b.w .Ladcabxy\n"  /* r12 = Y */
".Lo_7D:\n mov r12, r5\n"                   /* r12 = X */
".Ladcabxy:\n"
"tst r9, #0x08\n bne.w .Lfb\n"
"str r12, [sp, #" S(S2) "]\n"
"bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
"bl .Lfetch\n ldr r1, [sp, #" S(S0) "]\n orr r1, r1, r0, lsl #8\n"
"str r1, [sp, #" S(S0) "]\n"
"tst r9, #0x10\n beq 1f\n"
"ldr r0, [sp, #" S(S2) "]\n"
"add r2, r1, r0\n lsr r2, r2, #8\n lsr r1, r1, #8\n cmp r1, r2\n beq 2f\n"
"1: bl .Lidl\n"
"2: ldr r0, [sp, #" S(FDB) "]\n lsl r0, r0, #16\n"
"ldr r1, [sp, #" S(S0) "]\n add r0, r0, r1\n"
"ldr r1, [sp, #" S(S2) "]\n add r0, r0, r1\n"
"bic r1, r0, #0xFF000000\n"
"tst r9, #0x20\n beq.w .Ladcabxy16\n"
"bl .Lint\n bl .Lrd\n"
"and  r1, r4, #0xFF\n and r2, r9, #1\n"
"add  r3, r1, r0\n add r3, r3, r2\n"
"eor  r2, r1, r0\n mvn r2, r2\n eor r12, r1, r3\n and r2, r2, r12\n"
"bic  r9, r9, #0xC3\n"
"tst  r2, #0x80\n beq 1f\n orr r9, r9, #0x40\n"
"1: cmp r3, #0x100\n blo 1f\n orr r9, r9, #0x01\n"
"1: tst r3, #0xFF\n  bne 1f\n orr r9, r9, #0x02\n"
"1: tst r3, #0x80\n  beq 1f\n orr r9, r9, #0x80\n"
"1: bfi r4, r3, #0, #8\n b.w .Lnext\n"
".Ladcabxy16:\n"
"str r1, [sp, #" S(S0) "]\n bl .Lrd\n str r0, [sp, #" S(S1) "]\n"
"ldr r1, [sp, #" S(S0) "]\n adds r1, #1\n bic r1, r1, #0xFF000000\n"
"bl .Lint\n bl .Lrd\n"
"lsl r0, r0, #8\n ldr r1, [sp, #" S(S1) "]\n orr r0, r1, r0\n"
"uxth r1, r4\n and r2, r9, #1\n"
"add  r3, r1, r0\n add r3, r3, r2\n"
"eor  r2, r1, r0\n mvn r2, r2\n eor r12, r1, r3\n and r2, r2, r12\n"
"bic  r9, r9, #0xC3\n"
"tst  r2, #0x8000\n beq 1f\n orr r9, r9, #0x40\n"
"1: movw r2, #0\n movt r2, #1\n cmp r3, r2\n blo 1f\n orr r9, r9, #0x01\n"
"1: uxth r2, r3\n cmp r2, #0\n bne 1f\n orr r9, r9, #0x02\n"
"1: tst r3, #0x8000\n beq 1f\n orr r9, r9, #0x80\n"
"1: uxth r4, r3\n b.w .Lnext\n"
".Lo_D5:\n"
"bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
"ldr r0, [sp, #" S(FDP) "]\n tst r0, #0xFF\n beq 1f\n bl .Lidl\n"
"1: bl .Lidl\n"
"ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
"add r1, r1, r0\n add r1, r1, r5\n uxth r1, r1\n"
"tst r9, #0x20\n beq.w .Ld5_16\n"
"bl .Lint\n bl .Lrd\n b.w .Lcmp8\n"
".Ld5_16:\n"
"bl .Lrd\n str r0, [sp, #" S(S1) "]\n"
"ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
"add r1, r1, r0\n add r1, r1, r5\n adds r1, #1\n uxth r1, r1\n"
"bl .Lint\n bl .Lrd\n"
"lsl r0, r0, #8\n ldr r1, [sp, #" S(S1) "]\n orr r0, r1, r0\n"
"b .Lcmp16\n"
".Lo_D9:\n mov r12, r6\n b.w .Lcmpabxy\n"  /* r12 = Y */
".Lo_DD:\n mov r12, r5\n"                    /* r12 = X */
".Lcmpabxy:\n"
"str r12, [sp, #" S(S2) "]\n"
"bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
"bl .Lfetch\n ldr r1, [sp, #" S(S0) "]\n orr r1, r1, r0, lsl #8\n"
"str r1, [sp, #" S(S0) "]\n"
"tst r9, #0x10\n beq 1f\n"
"ldr r0, [sp, #" S(S2) "]\n"
"add r2, r1, r0\n lsr r2, r2, #8\n lsr r1, r1, #8\n cmp r1, r2\n beq 2f\n"
"1: bl .Lidl\n"
"2: ldr r0, [sp, #" S(FDB) "]\n lsl r0, r0, #16\n"
"ldr r1, [sp, #" S(S0) "]\n add r0, r0, r1\n"
"ldr r1, [sp, #" S(S2) "]\n add r0, r0, r1\n"
"bic r1, r0, #0xFF000000\n"
"tst r9, #0x20\n beq.w .Lcmpabxy16\n"
"bl .Lint\n bl .Lrd\n b.w .Lcmp8\n"
".Lcmpabxy16:\n"
"str r1, [sp, #" S(S0) "]\n bl .Lrd\n str r0, [sp, #" S(S1) "]\n"
"ldr r1, [sp, #" S(S0) "]\n adds r1, #1\n bic r1, r1, #0xFF000000\n"
"bl .Lint\n bl .Lrd\n"
"lsl r0, r0, #8\n ldr r1, [sp, #" S(S1) "]\n orr r0, r1, r0\n"
"b .Lcmp16\n"
".Lo_AB:\n"
"bl .Lidl\n bl .Lidl\n"
"add r7, r7, #1\n uxth r7, r7\n"
"ldr r0, [sp, #" S(FE) "]\n cmp r0, #0\n beq 1f\n and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
"1: mov r1, r7\n bl .Lint\n bl .Lrd\n"
"uxtb r0, r0\n str r0, [sp, #" S(FDB) "]\n"
"bl .Lzn8\n b.w .Lnext\n"

    ".ltorg\n"
    /* ======== 256-entry jump table ======== */
    ".align 4\n"
    ".Ljt:\n"
    ".word .Lo_00+1\n"   /* 0x00 */
    ".word .Lo_01+1\n"   /* 0x01 */
    ".word .Lo_02+1\n"   /* 0x02 */
    ".word .Lo_03+1\n"   /* 0x03 */
    ".word .Lo_04+1\n"   /* 0x04 */
    ".word .Lo_05+1\n"   /* 0x05 */
    ".word .Lo_06+1\n"   /* 0x06 */
    ".word .Lo_07+1\n"   /* 0x07 */
    ".word .Lo_08+1\n"   /* 0x08 */
    ".word .Lo_09+1\n"   /* 0x09 */
    ".word .Lo_0A+1\n"   /* 0x0A */
    ".word .Lo_0B+1\n"   /* 0x0B */
    ".word .Lo_0C+1\n"   /* 0x0C */
    ".word .Lo_0D+1\n"   /* 0x0D */
    ".word .Lo_0E+1\n"   /* 0x0E */
    ".word .Lo_0F+1\n"   /* 0x0F */
    ".word .Lo_10+1\n"   /* 0x10 */
    ".word .Lo_11+1\n"   /* 0x11 */
    ".word .Lo_12+1\n"   /* 0x12 */
    ".word .Lo_13+1\n"   /* 0x13 */
    ".word .Lo_14+1\n"   /* 0x14 */
    ".word .Lo_15+1\n"   /* 0x15 */
    ".word .Lo_16+1\n"   /* 0x16 */
    ".word .Lo_17+1\n"   /* 0x17 */
    ".word .Lo_18+1\n"   /* 0x18 */
    ".word .Lo_19+1\n"   /* 0x19 */
    ".word .Lo_1A+1\n"   /* 0x1A */
    ".word .Lo_1B+1\n"   /* 0x1B */
    ".word .Lo_1C+1\n"   /* 0x1C */
    ".word .Lo_1D+1\n"   /* 0x1D */
    ".word .Lo_1E+1\n"   /* 0x1E */
    ".word .Lo_1F+1\n"   /* 0x1F */
    ".word .Lo_20+1\n"   /* 0x20 */
    ".word .Lo_21+1\n"   /* 0x21 */
    ".word .Lo_22+1\n"   /* 0x22 */
    ".word .Lo_23+1\n"   /* 0x23 */
    ".word .Lo_24+1\n"   /* 0x24 */
    ".word .Lo_25+1\n"   /* 0x25 */
    ".word .Lo_26+1\n"   /* 0x26 */
    ".word .Lo_27+1\n"   /* 0x27 */
    ".word .Lo_28+1\n"   /* 0x28 */
    ".word .Lo_29+1\n"   /* 0x29 */
    ".word .Lo_2A+1\n"   /* 0x2A */
    ".word .Lo_2B+1\n"   /* 0x2B */
    ".word .Lo_2C+1\n"   /* 0x2C */
    ".word .Lo_2D+1\n"   /* 0x2D */
    ".word .Lo_2E+1\n"   /* 0x2E */
    ".word .Lo_2F+1\n"   /* 0x2F */
    ".word .Lo_30+1\n"   /* 0x30 */
    ".word .Lo_31+1\n"   /* 0x31 */
    ".word .Lo_32+1\n"   /* 0x32 */
    ".word .Lo_33+1\n"   /* 0x33 */
    ".word .Lo_34+1\n"   /* 0x34 */
    ".word .Lo_35+1\n"   /* 0x35 */
    ".word .Lo_36+1\n"   /* 0x36 */
    ".word .Lo_37+1\n"   /* 0x37 */
    ".word .Lo_38+1\n"   /* 0x38 */
    ".word .Lo_39+1\n"   /* 0x39 */
    ".word .Lo_3A+1\n"   /* 0x3A */
    ".word .Lo_3B+1\n"   /* 0x3B */
    ".word .Lo_3C+1\n"   /* 0x3C */
    ".word .Lo_3D+1\n"   /* 0x3D */
    ".word .Lo_3E+1\n"   /* 0x3E */
    ".word .Lo_3F+1\n"   /* 0x3F */
    ".word .Lo_40+1\n"   /* 0x40 */
    ".word .Lo_41+1\n"   /* 0x41 */
    ".word .Lo_42+1\n"   /* 0x42 */
    ".word .Lo_43+1\n"   /* 0x43 */
    ".word .Lo_44+1\n"   /* 0x44 */
    ".word .Lo_45+1\n"   /* 0x45 */
    ".word .Lo_46+1\n"   /* 0x46 */
    ".word .Lo_47+1\n"   /* 0x47 */
    ".word .Lo_48+1\n"   /* 0x48 */
    ".word .Lo_49+1\n"   /* 0x49 */
    ".word .Lo_4A+1\n"   /* 0x4A */
    ".word .Lo_4B+1\n"   /* 0x4B */
    ".word .Lo_4C+1\n"   /* 0x4C */
    ".word .Lo_4D+1\n"   /* 0x4D */
    ".word .Lo_4E+1\n"   /* 0x4E */
    ".word .Lo_4F+1\n"   /* 0x4F */
    ".word .Lo_50+1\n"   /* 0x50 */
    ".word .Lo_51+1\n"   /* 0x51 */
    ".word .Lo_52+1\n"   /* 0x52 */
    ".word .Lo_53+1\n"   /* 0x53 */
    ".word .Lo_54+1\n"   /* 0x54 */
    ".word .Lo_55+1\n"   /* 0x55 */
    ".word .Lo_56+1\n"   /* 0x56 */
    ".word .Lo_57+1\n"   /* 0x57 */
    ".word .Lo_58+1\n"   /* 0x58 */
    ".word .Lo_59+1\n"   /* 0x59 */
    ".word .Lo_5A+1\n"   /* 0x5A */
    ".word .Lo_5B+1\n"   /* 0x5B */
    ".word .Lo_5C+1\n"   /* 0x5C */
    ".word .Lo_5D+1\n"   /* 0x5D */
    ".word .Lo_5E+1\n"   /* 0x5E */
    ".word .Lo_5F+1\n"   /* 0x5F */
    ".word .Lo_60+1\n"   /* 0x60 */
    ".word .Lo_61+1\n"   /* 0x61 */
    ".word .Lo_62+1\n"   /* 0x62 */
    ".word .Lo_63+1\n"   /* 0x63 */
    ".word .Lo_64+1\n"   /* 0x64 */
    ".word .Lo_65+1\n"   /* 0x65 */
    ".word .Lo_66+1\n"   /* 0x66 */
    ".word .Lo_67+1\n"   /* 0x67 */
    ".word .Lo_68+1\n"   /* 0x68 */
    ".word .Lo_69+1\n"   /* 0x69 */
    ".word .Lo_6A+1\n"   /* 0x6A */
    ".word .Lo_6B+1\n"   /* 0x6B */
    ".word .Lo_6C+1\n"   /* 0x6C */
    ".word .Lo_6D+1\n"   /* 0x6D */
    ".word .Lo_6E+1\n"   /* 0x6E */
    ".word .Lo_6F+1\n"   /* 0x6F */
    ".word .Lo_70+1\n"   /* 0x70 */
    ".word .Lo_71+1\n"   /* 0x71 */
    ".word .Lo_72+1\n"   /* 0x72 */
    ".word .Lo_73+1\n"   /* 0x73 */
    ".word .Lo_74+1\n"   /* 0x74 */
    ".word .Lo_75+1\n"   /* 0x75 */
    ".word .Lo_76+1\n"   /* 0x76 */
    ".word .Lo_77+1\n"   /* 0x77 */
    ".word .Lo_78+1\n"   /* 0x78 */
    ".word .Lo_79+1\n"   /* 0x79 */
    ".word .Lo_7A+1\n"   /* 0x7A */
    ".word .Lo_7B+1\n"   /* 0x7B */
    ".word .Lo_7C+1\n"   /* 0x7C */
    ".word .Lo_7D+1\n"   /* 0x7D */
    ".word .Lo_7E+1\n"   /* 0x7E */
    ".word .Lo_7F+1\n"   /* 0x7F */
    ".word .Lo_80+1\n"   /* 0x80 */
    ".word .Lo_81+1\n"   /* 0x81 */
    ".word .Lo_82+1\n"   /* 0x82 */
    ".word .Lo_83+1\n"   /* 0x83 */
    ".word .Lo_84+1\n"   /* 0x84 */
    ".word .Lo_85+1\n"   /* 0x85 */
    ".word .Lo_86+1\n"   /* 0x86 */
    ".word .Lo_87+1\n"   /* 0x87 */
    ".word .Lo_88+1\n"   /* 0x88 */
    ".word .Lo_89+1\n"   /* 0x89 */
    ".word .Lo_8A+1\n"   /* 0x8A */
    ".word .Lo_8B+1\n"   /* 0x8B */
    ".word .Lo_8C+1\n"   /* 0x8C */
    ".word .Lo_8D+1\n"   /* 0x8D */
    ".word .Lo_8E+1\n"   /* 0x8E */
    ".word .Lo_8F+1\n"   /* 0x8F */
    ".word .Lo_90+1\n"   /* 0x90 */
    ".word .Lo_91+1\n"   /* 0x91 */
    ".word .Lo_92+1\n"   /* 0x92 */
    ".word .Lo_93+1\n"   /* 0x93 */
    ".word .Lo_94+1\n"   /* 0x94 */
    ".word .Lo_95+1\n"   /* 0x95 */
    ".word .Lo_96+1\n"   /* 0x96 */
    ".word .Lo_97+1\n"   /* 0x97 */
    ".word .Lo_98+1\n"   /* 0x98 */
    ".word .Lo_99+1\n"   /* 0x99 */
    ".word .Lo_9A+1\n"   /* 0x9A */
    ".word .Lo_9B+1\n"   /* 0x9B */
    ".word .Lo_9C+1\n"   /* 0x9C */
    ".word .Lo_9D+1\n"   /* 0x9D */
    ".word .Lo_9E+1\n"   /* 0x9E */
    ".word .Lo_9F+1\n"   /* 0x9F */
    ".word .Lo_A0+1\n"   /* 0xA0 */
    ".word .Lo_A1+1\n"   /* 0xA1 */
    ".word .Lo_A2+1\n"   /* 0xA2 */
    ".word .Lo_A3+1\n"   /* 0xA3 */
    ".word .Lo_A4+1\n"   /* 0xA4 */
    ".word .Lo_A5+1\n"   /* 0xA5 */
    ".word .Lo_A6+1\n"   /* 0xA6 */
    ".word .Lo_A7+1\n"   /* 0xA7 */
    ".word .Lo_A8+1\n"   /* 0xA8 */
    ".word .Lo_A9+1\n"   /* 0xA9 */
    ".word .Lo_AA+1\n"   /* 0xAA */
    ".word .Lo_AB+1\n"   /* 0xAB */
    ".word .Lo_AC+1\n"   /* 0xAC */
    ".word .Lo_AD+1\n"   /* 0xAD */
    ".word .Lo_AE+1\n"   /* 0xAE */
    ".word .Lo_AF+1\n"   /* 0xAF */
    ".word .Lo_B0+1\n"   /* 0xB0 */
    ".word .Lo_B1+1\n"   /* 0xB1 */
    ".word .Lo_B2+1\n"   /* 0xB2 */
    ".word .Lo_B3+1\n"   /* 0xB3 */
    ".word .Lo_B4+1\n"   /* 0xB4 */
    ".word .Lo_B5+1\n"   /* 0xB5 */
    ".word .Lo_B6+1\n"   /* 0xB6 */
    ".word .Lo_B7+1\n"   /* 0xB7 */
    ".word .Lo_B8+1\n"   /* 0xB8 */
    ".word .Lo_B9+1\n"   /* 0xB9 */
    ".word .Lo_BA+1\n"   /* 0xBA */
    ".word .Lo_BB+1\n"   /* 0xBB */
    ".word .Lo_BC+1\n"   /* 0xBC */
    ".word .Lo_BD+1\n"   /* 0xBD */
    ".word .Lo_BE+1\n"   /* 0xBE */
    ".word .Lo_BF+1\n"   /* 0xBF */
    ".word .Lo_C0+1\n"   /* 0xC0 */
    ".word .Lo_C1+1\n"   /* 0xC1 */
    ".word .Lo_C2+1\n"   /* 0xC2 */
    ".word .Lo_C3+1\n"   /* 0xC3 */
    ".word .Lo_C4+1\n"   /* 0xC4 */
    ".word .Lo_C5+1\n"   /* 0xC5 */
    ".word .Lo_C6+1\n"   /* 0xC6 */
    ".word .Lo_C7+1\n"   /* 0xC7 */
    ".word .Lo_C8+1\n"   /* 0xC8 */
    ".word .Lo_C9+1\n"   /* 0xC9 */
    ".word .Lo_CA+1\n"   /* 0xCA */
    ".word .Lo_CB+1\n"   /* 0xCB */
    ".word .Lo_CC+1\n"   /* 0xCC */
    ".word .Lo_CD+1\n"   /* 0xCD */
    ".word .Lo_CE+1\n"   /* 0xCE */
    ".word .Lo_CF+1\n"   /* 0xCF */
    ".word .Lo_D0+1\n"   /* 0xD0 */
    ".word .Lo_D1+1\n"   /* 0xD1 */
    ".word .Lo_D2+1\n"   /* 0xD2 */
    ".word .Lo_D3+1\n"   /* 0xD3 */
    ".word .Lo_D4+1\n"   /* 0xD4 */
    ".word .Lo_D5+1\n"   /* 0xD5 */
    ".word .Lo_D6+1\n"   /* 0xD6 */
    ".word .Lo_D7+1\n"   /* 0xD7 */
    ".word .Lo_D8+1\n"   /* 0xD8 */
    ".word .Lo_D9+1\n"   /* 0xD9 */
    ".word .Lo_DA+1\n"   /* 0xDA */
    ".word .Lo_DB+1\n"   /* 0xDB */
    ".word .Lo_DC+1\n"   /* 0xDC */
    ".word .Lo_DD+1\n"   /* 0xDD */
    ".word .Lo_DE+1\n"   /* 0xDE */
    ".word .Lo_DF+1\n"   /* 0xDF */
    ".word .Lo_E0+1\n"   /* 0xE0 */
    ".word .Lo_E1+1\n"   /* 0xE1 */
    ".word .Lo_E2+1\n"   /* 0xE2 */
    ".word .Lo_E3+1\n"   /* 0xE3 */
    ".word .Lo_E4+1\n"   /* 0xE4 */
    ".word .Lo_E5+1\n"   /* 0xE5 */
    ".word .Lo_E6+1\n"   /* 0xE6 */
    ".word .Lo_E7+1\n"   /* 0xE7 */
    ".word .Lo_E8+1\n"   /* 0xE8 */
    ".word .Lo_E9+1\n"   /* 0xE9 */
    ".word .Lo_EA+1\n"   /* 0xEA */
    ".word .Lo_EB+1\n"   /* 0xEB */
    ".word .Lo_EC+1\n"   /* 0xEC */
    ".word .Lo_ED+1\n"   /* 0xED */
    ".word .Lo_EE+1\n"   /* 0xEE */
    ".word .Lo_EF+1\n"   /* 0xEF */
    ".word .Lo_F0+1\n"   /* 0xF0 */
    ".word .Lo_F1+1\n"   /* 0xF1 */
    ".word .Lo_F2+1\n"   /* 0xF2 */
    ".word .Lo_F3+1\n"   /* 0xF3 */
    ".word .Lo_F4+1\n"   /* 0xF4 */
    ".word .Lo_F5+1\n"   /* 0xF5 */
    ".word .Lo_F6+1\n"   /* 0xF6 */
    ".word .Lo_F7+1\n"   /* 0xF7 */
    ".word .Lo_F8+1\n"   /* 0xF8 */
    ".word .Lo_F9+1\n"   /* 0xF9 */
    ".word .Lo_FA+1\n"   /* 0xFA */
    ".word .Lo_FB+1\n"   /* 0xFB */
    ".word .Lo_FC+1\n"   /* 0xFC */
    ".word .Lo_FD+1\n"   /* 0xFD */
    ".word .Lo_FE+1\n"   /* 0xFE */
    ".word .Lo_FF+1\n"   /* 0xFF */
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
