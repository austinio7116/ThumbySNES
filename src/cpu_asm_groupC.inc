/*
 * cpu_asm_groupC.inc — ARM Thumb-2 inline asm for 65816 Group C opcodes.
 *
 * This file is meant to be #include-d (or copy-pasted) into the opcode
 * body of cpu_asm.c, inside the __asm__ volatile block, between the last
 * existing opcode and the .ltorg directive.
 *
 * Register map (same as cpu_asm.c):
 *   R4  = A      R5  = X      R6  = Y      R7  = SP
 *   R8  = PC     R9  = packed_flags  R10 = Cpu*  R11 = Snes*
 *   R9 flags: b0=C b1=Z b2=I b3=D b4=XF b5=MF b6=V b7=N
 *
 * Frame offsets (defined in cpu_asm.c — not redefined here):
 *   FK=0  FDB=4  FDP=8  FE=12  FMAX=16  FCNT=20
 *   FRD=24  FWR=28  FID=32  FFL=36  FRO=40  FLR=44
 *   S0=48  S1=52  S2=56  S3=60  S4=64
 *
 * ============================================================
 * DISPATCH TABLE ADDITIONS (add these lines to the dispatch
 * chain in .Lloop, before the .Lfb fallback label):
 *
 *   "cmp r0, #0x48\n beq.w .Lo_48\n"   // PHA
 *   "cmp r0, #0xDA\n beq.w .Lo_DA\n"   // PHX
 *   "cmp r0, #0x5A\n beq.w .Lo_5A\n"   // PHY
 *   "cmp r0, #0x08\n beq.w .Lo_08\n"   // PHP
 *   "cmp r0, #0x0B\n beq.w .Lo_0B\n"   // PHD
 *   "cmp r0, #0x4B\n beq.w .Lo_4B\n"   // PHK
 *   "cmp r0, #0x8B\n beq.w .Lo_8B\n"   // PHB
 *   "cmp r0, #0x68\n beq.w .Lo_68\n"   // PLA
 *   "cmp r0, #0xFA\n beq.w .Lo_FA\n"   // PLX
 *   "cmp r0, #0x7A\n beq.w .Lo_7A\n"   // PLY
 *   "cmp r0, #0x28\n beq.w .Lo_28\n"   // PLP
 *   "cmp r0, #0x2B\n beq.w .Lo_2B\n"   // PLD
 *   "cmp r0, #0x4C\n beq.w .Lo_4C\n"   // JMP abs
 *   "cmp r0, #0x6C\n beq.w .Lo_6C\n"   // JMP (ind)
 *   "cmp r0, #0x7C\n beq.w .Lo_7C\n"   // JMP (abs,X)
 *   "cmp r0, #0x5C\n beq.w .Lo_5C\n"   // JML abl
 *   "cmp r0, #0xDC\n beq.w .Lo_DC\n"   // JML (ial)
 *   "cmp r0, #0x22\n beq.w .Lo_22\n"   // JSL abl
 *   "cmp r0, #0x6B\n beq.w .Lo_6B\n"   // RTL
 *   "cmp r0, #0x40\n beq.w .Lo_40\n"   // RTI
 *   "cmp r0, #0xEB\n beq.w .Lo_EB\n"   // XBA
 *   "cmp r0, #0xFB\n beq.w .Lo_FB\n"   // XCE
 *   "cmp r0, #0x42\n beq.w .Lo_42\n"   // WDM
 *   "cmp r0, #0x24\n beq.w .Lo_24\n"   // BIT dp
 *   "cmp r0, #0x2C\n beq.w .Lo_2C\n"   // BIT abs
 *   "cmp r0, #0x34\n beq.w .Lo_34\n"   // BIT dp,X
 *   "cmp r0, #0x3C\n beq.w .Lo_3C\n"   // BIT abs,X
 *   "cmp r0, #0x89\n beq.w .Lo_89\n"   // BIT imm
 *   "cmp r0, #0xA1\n beq.w .Lo_A1\n"   // LDA (dp,X)  — example idx
 *   "cmp r0, #0xB1\n beq.w .Lo_B1\n"   // LDA (dp),Y  — example idy
 *   "cmp r0, #0xB2\n beq.w .Lo_B2\n"   // LDA (dp)    — example idp
 *   "cmp r0, #0xA3\n beq.w .Lo_A3\n"   // LDA sr,S    — example sr
 *   "cmp r0, #0xB3\n beq.w .Lo_B3\n"   // LDA (sr,S),Y — example isy
 *
 * ============================================================
 *
 * HELPER MACRO: E-mode SP wrap after decrement (SP wraps within page 1).
 * Usage: inline after each SP--.
 *   .Lsp_wrap_dec:
 *   ldr r0,[sp,#FE]; cbz r0,1f; and r7,r7,#0xFF; orr r7,r7,#0x100; 1:
 *
 * HELPER MACRO: E-mode SP wrap after increment.
 *   .Lsp_wrap_inc:
 *   ldr r0,[sp,#FE]; cbz r0,1f; and r7,r7,#0xFF; orr r7,r7,#0x100; 1:
 *
 * Both are inlined at every push/pull site per the cpu_asm.c pattern.
 * ============================================================
 */

/* ============================================================
 * STACK PUSH OPCODES
 * Push pattern:
 *   1. Implied cycle (.Lint check, idle or re-fetch if int pending)
 *   2. Write byte/word to SP, decrement SP (with E wrap after each dec)
 * ============================================================ */

/* ---- PHA (48) — push A: 8-bit if MF, 16-bit if !MF ---- */
/*
 * Cycles: implied(1) + write(1 or 2)
 * 8-bit: 3 cycles total; 16-bit: 4 cycles total.
 */
".Lo_48:\n"
/* implied cycle */
"bl .Lint\n"
"ldrb r0, [r10, #" S(CPU_INTWANTED) "]\n cbnz r0, 1f\n"
"bl .Lidl\n b 2f\n"
"1: ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n bl .Lrd\n"
"2:\n"
"tst r9, #0x20\n beq.w .L48_16\n"
/* 8-bit: push A low byte */
"mov r1, r7\n uxtb r2, r4\n bl .Lwr\n"
"sub r7, r7, #1\n uxth r7, r7\n"
"ldr r0, [sp, #" S(FE) "]\n cbz r0, 1f\n and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
"1: b.w .Lnext\n"
".L48_16:\n"
/* 16-bit: push A high byte first, then low byte */
"mov r1, r7\n lsr r2, r4, #8\n uxtb r2, r2\n bl .Lwr\n"
"sub r7, r7, #1\n uxth r7, r7\n"
"ldr r0, [sp, #" S(FE) "]\n cbz r0, 1f\n and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
"1: bl .Lint\n"
"mov r1, r7\n uxtb r2, r4\n bl .Lwr\n"
"sub r7, r7, #1\n uxth r7, r7\n"
"ldr r0, [sp, #" S(FE) "]\n cbz r0, 1f\n and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
"1: b.w .Lnext\n"

/* ---- PHX (DA) — push X: 8-bit if XF, 16-bit if !XF ---- */
".Lo_DA:\n"
"bl .Lint\n"
"ldrb r0, [r10, #" S(CPU_INTWANTED) "]\n cbnz r0, 1f\n"
"bl .Lidl\n b 2f\n"
"1: ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n bl .Lrd\n"
"2:\n"
"tst r9, #0x10\n beq.w .LDA_16\n"
"mov r1, r7\n uxtb r2, r5\n bl .Lwr\n"
"sub r7, r7, #1\n uxth r7, r7\n"
"ldr r0, [sp, #" S(FE) "]\n cbz r0, 1f\n and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
"1: b.w .Lnext\n"
".LDA_16:\n"
"mov r1, r7\n lsr r2, r5, #8\n uxtb r2, r2\n bl .Lwr\n"
"sub r7, r7, #1\n uxth r7, r7\n"
"ldr r0, [sp, #" S(FE) "]\n cbz r0, 1f\n and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
"1: bl .Lint\n"
"mov r1, r7\n uxtb r2, r5\n bl .Lwr\n"
"sub r7, r7, #1\n uxth r7, r7\n"
"ldr r0, [sp, #" S(FE) "]\n cbz r0, 1f\n and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
"1: b.w .Lnext\n"

/* ---- PHY (5A) — push Y: 8-bit if XF, 16-bit if !XF ---- */
".Lo_5A:\n"
"bl .Lint\n"
"ldrb r0, [r10, #" S(CPU_INTWANTED) "]\n cbnz r0, 1f\n"
"bl .Lidl\n b 2f\n"
"1: ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n bl .Lrd\n"
"2:\n"
"tst r9, #0x10\n beq.w .L5A_16\n"
"mov r1, r7\n uxtb r2, r6\n bl .Lwr\n"
"sub r7, r7, #1\n uxth r7, r7\n"
"ldr r0, [sp, #" S(FE) "]\n cbz r0, 1f\n and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
"1: b.w .Lnext\n"
".L5A_16:\n"
"mov r1, r7\n lsr r2, r6, #8\n uxtb r2, r2\n bl .Lwr\n"
"sub r7, r7, #1\n uxth r7, r7\n"
"ldr r0, [sp, #" S(FE) "]\n cbz r0, 1f\n and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
"1: bl .Lint\n"
"mov r1, r7\n uxtb r2, r6\n bl .Lwr\n"
"sub r7, r7, #1\n uxth r7, r7\n"
"ldr r0, [sp, #" S(FE) "]\n cbz r0, 1f\n and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
"1: b.w .Lnext\n"

/* ---- PHP (08) — push processor status (always 8-bit) ---- */
/*
 * Push the flags byte (R9 as packed byte). Always 8-bit regardless of MF.
 * In E mode the B flag (bit 4) is set (B = 1 for software break distinction).
 * Here we just push R9 low byte directly — the B bit is already 0 in native
 * mode and callers must set it via the C fallback for BRK; PHP always pushes
 * with B=1 in E mode. We handle the E-mode B bit by ORing bit4 when E.
 */
".Lo_08:\n"
"bl .Lint\n"
"ldrb r0, [r10, #" S(CPU_INTWANTED) "]\n cbnz r0, 1f\n"
"bl .Lidl\n b 2f\n"
"1: ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n bl .Lrd\n"
"2:\n"
/* Build flags byte in r2 */
"uxtb r2, r9\n"
/* In E mode force B (bit 4) = 1 */
"ldr r0, [sp, #" S(FE) "]\n cbz r0, 1f\n orr r2, r2, #0x10\n"
"1: mov r1, r7\n bl .Lwr\n"
"sub r7, r7, #1\n uxth r7, r7\n"
"ldr r0, [sp, #" S(FE) "]\n cbz r0, 1f\n and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
"1: b.w .Lnext\n"

/* ---- PHD (0B) — push DP (Direct Page register, always 16-bit) ---- */
/*
 * Cycles: implied(1) + write-hi(1) + write-lo(1) = 3 cycles.
 * DP is stored in frame slot FDP as a 16-bit value.
 */
".Lo_0B:\n"
"bl .Lint\n"
"ldrb r0, [r10, #" S(CPU_INTWANTED) "]\n cbnz r0, 1f\n"
"bl .Lidl\n b 2f\n"
"1: ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n bl .Lrd\n"
"2:\n"
"ldr r0, [sp, #" S(FDP) "]\n"          /* r0 = DP value */
"mov r1, r7\n lsr r2, r0, #8\n uxtb r2, r2\n bl .Lwr\n"   /* push hi */
"sub r7, r7, #1\n uxth r7, r7\n"
"ldr r1, [sp, #" S(FE) "]\n cbz r1, 1f\n and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
"1: bl .Lint\n"
"ldr r0, [sp, #" S(FDP) "]\n"
"mov r1, r7\n uxtb r2, r0\n bl .Lwr\n"                    /* push lo */
"sub r7, r7, #1\n uxth r7, r7\n"
"ldr r1, [sp, #" S(FE) "]\n cbz r1, 1f\n and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
"1: b.w .Lnext\n"

/* ---- PHK (4B) — push Program Bank K (always 8-bit) ---- */
".Lo_4B:\n"
"bl .Lint\n"
"ldrb r0, [r10, #" S(CPU_INTWANTED) "]\n cbnz r0, 1f\n"
"bl .Lidl\n b 2f\n"
"1: ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n bl .Lrd\n"
"2:\n"
"ldr r0, [sp, #" S(FK) "]\n"           /* r0 = K */
"mov r1, r7\n uxtb r2, r0\n bl .Lwr\n"
"sub r7, r7, #1\n uxth r7, r7\n"
"ldr r0, [sp, #" S(FE) "]\n cbz r0, 1f\n and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
"1: b.w .Lnext\n"

/* ---- PHB (8B) — push Data Bank DB (always 8-bit) ---- */
".Lo_8B:\n"
"bl .Lint\n"
"ldrb r0, [r10, #" S(CPU_INTWANTED) "]\n cbnz r0, 1f\n"
"bl .Lidl\n b 2f\n"
"1: ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n bl .Lrd\n"
"2:\n"
"ldr r0, [sp, #" S(FDB) "]\n"          /* r0 = DB */
"mov r1, r7\n uxtb r2, r0\n bl .Lwr\n"
"sub r7, r7, #1\n uxth r7, r7\n"
"ldr r0, [sp, #" S(FE) "]\n cbz r0, 1f\n and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
"1: b.w .Lnext\n"

/* ============================================================
 * STACK PULL OPCODES
 * Pull pattern:
 *   1. Two idle cycles
 *   2. Increment SP (with E wrap), read byte
 *   3. For 16-bit: increment SP again (with E wrap), read high byte
 * ============================================================ */

/* ---- PLA (68) — pull A: 8-bit if MF, 16-bit if !MF ---- */
".Lo_68:\n"
"bl .Lidl\n bl .Lidl\n"
"add r7, r7, #1\n uxth r7, r7\n"
"ldr r0, [sp, #" S(FE) "]\n cbz r0, 1f\n and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
"1: tst r9, #0x20\n beq.w .L68_16\n"
/* 8-bit */
"mov r1, r7\n bl .Lint\n bl .Lrd\n"
"bfi r4, r0, #0, #8\n bl .Lzn8\n b.w .Lnext\n"
".L68_16:\n"
/* pull low byte */
"mov r1, r7\n bl .Lrd\n str r0, [sp, #" S(S0) "]\n"
"add r7, r7, #1\n uxth r7, r7\n"
"ldr r0, [sp, #" S(FE) "]\n cbz r0, 1f\n and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
/* pull high byte */
"1: mov r1, r7\n bl .Lint\n bl .Lrd\n"
"lsl r0, r0, #8\n ldr r1, [sp, #" S(S0) "]\n orr r4, r1, r0\n"
"uxth r4, r4\n mov r0, r4\n bl .Lzn16\n b.w .Lnext\n"

/* ---- PLX (FA) — pull X: 8-bit if XF, 16-bit if !XF ---- */
".Lo_FA:\n"
"bl .Lidl\n bl .Lidl\n"
"add r7, r7, #1\n uxth r7, r7\n"
"ldr r0, [sp, #" S(FE) "]\n cbz r0, 1f\n and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
"1: tst r9, #0x10\n beq.w .LFA_16\n"
"mov r1, r7\n bl .Lint\n bl .Lrd\n"
"uxtb r5, r0\n mov r0, r5\n bl .Lzn8\n b.w .Lnext\n"
".LFA_16:\n"
"mov r1, r7\n bl .Lrd\n str r0, [sp, #" S(S0) "]\n"
"add r7, r7, #1\n uxth r7, r7\n"
"ldr r0, [sp, #" S(FE) "]\n cbz r0, 1f\n and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
"1: mov r1, r7\n bl .Lint\n bl .Lrd\n"
"lsl r0, r0, #8\n ldr r1, [sp, #" S(S0) "]\n orr r5, r1, r0\n"
"uxth r5, r5\n mov r0, r5\n bl .Lzn16\n b.w .Lnext\n"

/* ---- PLY (7A) — pull Y: 8-bit if XF, 16-bit if !XF ---- */
".Lo_7A:\n"
"bl .Lidl\n bl .Lidl\n"
"add r7, r7, #1\n uxth r7, r7\n"
"ldr r0, [sp, #" S(FE) "]\n cbz r0, 1f\n and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
"1: tst r9, #0x10\n beq.w .L7A_16\n"
"mov r1, r7\n bl .Lint\n bl .Lrd\n"
"uxtb r6, r0\n mov r0, r6\n bl .Lzn8\n b.w .Lnext\n"
".L7A_16:\n"
"mov r1, r7\n bl .Lrd\n str r0, [sp, #" S(S0) "]\n"
"add r7, r7, #1\n uxth r7, r7\n"
"ldr r0, [sp, #" S(FE) "]\n cbz r0, 1f\n and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
"1: mov r1, r7\n bl .Lint\n bl .Lrd\n"
"lsl r0, r0, #8\n ldr r1, [sp, #" S(S0) "]\n orr r6, r1, r0\n"
"uxth r6, r6\n mov r0, r6\n bl .Lzn16\n b.w .Lnext\n"

/* ---- PLP (28) — pull processor status ---- */
/*
 * Pull flags byte, unpack into R9. Also:
 *   - If E mode: force MF=XF=1, SP stays in page 1.
 *   - If XF set after pull: truncate X and Y to 8-bit.
 */
".Lo_28:\n"
"bl .Lidl\n bl .Lidl\n"
"add r7, r7, #1\n uxth r7, r7\n"
"ldr r0, [sp, #" S(FE) "]\n cbz r0, 1f\n and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
"1: mov r1, r7\n bl .Lint\n bl .Lrd\n"
/* r0 = flags byte pulled from stack */
"uxtb r9, r0\n"
/* If E mode: force MF and XF bits (bits 5,4) */
"ldr r0, [sp, #" S(FE) "]\n cbz r0, 1f\n"
"orr r9, r9, #0x30\n"
"and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
"1:\n"
/* If XF set: truncate X and Y to 8-bit */
"tst r9, #0x10\n beq.w 1f\n"
"uxtb r5, r5\n uxtb r6, r6\n"
"1: b.w .Lnext\n"

/* ---- PLD (2B) — pull Direct Page register (always 16-bit) ---- */
/*
 * Cycles: 2 idle + pull-lo + pull-hi + 1 idle = 5 cycles.
 * Updates FDP frame slot.
 */
".Lo_2B:\n"
"bl .Lidl\n bl .Lidl\n"
/* pull low byte */
"add r7, r7, #1\n uxth r7, r7\n"
"ldr r0, [sp, #" S(FE) "]\n cbz r0, 1f\n and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
"1: mov r1, r7\n bl .Lrd\n str r0, [sp, #" S(S0) "]\n"
/* pull high byte */
"add r7, r7, #1\n uxth r7, r7\n"
"ldr r0, [sp, #" S(FE) "]\n cbz r0, 1f\n and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
"1: mov r1, r7\n bl .Lint\n bl .Lrd\n"
"lsl r0, r0, #8\n ldr r1, [sp, #" S(S0) "]\n orr r0, r1, r0\n"
"uxth r0, r0\n str r0, [sp, #" S(FDP) "]\n"
/* set Z/N from new DP value (16-bit) */
"bl .Lzn16\n b.w .Lnext\n"

/* ============================================================
 * JUMP OPCODES
 * ============================================================ */

/* ---- JMP abs (4C) — absolute jump within current bank ---- */
/*
 * Cycles: fetch-lo(1) + fetch-hi(1) = 2 fetch cycles (already counted
 * by .Lfetch). PC = (hi<<8)|lo in same bank K.
 */
".Lo_4C:\n"
"bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"   /* lo */
"bl .Lint\n bl .Lfetch\n"                     /* hi */
"ldr r1, [sp, #" S(S0) "]\n orr r8, r1, r0, lsl #8\n uxth r8, r8\n"
"b.w .Lnext\n"

/* ---- JMP (ind) (6C) — indirect jump, bank 0 pointer ---- */
/*
 * Read 16-bit pointer from bank 0 address (operand is 16-bit in bank 0).
 * Then PC = value at [0:operand], [0:operand+1].
 * Note: wraps within bank 0 (mask off upper byte after +1).
 */
".Lo_6C:\n"
"bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"   /* ptr_lo */
"bl .Lfetch\n"                                /* ptr_hi */
"ldr r1, [sp, #" S(S0) "]\n orr r1, r1, r0, lsl #8\n"  /* r1 = pointer address (bank 0) */
"str r1, [sp, #" S(S1) "]\n"
/* read PC low byte from [0:ptr] */
"bl .Lint\n bl .Lrd\n str r0, [sp, #" S(S0) "]\n"
/* read PC high byte from [0:ptr+1] — wraps in bank 0 */
"ldr r1, [sp, #" S(S1) "]\n adds r1, #1\n uxth r1, r1\n"
"bl .Lrd\n"
"lsl r0, r0, #8\n ldr r1, [sp, #" S(S0) "]\n orr r8, r1, r0\n uxth r8, r8\n"
"b.w .Lnext\n"

/* ---- JMP (abs,X) (7C) — indexed indirect jump in bank K ---- */
/*
 * EA = K:(operand + X). Read 16-bit PC from EA.
 * An extra idle cycle is consumed for the index arithmetic.
 */
".Lo_7C:\n"
"bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"   /* ptr_lo */
"bl .Lfetch\n"                                /* ptr_hi */
"ldr r1, [sp, #" S(S0) "]\n orr r0, r1, r0, lsl #8\n"  /* r0 = operand */
"add r0, r0, r5\n uxth r0, r0\n"             /* + X (16-bit, stays in bank K) */
"ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r0, r1\n"  /* ea = K:addr */
"str r1, [sp, #" S(S0) "]\n"
"bl .Lint\n bl .Lidl\n"                       /* extra cycle for index */
/* read low byte */
"ldr r1, [sp, #" S(S0) "]\n bl .Lrd\n str r0, [sp, #" S(S1) "]\n"
/* read high byte */
"ldr r1, [sp, #" S(S0) "]\n adds r1, #1\n bic r1, r1, #0xFF000000\n"
"bl .Lrd\n"
"lsl r0, r0, #8\n ldr r1, [sp, #" S(S1) "]\n orr r8, r1, r0\n uxth r8, r8\n"
"b.w .Lnext\n"

/* ---- JML abl (5C) — long jump, sets K and PC ---- */
/*
 * Operand: 3 bytes (lo, hi, bank). Sets PC = (hi<<8)|lo, K = bank.
 */
".Lo_5C:\n"
"bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"   /* lo */
"bl .Lfetch\n str r0, [sp, #" S(S1) "]\n"   /* hi */
"bl .Lint\n bl .Lfetch\n"                     /* bank */
"str r0, [sp, #" S(FK) "]\n"                  /* K = new bank */
"ldr r1, [sp, #" S(S0) "]\n ldr r2, [sp, #" S(S1) "]\n"
"orr r8, r1, r2, lsl #8\n uxth r8, r8\n"
"b.w .Lnext\n"

/* ---- JML (ial) (DC) — indirect long jump, 24-bit pointer from bank 0 ---- */
/*
 * Read 24-bit target from [0:operand], [0:operand+1], [0:operand+2].
 * operand is a 16-bit address in bank 0.
 */
".Lo_DC:\n"
"bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"   /* ptr_lo */
"bl .Lfetch\n"                                /* ptr_hi */
"ldr r1, [sp, #" S(S0) "]\n orr r1, r1, r0, lsl #8\n"  /* r1 = pointer (bank 0) */
"str r1, [sp, #" S(S1) "]\n"
/* read target PC lo */
"bl .Lint\n bl .Lrd\n str r0, [sp, #" S(S0) "]\n"
/* read target PC hi */
"ldr r1, [sp, #" S(S1) "]\n adds r1, #1\n uxth r1, r1\n"
"bl .Lrd\n str r0, [sp, #" S(S2) "]\n"
/* read target bank */
"ldr r1, [sp, #" S(S1) "]\n adds r1, #2\n uxth r1, r1\n"
"bl .Lrd\n"
"str r0, [sp, #" S(FK) "]\n"                  /* new K */
"ldr r1, [sp, #" S(S0) "]\n ldr r2, [sp, #" S(S2) "]\n"
"orr r8, r1, r2, lsl #8\n uxth r8, r8\n"
"b.w .Lnext\n"

/* ---- JSL abl (22) — jump to subroutine long ---- */
/*
 * Push K, push PCreturn (PC+2 past the 3-byte operand, minus 1 to get
 * the address of the last operand byte), then jump long.
 * By the time we get here PC has already been incremented past the
 * opcode byte to point at the first operand byte.
 *
 * 65816 JSL timing:
 *   fetch-lo, fetch-hi, fetch-bank (3 fetches = PC += 3 from before opcode),
 *   push K, idle, push PC_ret_hi, push PC_ret_lo, then set K:PC.
 *
 * PC_ret = (current PC after 3 fetches) - 1.
 * We fetch three bytes first, then push.
 */
".Lo_22:\n"
"bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"   /* target lo */
"bl .Lfetch\n str r0, [sp, #" S(S1) "]\n"   /* target hi */
"bl .Lfetch\n str r0, [sp, #" S(S2) "]\n"   /* target bank */
/* compute ret addr = PC - 1 (last byte of JSL instruction) */
"sub r0, r8, #1\n uxth r0, r0\n str r0, [sp, #" S(S3) "]\n"
/* push K */
"ldr r0, [sp, #" S(FK) "]\n"
"mov r1, r7\n uxtb r2, r0\n bl .Lwr\n"
"sub r7, r7, #1\n uxth r7, r7\n"
"ldr r0, [sp, #" S(FE) "]\n cbz r0, 1f\n and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
/* idle cycle */
"1: bl .Lint\n bl .Lidl\n"
/* push ret_hi */
"ldr r0, [sp, #" S(S3) "]\n"
"mov r1, r7\n lsr r2, r0, #8\n uxtb r2, r2\n bl .Lwr\n"
"sub r7, r7, #1\n uxth r7, r7\n"
"ldr r0, [sp, #" S(FE) "]\n cbz r0, 1f\n and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
/* push ret_lo */
"1: ldr r0, [sp, #" S(S3) "]\n"
"mov r1, r7\n uxtb r2, r0\n bl .Lwr\n"
"sub r7, r7, #1\n uxth r7, r7\n"
"ldr r0, [sp, #" S(FE) "]\n cbz r0, 1f\n and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
/* set new K and PC */
"1: ldr r0, [sp, #" S(S2) "]\n str r0, [sp, #" S(FK) "]\n"
"ldr r1, [sp, #" S(S0) "]\n ldr r2, [sp, #" S(S1) "]\n"
"orr r8, r1, r2, lsl #8\n uxth r8, r8\n"
"b.w .Lnext\n"

/* ---- RTL (6B) — return from subroutine long ---- */
/*
 * Pull PC_lo, PC_hi, K_byte; PC += 1.
 * Cycles: 2 idle + pull-lo + pull-hi + pull-k = 6 cycles.
 */
".Lo_6B:\n"
"bl .Lidl\n bl .Lidl\n"
/* pull PC lo */
"add r7, r7, #1\n uxth r7, r7\n"
"ldr r0, [sp, #" S(FE) "]\n cbz r0, 1f\n and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
"1: mov r1, r7\n bl .Lrd\n str r0, [sp, #" S(S0) "]\n"
/* pull PC hi */
"add r7, r7, #1\n uxth r7, r7\n"
"ldr r0, [sp, #" S(FE) "]\n cbz r0, 1f\n and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
"1: mov r1, r7\n bl .Lrd\n str r0, [sp, #" S(S1) "]\n"
/* pull K */
"add r7, r7, #1\n uxth r7, r7\n"
"ldr r0, [sp, #" S(FE) "]\n cbz r0, 1f\n and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
"1: mov r1, r7\n bl .Lint\n bl .Lrd\n"
"str r0, [sp, #" S(FK) "]\n"                  /* new K */
/* assemble PC, add 1 */
"ldr r1, [sp, #" S(S0) "]\n ldr r2, [sp, #" S(S1) "]\n"
"orr r8, r1, r2, lsl #8\n"
"adds r8, #1\n uxth r8, r8\n"
"b.w .Lnext\n"

/* ---- RTI (40) — return from interrupt ---- */
/*
 * Native mode: pull P, pull PC_lo, pull PC_hi, pull K_byte.
 * E mode:      pull P, pull PC_lo, pull PC_hi (no K pull).
 * Cycles: 2 idle + pull-P + pull-PClo + pull-PChi [+ pull-K].
 */
".Lo_40:\n"
"bl .Lidl\n bl .Lidl\n"
/* pull P */
"add r7, r7, #1\n uxth r7, r7\n"
"ldr r0, [sp, #" S(FE) "]\n cbz r0, 1f\n and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
"1: mov r1, r7\n bl .Lrd\n"
/* unpack flags from r0 */
"uxtb r9, r0\n"
/* E mode: force MF+XF, SP stays in page 1 */
"ldr r0, [sp, #" S(FE) "]\n cbz r0, 1f\n"
"orr r9, r9, #0x30\n"
"and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
"1:\n"
/* If XF: truncate X,Y */
"tst r9, #0x10\n beq.w 1f\n uxtb r5, r5\n uxtb r6, r6\n"
"1:\n"
/* pull PC lo */
"add r7, r7, #1\n uxth r7, r7\n"
"ldr r0, [sp, #" S(FE) "]\n cbz r0, 1f\n and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
"1: mov r1, r7\n bl .Lrd\n str r0, [sp, #" S(S0) "]\n"
/* pull PC hi */
"add r7, r7, #1\n uxth r7, r7\n"
"ldr r0, [sp, #" S(FE) "]\n cbz r0, 1f\n and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
"1: mov r1, r7\n bl .Lrd\n str r0, [sp, #" S(S1) "]\n"
/* check E mode: if E, skip K pull */
"ldr r0, [sp, #" S(FE) "]\n cbnz r0, .Lrti_e\n"
/* native: pull K */
"add r7, r7, #1\n uxth r7, r7\n"
"mov r1, r7\n bl .Lint\n bl .Lrd\n"
"str r0, [sp, #" S(FK) "]\n"
"ldr r1, [sp, #" S(S0) "]\n ldr r2, [sp, #" S(S1) "]\n"
"orr r8, r1, r2, lsl #8\n uxth r8, r8\n"
"b.w .Lnext\n"
/* E mode: no K pull; PC-hi was last bus cycle, go directly to .Lnext */
".Lrti_e:\n"
"ldr r1, [sp, #" S(S0) "]\n ldr r2, [sp, #" S(S1) "]\n"
"orr r8, r1, r2, lsl #8\n uxth r8, r8\n"
"b.w .Lnext\n"

/* ============================================================
 * SIMPLE / SPECIAL OPCODES
 * ============================================================ */

/* ---- XBA (EB) — exchange B and A accumulators (swap bytes) ---- */
/*
 * Always 16-bit swap of A high and low bytes. Flags set from new low byte.
 * Cycles: 3 total (1 opcode fetch already counted + 2 internal).
 * Internal: idle, idle (swap and flag-set happen in the second internal cycle).
 */
".Lo_EB:\n"
"bl .Lidl\n"
/* swap low and high bytes of r4 — do the swap before the second internal cycle */
"uxtb r0, r4\n"                /* r0 = old low byte (becomes new high) */
"lsr r1, r4, #8\n uxtb r1, r1\n"  /* r1 = old high byte (becomes new low) */
"orr r4, r1, r0, lsl #8\n"    /* r4 = (old_low<<8) | old_high */
/* second internal cycle — check interrupt, do idle or re-fetch */
"bl .Lint\n"
"ldrb r0, [r10, #" S(CPU_INTWANTED) "]\n cbnz r0, 1f\n"
"bl .Lidl\n b 2f\n"
"1: ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n bl .Lrd\n"
"2:\n"
/* Z/N from new low byte (low byte of r4 after swap) */
"uxtb r0, r4\n bl .Lzn8\n"
"b.w .Lnext\n"

/* ---- XCE (FB) — exchange Carry and Emulation flags ---- */
/*
 * Swaps C (bit 0 of R9) with E (FE slot).
 * After: if new E=1, force MF=XF=1 and SP in page 1.
 *         if new E=0, no forced changes.
 *         if XF set after: truncate X,Y.
 */
".Lo_FB:\n"
"and r0, r9, #1\n"             /* r0 = old C */
"ldr r1, [sp, #" S(FE) "]\n"  /* r1 = old E */
"bfi r9, r1, #0, #1\n"        /* new C = old E */
"str r0, [sp, #" S(FE) "]\n"  /* new E = old C */
/* if new E=1: force MF+XF, SP in page 1 */
"ldr r0, [sp, #" S(FE) "]\n cbz r0, 1f\n"
"orr r9, r9, #0x30\n"
"and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
"1:\n"
/* if XF set: truncate X,Y */
"tst r9, #0x10\n beq.w 1f\n uxtb r5, r5\n uxtb r6, r6\n"
"1:\n"
/* implied cycle */
"bl .Lint\n"
"ldrb r0, [r10, #" S(CPU_INTWANTED) "]\n cbnz r0, 1f\n"
"bl .Lidl\n b.w .Lnext\n"
"1: ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n bl .Lrd\n"
"b.w .Lnext\n"

/* ---- WDM (42) — reserved/WDM: read and discard operand byte ---- */
/*
 * 2-byte, 2-cycle opcode. Just fetch the next byte and ignore it.
 */
".Lo_42:\n"
"bl .Lint\n bl .Lfetch\n"
"b.w .Lnext\n"

/* ---- WAI (CB) — wait for interrupt: fallback to C ---- */
/*
 * Sets cpu->waiting flag. We fall back to C since the scheduler must
 * spin on idle cycles until an interrupt is pending — too complex to
 * implement inline without re-entering the loop or stalling indefinitely.
 */
".Lo_CB:\n b.w .Lfb\n"

/* ---- STP (DB) — stop the processor: fallback to C ---- */
".Lo_DB_stp:\n b.w .Lfb\n"

/* ---- BRK (00) — software break: fallback to C ---- */
".Lo_00:\n b.w .Lfb\n"

/* ---- COP (02) — coprocessor enable: fallback to C ---- */
".Lo_02:\n b.w .Lfb\n"

/* ============================================================
 * BIT TEST OPCODES
 *
 * BIT sets: Z = (A & operand) == 0
 *           N = operand.bit7
 *           V = operand.bit6
 * BIT imm (89) only sets Z; N and V are unchanged.
 *
 * For 8-bit (MF=1): test against A low byte, use bits 7,6 of operand byte.
 * For 16-bit (MF=0): test against full A, N=bit15, V=bit14.
 * ============================================================ */

/* ---- BIT dp (24) ---- */
".Lo_24:\n"
"bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
"ldr r0, [sp, #" S(FDP) "]\n tst r0, #0xFF\n beq 1f\n bl .Lidl\n"
"1: ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
"add r1, r1, r0\n uxth r1, r1\n"
"tst r9, #0x20\n beq.w .L24_16\n"
"bl .Lint\n bl .Lrd\n b .Lbit8\n"
".L24_16:\n"
"bl .Lrd\n str r0, [sp, #" S(S1) "]\n"
"ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
"add r1, r1, r0\n adds r1, #1\n uxth r1, r1\n"
"bl .Lint\n bl .Lrd\n"
"lsl r0, r0, #8\n ldr r1, [sp, #" S(S1) "]\n orr r0, r1, r0\n"
"b .Lbit16\n"

/* ---- BIT abs (2C) ---- */
".Lo_2C:\n"
"bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
"bl .Lfetch\n ldr r1, [sp, #" S(S0) "]\n orr r1, r1, r0, lsl #8\n"
"ldr r0, [sp, #" S(FDB) "]\n add r1, r1, r0, lsl #16\n"
"tst r9, #0x20\n beq.w .L2C_16\n"
"bl .Lint\n bl .Lrd\n b .Lbit8\n"
".L2C_16:\n"
"str r1, [sp, #" S(S0) "]\n bl .Lrd\n str r0, [sp, #" S(S1) "]\n"
"ldr r1, [sp, #" S(S0) "]\n adds r1, #1\n bic r1, r1, #0xFF000000\n"
"bl .Lint\n bl .Lrd\n"
"lsl r0, r0, #8\n ldr r1, [sp, #" S(S1) "]\n orr r0, r1, r0\n"
"b .Lbit16\n"

/* ---- BIT dp,X (34) ---- */
".Lo_34:\n"
"bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
"ldr r0, [sp, #" S(FDP) "]\n tst r0, #0xFF\n beq 1f\n bl .Lidl\n"
"1: bl .Lidl\n"  /* indexed penalty */
"ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
"add r1, r1, r0\n add r1, r1, r5\n uxth r1, r1\n"
"tst r9, #0x20\n beq.w .L34_16\n"
"bl .Lint\n bl .Lrd\n b .Lbit8\n"
".L34_16:\n"
"bl .Lrd\n str r0, [sp, #" S(S1) "]\n"
"ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
"add r1, r1, r0\n add r1, r1, r5\n adds r1, #1\n uxth r1, r1\n"
"bl .Lint\n bl .Lrd\n"
"lsl r0, r0, #8\n ldr r1, [sp, #" S(S1) "]\n orr r0, r1, r0\n"
"b .Lbit16\n"

/* ---- BIT abs,X (3C) ---- */
".Lo_3C:\n"
"str r5, [sp, #" S(S2) "]\n"   /* S2 = X */
"bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
"bl .Lfetch\n ldr r1, [sp, #" S(S0) "]\n orr r1, r1, r0, lsl #8\n"
"str r1, [sp, #" S(S0) "]\n"   /* S0 = base */
/* page cross check (if XF=0, always idle) */
"tst r9, #0x10\n beq 1f\n"
"ldr r0, [sp, #" S(S2) "]\n"
"add r2, r1, r0\n lsr r2, r2, #8\n lsr r1, r1, #8\n cmp r1, r2\n beq.w 2f\n"
"1: bl .Lidl\n"
"2: ldr r0, [sp, #" S(FDB) "]\n lsl r0, r0, #16\n"
"ldr r1, [sp, #" S(S0) "]\n add r0, r0, r1\n"
"ldr r1, [sp, #" S(S2) "]\n add r0, r0, r1\n"
"bic r1, r0, #0xFF000000\n"    /* r1 = ea */
"tst r9, #0x20\n beq.w .L3C_16\n"
"bl .Lint\n bl .Lrd\n b .Lbit8\n"
".L3C_16:\n"
"str r1, [sp, #" S(S0) "]\n bl .Lrd\n str r0, [sp, #" S(S1) "]\n"
"ldr r1, [sp, #" S(S0) "]\n adds r1, #1\n bic r1, r1, #0xFF000000\n"
"bl .Lint\n bl .Lrd\n"
"lsl r0, r0, #8\n ldr r1, [sp, #" S(S1) "]\n orr r0, r1, r0\n"
"b .Lbit16\n"

/* ---- BIT imm (89) — only sets Z; N and V are NOT changed ---- */
/*
 * 8-bit if MF, 16-bit if !MF.
 */
".Lo_89:\n"
"tst r9, #0x20\n beq.w .L89_16\n"
/* 8-bit imm */
"ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n"
"add r8, r8, #1\n uxth r8, r8\n"
"bl .Lint\n bl .Lrd\n"
"and r0, r4, r0\n"             /* A & imm */
"bic r9, r9, #0x02\n"          /* clear Z only */
"tst r0, #0xFF\n bne.w 1f\n orr r9, r9, #0x02\n"
"1: b.w .Lnext\n"
".L89_16:\n"
/* 16-bit imm: read two bytes */
"ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n"
"add r8, r8, #1\n uxth r8, r8\n bl .Lrd\n str r0, [sp, #" S(S0) "]\n"
"ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n"
"add r8, r8, #1\n uxth r8, r8\n bl .Lint\n bl .Lrd\n"
"lsl r0, r0, #8\n ldr r1, [sp, #" S(S0) "]\n orr r0, r1, r0\n"
"uxth r1, r4\n and r0, r1, r0\n"  /* A16 & imm16 */
"bic r9, r9, #0x02\n"
"uxth r0, r0\n cbnz r0, 1f\n orr r9, r9, #0x02\n"
"1: b.w .Lnext\n"

/* BIT common finish routines: r0 = operand (8 or 16-bit) */
/* .Lbit8: 8-bit BIT finish */
".Lbit8:\n"
"and r1, r4, r0\n"              /* A & operand (8-bit) */
"bic r9, r9, #0xC2\n"          /* clear N(7), V(6), Z(1) */
"tst r1, #0xFF\n bne.w 1f\n orr r9, r9, #0x02\n"   /* Z */
"1: tst r0, #0x80\n beq.w 1f\n orr r9, r9, #0x80\n" /* N from operand */
"1: tst r0, #0x40\n beq.w 1f\n orr r9, r9, #0x40\n" /* V from operand */
"1: b.w .Lnext\n"

/* .Lbit16: 16-bit BIT finish */
".Lbit16:\n"
"uxth r1, r4\n and r1, r1, r0\n"  /* A16 & operand16 */
"bic r9, r9, #0xC2\n"
"uxth r1, r1\n cbnz r1, 1f\n orr r9, r9, #0x02\n"     /* Z */
"1: tst r0, #0x8000\n beq.w 1f\n orr r9, r9, #0x80\n" /* N from bit15 */
"1: tst r0, #0x4000\n beq.w 1f\n orr r9, r9, #0x40\n" /* V from bit14 */
"1: b.w .Lnext\n"

/* ============================================================
 * INDIRECT ADDRESSING MODE EA COMPUTATION PATTERNS
 *
 * Each pattern leaves the effective address in R1 for .Lrd/.Lwr.
 * Scratch slots: S0=dp-offset, S1=ptr-lo, S2=ptr-hi.
 * These patterns are inlined into individual opcodes below.
 *
 * PATTERN idx  — (dp,X):
 *   fetch dp byte; add DP+X (16-bit, zero bank); read 16-bit ptr; ea = DB:ptr
 *
 * PATTERN idy  — (dp),Y:
 *   fetch dp byte; read 16-bit ptr from [DP+dp]; ea = DB:ptr + Y (+idle if page cross)
 *
 * PATTERN idp  — (dp):
 *   like idy but no Y addition.
 *
 * PATTERN sr   — sr,S:
 *   fetch offset; ea = SP + offset (24-bit, bank 0)
 *
 * PATTERN isy  — (sr,S),Y:
 *   fetch offset; ea = SP+offset; read 16-bit ptr; ea = DB:ptr + Y
 * ============================================================ */

/* ---- LDA (dp,X) — indexed indirect (A1) ---- */
/*
 * idx pattern:
 *   1. fetch dp byte
 *   2. if DP low != 0: idle (DP page penalty)
 *   3. idle (for index add)
 *   4. ptr_addr = (DP + dp + X) & 0xFFFF  (page 0)
 *   5. read ptr_lo from [ptr_addr]
 *   6. read ptr_hi from [ptr_addr+1]
 *   7. ea = DB:((ptr_hi<<8)|ptr_lo)
 *   8. .Lint + read ea
 */
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

/* ---- LDA (dp),Y — indirect indexed (B1) ---- */
/*
 * idy pattern:
 *   1. fetch dp byte
 *   2. if DP low != 0: idle
 *   3. ptr_addr = DP + dp (16-bit)
 *   4. read ptr_lo from [ptr_addr]
 *   5. read ptr_hi from [ptr_addr+1] (wraps in DP page)
 *   6. base = (ptr_hi<<8)|ptr_lo
 *   7. page-cross check on base+Y vs base: if MF=0 or cross: idle
 *   8. ea = DB:(base+Y)
 *   9. .Lint + read ea
 */
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
/* page cross: if MF=0 (16-bit) always idle; if MF=1 only if page crosses */
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

/* ---- LDA (dp) — indirect (B2) ---- */
/*
 * idp pattern: like idy but no Y addition.
 * ea = DB:((ptr_hi<<8)|ptr_lo).
 */
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

/* ---- LDA sr,S — stack relative (A3) ---- */
/*
 * sr pattern:
 *   1. fetch offset byte
 *   2. idle (stack-relative penalty)
 *   3. ea = (SP + offset) & 0xFFFF, bank 0
 *   4. .Lint + read
 */
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

/* ---- LDA (sr,S),Y — stack-relative indirect indexed (B3) ---- */
/*
 * isy pattern:
 *   1. fetch offset byte
 *   2. idle (stack-relative penalty)
 *   3. ptr_addr = (SP + offset) & 0xFFFF, bank 0
 *   4. read ptr_lo from [ptr_addr]
 *   5. read ptr_hi from [ptr_addr+1]
 *   6. idle (for Y indexing)
 *   7. ea = DB:((ptr_hi<<8)|ptr_lo) + Y
 *   8. .Lint + read ea
 */
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

/*
 * END OF GROUP C
 *
 * To add STA/ORA/AND/EOR/ADC/SBC/CMP variants for idx/idy/idp/sr/isy:
 * Replicate the EA computation above (up to the .Lint+.Lrd call), then
 * substitute the operation at the end (.Lrd -> .Lwr for stores, or
 * call the appropriate ALU finish label for logic/arithmetic ops).
 *
 * Note on WAI/STP: both are dispatched above as .Lfb fallbacks because
 * they require scheduler cooperation (spin on idle until IRQ, or full stop).
 * The dispatch entries for 0xCB and 0xDB must NOT collide with existing labels.
 * Use:
 *   "cmp r0, #0xCB\n beq.w .Lfb\n"   // WAI -> C fallback
 *   "cmp r0, #0xDB\n beq.w .Lfb\n"   // STP -> C fallback
 *   "cmp r0, #0x00\n beq.w .Lfb\n"   // BRK -> C fallback
 *   "cmp r0, #0x02\n beq.w .Lfb\n"   // COP -> C fallback
 * (or keep them out of the dispatch table entirely so they fall through to .Lfb)
 */
