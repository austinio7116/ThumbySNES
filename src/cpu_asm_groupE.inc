/*
 * cpu_asm_groupE.inc — Group E opcode implementations.
 *
 * STZ (64,74,9C,9E), STY (84,8C,94), STX (86,8E,96),
 * LDY indexed (B4,BC), LDX indexed (B6,BE),
 * ADC indexed (6D,75,79,7D), CMP indexed (D5,D9,DD),
 * PLB (AB).
 *
 * Register map (same as cpu_asm.c):
 *   R4=A  R5=X  R6=Y  R7=SP  R8=PC  R9=packed_flags  R10=Cpu*  R11=Snes*
 *   R9 flags: b0=C b1=Z b2=I b3=D b4=XF b5=MF b6=V b7=N
 *   Subroutines: .Lfetch .Lrd .Lwr .Lidl .Lint .Lzn8 .Lzn16
 *   Stack scratch: S0-S4  Frame: FDP FK FDB FE FLR
 *
 * INSERT DISPATCH ENTRIES into the dispatch chain in cpu_asm.c, before .Lfb:
 *
 *   "cmp r0, #0x64\n beq.w .Lo_64\n"   // STZ dp
 *   "cmp r0, #0x74\n beq.w .Lo_74\n"   // STZ dp,X
 *   "cmp r0, #0x9C\n beq.w .Lo_9C\n"   // STZ abs
 *   "cmp r0, #0x9E\n beq.w .Lo_9E\n"   // STZ abs,X
 *   "cmp r0, #0x84\n beq.w .Lo_84\n"   // STY dp
 *   "cmp r0, #0x8C\n beq.w .Lo_8C\n"   // STY abs
 *   "cmp r0, #0x94\n beq.w .Lo_94\n"   // STY dp,X
 *   "cmp r0, #0x86\n beq.w .Lo_86\n"   // STX dp
 *   "cmp r0, #0x8E\n beq.w .Lo_8E\n"   // STX abs
 *   "cmp r0, #0x96\n beq.w .Lo_96\n"   // STX dp,Y
 *   "cmp r0, #0xB4\n beq.w .Lo_B4\n"   // LDY dp,X
 *   "cmp r0, #0xBC\n beq.w .Lo_BC\n"   // LDY abs,X
 *   "cmp r0, #0xB6\n beq.w .Lo_B6\n"   // LDX dp,Y
 *   "cmp r0, #0xBE\n beq.w .Lo_BE\n"   // LDX abs,Y
 *   "cmp r0, #0x6D\n beq.w .Lo_6D\n"   // ADC abs
 *   "cmp r0, #0x75\n beq.w .Lo_75\n"   // ADC dp,X
 *   "cmp r0, #0x79\n beq.w .Lo_79\n"   // ADC abs,Y
 *   "cmp r0, #0x7D\n beq.w .Lo_7D\n"   // ADC abs,X
 *   "cmp r0, #0xD5\n beq.w .Lo_D5\n"   // CMP dp,X
 *   "cmp r0, #0xD9\n beq.w .Lo_D9\n"   // CMP abs,Y
 *   "cmp r0, #0xDD\n beq.w .Lo_DD\n"   // CMP abs,X
 *   "cmp r0, #0xAB\n beq.w .Lo_AB\n"   // PLB
 *
 * INSERT OPCODE BODIES below the existing opcodes, before the epilogue.
 */

/* ======================================================================== */
/* STZ — Store Zero to Memory                                                */
/* MF selects 8/16 bit.                                                      */
/* ======================================================================== */

/* ---- STZ dp (64) ---- */
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

/* ---- STZ dp,X (74) ---- */
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

/* ---- STZ abs (9C) ---- */
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

/* ---- STZ abs,X (9E) ---- */
/* write=true: always idle (no page-cross check) */
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

/* ======================================================================== */
/* STY — Store Y to Memory                                                   */
/* XF selects 8/16 bit.                                                      */
/* ======================================================================== */

/* ---- STY dp (84) ---- */
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

/* ---- STY abs (8C) ---- */
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

/* ---- STY dp,X (94) ---- */
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

/* ======================================================================== */
/* STX — Store X to Memory                                                   */
/* XF selects 8/16 bit.                                                      */
/* ======================================================================== */

/* ---- STX dp (86) ---- */
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

/* ---- STX abs (8E) ---- */
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

/* ---- STX dp,Y (96) — NOTE: indexed by Y, not X ---- */
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

/* ======================================================================== */
/* LDY indexed — Load Y from Memory (indexed addressing modes)               */
/* XF selects 8/16 bit.                                                      */
/* ======================================================================== */

/* ---- LDY dp,X (B4) ---- */
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

/* ---- LDY abs,X (BC) — page-cross check ---- */
".Lo_BC:\n"
"bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
"bl .Lfetch\n ldr r1, [sp, #" S(S0) "]\n orr r1, r1, r0, lsl #8\n"
"str r1, [sp, #" S(S0) "]\n"
/* page cross check: idle if XF=0 or page crossed */
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

/* ======================================================================== */
/* LDX indexed — Load X from Memory (indexed addressing modes)               */
/* XF selects 8/16 bit.                                                      */
/* ======================================================================== */

/* ---- LDX dp,Y (B6) — indexed by Y ---- */
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

/* ---- LDX abs,Y (BE) — page-cross check, indexed by Y ---- */
".Lo_BE:\n"
"bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
"bl .Lfetch\n ldr r1, [sp, #" S(S0) "]\n orr r1, r1, r0, lsl #8\n"
"str r1, [sp, #" S(S0) "]\n"
/* page cross check: idle if XF=0 or page crossed */
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

/* ======================================================================== */
/* ADC indexed — Add with Carry (binary mode only; decimal falls back)       */
/* MF selects 8/16 bit.                                                      */
/* ======================================================================== */

/* ---- ADC abs (6D) ---- */
".Lo_6D:\n"
"tst r9, #0x08\n bne.w .Lfb\n"
"bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
"bl .Lfetch\n ldr r1, [sp, #" S(S0) "]\n orr r1, r1, r0, lsl #8\n"
"ldr r0, [sp, #" S(FDB) "]\n add r1, r1, r0, lsl #16\n"
"tst r9, #0x20\n beq.w .L6d_16\n"
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
"1: uxth r2, r3\n cbnz r2, 1f\n orr r9, r9, #0x02\n"
"1: tst r3, #0x8000\n beq 1f\n orr r9, r9, #0x80\n"
"1: uxth r4, r3\n b.w .Lnext\n"

/* ---- ADC dp,X (75) ---- */
".Lo_75:\n"
"tst r9, #0x08\n bne.w .Lfb\n"
"bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
"ldr r0, [sp, #" S(FDP) "]\n tst r0, #0xFF\n beq 1f\n bl .Lidl\n"
"1: bl .Lidl\n"
"ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
"add r1, r1, r0\n add r1, r1, r5\n uxth r1, r1\n"
"tst r9, #0x20\n beq.w .L75_16\n"
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
"1: uxth r2, r3\n cbnz r2, 1f\n orr r9, r9, #0x02\n"
"1: tst r3, #0x8000\n beq 1f\n orr r9, r9, #0x80\n"
"1: uxth r4, r3\n b.w .Lnext\n"

/* ---- ADC abs,Y (79) / ADC abs,X (7D) — shared .Ladcabxy ---- */
".Lo_79:\n mov r12, r6\n b.w .Ladcabxy\n"  /* r12 = Y */
".Lo_7D:\n mov r12, r5\n"                   /* r12 = X */
".Ladcabxy:\n"
"tst r9, #0x08\n bne.w .Lfb\n"
"str r12, [sp, #" S(S2) "]\n"
"bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
"bl .Lfetch\n ldr r1, [sp, #" S(S0) "]\n orr r1, r1, r0, lsl #8\n"
"str r1, [sp, #" S(S0) "]\n"
/* page cross check: idle if XF=0 or page crossed */
"tst r9, #0x10\n beq 1f\n"
"ldr r0, [sp, #" S(S2) "]\n"
"add r2, r1, r0\n lsr r2, r2, #8\n lsr r1, r1, #8\n cmp r1, r2\n beq 2f\n"
"1: bl .Lidl\n"
"2: ldr r0, [sp, #" S(FDB) "]\n lsl r0, r0, #16\n"
"ldr r1, [sp, #" S(S0) "]\n add r0, r0, r1\n"
"ldr r1, [sp, #" S(S2) "]\n add r0, r0, r1\n"
"bic r1, r0, #0xFF000000\n"
"tst r9, #0x20\n beq.w .Ladcabxy16\n"
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
"1: uxth r2, r3\n cbnz r2, 1f\n orr r9, r9, #0x02\n"
"1: tst r3, #0x8000\n beq 1f\n orr r9, r9, #0x80\n"
"1: uxth r4, r3\n b.w .Lnext\n"

/* ======================================================================== */
/* CMP indexed — Compare Accumulator (indexed addressing modes)              */
/* MF selects 8/16 bit.                                                      */
/* ======================================================================== */

/* ---- CMP dp,X (D5) ---- */
".Lo_D5:\n"
"bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
"ldr r0, [sp, #" S(FDP) "]\n tst r0, #0xFF\n beq 1f\n bl .Lidl\n"
"1: bl .Lidl\n"
"ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
"add r1, r1, r0\n add r1, r1, r5\n uxth r1, r1\n"
"tst r9, #0x20\n beq.w .Ld5_16\n"
"bl .Lint\n bl .Lrd\n b .Lcmp8\n"
".Ld5_16:\n"
"bl .Lrd\n str r0, [sp, #" S(S1) "]\n"
"ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
"add r1, r1, r0\n add r1, r1, r5\n adds r1, #1\n uxth r1, r1\n"
"bl .Lint\n bl .Lrd\n"
"lsl r0, r0, #8\n ldr r1, [sp, #" S(S1) "]\n orr r0, r1, r0\n"
"b .Lcmp16\n"

/* ---- CMP abs,Y (D9) / CMP abs,X (DD) — shared .Lcmpabxy ---- */
".Lo_D9:\n mov r12, r6\n b.w .Lcmpabxy\n"  /* r12 = Y */
".Lo_DD:\n mov r12, r5\n"                    /* r12 = X */
".Lcmpabxy:\n"
"str r12, [sp, #" S(S2) "]\n"
"bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
"bl .Lfetch\n ldr r1, [sp, #" S(S0) "]\n orr r1, r1, r0, lsl #8\n"
"str r1, [sp, #" S(S0) "]\n"
/* page cross check: idle if XF=0 or page crossed */
"tst r9, #0x10\n beq 1f\n"
"ldr r0, [sp, #" S(S2) "]\n"
"add r2, r1, r0\n lsr r2, r2, #8\n lsr r1, r1, #8\n cmp r1, r2\n beq 2f\n"
"1: bl .Lidl\n"
"2: ldr r0, [sp, #" S(FDB) "]\n lsl r0, r0, #16\n"
"ldr r1, [sp, #" S(S0) "]\n add r0, r0, r1\n"
"ldr r1, [sp, #" S(S2) "]\n add r0, r0, r1\n"
"bic r1, r0, #0xFF000000\n"
"tst r9, #0x20\n beq.w .Lcmpabxy16\n"
"bl .Lint\n bl .Lrd\n b .Lcmp8\n"
".Lcmpabxy16:\n"
"str r1, [sp, #" S(S0) "]\n bl .Lrd\n str r0, [sp, #" S(S1) "]\n"
"ldr r1, [sp, #" S(S0) "]\n adds r1, #1\n bic r1, r1, #0xFF000000\n"
"bl .Lint\n bl .Lrd\n"
"lsl r0, r0, #8\n ldr r1, [sp, #" S(S1) "]\n orr r0, r1, r0\n"
"b .Lcmp16\n"

/* ======================================================================== */
/* PLB — Pull Data Bank Register (always 8-bit)                              */
/* Cycles: idle, idle, checkInt, pull byte from stack, store to DB, set Z/N  */
/* ======================================================================== */

/* ---- PLB (AB) ---- */
".Lo_AB:\n"
"bl .Lidl\n bl .Lidl\n"
/* increment SP, E-mode wrap */
"add r7, r7, #1\n uxth r7, r7\n"
"ldr r0, [sp, #" S(FE) "]\n cbz r0, 1f\n and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
"1: mov r1, r7\n bl .Lint\n bl .Lrd\n"
/* store to DB (frame slot FDB) */
"uxtb r0, r0\n str r0, [sp, #" S(FDB) "]\n"
/* set Z/N from pulled value (8-bit) */
"bl .Lzn8\n b.w .Lnext\n"
