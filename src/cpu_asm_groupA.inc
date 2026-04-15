/*
 * cpu_asm_groupA.inc — Group A opcode implementations.
 *
 * ORA (09,05,0D,15,19,1D), AND (29,25,2D,35,39,3D), EOR (49,45,4D,55,59,5D),
 * Flag ops (18,38,58,78,D8,F8,B8,EA),
 * Register transfers (AA,A8,8A,98,9A,BA,9B,BB),
 * INC/DEC A (1A,3A), INX/DEX/INY/DEY (E8,CA,C8,88).
 *
 * INSERT DISPATCH ENTRIES into the dispatch chain in cpu_asm.c, before .Lfb:
 *
 *   "cmp r0, #0x09\n beq.w .Lo_09\n"
 *   "cmp r0, #0x05\n beq.w .Lo_05\n"
 *   "cmp r0, #0x0D\n beq.w .Lo_0D\n"
 *   "cmp r0, #0x15\n beq.w .Lo_15\n"
 *   "cmp r0, #0x19\n beq.w .Lo_19\n"
 *   "cmp r0, #0x1D\n beq.w .Lo_1D\n"
 *   "cmp r0, #0x29\n beq.w .Lo_29\n"
 *   "cmp r0, #0x25\n beq.w .Lo_25\n"
 *   "cmp r0, #0x2D\n beq.w .Lo_2D\n"
 *   "cmp r0, #0x35\n beq.w .Lo_35\n"
 *   "cmp r0, #0x39\n beq.w .Lo_39\n"
 *   "cmp r0, #0x3D\n beq.w .Lo_3D\n"
 *   "cmp r0, #0x49\n beq.w .Lo_49\n"
 *   "cmp r0, #0x45\n beq.w .Lo_45\n"
 *   "cmp r0, #0x4D\n beq.w .Lo_4D\n"
 *   "cmp r0, #0x55\n beq.w .Lo_55\n"
 *   "cmp r0, #0x59\n beq.w .Lo_59\n"
 *   "cmp r0, #0x5D\n beq.w .Lo_5D\n"
 *   "cmp r0, #0x18\n beq.w .Lo_18\n"
 *   "cmp r0, #0x38\n beq.w .Lo_38\n"
 *   "cmp r0, #0x58\n beq.w .Lo_58\n"
 *   "cmp r0, #0x78\n beq.w .Lo_78\n"
 *   "cmp r0, #0xD8\n beq.w .Lo_D8\n"
 *   "cmp r0, #0xF8\n beq.w .Lo_F8\n"
 *   "cmp r0, #0xB8\n beq.w .Lo_B8\n"
 *   "cmp r0, #0xEA\n beq.w .Lo_EA\n"
 *   "cmp r0, #0xAA\n beq.w .Lo_AA\n"
 *   "cmp r0, #0xA8\n beq.w .Lo_A8\n"
 *   "cmp r0, #0x8A\n beq.w .Lo_8A\n"
 *   "cmp r0, #0x98\n beq.w .Lo_98\n"
 *   "cmp r0, #0x9A\n beq.w .Lo_9A\n"
 *   "cmp r0, #0xBA\n beq.w .Lo_BA\n"
 *   "cmp r0, #0x9B\n beq.w .Lo_9B\n"
 *   "cmp r0, #0xBB\n beq.w .Lo_BB\n"
 *   "cmp r0, #0x1A\n beq.w .Lo_1A\n"
 *   "cmp r0, #0x3A\n beq.w .Lo_3A\n"
 *   "cmp r0, #0xE8\n beq.w .Lo_E8\n"
 *   "cmp r0, #0xCA\n beq.w .Lo_CA\n"
 *   "cmp r0, #0xC8\n beq.w .Lo_C8\n"
 *   "cmp r0, #0x88\n beq.w .Lo_88\n"
 *
 * INSERT OPCODE BODIES below the existing opcodes, before the epilogue.
 */

/* ======================================================================== */
/* ORA — Logical OR with Accumulator                                         */
/* ======================================================================== */

/* ---- ORA imm (09) ---- */
".Lo_09:\n"
"tst r9, #0x20\n beq.w .L09_16\n"
/* 8-bit: read one immediate byte from K:PC, OR into A-low */
"ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n"
"add r8, r8, #1\n uxth r8, r8\n"
"bl .Lint\n bl .Lrd\n"
"uxtb r0, r0\n uxtb r1, r4\n orr r0, r1, r0\n bfi r4, r0, #0, #8\n"
"bl .Lzn8\n b.w .Lnext\n"
".L09_16:\n"
/* 16-bit: read lo then hi immediate */
"ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n"
"add r8, r8, #1\n uxth r8, r8\n bl .Lrd\n str r0, [sp, #" S(S0) "]\n"
"ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n"
"add r8, r8, #1\n uxth r8, r8\n bl .Lint\n bl .Lrd\n"
"lsl r0, r0, #8\n ldr r1, [sp, #" S(S0) "]\n orr r0, r1, r0\n"
"uxth r1, r4\n orr r4, r1, r0\n uxth r4, r4\n"
"mov r0, r4\n bl .Lzn16\n b.w .Lnext\n"

/* ---- ORA dp (05) ---- */
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

/* ---- ORA abs (0D) ---- */
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

/* ---- ORA dp,X (15) ---- */
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

/* ---- ORA abs,Y (19) / ORA abs,X (1D) — shared .Loraxy ---- */
".Lo_19:\n mov r12, r6\n b.w .Loraxy\n"
".Lo_1D:\n mov r12, r5\n"
".Loraxy:\n"
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

/* ======================================================================== */
/* AND — Logical AND with Accumulator                                        */
/* ======================================================================== */

/* ---- AND imm (29) ---- */
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

/* ---- AND dp (25) ---- */
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

/* ---- AND abs (2D) ---- */
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

/* ---- AND dp,X (35) ---- */
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

/* ---- AND abs,Y (39) / AND abs,X (3D) — shared .Landxy ---- */
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

/* ======================================================================== */
/* EOR — Logical Exclusive-OR with Accumulator                               */
/* ======================================================================== */

/* ---- EOR imm (49) ---- */
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

/* ---- EOR dp (45) ---- */
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

/* ---- EOR abs (4D) ---- */
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

/* ---- EOR dp,X (55) ---- */
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

/* ---- EOR abs,Y (59) / EOR abs,X (5D) — shared .Leorxy ---- */
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

/* ======================================================================== */
/* Simple flag-bit operations (2-cycle implied)                              */
/* Each uses the same implied-mode cycle pattern: .Lint then idle/read,     */
/* but for flag-only ops the CPU spec just does an internal operation cycle. */
/* We match the pattern used by existing branches: bl .Lint then bl .Lidl.  */
/* ======================================================================== */

/* ---- CLC (18) ---- */
".Lo_18:\n"
"bl .Lint\n"
"ldrb r0, [r10, #" S(CPU_INTWANTED) "]\n"
"cbnz r0, 1f\n bl .Lidl\n b 2f\n"
"1: ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n bl .Lrd\n"
"2: bic r9, r9, #0x01\n b.w .Lnext\n"

/* ---- SEC (38) ---- */
".Lo_38:\n"
"bl .Lint\n"
"ldrb r0, [r10, #" S(CPU_INTWANTED) "]\n"
"cbnz r0, 1f\n bl .Lidl\n b 2f\n"
"1: ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n bl .Lrd\n"
"2: orr r9, r9, #0x01\n b.w .Lnext\n"

/* ---- CLI (58) ---- */
".Lo_58:\n"
"bl .Lint\n"
"ldrb r0, [r10, #" S(CPU_INTWANTED) "]\n"
"cbnz r0, 1f\n bl .Lidl\n b 2f\n"
"1: ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n bl .Lrd\n"
"2: bic r9, r9, #0x04\n b.w .Lnext\n"

/* ---- SEI (78) ---- */
".Lo_78:\n"
"bl .Lint\n"
"ldrb r0, [r10, #" S(CPU_INTWANTED) "]\n"
"cbnz r0, 1f\n bl .Lidl\n b 2f\n"
"1: ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n bl .Lrd\n"
"2: orr r9, r9, #0x04\n b.w .Lnext\n"

/* ---- CLD (D8) ---- */
".Lo_D8:\n"
"bl .Lint\n"
"ldrb r0, [r10, #" S(CPU_INTWANTED) "]\n"
"cbnz r0, 1f\n bl .Lidl\n b 2f\n"
"1: ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n bl .Lrd\n"
"2: bic r9, r9, #0x08\n b.w .Lnext\n"

/* ---- SED (F8) ---- */
".Lo_F8:\n"
"bl .Lint\n"
"ldrb r0, [r10, #" S(CPU_INTWANTED) "]\n"
"cbnz r0, 1f\n bl .Lidl\n b 2f\n"
"1: ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n bl .Lrd\n"
"2: orr r9, r9, #0x08\n b.w .Lnext\n"

/* ---- CLV (B8) ---- */
".Lo_B8:\n"
"bl .Lint\n"
"ldrb r0, [r10, #" S(CPU_INTWANTED) "]\n"
"cbnz r0, 1f\n bl .Lidl\n b 2f\n"
"1: ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n bl .Lrd\n"
"2: bic r9, r9, #0x40\n b.w .Lnext\n"

/* ---- NOP (EA) ---- */
".Lo_EA:\n"
"bl .Lint\n"
"ldrb r0, [r10, #" S(CPU_INTWANTED) "]\n"
"cbnz r0, 1f\n bl .Lidl\n b.w .Lnext\n"
"1: ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n bl .Lrd\n"
"b.w .Lnext\n"

/* ======================================================================== */
/* Register Transfers (implied 2-cycle)                                      */
/*                                                                           */
/* Implied cycle helper macro (inline):                                      */
/*   bl .Lint                                                                */
/*   ldrb r0, [r10, #CPU_INTWANTED]                                         */
/*   cbnz r0, 1f / bl .Lidl / b 2f                                          */
/*   1: ldr r1, [sp, #FK] / lsl r1, r1, #16 / orr r1, r1, r8 / bl .Lrd    */
/*   2: <transfer>                                                           */
/* ======================================================================== */

/* ---- TAX (AA) — A -> X, respect XF for width, set Z/N ---- */
".Lo_AA:\n"
"bl .Lint\n"
"ldrb r0, [r10, #" S(CPU_INTWANTED) "]\n"
"cbnz r0, 1f\n bl .Lidl\n b 2f\n"
"1: ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n bl .Lrd\n"
"2: tst r9, #0x10\n beq.w .Ltax16\n"
/* 8-bit: X gets low byte of A */
"uxtb r5, r4\n mov r0, r5\n bl .Lzn8\n b.w .Lnext\n"
".Ltax16:\n"
"uxth r5, r4\n mov r0, r5\n bl .Lzn16\n b.w .Lnext\n"

/* ---- TAY (A8) — A -> Y, respect XF, set Z/N ---- */
".Lo_A8:\n"
"bl .Lint\n"
"ldrb r0, [r10, #" S(CPU_INTWANTED) "]\n"
"cbnz r0, 1f\n bl .Lidl\n b 2f\n"
"1: ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n bl .Lrd\n"
"2: tst r9, #0x10\n beq.w .Ltay16\n"
"uxtb r6, r4\n mov r0, r6\n bl .Lzn8\n b.w .Lnext\n"
".Ltay16:\n"
"uxth r6, r4\n mov r0, r6\n bl .Lzn16\n b.w .Lnext\n"

/* ---- TXA (8A) — X -> A, respect MF, set Z/N ---- */
".Lo_8A:\n"
"bl .Lint\n"
"ldrb r0, [r10, #" S(CPU_INTWANTED) "]\n"
"cbnz r0, 1f\n bl .Lidl\n b 2f\n"
"1: ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n bl .Lrd\n"
"2: tst r9, #0x20\n beq.w .Ltxa16\n"
/* 8-bit MF: put low byte of X into low byte of A */
"uxtb r0, r5\n bfi r4, r0, #0, #8\n bl .Lzn8\n b.w .Lnext\n"
".Ltxa16:\n"
"uxth r4, r5\n mov r0, r4\n bl .Lzn16\n b.w .Lnext\n"

/* ---- TYA (98) — Y -> A, respect MF, set Z/N ---- */
".Lo_98:\n"
"bl .Lint\n"
"ldrb r0, [r10, #" S(CPU_INTWANTED) "]\n"
"cbnz r0, 1f\n bl .Lidl\n b 2f\n"
"1: ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n bl .Lrd\n"
"2: tst r9, #0x20\n beq.w .Ltya16\n"
"uxtb r0, r6\n bfi r4, r0, #0, #8\n bl .Lzn8\n b.w .Lnext\n"
".Ltya16:\n"
"uxth r4, r6\n mov r0, r4\n bl .Lzn16\n b.w .Lnext\n"

/* ---- TXS (9A) — X -> SP, no flags, no width consideration ---- */
/* SP is always 16-bit in native mode; in emulation mode high byte = 0x01.  */
/* We just copy X (the full 16-bit value) to SP; E-mode wrapping handled by */
/* the C fallback if needed, but snes9x2002 doesn't special-case here.      */
".Lo_9A:\n"
"bl .Lint\n"
"ldrb r0, [r10, #" S(CPU_INTWANTED) "]\n"
"cbnz r0, 1f\n bl .Lidl\n b 2f\n"
"1: ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n bl .Lrd\n"
"2: mov r7, r5\n uxth r7, r7\n"
/* In emulation mode SP high byte must stay 0x01 */
"ldr r0, [sp, #" S(FE) "]\n cbz r0, 3f\n"
"and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
"3: b.w .Lnext\n"

/* ---- TSX (BA) — SP -> X, respect XF, set Z/N ---- */
".Lo_BA:\n"
"bl .Lint\n"
"ldrb r0, [r10, #" S(CPU_INTWANTED) "]\n"
"cbnz r0, 1f\n bl .Lidl\n b 2f\n"
"1: ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n bl .Lrd\n"
"2: tst r9, #0x10\n beq.w .Ltsx16\n"
"uxtb r5, r7\n mov r0, r5\n bl .Lzn8\n b.w .Lnext\n"
".Ltsx16:\n"
"uxth r5, r7\n mov r0, r5\n bl .Lzn16\n b.w .Lnext\n"

/* ---- TXY (9B) — X -> Y, respect XF, set Z/N ---- */
".Lo_9B:\n"
"bl .Lint\n"
"ldrb r0, [r10, #" S(CPU_INTWANTED) "]\n"
"cbnz r0, 1f\n bl .Lidl\n b 2f\n"
"1: ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n bl .Lrd\n"
"2: tst r9, #0x10\n beq.w .Ltxy16\n"
"uxtb r6, r5\n mov r0, r6\n bl .Lzn8\n b.w .Lnext\n"
".Ltxy16:\n"
"uxth r6, r5\n mov r0, r6\n bl .Lzn16\n b.w .Lnext\n"

/* ---- TYX (BB) — Y -> X, respect XF, set Z/N ---- */
".Lo_BB:\n"
"bl .Lint\n"
"ldrb r0, [r10, #" S(CPU_INTWANTED) "]\n"
"cbnz r0, 1f\n bl .Lidl\n b 2f\n"
"1: ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n bl .Lrd\n"
"2: tst r9, #0x10\n beq.w .Ltyx16\n"
"uxtb r5, r6\n mov r0, r5\n bl .Lzn8\n b.w .Lnext\n"
".Ltyx16:\n"
"uxth r5, r6\n mov r0, r5\n bl .Lzn16\n b.w .Lnext\n"

/* ======================================================================== */
/* INC / DEC Accumulator (1A / 3A) — implied 2-cycle                        */
/* ======================================================================== */

/* ---- INC A (1A) ---- */
".Lo_1A:\n"
"bl .Lint\n"
"ldrb r0, [r10, #" S(CPU_INTWANTED) "]\n"
"cbnz r0, 1f\n bl .Lidl\n b 2f\n"
"1: ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n bl .Lrd\n"
"2: tst r9, #0x20\n beq.w .L1A_16\n"
/* 8-bit: increment low byte of A */
"uxtb r0, r4\n adds r0, #1\n uxtb r0, r0\n bfi r4, r0, #0, #8\n"
"bl .Lzn8\n b.w .Lnext\n"
".L1A_16:\n"
"uxth r4, r4\n adds r4, #1\n uxth r4, r4\n"
"mov r0, r4\n bl .Lzn16\n b.w .Lnext\n"

/* ---- DEC A (3A) ---- */
".Lo_3A:\n"
"bl .Lint\n"
"ldrb r0, [r10, #" S(CPU_INTWANTED) "]\n"
"cbnz r0, 1f\n bl .Lidl\n b 2f\n"
"1: ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n bl .Lrd\n"
"2: tst r9, #0x20\n beq.w .L3A_16\n"
"uxtb r0, r4\n subs r0, #1\n uxtb r0, r0\n bfi r4, r0, #0, #8\n"
"bl .Lzn8\n b.w .Lnext\n"
".L3A_16:\n"
"uxth r4, r4\n subs r4, #1\n uxth r4, r4\n"
"mov r0, r4\n bl .Lzn16\n b.w .Lnext\n"

/* ======================================================================== */
/* INX / DEX / INY / DEY (E8 / CA / C8 / 88) — implied 2-cycle             */
/* ======================================================================== */

/* ---- INX (E8) ---- */
".Lo_E8:\n"
"bl .Lint\n"
"ldrb r0, [r10, #" S(CPU_INTWANTED) "]\n"
"cbnz r0, 1f\n bl .Lidl\n b 2f\n"
"1: ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n bl .Lrd\n"
"2: tst r9, #0x10\n beq.w .LE8_16\n"
"uxtb r0, r5\n adds r0, #1\n uxtb r5, r0\n"
"bl .Lzn8\n b.w .Lnext\n"
".LE8_16:\n"
"uxth r5, r5\n adds r5, #1\n uxth r5, r5\n"
"mov r0, r5\n bl .Lzn16\n b.w .Lnext\n"

/* ---- DEX (CA) ---- */
".Lo_CA:\n"
"bl .Lint\n"
"ldrb r0, [r10, #" S(CPU_INTWANTED) "]\n"
"cbnz r0, 1f\n bl .Lidl\n b 2f\n"
"1: ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n bl .Lrd\n"
"2: tst r9, #0x10\n beq.w .LCA_16\n"
"uxtb r0, r5\n subs r0, #1\n uxtb r5, r0\n"
"bl .Lzn8\n b.w .Lnext\n"
".LCA_16:\n"
"uxth r5, r5\n subs r5, #1\n uxth r5, r5\n"
"mov r0, r5\n bl .Lzn16\n b.w .Lnext\n"

/* ---- INY (C8) ---- */
".Lo_C8:\n"
"bl .Lint\n"
"ldrb r0, [r10, #" S(CPU_INTWANTED) "]\n"
"cbnz r0, 1f\n bl .Lidl\n b 2f\n"
"1: ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n bl .Lrd\n"
"2: tst r9, #0x10\n beq.w .LC8_16\n"
"uxtb r0, r6\n adds r0, #1\n uxtb r6, r0\n"
"bl .Lzn8\n b.w .Lnext\n"
".LC8_16:\n"
"uxth r6, r6\n adds r6, #1\n uxth r6, r6\n"
"mov r0, r6\n bl .Lzn16\n b.w .Lnext\n"

/* ---- DEY (88) ---- */
".Lo_88:\n"
"bl .Lint\n"
"ldrb r0, [r10, #" S(CPU_INTWANTED) "]\n"
"cbnz r0, 1f\n bl .Lidl\n b 2f\n"
"1: ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n bl .Lrd\n"
"2: tst r9, #0x10\n beq.w .L88_16\n"
"uxtb r0, r6\n subs r0, #1\n uxtb r6, r0\n"
"bl .Lzn8\n b.w .Lnext\n"
".L88_16:\n"
"uxth r6, r6\n subs r6, #1\n uxth r6, r6\n"
"mov r0, r6\n bl .Lzn16\n b.w .Lnext\n"
