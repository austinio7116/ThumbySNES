/*
 * cpu_asm_groupB.inc — GROUP B opcode implementations
 *
 * SBC, shifts/rotates (accumulator + memory RMW), INC/DEC memory, CPX/CPY
 *
 * Register map (same as cpu_asm.c):
 *   R4=A  R5=X  R6=Y  R7=SP  R8=PC  R9=packed_flags  R10=Cpu*  R11=Snes*
 *   R9 flags: b0=C b1=Z b2=I b3=D b4=XF b5=MF b6=V b7=N
 *   Subroutines: .Lfetch .Lrd .Lwr .Lidl .Lint .Lzn8 .Lzn16
 *   Stack scratch: S0-S4  Frame: FDP FK FDB FE
 *
 * Dispatch lines to insert in the main dispatch table:
 *
 *   "cmp r0, #0xE9\n beq.w .Lo_E9\n"   // SBC imm
 *   "cmp r0, #0xE5\n beq.w .Lo_E5\n"   // SBC dp
 *   "cmp r0, #0xED\n beq.w .Lo_ED\n"   // SBC abs
 *   "cmp r0, #0xF5\n beq.w .Lo_F5\n"   // SBC dp,X
 *   "cmp r0, #0xF9\n beq.w .Lo_F9\n"   // SBC abs,Y
 *   "cmp r0, #0xFD\n beq.w .Lo_FD\n"   // SBC abs,X
 *   "cmp r0, #0x0A\n beq.w .Lo_0A\n"   // ASL A
 *   "cmp r0, #0x4A\n beq.w .Lo_4A\n"   // LSR A
 *   "cmp r0, #0x2A\n beq.w .Lo_2A\n"   // ROL A
 *   "cmp r0, #0x6A\n beq.w .Lo_6A\n"   // ROR A
 *   "cmp r0, #0x06\n beq.w .Lo_06\n"   // ASL dp
 *   "cmp r0, #0x0E\n beq.w .Lo_0E\n"   // ASL abs
 *   "cmp r0, #0x16\n beq.w .Lo_16\n"   // ASL dp,X
 *   "cmp r0, #0x1E\n beq.w .Lo_1E\n"   // ASL abs,X
 *   "cmp r0, #0x46\n beq.w .Lo_46\n"   // LSR dp
 *   "cmp r0, #0x4E\n beq.w .Lo_4E\n"   // LSR abs
 *   "cmp r0, #0x56\n beq.w .Lo_56\n"   // LSR dp,X
 *   "cmp r0, #0x5E\n beq.w .Lo_5E\n"   // LSR abs,X
 *   "cmp r0, #0x26\n beq.w .Lo_26\n"   // ROL dp
 *   "cmp r0, #0x2E\n beq.w .Lo_2E\n"   // ROL abs
 *   "cmp r0, #0x36\n beq.w .Lo_36\n"   // ROL dp,X
 *   "cmp r0, #0x3E\n beq.w .Lo_3E\n"   // ROL abs,X
 *   "cmp r0, #0x66\n beq.w .Lo_66\n"   // ROR dp
 *   "cmp r0, #0x6E\n beq.w .Lo_6E\n"   // ROR abs
 *   "cmp r0, #0x76\n beq.w .Lo_76\n"   // ROR dp,X
 *   "cmp r0, #0x7E\n beq.w .Lo_7E\n"   // ROR abs,X
 *   "cmp r0, #0xE6\n beq.w .Lo_E6\n"   // INC dp
 *   "cmp r0, #0xEE\n beq.w .Lo_EE\n"   // INC abs
 *   "cmp r0, #0xF6\n beq.w .Lo_F6\n"   // INC dp,X
 *   "cmp r0, #0xFE\n beq.w .Lo_FE\n"   // INC abs,X
 *   "cmp r0, #0xC6\n beq.w .Lo_C6\n"   // DEC dp
 *   "cmp r0, #0xCE\n beq.w .Lo_CE\n"   // DEC abs
 *   "cmp r0, #0xD6\n beq.w .Lo_D6\n"   // DEC dp,X
 *   "cmp r0, #0xDE\n beq.w .Lo_DE\n"   // DEC abs,X
 *   "cmp r0, #0xE0\n beq.w .Lo_E0\n"   // CPX imm
 *   "cmp r0, #0xE4\n beq.w .Lo_E4\n"   // CPX dp
 *   "cmp r0, #0xEC\n beq.w .Lo_EC\n"   // CPX abs
 *   "cmp r0, #0xC0\n beq.w .Lo_C0\n"   // CPY imm
 *   "cmp r0, #0xC4\n beq.w .Lo_C4\n"   // CPY dp
 *   "cmp r0, #0xCC\n beq.w .Lo_CC\n"   // CPY abs
 */

/* ======================================================================
 * SBC — Subtract with Carry (binary mode only; decimal falls back to C)
 *
 * SBC: result = A + ~operand + C
 * 8-bit:  result = (A&0xFF) + (op^0xFF) + C
 *         C = result > 0xFF
 *         V = ((A^result) & (A^operand)) & 0x80   [note: A^operand, not ~]
 * 16-bit: same but 16-bit width
 *
 * V flag derivation for SBC (different sign from ADC):
 *   Overflow if A and operand have DIFFERENT signs, and result has operand's
 *   sign. Equivalently: V = ((A ^ result) & (A ^ operand)) >> 7
 *   (opposite of ADC which uses ~(A^op) & (A^res))
 * ====================================================================== */

/*
 * .Lsbc8 / .Lsbc16 — shared SBC ALU finish.
 * Entry:  R0 = operand (8 or 16-bit depending on MF)
 *         R4 = A (full 32-bit reg, low byte/word is A)
 *         R9 = packed flags
 * Exit:   R4 updated, R9 C/Z/V/N updated, R0-R3 R12 clobbered.
 */

/* ---- SBC imm (E9) ---- */
    ".Lo_E9:\n"
    "tst r9, #0x08\n bne.w .Lfb\n"          /* decimal -> fallback */
    "tst r9, #0x20\n beq.w .Le9_16\n"
    /* 8-bit immediate */
    "ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n"
    "add r8, r8, #1\n uxth r8, r8\n"
    "bl .Lint\n bl .Lrd\n"                  /* r0 = operand */
    "b .Lsbc8\n"
    ".Le9_16:\n"
    /* 16-bit immediate — fetch low then high byte */
    "ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n"
    "add r8, r8, #1\n uxth r8, r8\n bl .Lrd\n str r0, [sp, #" S(S0) "]\n"
    "ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n"
    "add r8, r8, #1\n uxth r8, r8\n bl .Lint\n bl .Lrd\n"
    "lsl r0, r0, #8\n ldr r1, [sp, #" S(S0) "]\n orr r0, r1, r0\n"
    "b .Lsbc16\n"

/* ---- SBC dp (E5) ---- */
    ".Lo_E5:\n"
    "tst r9, #0x08\n bne.w .Lfb\n"
    "bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
    "ldr r0, [sp, #" S(FDP) "]\n tst r0, #0xFF\n beq 1f\n bl .Lidl\n"
    "1: ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
    "add r1, r1, r0\n uxth r1, r1\n"
    "tst r9, #0x20\n beq.w .Le5_16\n"
    "bl .Lint\n bl .Lrd\n b .Lsbc8\n"
    ".Le5_16:\n"
    "bl .Lrd\n str r0, [sp, #" S(S1) "]\n"
    "ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
    "add r1, r1, r0\n adds r1, #1\n uxth r1, r1\n"
    "bl .Lint\n bl .Lrd\n"
    "lsl r0, r0, #8\n ldr r1, [sp, #" S(S1) "]\n orr r0, r1, r0\n"
    "b .Lsbc16\n"

/* ---- SBC abs (ED) ---- */
    ".Lo_ED:\n"
    "tst r9, #0x08\n bne.w .Lfb\n"
    "bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
    "bl .Lfetch\n ldr r1, [sp, #" S(S0) "]\n orr r1, r1, r0, lsl #8\n"
    "ldr r0, [sp, #" S(FDB) "]\n add r1, r1, r0, lsl #16\n"
    "tst r9, #0x20\n beq.w .Led_16\n"
    "bl .Lint\n bl .Lrd\n b .Lsbc8\n"
    ".Led_16:\n"
    "str r1, [sp, #" S(S0) "]\n bl .Lrd\n str r0, [sp, #" S(S1) "]\n"
    "ldr r1, [sp, #" S(S0) "]\n adds r1, #1\n bic r1, r1, #0xFF000000\n"
    "bl .Lint\n bl .Lrd\n"
    "lsl r0, r0, #8\n ldr r1, [sp, #" S(S1) "]\n orr r0, r1, r0\n"
    "b .Lsbc16\n"

/* ---- SBC dp,X (F5) ---- */
    ".Lo_F5:\n"
    "tst r9, #0x08\n bne.w .Lfb\n"
    "bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
    "ldr r0, [sp, #" S(FDP) "]\n tst r0, #0xFF\n beq 1f\n bl .Lidl\n"
    "1: bl .Lidl\n"                          /* indexed penalty */
    "ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
    "add r1, r1, r0\n add r1, r1, r5\n uxth r1, r1\n"
    "tst r9, #0x20\n beq.w .Lf5_16\n"
    "bl .Lint\n bl .Lrd\n b .Lsbc8\n"
    ".Lf5_16:\n"
    "bl .Lrd\n str r0, [sp, #" S(S1) "]\n"
    "ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
    "add r1, r1, r0\n add r1, r1, r5\n adds r1, #1\n uxth r1, r1\n"
    "bl .Lint\n bl .Lrd\n"
    "lsl r0, r0, #8\n ldr r1, [sp, #" S(S1) "]\n orr r0, r1, r0\n"
    "b .Lsbc16\n"

/* ---- SBC abs,Y (F9) / SBC abs,X (FD) ---- */
    ".Lo_F9:\n mov r12, r6\n b.w .Lsbcabxy\n"
    ".Lo_FD:\n mov r12, r5\n"
    ".Lsbcabxy:\n"
    "tst r9, #0x08\n bne.w .Lfb\n"
    "str r12, [sp, #" S(S2) "]\n"
    "bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
    "bl .Lfetch\n ldr r1, [sp, #" S(S0) "]\n orr r1, r1, r0, lsl #8\n"
    "str r1, [sp, #" S(S0) "]\n"
    /* page-cross idle — XF=0 means 16-bit index, always idle; XF=1 only if page crossed */
    "tst r9, #0x10\n beq 1f\n"              /* XF=0 -> idle */
    "ldr r0, [sp, #" S(S2) "]\n"
    "add r2, r1, r0\n lsr r2, r2, #8\n lsr r1, r1, #8\n cmp r1, r2\n beq 2f\n"
    "1: bl .Lidl\n"
    "2: ldr r0, [sp, #" S(FDB) "]\n lsl r0, r0, #16\n"
    "ldr r1, [sp, #" S(S0) "]\n add r0, r0, r1\n"
    "ldr r1, [sp, #" S(S2) "]\n add r0, r0, r1\n"
    "bic r1, r0, #0xFF000000\n"             /* r1 = ea */
    "tst r9, #0x20\n beq.w .Lsbcabxy16\n"
    "bl .Lint\n bl .Lrd\n b .Lsbc8\n"
    ".Lsbcabxy16:\n"
    "str r1, [sp, #" S(S0) "]\n bl .Lrd\n str r0, [sp, #" S(S1) "]\n"
    "ldr r1, [sp, #" S(S0) "]\n adds r1, #1\n bic r1, r1, #0xFF000000\n"
    "bl .Lint\n bl .Lrd\n"
    "lsl r0, r0, #8\n ldr r1, [sp, #" S(S1) "]\n orr r0, r1, r0\n"
    "b .Lsbc16\n"

/* ---- SBC ALU core (shared) ---- */
/*
 * .Lsbc8 — 8-bit SBC ALU finish.
 * Entry: R0 = 8-bit operand, R4 = A, R9 = flags
 * Clobbers: R1 R2 R3 R12
 */
    ".Lsbc8:\n"
    "and  r1, r4, #0xFF\n"                  /* r1 = A low byte */
    "str  r0, [sp, #" S(S3) "]\n"           /* S3 = original operand (for V) */
    "mvn  r2, r0\n uxtb r2, r2\n"          /* r2 = ~operand & 0xFF */
    "and  r3, r9, #1\n"                     /* r3 = C flag */
    "add  r0, r1, r2\n add r0, r0, r3\n"   /* r0 = A + ~op + C (result) */
    /* C = result > 0xFF */
    "bic  r9, r9, #0xC3\n"                 /* clear C,Z,V,N */
    "cmp  r0, #0x100\n blo 1f\n orr r9, r9, #0x01\n"
    /* V = ((A ^ result) & (A ^ operand)) & 0x80 */
    "1: ldr r2, [sp, #" S(S3) "]\n"        /* r2 = original operand */
    "eor  r3, r1, r0\n"                    /* A ^ result */
    "eor  r12, r1, r2\n"                   /* A ^ operand */
    "and  r3, r3, r12\n"
    "tst  r3, #0x80\n beq 1f\n orr r9, r9, #0x40\n"
    "1: tst r0, #0xFF\n  bne 1f\n orr r9, r9, #0x02\n"
    "1: tst r0, #0x80\n  beq 1f\n orr r9, r9, #0x80\n"
    "1: bfi r4, r0, #0, #8\n b.w .Lnext\n"

/*
 * .Lsbc16 — 16-bit SBC ALU finish.
 * Entry: R0 = 16-bit operand, R4 = A, R9 = flags
 * Clobbers: R1 R2 R3 R12
 */
    ".Lsbc16:\n"
    "uxth r1, r4\n"                         /* r1 = A low 16 bits */
    "str  r0, [sp, #" S(S3) "]\n"           /* S3 = original operand */
    "movw r2, #0xFFFF\n eor r2, r0, r2\n"  /* r2 = ~operand & 0xFFFF */
    "and  r3, r9, #1\n"                     /* r3 = C */
    "add  r0, r1, r2\n add r0, r0, r3\n"   /* r0 = result */
    /* C = result > 0xFFFF */
    "bic  r9, r9, #0xC3\n"
    "movw r2, #0\n movt r2, #1\n cmp r0, r2\n blo 1f\n orr r9, r9, #0x01\n"
    /* V = ((A ^ result) & (A ^ operand)) & 0x8000 */
    "1: ldr r2, [sp, #" S(S3) "]\n"
    "eor  r3, r1, r0\n"
    "eor  r12, r1, r2\n"
    "and  r3, r3, r12\n"
    "tst  r3, #0x8000\n beq 1f\n orr r9, r9, #0x40\n"
    "1: uxth r2, r0\n cbnz r2, 1f\n orr r9, r9, #0x02\n"
    "1: tst r0, #0x8000\n beq 1f\n orr r9, r9, #0x80\n"
    "1: uxth r4, r0\n b.w .Lnext\n"

/* ======================================================================
 * ASL A (0A) — Arithmetic Shift Left, accumulator
 * Implied: idle cycle, then operate.
 * Old bit 7 (or 15) -> C.  Result -> A.  Z/N set.
 * ====================================================================== */
    ".Lo_0A:\n"
    "bl .Lint\n bl .Lidl\n"
    "tst r9, #0x20\n beq.w .L0a_16\n"
    /* 8-bit */
    "uxtb r0, r4\n"
    "lsr  r1, r0, #7\n bfi r9, r1, #0, #1\n"   /* old bit7 -> C */
    "lsl  r0, r0, #1\n uxtb r0, r0\n"
    "bfi  r4, r0, #0, #8\n bl .Lzn8\n b.w .Lnext\n"
    ".L0a_16:\n"
    "uxth r0, r4\n"
    "lsr  r1, r0, #15\n bfi r9, r1, #0, #1\n"
    "lsl  r0, r0, #1\n uxth r4, r0\n"
    "mov  r0, r4\n bl .Lzn16\n b.w .Lnext\n"

/* ======================================================================
 * LSR A (4A) — Logical Shift Right, accumulator
 * Old bit 0 -> C.
 * ====================================================================== */
    ".Lo_4A:\n"
    "bl .Lint\n bl .Lidl\n"
    "tst r9, #0x20\n beq.w .L4a_16\n"
    /* 8-bit */
    "uxtb r0, r4\n"
    "bfi  r9, r0, #0, #1\n"                /* bit0 -> C */
    "lsr  r0, r0, #1\n"
    "bfi  r4, r0, #0, #8\n bl .Lzn8\n b.w .Lnext\n"
    ".L4a_16:\n"
    "uxth r0, r4\n"
    "bfi  r9, r0, #0, #1\n"
    "lsr  r0, r0, #1\n uxth r4, r0\n"
    "mov  r0, r4\n bl .Lzn16\n b.w .Lnext\n"

/* ======================================================================
 * ROL A (2A) — Rotate Left through Carry, accumulator
 * Old bit 7 (15) -> new C.  Old C -> new bit 0.
 * ====================================================================== */
    ".Lo_2A:\n"
    "bl .Lint\n bl .Lidl\n"
    "tst r9, #0x20\n beq.w .L2a_16\n"
    /* 8-bit */
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

/* ======================================================================
 * ROR A (6A) — Rotate Right through Carry, accumulator
 * Old bit 0 -> new C.  Old C -> new bit 7 (15).
 * ====================================================================== */
    ".Lo_6A:\n"
    "bl .Lint\n bl .Lidl\n"
    "tst r9, #0x20\n beq.w .L6a_16\n"
    /* 8-bit */
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

/* ======================================================================
 * READ-MODIFY-WRITE shift/rotate on MEMORY
 *
 * Pattern: compute ea, read (low), [read high if 16-bit], idle (modify cycle),
 *          operate, write [high then] low.
 * Addressing: dp, abs, dp,X, abs,X.
 *
 * Shared ALU kernels .Lasl_rmw / .Llsr_rmw / .Lrol_rmw / .Lror_rmw:
 *   Entry:  R0 = value (8 or 16-bit; caller selects based on MF)
 *           R9 = flags
 *   Exit:   R0 = modified value, R9 C updated.
 *
 * RMW timing per 65816:
 *   dp:    fetch dp, [dp-idle if DPL!=0], read, idle, write      (5/6 cycles 8-bit; +2 for 16)
 *   abs:   fetch lo, fetch hi, read, idle, write                  (6 cycles 8-bit; +2 for 16)
 *   dp,X:  fetch dp, [dp-idle], idle(index), read, idle, write   (6/7 cycles; +2 for 16)
 *   abs,X: fetch lo, fetch hi, idle(index), read, idle, write    (7 cycles; +2 for 16)
 *
 * Convention here: idle for the "modify" cycle done AFTER read, BEFORE write.
 * ====================================================================== */

/* --- Shared RMW ALU kernels --- */

/* .Lasl_rmw8 / .Lasl_rmw16 */
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

/*
 * Shared RMW read-operate-write sequences.
 * Entry: R1 = effective address (ea_lo).
 *        S0 = ea (saved for 16-bit high byte).
 *
 * We use a trampoline pattern: the dispatch opcode branches to a stub that
 * stores the ALU kernel address in S4, then falls into a common RMW body.
 *
 * .Lrmw8_body / .Lrmw16_body:
 *   R1 = ea_lo
 *   S4 = ALU fn pointer (kernel above; leaf, uses bx lr)
 */
    ".Lrmw8_body:\n"
    /* read */
    "bl .Lrd\n"                            /* r0 = value */
    "bl .Lidl\n"                           /* modify cycle */
    /* operate — call ALU kernel (leaf) */
    "str lr, [sp, #" S(FLR) "]\n"
    "ldr r3, [sp, #" S(S4) "]\n blx r3\n"
    "ldr lr, [sp, #" S(FLR) "]\n"
    /* r0 = result; write back */
    "ldr r1, [sp, #" S(S0) "]\n"
    "mov r2, r0\n bl .Lwr\n"
    "bl .Lzn8\n b.w .Lnext\n"

    ".Lrmw16_body:\n"
    /* read low byte */
    "bl .Lrd\n str r0, [sp, #" S(S1) "]\n"
    /* read high byte */
    "ldr r1, [sp, #" S(S0) "]\n adds r1, #1\n bic r1, r1, #0xFF000000\n"
    "bl .Lrd\n"
    "lsl r0, r0, #8\n ldr r1, [sp, #" S(S1) "]\n orr r0, r1, r0\n"  /* 16-bit value */
    "bl .Lidl\n"                           /* modify cycle */
    /* operate */
    "str lr, [sp, #" S(FLR) "]\n"
    "ldr r3, [sp, #" S(S4) "]\n blx r3\n"
    "ldr lr, [sp, #" S(FLR) "]\n"
    /* write high byte first, then low */
    "str r0, [sp, #" S(S1) "]\n"
    "ldr r1, [sp, #" S(S0) "]\n adds r1, #1\n bic r1, r1, #0xFF000000\n"
    "lsr r2, r0, #8\n uxtb r2, r2\n bl .Lwr\n"
    "ldr r1, [sp, #" S(S0) "]\n"
    "ldr r0, [sp, #" S(S1) "]\n uxtb r2, r0\n bl .Lwr\n"
    "ldr r0, [sp, #" S(S1) "]\n"
    "bl .Lzn16\n b.w .Lnext\n"

/* ====================================================================
 * Address computation helpers (set R1=ea_lo, save in S0) then branch
 * to RMW body.  Each opcode needs: compute ea -> S0/R1, pick kernel -> S4,
 * test MF, jump to rmw8_body or rmw16_body.
 *
 * Macro-like pattern used for each:
 *   1. Compute ea, store in S0, put in R1.
 *   2. Store ALU kernel ptr in S4.
 *   3. tst r9, #0x20; beq -> rmw16_body; else -> rmw8_body.
 * ==================================================================== */

/* Helper macros emulated via labels for each addressing mode x operation */

/* ---- ASL dp (06) ---- */
    ".Lo_06:\n"
    "bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
    "ldr r0, [sp, #" S(FDP) "]\n tst r0, #0xFF\n beq 1f\n bl .Lidl\n"
    "1: ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
    "add r1, r1, r0\n uxth r1, r1\n str r1, [sp, #" S(S0) "]\n"
    "tst r9, #0x20\n beq.w 2f\n"
    "ldr r3, =.Lasl_rmw8\n str r3, [sp, #" S(S4) "]\n b .Lrmw8_body\n"
    "2: ldr r3, =.Lasl_rmw16\n str r3, [sp, #" S(S4) "]\n b .Lrmw16_body\n"

/* ---- ASL abs (0E) ---- */
    ".Lo_0E:\n"
    "bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
    "bl .Lfetch\n ldr r1, [sp, #" S(S0) "]\n orr r1, r1, r0, lsl #8\n"
    "ldr r0, [sp, #" S(FDB) "]\n add r1, r1, r0, lsl #16\n"
    "bic r1, r1, #0xFF000000\n str r1, [sp, #" S(S0) "]\n"
    "tst r9, #0x20\n beq.w 2f\n"
    "ldr r3, =.Lasl_rmw8\n str r3, [sp, #" S(S4) "]\n b .Lrmw8_body\n"
    "2: ldr r3, =.Lasl_rmw16\n str r3, [sp, #" S(S4) "]\n b .Lrmw16_body\n"

/* ---- ASL dp,X (16) ---- */
    ".Lo_16:\n"
    "bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
    "ldr r0, [sp, #" S(FDP) "]\n tst r0, #0xFF\n beq 1f\n bl .Lidl\n"
    "1: bl .Lidl\n"
    "ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
    "add r1, r1, r0\n add r1, r1, r5\n uxth r1, r1\n str r1, [sp, #" S(S0) "]\n"
    "tst r9, #0x20\n beq.w 2f\n"
    "ldr r3, =.Lasl_rmw8\n str r3, [sp, #" S(S4) "]\n b .Lrmw8_body\n"
    "2: ldr r3, =.Lasl_rmw16\n str r3, [sp, #" S(S4) "]\n b .Lrmw16_body\n"

/* ---- ASL abs,X (1E) ---- */
    ".Lo_1E:\n"
    "bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
    "bl .Lfetch\n ldr r1, [sp, #" S(S0) "]\n orr r1, r1, r0, lsl #8\n"
    "str r1, [sp, #" S(S0) "]\n"
    "bl .Lidl\n"                           /* index penalty (always for write) */
    "ldr r0, [sp, #" S(FDB) "]\n lsl r0, r0, #16\n"
    "ldr r1, [sp, #" S(S0) "]\n add r0, r0, r1\n add r0, r0, r5\n"
    "bic r1, r0, #0xFF000000\n str r1, [sp, #" S(S0) "]\n"
    "tst r9, #0x20\n beq.w 2f\n"
    "ldr r3, =.Lasl_rmw8\n str r3, [sp, #" S(S4) "]\n b .Lrmw8_body\n"
    "2: ldr r3, =.Lasl_rmw16\n str r3, [sp, #" S(S4) "]\n b .Lrmw16_body\n"

/* ---- LSR dp (46) ---- */
    ".Lo_46:\n"
    "bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
    "ldr r0, [sp, #" S(FDP) "]\n tst r0, #0xFF\n beq 1f\n bl .Lidl\n"
    "1: ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
    "add r1, r1, r0\n uxth r1, r1\n str r1, [sp, #" S(S0) "]\n"
    "tst r9, #0x20\n beq.w 2f\n"
    "ldr r3, =.Llsr_rmw8\n str r3, [sp, #" S(S4) "]\n b .Lrmw8_body\n"
    "2: ldr r3, =.Llsr_rmw16\n str r3, [sp, #" S(S4) "]\n b .Lrmw16_body\n"

/* ---- LSR abs (4E) ---- */
    ".Lo_4E:\n"
    "bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
    "bl .Lfetch\n ldr r1, [sp, #" S(S0) "]\n orr r1, r1, r0, lsl #8\n"
    "ldr r0, [sp, #" S(FDB) "]\n add r1, r1, r0, lsl #16\n"
    "bic r1, r1, #0xFF000000\n str r1, [sp, #" S(S0) "]\n"
    "tst r9, #0x20\n beq.w 2f\n"
    "ldr r3, =.Llsr_rmw8\n str r3, [sp, #" S(S4) "]\n b .Lrmw8_body\n"
    "2: ldr r3, =.Llsr_rmw16\n str r3, [sp, #" S(S4) "]\n b .Lrmw16_body\n"

/* ---- LSR dp,X (56) ---- */
    ".Lo_56:\n"
    "bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
    "ldr r0, [sp, #" S(FDP) "]\n tst r0, #0xFF\n beq 1f\n bl .Lidl\n"
    "1: bl .Lidl\n"
    "ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
    "add r1, r1, r0\n add r1, r1, r5\n uxth r1, r1\n str r1, [sp, #" S(S0) "]\n"
    "tst r9, #0x20\n beq.w 2f\n"
    "ldr r3, =.Llsr_rmw8\n str r3, [sp, #" S(S4) "]\n b .Lrmw8_body\n"
    "2: ldr r3, =.Llsr_rmw16\n str r3, [sp, #" S(S4) "]\n b .Lrmw16_body\n"

/* ---- LSR abs,X (5E) ---- */
    ".Lo_5E:\n"
    "bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
    "bl .Lfetch\n ldr r1, [sp, #" S(S0) "]\n orr r1, r1, r0, lsl #8\n"
    "str r1, [sp, #" S(S0) "]\n"
    "bl .Lidl\n"
    "ldr r0, [sp, #" S(FDB) "]\n lsl r0, r0, #16\n"
    "ldr r1, [sp, #" S(S0) "]\n add r0, r0, r1\n add r0, r0, r5\n"
    "bic r1, r0, #0xFF000000\n str r1, [sp, #" S(S0) "]\n"
    "tst r9, #0x20\n beq.w 2f\n"
    "ldr r3, =.Llsr_rmw8\n str r3, [sp, #" S(S4) "]\n b .Lrmw8_body\n"
    "2: ldr r3, =.Llsr_rmw16\n str r3, [sp, #" S(S4) "]\n b .Lrmw16_body\n"

/* ---- ROL dp (26) ---- */
    ".Lo_26:\n"
    "bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
    "ldr r0, [sp, #" S(FDP) "]\n tst r0, #0xFF\n beq 1f\n bl .Lidl\n"
    "1: ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
    "add r1, r1, r0\n uxth r1, r1\n str r1, [sp, #" S(S0) "]\n"
    "tst r9, #0x20\n beq.w 2f\n"
    "ldr r3, =.Lrol_rmw8\n str r3, [sp, #" S(S4) "]\n b .Lrmw8_body\n"
    "2: ldr r3, =.Lrol_rmw16\n str r3, [sp, #" S(S4) "]\n b .Lrmw16_body\n"

/* ---- ROL abs (2E) ---- */
    ".Lo_2E:\n"
    "bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
    "bl .Lfetch\n ldr r1, [sp, #" S(S0) "]\n orr r1, r1, r0, lsl #8\n"
    "ldr r0, [sp, #" S(FDB) "]\n add r1, r1, r0, lsl #16\n"
    "bic r1, r1, #0xFF000000\n str r1, [sp, #" S(S0) "]\n"
    "tst r9, #0x20\n beq.w 2f\n"
    "ldr r3, =.Lrol_rmw8\n str r3, [sp, #" S(S4) "]\n b .Lrmw8_body\n"
    "2: ldr r3, =.Lrol_rmw16\n str r3, [sp, #" S(S4) "]\n b .Lrmw16_body\n"

/* ---- ROL dp,X (36) ---- */
    ".Lo_36:\n"
    "bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
    "ldr r0, [sp, #" S(FDP) "]\n tst r0, #0xFF\n beq 1f\n bl .Lidl\n"
    "1: bl .Lidl\n"
    "ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
    "add r1, r1, r0\n add r1, r1, r5\n uxth r1, r1\n str r1, [sp, #" S(S0) "]\n"
    "tst r9, #0x20\n beq.w 2f\n"
    "ldr r3, =.Lrol_rmw8\n str r3, [sp, #" S(S4) "]\n b .Lrmw8_body\n"
    "2: ldr r3, =.Lrol_rmw16\n str r3, [sp, #" S(S4) "]\n b .Lrmw16_body\n"

/* ---- ROL abs,X (3E) ---- */
    ".Lo_3E:\n"
    "bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
    "bl .Lfetch\n ldr r1, [sp, #" S(S0) "]\n orr r1, r1, r0, lsl #8\n"
    "str r1, [sp, #" S(S0) "]\n"
    "bl .Lidl\n"
    "ldr r0, [sp, #" S(FDB) "]\n lsl r0, r0, #16\n"
    "ldr r1, [sp, #" S(S0) "]\n add r0, r0, r1\n add r0, r0, r5\n"
    "bic r1, r0, #0xFF000000\n str r1, [sp, #" S(S0) "]\n"
    "tst r9, #0x20\n beq.w 2f\n"
    "ldr r3, =.Lrol_rmw8\n str r3, [sp, #" S(S4) "]\n b .Lrmw8_body\n"
    "2: ldr r3, =.Lrol_rmw16\n str r3, [sp, #" S(S4) "]\n b .Lrmw16_body\n"

/* ---- ROR dp (66) ---- */
    ".Lo_66:\n"
    "bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
    "ldr r0, [sp, #" S(FDP) "]\n tst r0, #0xFF\n beq 1f\n bl .Lidl\n"
    "1: ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
    "add r1, r1, r0\n uxth r1, r1\n str r1, [sp, #" S(S0) "]\n"
    "tst r9, #0x20\n beq.w 2f\n"
    "ldr r3, =.Lror_rmw8\n str r3, [sp, #" S(S4) "]\n b .Lrmw8_body\n"
    "2: ldr r3, =.Lror_rmw16\n str r3, [sp, #" S(S4) "]\n b .Lrmw16_body\n"

/* ---- ROR abs (6E) ---- */
    ".Lo_6E:\n"
    "bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
    "bl .Lfetch\n ldr r1, [sp, #" S(S0) "]\n orr r1, r1, r0, lsl #8\n"
    "ldr r0, [sp, #" S(FDB) "]\n add r1, r1, r0, lsl #16\n"
    "bic r1, r1, #0xFF000000\n str r1, [sp, #" S(S0) "]\n"
    "tst r9, #0x20\n beq.w 2f\n"
    "ldr r3, =.Lror_rmw8\n str r3, [sp, #" S(S4) "]\n b .Lrmw8_body\n"
    "2: ldr r3, =.Lror_rmw16\n str r3, [sp, #" S(S4) "]\n b .Lrmw16_body\n"

/* ---- ROR dp,X (76) ---- */
    ".Lo_76:\n"
    "bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
    "ldr r0, [sp, #" S(FDP) "]\n tst r0, #0xFF\n beq 1f\n bl .Lidl\n"
    "1: bl .Lidl\n"
    "ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
    "add r1, r1, r0\n add r1, r1, r5\n uxth r1, r1\n str r1, [sp, #" S(S0) "]\n"
    "tst r9, #0x20\n beq.w 2f\n"
    "ldr r3, =.Lror_rmw8\n str r3, [sp, #" S(S4) "]\n b .Lrmw8_body\n"
    "2: ldr r3, =.Lror_rmw16\n str r3, [sp, #" S(S4) "]\n b .Lrmw16_body\n"

/* ---- ROR abs,X (7E) ---- */
    ".Lo_7E:\n"
    "bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
    "bl .Lfetch\n ldr r1, [sp, #" S(S0) "]\n orr r1, r1, r0, lsl #8\n"
    "str r1, [sp, #" S(S0) "]\n"
    "bl .Lidl\n"
    "ldr r0, [sp, #" S(FDB) "]\n lsl r0, r0, #16\n"
    "ldr r1, [sp, #" S(S0) "]\n add r0, r0, r1\n add r0, r0, r5\n"
    "bic r1, r0, #0xFF000000\n str r1, [sp, #" S(S0) "]\n"
    "tst r9, #0x20\n beq.w 2f\n"
    "ldr r3, =.Lror_rmw8\n str r3, [sp, #" S(S4) "]\n b .Lrmw8_body\n"
    "2: ldr r3, =.Lror_rmw16\n str r3, [sp, #" S(S4) "]\n b .Lrmw16_body\n"

/* ======================================================================
 * INC / DEC memory — read-modify-write, no C flag change.
 *
 * Shared ALU kernels .Linc_rmw8 / .Linc_rmw16 / .Ldec_rmw8 / .Ldec_rmw16
 * are leaf functions (bx lr).  They only set Z/N via the common RMW body
 * (which calls .Lzn8 / .Lzn16 after the operation).
 * ====================================================================== */

    ".Linc_rmw8:\n"
    "uxtb r0, r0\n add r0, r0, #1\n uxtb r0, r0\n bx lr\n"

    ".Linc_rmw16:\n"
    "uxth r0, r0\n add r0, r0, #1\n uxth r0, r0\n bx lr\n"

    ".Ldec_rmw8:\n"
    "uxtb r0, r0\n sub r0, r0, #1\n uxtb r0, r0\n bx lr\n"

    ".Ldec_rmw16:\n"
    "uxth r0, r0\n sub r0, r0, #1\n uxth r0, r0\n bx lr\n"

/* ---- INC dp (E6) ---- */
    ".Lo_E6:\n"
    "bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
    "ldr r0, [sp, #" S(FDP) "]\n tst r0, #0xFF\n beq 1f\n bl .Lidl\n"
    "1: ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
    "add r1, r1, r0\n uxth r1, r1\n str r1, [sp, #" S(S0) "]\n"
    "tst r9, #0x20\n beq.w 2f\n"
    "ldr r3, =.Linc_rmw8\n str r3, [sp, #" S(S4) "]\n b .Lrmw8_body\n"
    "2: ldr r3, =.Linc_rmw16\n str r3, [sp, #" S(S4) "]\n b .Lrmw16_body\n"

/* ---- INC abs (EE) ---- */
    ".Lo_EE:\n"
    "bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
    "bl .Lfetch\n ldr r1, [sp, #" S(S0) "]\n orr r1, r1, r0, lsl #8\n"
    "ldr r0, [sp, #" S(FDB) "]\n add r1, r1, r0, lsl #16\n"
    "bic r1, r1, #0xFF000000\n str r1, [sp, #" S(S0) "]\n"
    "tst r9, #0x20\n beq.w 2f\n"
    "ldr r3, =.Linc_rmw8\n str r3, [sp, #" S(S4) "]\n b .Lrmw8_body\n"
    "2: ldr r3, =.Linc_rmw16\n str r3, [sp, #" S(S4) "]\n b .Lrmw16_body\n"

/* ---- INC dp,X (F6) ---- */
    ".Lo_F6:\n"
    "bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
    "ldr r0, [sp, #" S(FDP) "]\n tst r0, #0xFF\n beq 1f\n bl .Lidl\n"
    "1: bl .Lidl\n"
    "ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
    "add r1, r1, r0\n add r1, r1, r5\n uxth r1, r1\n str r1, [sp, #" S(S0) "]\n"
    "tst r9, #0x20\n beq.w 2f\n"
    "ldr r3, =.Linc_rmw8\n str r3, [sp, #" S(S4) "]\n b .Lrmw8_body\n"
    "2: ldr r3, =.Linc_rmw16\n str r3, [sp, #" S(S4) "]\n b .Lrmw16_body\n"

/* ---- INC abs,X (FE) ---- */
    ".Lo_FE:\n"
    "bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
    "bl .Lfetch\n ldr r1, [sp, #" S(S0) "]\n orr r1, r1, r0, lsl #8\n"
    "str r1, [sp, #" S(S0) "]\n"
    "bl .Lidl\n"
    "ldr r0, [sp, #" S(FDB) "]\n lsl r0, r0, #16\n"
    "ldr r1, [sp, #" S(S0) "]\n add r0, r0, r1\n add r0, r0, r5\n"
    "bic r1, r0, #0xFF000000\n str r1, [sp, #" S(S0) "]\n"
    "tst r9, #0x20\n beq.w 2f\n"
    "ldr r3, =.Linc_rmw8\n str r3, [sp, #" S(S4) "]\n b .Lrmw8_body\n"
    "2: ldr r3, =.Linc_rmw16\n str r3, [sp, #" S(S4) "]\n b .Lrmw16_body\n"

/* ---- DEC dp (C6) ---- */
    ".Lo_C6:\n"
    "bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
    "ldr r0, [sp, #" S(FDP) "]\n tst r0, #0xFF\n beq 1f\n bl .Lidl\n"
    "1: ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
    "add r1, r1, r0\n uxth r1, r1\n str r1, [sp, #" S(S0) "]\n"
    "tst r9, #0x20\n beq.w 2f\n"
    "ldr r3, =.Ldec_rmw8\n str r3, [sp, #" S(S4) "]\n b .Lrmw8_body\n"
    "2: ldr r3, =.Ldec_rmw16\n str r3, [sp, #" S(S4) "]\n b .Lrmw16_body\n"

/* ---- DEC abs (CE) ---- */
    ".Lo_CE:\n"
    "bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
    "bl .Lfetch\n ldr r1, [sp, #" S(S0) "]\n orr r1, r1, r0, lsl #8\n"
    "ldr r0, [sp, #" S(FDB) "]\n add r1, r1, r0, lsl #16\n"
    "bic r1, r1, #0xFF000000\n str r1, [sp, #" S(S0) "]\n"
    "tst r9, #0x20\n beq.w 2f\n"
    "ldr r3, =.Ldec_rmw8\n str r3, [sp, #" S(S4) "]\n b .Lrmw8_body\n"
    "2: ldr r3, =.Ldec_rmw16\n str r3, [sp, #" S(S4) "]\n b .Lrmw16_body\n"

/* ---- DEC dp,X (D6) ---- */
    ".Lo_D6:\n"
    "bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
    "ldr r0, [sp, #" S(FDP) "]\n tst r0, #0xFF\n beq 1f\n bl .Lidl\n"
    "1: bl .Lidl\n"
    "ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
    "add r1, r1, r0\n add r1, r1, r5\n uxth r1, r1\n str r1, [sp, #" S(S0) "]\n"
    "tst r9, #0x20\n beq.w 2f\n"
    "ldr r3, =.Ldec_rmw8\n str r3, [sp, #" S(S4) "]\n b .Lrmw8_body\n"
    "2: ldr r3, =.Ldec_rmw16\n str r3, [sp, #" S(S4) "]\n b .Lrmw16_body\n"

/* ---- DEC abs,X (DE) ---- */
    ".Lo_DE:\n"
    "bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
    "bl .Lfetch\n ldr r1, [sp, #" S(S0) "]\n orr r1, r1, r0, lsl #8\n"
    "str r1, [sp, #" S(S0) "]\n"
    "bl .Lidl\n"
    "ldr r0, [sp, #" S(FDB) "]\n lsl r0, r0, #16\n"
    "ldr r1, [sp, #" S(S0) "]\n add r0, r0, r1\n add r0, r0, r5\n"
    "bic r1, r0, #0xFF000000\n str r1, [sp, #" S(S0) "]\n"
    "tst r9, #0x20\n beq.w 2f\n"
    "ldr r3, =.Ldec_rmw8\n str r3, [sp, #" S(S4) "]\n b .Lrmw8_body\n"
    "2: ldr r3, =.Ldec_rmw16\n str r3, [sp, #" S(S4) "]\n b .Lrmw16_body\n"

/* ======================================================================
 * CPX / CPY — Compare Index Register with Memory
 *
 * Uses XF (bit4) for 8/16-bit selection (NOT MF/bit5 like CMP/SBC).
 * CMP finish: result = reg + ~operand + 1 (i.e. reg - operand, no borrow)
 *   C = result >= operand (carry out), Z = equal, N = result bit7/15.
 *   No V flag update (compare never sets V).
 *
 * Shared finish: .Lcmpx8 / .Lcmpx16
 *   Entry: R0 = operand, R1 = index register value (R5 or R6)
 * ====================================================================== */

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
    "1: uxth r2, r0\n cbnz r2, 1f\n orr r9, r9, #0x02\n"
    "1: tst r0, #0x8000\n beq 1f\n orr r9, r9, #0x80\n"
    "1: b.w .Lnext\n"

/* ---- CPX imm (E0) ---- */
    ".Lo_E0:\n"
    "tst r9, #0x10\n beq.w .Le0_16\n"
    /* 8-bit */
    "ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n"
    "add r8, r8, #1\n uxth r8, r8\n"
    "bl .Lint\n bl .Lrd\n mov r1, r5\n b .Lcmpx8\n"
    ".Le0_16:\n"
    "ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n"
    "add r8, r8, #1\n uxth r8, r8\n bl .Lrd\n str r0, [sp, #" S(S0) "]\n"
    "ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n"
    "add r8, r8, #1\n uxth r8, r8\n bl .Lint\n bl .Lrd\n"
    "lsl r0, r0, #8\n ldr r1, [sp, #" S(S0) "]\n orr r0, r1, r0\n"
    "mov r1, r5\n b .Lcmpx16\n"

/* ---- CPX dp (E4) ---- */
    ".Lo_E4:\n"
    "bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
    "ldr r0, [sp, #" S(FDP) "]\n tst r0, #0xFF\n beq 1f\n bl .Lidl\n"
    "1: ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
    "add r1, r1, r0\n uxth r1, r1\n"
    "tst r9, #0x10\n beq.w .Le4_16\n"
    "bl .Lint\n bl .Lrd\n mov r1, r5\n b .Lcmpx8\n"
    ".Le4_16:\n"
    "bl .Lrd\n str r0, [sp, #" S(S1) "]\n"
    "ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
    "add r1, r1, r0\n adds r1, #1\n uxth r1, r1\n"
    "bl .Lint\n bl .Lrd\n"
    "lsl r0, r0, #8\n ldr r1, [sp, #" S(S1) "]\n orr r0, r1, r0\n"
    "mov r1, r5\n b .Lcmpx16\n"

/* ---- CPX abs (EC) ---- */
    ".Lo_EC:\n"
    "bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
    "bl .Lfetch\n ldr r1, [sp, #" S(S0) "]\n orr r1, r1, r0, lsl #8\n"
    "ldr r0, [sp, #" S(FDB) "]\n add r1, r1, r0, lsl #16\n"
    "tst r9, #0x10\n beq.w .Lec_16\n"
    "bl .Lint\n bl .Lrd\n mov r1, r5\n b .Lcmpx8\n"
    ".Lec_16:\n"
    "str r1, [sp, #" S(S0) "]\n bl .Lrd\n str r0, [sp, #" S(S1) "]\n"
    "ldr r1, [sp, #" S(S0) "]\n adds r1, #1\n bic r1, r1, #0xFF000000\n"
    "bl .Lint\n bl .Lrd\n"
    "lsl r0, r0, #8\n ldr r1, [sp, #" S(S1) "]\n orr r0, r1, r0\n"
    "mov r1, r5\n b .Lcmpx16\n"

/* ---- CPY imm (C0) ---- */
    ".Lo_C0:\n"
    "tst r9, #0x10\n beq.w .Lc0_16\n"
    "ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n"
    "add r8, r8, #1\n uxth r8, r8\n"
    "bl .Lint\n bl .Lrd\n mov r1, r6\n b .Lcmpx8\n"
    ".Lc0_16:\n"
    "ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n"
    "add r8, r8, #1\n uxth r8, r8\n bl .Lrd\n str r0, [sp, #" S(S0) "]\n"
    "ldr r1, [sp, #" S(FK) "]\n lsl r1, r1, #16\n orr r1, r1, r8\n"
    "add r8, r8, #1\n uxth r8, r8\n bl .Lint\n bl .Lrd\n"
    "lsl r0, r0, #8\n ldr r1, [sp, #" S(S0) "]\n orr r0, r1, r0\n"
    "mov r1, r6\n b .Lcmpx16\n"

/* ---- CPY dp (C4) ---- */
    ".Lo_C4:\n"
    "bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
    "ldr r0, [sp, #" S(FDP) "]\n tst r0, #0xFF\n beq 1f\n bl .Lidl\n"
    "1: ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
    "add r1, r1, r0\n uxth r1, r1\n"
    "tst r9, #0x10\n beq.w .Lc4_16\n"
    "bl .Lint\n bl .Lrd\n mov r1, r6\n b .Lcmpx8\n"
    ".Lc4_16:\n"
    "bl .Lrd\n str r0, [sp, #" S(S1) "]\n"
    "ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
    "add r1, r1, r0\n adds r1, #1\n uxth r1, r1\n"
    "bl .Lint\n bl .Lrd\n"
    "lsl r0, r0, #8\n ldr r1, [sp, #" S(S1) "]\n orr r0, r1, r0\n"
    "mov r1, r6\n b .Lcmpx16\n"

/* ---- CPY abs (CC) ---- */
    ".Lo_CC:\n"
    "bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
    "bl .Lfetch\n ldr r1, [sp, #" S(S0) "]\n orr r1, r1, r0, lsl #8\n"
    "ldr r0, [sp, #" S(FDB) "]\n add r1, r1, r0, lsl #16\n"
    "tst r9, #0x10\n beq.w .Lcc_16\n"
    "bl .Lint\n bl .Lrd\n mov r1, r6\n b .Lcmpx8\n"
    ".Lcc_16:\n"
    "str r1, [sp, #" S(S0) "]\n bl .Lrd\n str r0, [sp, #" S(S1) "]\n"
    "ldr r1, [sp, #" S(S0) "]\n adds r1, #1\n bic r1, r1, #0xFF000000\n"
    "bl .Lint\n bl .Lrd\n"
    "lsl r0, r0, #8\n ldr r1, [sp, #" S(S1) "]\n orr r0, r1, r0\n"
    "mov r1, r6\n b .Lcmpx16\n"
