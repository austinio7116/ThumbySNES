/*
 * cpu_asm_groupD.inc — Group D opcode implementations.
 *
 * Interrupts: BRK (00), COP (02), RTI (40)
 * Block moves: MVP (44), MVN (54)
 * Indirect addressing ALU:
 *   idx (dp,X):  01,21,41,61,81,A1,C1,E1
 *   idy (dp),Y:  11,31,51,71,91,B1,D1,F1
 *   idp (dp):    12,32,52,72,92,B2,D2,F2
 *   idl [dp]:    07,27,47,67,87,A7,C7,E7
 *   ily [dp],Y:  17,37,57,77,97,B7,D7,F7
 *   sr  d,S:     03,23,43,63,83,A3,C3,E3
 *   isy (d,S),Y: 13,33,53,73,93,B3,D3,F3
 *   abl long:    0F,2F,4F,6F,8F,AF,CF,EF
 *   alx long,X:  1F,3F,5F,7F,9F,BF,DF,FF
 * Jumps/branches: BRA(80), BRL(82), BVC(50), BVS(70),
 *   JMP abs(4C), JMP ind(6C), JMP iax(7C), JML abl(5C), JML ial(DC),
 *   JSL abl(22), JSR iax(FC), RTL(6B)
 * Stack: PEA(F4), PEI(D4), PER(62)
 * Flag ops: REP(C2), SEP(E2)
 * TSB/TRB: TSB dp(04), TSB abs(0C), TRB dp(14), TRB abs(1C)
 * Transfers: TCS(1B), TSC(3B), TCD(5B), TDC(7B)
 * WAI(CB), STP(DB), WDM(42)
 *
 * INSERT DISPATCH ENTRIES into the dispatch chain in cpu_asm.c, before .Lfb:
 *
 *   "cmp r0, #0x00\n beq.w .Lo_00\n"
 *   "cmp r0, #0x02\n beq.w .Lo_02\n"
 *   "cmp r0, #0x40\n beq.w .Lo_40\n"
 *   "cmp r0, #0x44\n beq.w .Lo_44\n"
 *   "cmp r0, #0x54\n beq.w .Lo_54\n"
 *   "cmp r0, #0x01\n beq.w .Lo_01\n"
 *   "cmp r0, #0x21\n beq.w .Lo_21\n"
 *   "cmp r0, #0x41\n beq.w .Lo_41\n"
 *   "cmp r0, #0x61\n beq.w .Lo_61\n"
 *   "cmp r0, #0x81\n beq.w .Lo_81\n"
 *   "cmp r0, #0xA1\n beq.w .Lo_A1\n"
 *   "cmp r0, #0xC1\n beq.w .Lo_C1\n"
 *   "cmp r0, #0xE1\n beq.w .Lo_E1\n"
 *   "cmp r0, #0x11\n beq.w .Lo_11\n"
 *   "cmp r0, #0x31\n beq.w .Lo_31\n"
 *   "cmp r0, #0x51\n beq.w .Lo_51\n"
 *   "cmp r0, #0x71\n beq.w .Lo_71\n"
 *   "cmp r0, #0x91\n beq.w .Lo_91\n"
 *   "cmp r0, #0xB1\n beq.w .Lo_B1\n"
 *   "cmp r0, #0xD1\n beq.w .Lo_D1\n"
 *   "cmp r0, #0xF1\n beq.w .Lo_F1\n"
 *   "cmp r0, #0x12\n beq.w .Lo_12\n"
 *   "cmp r0, #0x32\n beq.w .Lo_32\n"
 *   "cmp r0, #0x52\n beq.w .Lo_52\n"
 *   "cmp r0, #0x72\n beq.w .Lo_72\n"
 *   "cmp r0, #0x92\n beq.w .Lo_92\n"
 *   "cmp r0, #0xB2\n beq.w .Lo_B2\n"
 *   "cmp r0, #0xD2\n beq.w .Lo_D2\n"
 *   "cmp r0, #0xF2\n beq.w .Lo_F2\n"
 *   "cmp r0, #0x07\n beq.w .Lo_07\n"
 *   "cmp r0, #0x27\n beq.w .Lo_27\n"
 *   "cmp r0, #0x47\n beq.w .Lo_47\n"
 *   "cmp r0, #0x67\n beq.w .Lo_67\n"
 *   "cmp r0, #0x87\n beq.w .Lo_87\n"
 *   "cmp r0, #0xA7\n beq.w .Lo_A7\n"
 *   "cmp r0, #0xC7\n beq.w .Lo_C7\n"
 *   "cmp r0, #0xE7\n beq.w .Lo_E7\n"
 *   "cmp r0, #0x17\n beq.w .Lo_17\n"
 *   "cmp r0, #0x37\n beq.w .Lo_37\n"
 *   "cmp r0, #0x57\n beq.w .Lo_57\n"
 *   "cmp r0, #0x77\n beq.w .Lo_77\n"
 *   "cmp r0, #0x97\n beq.w .Lo_97\n"
 *   "cmp r0, #0xB7\n beq.w .Lo_B7\n"
 *   "cmp r0, #0xD7\n beq.w .Lo_D7\n"
 *   "cmp r0, #0xF7\n beq.w .Lo_F7\n"
 *   "cmp r0, #0x03\n beq.w .Lo_03\n"
 *   "cmp r0, #0x23\n beq.w .Lo_23\n"
 *   "cmp r0, #0x43\n beq.w .Lo_43\n"
 *   "cmp r0, #0x63\n beq.w .Lo_63\n"
 *   "cmp r0, #0x83\n beq.w .Lo_83\n"
 *   "cmp r0, #0xA3\n beq.w .Lo_A3\n"
 *   "cmp r0, #0xC3\n beq.w .Lo_C3\n"
 *   "cmp r0, #0xE3\n beq.w .Lo_E3\n"
 *   "cmp r0, #0x13\n beq.w .Lo_13\n"
 *   "cmp r0, #0x33\n beq.w .Lo_33\n"
 *   "cmp r0, #0x53\n beq.w .Lo_53\n"
 *   "cmp r0, #0x73\n beq.w .Lo_73\n"
 *   "cmp r0, #0x93\n beq.w .Lo_93\n"
 *   "cmp r0, #0xB3\n beq.w .Lo_B3\n"
 *   "cmp r0, #0xD3\n beq.w .Lo_D3\n"
 *   "cmp r0, #0xF3\n beq.w .Lo_F3\n"
 *   "cmp r0, #0x0F\n beq.w .Lo_0F\n"
 *   "cmp r0, #0x2F\n beq.w .Lo_2F\n"
 *   "cmp r0, #0x4F\n beq.w .Lo_4F\n"
 *   "cmp r0, #0x6F\n beq.w .Lo_6F\n"
 *   "cmp r0, #0x8F\n beq.w .Lo_8F\n"
 *   "cmp r0, #0xAF\n beq.w .Lo_AF\n"
 *   "cmp r0, #0xCF\n beq.w .Lo_CF\n"
 *   "cmp r0, #0xEF\n beq.w .Lo_EF\n"
 *   "cmp r0, #0x1F\n beq.w .Lo_1F\n"
 *   "cmp r0, #0x3F\n beq.w .Lo_3F\n"
 *   "cmp r0, #0x5F\n beq.w .Lo_5F\n"
 *   "cmp r0, #0x7F\n beq.w .Lo_7F\n"
 *   "cmp r0, #0x9F\n beq.w .Lo_9F\n"
 *   "cmp r0, #0xBF\n beq.w .Lo_BF\n"
 *   "cmp r0, #0xDF\n beq.w .Lo_DF\n"
 *   "cmp r0, #0xFF\n beq.w .Lo_FF\n"
 *   "cmp r0, #0x80\n beq.w .Lo_80\n"
 *   "cmp r0, #0x82\n beq.w .Lo_82\n"
 *   "cmp r0, #0x50\n beq.w .Lo_50\n"
 *   "cmp r0, #0x70\n beq.w .Lo_70\n"
 *   "cmp r0, #0x4C\n beq.w .Lo_4C\n"
 *   "cmp r0, #0x6C\n beq.w .Lo_6C\n"
 *   "cmp r0, #0x7C\n beq.w .Lo_7C\n"
 *   "cmp r0, #0x5C\n beq.w .Lo_5C\n"
 *   "cmp r0, #0xDC\n beq.w .Lo_DC\n"
 *   "cmp r0, #0x22\n beq.w .Lo_22\n"
 *   "cmp r0, #0xFC\n beq.w .Lo_FC\n"
 *   "cmp r0, #0x6B\n beq.w .Lo_6B\n"
 *   "cmp r0, #0xF4\n beq.w .Lo_F4\n"
 *   "cmp r0, #0xD4\n beq.w .Lo_D4\n"
 *   "cmp r0, #0x62\n beq.w .Lo_62\n"
 *   "cmp r0, #0xC2\n beq.w .Lo_C2\n"
 *   "cmp r0, #0xE2\n beq.w .Lo_E2\n"
 *   "cmp r0, #0x04\n beq.w .Lo_04\n"
 *   "cmp r0, #0x0C\n beq.w .Lo_0C\n"
 *   "cmp r0, #0x14\n beq.w .Lo_14\n"
 *   "cmp r0, #0x1C\n beq.w .Lo_1C\n"
 *   "cmp r0, #0x1B\n beq.w .Lo_1B\n"
 *   "cmp r0, #0x3B\n beq.w .Lo_3B\n"
 *   "cmp r0, #0x5B\n beq.w .Lo_5B\n"
 *   "cmp r0, #0x7B\n beq.w .Lo_7B\n"
 *   "cmp r0, #0xCB\n beq.w .Lo_CB\n"
 *   "cmp r0, #0xDB\n beq.w .Lo_DB\n"
 *   "cmp r0, #0x42\n beq.w .Lo_42\n"
 *
 * INSERT OPCODE BODIES below the existing opcodes, before the epilogue.
 */

/* ======================================================================== */
/* ADDRESSING MODE SUBROUTINES                                               */
/*                                                                           */
/* Each computes EA into S3(low), S4(high), then returns.                    */
/* Scratch: S0, S1 for intermediates. LR saved in S2.                        */
/* Non-leaf: calls .Lfetch/.Lrd/.Lidl which use FLR (separate slot).         */
/*                                                                           */
/* IMPORTANT: r12 is caller-save and clobbered by any bl subroutine that     */
/* calls an external function. All temps must be in stack scratch slots.      */
/* ======================================================================== */

/*
 * .Ladr_idx — (dp,X) indirect indexed
 * Cycle cost: fetch + (DPL?idle) + idle + read_lo + read_hi
 */
".Ladr_idx:\n"
"str lr, [sp, #" S(S2) "]\n"
"bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"  /* S0 = dp offset */
"ldr r0, [sp, #" S(FDP) "]\n tst r0, #0xFF\n beq.w 1f\n bl .Lidl\n"
"1: bl .Lidl\n"  /* indexed extra cycle */
/* compute base addr = (DP + offset + X) & 0xFFFF, save to S1 */
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

/*
 * .Ladr_idy — (dp),Y indirect Y-indexed
 * Caller sets S4 = 1 for write opcodes, S4 = 0 for read opcodes.
 * Extra idle if: write OR XF=0 OR page crossed.
 */
".Ladr_idy:\n"
"str lr, [sp, #" S(S2) "]\n"
"bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"  /* S0 = dp offset */
"ldr r0, [sp, #" S(FDP) "]\n tst r0, #0xFF\n beq.w 1f\n bl .Lidl\n"
"1:\n"
/* read 16-bit pointer from (DP+offset) */
"ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
"add r1, r1, r0\n uxth r1, r1\n str r1, [sp, #" S(S1) "]\n"  /* S1 = base */
"bl .Lrd\n str r0, [sp, #" S(S0) "]\n"  /* S0 = ptr_lo */
"ldr r1, [sp, #" S(S1) "]\n adds r1, #1\n uxth r1, r1\n"
"bl .Lrd\n"  /* r0 = ptr_hi */
"ldr r1, [sp, #" S(S0) "]\n orr r1, r1, r0, lsl #8\n"  /* r1 = 16-bit pointer */
"str r1, [sp, #" S(S1) "]\n"  /* S1 = pointer */
/* page cross check: if write OR !XF OR page crossed, idle */
"ldr r0, [sp, #" S(S4) "]\n"  /* write flag from caller */
"cmp r0, #0\n bne.w 2f\n"     /* write -> always idle */
"tst r9, #0x10\n beq.w 2f\n"  /* XF=0 -> idle */
/* check page cross: (ptr>>8) != ((ptr+Y)>>8) */
"ldr r1, [sp, #" S(S1) "]\n"
"add r0, r1, r6\n lsr r0, r0, #8\n lsr r3, r1, #8\n cmp r0, r3\n beq.w 3f\n"
"2: bl .Lidl\n"
"3:\n"
/* ea = (DB<<16 + pointer + Y) & 0xFFFFFF */
"ldr r0, [sp, #" S(FDB) "]\n lsl r0, r0, #16\n"
"ldr r1, [sp, #" S(S1) "]\n add r0, r0, r1\n add r0, r0, r6\n"
"bic r0, r0, #0xFF000000\n str r0, [sp, #" S(S3) "]\n"
"adds r0, #1\n bic r0, r0, #0xFF000000\n str r0, [sp, #" S(S4) "]\n"
"ldr lr, [sp, #" S(S2) "]\n bx lr\n"

/*
 * .Ladr_idp — (dp) indirect
 */
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

/*
 * .Ladr_idl — [dp] indirect long (24-bit pointer)
 */
".Ladr_idl:\n"
"str lr, [sp, #" S(S2) "]\n"
"bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"  /* S0 = dp offset */
"ldr r0, [sp, #" S(FDP) "]\n tst r0, #0xFF\n beq.w 1f\n bl .Lidl\n"
"1:\n"
/* addr = (DP + offset) & 0xFFFF */
"ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
"add r1, r1, r0\n uxth r1, r1\n str r1, [sp, #" S(S1) "]\n"  /* S1 = base */
"bl .Lrd\n str r0, [sp, #" S(S0) "]\n"  /* S0 = ptr byte 0 */
"ldr r1, [sp, #" S(S1) "]\n adds r1, #1\n uxth r1, r1\n"
"bl .Lrd\n"  /* r0 = ptr byte 1 */
"ldr r1, [sp, #" S(S0) "]\n orr r1, r1, r0, lsl #8\n str r1, [sp, #" S(S0) "]\n"
/* S0 now has low 16 bits of pointer */
"ldr r1, [sp, #" S(S1) "]\n adds r1, #2\n uxth r1, r1\n"
"bl .Lrd\n"  /* r0 = ptr byte 2 (bank) */
"ldr r1, [sp, #" S(S0) "]\n orr r1, r1, r0, lsl #16\n"  /* 24-bit pointer */
"str r1, [sp, #" S(S3) "]\n"
"adds r1, #1\n bic r1, r1, #0xFF000000\n str r1, [sp, #" S(S4) "]\n"
"ldr lr, [sp, #" S(S2) "]\n bx lr\n"

/*
 * .Ladr_ily — [dp],Y indirect long Y-indexed
 */
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
/* ea = (pointer + Y) & 0xFFFFFF */
"add r1, r1, r6\n bic r1, r1, #0xFF000000\n"
"str r1, [sp, #" S(S3) "]\n"
"adds r1, #1\n bic r1, r1, #0xFF000000\n str r1, [sp, #" S(S4) "]\n"
"ldr lr, [sp, #" S(S2) "]\n bx lr\n"

/*
 * .Ladr_sr — stack relative (d,S)
 * EA = (SP + offset) & 0xFFFF. Always bank 0.
 */
".Ladr_sr:\n"
"str lr, [sp, #" S(S2) "]\n"
"bl .Lfetch\n"  /* r0 = offset */
"str r0, [sp, #" S(S0) "]\n"
"bl .Lidl\n"
"ldr r0, [sp, #" S(S0) "]\n add r1, r7, r0\n uxth r1, r1\n"
"str r1, [sp, #" S(S3) "]\n"
"adds r1, #1\n uxth r1, r1\n str r1, [sp, #" S(S4) "]\n"
"ldr lr, [sp, #" S(S2) "]\n bx lr\n"

/*
 * .Ladr_isy — (d,S),Y stack relative indirect Y-indexed
 */
".Ladr_isy:\n"
"str lr, [sp, #" S(S2) "]\n"
"bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
"bl .Lidl\n"
/* read 16-bit pointer from (SP+offset) */
"ldr r0, [sp, #" S(S0) "]\n add r1, r7, r0\n uxth r1, r1\n"
"str r1, [sp, #" S(S1) "]\n"  /* S1 = addr */
"bl .Lrd\n str r0, [sp, #" S(S0) "]\n"  /* S0 = ptr_lo */
"ldr r1, [sp, #" S(S1) "]\n adds r1, #1\n uxth r1, r1\n"
"bl .Lrd\n"  /* r0 = ptr_hi */
"ldr r1, [sp, #" S(S0) "]\n orr r1, r1, r0, lsl #8\n"  /* r1 = 16-bit pointer */
"str r1, [sp, #" S(S1) "]\n"  /* save pointer */
"bl .Lidl\n"  /* extra idle cycle */
/* ea = (DB<<16 + pointer + Y) & 0xFFFFFF */
"ldr r0, [sp, #" S(FDB) "]\n lsl r0, r0, #16\n"
"ldr r1, [sp, #" S(S1) "]\n add r0, r0, r1\n add r0, r0, r6\n"
"bic r0, r0, #0xFF000000\n str r0, [sp, #" S(S3) "]\n"
"adds r0, #1\n bic r0, r0, #0xFF000000\n str r0, [sp, #" S(S4) "]\n"
"ldr lr, [sp, #" S(S2) "]\n bx lr\n"

/*
 * .Ladr_abl — absolute long (24-bit address from PC)
 */
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

/*
 * .Ladr_alx — absolute long,X
 */
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

/* ======================================================================== */
/* SHARED ALU FINISH ROUTINES                                                */
/*                                                                           */
/* EA is in S3(low), S4(high). These read operand, do ALU, update flags.     */
/* ======================================================================== */

/* ---- ORA finish ---- */
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

/* ---- AND finish ---- */
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

/* ---- EOR finish ---- */
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

/* ---- LDA finish ---- */
".Lalu_lda:\n"
"tst r9, #0x20\n beq.w .Lalu_lda16\n"
"ldr r1, [sp, #" S(S3) "]\n bl .Lint\n bl .Lrd\n"
"bfi r4, r0, #0, #8\n bl .Lzn8\n b.w .Lnext\n"
".Lalu_lda16:\n"
"ldr r1, [sp, #" S(S3) "]\n bl .Lrd\n str r0, [sp, #" S(S0) "]\n"
"ldr r1, [sp, #" S(S4) "]\n bl .Lint\n bl .Lrd\n"
"lsl r0, r0, #8\n ldr r1, [sp, #" S(S0) "]\n orr r4, r1, r0\n"
"mov r0, r4\n bl .Lzn16\n b.w .Lnext\n"

/* ---- STA finish ---- */
".Lalu_sta:\n"
"tst r9, #0x20\n beq.w .Lalu_sta16\n"
"ldr r1, [sp, #" S(S3) "]\n uxtb r2, r4\n bl .Lint\n bl .Lwr\n b.w .Lnext\n"
".Lalu_sta16:\n"
"ldr r1, [sp, #" S(S3) "]\n uxtb r2, r4\n bl .Lwr\n"
"ldr r1, [sp, #" S(S4) "]\n lsr r2, r4, #8\n uxtb r2, r2\n bl .Lint\n bl .Lwr\n b.w .Lnext\n"

/* ---- CMP finish ---- */
".Lalu_cmp:\n"
"tst r9, #0x20\n beq.w .Lalu_cmp16\n"
"ldr r1, [sp, #" S(S3) "]\n bl .Lint\n bl .Lrd\n"
"b.w .Lcmp8\n"
".Lalu_cmp16:\n"
"ldr r1, [sp, #" S(S3) "]\n bl .Lrd\n str r0, [sp, #" S(S0) "]\n"
"ldr r1, [sp, #" S(S4) "]\n bl .Lint\n bl .Lrd\n"
"lsl r0, r0, #8\n ldr r1, [sp, #" S(S0) "]\n orr r0, r1, r0\n"
"b.w .Lcmp16\n"

/* ---- ADC finish (binary only) ---- */
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
"1: uxth r2, r3\n cbnz r2, 1f\n orr r9, r9, #0x02\n"
"1: tst r3, #0x8000\n beq 1f\n orr r9, r9, #0x80\n"
"1: uxth r4, r3\n b.w .Lnext\n"

/* ---- SBC finish (binary only) ---- */
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
"1: uxth r2, r3\n cbnz r2, 1f\n orr r9, r9, #0x02\n"
"1: tst r3, #0x8000\n beq 1f\n orr r9, r9, #0x80\n"
"1: uxth r4, r3\n b.w .Lnext\n"

".ltorg\n"

/* ======================================================================== */
/* INTERRUPT OPCODES                                                         */
/* ======================================================================== */

/* ---- BRK (00) ----
 * cpu.c: readOpcode (discard sig byte), pushByte(K), pushWord(PC,false),
 *        pushByte(getFlags), I=1 D=0 K=0,
 *        PC = readWord(0xFFE6, 0xFFE7, true).
 * (lakesnes always pushes K, always uses native vector)
 */
".Lo_00:\n"
"bl .Lfetch\n"  /* read & discard signature byte */
/* pushByte(K) */
"mov r1, r7\n ldr r2, [sp, #" S(FK) "]\n uxtb r2, r2\n bl .Lwr\n"
"subs r7, #1\n uxth r7, r7\n"
"ldr r0, [sp, #" S(FE) "]\n cbz r0, 1f\n and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
"1:\n"
/* pushWord(PC, false) — high byte first */
"mov r1, r7\n lsr r2, r8, #8\n uxtb r2, r2\n bl .Lwr\n"
"subs r7, #1\n uxth r7, r7\n"
"ldr r0, [sp, #" S(FE) "]\n cbz r0, 1f\n and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
"1:\n"
/* low byte — no intCheck (false) */
"mov r1, r7\n uxtb r2, r8\n bl .Lwr\n"
"subs r7, #1\n uxth r7, r7\n"
"ldr r0, [sp, #" S(FE) "]\n cbz r0, 1f\n and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
"1:\n"
/* pushByte(flags) — r9 is the packed flags byte */
"mov r1, r7\n uxtb r2, r9\n bl .Lwr\n"
"subs r7, #1\n uxth r7, r7\n"
"ldr r0, [sp, #" S(FE) "]\n cbz r0, 1f\n and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
"1:\n"
/* I=1, D=0, K=0 */
"orr r9, r9, #4\n"
"bic r9, r9, #8\n"
"movs r0, #0\n str r0, [sp, #" S(FK) "]\n"
/* readWord(0xFFE6, 0xFFE7, true) — intCheck before high byte */
"movw r1, #0xFFE6\n bl .Lrd\n str r0, [sp, #" S(S0) "]\n"
"movw r1, #0xFFE7\n bl .Lint\n bl .Lrd\n"
"ldr r1, [sp, #" S(S0) "]\n orr r8, r1, r0, lsl #8\n uxth r8, r8\n"
"b.w .Lnext\n"

/* ---- COP (02) ----
 * Same as BRK but vector at $FFE4/$FFE5.
 */
".Lo_02:\n"
"bl .Lfetch\n"
"mov r1, r7\n ldr r2, [sp, #" S(FK) "]\n uxtb r2, r2\n bl .Lwr\n"
"subs r7, #1\n uxth r7, r7\n"
"ldr r0, [sp, #" S(FE) "]\n cbz r0, 1f\n and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
"1:\n"
"mov r1, r7\n lsr r2, r8, #8\n uxtb r2, r2\n bl .Lwr\n"
"subs r7, #1\n uxth r7, r7\n"
"ldr r0, [sp, #" S(FE) "]\n cbz r0, 1f\n and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
"1:\n"
"mov r1, r7\n uxtb r2, r8\n bl .Lwr\n"
"subs r7, #1\n uxth r7, r7\n"
"ldr r0, [sp, #" S(FE) "]\n cbz r0, 1f\n and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
"1:\n"
"mov r1, r7\n uxtb r2, r9\n bl .Lwr\n"
"subs r7, #1\n uxth r7, r7\n"
"ldr r0, [sp, #" S(FE) "]\n cbz r0, 1f\n and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
"1:\n"
"orr r9, r9, #4\n bic r9, r9, #8\n"
"movs r0, #0\n str r0, [sp, #" S(FK) "]\n"
"movw r1, #0xFFE4\n bl .Lrd\n str r0, [sp, #" S(S0) "]\n"
"movw r1, #0xFFE5\n bl .Lint\n bl .Lrd\n"
"ldr r1, [sp, #" S(S0) "]\n orr r8, r1, r0, lsl #8\n uxth r8, r8\n"
"b.w .Lnext\n"

/* ---- RTI (40) ----
 * cpu.c: idle, idle, setFlags(pullByte), pc = pullWord(false),
 *        checkInt, k = pullByte.
 */
".Lo_40:\n"
"bl .Lidl\n bl .Lidl\n"
/* pullByte -> setFlags */
"adds r7, #1\n uxth r7, r7\n"
"ldr r0, [sp, #" S(FE) "]\n cbz r0, 1f\n and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
"1: mov r1, r7\n bl .Lrd\n"
/* r0 = pulled flags byte. Apply setFlags logic: */
"mov r9, r0\n"
/* if E mode: force MF=1, XF=1, SP wrap */
"ldr r0, [sp, #" S(FE) "]\n cbz r0, .Lrti_no_e\n"
"orr r9, r9, #0x30\n"
"and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
".Lrti_no_e:\n"
/* if XF set: truncate X,Y */
"tst r9, #0x10\n beq.w .Lrti_no_trunc\n"
"uxtb r5, r5\n uxtb r6, r6\n"
".Lrti_no_trunc:\n"
/* pullWord(false): pull PC low then high */
"adds r7, #1\n uxth r7, r7\n"
"ldr r0, [sp, #" S(FE) "]\n cbz r0, 1f\n and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
"1: mov r1, r7\n bl .Lrd\n str r0, [sp, #" S(S0) "]\n"
"adds r7, #1\n uxth r7, r7\n"
"ldr r0, [sp, #" S(FE) "]\n cbz r0, 1f\n and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
"1: mov r1, r7\n bl .Lrd\n"
"ldr r1, [sp, #" S(S0) "]\n orr r8, r1, r0, lsl #8\n uxth r8, r8\n"
/* checkInt */
"bl .Lint\n"
/* pullByte -> K */
"adds r7, #1\n uxth r7, r7\n"
"ldr r0, [sp, #" S(FE) "]\n cbz r0, 1f\n and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
"1: mov r1, r7\n bl .Lrd\n str r0, [sp, #" S(FK) "]\n"
"b.w .Lnext\n"

/* ======================================================================== */
/* BLOCK MOVE OPCODES                                                        */
/* ======================================================================== */

/* ---- MVP (44) — block move previous (decrement) ----
 * cpu.c: dest = readOpcode, src = readOpcode, DB = dest,
 *   write(dest<<16|Y, read(src<<16|X)), A--, X--, Y--,
 *   if A != 0xFFFF: PC -= 3,
 *   if XF: X &= 0xFF, Y &= 0xFF,
 *   idle, checkInt, idle.
 */
".Lo_44:\n"
"bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"   /* S0 = dest bank */
"bl .Lfetch\n str r0, [sp, #" S(S1) "]\n"   /* S1 = src bank */
/* DB = dest */
"ldr r0, [sp, #" S(S0) "]\n str r0, [sp, #" S(FDB) "]\n"
/* read from (src<<16 | X) */
"ldr r0, [sp, #" S(S1) "]\n lsl r1, r0, #16\n orr r1, r1, r5\n"
"bl .Lrd\n str r0, [sp, #" S(S3) "]\n"  /* S3 = data byte */
/* write to (dest<<16 | Y) */
"ldr r0, [sp, #" S(S0) "]\n lsl r1, r0, #16\n orr r1, r1, r6\n"
"ldr r2, [sp, #" S(S3) "]\n uxtb r2, r2\n bl .Lwr\n"
/* A-- */
"subs r4, #1\n uxth r4, r4\n"
/* X--, Y-- */
"subs r5, #1\n uxth r5, r5\n"
"subs r6, #1\n uxth r6, r6\n"
/* if A != 0xFFFF: PC -= 3 */
"movw r0, #0xFFFF\n cmp r4, r0\n beq.w 1f\n"
"subs r8, #3\n uxth r8, r8\n"
"1:\n"
/* if XF: truncate X,Y */
"tst r9, #0x10\n beq.w 1f\n"
"uxtb r5, r5\n uxtb r6, r6\n"
"1:\n"
"bl .Lidl\n bl .Lint\n bl .Lidl\n"
"b.w .Lnext\n"

/* ---- MVN (54) — block move next (increment) ----
 * Same as MVP but X++, Y++ instead of X--, Y--.
 */
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

/* ======================================================================== */
/* INDIRECT ADDRESSING MODE ALU — (dp,X) idx                                */
/* Opcodes: ORA(01), AND(21), EOR(41), ADC(61), STA(81),                    */
/*          LDA(A1), CMP(C1), SBC(E1)                                       */
/* ADC/SBC: check D flag before addressing; if set, fall back to C.          */
/* ======================================================================== */

".Lo_01:\n bl .Ladr_idx\n b.w .Lalu_ora\n"
".Lo_21:\n bl .Ladr_idx\n b.w .Lalu_and\n"
".Lo_41:\n bl .Ladr_idx\n b.w .Lalu_eor\n"
".Lo_61:\n tst r9, #0x08\n bne.w .Lfb\n bl .Ladr_idx\n b.w .Lalu_adc\n"
".Lo_81:\n bl .Ladr_idx\n b.w .Lalu_sta\n"
".Lo_A1:\n bl .Ladr_idx\n b.w .Lalu_lda\n"
".Lo_C1:\n bl .Ladr_idx\n b.w .Lalu_cmp\n"
".Lo_E1:\n tst r9, #0x08\n bne.w .Lfb\n bl .Ladr_idx\n b.w .Lalu_sbc\n"

/* ======================================================================== */
/* INDIRECT ADDRESSING MODE ALU — (dp),Y idy                                */
/* For reads: S4 = 0. For STA (write): S4 = 1.                              */
/* ======================================================================== */

".Lo_11:\n movs r0, #0\n str r0, [sp, #" S(S4) "]\n bl .Ladr_idy\n b.w .Lalu_ora\n"
".Lo_31:\n movs r0, #0\n str r0, [sp, #" S(S4) "]\n bl .Ladr_idy\n b.w .Lalu_and\n"
".Lo_51:\n movs r0, #0\n str r0, [sp, #" S(S4) "]\n bl .Ladr_idy\n b.w .Lalu_eor\n"
".Lo_71:\n tst r9, #0x08\n bne.w .Lfb\n movs r0, #0\n str r0, [sp, #" S(S4) "]\n bl .Ladr_idy\n b.w .Lalu_adc\n"
".Lo_91:\n movs r0, #1\n str r0, [sp, #" S(S4) "]\n bl .Ladr_idy\n b.w .Lalu_sta\n"
".Lo_B1:\n movs r0, #0\n str r0, [sp, #" S(S4) "]\n bl .Ladr_idy\n b.w .Lalu_lda\n"
".Lo_D1:\n movs r0, #0\n str r0, [sp, #" S(S4) "]\n bl .Ladr_idy\n b.w .Lalu_cmp\n"
".Lo_F1:\n tst r9, #0x08\n bne.w .Lfb\n movs r0, #0\n str r0, [sp, #" S(S4) "]\n bl .Ladr_idy\n b.w .Lalu_sbc\n"

/* ======================================================================== */
/* INDIRECT ADDRESSING MODE ALU — (dp) idp                                  */
/* ======================================================================== */

".Lo_12:\n bl .Ladr_idp\n b.w .Lalu_ora\n"
".Lo_32:\n bl .Ladr_idp\n b.w .Lalu_and\n"
".Lo_52:\n bl .Ladr_idp\n b.w .Lalu_eor\n"
".Lo_72:\n tst r9, #0x08\n bne.w .Lfb\n bl .Ladr_idp\n b.w .Lalu_adc\n"
".Lo_92:\n bl .Ladr_idp\n b.w .Lalu_sta\n"
".Lo_B2:\n bl .Ladr_idp\n b.w .Lalu_lda\n"
".Lo_D2:\n bl .Ladr_idp\n b.w .Lalu_cmp\n"
".Lo_F2:\n tst r9, #0x08\n bne.w .Lfb\n bl .Ladr_idp\n b.w .Lalu_sbc\n"

/* ======================================================================== */
/* INDIRECT ADDRESSING MODE ALU — [dp] idl                                  */
/* ======================================================================== */

".Lo_07:\n bl .Ladr_idl\n b.w .Lalu_ora\n"
".Lo_27:\n bl .Ladr_idl\n b.w .Lalu_and\n"
".Lo_47:\n bl .Ladr_idl\n b.w .Lalu_eor\n"
".Lo_67:\n tst r9, #0x08\n bne.w .Lfb\n bl .Ladr_idl\n b.w .Lalu_adc\n"
".Lo_87:\n bl .Ladr_idl\n b.w .Lalu_sta\n"
".Lo_A7:\n bl .Ladr_idl\n b.w .Lalu_lda\n"
".Lo_C7:\n bl .Ladr_idl\n b.w .Lalu_cmp\n"
".Lo_E7:\n tst r9, #0x08\n bne.w .Lfb\n bl .Ladr_idl\n b.w .Lalu_sbc\n"

/* ======================================================================== */
/* INDIRECT ADDRESSING MODE ALU — [dp],Y ily                                */
/* ======================================================================== */

".Lo_17:\n bl .Ladr_ily\n b.w .Lalu_ora\n"
".Lo_37:\n bl .Ladr_ily\n b.w .Lalu_and\n"
".Lo_57:\n bl .Ladr_ily\n b.w .Lalu_eor\n"
".Lo_77:\n tst r9, #0x08\n bne.w .Lfb\n bl .Ladr_ily\n b.w .Lalu_adc\n"
".Lo_97:\n bl .Ladr_ily\n b.w .Lalu_sta\n"
".Lo_B7:\n bl .Ladr_ily\n b.w .Lalu_lda\n"
".Lo_D7:\n bl .Ladr_ily\n b.w .Lalu_cmp\n"
".Lo_F7:\n tst r9, #0x08\n bne.w .Lfb\n bl .Ladr_ily\n b.w .Lalu_sbc\n"

/* ======================================================================== */
/* STACK RELATIVE ALU — d,S (sr)                                             */
/* ======================================================================== */

".Lo_03:\n bl .Ladr_sr\n b.w .Lalu_ora\n"
".Lo_23:\n bl .Ladr_sr\n b.w .Lalu_and\n"
".Lo_43:\n bl .Ladr_sr\n b.w .Lalu_eor\n"
".Lo_63:\n tst r9, #0x08\n bne.w .Lfb\n bl .Ladr_sr\n b.w .Lalu_adc\n"
".Lo_83:\n bl .Ladr_sr\n b.w .Lalu_sta\n"
".Lo_A3:\n bl .Ladr_sr\n b.w .Lalu_lda\n"
".Lo_C3:\n bl .Ladr_sr\n b.w .Lalu_cmp\n"
".Lo_E3:\n tst r9, #0x08\n bne.w .Lfb\n bl .Ladr_sr\n b.w .Lalu_sbc\n"

/* ======================================================================== */
/* STACK RELATIVE INDIRECT Y-INDEXED ALU — (d,S),Y (isy)                    */
/* ======================================================================== */

".Lo_13:\n bl .Ladr_isy\n b.w .Lalu_ora\n"
".Lo_33:\n bl .Ladr_isy\n b.w .Lalu_and\n"
".Lo_53:\n bl .Ladr_isy\n b.w .Lalu_eor\n"
".Lo_73:\n tst r9, #0x08\n bne.w .Lfb\n bl .Ladr_isy\n b.w .Lalu_adc\n"
".Lo_93:\n bl .Ladr_isy\n b.w .Lalu_sta\n"
".Lo_B3:\n bl .Ladr_isy\n b.w .Lalu_lda\n"
".Lo_D3:\n bl .Ladr_isy\n b.w .Lalu_cmp\n"
".Lo_F3:\n tst r9, #0x08\n bne.w .Lfb\n bl .Ladr_isy\n b.w .Lalu_sbc\n"

/* ======================================================================== */
/* ABSOLUTE LONG ALU — abl                                                   */
/* ======================================================================== */

".Lo_0F:\n bl .Ladr_abl\n b.w .Lalu_ora\n"
".Lo_2F:\n bl .Ladr_abl\n b.w .Lalu_and\n"
".Lo_4F:\n bl .Ladr_abl\n b.w .Lalu_eor\n"
".Lo_6F:\n tst r9, #0x08\n bne.w .Lfb\n bl .Ladr_abl\n b.w .Lalu_adc\n"
".Lo_8F:\n bl .Ladr_abl\n b.w .Lalu_sta\n"
".Lo_AF:\n bl .Ladr_abl\n b.w .Lalu_lda\n"
".Lo_CF:\n bl .Ladr_abl\n b.w .Lalu_cmp\n"
".Lo_EF:\n tst r9, #0x08\n bne.w .Lfb\n bl .Ladr_abl\n b.w .Lalu_sbc\n"

/* ======================================================================== */
/* ABSOLUTE LONG,X ALU — alx                                                */
/* ======================================================================== */

".Lo_1F:\n bl .Ladr_alx\n b.w .Lalu_ora\n"
".Lo_3F:\n bl .Ladr_alx\n b.w .Lalu_and\n"
".Lo_5F:\n bl .Ladr_alx\n b.w .Lalu_eor\n"
".Lo_7F:\n tst r9, #0x08\n bne.w .Lfb\n bl .Ladr_alx\n b.w .Lalu_adc\n"
".Lo_9F:\n bl .Ladr_alx\n b.w .Lalu_sta\n"
".Lo_BF:\n bl .Ladr_alx\n b.w .Lalu_lda\n"
".Lo_DF:\n bl .Ladr_alx\n b.w .Lalu_cmp\n"
".Lo_FF:\n tst r9, #0x08\n bne.w .Lfb\n bl .Ladr_alx\n b.w .Lalu_sbc\n"

".ltorg\n"

/* ======================================================================== */
/* BRANCHES                                                                  */
/* ======================================================================== */

/* ---- BRA (80) — branch always ---- */
".Lo_80:\n"
"bl .Lfetch\n sxtb r0, r0\n str r0, [sp, #" S(S0) "]\n"
"bl .Lint\n bl .Lidl\n"
"ldr r0, [sp, #" S(S0) "]\n add r8, r8, r0\n uxth r8, r8\n b.w .Lnext\n"

/* ---- BRL (82) — branch long (16-bit relative) ---- */
/* cpu.c: pc += (int16_t)readOpcodeWord(false); checkInt; idle */
".Lo_82:\n"
"bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
"bl .Lfetch\n"
"ldr r1, [sp, #" S(S0) "]\n orr r0, r1, r0, lsl #8\n"
"sxth r0, r0\n"
"add r8, r8, r0\n uxth r8, r8\n"
"bl .Lint\n bl .Lidl\n"
"b.w .Lnext\n"

/* ---- BVC (50) — branch if V clear ---- */
".Lo_50:\n tst r9, #0x40\n beq.w .Lbtk\n b .Lbnt\n"

/* ---- BVS (70) — branch if V set ---- */
".Lo_70:\n tst r9, #0x40\n bne.w .Lbtk\n b .Lbnt\n"

/* ======================================================================== */
/* JUMP OPCODES                                                              */
/* ======================================================================== */

/* ---- JMP abs (4C) ---- */
/* cpu.c: pc = readOpcodeWord(true) */
".Lo_4C:\n"
"bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
"bl .Lint\n bl .Lfetch\n"
"ldr r1, [sp, #" S(S0) "]\n orr r8, r1, r0, lsl #8\n uxth r8, r8\n"
"b.w .Lnext\n"

/* ---- JMP (ind) (6C) ---- */
/* cpu.c: adr = readOpcodeWord(false); pc = readWord(adr, (adr+1)&0xFFFF, true) */
".Lo_6C:\n"
"bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
"bl .Lfetch\n ldr r1, [sp, #" S(S0) "]\n orr r1, r1, r0, lsl #8\n"
"str r1, [sp, #" S(S0) "]\n"
"bl .Lrd\n str r0, [sp, #" S(S1) "]\n"
"ldr r1, [sp, #" S(S0) "]\n adds r1, #1\n uxth r1, r1\n"
"bl .Lint\n bl .Lrd\n"
"ldr r1, [sp, #" S(S1) "]\n orr r8, r1, r0, lsl #8\n uxth r8, r8\n"
"b.w .Lnext\n"

/* ---- JMP (abs,X) (7C) ---- */
/* cpu.c: adr = readOpcodeWord(false); idle;
 *        pc = readWord((k<<16)|((adr+x)&0xFFFF), ..., true) */
".Lo_7C:\n"
"bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
"bl .Lfetch\n ldr r1, [sp, #" S(S0) "]\n orr r1, r1, r0, lsl #8\n"
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

/* ---- JML abl (5C) ---- */
/* cpu.c: value = readOpcodeWord(false); checkInt; k = readOpcode; pc = value */
".Lo_5C:\n"
"bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
"bl .Lfetch\n ldr r1, [sp, #" S(S0) "]\n orr r1, r1, r0, lsl #8\n"
"str r1, [sp, #" S(S0) "]\n"
"bl .Lint\n"
"bl .Lfetch\n str r0, [sp, #" S(FK) "]\n"
"ldr r8, [sp, #" S(S0) "]\n uxth r8, r8\n"
"b.w .Lnext\n"

/* ---- JML [ial] (DC) ---- */
/* cpu.c: adr = readOpcodeWord(false);
 *        pc = readWord(adr, (adr+1)&0xFFFF, false);
 *        checkInt; k = read((adr+2)&0xFFFF) */
".Lo_DC:\n"
"bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
"bl .Lfetch\n ldr r1, [sp, #" S(S0) "]\n orr r1, r1, r0, lsl #8\n"
"str r1, [sp, #" S(S0) "]\n"
"bl .Lrd\n str r0, [sp, #" S(S1) "]\n"
"ldr r1, [sp, #" S(S0) "]\n adds r1, #1\n uxth r1, r1\n"
"bl .Lrd\n"
"ldr r1, [sp, #" S(S1) "]\n orr r8, r1, r0, lsl #8\n uxth r8, r8\n"
"bl .Lint\n"
"ldr r1, [sp, #" S(S0) "]\n adds r1, #2\n uxth r1, r1\n"
"bl .Lrd\n str r0, [sp, #" S(FK) "]\n"
"b.w .Lnext\n"

/* ---- JSL abl (22) ---- */
/* cpu.c: value = readOpcodeWord(false); pushByte(K); idle;
 *        newK = readOpcode; pushWord(PC-1, true); PC = value; K = newK */
".Lo_22:\n"
"bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
"bl .Lfetch\n ldr r1, [sp, #" S(S0) "]\n orr r1, r1, r0, lsl #8\n"
"str r1, [sp, #" S(S0) "]\n"
/* pushByte(K) */
"mov r1, r7\n ldr r2, [sp, #" S(FK) "]\n uxtb r2, r2\n bl .Lwr\n"
"subs r7, #1\n uxth r7, r7\n"
"ldr r0, [sp, #" S(FE) "]\n cbz r0, 1f\n and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
"1:\n"
"bl .Lidl\n"
/* newK = readOpcode */
"bl .Lfetch\n str r0, [sp, #" S(S1) "]\n"
/* pushWord(PC-1, true) — high first, intCheck before low */
"sub r0, r8, #1\n uxth r0, r0\n str r0, [sp, #" S(S3) "]\n"
"mov r1, r7\n lsr r2, r0, #8\n uxtb r2, r2\n bl .Lwr\n"
"subs r7, #1\n uxth r7, r7\n"
"ldr r0, [sp, #" S(FE) "]\n cbz r0, 1f\n and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
"1: bl .Lint\n"
"ldr r0, [sp, #" S(S3) "]\n uxtb r2, r0\n mov r1, r7\n bl .Lwr\n"
"subs r7, #1\n uxth r7, r7\n"
"ldr r0, [sp, #" S(FE) "]\n cbz r0, 1f\n and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
"1:\n"
/* PC = value, K = newK */
"ldr r8, [sp, #" S(S0) "]\n uxth r8, r8\n"
"ldr r0, [sp, #" S(S1) "]\n str r0, [sp, #" S(FK) "]\n"
"b.w .Lnext\n"

/* ---- JSR (abs,X) (FC) ---- */
/* cpu.c: adrl = readOpcode; pushWord(PC, false);
 *        adr = adrl | (readOpcode<<8); idle;
 *        value = readWord((k<<16)|((adr+x)&0xFFFF), ..., true); PC = value
 * Note: pushWord pushes the CURRENT PC (pointing at the high byte of adr).
 */
".Lo_FC:\n"
"bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"  /* S0 = adrl */
/* pushWord(PC, false) — PC currently points at adr high byte */
"mov r1, r7\n lsr r2, r8, #8\n uxtb r2, r2\n bl .Lwr\n"
"subs r7, #1\n uxth r7, r7\n"
"ldr r0, [sp, #" S(FE) "]\n cbz r0, 1f\n and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
"1:\n"
"mov r1, r7\n uxtb r2, r8\n bl .Lwr\n"
"subs r7, #1\n uxth r7, r7\n"
"ldr r0, [sp, #" S(FE) "]\n cbz r0, 1f\n and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
"1:\n"
/* read adr high byte */
"bl .Lfetch\n"
"ldr r1, [sp, #" S(S0) "]\n orr r1, r1, r0, lsl #8\n"
"add r1, r1, r5\n uxth r1, r1\n str r1, [sp, #" S(S0) "]\n"
"bl .Lidl\n"
/* readWord from (K<<16 | (adr+X)) */
"ldr r0, [sp, #" S(FK) "]\n lsl r0, r0, #16\n"
"ldr r1, [sp, #" S(S0) "]\n orr r1, r0, r1\n"
"bl .Lrd\n str r0, [sp, #" S(S1) "]\n"
"ldr r0, [sp, #" S(FK) "]\n lsl r0, r0, #16\n"
"ldr r1, [sp, #" S(S0) "]\n adds r1, #1\n uxth r1, r1\n orr r1, r0, r1\n"
"bl .Lint\n bl .Lrd\n"
"ldr r1, [sp, #" S(S1) "]\n orr r8, r1, r0, lsl #8\n uxth r8, r8\n"
"b.w .Lnext\n"

/* ---- RTL (6B) ---- */
/* cpu.c: idle; idle; pc = pullWord(false) + 1; checkInt; k = pullByte */
".Lo_6B:\n"
"bl .Lidl\n bl .Lidl\n"
/* pullByte: SP++, wrap if E, read */
"adds r7, #1\n uxth r7, r7\n"
"ldr r0, [sp, #" S(FE) "]\n cbz r0, 1f\n and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
"1: mov r1, r7\n bl .Lrd\n str r0, [sp, #" S(S0) "]\n"
"adds r7, #1\n uxth r7, r7\n"
"ldr r0, [sp, #" S(FE) "]\n cbz r0, 1f\n and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
"1: mov r1, r7\n bl .Lrd\n"
"ldr r1, [sp, #" S(S0) "]\n orr r8, r1, r0, lsl #8\n"
"adds r8, #1\n uxth r8, r8\n"
"bl .Lint\n"
/* pullByte -> K */
"adds r7, #1\n uxth r7, r7\n"
"ldr r0, [sp, #" S(FE) "]\n cbz r0, 1f\n and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
"1: mov r1, r7\n bl .Lrd\n str r0, [sp, #" S(FK) "]\n"
"b.w .Lnext\n"

".ltorg\n"

/* ======================================================================== */
/* STACK OPCODES                                                             */
/* ======================================================================== */

/* ---- PEA (F4) — push 16-bit immediate ---- */
/* cpu.c: pushWord(readOpcodeWord(false), true) */
".Lo_F4:\n"
"bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
"bl .Lfetch\n ldr r1, [sp, #" S(S0) "]\n orr r0, r1, r0, lsl #8\n"
"str r0, [sp, #" S(S0) "]\n"
/* pushWord: high byte first */
"lsr r2, r0, #8\n uxtb r2, r2\n mov r1, r7\n bl .Lwr\n"
"subs r7, #1\n uxth r7, r7\n"
"ldr r0, [sp, #" S(FE) "]\n cbz r0, 1f\n and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
"1: bl .Lint\n"
"ldr r0, [sp, #" S(S0) "]\n uxtb r2, r0\n mov r1, r7\n bl .Lwr\n"
"subs r7, #1\n uxth r7, r7\n"
"ldr r0, [sp, #" S(FE) "]\n cbz r0, 1f\n and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
"1:\n"
"b.w .Lnext\n"

/* ---- PEI (D4) — push 16-bit from dp address ---- */
/* cpu.c: low/high = adrDp; pushWord(readWord(low,high,false), true) */
".Lo_D4:\n"
"bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
"ldr r0, [sp, #" S(FDP) "]\n tst r0, #0xFF\n beq.w 1f\n bl .Lidl\n"
"1: ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
"add r1, r1, r0\n uxth r1, r1\n str r1, [sp, #" S(S1) "]\n"
/* readWord(low, high, false) */
"bl .Lrd\n str r0, [sp, #" S(S3) "]\n"
"ldr r1, [sp, #" S(S1) "]\n adds r1, #1\n uxth r1, r1\n"
"bl .Lrd\n"
"ldr r1, [sp, #" S(S3) "]\n orr r0, r1, r0, lsl #8\n str r0, [sp, #" S(S0) "]\n"
/* pushWord(value, true) */
"lsr r2, r0, #8\n uxtb r2, r2\n mov r1, r7\n bl .Lwr\n"
"subs r7, #1\n uxth r7, r7\n"
"ldr r0, [sp, #" S(FE) "]\n cbz r0, 1f\n and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
"1: bl .Lint\n"
"ldr r0, [sp, #" S(S0) "]\n uxtb r2, r0\n mov r1, r7\n bl .Lwr\n"
"subs r7, #1\n uxth r7, r7\n"
"ldr r0, [sp, #" S(FE) "]\n cbz r0, 1f\n and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
"1:\n"
"b.w .Lnext\n"

/* ---- PER (62) — push PC + signed 16-bit offset ---- */
/* cpu.c: value = readOpcodeWord(false); idle; pushWord(PC + (int16_t)value, true) */
".Lo_62:\n"
"bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
"bl .Lfetch\n ldr r1, [sp, #" S(S0) "]\n orr r0, r1, r0, lsl #8\n"
"sxth r0, r0\n"
"add r0, r8, r0\n uxth r0, r0\n str r0, [sp, #" S(S0) "]\n"
"bl .Lidl\n"
/* pushWord(value, true) */
"ldr r0, [sp, #" S(S0) "]\n lsr r2, r0, #8\n uxtb r2, r2\n mov r1, r7\n bl .Lwr\n"
"subs r7, #1\n uxth r7, r7\n"
"ldr r0, [sp, #" S(FE) "]\n cbz r0, 1f\n and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
"1: bl .Lint\n"
"ldr r0, [sp, #" S(S0) "]\n uxtb r2, r0\n mov r1, r7\n bl .Lwr\n"
"subs r7, #1\n uxth r7, r7\n"
"ldr r0, [sp, #" S(FE) "]\n cbz r0, 1f\n and r7, r7, #0xFF\n orr r7, r7, #0x100\n"
"1:\n"
"b.w .Lnext\n"

/* ======================================================================== */
/* REP / SEP — Modify processor status flags                                */
/* ======================================================================== */

/* ---- REP (C2) — clear flag bits ---- */
/* cpu.c: val = readOpcode; checkInt; setFlags(getFlags & ~val); idle */
".Lo_C2:\n"
"bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
"bl .Lint\n"
/* new_flags = r9 & ~operand */
"ldr r0, [sp, #" S(S0) "]\n bic r0, r9, r0\n"
/* setFlags: E mode forces MF=1, XF=1 */
"ldr r1, [sp, #" S(FE) "]\n cbz r1, 1f\n"
"orr r0, r0, #0x30\n"
"1:\n"
"mov r9, r0\n"
/* if XF set: truncate X,Y */
"tst r9, #0x10\n beq.w 1f\n"
"uxtb r5, r5\n uxtb r6, r6\n"
"1: bl .Lidl\n"
"b.w .Lnext\n"

/* ---- SEP (E2) — set flag bits ---- */
/* cpu.c: val = readOpcode; checkInt; setFlags(getFlags | val); idle */
".Lo_E2:\n"
"bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
"bl .Lint\n"
"ldr r0, [sp, #" S(S0) "]\n orr r0, r9, r0\n"
"ldr r1, [sp, #" S(FE) "]\n cbz r1, 1f\n"
"orr r0, r0, #0x30\n"
"1:\n"
"mov r9, r0\n"
"tst r9, #0x10\n beq.w 1f\n"
"uxtb r5, r5\n uxtb r6, r6\n"
"1: bl .Lidl\n"
"b.w .Lnext\n"

/* ======================================================================== */
/* TSB / TRB — Test and Set/Reset Bits                                       */
/* ======================================================================== */

/* ---- TSB dp (04) ---- */
".Lo_04:\n"
"bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
"ldr r0, [sp, #" S(FDP) "]\n tst r0, #0xFF\n beq.w 1f\n bl .Lidl\n"
"1: ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
"add r1, r1, r0\n uxth r1, r1\n str r1, [sp, #" S(S1) "]\n"
"tst r9, #0x20\n beq.w .Ltsb04_16\n"
/* 8-bit: read value */
"bl .Lrd\n str r0, [sp, #" S(S3) "]\n"
"bl .Lidl\n"
/* Z = ((A & 0xFF) & value) == 0 */
"ldr r0, [sp, #" S(S3) "]\n uxtb r1, r4\n and r1, r1, r0\n"
"bic r9, r9, #0x02\n cmp r1, #0\n bne.w 1f\n orr r9, r9, #0x02\n"
"1: bl .Lint\n"
/* write: value | (A & 0xFF) */
"ldr r1, [sp, #" S(S1) "]\n ldr r0, [sp, #" S(S3) "]\n uxtb r2, r4\n orr r2, r0, r2\n"
"bl .Lwr\n b.w .Lnext\n"
".Ltsb04_16:\n"
/* 16-bit: readWord */
"ldr r1, [sp, #" S(S1) "]\n bl .Lrd\n str r0, [sp, #" S(S3) "]\n"
"ldr r1, [sp, #" S(S1) "]\n adds r1, #1\n uxth r1, r1\n"
"bl .Lrd\n ldr r1, [sp, #" S(S3) "]\n orr r0, r1, r0, lsl #8\n str r0, [sp, #" S(S3) "]\n"
"bl .Lidl\n"
/* Z = (A & value) == 0 */
"ldr r0, [sp, #" S(S3) "]\n uxth r1, r4\n and r1, r1, r0\n"
"bic r9, r9, #0x02\n cmp r1, #0\n bne.w 1f\n orr r9, r9, #0x02\n"
"1:\n"
/* writeWord reversed: high first, intCheck, low */
"ldr r0, [sp, #" S(S3) "]\n uxth r1, r4\n orr r0, r0, r1\n uxth r0, r0\n str r0, [sp, #" S(S3) "]\n"
"ldr r1, [sp, #" S(S1) "]\n adds r1, #1\n uxth r1, r1\n"
"ldr r0, [sp, #" S(S3) "]\n lsr r2, r0, #8\n uxtb r2, r2\n bl .Lwr\n"
"bl .Lint\n"
"ldr r1, [sp, #" S(S1) "]\n ldr r0, [sp, #" S(S3) "]\n uxtb r2, r0\n bl .Lwr\n"
"b.w .Lnext\n"

/* ---- TSB abs (0C) ---- */
".Lo_0C:\n"
"bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
"bl .Lfetch\n ldr r1, [sp, #" S(S0) "]\n orr r1, r1, r0, lsl #8\n"
"ldr r0, [sp, #" S(FDB) "]\n add r1, r1, r0, lsl #16\n"
"str r1, [sp, #" S(S1) "]\n"
"tst r9, #0x20\n beq.w .Ltsb0c_16\n"
/* 8-bit */
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

/* ---- TRB dp (14) ---- */
".Lo_14:\n"
"bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
"ldr r0, [sp, #" S(FDP) "]\n tst r0, #0xFF\n beq.w 1f\n bl .Lidl\n"
"1: ldr r1, [sp, #" S(FDP) "]\n ldr r0, [sp, #" S(S0) "]\n"
"add r1, r1, r0\n uxth r1, r1\n str r1, [sp, #" S(S1) "]\n"
"tst r9, #0x20\n beq.w .Ltrb14_16\n"
/* 8-bit */
"bl .Lrd\n str r0, [sp, #" S(S3) "]\n"
"bl .Lidl\n"
"ldr r0, [sp, #" S(S3) "]\n uxtb r1, r4\n and r1, r1, r0\n"
"bic r9, r9, #0x02\n cmp r1, #0\n bne.w 1f\n orr r9, r9, #0x02\n"
"1: bl .Lint\n"
/* write: value & ~(A & 0xFF) */
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
/* value & ~A */
"ldr r0, [sp, #" S(S3) "]\n uxth r1, r4\n bic r0, r0, r1\n uxth r0, r0\n str r0, [sp, #" S(S3) "]\n"
"ldr r1, [sp, #" S(S1) "]\n adds r1, #1\n uxth r1, r1\n"
"ldr r0, [sp, #" S(S3) "]\n lsr r2, r0, #8\n uxtb r2, r2\n bl .Lwr\n"
"bl .Lint\n"
"ldr r1, [sp, #" S(S1) "]\n ldr r0, [sp, #" S(S3) "]\n uxtb r2, r0\n bl .Lwr\n"
"b.w .Lnext\n"

/* ---- TRB abs (1C) ---- */
".Lo_1C:\n"
"bl .Lfetch\n str r0, [sp, #" S(S0) "]\n"
"bl .Lfetch\n ldr r1, [sp, #" S(S0) "]\n orr r1, r1, r0, lsl #8\n"
"ldr r0, [sp, #" S(FDB) "]\n add r1, r1, r0, lsl #16\n"
"str r1, [sp, #" S(S1) "]\n"
"tst r9, #0x20\n beq.w .Ltrb1c_16\n"
/* 8-bit */
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

/* ======================================================================== */
/* REGISTER TRANSFERS                                                        */
/* ======================================================================== */

/* ---- TCS (1B) — A -> SP, no flags ---- */
/* cpu.c: adrImp; sp = a.
 * adrImp = checkInt; if intWanted: read(K:PC) else idle.
 * Simplify: the opcode doesn't check intWanted for the read-vs-idle
 * distinction in our ASM path since we exit to .Lnext which re-enters
 * the main loop's interrupt check. Just do checkInt + idle.
 */
".Lo_1B:\n"
"bl .Lint\n bl .Lidl\n"
"mov r7, r4\n uxth r7, r7\n"
"b.w .Lnext\n"

/* ---- TSC (3B) — SP -> A, set Z/N always 16-bit ---- */
".Lo_3B:\n"
"bl .Lint\n bl .Lidl\n"
"mov r4, r7\n uxth r4, r4\n"
"mov r0, r4\n bl .Lzn16\n b.w .Lnext\n"

/* ---- TCD (5B) — A -> DP, set Z/N always 16-bit ---- */
".Lo_5B:\n"
"bl .Lint\n bl .Lidl\n"
"uxth r0, r4\n str r0, [sp, #" S(FDP) "]\n"
"bl .Lzn16\n b.w .Lnext\n"

/* ---- TDC (7B) — DP -> A, set Z/N always 16-bit ---- */
".Lo_7B:\n"
"bl .Lint\n bl .Lidl\n"
"ldr r4, [sp, #" S(FDP) "]\n uxth r4, r4\n"
"mov r0, r4\n bl .Lzn16\n b.w .Lnext\n"

/* ======================================================================== */
/* WAI / STP / WDM                                                           */
/* ======================================================================== */

/* ---- WAI (CB) — wait for interrupt ---- */
/* cpu.c: waiting = true; idle; idle */
".Lo_CB:\n"
"movs r0, #1\n strb r0, [r10, #" S(CPU_WAITING) "]\n"
"bl .Lidl\n bl .Lidl\n"
"b.w .Lnext\n"

/* ---- STP (DB) — stop processor ---- */
/* cpu.c: stopped = true; idle; idle */
".Lo_DB:\n"
"movs r0, #1\n strb r0, [r10, #" S(CPU_STOPPED) "]\n"
"bl .Lidl\n bl .Lidl\n"
"b.w .Lnext\n"

/* ---- WDM (42) — reserved 2-byte NOP ---- */
/* cpu.c: checkInt; readOpcode (discard) */
".Lo_42:\n"
"bl .Lint\n bl .Lfetch\n"
"b.w .Lnext\n"

".ltorg\n"
