/*
 * 256-entry jump table for the opcode dispatcher.
 * Each entry is a 4-byte Thumb address.
 * Unimplemented opcodes point to .Lfb (C fallback).
 *
 * Dispatch code (replaces the linear compare chain):
 *   "ldr  r12, =.Ljt\n"
 *   "add  r0, r12, r0, lsl #2\n"
 *   "ldr  r0, [r0]\n"
 *   "bx   r0\n"
 *
 * Table entries below. Update .Lfb to the correct label as opcodes
 * are implemented. Format: .word .Lo_XX for implemented, .word .Lfb+1 for fallback.
 *
 * NOTE: .Lfb+1 sets Thumb bit for indirect branch via bx.
 * Local labels (.Lo_XX) already have Thumb bit when referenced as .word in .thumb mode.
 */
    ".align 4\n"
    ".Ljt:\n"
    /* 0x00-0x0F */
    ".word .Lfb+1\n"     /* 00 BRK — groupD */
    ".word .Lfb+1\n"     /* 01 ORA idx — groupD */
    ".word .Lfb+1\n"     /* 02 COP — groupD */
    ".word .Lfb+1\n"     /* 03 ORA sr — groupD */
    ".word .Lfb+1\n"     /* 04 TSB dp — groupD */
    ".word .Lo_05+1\n"   /* 05 ORA dp — groupA */
    ".word .Lo_06+1\n"   /* 06 ASL dp — groupB */
    ".word .Lfb+1\n"     /* 07 ORA idl — groupD */
    ".word .Lfb+1\n"     /* 08 PHP — groupC */
    ".word .Lo_09+1\n"   /* 09 ORA imm — groupA */
    ".word .Lo_0A+1\n"   /* 0A ASL A — groupB */
    ".word .Lfb+1\n"     /* 0B PHD — groupC */
    ".word .Lfb+1\n"     /* 0C TSB abs — groupD */
    ".word .Lo_0D+1\n"   /* 0D ORA abs — groupA */
    ".word .Lo_0E+1\n"   /* 0E ASL abs — groupB */
    ".word .Lfb+1\n"     /* 0F ORA abl — groupD */
    /* 0x10-0x1F */
    ".word .Lo_10+1\n"   /* 10 BPL — existing */
    ".word .Lfb+1\n"     /* 11 ORA idy — groupD */
    ".word .Lfb+1\n"     /* 12 ORA idp — groupD */
    ".word .Lfb+1\n"     /* 13 ORA isy — groupD */
    ".word .Lfb+1\n"     /* 14 TRB dp — groupD */
    ".word .Lo_15+1\n"   /* 15 ORA dpx — groupA */
    ".word .Lo_16+1\n"   /* 16 ASL dpx — groupB */
    ".word .Lfb+1\n"     /* 17 ORA ily — groupD */
    ".word .Lo_18+1\n"   /* 18 CLC — groupA */
    ".word .Lo_19+1\n"   /* 19 ORA aby — groupA */
    ".word .Lo_1A+1\n"   /* 1A INC A — groupA */
    ".word .Lfb+1\n"     /* 1B TCS — groupC */
    ".word .Lfb+1\n"     /* 1C TRB abs — groupD */
    ".word .Lo_1D+1\n"   /* 1D ORA abx — groupA */
    ".word .Lo_1E+1\n"   /* 1E ASL abx — groupB */
    ".word .Lfb+1\n"     /* 1F ORA alx — groupD */
    /* 0x20-0x2F */
    ".word .Lo_20+1\n"   /* 20 JSR — existing */
    ".word .Lfb+1\n"     /* 21 AND idx — groupD */
    ".word .Lfb+1\n"     /* 22 JSL — groupD */
    ".word .Lfb+1\n"     /* 23 AND sr — groupD */
    ".word .Lfb+1\n"     /* 24 BIT dp — groupC */
    ".word .Lo_25+1\n"   /* 25 AND dp — groupA */
    ".word .Lo_26+1\n"   /* 26 ROL dp — groupB */
    ".word .Lfb+1\n"     /* 27 AND idl — groupD */
    ".word .Lfb+1\n"     /* 28 PLP — groupC */
    ".word .Lo_29+1\n"   /* 29 AND imm — groupA */
    ".word .Lo_2A+1\n"   /* 2A ROL A — groupB */
    ".word .Lfb+1\n"     /* 2B PLD — groupC */
    ".word .Lfb+1\n"     /* 2C BIT abs — groupC */
    ".word .Lo_2D+1\n"   /* 2D AND abs — groupA */
    ".word .Lo_2E+1\n"   /* 2E ROL abs — groupB */
    ".word .Lfb+1\n"     /* 2F AND abl — groupD */
    /* 0x30-0x3F */
    ".word .Lo_30+1\n"   /* 30 BMI — existing */
    ".word .Lfb+1\n"     /* 31 AND idy — groupD */
    ".word .Lfb+1\n"     /* 32 AND idp — groupD */
    ".word .Lfb+1\n"     /* 33 AND isy — groupD */
    ".word .Lfb+1\n"     /* 34 BIT dpx — groupC */
    ".word .Lo_35+1\n"   /* 35 AND dpx — groupA */
    ".word .Lo_36+1\n"   /* 36 ROL dpx — groupB */
    ".word .Lfb+1\n"     /* 37 AND ily — groupD */
    ".word .Lo_38+1\n"   /* 38 SEC — groupA */
    ".word .Lo_39+1\n"   /* 39 AND aby — groupA */
    ".word .Lo_3A+1\n"   /* 3A DEC A — groupA */
    ".word .Lfb+1\n"     /* 3B TSC — groupC */
    ".word .Lfb+1\n"     /* 3C BIT abx — groupC */
    ".word .Lo_3D+1\n"   /* 3D AND abx — groupA */
    ".word .Lo_3E+1\n"   /* 3E ROL abx — groupB */
    ".word .Lfb+1\n"     /* 3F AND alx — groupD */
    /* 0x40-0x4F */
    ".word .Lfb+1\n"     /* 40 RTI — groupD */
    ".word .Lfb+1\n"     /* 41 EOR idx — groupD */
    ".word .Lfb+1\n"     /* 42 WDM — groupD */
    ".word .Lfb+1\n"     /* 43 EOR sr — groupD */
    ".word .Lfb+1\n"     /* 44 MVP — groupD */
    ".word .Lo_45+1\n"   /* 45 EOR dp — groupA */
    ".word .Lo_46+1\n"   /* 46 LSR dp — groupB */
    ".word .Lfb+1\n"     /* 47 EOR idl — groupD */
    ".word .Lfb+1\n"     /* 48 PHA — groupC */
    ".word .Lo_49+1\n"   /* 49 EOR imm — groupA */
    ".word .Lo_4A+1\n"   /* 4A LSR A — groupB */
    ".word .Lfb+1\n"     /* 4B PHK — groupC */
    ".word .Lfb+1\n"     /* 4C JMP abs — groupC */
    ".word .Lo_4D+1\n"   /* 4D EOR abs — groupA */
    ".word .Lo_4E+1\n"   /* 4E LSR abs — groupB */
    ".word .Lfb+1\n"     /* 4F EOR abl — groupD */
    /* 0x50-0x5F */
    ".word .Lfb+1\n"     /* 50 BVC — groupD */
    ".word .Lfb+1\n"     /* 51 EOR idy — groupD */
    ".word .Lfb+1\n"     /* 52 EOR idp — groupD */
    ".word .Lfb+1\n"     /* 53 EOR isy — groupD */
    ".word .Lfb+1\n"     /* 54 MVN — groupD */
    ".word .Lo_55+1\n"   /* 55 EOR dpx — groupA */
    ".word .Lo_56+1\n"   /* 56 LSR dpx — groupB */
    ".word .Lfb+1\n"     /* 57 EOR ily — groupD */
    ".word .Lo_58+1\n"   /* 58 CLI — groupA */
    ".word .Lo_59+1\n"   /* 59 EOR aby — groupA */
    ".word .Lfb+1\n"     /* 5A PHY — groupC */
    ".word .Lfb+1\n"     /* 5B TCD — groupC */
    ".word .Lfb+1\n"     /* 5C JML — groupD */
    ".word .Lo_5D+1\n"   /* 5D EOR abx — groupA */
    ".word .Lo_5E+1\n"   /* 5E LSR abx — groupB */
    ".word .Lfb+1\n"     /* 5F EOR alx — groupD */
    /* 0x60-0x6F */
    ".word .Lo_60+1\n"   /* 60 RTS — existing */
    ".word .Lfb+1\n"     /* 61 ADC idx — groupD */
    ".word .Lfb+1\n"     /* 62 PER — groupD */
    ".word .Lfb+1\n"     /* 63 ADC sr — groupD */
    ".word .Lfb+1\n"     /* 64 STZ dp — groupD */
    ".word .Lo_65+1\n"   /* 65 ADC dp — existing */
    ".word .Lo_66+1\n"   /* 66 ROR dp — groupB */
    ".word .Lfb+1\n"     /* 67 ADC idl — groupD */
    ".word .Lfb+1\n"     /* 68 PLA — groupC */
    ".word .Lo_69+1\n"   /* 69 ADC imm — existing */
    ".word .Lo_6A+1\n"   /* 6A ROR A — groupB */
    ".word .Lfb+1\n"     /* 6B RTL — groupD */
    ".word .Lfb+1\n"     /* 6C JMP ind — groupC */
    ".word .Lo_6D+1\n"   /* 6D ADC abs — MISSING — fallback */
    ".word .Lo_6E+1\n"   /* 6E ROR abs — groupB */
    ".word .Lfb+1\n"     /* 6F ADC abl — groupD */
    /* 0x70-0x7F */
    ".word .Lfb+1\n"     /* 70 BVS — groupD */
    ".word .Lfb+1\n"     /* 71 ADC idy — groupD */
    ".word .Lfb+1\n"     /* 72 ADC idp — groupD */
    ".word .Lfb+1\n"     /* 73 ADC isy — groupD */
    ".word .Lfb+1\n"     /* 74 STZ dpx — groupD */
    ".word .Lfb+1\n"     /* 75 ADC dpx — groupD */
    ".word .Lo_76+1\n"   /* 76 ROR dpx — groupB */
    ".word .Lfb+1\n"     /* 77 ADC ily — groupD */
    ".word .Lo_78+1\n"   /* 78 SEI — groupA */
    ".word .Lfb+1\n"     /* 79 ADC aby — groupD */
    ".word .Lfb+1\n"     /* 7A PLY — groupC */
    ".word .Lfb+1\n"     /* 7B TDC — groupC */
    ".word .Lfb+1\n"     /* 7C JMP iax — groupC */
    ".word .Lfb+1\n"     /* 7D ADC abx — groupD */
    ".word .Lo_7E+1\n"   /* 7E ROR abx — groupB */
    ".word .Lfb+1\n"     /* 7F ADC alx — groupD */
    /* 0x80-0x8F */
    ".word .Lfb+1\n"     /* 80 BRA — groupD */
    ".word .Lfb+1\n"     /* 81 STA idx — groupD */
    ".word .Lfb+1\n"     /* 82 BRL — groupD */
    ".word .Lfb+1\n"     /* 83 STA sr — groupD */
    ".word .Lfb+1\n"     /* 84 STY dp — groupD */
    ".word .Lo_85+1\n"   /* 85 STA dp — existing */
    ".word .Lfb+1\n"     /* 86 STX dp — groupD */
    ".word .Lfb+1\n"     /* 87 STA idl — groupD */
    ".word .Lo_88+1\n"   /* 88 DEY — groupA */
    ".word .Lfb+1\n"     /* 89 BIT imm — groupC */
    ".word .Lo_8A+1\n"   /* 8A TXA — groupA */
    ".word .Lfb+1\n"     /* 8B PHB — groupC */
    ".word .Lfb+1\n"     /* 8C STY abs — groupD */
    ".word .Lo_8D+1\n"   /* 8D STA abs — existing */
    ".word .Lfb+1\n"     /* 8E STX abs — groupD */
    ".word .Lfb+1\n"     /* 8F STA abl — groupD */
    /* 0x90-0x9F */
    ".word .Lo_90+1\n"   /* 90 BCC — existing */
    ".word .Lfb+1\n"     /* 91 STA idy — groupD */
    ".word .Lfb+1\n"     /* 92 STA idp — groupD */
    ".word .Lfb+1\n"     /* 93 STA isy — groupD */
    ".word .Lfb+1\n"     /* 94 STY dpx — groupD */
    ".word .Lo_95+1\n"   /* 95 STA dpx — existing */
    ".word .Lfb+1\n"     /* 96 STX dpy — groupD */
    ".word .Lfb+1\n"     /* 97 STA ily — groupD */
    ".word .Lo_98+1\n"   /* 98 TYA — groupA */
    ".word .Lo_99+1\n"   /* 99 STA aby — existing */
    ".word .Lo_9A+1\n"   /* 9A TXS — groupA */
    ".word .Lo_9B+1\n"   /* 9B TXY — groupA */
    ".word .Lfb+1\n"     /* 9C STZ abs — groupD */
    ".word .Lo_9D+1\n"   /* 9D STA abx — existing */
    ".word .Lfb+1\n"     /* 9E STZ abx — groupD */
    ".word .Lfb+1\n"     /* 9F STA alx — groupD */
    /* 0xA0-0xAF */
    ".word .Lo_A0+1\n"   /* A0 LDY imm — existing */
    ".word .Lfb+1\n"     /* A1 LDA idx — groupD */
    ".word .Lo_A2+1\n"   /* A2 LDX imm — existing */
    ".word .Lfb+1\n"     /* A3 LDA sr — groupD */
    ".word .Lo_A4+1\n"   /* A4 LDY dp — existing */
    ".word .Lo_A5+1\n"   /* A5 LDA dp — existing */
    ".word .Lo_A6+1\n"   /* A6 LDX dp — existing */
    ".word .Lfb+1\n"     /* A7 LDA idl — groupD */
    ".word .Lo_A8+1\n"   /* A8 TAY — groupA */
    ".word .Lo_A9+1\n"   /* A9 LDA imm — existing */
    ".word .Lo_AA+1\n"   /* AA TAX — groupA */
    ".word .Lfb+1\n"     /* AB PLB — groupC */
    ".word .Lo_AC+1\n"   /* AC LDY abs — existing */
    ".word .Lo_AD+1\n"   /* AD LDA abs — existing */
    ".word .Lo_AE+1\n"   /* AE LDX abs — existing */
    ".word .Lfb+1\n"     /* AF LDA abl — groupD */
    /* 0xB0-0xBF */
    ".word .Lo_B0+1\n"   /* B0 BCS — existing */
    ".word .Lfb+1\n"     /* B1 LDA idy — groupD */
    ".word .Lfb+1\n"     /* B2 LDA idp — groupD */
    ".word .Lfb+1\n"     /* B3 LDA isy — groupD */
    ".word .Lfb+1\n"     /* B4 LDY dpx — groupD */
    ".word .Lo_B5+1\n"   /* B5 LDA dpx — existing */
    ".word .Lfb+1\n"     /* B6 LDX dpy — groupD */
    ".word .Lfb+1\n"     /* B7 LDA ily — groupD */
    ".word .Lo_B8+1\n"   /* B8 CLV — groupA */
    ".word .Lo_B9+1\n"   /* B9 LDA aby — existing */
    ".word .Lo_BA+1\n"   /* BA TSX — groupA */
    ".word .Lo_BB+1\n"   /* BB TYX — groupA */
    ".word .Lfb+1\n"     /* BC LDY abx — groupD */
    ".word .Lo_BD+1\n"   /* BD LDA abx — existing */
    ".word .Lfb+1\n"     /* BE LDX aby — groupD */
    ".word .Lfb+1\n"     /* BF LDA alx — groupD */
    /* 0xC0-0xCF */
    ".word .Lo_C0+1\n"   /* C0 CPY imm — groupB */
    ".word .Lfb+1\n"     /* C1 CMP idx — groupD */
    ".word .Lfb+1\n"     /* C2 REP — groupD */
    ".word .Lfb+1\n"     /* C3 CMP sr — groupD */
    ".word .Lo_C4+1\n"   /* C4 CPY dp — groupB */
    ".word .Lo_C5+1\n"   /* C5 CMP dp — existing */
    ".word .Lo_C6+1\n"   /* C6 DEC dp — groupB */
    ".word .Lfb+1\n"     /* C7 CMP idl — groupD */
    ".word .Lo_C8+1\n"   /* C8 INY — groupA */
    ".word .Lo_C9+1\n"   /* C9 CMP imm — existing */
    ".word .Lo_CA+1\n"   /* CA DEX — groupA */
    ".word .Lfb+1\n"     /* CB WAI — groupD */
    ".word .Lo_CC+1\n"   /* CC CPY abs — groupB */
    ".word .Lo_CD+1\n"   /* CD CMP abs — existing */
    ".word .Lo_CE+1\n"   /* CE DEC abs — groupB */
    ".word .Lfb+1\n"     /* CF CMP abl — groupD */
    /* 0xD0-0xDF */
    ".word .Lo_D0+1\n"   /* D0 BNE — existing */
    ".word .Lfb+1\n"     /* D1 CMP idy — groupD */
    ".word .Lfb+1\n"     /* D2 CMP idp — groupD */
    ".word .Lfb+1\n"     /* D3 CMP isy — groupD */
    ".word .Lfb+1\n"     /* D4 PEI — groupD */
    ".word .Lfb+1\n"     /* D5 CMP dpx — groupD */
    ".word .Lo_D6+1\n"   /* D6 DEC dpx — groupB */
    ".word .Lfb+1\n"     /* D7 CMP ily — groupD */
    ".word .Lo_D8+1\n"   /* D8 CLD — groupA */
    ".word .Lfb+1\n"     /* D9 CMP aby — groupD */
    ".word .Lfb+1\n"     /* DA PHX — groupC */
    ".word .Lfb+1\n"     /* DB STP — groupD */
    ".word .Lfb+1\n"     /* DC JML ial — groupD */
    ".word .Lfb+1\n"     /* DD CMP abx — groupD */
    ".word .Lo_DE+1\n"   /* DE DEC abx — groupB */
    ".word .Lfb+1\n"     /* DF CMP alx — groupD */
    /* 0xE0-0xEF */
    ".word .Lo_E0+1\n"   /* E0 CPX imm — groupB */
    ".word .Lfb+1\n"     /* E1 SBC idx — groupD */
    ".word .Lfb+1\n"     /* E2 SEP — groupD */
    ".word .Lfb+1\n"     /* E3 SBC sr — groupD */
    ".word .Lo_E4+1\n"   /* E4 CPX dp — groupB */
    ".word .Lo_E5+1\n"   /* E5 SBC dp — groupB */
    ".word .Lo_E6+1\n"   /* E6 INC dp — groupB */
    ".word .Lfb+1\n"     /* E7 SBC idl — groupD */
    ".word .Lo_E8+1\n"   /* E8 INX — groupA */
    ".word .Lo_E9+1\n"   /* E9 SBC imm — groupB */
    ".word .Lo_EA+1\n"   /* EA NOP — groupA */
    ".word .Lfb+1\n"     /* EB XBA — groupC */
    ".word .Lo_EC+1\n"   /* EC CPX abs — groupB */
    ".word .Lo_ED+1\n"   /* ED SBC abs — groupB */
    ".word .Lo_EE+1\n"   /* EE INC abs — groupB */
    ".word .Lfb+1\n"     /* EF SBC abl — groupD */
    /* 0xF0-0xFF */
    ".word .Lo_F0+1\n"   /* F0 BEQ — existing */
    ".word .Lfb+1\n"     /* F1 SBC idy — groupD */
    ".word .Lfb+1\n"     /* F2 SBC idp — groupD */
    ".word .Lfb+1\n"     /* F3 SBC isy — groupD */
    ".word .Lfb+1\n"     /* F4 PEA — groupD */
    ".word .Lo_F5+1\n"   /* F5 SBC dpx — groupB */
    ".word .Lo_F6+1\n"   /* F6 INC dpx — groupB */
    ".word .Lfb+1\n"     /* F7 SBC ily — groupD */
    ".word .Lo_F8+1\n"   /* F8 SED — groupA */
    ".word .Lo_F9+1\n"   /* F9 SBC aby — groupB */
    ".word .Lfb+1\n"     /* FA PLX — groupC */
    ".word .Lfb+1\n"     /* FB XCE — groupC */
    ".word .Lfb+1\n"     /* FC JSR iax — groupD */
    ".word .Lo_FD+1\n"   /* FD SBC abx — groupB */
    ".word .Lo_FE+1\n"   /* FE INC abx — groupB */
    ".word .Lfb+1\n"     /* FF SBC alx — groupD */
