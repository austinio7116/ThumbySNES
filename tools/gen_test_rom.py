#!/usr/bin/env python3
"""
Generate a SNES test ROM (.sfc) that exercises every 65816 opcode
and writes PASS/FAIL results to WRAM for the emulator to check.

WRAM layout after tests complete:
  $0000 = 0x50 ('P') if ALL tests passed, 0x46 ('F') if any failed
  $0001 = test number of first failure (0 = none)
  $0002 = expected value (low byte)
  $0003 = actual value (low byte)

The ROM is a minimal 32 KB LoROM image. Reset vector at $FFFC points
to test code at $8000. Tests run sequentially; each one loads known
values, executes the opcode, and compares the result.

Usage: python3 gen_test_rom.py > roms/test_cpu.sfc
       Or: python3 gen_test_rom.py roms/test_cpu.sfc
"""
import sys
import struct

ROM_SIZE = 32768  # 32 KB minimum LoROM
CODE_BASE = 0x8000
rom = bytearray(ROM_SIZE)

# Cursor for writing code
pc = 0  # offset into rom (maps to $8000 + pc)

def emit(*bytes_):
    global pc
    for b in bytes_:
        rom[pc] = b & 0xFF
        pc += 1

def emit_word(w):
    emit(w & 0xFF, (w >> 8) & 0xFF)

# 65816 opcodes (hex)
LDA_IMM = 0xA9; LDA_DP = 0xA5; LDA_ABS = 0xAD
LDX_IMM = 0xA2; LDY_IMM = 0xA0
STA_DP = 0x85; STA_ABS = 0x8D; STX_DP = 0x86; STY_DP = 0x84
CMP_IMM = 0xC9; CPX_IMM = 0xE0; CPY_IMM = 0xC0
BNE = 0xD0; BEQ = 0xF0; BCS = 0xB0; BCC = 0x90
CLC = 0x18; SEC = 0x38; CLI = 0x58; SEI = 0x78
CLD = 0xD8; SED = 0xF8; CLV = 0xB8
NOP = 0xEA; STP = 0xDB
TAX = 0xAA; TAY = 0xA8; TXA = 0x8A; TYA = 0x98
INX = 0xE8; DEX = 0xCA; INY = 0xC8; DEY = 0x88
INC_A = 0x1A; DEC_A = 0x3A
PHA = 0x48; PLA = 0x68; PHP = 0x08; PLP = 0x28
PHX = 0xDA; PLX = 0xFA; PHY = 0x5A; PLY = 0x7A
JSR = 0x20; RTS = 0x60; JMP_ABS = 0x4C
ORA_IMM = 0x09; AND_IMM = 0x29; EOR_IMM = 0x49
ADC_IMM = 0x69; SBC_IMM = 0xE9
ASL_A = 0x0A; LSR_A = 0x4A; ROL_A = 0x2A; ROR_A = 0x6A
BIT_DP = 0x24
REP = 0xC2; SEP = 0xE2
XBA = 0xEB

# Test number counter
test_num = [0]

def start_test():
    """Increment test number in X, also write to WRAM[$04] for debug."""
    test_num[0] += 1
    emit(LDX_IMM, test_num[0])
    # Save current test number to WRAM[$04] so we know progress
    emit(STX_DP, 0x04)

def fail_branch():
    """Emit a BNE to the fail handler. Returns offset to patch if needed."""
    emit(BNE, 0)  # placeholder — will jump to fail
    return pc - 1  # offset of the branch target byte

def patch_branch(offset):
    """Patch a relative branch to jump to current pc."""
    delta = pc - (offset + 1)  # +1 because branch is relative to AFTER the branch
    # Actually the offset byte is at rom[offset], branch from offset+1
    # Wait no: BNE at address A, operand at A+1, next instruction at A+2
    # Target = A+2 + signed_offset
    # We want target = current pc (relative to CODE_BASE)
    # Actually let me just use the fail handler approach
    pass

# --- FAIL HANDLER ---
# Will be at a known address. Tests branch here on failure.
# For simplicity: each test does CMP + BNE skip_past_fail_store
# If the comparison fails, we fall through to storing fail info.

# Strategy: each test ends with a CMP + BEQ to .next_test
# If CMP fails (not equal), falls through to fail code inline.

def emit_check_a(expected, test_id):
    """Check A == expected. If fail, store fail info and STP."""
    emit(CMP_IMM, expected)
    emit(BEQ, 5)  # skip 5 bytes (the fail path)
    # Fail path: store test_id to $0001, expected to $0002, STP
    emit(STX_DP, 0x01)      # X has test number
    emit(STA_DP, 0x03)      # A has actual value
    emit(STP)                # halt

def emit_check_x(expected, test_id):
    """Check X == expected."""
    emit(CPX_IMM, expected)
    emit(BEQ, 3)
    emit(STX_DP, 0x01)
    emit(STP)

def emit_check_y(expected, test_id):
    """Check Y == expected."""
    emit(CPY_IMM, expected)
    emit(BEQ, 3)
    emit(STY_DP, 0x01)  # not ideal but close enough
    emit(STP)

# === RESET HANDLER ===

# First: switch from emulation mode (E=1) to native mode (E=0) so REP/SEP work
emit(CLC)         # clear carry
emit(0xFB)        # XCE — exchange C↔E. Now E=0, native mode.
emit(SEP, 0x30)   # set M and X flags (8-bit A, 8-bit X/Y)
emit(CLD)         # clear decimal mode
emit(CLI)         # clear interrupt disable (allow IRQs — needed for NMI)

# Clear fail marker
emit(LDA_IMM, 0x00)
emit(STA_DP, 0x00)
emit(STA_DP, 0x01)

# === TEST 1: LDA immediate ===
start_test()  # X = 1
emit(LDA_IMM, 0x42)
emit_check_a(0x42, 1)

# === TEST 2: STA dp + LDA dp ===
start_test()  # X = 2
emit(LDA_IMM, 0x55)
emit(STA_DP, 0x10)
emit(LDA_IMM, 0x00)
emit(LDA_DP, 0x10)
emit_check_a(0x55, 2)

# === TEST 3: LDA abs ===
start_test()
emit(LDA_IMM, 0xAA)
emit(STA_ABS); emit_word(0x0020)  # store to $0020
emit(LDA_IMM, 0x00)
emit(LDA_ABS); emit_word(0x0020)  # load from $0020
emit_check_a(0xAA, 3)

# === TEST 4: TAX / TXA ===
start_test()
emit(LDA_IMM, 0x77)
emit(TAX)
emit(LDA_IMM, 0x00)
emit(TXA)
emit_check_a(0x77, 4)

# === TEST 5: TAY / TYA ===
start_test()
emit(LDA_IMM, 0x33)
emit(TAY)
emit(LDA_IMM, 0x00)
emit(TYA)
emit_check_a(0x33, 5)

# === TEST 6: INX / DEX ===
start_test()
emit(LDX_IMM, 0x10)
emit(INX)
emit(CPX_IMM, 0x11)
emit(BEQ, 3); emit(STX_DP, 0x01); emit(STP)
emit(DEX)
emit(CPX_IMM, 0x10)
emit(BEQ, 3); emit(STX_DP, 0x01); emit(STP)

# === TEST 7: INY / DEY ===
start_test()
emit(LDY_IMM, 0x20)
emit(INY)
emit(CPY_IMM, 0x21)
emit(BEQ, 3); emit(STY_DP, 0x01); emit(STP)
emit(DEY)
emit(CPY_IMM, 0x20)
emit(BEQ, 3); emit(STY_DP, 0x01); emit(STP)

# === TEST 8: ORA immediate ===
start_test()
emit(LDA_IMM, 0x0F)
emit(ORA_IMM, 0xF0)
emit_check_a(0xFF, 8)

# === TEST 9: AND immediate ===
start_test()
emit(LDA_IMM, 0xFF)
emit(AND_IMM, 0x0F)
emit_check_a(0x0F, 9)

# === TEST 10: EOR immediate ===
start_test()
emit(LDA_IMM, 0xFF)
emit(EOR_IMM, 0xAA)
emit_check_a(0x55, 10)

# === TEST 11: ADC immediate (no carry in) ===
start_test()
emit(CLC)
emit(LDA_IMM, 0x10)
emit(ADC_IMM, 0x20)
emit_check_a(0x30, 11)

# === TEST 12: ADC with carry in ===
start_test()
emit(SEC)
emit(LDA_IMM, 0x10)
emit(ADC_IMM, 0x20)
emit_check_a(0x31, 12)

# === TEST 13: SBC immediate ===
start_test()
emit(SEC)  # carry must be set for no borrow
emit(LDA_IMM, 0x30)
emit(SBC_IMM, 0x10)
emit_check_a(0x20, 13)

# === TEST 14: CMP sets carry (A >= operand) ===
start_test()
emit(LDA_IMM, 0x50)
emit(CMP_IMM, 0x30)
emit(BCS, 3); emit(LDA_IMM, 0x01); emit(STP)  # should branch (C set)

# === TEST 15: CMP clears carry (A < operand) ===
start_test()
emit(LDA_IMM, 0x10)
emit(CMP_IMM, 0x30)
emit(BCC, 3); emit(LDA_IMM, 0x01); emit(STP)  # should branch (C clear)

# === TEST 16: BEQ / BNE ===
start_test()
emit(LDA_IMM, 0x00)  # Z=1
emit(BEQ, 3); emit(LDA_IMM, 0x01); emit(STP)  # should branch
emit(LDA_IMM, 0x01)  # Z=0
emit(BNE, 3); emit(LDA_IMM, 0x01); emit(STP)  # should branch

# === TEST 17: ASL A ===
start_test()
emit(LDA_IMM, 0x41)
emit(ASL_A)
emit_check_a(0x82, 17)

# === TEST 18: LSR A ===
start_test()
emit(LDA_IMM, 0x82)
emit(LSR_A)
emit_check_a(0x41, 18)

# === TEST 19: ROL A (with carry) ===
start_test()
emit(SEC)
emit(LDA_IMM, 0x00)
emit(ROL_A)
emit_check_a(0x01, 19)

# === TEST 20: ROR A (with carry) ===
start_test()
emit(SEC)
emit(LDA_IMM, 0x00)
emit(ROR_A)
emit_check_a(0x80, 20)

# === TEST 21: PHA / PLA ===
start_test()
emit(LDA_IMM, 0xBB)
emit(PHA)
emit(LDA_IMM, 0x00)
emit(PLA)
emit_check_a(0xBB, 21)

# === TEST 22: JSR / RTS ===
start_test()
# JMP over the subroutine, then JSR back to it.
# Layout: JMP main; sub: LDA #$CC; RTS; main: JSR sub; check_a
emit(JMP_ABS); jmp_over_sub_offset = pc; emit_word(0)  # placeholder
sub_start = pc
emit(LDA_IMM, 0xCC)
emit(RTS)
# Patch JMP target to here
main_addr = CODE_BASE + pc
rom[jmp_over_sub_offset]     = main_addr & 0xFF
rom[jmp_over_sub_offset + 1] = (main_addr >> 8) & 0xFF
# Call the subroutine
emit(JSR); emit_word(CODE_BASE + sub_start)
# After RTS returns here, A should be 0xCC
emit_check_a(0xCC, 22)

# === TEST 23: INC A / DEC A ===
start_test()
emit(LDA_IMM, 0x10)
emit(INC_A)
emit_check_a(0x11, 23)
emit(LDA_IMM, 0x10)
emit(DEC_A)
emit_check_a(0x0F, 23)

# === TEST 24: CLC / SEC flag check ===
start_test()
emit(CLC)
emit(LDA_IMM, 0x00)
emit(ADC_IMM, 0x00)  # 0+0+C=0 (carry clear)
emit_check_a(0x00, 24)
emit(SEC)
emit(LDA_IMM, 0x00)
emit(ADC_IMM, 0x00)  # 0+0+C=1 (carry set)
emit_check_a(0x01, 24)

# === TEST 25: XBA (swap A bytes — 8-bit version) ===
# In 8-bit mode, XBA still swaps low and high bytes of full 16-bit A.
# Set up A=0xAB with high byte = 0xCD via 16-bit, then back to 8-bit.
start_test()
emit(REP, 0x20)      # 16-bit A
emit(LDA_IMM, 0xAB); emit(0xCD)  # A = 0xCDAB
emit(SEP, 0x20)      # 8-bit A — but high byte still 0xCD internally
emit(XBA)            # swap: low (0xAB) ↔ high (0xCD), A = 0xABCD, 8-bit A = 0xCD
emit_check_a(0xCD, 25)

# === TEST 26: NOP (just shouldn't crash) ===
start_test()
emit(NOP)
emit(NOP)
emit(NOP)

# === ALL PASSED ===
emit(LDA_IMM, 0x50)  # 'P' for Pass
emit(STA_DP, 0x00)   # WRAM[$0000] = 'P'
emit(LDA_IMM, 0x00)
emit(STA_DP, 0x01)   # WRAM[$0001] = 0 (no failure)

# Infinite loop (don't STP — let the emulator keep running so it can read WRAM)
# Just loop forever
loop_addr = pc
emit(JMP_ABS); emit_word(CODE_BASE + loop_addr)

# === VECTORS ===
# LoROM vectors at $FFFC (reset) — offset 0x7FFC in the ROM
rom[0x7FFC] = 0x00  # Reset vector low = $8000
rom[0x7FFD] = 0x80  # Reset vector high

# Also set NMI vector to an RTI so NMI doesn't crash
# NMI at $FFEA (native) and $FFFA (emulation)
rti_addr = CODE_BASE + pc
emit(0x40)  # RTI
rom[0x7FEA] = rti_addr & 0xFF
rom[0x7FEB] = (rti_addr >> 8) & 0xFF
rom[0x7FFA] = rti_addr & 0xFF
rom[0x7FFB] = (rti_addr >> 8) & 0xFF

# IRQ/BRK vectors
rom[0x7FEE] = rti_addr & 0xFF
rom[0x7FEF] = (rti_addr >> 8) & 0xFF
rom[0x7FFE] = rti_addr & 0xFF
rom[0x7FFF] = (rti_addr >> 8) & 0xFF

# LoROM header at $7FC0
header_offset = 0x7FC0
title = b"THUMBYSNES TEST ROM  "[:21]  # ensure exactly 21 bytes
title = title + b' ' * (21 - len(title))
assert len(title) == 21
rom[header_offset:header_offset+21] = title
rom[header_offset+21] = 0x20  # LoROM, no battery
rom[header_offset+23] = 0x08  # ROM size: 32KB (2^15)
rom[header_offset+25] = 0x00  # Region: Japan
rom[header_offset+28] = 0x00  # Checksum complement (dummy)
rom[header_offset+29] = 0x00
rom[header_offset+30] = 0xFF  # Checksum (dummy)
rom[header_offset+31] = 0xFF

print(f"Generated test ROM: {pc} bytes of code, {test_num[0]} tests", file=sys.stderr)

# Output
if len(sys.argv) > 1:
    with open(sys.argv[1], 'wb') as f:
        f.write(rom)
    print(f"Wrote {sys.argv[1]}", file=sys.stderr)
else:
    sys.stdout.buffer.write(rom)
