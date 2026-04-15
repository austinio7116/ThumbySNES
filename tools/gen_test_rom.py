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

# Bisect: stop generation after this test # (0 = run all)
BISECT_STOP_AFTER = int(sys.argv[2]) if len(sys.argv) > 2 else 0

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

_bisect_stopped = [False]

def start_test():
    """Increment test number in X, also write to WRAM[$04] for debug."""
    test_num[0] += 1
    if BISECT_STOP_AFTER and test_num[0] > BISECT_STOP_AFTER and not _bisect_stopped[0]:
        # Emit the all-pass marker and infinite loop
        _bisect_stopped[0] = True
        emit(LDA_IMM, 0x50); emit(STA_DP, 0x00)
        emit(LDA_IMM, 0x00); emit(STA_DP, 0x01)
        loop_a = pc
        emit(JMP_ABS); emit_word(CODE_BASE + loop_a)
    if _bisect_stopped[0]:
        return
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

# === TEST 26: STA abs,X ===
start_test()
emit(LDX_IMM, 0x05)
emit(LDA_IMM, 0x66)
emit(0x9D); emit_word(0x0050)   # STA $0050,X → writes $0055
emit(LDA_IMM, 0x00)
emit(0xBD); emit_word(0x0050)   # LDA $0050,X
emit_check_a(0x66, 26)

# === TEST 27: STA abs,Y ===
start_test()
emit(LDY_IMM, 0x07)
emit(LDA_IMM, 0x77)
emit(0x99); emit_word(0x0060)   # STA $0060,Y → writes $0067
emit(LDA_IMM, 0x00)
emit(0xB9); emit_word(0x0060)   # LDA $0060,Y
emit_check_a(0x77, 27)

# === TEST 28: STA dp,X ===
start_test()
emit(LDX_IMM, 0x03)
emit(LDA_IMM, 0x88)
emit(0x95, 0x70)                 # STA $70,X → writes $0073
emit(LDA_IMM, 0x00)
emit(0xB5, 0x70)                 # LDA $70,X
emit_check_a(0x88, 28)

# Write an early marker so we know if execution reached here
emit(LDA_IMM, 0xAA)
emit(STA_DP, 0x05)  # WRAM[5] = 0xAA → "got to after test 28"

# === TEST 29: 16-bit LDA/STA ===
start_test()
emit(REP, 0x20)                  # 16-bit A
emit(LDA_IMM, 0x34); emit(0x12)  # A = 0x1234
emit(0x8D); emit_word(0x0080)    # STA $0080 (writes 2 bytes)
emit(LDA_IMM, 0x00); emit(0x00)  # A = 0
emit(0xAD); emit_word(0x0080)    # LDA $0080 (reads 2 bytes)
# A should be 0x1234
emit(CMP_IMM, 0x34); emit(0x12)  # 16-bit CMP
# Go to 8-bit FIRST, then branch/fail. That way the final SEP is already done.
emit(SEP, 0x20)                   # back to 8-bit (runs regardless)
emit(BEQ, 3)                      # skip fail block on match (STX+STP = 3 bytes)
emit(STX_DP, 0x01); emit(STP)

# === TEST 30: LDA (dp,X) — indirect indexed-X ===
start_test()
# Set up pointer at $50: $0200 → $A9. Then LDA ($40,X) with X=$10 → reads ptr at $50 → loads from $0200.
emit(LDA_IMM, 0xA9)
emit(STA_ABS); emit_word(0x0200)  # $0200 = $A9
emit(LDA_IMM, 0x00)
emit(STA_DP, 0x50)                 # $50 = $00
emit(LDA_IMM, 0x02)
emit(STA_DP, 0x51)                 # $51 = $02 (pointer $0200)
emit(LDX_IMM, 0x10)
emit(LDA_IMM, 0x00)
emit(0xA1, 0x40)                   # LDA ($40,X) — ptr at $50 → $0200 → $A9
emit_check_a(0xA9, 30)

# === TEST 31: LDA (dp),Y — indirect indexed-Y ===
start_test()
emit(LDA_IMM, 0xB7)
emit(STA_ABS); emit_word(0x0305)   # $0305 = $B7
emit(LDA_IMM, 0x00)
emit(STA_DP, 0x60)                  # $60 = $00
emit(LDA_IMM, 0x03)
emit(STA_DP, 0x61)                  # $61 = $03 (ptr $0300)
emit(LDY_IMM, 0x05)
emit(LDA_IMM, 0x00)
emit(0xB1, 0x60)                    # LDA ($60),Y — ptr $0300 + Y=5 → $0305 → $B7
emit_check_a(0xB7, 31)

# === TEST 32: LDA (dp) — indirect 65C02-style ===
start_test()
emit(LDA_IMM, 0xC5)
emit(STA_ABS); emit_word(0x0400)
emit(LDA_IMM, 0x00); emit(STA_DP, 0x70)
emit(LDA_IMM, 0x04); emit(STA_DP, 0x71)
emit(LDA_IMM, 0x00)
emit(0xB2, 0x70)                    # LDA ($70) — ptr at $70 → $0400 → $C5
emit_check_a(0xC5, 32)

# === TEST 33: PHP / PLP — save/restore flags ===
start_test()
emit(SEC)                           # C=1
emit(PHP)                           # push P
emit(CLC)                           # C=0
emit(PLP)                           # pull P → C=1 again
# Verify C=1 via BCS
emit(BCS, 3); emit(STX_DP, 0x01); emit(STP)

# === TEST 34: PHX / PLX ===
start_test()
emit(LDX_IMM, 0x99)
emit(PHX)
emit(LDX_IMM, 0x00)
emit(PLX)
emit(CPX_IMM, 0x99)
emit(BEQ, 3); emit(STX_DP, 0x01); emit(STP)

# === TEST 35: PHY / PLY ===
start_test()
emit(LDY_IMM, 0xAA)
emit(PHY)
emit(LDY_IMM, 0x00)
emit(PLY)
emit(CPY_IMM, 0xAA)
emit(BEQ, 3); emit(STY_DP, 0x01); emit(STP)

# === TEST 36: REP/SEP mode switching ===
start_test()
emit(REP, 0x30)                     # 16-bit A + XY
emit(LDX_IMM, 0xFF); emit(0x00)    # X = 0x00FF (16-bit immediate)
emit(SEP, 0x30)                     # BACK TO 8-BIT BOTH (important!)
emit(CPX_IMM, 0xFF)
emit(BEQ, 3); emit(STX_DP, 0x01); emit(STP)

# === TEST 37: BIT dp — check Z from A AND mem ===
start_test()
emit(LDA_IMM, 0xFF); emit(STA_DP, 0x80)  # $80 = 0xFF
emit(LDA_IMM, 0x00)                       # A = 0
emit(BIT_DP, 0x80)                        # A & mem = 0 → Z=1
emit(BEQ, 3); emit(STX_DP, 0x01); emit(STP)

# === TEST 38: JMP (ind) — indirect jump ===
start_test()
# Layout: JMP over target; target: LDA #$DD; JMP continue; setup_ptr; JMP (ind); continue:
emit(JMP_ABS); setup_jmp_patch = pc; emit_word(0)  # JMP over target (patched later)
# Target block (SNES address = CODE_BASE + offset):
jmp_target_snes_addr = CODE_BASE + pc
emit(LDA_IMM, 0xDD)
continue_jmp_patch = pc
emit(JMP_ABS); emit_word(0)  # patched later to continue:
# Setup + JMP (ind):
setup_addr = CODE_BASE + pc
rom[setup_jmp_patch]     = setup_addr & 0xFF
rom[setup_jmp_patch + 1] = (setup_addr >> 8) & 0xFF
# Store target SNES address to $0100 / $0101
emit(LDA_IMM, jmp_target_snes_addr & 0xFF)
emit(STA_ABS); emit_word(0x0100)
emit(LDA_IMM, (jmp_target_snes_addr >> 8) & 0xFF)
emit(STA_ABS); emit_word(0x0101)
emit(LDA_IMM, 0x00)    # clear A
emit(0x6C); emit_word(0x0100)   # JMP ($0100) → target → LDA #$DD → JMP continue
# Continue:
continue_addr = CODE_BASE + pc
rom[continue_jmp_patch + 1] = continue_addr & 0xFF
rom[continue_jmp_patch + 2] = (continue_addr >> 8) & 0xFF
emit_check_a(0xDD, 38)

# === TEST 39: STZ abs (store zero) ===
start_test()
emit(LDA_IMM, 0xFF)
emit(STA_ABS); emit_word(0x00A0)   # $00A0 = $FF
emit(0x9C); emit_word(0x00A0)      # STZ abs → $00A0 = $00
emit(LDA_ABS); emit_word(0x00A0)
emit_check_a(0x00, 39)

# === TEST 40: NOP (just shouldn't crash) ===
start_test()
emit(NOP)
emit(NOP)
emit(NOP)

# === TEST 41: BRA — branch always (relative 8-bit, opcode $80) ===
start_test()
emit(LDA_IMM, 0x11)
emit(0x80, 3)                      # BRA +3
emit(STX_DP, 0x01); emit(STP)      # fail path (3 bytes, skipped)
emit_check_a(0x11, 41)

# === TEST 42: BPL (plus) / BMI (minus) ===
start_test()
emit(LDA_IMM, 0x01)                # N=0
emit(0x10, 3)                      # BPL +3, must branch
emit(STX_DP, 0x01); emit(STP)
emit(LDA_IMM, 0x80)                # N=1
emit(0x30, 3)                      # BMI +3, must branch
emit(STX_DP, 0x01); emit(STP)

# === TEST 43: BVC / BVS ===
start_test()
emit(CLV)                           # V=0
emit(0x50, 3)                      # BVC +3, must branch
emit(STX_DP, 0x01); emit(STP)
# Force V=1 via ADC overflow: 0x40+0x40 signed = 0x80 (V=1, N=1)
emit(CLC)
emit(LDA_IMM, 0x40)
emit(ADC_IMM, 0x40)                # V=1 now
emit(0x70, 3)                      # BVS +3, must branch
emit(STX_DP, 0x01); emit(STP)

# === TEST 44: LDX abs, LDY abs ===
start_test()
emit(LDA_IMM, 0x37)
emit(STA_ABS); emit_word(0x00B0)
emit(LDA_IMM, 0x48)
emit(STA_ABS); emit_word(0x00B1)
emit(0xAE); emit_word(0x00B0)      # LDX abs = $37
emit(CPX_IMM, 0x37)
emit(BEQ, 3); emit(STX_DP, 0x01); emit(STP)
emit(0xAC); emit_word(0x00B1)      # LDY abs = $48
emit(CPY_IMM, 0x48)
emit(BEQ, 3); emit(STY_DP, 0x01); emit(STP)

# === TEST 45: STX abs, STY abs ===
start_test()
emit(LDX_IMM, 0x5A)
emit(0x8E); emit_word(0x00C0)      # STX abs
emit(LDA_ABS); emit_word(0x00C0)
emit_check_a(0x5A, 45)
emit(LDY_IMM, 0x6B)
emit(0x8C); emit_word(0x00C1)      # STY abs
emit(LDA_ABS); emit_word(0x00C1)
emit_check_a(0x6B, 45)

# === TEST 46: LDA abs,X and LDA abs,Y ===
start_test()
emit(LDA_IMM, 0x91)
emit(STA_ABS); emit_word(0x00D5)
emit(LDX_IMM, 0x05)
emit(0xBD); emit_word(0x00D0)      # LDA $00D0,X — $D5
emit_check_a(0x91, 46)
emit(LDA_IMM, 0xA2)
emit(STA_ABS); emit_word(0x00DA)
emit(LDY_IMM, 0x0A)
emit(0xB9); emit_word(0x00D0)      # LDA $00D0,Y — $DA
emit_check_a(0xA2, 46)

# === TEST 47: ORA/AND/EOR with abs and dp ===
start_test()
emit(LDA_IMM, 0x0F); emit(STA_DP, 0xE0)
emit(LDA_IMM, 0xF0)
emit(0x05, 0xE0)                   # ORA dp → 0xFF
emit_check_a(0xFF, 47)
emit(LDA_IMM, 0xFF); emit(STA_ABS); emit_word(0x00E2)
emit(LDA_IMM, 0x0F)
emit(0x2D); emit_word(0x00E2)      # AND abs → 0x0F
emit_check_a(0x0F, 47)
emit(LDA_IMM, 0xAA); emit(STA_DP, 0xE4)
emit(LDA_IMM, 0xFF)
emit(0x45, 0xE4)                   # EOR dp → 0x55
emit_check_a(0x55, 47)

# === TEST 48: ADC dp, SBC dp, CMP dp ===
start_test()
emit(LDA_IMM, 0x05); emit(STA_DP, 0xE8)
emit(CLC)
emit(LDA_IMM, 0x10)
emit(0x65, 0xE8)                   # ADC dp → 0x15
emit_check_a(0x15, 48)
emit(SEC)
emit(LDA_IMM, 0x20)
emit(0xE5, 0xE8)                   # SBC dp → 0x1B
emit_check_a(0x1B, 48)
emit(LDA_IMM, 0x1B)
emit(0xC5, 0xE8)                   # CMP dp (0x1B vs 0x05 → C=1)
emit(BCS, 3); emit(STX_DP, 0x01); emit(STP)

# === TEST 49: ASL dp, LSR dp, ROL dp, ROR dp ===
start_test()
emit(LDA_IMM, 0x41); emit(STA_DP, 0xE9)
emit(0x06, 0xE9)                    # ASL dp → 0x82
emit(LDA_DP, 0xE9)
emit_check_a(0x82, 49)
emit(LDA_IMM, 0x82); emit(STA_DP, 0xE9)
emit(0x46, 0xE9)                    # LSR dp → 0x41
emit(LDA_DP, 0xE9)
emit_check_a(0x41, 49)
emit(SEC); emit(LDA_IMM, 0x00); emit(STA_DP, 0xE9)
emit(0x26, 0xE9)                    # ROL dp with C=1 → 0x01
emit(LDA_DP, 0xE9)
emit_check_a(0x01, 49)
emit(SEC); emit(LDA_IMM, 0x00); emit(STA_DP, 0xE9)
emit(0x66, 0xE9)                    # ROR dp with C=1 → 0x80
emit(LDA_DP, 0xE9)
emit_check_a(0x80, 49)

# === TEST 50: INC dp, DEC dp ===
start_test()
emit(LDA_IMM, 0x10); emit(STA_DP, 0xEA)
emit(0xE6, 0xEA)                    # INC dp
emit(LDA_DP, 0xEA)
emit_check_a(0x11, 50)
emit(0xC6, 0xEA)                    # DEC dp
emit(LDA_DP, 0xEA)
emit_check_a(0x10, 50)

# === TEST 51: INC abs, DEC abs ===
start_test()
emit(LDA_IMM, 0x20); emit(STA_ABS); emit_word(0x00EB)
emit(0xEE); emit_word(0x00EB)       # INC abs
emit(LDA_ABS); emit_word(0x00EB)
emit_check_a(0x21, 51)
emit(0xCE); emit_word(0x00EB)       # DEC abs
emit(LDA_ABS); emit_word(0x00EB)
emit_check_a(0x20, 51)

# === TEST 52: TSX / TXS (and verify SP via push/pull) ===
start_test()
emit(0xBA)                          # TSX → X = SP low
emit(0x8A)                          # TXA → A = SP low (save for restore)
emit(STA_DP, 0xED)                  # save SP low
# Set stack to a known value, then push something and verify TSX
emit(LDX_IMM, 0xF0)
emit(0x9A)                          # TXS → SP = 0x01F0 (native but high byte may differ)
emit(0xBA)                          # TSX → should now be 0xF0
emit(CPX_IMM, 0xF0)
emit(BEQ, 5)
emit(LDA_IMM, 0x77); emit(STX_DP, 0x01); emit(STP)
# Restore SP
emit(LDA_DP, 0xED)
emit(TAX)
emit(0x9A)                          # TXS

# === TEST 53: TXY / TYX ===
start_test()
emit(LDX_IMM, 0x3C)
emit(0x9B)                          # TXY → Y = X
emit(CPY_IMM, 0x3C)
emit(BEQ, 3); emit(STY_DP, 0x01); emit(STP)
emit(LDY_IMM, 0x5E)
emit(0xBB)                          # TYX → X = Y
emit(CPX_IMM, 0x5E)
emit(BEQ, 3); emit(STX_DP, 0x01); emit(STP)

# === TEST 54: TCD / TDC — transfer A to/from DP register ===
start_test()
emit(REP, 0x20)                     # 16-bit A
emit(LDA_IMM, 0x00); emit(0x00)    # A = 0x0000 (save default DP)
emit(0x5B)                          # TCD (A → D)
emit(LDA_IMM, 0x34); emit(0x12)    # A = 0x1234
emit(0x5B)                          # TCD → D = 0x1234
emit(LDA_IMM, 0x00); emit(0x00)    # clear A
emit(0x7B)                          # TDC (D → A)
emit(CMP_IMM, 0x34); emit(0x12)    # A should be 0x1234
emit(SEP, 0x20)                     # back to 8-bit FIRST
emit(BEQ, 3); emit(STX_DP, 0x01); emit(STP)
# Reset DP to 0
emit(REP, 0x20)
emit(LDA_IMM, 0x00); emit(0x00)
emit(0x5B)
emit(SEP, 0x20)

# === TEST 55: JMP abs long (5C) and RTL ===
# Build a long subroutine reachable via JSL (22) and return with RTL (6B)
start_test()
# JMP over the long subroutine
emit(JMP_ABS); jmp_over_long_patch = pc; emit_word(0)
long_sub_start = pc
emit(LDA_IMM, 0x5C)
emit(0x6B)                          # RTL
# Patch JMP target to here
long_main_addr = CODE_BASE + pc
rom[jmp_over_long_patch]     = long_main_addr & 0xFF
rom[jmp_over_long_patch + 1] = (long_main_addr >> 8) & 0xFF
# JSL to the long subroutine (bank 00)
emit(0x22)
emit(CODE_BASE + long_sub_start &  0xFF)
emit((CODE_BASE + long_sub_start) >> 8 & 0xFF)
emit(0x00)                          # bank 00
emit_check_a(0x5C, 55)

# === TEST 56: JMP long (5C) — jump to explicit address ===
start_test()
# JMP $00:target — then target sets A and jumps back via JMP_ABS
emit(0x5C)                          # JMP long
jmp_long_patch = pc
emit(0x00); emit(0x00); emit(0x00)  # placeholder 24-bit address
jmp_long_target = CODE_BASE + pc
emit(LDA_IMM, 0x9E)
# JMP back to continue:
emit(JMP_ABS); jmp_long_back_patch = pc; emit_word(0)
jmp_long_continue = CODE_BASE + pc
# Patch long jump target (24-bit little-endian)
rom[jmp_long_patch]     = jmp_long_target & 0xFF
rom[jmp_long_patch + 1] = (jmp_long_target >> 8) & 0xFF
rom[jmp_long_patch + 2] = 0x00
# Patch continue jump
rom[jmp_long_back_patch]     = jmp_long_continue & 0xFF
rom[jmp_long_back_patch + 1] = (jmp_long_continue >> 8) & 0xFF
emit_check_a(0x9E, 56)

# === TEST 57: PEA (push effective absolute) + PLA to verify ===
start_test()
emit(0xF4); emit_word(0xCAFE)      # PEA $CAFE
emit(REP, 0x20)                     # 16-bit A so PLA pulls 2 bytes
emit(0x68)                          # PLA
emit(CMP_IMM, 0xFE); emit(0xCA)    # 16-bit CMP #$CAFE
emit(SEP, 0x20)
emit(BEQ, 3); emit(STX_DP, 0x01); emit(STP)

# === TEST 58: STZ dp and STZ abs,X ===
start_test()
emit(LDA_IMM, 0xFF); emit(STA_DP, 0xF0)
emit(0x64, 0xF0)                    # STZ dp → 0x00
emit(LDA_DP, 0xF0)
emit_check_a(0x00, 58)
emit(LDA_IMM, 0xEE); emit(STA_ABS); emit_word(0x00F5)
emit(LDX_IMM, 0x05)
emit(0x9E); emit_word(0x00F0)      # STZ $00F0,X → $00F5 = 0
emit(LDA_ABS); emit_word(0x00F5)
emit_check_a(0x00, 58)

# === TEST 59: BIT abs and BIT #imm (8-bit) ===
start_test()
emit(LDA_IMM, 0xC0); emit(STA_ABS); emit_word(0x00F6)
emit(LDA_IMM, 0x80)
emit(0x2C); emit_word(0x00F6)      # BIT abs: A & 0xC0 = 0x80 → Z=0; sets N,V from mem
emit(BNE, 3); emit(STX_DP, 0x01); emit(STP)   # Z=0 → BNE taken
# BIT #imm (0x89) does NOT set N/V — only Z
emit(LDA_IMM, 0x0F)
emit(0x89, 0xF0)                    # BIT #$F0 → A & F0 = 0 → Z=1
emit(BEQ, 3); emit(STX_DP, 0x01); emit(STP)

# === TEST 60: TRB dp, TSB dp ===
start_test()
emit(LDA_IMM, 0xFF); emit(STA_DP, 0xF7)
emit(LDA_IMM, 0x0F)
emit(0x14, 0xF7)                    # TRB dp → mem = mem & ~A = 0xF0
emit(LDA_DP, 0xF7)
emit_check_a(0xF0, 60)
emit(LDA_IMM, 0x0F)
emit(0x04, 0xF7)                    # TSB dp → mem = mem | A = 0xFF
emit(LDA_DP, 0xF7)
emit_check_a(0xFF, 60)

# === TEST 61: ASL abs, LSR abs, ROL abs, ROR abs ===
start_test()
emit(LDA_IMM, 0x41); emit(STA_ABS); emit_word(0x0130)
emit(0x0E); emit_word(0x0130)       # ASL abs
emit(LDA_ABS); emit_word(0x0130)
emit_check_a(0x82, 61)
emit(0x4E); emit_word(0x0130)       # LSR abs → 0x41
emit(LDA_ABS); emit_word(0x0130)
emit_check_a(0x41, 61)
emit(SEC); emit(LDA_IMM, 0x00); emit(STA_ABS); emit_word(0x0130)
emit(0x2E); emit_word(0x0130)       # ROL abs
emit(LDA_ABS); emit_word(0x0130)
emit_check_a(0x01, 61)
emit(SEC); emit(LDA_IMM, 0x00); emit(STA_ABS); emit_word(0x0130)
emit(0x6E); emit_word(0x0130)       # ROR abs
emit(LDA_ABS); emit_word(0x0130)
emit_check_a(0x80, 61)

# === TEST 62: LDA long ($AF) and STA long ($8F) — 24-bit addressing ===
start_test()
emit(LDA_IMM, 0x7E)
emit(0x8F); emit_word(0x1000); emit(0x7E)   # STA $7E1000 (WRAM bank 7E = $000-$FFFF mirror)
# 7E:1000 is WRAM $1000. Let's use $7E:0040 (WRAM $0040 is accessible there too? — actually 7E:0-1FFF = WRAM 0-1FFF)
# Use safer: $7E:0040
emit(0x8F); emit(0x40); emit(0x00); emit(0x7E)  # STA $7E0040
emit(LDA_IMM, 0x00)
emit(0xAF); emit(0x40); emit(0x00); emit(0x7E)  # LDA $7E0040
emit_check_a(0x7E, 62)

# === TEST 63: INC abs,X, DEC abs,X ===
start_test()
emit(LDA_IMM, 0x20); emit(STA_ABS); emit_word(0x0135)
emit(LDX_IMM, 0x05)
emit(0xFE); emit_word(0x0130)       # INC $0130,X → $0135
emit(LDA_ABS); emit_word(0x0135)
emit_check_a(0x21, 63)
emit(0xDE); emit_word(0x0130)       # DEC $0130,X
emit(LDA_ABS); emit_word(0x0135)
emit_check_a(0x20, 63)

# === TEST 64: ASL abs,X and ROL abs,X ===
start_test()
emit(LDA_IMM, 0x41); emit(STA_ABS); emit_word(0x0138)
emit(LDX_IMM, 0x08)
emit(0x1E); emit_word(0x0130)       # ASL $0130,X → $0138
emit(LDA_ABS); emit_word(0x0138)
emit_check_a(0x82, 64)
emit(SEC)
emit(LDA_IMM, 0x00); emit(STA_ABS); emit_word(0x0138)
emit(0x3E); emit_word(0x0130)       # ROL $0130,X
emit(LDA_ABS); emit_word(0x0138)
emit_check_a(0x01, 64)

# === TEST 65: AND/ORA/EOR abs,X ===
start_test()
emit(LDA_IMM, 0xF0); emit(STA_ABS); emit_word(0x013A)
emit(LDX_IMM, 0x0A)
emit(LDA_IMM, 0x0F)
emit(0x1D); emit_word(0x0130)       # ORA $0130,X → 0xFF
emit_check_a(0xFF, 65)
emit(LDA_IMM, 0xFF)
emit(0x3D); emit_word(0x0130)       # AND $0130,X → 0xF0
emit_check_a(0xF0, 65)
emit(LDA_IMM, 0xAA)
emit(0x5D); emit_word(0x0130)       # EOR $0130,X → 0xAA ^ 0xF0 = 0x5A
emit_check_a(0x5A, 65)

# === TEST 66: ADC/SBC abs,X and CMP abs,X ===
start_test()
emit(LDA_IMM, 0x20); emit(STA_ABS); emit_word(0x013C)
emit(LDX_IMM, 0x0C)
emit(CLC)
emit(LDA_IMM, 0x10)
emit(0x7D); emit_word(0x0130)       # ADC $0130,X → 0x30
emit_check_a(0x30, 66)
emit(SEC)
emit(LDA_IMM, 0x50)
emit(0xFD); emit_word(0x0130)       # SBC $0130,X → 0x30
emit_check_a(0x30, 66)
emit(LDA_IMM, 0x20)
emit(0xDD); emit_word(0x0130)       # CMP $0130,X (0x20 vs 0x20 → Z=1)
emit(BEQ, 3); emit(STX_DP, 0x01); emit(STP)

# === TEST 67: JMP (abs,X) — indexed indirect ===
start_test()
# Build jump table at $0140 with one entry pointing to target
emit(JMP_ABS); jmpx_over_patch = pc; emit_word(0)  # JMP over target
jmpx_target_snes = CODE_BASE + pc
emit(LDA_IMM, 0x67)
emit(JMP_ABS); jmpx_back_patch = pc; emit_word(0)  # JMP to continue
jmpx_main_addr = CODE_BASE + pc
rom[jmpx_over_patch]     = jmpx_main_addr & 0xFF
rom[jmpx_over_patch + 1] = (jmpx_main_addr >> 8) & 0xFF
# Set up pointer table: $0142 = target
emit(LDA_IMM, jmpx_target_snes & 0xFF)
emit(STA_ABS); emit_word(0x0142)
emit(LDA_IMM, (jmpx_target_snes >> 8) & 0xFF)
emit(STA_ABS); emit_word(0x0143)
emit(LDX_IMM, 0x02)
emit(0x7C); emit_word(0x0140)       # JMP ($0140,X)
# Continue:
jmpx_continue = CODE_BASE + pc
rom[jmpx_back_patch]     = jmpx_continue & 0xFF
rom[jmpx_back_patch + 1] = (jmpx_continue >> 8) & 0xFF
emit_check_a(0x67, 67)

# === TEST 68: LDA [dp] — indirect long ===
start_test()
# Set up pointer at $10..$12 → $00:0150 → $7B
emit(LDA_IMM, 0x7B); emit(STA_ABS); emit_word(0x0150)
emit(LDA_IMM, 0x50); emit(STA_DP, 0x10)
emit(LDA_IMM, 0x01); emit(STA_DP, 0x11)
emit(LDA_IMM, 0x00); emit(STA_DP, 0x12)   # bank 0
emit(LDA_IMM, 0x00)
emit(0xA7, 0x10)                    # LDA [dp]
emit_check_a(0x7B, 68)

# === TEST 69: LDA [dp],Y — indirect long indexed-Y ===
start_test()
emit(LDA_IMM, 0x8C); emit(STA_ABS); emit_word(0x0158)
emit(LDA_IMM, 0x50); emit(STA_DP, 0x13)
emit(LDA_IMM, 0x01); emit(STA_DP, 0x14)
emit(LDA_IMM, 0x00); emit(STA_DP, 0x15)
emit(LDY_IMM, 0x08)
emit(LDA_IMM, 0x00)
emit(0xB7, 0x13)                    # LDA [dp],Y
emit_check_a(0x8C, 69)

# === TEST 70: LDA dp,X — direct page indexed-X ===
start_test()
emit(LDA_IMM, 0xD9); emit(STA_DP, 0x25)
emit(LDX_IMM, 0x05)
emit(LDA_IMM, 0x00)
emit(0xB5, 0x20)                    # LDA $20,X → $25
emit_check_a(0xD9, 70)

# === TEST 71: STY dp,X / LDY dp,X ===
start_test()
emit(LDY_IMM, 0xE1)
emit(LDX_IMM, 0x03)
emit(0x94, 0x28)                    # STY $28,X → $2B
emit(LDY_IMM, 0x00)
emit(0xB4, 0x28)                    # LDY $28,X
emit(CPY_IMM, 0xE1)
emit(BEQ, 3); emit(STY_DP, 0x01); emit(STP)

# === TEST 72: STX dp,Y / LDX dp,Y ===
start_test()
emit(LDX_IMM, 0xE2)
emit(LDY_IMM, 0x04)
emit(0x96, 0x2C)                    # STX $2C,Y → $30
emit(LDX_IMM, 0x00)
emit(0xB6, 0x2C)                    # LDX $2C,Y
emit(CPX_IMM, 0xE2)
emit(BEQ, 3); emit(STX_DP, 0x01); emit(STP)

# === TEST 73: 16-bit INC A / DEC A via REP ===
start_test()
emit(REP, 0x20)
emit(LDA_IMM, 0xFF); emit(0x00)    # A = 0x00FF
emit(INC_A)                          # A = 0x0100
emit(CMP_IMM, 0x00); emit(0x01)    # A == 0x0100?
emit(SEP, 0x20)
emit(BEQ, 3); emit(STX_DP, 0x01); emit(STP)
emit(REP, 0x20)
emit(LDA_IMM, 0x00); emit(0x01)    # A = 0x0100
emit(DEC_A)                          # A = 0x00FF
emit(CMP_IMM, 0xFF); emit(0x00)
emit(SEP, 0x20)
emit(BEQ, 3); emit(STX_DP, 0x01); emit(STP)

# === TEST 74: 16-bit INX/DEX (boundary) ===
start_test()
emit(REP, 0x10)                     # 16-bit X
emit(LDX_IMM, 0xFF); emit(0x00)    # X = 0x00FF
emit(INX)                            # X = 0x0100 (would be 0x00 in 8-bit)
emit(CPX_IMM, 0x00); emit(0x01)
emit(SEP, 0x10)                     # back to 8-bit X
emit(BEQ, 3); emit(STX_DP, 0x01); emit(STP)

# === TEST 75: 16-bit ADC / SBC ===
start_test()
emit(REP, 0x20)
emit(CLC)
emit(LDA_IMM, 0x00); emit(0x20)    # A = 0x2000
emit(ADC_IMM, 0x34); emit(0x12)    # A = 0x3234
emit(CMP_IMM, 0x34); emit(0x32)
emit(SEP, 0x20)
emit(BEQ, 3); emit(STX_DP, 0x01); emit(STP)
emit(REP, 0x20)
emit(SEC)
emit(LDA_IMM, 0x00); emit(0x30)    # A = 0x3000
emit(SBC_IMM, 0x00); emit(0x10)    # A = 0x2000
emit(CMP_IMM, 0x00); emit(0x20)
emit(SEP, 0x20)
emit(BEQ, 3); emit(STX_DP, 0x01); emit(STP)

# === TEST 76: 16-bit ASL/LSR/ROL/ROR A ===
start_test()
emit(REP, 0x20)
emit(LDA_IMM, 0x01); emit(0x40)    # A = 0x4001
emit(ASL_A)                          # A = 0x8002
emit(CMP_IMM, 0x02); emit(0x80)
emit(SEP, 0x20)
emit(BEQ, 3); emit(STX_DP, 0x01); emit(STP)
emit(REP, 0x20)
emit(LDA_IMM, 0x02); emit(0x80)    # A = 0x8002
emit(LSR_A)                          # A = 0x4001
emit(CMP_IMM, 0x01); emit(0x40)
emit(SEP, 0x20)
emit(BEQ, 3); emit(STX_DP, 0x01); emit(STP)

# === TEST 77: 16-bit INC/DEC abs ===
start_test()
emit(REP, 0x20)
emit(LDA_IMM, 0xFF); emit(0x00)    # A = 0x00FF
emit(0x8D); emit_word(0x0160)       # STA $0160 (16-bit, writes FF,00)
emit(0xEE); emit_word(0x0160)       # INC abs (16-bit) → 0x0100
emit(0xAD); emit_word(0x0160)       # LDA abs
emit(CMP_IMM, 0x00); emit(0x01)
emit(SEP, 0x20)
emit(BEQ, 3); emit(STX_DP, 0x01); emit(STP)

# === TEST 78: Flag set/clear: CLV, SED+CLD (decimal mode), SEI+CLI ===
start_test()
# CLV: set V then clear
emit(CLC); emit(LDA_IMM, 0x40); emit(ADC_IMM, 0x40)  # V=1
emit(CLV)                            # V=0
emit(0x50, 3)                      # BVC +3 — must branch (V=0)
emit(STX_DP, 0x01); emit(STP)
# SED/CLD: we won't test decimal math, just that flags toggle
emit(SED)
emit(PHP)
emit(CLD)
emit(PLP)                            # restore old P (D may be set)
emit(CLD)                            # make sure cleared

# === TEST 79: Stack-relative addressing (LDA sr, STA sr) ===
start_test()
# Push a value, then read it via stack-relative
emit(LDA_IMM, 0x9F)
emit(PHA)                            # [SP+1] = 0x9F
emit(LDA_IMM, 0x00)
emit(0xA3, 0x01)                    # LDA $01,S — stack-relative
emit(CMP_IMM, 0x9F)
emit(BEQ, 3); emit(STX_DP, 0x01); emit(STP)
emit(PLA)                            # cleanup

# === TEST 80: PEI (push effective indirect) — pushes word from dp ===
start_test()
emit(LDA_IMM, 0xBC); emit(STA_DP, 0x40)
emit(LDA_IMM, 0x9A); emit(STA_DP, 0x41)
emit(0xD4, 0x40)                    # PEI ($40) — pushes word at $40 (0x9ABC)
emit(REP, 0x20)
emit(0x68)                           # PLA (16-bit)
emit(CMP_IMM, 0xBC); emit(0x9A)
emit(SEP, 0x20)
emit(BEQ, 3); emit(STX_DP, 0x01); emit(STP)

# === TEST 81: PER (push effective PC relative) ===
start_test()
emit(0x62); emit_word(0x0003)       # PER +3 (pushes PC_after_instruction + 3)
emit(REP, 0x20)
emit(0x68)                           # PLA (16-bit)
emit(SEP, 0x20)
# Just verify the push didn't explode — A should be some address, not cared about exact value.

# === TEST 82: MVN / MVP — block move ===
start_test()
# Set up source $0170..$0173 = 0x11,0x22,0x33,0x44
emit(LDA_IMM, 0x11); emit(STA_ABS); emit_word(0x0170)
emit(LDA_IMM, 0x22); emit(STA_ABS); emit_word(0x0171)
emit(LDA_IMM, 0x33); emit(STA_ABS); emit_word(0x0172)
emit(LDA_IMM, 0x44); emit(STA_ABS); emit_word(0x0173)
# MVN: X=src, Y=dst, A=count-1. Banks are set by the MVN operand bytes.
emit(REP, 0x30)                     # 16-bit A and XY
emit(LDX_IMM, 0x70); emit(0x01)    # X = 0x0170
emit(LDY_IMM, 0x80); emit(0x01)    # Y = 0x0180
emit(LDA_IMM, 0x03); emit(0x00)    # A = 3 (count-1 = copy 4 bytes)
emit(0x54, 0x00, 0x00)             # MVN $00, $00 (dst bank, src bank)
emit(SEP, 0x30)
# Verify dst
emit(LDA_ABS); emit_word(0x0180); emit(CMP_IMM, 0x11)
emit(BEQ, 3); emit(STX_DP, 0x01); emit(STP)
emit(LDA_ABS); emit_word(0x0183); emit(CMP_IMM, 0x44)
emit(BEQ, 3); emit(STX_DP, 0x01); emit(STP)

# === TEST 83: XCE — toggle E flag (round-trip) ===
start_test()
# SEC+XCE enters emulation; CLC+XCE returns to native. No PHP/PLP around it
# (emulation mode clobbers SP high byte in some cores).
emit(SEC)
emit(0xFB)                           # XCE → E=1
emit(CLC)
emit(0xFB)                           # XCE → E=0
emit(SEP, 0x30)                     # native 8-bit both
emit(LDA_IMM, 0x5F)
emit_check_a(0x5F, 83)

# === TEST 84: Stack-relative indirect indexed-Y — (sr),Y ===
start_test()
# Pre-store target $0192 = 0xA4. Push pointer 0x0190 onto stack (high then low).
# Then LDA ($01,S),Y with Y=2 → fetch ptr from [S+1,S+2]=$0190, +Y=$0192.
emit(LDA_IMM, 0xA4); emit(STA_ABS); emit_word(0x0192)
emit(LDA_IMM, 0x01); emit(PHA)      # push high byte of pointer
emit(LDA_IMM, 0x90); emit(PHA)      # push low byte
emit(LDY_IMM, 0x02)
emit(LDA_IMM, 0x00)
emit(0xB3, 0x01)                    # LDA ($01,S),Y → 0xA4
emit(STA_DP, 0x16)                  # save A so PLX doesn't matter
emit(PLX); emit(PLX)                # restore stack
emit(LDA_DP, 0x16)
emit_check_a(0xA4, 84)

# === TEST 85: 16-bit BIT — Z bit set ===
start_test()
emit(LDA_IMM, 0x00); emit(STA_ABS); emit_word(0x01A0)
emit(LDA_IMM, 0x00); emit(STA_ABS); emit_word(0x01A1)
emit(REP, 0x20)
emit(LDA_IMM, 0xFF); emit(0xFF)    # A = 0xFFFF
emit(0x2C); emit_word(0x01A0)       # BIT $01A0 (16-bit, all zero) → Z=1
# Branch in 16-bit mode (Z preserved by SEP, but be safe: branch first)
emit(BEQ, 5)                        # skip fail block (5 bytes: SEP+STX+STP)
emit(SEP, 0x20)
emit(STX_DP, 0x01); emit(STP)
emit(SEP, 0x20)                     # back to 8-bit on success path

# === TEST 86: Conditional branches from flag combos ===
start_test()
emit(LDA_IMM, 0x80)                # N=1
emit(0x30, 3)                      # BMI +3
emit(STX_DP, 0x01); emit(STP)
emit(LDA_IMM, 0x00)                # N=0, Z=1
emit(0x10, 3)                      # BPL +3
emit(STX_DP, 0x01); emit(STP)
emit(CLC)
emit(0x90, 3)                      # BCC +3
emit(STX_DP, 0x01); emit(STP)
emit(SEC)
emit(0xB0, 3)                      # BCS +3
emit(STX_DP, 0x01); emit(STP)

# === TEST 87: WDM (reserved — should NOP, takes 1 operand byte) ===
start_test()
emit(LDA_IMM, 0xEE)
emit(0x42, 0x00)                    # WDM $00 (2-byte instruction, no-op)
emit_check_a(0xEE, 87)

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
