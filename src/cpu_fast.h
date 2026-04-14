/*
 * ThumbySNES — fast-path 65816 opcode handler.
 *
 * Handles the top-10 hottest opcodes with 65816 registers cached in
 * ARM local variables (compiler keeps them in R4-R11 instead of
 * loading/storing from the Cpu struct on every access). Returns 1 if
 * the opcode was handled, 0 for fallback to the C switch.
 *
 * The remaining 246 opcodes fall through to the existing cpu_doOpcode.
 */
#ifndef THUMBYSNES_CPU_FAST_H
#define THUMBYSNES_CPU_FAST_H

#include "cpu.h"

/* Returns 1 if opcode was handled by the fast path, 0 otherwise. */
int cpu_doOpcodeFast(Cpu* cpu, uint8_t opcode);

#endif
