/*
 * ThumbySNES — ARM Thumb-2 assembly 65816 CPU dispatcher.
 *
 * Runs a batch of 65816 opcodes with registers pinned in ARM
 * callee-saved registers R4-R11. Implements 28 hot opcodes in
 * inline assembly; all others fall back to the C cpu_runOpcode.
 *
 * Returns the number of opcodes executed (always == maxOpcodes
 * unless the CPU enters a special state).
 */
#ifndef THUMBYSNES_CPU_ASM_H
#define THUMBYSNES_CPU_ASM_H

#include "cpu.h"

int cpu_runBatchAsm(Cpu *cpu, int maxOpcodes);

#endif
