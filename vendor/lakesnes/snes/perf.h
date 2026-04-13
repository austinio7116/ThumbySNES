/*
 * ThumbySNES — performance attributes for hot LakeSnes paths.
 *
 * `LAKESNES_HOT` marks a function for placement in `.time_critical.snes`
 * which the Pico SDK linker script copies into SRAM at boot. That dodges
 * flash-XIP cache misses on the inner emulator loop. On host builds it
 * expands to nothing.
 *
 * Activate by passing `-DLAKESNES_HOT_SRAM=1` to the compiler (the
 * device CMake does this; host CMake does not).
 */
#ifndef LAKESNES_PERF_H
#define LAKESNES_PERF_H

#if defined(LAKESNES_HOT_SRAM) && LAKESNES_HOT_SRAM
#define LAKESNES_HOT __attribute__((section(".time_critical.snes")))
#else
#define LAKESNES_HOT
#endif

#endif
