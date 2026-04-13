/*
 * ThumbySNES — device-side emulator driver.
 *
 * Phase 3: stub. Shows a "loading ROM" splash for 1 s then returns,
 * so the picker/device_main loop is testable without an emulator
 * actually running. Phase 4 replaces the body with snes_core calls
 * + the scanline-into-LCD render hook.
 */
#ifndef THUMBYSNES_RUN_H
#define THUMBYSNES_RUN_H

#include <stdint.h>
#include "snes_picker.h"

/* Run the picked cart.  Returns 0 on normal exit (user MENU'd back
 * to picker), non-zero on error. `fb` is the 128x128 RGB565 LCD
 * backing buffer. */
int snes_run_rom(const snes_rom_entry *rom, uint16_t *fb);

/* Per-ROM clock override (125/150/200/250 MHz). Returns 0 to mean
 * "use the global default". Phase 3: always 0. */
int snes_run_clock_override(const char *rom_name);

#endif
