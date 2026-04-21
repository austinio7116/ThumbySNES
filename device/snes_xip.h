#ifndef SNES_XIP_H
#define SNES_XIP_H

#include <stdint.h>

/* Read current QMI RFMT register for chip-select 0 (flash). This is
 * the register the bootrom / boot2 sets up to describe how XIP fetches
 * are issued to the flash chip. On RP2350 we want this to be the
 * quad-width 0xEB "Fast Read Quad I/O" form, which produces roughly
 * 4x the throughput of the default single-width 0x03 form.
 *
 * Any flash write op (flash_range_erase / flash_range_program) exits
 * XIP temporarily and is expected to re-enter it via the copied-out
 * boot2 binary. If that re-entry is broken or a non-SDK codepath
 * leaves flash in slow-03h mode, this register will differ from what
 * we captured at boot. */
uint32_t snes_xip_read_rfmt(void);

/* Convenience — returns the DATA_WIDTH field (0=S, 1=D, 2=Q). */
int snes_xip_data_width(void);

/* Decoded human-readable description. Returns a static string. */
const char *snes_xip_mode_desc(void);

/* Re-run boot2 to force XIP into the mode boot2 configures (which for
 * the W25Q080 boot2 is Fast Read Quad I/O 0xEB continuous mode). Used
 * to recover from whatever pico-sdk's runtime init does that leaves
 * the QMI block in slow dual mode despite boot2 having programmed
 * quad. Safe to call repeatedly — boot2 is idempotent.
 *
 * Disables interrupts for the duration (~10 µs). Returns the RFMT
 * value measured AFTER re-running boot2. */
uint32_t snes_xip_rerun_boot2(void);

#endif
