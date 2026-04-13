/*
 * ThumbyNES — ROM picker.
 *
 * Scans the FAT volume for *.snes files in the root directory and
 * presents a simple scrollable list. Returns the selected file
 * name or -1 if cancelled. Phase 3: list-only, no thumbnails.
 */
#ifndef THUMBYSNES_PICKER_H
#define THUMBYSNES_PICKER_H

#include <stddef.h>
#include <stdint.h>

#define SNES_PICKER_MAX_ROMS  64
/* Max file name length stored in snes_rom_entry. Real ROM dumps with
 * full no-intro / GoodSNES tags push 60+ chars. 96 covers every
 * realistic name without burning much SRAM. */
#define SNES_PICKER_NAME_MAX  96
#define SNES_PICKER_PATH_MAX  (SNES_PICKER_NAME_MAX + 16)

/* ThumbySNES is single-system — only SNES ROMs. Kept as an 8-bit field
 * so the struct layout stays 1:1 with the ThumbyNES picker. */
#define ROM_SYS_SNES  0

typedef struct {
    char     name[SNES_PICKER_NAME_MAX];  /* base file name in / */
    uint32_t size;
    uint8_t  mapper;                      /* SNES mapper: 0=LoROM, 1=HiROM, 0xFF=unknown */
    uint8_t  pal_hint;                    /* 0 = NTSC default, 1 = PAL detected */
    uint8_t  system;                      /* always ROM_SYS_SNES */
    uint8_t  _pad;
} snes_rom_entry;

/* Scan / for *.smc / *.sfc files. Returns count placed in `out`. */
int snes_picker_scan(snes_rom_entry *out, int max);

/* Run the picker UI against `fb` (128×128 RGB565). Reads buttons,
 * presents to the LCD each frame. Returns the index of the chosen
 * ROM in `entries[]` when the user launches a cart.
 *
 * `entries` is mutable and `n_entries` is in/out — the picker
 * re-scans the FAT volume into the buffer whenever it detects USB
 * MSC activity has gone quiet, so files added or deleted via USB
 * appear/disappear without having to power-cycle. The caller's
 * `*n_entries` is updated to reflect the new total.
 *
 * If `*n_entries == 0` on entry the picker shows a "no ROMs" splash
 * and stays in that state until a ROM appears (then returns 0). The
 * caller is expected to keep pumping USB MSC tasks. */
int snes_picker_run(uint16_t *fb,
                    snes_rom_entry *entries, int *n_entries);

/* Slurp a ROM file into a malloc'd buffer. Caller frees.
 * Use only for small (< ~300 KB) files — see snes_picker_mmap_rom
 * for the zero-copy path used by the ROM runner. */
uint8_t *snes_picker_load_rom(const char *name, size_t *out_len);

/* Map a ROM file directly from XIP flash without copying it into
 * RAM. Returns 0 on success and writes a pointer into flash + the
 * file size. The pointer remains valid until the file is deleted
 * or the FAT volume is reformatted. Returns nonzero if the file
 * isn't contiguous on disk (in which case the caller can fall
 * back to snes_picker_load_rom). */
int snes_picker_mmap_rom(const char *name,
                          const uint8_t **out_data, size_t *out_len);

/* Defragmenter — rewrites every fragmented file in / so its cluster
 * chain becomes contiguous, which lets the XIP mmap path serve large
 * carts that would otherwise fall back to malloc and OOM. Walks the
 * root, checks each file with the same chain_is_contiguous logic the
 * mmap path uses, and rewrites the offenders via the f_expand
 * temp-file dance. Progress is drawn into `fb` so the user sees what
 * the picker is doing.
 *
 * Returns the number of files rewritten (>= 0) or a negative error
 * code if the pass aborted. The function pumps tud_task() between
 * files so USB stays alive while it runs. */
int snes_picker_defrag(uint16_t *fb);

/* Global preferences — applies across every cart. Lives in /.global
 * on the FAT volume. The picker menu and the in-game menus both
 * adjust the same values; per-cart .cfg sidecars no longer carry
 * volume.
 *
 * Overclock choices are MHz values: 125 / 150 / 200 / 250. The
 * default is 250 (the same fixed clock the firmware booted with
 * before this option existed). Setting takes effect on the next
 * ROM launch — snes_device_main re-applies the saved value before
 * each runner starts.  */
int  snes_picker_global_volume(void);
void snes_picker_global_set_volume(int v);

int  snes_picker_global_clock_mhz(void);
void snes_picker_global_set_clock_mhz(int mhz);

#endif
