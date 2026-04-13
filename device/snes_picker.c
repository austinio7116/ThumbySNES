/*
 * ThumbySNES — ROM picker.
 *
 * Single-system SNES picker: scans / for *.smc / *.sfc, shows a
 * scrollable list, returns the selected index on A. Written fresh
 * for ThumbySNES — ThumbyNES's picker had a tabbed multi-core UI with
 * iNES header parsing; we don't need any of that here.
 *
 * Phase 3 scope:
 *   - Boot-time scan
 *   - Paged list with D-pad nav + A select
 *   - "no ROMs" splash with rescan-on-USB-idle
 *   - Favorites (/.favs) sorted to the top
 *   - XIP mmap path used by the runner (zero-copy ROM load)
 */
#include "snes_picker.h"
#include "snes_font.h"
#include "snes_lcd_gc9107.h"
#include "snes_buttons.h"
#include "snes_battery.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

#include "pico/stdlib.h"
#include "hardware/flash.h"
#include "hardware/regs/addressmap.h"
#include "tusb.h"
#include "ff.h"

#include "snes_flash_disk.h"

extern FATFS g_fs;
extern volatile uint64_t g_msc_last_op_us;

#define FB_W 128
#define FB_H 128

/* --- favorites store ------------------------------------------------ */

#define FAVS_PATH      "/.favs"
#define FAVS_BUF_SIZE  4096

static char   favs_buf[FAVS_BUF_SIZE];
static size_t favs_len   = 0;
static bool   favs_dirty = false;

static void favs_load(void) {
    favs_len = 0;
    favs_dirty = false;
    FIL f;
    if (f_open(&f, FAVS_PATH, FA_READ) != FR_OK) return;
    UINT br = 0;
    f_read(&f, favs_buf, FAVS_BUF_SIZE - 1, &br);
    f_close(&f);
    favs_len = br;
    favs_buf[favs_len] = 0;
}

static void favs_save(void) {
    if (!favs_dirty) return;
    FIL f;
    if (f_open(&f, FAVS_PATH, FA_WRITE | FA_CREATE_ALWAYS) != FR_OK) return;
    UINT bw = 0;
    f_write(&f, favs_buf, (UINT)favs_len, &bw);
    f_close(&f);
    favs_dirty = false;
}

static int favs_find(const char *name) {
    size_t name_len = strlen(name);
    size_t i = 0;
    while (i < favs_len) {
        size_t j = i;
        while (j < favs_len && favs_buf[j] != '\n') j++;
        size_t line_len = j - i;
        if (line_len == name_len && memcmp(&favs_buf[i], name, name_len) == 0)
            return (int)i;
        i = j + 1;
    }
    return -1;
}

static int is_favorite(const char *name) { return favs_find(name) >= 0; }

static void favs_toggle(const char *name) {
    int off = favs_find(name);
    if (off >= 0) {
        size_t j = off;
        while (j < favs_len && favs_buf[j] != '\n') j++;
        if (j < favs_len) j++;
        memmove(&favs_buf[off], &favs_buf[j], favs_len - j);
        favs_len -= (j - off);
        favs_dirty = true;
    } else {
        size_t name_len = strlen(name);
        if (favs_len + name_len + 1 >= FAVS_BUF_SIZE) return;
        memcpy(&favs_buf[favs_len], name, name_len);
        favs_buf[favs_len + name_len] = '\n';
        favs_len += name_len + 1;
        favs_dirty = true;
    }
}

/* --- filename heuristic for PAL region tag ------------------------- */

static int filename_says_pal(const char *fname) {
    char lower[SNES_PICKER_NAME_MAX];
    size_t i;
    for (i = 0; fname[i] && i < sizeof(lower) - 1; i++) {
        char c = fname[i];
        if (c >= 'A' && c <= 'Z') c += 32;
        lower[i] = c;
    }
    lower[i] = 0;
    return strstr(lower, "(europe)") != NULL
        || strstr(lower, "(pal)")    != NULL
        || strstr(lower, "(e)")      != NULL;
}

/* --- scan ----------------------------------------------------------- */

int snes_picker_scan(snes_rom_entry *out, int max) {
    DIR dir;
    FILINFO info;
    if (f_opendir(&dir, "/") != FR_OK) return 0;
    int n = 0;
    while (n < max && f_readdir(&dir, &info) == FR_OK) {
        if (info.fname[0] == 0) break;
        if (info.fattrib & AM_DIR) continue;
        size_t L = strlen(info.fname);
        if (L < 5) continue;
        const char *ext = info.fname + L - 4;
        if (strcasecmp(ext, ".smc") != 0 && strcasecmp(ext, ".sfc") != 0) continue;
        strncpy(out[n].name, info.fname, SNES_PICKER_NAME_MAX - 1);
        out[n].name[SNES_PICKER_NAME_MAX - 1] = 0;
        out[n].size     = (uint32_t)info.fsize;
        out[n].system   = ROM_SYS_SNES;
        out[n].mapper   = 0xFF;
        out[n].pal_hint = filename_says_pal(info.fname);
        n++;
    }
    f_closedir(&dir);
    return n;
}

/* --- load / mmap helpers -------------------------------------------- */

uint8_t *snes_picker_load_rom(const char *name, size_t *out_len) {
    char path[SNES_PICKER_PATH_MAX];
    snprintf(path, sizeof(path), "/%s", name);
    FIL f;
    if (f_open(&f, path, FA_READ) != FR_OK) return NULL;
    FSIZE_t sz = f_size(&f);
    if (sz < 16 || sz > 512 * 1024) { f_close(&f); return NULL; }
    uint8_t *buf = (uint8_t *)malloc((size_t)sz);
    if (!buf) { f_close(&f); return NULL; }
    UINT br;
    if (f_read(&f, buf, (UINT)sz, &br) != FR_OK || br != sz) {
        free(buf); f_close(&f); return NULL;
    }
    f_close(&f);
    if (out_len) *out_len = (size_t)sz;
    return buf;
}

int snes_picker_mmap_rom(const char *name,
                          const uint8_t **out_data, size_t *out_len) {
    char path[SNES_PICKER_PATH_MAX];
    snprintf(path, sizeof(path), "/%s", name);
    FIL f;
    if (f_open(&f, path, FA_READ) != FR_OK) return -1;
    FSIZE_t sz = f_size(&f);
    if (sz < 16) { f_close(&f); return -2; }

    /* FAT cluster → XIP flash offset. Assumes a contiguous cluster
     * chain (the typical case for ROMs dumped via USB MSC); if the
     * file is fragmented the ROM data read from the returned pointer
     * will be garbled past the first cluster. A defrag pass (ported
     * from ThumbyNES in Phase 6) will rewrite fragmented files. */
    DWORD clust = f.obj.sclust;
    uint32_t cluster_bytes = g_fs.csize * 512u;
    uint32_t data_base     = (g_fs.database) * 512u;
    uint32_t file_offset   = data_base + (clust - 2) * cluster_bytes;

    *out_data = (const uint8_t *)(XIP_BASE
                 + FLASH_DISK_OFFSET
                 + file_offset);
    *out_len  = (size_t)sz;
    f_close(&f);
    return 0;
}

/* --- list UI --------------------------------------------------------- */

#define ROW_H       10
#define HEADER_H    12
#define FOOTER_H    10
#define ROW_COUNT   ((FB_H - HEADER_H - FOOTER_H) / ROW_H)

static int cmp_entry(const void *a, const void *b) {
    const snes_rom_entry *ea = (const snes_rom_entry *)a;
    const snes_rom_entry *eb = (const snes_rom_entry *)b;
    int fa = is_favorite(ea->name) ? 0 : 1;
    int fb = is_favorite(eb->name) ? 0 : 1;
    if (fa != fb) return fa - fb;
    return strcasecmp(ea->name, eb->name);
}

static void fill_rect(uint16_t *fb, int x, int y, int w, int h, uint16_t c) {
    for (int yy = y; yy < y + h && yy < FB_H; yy++)
        for (int xx = x; xx < x + w && xx < FB_W; xx++)
            fb[yy * FB_W + xx] = c;
}

static void draw_list(uint16_t *fb,
                      const snes_rom_entry *e, int n,
                      int cursor, int top) {
    memset(fb, 0, FB_W * FB_H * sizeof(uint16_t));

    /* Header */
    fill_rect(fb, 0, 0, FB_W, HEADER_H, 0x2124);
    snes_font_draw(fb, "ThumbySNES", 3, 2, 0xFFFF);

    /* Battery pip */
    int batt = snes_battery_percent();
    uint16_t batt_col = (batt > 30) ? 0x07E0 : (batt > 10) ? 0xFFE0 : 0xF800;
    fill_rect(fb, FB_W - 14, 3, batt / 10, 5, batt_col);
    fill_rect(fb, FB_W - 14, 2, 10, 1, 0xFFFF);
    fill_rect(fb, FB_W - 14, 8, 10, 1, 0xFFFF);
    fill_rect(fb, FB_W - 14, 3, 1, 5, 0xFFFF);
    fill_rect(fb, FB_W -  5, 3, 1, 5, 0xFFFF);

    /* Rows */
    for (int row = 0; row < ROW_COUNT && top + row < n; row++) {
        int idx = top + row;
        int y = HEADER_H + row * ROW_H;
        int is_cur = (idx == cursor);
        if (is_cur) fill_rect(fb, 0, y, FB_W, ROW_H, 0x10A2);
        if (is_favorite(e[idx].name))
            snes_font_draw(fb, "*", 2, y + 1, 0xFD20);
        char buf[22];
        size_t L = strlen(e[idx].name);
        if (L >= 20) { memcpy(buf, e[idx].name, 19); buf[19] = '~'; buf[20] = 0; }
        else         { memcpy(buf, e[idx].name, L + 1); }
        snes_font_draw(fb, buf, 8, y + 1, is_cur ? 0xFFFF : 0xCE59);
    }

    /* Footer */
    fill_rect(fb, 0, FB_H - FOOTER_H, FB_W, FOOTER_H, 0x2124);
    char foot[24];
    snprintf(foot, sizeof(foot), "%d/%d  A:play RB:fav", cursor + 1, n);
    snes_font_draw(fb, foot, 2, FB_H - FOOTER_H + 1, 0xCE59);
}

static void draw_splash(uint16_t *fb) {
    memset(fb, 0, FB_W * FB_H * sizeof(uint16_t));
    snes_font_draw(fb, "ThumbySNES",        20,  20, 0xFFFF);
    snes_font_draw(fb, "no ROMs found",     19,  50, 0xCE59);
    snes_font_draw(fb, "drag .smc/.sfc",    15,  68, 0xCE59);
    snes_font_draw(fb, "over USB to /",     19,  80, 0xCE59);
    snes_font_draw(fb, "snes9x2002 core",   10, 110, 0x6B4D);
}

int snes_picker_run(uint16_t *fb, snes_rom_entry *entries, int *n_entries) {
    favs_load();

    int n      = *n_entries;
    int cursor = 0;
    int top    = 0;

    if (n > 0) qsort(entries, n, sizeof(snes_rom_entry), cmp_entry);

    uint32_t last_scan_ms = 0;
    while (1) {
        snes_buttons_poll();
        tud_task();

        uint64_t idle_us = (uint64_t)to_us_since_boot(get_absolute_time())
                         - g_msc_last_op_us;
        uint32_t now_ms  = to_ms_since_boot(get_absolute_time());
        bool want_rescan = (idle_us > 500000)
                        && (n == 0 || (int)(now_ms - last_scan_ms) > 2000);
        if (want_rescan) {
            int new_n = snes_picker_scan(entries, SNES_PICKER_MAX_ROMS);
            if (new_n != n) {
                n = new_n;
                *n_entries = n;
                qsort(entries, n, sizeof(snes_rom_entry), cmp_entry);
                if (cursor >= n) cursor = n > 0 ? n - 1 : 0;
                if (top > cursor) top = cursor;
            }
            last_scan_ms = now_ms;
        }

        if (n == 0) { draw_splash(fb); snes_lcd_present(fb); continue; }

        if (snes_buttons_just_pressed(SNES_BTN_DOWN))  cursor = (cursor + 1) % n;
        if (snes_buttons_just_pressed(SNES_BTN_UP))    cursor = (cursor + n - 1) % n;
        if (snes_buttons_just_pressed(SNES_BTN_RIGHT)) cursor = (cursor + ROW_COUNT < n) ? cursor + ROW_COUNT : n - 1;
        if (snes_buttons_just_pressed(SNES_BTN_LEFT))  cursor = (cursor - ROW_COUNT > 0)  ? cursor - ROW_COUNT : 0;
        if (snes_buttons_just_pressed(SNES_BTN_RB)) {
            favs_toggle(entries[cursor].name);
            favs_save();
            qsort(entries, n, sizeof(snes_rom_entry), cmp_entry);
        }
        if (snes_buttons_just_pressed(SNES_BTN_A)) {
            favs_save();
            return cursor;
        }

        if (cursor < top) top = cursor;
        if (cursor >= top + ROW_COUNT) top = cursor - ROW_COUNT + 1;

        draw_list(fb, entries, n, cursor, top);
        snes_lcd_present(fb);
    }
}

/* --- defrag (Phase 6) ----------------------------------------------- */

int snes_picker_defrag(uint16_t *fb) {
    (void)fb;
    return 0;
}

/* --- global prefs ---------------------------------------------------- */

#define GLOBAL_PATH  "/.global"
#define GLOBAL_MAGIC 0x53534E53u  /* 'SNSS' */

typedef struct {
    uint32_t magic;
    int      volume;
    int      clock_mhz;
    uint8_t  _pad[16];
} global_cfg_t;

static global_cfg_t g_global = { .magic = GLOBAL_MAGIC, .volume = 10, .clock_mhz = 250 };
static bool g_global_loaded = false;

static void global_load(void) {
    if (g_global_loaded) return;
    FIL f;
    if (f_open(&f, GLOBAL_PATH, FA_READ) == FR_OK) {
        UINT br;
        global_cfg_t c;
        if (f_read(&f, &c, sizeof(c), &br) == FR_OK && br == sizeof(c)
            && c.magic == GLOBAL_MAGIC) {
            g_global = c;
        }
        f_close(&f);
    }
    g_global_loaded = true;
}

static void global_save(void) {
    FIL f;
    if (f_open(&f, GLOBAL_PATH, FA_WRITE | FA_CREATE_ALWAYS) != FR_OK) return;
    UINT bw;
    f_write(&f, &g_global, sizeof(g_global), &bw);
    f_close(&f);
}

int  snes_picker_global_volume(void)        { global_load(); return g_global.volume; }
void snes_picker_global_set_volume(int v)   { global_load(); g_global.volume = v; global_save(); }
int  snes_picker_global_clock_mhz(void)     { global_load(); return g_global.clock_mhz; }
void snes_picker_global_set_clock_mhz(int m){ global_load(); g_global.clock_mhz = m; global_save(); }
