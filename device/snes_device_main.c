/*
 * ThumbySNES — Thumby Color device runtime entry point.
 *
 * Phase 3 scope:
 *   - Boots the chip.
 *   - Brings up LCD + buttons + audio + flash disk + USB MSC.
 *   - Mounts the FAT volume (reformats only on mount failure).
 *   - Scans / for *.smc / *.sfc ROMs.
 *   - Runs the picker; on pick, calls snes_run_rom (currently a stub).
 *   - Loops back to picker on MENU or stub return.
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/clocks.h"
#include "tusb.h"
#include "ff.h"
#include "diskio.h"

#include "snes_lcd_gc9107.h"
#include "snes_buttons.h"
#include "snes_audio_pwm.h"
#include "snes_flash_disk.h"
#include "snes_picker.h"
#include "snes_font.h"
#include "snes_run.h"
#include "snes_battery.h"

#define THUMBYSNES_VERSION "0.1"
#define FB_W 128
#define FB_H 128

/* g_fs is the one FATFS instance shared across picker + diskio.
 * g_msc_last_op_us is defined in snes_msc.c (where the MSC callbacks
 * live) and extern'd by picker. */
FATFS g_fs;
extern volatile uint64_t g_msc_last_op_us;

static uint16_t       g_fb[FB_W * FB_H];
static snes_rom_entry g_roms[SNES_PICKER_MAX_ROMS];

static void draw_boot_splash(void) {
    memset(g_fb, 0, sizeof(g_fb));
    snes_font_draw(g_fb, "ThumbySNES",          20,  30, 0xFFFF);
    snes_font_draw(g_fb, "v" THUMBYSNES_VERSION, 48, 44, 0xCE59);
    snes_font_draw(g_fb, "snes9x2002",          30,  80, 0x07FF);
    snes_font_draw(g_fb, "mounting...",         25, 104, 0x6B4D);
    snes_lcd_present(g_fb);
}

static void mount_or_format(void) {
    FRESULT r = f_mount(&g_fs, "", 1);
    if (r == FR_NO_FILESYSTEM) {
        static BYTE work[FF_MAX_SS];
        MKFS_PARM p = {
            .fmt = FM_FAT | FM_SFD,
            .n_fat = 1,
            .au_size = 4096,
            .align = 0,
            .n_root = 0
        };
        f_mkfs("", &p, work, sizeof(work));
        f_mount(&g_fs, "", 1);
        f_setlabel("THUMBYSNES");
    }
}

int main(void) {
    stdio_init_all();

    /* Core clock — snes9x2002 likes 250 MHz. Per-user override in
     * Phase 6 can bump to 300 or drop to save battery. */
    set_sys_clock_khz(250000, true);

    snes_lcd_init();
    snes_buttons_init();
    snes_audio_pwm_init();
    snes_battery_init();

    snes_flash_disk_init();
    tusb_init();

    draw_boot_splash();
    mount_or_format();

    while (1) {
        int n = snes_picker_scan(g_roms, SNES_PICKER_MAX_ROMS);
        int sel = snes_picker_run(g_fb, g_roms, &n);
        if (sel < 0 || sel >= n) continue;
        snes_run_rom(&g_roms[sel], g_fb);
    }
}
