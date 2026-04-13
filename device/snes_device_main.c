/*
 * ThumbySNES — Thumby Color device runtime entry point.
 *
 * Phase 3 scope:
 *   - Boots the chip.
 *   - Brings up LCD + buttons + audio + flash disk.
 *   - Mounts the FAT volume (reformats only on mount failure).
 *   - Starts TinyUSB AFTER the FAT volume is fully written.
 *   - Scans / for *.smc / *.sfc ROMs.
 *   - Runs the picker; on pick, calls snes_run_rom (Phase 3 stub).
 *
 * Init order closely mirrors ThumbyNES — particularly: FAT must be
 * fully formatted + flushed BEFORE tusb_init(), and we must pump
 * tud_task() for ~1 s after USB start so the host enumerates before
 * we hand off to the picker.
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "pico/stdlib.h"
#include "pico/time.h"
#include "pico/multicore.h"
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

#if THUMBYSNES_HAVE_CORE
#include "snes_core.h"

/* Core 1 entry — parks in snes_apu_core1_loop. When a ROM is loaded
 * via snes_load*, the core1 loop picks up the shared snes pointer and
 * starts running spc_runOpcode continuously. CPU + PPU stay on core 0. */
static void core1_apu_entry(void) {
    snes_apu_core1_loop();
}
#endif

#define THUMBYSNES_VERSION "0.1"
#define FB_W 128
#define FB_H 128

FATFS g_fs;
extern volatile uint64_t g_msc_last_op_us;

static uint16_t       g_fb[FB_W * FB_H];
static snes_rom_entry g_roms[SNES_PICKER_MAX_ROMS];

/* Fill the LCD framebuffer with a solid colour + short label. Matches
 * ThumbyNES's splash_color/boot pattern — these are the only visible
 * signal we have during init if USB fails. */
static void splash(uint16_t colour, const char *msg) {
    for (int i = 0; i < FB_W * FB_H; i++) g_fb[i] = colour;
    snes_font_draw(g_fb, "ThumbySNES",          20,  30, 0xFFFF);
    snes_font_draw(g_fb, "v" THUMBYSNES_VERSION, 48, 44, 0xFFFF);
    if (msg) snes_font_draw(g_fb, msg, 2, 104, 0xFFFF);
    snes_lcd_present(g_fb);
}

static int boot_filesystem(void) {
    int needs_format = 0;
    FRESULT r = f_mount(&g_fs, "", 1);
    if (r != FR_OK) needs_format = 1;

    if (needs_format) {
        splash(0xFFE0, "formatting...");
        f_unmount("");
        memset(&g_fs, 0, sizeof(g_fs));

        static BYTE work[FF_MAX_SS * 4];
        MKFS_PARM opt = { FM_FAT, 1, 0, 0, 1024 };
        if (f_mkfs("", &opt, work, sizeof(work)) != FR_OK) {
            splash(0xF800, "mkfs failed");
            return -1;
        }
        snes_flash_disk_flush();

        if (f_mount(&g_fs, "", 1) != FR_OK) {
            splash(0xFA00, "remount failed");
            return -1;
        }
        f_setlabel("THUMBYSNES");
        snes_flash_disk_flush();
    }
    return 0;
}

int main(void) {
    /* Clock BEFORE stdio_init so the USB PLL comes up at the right
     * divider. SNES emulation is CPU-bound — bump from 250 to 300 MHz
     * for a free 20% on the inner loop. RP2350 spec is 150 MHz nominal
     * but the silicon happily runs at 300+ MHz with stock voltage. */
    set_sys_clock_khz(300000, true);
    stdio_init_all();

    snes_buttons_init();
    snes_lcd_init();
    snes_audio_pwm_init();
    snes_battery_init();
    splash(0x194A, "LCD alive");

    /* FAT must come up BEFORE USB. If we let the host start enumerating
     * while mkfs is mid-flight, Windows reads inconsistent BPB state
     * and File Explorer trips over the half-written directory tree. */
    splash(0xFD00, "mounting...");
    snes_flash_disk_init();
    if (boot_filesystem() < 0) {
        while (1) tight_loop_contents();
    }
    snes_flash_disk_flush();

    /* USB stack. Disk is fully on flash → enumeration is safe. */
    splash(0x07FF, "USB starting");
    tusb_init();
    {
        absolute_time_t until = make_timeout_time_ms(1000);
        while (!time_reached(until)) {
            tud_task();
            sleep_us(100);
        }
    }
    splash(0x07E0, "ready");
    sleep_ms(200);

#if THUMBYSNES_HAVE_CORE
    /* Launch core 1 for SPC700 + DSP. It parks until a ROM is loaded,
     * then runs SPC opcodes continuously while core 0 handles
     * CPU + PPU + LCD + USB. */
    multicore_launch_core1(core1_apu_entry);
#endif

    while (1) {
        int n = snes_picker_scan(g_roms, SNES_PICKER_MAX_ROMS);
        int sel = snes_picker_run(g_fb, g_roms, &n);
        if (sel < 0 || sel >= n) continue;
        snes_run_rom(&g_roms[sel], g_fb);
    }
}
