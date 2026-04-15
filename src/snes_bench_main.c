/*
 * ThumbySNES — Phase 0 headless bench.
 *
 * Usage:  snesbench <rom.smc> [frames]
 *
 * mmap()s the ROM file, calls snes_load + snes_run_frame in a loop, and
 * prints wall-clock + approximate MHz. Used to establish the baseline
 * perf number before any rendering work.
 */
#include "snes_core.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

static double now_secs(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec * 1e-9;
}

int main(int argc, char **argv)
{
    if (argc < 2) {
        fprintf(stderr, "usage: %s <rom.smc> [frames=600] [--xip] [--lcd] [--frameskip=N]\n", argv[0]);
        return 1;
    }
    const char *path = argv[1];
    int frames = (argc >= 3) ? atoi(argv[2]) : 600;
    int use_xip = 0;
    int use_lcd = 0;
    int frameskip = 0;
    for (int i = 1; i < argc; i++) {
        if      (!strcmp(argv[i], "--xip"))  use_xip = 1;
        else if (!strcmp(argv[i], "--lcd"))  use_lcd = 1;
        else if (!strncmp(argv[i], "--frameskip=", 12)) frameskip = atoi(argv[i] + 12);
    }

    int fd = open(path, O_RDONLY);
    if (fd < 0) { perror("open"); return 1; }
    struct stat st;
    if (fstat(fd, &st) != 0) { perror("fstat"); close(fd); return 1; }
    size_t rom_len = (size_t)st.st_size;
    const uint8_t *rom = mmap(NULL, rom_len, PROT_READ, MAP_PRIVATE, fd, 0);
    if (rom == MAP_FAILED) { perror("mmap"); close(fd); return 1; }

    snes_result_t r = use_xip ? snes_load_xip(rom, rom_len) : snes_load(rom, rom_len);
    if (r != SNES_OK) {
        fprintf(stderr, "snes_load: %d (stub until Phase 0 core vendor — see PLAN.md §10)\n", r);
        munmap((void*)rom, rom_len);
        close(fd);
        return (r == SNES_ERR_NOT_IMPLEMENTED) ? 0 : 2;
    }

    static uint16_t fb[256 * 224];
    static uint16_t lcd_fb[128 * 128];
    if (use_lcd) {
        /* Device performance path: native-LCD rendering + color-math
         * skip + frameskip. Match the settings in device/snes_run.c. */
        snes_set_skip_color_math(1);
        snes_set_lcd_mode(lcd_fb, 128, 128);
        snes_set_frameskip(frameskip);
    }

    double t0 = now_secs();
    int first_nonzero_frame = -1;
    for (int i = 0; i < frames; ++i) {
        r = snes_run_frame();
        if (r != SNES_OK) { fprintf(stderr, "snes_run_frame[%d]: %d\n", i, r); break; }
        if (first_nonzero_frame < 0 && (i & 0x1f) == 0) {
            if (use_lcd) {
                for (int p = 0; p < 128 * 128; p++) {
                    if (lcd_fb[p]) { first_nonzero_frame = i; break; }
                }
            } else {
                snes_get_framebuffer(fb);
                for (int p = 0; p < 256 * 224; p++) {
                    if (fb[p]) { first_nonzero_frame = i; break; }
                }
            }
        }
    }
    double t1 = now_secs();
    if (first_nonzero_frame >= 0)
        printf("first non-zero frame: %d\n", first_nonzero_frame);
    else
        printf("frame buffer stayed all-zero for entire bench\n");

    double secs = t1 - t0;
    double fps  = (secs > 0.0) ? (double)frames / secs : 0.0;
    printf("ThumbySNES bench: %d frames in %.3fs = %.1f fps (%.2fx realtime)  [%s%s%s]\n",
           frames, secs, fps, fps / 60.0,
           use_lcd ? "lcd" : "classic",
           frameskip ? " frameskip=" : "",
           frameskip ? (frameskip == 1 ? "1" : (frameskip == 2 ? "2" : "N")) : "");

    
    /* DEBUG: dump CPU + WRAM state */
    printf("DEBUG: PC=0x%04x PB=%02x A=0x%04x\n", snes_dbg_pc(), snes_dbg_pb(), snes_dbg_a());
    printf("DEBUG: WRAM[0..15]:");
    for (int _i = 0; _i < 16; _i++) printf(" %02x", snes_dbg_wram(_i));
    printf("\n");

    /* Test ROM check: WRAM[$0000] = 'P' (0x50) on pass, 'F' (0x46) on
     * fail. WRAM[$0001] = test number that failed (0 = none).
     * WRAM[$0002] = expected, WRAM[$0003] = actual. */
    {
        uint8_t marker = snes_dbg_wram(0x0000);
        uint8_t failed_test = snes_dbg_wram(0x0001);
        uint8_t expected = snes_dbg_wram(0x0002);
        uint8_t actual = snes_dbg_wram(0x0003);
        if (marker == 0x50) {
            printf("\n*** TEST ROM PASSED (all tests OK) ***\n");
        } else if (failed_test != 0) {
            printf("\n*** TEST ROM FAILED at test #%d (expected 0x%02x, got 0x%02x) ***\n",
                   failed_test, expected, actual);
        } else if (marker == 0 && failed_test == 0) {
            printf("\n*** TEST ROM did not complete (no marker set) ***\n");
        }
    }

    snes_unload();
    munmap((void*)rom, rom_len);
    close(fd);
    return 0;
}
