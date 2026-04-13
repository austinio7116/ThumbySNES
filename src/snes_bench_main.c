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
        fprintf(stderr, "usage: %s <rom.smc> [frames=600]\n", argv[0]);
        return 1;
    }
    const char *path = argv[1];
    int frames = (argc >= 3) ? atoi(argv[2]) : 600;

    int fd = open(path, O_RDONLY);
    if (fd < 0) { perror("open"); return 1; }
    struct stat st;
    if (fstat(fd, &st) != 0) { perror("fstat"); close(fd); return 1; }
    size_t rom_len = (size_t)st.st_size;
    const uint8_t *rom = mmap(NULL, rom_len, PROT_READ, MAP_PRIVATE, fd, 0);
    if (rom == MAP_FAILED) { perror("mmap"); close(fd); return 1; }

    snes_result_t r = snes_load(rom, rom_len);
    if (r != SNES_OK) {
        fprintf(stderr, "snes_load: %d (stub until Phase 0 core vendor — see PLAN.md §10)\n", r);
        munmap((void*)rom, rom_len);
        close(fd);
        return (r == SNES_ERR_NOT_IMPLEMENTED) ? 0 : 2;
    }

    double t0 = now_secs();
    for (int i = 0; i < frames; ++i) {
        r = snes_run_frame();
        if (r != SNES_OK) { fprintf(stderr, "snes_run_frame[%d]: %d\n", i, r); break; }
    }
    double t1 = now_secs();

    double secs = t1 - t0;
    double fps  = (secs > 0.0) ? (double)frames / secs : 0.0;
    printf("ThumbySNES bench: %d frames in %.3fs = %.1f fps (%.2fx realtime)\n",
           frames, secs, fps, fps / 60.0);

    snes_unload();
    munmap((void*)rom, rom_len);
    close(fd);
    return 0;
}
