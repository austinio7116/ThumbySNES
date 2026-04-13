/*
 * ThumbySNES — Phase 2 memory audit.
 *
 * Intercepts malloc/calloc/realloc/free via GNU ld --wrap so every
 * allocation made by the core + our glue is logged with size + site.
 * Tracks live bytes and peak watermark across boot → LoadROM → N frames
 * → unload.
 *
 * Output shape (per ROM):
 *   === <rom>: <size> bytes ===
 *     boot peak : NNNN  (Settings / GFX / MemoryInit / InitAPU / InitSound / GraphicsInit)
 *     load peak : NNNN  (add LoadROM / ROM copy)
 *     run  peak : NNNN  (steady state during snes_run_frame)
 *     top allocations:  (sorted, live-at-peak)
 *       N bytes  × addr   [call site if we can get it cheaply]
 *
 * Usage:  memaudit [frames=120] [rom1.sfc rom2.sfc ...]
 *   No ROM args → walks roms/ alphabetically.
 */
#include "snes_core.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <dirent.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

/* --- allocation tracker ------------------------------------------------ */

#define MAX_LIVE 8192

typedef struct {
    void  *ptr;
    size_t sz;
} alloc_t;

static alloc_t g_live_tbl[MAX_LIVE];
static size_t  g_live_bytes = 0;
static size_t  g_peak_bytes = 0;
static size_t  g_alloc_count = 0;
static int     g_enabled = 0;

static void track_add(void *p, size_t sz)
{
    if (!p || !g_enabled) return;
    g_live_bytes += sz;
    g_alloc_count++;
    if (g_live_bytes > g_peak_bytes) g_peak_bytes = g_live_bytes;
    for (int i = 0; i < MAX_LIVE; i++) {
        if (g_live_tbl[i].ptr == NULL) {
            g_live_tbl[i].ptr = p;
            g_live_tbl[i].sz  = sz;
            return;
        }
    }
    fprintf(stderr, "memaudit: live table overflow — bump MAX_LIVE\n");
}

static size_t track_remove(void *p)
{
    if (!p || !g_enabled) return 0;
    for (int i = 0; i < MAX_LIVE; i++) {
        if (g_live_tbl[i].ptr == p) {
            size_t sz = g_live_tbl[i].sz;
            g_live_tbl[i].ptr = NULL;
            g_live_tbl[i].sz  = 0;
            g_live_bytes -= sz;
            return sz;
        }
    }
    return 0;
}

/* --- wrap interposers (linker flag provides __real_ prefix) ------------ */

extern void *__real_malloc(size_t n);
extern void *__real_calloc(size_t n, size_t s);
extern void *__real_realloc(void *p, size_t n);
extern void  __real_free(void *p);

void *__wrap_malloc(size_t n)
{
    void *p = __real_malloc(n);
    track_add(p, n);
    return p;
}

void *__wrap_calloc(size_t n, size_t s)
{
    void *p = __real_calloc(n, s);
    track_add(p, n * s);
    return p;
}

void *__wrap_realloc(void *p, size_t n)
{
    size_t old = p ? track_remove(p) : 0;
    (void)old;
    void *q = __real_realloc(p, n);
    if (q) track_add(q, n);
    return q;
}

void __wrap_free(void *p)
{
    track_remove(p);
    __real_free(p);
}

/* --- reporting --------------------------------------------------------- */

static int cmp_alloc_desc(const void *a, const void *b)
{
    size_t sa = ((const alloc_t *)a)->sz;
    size_t sb = ((const alloc_t *)b)->sz;
    return (sb > sa) - (sb < sa);
}

static void dump_top_live(int top_n)
{
    alloc_t snap[MAX_LIVE];
    memcpy(snap, g_live_tbl, sizeof(snap));
    qsort(snap, MAX_LIVE, sizeof(alloc_t), cmp_alloc_desc);
    int shown = 0;
    size_t shown_bytes = 0;
    for (int i = 0; i < MAX_LIVE && shown < top_n; i++) {
        if (!snap[i].ptr) break;
        printf("    %8zu B  %p\n", snap[i].sz, snap[i].ptr);
        shown_bytes += snap[i].sz;
        shown++;
    }
    size_t rest = g_live_bytes - shown_bytes;
    if (rest)
        printf("    %8zu B  (rest, %zu allocations)\n", rest, g_alloc_count);
}

/* --- per-ROM run ------------------------------------------------------- */

static int run_rom(const char *path, int frames)
{
    int fd = open(path, O_RDONLY);
    if (fd < 0) { perror(path); return -1; }
    struct stat st;
    fstat(fd, &st);
    const uint8_t *rom = mmap(NULL, st.st_size, PROT_READ, MAP_PRIVATE, fd, 0);
    if (rom == MAP_FAILED) { perror("mmap"); close(fd); return -1; }

    const char *name = strrchr(path, '/');
    name = name ? name + 1 : path;
    printf("=== %s: %lld bytes ===\n", name, (long long)st.st_size);

    /* boot — includes MemoryInit / InitAPU / InitSound / GraphicsInit / GFX buffers */
    g_enabled = 1;
    size_t before_load = 0;

    extern int g_audit_xip; /* set in main */
    snes_result_t r = g_audit_xip ? snes_load_xip(rom, st.st_size)
                                  : snes_load    (rom, st.st_size);
    if (r != SNES_OK) {
        fprintf(stderr, "  snes_load failed: %d\n", r);
        munmap((void *)rom, st.st_size);
        close(fd);
        return -1;
    }
    size_t after_load_peak = g_peak_bytes;
    size_t after_load_live = g_live_bytes;

    /* run — steady state */
    for (int i = 0; i < frames; i++) snes_run_frame();
    size_t run_peak = g_peak_bytes;
    size_t run_live = g_live_bytes;

    printf("  after load : live=%zu KB  peak=%zu KB  (SNES state resident)\n",
           after_load_live / 1024, after_load_peak / 1024);
    printf("  after %d f  : live=%zu KB  peak=%zu KB\n",
           frames, run_live / 1024, run_peak / 1024);
    printf("  top-10 live allocations at end of run:\n");
    dump_top_live(10);
    printf("\n");

    snes_unload();
    g_enabled = 0;
    munmap((void *)rom, st.st_size);
    close(fd);

    /* reset for next ROM */
    g_peak_bytes = 0;
    g_alloc_count = 0;
    (void)before_load;
    return (int)run_peak;
}

/* --- main -------------------------------------------------------------- */

static int ends_with(const char *s, const char *suf)
{
    size_t ls = strlen(s), lf = strlen(suf);
    return ls >= lf && strcasecmp(s + ls - lf, suf) == 0;
}

static int str_cmp_fwd(const void *a, const void *b)
{
    return strcmp(*(const char *const *)a, *(const char *const *)b);
}

int g_audit_xip = 0;

int main(int argc, char **argv)
{
    int frames = 120;
    int first_rom = 1;
    if (argc >= 2) {
        char *end;
        long n = strtol(argv[1], &end, 10);
        if (*end == '\0') { frames = (int)n; first_rom = 2; }
    }
    for (int i = 1; i < argc; i++) if (!strcmp(argv[i], "--xip")) g_audit_xip = 1;

    size_t global_peak = 0;
    const char *peak_rom = "";

    if (first_rom < argc) {
        for (int i = first_rom; i < argc; i++) {
            int p = run_rom(argv[i], frames);
            if (p > (int)global_peak) { global_peak = p; peak_rom = argv[i]; }
        }
    } else {
        DIR *d = opendir("roms");
        if (!d) { fprintf(stderr, "no roms/ dir — pass ROM paths explicitly\n"); return 1; }
        static char *names[256];
        int n = 0;
        struct dirent *e;
        while ((e = readdir(d))) {
            if (e->d_name[0] == '.') continue;
            if (!ends_with(e->d_name, ".sfc") && !ends_with(e->d_name, ".smc")) continue;
            names[n] = malloc(strlen(e->d_name) + 6);
            sprintf(names[n], "roms/%s", e->d_name);
            n++;
            if (n == 256) break;
        }
        closedir(d);
        qsort(names, n, sizeof(char *), str_cmp_fwd);
        for (int i = 0; i < n; i++) {
            int p = run_rom(names[i], frames);
            if (p > (int)global_peak) { global_peak = p; peak_rom = names[i]; }
            free(names[i]);
        }
    }

    printf("=== summary ===\n");
    printf("  worst-case heap peak: %zu KB  (%s)\n",
           global_peak / 1024, peak_rom);
    printf("  device budget       : ~390 KB (see PLAN.md §4.1)\n");
    return 0;
}
