# Vendored sources

## snes9x2002/

SNES emulation core. To be vendored from the libretro `snes9x2002` fork at
Phase 0 — not yet present in this repo.

- Upstream: https://github.com/libretro/snes9x2002
- Commit pinned: _(record at Phase 0 drop)_
- License: snes9x non-commercial (see `snes9x2002/LICENSE` after vendor)
- Original authors: Gary Henderson, Jerremy Koot, and the snes9x team;
  MIPS/ARM assembly contributions for handheld ports from zsKnight, John
  Weidman, _Demo_, and others. libretro fork maintained by the RetroArch
  team.

### Applied patches (Phase 2, 2026-04-13)

Vendor tree is committed in-tree (`.git` stripped on import so it's a flat
set of files we own). The upstream commit is pinned above; if we ever need
to re-vendor from upstream, reapply these:

1. **`src/memmap.c:175`** — cap `Memory.SRAM` alloc at 64 KB (was 128 KB).
   Largest real SNES cart SRAM is 64 KB (Star Fox). Saves 64 KB heap.
   ```c
   /* was */ Memory.SRAM = (uint8 *) malloc (0x20000);
   /* now */ Memory.SRAM = (uint8 *) malloc (0x10000);
   ```

2. **`src/dma.c:59`** — shrink SDD1 decompression staging buffer from
   64 KB to 16 KB. Gated by `Settings.SDD1` which stays FALSE for every
   cart in our support set (SDD1 is only SF Alpha 2 + Star Ocean, non-goals
   per `PLAN.md §9`). Saves 48 KB BSS.
   ```c
   /* was */ uint8 buffer[0x10000];
   /* now */ uint8 buffer[0x4000];
   ```

3. **`src/globals.c:171`** and **`src/soundux.h:185`** — shrink `Echo`
   delay buffer from `int[24000]` to `int[16384]`. SNES HW max echo is
   240 ms at 32 kHz stereo = 15360 samples; 16384 gives a 1 KB safety
   margin. Saves 30 KB BSS.

### Expected future patches (Phase 3+)

1. **`src/port.h`** — wrap any platform-defining block in `#ifndef` so the
   device build can pick `IRAM_ATTR` via `-DIRAM_ATTR=...` (same trick used
   in ThumbyNES's nofrendo patch).
2. **Hot loop tagging** — expect to add `IRAM_ATTR` to `S9xMainLoop`,
   `S9xDoHEventProcessing`, `S9xDoHBlankProcessing`, the per-line renderer,
   and `APU_EXECUTE1` so those functions land in `.time_critical.snes` on
   the device build. On host builds `IRAM_ATTR` expands to nothing.
3. **`src/memmap.c:177` + `LoadROM` in memmap.c** — patch to skip the 6 MB
   ROM copy and use an externally-provided pointer (XIP flash mapping on
   device). Saves 6 MB heap. Device-only.
4. **Tile cache eviction** (Phase 4+) — `tile.c:561-566` + `ppu.h` — add
   a hash/LRU path to let `MAX_*BIT_TILES` drop from 4096/2048/1024 to
   smaller sizes (~32 slots/depth). Currently the caches must stay at
   worst-case-VRAM sizes because tile indices are computed `(TileAddr &
   0xffff) >> TileShift` with no bounds check. ~880 KB SRAM on the table.
5. **State save bridge** — if we add save states, patch any `fopen` /
   `fwrite` / `fread` / `fseek` / `fclose` in `snapshot.c` to route through
   `device/thumby_state_bridge.[ch]` when compiled with
   `-DTHUMBY_STATE_BRIDGE` (same pattern as nofrendo / smsplus in
   ThumbyNES).
