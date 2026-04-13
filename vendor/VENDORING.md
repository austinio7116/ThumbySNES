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

### Expected patches

_(populate as applied)_

1. **`src/port.h`** — wrap any platform-defining block in `#ifndef` so the
   device build can pick `IRAM_ATTR` via `-DIRAM_ATTR=...` (same trick used
   in ThumbyNES's nofrendo patch).
2. **Hot loop tagging** — expect to add `IRAM_ATTR` to `S9xMainLoop`,
   `S9xDoHEventProcessing`, `S9xDoHBlankProcessing`, the per-line renderer,
   and `APU_EXECUTE1` so those functions land in `.time_critical.snes` on
   the device build. On host builds `IRAM_ATTR` expands to nothing.
3. **`libretro/`** — delete the libretro / retro-arch frontend glue we
   don't need; keep only the core emulation sources (`src/`, `cpu/`,
   `apu/`, `dsp/`, `gfx/`, `tables/`).
4. **State save bridge** — if we add save states, patch any `fopen` /
   `fwrite` / `fread` / `fseek` / `fclose` in the core's save-state path to
   route through `device/thumby_state_bridge.[ch]` when compiled with
   `-DTHUMBY_STATE_BRIDGE` (same pattern as nofrendo / smsplus in
   ThumbyNES).
