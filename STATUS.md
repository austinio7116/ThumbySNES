# ThumbySNES — current status (2026-04-13)

> **Update later on 2026-04-13**: per-line RGB565 composite + dual-core
> APU + BG empty-slot skip landed. **Measured on device**:
> Zelda ALTTP title ≈ **8 fps** (from ~4.8), FF II ≈ **9.6 fps** (from
> ~5). 2×2 blend preserved — no visual compromise vs the pre-session
> baseline. Still short of the 30 fps target; the remaining big levers
> are a tile-major PPU rewrite and/or ARM asm for the master-cycle
> scheduler (see "Why we can't get to 30+ fps").

## What works

- **SNES emulation runs on stock Thumby Color hardware** (RP2350 M33 @ 300 MHz, 520 KB SRAM, 16 MB flash).
- Vendored core: **LakeSnes** (MIT, ~6,500 LOC, pure C).
- **Memory fits** comfortably: heap demand ~265 KB against 297 KB available, with all SNES state (WRAM 128 + VRAM 64 + ARAM 64 + cart SRAM 8) resident.
- **ROM lives in XIP flash** — zero-copy load via patched `cart_load`. Up to 4 MB ROMs supported (flash holds 16 MB total).
- **Per-scanline output** — patched LakeSnes PPU pixelBuffer from 978 KB → 2 KB, scanline callback fires after each visible line. Device's callback applies a **2×2 RGB565 box-blend with 7:4 stride** (FILL mode) directly into the 128×128 LCD framebuffer.
- **Picker, USB MSC, FatFs** all carry over from the ThumbyNES device-layer port.
- **Buttons**: A/B/D-pad/LB/RB → SNES A/B/D-pad/Y/X (YX profile, see `PLAN.md §7`). MENU short-press = Start, MENU long-press (≥800ms) = exit to picker.

## What doesn't work

- **Super Mario World (Europe)** hangs after ~70 frames at PC=`$809D` in an SPC handshake wait loop. SPC writes `0xBB` then stops echoing past `0x0D`. Game-specific quirk we couldn't crack — Zelda EU + FFII + DKC etc all boot fine, so it's not a PAL or LoROM issue. SMW US not tested.
- **Audio is silent** — `snes_get_audio` returns zeroes. Phase 5 work would route LakeSnes's `snes_setSamples` output through the existing PWM driver from ThumbyNES.
- **Color math / transparency disabled** for performance. HUD fades, lantern halos, water tints render as flat colors.
- **Mode 7** PPU paths fall through to the per-pixel slow path — works, just slowest.
- **Enhancement chips not supported** — SuperFX (Star Fox, Yoshi's Island), SA-1 (Mario RPG, Kirby Super Star), DSP-1 (Mario Kart): LakeSnes simply doesn't implement them.

## Performance

- **~4.8 fps** on FF II / Zelda content scenes (full-screen rendering, sprites + BGs)
- **~9 fps** on idle / blank screens
- Bottleneck profile (host gprof, M33 likely similar):
  - 47% `ppu_getPixel` (per-pixel layer compositor, layer iteration, palette resolve)
  - 15% `ppu_runLine` (scanline orchestration)
  - 11% `snes_runCycles` (per-cycle scheduler)
  - 5% `ppu_getWindowState`
  - 4% `dsp_cycle` (audio mixing — runs even though we discard samples)
  - 3% `cpu_doOpcode` (65816 dispatcher itself is fast)

*(See `PERF.md` for the full catalog of optimizations currently active,
grouped by category. Summary table below is chronological / highest
impact first.)*

### Optimizations applied (cumulative, in order of impact)

| Change | Effect |
|---|---|
| Move from snes9x2002 → LakeSnes | enabled fitting in SRAM at all (per-scanline architecture, no 4MB GFX buffer) |
| `set_sys_clock_khz(300000)` overclock | +20% raw CPU |
| `LAKESNES_HOT` macro on hot top-level functions → `.time_critical.snes` SRAM section | dodges flash XIP cache misses on inner loop |
| `double` → `float` for APU catch-up math | M33 has single-precision FPU; doubles went through softfloat |
| Skip color math (`skipColorMath`) | dodges per-pixel recursive subscreen fetch |
| Per-line BG layer pre-render cache | small gain — gcc was already inlining the per-pixel path well |
| `renderXStart`/`renderXEnd` to skip the 32 cropped edge pixels per line | ~12% fewer per-pixel calls |
| `--gc-sections`, `-ffunction-sections`, `-fdata-sections` | dead-strip unused code |
| **Native-LCD render mode** (`snes_set_lcd_mode`, available but unused on device) | PPU composites RGB565 straight into the 128×128 device framebuffer at native resolution. Nearest-neighbour 7:4 — text unreadable on content screens, so the classic blend path stays default. Opt-in for specific ROMs. |
| **CGRAM → RGB565 table** (brightness baked in, lazy rebuild on $2100/$2122) | Collapses the per-pixel 3-channel brightness math + byte packing into a single uint16 array read. |
| **Per-line RGB565 composite** (`ppu_composeLineRgb565`, `snes_set_scanline_cb_rgb565`) | Replaces per-pixel `ppu_handlePixel` walk with per-line tight-array passes, one per layer slot, bottom-up. 256-pixel RGB565 out directly — device scanline callback gets native RGB565 so the 2×2 blend has no conversion inside. Host bench: **+50-78%**; device Zelda 4.8→8 fps, FFII ≈5→9.6 fps. |
| **Dual-core APU** (`THUMBYSNES_DUAL_CORE`) | SPC700 + DSP run on core 1; core 0 runs CPU + PPU + LCD. `snes_catchupApu` compiles to a no-op and the per-master-cycle `apuCatchupCycles` float accumulator update is skipped — saves one FPU multiply + add per ~1-2M `snes_runCycle` calls per frame. inPorts/outPorts are byte-atomic; no mailbox needed. Side benefit: SMW's SPC handshake hang may resolve (SPC no longer starved of cycles during CPU polls). |
| **BG line-cache empty-slot skip** (`bgLineNonEmpty[L][P]`) | `ppu_renderBgLine` flags whether it wrote any non-zero pixel; the compositor skips slots with the flag clear (common — e.g. BG3 on outdoor scenes, BG2 on HUD-only screens). One check replaces a 256-wide pass. |
| **Frameskip** (`snes_set_frameskip`, available but off) | Opt-in: alternate frames skip PPU. Halves visible fps for a game-speed bump — net-negative at current base rates. |

### Optimizations tried + reverted

| Change | Why reverted |
|---|---|
| `LAKESNES_HOT` on static functions (`ppu_getPixel`, `ppu_handlePixel`, etc.) | section attribute on inlinable statics caused boot hang — gcc generated mismatched section + inline copies |
| `-flto` | code bloat exceeded M33's 16KB icache — slight regression |

### Why we can't get to 30+ fps without bigger surgery

*(After the 2026-04-13-later pass we're measured at 8–10 fps in
content. Getting to 30 fps still requires at least one of:)*

`ppu_handlePixel` is called 224 × 256 = ~57K times per frame. Each call iterates layer slots (4-5 in mode 1), reads VRAM + palette, writes pixelBuffer. At ~600 cycles/call on M33, that's ~34M cycles/frame just in per-pixel work — 50%+ of total. Even fully eliminating per-pixel work only gets us from ~5 fps to ~10 fps.

LakeSnes is a clean reference implementation focused on accuracy. Real speed requires:
1. **Tile-major PPU rewrite** — decode each tile once per scanline, blit 8 pixels at a time. Same architectural choice snes9x2002 made for ARM handhelds. Estimated 3-5× speedup; ~1-2 weeks of careful vendor surgery.
2. **Dual-core split** — SPC700 + DSP on core 1, CPU + PPU on core 0. ~20-30% additional once architecturally correct. Several days work + lock-free queue + careful timing.
3. **Lower internal resolution** — render at 128×112 directly instead of 256×224 + downscale. ~4× fewer pixels but requires PPU teaching half-resolution sampling. Major surgery.

## Memory

| Region | KB | Notes |
|---|---:|---|
| Total SRAM | 520 | RP2350 spec |
| BSS (LakeSnes statics + Pico SDK + drivers + picker buffers) | 190 | |
| `.data` / `.time_critical.snes` (SRAM-resident hot code) | 29 | |
| Stack + ram_vector_table | 8 | |
| Heap available | 297 | |
| **Heap demand at runtime**: | **~265** | |
| ⤷ Snes struct (incl. WRAM 128 KB) | 132 | |
| ⤷ Apu struct (incl. ARAM 64 KB) | 68 | |
| ⤷ Ppu struct (incl. VRAM 64 KB + 2 KB bgLine cache) | 66 | |
| ⤷ Cart RAM (game-dependent, max 64 KB) | 0–64 | SMW = 2 KB, Star Fox = 64 KB |
| ⤷ misc | ~5 | |

**Margin: ~30 KB** at runtime. Comfortable for current scope.

## Build

```bash
# Host SDL bench/runner (verifies emulator correctness)
cmake -B build -S .
cmake --build build -j8
./build/snesbench "roms/<rom>.sfc" 600 --xip
./build/sneshost  "roms/<rom>.sfc" --full --xip

# Device firmware
cmake -B build_device -S device \
      -DPICO_SDK_PATH=/home/maustin/mp-thumby/lib/pico-sdk \
      -DTHUMBYSNES_LINK_CORE=ON
cmake --build build_device -j8
# Output: firmware/snesrun_device.uf2
```

## Files

- `vendor/lakesnes/snes/` — vendored LakeSnes, with Thumby-specific patches (per-scanline output, XIP cart, color-math skip, per-line BG cache, render-x-range). Patches documented inline.
- `src/snes_core.[ch]` — public emulator API used by host + device
- `src/snes_host_main.c` — SDL2 SDL frontend (TAB cycles FILL/FIT/FULL display modes)
- `src/snes_bench_main.c`, `src/snes_memaudit_main.c` — perf + memory tooling
- `device/snes_run.c` — Phase 4 device driver: XIP load → scanline-blit → frame loop
- `device/snes_*.[ch]` — LCD, buttons, audio, FatFs, USB MSC, picker
- `device/CMakeLists.txt` — RP2350 firmware build
- `firmware/snesrun_device.uf2` — flashable build artifact
- `tools/memaudit.md` — full memory analysis (snes9x2002 era + LakeSnes findings)
- `PLAN.md` — original architectural plan, mostly historical now (chose LakeSnes after Phase 4 surgery on snes9x2002 hit a wall)

## Next steps (in priority order if/when picked up again)

1. **Wire audio** — `snes_setSamples` → ring buffer → existing `snes_audio_pwm_push` from device layer. Phase 5 was always next.
2. **Hunt the SMW EU hang** — probably a single 65816 opcode behaving subtly differently on M33 vs x86, or a SPC700-driver-specific quirk in the music-upload protocol. Could do bisection by stubbing SPC reads at $2140-$2143 with synthetic ack values to see how far CPU progresses.
3. **In-game menu** — currently MENU long-press is the only control. Needs frame skip, pause, save-state, exit options.
4. **Save state / SRAM persistence** — LakeSnes has `snes_saveState` / `snes_loadBattery` already; just need to wire FatFs sidecars.
5. **Performance**: see "Why we can't get to 30+ fps" above. Tile-major PPU rewrite is the highest-impact single project.

## Verdict

Functional SNES emulator on stock Thumby Color hardware — fits in 520 KB SRAM, runs LoROM and HiROM games, uses the device's USB-MSC + picker UX. Plays at ~5 fps in content / ~10 fps idle, with no audio and no transparency effects. **Proof of concept achieved**; making it polished would be another phase of work.
