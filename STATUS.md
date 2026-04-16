# ThumbySNES — current status (updated 2026-04-16)

> **2026-04-16 session**: DMA fast path, persistent tile-row decode
> cache, per-slot window-mask precompute, bit-lane LUT into SRAM,
> packed-32 RGB565 blend landed. Host bench (FFII, Zelda, SMW, Metroid,
> CT) shows Zelda +19%, SMW +5%, FFII +4%, Metroid +1%, CT -2% (noise).
> Device numbers pending flash.
>
> **Current bottleneck profile is UNKNOWN.** The 2026-04-13 profile in
> the "Bottleneck profile" subsection below predates both the per-line
> RGB565 composite (Pass 3) and the Thumb-2 ASM dispatcher (Pass 5) —
> `ppu_getPixel` and `cpu_doOpcode` are no longer in the hot path. A
> re-profile is the prerequisite for deciding the next big lever.

## What works

- **SNES emulation runs on stock Thumby Color hardware** (RP2350 M33 @ 300 MHz, 520 KB SRAM, 16 MB flash).
- Vendored core: **LakeSnes** (MIT, ~6,500 LOC, pure C).
- **Memory fits** comfortably: heap demand ~265 KB against 297 KB available, with all SNES state (WRAM 128 + VRAM 64 + ARAM 64 + cart SRAM 8) resident.
- **ROM lives in XIP flash** — zero-copy load via patched `cart_load`. Up to 4 MB ROMs supported (flash holds 16 MB total).
- **Per-scanline output** — patched LakeSnes PPU pixelBuffer from 978 KB → 2 KB, scanline callback fires after each visible line. Device's callback applies a **2×2 RGB565 box-blend with 7:4 stride** (FILL mode) directly into the 128×128 LCD framebuffer.
- **Picker, USB MSC, FatFs** all carry over from the ThumbyNES device-layer port.
- **Buttons**: A/B/D-pad/LB/RB → SNES A/B/D-pad/Y/X (YX profile, see `PLAN.md §7`). MENU short-press = Start, MENU long-press (≥800ms) = exit to picker.

## What doesn't work

- **Super Mario World (Europe)** hangs after ~70 frames at PC=`$809D` in an SPC handshake wait loop. SPC writes `0xBB` then stops echoing past `0x0D`. Game-specific quirk we couldn't crack — Zelda EU + FFII + DKC etc all boot fine, so it's not a PAL or LoROM issue. SMW US not tested. (The dual-core split may have changed this — not re-tested since.)
- **Color math / transparency disabled** for performance. HUD fades, lantern halos, water tints render as flat colors.
- **Mode 7** PPU paths fall through to the per-pixel slow path — works, just slowest.
- **Enhancement chips not supported** — SuperFX (Star Fox, Yoshi's Island), SA-1 (Mario RPG, Kirby Super Star), DSP-1 (Mario Kart): LakeSnes simply doesn't implement them.

## Performance

### Current (2026-04-14, post-optimization session)

| Scene | FPS | Notes |
|---|---|---|
| FFII flying intro | ~20 | Light: 1 BG layer, minimal sprites |
| FFII menus | ~20-24 | Light-medium |
| FFII first gameplay scene | ~7.2 | Heavy: full BG + sprites + DMA |
| Zelda ALTTP title/load | ~16-20 | Medium |

Audio wired end-to-end (DSP → stereo fold → PWM). SPC on core 1 is wall-clock-throttled against `time_us_64()` so production rate matches the real 1.024 MHz APU clock regardless of emulation framerate. `dsp_getSamples` tracks a `lastReadOffset` cursor and resamples the range actually produced between pulls. Pitch stays correct at any fps; tempo slows gracefully when heavy scenes starve SPC of core-1 cycles. See `README.md` → Audio section.

All buttons working (TBY_BTN_* namespace fix). LB+RB chord = Start / exit for broken MENU buttons.

### Baseline (before optimization session)

- **~4.8 fps** on FF II / Zelda content scenes (full-screen rendering, sprites + BGs)
- **~9 fps** on idle / blank screens

### Bottleneck profile — STALE (pre-pass-3, pre-pass-5)

The profile below is from before `ppu_composeLineRgb565` replaced
`ppu_getPixel` (Pass 3) and before the Thumb-2 ASM dispatcher
replaced `cpu_doOpcode` (Pass 5). Do **not** plan next-big-lever
decisions on these numbers — re-profile first.

  - 47% `ppu_getPixel`       ← function no longer in hot path
  - 15% `ppu_runLine`
  - 11% `snes_runCycles`
  - 5% `ppu_getWindowState`  ← partially mitigated by window-mask precompute (#28)
  - 4% `dsp_cycle`           ← now on core 1 (dual-core)
  - 3% `cpu_doOpcode`        ← replaced by `cpu_runBatchAsm`

*(See `PERF.md` for the full 30-item catalog of optimizations
currently active, grouped by category. Summary table below is
chronological / highest impact first.)*

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

### Next-big-lever candidates (pending re-profile)

See `README.md` → "Where time actually goes now — TBD, needs re-profile"
for the up-to-date list and reasoning. The historical claim that a
"tile-major PPU rewrite" gives 3-5× is **itself stale** — we already
decode tile-by-tile in `ppu_renderBgLine`, and the 2026-04-16 session's
tile-row cache (#27) often elides even that. Before picking the next
big project, get fresh profile numbers against the ROMs we actually
play.

What we HAVE done since the old "Real speed requires…" list:
- **Dual-core split** landed (Pass 4, listed in table above).
- **ARM asm for the 65816 dispatcher** landed (Pass 5, `src/cpu_asm.c`).
- **Persistent tile-row decode cache** landed (2026-04-16, #27).
- **Audio wired end-to-end** (DSP → PWM, runs on core 1).

What remains on the shelf, roughly ordered by expected ROI:
1. **Inline WRAM fast path in top-4 ASM opcodes** — LDA/STA imm/abs/dp
   currently BLX into `.Lrd`/`.Lwr` even for plain readMap hits.
   Bounded ~1 day of ASM. Estimated +1-2 fps.
2. **Aggressive accuracy drops** — mode 7 as flat BG, mosaic off,
   hires collapsed to mode 1. Project policy prefers speed > accuracy;
   small per-game wins, low risk.
3. **Tile-major RGB565 composite** — merge `ppu_renderBgLine` +
   `ppu_composeLineRgb565` into one pass, skipping the `bgLine[L][P]`
   intermediate. 2-3 days. Conditional on PPU still being the
   dominant cost.
4. **Half-resolution internal render (128×112 native)**. Major
   surgery; teaches the PPU half-res sampling.

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

1. **Re-profile** against FFII gameplay + Zelda overworld with the
   current code (post-pass-5, post-2026-04-16 session). Every
   next-big-lever decision is blocked on this.
2. **Hunt the SMW EU hang** — probably a single 65816 opcode behaving subtly differently on M33 vs x86, or a SPC700-driver-specific quirk in the music-upload protocol. Could do bisection by stubbing SPC reads at $2140-$2143 with synthetic ack values to see how far CPU progresses. (Dual-core split may have changed this — worth retesting.)
3. **In-game menu** — currently MENU long-press is the only control. Needs frame skip, pause, save-state, exit options.
4. **Save state / SRAM persistence** — LakeSnes has `snes_saveState` / `snes_loadBattery` already; just need to wire FatFs sidecars.
5. **Performance**: see "Next-big-lever candidates" above. Pick from that list once the re-profile is in.

## Verdict

Functional SNES emulator on stock Thumby Color hardware — fits in 520 KB SRAM, runs LoROM and HiROM games, uses the device's USB-MSC + picker UX. Audio working end-to-end. Dual-core: CPU + PPU pipeline split across both cores, SPC + DSP running free on core 1, CPU dispatch in hand-rolled Thumb-2 on core 0.

Performance: **7-25 fps depending on scene complexity** (up from
~4.8 fps baseline). Light scenes (menus, simple scroll) hit ~20-25
fps; heavy gameplay scenes (dense sprites, DMA) drop to ~7-8 fps.
Turn-based RPGs (FFII, Chrono Trigger, FF Mystic Quest) are the most
playable genre at these rates.

The bottleneck is no longer known with confidence — the old
"47% `ppu_getPixel`" profile is from before the per-line composite
and the ASM dispatcher, so it's not a reliable guide anymore. Pick
the next-big-lever off a fresh profile.

See `PERF.md` for the complete catalog of every optimization currently active.
