# ThumbySNES — Phase 2 memory audit (2026-04-13)

## Question

Can we fit snes9x2002's runtime footprint inside the ~390 KB of SRAM the
stock Thumby Color has available after firmware + display framebuffer +
drivers? (See `PLAN.md §4.1` for the budget derivation.)

## Headline finding

**No — not without surgery.** As-shipped the core needs roughly **16 MB of
heap + 744 KB of static BSS**. That's ~40× the heap budget and ~2× the
static budget. But — most of the bulk is in oversized **compile-time
buffers** and **optional features** (cheats, SA-1, DSP1 LUTs) that can be
removed or replaced with flash-resident alternatives. Realistic device
footprint after the shrinks below: **~360-400 KB — tight but viable.**

## How it was measured

`src/snes_memaudit_main.c` intercepts `malloc` / `calloc` / `realloc` /
`free` via GNU ld `--wrap` and tracks live bytes + peak watermark across
boot → `snes_load` → N frames → `snes_unload` for each ROM. Build target
`memaudit` (host-only, Phase 2-only).

Static BSS/data was read from `size` and `nm --size-sort` on
`build/libsnes9x2002.a`.

Host is x86-64 (64-bit pointers), so pointer-heavy structs (`Memory.Map`,
`Memory.WriteMap`) cost 2× what they will on the 32-bit ARM device.

## Per-ROM heap peaks

Every ROM in the collection hit the **same 15,783 KB** peak (SMW 512 KB,
Chrono Trigger 4 MB, Super Metroid 4 MB, Mario Kart + DSP1, Star Fox +
SuperFX — all identical). snes9x2002 sizes buffers at compile-time maxima
regardless of actual ROM size, so the audit collapses to one number.

## Live heap breakdown (at steady-state, any ROM)

| Source | Bytes | KB | Device fate |
|---|---:|---:|---|
| `Memory.ROM` — `MAX_ROM_SIZE + 0x200 + 0x8000` in `memmap.c:177` | 6,324,736 | **6,180** | **Drop alloc. Point `Memory.ROM` at XIP flash mmap of the FatFs file.** Same trick as `ThumbyNES/device/nes_picker.c`. |
| `GFX.Screen_buffer` — `2048 × 512 × 2 × 2 + safety` (our init) | 4,194,432 | **4,096** | Shrink to ~64 KB scanline window OR render direct into 128×128 LCD framebuffer (32 KB) + tiny one-line bounce. |
| `GFX.ZBuffer_buffer` | 2,097,280 | **2,048** | Shrink to 256×224 = 56 KB, or per-scanline 256 B. |
| `GFX.SubZBuffer_buffer` | 2,097,280 | **2,048** | Same as ZBuffer. |
| `IPPU.TileCache[TILE_2BIT]` — `MAX_2BIT_TILES=4096 × 128` | 524,288 | 512 | Shrink `MAX_2BIT_TILES` to 512 → 64 KB. Rarely saturated. |
| `IPPU.TileCache[TILE_4BIT]` — `MAX_4BIT_TILES=2048 × 128` | 262,144 | 256 | Shrink to 512 → 64 KB. |
| `IPPU.TileCache[TILE_8BIT]` — `MAX_8BIT_TILES=1024 × 128` — Mode 7 only | 131,072 | 128 | Shrink to 256 → 32 KB, or disable for non-M7 games (per-ROM config). |
| `Memory.RAM` (SNES WRAM) | 131,072 | 128 | **Keep as-is — mandatory.** |
| `Memory.SRAM` (cart save RAM, oversized) | 131,072 | 128 | Cap at 32 KB (0x8000). Largest real SNES cart SRAM is 64 KB; snes9x2002 allocates 128 KB unconditionally. |
| `GFX.X2` — 16-bit × 65536 color-math LUT | 131,072 | 128 | Move to flash const. |
| `GFX.ZERO_OR_X2` — ditto | 131,072 | 128 | Move to flash const. |
| `GFX.ZERO` — ditto | 131,072 | 128 | Move to flash const. |
| `Memory.VRAM` | 65,536 | 64 | **Keep as-is — mandatory.** |
| `IAPU.RAM` (SNES ARAM) | 65,536 | 64 | **Keep as-is — mandatory.** |
| 3× `IPPU.TileCached[...]` (validity bitmaps) | 7,168 | 7 | Shrink with tile caches. |
| DSP1 `Cos/SinTable2Fix`, `Cos/SinTable2` (runtime `malloc` copies) | ~32,000 | 32 | Already duplicated as static BSS — drop runtime allocs. |
| `misc` (small snes9x bookkeeping + memory_stream tmps) | ~138,000 | ~135 | Leave alone; most is freed after load. |
| **Total** | **~15,784 KB** | | Target: ~300 KB after shrinks (see below). |

## Static BSS breakdown (library-wide: 744 KB)

| Symbol | File | Size | Device fate |
|---|---|---:|---|
| `Cheat` | cheats.c | **309,632 B (302 KB)** | **Remove. Compile out cheats.c / cheats2.c.** Not referenced from cpu.c / memmap.c / cpuexec.c — fully isolated. Saves 302 KB BSS + 12 KB text. |
| `Echo` — `int[24000]` delay buffer | globals.c:171 | 96,000 B (94 KB) | Switch to `int16` → 48 KB, or shrink to 8000 samples → 16 KB. Some audio quality loss. |
| `Memory` struct — includes `Map[0x1000]`, `WriteMap[0x1000]` | globals.c | 90,264 B (88 KB) | On 32-bit ARM the two map tables halve to 16 KB each; real device size ~56 KB. Keep. |
| `buffer` — DMA staging | dma.c:59 | 65,536 B (64 KB) | DMA transfer staging. Shrink to 16 KB — real-world DMAs rarely exceed that in one burst. |
| `SA1_Map` | globals.c | 32,768 B (32 KB) | **Remove. Compile out sa1.c / sa1cpu.c**, `Settings.ForceNoSA1=TRUE`. Saves 64 KB BSS + ~80 KB text. |
| `SA1_WriteMap` | globals.c | 32,768 B (32 KB) | Same. |
| `bytes0x2000` | ? | 8,192 B | Investigate. |
| `SinTable2Fix` | DSP1 | 8,192 B | Move to flash const (fixed-point lookup table). |
| `DirectColourMaps` | globals.c | 8,192 B | Move to flash const. |
| `SinTable2` | DSP1 | 8,192 B | Move to flash const. |
| `CosTable2Fix` | DSP1 | 8,192 B | Move to flash const. |
| `CosTable2` | DSP1 | 8,192 B | Move to flash const. |
| `wave`, `MixBuffer`, `EchoBuffer`, `DummyEchoBuffer` | soundux | 4× 7,056 B = 28 KB | Combined with SOUND_BUFFER_SIZE = 2×44100/50. If we drop core sample rate to 32040 Hz and 60 fps: 2×32040/60=1068 samples. Saves ~22 KB. |
| `IPPU` struct | ppu.h | 4,888 B | Keep. |
| `LineData` | gfx | 3,840 B | Keep. |
| `GSU` | fxemu | 3,288 B | **Remove with SuperFX** (Settings.ForceNoSuperFX). Star Fox + Yoshi's Island won't run — acceptable per §11 non-goals. |
| `LineMatrixData` | gfx | 2,880 B | Keep. |
| (rest) | various | ~80 KB | Keep. |
| **Total** | | **~744 KB** | Target after shrinks: **~300 KB** |

## Device path — surgeries, expected savings

Priority order. Each step stands alone; cumulative savings shown.

| # | Surgery | Effort | Saves (static + heap) | Cumulative |
|--:|---|---|--:|--:|
| 1 | **Skip ROM copy — point `Memory.ROM` at XIP** | Patch `LoadROM()` in memmap.c — read header from memstream, then set `Memory.ROM = (uint8*)XIP_BASE + offset` | — +6,180 KB heap | -6,180 KB |
| 2 | **Render straight into 128×128 LCD fb — drop `GFX.Screen_buffer` to scanline-only** | Replace the 4 MB `Screen_buffer` alloc with a tiny ~16 KB line-bounce buffer, add inline blit\_fill / blit\_fit hook in the per-scanline path (Phase 4 work anyway) | — +4,064 KB heap | -10,244 KB |
| 3 | **Shrink ZBuffer + SubZBuffer to 256×224** | 2 × 56 KB instead of 2 × 2 MB | — +3,984 KB heap | -14,228 KB |
| 4 | **Move `GFX.X2` / `ZERO_OR_X2` / `ZERO` LUTs to flash const** | Patch `S9xGraphicsInit` in gfx.c — drop the malloc, point at flash array, fill at build time via generator under `tools/` | — +384 KB heap | -14,612 KB |
| 5 | **Shrink tile caches** — `MAX_2BIT_TILES 4096→512, 4BIT 2048→512, 8BIT 1024→256` | One-line ppu.h changes, stress-test to confirm no misses during gameplay | — +832 KB heap | -15,444 KB |
| 6 | **Cap `Memory.SRAM` at 32 KB** | Change malloc in memmap.c:175 | — +96 KB heap | -15,540 KB |
| 7 | **Compile out cheats.c / cheats2.c** | CMake source-list pruning + `Settings.ApplyCheats=FALSE` | -302 KB static | -15,540 h + -302 s |
| 8 | **Compile out sa1.c / sa1cpu.c** | CMake source-list pruning + `Settings.ForceNoSA1=TRUE`. Breaks Super Mario RPG, Kirby Super Star — noted in PLAN.md §11 | -64 KB static -80 KB text | -15,540 h + -366 s |
| 9 | **Move DSP1 tables to flash const + drop runtime mallocs** | dsp1emu_yo.c:77-81 | -32 KB static +~32 KB heap | -15,572 h + -398 s |
| 10 | **Drop `Cheat` runtime malloc path + shrink Echo to int16[8000]** | Small patch to soundux / globals | -80 KB static | -15,572 h + -478 s |

## Post-surgery device budget (estimated)

| Region | Size | Notes |
|---|---:|---|
| Keep (WRAM / VRAM / ARAM / Memory struct on 32-bit / IPPU etc) | ~360 KB | Non-negotiable SNES state + core bookkeeping |
| Post-shrink GFX buffers (Z + SubZ + scanline, no Screen_buffer) | ~130 KB | 256×224 Z+SubZ = 112 KB + ~16 KB line bounce |
| Post-shrink SRAM + tile caches + DMA buffer | ~150 KB | 32 + 96 + 16 + LUTs on flash |
| **Total** | **~640 KB** | **Still 250 KB over 390 KB budget** |

**Gap analysis**: the ZBuffer + SubZBuffer at 2 × 56 KB = 112 KB is still
large. They exist for BG priority and color-math depth testing. Device
option: run per-scanline, 256 bytes × 2 = 0.5 KB total. Requires confirming
snes9x2002's renderer doesn't cross-reference rows across scanlines
(probably does — color math looks at the subscreen from the main scanline).

Second option: drop Settings.Transparency / color math entirely — saves
SubZBuffer + SubScreen. Visually significant loss (many SNES games use
transparency for status bars, menus, HUD fades) but buys back ~80 KB.

Third option: the `Memory.Map` tables at 32 KB × 2 (16 KB × 2 on 32-bit)
could be replaced with a smaller hash or direct-mapped scheme — saves
~24 KB but changes the core's hot path.

## Next steps for Phase 2

1. **Prove the surgeries one at a time**, confirming the host bench still
   plays SMW between each step:
   - [ ] Patch `LoadROM` to accept a pre-mapped pointer (skip copy).
   - [ ] Drop `Memory.SRAM` to 32 KB.
   - [ ] Remove cheats.c / cheats2.c from CMake; confirm no missing symbols.
   - [ ] Remove sa1.c / sa1cpu.c from CMake.
   - [ ] Shrink tile cache maxima; run all 17 ROMs.
   - [ ] Shrink `GFX.Screen_buffer` + Z buffers to 256×224 rectangles.

2. **Re-run memaudit** after each surgery; record new watermark in this doc.

3. **Decide on transparency / color-math cut** based on the ZBuffer gap.
   This is the last knob before the per-scanline render work in Phase 4.

4. **If still over 390 KB**: the remaining gap is probably the GFX Screen +
   ZBuffer. Push to Phase 4's scanline-renderer surgery — those buffers
   become per-line local state and stop being allocated at all.

Exit criterion for Phase 2: memaudit reports heap ≤ 350 KB after all
surgeries on host build. Any fixed-cost BSS the core can't let go of we
book as a debt and test against the real device budget in Phase 3.

---

## Phase 2 results (2026-04-13)

| # | Surgery | Status | Actual saving |
|--:|---|---|--:|
| 1 | Remove cheats (gc-sections + stub `S9xInitCheatData`/`S9xApplyCheats`) | ✅ | **-186 KB BSS** |
| 2 | Compile out sa1.c / sa1cpu.c, stub 5 entry points | ✅ | **-104 KB text** (flash-only benefit; SA-1 struct stays in globals.c because memmap.c references `SA1_Map`) |
| 3 | Cap `Memory.SRAM` 128 KB → 64 KB (largest real cart SRAM = Star Fox) | ✅ | **-64 KB heap** |
| 4 | Move `GFX.ZERO` LUT to flash const | **Deferred** — USE\_OLD\_COLOUR\_OPS is off, only one 128 KB LUT allocated (not three); move is device-specific work (CMake-time generator). Skip for host phase. | (0 on host / -128 KB on device) |
| 5 | Shrink tile caches (MAX\_2BIT 512, MAX\_4BIT 512, MAX\_8BIT 256) | **REVERTED** | See note below |
| 6 | Shrink DMA `buffer` 64 KB → 16 KB (disables SDD1 games — non-goal) | ✅ | **-48 KB BSS** |
| 7 | Shrink `Echo` `int[24000]` → `int[16384]` (still covers SNES HW max 240 ms echo at 32 kHz) | ✅ | **-30 KB BSS** |
| 8 | XIP ROM (skip LoadROM copy) | **Deferred to device port** — host mmap already doesn't hit this path | (0 on host / -6,180 KB on device) |

### Failed surgery: tile cache shrink

Attempted `MAX_2BIT_TILES=512`, `MAX_4BIT_TILES=512`, `MAX_8BIT_TILES=256`.
Result: `free(): invalid pointer` inside `MemoryDeinit` — heap corruption.

**Why**: `tile.c:561-566` computes `TileNumber = (TileAddr & 0xffff) >>
BG.TileShift` with no bounds check against `MAX_*BIT_TILES`. For 4 bpp,
`TileShift=5`, so `TileNumber` ranges 0..2047. A 512-slot cache would be
indexed out-of-bounds by up to 4×. Upstream's values (4096 / 2048 / 1024)
are the **minimum correct sizes** for each bit-depth, dictated by VRAM.

**Consequence**: the 896 KB of tile-cache heap (128 + 256 + 512 KB) stays
baked in on device unless we add an eviction path to `tile.c` — turn the
indexed cache into a hash/LRU. That's a Phase 4+ surgery, not Phase 2.

### Headline numbers (host, SMW, steady state)

|  | Before | After Phase 2 | Δ |
|---|---:|---:|---:|
| Heap peak | 15,783 KB | **15,591 KB** | -192 KB (primarily SRAM cap + Echo) |
| Static BSS (libsnes9x2002.a linked into memaudit) | 744 KB | **479 KB** | **-265 KB** (cheats + DMA + Echo) |
| Static text (same) | 688 KB | **588 KB** | -100 KB (SA-1) |

### Regression check

All 17 ROMs boot + run 120-frame bench cleanly after Phase 2 surgeries
(range: Star Fox 50× to Super Metroid 277× realtime on host). Super
Mario RPG parses + runs despite SA-1 stubs — game will hang at title when
SA-1 computations are expected, acceptable per §9 non-goals.

### Device projection with all surgeries (including device-only)

| Bucket | SRAM KB |
|---|---:|
| WRAM + VRAM + ARAM + capped SRAM | 320 |
| Tile caches (can't shrink without eviction path) | 896 |
| Memory struct (64-bit host → halves on 32-bit) | ~44 |
| DMA buffer (shrunk) | 16 |
| Echo (shrunk) + other sound | ~30 |
| Other core BSS (IPPU, GSU, LineData, Settings, etc.) | ~70 |
| GFX.Screen_buffer → per-scanline bounce (Phase 4) | ~16 |
| GFX.ZBuffer + SubZBuffer → per-scanline locals (Phase 4) | ~1 |
| GFX.ZERO LUT → flash const | 0 |
| Memory.ROM → XIP | 0 |
| **Subtotal emulator** | **~1,393 KB** |

Against a 390 KB budget → **still 1 MB over**. The tile caches alone (896 KB) eat
2.3× the budget. **Phase 2 alone cannot close the gap.**

### Path to fit (what Phase 4+ must do)

1. **Replace tile cache with LRU / hashed** — 896 KB → ~16 KB (4-8 KB per
   bit-depth, evict on collision). Accepts cache-miss re-decode cost.
2. Phase 4's scanline renderer surgery (already planned).
3. Static BSS review — Memory struct map tables halve on 32-bit; smaller
   state like `Settings` struct is already small.

With (1) applied: subtotal drops from 1,393 → ~513 KB, then minus device
pointer halving (Memory struct saves ~22 KB) + scanline wins already
counted → **~491 KB**. Still ~100 KB over, and we haven't counted the
firmware overhead yet.

Further cuts if needed:
- Drop Settings.Transparency / color math subsystem (SubScreen + depth) —
  costly visually but saves another ~30 KB of stateful buffers.
- Reduce Echo further (shrink to 8192 → -32 KB with audible artifacts).
- Move Echo + DSP tables + other large const LUTs explicitly to flash.

**Honest verdict**: fitting in 390 KB requires both Phase 2 surgeries (done)
AND Phase 4+ tile-cache eviction AND probably one more knob
(transparency off OR color math off OR echo clipped). We won't know
whether we need the third knob until Phase 5 profiling on device — let
it be driven by real measurements, not speculation.

