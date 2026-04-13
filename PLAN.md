# ThumbySNES — SNES Emulator Firmware for Thumby Color

A standalone bare-metal firmware (mirroring **ThumbyNES** / **ThumbyP8**) that
turns the Thumby Color into a pocket SNES. Flash one `.uf2`, drag `.smc` /
`.sfc` ROMs onto the device over USB MSC, pick one from a cart picker, play
with sound + LCD + buttons.

This document is the **plan**. The repo has been scaffolded (copied from
ThumbyNES); the emulator core is not yet wired up. Decisions, trade-offs,
vendor choices, and a phased build order follow.

---

## 1. Hardware budget recap (Thumby Color, unchanged from ThumbyNES)

| Resource | Amount | SNES implication |
|---|---|---|
| MCU | RP2350 dual-core Cortex-M33 @ up to 300 MHz | Core0 = 65C816 + PPU + LCD, Core1 = SPC700 + DSP + audio DMA feed |
| SRAM | 520 KB | **Binding constraint.** ~390 KB free for emulator after firmware. SNES state = 128+64+64 KB = 256 KB minimum. |
| Flash | 16 MB (≈12 MB usable as FAT after firmware) | 4 MB SNES ROMs fit easily. **ROMs live in flash, XIP-mapped, zero SRAM cost.** |
| Display | 128×128 RGB565, GC9107, DMA SPI | SNES is **256×224** — must downscale 2:1 horizontally, letterbox or crop vertically |
| Audio | 9-bit PWM + sample IRQ (proven on P8 / NES) | Run DSP mixer at 22050 Hz mono, same path |
| Input | A, B, D-pad, LB, RB, MENU | Maps cleanly onto SNES (A, B, X=LB, Y=RB, Start=MENU, Select=?) |
| USB | TinyUSB MSC class (proven on NES) | Drag-and-drop `.smc` / `.sfc` onto a FAT volume |

**Headline constraints**:
1. **SRAM is tight, not impossible.** ~390 KB free after firmware. SNES needs
   ~302–332 KB for WRAM+VRAM+ARAM+state+emulator working set. Leaves a
   60–100 KB margin — usable, but every alloc counts. See §4.
2. **CPU is the binding constraint.** M33 @ 250 MHz = ~4.17M cycles/frame.
   Optimized SNES cores want ~3.3M cycles/frame. ~80% utilization with no
   headroom for cache misses or heavy scenes. Dual-core (CPU+PPU on core0,
   SPC700+DSP on core1) + `.time_critical` SRAM placement + ARM asm for
   hot paths are mandatory, not optional.
3. **Screen resolution mismatch.** SNES 256×224 → 128×128 via 2:1 horizontal
   downscale inline in the PPU line renderer (like ThumbyNES does for the
   256-wide NES output). Vertically 224/2 = 112, giving 16 px of letterbox.
4. **No Super FX, no SA-1, no DSP-1/2/3/4, no C4.** Those chips require
   co-processor emulation. Start with LoROM + HiROM without enhancement
   chips. Revisit if there's budget left.

---

## 2. Vendor choice: which SNES core?

We are not writing a 65C816 + PPU + SPC700 + DSP from scratch. Candidates,
ranked by fitness:

| Core | Lang | Size | Accuracy | License | Notes |
|---|---|---|---|---|---|
| **snes9x2002** (libretro fork of snes9x 1.39) | C | ~20k LOC | Low–Medium | non-commercial (snes9x) | ARM-optimized, designed for constrained handhelds (GP2X, Dingoo). Has ARM asm for 65C816. **First choice.** |
| **snes9x2005** (libretro fork of snes9x 1.43) | C | ~40k LOC | Medium | non-commercial (snes9x) | Better accuracy, ~2x the CPU cost. Fallback if 2002 doesn't render a given game. |
| **snes9x2005-plus** | C | ~45k LOC | Medium+ | non-commercial | Snes9x 2005 + upstream patches, slightly slower. |
| **bsnes / higan** | C++ | huge | Perfect | GPLv3 | Not realistic on M33. |
| **mednafen-snes** | C++ | huge | High | GPLv2 | Too large. |

**Recommendation: vendor `snes9x2002`** (from libretro's fork repo).
- Specifically designed for ARM handhelds (GP2X at 200 MHz ARM9, Dingoo A320
  at 336 MHz MIPS). That is the neighborhood of our M33 @ 250 MHz.
- Ships with ARM assembly paths for the 65C816 and 8×8 / 16×16 tile renderers.
- Library size (~20k LOC) is manageable; hot path is small.
- Known to drop enhancement chips gracefully — LoROM/HiROM just work.
- Prior art: multiple RP2040/RP2350 experiments exist using this core.

**License hygiene**: snes9x carries a non-commercial, source-available
license ("You may use this free of charge. You may not sell or distribute
this for profit... You must include the copyright notice"). Compatible with
hobbyist firmware distribution as long as we ship source, carry the
`LICENSE` / `snes9x-license.txt` file through, and don't monetize the
firmware. Record this in `vendor/VENDORING.md` with the exact upstream
commit. Our firmware glue stays under the same terms for clean redistribution.

**Fallback plan**: if `snes9x2002` proves too slow after Phase 5
optimization, we evaluate whether the PPU is the bottleneck (replace with a
simplified scanline renderer that drops color math and HDMA) or the CPU is
(write a small subset of 65C816 ops in ARM asm). We do NOT switch core mid-way
— snes9x2002 is already the smallest accurate choice.

---

## 3. Repository layout (mirrors ThumbyNES / ThumbyP8)

```
ThumbySNES/
├── PLAN.md                       ← this file
├── README.md                     ← user-facing, expanded once Phase 1 lands
├── CMakeLists.txt                ← host build (SDL2 runner + bench)
├── src/
│   ├── snes_core.h / .c          ← Thumby-side glue around snes9x2002
│   ├── snes_bench_main.c         ← headless N-frame bench (Phase 0)
│   └── snes_host_main.c          ← SDL2 host runner (Phase 1)
├── vendor/
│   ├── VENDORING.md              ← upstream refs + any patches
│   └── snes9x2002/               ← (not yet vendored — Phase 0)
├── device/
│   ├── CMakeLists.txt            ← RP2350 firmware build (ported Phase 3)
│   ├── fatfs/                    ← copied from ThumbyNES — reusable as-is
│   ├── tusb_config.h             ← copied from ThumbyNES — reusable
│   ├── usb_descriptors.c         ← copied from ThumbyNES — reusable
│   ├── snes_lcd_gc9107.[ch]      ← port of nes_lcd_gc9107.[ch]
│   ├── snes_buttons.[ch]         ← port of nes_buttons.[ch]
│   ├── snes_audio_pwm.[ch]       ← port of nes_audio_pwm.[ch]
│   ├── snes_flash_disk.[ch]      ← port of nes_flash_disk.[ch]
│   ├── snes_font.[ch]            ← port of nes_font.[ch]
│   ├── snes_menu.[ch]            ← port of nes_menu.[ch]
│   ├── snes_picker.[ch]          ← port of nes_picker.[ch]
│   ├── snes_battery.[ch]         ← port of nes_battery.[ch]
│   ├── snes_msc.c                ← port of nes_msc.c
│   ├── snes_device_main.c        ← port of nes_device_main.c
│   ├── snes_run.[ch]             ← emulator driver (core0 entry)
│   └── thumby_state_bridge.[ch]  ← port of thumby_state_bridge.[ch]
├── tools/                        ← LUT generators if needed
└── firmware/
    └── snesrun_device.uf2        ← release binary (Phase 3+)
```

Almost everything under `device/` below the core is a rename + light port
from ThumbyNES (LCD, buttons, PWM audio, FatFs, USB MSC, menu, picker, font,
battery monitor, state bridge). That layer is proven and reusable — the
emulator-specific code lives in `snes_run.[ch]`, `snes_core.[ch]`, and the
vendored `snes9x2002/`.

### 3.1 Display modes (confirmed 2026-04-13 on host)

Three scale modes cycled by the in-game menu (same pattern as ThumbyNES's
`blit_fit` / `blit_blend` / `blit_crop`). Decided after seeing SMW rendered
in each mode:

| Mode | Math | Use case |
|---|---|---|
| **FILL** (default) | 224×224 square crop (16 px off each horizontal edge) → 128×128 via 7:4 stride with 2×2 RGB565 blend — fills the whole screen | Action games — uses every pixel; sprites twice as tall as FIT. Lose 16 columns on each horizontal side. |
| **FIT** | 256×224 → 128×112 via 2×2 RGB565 box average, centered with 8 px letterbox top/bottom | Games that keep essential info near the left/right edges (e.g. HUD strips). |
| **CROP** | 1:1 128×128 window, panned (default center = (64, 48)) | Text-heavy screens — menus, dialog boxes, password entry in RPGs. |

**FILL rationale**: SMW at FIT was "pretty hard to read" per user — sprites
compressed to 112 rows of 128 px means everything is half-height. FILL gives
128 rows of 128 px (full-height sprites) at the cost of 16 columns off each
side, which is mostly dead HUD or just repeating sky in platformers. 7:4
stride + 2×2 blend keeps the math cheap (every output pixel is still a
box-average of 4 source pixels, just sampled at uneven stride).

Per-ROM config can pin a game to whichever mode + pan offset works best.
Implementation lives in `src/snes_host_main.c: downscale_fill /
downscale_fit` for host testing; Phase 4 ports these inline into the PPU
scanline renderer on device.

---

## 4. Memory plan

### 4.1 SRAM budget (device build, bare metal)

| Region | Size | Notes |
|---|---|---|
| Total SRAM | 520 KB | RP2350 spec |
| `.time_critical.*` (hot loops in SRAM) | ~60 KB | CPU dispatcher, PPU line render, DSP mixer |
| Stack + C runtime | ~16 KB | Per-core stacks + heap metadata |
| LCD framebuffer (128×128 RGB565) | 32 KB | Single buffer; render scanline-by-scanline, no double-buffer. |
| Audio ring buffer | 4 KB | 2048 samples × 16-bit stereo, or 4096 mono. |
| FatFs + TinyUSB + drivers | ~20 KB | FF work area, TUD buffers, disk cache |
| **Available for emulator state** | **~388 KB** | |

### 4.2 SNES state (lives in SRAM)

| Region | Size | Notes |
|---|---|---|
| WRAM (`$7E0000` mirror) | 128 KB | Main CPU RAM. Non-negotiable. |
| VRAM | 64 KB | PPU tile / tilemap / OAM backing. Read every scanline; **must stay in SRAM**. |
| ARAM | 64 KB | SPC700 audio CPU RAM. |
| OAM + CGRAM + misc | ~4 KB | 544 B OAM + 512 B CGRAM + registers + shadows |
| CPU + SPC700 + PPU + DMA state | ~10 KB | Emulator struct bookkeeping |
| Emulator working buffers + LUTs | ~30–60 KB | Pre-decoded tile caches, tint tables, SPC echo buffer, ARM asm scratch |
| Scanline render buffer (256×8bpp + priorities) | ~2 KB | One line at a time; downscale 2:1 into LCD framebuffer on flush |
| **Total** | **~302–332 KB** | |

### 4.3 ROM (lives in flash, XIP-mapped)

- `.smc` / `.sfc` files land on the FatFs partition via USB MSC.
- Picker memory-maps the chosen file at `0x10000000 + cluster_chain_base` —
  same trick ThumbyNES uses for `.nes`. Zero SRAM cost for ROM.
- Mapper tables (`MemoryMap.Map[]`) are sized from the header and built in
  SRAM at boot (~1–2 KB).
- LoROM / HiROM detection: standard snes9x heuristic (checksum + vector
  sanity at $00:7FD5 vs $00:FFD5).
- Hard cap 4 MB ROM for Phase 1. Larger ROMs exist (~6 MB `Tales of
  Phantasia`) — defer until Phase 6.

### 4.4 Margin

388 (avail) − 332 (worst-case emulator) = **~56 KB margin**. Realistic
tracked working margin once snes9x2002 is instrumented, expected 60–100 KB.
Any single core-dependent allocation over ~50 KB (e.g. BG3 high-res cache,
HDMA per-scanline snapshot) is a red flag and must land in flash const data
or be dropped.

---

## 5. CPU plan

### 5.1 Budget

- M33 core clock: **250 MHz** (standard Thumby Color clock; 300 MHz possible
  but thermally dubious for sustained work, and PSU draw matters on battery).
- Per-frame budget at 60 fps: **4.17M cycles**.
- snes9x2002 typical on GP2X @ 200 MHz ARM9 (no cache!): real-time for
  non-enhancement-chip games with occasional frameskip. M33 has better IPC
  but similar budget. **Expect parity only with dual-core + SRAM hot code.**

### 5.2 Dual-core split (same pattern as ThumbyP8, ThumbyGB, ThumbyNES)

- **Core 0**: 65C816 CPU dispatch, memory map, DMA/HDMA, PPU scanline render,
  LCD flush. Drives the 60 Hz frame timer.
- **Core 1**: SPC700 CPU, DSP mixer, audio ring-buffer fill. Pulled by a
  PWM-sample IRQ at 22050 Hz.
- Sync: after each SNES scanline, core 0 posts a timestamp to core 1; core 1
  catches the SPC700 up to that timestamp before returning to its audio
  pump. Lock-free SPSC queue in SRAM.

### 5.3 Hot-path placement

- `S9xMainLoop`, `S9xDoHEventProcessing`, `S9xDoHBlankProcessing` → SRAM
  (`.time_critical.snes`, analogous to ThumbyNES's `IRAM_ATTR`).
- `S9xUpdateScreen` line renderer and tile decoders → SRAM.
- SPC700 `APU_EXECUTE1` core loop → SRAM on core 1.
- DSP `Envelope` / `MixStereo` → SRAM on core 1.
- Everything else (boot, ROM parse, save state, menu) stays in flash.

### 5.4 ARM asm opportunities

- snes9x2002 ships ARM asm for 65C816 in `tables.s` / `c4.s` and an 8×8
  tile blitter; **use as-is** where the ARM9 calling convention matches
  M33. Most of it is plain ARMv5TE so it ports 1:1 to ARMv8-M mainline
  except for deprecated ops — audit on landing.
- If CPU-bound, the opcodes to hand-tune next are: `ADC`/`SBC` decimal mode,
  `JSR`/`RTS`, indirect-long addressing. These dominate profile on many games.

---

## 6. Audio plan

- Reuse the PWM + DMA path proven by ThumbyP8 and ThumbyNES.
- 22050 Hz, mono mix (SNES DSP outputs 32 kHz stereo — downsample + fold to
  mono inline in the mixer).
- Ring buffer: 4 KB (~185 ms at 22050). DMA half-full IRQ swaps halves.
- DSP echo buffer (`EON` / `ESA`) lives in ARAM — no extra allocation.
- Cost of 32→22 kHz resample: use linear interp per sample, < 0.5% CPU on
  core 1. Good enough for a 128×128 handheld.

---

## 7. Input mapping

SNES has 12 buttons (A B X Y + L R + Start Select + 4-dir D-pad); Thumby has
6 (A B LB RB MENU + 4-dir D-pad). We are 2 short. Decision baked in 2026-04-13
after SMW was running on the host:

### Default profile — "YX"  *(platformer / action-adventure / RPG)*

| SNES | Thumby | Notes |
|---|---|---|
| A | A | Primary action / jump in most games |
| B | B | Primary attack / alt jump |
| Y | LB | Run / secondary attack — the held button |
| X | RB | Menu / item cycle |
| Start | MENU short-press | Pause |
| Select | MENU double-tap | Rare in-game — rarely needed during fast action |
| D-pad | D-pad | |
| L / R | — | Not mapped |
| (open) | MENU long-press | In-game menu (frameskip, profile swap, save state) |

### Alternative profile — "LR"  *(Metroid / Contra III / Castlevania IV / Mario Kart)*

| SNES | Thumby |
|---|---|
| L | LB |
| R | RB |
| X / Y | — (not mapped) |

Everything else matches the YX profile.

### Rationale

Y is needed in far more games than L/R — losing Y breaks SMW's run (shells,
hold-jump, fireballs). Losing L/R degrades Metroid / Castlevania but doesn't
break them. YX is therefore the safer default for the golden-path title.
LR profile exists for games where L/R is mechanically essential.

Menu-chord modifiers (hold MENU, press A/B) were considered for X/Y/L/R but
rejected: these are high-frequency actions (run, aim-diagonal) and holding a
chord during continuous gameplay is unworkable on a 6-button handheld.

### Implementation

- Profile is a per-ROM config flag stored via the save API (default YX).
- In-game menu exposes a profile toggle (Phase 6).
- 6-button fighting games (SF II Turbo, Mortal Kombat) lose either way —
  listed in §11 as "known degraded experience".

Phase 1 host build hard-codes YX (keyboard `A` → SNES Y, `S` → SNES X, `Q`/`W`
→ L/R for testing). Phase 3+ device build reads the profile from save.

---

## 8. Phased build order

### Phase 0 — Scaffold + vendor the core *(scope for Day 1)*
- [x] Copy ThumbyNES structure into ThumbySNES
- [x] Strip NES / SMS / GB vendor + glue
- [x] Write PLAN.md (this file)
- [x] Vendor `snes9x2002` under `vendor/snes9x2002/`
      at commit `15826a2afc1474b30c310502b5dbe25c639be59d`.
- [x] Get `snes9x2002` compiling standalone (host cc, x86-64) as a static lib,
      non-ARM raster path (`ppu_.c` + `gfx.c` + `tile.c`, `-D__OLD_RASTER_FX__`).
- [x] `src/snes_core.c` drives init (MemoryInit / S9xInitAPU / S9xInitSound /
      S9xGraphicsInit / GFX buffers / `memstream_set_buffer` + `LoadROM` +
      `S9xReset` / `S9xMainLoop`) and provides the host-glue stubs the core
      expects (S9xReadJoypad, S9xDeinitUpdate, S9xMessage, S9xSyncSpeed,
      overclock knobs, ROMAPUEnabled).
- [x] `src/snes_bench_main.c` mmap's a ROM, runs N frames, prints fps.

**Exit criterion met 2026-04-13**: `./build/snesbench "roms/Super Mario World (Europe).sfc" 600`
→ 600 frames in 0.106s = 5643 fps = 94× realtime. All 17 ROMs in the test
collection parse + run headlessly (SMW, Zelda ALTTP, Super Metroid, Mario
Kart+DSP1, Chrono Trigger, Star Fox+SuperFX, DKC: 35×–172× realtime).

### Phase 1 — Host SDL2 runner
- [x] `src/snes_host_main.c` — SDL2 window, 256×224 output scaled 2× for
      visibility, 32040 Hz stereo audio via `SDL_OpenAudioDevice`.
      (Sample rate matched to core's native output; device build will
      resample to 22050 in Phase 5.)
- [x] Keyboard input mapped to SNES controller 1 (YX profile, see §7).
- [x] Verified on *Super Mario World* — playable, audio clean, controls
      responsive. Next verification pass: *Zelda: A Link to the Past*,
      *Super Metroid*, *F-Zero* once a ROM is available.
- [x] 256→128 downscale prototyped in sneshost (TAB cycles FILL / FIT / FULL).
      Both downscale modes use the 2×2 RGB565 box-average ported from
      `ThumbyNES/device/nes_run.c: blit_blend` — simpler than NES because
      snes9x2002 already emits RGB565 directly. FILL is the default: a 224×224
      crop scaled 7:4 → 128×128 to fill the whole screen. FIT keeps full
      horizontal coverage with letterbox. See §3.1.

**Fixes landed during Phase 1**:
1. `S9xMixSamples` overflowed its static 1764-sample `MixBuffer` when SDL
   requested 2048 samples/callback — `snes_get_audio` now chunks.
2. Framebuffer read used `GFX.Pitch` (2048); `__OLD_RASTER_FX__` raster path
   writes with stride `GFX_PITCH` = 640. Corrected.
3. SDL rate was 22050 against a 32040 core output — fixed to 32040 so host
   audio isn't pitched / drained wrong.

**Exit criterion**: SMW playable at full speed with audio on desktop ✅
(downscale proof still to do).

### Phase 2 — Memory audit
- [ ] Instrument `snes9x2002` allocations (hook `malloc` / `calloc` and log).
- [ ] Confirm worst-case SRAM footprint ≤ 390 KB across the test-ROM set.
- [ ] If over budget: move fixed LUTs to flash const (`__attribute__((section(".rodata")))`),
      shrink pre-decoded tile caches, drop color-math scratch.
- [ ] Decide whether to memory-map ROM from host file (simulates XIP) or
      `memcpy` into RAM (simulates the failure case). Both behind flags.

**Exit criterion**: bench reports peak heap ≤ 390 KB including VRAM/WRAM/ARAM.

### Phase 3 — Device firmware skeleton
- [ ] Port `device/nes_*` → `device/snes_*`: LCD, buttons, audio, flash disk,
      MSC, menu, picker, font, battery, state bridge. Mostly mechanical
      rename + header-guard updates.
- [ ] `device/snes_device_main.c` — lobby + USB-MSC + picker. No emulator
      wired in yet. Should boot, show a picker, list `.smc` files, exit
      cleanly on MENU.
- [ ] `device/snes_run.c` — empty stub that clears the screen and returns.

**Exit criterion**: a `.uf2` flashes, device enumerates as USB MSC, picker
lists ROMs, selecting a ROM returns to picker (no emulator).

### Phase 4 — Single-core device emulator
- [ ] Hook `snes_run.c` to `snes_core.[ch]` glue; boot the picked ROM from
      flash XIP.
- [ ] Inline 256→128 downscale in PPU line renderer → write straight into
      LCD framebuffer.
- [ ] `.time_critical.snes` section for the hot paths listed in §5.3.
- [ ] Silent (no audio) — just prove the CPU+PPU loop fits and renders.

**Exit criterion**: SMW boots, title screen visible, probably sub-real-time.
We establish the frames/second baseline here.

### Phase 5 — Dual-core + audio + performance
- [ ] Split SPC700+DSP onto core 1, wire the SPSC scanline-timestamp queue.
- [ ] Turn on PWM audio via the proven NES / P8 path.
- [ ] Profile with the test-ROM set, move next-hottest functions into SRAM.
- [ ] Evaluate ARM asm paths already in snes9x2002's `tables.s` — enable if
      the M33 decode matches; port / replace if not.
- [ ] Add frameskip knob (per-ROM config) if some titles can't hit 60.

**Exit criterion**: SMW at 60 fps with audio on device. F-Zero at ≥ 40 fps
with optional frameskip. A Link to the Past playable.

### Phase 6 — Polish
- [ ] Full 12-button input scheme (L/R, Select).
- [ ] SRAM save support (`.srm` sidecar on FatFs volume).
- [ ] Save states (reuse `thumby_state_bridge` — SNES state is large, ~300 KB;
      may need to chunk-write to avoid the FatFs sector cache thrashing).
- [ ] Per-ROM config (frameskip, audio on/off, palette tweaks).
- [ ] Battery / rumble / indicator LED polish as on ThumbyNES.

**Exit criterion**: feature parity with ThumbyNES firmware for the supported
game subset.

---

## 9. Known risks & non-goals

**Risks**:
1. **CPU shortfall.** Even with dual-core + SRAM hot code, some games (heavy
   HDMA, Mode 7 with rotation, dense transparency) may land at ~30–45 fps.
   Frameskip is the safety valve. We are not a cycle-accurate emulator.
2. **Memory cliff.** If VRAM/WRAM/ARAM + snes9x2002 working set overruns
   390 KB, there is no fallback on stock Thumby Color. We would need to drop
   features from the core (disable HDMA, disable SPC700 echo, disable color
   math).
3. **Save state size.** A full SNES snapshot is ~320 KB. Writing that to
   FatFs at 30 fps is not realistic — save state is a menu action, not an
   auto-save.
4. **Audio quality.** 22050 mono from a 32 kHz stereo source is noticeably
   degraded. Accept as the platform limit.

**Non-goals (Phase 1–6)**:
- Super FX, SA-1, DSP-1..4, Cx4, S-DD1, SPC7110, OBC-1 — no enhancement
  chips. The LoROM/HiROM library is plenty for a handheld.
- BS-X, Sufami Turbo, Satellaview — out of scope forever.
- Net play / USB link multiplayer.
- Rewindable history (high RAM cost).
- Per-pixel Mode 7 perspective with floating-point — keep the fixed-point
  snes9x2002 path.

**Not decided yet** (Phase 0/1 data informs):
- Whether to enable HDMA at all in the first playable build, or stub it
  (kills `F-Zero` horizon, but buys significant CPU).
- Whether SPC700 runs cycle-accurately or uses snes9x2002's fast "HLE-ish"
  mode.
- Whether to overclock to 300 MHz (measure thermals + power first).

---

## 10. Day 1 checklist (tomorrow)

Follow this order — each step is a forcing function for the next.

1. `cd vendor && git clone --depth 1 https://github.com/libretro/snes9x2002`,
   pin the commit in `VENDORING.md`, copy `LICENSE` up to `vendor/` root.
2. Delete retro-arch callback files we won't use (`libretro.c`,
   `libretro-common/`). Keep `src/`, `cpu/`, `apu/`, `dsp/`, `gfx/`, `tables/`.
3. Extend the top-level `CMakeLists.txt` to build `snes9x2002` as a static
   lib with the same warning-suppression flags used for nofrendo.
4. Fill in `src/snes_core.[ch]` stubs — thin `snes_load / snes_run_frame /
   snes_reset / snes_get_audio` wrapper around `S9xLoadROM`, `S9xMainLoop`,
   `S9xReset`, `S9xMixSamples`.
5. Fill in `src/snes_bench_main.c` — mmap a ROM, call `snes_load`, loop
   `snes_run_frame` 600 times, print wall clock + perf stats.
6. First build: `cmake -B build -S . && cmake --build build -j8 && ./build/snesbench path/to/smw.smc 600`.
7. When it runs to completion, that's the Phase 0 exit criterion. Move to
   Phase 1 (SDL2).

Expect the compile to not be clean on first try — snes9x2002 assumes various
retro-arch headers. Work through the undefineds with stubs in
`src/snes_core.c`.

---

## 11. Success definition

- **Minimum viable**: Super Mario World playable at 60 fps with audio on a
  stock Thumby Color, via USB-MSC ROM drag-and-drop, with a ROM picker and
  in-game menu.
- **Stretch**: A Link to the Past and F-Zero playable at ≥ 45 fps with audio.
- **Not-required**: any game with an enhancement chip, save states, or L/R
  buttons in Phase 1.

If we hit the minimum, this is a successful hobby firmware on par with
ThumbyNES. Everything beyond is a nice-to-have on top of a working base.
