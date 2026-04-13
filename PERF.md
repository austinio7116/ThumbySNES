# ThumbySNES — catalog of performance attempts

Every optimization currently active in the firmware, grouped by category.
Mix of things already present in the upstream LakeSnes core when we
adopted it, stock Pico SDK / toolchain levers, and targeted changes this
project has made on top.

Use this as a reference when anyone asks "have you tried X?" — if it's
listed here, it's in place; if not, it's either discarded (see
"Optimizations tried + reverted" in `STATUS.md`) or unexplored.

---

## Baseline choice

1. **LakeSnes** (pure C, ~6500 LOC, per-scanline rendering native to
   the core). Chosen over snes9x2002 because the latter's 4 MB GFX
   buffer architecture couldn't fit in 520 KB SRAM. Everything below
   stacks on top of this choice.

## System / build-level

2. **300 MHz overclock** (`set_sys_clock_khz(300000)` in
   `device/snes_device_main.c`). RP2350 spec is 150 MHz; +20% on
   raw CPU for free.
3. **`LAKESNES_HOT` → `.time_critical.snes` SRAM placement.** Hot
   functions get copied from flash XIP into SRAM at boot, dodging
   flash cache misses on the inner loop. Currently marked:
   `cpu_runOpcode`, `spc_runOpcode`, `ppu_runLine`, `ppu_renderBgLine`,
   `snes_runCycle`/`runCycles`, `snes_readBBus`/`writeBBus`,
   `snes_read`/`write`/`cpuRead`/`cpuWrite`, `dma_handleDma`,
   `apu_runCycles`, `apu_spcRead`/`Write`/`Idle`, `dsp_cycle`.
4. **`double` → `float`** for APU catch-up math. M33 has a
   single-precision FPU; doubles went through softfloat at ~50
   cycles per op.
5. **`-O3 -fomit-frame-pointer -fstrict-aliasing -ffunction-sections
   -fdata-sections`** on the core library; **`-Wl,--gc-sections`** on
   link. Dead-strips unused code.
6. **SPC fractional-cycle catch-up fix.** Upstream cast the cycle
   accumulator to `int` before running SPC opcodes, permanently
   losing fractional cycles on tight CPU polls — this is what the
   SMW hang likely is. Runs while accumulator > 0.0f. (Still in
   single-core fallback; no longer in the hot path on device
   thanks to the dual-core split below.)

## ROM / memory layout

7. **XIP zero-copy ROM.** Patched `cart_load` to take the caller's
   pointer when `thumbysnes_cart_xip=1`; `snes_loadRom` skips its
   power-of-2 mirror malloc on the XIP path. ROM lives in flash,
   accessed directly via address arithmetic in `cart_read`
   (LoROM/HiROM mapper). No bank copying into SRAM — SRAM is
   ~fully consumed by WRAM 128 + VRAM 64 + ARAM 64 + SNES state +
   hot code / driver state, ~30 KB headroom. XIP cache handles
   hot-bank locality.
8. **Per-scanline pixelBuffer.** Upstream LakeSnes carried a 978 KB
   XBGR frame buffer for the SDL frontend; patched to 2 KB (one
   line). Scanline callback fires after each visible line so the
   frontend can consume + discard. The only reason this emulator
   fits in SRAM at all.

## PPU performance knobs

9. **`skipColorMath`.** Drops subscreen fetch + add/sub blending.
   Loses transparency / fade effects; cuts per-pixel work nearly
   in half.
10. **`renderXStart`/`renderXEnd`.** Skips the ~32 invisible edge
    columns under FILL-mode crop. ~12% fewer per-pixel calls.
11. **Per-line BG cache.** `bgLine[layer][priority][x]` decoded once
    per scanline with tile-major inner loop; compositor reads the
    array instead of decoding tiles per pixel.
12. **BG empty-slot skip.** `bgLineNonEmpty[L][P]` flag lets the
    compositor skip full 256-wide passes for fully-transparent
    (layer, priority) slots — common on scenes using only BG1 +
    sprites.
13. **Per-line RGB565 full composite.** Walks the mode's layer-slot
    list once per line bottom-up, writing RGB565 directly via
    `cgramRgb565`. Replaces the per-pixel `ppu_handlePixel` walk
    (which did 8-12 slot iterations per pixel with
    `ppu_getWindowState` function calls inside the hot loop).
14. **CGRAM → RGB565 LUT with brightness baked in.** `cgramRgb565[256]`,
    lazy rebuild on $2100/$2121/$2122 writes via a `cgramDirty`
    flag. Collapses 3-channel brightness math + byte-pack into
    one `uint16_t` read.
15. **Tile-decode bit-lane LUT.** 256-entry byte→8-lane expansion
    table used by `ppu_renderBgLine` AND `ppu_evaluateSprites`.
    Replaces 8 × {shift + AND + OR} inner loops with pointer
    derefs + array reads. ~2× faster per 4bpp tile.
16. **Native-LCD mode** (opt-in via `snes_set_lcd_mode`, NOT
    default). PPU composites RGB565 directly into the 128×128
    device framebuffer at native resolution (128 of 256 cols
    sampled, ~43% of lines skipped). Nearest-neighbour — text
    unreadable, so held back from default. Kept as per-ROM opt-in.
17. **Frameskip** (opt-in via `snes_set_frameskip(n)`, NOT enabled).
    Sets a `skipRender` flag that bails at the top of `ppu_runLine`
    before sprite eval + BG cache + composite. Halves PPU cost at
    the price of visible-fps halving.

## CPU / bus / scheduler

18. **Batched cycle scheduler.** `snes_runCycles` fast-forwards hPos
    between events (0 / 16 / 512 / 1104 / hTimer×4 / wrap territory)
    instead of looping `snes_runCycle` per 2 master cycles.
    Advances `snes->cycles`, `snes->hPos`, `autoJoyTimer` in bulk;
    re-evaluates IRQ condition at batch end with the false→true
    edge detector intact. ~300× fewer slow-path calls per frame.

## Dual-core split

19. **`THUMBYSNES_DUAL_CORE=1`** compile-time flag on device build.
20. **Core 0**: 65816 CPU + PPU + LCD SPI DMA + bus + DMA + input +
    USB. **Core 1**: SPC700 + DSP only (audio DSP, not output).
21. **Core 1 entry** (`snes_apu_core1_loop`) parks reading a shared
    `Snes*` pointer; launched once at boot via
    `multicore_launch_core1` and starts running `spc_runOpcode` in a
    tight loop whenever a ROM is loaded.
22. **`snes_catchupApu` compiles to no-op** on dual-core builds.
23. **Per-master-cycle `apuCatchupCycles` accumulator update skipped.**
    Saves an FPU multiply + add per ~1–2M `snes_runCycle` calls per
    frame.
24. **Byte-atomic `inPorts` / `outPorts`.** No mailbox or spinlock;
    byte writes/reads are atomic on M33, and games' polling loops
    handle cross-core latency naturally. Side benefit: SMW's SPC
    handshake hang may resolve as the SPC is no longer starved of
    cycles during tight CPU polls.

---

## Known gaps (NOT yet attempted — ordered by expected impact)

These aren't active optimizations, listed here for honesty:

- **Persistent cross-line tile cache.** Decode each tile once per
  *frame* (invalidated on VRAM write) instead of per line. Most
  tiles don't change frame-to-frame. snes9x2005 does this.
  Estimated 2-3× on BG decode. Significant work (~days).
- **ARM asm for 65816 dispatcher hot opcodes.** snes9x2002 has it;
  LakeSnes is pure C. Bounded scope but hand-rolled asm.
  Estimated 1.5-2× on CPU dispatch.
- **Cross-core pipelining beyond APU.** Currently core 1 only runs
  SPC+DSP. Ship sprite eval and/or per-line BG cache decode to
  core 1 as a producer, core 0 compositor consumer. Tricky timing
  and cross-core memory traffic; may or may not win.
- **Audio wiring.** Infrastructure exists end-to-end
  (`snes_audio_pwm.[ch]`, `snes_setSamples`), just not connected.
  Doesn't affect fps; useful once fps is high enough that audio
  plays at approximately correct pitch.
