# ThumbySNES

A bare-metal SNES emulator firmware for the [Thumby Color](https://thumby.us/)
(RP2350 Cortex-M33 @ 300 MHz, 520 KB SRAM, 128×128 RGB565 LCD).
Flashes as a single `.uf2`, exposes the device as a USB drive for dragging
`.smc` / `.sfc` ROMs onto it, and runs them with a ROM picker and in-game
menu — the same shape as [ThumbyNES](../ThumbyNES).

Built on a heavily-modified [LakeSnes](https://github.com/elzo-d/LakeSnes)
core. CPU dispatch is a 256-handler ARM Thumb-2 inline-assembly state
machine running in `time_critical.snes` flash; PPU/SPC/DSP run on the
second core; LCD output is a per-line 2×2 RGB565 blend with horizontal
crop ("FILL" mode).

## Build

### Host (development / test ROM verification)

```bash
cmake -B build -S .
cmake --build build -j8
./build/snesbench path/to/rom.smc 600    # headless N-frame benchmark
./build/sneshost  path/to/rom.smc        # SDL2 runner (needs libsdl2-dev)
```

### Device (RP2350 firmware)

```bash
cmake -B build_device -S device \
      -DPICO_SDK_PATH=$HOME/mp-thumby/lib/pico-sdk
cmake --build build_device -j8
# Output: build_device/snesrun_device.uf2
# Stage:  cp build_device/snesrun_device.uf2 firmware/snesrun_device.uf2
```

Hold DOWN dpad while powering the device on to enter BOOTSEL — it
mounts as a USB drive in Windows; drag the `.uf2` over to flash.

## In-game controls

| Combo                          | Effect                                  |
| ------------------------------ | --------------------------------------- |
| MENU (long-press, ≥ 800 ms)    | exit to ROM picker                      |
| MENU (tap)                     | SNES Start                              |
| LB+RB chord (long-press)       | exit to ROM picker (broken-MENU units)  |
| LB+RB chord (tap)              | SNES Start                              |
| **MENU + UP** (edge)           | increase frameskip (0 → 1 → 2)          |
| **MENU + DOWN** (edge)         | decrease frameskip (2 → 1 → 0)          |

Default frameskip is 1 (render every other frame). The CPU and APU
emulate at full rate regardless of frameskip — only PPU rendering is
suppressed — so audio pitch and game logic are unaffected.

A small FPS counter sits in the top-left corner of every game.

## Architecture notes

- **Dispatch**: `cpu_runBatchAsm` (`src/cpu_asm.c`) runs 64 opcodes per
  call as a single Thumb-2 routine in flash. Pinned ARM registers:

  ```
  R4  = A          R5  = X          R6  = Y         R7  = SP
  R8  = PC         R9  = packed P   R10 = Cpu*      R11 = Snes*
  ```

  Helpers (`.Lrd`, `.Lwr`, `.Lidl`, `.Lfetch`, `.Lzn8`, `.Lzn16`,
  `.Lint`, …) preserve `r4-r11`, save LR through the `FLR` stack slot,
  and call back into the C-side `snes_cpuRead` / `snes_cpuWrite` /
  `snes_cpuIdle` handlers via function-pointer slots in the dispatch
  frame.

- **PPU on core 1**: `s_ppu_pipeline_line` / `s_ppu_pipeline_done`
  hand off scanline rendering. Core 0 dispatches a line at hPos=512 and
  continues running the CPU; core 1 composites and pushes to LCD. The
  scanline blend callback (`on_scanline_565`) does a 2×2 RGB565 average
  with horizontal-crop FILL.

- **SPC700 + DSP on core 1**: free-running, sample-driven. Core 0 pulls
  resampled stereo from `dsp_getSamples` once per emulated frame and
  pushes mono into the PWM ring at 22050 Hz.

## ASM dispatcher debugging history (April 2026)

The Thumb-2 dispatcher was generated opcode-by-opcode then merged into
one giant naked function. After the merge games rendered black or
glitched and the CPU appeared to hang. Two latent bugs were responsible
for almost every failure mode:

### Bug 1 — `r0` clobber by `.Lidl` / `.Lwr`

These two helpers set `r0 = r11` (Snes pointer) before `blx`-ing the
C-side `snes_cpuIdle` / `snes_cpuWrite`. They restored LR via the `FLR`
stack slot but left `r0` in its post-call state.

The RMW body (`.Lrmw{8,16}_body`, used by `ASL`, `LSR`, `ROL`, `ROR`,
`INC`, `DEC`, `TRB`, `TSB` for memory operands) reads the byte into
`r0`, calls `.Lidl` for the modify cycle, then expects `r0` still to
hold the byte before invoking the per-op routine pointer through `S4`.
The pre-fix sequence ended up shifting the *Snes pointer* and writing
the result back to memory — corrupting whatever the dp / abs / abs,X
address pointed at. Real games saw broken VRAM uploads and OAM tables;
the dedicated test ROM hung at test 49 (ASL dp).

**Fix**: a new `SR0` stack slot. `.Lidl` and `.Lwr` save caller-`r0`
into `SR0` on entry and restore it before `bx lr`. The fix is wholly
in the helpers, so all 173+ callsites benefit without per-site changes.
`FSZ` bumped 72 → 80 to accommodate the slot (8-byte aligned).

```asm
.Lidl:                                       ; r0 = Snes* internally,
    str  r0, [sp, #SR0]                      ; preserved for caller
    str  lr, [sp, #FLR]
    mov  r0, r11
    movs r1, #0
    ldr  r3, [sp, #FID];  blx r3             ; snes_cpuIdle(snes, false)
    ldr  lr, [sp, #FLR]
    ldr  r0, [sp, #SR0];  bx lr
```

### Bug 2 — Missing Thumb bit on internal function pointers

The RMW body indirects through a per-opcode routine pointer stored in
`S4`:

```asm
ldr  r3, =.Lasl_rmw8     ; load address of internal helper
str  r3, [sp, #S4]
b.w  .Lrmw8_body
…
ldr  r3, [sp, #S4]
blx  r3                  ; ⟵ requires r3.bit0 == 1 for Thumb mode
```

There are 48 such `ldr r3, =.LXXX` literals (ASL/LSR/ROL/ROR × 8/16-bit
× a dispatch table per addressing mode). Without `.thumb_func` markers
on the targets, GAS emits the bare label address — bit 0 is 0. `blx r3`
then attempts an ARM-mode switch, the M33 raises a UsageFault, and the
default fault handler enters an infinite loop. The user-visible symptom
was *hangs immediately on the first ASL/LSR/ROL/ROR/etc.*, with the
on-device trace overlay frozen at `op00 n0`.

**Fix**: append `+1` to every internal function-pointer literal:

```asm
ldr  r3, =.Lasl_rmw8+1
```

48 sites patched in a single regex pass.

### Diagnostic infrastructure (compile-time, off by default)

A four-field trace on the `Cpu` struct — `lastOpcode`, `lastPb`,
`lastPc`, `opcodeCount` — lets the on-device picker overlay surface
the dispatcher's last fetched opcode and PC. Updated from both the ASM
loop (after the opcode fetch, before `bx r0`) and from `cpu_runOpcode`
(C path, after fetch). Build with `-DTHUMBYSNES_ASM_TRACE=1` to enable;
adds 6 instructions per ASM dispatch, 3 stores per C dispatch.

The trace turned bisection from "rebuild the firmware 8 times to
binary-search 256 handlers" into "see what opcode the screen says is
hung". Bug 2 was found in two flashes: full-fallback build worked,
ASM-only build hung at `n0` → dispatcher prologue / first opcode →
inspection → missing Thumb bit.

### Test ROM

`tools/gen_test_rom.py` generates a 32 KB LoROM cart that exercises 87
opcode patterns, writing `'P'`/`'F'` plus the failed-test number into
WRAM[0..3] for the picker to read out at the bottom of the screen.
Coverage:

- Loads/stores: `LDA/LDX/LDY` and `STA/STX/STY` in immediate, dp, abs,
  abs,X, abs,Y, dp,X, dp,Y, indirect, indirect-indexed, stack-relative,
  long, long-indexed forms.
- ALU: `ORA/AND/EOR/ADC/SBC/CMP/CPX/CPY` 8- and 16-bit.
- Shifts/rotates: `ASL/LSR/ROL/ROR` on accumulator and memory (dp, abs,
  abs,X) in both 8- and 16-bit modes.
- Increments: `INC/DEC` accumulator, dp, abs, abs,X, plus `INX/DEX/INY/DEY`.
- Branches: every conditional + `BRA` + `JMP abs` + `JMP (ind)` +
  `JMP (abs,X)` + `JMP long` + `JSR/RTS` + `JSL/RTL`.
- Stack: `PHA/PLA/PHX/PLX/PHY/PLY/PHP/PLP/PEA/PEI/PER`.
- Mode: `REP/SEP` flag toggles, `XCE` round-trip, `CLC/SEC/CLD/SED/SEI/CLI/CLV`.
- Misc: `BIT` (dp/abs/imm), `STZ` (dp/abs/abs,X), `TRB/TSB`, `MVN/MVP`
  block move, `WDM`, `XBA`, `TCD/TDC/TSX/TXS/TXY/TYX/TAX/TAY/TXA/TYA`.

Run on host:

```bash
python3 tools/gen_test_rom.py roms/test_cpu.sfc
./build/snesbench roms/test_cpu.sfc 100
# expects: *** TEST ROM PASSED (all tests OK) ***
```

Run on device: drop `roms/test_cpu.sfc` on the USB drive, pick it from
the menu. A green `PASS 87` at the bottom means all opcodes work; a red
`FAIL#N eXX aYY` shows which test failed and the expected vs actual
byte.

The bisection script `/tmp/toggle_asm.py` (not committed) flips ranges
of opcodes between ASM and C-fallback for debugging — useful next time
a regression appears.

## Audio

`dsp_getSamples` in dual-core mode tracks a moving read cursor
(`lastReadOffset`) over the SPC sample ring. Each call resamples
exactly the samples produced since the last call — so emulator
framerates below 60 fps no longer compress one frame's worth of audio
into too few output samples. Audio plays at correct realtime pitch at
any framerate. The single-core code path is unaffected (delta is
always one frame's worth, matching the original 534/641-sample
behaviour).

## License

ThumbySNES glue: see `LICENSE` (matches snes9x's non-commercial terms
for redistribution compatibility).

Vendored `LakeSnes` is MIT-licensed — see `vendor/lakesnes/LICENSE`.
