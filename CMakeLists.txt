cmake_minimum_required(VERSION 3.16)
project(ThumbySNES C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

add_compile_options(
    -Wall
    -Wno-unused-parameter
    -Wno-unused-function
    -Wno-unused-variable
    -Wno-unused-but-set-variable
    -Wno-unused-result
    -Wno-sign-compare
    -Wno-array-bounds
    -Wno-format
    -Wno-pointer-sign
    -Wno-implicit-function-declaration
    -Wno-parentheses
    -Wno-write-strings
    -Wno-comment
    -fno-strict-aliasing
    -fsigned-char
)

# ----------------------------------------------------------------------
# Vendored snes9x2002 (snes9x non-commercial license — see vendor/VENDORING.md)
# Source list and defines mirror vendor/snes9x2002/Makefile.common non-ARM_ASM path.
# ----------------------------------------------------------------------
set(S9X_DIR ${CMAKE_CURRENT_SOURCE_DIR}/vendor/snes9x2002)
set(S9X_SRC ${S9X_DIR}/src)
set(S9X_RET ${S9X_DIR}/libretro)
set(S9X_COMMON ${S9X_RET}/libretro-common)

if(EXISTS ${S9X_SRC}/cpu.c)
    set(SNES9X2002_CORE_SRC
        ${S9X_SRC}/apu.c
        ${S9X_SRC}/apuaux.c
        ${S9X_SRC}/c4.c
        ${S9X_SRC}/c4emu.c
        # cheats.c / cheats2.c removed — 302 KB static `Cheat` array.
        # Stubs for S9xInitCheatData / S9xApplyCheats live in snes_core.c
        # (referenced from memmap.c, gfx.c, gfx16.c).
        ${S9X_SRC}/clip.c
        ${S9X_SRC}/data.c
        ${S9X_SRC}/dsp1.c
        ${S9X_SRC}/fxemu.c
        ${S9X_SRC}/fxinst.c
        ${S9X_SRC}/globals.c
        ${S9X_SRC}/dma.c
        ${S9X_SRC}/memmap.c
        ${S9X_SRC}/cpu.c
        ${S9X_SRC}/cpuexec.c
        ${S9X_SRC}/cpuops.c
        # sa1.c / sa1cpu.c removed — Super FX / SA-1 chips out of scope.
        # Stub for S9xSA1MainLoop lives in snes_core.c; SA-1 struct stays
        # in globals.c (memmap.c still reads SA1_Map during mapper setup)
        # but SA1.Executing stays FALSE so the loop is never entered.
        # Saves ~80 KB text (flash) — SRAM impact is minor.
        ${S9X_SRC}/sdd1.c
        ${S9X_SRC}/sdd1emu.c
        ${S9X_SRC}/snapshot.c
        ${S9X_SRC}/soundux.c
        ${S9X_SRC}/spc700.c
        ${S9X_SRC}/srtc.c
        # non-ARM_ASM raster path
        ${S9X_SRC}/ppu_.c
        ${S9X_SRC}/gfx.c
        ${S9X_SRC}/tile.c
        # libretro-common memory stream (used by the core's STREAM macros)
        ${S9X_COMMON}/streams/memory_stream.c
    )

    add_library(snes9x2002 STATIC ${SNES9X2002_CORE_SRC})
    target_include_directories(snes9x2002 PUBLIC
        ${S9X_SRC}
        ${S9X_RET}
        ${S9X_COMMON}/include
    )
    target_compile_definitions(snes9x2002 PUBLIC
        __LIBRETRO__
        HAVE_STDINT_H
        HAVE_INTTYPES_H
        HAVE_STRINGS_H
        USE_SA1
        __OLD_RASTER_FX__
    )
    target_compile_options(snes9x2002 PRIVATE
        -O3
        -fomit-frame-pointer
        -ffast-math
        -fstrict-aliasing
        # Per-function / per-data sections so the linker can dead-strip
        # unused globals (e.g. the 302 KB `Cheat` struct in globals.c
        # once cheats.c/cheats2.c are out of the build).
        -ffunction-sections
        -fdata-sections
    )
    set(SNES9X2002_PRESENT TRUE)
else()
    message(WARNING "ThumbySNES: vendor/snes9x2002 not present — skipping emulator lib. "
                    "See PLAN.md §10 for the Phase 0 vendor steps.")
    set(SNES9X2002_PRESENT FALSE)
endif()

# ----------------------------------------------------------------------
# ThumbySNES runtime glue
# ----------------------------------------------------------------------
add_library(thumbysnes STATIC src/snes_core.c)
target_include_directories(thumbysnes PUBLIC src)
if(SNES9X2002_PRESENT)
    target_link_libraries(thumbysnes PUBLIC snes9x2002 m)
    target_compile_definitions(thumbysnes PRIVATE THUMBYSNES_HAVE_CORE=1)
endif()

# ----------------------------------------------------------------------
# Phase 0 bench harness (headless N-frame run + timing)
# ----------------------------------------------------------------------
add_executable(snesbench src/snes_bench_main.c)
target_link_libraries(snesbench PRIVATE thumbysnes)
target_link_options(snesbench PRIVATE -Wl,--gc-sections)

# ----------------------------------------------------------------------
# Phase 2 memory audit — wraps malloc/calloc/realloc/free at link time
# to track live heap + peak watermark across load + N frames per ROM.
# Only the audit target gets the wrap; snesbench/sneshost stay clean.
# ----------------------------------------------------------------------
if(SNES9X2002_PRESENT)
    add_executable(memaudit src/snes_memaudit_main.c)
    target_link_libraries(memaudit PRIVATE thumbysnes)
    target_link_options(memaudit PRIVATE
        -Wl,--wrap=malloc
        -Wl,--wrap=calloc
        -Wl,--wrap=realloc
        -Wl,--wrap=free
        -Wl,--gc-sections
    )
endif()

# ----------------------------------------------------------------------
# Phase 1 SDL2 host runner (built only if SDL2 is available)
# ----------------------------------------------------------------------
find_package(SDL2 QUIET)
if(SDL2_FOUND)
    add_executable(sneshost src/snes_host_main.c)
    target_link_libraries(sneshost PRIVATE thumbysnes SDL2::SDL2)
    target_link_options(sneshost PRIVATE -Wl,--gc-sections)
    message(STATUS "ThumbySNES: SDL2 found, building sneshost")
else()
    find_path(SDL2_INCLUDE_DIR SDL.h PATH_SUFFIXES SDL2)
    find_library(SDL2_LIBRARY NAMES SDL2)
    if(SDL2_INCLUDE_DIR AND SDL2_LIBRARY)
        add_executable(sneshost src/snes_host_main.c)
        target_include_directories(sneshost PRIVATE ${SDL2_INCLUDE_DIR})
        target_link_libraries(sneshost PRIVATE thumbysnes ${SDL2_LIBRARY})
        target_link_options(sneshost PRIVATE -Wl,--gc-sections)
        message(STATUS "ThumbySNES: SDL2 (manual) found, building sneshost")
    else()
        message(STATUS "ThumbySNES: SDL2 not found, skipping sneshost")
    endif()
endif()
