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
# Vendored LakeSnes (MIT — see vendor/lakesnes/LICENSE.txt + VENDORING.md)
# Pure-C SNES core, ~6,500 LOC, per-scanline rendering native (the
# essential property snes9x2002 was missing). Patches over upstream:
#   - ppu.h: pixelBuffer shrunk from 978 KB → 2 KB (single line) +
#            scanlineCb hook, see vendor/VENDORING.md.
#   - cart.c / snes_other.c: zero-copy XIP path under thumbysnes_cart_xip.
# ----------------------------------------------------------------------
set(LK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/vendor/lakesnes)
set(LK_SRC ${LK_DIR}/snes)

if(EXISTS ${LK_SRC}/snes.h)
    set(LAKESNES_CORE_SRC
        ${LK_SRC}/apu.c
        ${LK_SRC}/cart.c
        ${LK_SRC}/cpu.c
        ${LK_SRC}/dma.c
        ${LK_SRC}/dsp.c
        ${LK_SRC}/input.c
        ${LK_SRC}/ppu.c
        ${LK_SRC}/snes.c
        ${LK_SRC}/snes_other.c
        ${LK_SRC}/spc.c
        ${LK_SRC}/statehandler.c
    )

    add_library(lakesnes STATIC ${LAKESNES_CORE_SRC})
    target_include_directories(lakesnes PUBLIC ${LK_SRC})
    target_compile_options(lakesnes PRIVATE
        -O3
        -fno-omit-frame-pointer
        -fstrict-aliasing
        -ffunction-sections
        -fdata-sections
        -Wno-unused-parameter
        -Wno-unused-but-set-variable
    )
    set(LAKESNES_PRESENT TRUE)
else()
    message(WARNING "ThumbySNES: vendor/lakesnes not present — skipping emulator lib.")
    set(LAKESNES_PRESENT FALSE)
endif()

# ----------------------------------------------------------------------
# ThumbySNES runtime glue
# ----------------------------------------------------------------------
add_library(thumbysnes STATIC src/snes_core.c)
# cpu_fast.c is #included from cpu.c (same TU for inlining)
target_include_directories(thumbysnes PUBLIC src)
if(LAKESNES_PRESENT)
    target_link_libraries(thumbysnes PUBLIC lakesnes m)
    target_compile_definitions(thumbysnes PRIVATE THUMBYSNES_HAVE_CORE=1)
endif()

# CPU fast-path opcode tests
add_executable(cpu_fast_test src/cpu_fast_test.c src/cpu_fast.c)
target_include_directories(cpu_fast_test PRIVATE src ${LK_SRC})

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
if(LAKESNES_PRESENT)
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
