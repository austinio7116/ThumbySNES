/*
 * ThumbySNES — fast-path 65816 opcode handler.
 *
 * Top-10 opcodes by execution frequency, with 65816 registers cached
 * in ARM local variables. The compiler keeps these in R4-R11 instead
 * of loading/storing from the Cpu struct per access. Saves ~6-10
 * memory operations per opcode for the ~60% of execution these cover.
 *
 * Each opcode body is a flattened version of the C helpers in cpu.c
 * (cpu_adrDp, cpu_lda, cpu_setZN, etc.) — no function calls except
 * cpu_read/cpu_write/cpu_idle which must go through the bus.
 */
#include "cpu_fast.h"
#include "perf.h"

/* Bus access — goes through the normal snes_cpuRead/Write/Idle path.
 * With THUMBYSNES_DIRECT_CPU_CALLS these are direct BL, otherwise
 * indirect via function pointer. */
#if defined(THUMBYSNES_DIRECT_CPU_CALLS) && THUMBYSNES_DIRECT_CPU_CALLS
extern uint8_t snes_cpuRead(void* mem, uint32_t adr);
extern void    snes_cpuWrite(void* mem, uint32_t adr, uint8_t val);
extern void    snes_cpuIdle(void* mem, bool waiting);
#define FREAD(cpu, adr)       snes_cpuRead((cpu)->mem, (adr))
#define FWRITE(cpu, adr, val) snes_cpuWrite((cpu)->mem, (adr), (val))
#define FIDLE(cpu)            snes_cpuIdle((cpu)->mem, false)
#else
#define FREAD(cpu, adr)       (cpu)->read((cpu)->mem, (adr))
#define FWRITE(cpu, adr, val) (cpu)->write((cpu)->mem, (adr), (val))
#define FIDLE(cpu)            (cpu)->idle((cpu)->mem, false)
#endif

/* Inline interrupt check — same as cpu_checkInt in cpu.c. */
#define FCHECK_INT(cpu) \
    (cpu)->intWanted = (cpu)->nmiWanted || ((cpu)->irqWanted && !(cpu)->i)

LAKESNES_HOT int cpu_doOpcodeFast(Cpu* cpu, uint8_t opcode) {
    /* Cache hot 65816 state in locals. The compiler maps these to ARM
     * registers — zero memory traffic for register access within the
     * opcode body. Stored back at the end. */
    uint16_t a  = cpu->a;
    uint16_t x  = cpu->x;
    uint16_t y  = cpu->y;
    uint16_t sp = cpu->sp;
    uint16_t pc = cpu->pc;
    uint16_t dp = cpu->dp;
    uint8_t  k  = cpu->k;
    uint8_t  db = cpu->db;
    bool mf = cpu->mf;
    bool e  = cpu->e;
    bool z  = cpu->z;
    bool n  = cpu->n;
    bool c  = cpu->c;

    switch (opcode) {

    /* ---- LDA dp (0xA5) ---- */
    case 0xa5: {
        uint8_t adr = FREAD(cpu, ((uint32_t)k << 16) | pc++);
        if (dp & 0xff) FIDLE(cpu);
        uint16_t ea_lo = (dp + adr) & 0xffff;
        if (mf) {
            FCHECK_INT(cpu);
            uint8_t val = FREAD(cpu, ea_lo);
            a = (a & 0xff00) | val;
            z = (val == 0);
            n = (val & 0x80);
        } else {
            uint8_t lo = FREAD(cpu, ea_lo);
            FCHECK_INT(cpu);
            uint8_t hi = FREAD(cpu, (dp + adr + 1) & 0xffff);
            a = lo | ((uint16_t)hi << 8);
            z = (a == 0);
            n = (a & 0x8000);
        }
        break;
    }

    /* ---- STA dp (0x85) ---- */
    case 0x85: {
        uint8_t adr = FREAD(cpu, ((uint32_t)k << 16) | pc++);
        if (dp & 0xff) FIDLE(cpu);
        uint16_t ea_lo = (dp + adr) & 0xffff;
        if (mf) {
            FCHECK_INT(cpu);
            FWRITE(cpu, ea_lo, (uint8_t)a);
        } else {
            FWRITE(cpu, ea_lo, (uint8_t)(a & 0xff));
            FCHECK_INT(cpu);
            FWRITE(cpu, (dp + adr + 1) & 0xffff, (uint8_t)(a >> 8));
        }
        break;
    }

    /* ---- BNE rel (0xD0) ---- */
    case 0xd0: {
        bool take = !z;
        if (!take) FCHECK_INT(cpu);
        uint8_t off = FREAD(cpu, ((uint32_t)k << 16) | pc++);
        if (take) {
            FCHECK_INT(cpu);
            FIDLE(cpu);
            pc += (int8_t)off;
        }
        break;
    }

    /* ---- BEQ rel (0xF0) ---- */
    case 0xf0: {
        bool take = z;
        if (!take) FCHECK_INT(cpu);
        uint8_t off = FREAD(cpu, ((uint32_t)k << 16) | pc++);
        if (take) {
            FCHECK_INT(cpu);
            FIDLE(cpu);
            pc += (int8_t)off;
        }
        break;
    }

    /* ---- LDA imm (0xA9) ---- */
    case 0xa9: {
        if (mf) {
            uint32_t ea = ((uint32_t)k << 16) | pc++;
            FCHECK_INT(cpu);
            uint8_t val = FREAD(cpu, ea);
            a = (a & 0xff00) | val;
            z = (val == 0);
            n = (val & 0x80);
        } else {
            uint32_t ea_lo = ((uint32_t)k << 16) | pc++;
            uint32_t ea_hi = ((uint32_t)k << 16) | pc++;
            uint8_t lo = FREAD(cpu, ea_lo);
            FCHECK_INT(cpu);
            uint8_t hi = FREAD(cpu, ea_hi);
            a = lo | ((uint16_t)hi << 8);
            z = (a == 0);
            n = (a & 0x8000);
        }
        break;
    }

    /* ---- CMP imm (0xC9) ---- */
    case 0xc9: {
        if (mf) {
            uint32_t ea = ((uint32_t)k << 16) | pc++;
            FCHECK_INT(cpu);
            uint8_t val = FREAD(cpu, ea) ^ 0xff;
            int result = (a & 0xff) + val + 1;
            c = (result > 0xff);
            z = ((result & 0xff) == 0);
            n = (result & 0x80);
        } else {
            uint32_t ea_lo = ((uint32_t)k << 16) | pc++;
            uint32_t ea_hi = ((uint32_t)k << 16) | pc++;
            uint8_t lo = FREAD(cpu, ea_lo);
            FCHECK_INT(cpu);
            uint8_t hi = FREAD(cpu, ea_hi);
            uint16_t val = (lo | ((uint16_t)hi << 8)) ^ 0xffff;
            int result = a + val + 1;
            c = (result > 0xffff);
            z = ((result & 0xffff) == 0);
            n = (result & 0x8000);
        }
        break;
    }

    /* ---- LDA abs (0xAD) ---- */
    case 0xad: {
        uint8_t lo_adr = FREAD(cpu, ((uint32_t)k << 16) | pc++);
        uint8_t hi_adr = FREAD(cpu, ((uint32_t)k << 16) | pc++);
        uint32_t ea = ((uint32_t)db << 16) + (lo_adr | ((uint16_t)hi_adr << 8));
        if (mf) {
            FCHECK_INT(cpu);
            uint8_t val = FREAD(cpu, ea);
            a = (a & 0xff00) | val;
            z = (val == 0);
            n = (val & 0x80);
        } else {
            uint8_t lo = FREAD(cpu, ea);
            FCHECK_INT(cpu);
            uint8_t hi = FREAD(cpu, (ea + 1) & 0xffffff);
            a = lo | ((uint16_t)hi << 8);
            z = (a == 0);
            n = (a & 0x8000);
        }
        break;
    }

    /* ---- STA abs (0x8D) ---- */
    case 0x8d: {
        uint8_t lo_adr = FREAD(cpu, ((uint32_t)k << 16) | pc++);
        uint8_t hi_adr = FREAD(cpu, ((uint32_t)k << 16) | pc++);
        uint32_t ea = ((uint32_t)db << 16) + (lo_adr | ((uint16_t)hi_adr << 8));
        if (mf) {
            FCHECK_INT(cpu);
            FWRITE(cpu, ea, (uint8_t)a);
        } else {
            FWRITE(cpu, ea, (uint8_t)(a & 0xff));
            FCHECK_INT(cpu);
            FWRITE(cpu, (ea + 1) & 0xffffff, (uint8_t)(a >> 8));
        }
        break;
    }

    /* ---- JSR abs (0x20) ---- */
    case 0x20: {
        uint8_t lo_adr = FREAD(cpu, ((uint32_t)k << 16) | pc++);
        uint8_t hi_adr = FREAD(cpu, ((uint32_t)k << 16) | pc++);
        FIDLE(cpu);
        /* Push return address (pc - 1). High byte first. */
        uint16_t ret = pc - 1;
        FWRITE(cpu, sp, (uint8_t)(ret >> 8));
        sp--;
        if (e) sp = (sp & 0xff) | 0x100;
        FCHECK_INT(cpu);
        FWRITE(cpu, sp, (uint8_t)(ret & 0xff));
        sp--;
        if (e) sp = (sp & 0xff) | 0x100;
        pc = lo_adr | ((uint16_t)hi_adr << 8);
        break;
    }

    /* ---- RTS (0x60) ---- */
    case 0x60: {
        FIDLE(cpu);
        FIDLE(cpu);
        /* Pull return address. Low byte first. */
        sp++;
        if (e) sp = (sp & 0xff) | 0x100;
        uint8_t lo = FREAD(cpu, sp);
        sp++;
        if (e) sp = (sp & 0xff) | 0x100;
        uint8_t hi = FREAD(cpu, sp);
        pc = (lo | ((uint16_t)hi << 8)) + 1;
        FCHECK_INT(cpu);
        FIDLE(cpu);
        break;
    }

    default:
        return 0; /* not handled — caller falls through to cpu_doOpcode */
    }

    /* Store registers back to struct. Only modified fields need writing
     * but the compiler is smart enough to elide stores for unchanged
     * locals (constant propagation through the switch cases). */
    cpu->a  = a;
    cpu->x  = x;
    cpu->y  = y;
    cpu->sp = sp;
    cpu->pc = pc;
    cpu->z  = z;
    cpu->n  = n;
    cpu->c  = c;
    return 1;
}
