/*
 * Chip8 virtual CPU header.
 *
 * Copyright (c) 2011-2012 Jia Liu <proljc@gmail.com>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see <http://www.gnu.org/licenses/>.
 */

#ifndef CHIP8_CPU_H
#define CHIP8_CPU_H

#include "exec/cpu-defs.h"
#include "hw/core/cpu.h"

/* cpu_chip8_map_address_* in CPUChip8TLBContext need this decl.  */
struct Chip8CPU;

#define TYPE_CHIP8_CPU "chip8-cpu"

#define CHIP8_CPU_CLASS(klass) \
    OBJECT_CLASS_CHECK(Chip8CPUClass, (klass), TYPE_CHIP8_CPU)
#define CHIP8_CPU(obj) \
    OBJECT_CHECK(Chip8CPU, (obj), TYPE_CHIP8_CPU)
#define CHIP8_CPU_GET_CLASS(obj) \
    OBJECT_GET_CLASS(Chip8CPUClass, (obj), TYPE_CHIP8_CPU)

/**
 * Chip8CPUClass:
 * @parent_realize: The parent class' realize handler.
 * @parent_reset: The parent class' reset handler.
 *
 * A Chip8 CPU model.
 */
typedef struct Chip8CPUClass {
    /*< private >*/
    CPUClass parent_class;
    /*< public >*/

    DeviceRealize parent_realize;
    DeviceReset parent_reset;
} Chip8CPUClass;

#define TARGET_INSN_START_EXTRA_WORDS 1

enum {
    MMU_NOMMU_IDX = 0,
    MMU_SUPERVISOR_IDX = 1,
    MMU_USER_IDX = 2,
};

/* Interrupt */
#define NR_IRQS  32


/* Exceptions indices */
enum {
    EXCP_RESET    = 0x1,
    EXCP_BUSERR   = 0x2,
    EXCP_DPF      = 0x3,
    EXCP_IPF      = 0x4,
    EXCP_TICK     = 0x5,
    EXCP_ALIGN    = 0x6,
    EXCP_ILLEGAL  = 0x7,
    EXCP_INT      = 0x8,
    EXCP_DTLBMISS = 0x9,
    EXCP_ITLBMISS = 0xa,
    EXCP_RANGE    = 0xb,
    EXCP_SYSCALL  = 0xc,
    EXCP_FPE      = 0xd,
    EXCP_TRAP     = 0xe,
    EXCP_NR,
};

/* TLB size */
enum {
    TLB_SIZE = 128,
    TLB_MASK = TLB_SIZE - 1,
};

/* TLB prot */
enum {
    URE = (1 << 6),
    UWE = (1 << 7),
    SRE = (1 << 8),
    SWE = (1 << 9),

    SXE = (1 << 6),
    UXE = (1 << 7),
};

typedef struct CPUChip8State {
    uint32_t V[16]; // 8-bit
    uint32_t I; // 16-bit

    uint32_t delay_timer; // 8-bit
    uint32_t sound_timer; // 8-bit

    uint32_t pc; // 16-bit
    uint32_t sp; // 8-bit

    QEMUTimer *timer;
    uint32_t ttmr;
    int is_counting;
} CPUChip8State;

/**
 * Chip8CPU:
 * @env: #CPUChip8State
 *
 * A Chip8 CPU.
 */
typedef struct Chip8CPU {
    /*< private >*/
    CPUState parent_obj;
    /*< public >*/

    CPUNegativeOffsetState neg;
    CPUChip8State env;
} Chip8CPU;


void cpu_chip8_list(void);
void chip8_cpu_do_interrupt(CPUState *cpu);
bool chip8_cpu_exec_interrupt(CPUState *cpu, int int_req);
void chip8_cpu_dump_state(CPUState *cpu, FILE *f, int flags);
hwaddr chip8_cpu_get_phys_page_debug(CPUState *cpu, vaddr addr);
int chip8_cpu_gdb_read_register(CPUState *cpu, GByteArray *buf, int reg);
int chip8_cpu_gdb_write_register(CPUState *cpu, uint8_t *buf, int reg);
void chip8_translate_init(void);
bool chip8_cpu_tlb_fill(CPUState *cs, vaddr address, int size,
                           MMUAccessType access_type, int mmu_idx,
                           bool probe, uintptr_t retaddr);
int cpu_chip8_signal_handler(int host_signum, void *pinfo, void *puc);
int print_insn_or1k(bfd_vma addr, disassemble_info *info);

#define cpu_list cpu_chip8_list
#define cpu_signal_handler cpu_chip8_signal_handler

#ifndef CONFIG_USER_ONLY
extern const VMStateDescription vmstate_chip8_cpu;

/* hw/chip8_pic.c */
void cpu_chip8_pic_init(Chip8CPU *cpu);

/* hw/chip8_timer.c */
void cpu_chip8_clock_init(Chip8CPU *cpu);
uint32_t cpu_chip8_count_get(Chip8CPU *cpu);
void cpu_chip8_count_set(Chip8CPU *cpu, uint32_t val);
void cpu_chip8_count_update(Chip8CPU *cpu);
void cpu_chip8_timer_update(Chip8CPU *cpu);
void cpu_chip8_count_start(Chip8CPU *cpu);
void cpu_chip8_count_stop(Chip8CPU *cpu);
#endif

#define CHIP8_CPU_TYPE_SUFFIX "-" TYPE_CHIP8_CPU
#define CHIP8_CPU_TYPE_NAME(model) model CHIP8_CPU_TYPE_SUFFIX
#define CPU_RESOLVING_TYPE TYPE_CHIP8_CPU

typedef CPUChip8State CPUArchState;
typedef Chip8CPU ArchCPU;

#include "exec/cpu-all.h"

static inline uint32_t cpu_get_gpr(const CPUChip8State *env, int i)
{
    return env->V[i];
}

static inline void cpu_set_gpr(CPUChip8State *env, int i, uint32_t val)
{
    env->V[i] = val;
}

static inline void cpu_get_tb_cpu_state(CPUChip8State *env,
                                        target_ulong *pc,
                                        target_ulong *cs_base, uint32_t *flags)
{
    *pc = env->pc;
    *cs_base = 0;
    *flags = 0;
}

static inline int cpu_mmu_index(CPUChip8State *env, bool ifetch)
{
    return MMU_NOMMU_IDX;  /* mmu is disabled */
}

#define CPU_INTERRUPT_TIMER   CPU_INTERRUPT_TGT_INT_0

#endif /* CHIP8_CPU_H */
