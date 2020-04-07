/*
 *  CHIP8 emulation cpu helpers for qemu.
 *
 *  Copyright (c) 2007 Jocelyn Mayer
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see <http://www.gnu.org/licenses/>.
 */

#include "qemu/osdep.h"

#include "cpu.h"
#include "exec/exec-all.h"
#include "fpu/softfloat-types.h"
#include "exec/helper-proto.h"
#include "qemu/qemu-print.h"


#define CONVERT_BIT(X, SRC, DST) \
    (SRC > DST ? (X) / (SRC / DST) & (DST) : ((X) & SRC) * (DST / SRC))

static uint64_t *cpu_chip8_addr_gr(CPUChip8State *env, unsigned reg)
{
    return &env->V[reg];
}

uint64_t cpu_chip8_load_gr(CPUChip8State *env, unsigned reg)
{
    return *cpu_alpha_addr_gr(env, reg);
}

void cpu_chip8_store_gr(CPUChip8State *env, unsigned reg, uint64_t val)
{
    *cpu_chip8_addr_gr(env, reg) = val;
}

void chip8_cpu_do_interrupt(CPUState *cs)
{
    Chip8CPU *cpu = CHIP8_CPU(cs);
    CPUChip8State *env = &cpu->env;
    int i = cs->exception_index;

    if (qemu_loglevel_mask(CPU_LOG_INT)) {
        static int count;
        const char *name = "<unknown>";

        switch (i) {
        case EXCP_RESET:
            name = "reset";
            break;
        case EXCP_MCHK:
            name = "mchk";
            break;
        case EXCP_SMP_INTERRUPT:
            name = "smp_interrupt";
            break;
        case EXCP_CLK_INTERRUPT:
            name = "clk_interrupt";
            break;
        case EXCP_DEV_INTERRUPT:
            name = "dev_interrupt";
            break;
        case EXCP_MMFAULT:
            name = "mmfault";
            break;
        case EXCP_UNALIGN:
            name = "unalign";
            break;
        case EXCP_OPCDEC:
            name = "opcdec";
            break;
        case EXCP_ARITH:
            name = "arith";
            break;
        case EXCP_FEN:
            name = "fen";
            break;
        case EXCP_CALL_PAL:
            name = "call_pal";
            break;
        }
        qemu_log("INT %6d: %s(%#x) cpu=%d pc=%016"
                 PRIx64 " sp=%016" PRIx64 "\n",
                 ++count, name, env->error_code, cs->cpu_index,
                 env->pc, env->ir[IR_SP]);
    }

    cs->exception_index = -1;

#if !defined(CONFIG_USER_ONLY)
    switch (i) {
    case EXCP_RESET:
        i = 0x0000;
        break;
    case EXCP_MCHK:
        i = 0x0080;
        break;
    case EXCP_SMP_INTERRUPT:
        i = 0x0100;
        break;
    case EXCP_CLK_INTERRUPT:
        i = 0x0180;
        break;
    case EXCP_DEV_INTERRUPT:
        i = 0x0200;
        break;
    case EXCP_MMFAULT:
        i = 0x0280;
        break;
    case EXCP_UNALIGN:
        i = 0x0300;
        break;
    case EXCP_OPCDEC:
        i = 0x0380;
        break;
    case EXCP_ARITH:
        i = 0x0400;
        break;
    case EXCP_FEN:
        i = 0x0480;
        break;
    case EXCP_CALL_PAL:
        i = env->error_code;
        /* There are 64 entry points for both privileged and unprivileged,
           with bit 0x80 indicating unprivileged.  Each entry point gets
           64 bytes to do its job.  */
        if (i & 0x80) {
            i = 0x2000 + (i - 0x80) * 64;
        } else {
            i = 0x1000 + i * 64;
        }
        break;
    default:
        cpu_abort(cs, "Unhandled CPU exception");
    }

    /* Remember where the exception happened.  Emulate real hardware in
       that the low bit of the PC indicates PALmode.  */
    env->exc_addr = env->pc | (env->flags & ENV_FLAG_PAL_MODE);

    /* Continue execution at the PALcode entry point.  */
    env->pc = env->palbr + i;

    /* Switch to PALmode.  */
    env->flags |= ENV_FLAG_PAL_MODE;
#endif /* !USER_ONLY */
}

bool chip8_cpu_exec_interrupt(CPUState *cs, int interrupt_request)
{
    Chip8CPU *cpu = CHIP8_CPU(cs);
    CPUChip8State *env = &cpu->env;
    int idx = -1;

    /* We never take interrupts while in PALmode.  */
    if (env->flags & ENV_FLAG_PAL_MODE) {
        return false;
    }

    /* Fall through the switch, collecting the highest priority
       interrupt that isn't masked by the processor status IPL.  */
    /* ??? This hard-codes the OSF/1 interrupt levels.  */
    switch ((env->flags >> ENV_FLAG_PS_SHIFT) & PS_INT_MASK) {
    case 0 ... 3:
        if (interrupt_request & CPU_INTERRUPT_HARD) {
            idx = EXCP_DEV_INTERRUPT;
        }
        /* FALLTHRU */
    case 4:
        if (interrupt_request & CPU_INTERRUPT_TIMER) {
            idx = EXCP_CLK_INTERRUPT;
        }
        /* FALLTHRU */
    case 5:
        if (interrupt_request & CPU_INTERRUPT_SMP) {
            idx = EXCP_SMP_INTERRUPT;
        }
        /* FALLTHRU */
    case 6:
        if (interrupt_request & CPU_INTERRUPT_MCHK) {
            idx = EXCP_MCHK;
        }
    }
    if (idx >= 0) {
        cs->exception_index = idx;
        env->error_code = 0;
        chip8_cpu_do_interrupt(cs);
        return true;
    }
    return false;
}

void chip8_cpu_dump_state(CPUState *cs, FILE *f, int flags)
{
    Chip8CPU *cpu = CHIP8_CPU(cs);
    CPUChip8State *env = &cpu->env;
    int i;

    qemu_fprintf(f, "PC      " TARGET_FMT_lx " PS      %02x\n",
                 env->pc, extract32(env->flags, ENV_FLAG_PS_SHIFT, 8));
    for (i = 0; i < 16; i++) {
        qemu_fprintf(f, "V%d\t" TARGET_FMT_lx "%c",
                     i, cpu_chip8_load_gr(env, i),
                     (i % 3) == 2 ? '\n' : ' ');
    }

    qemu_fprintf(f, "delay_timer  " TARGET_FMT_lx " sound_timer  " TARGET_FMT_lx "\n",
                 env->delay_timer, env->sound_timer);
    qemu_fprintf(f, "\n");
}

/* This should only be called from translate, via gen_excp.
   We expect that ENV->PC has already been updated.  */
void QEMU_NORETURN helper_excp(CPUChip8State *env, int excp, int error)
{
    CPUState *cs = env_cpu(env);

    cs->exception_index = excp;
    env->error_code = error;
    cpu_loop_exit(cs);
}

/* This may be called from any of the helpers to set up EXCEPTION_INDEX.  */
void QEMU_NORETURN dynamic_excp(CPUChip8State *env, uintptr_t retaddr,
                                int excp, int error)
{
    CPUState *cs = env_cpu(env);

    cs->exception_index = excp;
    env->error_code = error;
    if (retaddr) {
        cpu_restore_state(cs, retaddr, true);
        /* Floating-point exceptions (our only users) point to the next PC.  */
        env->pc += 4;
    }
    cpu_loop_exit(cs);
}

void QEMU_NORETURN arith_excp(CPUChip8State *env, uintptr_t retaddr,
                              int exc, uint64_t mask)
{
    env->trap_arg0 = exc;
    env->trap_arg1 = mask;
    dynamic_excp(env, retaddr, EXCP_ARITH, 0);
}
