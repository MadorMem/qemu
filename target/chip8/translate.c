/*
 * CHIP8 translation
 *
 * Copyright (c) 2011-2012 Jia Liu <proljc@gmail.com>
 *                         Feng Gao <gf91597@gmail.com>
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

#include "qemu/osdep.h"
#include "cpu.h"
#include "exec/exec-all.h"
#include "disas/disas.h"
#include "tcg/tcg-op.h"
#include "qemu/log.h"
#include "qemu/bitops.h"
#include "qemu/qemu-print.h"
#include "exec/cpu_ldst.h"
#include "exec/translator.h"

#include "exec/helper-proto.h"
#include "exec/helper-gen.h"
#include "exec/gen-icount.h"

#include "trace-tcg.h"
#include "exec/log.h"

/* is_jmp field values */
#define DISAS_EXIT    DISAS_TARGET_0  /* force exit to main loop */
#define DISAS_JUMP    DISAS_TARGET_1  /* exit via jmp_pc/jmp_pc_imm */

typedef struct DisasContext {
    DisasContextBase base;
    uint32_t mem_idx;
    uint32_t tb_flags;
    uint32_t delayed_branch;
    uint32_t cpucfgr;

    /* If not -1, jmp_pc contains this value and so is a direct jump.  */
    target_ulong jmp_pc_imm;
} DisasContext;

static inline bool is_user(DisasContext *dc)
{
    return false;
}

/* Include the auto-generated decoder.  */
#include "decode.inc.c"

static TCGv cpu_regs[32];
static TCGv cpu_i;
static TCGv cpu_pc;
static TCGv cpu_sp;
static TCGv cpu_delay_timer;
static TCGv cpu_sound_timer;

void chip8_translate_init(void)
{
    static const char * const regnames[] = {
        "V0", "V1", "V2", "V3", "V4", "V5", "V6", "V7",
        "V8", "V9", "VA", "VB", "VC", "VD", "VE", "VF"
    };
    int i;

    cpu_pc = tcg_global_mem_new(cpu_env,
                                offsetof(CPUChip8State, pc), "pc");
    cpu_sp = tcg_global_mem_new(cpu_env,
                                offsetof(CPUChip8State, sp), "sp");

    cpu_delay_timer = tcg_global_mem_new(cpu_env,
                                offsetof(CPUChip8State, delay_timer), "delay_timer");
    cpu_sound_timer = tcg_global_mem_new(cpu_env,
                                offsetof(CPUChip8State, sound_timer), "sound_timer");

    cpu_i = tcg_global_mem_new(cpu_env,
                                offsetof(CPUChip8State, I), "I");

    for (i = 0; i < 16; i++) {
        cpu_regs[i] = tcg_global_mem_new(cpu_env,
                                         offsetof(CPUChip8State,
                                                  V[i]),
                                         regnames[i]);
    }
}

static void gen_exception(DisasContext *dc, unsigned int excp)
{
    TCGv_i32 tmp = tcg_const_i32(excp);
    gen_helper_exception(cpu_env, tmp);
    tcg_temp_free_i32(tmp);
}

static void gen_illegal_exception(DisasContext *dc)
{
    tcg_gen_movi_tl(cpu_pc, dc->base.pc_next);
    gen_exception(dc, EXCP_ILLEGAL);
    dc->base.is_jmp = DISAS_NORETURN;
}

static bool trans_CLS(DisasContext *ctx, arg_CLS *a)
{
    return true;
}

static bool trans_RET(DisasContext *ctx, arg_RET *a)
{
    return false;
}

static bool trans_JP(DisasContext *ctx, arg_JP *a)
{
    return false;
}

static bool trans_CALL(DisasContext *ctx, arg_CALL *a)
{
    return false;
}

static bool trans_SE_i(DisasContext *ctx, arg_SE_i *a)
{
    return false;
}

static bool trans_SNE_i(DisasContext *ctx, arg_SNE_i *a)
{
    return false;
}

static bool trans_SE_r(DisasContext *ctx, arg_SE_r *a)
{
    return false;
}

static bool trans_LD_i(DisasContext *ctx, arg_LD_i *a
{
    return false;
}

static bool trans_ADD_i(DisasContext *ctx, arg_ADD_i *a
{
    return false;
}

static bool trans_LD_r(DisasContext *ctx, arg_LD_r *a
{
    return false;
}

static bool trans_OR(DisasContext *ctx, arg_OR *a
{
    return false;
}

static bool trans_AND(DisasContext *ctx, arg_AND *a
{
    return false;
}

static bool trans_XOR(DisasContext *ctx, arg_XOR *a
{
    return false;
}

static bool trans_ADD_r(DisasContext *ctx, arg_ADD_r *a
{
    return false;
}

static bool trans_SUB(DisasContext *ctx, arg_SUB *a
{
    return false;
}

static bool trans_SHR(DisasContext *ctx, arg_SHR *a
{
    return false;
}

static bool trans_SUBN(DisasContext *ctx, arg_SUBN *a
{
    return false;
}

static bool trans_SHL(DisasContext *ctx, arg_SHL *a
{
    return false;
}

static bool trans_SNE_r(DisasContext *ctx, arg_SNE_r *a
{
    return false;
}

static bool trans_LDI(DisasContext *ctx, arg_LDI *a
{
    return false;
}

static bool trans_JP_off(DisasContext *ctx, arg_JP_off *a
{
    return false;
}

static bool trans_RND(DisasContext *ctx, arg_RND *a
{
    return false;
}

static bool trans_DRW(DisasContext *ctx, arg_DRW *a
{
    return false;
}

static bool trans_SKP(DisasContext *ctx, arg_SKP *a
{
    return false;
}

static bool trans_SKNP(DisasContext *ctx, arg_SKNP *a
{
    return false;
}

static bool trans_LDDT(DisasContext *ctx, arg_LDDT *a
{
    return false;
}

static bool trans_LDK(DisasContext *ctx, arg_LDK *a
{
    return false;
}

static bool trans_STDT(DisasContext *ctx, arg_STDT *a
{
    return false;
}

static bool trans_STST(DisasContext *ctx, arg_STST *a
{
    return false;
}

static bool trans_ADDI(DisasContext *ctx, arg_ADDI *a
{
    return false;
}

static bool trans_DIG(DisasContext *ctx, arg_DIG *a
{
    return false;
}

static bool trans_BCD(DisasContext *ctx, arg_BCD *a
{
    return false;
}

static bool trans_STM(DisasContext *ctx, arg_STM *a
{
    return false;
}

static bool trans_LDM(DisasContext *ctx, arg_LDM *a
{
    return false;
}

static void chip8_tr_init_disas_context(DisasContextBase *dcb, CPUState *cs)
{
    DisasContext *dc = container_of(dcb, DisasContext, base);
    CPUChip8State *env = cs->env_ptr;
    int bound;

    dc->mem_idx = cpu_mmu_index(env, false);
    dc->tb_flags = dc->base.tb->flags;
    dc->delayed_branch = (dc->tb_flags & TB_FLAGS_DFLAG) != 0;
    dc->cpucfgr = env->cpucfgr;
    dc->jmp_pc_imm = -1;

    bound = -(dc->base.pc_first | TARGET_PAGE_MASK) / 4;
    dc->base.max_insns = MIN(dc->base.max_insns, bound);
}

static void chip8_tr_tb_start(DisasContextBase *db, CPUState *cs)
{
    DisasContext *dc = container_of(db, DisasContext, base);

    /* Allow the TCG optimizer to see that R0 == 0,
       when it's true, which is the common case.  */
    if (dc->tb_flags & TB_FLAGS_R0_0) {
        dc->R0 = tcg_const_tl(0);
    } else {
        dc->R0 = cpu_regs[0];
    }
}

static void chip8_tr_insn_start(DisasContextBase *dcbase, CPUState *cs)
{
    DisasContext *dc = container_of(dcbase, DisasContext, base);

    tcg_gen_insn_start(dc->base.pc_next, (dc->delayed_branch ? 1 : 0)
                       | (dc->base.num_insns > 1 ? 2 : 0));
}

static bool chip8_tr_breakpoint_check(DisasContextBase *dcbase, CPUState *cs,
                                         const CPUBreakpoint *bp)
{
    DisasContext *dc = container_of(dcbase, DisasContext, base);

    tcg_gen_movi_tl(cpu_pc, dc->base.pc_next);
    gen_exception(dc, EXCP_DEBUG);
    dc->base.is_jmp = DISAS_NORETURN;
    /* The address covered by the breakpoint must be included in
       [tb->pc, tb->pc + tb->size) in order to for it to be
       properly cleared -- thus we increment the PC here so that
       the logic setting tb->size below does the right thing.  */
    dc->base.pc_next += 4;
    return true;
}

static void chip8_tr_translate_insn(DisasContextBase *dcbase, CPUState *cs)
{
    DisasContext *dc = container_of(dcbase, DisasContext, base);
    OpenRISCCPU *cpu = CHIP8_CPU(cs);
    uint32_t insn = translator_ldl(&cpu->env, dc->base.pc_next);

    if (!decode(dc, insn)) {
        gen_illegal_exception(dc);
    }
    dc->base.pc_next += 4;

    /* When exiting the delay slot normally, exit via jmp_pc.
     * For DISAS_NORETURN, we have raised an exception and already exited.
     * For DISAS_EXIT, we found l.rfe in a delay slot.  There's nothing
     * in the manual saying this is illegal, but it surely it should.
     * At least or1ksim overrides pcnext and ignores the branch.
     */
    if (dc->delayed_branch
        && --dc->delayed_branch == 0
        && dc->base.is_jmp == DISAS_NEXT) {
        dc->base.is_jmp = DISAS_JUMP;
    }
}

static void chip8_tr_tb_stop(DisasContextBase *dcbase, CPUState *cs)
{
    DisasContext *dc = container_of(dcbase, DisasContext, base);
    target_ulong jmp_dest;

    /* If we have already exited the TB, nothing following has effect.  */
    if (dc->base.is_jmp == DISAS_NORETURN) {
        return;
    }

    /* Adjust the delayed branch state for the next TB.  */
    if ((dc->tb_flags & TB_FLAGS_DFLAG ? 1 : 0) != (dc->delayed_branch != 0)) {
        tcg_gen_movi_i32(cpu_dflag, dc->delayed_branch != 0);
    }

    /* For DISAS_TOO_MANY, jump to the next insn.  */
    jmp_dest = dc->base.pc_next;
    tcg_gen_movi_tl(cpu_ppc, jmp_dest - 4);

    switch (dc->base.is_jmp) {
    case DISAS_JUMP:
        jmp_dest = dc->jmp_pc_imm;
        if (jmp_dest == -1) {
            /* The jump destination is indirect/computed; use jmp_pc.  */
            tcg_gen_mov_tl(cpu_pc, jmp_pc);
            tcg_gen_discard_tl(jmp_pc);
            if (unlikely(dc->base.singlestep_enabled)) {
                gen_exception(dc, EXCP_DEBUG);
            } else {
                tcg_gen_lookup_and_goto_ptr();
            }
            break;
        }
        /* The jump destination is direct; use jmp_pc_imm.
           However, we will have stored into jmp_pc as well;
           we know now that it wasn't needed.  */
        tcg_gen_discard_tl(jmp_pc);
        /* fallthru */

    case DISAS_TOO_MANY:
        if (unlikely(dc->base.singlestep_enabled)) {
            tcg_gen_movi_tl(cpu_pc, jmp_dest);
            gen_exception(dc, EXCP_DEBUG);
        } else if ((dc->base.pc_first ^ jmp_dest) & TARGET_PAGE_MASK) {
            tcg_gen_movi_tl(cpu_pc, jmp_dest);
            tcg_gen_lookup_and_goto_ptr();
        } else {
            tcg_gen_goto_tb(0);
            tcg_gen_movi_tl(cpu_pc, jmp_dest);
            tcg_gen_exit_tb(dc->base.tb, 0);
        }
        break;

    case DISAS_EXIT:
        if (unlikely(dc->base.singlestep_enabled)) {
            gen_exception(dc, EXCP_DEBUG);
        } else {
            tcg_gen_exit_tb(NULL, 0);
        }
        break;
    default:
        g_assert_not_reached();
    }
}

static void chip8_tr_disas_log(const DisasContextBase *dcbase, CPUState *cs)
{
    DisasContext *s = container_of(dcbase, DisasContext, base);

    qemu_log("IN: %s\n", lookup_symbol(s->base.pc_first));
    log_target_disas(cs, s->base.pc_first, s->base.tb->size);
}

static const TranslatorOps chip8_tr_ops = {
    .init_disas_context = chip8_tr_init_disas_context,
    .tb_start           = chip8_tr_tb_start,
    .insn_start         = chip8_tr_insn_start,
    .breakpoint_check   = chip8_tr_breakpoint_check,
    .translate_insn     = chip8_tr_translate_insn,
    .tb_stop            = chip8_tr_tb_stop,
    .disas_log          = chip8_tr_disas_log,
};

void gen_intermediate_code(CPUState *cs, TranslationBlock *tb, int max_insns)
{
    DisasContext ctx;

    translator_loop(&chip8_tr_ops, &ctx.base, cs, tb, max_insns);
}

void chip8_cpu_dump_state(CPUState *cs, FILE *f, int flags)
{
    Chip8CPU *cpu = CHIP8_CPU(cs);
    CPUChip8State *env = &cpu->env;
    int i;

    qemu_fprintf(f, "PC=%08x\n", env->pc);
    for (i = 0; i < 16; ++i) {
        qemu_fprintf(f, "V%02d=%08x%c", i, cpu_get_gpr(env, i),
                     (i % 4) == 3 ? '\n' : ' ');
    }
}

void restore_state_to_opc(CPUChip8State *env, TranslationBlock *tb,
                          target_ulong *data)
{
    env->pc = data[0];
}
