/*
 * QEMU Chip8 CPU
 *
 * Copyright (c) 2012 Jia Liu <proljc@gmail.com>
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
#include "qapi/error.h"
#include "qemu/qemu-print.h"
#include "cpu.h"

static void chip8_cpu_set_pc(CPUState *cs, vaddr value)
{
    Chip8CPU *cpu = CHIP8_CPU(cs);

    cpu->env.pc = value;
}

static bool chip8_cpu_has_work(CPUState *cs)
{
    return cs->interrupt_request & (CPU_INTERRUPT_HARD |
                                    CPU_INTERRUPT_TIMER);
}

static void chip8_disas_set_info(CPUState *cpu, disassemble_info *info)
{
    info->print_insn = print_insn_or1k;
}

static void chip8_cpu_reset(DeviceState *dev)
{
    CPUState *s = CPU(dev);
    Chip8CPU *cpu = CHIP8_CPU(s);
    Chip8CPUClass *occ = CHIP8_CPU_GET_CLASS(cpu);

    occ->parent_reset(dev);

    memset(&cpu->env, 0, offsetof(CPUOpenRISCState, end_reset_fields));

    cpu->env.pc = 0x200;

#ifndef CONFIG_USER_ONLY
    cpu->env.picmr = 0x00000000;
    cpu->env.picsr = 0x00000000;

    cpu->env.ttmr = 0x00000000;
#endif
}

static void chip8_cpu_realizefn(DeviceState *dev, Error **errp)
{
    CPUState *cs = CPU(dev);
    Chip8CPUClass *occ = CHIP8_CPU_GET_CLASS(dev);
    Error *local_err = NULL;

    cpu_exec_realizefn(cs, &local_err);
    if (local_err != NULL) {
        error_propagate(errp, local_err);
        return;
    }

    qemu_init_vcpu(cs);
    cpu_reset(cs);

    occ->parent_realize(dev, errp);
}

static void chip8_cpu_initfn(Object *obj)
{
    Chip8CPU *cpu = CHIP8_CPU(obj);

    cpu_set_cpustate_pointers(cpu);
}

/* CPU models */

static ObjectClass *chip8_cpu_class_by_name(const char *cpu_model)
{
    ObjectClass *oc;
    char *typename;

    typename = g_strdup_printf(CHIP8_CPU_TYPE_NAME("%s"), cpu_model);
    oc = object_class_by_name(typename);
    g_free(typename);
    if (oc != NULL && (!object_class_dynamic_cast(oc, TYPE_CHIP8_CPU) ||
                       object_class_is_abstract(oc))) {
        return NULL;
    }
    return oc;
}

static void chip8_any_initfn(Object *obj)
{
    Chip8CPU *cpu = CHIP8_CPU(obj);

    cpu->env.vr = 0x13000040;   /* Obsolete VER + UVRP for new SPRs */
    cpu->env.vr2 = 0;           /* No version specific id */
    cpu->env.avr = 0x01030000;  /* Architecture v1.3 */

    cpu->env.upr = UPR_UP | UPR_DMP | UPR_IMP | UPR_PICP | UPR_TTP | UPR_PMP;
    cpu->env.cpucfgr = CPUCFGR_NSGF | CPUCFGR_OB32S | CPUCFGR_OF32S |
                       CPUCFGR_AVRP | CPUCFGR_EVBARP | CPUCFGR_OF64A32S;
}

static void chip8_cpu_class_init(ObjectClass *oc, void *data)
{
    Chip8CPUClass *occ = CHIP8_CPU_CLASS(oc);
    CPUClass *cc = CPU_CLASS(occ);
    DeviceClass *dc = DEVICE_CLASS(oc);

    device_class_set_parent_realize(dc, chip8_cpu_realizefn,
                                    &occ->parent_realize);
    device_class_set_parent_reset(dc, chip8_cpu_reset, &occ->parent_reset);

    cc->class_by_name = chip8_cpu_class_by_name;
    cc->has_work = chip8_cpu_has_work;
    cc->do_interrupt = chip8_cpu_do_interrupt;
    cc->cpu_exec_interrupt = chip8_cpu_exec_interrupt;
    cc->dump_state = chip8_cpu_dump_state;
    cc->set_pc = chip8_cpu_set_pc;
    cc->gdb_read_register = chip8_cpu_gdb_read_register;
    cc->gdb_write_register = chip8_cpu_gdb_write_register;
    cc->tlb_fill = chip8_cpu_tlb_fill;
#ifndef CONFIG_USER_ONLY
    cc->get_phys_page_debug = chip8_cpu_get_phys_page_debug;
    dc->vmsd = &vmstate_chip8_cpu;
#endif
    cc->gdb_num_core_regs = 32 + 3;
    cc->tcg_initialize = chip8_translate_init;
    cc->disas_set_info = chip8_disas_set_info;
}

/* Sort alphabetically by type name, except for "any". */
static gint chip8_cpu_list_compare(gconstpointer a, gconstpointer b)
{
    ObjectClass *class_a = (ObjectClass *)a;
    ObjectClass *class_b = (ObjectClass *)b;
    const char *name_a, *name_b;

    name_a = object_class_get_name(class_a);
    name_b = object_class_get_name(class_b);
    if (strcmp(name_a, "any-" TYPE_CHIP8_CPU) == 0) {
        return 1;
    } else if (strcmp(name_b, "any-" TYPE_CHIP8_CPU) == 0) {
        return -1;
    } else {
        return strcmp(name_a, name_b);
    }
}

static void chip8_cpu_list_entry(gpointer data, gpointer user_data)
{
    ObjectClass *oc = data;
    const char *typename;
    char *name;

    typename = object_class_get_name(oc);
    name = g_strndup(typename,
                     strlen(typename) - strlen("-" TYPE_CHIP8_CPU));
    qemu_printf("  %s\n", name);
    g_free(name);
}

void cpu_chip8_list(void)
{
    GSList *list;

    list = object_class_get_list(TYPE_CHIP8_CPU, false);
    list = g_slist_sort(list, chip8_cpu_list_compare);
    qemu_printf("Available CPUs:\n");
    g_slist_foreach(list, chip8_cpu_list_entry, NULL);
    g_slist_free(list);
}

#define DEFINE_CHIP8_CPU_TYPE(cpu_model, initfn) \
    {                                               \
        .parent = TYPE_CHIP8_CPU,                \
        .instance_init = initfn,                    \
        .name = CHIP8_CPU_TYPE_NAME(cpu_model),  \
    }

static const TypeInfo chip8_cpus_type_infos[] = {
    { /* base class should be registered first */
        .name = TYPE_CHIP8_CPU,
        .parent = TYPE_CPU,
        .instance_size = sizeof(Chip8CPU),
        .instance_init = chip8_cpu_initfn,
        .abstract = true,
        .class_size = sizeof(Chip8CPUClass),
        .class_init = chip8_cpu_class_init,
    },
    DEFINE_CHIP8_CPU_TYPE("any", chip8_any_initfn),
};

DEFINE_TYPES(chip8_cpus_type_infos)
