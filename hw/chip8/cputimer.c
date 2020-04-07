/*
 * QEMU CHIP8 timer support
 *
 * Copyright (c) 2011-2012 Jia Liu <proljc@gmail.com>
 *                         Zhizhou Zhang <etouzh@gmail.com>
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
#include "migration/vmstate.h"
#include "qemu/timer.h"

#define TIMER_PERIOD (1000000000 / 60) /* 60Hz */

/* Tick Timer global state to allow all cores to be in sync */
typedef struct Chip8TimerState {
    uint32_t ttcr;
    uint64_t last_clk;
} Chip8TimerState;

static Chip8TimerState *chip8_timer;

void cpu_chip8_count_set(Chip8CPU *cpu, uint32_t val)
{
    chip8_timer->ttcr = val;
}

uint32_t cpu_chip8_count_get(Chip8CPU *cpu)
{
    return chip8_timer->ttcr;
}

/* Add elapsed ticks to ttcr */
void cpu_chip8_count_update(Chip8CPU *cpu)
{
    uint64_t now;

    if (!cpu->env.is_counting) {
        return;
    }
    now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    chip8_timer->ttcr += (uint32_t)((now - chip8_timer->last_clk)
                                    / TIMER_PERIOD);
    chip8_timer->last_clk = now;
}

/* Update the next timeout time as difference between ttmr and ttcr */
void cpu_chip8_timer_update(Chip8CPU *cpu)
{
    uint64_t now;

    if (!cpu->env.is_counting) {
        return;
    }

    cpu_chip8_count_update(cpu);
    now = chip8_timer->last_clk;

    timer_mod(cpu->env.timer, now + TIMER_PERIOD);
}

void cpu_chip8_count_start(Chip8CPU *cpu)
{
    cpu->env.is_counting = 1;
    cpu_chip8_count_update(cpu);
}

void cpu_chip8_count_stop(Chip8CPU *cpu)
{
    timer_del(cpu->env.timer);
    cpu_chip8_count_update(cpu);
    cpu->env.is_counting = 0;
}

static void chip8_timer_cb(void *opaque)
{
    Chip8CPU *cpu = opaque;

    cpu_chip8_count_stop(cpu);
    cpu_chip8_timer_update(cpu);
    qemu_cpu_kick(CPU(cpu));
}

static const VMStateDescription vmstate_chip8_timer = {
    .name = "chip8_timer",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(ttcr, Chip8TimerState),
        VMSTATE_UINT64(last_clk, Chip8TimerState),
        VMSTATE_END_OF_LIST()
    }
};

void cpu_chip8_clock_init(Chip8CPU *cpu)
{
    cpu->env.timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, &chip8_timer_cb, cpu);
    cpu->env.ttmr = 0x00000000;

    if (chip8_timer == NULL) {
        chip8_timer = g_new0(Chip8TimerState, 1);
        vmstate_register(NULL, 0, &vmstate_chip8_timer, chip8_timer);
    }
}
