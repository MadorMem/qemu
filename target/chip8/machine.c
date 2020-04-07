#include "qemu/osdep.h"
#include "cpu.h"
#include "migration/cpu.h"

static VMStateField vmstate_env_fields[] = {
    VMSTATE_UINTTL_ARRAY(V, CPUChip8State, 16),
    VMSTATE_UINTTL(pc, CPUChip8State),
    VMSTATE_END_OF_LIST()
};

static const VMStateDescription vmstate_env = {
    .name = "env",
    .version_id = 3,
    .minimum_version_id = 3,
    .fields = vmstate_env_fields,
};

static VMStateField vmstate_cpu_fields[] = {
    VMSTATE_CPU(),
    VMSTATE_STRUCT(env, Chip8CPU, 1, vmstate_env, CPUChip8State),
    VMSTATE_END_OF_LIST()
};

const VMStateDescription vmstate_alpha_cpu = {
    .name = "cpu",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = vmstate_cpu_fields,
};
