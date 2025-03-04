/*
 * LOONGARCH gdb server stub
 *
 * Copyright (c) 2021 Loongson Technology Corporation Limited
 *
 * SPDX-License-Identifier: LGPL-2.1+
 */

#include "qemu/osdep.h"
#include "qemu-common.h"
#include "cpu.h"
#include "internals.h"
#include "exec/gdbstub.h"
#include "exec/helper-proto.h"

int loongarch_cpu_gdb_read_register(CPUState *cs, GByteArray *mem_buf, int n)
{
    LoongArchCPU *cpu = LOONGARCH_CPU(cs);
    CPULoongArchState *env = &cpu->env;

    if (0 <= n && n < 32) {
        return gdb_get_regl(mem_buf, env->gpr[n]);
    } else if (n == 32) {
        return gdb_get_regl(mem_buf, env->pc);
    } else if (n == 33) {
        return gdb_get_regl(mem_buf, env->badaddr);
    } else if (n == 34) {
        return gdb_get_regl(mem_buf, env->CSR_CRMD);
    } else if (n == 35) {
        return gdb_get_regl(mem_buf, env->CSR_ECFG);
    } else if (n == 36) {
        return gdb_get_regl(mem_buf, env->CSR_ESTAT);
    } else if (n == 37) {
        return gdb_get_regl(mem_buf, env->CSR_ERA);
    } else if (n == 38) {
        return gdb_get_regl(mem_buf, env->CSR_BADV);
    } else if (n == 39) {
        return gdb_get_regl(mem_buf, env->CSR_BADI);
    } else if (n == 40) {
        return gdb_get_regl(mem_buf, env->CSR_EENTRY);
    } else if (n == 41) {
        return gdb_get_regl(mem_buf, env->CSR_TLBRENTRY);
    } else if (n == 42) {
        return gdb_get_regl(mem_buf, env->CSR_TLBRBADV);
    } else if (n == 43) {
        return gdb_get_regl(mem_buf, env->CSR_TLBRERA);
    } else if (n == 44) {
        return gdb_get_regl(mem_buf, env->CSR_DMW[0]);
    } else if (n == 45) {
        return gdb_get_regl(mem_buf, env->CSR_DMW[1]);
    }
    return 0;
}

int loongarch_cpu_gdb_write_register(CPUState *cs, uint8_t *mem_buf, int n)
{
    LoongArchCPU *cpu = LOONGARCH_CPU(cs);
    CPULoongArchState *env = &cpu->env;
    target_ulong tmp = ldtul_p(mem_buf);

    if (0 <= n && n < 32) {
        return env->gpr[n] = tmp, sizeof(target_ulong);
    } else if (n == 32) {
        return env->pc = tmp, sizeof(target_ulong);
    }else if (n == 34) {
        return env->CSR_CRMD, sizeof(target_ulong);
    } else if (n == 35) {
        return gdb_get_regl(mem_buf, env->CSR_ECFG);
    } else if (n == 36) {
        return gdb_get_regl(mem_buf, env->CSR_ESTAT);
    } else if (n == 37) {
        return gdb_get_regl(mem_buf, env->CSR_ERA);
    } else if (n == 38) {
        return gdb_get_regl(mem_buf, env->CSR_BADV);
    } else if (n == 39) {
        return gdb_get_regl(mem_buf, env->CSR_BADI);
    } else if (n == 40) {
        return gdb_get_regl(mem_buf, env->CSR_EENTRY);
    } else if (n == 41) {
        return gdb_get_regl(mem_buf, env->CSR_TLBRENTRY);
    } else if (n == 42) {
        return gdb_get_regl(mem_buf, env->CSR_TLBRBADV);
    } else if (n == 43) {
        return gdb_get_regl(mem_buf, env->CSR_TLBRERA);
    } else if (n == 44) {
        return gdb_get_regl(mem_buf, env->CSR_DMW[0]);
    } else if (n == 45) {
        return gdb_get_regl(mem_buf, env->CSR_DMW[1]);
    }
    return 0;
}

static int loongarch_gdb_get_fpu(CPULoongArchState *env,
                                 GByteArray *mem_buf, int n)
{
    if (0 <= n && n < 32) {
        return gdb_get_reg64(mem_buf, env->fpr[n]);
    } else if (32 <= n && n < 40) {
        return gdb_get_reg8(mem_buf, env->cf[n - 32]);
    } else if (n == 40) {
        return gdb_get_reg32(mem_buf, env->fcsr0);
    }
    return 0;
}

static int loongarch_gdb_set_fpu(CPULoongArchState *env,
                                 uint8_t *mem_buf, int n)
{
    if (0 <= n && n < 32) {
        return env->fpr[n] = ldq_p(mem_buf), 8;
    } else if (32 <= n && n < 40) {
        return env->cf[n - 32] = ldub_p(mem_buf), 1;
    } else if (n == 40) {
        return env->fcsr0 = ldl_p(mem_buf), 4;
    }
    return 0;
}

void loongarch_cpu_register_gdb_regs_for_features(CPUState *cs)
{
#ifdef TARGET_LOONGARCH32
    gdb_register_coprocessor(cs, loongarch_gdb_get_fpu, loongarch_gdb_set_fpu,
                             41, "loongarch-fpu32.xml", 0);
#else
    gdb_register_coprocessor(cs, loongarch_gdb_get_fpu, loongarch_gdb_set_fpu,
                             41, "loongarch-fpu64.xml", 0);
#endif
}

int loongarch_read_qxfer(CPUState *cs, const char *annex, uint8_t *read_buf,
                         unsigned long offset, unsigned long len)
{
    if (strncmp(annex, "cpucfg", sizeof("cpucfg") - 1) == 0) {
        if (offset % 4 != 0 || len % 4 != 0) {
            return 0;
        }

        size_t i;
        for (i = offset; i < offset + len; i += 4)
            ((uint32_t *)read_buf)[(i - offset) / 4] =
                helper_cpucfg(&(LOONGARCH_CPU(cs)->env), i / 4);
        return 32 * 4;
    }
    return 0;
}

int loongarch_write_qxfer(CPUState *cs, const char *annex,
                          const uint8_t *write_buf, unsigned long offset,
                          unsigned long len)
{
    return 0;
}
