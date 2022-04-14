/*
 * LoongArch 32 reduced ISA Soc emulation
 *
 * Copyright (C) 2022 Fanrui Meng
 * Written by Fanrui Meng <mengfanrui@loongson.cn>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/hw.h"
#include "hw/i386/pc.h"
#include "hw/char/serial.h"
#include "hw/block/fdc.h"
#include "net/net.h"
#include "hw/boards.h"
#include "hw/i2c/i2c.h"
#include "block/block.h"
#include "hw/block/flash.h"
#include "hw/loongarch/loongarch.h"
#include "hw/pci/pci.h"
#include "hw/pci/pci_bridge.h"
#include "sysemu/reset.h"
#include "sysemu/sysemu.h"
#include "sysemu/arch_init.h"
#include "sysemu/numa.h"
#include "qemu/log.h"
#include "hw/ide.h"
#include "hw/loader.h"
#include "elf.h"
#include "hw/timer/i8254.h"
#include "sysemu/blockdev.h"
#include "exec/address-spaces.h"
#include "hw/sysbus.h"             /* SysBusDevice */
#include "qemu/host-utils.h"
#include "sysemu/qtest.h"
#include "qemu/error-report.h"
#include "hw/ssi/ssi.h"
#include "hw/ide/pci.h"
#include "hw/ide/ahci_internal.h"
#include "hw/pci/pcie_host.h"
#include "hw/pci/pcie_port.h"
#include "qemu/units.h"
#include <stdlib.h>
#include <stddef.h>
#include <sys/time.h>
#include <time.h>
#if defined(TARGET_LOONGARCH32)
uint64_t cpu_la32_KPn_to_phys(void *opaque, uint64_t addr)
{
     return addr & 0x1fffffffUL;
}
#endif
#define CORES_PER_NODE  4
#define MAX_CORES   256

#define PHYS_TO_VIRT(x) ((x) | ~(target_ulong)0x7fffffff)
#define TARGET_REALPAGE_MASK (TARGET_PAGE_MASK << 2)


/* la32_soc io */
/* la32_soc end */

int cores_per_node;
/* i8254 PIT is attached to the IRQ0 at PIC i8259 */
static struct _loaderparams {
    unsigned long ram_size;
    struct NumaState *numa;
    const char *kernel_filename;
    const char *kernel_cmdline;
    const char *initrd_filename;
    target_ulong a0, a1, a2;
} loaderparams;
static void *boot_params_buf;

#define align(x) (((x) + 15) & ~15)

#define _str(x) #x
#define str(x) _str(x)
#define SIMPLE_OPS(ADDR , SIZE) \
({\
     MemoryRegion *iomem = g_new(MemoryRegion, 1);\
     memory_region_init_io(iomem, NULL, &la32_qemu_ops, \
                                (void *)ADDR, str(ADDR) , SIZE);\
     memory_region_add_subregion_overlap(address_space_mem, ADDR, iomem, 1);\
     iomem;\
})

static int clkreg[2] = { 0x3, 0x86184000 };


static void la32_qemu_writel(void *opaque, hwaddr addr,
        uint64_t val, unsigned size)
{
    /* loongson 32 primary */
    addr = ((hwaddr)(long)opaque) + addr;
    switch (addr) {
    case 0x1fe78030:
    case 0x1fe78034:
        clkreg[(addr - 0x1fe78030) / 4] = val;
        break;
    }
}

static uint64_t la32_qemu_readl(void *opaque, hwaddr addr, unsigned size)
{
    addr = ((hwaddr)(long)opaque) + addr;
    switch (addr) {
    case 0x1fe78030:
    case 0x1fe78034:
        return clkreg[(addr - 0x1fe78030) / 4];
    }
    return 0;
}

static const MemoryRegionOps la32_qemu_ops = {
    .read = la32_qemu_readl,
    .write = la32_qemu_writel,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

typedef struct ResetData {
    LoongArchCPU *cpu;
    uint64_t vector;
} ResetData;

#define BOOTPARAM_PHYADDR ((0x4f << 20))
#define BOOTPARAM_ADDR (0xa0000000UL + BOOTPARAM_PHYADDR)

/* should set argc,argv */
static int set_bootparam(ram_addr_t initrd_offset, long initrd_size)
{
    target_ulong params_size;
    void *params_buf;
    target_ulong *parg_env;
    target_ulong ret;

    /* Store command line.  */
    params_size = 0x100000;
    params_buf = g_malloc(params_size);

    parg_env = (void *) params_buf;

    /*
     * pram buf like this:
     * argv[0] argv[1] 0 env[0] env[1] ...env[i], 0, argv[0]'s data,
     * argv[1]'s data , env[0]'data, ... , env[i]'s data
     */

    /* jump over argv and env area */
    ret = (3 + 1) * sizeof(target_ulong);
    /* argv0 */
    *parg_env++ = BOOTPARAM_ADDR + ret;
    ret = ret + 1 + snprintf(params_buf + ret, 256 - ret, "g");
    /* argv1 */
    *parg_env++ = BOOTPARAM_ADDR + ret;
    if (initrd_size > 0) {
        ret += 1 + snprintf(params_buf + ret, 256 - ret, "rd_start=0x"
                TARGET_FMT_lx " rd_size=%li %s",
                PHYS_TO_VIRT((uint32_t)initrd_offset),
                initrd_size, loaderparams.kernel_cmdline);
    } else {
        ret += 1 + snprintf(params_buf + ret, 256 - ret,
                "%s", loaderparams.kernel_cmdline);
    }
    /* argv2 */
    *parg_env++ = 0;

    /* Mem-related envs */
    {
        char memenv[32];
        char highmemenv[32];
        NumaState *ns = loaderparams.numa;
        uint64_t ram0_size = ns->nodes[0].node_mem;
        /* Node 0 */
        sprintf(memenv, "%ld", ram0_size > 0x10000000 ?
                256 : (ram0_size >> 20));
        sprintf(highmemenv, "%ld", ram0_size > 0x10000000 ?
                (ram0_size >> 20) - 256 : 0);
        setenv("memsize", memenv, 1);
        setenv("highmemsize", highmemenv, 1);
    }

    ret = ((ret + 32) & ~31);

    boot_params_buf = (void *)(params_buf + ret);
    /* copy params_buf to physical address.La32 Physical Address Base is */
    rom_add_blob_fixed("params", params_buf, params_size,
            BOOTPARAM_PHYADDR);
    loaderparams.a0 = 2;
    loaderparams.a1 = BOOTPARAM_ADDR;
    loaderparams.a2 = BOOTPARAM_ADDR + ret;
    return 0;
}

static int64_t load_kernel(void)
{
    int64_t entry, kernel_low, kernel_high;
    long kernel_size, initrd_size;
    ram_addr_t initrd_offset;

    if (getenv("BOOTROM")) {
        kernel_size = load_image_targphys(loaderparams.kernel_filename,
                (kernel_high = qemu_strtoul(getenv("BOOTROM"), 0, 0)),
                loaderparams.ram_size);
        /*qemu_get_ram_ptr*/
        kernel_high += kernel_size;
        entry = 0;
    } else {
        kernel_size = load_elf(loaderparams.kernel_filename, NULL,
                cpu_la32_KPn_to_phys, NULL,
                (uint64_t *)&entry, (uint64_t *)&kernel_low,
                (uint64_t *)&kernel_high, NULL, 0, EM_LOONGARCH, 1, 0);
        if (kernel_size >= 0) {
            if ((entry & ~0x7fffffffULL) == 0x80000000) {
                entry = (int32_t)entry;
            }
        } else {
            fprintf(stderr, "qemu: could not load kernel '%s'\n",
                    loaderparams.kernel_filename);
            exit(1);
        }
    }

    /* load initrd */
    initrd_size = 0;
    initrd_offset = 0;
    if (loaderparams.initrd_filename) {
        initrd_size = get_image_size(loaderparams.initrd_filename);
        if (initrd_size > 0) {
            initrd_offset = (kernel_high + ~TARGET_REALPAGE_MASK)
                            & TARGET_REALPAGE_MASK;
            if (initrd_offset + initrd_size > loaderparams.ram_size) {
                fprintf(stderr,
                        "qemu: memory too small for initial ram disk '%s'\n",
                        loaderparams.initrd_filename);
                exit(1);
            }
            if (getenv("INITRD_OFFSET")) {
                initrd_offset = qemu_strtoul(getenv("INITRD_OFFSET"), 0, 0);
            }
            initrd_size = load_image_targphys(loaderparams.initrd_filename,
                    initrd_offset, loaderparams.ram_size - initrd_offset);
            /*qemu_get_ram_ptr*/
        }
        if (initrd_size == (target_ulong)-1) {
            fprintf(stderr, "qemu: could not load initial ram disk '%s'\n",
                    loaderparams.initrd_filename);
            exit(1);
        }
    }

    set_bootparam(initrd_offset, initrd_size);

    return entry;
}
static void main_cpu_reset(void *opaque)
{
    ResetData *s = (ResetData *)opaque;
    CPULoongArchState *env = &s->cpu->env;

    cpu_reset(CPU(s->cpu));
    env->pc = s->vector;
    env->gpr[4] = loaderparams.a0;
    env->gpr[5] = loaderparams.a1;
    env->gpr[6] = loaderparams.a2;
    /*
     * TODO:
     * now we use these code to set PG mode before enter system
     */
    env->CSR_DMW[0] = 0xa0000011;
    env->CSR_DMW[1] = 0x80000011;
    env->CSR_CRMD   = 0xb0;
}

static CPULoongArchState *mycpu[MAX_CORES];

/* #define DEBUG_LA32 */
/*
 * interrupt controller in la32.
 */

static void loongarch_cpu_set_irq(void *opaque, int irq, int level)
{
    LoongArchCPU *cpu = opaque;
    CPULoongArchState *env = &cpu->env;
    CPUState *cs = CPU(cpu);
    /* if illegal irq */
    if (irq < 0 || irq > N_IRQS) {
        return;
    }
    /* if level irq */
    if (level) {
        env->CSR_ESTAT |= 1 << irq;
    } else {
        env->CSR_ESTAT &= ~(1 << irq);
    }
    /* judge cpuic irq */
    if (FIELD_EX64(env->CSR_ESTAT, CSR_ESTAT, IS)) {
        cpu_interrupt(cs, CPU_INTERRUPT_HARD);
    } else {
        cpu_reset_interrupt(cs, CPU_INTERRUPT_HARD);
    }
}


static void loongson32_init(MachineState *machine)
{
    ram_addr_t ram_size = machine->ram_size;
    const char *kernel_filename = machine->kernel_filename;
    const char *kernel_cmdline = machine->kernel_cmdline;
    const char *initrd_filename = machine->initrd_filename;
    MemoryRegion *address_space_mem = get_system_memory();
    MemoryRegion *rams[MAX_NODES] = {NULL};
    MemoryRegion *ram;
    LoongArchCPU *cpu;
    CPULoongArchState *env;
    ResetData **reset_info;
    MemoryRegion *iomem_root = g_new(MemoryRegion, 1);
    AddressSpace *as = g_new(AddressSpace, 1);
    int i;
    struct NumaState *ns = machine->numa_state;
    int ls3a_num_nodes;

    /*
     * Loongisa kernel treats smp-16 as 4 nodes, so we have to
     * init node-related memory ops
     */

    if (getenv("cores_per_node")) {
        cores_per_node = atoi(getenv("cores_per_node"));
    } else {
        cores_per_node = 4;
    }
    ls3a_num_nodes = (machine->smp.cpus + cores_per_node - 1) / cores_per_node;

    if (ls3a_num_nodes > 1) {
        assert(ls3a_num_nodes == ns->num_nodes);
    }
    reset_info = g_malloc0(sizeof(ResetData *) * machine->smp.cpus);
    /* One node default */
    if (ns->num_nodes == 0) {
        ns->num_nodes = 1;
        ns->nodes[0].node_mem = ram_size;
    }

    /* init CPUs */
    for (i = 0; i < machine->smp.cpus; i++) {
        cpu = LOONGARCH_CPU(cpu_create(machine->cpu_type));
        qdev_init_gpio_in(DEVICE(cpu), loongarch_cpu_set_irq, N_IRQS);
        env = &cpu->env;
        env->CSR_TID |= i;
        mycpu[i] = env;
        reset_info[i] = g_malloc0(sizeof(ResetData));
        reset_info[i]->cpu = cpu;
        reset_info[i]->vector = env->pc;
        qemu_register_reset(main_cpu_reset, reset_info[i]);

        timer_init_ns(&cpu->timer, QEMU_CLOCK_VIRTUAL,
                   &loongarch_constant_timer_cb, cpu);
    }

    env = mycpu[0];
     /* Allocate RAM */
    for (i = 0; i < ns->num_nodes; i++) {
        rams[i] = g_new(MemoryRegion, 1);
    }
    ram = rams[0];

    printf("%s: num_nodes %d\n", __func__, ns->num_nodes);
    for (i = 0; i < ns->num_nodes; i++) {
        printf("%s: node %d mem 0x%lx\n", __func__, i, ns->nodes[i].node_mem);
    }

    memory_region_init(iomem_root, NULL,  "ls3a axi", UINT64_MAX);
    address_space_init(as, iomem_root, "ls3a axi memory");

    /* Node 0 */
    {
        uint64_t ram0_size = ns->nodes[0].node_mem;
        memory_region_init_ram(ram, NULL, "loongarch32.ram0",
                ram0_size, &error_fatal);
        memory_region_add_subregion(address_space_mem, 0x0, ram);
    }
    /* Other nodes */
    {
        for (i = 1; i < ns->num_nodes; i++) {
            rams[i] = g_new(MemoryRegion, 1);
            MemoryRegion *ram1 = g_new(MemoryRegion, 1);
            hwaddr off = ((hwaddr)i << 44);
            uint64_t nm_size = ns->nodes[i].node_mem;
            char name[32];
            sprintf(name, "%s%d\n", "la32.ram", i);

            memory_region_init_ram(rams[i], NULL, name, nm_size, &error_fatal);
            memory_region_init_alias(ram1, NULL, "lowmem",
                    rams[i], 0, MIN(nm_size, 0x10000000));
            memory_region_add_subregion(address_space_mem, off, ram1);
            memory_region_add_subregion(address_space_mem,
                    off + 0x80000000ULL, rams[i]);
        }
    }
    MemoryRegion *ram2 = g_new(MemoryRegion, 1);
    memory_region_init_alias(ram2, NULL, "dmalowmem", ram, 0,
            MIN(ram_size, 0x10000000));
    memory_region_add_subregion(iomem_root, 0, ram2);



    /*
     * Try to load a BIOS image. If this fails, we continue regardless,
     * but initialize the hardware ourselves. When a kernel gets
     * preloaded we also initialize the hardware, since the BIOS wasn't
     * run.
     */
    if (kernel_filename) {
        loaderparams.ram_size = ram_size;
        loaderparams.kernel_filename = kernel_filename;
        loaderparams.kernel_cmdline = kernel_cmdline;
        loaderparams.initrd_filename = initrd_filename;
        loaderparams.numa = ns;
        reset_info[0]->vector = load_kernel() ? : reset_info[0]->vector;
    }

    /* todo: */
    env->CSR_DMW[0] = 0xa0000011;
    env->CSR_DMW[1] = 0x80000011;
    env->CSR_CRMD   = 0xb0;

    DeviceState *cpudev = DEVICE(qemu_get_cpu(0));
    serial_mm_init(address_space_mem, 0x1fe001e0, 0,
            qdev_get_gpio_in(cpudev, 3), 115200, serial_hd(0),
            DEVICE_NATIVE_ENDIAN);

    if (0) {
        DeviceState *dev;
        dev = qdev_new("sysbus-synopgmac");
        if (nd_table[0].used) {
                qdev_set_nic_properties(dev, &nd_table[0]);
        }
        qdev_prop_set_int32(dev, "enh_desc", 1);
        qdev_prop_set_uint32(dev, "version", 0xd137);
        qdev_prop_set_uint32(dev, "hwcap", 0x1b0d2fbf);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, 0x1ff00000);
        sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, qdev_get_gpio_in(cpudev, 2));
    }

        /*
         * FIXME: la32_soc's real network card is
         * not synopgmac, but the real one
         * is not supported by qemu currently, enable synopgmac
         * here for only qemu network temporarily.
         */
    {
        MemoryRegion *iomem = g_new(MemoryRegion, 1);
        memory_region_init_io(iomem, NULL, &la32_qemu_ops,
                (void *)0x1fe78030, "0x1fe78030", 0x8);
        memory_region_add_subregion(address_space_mem, 0x1fe78030, iomem);
    }

    {
        MemoryRegion *iomem = g_new(MemoryRegion, 1);
        memory_region_init_io(iomem, NULL, &la32_qemu_ops,
                (void *)0x1fd00420, "0x1fd00420", 0x8);
        memory_region_add_subregion(address_space_mem, 0x1fd00420, iomem);
    }
    g_free(reset_info);
}

CpuInstanceProperties
ls3a_cpu_index_to_props(MachineState *ms, unsigned cpu_index)
{
    MachineClass *mc = MACHINE_GET_CLASS(ms);
    const CPUArchIdList *possible_cpus = mc->possible_cpu_arch_ids(ms);

    assert(cpu_index < possible_cpus->len);
    return possible_cpus->cpus[cpu_index].props;
}

int64_t ls3a_get_default_cpu_node_id(const MachineState *ms, int idx)
{
    MachineClass *mc = MACHINE_GET_CLASS(ms);
    CPUArchIdList *possible_cpus = mc->possible_cpu_arch_ids(ms);

    assert(idx < possible_cpus->len);
    return possible_cpus->cpus[idx].props.node_id;
}

const CPUArchIdList *ls3a_possible_cpu_arch_ids(MachineState *ms)
{
    int i;
    unsigned int max_cpus = ms->smp.max_cpus;

    if (ms->possible_cpus) {
        /*
         * make sure that max_cpus hasn't changed since the first use, i.e.
         * -smp hasn't been parsed after it
         */
        assert(ms->possible_cpus->len == max_cpus);
        return ms->possible_cpus;
    }

    ms->possible_cpus = g_malloc0(sizeof(CPUArchIdList) +
                                  sizeof(CPUArchId) * max_cpus);
    ms->possible_cpus->len = max_cpus;

    if (getenv("cores_per_node")) {
        cores_per_node = atoi(getenv("cores_per_node"));
    } else {
        cores_per_node = 4;
    }
    for (i = 0; i < ms->possible_cpus->len; i++) {
        ms->possible_cpus->cpus[i].type = ms->cpu_type;
        ms->possible_cpus->cpus[i].vcpus_count = 1;
        ms->possible_cpus->cpus[i].props.has_socket_id = false;
        ms->possible_cpus->cpus[i].props.has_die_id = false;
        ms->possible_cpus->cpus[i].props.has_thread_id = false;
        ms->possible_cpus->cpus[i].props.has_core_id = true;
        ms->possible_cpus->cpus[i].props.core_id = i;
        ms->possible_cpus->cpus[i].props.has_node_id = true;
        ms->possible_cpus->cpus[i].props.node_id = i % cores_per_node;
    }
    return ms->possible_cpus;
}

static void ls3a5k32_machine_init(MachineClass *mc)
{
    mc->desc = "ls3a32 test platform";
    mc->init = loongson32_init;
    mc->max_cpus = 32;
    mc->block_default_type = IF_IDE;
    mc->default_cpu_type = LOONGARCH_CPU_TYPE_NAME("la32");
    setenv("has_nodecounter", "1", 1);
    mc->cpu_index_to_instance_props = ls3a_cpu_index_to_props;
    mc->get_default_cpu_node_id = ls3a_get_default_cpu_node_id;
    mc->possible_cpu_arch_ids = ls3a_possible_cpu_arch_ids;
}
DEFINE_MACHINE("ls3a5k32", ls3a5k32_machine_init)
