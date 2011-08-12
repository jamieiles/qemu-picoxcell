/*
 * Picochip picoxcell picoxcell emulation.
 *
 * Copyright (c) 2011 Picochip Ltd, Jamie Iles
 *
 * This code is licenced under the GPL.
 */

#include "sysbus.h"
#include "arm-misc.h"
#include "primecell.h"
#include "devices.h"
#include "net.h"
#include "sysemu.h"
#include "pci.h"
#include "usb-ohci.h"
#include "boards.h"
#include "blockdev.h"
#include "pc.h"
#include "flash.h"

static struct arm_boot_info picoxcell_binfo;

struct nand_state_t {
    SysBusDevice busdev;
    DeviceState *nand;
    int rdy;
    int ale;
    int cle;
    int ce;

    qemu_irq rdy_irq;
};

static uint32_t nand_readl(void *opaque, target_phys_addr_t addr)
{
    struct nand_state_t *s = opaque;
    uint32_t r;
    int rdy;

    r = nand_getio(s->nand);
    nand_getpins(s->nand, &rdy);
    if (s->rdy != rdy) {
        s->rdy = rdy;
        qemu_set_irq(s->rdy_irq, rdy);
    }

    return r;
}

static void
nand_writel(void *opaque, target_phys_addr_t addr, uint32_t value)
{
    struct nand_state_t *s = opaque;
    int rdy;

    nand_setpins(s->nand, s->cle, s->ale, s->ce, 1, 0);
    nand_setio(s->nand, value & 0xff);
    nand_getpins(s->nand, &rdy);

    if (s->rdy != rdy) {
        s->rdy = rdy;
        qemu_set_irq(s->rdy_irq, rdy);
    }
}

static CPUReadMemoryFunc * const nand_read[] = {
    &nand_readl,
    &nand_readl,
    &nand_readl,
};

static CPUWriteMemoryFunc * const nand_write[] = {
    &nand_writel,
    &nand_writel,
    &nand_writel,
};

static void picoxcell_nand_io(void *opaque, int irq, int level)
{
    struct nand_state_t *t = opaque;

    assert(irq >= 0 && irq <= 2);

    if (irq == 0)
        t->ale = level;
    else if (irq == 1)
        t->cle = level;
    else if (irq == 2)
        t->ce = level;
}

static int picoxcell_nand_init(SysBusDevice *dev)
{
    struct nand_state_t *t = FROM_SYSBUS(struct nand_state_t, dev);
    int regs;
    struct DriveInfo *nand;

    nand = drive_get(IF_MTD, 0, 0);
    t->nand = nand_init(nand ? nand->bdrv : NULL, NAND_MFR_MICRON, 0xda);

    regs = cpu_register_io_memory(nand_read, nand_write, t,
                                  DEVICE_NATIVE_ENDIAN);
    sysbus_init_mmio(dev, 0x08000000, regs);
    qdev_init_gpio_in(&dev->qdev, picoxcell_nand_io, 3);
    sysbus_init_irq(dev, &t->rdy_irq);

    return 0;
}

static SysBusDeviceInfo picoxcell_nand_info = {
    .init = picoxcell_nand_init,
    .qdev.name = "picoxcell_nand",
    .qdev.size = sizeof(struct nand_state_t),
    .qdev.props = (Property[]) {
        DEFINE_PROP_END_OF_LIST(),
    }
};

static void picoxcell_nand_register(void)
{
    sysbus_register_withprop(&picoxcell_nand_info);
}
device_init(picoxcell_nand_register);

static void picoxcell_init(ram_addr_t ram_size,
                        const char *boot_device,
                        const char *kernel_filename,
                        const char *kernel_cmdline,
                        const char *initrd_filename,
                        const char *cpu_model,
                        const char *dtb_file,
                        int board_id,
                        uint16_t device_id)
{
    CPUState *env;
    ram_addr_t ram_offset;
    qemu_irq *cpu_pic;
    qemu_irq vic0[32];
    qemu_irq vic1[32];
    qemu_irq otp_io[9];
    DeviceState *dev, *fuse, *nand;
    int n;
    qemu_irq rdy;

    if (!cpu_model)
        cpu_model = "arm1176";
    env = cpu_init(cpu_model);
    if (!env) {
        fprintf(stderr, "Unable to find CPU definition\n");
        exit(1);
    }
    ram_offset = qemu_ram_alloc(NULL, "picoxcell.ram", ram_size);
    cpu_register_physical_memory(0, ram_size, ram_offset | IO_MEM_RAM);

    ram_offset = qemu_ram_alloc(NULL, "picoxcell.sram", 128 * 1024);
    cpu_register_physical_memory(0x20000000, 128 * 1024, ram_offset | IO_MEM_RAM);

    ram_offset = qemu_ram_alloc(NULL, "fuse.ram", 0x10000);
    cpu_register_physical_memory(0, 0x10000, ram_offset | IO_MEM_RAM);

    /*
     * Register the VIC's.  picoxcell actually has 2xPL192's rather than
     * PL190's, but we can use the pl190 model as long as the system doesn't
     * require vectored interrupt support.  The picoxcell Linux port doesn't
     * use this so we're OK for now.
     */
    cpu_pic = arm_pic_init_cpu(env);
    dev = sysbus_create_varargs("pl190", 0x80060000,
                                cpu_pic[0], cpu_pic[1], NULL);
    for (n = 0; n < 32; n++)
        vic0[n] = qdev_get_gpio_in(dev, n);

    dev = sysbus_create_varargs("pl190", 0x80064000,
                                cpu_pic[0], cpu_pic[1], NULL);
    for (n = 0; n < 32; n++)
        vic1[n] = qdev_get_gpio_in(dev, n);

    serial_mm_init(0x80230000, 2, vic1[10], 115200, serial_hds[0], 1, 0);
    serial_mm_init(0x80240000, 2, vic1[9], 115200, serial_hds[1] ?
                   serial_hds[1] : qemu_chr_open("uart2", "null", NULL), 1,
                   0);

    if (device_id == 0x8003 || device_id == 0x8007) {
        dev = sysbus_create_varargs("axi2cfg,pc3x2", 0x800A0000, vic0[9], vic0[8],
                                    NULL);
        fuse = sysbus_create_varargs("picoxcell_fuse", 0x80080000, NULL);
    } else if (device_id == 0x20 || device_id == 0x21 || device_id == 0x22) {
        dev = sysbus_create_varargs("axi2cfg,pc3x3", 0x800A0000, vic0[9], vic0[8],
                                    NULL);
        fuse = sysbus_create_varargs("picoxcell_fuse", 0x80080000, NULL);
    } else {
        dev = sysbus_create_simple("pc30xx_otp", 0xffff8000, NULL);
        for (n = 0; n < 9; ++n) {
            otp_io[n] = qdev_get_gpio_in(dev, n);
        }

        fuse = sysbus_create_varargs("picoxcell_fuse", 0x80080000, otp_io[0],
                                     otp_io[1], otp_io[2], otp_io[3],
                                     otp_io[4], otp_io[5], otp_io[6],
                                     otp_io[7], otp_io[8], NULL);
        dev = sysbus_create_simple("axi2cfg,pc30xx", 0x800A0000, vic0[8]);
    }
    qdev_prop_set_uint32(fuse, "device_id", device_id);
    qdev_prop_set_uint32(dev, "device_id", device_id);

    sysbus_create_varargs("dwapb_timer", 0x80210000, vic0[4], vic0[5], NULL);
    sysbus_create_simple("dwapb_rtc", 0x80200000, NULL);
    dev = sysbus_create_simple("dwapb_gpio", 0x80220000, NULL);

    rdy = qdev_get_gpio_in(dev, 1);
    nand = sysbus_create_varargs("picoxcell_nand", 0x50000000, rdy, NULL);

    qdev_connect_gpio_out(dev, 3, qdev_get_gpio_in(nand, 0));
    qdev_connect_gpio_out(dev, 4, qdev_get_gpio_in(nand, 1));
    qdev_connect_gpio_out(dev, 2, qdev_get_gpio_in(nand, 2));

    picoxcell_binfo.ram_size = ram_size;
    picoxcell_binfo.kernel_filename = kernel_filename;
    picoxcell_binfo.kernel_cmdline = kernel_cmdline;
    picoxcell_binfo.initrd_filename = initrd_filename;
    picoxcell_binfo.board_id = board_id;
    picoxcell_binfo.dtb_filename = dtb_file;
    arm_load_kernel(env, &picoxcell_binfo);
}

static void pc7302_pc3x2_init(ram_addr_t ram_size,
                              const char *boot_device,
                              const char *kernel_filename,
                              const char *kernel_cmdline,
                              const char *initrd_filename,
                              const char *cpu_model)
{
    picoxcell_init(ram_size, boot_device, kernel_filename, kernel_cmdline,
                initrd_filename, cpu_model, NULL, 2220, 0x8003);
}

static QEMUMachine pc7302_pc3x2_machine = {
    .name = "pc7302-pc3x2",
    .desc = "picoxcell pc7302 (ARM1176JZ-S)",
    .init = pc7302_pc3x2_init,
};

static void pc7302_pc3x3_init(ram_addr_t ram_size,
                              const char *boot_device,
                              const char *kernel_filename,
                              const char *kernel_cmdline,
                              const char *initrd_filename,
                              const char *cpu_model)
{
    picoxcell_init(ram_size, boot_device, kernel_filename, kernel_cmdline,
                   initrd_filename, cpu_model, NULL, 2220, 0x0022);
}

static QEMUMachine pc7302_pc3x3_machine = {
    .name = "pc7302-pc3x3",
    .desc = "picoxcell pc7302 (ARM1176JZ-S)",
    .init = pc7302_pc3x3_init,
};

static void pc7302_dt_init(ram_addr_t ram_size,
                              const char *boot_device,
                              const char *kernel_filename,
                              const char *kernel_cmdline,
                              const char *initrd_filename,
                              const char *cpu_model)
{
    picoxcell_init(ram_size, boot_device, kernel_filename, kernel_cmdline,
                   initrd_filename, cpu_model, "pc7302.dtb", 0xffffffff, 0x0022);
}

static QEMUMachine pc7302_dt_machine = {
    .name = "pc7302-dt",
    .desc = "picoxcell device tree (ARM1176JZ-S)",
    .init = pc7302_dt_init,
};

static void pc7308_pc3008_init(ram_addr_t ram_size,
                               const char *boot_device,
                               const char *kernel_filename,
                               const char *kernel_cmdline,
                               const char *initrd_filename,
                               const char *cpu_model)
{
    picoxcell_init(ram_size, boot_device, kernel_filename, kernel_cmdline,
                   initrd_filename, cpu_model, NULL, 3468, 0x0030);
}

static QEMUMachine pc7308_pc3008_machine = {
    .name = "pc7308-pc3008",
    .desc = "picoxcell pc7308 (ARM1176JZ-S)",
    .init = pc7308_pc3008_init,
};

static void picoxcell_machine_init(void)
{
    qemu_register_machine(&pc7302_pc3x2_machine);
    qemu_register_machine(&pc7302_pc3x3_machine);
    qemu_register_machine(&pc7302_dt_machine);
    qemu_register_machine(&pc7308_pc3008_machine);
}
machine_init(picoxcell_machine_init);
