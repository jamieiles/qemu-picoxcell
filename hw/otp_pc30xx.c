/*
 * QEMU model of the picoxcell pc30xx OTP block.
 *
 * Copyright (c) 2011 Picochip Ltd, Jamie Iles
 *
 * This code is licenced under the GPL.
 */
#include <stdio.h>
#include "bitmap.h"
#include "sysbus.h"
#include "qemu-timer.h"
#include "qemu-log.h"

struct pc30xx_otp {
    SysBusDevice busdev;
    QEMUBH *bh;
    ptimer_state *ptimer;
    uint32_t pwdn;
#define PWDN_DIRIO_EN           (1 << 31)
#define PWDN_PWDN_EN            (1 << 0)
    uint32_t status;
#define STATUS_PGM_FAIL         (1 << 3)
#define STATUS_JTAG_EN          (1 << 2)
#define STATUS_IN_PWDN          (1 << 1)
#define STATUS_BUSY             (1 << 0)
    uint32_t rstb_dly;
    uint32_t ceb_dly;
    uint32_t readen_dly;
    uint32_t web_width;
    uint32_t web_setup;
    uint32_t web_hold;
    uint32_t pgmen_dly;
    uint32_t pgm_addr;
    uint32_t pgm_cyc;
    uint32_t pgm_datal;
    uint32_t pgm_datah;
    uint32_t pgm_cmd;
#define PGM_CMD_START_MAGIC      0x50524f47
    uint32_t direct_io;
#define PGM_STATUS_IN_PROG      (1 << 0)
    uint32_t pgm_status;
    uint32_t io_flags;
#define IO_FLAG_ROPB_BASE       0
#define IO_FLAG_LTP_BASE        4
#define IO_FLAG_GLOBAL_LTP      (1 << 8)
    unsigned long read_bitmap[BITS_TO_LONGS(4096)];

    int backing;
};

static uint32_t otp_read_word(struct pc30xx_otp *otp, uint32_t word_addr)
{
    uint32_t val;
    ssize_t br;

    lseek(otp->backing, word_addr * sizeof(uint32_t), SEEK_SET);
    br = read(otp->backing, &val, sizeof(uint32_t));
    assert(br == sizeof(uint32_t));

    return val;
}

static void otp_write_word(struct pc30xx_otp *otp, uint32_t word_addr,
                           uint32_t val)
{
    ssize_t bw;

    lseek(otp->backing, word_addr * sizeof(val), SEEK_SET);
    bw = write(otp->backing, &val, sizeof(val));
    assert(bw == sizeof(val));
}

static uint32_t pc30xx_otp_readl(void *opaque, target_phys_addr_t addr)
{
    struct pc30xx_otp *otp = opaque;
    uint32_t word_addr = addr / 4;
    uint32_t sector = addr / 4096;

    switch (addr) {
    case 0x4000:
        return otp->pwdn;
    case 0x4004:
        return otp->status;
    case 0x4008:
        return otp->rstb_dly;
    case 0x400c:
        return otp->ceb_dly;
    case 0x4010:
        return otp->readen_dly;
    case 0x4014:
        return otp->web_width;
    case 0x4018:
        return otp->web_setup;
    case 0x401c:
        return otp->web_hold;
    case 0x4020:
        return otp->pgmen_dly;
    case 0x4028:
        return otp->pgm_addr;
    case 0x402c:
        return otp->pgm_cyc;
    case 0x4030:
        return otp->pgm_datal;
    case 0x4034:
        return otp->pgm_datah;
    case 0x4038:
        return otp->pgm_cmd;
    case 0x403c:
        return otp->pgm_status;
    case 0x4048:
        return otp->direct_io;

    case 0x0000 ... 0x3fff:
        /*
         * The Read Once Per Boot flags work on 32-bit quantities.  We track
         * accesses to each word so that it can only be read once.  If the
         * word has previously been read, return all 1's.
         */
        if ((otp->io_flags & (1 << (IO_FLAG_ROPB_BASE + sector))) &&
            test_and_set_bit(word_addr, otp->read_bitmap))
                return ~0;
        return otp_read_word(otp, word_addr);

    default:
        return 0;
    }
}

/*
 * Check if we can write to the given sector.  If either the global last time
 * program or the sector last time program is active then it's a no-go.
 */
static bool otp_sector_can_write(struct pc30xx_otp *otp, uint32_t sector)
{
    return !((otp->io_flags & IO_FLAG_GLOBAL_LTP) ||
             (otp->io_flags & (1 << (IO_FLAG_LTP_BASE + sector))));
}

static void otp_prog_done(void *opaque)
{
    struct pc30xx_otp *otp = opaque;
    uint32_t addr = otp->pgm_addr / 4;
    uint32_t sector = otp->pgm_addr / 1024;

    if (otp_sector_can_write(otp, sector)) {
        uint32_t v;
        v = otp_read_word(otp, addr);
        otp_write_word(otp, addr, v | otp->pgm_datal);
        v = otp_read_word(otp, addr + 1);
        otp_write_word(otp, addr + 1, v | otp->pgm_datah);
    }

    otp->status &= ~STATUS_BUSY;
    otp->pgm_status &= ~PGM_STATUS_IN_PROG;
}

static void otp_start_program(struct pc30xx_otp *otp)
{
    otp->status |= STATUS_BUSY;
    otp->pgm_status |= PGM_STATUS_IN_PROG;

    /*
     * Emulate a 250us delay for programming completion.  The time it takes in
     * reality can vary dependent on the process...
     */
    ptimer_stop(otp->ptimer);
    ptimer_set_limit(otp->ptimer, 250, 1);
    ptimer_run(otp->ptimer, 0);
}

static void pc30xx_otp_writel(void *opaque, target_phys_addr_t addr,
                              uint32_t val)
{
    struct pc30xx_otp *otp = opaque;

    switch (addr) {
    case 0x4000:
        otp->pwdn = val & 0x80000001;
        otp->status &= ~(otp->status & STATUS_IN_PWDN);
        if (val & PWDN_PWDN_EN)
            otp->status |= STATUS_IN_PWDN;
        break;
    case 0x4004:
        otp->status &= ~STATUS_PGM_FAIL;
        break;
    case 0x4008:
        otp->rstb_dly = val & ((1 << 20) - 1);
        break;
    case 0x400c:
        otp->ceb_dly = val & (0xffff);
        break;
    case 0x4010:
        otp->readen_dly = val & ((1 << 10) - 1);
        break;
    case 0x4014:
        otp->web_width = val & 0xffff;
        break;
    case 0x4018:
        otp->web_setup = val & 0xffff;
        break;
    case 0x401c:
        otp->web_hold = val & 0xffff;
        break;
    case 0x4020:
        otp->pgmen_dly = val & 0xffffff;
        break;
    case 0x4028:
        otp->pgm_addr = val & 0x1ffff;
        break;
    case 0x402c:
        otp->pgm_cyc = val & 0x1ff;
        break;
    case 0x4030:
        otp->pgm_datal = val;
        break;
    case 0x4034:
        otp->pgm_datah = val;
        break;
    case 0x4038:
        otp->pgm_cmd = val;
        if (otp->pgm_cmd == PGM_CMD_START_MAGIC)
            otp_start_program(otp);
        break;
    case 0x4048:
        otp->direct_io = val & 0x0f07ffff;
        break;
    }
}

static CPUReadMemoryFunc * const pc30xx_otp_read[] = {
    NULL,
    NULL,
    pc30xx_otp_readl
};

static CPUWriteMemoryFunc * const pc30xx_otp_write[] = {
    NULL,
    NULL,
    pc30xx_otp_writel
};

static void pc30xx_otp_set_io(void *opaque, int irq, int level)
{
    struct pc30xx_otp *otp = opaque;

    if (level)
        otp->io_flags |= (1 << irq);
    else
        otp->io_flags &= ~(1 << irq);
}

static int pc30xx_otp_init(SysBusDevice *dev)
{
    struct pc30xx_otp *t = FROM_SYSBUS(struct pc30xx_otp, dev);
    int regs, err;

    t->backing = open("otp_pc30xx.bin", O_RDWR | O_CREAT, 0644);
    /* Make sure that all of the OTP exists. */
    err = ftruncate(t->backing, 16 * 1024);
    if (err)
        goto out_close;

    regs = cpu_register_io_memory(pc30xx_otp_read, pc30xx_otp_write, t,
				  DEVICE_NATIVE_ENDIAN);
    sysbus_init_mmio(dev, 0x8000, regs);
    /*
     * We have 9 I/O's:
     *  - 4 x Read Once Per Boot flags
     *  - 4 x Last Time Program flags
     *  - 1 x Global Last Time Program flag
     */
    qdev_init_gpio_in(&dev->qdev, pc30xx_otp_set_io, 9);

    t->bh = qemu_bh_new(otp_prog_done, t);
    t->ptimer = ptimer_init(t->bh);
    ptimer_set_freq(t->ptimer, 1000000);

    return 0;

out_close:
    close(t->backing);

    return err;
}

static SysBusDeviceInfo pc30xx_otp_info = {
    .init = pc30xx_otp_init,
    .qdev.name = "pc30xx_otp",
    .qdev.size = sizeof(struct pc30xx_otp),
    .qdev.props = (Property[]) {
        DEFINE_PROP_END_OF_LIST(),
    }
};

static void pc30xx_otp_register(void)
{
    sysbus_register_withprop(&pc30xx_otp_info);
}
device_init(pc30xx_otp_register);
