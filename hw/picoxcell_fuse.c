/*
 * QEMU model of the picoxcell pc30xx Fuse block.
 *
 * Copyright (c) 2011 Picochip Ltd, Jamie Iles
 *
 * This code is licenced under the GPL.
 */
#include <stdio.h>
#include "bitmap.h"
#include "sysbus.h"
#include "qdev.h"
#include "qemu-timer.h"
#include "qemu-log.h"

/*
 * A logical group of fuses. This could be a single fuse such as one to
 * disable the memif_arm on a picoXcell device or a group of fuses to
 * represent the serial number or a secure key.
 */
struct picoxcell_fuse_range {
	const char		*name;
	int			start;
	int			end;

	/*
	 * Index of the read once per boot, jtag disable and last time program
	 * fuses. If the read once per boot fuse is blown then this range will
	 * only be able to be read once per boot with valid data. Some fuse
	 * ranges will not have a read once per boot fuse so this will be -1.
	 *
	 * The jtag disable fuse prevents the range being read through the
	 * JTAG interface and the last time program prevents the range from
	 * being overwritten.
	 */
	int			read_once;
	int			jtag_disable;
	int			last_time_prog;
};

/*
 * Define a fuse range with a given name, start and end fuse index.
 */
#define FUSE_RANGE(__name, __start, __end) { \
		.name			= #__name, \
		.start			= __start, \
		.end			= __end, \
		.read_once		= -1, \
		.jtag_disable		= -1, \
		.last_time_prog		= -1, \
	}

/*
 * Define a fuse range with a given name, start and end fuse index. This range
 * also has protection bits for read once per boot, jtag disable and last time
 * program.
 */
#define FUSE_RANGE_PROTECTED(__name, __start, __end, __read_once, \
			     __last_time, __jtag_disable) { \
		.name			= #__name, \
		.start			= __start, \
		.end			= __end, \
		.read_once		= __read_once, \
		.jtag_disable		= __jtag_disable, \
		.last_time_prog		= __last_time, \
	}, \
	FUSE_RANGE(__name ## _last_time_prog, __last_time, __last_time), \
	FUSE_RANGE(__name ## _read_once, __read_once, __read_once), \
	FUSE_RANGE(__name ## _jtag_disable, __jtag_disable, __jtag_disable)

#define FUSE_RANGE_NULL {}

/*
 * The fuse map to be embedded in the picoxcell-fuse platform device as
 * platform data. The .ltp_fuse gives the global last time program fuse index.
 * If this fuse is blown then no writes to any fuse will be allowed.
 */
struct picoxcell_fuse_map {
	int				nr_fuses;
	int				ltp_fuse;

	/*
	 * The VDDQ supply to the fuse block is external to the chip and is
	 * controlled by an enable pin that controls an external transistor.
	 * This switching will take some time to reach the correct voltage and
	 * these times should be described here. To operate within spec, the
	 * VDDQ voltage should only be applied for a maximum of 1 second in
	 * the device's lifetime.
	 */
	unsigned			vddq_rise_usec;
	unsigned			vddq_fall_usec;
	struct picoxcell_fuse_range	ranges[];
};

static struct picoxcell_fuse_map pc30xx_fuse_map = {
	.nr_fuses	= 4096,
	.ltp_fuse	= 994,
	.ranges		= {
		FUSE_RANGE_PROTECTED(secure_bootstrap, 0, 127, 928, 938, 948),
		FUSE_RANGE_PROTECTED(counter_iv, 128, 255, 929, 939, 949),
		FUSE_RANGE_PROTECTED(key2, 256, 383, 930, 940, 950),
		FUSE_RANGE_PROTECTED(key3, 384, 511, 931, 941, 951),
		FUSE_RANGE_PROTECTED(key4, 512, 639, 932, 942, 952),
		FUSE_RANGE_PROTECTED(key5, 640, 767, 933, 943, 953),
		FUSE_RANGE_PROTECTED(die_ident, 768, 895, 934, 944, 954),
		FUSE_RANGE_PROTECTED(temp_cal_offset, 896, 903, 934, 944, 954),
		FUSE_RANGE_PROTECTED(partition1, 1024, 2047, 935, 945, 955),
		FUSE_RANGE_PROTECTED(partition2, 2048, 3071, 936, 946, 956),
		FUSE_RANGE_PROTECTED(partition3, 3072, 4095, 937, 947, 957),
		FUSE_RANGE(secure_boot, 992, 992),
		FUSE_RANGE(disable_tz, 993, 993),
		FUSE_RANGE(global_ltp, 994, 994),
		FUSE_RANGE(disable_debug, 995, 995),
		FUSE_RANGE(disable_isc, 996, 996),
		FUSE_RANGE(disable_jtag, 997, 997),
		FUSE_RANGE(disable_invasive_debug, 998, 998),
		FUSE_RANGE(disable_noninvasive_debug, 999, 999),
		FUSE_RANGE(disable_cp15, 1000, 1000),
		FUSE_RANGE(disable_memif_arm, 1001, 1001),
		FUSE_RANGE(disable_nonsecure_parallel_flash, 1002, 1002),
		FUSE_RANGE(global_otp_ltp, 1015, 1015),
		FUSE_RANGE(otp_disable_jtag, 1016, 1016),
		FUSE_RANGE(otp_boot_mode, 1017, 1018),
		FUSE_RANGE(otp_direct_io_disable, 1021, 1021),
		FUSE_RANGE(otp_robp1, 1003, 1003),
		FUSE_RANGE(otp_robp2, 1004, 1004),
		FUSE_RANGE(otp_robp3, 1005, 1005),
		FUSE_RANGE(otp_robp4, 1006, 1006),
		FUSE_RANGE(otp_ltp1, 1007, 1007),
		FUSE_RANGE(otp_ltp2, 1008, 1008),
		FUSE_RANGE(otp_ltp3, 1009, 1009),
		FUSE_RANGE(otp_ltp4, 1010, 1010),
		FUSE_RANGE(otp_disable_jtag1, 1011, 1011),
		FUSE_RANGE(otp_disable_jtag2, 1012, 1012),
		FUSE_RANGE(otp_disable_jtag3, 1013, 1013),
		FUSE_RANGE(otp_disable_jtag4, 1014, 1014),
		FUSE_RANGE_NULL,
	},
};

static struct picoxcell_fuse_map pc3x2_fuse_map = {
	.nr_fuses	= 4096,
	.ltp_fuse	= 994,
	.ranges		= {
		FUSE_RANGE_PROTECTED(secure_bootstrap, 0, 127, 928, 938, 948),
		FUSE_RANGE_PROTECTED(counter_iv, 128, 255, 929, 939, 949),
		FUSE_RANGE_PROTECTED(key2, 256, 383, 930, 940, 950),
		FUSE_RANGE_PROTECTED(key3, 384, 511, 931, 941, 951),
		FUSE_RANGE_PROTECTED(key4, 512, 639, 932, 942, 952),
		FUSE_RANGE_PROTECTED(key5, 640, 767, 933, 943, 953),
		FUSE_RANGE_PROTECTED(die_ident, 768, 895, 934, 944, 954),
		FUSE_RANGE_PROTECTED(partition1, 1024, 2047, 935, 945, 955),
		FUSE_RANGE_PROTECTED(partition2, 2048, 3071, 936, 946, 956),
		FUSE_RANGE_PROTECTED(partition3, 3072, 4095, 937, 947, 957),
		FUSE_RANGE(secure_boot, 992, 992),
		FUSE_RANGE(disable_tz, 993, 993),
		FUSE_RANGE(global_ltp, 994, 994),
		FUSE_RANGE(disable_debug, 995, 995),
		FUSE_RANGE(disable_isc, 996, 996),
		FUSE_RANGE(disable_jtag, 997, 997),
		FUSE_RANGE(disable_invasive_debug, 998, 998),
		FUSE_RANGE(disable_noninvasive_debug, 999, 999),
		FUSE_RANGE(disable_cp15, 1000, 1000),
		FUSE_RANGE(disable_memif_arm, 1001, 1001),
		FUSE_RANGE(disable_nonsecure_parallel_flash, 1002, 1002),
		FUSE_RANGE_NULL,
	},
};

static struct picoxcell_fuse_map pc3x3_fuse_map = {
	.nr_fuses	= 4096,
	.ltp_fuse	= 994,
	.ranges		= {
		FUSE_RANGE_PROTECTED(secure_bootstrap, 0, 127, 928, 938, 948),
		FUSE_RANGE_PROTECTED(counter_iv, 128, 255, 929, 939, 949),
		FUSE_RANGE_PROTECTED(key2, 256, 383, 930, 940, 950),
		FUSE_RANGE_PROTECTED(key3, 384, 511, 931, 941, 951),
		FUSE_RANGE_PROTECTED(key4, 512, 639, 932, 942, 952),
		FUSE_RANGE_PROTECTED(key5, 640, 767, 933, 943, 953),
		FUSE_RANGE_PROTECTED(die_ident, 768, 895, 934, 944, 954),
		FUSE_RANGE_PROTECTED(partition1, 1024, 2047, 935, 945, 955),
		FUSE_RANGE_PROTECTED(partition2, 2048, 3071, 936, 946, 956),
		FUSE_RANGE_PROTECTED(partition3, 3072, 4095, 937, 947, 957),
		FUSE_RANGE(secure_boot, 992, 992),
		FUSE_RANGE(disable_tz, 993, 993),
		FUSE_RANGE(global_ltp, 994, 994),
		FUSE_RANGE(disable_debug, 995, 995),
		FUSE_RANGE(disable_isc, 996, 996),
		FUSE_RANGE(disable_jtag, 997, 997),
		FUSE_RANGE(disable_invasive_debug, 998, 998),
		FUSE_RANGE(disable_noninvasive_debug, 999, 999),
		FUSE_RANGE(disable_cp15, 1000, 1000),
		FUSE_RANGE(disable_memif_arm, 1001, 1001),
		FUSE_RANGE(disable_nonsecure_parallel_flash, 1002, 1002),
		FUSE_RANGE(global_otp_ltp, 1015, 1015),
		FUSE_RANGE(otp_disable_jtag, 1016, 1016),
		FUSE_RANGE(otp_boot_mode, 1017, 1018),
		FUSE_RANGE(otp_robp1, 1003, 1003),
		FUSE_RANGE(otp_robp2, 1004, 1004),
		FUSE_RANGE(otp_robp3, 1005, 1005),
		FUSE_RANGE(otp_robp4, 1006, 1006),
		FUSE_RANGE(otp_ltp1, 1007, 1007),
		FUSE_RANGE(otp_ltp2, 1008, 1008),
		FUSE_RANGE(otp_ltp3, 1009, 1009),
		FUSE_RANGE(otp_ltp4, 1010, 1010),
		FUSE_RANGE(otp_disable_jtag1, 1011, 1011),
		FUSE_RANGE(otp_disable_jtag2, 1012, 1012),
		FUSE_RANGE(otp_disable_jtag3, 1013, 1013),
		FUSE_RANGE(otp_disable_jtag4, 1014, 1014),
		FUSE_RANGE_NULL,
	},
};

struct picoxcell_fuse {
    SysBusDevice busdev;
    QEMUBH *bh;
    ptimer_state *ptimer;
    qemu_irq otp_io[9];

    uint32_t device_id;
    uint32_t fuse_ctrl;
#define FUSE_CTRL_WRITE_BUSY    (1 << 0)
#define FUSE_CTRL_VDDQ_OE       (1 << 1)
#define FUSE_CTRL_VDDQ          (1 << 2)
    uint32_t fuse_write_bit_addr;

    uint32_t fuses[1024];

    unsigned long read_bitmap[BITS_TO_LONGS(1024)];
    struct picoxcell_fuse_map *map;

    int backing;
};

static uint32_t fuse_read_word_file(struct picoxcell_fuse *fuse,
                                    uint32_t word_addr)
{
    uint32_t val;
    ssize_t br;

    lseek(fuse->backing, word_addr * sizeof(uint32_t), SEEK_SET);
    br = read(fuse->backing, &val, sizeof(uint32_t));
    assert(br == sizeof(uint32_t));

    return val;
}

static const struct picoxcell_fuse_range *
find_range(const struct picoxcell_fuse *fuse, int fuse_idx)
{
	int i;

	for (i = 0; fuse->map->ranges[i].name; ++i)
		if (fuse_idx >= fuse->map->ranges[i].start &&
		    fuse_idx <= fuse->map->ranges[i].end)
			return &fuse->map->ranges[i];

	return NULL;
}

static inline bool fuse_is_blown(const struct picoxcell_fuse *fuse,
                                 int idx)
{
    return !!(fuse->fuses[idx / 32] & (1 << (idx & 31)));
}

static inline bool fuse_name_is_blown(const struct picoxcell_fuse *fuse,
                                      const char *name)
{
    const struct picoxcell_fuse_range *range;

    for (range = &fuse->map->ranges[0]; range->name; range++) {
        if (!strcmp(range->name, name)) {
            int i;

            for (i = range->start; i <= range->end; ++i)
                if (!fuse_is_blown(fuse, i))
                    return false;

            return true;
        }
    }

    return false;
}

static uint32_t fuse_read_word(struct picoxcell_fuse *fuse,
                               uint32_t word_addr)
{
    const struct picoxcell_fuse_range *range = find_range(fuse, word_addr * 32);

    if (!range)
        return 0;

    if (range->read_once > 0 && fuse_is_blown(fuse, range->read_once))
        return ~0;

    return fuse->fuses[word_addr];
}

static void fuse_write_word(struct picoxcell_fuse *fuse, uint32_t word_addr,
                            uint32_t val)
{
    ssize_t bw;

    lseek(fuse->backing, word_addr * sizeof(val), SEEK_SET);
    bw = write(fuse->backing, &val, sizeof(val));
    assert(bw == sizeof(val));
}

static uint32_t picoxcell_fuse_readl(void *opaque, target_phys_addr_t addr)
{
    struct picoxcell_fuse *fuse = opaque;

    switch (addr) {
    case 0x200:
        return fuse->fuse_ctrl;
    case 0x204:
        return fuse->fuse_write_bit_addr;
    case 0x000 ... 0x1ff:
        return fuse_read_word(fuse, addr / 4);
    default:
        return 0;
    }
}

static inline bool fuse_can_blow(const struct picoxcell_fuse *fuse,
                                 int idx)
{
    const struct picoxcell_fuse_range *range = find_range(fuse, idx);

    return !((range->last_time_prog >= 0 &&
              fuse_is_blown(fuse, range->last_time_prog)) ||
             fuse_is_blown(fuse, fuse->map->ltp_fuse));
}

static void fuse_prog_done(void *opaque)
{
    struct picoxcell_fuse *fuse = opaque;
    uint32_t bit_addr = fuse->fuse_write_bit_addr;

    if (fuse_can_blow(fuse, bit_addr)) {
        uint32_t v, addr = bit_addr / 32;
        v = fuse_read_word_file(fuse, addr);
        fuse_write_word(fuse, addr, v | (1 << (bit_addr & 31)));
    }

    fuse->fuse_ctrl &= ~FUSE_CTRL_WRITE_BUSY;
}

static void fuse_start_program(struct picoxcell_fuse *fuse)
{
    if ((fuse->fuse_ctrl & FUSE_CTRL_VDDQ_OE) &&
        (fuse->fuse_ctrl & FUSE_CTRL_VDDQ)) {
        fuse->fuse_ctrl |= FUSE_CTRL_WRITE_BUSY;
        /*
         * Emulate a 250us delay for programming completion.  The time it takes in
         * reality can vary dependent on the process...
         */
        ptimer_stop(fuse->ptimer);
        ptimer_set_limit(fuse->ptimer, 250, 1);
        ptimer_run(fuse->ptimer, 0);
    } else {
        fuse_prog_done(fuse);
    }
}

static void picoxcell_fuse_writel(void *opaque, target_phys_addr_t addr,
                               uint32_t val)
{
    struct picoxcell_fuse *fuse = opaque;

    switch (addr) {
    case 0x000 ... 0x1ff:
        fuse->fuses[addr / 4] = val;
        break;
    case 0x204:
        fuse->fuse_write_bit_addr = val & 0xfff;
        break;
    case 0x208:
        if (val == 0x66757365)
            fuse_start_program(fuse);
        break;
    case 0x20c:
        if (val == 0x656e626c)
            fuse->fuse_ctrl |= FUSE_CTRL_VDDQ_OE;
        else
            fuse->fuse_ctrl &= ~FUSE_CTRL_VDDQ_OE;
        break;
    case 0x210:
        if (val == 0x5644451)
            fuse->fuse_ctrl |= FUSE_CTRL_VDDQ;
        else
            fuse->fuse_ctrl &= ~FUSE_CTRL_VDDQ;
        break;
    }
}

static CPUReadMemoryFunc * const picoxcell_fuse_read[] = {
    NULL,
    NULL,
    picoxcell_fuse_readl
};

static CPUWriteMemoryFunc * const picoxcell_fuse_write[] = {
    NULL,
    NULL,
    picoxcell_fuse_writel
};

static void picoxcell_fuse_reset(DeviceState *d)
{
    struct picoxcell_fuse *fuse = FROM_SYSBUS(struct picoxcell_fuse,
                                           sysbus_from_qdev(d));

    switch (fuse->device_id) {
    case 0x8003:
    case 0x8007:
        fuse->map = &pc3x2_fuse_map;
        break;
    case 0x20 ... 0x22:
        fuse->map = &pc3x3_fuse_map;
        break;
    case 0x30 ... 0x3F:
        fuse->map = &pc30xx_fuse_map;
        qemu_set_irq(fuse->otp_io[0], fuse_name_is_blown(fuse, "otp_ropb1"));
        qemu_set_irq(fuse->otp_io[1], fuse_name_is_blown(fuse, "otp_ropb2"));
        qemu_set_irq(fuse->otp_io[2], fuse_name_is_blown(fuse, "otp_ropb3"));
        qemu_set_irq(fuse->otp_io[3], fuse_name_is_blown(fuse, "otp_ropb4"));
        qemu_set_irq(fuse->otp_io[4], fuse_name_is_blown(fuse, "otp_ltp1"));
        qemu_set_irq(fuse->otp_io[5], fuse_name_is_blown(fuse, "otp_ltp2"));
        qemu_set_irq(fuse->otp_io[6], fuse_name_is_blown(fuse, "otp_ltp3"));
        qemu_set_irq(fuse->otp_io[7], fuse_name_is_blown(fuse, "otp_ltp4"));
        qemu_set_irq(fuse->otp_io[8], fuse_name_is_blown(fuse, "global_otp_ltp"));
    }
}

static int picoxcell_fuse_init(SysBusDevice *dev)
{
    struct picoxcell_fuse *t = FROM_SYSBUS(struct picoxcell_fuse, dev);
    int regs, err, n;

    t->backing = open("fuse_pc30xx.bin", O_RDWR | O_CREAT, 0644);
    /* Make sure that all of the fuses exist. */
    err = ftruncate(t->backing, 4 * 1024);
    if (err)
        goto out_close;

    for (n = 0; n < 1024; ++n)
        t->fuses[n] = fuse_read_word_file(t, n);

    regs = cpu_register_io_memory(picoxcell_fuse_read, picoxcell_fuse_write, t,
				  DEVICE_NATIVE_ENDIAN);
    sysbus_init_mmio(dev, 0x8000, regs);

    /*
     * We have 9 I/O's:
     *  - 4 x OTP Read Once Per Boot flags
     *  - 4 x OTP Last Time Program flags
     *  - 1 x OTP Global Last Time Program flag
     */
    for (n = 0; n < 9; ++n)
        sysbus_init_irq(dev, &t->otp_io[n]);

    t->bh = qemu_bh_new(fuse_prog_done, t);
    t->ptimer = ptimer_init(t->bh);
    ptimer_set_freq(t->ptimer, 1000000);

    return 0;

out_close:
    close(t->backing);

    return err;
}

static SysBusDeviceInfo picoxcell_fuse_info = {
    .init = picoxcell_fuse_init,
    .qdev.name = "picoxcell_fuse",
    .qdev.size = sizeof(struct picoxcell_fuse),
    .qdev.reset = picoxcell_fuse_reset,
    .qdev.props = (Property[]) {
        DEFINE_PROP_UINT32("device_id", struct picoxcell_fuse, device_id, 0x8003),
        DEFINE_PROP_END_OF_LIST(),
    }
};

static void picoxcell_fuse_register(void)
{
    sysbus_register_withprop(&picoxcell_fuse_info);
}
device_init(picoxcell_fuse_register);
