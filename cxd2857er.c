//SPX-License-Identifier: GPL-2.0-or-later
/*
 * Sony CXD2857
 * Copyright (c) 2024 otya <otya281@gmail.com>
 * Copyright (c) 2021 Davin zhang <Davin@tbsdtv.com> www.Turbosight.com
 */
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <media/dvb_frontend.h>
#include <linux/mutex.h>

#include "cxd2857.h"
#include "cxd2857_priv.h"

static LIST_HEAD(cxdlist);

struct cxd_base {
	struct list_head cxdlist;
	struct i2c_adapter *i2c;
	struct mutex i2c_lock; //for two adapter at the same i2c bus
	u8 adr; //
	u32 count; //
	struct cxd2857_config config;
};

enum sony_tuner_state_t {
	SONY_TUNER_STATE_NONE,
	SONY_TUNER_STATE_T,
	SONY_TUNER_STATE_S,
};

struct cxd2878_dev {
	struct cxd_base *base;
	bool warm; //start
	struct dvb_frontend fe;
	enum sony_dtv_system_t system;
	enum sony_dtv_bandwidth_t bandwidth;
	u32 symbol_rate;
	enum sony_demod_state_t state;
	enum sony_tuner_state_t tuner_state;
	u8 slvt; //for slvt addr;
	u8 slvx; //addr
	u8 slvr; //addr
	u8 slvm; //addr
	u8 tuner_addr;
	enum sony_demod_chip_id_t chipid;
	enum sony_ascot3_chip_id_t tunerid;
	struct sony_demod_iffreq_config_t iffreqConfig;

	u32 atscNoSignalThresh;
	u32 atscSignalThresh;
	u32 tune_time;
};

static struct sony_freia_terr_adjust_param_t
	g_terr_param_table_cxd2857[SONY_FREIA_TERR_TV_SYSTEM_NUM] = {
		[SONY_FREIA_DTV_ISDBT_6] = { AUTO, 0x07, 0x0d, 0x0d, 0x0d, 0x03,
					     0x03, 0x03, 0x00, BW_6, OFFSET(-9),
					     OFFSET(-5), AUTO, AUTO },
		[SONY_FREIA_DTV_ISDBT_7] = { AUTO, 0x07, 0x0d, 0x0d, 0x0d, 0x03,
					     0x03, 0x03, 0x00, BW_7, OFFSET(-7),
					     OFFSET(-6), AUTO, AUTO },
		[SONY_FREIA_DTV_ISDBT_8] = { AUTO, 0x07, 0x0d, 0x0d, 0x0d, 0x03,
					     0x03, 0x03, 0x00, BW_8, OFFSET(-5),
					     OFFSET(-7), AUTO, AUTO },
	};

static const u8 log2LookUp[] = {
	0,  /* 0 */ 4, /* 0.04439 */
	9,  /* 0.08746 */ 13, /* 0.12928 */
	17, /* 0.16993 */ 21, /* 0.20945 */
	25, /* 0.24793 */ 29, /* 0.28540 */
	32, /* 0.32193 */ 36, /* 0.35755 */
	39, /* 0.39232 */ 43, /* 0.42627 */
	46, /* 0.45943 */ 49, /* 0.49185 */
	52, /* 0.52356 */ 55, /* 0.55249 */
	58, /* 0.58496 */ 61, /* 0.61471 */
	64, /* 0.64386 */ 67, /* 0.67246 */
	70, /* 0.70044 */ 73, /* 0.72792 */
	75, /* 0.75489 */ 78, /* 0.78136 */
	81, /* 0.80736 */ 83, /* 0.83289 */
	86, /* 0.85798 */ 88, /* 0.88264 */
	91, /* 0.90689 */ 93, /* 0.93074 */
	95, /* 0.95420 */ 98 /* 0.97728 */
};

static u32 sony_math_log2(u32 x)
{
	u8 count = 0;
	u8 index = 0;
	u32 xval = x;

	/* Get the MSB position. */
	for (x >>= 1; x > 0; x >>= 1) {
		count++;
	}

	x = count * 100;

	if (count > 0) {
		if (count <= MAX_BIT_PRECISION) {
			/* Mask the bottom bits. */
			index = (u8)(xval << (MAX_BIT_PRECISION - count)) &
				FRAC_BITMASK;
			x += log2LookUp[index];
		} else {
			/* Mask the bits just below the radix. */
			index = (u8)(xval >> (count - MAX_BIT_PRECISION)) &
				FRAC_BITMASK;
			x += log2LookUp[index];
		}
	}

	return (x);
}
static u32 sony_math_log10(u32 x)
{
	/* log10(x) = log2 (x) / log2 (10) */
	/* Note uses: logN (x) = logM (x) / logM (N) */
	return ((100 * sony_math_log2(x) + LOG2_10_100X / 2) / LOG2_10_100X);
}
static s32 sony_Convert2SComplement(u32 value, u32 bitlen)
{
	if ((bitlen == 0) || (bitlen >= 32)) {
		return (s32)value;
	}

	if (value & (u32)(1 << (bitlen - 1))) {
		/* minus value */
		return (s32)(MASKUPPER(32 - bitlen) | value);
	} else {
		/* plus value */
		return (s32)(MASKLOWER(bitlen) & value);
	}
}
/*write multi registers*/
static int cxd2878_wrm(struct cxd2878_dev *dev, u8 addr, u8 reg, u8 *buf,
		       u8 len)
{
	int ret;
	u8 b0[50];
	b0[0] = reg;
	memcpy(&b0[1], buf, len);
	struct i2c_msg msg = {
		.addr = addr,
		.flags = 0,
		.buf = b0,
		.len = len + 1,
	};
	ret = i2c_transfer(dev->base->i2c, &msg, 1);
	if (ret < 0) {
		dev_warn(&dev->base->i2c->dev,
			 "%s: i2c wrm err(%i) @0x%02x (len=%d)\n",
			 KBUILD_MODNAME, ret, reg, len);
		return ret;
	}

	//printk("wrm : addr = 0x%x args=%*ph\n",addr*2,len+1,b0);
	return 0;
}
/*write one register*/
static int cxd2878_wr(struct cxd2878_dev *dev, u8 addr, u8 reg, u8 data)
{
	return cxd2878_wrm(dev, addr, reg, &data, 1);
}
/*read one or more registers*/
static int cxd2878_rdm(struct cxd2878_dev *dev, u8 addr, u8 reg, u8 *buf,
		       u32 len)
{
	int ret;
	struct i2c_msg msg[] = {
		{ .addr = addr, .flags = 0, .buf = &reg, .len = 1 },
		{ .addr = addr, .flags = I2C_M_RD, .buf = buf, .len = len }
	};

	ret = i2c_transfer(dev->base->i2c, msg, 2);
	if (ret < 0) {
		dev_warn(&dev->base->i2c->dev,
			 "%s: i2c rdm err(%i) @0x%02x (len=%d)\n",
			 KBUILD_MODNAME, ret, addr, len);
		return ret;
	}

	//printk("rdm :addr = 0x%x,reg =0x%x data=%*ph\n",addr*2,reg,len,buf);

	return 0;
}
static int cxd2878_SetRegisterBits(struct cxd2878_dev *dev, u8 slaveaddress,
				   u8 registerAddr, u8 data, u8 mask)
{
	int ret;
	u8 rdata = 0x00;

	if (mask == 0)
		return 0;

	if (mask != 0xFF) {
		ret = cxd2878_rdm(dev, slaveaddress, registerAddr, &rdata, 1);
		if (ret)
			return ret;
		data = ((data & mask) | (rdata & (mask ^ 0xFF)));
	}

	//	printk("%s: data = 0x%x",__FUNCTION__,data);
	ret = cxd2878_wr(dev, slaveaddress, registerAddr, data);

	if (ret)
		goto err;

	return 0;
err:
	dev_err(&dev->base->i2c->dev, "set bank and registerbits error.\n");
	return ret;
}
static int cxd2878_SetBankAndRegisterBits(struct cxd2878_dev *dev,
					  u8 slaveAddress, u8 bank,
					  u8 registerAddress, u8 value,
					  u8 bitMask)
{
	int ret;

	ret = cxd2878_wr(dev, slaveAddress, 0x00, bank);
	if (ret)
		goto err;

	ret = cxd2878_SetRegisterBits(dev, slaveAddress, registerAddress, value,
				      bitMask);
	if (ret)
		goto err;

	return 0;
err:
	dev_err(&dev->base->i2c->dev, "set bank and registerbits error.\n");
	return ret;
}

static int cxd2878_i2c_repeater(struct cxd2878_dev *dev, bool enable)
{
	int ret;

	ret = cxd2878_wr(dev, dev->slvx, 0x08, enable ? 1 : 0);
	if (ret)
		goto err;

	msleep(20);

	return 0;

err:
	dev_err(&dev->base->i2c->dev, "%s : %sable thee repeater failed! \n",
		KBUILD_MODNAME, enable ? "en" : "dis");
	return ret;
}

static int cxd2857_read_rssi_isdbt(struct cxd2878_dev *dev, u32 frequency,
				   s32 *rssi)
{
	int ret = 0;
	u8 ifagc, rfagc, tmp;
	s32 ifgain = 0, rfgain = 0, d;
	u8 data[2];
	s32 if_bpf_gc_table[] = { -6, -4, -2, 0,  2,  4,  6,  8,
				  10, 12, 14, 16, 18, 20, 20, 20 };
	s32 if_bpf_gc_x100 = 0;
	s32 agcreg_x140 = 0, maxagcreg_x140 = 0, rfgainmax_100 = 0;

	data[0] = 0xc4;
	data[1] = 0x41; //(x87 x88)
	ret = cxd2878_wrm(dev, dev->tuner_addr, 0x87, data, 2);
	if (ret)
		goto err;

	data[0] = 0x7e;
	data[1] = 0x06;
	ret = cxd2878_wrm(dev, dev->tuner_addr, 0x17, data, 2);
	if (ret)
		goto err;
	msleep(4);

	ret |= cxd2878_rdm(dev, dev->tuner_addr, 0x1a, &tmp, 1);
	if (tmp != 0) {
		ret = -EINVAL;
		goto err;
	}
	ret |= cxd2878_rdm(dev, dev->tuner_addr, 0x19, &tmp, 1);
	d = tmp & 0xf;

	data[0] = 0x05;
	data[1] = 0x01;
	ret |= cxd2878_wrm(dev, dev->tuner_addr, 0x59, data, 2);

	ret |= cxd2878_rdm(dev, dev->tuner_addr, 0x5B, &ifagc, 1);
	if (ret)
		goto err;
	data[0] = 0x03;
	data[1] = 0x01;
	ret |= cxd2878_wrm(dev, dev->tuner_addr, 0x59, data, 2);
	ret |= cxd2878_rdm(dev, dev->tuner_addr, 0x5B, &rfagc, 1);
	if (ret)
		goto err;
	ret |= cxd2878_wr(dev, dev->tuner_addr, 0x59, 0x04);
	ret |= cxd2878_wr(dev, dev->tuner_addr, 0x88, 0x00);
	ret |= cxd2878_wr(dev, dev->tuner_addr, 0x87, 0xC0);
	if (ret)
		goto err;

	agcreg_x140 = ifagc * 140;

	cxd2878_rdm(dev, dev->tuner_addr, 0x69, &tmp, 1);
	if_bpf_gc_x100 = if_bpf_gc_table[tmp & 0xF] * 100;
	if (agcreg_x140 >= 10200)
		ifgain = if_bpf_gc_x100 + 820;
	else if (agcreg_x140 >= 7650)
		ifgain = (if_bpf_gc_x100 + 1520) -
			 (700 * (agcreg_x140 - 7650) + 1275) / 2550;
	else
		ifgain = (if_bpf_gc_x100 + 3860) -
			 (agcreg_x140 * 780 + 1275) / 2550;

	if (ifagc > rfagc)
		maxagcreg_x140 = ifagc * 140;
	else
		maxagcreg_x140 = rfagc * 140;

	if (frequency < 55000)
		rfgainmax_100 = 4690;
	else if (frequency < 65000)
		rfgainmax_100 = 4800;
	else if (frequency < 86000)
		rfgainmax_100 = 4920;
	else if (frequency < 125000)
		rfgainmax_100 = 4960;
	else if (frequency < 142000)
		rfgainmax_100 = 4890;
	else if (frequency < 165000)
		rfgainmax_100 = 4770;
	else if (frequency < 172000)
		rfgainmax_100 = 4610;
	else if (frequency < 200000)
		rfgainmax_100 = 4580;
	else if (frequency < 225000)
		rfgainmax_100 = 4680;
	else if (frequency < 250000)
		rfgainmax_100 = 4770;
	else if (frequency < 320000)
		rfgainmax_100 = 4840;
	else if (frequency < 350000)
		rfgainmax_100 = 4740;
	else if (frequency < 400000)
		rfgainmax_100 = 4750;
	else if (frequency < 464000)
		rfgainmax_100 = 4750;
	else if (frequency < 532000)
		rfgainmax_100 = 4450;
	else if (frequency < 600000)
		rfgainmax_100 = 4530;
	else if (frequency < 664000)
		rfgainmax_100 = 4580;
	else if (frequency < 766000)
		rfgainmax_100 = 4630;
	else if (frequency < 868000)
		rfgainmax_100 = 4630;
	else if (frequency < 900000)
		rfgainmax_100 = 4600;
	else if (frequency < 950000)
		rfgainmax_100 = 4480;
	else
		rfgainmax_100 = 4300;

	if (maxagcreg_x140 < 3825)
		rfgain = rfgainmax_100;
	else if (maxagcreg_x140 < 6375)
		rfgain = rfgainmax_100 -
			 ((maxagcreg_x140 - 3825) * 160 + 12750) / 25500;
	else if (maxagcreg_x140 < 7650)
		rfgain = (rfgainmax_100 - 16) -
			 ((maxagcreg_x140 - 6375) * 1340 + 12750) / 25500;
	else if (maxagcreg_x140 < 8925)
		rfgain = (rfgainmax_100 - 83) -
			 ((maxagcreg_x140 - 7650) * 3440 + 12750) / 25500;
	else if (maxagcreg_x140 < 10200)
		rfgain = (rfgainmax_100 - 258) -
			 ((maxagcreg_x140 - 8925) * 5421 + 12750) / 25500;
	else if (maxagcreg_x140 < 11475)
		rfgain = (rfgainmax_100 - 529) -
			 ((maxagcreg_x140 - 10200) * 7451 + 12750) / 25500;
	else if (maxagcreg_x140 < 15300)
		rfgain = (rfgainmax_100 - 902) -
			 ((maxagcreg_x140 - 11475) * 8253 + 12750) / 25500;
	else if (maxagcreg_x140 < 19125)
		rfgain = (rfgainmax_100 - 2139) -
			 ((maxagcreg_x140 - 15300) * 6979 + 12750) / 25500;
	else if (maxagcreg_x140 < 24225)
		rfgain = (rfgainmax_100 - 3186) -
			 ((maxagcreg_x140 - 19125) * 7468 + 12750) / 25500;
	else if (maxagcreg_x140 < 26775)
		rfgain = (rfgainmax_100 - 4680) -
			 ((maxagcreg_x140 - 24225) * 5674 + 12750) / 25500;
	else if (maxagcreg_x140 < 29325)
		rfgain = (rfgainmax_100 - 5247) -
			 ((maxagcreg_x140 - 26775) * 14592 + 12750) / 25500;
	else if (maxagcreg_x140 < 31875)
		rfgain = (rfgainmax_100 - 6717) -
			 ((maxagcreg_x140 - 29325) * 16676 + 12750) / 25500;
	else
		rfgain = rfgainmax_100 - 8384;

	*rssi = -ifgain - rfgain - (sony_Convert2SComplement(d, 4) * 100);
	return 0;

err:
	dev_err(&dev->base->i2c->dev, "%s : cxd2857_read_rssi_isdbt failed! \n",
		KBUILD_MODNAME);
	return ret;
}

static int cxd2857_read_rflevel_isdbs(struct cxd2878_dev *dev, u32 frequency,
				      s32 *rflevel)
{
	int ret = 0;
	u8 data[2];
	u32 ifagc, q, r;
	s32 agcdb;

	ret |= cxd2878_wr(dev, dev->slvt, 0x00, 0xa0);
	if (ret)
		goto err;
	ret = cxd2878_rdm(dev, dev->slvt, 0x1f, data, 2);
	if (ret)
		goto err;
	ifagc = ((data[0] & 0x1f) << 8) | data[1];
	s32 a = (s32)ifagc * -1400;
	if (a > 0) {
		q = (u32)a / 403;
		r = (u32)a % 403;
	} else {
		q = (u32)(-a) / 403;
		r = (u32)(-a) % 403;
	}
	if (r >= 403 / 2)
		agcdb = (s32)q + 1;
	else
		agcdb = (s32)q;
	if (a < 0)
		agcdb = -agcdb;
	*rflevel = (agcdb + 9700) * -10;
	return 0;

err:
	dev_err(&dev->base->i2c->dev,
		"%s : cxd2857_read_rflevel_isdbs failed! \n", KBUILD_MODNAME);
	return ret;
}

struct cn_data_entry {
	uint32_t value;
	int32_t cnr;
};

static const struct cn_data_entry isdbs3_cn_table[] = {
	{ 0x10da5, -4000 }, { 0x107dd, -3900 }, { 0x1023c, -3800 },
	{ 0x0fcc1, -3700 }, { 0x0f76b, -3600 }, { 0x0f237, -3500 },
	{ 0x0ed25, -3400 }, { 0x0e834, -3300 }, { 0x0e361, -3200 },
	{ 0x0deab, -3100 }, { 0x0da13, -3000 }, { 0x0d596, -2900 },
	{ 0x0d133, -2800 }, { 0x0cceb, -2700 }, { 0x0c8bb, -2600 },
	{ 0x0c4a4, -2500 }, { 0x0c0a3, -2400 }, { 0x0bcba, -2300 },
	{ 0x0b8e6, -2200 }, { 0x0b527, -2100 }, { 0x0b17d, -2000 },
	{ 0x0ade6, -1900 }, { 0x0aa63, -1800 }, { 0x0a6f3, -1700 },
	{ 0x0a395, -1600 }, { 0x0a049, -1500 }, { 0x09d0e, -1400 },
	{ 0x099e4, -1300 }, { 0x096cb, -1200 }, { 0x093c1, -1100 },
	{ 0x090c7, -1000 }, { 0x08ddc, -900 },	{ 0x08b00, -800 },
	{ 0x08833, -700 },  { 0x08573, -600 },	{ 0x082c2, -500 },
	{ 0x0801e, -400 },  { 0x07d87, -300 },	{ 0x07afd, -200 },
	{ 0x0787f, -100 },  { 0x0760c, 0 },	{ 0x073a9, 100 },
	{ 0x0714f, 200 },   { 0x06f02, 300 },	{ 0x06cbf, 400 },
	{ 0x06a88, 500 },   { 0x0685b, 600 },	{ 0x06639, 700 },
	{ 0x06421, 800 },   { 0x06214, 900 },	{ 0x06010, 1000 },
	{ 0x05e16, 1100 },  { 0x05c26, 1200 },	{ 0x05a40, 1300 },
	{ 0x05862, 1400 },  { 0x0568e, 1500 },	{ 0x054c2, 1600 },
	{ 0x05300, 1700 },  { 0x05146, 1800 },	{ 0x04f94, 1900 },
	{ 0x04deb, 2000 },  { 0x04c49, 2100 },	{ 0x04ab0, 2200 },
	{ 0x0491f, 2300 },  { 0x04795, 2400 },	{ 0x04613, 2500 },
	{ 0x04498, 2600 },  { 0x04325, 2700 },	{ 0x041b8, 2800 },
	{ 0x04053, 2900 },  { 0x03ef5, 3000 },	{ 0x03d9e, 3100 },
	{ 0x03c4d, 3200 },  { 0x03b02, 3300 },	{ 0x039bf, 3400 },
	{ 0x03881, 3500 },  { 0x0374a, 3600 },	{ 0x03619, 3700 },
	{ 0x034ee, 3800 },  { 0x033c9, 3900 },	{ 0x032aa, 4000 },
	{ 0x03191, 4100 },  { 0x0307d, 4200 },	{ 0x02f6f, 4300 },
	{ 0x02e66, 4400 },  { 0x02d62, 4500 },	{ 0x02c64, 4600 },
	{ 0x02b6b, 4700 },  { 0x02a77, 4800 },	{ 0x02988, 4900 },
	{ 0x0289e, 5000 },  { 0x027b9, 5100 },	{ 0x026d9, 5200 },
	{ 0x025fd, 5300 },  { 0x02526, 5400 },	{ 0x02453, 5500 },
	{ 0x02385, 5600 },  { 0x022bb, 5700 },	{ 0x021f5, 5800 },
	{ 0x02134, 5900 },  { 0x02076, 6000 },	{ 0x01fbd, 6100 },
	{ 0x01f08, 6200 },  { 0x01e56, 6300 },	{ 0x01da9, 6400 },
	{ 0x01cff, 6500 },  { 0x01c58, 6600 },	{ 0x01bb5, 6700 },
	{ 0x01b16, 6800 },  { 0x01a7b, 6900 },	{ 0x019e2, 7000 },
	{ 0x0194d, 7100 },  { 0x018bc, 7200 },	{ 0x0182d, 7300 },
	{ 0x017a2, 7400 },  { 0x0171a, 7500 },	{ 0x01695, 7600 },
	{ 0x01612, 7700 },  { 0x01593, 7800 },	{ 0x01517, 7900 },
	{ 0x0149d, 8000 },  { 0x01426, 8100 },	{ 0x013b2, 8200 },
	{ 0x01340, 8300 },  { 0x012d1, 8400 },	{ 0x01264, 8500 },
	{ 0x011fa, 8600 },  { 0x01193, 8700 },	{ 0x0112d, 8800 },
	{ 0x010ca, 8900 },  { 0x01069, 9000 },	{ 0x0100b, 9100 },
	{ 0x00fae, 9200 },  { 0x00f54, 9300 },	{ 0x00efb, 9400 },
	{ 0x00ea5, 9500 },  { 0x00e51, 9600 },	{ 0x00dff, 9700 },
	{ 0x00dae, 9800 },  { 0x00d60, 9900 },	{ 0x00d13, 10000 },
	{ 0x00cc8, 10100 }, { 0x00c7e, 10200 }, { 0x00c37, 10300 },
	{ 0x00bf1, 10400 }, { 0x00bac, 10500 }, { 0x00b6a, 10600 },
	{ 0x00b28, 10700 }, { 0x00ae8, 10800 }, { 0x00aaa, 10900 },
	{ 0x00a6d, 11000 }, { 0x00a32, 11100 }, { 0x009f8, 11200 },
	{ 0x009bf, 11300 }, { 0x00987, 11400 }, { 0x00951, 11500 },
	{ 0x0091c, 11600 }, { 0x008e8, 11700 }, { 0x008b6, 11800 },
	{ 0x00884, 11900 }, { 0x00854, 12000 }, { 0x00825, 12100 },
	{ 0x007f7, 12200 }, { 0x007ca, 12300 }, { 0x0079e, 12400 },
	{ 0x00773, 12500 }, { 0x00749, 12600 }, { 0x00720, 12700 },
	{ 0x006f8, 12800 }, { 0x006d0, 12900 }, { 0x006aa, 13000 },
	{ 0x00685, 13100 }, { 0x00660, 13200 }, { 0x0063c, 13300 },
	{ 0x00619, 13400 }, { 0x005f7, 13500 }, { 0x005d6, 13600 },
	{ 0x005b5, 13700 }, { 0x00595, 13800 }, { 0x00576, 13900 },
	{ 0x00558, 14000 }, { 0x0053a, 14100 }, { 0x0051d, 14200 },
	{ 0x00501, 14300 }, { 0x004e5, 14400 }, { 0x004c9, 14500 },
	{ 0x004af, 14600 }, { 0x00495, 14700 }, { 0x0047b, 14800 },
	{ 0x00462, 14900 }, { 0x0044a, 15000 }, { 0x00432, 15100 },
	{ 0x0041b, 15200 }, { 0x00404, 15300 }, { 0x003ee, 15400 },
	{ 0x003d8, 15500 }, { 0x003c3, 15600 }, { 0x003ae, 15700 },
	{ 0x0039a, 15800 }, { 0x00386, 15900 }, { 0x00373, 16000 },
	{ 0x00360, 16100 }, { 0x0034d, 16200 }, { 0x0033b, 16300 },
	{ 0x00329, 16400 }, { 0x00318, 16500 }, { 0x00307, 16600 },
	{ 0x002f6, 16700 }, { 0x002e6, 16800 }, { 0x002d6, 16900 },
	{ 0x002c7, 17000 }, { 0x002b7, 17100 }, { 0x002a9, 17200 },
	{ 0x0029a, 17300 }, { 0x0028c, 17400 }, { 0x0027e, 17500 },
	{ 0x00270, 17600 }, { 0x00263, 17700 }, { 0x00256, 17800 },
	{ 0x00249, 17900 }, { 0x0023d, 18000 }, { 0x00231, 18100 },
	{ 0x00225, 18200 }, { 0x00219, 18300 }, { 0x0020e, 18400 },
	{ 0x00203, 18500 }, { 0x001f8, 18600 }, { 0x001ed, 18700 },
	{ 0x001e3, 18800 }, { 0x001d9, 18900 }, { 0x001cf, 19000 },
	{ 0x001c5, 19100 }, { 0x001bc, 19200 }, { 0x001b2, 19300 },
	{ 0x001a9, 19400 }, { 0x001a0, 19500 }, { 0x00198, 19600 },
	{ 0x0018f, 19700 }, { 0x00187, 19800 }, { 0x0017f, 19900 },
	{ 0x00177, 20000 }, { 0x0016f, 20100 }, { 0x00168, 20200 },
	{ 0x00160, 20300 }, { 0x00159, 20400 }, { 0x00152, 20500 },
	{ 0x0014b, 20600 }, { 0x00144, 20700 }, { 0x0013e, 20800 },
	{ 0x00137, 20900 }, { 0x00131, 21000 }, { 0x0012b, 21100 },
	{ 0x00125, 21200 }, { 0x0011f, 21300 }, { 0x00119, 21400 },
	{ 0x00113, 21500 }, { 0x0010e, 21600 }, { 0x00108, 21700 },
	{ 0x00103, 21800 }, { 0x000fe, 21900 }, { 0x000f9, 22000 },
	{ 0x000f4, 22100 }, { 0x000ef, 22200 }, { 0x000eb, 22300 },
	{ 0x000e6, 22400 }, { 0x000e2, 22500 }, { 0x000de, 22600 },
	{ 0x000da, 22700 }, { 0x000d5, 22800 }, { 0x000d1, 22900 },
	{ 0x000cd, 23000 }, { 0x000ca, 23100 }, { 0x000c6, 23200 },
	{ 0x000c2, 23300 }, { 0x000be, 23400 }, { 0x000bb, 23500 },
	{ 0x000b7, 23600 }, { 0x000b4, 23700 }, { 0x000b1, 23800 },
	{ 0x000ae, 23900 }, { 0x000aa, 24000 }, { 0x000a7, 24100 },
	{ 0x000a4, 24200 }, { 0x000a2, 24300 }, { 0x0009f, 24400 },
	{ 0x0009c, 24500 }, { 0x00099, 24600 }, { 0x00097, 24700 },
	{ 0x00094, 24800 }, { 0x00092, 24900 }, { 0x0008f, 25000 },
	{ 0x0008d, 25100 }, { 0x0008b, 25200 }, { 0x00088, 25300 },
	{ 0x00086, 25400 }, { 0x00084, 25500 }, { 0x00082, 25600 },
	{ 0x00080, 25700 }, { 0x0007e, 25800 }, { 0x0007c, 25900 },
	{ 0x0007a, 26000 }, { 0x00078, 26100 }, { 0x00076, 26200 },
	{ 0x00074, 26300 }, { 0x00073, 26400 }, { 0x00071, 26500 },
	{ 0x0006f, 26600 }, { 0x0006d, 26700 }, { 0x0006c, 26800 },
	{ 0x0006a, 26900 }, { 0x00069, 27000 }, { 0x00067, 27100 },
	{ 0x00066, 27200 }, { 0x00064, 27300 }, { 0x00063, 27400 },
	{ 0x00061, 27500 }, { 0x00060, 27600 }, { 0x0005f, 27700 },
	{ 0x0005d, 27800 }, { 0x0005c, 27900 }, { 0x0005b, 28000 },
	{ 0x0005a, 28100 }, { 0x00059, 28200 }, { 0x00057, 28300 },
	{ 0x00056, 28400 }, { 0x00055, 28500 }, { 0x00054, 28600 },
	{ 0x00053, 28700 }, { 0x00052, 28800 }, { 0x00051, 28900 },
	{ 0x00050, 29000 }, { 0x0004f, 29100 }, { 0x0004e, 29200 },
	{ 0x0004d, 29300 }, { 0x0004c, 29400 }, { 0x0004b, 29500 },
	{ 0x0004a, 29600 }, { 0x00049, 29700 }, { 0x00048, 29900 },
	{ 0x00047, 30000 },
};

static const struct cn_data_entry isdbs_cn_table[] = {
	{ 0x05af, 0 },	   { 0x0597, 100 },   { 0x057e, 200 },
	{ 0x0567, 300 },   { 0x0550, 400 },   { 0x0539, 500 },
	{ 0x0522, 600 },   { 0x050c, 700 },   { 0x04f6, 800 },
	{ 0x04e1, 900 },   { 0x04cc, 1000 },  { 0x04b6, 1100 },
	{ 0x04a1, 1200 },  { 0x048c, 1300 },  { 0x0477, 1400 },
	{ 0x0463, 1500 },  { 0x044f, 1600 },  { 0x043c, 1700 },
	{ 0x0428, 1800 },  { 0x0416, 1900 },  { 0x0403, 2000 },
	{ 0x03ef, 2100 },  { 0x03dc, 2200 },  { 0x03c9, 2300 },
	{ 0x03b6, 2400 },  { 0x03a4, 2500 },  { 0x0392, 2600 },
	{ 0x0381, 2700 },  { 0x036f, 2800 },  { 0x035f, 2900 },
	{ 0x034e, 3000 },  { 0x033d, 3100 },  { 0x032d, 3200 },
	{ 0x031d, 3300 },  { 0x030d, 3400 },  { 0x02fd, 3500 },
	{ 0x02ee, 3600 },  { 0x02df, 3700 },  { 0x02d0, 3800 },
	{ 0x02c2, 3900 },  { 0x02b4, 4000 },  { 0x02a6, 4100 },
	{ 0x0299, 4200 },  { 0x028c, 4300 },  { 0x027f, 4400 },
	{ 0x0272, 4500 },  { 0x0265, 4600 },  { 0x0259, 4700 },
	{ 0x024d, 4800 },  { 0x0241, 4900 },  { 0x0236, 5000 },
	{ 0x022b, 5100 },  { 0x0220, 5200 },  { 0x0215, 5300 },
	{ 0x020a, 5400 },  { 0x0200, 5500 },  { 0x01f6, 5600 },
	{ 0x01ec, 5700 },  { 0x01e2, 5800 },  { 0x01d8, 5900 },
	{ 0x01cf, 6000 },  { 0x01c6, 6100 },  { 0x01bc, 6200 },
	{ 0x01b3, 6300 },  { 0x01aa, 6400 },  { 0x01a2, 6500 },
	{ 0x0199, 6600 },  { 0x0191, 6700 },  { 0x0189, 6800 },
	{ 0x0181, 6900 },  { 0x0179, 7000 },  { 0x0171, 7100 },
	{ 0x0169, 7200 },  { 0x0161, 7300 },  { 0x015a, 7400 },
	{ 0x0153, 7500 },  { 0x014b, 7600 },  { 0x0144, 7700 },
	{ 0x013d, 7800 },  { 0x0137, 7900 },  { 0x0130, 8000 },
	{ 0x012a, 8100 },  { 0x0124, 8200 },  { 0x011e, 8300 },
	{ 0x0118, 8400 },  { 0x0112, 8500 },  { 0x010c, 8600 },
	{ 0x0107, 8700 },  { 0x0101, 8800 },  { 0x00fc, 8900 },
	{ 0x00f7, 9000 },  { 0x00f2, 9100 },  { 0x00ec, 9200 },
	{ 0x00e7, 9300 },  { 0x00e2, 9400 },  { 0x00dd, 9500 },
	{ 0x00d8, 9600 },  { 0x00d4, 9700 },  { 0x00cf, 9800 },
	{ 0x00ca, 9900 },  { 0x00c6, 10000 }, { 0x00c2, 10100 },
	{ 0x00be, 10200 }, { 0x00b9, 10300 }, { 0x00b5, 10400 },
	{ 0x00b1, 10500 }, { 0x00ae, 10600 }, { 0x00aa, 10700 },
	{ 0x00a6, 10800 }, { 0x00a3, 10900 }, { 0x009f, 11000 },
	{ 0x009b, 11100 }, { 0x0098, 11200 }, { 0x0095, 11300 },
	{ 0x0091, 11400 }, { 0x008e, 11500 }, { 0x008b, 11600 },
	{ 0x0088, 11700 }, { 0x0085, 11800 }, { 0x0082, 11900 },
	{ 0x007f, 12000 }, { 0x007c, 12100 }, { 0x007a, 12200 },
	{ 0x0077, 12300 }, { 0x0074, 12400 }, { 0x0072, 12500 },
	{ 0x006f, 12600 }, { 0x006d, 12700 }, { 0x006b, 12800 },
	{ 0x0068, 12900 }, { 0x0066, 13000 }, { 0x0064, 13100 },
	{ 0x0061, 13200 }, { 0x005f, 13300 }, { 0x005d, 13400 },
	{ 0x005b, 13500 }, { 0x0059, 13600 }, { 0x0057, 13700 },
	{ 0x0055, 13800 }, { 0x0053, 13900 }, { 0x0051, 14000 },
	{ 0x004f, 14100 }, { 0x004e, 14200 }, { 0x004c, 14300 },
	{ 0x004a, 14400 }, { 0x0049, 14500 }, { 0x0047, 14600 },
	{ 0x0045, 14700 }, { 0x0044, 14800 }, { 0x0042, 14900 },
	{ 0x0041, 15000 }, { 0x003f, 15100 }, { 0x003e, 15200 },
	{ 0x003c, 15300 }, { 0x003b, 15400 }, { 0x003a, 15500 },
	{ 0x0038, 15600 }, { 0x0037, 15700 }, { 0x0036, 15800 },
	{ 0x0034, 15900 }, { 0x0033, 16000 }, { 0x0032, 16100 },
	{ 0x0031, 16200 }, { 0x0030, 16300 }, { 0x002f, 16400 },
	{ 0x002e, 16500 }, { 0x002d, 16600 }, { 0x002c, 16700 },
	{ 0x002b, 16800 }, { 0x002a, 16900 }, { 0x0029, 17000 },
	{ 0x0028, 17100 }, { 0x0027, 17200 }, { 0x0026, 17300 },
	{ 0x0025, 17400 }, { 0x0024, 17500 }, { 0x0023, 17600 },
	{ 0x0022, 17800 }, { 0x0021, 17900 }, { 0x0020, 18000 },
	{ 0x001f, 18200 }, { 0x001e, 18300 }, { 0x001d, 18500 },
	{ 0x001c, 18700 }, { 0x001b, 18900 }, { 0x001a, 19000 },
	{ 0x0019, 19200 }, { 0x0018, 19300 }, { 0x0017, 19500 },
	{ 0x0016, 19700 }, { 0x0015, 19900 }, { 0x0014, 20000 },
};

static int cxd2857_read_cnr_isdbs3(struct cxd2878_dev *dev, s64 *cnr)
{
	int ret;
	u8 data[4];
	u32 value;
	size_t i;

	ret = cxd2878_wr(dev, dev->slvt, 0x00, 0xd0);
	if (ret)
		goto err;
	ret = cxd2878_rdm(dev, dev->slvt, 0xf3, data, 4);
	if (ret)
		goto err;
	if (!(data[0] & 1)) {
		return -EINVAL;
	}
	value = ((data[1] << 16) | (data[2] << 8) | data[3]) & 0x1ffff;
	for (i = 1; i < sizeof(isdbs3_cn_table) / sizeof(isdbs3_cn_table[0]);
	     i++) {
		if (isdbs3_cn_table[i].value < value) {
			break;
		}
	}
	*cnr = isdbs3_cn_table[i - 1].cnr;
	return 0;

err:
	dev_err(&dev->base->i2c->dev, "%s : cxd2857_read_cnr_isdbs3 failed! \n",
		KBUILD_MODNAME);
	return ret;
}

static int cxd2857_read_cnr_isdbs(struct cxd2878_dev *dev, s64 *cnr)
{
	int ret;
	u8 data[4];
	u32 value;
	size_t i;

	ret = cxd2878_wr(dev, dev->slvt, 0x00, 0xa1);
	if (ret)
		goto err;
	ret = cxd2878_rdm(dev, dev->slvt, 0x10, data, 3);
	if (ret)
		goto err;
	if (!(data[0] & 1)) {
		return -EINVAL;
	}
	value = ((data[1] << 8) | data[2]) & 0x1fff;
	for (i = 1; i < sizeof(isdbs_cn_table) / sizeof(isdbs_cn_table[0]);
	     i++) {
		if (isdbs_cn_table[i].value < value) {
			break;
		}
	}
	*cnr = isdbs_cn_table[i - 1].cnr;
	return 0;

err:
	dev_err(&dev->base->i2c->dev, "%s : cxd2857_read_cnr_isdbs failed! \n",
		KBUILD_MODNAME);
	return ret;
}

static int cxd2857_tune(struct cxd2878_dev *dev, u32 frequencykHz)
{
	int ret;
	enum sony_freia_tv_system_t tvSystem;
	enum sony_tuner_state_t tuner_state = SONY_TUNER_STATE_NONE;

	switch (dev->system) {
	case SONY_DTV_SYSTEM_ISDBT:
		switch (dev->bandwidth) {
		case SONY_DTV_BW_6_MHZ:
			tvSystem = SONY_FREIA_DTV_ISDBT_6;
			break;
		case SONY_DTV_BW_7_MHZ:
			tvSystem = SONY_FREIA_DTV_ISDBT_7;
			break;
		case SONY_DTV_BW_8_MHZ:
			tvSystem = SONY_FREIA_DTV_ISDBT_8;
			break;
		default:
			tvSystem = SONY_FREIA_DTV_ISDBT_6;
			break;
		}
		tuner_state = SONY_TUNER_STATE_T;
		frequencykHz /= 1000;
		break;

	case SONY_DTV_SYSTEM_ISDBS:
	case SONY_DTV_SYSTEM_ISDBS3:
		tvSystem = SONY_FREIA_STV_ISDBS3;
		tuner_state = SONY_TUNER_STATE_S;
		break;
	default:
	case SONY_DTV_SYSTEM_UNKNOWN:
		goto err;
	}

	if (dev->tuner_state == SONY_TUNER_STATE_T &&
	    tuner_state == SONY_TUNER_STATE_S) {
		u8 data[] = { 0x15, 0x00, 0x00 };
		cxd2878_wr(dev, dev->tuner_addr, 0x74, 0x02);
		cxd2878_SetRegisterBits(dev, dev->tuner_addr, 0x67, 0x00, 0xfe);
		cxd2878_wrm(dev, dev->tuner_addr, 0x5e, data, 3);
		cxd2878_wr(dev, dev->tuner_addr, 0x88, 0x00);
		cxd2878_wr(dev, dev->tuner_addr, 0x87, 0xc0);
	}
	dev->tuner_state = tuner_state;
	if (tuner_state == SONY_TUNER_STATE_S) {
		ret = cxd2878_wr(dev, dev->tuner_addr, 0x15, 0x12);
		if (ret)
			goto err;
		u8 data[] = { 0x00, 0x00 };
		cxd2878_wrm(dev, dev->tuner_addr, 0x6a, data, 2);
		u8 if_out_sel[] = { 0x12, 0xf9, 0x0f, 0x25, 0x44 };
		cxd2878_wrm(dev, dev->tuner_addr, 0x74, if_out_sel, 5);
		cxd2878_wr(dev, dev->tuner_addr, 0x75, 0xf9);
		cxd2878_wr(dev, dev->tuner_addr, 0x40, 0x07);
		cxd2878_wr(dev, dev->tuner_addr, 0x41, 0x07);
		cxd2878_wr(dev, dev->tuner_addr, 0x45, 0x03);
		cxd2878_wr(dev, dev->tuner_addr, 0x48, 0x07);
		u8 data0x04[8] = { 0xc4, 0x40 };
		data0x04[2] = dev->base->config.tuner_xtal ==
					      SONY_ASCOT3_XTAL_24000KHz ?
				      0x03 :
				      0x02;
		if (tvSystem == SONY_FREIA_STV_ISDBS3) {
			data0x04[3] = 0x00;
			data0x04[4] = 0xb4;
			data0x04[5] = 0x78;
			data0x04[6] = 0x08;
			data0x04[7] = 0x30;
		} else {
			data0x04[3] = 0x80;
			data0x04[4] = 0x70;
			data0x04[5] = 0x1e;
			data0x04[6] = 0x02;
			data0x04[7] = 0x24;
		}
		cxd2878_wrm(dev, dev->tuner_addr, 0x04, data0x04, 8);
		u32 f = (frequencykHz + 2) / 4;
		if (2150000 < frequencykHz &&
		    tvSystem == SONY_FREIA_STV_ISDBS3) {
			// left
			cxd2878_wr(dev, dev->tuner_addr, 0x45, 0x02);
			cxd2878_wr(dev, dev->tuner_addr, 0x01, 0x03);
			u8 data0x0c[10] = { 0xfc, 0x32, 0x9e, 0x16, 0x00,
					    0x00, 0x00, 0xff, 0x00, 0x01 };
			data0x0c[4] = (u8)(f & 0xFF);
			data0x0c[5] = (u8)((f >> 8) & 0xFF);
			data0x0c[6] = (u8)((f >> 16) & 0xF);
			cxd2878_wrm(dev, dev->tuner_addr, 0x0c, data0x0c, 10);
		} else {
			cxd2878_wr(dev, dev->tuner_addr, 0x43, 0x04);
			cxd2878_wr(dev, dev->tuner_addr, 0x01, 0x01);
			u8 data0x0c[10] = { 0xfe, 0x22, 0x9e, 0x16, 0x00,
					    0x00, 0x00, 0xff, 0x00, 0x01 };
			data0x0c[4] = (u8)(f & 0xFF);
			data0x0c[5] = (u8)((f >> 8) & 0xFF);
			data0x0c[6] = (u8)((f >> 16) & 0xF);
			cxd2878_wrm(dev, dev->tuner_addr, 0x0c, data0x0c, 10);
		}
		msleep(10);
		cxd2878_wr(dev, dev->tuner_addr, 0x05, 0x00);
		cxd2878_wr(dev, dev->tuner_addr, 0x04, 0xc0);
	} else {
		ret = cxd2878_wr(dev, dev->tuner_addr, 0x01, 0x00);
		if (ret)
			goto err;

		u8 if_out_sel[] = { 0x12, 0xf9, 0x0f, 0x05, 0x44 };
		cxd2878_wrm(dev, dev->tuner_addr, 0x74, if_out_sel, 5);

		u8 cdata[] = { 0xc4, 0x40 };
		cxd2878_wrm(dev, dev->tuner_addr, 0x87, cdata, 2);

		cxd2878_wr(dev, dev->tuner_addr, 0x79, 0xa1);
		cxd2878_wr(dev, dev->tuner_addr, 0x7d, 0x00);
		cxd2878_wr(dev, dev->tuner_addr, 0x8d, 0x00);
		cxd2878_wr(dev, dev->tuner_addr, 0x8e, 0x08);
		u8 data0x91[] = { 0x0a, 0x0f };
		cxd2878_wrm(dev, dev->tuner_addr, 0x91, data0x91, 2);
		u8 data0x9c[] = { 0x90, 0x00 };
		cxd2878_wrm(dev, dev->tuner_addr, 0x9c, data0x9c, 2);
		u8 data0x5e[] = { 0xee, 0x02, 0x9e, 0x67, 0x00,
				  0x38, 0x1e, 0x02, 0x24 };
		data0x5e[4] = dev->base->config.tuner_xtal ==
					      SONY_ASCOT3_XTAL_24000KHz ?
				      0x03 :
				      0x02;
		cxd2878_wrm(dev, dev->tuner_addr, 0x5e, data0x5e, 9);
		ret = cxd2878_SetRegisterBits(dev, dev->tuner_addr, 0x67, 0x00,
					      0x02);
		if (ret)
			goto err;
		/*0x68~0x78*/
		u8 tmp[17];
		tmp[0] = 0x00;

		if (g_terr_param_table_cxd2857[tvSystem].RF_GAIN == AUTO)
			tmp[1] = 0x80;
		else
			tmp[1] = (u8)((g_terr_param_table_cxd2857[tvSystem]
					       .RF_GAIN
				       << 4) &
				      0x70);

		/* IF_BPF_GC setting */
		tmp[1] |= (u8)(g_terr_param_table_cxd2857[tvSystem].IF_BPF_GC &
			       0x0F);
		/* Setting for internal RFAGC (0x6A, 0x6B, 0x6C) */
		tmp[2] = 0x00; /* Normal operation */
		if (frequencykHz <= 172000) {
			tmp[3] = (u8)(g_terr_param_table_cxd2857[tvSystem]
					      .RFOVLD_DET_LV1_VL &
				      0x0F);
			tmp[4] = (u8)(g_terr_param_table_cxd2857[tvSystem]
					      .IFOVLD_DET_LV_VL &
				      0x07);
		} else if (frequencykHz <= 464000) {
			tmp[3] = (u8)(g_terr_param_table_cxd2857[tvSystem]
					      .RFOVLD_DET_LV1_VH &
				      0x0F);
			tmp[4] = (u8)(g_terr_param_table_cxd2857[tvSystem]
					      .IFOVLD_DET_LV_VH &
				      0x07);
		} else {
			tmp[3] = (u8)(g_terr_param_table_cxd2857[tvSystem]
					      .RFOVLD_DET_LV1_U &
				      0x0F);
			tmp[4] = (u8)(g_terr_param_table_cxd2857[tvSystem]
					      .IFOVLD_DET_LV_U &
				      0x07);
		}
		tmp[4] |= 0x30;
		/* Setting for IF frequency and bandwidth */
		/* IF filter center frequency offset (IF_BPF_F0) (0x6D) */
		tmp[5] = (u8)((g_terr_param_table_cxd2857[tvSystem].IF_BPF_F0
			       << 4) &
			      0x30);
		/* IF filter band width (BW) (0x6D) */
		tmp[5] |= (u8)(g_terr_param_table_cxd2857[tvSystem].BW & 0x03);
		/* IF frequency offset value (FIF_OFFSET) (0x6E) */
		tmp[6] = (u8)(g_terr_param_table_cxd2857[tvSystem].FIF_OFFSET &
			      0x1F);
		/* IF band width offset value (BW_OFFSET) (0x6F) */
		tmp[7] = (u8)(g_terr_param_table_cxd2857[tvSystem].BW_OFFSET &
			      0x1F);
		/* RF tuning frequency setting (0x70, 0x71, 0x72) */
		tmp[8] = (u8)(frequencykHz & 0xFF); /* FRF_L */
		tmp[9] = (u8)((frequencykHz >> 8) & 0xFF); /* FRF_M */
		tmp[10] = (u8)((frequencykHz >> 16) &
			       0x1F); /* FRF_H (bit[4:0]) */

		tmp[11] = 0xFF;
		tmp[12] = 0x11;
		ret = cxd2878_wrm(dev, dev->tuner_addr, 0x68, tmp, 13);
		if (ret)
			goto err;

		msleep(50);

		cxd2878_wr(dev, dev->tuner_addr, 0x88, 0x00);
		cxd2878_wr(dev, dev->tuner_addr, 0x87, 0xc0);
	}
	return 0;

err:
	dev_err(&dev->base->i2c->dev, "%s: Tuner cxd2857 tuner error !",
		KBUILD_MODNAME);
	return ret;
}

static int cxd2857_init(struct cxd2878_dev *dev)
{
	int ret = 0;
	u8 tunerid = 0x00, rdata = 0x00;
	u8 data[20];

	ret = cxd2878_rdm(dev, dev->tuner_addr, 0x7F, &tunerid, 1);
	if (ret)
		goto err;
	dev_info(&dev->base->i2c->dev, "tuner id is 0x%x", (tunerid & 0xFC));

	//x_pon
	cxd2878_wr(dev, dev->tuner_addr, 0x01, 0x00);
	cxd2878_wr(dev, dev->tuner_addr, 0x67, 0x00);
	cxd2878_wr(dev, dev->tuner_addr, 0x43, 0x06);
	cxd2878_wr(dev, dev->tuner_addr, 0x45, 0x02);

	u8 dataT[3] = { 0x15, 0x00, 0x00 };
	cxd2878_wrm(dev, dev->tuner_addr, 0x5E, &dataT[0], 3);

	cxd2878_wr(dev, dev->tuner_addr, 0x0c, 0x14);
	cxd2878_wr(dev, dev->tuner_addr, 0x0d, 0x00);

	u8 cdata[2] = { 0x7a, 0x01 };
	cxd2878_wrm(dev, dev->tuner_addr, 0x99, cdata, 2);

	/*regs 0x81~0x94*/
	data[0] = dev->base->config.tuner_xtal == SONY_ASCOT3_XTAL_24000KHz ?
			  0x18 :
			  0x10; //frequency setting for crystal oscillator(0x81)
	/*Driver current setting for crystal oscillator (0x82)*/
	/*Load capacitance setting for crystal oscillator (0x83)*/

	data[1] =
		0x84; // if use extra reference ,0x00 ; if use internal 0x80|(xosc_sel&0x1F)
	/*setting for REFOUT signal output(0x84)*/

	data[1] = 0x84;
	data[2] = 0xa4;
	data[3] = 0x80;
	/* GPIO0, GPIO1 port setting (0x85, 0x86) */
	/* GPIO setting should be done by sony_ascot3_SetGPO after initialization */
	data[4] = 0x01;
	if (dev->base->config.tuner_index > 1) {
		data[1] = 0;
		data[2] = 0;
		data[3] = 0;
		data[4] = 0x00;
	}
	data[5] = 0x10;

	/* Clock enable for internal logic block (0x87) */
	data[6] = 0xc4;

	/* Start CPU boot-up (0x88) */
	data[7] = 0x40;

	/* For burst-write (0x89) */
	data[8] = 0x10;

	/* Setting for internal RFAGC (0x8A, 0x8B, 0x8C) */
	data[9] = 0x00;
	data[10] = 0x45;
	data[11] = 0x75;
	/* Setting for analog block (0x8D) */
	data[12] = 0x07;

	/* Initial setting for internal analog block (0x8E, 0x8F, 0x90, 0x91, 0x92, 0x93, 0x94) */
	data[13] = 0x08;
	data[14] = 0x00;
	data[15] = 0x00;
	data[16] = 0x10;
	data[17] = 0x20;
	data[18] = 0x0A;
	data[19] = 0x00;

	ret = cxd2878_wrm(dev, dev->tuner_addr, 0x81, data, 20);
	if (ret)
		goto err;

	cxd2878_wr(dev, dev->tuner_addr, 0x9b, 0x00);

	msleep(10);
	/* Check CPU_STT (0x1A) */
	cxd2878_rdm(dev, dev->tuner_addr, 0x1A, &rdata, 1);
	if (rdata != 0x00) /* CPU_STT != 0x00 and CPU_ERR != 0x00 */
		goto err;
	cxd2878_wr(dev, dev->tuner_addr, 0x74, 0x12);
	cxd2878_wr(dev, dev->tuner_addr, 0x67, 0x00);
	cxd2878_wr(dev, dev->tuner_addr, 0x88, 0x00);
	cxd2878_wr(dev, dev->tuner_addr, 0x87, 0xc0);
	cxd2878_wr(dev, dev->tuner_addr, 0x80, 0x01);
	cxd2878_wr(dev, dev->tuner_addr, 0x41, 0x07);
	cxd2878_wr(dev, dev->tuner_addr, 0x42, 0x00);
	cxd2878_wr(dev, dev->tuner_addr, 0x46, 0x00);
	u8 tmp2[2] = { 0x02, 0x01 };
	cxd2878_wrm(dev, dev->tuner_addr, 0x7b, tmp2, 2);
	return 0;
err:
	dev_err(&dev->base->i2c->dev, "%s: Tuner freia i2c error !",
		KBUILD_MODNAME);
	return ret;
}
static int cxd2878_setstreamoutput(struct cxd2878_dev *dev, int enable)
{
	int ret;
	u8 data = 0;
	/* slave    Bank    Addr    Bit    default    Name
	 * ---------------------------------------------------
	 * <SLV-T>  00h     A9h     [1:0]  2'b0       OREG_TSTLVALPSEL
	 */

	/* Set SLV-T Bank : 0x00 */
	if (cxd2878_wr(dev, dev->slvt, 0x00, 0x00) != 0) {
		goto err;
	}
	if (cxd2878_wr(dev, dev->slvt, 0xfe, 0x01) != 0) {
		goto err;
	}
	if (cxd2878_rdm(dev, dev->slvt, 0xA9, &data, 1) != 0) {
		goto err;
	}

	if ((data & 0x03) == 0x00) {
		/* TS output */
		/* Set SLV-T Bank : 0x00 */
		if (cxd2878_wr(dev, dev->slvt, 0x00, 0x00) != 0) {
			goto err;
		}
		/* Enable TS output */
		if (cxd2878_wr(dev, dev->slvt, 0xC3, enable ? 0x00 : 0x01) !=
		    0) {
			goto err;
		}
	} else if (data & 0x01) {
		/* TLV output */
		/* Set SLV-T Bank : 0x01 */
		if (cxd2878_wr(dev, dev->slvt, 0x00, 0x01) != 0) {
			goto err;
		}
		/* Enable TLV output */
		if (cxd2878_wr(dev, dev->slvt, 0xC0, enable ? 0x00 : 0x01) !=
		    0) {
			goto err;
		}
	}
	return 0;
err:
	dev_err(&dev->base->i2c->dev, "%s: cxd2878_setstreamoutput error !",
		KBUILD_MODNAME);
	return ret;
}
static int cxd2878_setTSClkModeAndFreq(struct cxd2878_dev *dev)
{
	int ret;
	u8 serialTS;
	u8 tsRateCtrlOff = 0;
	u8 tsInOff = 0;

	struct sony_demod_ts_clk_configuration_t tsClkConfiguration;

	struct sony_demod_ts_clk_configuration_t serialTSClkSettings[2][6] = {
		{
			/* Gated Clock */
			/* OSERCKMODE  OSERDUTYMODE  OTSCKPERIOD  OREG_CKSEL_TSTLVIF                         */
			{ 3, 1, 8, 0 }, /* High Freq, full rate */
			{ 3, 1, 8, 1 }, /* Mid Freq,  full rate */
			{ 3, 1, 8, 2 }, /* Low Freq,  full rate */
			{ 0, 2, 16, 0 }, /* High Freq, half rate */
			{ 0, 2, 16, 1 }, /* Mid Freq,  half rate */
			{ 0, 2, 16, 2 } /* Low Freq,  half rate */
		},
		{
			/* Continuous Clock */
			/* OSERCKMODE  OSERDUTYMODE  OTSCKPERIOD  OREG_CKSEL_TSTLVIF                         */
			{ 1, 1, 8, 0 }, /* High Freq, full rate */
			{ 1, 1, 8, 1 }, /* Mid Freq,  full rate */
			{ 1, 1, 8, 2 }, /* Low Freq,  full rate */
			{ 2, 2, 16, 0 }, /* High Freq, half rate */
			{ 2, 2, 16, 1 }, /* Mid Freq,  half rate */
			{ 2, 2, 16, 2 } /* Low Freq,  half rate */
		}
	};

	struct sony_demod_ts_clk_configuration_t parallelTSClkSetting = {
		/* OSERCKMODE  OSERDUTYMODE  OTSCKPERIOD  OREG_CKSEL_TSTLVIF */
		0, 0, 8, 1
	};
	/* NOTE: For ISDB-S3, OREG_CKSEL_TSTLVIF should be 1 */

	//    struct sony_demod_ts_clk_configuration_t backwardsCompatibleSerialTSClkSetting [2] =
	//   {  /* OSERCKMODE  OSERDUTYMODE  OTSCKPERIOD  OREG_CKSEL_TSTLVIF                         */
	//       {      3,          1,            8,             1        }, /* Gated Clock          */
	//      {      1,          1,            8,             1        }  /* Continuous Clock     */
	//   };

	//    struct sony_demod_ts_clk_configuration_t backwardsCompatibleParallelTSClkSetting =
	//    {  /* OSERCKMODE  OSERDUTYMODE  OTSCKPERIOD  OREG_CKSEL_TSTLVIF */
	//               0,          0,            8,             1
	//   };

	ret = cxd2878_wr(dev, dev->slvt, 0x00, 0x00);
	if (ret)
		goto err;
	ret = cxd2878_rdm(dev, dev->slvt, 0xC4, &serialTS, 1);
	if (ret)
		goto err;
	if ((dev->system == SONY_DTV_SYSTEM_ISDBT) ||
	    (dev->system == SONY_DTV_SYSTEM_ISDBS) ||
	    (dev->system == SONY_DTV_SYSTEM_ISDBS3))
		tsRateCtrlOff = 1;
	if (dev->system == SONY_DTV_SYSTEM_ISDBS3)
		tsInOff = 1;

	cxd2878_SetRegisterBits(dev, dev->slvt, 0xD3, tsRateCtrlOff, 0x01);
	cxd2878_SetRegisterBits(dev, dev->slvt, 0xDE, tsInOff, 0x01);
	cxd2878_SetRegisterBits(dev, dev->slvt, 0xDA, 0x00, 0x01);
	if (serialTS & 0x80) {
		/* Serial TS */
		tsClkConfiguration = serialTSClkSettings[1][1];
	} else {
		/* Parallel TS */
		tsClkConfiguration = parallelTSClkSetting;
		tsClkConfiguration.tsClkPeriod = 0x08;
	}

	if (serialTS & 0x80) {
		/* Serial TS, so set serial TS specific registers */

		/* slave	Bank	Addr	Bit    default	  Name
		 * -----------------------------------------------------
		 * <SLV-T>	00h 	C4h 	[1:0]  2'b01	  OSERCKMODE
		 */
		cxd2878_SetRegisterBits(dev, dev->slvt, 0xC4,
					tsClkConfiguration.serialClkMode, 0x03);

		/* slave	Bank	Addr	Bit    default	  Name
		 * -------------------------------------------------------
		 * <SLV-T>	00h 	D1h 	[1:0]  2'b01	  OSERDUTYMODE
		 */
		cxd2878_SetRegisterBits(dev, dev->slvt, 0xD1,
					tsClkConfiguration.serialDutyMode,
					0x03);
	}

	ret = cxd2878_wr(dev, dev->slvt, 0xD9, tsClkConfiguration.tsClkPeriod);
	if (ret)
		goto err;
	/* Disable TS IF Clock */
	/* slave    Bank    Addr    Bit    default    Name
	 * -------------------------------------------------------
	 * <SLV-T>  00h     32h     [0]    1'b1       OREG_CK_TSTLVIF_EN
	 */
	cxd2878_SetRegisterBits(dev, dev->slvt, 0x32, 0x00, 0x01);

	/* slave    Bank    Addr    Bit    default    Name
	 * -------------------------------------------------------
	 * <SLV-T>  00h     33h     [1:0]  2'b01      OREG_CKSEL_TSTLVIF
	 */
		cxd2878_SetRegisterBits(dev, dev->slvt, 0x33,
				tsClkConfiguration.clkSelTSIf, 0x03);

	/* Enable TS IF Clock */
	/* slave    Bank    Addr    Bit    default    Name
	 * -------------------------------------------------------
	 * <SLV-T>  00h     32h     [0]    1'b1       OREG_CK_TSTLVIF_EN
	 */
	cxd2878_SetRegisterBits(dev, dev->slvt, 0x32, 0x01, 0x01);
	/* Set parity period enable / disable based on backwards compatible TS configuration.
         * These registers are set regardless of broadcasting system for simplicity.
         */
	/* Enable parity period for DVB-T */
	/* Set SLV-T Bank : 0x10 */
	ret = cxd2878_wr(dev, dev->slvt, 0x00, 0x10);
	if (ret)
		goto err;

	/* slave    Bank    Addr    Bit    default    Name
	 * ---------------------------------------------------------------
	 * <SLV-T>  10h     66h     [0]    1'b1       OREG_TSIF_PCK_LENGTH
	 */
	cxd2878_SetRegisterBits(dev, dev->slvt, 0x66, 0x01, 0x01);

	/* Enable parity period for DVB-C (but affect to ISDB-C/J.83B) */
	/* Set SLV-T Bank : 0x40 */
	ret = cxd2878_wr(dev, dev->slvt, 0x00, 0x40);
	if (ret)
		goto err;

	/* slave    Bank    Addr    Bit    default    Name
	 * ---------------------------------------------------------------
	 * <SLV-T>  40h     66h     [0]    1'b1       OREG_TSIF_PCK_LENGTH
	 */
	cxd2878_SetRegisterBits(dev, dev->slvt, 0x66, 0x01, 0x01);

	return 0;
err:
	dev_err(&dev->base->i2c->dev, "%s: set TSClkModeAndFreq error !",
		KBUILD_MODNAME);
	return ret;
}
static int cxd2878_setTLVClkModeAndFreq(struct cxd2878_dev *dev)
{
	int ret;
	u8 serialTLV;

	struct sony_demod_ts_clk_configuration_t tlvClkConfiguration;

	struct sony_demod_ts_clk_configuration_t serialTLVClkSettings[2][6] = {
		{
			/* Gated Clock */
			{ 3, 1, 0, 3 }, /* High Freq, full rate */
			{ 3, 1, 1, 4 }, /* Mid Freq,  full rate */
			{ 3, 1, 2, 5 }, /* Low Freq,  full rate */
			{ 0, 2, 0, 3 }, /* High Freq, half rate */
			{ 0, 2, 1, 4 }, /* Mid Freq,  half rate */
			{ 0, 2, 2, 5 } /* Low Freq,  half rate */
		},
		{
			/* Continuous Clock */
			{ 1, 1, 0, 3 }, /* High Freq, full rate */
			{ 1, 1, 1, 4 }, /* Mid Freq,  full rate */
			{ 1, 1, 2, 5 }, /* Low Freq,  full rate */
			{ 2, 2, 0, 3 }, /* High Freq, half rate */
			{ 2, 2, 1, 4 }, /* Mid Freq,  half rate */
			{ 2, 2, 2, 5 } /* Low Freq,  half rate */
		}
	};

	struct sony_demod_ts_clk_configuration_t parallelTLVClkSetting = { 1, 1,
									   2,
									   4 };
	struct sony_demod_ts_clk_configuration_t
		twoBitParallelTLVClkSetting[3] = {
			{ 0, 0, 0, 3 },
			{ 0, 0, 1, 4 },
			{ 0, 0, 2, 5 },
		};
	ret = cxd2878_wr(dev, dev->slvt, 0x00, 0x01);
	if (ret)
		goto err;
	ret = cxd2878_rdm(dev, dev->slvt, 0xC1, &serialTLV, 1);
	if (ret)
		goto err;
	u8 twoBitParallel;
	ret = cxd2878_rdm(dev, dev->slvt, 0xCF, &twoBitParallel, 1);
	if (ret)
		goto err;
	if ((dev->system == SONY_DTV_SYSTEM_ISDBS3) && (serialTLV & 0x80))
		cxd2878_SetRegisterBits(dev, dev->slvt, 0xE7, 0x00, 0x01);
	else
		cxd2878_SetRegisterBits(dev, dev->slvt, 0xE7, 0x01, 0x01);
	if (serialTLV & 0x80) {
		/* Serial TLV */
		/* Intentional fall through */
		tlvClkConfiguration = serialTLVClkSettings[1][1];
	} else {
		/* Parallel TLV */
		if (twoBitParallel & 0x01) {
			tlvClkConfiguration = twoBitParallelTLVClkSetting[1];
		} else {
			tlvClkConfiguration = parallelTLVClkSetting;
			cxd2878_SetBankAndRegisterBits(dev, dev->slvt, 0x01,
						       0xc1, 0x00, 0x80);
			cxd2878_SetBankAndRegisterBits(dev, dev->slvt, 0x01,
						       0xcf, 0x00, 0x01);
		}
	}

	cxd2878_wr(dev, dev->slvt, 0x00, 0x56);
	ret = cxd2878_SetRegisterBits(dev, dev->slvt, 0x83,
				      tlvClkConfiguration.clkSelTSIf, 0x07);

	if (serialTLV & 0x80) {
		cxd2878_wr(dev, dev->slvt, 0x00, 0x01);
		cxd2878_SetRegisterBits(dev, dev->slvt, 0xC1,
					tlvClkConfiguration.serialClkMode,
					0x03);
		cxd2878_SetRegisterBits(dev, dev->slvt, 0xCC,
					tlvClkConfiguration.serialDutyMode,
					0x03);
	}
	cxd2878_wr(dev, dev->slvt, 0x00, 0x00);
	cxd2878_SetRegisterBits(dev, dev->slvt, 0x32, 0x00, 0x01);
	cxd2878_SetRegisterBits(dev, dev->slvt, 0x33,
				tlvClkConfiguration.tsClkPeriod, 0x03);
	cxd2878_SetRegisterBits(dev, dev->slvt, 0x32, 0x01, 0x01);
	return 0;
err:
	dev_err(&dev->base->i2c->dev, "%s: set TLVClkModeAndFreq error !",
		KBUILD_MODNAME);
	return ret;
}
static int cxd2878_setTSDataPinHiZ(struct cxd2878_dev *dev, u8 enable)
{
	u8 data = 0, tsDataMask = 0;
	int ret = 0;

	/* slave    Bank    Addr    Bit    default    Name
     * ---------------------------------------------------
     * <SLV-T>  00h     A9h     [0]    1'b0       OREG_TSTLVSEL
     *
     * <SLV-T>  00h     C4h     [7]    1'b0       OSERIALEN
     * <SLV-T>  00h     C4h     [3]    1'b1       OSEREXCHGB7
     *
     * <SLV-T>  01h     C1h     [7]    1'b0       OTLV_SERIALEN
     * <SLV-T>  01h     C1h     [3]    1'b1       OTLV_SEREXCHGB7
     * <SLV-T>  01h     CFh     [0]    1'b0       OTLV_PAR2SEL
     * <SLV-T>  01h     EAh     [6:4]  3'b1       OTLV_PAR2_B1SET
     * <SLV-T>  01h     EAh     [2:0]  3'b0       OTLV_PAR2_B0SET
     */
	/* Set SLV-T Bank : 0x00 */
	if (cxd2878_wr(dev, dev->slvt, 0x00, 0x00) != 0) {
		goto err;
	}

	if (cxd2878_rdm(dev, dev->slvt, 0xA9, &data, 1) != 0) {
		goto err;
	}

	if (data & 0x01) {
		/* TLV output */
		/* Set SLV-T Bank : 0x01 */
		if (cxd2878_wr(dev, dev->slvt, 0x00, 0x01) != 0) {
			goto err;
		}

		if (cxd2878_rdm(dev, dev->slvt, 0xC1, &data, 1) != 0) {
			goto err;
		}

		switch (data & 0x88) {
		case 0x80:
			/* Serial TLV, output from TSDATA0 */
			tsDataMask = 0x01;
			break;
		case 0x88:
			/* Serial TLV, output from TSDATA7 */
			tsDataMask = 0x80;
			break;
		case 0x08:
		case 0x00:
		default:
			/* Parallel TLV */
			if (cxd2878_rdm(dev, dev->slvt, 0xCF, &data, 1) != 0) {
				goto err;
			}
			if (data & 0x01) {
				/* TLV 2bit-parallel */
				if (cxd2878_rdm(dev, dev->slvt, 0xEA, &data,
						1) != 0) {
					goto err;
				}
				tsDataMask =
					(0x01 << (data & 0x07)); /* LSB pin */
				tsDataMask |= (0x01 << ((data >> 4) &
							0x07)); /* MSB pin */
			} else {
				/* TLV 8bit-parallel */
				tsDataMask = 0xFF;
			}
			break;
		}
	} else

	{
		/* TS output */
		if (cxd2878_rdm(dev, dev->slvt, 0xC4, &data, 1) != 0) {
			goto err;
		}

		switch (data & 0x88) {
		case 0x80:
			/* Serial TS, output from TSDATA0 */
			tsDataMask = 0x01;
			break;
		case 0x88:
			/* Serial TS, output from TSDATA7 */
			tsDataMask = 0x80;
			break;
		case 0x08:
		case 0x00:
		default:
			/* Parallel TS */
			tsDataMask = 0xFF;
			break;
		}
	}
	/* slave	Bank	Addr	Bit    default	  Name
	 * ---------------------------------------------------
	 * <SLV-T>	 00h	81h    [7:0]	8'hFF	OREG_TSDATA_HIZ
	 */
	/* Set SLV-T Bank : 0x00 */
	if (cxd2878_wr(dev, dev->slvt, 0x00, 0x00) != 0) {
		goto err;
	}

	if (cxd2878_SetRegisterBits(dev, dev->slvt, 0x81,
				    (u8)(enable ? 0xFF : 0x00),
				    tsDataMask) != 0) {
		goto err;
	}

	return 0;
err:
	dev_err(&dev->base->i2c->dev, "%s: cxd2878_setTSDataPinHiZ error !",
		KBUILD_MODNAME);
	return ret;
}
static int cxd2878_sleep(struct dvb_frontend *fe)
{
	struct cxd2878_dev *dev = fe->demodulator_priv;

	if (dev->state == SONY_DEMOD_STATE_ACTIVE) {
		cxd2878_setstreamoutput(dev, 0);
		cxd2878_wr(dev, dev->slvt, 0x00, 0x00);
		cxd2878_SetRegisterBits(dev, dev->slvt, 0x80, 0x1F, 0x1F);
		cxd2878_setTSDataPinHiZ(dev, 1);

		switch (dev->system) {
		case SONY_DTV_SYSTEM_ISDBT: {
			cxd2878_wr(dev, dev->slvt, 0x00, 0x10);
			cxd2878_wr(dev, dev->slvt, 0x69, 0x05);
			cxd2878_wr(dev, dev->slvt, 0x6B, 0x07);
			cxd2878_wr(dev, dev->slvt, 0x9D, 0x14);
			cxd2878_wr(dev, dev->slvt, 0xD3, 0x00);
			cxd2878_wr(dev, dev->slvt, 0xED, 0x01);
			cxd2878_wr(dev, dev->slvt, 0xE2, 0x4E);
			cxd2878_wr(dev, dev->slvt, 0xF2, 0x03);
			cxd2878_wr(dev, dev->slvt, 0xDE, 0x32);
			cxd2878_wr(dev, dev->slvt, 0x00, 0x15);
			cxd2878_wr(dev, dev->slvt, 0xDE, 0x03);
			cxd2878_wr(dev, dev->slvt, 0x00, 0x17);
			u8 data0[2] = { 0x01, 0x02 };
			cxd2878_wrm(dev, dev->slvt, 0x38, data0, 2);
			cxd2878_wr(dev, dev->slvt, 0x00, 0x1E);
			cxd2878_wr(dev, dev->slvt, 0x73, 0x00);
			cxd2878_wr(dev, dev->slvt, 0x00, 0x63);
			cxd2878_wr(dev, dev->slvt, 0x81, 0x01);

			/* Set SLV-X Bank : 0x00 */
			cxd2878_wr(dev, dev->slvx, 0x00, 0x00);
			/* TADC setting */
			cxd2878_wr(dev, dev->slvx, 0x18, 0x01);
			/* Set SLV-T Bank : 0x00 */
			cxd2878_wr(dev, dev->slvt, 0x00, 0x00);
			/* TADC setting */
			cxd2878_wr(dev, dev->slvt, 0x49, 0x33);
			/* TADC setting */
			cxd2878_wr(dev, dev->slvt, 0x4B, 0x21);
			/* Demodulator SW reset */
			cxd2878_wr(dev, dev->slvt, 0xFE, 0x01);
			/* Disable demodulator clock */
			cxd2878_wr(dev, dev->slvt, 0x2C, 0x00);
			/* Set tstlv mode to default */
			cxd2878_wr(dev, dev->slvt, 0xA9, 0x00);
			/* Set demodulator mode to default */
			cxd2878_wr(dev, dev->slvx, 0x17, 0x01);
			break;
		}
		case SONY_DTV_SYSTEM_ISDBS:
			cxd2878_wr(dev, dev->slvx, 0x00, 0x00);
			cxd2878_wr(dev, dev->slvx, 0x18, 0x01);
			cxd2878_wr(dev, dev->slvt, 0x00, 0x00);
			cxd2878_wr(dev, dev->slvt, 0x6a, 0x11);
			cxd2878_wr(dev, dev->slvt, 0x4b, 0x21);
			cxd2878_wr(dev, dev->slvx, 0x28, 0x13);
			cxd2878_wr(dev, dev->slvt, 0xfe, 0x01);
			cxd2878_wr(dev, dev->slvt, 0x2c, 0x00);
			cxd2878_wr(dev, dev->slvt, 0xa9, 0x00);
			cxd2878_wr(dev, dev->slvt, 0x2d, 0x00);
			cxd2878_wr(dev, dev->slvx, 0x17, 0x01);
			cxd2878_wr(dev, dev->slvt, 0x00, 0xa0);
			cxd2878_wr(dev, dev->slvt, 0xd7, 0xa0);
			break;
		case SONY_DTV_SYSTEM_ISDBS3: {
			cxd2878_wr(dev, dev->slvt, 0x00, 0xa3);
			u8 datax43[2] = { 0x0a, 0x0a };
			cxd2878_wrm(dev, dev->slvt, 0x43, datax43, 2);
			cxd2878_wr(dev, dev->slvx, 0x00, 0x00);
			cxd2878_wr(dev, dev->slvx, 0x18, 0x01);
			cxd2878_wr(dev, dev->slvt, 0x00, 0x00);
			cxd2878_wr(dev, dev->slvt, 0x6a, 0x11);
			cxd2878_wr(dev, dev->slvt, 0x4b, 0x21);
			cxd2878_wr(dev, dev->slvx, 0x28, 0x13);
			cxd2878_wr(dev, dev->slvt, 0xfe, 0x01);
			cxd2878_wr(dev, dev->slvt, 0x2c, 0x00);
			cxd2878_wr(dev, dev->slvt, 0xa9, 0x00);
			cxd2878_wr(dev, dev->slvt, 0x2d, 0x00);
			cxd2878_wr(dev, dev->slvx, 0x17, 0x01);
			cxd2878_wr(dev, dev->slvt, 0x00, 0xa0);
			cxd2878_wr(dev, dev->slvt, 0xd7, 0xa0);
			break;
		}
		default:
			break;
		}
	}

	dev->state = SONY_DEMOD_STATE_SLEEP;
	dev->tuner_state = SONY_TUNER_STATE_NONE;
	dev->system = SONY_DTV_SYSTEM_UNKNOWN;

	return 0;
}
static int cxd2878_tuneEnd(struct cxd2878_dev *dev)
{
	cxd2878_wr(dev, dev->slvt, 0x00, 0x00);
	cxd2878_wr(dev, dev->slvt, 0xFE, 0x01);
	cxd2878_setstreamoutput(dev, 1);

	return 0;
}
static int SLtoAIT_BandSetting(struct cxd2878_dev *dev)
{
	int ret = 0;
	u8 data[3], dataxd9[2];

	u8 nominalRate_8[5] = { /* TRCG Nominal Rate [37:0] */
				0x11, 0xB8, 0x00, 0x00, 0x00
	};

	u8 itbCoef_8[14] = {
		/*  COEF01 COEF02 COEF11 COEF12 COEF21 COEF22 COEF31 COEF32 COEF41 COEF42 COEF51 COEF52 COEF61 COEF62 */
		0x2F, 0xBA, 0x28, 0x9B, 0x28, 0x9D, 0x28,
		0xA1, 0x29, 0xA5, 0x2A, 0xAC, 0x29, 0xB5
	};

	u8 nominalRate_7[5] = { /* TRCG Nominal Rate [37:0] */
				0x14, 0x40, 0x00, 0x00, 0x00
	};
	u8 itbCoef_7[14] = {
		/*  COEF01 COEF02 COEF11 COEF12 COEF21 COEF22 COEF31 COEF32 COEF41 COEF42 COEF51 COEF52 COEF61 COEF62 */
		0x30, 0xB1, 0x29, 0x9A, 0x28, 0x9C, 0x28,
		0xA0, 0x29, 0xA2, 0x2B, 0xA6, 0x2B, 0xAD
	};
	u8 nominalRate_6[5] = { /* TRCG Nominal Rate [37:0] */
				0x17, 0xA0, 0x00, 0x00, 0x00
	};
	u8 itbCoef_6[14] = {
		/*  COEF01 COEF02 COEF11 COEF12 COEF21 COEF22 COEF31 COEF32 COEF41 COEF42 COEF51 COEF52 COEF61 COEF62 */
		0x31, 0xA8, 0x29, 0x9B, 0x27, 0x9C, 0x28,
		0x9E, 0x29, 0xA4, 0x29, 0xA2, 0x29, 0xA8
	};
	ret = cxd2878_wr(dev, dev->slvt, 0x00, 0x10);
	if (ret)
		goto err;
	switch (dev->bandwidth) {
	case SONY_DTV_BW_8_MHZ:
		cxd2878_wrm(dev, dev->slvt, 0x9F, nominalRate_8, 5);
		cxd2878_wrm(dev, dev->slvt, 0xA6, itbCoef_8, 14);
		data[0] = (u8)((dev->iffreqConfig.configISDBT_8 >> 16) & 0xFF);
		data[1] = (u8)((dev->iffreqConfig.configISDBT_8 >> 8) & 0xFF);
		data[2] = (u8)(dev->iffreqConfig.configISDBT_8 & 0xFF);
		cxd2878_wrm(dev, dev->slvt, 0xB6, data, 3);
		cxd2878_wr(dev, dev->slvt, 0xD7, 0x00);
		dataxd9[0] = 0x13;
		dataxd9[1] = 0xFC;
		cxd2878_wrm(dev, dev->slvt, 0xD9, dataxd9, 2);
		cxd2878_wr(dev, dev->slvt, 0x00, 0x12);
		cxd2878_wr(dev, dev->slvt, 0x71, 3);
		cxd2878_wr(dev, dev->slvt, 0x00, 0x15);
		cxd2878_wr(dev, dev->slvt, 0xBE, 3);
		break;
	case SONY_DTV_BW_7_MHZ:
		cxd2878_wrm(dev, dev->slvt, 0x9F, nominalRate_7, 5);
		cxd2878_wrm(dev, dev->slvt, 0xA6, itbCoef_7, 14);

		data[0] = (u8)((dev->iffreqConfig.configISDBT_7 >> 16) & 0xFF);
		data[1] = (u8)((dev->iffreqConfig.configISDBT_7 >> 8) & 0xFF);
		data[2] = (u8)(dev->iffreqConfig.configISDBT_7 & 0xFF);
		cxd2878_wrm(dev, dev->slvt, 0xB6, data, 3);
		cxd2878_wr(dev, dev->slvt, 0xD7, 0x00);
		dataxd9[0] = 0x1A;
		dataxd9[1] = 0xFA;
		cxd2878_wrm(dev, dev->slvt, 0xD9, dataxd9, 2);
		cxd2878_wr(dev, dev->slvt, 0x00, 0x12);
		cxd2878_wr(dev, dev->slvt, 0x71, 3);
		cxd2878_wr(dev, dev->slvt, 0x00, 0x15);
		cxd2878_wr(dev, dev->slvt, 0xBE, 2);
		break;
	case SONY_DTV_BW_6_MHZ:
		cxd2878_wrm(dev, dev->slvt, 0x9F, nominalRate_6, 5);
		cxd2878_wrm(dev, dev->slvt, 0xA6, itbCoef_6, 14);
		data[0] = (u8)((dev->iffreqConfig.configISDBT_6 >> 16) & 0xFF);
		data[1] = (u8)((dev->iffreqConfig.configISDBT_6 >> 8) & 0xFF);
		data[2] = (u8)(dev->iffreqConfig.configISDBT_6 & 0xFF);
		cxd2878_wrm(dev, dev->slvt, 0xB6, data, 3);
		cxd2878_wr(dev, dev->slvt, 0xD7, 0x04); // OREG_CHANNEL_WIDTH
		dataxd9[0] = 0x1F;
		dataxd9[1] = 0x79;
		cxd2878_wrm(dev, dev->slvt, 0xD9, dataxd9, 2);
		cxd2878_wr(dev, dev->slvt, 0x00, 0x12);
		cxd2878_wr(dev, dev->slvt, 0x71, 7);
		cxd2878_wr(dev, dev->slvt, 0x00, 0x15);
		cxd2878_wr(dev, dev->slvt, 0xBE, 2);
		break;
	default:
		goto err;
	}

	return 0;
err:
	dev_err(&dev->base->i2c->dev, "%s: SLtoAIT_BandSetting error !",
		KBUILD_MODNAME);
	return ret;
}
static int SLtoAIT(struct cxd2878_dev *dev)
{
	int ret = 0;

	ret = cxd2878_setTSClkModeAndFreq(dev);
	if (ret)
		goto err;
	ret = cxd2878_wr(dev, dev->slvx, 0x00, 0x00);
	if (ret)
		goto err;
	cxd2878_wr(dev, dev->slvx, 0x17, 0x06);
	cxd2878_wr(dev, dev->slvt, 0x00, 0x00);
	cxd2878_wr(dev, dev->slvt, 0xA9, 0x00);
	cxd2878_wr(dev, dev->slvt, 0x2C, 0x01);
	cxd2878_wr(dev, dev->slvt, 0x4B, 0x74);
	cxd2878_wr(dev, dev->slvt, 0x49, 0x00);
	cxd2878_wr(dev, dev->slvx, 0x18, 0x00);

	cxd2878_wr(dev, dev->slvt, 0x00, 0x11); //  SLtoAIT commonsetting
	cxd2878_wr(dev, dev->slvt, 0x6A, 0x50);
	cxd2878_wr(dev, dev->slvt, 0x00, 0x10);
	cxd2878_wr(dev, dev->slvt, 0xA5, 0x01);
	cxd2878_wr(dev, dev->slvt, 0x00, 0x00);

	u8 dataxce[2] = { 0x00, 0x00 };
	cxd2878_wrm(dev, dev->slvt, 0xCE, dataxce, 2);
	cxd2878_wr(dev, dev->slvt, 0x00, 0x10);
	cxd2878_wr(dev, dev->slvt, 0x69, 0x04);
	cxd2878_wr(dev, dev->slvt, 0x6B, 0x03);
	cxd2878_wr(dev, dev->slvt, 0x9D, 0x50);
	cxd2878_wr(dev, dev->slvt, 0xD3, 0x06);
	cxd2878_wr(dev, dev->slvt, 0xED, 0x00);
	cxd2878_wr(dev, dev->slvt, 0xE2, 0xCE);
	cxd2878_wr(dev, dev->slvt, 0xF2, 0x13);
	cxd2878_wr(dev, dev->slvt, 0xDE, 0x2E);
	cxd2878_wr(dev, dev->slvt, 0x00, 0x15);
	cxd2878_wr(dev, dev->slvt, 0xDE, 0x02);
	cxd2878_wr(dev, dev->slvt, 0x00, 0x17);

	u8 datax38[2] = { 0x00, 0x03 };
	cxd2878_wrm(dev, dev->slvt, 0x38, datax38, 2);
	cxd2878_wr(dev, dev->slvt, 0x00, 0x1E);
	cxd2878_wr(dev, dev->slvt, 0x73, 0x68);
	cxd2878_wr(dev, dev->slvt, 0x00, 0x63);
	cxd2878_wr(dev, dev->slvt, 0x81, 0x00);
	cxd2878_wr(dev, dev->slvt, 0x00, 0x11);

	u8 datax33[3] = { 0x00, 0x03, 0x3B };
	cxd2878_wrm(dev, dev->slvt, 0x33, datax33, 3);
	cxd2878_wr(dev, dev->slvt, 0x00, 0x60);

	u8 dataxa8[2] = { 0xB7, 0x1B };
	cxd2878_wrm(dev, dev->slvt, 0xA8, dataxa8, 2); //end

	SLtoAIT_BandSetting(dev);
	cxd2878_wr(dev, dev->slvt, 0x00, 0x00);
	cxd2878_SetRegisterBits(dev, dev->slvt, 0x80, 0x08, 0x1F);

	cxd2878_setTSDataPinHiZ(dev, 0);

	return 0;
err:
	dev_err(&dev->base->i2c->dev, "%s: SLtoAIT error !", KBUILD_MODNAME);
	return ret;
}
static int SLtoAIS(struct cxd2878_dev *dev)
{
	int ret = 0;

	ret = cxd2878_setTSClkModeAndFreq(dev);
	if (ret)
		goto err;
	ret = cxd2878_wr(dev, dev->slvx, 0x00, 0x00);
	if (ret)
		goto err;
	cxd2878_wr(dev, dev->slvx, 0x17, 0x0c);
	cxd2878_wr(dev, dev->slvt, 0x00, 0x00);
	cxd2878_wr(dev, dev->slvt, 0x2d, 0x00);
	cxd2878_wr(dev, dev->slvt, 0xa9, 0x00);
	cxd2878_wr(dev, dev->slvt, 0x2c, 0x01);
	cxd2878_wr(dev, dev->slvx, 0x28, 0x31);
	cxd2878_wr(dev, dev->slvt, 0x4b, 0x31);
	cxd2878_wr(dev, dev->slvt, 0x6a, 0x00);
	cxd2878_wr(dev, dev->slvx, 0x18, 0x00);
	cxd2878_wr(dev, dev->slvt, 0x00, 0x00);
	cxd2878_wr(dev, dev->slvt, 0x20, 0x01);

	u8 dataxce[2] = { 0x00, 0x00 };
	cxd2878_wrm(dev, dev->slvt, 0xCE, dataxce, 2);
	cxd2878_wr(dev, dev->slvt, 0x00, 0xae);
	u8 datax20[3] = { 0x07, 0x37, 0x0a };
	cxd2878_wrm(dev, dev->slvt, 0x20, datax20, 3);
	cxd2878_wr(dev, dev->slvt, 0x00, 0xa0);
	cxd2878_wr(dev, dev->slvt, 0xd7, 0x00);
	cxd2878_wr(dev, dev->slvt, 0x00, 0x00);
	cxd2878_SetRegisterBits(dev, dev->slvt, 0x80, 0x10, 0x1F);
	cxd2878_setTSDataPinHiZ(dev, 0);

	return 0;
err:
	dev_err(&dev->base->i2c->dev, "%s: SLtoAIS error !", KBUILD_MODNAME);
	return ret;
}
static int SLtoAIS3(struct cxd2878_dev *dev)
{
	int ret = 0;
	u8 tlv_output = 1;

	if (tlv_output)
		ret = cxd2878_setTLVClkModeAndFreq(dev);
	else
		ret = cxd2878_setTSClkModeAndFreq(dev);
	if (ret)
		goto err;
	ret = cxd2878_wr(dev, dev->slvx, 0x00, 0x00);
	if (ret)
		goto err;
	cxd2878_wr(dev, dev->slvx, 0x17, 0x0d);
	cxd2878_wr(dev, dev->slvt, 0x00, 0x00);
	cxd2878_wr(dev, dev->slvt, 0x2d, 0x00);
	cxd2878_wr(dev, dev->slvt, 0xa9, !!tlv_output);
	cxd2878_wr(dev, dev->slvt, 0x2c, 0x01);
	cxd2878_wr(dev, dev->slvx, 0x28, 0x31);
	cxd2878_wr(dev, dev->slvt, 0x4b, 0x31);
	cxd2878_wr(dev, dev->slvt, 0x6a, 0x00);
	cxd2878_wr(dev, dev->slvx, 0x18, 0x00);
	cxd2878_wr(dev, dev->slvt, 0x00, 0x00);
	cxd2878_wr(dev, dev->slvt, 0x20, 0x01);
	cxd2878_wr(dev, dev->slvt, 0x00, 0xa3);

	u8 datax43[2] = { 0xb, 0xb };
	cxd2878_wrm(dev, dev->slvt, 0x43, datax43, 2);
	cxd2878_wr(dev, dev->slvt, 0x00, 0xae);
	u8 datax46[2] = { 0x04, 0x91 };
	cxd2878_wrm(dev, dev->slvt, 0x46, datax46, 2);
	cxd2878_wr(dev, dev->slvt, 0x00, 0xb6);
	cxd2878_wr(dev, dev->slvt, 0x74, 0x59);
	cxd2878_wr(dev, dev->slvt, 0x00, 0xd5);
	u8 datax61[5] = { 0x48, 0xfe, 0x4e, 0x6e, 0xfe };
	cxd2878_wrm(dev, dev->slvt, 0x61, datax61, 5);
	u8 datax67[2] = { 0x4e, 0x6e };
	cxd2878_wrm(dev, dev->slvt, 0x67, datax67, 2);
	u8 datax90[3] = { 0x10, 0x03, 0x10 };
	cxd2878_wrm(dev, dev->slvt, 0x90, datax90, 3);
	cxd2878_wr(dev, dev->slvt, 0x00, 0xd6);
	u8 datax18[2] = { 0x33, 0xeb };
	cxd2878_wrm(dev, dev->slvt, 0x18, datax18, 2);
	cxd2878_wr(dev, dev->slvt, 0x1c, 0x59);
	cxd2878_wr(dev, dev->slvt, 0x1e, 0x6e);
	u8 datax21[4] = { 0x4e, 0x80, 0x80, 0x10 };
	cxd2878_wrm(dev, dev->slvt, 0x21, datax21, 4);
	u8 datax51[3] = { 0xb3, 0xb3, 0xc3 };
	cxd2878_wrm(dev, dev->slvt, 0x51, datax51, 3);
	cxd2878_wr(dev, dev->slvt, 0x00, 0xda);
	cxd2878_wr(dev, dev->slvt, 0xbc, 0x02);
	u8 dataxd6[2] = { 0x60, 0x70 };
	cxd2878_wrm(dev, dev->slvt, 0xd6, dataxd6, 2);
	cxd2878_wr(dev, dev->slvt, 0x00, 0xae);
	u8 datax20[3] = { 0x08, 0x70, 0x64 };
	cxd2878_wrm(dev, dev->slvt, 0x20, datax20, 3);
	cxd2878_wr(dev, dev->slvt, 0x00, 0xda);
	cxd2878_wr(dev, dev->slvt, 0xcb, 0x01);
	cxd2878_wr(dev, dev->slvt, 0xc1, 0x01);
	cxd2878_wr(dev, dev->slvt, 0x00, 0xa0);
	cxd2878_wr(dev, dev->slvt, 0xd7, 0x00);
	cxd2878_wr(dev, dev->slvt, 0x00, 0x00);
	cxd2878_SetRegisterBits(dev, dev->slvt, 0x80, 0x10, 0x1F);
	cxd2878_setTSDataPinHiZ(dev, 0);

	return 0;
err:
	dev_err(&dev->base->i2c->dev, "%s: SLtoAIS3 error !", KBUILD_MODNAME);
	return ret;
}
static int cxd2878_set_isdbt(struct dvb_frontend *fe)
{
	struct cxd2878_dev *dev = fe->demodulator_priv;
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	int ret = 0;

	if (dev->base->config.LED_switch)
		dev->base->config.LED_switch(dev->base->i2c, 5);

	dev->bandwidth = (enum sony_dtv_bandwidth_t)(c->bandwidth_hz / 1000000);
	if ((dev->state == SONY_DEMOD_STATE_ACTIVE) &&
	    (dev->system == SONY_DTV_SYSTEM_ISDBT)) {
		/* Demodulator Active and set to ISDB-T mode */
		cxd2878_wr(dev, dev->slvt, 0x00, 0x00);
		cxd2878_wr(dev, dev->slvt, 0xc3, 0x01);
		SLtoAIT_BandSetting(dev);

	} else if ((dev->state == SONY_DEMOD_STATE_ACTIVE) &&
		   (dev->system != SONY_DTV_SYSTEM_ISDBT)) {
		/* Demodulator Active but not ISDB-T mode */
		cxd2878_sleep(fe);
		dev->system = SONY_DTV_SYSTEM_ISDBT;
		SLtoAIT(dev);

	} else if (dev->state == SONY_DEMOD_STATE_SLEEP) {
		/* Demodulator in Sleep mode */
		dev->system = SONY_DTV_SYSTEM_ISDBT;

		SLtoAIT(dev);

	} else {
		goto err;
	}

	/* Update demodulator state */
	dev->state = SONY_DEMOD_STATE_ACTIVE;

	return 0;
err:
	dev_err(&dev->base->i2c->dev, "%s: set isdbt error !", KBUILD_MODNAME);
	return ret;
}

static void cxd2878_set_tsid(struct dvb_frontend *fe, u32 stream_id)
{
	struct cxd2878_dev *dev = fe->demodulator_priv;
	cxd2878_wr(dev, dev->slvt, 0x00, 0xc0);
	u8 data[3] = { stream_id >> 8, stream_id & 0xff,
		       stream_id < 8 /* relative TS number */ };
	cxd2878_wrm(dev, dev->slvt, 0xe9, data, 3);
}

static int cxd2878_set_isdbs3(struct dvb_frontend *fe);

static int cxd2878_set_isdbs(struct dvb_frontend *fe)
{
	struct cxd2878_dev *dev = fe->demodulator_priv;
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	int ret = 0;

	u16 network_id = (c->stream_id >> 12) & 0xf;
	if (network_id == 0xb || network_id == 0xc) {
		return cxd2878_set_isdbs3(fe);
	}

	if (dev->base->config.LED_switch)
		dev->base->config.LED_switch(dev->base->i2c, 5);

	if ((dev->state == SONY_DEMOD_STATE_ACTIVE) &&
	    (dev->system == SONY_DTV_SYSTEM_ISDBS)) {
		/* Demodulator Active and set to ISDB-S mode */
		cxd2878_set_tsid(fe, c->stream_id);
		cxd2878_wr(dev, dev->slvt, 0x00, 0x00);
		cxd2878_wr(dev, dev->slvt, 0xc3, 0x01);
	} else if ((dev->state == SONY_DEMOD_STATE_ACTIVE) &&
		   (dev->system != SONY_DTV_SYSTEM_ISDBS)) {
		/* Demodulator Active but not ISDB-S mode */
		cxd2878_sleep(fe);
		cxd2878_set_tsid(fe, c->stream_id);
		dev->system = SONY_DTV_SYSTEM_ISDBS;
		SLtoAIS(dev);

	} else if (dev->state == SONY_DEMOD_STATE_SLEEP) {
		/* Demodulator in Sleep mode */
		dev->system = SONY_DTV_SYSTEM_ISDBS;
		cxd2878_set_tsid(fe, c->stream_id);
		SLtoAIS(dev);

	} else {
		goto err;
	}

	/* Update demodulator state */
	dev->state = SONY_DEMOD_STATE_ACTIVE;
	dev->symbol_rate = 28860000;

	return 0;
err:
	dev_err(&dev->base->i2c->dev, "%s: set isdbs error !", KBUILD_MODNAME);
	return ret;
}

static void cxd2878_set_stream_id(struct dvb_frontend *fe, u32 stream_id)
{
	struct cxd2878_dev *dev = fe->demodulator_priv;
	cxd2878_wr(dev, dev->slvt, 0x00, 0xd0);
	u8 data[2] = { stream_id >> 8, stream_id & 0xff };
	cxd2878_wrm(dev, dev->slvt, 0x87, data, 2);
}

static int cxd2878_set_isdbs3(struct dvb_frontend *fe)
{
	struct cxd2878_dev *dev = fe->demodulator_priv;
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	int ret = 0;

	if (dev->base->config.LED_switch)
		dev->base->config.LED_switch(dev->base->i2c, 5);

	c->symbol_rate = 33750000;
	if ((dev->state == SONY_DEMOD_STATE_ACTIVE) &&
	    (dev->system == SONY_DTV_SYSTEM_ISDBS3)) {
		/* Demodulator Active and set to ISDB-S3 mode */
		cxd2878_set_stream_id(fe, c->stream_id);
		u8 tlv_output = 1;
		if (tlv_output) {
			cxd2878_wr(dev, dev->slvt, 0x00, 0x01);
			cxd2878_wr(dev, dev->slvt, 0xc0, 0x01);
		} else {
			cxd2878_wr(dev, dev->slvt, 0x00, 0x00);
			cxd2878_wr(dev, dev->slvt, 0xc3, 0x01);
		}
	} else if ((dev->state == SONY_DEMOD_STATE_ACTIVE) &&
		   (dev->system != SONY_DTV_SYSTEM_ISDBS3)) {
		/* Demodulator Active but not ISDB-S3 mode */
		cxd2878_sleep(fe);
		cxd2878_set_stream_id(fe, c->stream_id);
		dev->system = SONY_DTV_SYSTEM_ISDBS3;
		SLtoAIS3(dev);

	} else if (dev->state == SONY_DEMOD_STATE_SLEEP) {
		/* Demodulator in Sleep mode */
		dev->system = SONY_DTV_SYSTEM_ISDBS3;
		cxd2878_set_stream_id(fe, c->stream_id);
		SLtoAIS3(dev);

	} else {
		goto err;
	}

	/* Update demodulator state */
	dev->state = SONY_DEMOD_STATE_ACTIVE;
	dev->symbol_rate = c->symbol_rate;

	return 0;
err:
	dev_err(&dev->base->i2c->dev, "%s: set isdbs3 error !", KBUILD_MODNAME);
	return ret;
}

static int cxd2878_init(struct dvb_frontend *fe)
{
	struct cxd2878_dev *dev = fe->demodulator_priv;
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;

	int ret;

	mutex_lock(&dev->base->i2c_lock);

	if (dev->warm)
		goto warm_start;
	//clear all registers
	ret = cxd2878_wr(dev, dev->slvx, 0x02, 0x00);
	if (ret)
		goto err;

	msleep(4);

	cxd2878_wr(dev, dev->slvx, 0x00, 0x00);
	cxd2878_wr(dev, dev->slvx, 0x10, 0x01);
	cxd2878_wr(dev, dev->slvx, 0x18, 0x01);
	cxd2878_wr(dev, dev->slvx, 0x28, 0x13);
	cxd2878_wr(dev, dev->slvx, 0x17, 0x01);
	//init setting for crystal oscillator
	cxd2878_wr(dev, dev->slvx, 0x1D, 0x00);
	/* Clock mode setting */
	cxd2878_wr(dev, dev->slvx, 0x14, dev->base->config.xtal);
	cxd2878_wr(dev, dev->slvx, 0x1c, 0x03);
	msleep(6);
	cxd2878_wr(dev, dev->slvx, 0x50, 0x00);
	msleep(5);
	cxd2878_wr(dev, dev->slvx, 0x10, 0x00);
	msleep(5);

	dev->state = SONY_DEMOD_STATE_SLEEP;

	/*setup tuner i2c bus*/
	cxd2878_SetBankAndRegisterBits(dev, dev->slvx, 0x00, 0x1A, 0x01, 0xFF);
	msleep(2);

	//init internal tuner
	cxd2878_i2c_repeater(dev, 1);

	if (dev->chipid == SONY_DEMOD_CHIP_ID_CXD2857)
		cxd2857_init(dev);

	cxd2878_i2c_repeater(dev, 0);

	if (dev->chipid == SONY_DEMOD_CHIP_ID_CXD2857 &&
	    dev->base->config.tlv_mode) {
		cxd2878_SetBankAndRegisterBits(dev, dev->slvt, 0xa0, 0xb9, 0x01,
					       0x01);

		cxd2878_SetBankAndRegisterBits(dev, dev->slvt, 0x01, 0xc1, 0x00,
					       0x80); // serial disable
		cxd2878_SetBankAndRegisterBits(dev, dev->slvt, 0x01, 0xcf, 0x00,
					       0x01); // 8-bit parallel

		cxd2878_SetBankAndRegisterBits(dev, dev->slvt, 0x01, 0xc1, 0x00,
					       0x10); // MSB TSDATA[7]

		cxd2878_SetBankAndRegisterBits(dev, dev->slvt, 0x00, 0xc4, 0x00,
					       0x80);

		cxd2878_SetBankAndRegisterBits(dev, dev->slvt, 0x01, 0xc8, 0x00,
					       0x08);
		cxd2878_wr(dev, dev->slvx, 0x00, 0x01);
		cxd2878_wr(dev, dev->slvx, 0xf3, 0x02);
		cxd2878_SetBankAndRegisterBits(dev, dev->slvx, 0x00, 0xa5, 0x04,
					       0x0f);
		cxd2878_SetBankAndRegisterBits(dev, dev->slvx, 0x00, 0x82, 0x00,
					       1 << 2);
		cxd2878_SetBankAndRegisterBits(dev, dev->slvx, 0x00, 0x81, 0xff,
					       0);
	} else {
		ret = -EINVAL;
		goto err;
	}

warm_start:
	dev->warm = 1;

	c->cnr.len = 1;
	c->cnr.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
	c->post_bit_error.len = 1;
	c->post_bit_error.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
	c->post_bit_count.len = 1;
	c->post_bit_count.stat[0].scale = FE_SCALE_NOT_AVAILABLE;

	mutex_unlock(&dev->base->i2c_lock);

	return 0;
err:
	mutex_unlock(&dev->base->i2c_lock);
	dev_err(&dev->base->i2c->dev, "%s:Init failed!", KBUILD_MODNAME);

	return ret;
}

static int cxd2878_read_status(struct dvb_frontend *fe, enum fe_status *status)
{
	struct cxd2878_dev *dev = fe->demodulator_priv;
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	int ret = 0;
	u8 data = 0;
	u8 syncstat, tslockstat, unlockdetected;
	u32 per = 0;
	u8 tmp[2];
	u16 tmp16 = 0;
	s32 rflevel, snr = 0;
	u8 data3[3];

	mutex_lock(&dev->base->i2c_lock);

	*status = 0;
	switch (c->delivery_system) {
	case SYS_ISDBT:
		cxd2878_wr(dev, dev->slvt, 0x00, 0x60);
		cxd2878_rdm(dev, dev->slvt, 0x10, &data, 1);

		unlockdetected = (u8)((data & 0x10) ? 1 : 0);
		syncstat = (u8)((data & 0x02) ? 1 : 0);
		tslockstat = (u8)((data & 0x01) ? 1 : 0);

		if (!unlockdetected && syncstat && tslockstat)
			*status = FE_HAS_SIGNAL | FE_HAS_CARRIER |
				  FE_HAS_VITERBI | FE_HAS_SYNC | FE_HAS_LOCK;
		else
			*status = FE_HAS_SIGNAL | FE_HAS_CARRIER;
		break;
	case SYS_ISDBS: {
		if (dev->symbol_rate >= 33750000) {
			cxd2878_wr(dev, dev->slvt, 0x00, 0xa0);
			cxd2878_rdm(dev, dev->slvt, 0x10, data3, 2);
			// u8 agclockstat = (u8)((data3[0] & 0x20)? 1 : 0);
			u8 tstlvlockstat = (u8)((data3[1] & 0x40) ? 1 : 0);
			// u8 tmcclockstat = (u8)((data3[1] & 0x80) ? 1 : 0);

			if (tstlvlockstat)
				*status = FE_HAS_SIGNAL | FE_HAS_CARRIER |
					  FE_HAS_VITERBI | FE_HAS_SYNC |
					  FE_HAS_LOCK;
			else
				*status = FE_HAS_SIGNAL | FE_HAS_CARRIER;
		} else {
			cxd2878_wr(dev, dev->slvt, 0x00, 0xa0);
			cxd2878_rdm(dev, dev->slvt, 0x10, data3, 3);
			// u8 agclockstat = (u8)((data3[0] & 0x20)? 1 : 0);
			u8 tslockstat = (u8)((data3[2] & 0x40) ? 1 : 0);
			// u8 tmcclockstat = (u8)((data3[2] & 0x20) ? 1 : 0);

			if (tslockstat)
				*status = FE_HAS_SIGNAL | FE_HAS_CARRIER |
					  FE_HAS_VITERBI | FE_HAS_SYNC |
					  FE_HAS_LOCK;
			else
				*status = FE_HAS_SIGNAL | FE_HAS_CARRIER;
		}
		break;
	}
	default:
		ret = -EINVAL;
		goto err;
	}

	/*rf signal*/
	c->strength.len = 0;
	switch (c->delivery_system) {
	case SYS_ISDBT:
		ret |= cxd2878_i2c_repeater(dev, 1);
		ret |= cxd2857_read_rssi_isdbt(dev, c->frequency / 1000,
					       &rflevel);
		ret |= cxd2878_i2c_repeater(dev, 0);
		c->strength.len = 1;
		c->strength.stat[0].scale = FE_SCALE_DECIBEL;
		c->strength.stat[0].svalue = rflevel * 10 - 4500;
		break;
	case SYS_ISDBS:
		ret |= cxd2857_read_rflevel_isdbs(dev, c->frequency, &rflevel);
		c->strength.len = 1;
		c->strength.stat[0].scale = FE_SCALE_DECIBEL;
		c->strength.stat[0].svalue = rflevel;
		break;
	default:
		break;
	}

	c->cnr.len = 1;
	c->cnr.stat[0].scale = FE_SCALE_NOT_AVAILABLE;

	switch (c->delivery_system) {
	case SYS_ISDBS:
		if (dev->symbol_rate >= 33750000) {
			ret |= cxd2857_read_cnr_isdbs3(dev,
						       &c->cnr.stat[0].svalue);
		} else {
			ret |= cxd2857_read_cnr_isdbs(dev,
						      &c->cnr.stat[0].svalue);
		}
		if (!ret) {
			c->cnr.len = 1;
			c->cnr.stat[0].scale = FE_SCALE_DECIBEL;
		}
		ret = 0;
		break;
	default:
		break;
	}
	if (!(*status & FE_HAS_LOCK)) {
		mutex_unlock(&dev->base->i2c_lock);
		return ret;
	}

	if ((*status & FE_HAS_VITERBI) &&
	    c->cnr.stat[0].scale == FE_SCALE_NOT_AVAILABLE) {
		switch (c->delivery_system) {
		case SYS_ISDBT:
			cxd2878_wr(dev, dev->slvt, 0x00, 0x60);
			cxd2878_rdm(dev, dev->slvt, 0x28, tmp, 2);
			tmp16 = tmp[0] << 8 | tmp[1];
			snr = 100 * (s32)sony_math_log10(tmp16) - 9031;
			break;
		default:
			break;
		}
		c->cnr.len = 2;
		c->cnr.stat[0].scale = FE_SCALE_DECIBEL;
		c->cnr.stat[0].uvalue = snr - 1500;
		c->cnr.stat[1].scale = FE_SCALE_RELATIVE;
		c->cnr.stat[1].uvalue = (30 - ((snr - 1500) / 1000)) * 10;
		c->cnr.stat[1].uvalue =
			min(max((snr - 1500) / 1000 * 24 / 10, 0), 100) * 656;
		if (c->cnr.stat[1].uvalue > 0xffff)
			c->cnr.stat[1].uvalue = 0xffff;
	}

	if (*status & FE_HAS_LOCK) {
		u32 packeterr = 0, period = 0, Q = 0, R = 0;
		u8 datapacketerr[6], datapacketnum[2];

		switch (c->delivery_system) {
		case SYS_ISDBT:
			cxd2878_wr(dev, dev->slvt, 0x00, 0x40);
			cxd2878_rdm(dev, dev->slvt, 0x1F, datapacketerr, 6);
			cxd2878_rdm(dev, dev->slvt, 0x5B, datapacketnum, 2);
			period =
				((u32)datapacketnum[0] << 8) + datapacketnum[1];
			packeterr =
				((u32)datapacketerr[0] << 8) + datapacketerr[1];
			Q = (packeterr * 1000) / period;
			R = (packeterr * 1000) % period;
			R *= 1000;
			Q = Q * 1000 + R / period;
			R = R % period;
			if ((period != 1) && (R >= period / 2))
				per = Q + 1;
			else
				per = Q;

			break;
		default:
			break;
		}
		c->post_bit_error.stat[0].scale = FE_SCALE_COUNTER;
		c->post_bit_error.stat[0].uvalue = per;
		c->post_bit_count.stat[0].scale = FE_SCALE_COUNTER;
		c->post_bit_count.stat[0].uvalue = per;
	}

	c->post_bit_count.len = 1;
	c->post_bit_error.len = 1;

	mutex_unlock(&dev->base->i2c_lock);

	return 0;
err:
	mutex_unlock(&dev->base->i2c_lock);
	dev_err(&dev->base->i2c->dev, "%s:read status failed!", KBUILD_MODNAME);
	return ret;
}

static int cxd2878_set_isdbs(struct dvb_frontend *fe);

static int cxd2878_set_frontend(struct dvb_frontend *fe)
{
	struct cxd2878_dev *dev = fe->demodulator_priv;
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	int ret = 0;

	dev_dbg(&dev->base->i2c->dev,
		"delivery_system=%u modulation=%u frequency=%u bandwidth_hz=%u symbol_rate=%u inversion=%u stream_id=%d\n",
		c->delivery_system, c->modulation, c->frequency,
		c->bandwidth_hz, c->symbol_rate, c->inversion, c->stream_id);

	mutex_lock(&dev->base->i2c_lock);

	if (!dev->warm)
		cxd2878_init(fe);

	if (dev->base->config.RF_switch)
		dev->base->config.RF_switch(dev->base->i2c,
					    dev->base->config.rf_port, 1);
	if (dev->base->config.TS_switch)
		dev->base->config.TS_switch(dev->base->i2c, 1);

	switch (c->delivery_system) {
	case SYS_ISDBT:
		ret = cxd2878_set_isdbt(fe);
		break;
	case SYS_ISDBS:
		ret = cxd2878_set_isdbs(fe);
		break;
	default:
		goto err;
	}

	// set tuner
	ret |= cxd2878_i2c_repeater(dev, 1);
	if (dev->chipid == SONY_DEMOD_CHIP_ID_CXD2857)
		ret |= cxd2857_tune(dev, c->frequency); //unit khz

	ret |= cxd2878_i2c_repeater(dev, 0);
	ret |= cxd2878_tuneEnd(dev);

	if (ret)
		goto err;

	mutex_unlock(&dev->base->i2c_lock);

	msleep(20);

	return 0;
err:
	mutex_unlock(&dev->base->i2c_lock);
	dev_err(&dev->base->i2c->dev, "%s:set frontend failed!",
		KBUILD_MODNAME);
	return ret;
}
static int cxd2878_tune(struct dvb_frontend *fe, bool re_tune,
			unsigned int mode_flags, unsigned int *delay,
			enum fe_status *status)
{
	struct cxd2878_dev *dev = fe->demodulator_priv;
	int ret = 0;

	if (re_tune) {
		ret = cxd2878_set_frontend(fe);
		if (ret)
			return ret;
		dev->tune_time = jiffies;
	}
	*delay = HZ;

	ret = cxd2878_read_status(fe, status);
	if (ret)
		return ret;

	if (*status & FE_HAS_LOCK)
		return 0;

	return 0;
}

#ifdef TBS_DVB_EXTENSION
static int cxd2878_set_property(struct dvb_frontend *fe, u32 cmd, u32 data)
{
	int ret = 0;
	switch (cmd) {
	case DTV_DELIVERY_SYSTEM:
		switch (data) {
		default:
		case SYS_DVBT:
		case SYS_DVBT2:
			fe->ops.info.frequency_min_hz = 174 * MHz;
			fe->ops.info.frequency_max_hz = 868 * MHz;
			fe->ops.info.frequency_stepsize_hz = 250000;
			break;
		case SYS_ISDBT:
			fe->ops.info.frequency_min_hz = 42 * MHz;
			fe->ops.info.frequency_max_hz = 1002 * MHz;
			fe->ops.info.frequency_stepsize_hz = 0;
			break;
		case SYS_ISDBS:
			fe->ops.info.frequency_min_hz = 1032000 * MHz;
			fe->ops.info.frequency_max_hz = 3224000 * MHz;
			fe->ops.info.frequency_stepsize_hz = 0;
			break;
		case SYS_DVBC_ANNEX_A:
		case SYS_DVBC_ANNEX_B:
		case SYS_DVBC_ANNEX_C:
			fe->ops.info.frequency_min_hz = 47 * MHz;
			fe->ops.info.frequency_max_hz = 862 * MHz;
			fe->ops.info.frequency_stepsize_hz = 62500;
			fe->ops.info.symbol_rate_min = 1700000;
			fe->ops.info.symbol_rate_max = 7200000;
			break;
		case SYS_ATSC:
			fe->ops.info.frequency_min_hz = 54 * MHz;
			fe->ops.info.frequency_max_hz = 858 * MHz;
			fe->ops.info.frequency_stepsize_hz = 62500;
			break;
		}
	}

	return ret;
}
#endif
static enum dvbfe_algo cxd2878_get_algo(struct dvb_frontend *fe)
{
	return DVBFE_ALGO_HW;
}
static int cxd2878_read_ber(struct dvb_frontend *fe, u32 *ber)
{
	struct dtv_frontend_properties *p = &fe->dtv_property_cache;

	if (p->post_bit_error.stat[0].scale == FE_SCALE_COUNTER &&
	    p->post_bit_count.stat[0].scale == FE_SCALE_COUNTER)
		*ber = (u32)p->post_bit_count.stat[0].uvalue ?
			       (u32)p->post_bit_error.stat[0].uvalue /
				       (u32)p->post_bit_count.stat[0].uvalue :
			       0;

	return 0;
}
static int cxd2878_read_signal_strength(struct dvb_frontend *fe, u16 *strength)
{
	struct dtv_frontend_properties *p = &fe->dtv_property_cache;
	int i;

	*strength = 0;
	for (i = 0; i < p->strength.len; i++)
		if (p->strength.stat[i].scale == FE_SCALE_RELATIVE)
			*strength = (u16)p->strength.stat[i].uvalue;

	return 0;
}
static int cxd2878_read_snr(struct dvb_frontend *fe, u16 *snr)
{
	struct dtv_frontend_properties *p = &fe->dtv_property_cache;
	int i;

	*snr = 0;
	for (i = 0; i < p->cnr.len; i++)
		if (p->cnr.stat[i].scale == FE_SCALE_RELATIVE)
			*snr = (u16)p->cnr.stat[i].uvalue;

	return 0;
}

static int cxd2878_read_ucblocks(struct dvb_frontend *fe, u32 *ucblocks)
{
	*ucblocks = 0;

	return 0;
}
#ifdef TBS_DVB_EXTENSION
static void cxd2878_spi_read(struct dvb_frontend *fe, struct ecp3_info *ecp3inf)
{
	struct cxd2878_dev *dev = fe->demodulator_priv;

	if (dev->base->config.read_properties)
		dev->base->config.read_properties(dev->base->i2c, ecp3inf->reg,
						  &(ecp3inf->data));

	return;
}

static void cxd2878_spi_write(struct dvb_frontend *fe,
			      struct ecp3_info *ecp3inf)
{
	struct cxd2878_dev *dev = fe->demodulator_priv;

	if (dev->base->config.write_properties)
		dev->base->config.write_properties(dev->base->i2c, ecp3inf->reg,
						   ecp3inf->data);
	return;
}
static void cxd2878_eeprom_read(struct dvb_frontend *fe,
				struct eeprom_info *eepinf)
{
	struct cxd2878_dev *dev = fe->demodulator_priv;

	if (dev->base->config.read_eeprom)
		dev->base->config.read_eeprom(dev->base->i2c, eepinf->reg,
					      &(eepinf->data));
	return;
}

static void cxd2878_eeprom_write(struct dvb_frontend *fe,
				 struct eeprom_info *eepinf)
{
	struct cxd2878_dev *dev = fe->demodulator_priv;

	if (dev->base->config.write_eeprom)
		dev->base->config.write_eeprom(dev->base->i2c, eepinf->reg,
					       eepinf->data);

	return;
}
#endif
static void cxd2878_release(struct dvb_frontend *fe)
{
	struct cxd2878_dev *dev = fe->demodulator_priv;
	dev->base->count--;
	if (dev->base->count == 0) {
		list_del(&dev->base->cxdlist);
		kfree(dev->base);
	}
	kfree(dev);
}

static const struct dvb_frontend_ops cxd2878_ops = {
	.delsys = {SYS_ISDBT,SYS_ISDBS},
	.info = {
			.name = "SONY CXD2857",
			.frequency_min_hz = 45*MHz,
			.frequency_max_hz = 3224*MHz,
			.frequency_stepsize_hz = 1 * kHz,
		    .caps = FE_CAN_INVERSION_AUTO |
				FE_CAN_FEC_1_2 |
				FE_CAN_FEC_2_3 |
				FE_CAN_FEC_3_4 |
				FE_CAN_FEC_4_5 |
				FE_CAN_FEC_5_6	|
				FE_CAN_FEC_7_8	|
				FE_CAN_FEC_AUTO |
				FE_CAN_QPSK |
				FE_CAN_QAM_16 |
				FE_CAN_QAM_32 |
				FE_CAN_QAM_64 |
				FE_CAN_QAM_AUTO |
				FE_CAN_TRANSMISSION_MODE_AUTO |
				FE_CAN_GUARD_INTERVAL_AUTO |
				FE_CAN_HIERARCHY_AUTO |
				FE_CAN_2G_MODULATION |
				FE_CAN_RECOVER |
				FE_CAN_MUTE_TS,
	},

			.init 				= cxd2878_init,
			.sleep				= cxd2878_sleep,
			.release			= cxd2878_release,
			.set_frontend			= cxd2878_set_frontend,
			.tune				= cxd2878_tune,
			.get_frontend_algo		= cxd2878_get_algo,
			
			.read_status 			= cxd2878_read_status,
			.read_signal_strength		= cxd2878_read_signal_strength,
			.read_ber  			= cxd2878_read_ber,
			.read_snr			= cxd2878_read_snr,
			.read_ucblocks			= cxd2878_read_ucblocks,

#ifdef TBS_DVB_EXTENSION
			.set_property			= cxd2878_set_property,

			.spi_read			= cxd2878_spi_read,
			.spi_write			= cxd2878_spi_write,
			.eeprom_read			= cxd2878_eeprom_read,
			.eeprom_write			= cxd2878_eeprom_write,
#endif
};

static struct cxd_base *match_base(struct i2c_adapter *i2c, u8 adr)
{
	struct cxd_base *p;

	list_for_each_entry(p, &cxdlist, cxdlist)
		if (p->i2c == i2c && p->adr != adr)
			return p;
	return NULL;
}
struct dvb_frontend *cxd2857_attach(const struct cxd2857_config *config,
				    struct i2c_adapter *i2c)
{
	struct cxd2878_dev *dev;
	struct cxd_base *base;

	u16 id;
	u8 data[2];
	dev = kzalloc(sizeof(struct cxd2878_dev), GFP_KERNEL);
	if (!dev)
		goto err;

	dev->slvt = config->addr_slvt;
	dev->slvx = config->addr_slvt + 2;
	dev->slvr = config->addr_slvt - 0x20;
	dev->slvm = config->addr_slvt - 0x54;
	dev->tuner_addr = config->tuner_addr;

	dev->state = SONY_DEMOD_STATE_UNKNOWN;
	dev->system = SONY_DTV_SYSTEM_UNKNOWN;

	dev->iffreqConfig.configDVBT_5 = SONY_DEMOD_MAKE_IFFREQ_CONFIG(3.6);
	dev->iffreqConfig.configDVBT_6 = SONY_DEMOD_MAKE_IFFREQ_CONFIG(3.6);
	dev->iffreqConfig.configDVBT_7 = SONY_DEMOD_MAKE_IFFREQ_CONFIG(4.2);
	dev->iffreqConfig.configDVBT_8 = SONY_DEMOD_MAKE_IFFREQ_CONFIG(4.8);

	dev->iffreqConfig.configDVBT2_1_7 = SONY_DEMOD_MAKE_IFFREQ_CONFIG(3.5);
	dev->iffreqConfig.configDVBT2_5 = SONY_DEMOD_MAKE_IFFREQ_CONFIG(3.6);
	dev->iffreqConfig.configDVBT2_6 = SONY_DEMOD_MAKE_IFFREQ_CONFIG(3.6);
	dev->iffreqConfig.configDVBT2_7 = SONY_DEMOD_MAKE_IFFREQ_CONFIG(4.2);
	dev->iffreqConfig.configDVBT2_8 = SONY_DEMOD_MAKE_IFFREQ_CONFIG(4.8);

	dev->iffreqConfig.configDVBC_6 = SONY_DEMOD_MAKE_IFFREQ_CONFIG(3.7);
	dev->iffreqConfig.configDVBC_7 = SONY_DEMOD_MAKE_IFFREQ_CONFIG(4.9);
	dev->iffreqConfig.configDVBC_8 = SONY_DEMOD_MAKE_IFFREQ_CONFIG(4.9);

	dev->iffreqConfig.configATSC = SONY_DEMOD_ATSC_MAKE_IFFREQ_CONFIG(3.7);

	dev->iffreqConfig.configISDBT_6 = SONY_DEMOD_MAKE_IFFREQ_CONFIG(3.55);
	dev->iffreqConfig.configISDBT_7 = SONY_DEMOD_MAKE_IFFREQ_CONFIG(4.15);
	dev->iffreqConfig.configISDBT_8 = SONY_DEMOD_MAKE_IFFREQ_CONFIG(4.75);

	dev->iffreqConfig.configJ83B_5_06_5_36 =
		SONY_DEMOD_MAKE_IFFREQ_CONFIG(3.7);
	dev->iffreqConfig.configJ83B_5_60 = SONY_DEMOD_MAKE_IFFREQ_CONFIG(3.75);

	dev->atscNoSignalThresh = 0x7FFB61;
	dev->atscSignalThresh = 0x7C4926;
	dev->warm = 0;

	memcpy(&dev->fe.ops, &cxd2878_ops, sizeof(struct dvb_frontend_ops));
	dev->fe.demodulator_priv = dev;

	base = match_base(i2c, config->addr_slvt);
	if (base) {
		base->count++;
		dev->base = base;
	} else {
		base = kzalloc(sizeof(struct cxd_base), GFP_KERNEL);
		if (!base)
			goto err1;
		base->i2c = i2c;
		base->config = *config;
		base->adr = config->addr_slvt;
		base->count = 1;
		mutex_init(&base->i2c_lock);
		dev->base = base;
		list_add(&base->cxdlist, &cxdlist);
	}
	cxd2878_wr(dev, dev->slvx, 0x00, 0x00);
	cxd2878_rdm(dev, dev->slvx, 0xFB, &data[0], 1);
	cxd2878_rdm(dev, dev->slvx, 0xFD, &data[1], 1);

	id = ((data[0] & 0x03) << 8) | data[1];

	switch (id) {
	case SONY_DEMOD_CHIP_ID_CXD2857: /**< CXD2857 */
		dev_info(&i2c->dev, "Detect CXD2857 chip.");
		break;
	case SONY_DEMOD_CHIP_ID_UNKNOWN: /**< Unknown */
		dev_err(&i2c->dev, "%s:Can not detect the chip.\n",
			KBUILD_MODNAME);
		goto err1;
		break;
	}
	dev->chipid = id;

	dev_dbg(&i2c->dev, "%s: attaching frontend successfully.\n",
		KBUILD_MODNAME);

	return &dev->fe;

err1:
	kfree(dev);
err:
	dev_err(&i2c->dev, "%s:error attaching frontend.\n", KBUILD_MODNAME);
	return NULL;
}

EXPORT_SYMBOL_GPL(cxd2857_attach);

MODULE_AUTHOR("Davin zhang <Davin@tbsdtv.com>, otya <otya281@gmail.com>");
MODULE_DESCRIPTION("SONY CXD2857 Demodulator + CXD2868 Tuner driver");
MODULE_LICENSE("GPL");
