// SPDX-License-Identifier: GPL-2.0
/*
 * s5k4h7b camera driver
 *
 * Copyright (C) 2022 Rockchip Electronics Co., Ltd.
 *
 * V0.0X01.0X00 first version.
 * V0.0X01.0X01
 * 1.add flip and mirror support
 * 2.fix stream on sequential
 *
 */

// #define DEBUG
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/sysfs.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/compat.h>
#include <linux/rk-camera-module.h>
#include <media/media-entity.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-subdev.h>
#include <linux/pinctrl/consumer.h>

#define DRIVER_VERSION			KERNEL_VERSION(0, 0x01, 0x02)

#ifndef V4L2_CID_DIGITAL_GAIN
#define V4L2_CID_DIGITAL_GAIN		V4L2_CID_GAIN
#endif


#define S5K4H7B_LINK_FREQ		500000000 //1080P is 200M
/* pixel rate = link frequency * 2 * lanes / BITS_PER_SAMPLE */
#define S5K4H7B_PIXEL_RATE		(S5K4H7B_LINK_FREQ / 10 * 2 * 2)

#define S5K4H7B_XVCLK_FREQ		 24000000

#define CHIP_ID				0x487B
#define S5K4H7B_REG_CHIP_ID		0x0000	//read only reg

#define S5K4H7B_REG_CTRL_MODE		0x0100
#define S5K4H7B_MODE_SW_STANDBY	0x0
#define S5K4H7B_MODE_STREAMING		0x01
#define S5K4H7B_REG_STREAM_ON		0x3C1E

#define S5K4H7B_REG_EXPOSURE		0x0202
#define	S5K4H7B_EXPOSURE_MIN		2
#define	S5K4H7B_EXPOSURE_STEP		1
#define S5K4H7B_VTS_MAX		0xfffc

#define S5K4H7B_REG_ANALOG_GAIN	0x0204
#define S5K4H7B_GAIN_MIN		0x20
#define S5K4H7B_GAIN_MAX		0x200
#define S5K4H7B_GAIN_STEP		1
#define S5K4H7B_GAIN_DEFAULT	32

#define S5K4H7B_REG_TEST_PATTERN	0x0601
#define S5K4H7B_TEST_PATTERN_ENABLE	0x1
#define S5K4H7B_TEST_PATTERN_DISABLE	0x0

#define S5K4H7B_REG_VTS		0x0340

#define REG_NULL			0xFFFF

#define S5K4H7B_REG_VALUE_08BIT	1
#define S5K4H7B_REG_VALUE_16BIT	2
#define S5K4H7B_REG_VALUE_24BIT	3

#define S5K4H7B_LANES			2
#define S5K4H7B_BITS_PER_SAMPLE	10

#define S5K4H7B_CHIP_REVISION_REG	0x0002

#define OF_CAMERA_PINCTRL_STATE_DEFAULT	"rockchip,camera_default"
#define OF_CAMERA_PINCTRL_STATE_SLEEP	"rockchip,camera_sleep"

#define S5K4H7B_NAME			"s5k4h7b"

// #define S5K4H7B_MIRROR
// #define S5K4H7B_FLIP
// #define S5K4H7B_FLIP_MIRROR
#ifdef S5K4H7B_MIRROR
#define S5K4H7B_MEDIA_BUS_FMT		MEDIA_BUS_FMT_SRGGB10_1X10
#elif defined S5K4H7B_FLIP
#define S5K4H7B_MEDIA_BUS_FMT		MEDIA_BUS_FMT_SBGGR10_1X10
#elif defined S5K4H7B_FLIP_MIRROR
#define S5K4H7B_MEDIA_BUS_FMT		MEDIA_BUS_FMT_SGBRG10_1X10
#else
#define S5K4H7B_MEDIA_BUS_FMT		MEDIA_BUS_FMT_SGRBG10_1X10
#endif

static const char * const s5k4h7b_supply_names[] = {
	"iovdd",	/* Digital I/O power */
	"dvdd",		/* Digital core power */
	"avdd",		/* Analog power */
};

#define S5K4H7B_NUM_SUPPLIES ARRAY_SIZE(s5k4h7b_supply_names)

struct regval {
	u16 addr;
	u16 val;
};

struct s5k4h7b_mode {
	u32 width;
	u32 height;
	struct v4l2_fract max_fps;
	u32 hts_def;
	u32 vts_def;
	u32 exp_def;
	u32 link_freq_idx;
	u32 bpp;
	const struct regval *reg_list;
};

struct s5k4h7b {
	struct i2c_client	*client;
	struct clk		*xvclk;

	struct gpio_desc	*reset_gpio;
	struct gpio_desc	*pwdn_gpio;
	struct regulator	*avdd_regulator;	/* Analog power */
	struct regulator	*iovdd_regulator;	/* Digital I/O power */
	struct regulator	*dvdd_regulator;	/* Digital core powe */
	//struct gpio_desc	*iovdd_gpio;
	//struct gpio_desc	*avdd_gpio;
	//struct gpio_desc	*dvdd_gpio;
	struct regulator_bulk_data supplies[S5K4H7B_NUM_SUPPLIES];

	struct pinctrl		*pinctrl;
	struct pinctrl_state	*pins_default;
	struct pinctrl_state	*pins_sleep;

	struct v4l2_subdev	subdev;
	struct media_pad	pad;
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl	*exposure;
	struct v4l2_ctrl	*anal_gain;
	struct v4l2_ctrl	*digi_gain;
	struct v4l2_ctrl	*hblank;
	struct v4l2_ctrl	*vblank;
	struct v4l2_ctrl	*pixel_rate;
	struct v4l2_ctrl	*link_freq;
	struct v4l2_ctrl	*test_pattern;
	struct mutex		mutex;
	bool			streaming;
	bool			power_on;
	const struct s5k4h7b_mode *cur_mode;
	u32			module_index;
	const char		*module_facing;
	const char		*module_name;
	const char		*len_name;
};

#define to_s5k4h7b(sd) container_of(sd, struct s5k4h7b, subdev)

/*
 * Xclk 24Mhz
 */
static const struct regval s5k4h7b_global_regs[] = {
    {0x0100, 0x00},
    {0x0000, 0x12},
    {0x0000, 0x48},
    {0x0A02, 0x15},
    {0x0B05, 0x01},
    {0x3074, 0x06},
    {0x3075, 0x2F},
    {0x308A, 0x20},
    {0x308B, 0x08},
    {0x308C, 0x0B},
    {0x3081, 0x07},
    {0x307B, 0x85},
    {0x307A, 0x0A},
    {0x3079, 0x0A},
    {0x306E, 0x71},
    {0x306F, 0x28},
    {0x301F, 0x20},
    {0x306B, 0x9A},
    {0x3091, 0x1F},
    {0x30C4, 0x06},
    {0x3200, 0x09},
    {0x306A, 0x79},
    {0x30B0, 0xFF},
    {0x306D, 0x08},
    {0x3080, 0x00},
    {0x3929, 0x3F},
    {0x3084, 0x16},
    {0x3070, 0x0F},
    {0x3B45, 0x01},
    {0x30C2, 0x05},
    {0x3069, 0x87},
    {0x3924, 0x7F},
    {0x3925, 0xFD},
    {0x3C08, 0xFF},
    {0x3C09, 0xFF},
    {0x3C31, 0xFF},
    {0x3C32, 0xFF},
    {0x0b00, 0x01},
    {0x3931, 0x02},
    {0x392F, 0x01},
    {0x3930, 0x80},
    {0x3932, 0x80},
	{0x021c, 0x0001},//otp LSC_disable
	{REG_NULL, 0x00},
};

/*
 * Xclk 24Mhz
 * max_framerate 30fps
 * mipi_datarate per lane 500Mbps, 4lane
 */
static const struct regval s5k4h7b_linear_preview_regs[] = {
    {0x0136, 0x18},
    {0x0137, 0x00},
    {0x0305, 0x06},
    {0x0306, 0x00},
    {0x0307, 0x8C},
    {0x030D, 0x06},
    {0x030E, 0x00},
    {0x030F, 0xE1},
    {0x3C1F, 0x00},
    {0x3C17, 0x00},
    {0x3C1C, 0x05},
    {0x3C1D, 0x15},
    {0x0301, 0x04},
    {0x0820, 0x03},
    {0x0821, 0x84},
    {0x0822, 0x00},
    {0x0823, 0x00},
    {0x0112, 0x0A},
    {0x0113, 0x0A},
    {0x0114, 0x01},
    {0x3906, 0x04},
    {0x0344, 0x00},
    {0x0345, 0x08},
    {0x0346, 0x00},
    {0x0347, 0x08},
    {0x0348, 0x0C},
    {0x0349, 0xC7},
    {0x034A, 0x09},
    {0x034B, 0x97},
    {0x034C, 0x0C},
    {0x034D, 0xC0},
    {0x034E, 0x09},
    {0x034F, 0x90},
    {0x0900, 0x00},
    {0x0901, 0x00},
    {0x0381, 0x01},
    {0x0383, 0x01},
    {0x0385, 0x01},
    {0x0387, 0x01},
    {0x0101, 0x00},
    {0x0340, 0x09},
    {0x0341, 0xE4},
    {0x0342, 0x14},
    {0x0343, 0x8C},
    {0x0200, 0x13},
    {0x0201, 0xFC},
    {0x0202, 0x00},
    {0x0203, 0x02},
    {0x3400, 0x01},
	{REG_NULL, 0x00},
};

static const struct regval s5k4h7b_linear_snapshot_regs[] = {
    {0x0136, 0x18},
    {0x0137, 0x00},
    {0x0305, 0x06},
    {0x0306, 0x00},
    {0x0307, 0x8C},
    {0x030D, 0x06},
    {0x030E, 0x00},
    {0x030F, 0xE1},
    {0x3C1F, 0x00},
    {0x3C17, 0x00},
    {0x3C1C, 0x05},
    {0x3C1D, 0x15},
    {0x0301, 0x04},
    {0x0820, 0x03},
    {0x0821, 0x84},
    {0x0822, 0x00},
    {0x0823, 0x00},
    {0x0112, 0x0A},
    {0x0113, 0x0A},
    {0x0114, 0x01},
    {0x3906, 0x04},
    {0x0344, 0x00},
    {0x0345, 0x08},
    {0x0346, 0x00},
    {0x0347, 0x08},
    {0x0348, 0x0C},
    {0x0349, 0xC7},
    {0x034A, 0x09},
    {0x034B, 0x97},
    {0x034C, 0x0C},
    {0x034D, 0xC0},
    {0x034E, 0x09},
    {0x034F, 0x90},
    {0x0900, 0x00},
    {0x0901, 0x00},
    {0x0381, 0x01},
    {0x0383, 0x01},
    {0x0385, 0x01},
    {0x0387, 0x01},
    {0x0101, 0x00},
    {0x0340, 0x09},
    {0x0341, 0xE4},
    {0x0342, 0x14},
    {0x0343, 0x8C},
    {0x0200, 0x13},
    {0x0201, 0xFC},
    {0x0202, 0x00},
    {0x0203, 0x02},
    {0x3400, 0x01},
	{REG_NULL, 0x00},
};

static const struct s5k4h7b_mode supported_modes[] = {
	{
		.width = 3264,
		.height = 2448,
		.max_fps = {
			.numerator = 10000,
			.denominator = 300000,
		},
		.exp_def = 0x0990,
		.hts_def = 0x0DFC,
		.vts_def = 0x09E2,
		.bpp = 10,
		.reg_list = s5k4h7b_linear_preview_regs,
		.link_freq_idx = 0,
	},
	{
		.width = 3264,
		.height = 2448,
		.max_fps = {
			.numerator = 10000,
			.denominator = 300000,
		},
		.exp_def = 0x0990,
		.hts_def = 0x0DFC,
		.vts_def = 0x09E2,
		.bpp = 10,
		.reg_list = s5k4h7b_linear_snapshot_regs,
        .link_freq_idx = 1,

	},
};

static const s64 link_freq_items[] = {
	S5K4H7B_LINK_FREQ
};

static const char * const s5k4h7b_test_pattern_menu[] = {
	"Disabled",
	"Vertical Color Bar Type 1",
	"Vertical Color Bar Type 2",
	"Vertical Color Bar Type 3",
	"Vertical Color Bar Type 4"
};

/* Write registers up to 4 at a time */
static int s5k4h7b_write_reg(struct i2c_client *client, u16 reg,
			     u32 len, u32 val)
{
	u32 buf_i, val_i;
	u8 buf[6];
	u8 *val_p;
	__be32 val_be;

	dev_dbg(&client->dev, "write reg(0x%x val:0x%x)!\n", reg, val);

	if (len > 4)
		return -EINVAL;

	buf[0] = reg >> 8;
	buf[1] = reg & 0xff;

	val_be = cpu_to_be32(val);
	val_p = (u8 *)&val_be;
	buf_i = 2;
	val_i = 4 - len;

	while (val_i < 4)
		buf[buf_i++] = val_p[val_i++];

	if (i2c_master_send(client, buf, len + 2) != len + 2)
		return -EIO;

	return 0;
}

static int s5k4h7b_write_array(struct i2c_client *client,
			       const struct regval *regs)
{
	u32 i;
	int ret = 0;

	for (i = 0; ret == 0 && regs[i].addr != REG_NULL; i++)
		ret = s5k4h7b_write_reg(client, regs[i].addr,
					S5K4H7B_REG_VALUE_08BIT,
					regs[i].val);

	return ret;
}

/* Read registers up to 4 at a time */
static int s5k4h7b_read_reg(struct i2c_client *client, u16 reg,
			    unsigned int len, u32 *val)
{
	struct i2c_msg msgs[2];
	u8 *data_be_p;
	__be32 data_be = 0;
	__be16 reg_addr_be = cpu_to_be16(reg);
	int ret;

	if (len > 4 || !len)
		return -EINVAL;

	data_be_p = (u8 *)&data_be;
	/* Write register address */
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = 2;
	msgs[0].buf = (u8 *)&reg_addr_be;

	/* Read data from register */
	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = &data_be_p[4 - len];

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret != ARRAY_SIZE(msgs))
		return -EIO;

	*val = be32_to_cpu(data_be);

	return 0;
}

static int s5k4h7b_get_reso_dist(const struct s5k4h7b_mode *mode,
				 struct v4l2_mbus_framefmt *framefmt)
{
	return abs(mode->width - framefmt->width) +
	       abs(mode->height - framefmt->height);
}

static const struct s5k4h7b_mode *
s5k4h7b_find_best_fit(struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *framefmt = &fmt->format;
	int dist;
	int cur_best_fit = 0;
	int cur_best_fit_dist = -1;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(supported_modes); i++) {
		dist = s5k4h7b_get_reso_dist(&supported_modes[i], framefmt);
		if (cur_best_fit_dist == -1 || dist < cur_best_fit_dist) {
			cur_best_fit_dist = dist;
			cur_best_fit = i;
		}
	}

	return &supported_modes[cur_best_fit];
}

static int s5k4h7b_set_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct s5k4h7b *s5k4h7b = to_s5k4h7b(sd);
	const struct s5k4h7b_mode *mode;
	s64 h_blank, vblank_def;
	u64 pixel_rate = 0;
	u32 lane_num = S5K4H7B_LANES;

	mutex_lock(&s5k4h7b->mutex);

	mode = s5k4h7b_find_best_fit(fmt);
	fmt->format.code = S5K4H7B_MEDIA_BUS_FMT;
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		*v4l2_subdev_get_try_format(sd, cfg, fmt->pad) = fmt->format;
#else
		mutex_unlock(&s5k4h7b->mutex);
		return -ENOTTY;
#endif
	} else {
		s5k4h7b->cur_mode = mode;
		h_blank = mode->hts_def - mode->width;
		__v4l2_ctrl_modify_range(s5k4h7b->hblank, h_blank,
					 h_blank, 1, h_blank);
		vblank_def = mode->vts_def - mode->height;
		__v4l2_ctrl_modify_range(s5k4h7b->vblank, vblank_def,
					 S5K4H7B_VTS_MAX - mode->height,
					 1, vblank_def);
		pixel_rate = (u32)link_freq_items[mode->link_freq_idx] / mode->bpp * 2 * lane_num;

		__v4l2_ctrl_s_ctrl_int64(s5k4h7b->pixel_rate,
					 pixel_rate);
		__v4l2_ctrl_s_ctrl(s5k4h7b->link_freq,
				   mode->link_freq_idx);
	}

	mutex_unlock(&s5k4h7b->mutex);

	return 0;
}

static int s5k4h7b_get_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg,
			   struct v4l2_subdev_format *fmt)
{
	struct s5k4h7b *s5k4h7b = to_s5k4h7b(sd);
	const struct s5k4h7b_mode *mode = s5k4h7b->cur_mode;

	mutex_lock(&s5k4h7b->mutex);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		fmt->format = *v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
#else
		mutex_unlock(&s5k4h7b->mutex);
		return -ENOTTY;
#endif
	} else {
		fmt->format.width = mode->width;
		fmt->format.height = mode->height;
		fmt->format.code = S5K4H7B_MEDIA_BUS_FMT;
		fmt->format.field = V4L2_FIELD_NONE;
	}
	mutex_unlock(&s5k4h7b->mutex);

	return 0;
}

static int s5k4h7b_enum_mbus_code(struct v4l2_subdev *sd,
				  struct v4l2_subdev_pad_config *cfg,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index != 0)
		return -EINVAL;
	code->code = S5K4H7B_MEDIA_BUS_FMT;

	return 0;
}

static int s5k4h7b_enum_frame_sizes(struct v4l2_subdev *sd,
				    struct v4l2_subdev_pad_config *cfg,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index >= ARRAY_SIZE(supported_modes))
		return -EINVAL;

	if (fse->code != S5K4H7B_MEDIA_BUS_FMT)
		return -EINVAL;

	fse->min_width  = supported_modes[fse->index].width;
	fse->max_width  = supported_modes[fse->index].width;
	fse->max_height = supported_modes[fse->index].height;
	fse->min_height = supported_modes[fse->index].height;

	return 0;
}

static int s5k4h7b_enable_test_pattern(struct s5k4h7b *s5k4h7b, u32 pattern)
{
	u32 val;

	if (pattern)
		val = (pattern - 1) | S5K4H7B_TEST_PATTERN_ENABLE;
	else
		val = S5K4H7B_TEST_PATTERN_DISABLE;

	return s5k4h7b_write_reg(s5k4h7b->client,
				 S5K4H7B_REG_TEST_PATTERN,
				 S5K4H7B_REG_VALUE_08BIT,
				 val);
}

static int s5k4h7b_g_frame_interval(struct v4l2_subdev *sd,
				    struct v4l2_subdev_frame_interval *fi)
{
	struct s5k4h7b *s5k4h7b = to_s5k4h7b(sd);
	const struct s5k4h7b_mode *mode = s5k4h7b->cur_mode;

	mutex_lock(&s5k4h7b->mutex);
	fi->interval = mode->max_fps;
	mutex_unlock(&s5k4h7b->mutex);

	return 0;
}

static void s5k4h7b_get_module_inf(struct s5k4h7b *s5k4h7b,
				   struct rkmodule_inf *inf)
{
	memset(inf, 0, sizeof(*inf));
	strscpy(inf->base.sensor, S5K4H7B_NAME, sizeof(inf->base.sensor));
	strscpy(inf->base.module, s5k4h7b->module_name,
		sizeof(inf->base.module));
	strscpy(inf->base.lens, s5k4h7b->len_name, sizeof(inf->base.lens));
}

static long s5k4h7b_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct s5k4h7b *s5k4h7b = to_s5k4h7b(sd);
	long ret = 0;
	u32 stream = 0;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		s5k4h7b_get_module_inf(s5k4h7b, (struct rkmodule_inf *)arg);
		break;
	case RKMODULE_SET_QUICK_STREAM:

		stream = *((u32 *)arg);

		if (stream)
			ret = s5k4h7b_write_reg(s5k4h7b->client,
				 S5K4H7B_REG_CTRL_MODE,
				 S5K4H7B_REG_VALUE_08BIT,
				 S5K4H7B_MODE_STREAMING);
		else
			ret = s5k4h7b_write_reg(s5k4h7b->client,
				 S5K4H7B_REG_CTRL_MODE,
				 S5K4H7B_REG_VALUE_08BIT,
				 S5K4H7B_MODE_SW_STANDBY);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}

#ifdef CONFIG_COMPAT
static long s5k4h7b_compat_ioctl32(struct v4l2_subdev *sd,
				   unsigned int cmd, unsigned long arg)
{
	void __user *up = compat_ptr(arg);
	struct rkmodule_inf *inf;
	struct rkmodule_awb_cfg *cfg;
	long ret = 0;
	u32 stream = 0;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		inf = kzalloc(sizeof(*inf), GFP_KERNEL);
		if (!inf) {
			ret = -ENOMEM;
			return ret;
		}

		ret = s5k4h7b_ioctl(sd, cmd, inf);
		if (!ret) {
			ret = copy_to_user(up, inf, sizeof(*inf));
			if (ret)
				ret = -EFAULT;
		}
		kfree(inf);
		break;
	case RKMODULE_AWB_CFG:
		cfg = kzalloc(sizeof(*cfg), GFP_KERNEL);
		if (!cfg) {
			ret = -ENOMEM;
			return ret;
		}

		ret = copy_from_user(cfg, up, sizeof(*cfg));
		if (!ret)
			ret = s5k4h7b_ioctl(sd, cmd, cfg);
		else
			ret = -EFAULT;
		kfree(cfg);
		break;
	case RKMODULE_SET_QUICK_STREAM:
		ret = copy_from_user(&stream, up, sizeof(u32));
		if (!ret)
			ret = s5k4h7b_ioctl(sd, cmd, &stream);
		else
			ret = -EFAULT;
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}
#endif

static int __s5k4h7b_start_stream(struct s5k4h7b *s5k4h7b)
{
	int ret;

	ret = s5k4h7b_write_array(s5k4h7b->client, s5k4h7b->cur_mode->reg_list);
	if (ret)
		return ret;

	/* In case these controls are set before streaming */
	mutex_unlock(&s5k4h7b->mutex);
	ret = v4l2_ctrl_handler_setup(&s5k4h7b->ctrl_handler);
	mutex_lock(&s5k4h7b->mutex);
	if (ret)
		return ret;

	s5k4h7b_write_reg(s5k4h7b->client,
				 S5K4H7B_REG_STREAM_ON,
				 S5K4H7B_REG_VALUE_08BIT,
				 S5K4H7B_MODE_STREAMING);
	s5k4h7b_write_reg(s5k4h7b->client,
				 S5K4H7B_REG_CTRL_MODE,
				 S5K4H7B_REG_VALUE_08BIT,
				 S5K4H7B_MODE_STREAMING);
	s5k4h7b_write_reg(s5k4h7b->client,
				 S5K4H7B_REG_STREAM_ON,
				 S5K4H7B_REG_VALUE_08BIT,
				 S5K4H7B_MODE_SW_STANDBY);
	return 0;
}

static int __s5k4h7b_stop_stream(struct s5k4h7b *s5k4h7b)
{
	return s5k4h7b_write_reg(s5k4h7b->client,
				 S5K4H7B_REG_CTRL_MODE,
				 S5K4H7B_REG_VALUE_08BIT,
				 S5K4H7B_MODE_SW_STANDBY);
}

static int s5k4h7b_s_stream(struct v4l2_subdev *sd, int on)
{
	struct s5k4h7b *s5k4h7b = to_s5k4h7b(sd);
	struct i2c_client *client = s5k4h7b->client;
	int ret = 0;

	dev_info(&client->dev, "%s: on: %d, %dx%d@%d\n", __func__, on,
				s5k4h7b->cur_mode->width,
				s5k4h7b->cur_mode->height,
		DIV_ROUND_CLOSEST(s5k4h7b->cur_mode->max_fps.denominator,
				  s5k4h7b->cur_mode->max_fps.numerator));

	mutex_lock(&s5k4h7b->mutex);
	on = !!on;
	if (on == s5k4h7b->streaming)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		ret = __s5k4h7b_start_stream(s5k4h7b);
		if (ret) {
			v4l2_err(sd, "start stream failed while write regs\n");
			pm_runtime_put(&client->dev);
			goto unlock_and_return;
		}
	} else {
		__s5k4h7b_stop_stream(s5k4h7b);
		pm_runtime_put(&client->dev);
	}

	s5k4h7b->streaming = on;

unlock_and_return:
	mutex_unlock(&s5k4h7b->mutex);

	return ret;
}

static int s5k4h7b_s_power(struct v4l2_subdev *sd, int on)
{
	struct s5k4h7b *s5k4h7b = to_s5k4h7b(sd);
	struct i2c_client *client = s5k4h7b->client;
	int ret = 0;

	mutex_lock(&s5k4h7b->mutex);

	/* If the power state is not modified - no work to do. */
	if (s5k4h7b->power_on == !!on)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		ret = s5k4h7b_write_array(s5k4h7b->client, s5k4h7b_global_regs);
		if (ret) {
			v4l2_err(sd, "could not set init registers\n");
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		s5k4h7b->power_on = true;
	} else {
		pm_runtime_put(&client->dev);
		s5k4h7b->power_on = false;
	}

unlock_and_return:
	mutex_unlock(&s5k4h7b->mutex);

	return ret;
}

/* Calculate the delay in us by clock rate and clock cycles */
static inline u32 s5k4h7b_cal_delay(u32 cycles)
{
	return DIV_ROUND_UP(cycles, S5K4H7B_XVCLK_FREQ / 1000 / 1000);
}

static int __s5k4h7b_power_on(struct s5k4h7b *s5k4h7b)
{
	int ret;
	u32 delay_us;
	struct device *dev = &s5k4h7b->client->dev;

	if (!IS_ERR_OR_NULL(s5k4h7b->pins_default)) {
		ret = pinctrl_select_state(s5k4h7b->pinctrl,
					   s5k4h7b->pins_default);
		if (ret < 0)
			dev_err(dev, "could not set pins pins_default\n");
	}
	ret = clk_set_rate(s5k4h7b->xvclk, S5K4H7B_XVCLK_FREQ);
	if (ret < 0)
		dev_warn(dev, "Failed to set xvclk rate (24MHz)\n");
	if (clk_get_rate(s5k4h7b->xvclk) != S5K4H7B_XVCLK_FREQ)
		dev_warn(dev, "xvclk mismatched, modes are based on 24MHz\n");
	ret = clk_prepare_enable(s5k4h7b->xvclk);
	if (ret < 0) {
		dev_err(dev, "Failed to enable xvclk\n");
		return ret;
	}

	//
	gpiod_set_value_cansleep(s5k4h7b->pwdn_gpio, 0);
	gpiod_set_value_cansleep(s5k4h7b->reset_gpio, 0);
	clk_set_rate(s5k4h7b->xvclk, 0);
	//regulator_disable(s5k4h7b->iovdd_regulator);
	//regulator_disable(s5k4h7b->dvdd_regulator);
	//regulator_disable(s5k4h7b->avdd_regulator);
	usleep_range(1000, 1100);

	//
	ret = regulator_enable(s5k4h7b->iovdd_regulator);
	if (ret < 0) {
		dev_err(dev, "Failed to enable IOVDD regulator\n");
		goto disable_clk;
	}
	ret = regulator_enable(s5k4h7b->dvdd_regulator);
	if (ret < 0) {
		dev_err(dev, "Failed to enable DVDD regulator\n");
		goto disable_iovdd;
	}
	ret = regulator_enable(s5k4h7b->avdd_regulator);
	if (ret < 0) {
		dev_err(dev, "Failed to enable AVDD regulator\n");
		goto disable_dvdd;
	}
	usleep_range(1000, 1100);
	clk_set_rate(s5k4h7b->xvclk, S5K4H7B_XVCLK_FREQ);
	gpiod_set_value_cansleep(s5k4h7b->pwdn_gpio, 1);
	gpiod_set_value_cansleep(s5k4h7b->reset_gpio, 1);
	usleep_range(10 * 1000, 11 * 1000);
	/* 8192 cycles prior to first SCCB transaction */
	delay_us = s5k4h7b_cal_delay(8192);
	usleep_range(delay_us, delay_us * 2);

	return 0;

disable_dvdd:
	regulator_disable(s5k4h7b->dvdd_regulator);
disable_iovdd:
	regulator_disable(s5k4h7b->iovdd_regulator);
disable_clk:
	clk_disable_unprepare(s5k4h7b->xvclk);

	return ret;
}

static void __s5k4h7b_power_off(struct s5k4h7b *s5k4h7b)
{
	int ret;
	/* 512 xvclk cycles after the last SCCB transaction or MIPI frame end */
	usleep_range(30, 50);

	gpiod_set_value_cansleep(s5k4h7b->reset_gpio, 0);
		gpiod_set_value_cansleep(s5k4h7b->pwdn_gpio, 0);
	clk_disable_unprepare(s5k4h7b->xvclk);
	if (!IS_ERR_OR_NULL(s5k4h7b->pins_sleep)) {
		ret = pinctrl_select_state(s5k4h7b->pinctrl,
					   s5k4h7b->pins_sleep);
		if (ret < 0)
			dev_dbg(&s5k4h7b->client->dev, "could not set pins pins_sleep\n");
	}
	usleep_range(1*1000, 2 * 1000);
	regulator_disable(s5k4h7b->avdd_regulator);
	regulator_disable(s5k4h7b->dvdd_regulator);
	regulator_disable(s5k4h7b->iovdd_regulator);

}

static int s5k4h7b_runtime_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct s5k4h7b *s5k4h7b = to_s5k4h7b(sd);

	return __s5k4h7b_power_on(s5k4h7b);
}

static int s5k4h7b_runtime_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct s5k4h7b *s5k4h7b = to_s5k4h7b(sd);

	__s5k4h7b_power_off(s5k4h7b);

	return 0;
}

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static int s5k4h7b_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct s5k4h7b *s5k4h7b = to_s5k4h7b(sd);
	struct v4l2_mbus_framefmt *try_fmt =
				v4l2_subdev_get_try_format(sd, fh->pad, 0);
	const struct s5k4h7b_mode *def_mode = &supported_modes[0];

	mutex_lock(&s5k4h7b->mutex);
	/* Initialize try_fmt */
	try_fmt->width = def_mode->width;
	try_fmt->height = def_mode->height;
	try_fmt->code = S5K4H7B_MEDIA_BUS_FMT;
	try_fmt->field = V4L2_FIELD_NONE;

	mutex_unlock(&s5k4h7b->mutex);
	/* No crop or compose */

	return 0;
}
#endif

static int s5k4h7b_enum_frame_interval(struct v4l2_subdev *sd,
				       struct v4l2_subdev_pad_config *cfg,
				       struct v4l2_subdev_frame_interval_enum *fie)
{
	if (fie->index >= ARRAY_SIZE(supported_modes))
		return -EINVAL;

	if (fie->code != S5K4H7B_MEDIA_BUS_FMT)
		return -EINVAL;

	fie->width = supported_modes[fie->index].width;
	fie->height = supported_modes[fie->index].height;
	fie->interval = supported_modes[fie->index].max_fps;

	return 0;
}

static int s5k4h7b_g_mbus_config(struct v4l2_subdev *sd, unsigned int pad,
				struct v4l2_mbus_config *config)
{
	if (2 == S5K4H7B_LANES) {
		config->type = V4L2_MBUS_CSI2_DPHY;
		config->flags = V4L2_MBUS_CSI2_2_LANE |
				V4L2_MBUS_CSI2_CHANNEL_0 |
				V4L2_MBUS_CSI2_CONTINUOUS_CLOCK;
	} else if (4 == S5K4H7B_LANES) {
		config->type = V4L2_MBUS_CSI2_DPHY;
		config->flags = V4L2_MBUS_CSI2_4_LANE |
				V4L2_MBUS_CSI2_CHANNEL_0 |
				V4L2_MBUS_CSI2_CONTINUOUS_CLOCK;
	}

	return 0;
}

#define CROP_START(SRC, DST) (((SRC) - (DST)) / 2 / 4 * 4)
#define DST_WIDTH_2096 2096
#define DST_HEIGHT_1560 1560

static int s5k4h7b_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_pad_config *cfg,
				struct v4l2_subdev_selection *sel)
{
	struct s5k4h7b *s5k4h7b = to_s5k4h7b(sd);

	if (sel->target == V4L2_SEL_TGT_CROP_BOUNDS) {
		if (s5k4h7b->cur_mode->width == 2104) {
			sel->r.left = CROP_START(s5k4h7b->cur_mode->width, DST_WIDTH_2096);
			sel->r.width = DST_WIDTH_2096;
			sel->r.top = CROP_START(s5k4h7b->cur_mode->height, DST_HEIGHT_1560);
			sel->r.height = DST_HEIGHT_1560;
		} else {
			sel->r.left = CROP_START(s5k4h7b->cur_mode->width,
							s5k4h7b->cur_mode->width);
			sel->r.width = s5k4h7b->cur_mode->width;
			sel->r.top = CROP_START(s5k4h7b->cur_mode->height,
							s5k4h7b->cur_mode->height);
			sel->r.height = s5k4h7b->cur_mode->height;
		}
		return 0;
	}

	return -EINVAL;
}

static const struct dev_pm_ops s5k4h7b_pm_ops = {
	SET_RUNTIME_PM_OPS(s5k4h7b_runtime_suspend,
			   s5k4h7b_runtime_resume, NULL)
};

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static const struct v4l2_subdev_internal_ops s5k4h7b_internal_ops = {
	.open = s5k4h7b_open,
};
#endif

static const struct v4l2_subdev_core_ops s5k4h7b_core_ops = {
	.s_power = s5k4h7b_s_power,
	.ioctl = s5k4h7b_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = s5k4h7b_compat_ioctl32,
#endif
};

static const struct v4l2_subdev_video_ops s5k4h7b_video_ops = {
	.s_stream = s5k4h7b_s_stream,
	.g_frame_interval = s5k4h7b_g_frame_interval,
};

static const struct v4l2_subdev_pad_ops s5k4h7b_pad_ops = {
	.enum_mbus_code = s5k4h7b_enum_mbus_code,
	.enum_frame_size = s5k4h7b_enum_frame_sizes,
	.enum_frame_interval = s5k4h7b_enum_frame_interval,
	.get_fmt = s5k4h7b_get_fmt,
	.set_fmt = s5k4h7b_set_fmt,
	.get_selection = s5k4h7b_get_selection,
	.get_mbus_config = s5k4h7b_g_mbus_config,
};

static const struct v4l2_subdev_ops s5k4h7b_subdev_ops = {
	.core	= &s5k4h7b_core_ops,
	.video	= &s5k4h7b_video_ops,
	.pad	= &s5k4h7b_pad_ops,
};

static int s5k4h7b_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct s5k4h7b *s5k4h7b = container_of(ctrl->handler,
					     struct s5k4h7b, ctrl_handler);
	struct i2c_client *client = s5k4h7b->client;
	s64 max;
	int ret = 0;

	/* Propagate change of current control to all related controls */
	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		/* Update max exposure while meeting expected vblanking */
		max = s5k4h7b->cur_mode->height + ctrl->val - 4;
		__v4l2_ctrl_modify_range(s5k4h7b->exposure,
					 s5k4h7b->exposure->minimum, max,
					 s5k4h7b->exposure->step,
					 s5k4h7b->exposure->default_value);
		break;
	}

	if (!pm_runtime_get_if_in_use(&client->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		/* 4 least significant bits of expsoure are fractional part */
		ret = s5k4h7b_write_reg(s5k4h7b->client,
					S5K4H7B_REG_EXPOSURE,
					S5K4H7B_REG_VALUE_16BIT,
					ctrl->val);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		ret = s5k4h7b_write_reg(s5k4h7b->client,
					S5K4H7B_REG_ANALOG_GAIN,
					S5K4H7B_REG_VALUE_16BIT,
					ctrl->val);
		break;
	case V4L2_CID_VBLANK:
		ret = s5k4h7b_write_reg(s5k4h7b->client,
					S5K4H7B_REG_VTS,
					S5K4H7B_REG_VALUE_16BIT,
					ctrl->val + s5k4h7b->cur_mode->height);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = s5k4h7b_enable_test_pattern(s5k4h7b, ctrl->val);
		break;
	default:
		dev_warn(&client->dev, "%s Unhandled id:0x%x, val:0x%x\n",
			 __func__, ctrl->id, ctrl->val);
		break;
	}

	pm_runtime_put(&client->dev);

	return ret;
}

static const struct v4l2_ctrl_ops s5k4h7b_ctrl_ops = {
	.s_ctrl = s5k4h7b_set_ctrl,
};

static int s5k4h7b_initialize_controls(struct s5k4h7b *s5k4h7b)
{
	const struct s5k4h7b_mode *mode;
	struct v4l2_ctrl_handler *handler;
	s64 exposure_max, vblank_def;
	u32 h_blank;
	int ret;
	u64 dst_pixel_rate = 0;
	u32 lane_num = S5K4H7B_LANES;

	handler = &s5k4h7b->ctrl_handler;
	mode = s5k4h7b->cur_mode;
	ret = v4l2_ctrl_handler_init(handler, 8);
	if (ret)
		return ret;
	handler->lock = &s5k4h7b->mutex;

	s5k4h7b->link_freq = v4l2_ctrl_new_int_menu(handler, NULL,
			V4L2_CID_LINK_FREQ,
			1, 0, link_freq_items);

	dst_pixel_rate = (u32)link_freq_items[mode->link_freq_idx] / mode->bpp * 2 * lane_num;

	s5k4h7b->pixel_rate = v4l2_ctrl_new_std(handler, NULL,
			V4L2_CID_PIXEL_RATE,
			0, S5K4H7B_PIXEL_RATE,
			1, dst_pixel_rate);

	__v4l2_ctrl_s_ctrl(s5k4h7b->link_freq,
			   mode->link_freq_idx);

	h_blank = mode->hts_def - mode->width;
	s5k4h7b->hblank = v4l2_ctrl_new_std(handler, NULL, V4L2_CID_HBLANK,
				h_blank, h_blank, 1, h_blank);
	if (s5k4h7b->hblank)
		s5k4h7b->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	vblank_def = mode->vts_def - mode->height;
	s5k4h7b->vblank = v4l2_ctrl_new_std(handler, &s5k4h7b_ctrl_ops,
				V4L2_CID_VBLANK, vblank_def,
				S5K4H7B_VTS_MAX - mode->height,
				1, vblank_def);

	exposure_max = mode->vts_def - 4;
	s5k4h7b->exposure = v4l2_ctrl_new_std(handler, &s5k4h7b_ctrl_ops,
				V4L2_CID_EXPOSURE, S5K4H7B_EXPOSURE_MIN,
				exposure_max, S5K4H7B_EXPOSURE_STEP,
				mode->exp_def);

	s5k4h7b->anal_gain = v4l2_ctrl_new_std(handler, &s5k4h7b_ctrl_ops,
				V4L2_CID_ANALOGUE_GAIN, S5K4H7B_GAIN_MIN,
				S5K4H7B_GAIN_MAX, S5K4H7B_GAIN_STEP,
				S5K4H7B_GAIN_DEFAULT);

	s5k4h7b->test_pattern = v4l2_ctrl_new_std_menu_items(handler,
				&s5k4h7b_ctrl_ops, V4L2_CID_TEST_PATTERN,
				ARRAY_SIZE(s5k4h7b_test_pattern_menu) - 1,
				0, 0, s5k4h7b_test_pattern_menu);

	if (handler->error) {
		ret = handler->error;
		dev_err(&s5k4h7b->client->dev,
			"Failed to init controls(%d)\n", ret);
		goto err_free_handler;
	}

	s5k4h7b->subdev.ctrl_handler = handler;

	return 0;

err_free_handler:
	v4l2_ctrl_handler_free(handler);

	return ret;
}

static int s5k4h7b_check_sensor_id(struct s5k4h7b *s5k4h7b,
				   struct i2c_client *client)
{
	struct device *dev = &s5k4h7b->client->dev;
	u32 id = 0;
	int ret;

	ret = s5k4h7b_read_reg(client, S5K4H7B_REG_CHIP_ID,
			       S5K4H7B_REG_VALUE_16BIT, &id);
	if (id != CHIP_ID) {
		dev_err(dev, "Unexpected sensor id(%04x), ret(%d)\n", id, ret);
		return -ENODEV;
	}
	dev_info(dev, "s5k4h7b probe success, snesor id: 0X%04x\n", id);
	return 0;
}

static int s5k4h7b_configure_regulators(struct s5k4h7b *s5k4h7b)
{
	unsigned int i;

	for (i = 0; i < S5K4H7B_NUM_SUPPLIES; i++)
		s5k4h7b->supplies[i].supply = s5k4h7b_supply_names[i];

	return devm_regulator_bulk_get(&s5k4h7b->client->dev,
				       S5K4H7B_NUM_SUPPLIES,
				       s5k4h7b->supplies);
}

static int s5k4h7b_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;
	struct s5k4h7b *s5k4h7b;
	struct v4l2_subdev *sd;
	char facing[2];
	int ret;

	dev_info(dev, "driver version: %02x.%02x.%02x",
		DRIVER_VERSION >> 16,
		(DRIVER_VERSION & 0xff00) >> 8,
		DRIVER_VERSION & 0x00ff);

	s5k4h7b = devm_kzalloc(dev, sizeof(*s5k4h7b), GFP_KERNEL);
	if (!s5k4h7b)
		return -ENOMEM;

	ret = of_property_read_u32(node, RKMODULE_CAMERA_MODULE_INDEX,
				   &s5k4h7b->module_index);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_FACING,
				       &s5k4h7b->module_facing);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_NAME,
				       &s5k4h7b->module_name);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_LENS_NAME,
				       &s5k4h7b->len_name);
	if (ret) {
		dev_err(dev, "could not get module information!\n");
		return -EINVAL;
	}

	s5k4h7b->client = client;
	s5k4h7b->cur_mode = &supported_modes[0];

	s5k4h7b->xvclk = devm_clk_get(dev, "xvclk");
	if (IS_ERR(s5k4h7b->xvclk)) {
		dev_err(dev, "Failed to get xvclk\n");
		return -EINVAL;
	}

	s5k4h7b->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(s5k4h7b->reset_gpio)) {
		dev_err(dev, "Failed to get reset-gpios\n");
		return -EINVAL;
	}

	s5k4h7b->pwdn_gpio = devm_gpiod_get(dev, "pwdn", GPIOD_OUT_LOW);
	if (IS_ERR(s5k4h7b->pwdn_gpio)) {
		dev_err(dev, "Failed to get pwdn-gpios\n");
		return -EINVAL;
	}

	s5k4h7b->avdd_regulator = devm_regulator_get(dev, "avdd");
	if (IS_ERR(s5k4h7b->avdd_regulator)) {
		dev_err(dev, "Failed to get avdd-supply\n");
		return -EINVAL;
	}

	s5k4h7b->iovdd_regulator = devm_regulator_get(dev, "iovdd");
	if (IS_ERR(s5k4h7b->iovdd_regulator)) {
		dev_err(dev, "Failed to get iovdd-supply\n");
		return -EINVAL;
	}

	s5k4h7b->dvdd_regulator = devm_regulator_get(dev, "dvdd");
	if (IS_ERR(s5k4h7b->dvdd_regulator)) {
		dev_err(dev, "Failed to get dvdd-supply\n");
		return -EINVAL;
	}

	s5k4h7b->pinctrl = devm_pinctrl_get(dev);
	if (!IS_ERR(s5k4h7b->pinctrl)) {
		s5k4h7b->pins_default =
			pinctrl_lookup_state(s5k4h7b->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_DEFAULT);
		if (IS_ERR(s5k4h7b->pins_default))
			dev_err(dev, "could not get default pinstate\n");

		s5k4h7b->pins_sleep =
			pinctrl_lookup_state(s5k4h7b->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_SLEEP);
		if (IS_ERR(s5k4h7b->pins_sleep))
			dev_err(dev, "could not get sleep pinstate\n");
	} else {
		dev_err(dev, "no pinctrl\n");
	}

	mutex_init(&s5k4h7b->mutex);

	sd = &s5k4h7b->subdev;
	v4l2_i2c_subdev_init(sd, client, &s5k4h7b_subdev_ops);
	ret = s5k4h7b_initialize_controls(s5k4h7b);
	if (ret)
		goto err_destroy_mutex;

	ret = __s5k4h7b_power_on(s5k4h7b);
	if (ret)
		goto err_free_handler;

	ret = s5k4h7b_check_sensor_id(s5k4h7b, client);
	if (ret)
		goto err_power_off;

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
	sd->internal_ops = &s5k4h7b_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
#endif
#if defined(CONFIG_MEDIA_CONTROLLER)
	s5k4h7b->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sd->entity, 1, &s5k4h7b->pad);
	if (ret < 0)
		goto err_power_off;
#endif

	memset(facing, 0, sizeof(facing));
	if (strcmp(s5k4h7b->module_facing, "back") == 0)
		facing[0] = 'b';
	else
		facing[0] = 'f';

	snprintf(sd->name, sizeof(sd->name), "m%02d_%s_%s %s",
		 s5k4h7b->module_index, facing,
		 S5K4H7B_NAME, dev_name(sd->dev));
	ret = v4l2_async_register_subdev_sensor_common(sd);
	if (ret) {
		dev_err(dev, "v4l2 async register subdev failed\n");
		goto err_clean_entity;
	}

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_idle(dev);

	return 0;

err_clean_entity:
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
err_power_off:
	__s5k4h7b_power_off(s5k4h7b);
err_free_handler:
	v4l2_ctrl_handler_free(&s5k4h7b->ctrl_handler);
err_destroy_mutex:
	mutex_destroy(&s5k4h7b->mutex);

	return ret;
}

static int s5k4h7b_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct s5k4h7b *s5k4h7b = to_s5k4h7b(sd);

	v4l2_async_unregister_subdev(sd);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
	v4l2_ctrl_handler_free(&s5k4h7b->ctrl_handler);
	mutex_destroy(&s5k4h7b->mutex);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		__s5k4h7b_power_off(s5k4h7b);
	pm_runtime_set_suspended(&client->dev);

	return 0;
}

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id s5k4h7b_of_match[] = {
	{ .compatible = "quec,s5k4h7b" },
	{},
};
MODULE_DEVICE_TABLE(of, s5k4h7b_of_match);
#endif

static const struct i2c_device_id s5k4h7b_match_id[] = {
	{ "quec,s5k4h7b", 0 },
	{ },
};

static struct i2c_driver s5k4h7b_i2c_driver = {
	.driver = {
		.name = S5K4H7B_NAME,
		.pm = &s5k4h7b_pm_ops,
		.of_match_table = of_match_ptr(s5k4h7b_of_match),
	},
	.probe		= &s5k4h7b_probe,
	.remove		= &s5k4h7b_remove,
	.id_table	= s5k4h7b_match_id,
};

static int __init sensor_mod_init(void)
{
	return i2c_add_driver(&s5k4h7b_i2c_driver);
}

static void __exit sensor_mod_exit(void)
{
	i2c_del_driver(&s5k4h7b_i2c_driver);
}

device_initcall_sync(sensor_mod_init);
module_exit(sensor_mod_exit);

MODULE_DESCRIPTION("quectel s5k4h7b sensor driver");
MODULE_LICENSE("GPL v2");
