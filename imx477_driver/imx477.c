/*
 * Driver for IMX477 CMOS Image Sensor from Sony
 * (Raspberry Pi High Quality Camera)
 *
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd
 * Adapted for Radxa/Rockchip platform from IMX219 Radxa driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of_graph.h>
#include <linux/slab.h>
#include <linux/videodev2.h>
#include <linux/version.h>
#include <linux/rk-camera-module.h>
#include <linux/compat.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-image-sizes.h>
#include <media/v4l2-mediabus.h>

#define DRIVER_VERSION			KERNEL_VERSION(0, 0x01, 0x1)

#define IMX477_TABLE_END		0xffff

/*
 * Analog gain: register 0x0204:0x0205, 16-bit, range [0, 978]
 * Sensor gain = 1024 / (1024 - reg_val)
 * HAL passes gain * 256, so 256 = 1x.
 * Max reg 978 → gain ≈ 22.26x → HAL value ≈ 5699
 */
#define IMX477_ANALOGUE_GAIN_MULTIPLIER	256
#define IMX477_ANALOGUE_GAIN_MIN	(1 * IMX477_ANALOGUE_GAIN_MULTIPLIER)
#define IMX477_ANALOGUE_GAIN_MAX	(22 * IMX477_ANALOGUE_GAIN_MULTIPLIER)
#define IMX477_ANALOGUE_GAIN_DEFAULT	(1 * IMX477_ANALOGUE_GAIN_MULTIPLIER)

/* Digital gain: 0x020E:0x020F, 0x0100 = 1x */
#define IMX477_DIGITAL_GAIN_MIN		256
#define IMX477_DIGITAL_GAIN_MAX		65535
#define IMX477_DIGITAL_GAIN_DEFAULT	256

#define IMX477_DIGITAL_EXPOSURE_MIN	4
#define IMX477_DIGITAL_EXPOSURE_MAX	65513	/* FRAME_LENGTH_MAX - EXPOSURE_OFFSET */
#define IMX477_DIGITAL_EXPOSURE_DEFAULT	0x640

/*
 * Lines to subtract from VTS when computing maximum coarse integration time.
 * Matches IMX477_EXPOSURE_OFFSET = 22 in the Raspberry Pi driver.
 */
#define IMX477_EXP_LINES_MARGIN		22

#define IMX477_NAME			"imx477"
#define IMX477_LANES			2

/* Pixel rate is 840 MHz for all modes (fixed by PLL settings). */
#define IMX477_PIXEL_RATE		840000000ULL

static const s64 link_freq_menu_items[] = {
	450000000,
};

struct imx477_reg {
	u16 addr;
	u8 val;
};

struct imx477_mode {
	u32 width;
	u32 height;
	struct v4l2_fract max_fps;
	u32 hts_def;
	u32 vts_def;
	const struct imx477_reg *reg_list;
};

/* ------------------------------------------------------------------ */
/* Common init registers (all modes).                                  */
/* Derived from mode_common_regs[] in the Raspberry Pi IMX477 driver.  */
/* ------------------------------------------------------------------ */
static const struct imx477_reg imx477_common_regs[] = {
	{0x0136, 0x18}, /* EXCK_FREQ[15:8] = 24 MHz */
	{0x0137, 0x00},
	{0x0138, 0x01}, /* temperature sensor enable */
	{0xe000, 0x00},
	{0xe07a, 0x01},
	{0x0808, 0x02}, /* DPHY_CTRL = register mode */
	{0x4ae9, 0x18},
	{0x4aea, 0x08},
	{0xf61c, 0x04},
	{0xf61e, 0x04},
	{0x4ae9, 0x21},
	{0x4aea, 0x80},
	{0x38a8, 0x1f},
	{0x38a9, 0xff},
	{0x38aa, 0x1f},
	{0x38ab, 0xff},
	{0x55d4, 0x00},
	{0x55d5, 0x00},
	{0x55d6, 0x07},
	{0x55d7, 0xff},
	{0x55e8, 0x07},
	{0x55e9, 0xff},
	{0x55ea, 0x00},
	{0x55eb, 0x00},
	{0x574c, 0x07},
	{0x574d, 0xff},
	{0x574e, 0x00},
	{0x574f, 0x00},
	{0x5754, 0x00},
	{0x5755, 0x00},
	{0x5756, 0x07},
	{0x5757, 0xff},
	{0x5973, 0x04},
	{0x5974, 0x01},
	{0x5d13, 0xc3},
	{0x5d14, 0x58},
	{0x5d15, 0xa3},
	{0x5d16, 0x1d},
	{0x5d17, 0x65},
	{0x5d18, 0x8c},
	{0x5d1a, 0x06},
	{0x5d1b, 0xa9},
	{0x5d1c, 0x45},
	{0x5d1d, 0x3a},
	{0x5d1e, 0xab},
	{0x5d1f, 0x15},
	{0x5d21, 0x0e},
	{0x5d22, 0x52},
	{0x5d23, 0xaa},
	{0x5d24, 0x7d},
	{0x5d25, 0x57},
	{0x5d26, 0xa8},
	{0x5d37, 0x5a},
	{0x5d38, 0x5a},
	{0x5d77, 0x7f},
	{0x7b75, 0x0e},
	{0x7b76, 0x0b},
	{0x7b77, 0x08},
	{0x7b78, 0x0a},
	{0x7b79, 0x47},
	{0x7b7c, 0x00},
	{0x7b7d, 0x00},
	{0x8d1f, 0x00},
	{0x8d27, 0x00},
	{0x9004, 0x03},
	{0x9200, 0x50},
	{0x9201, 0x6c},
	{0x9202, 0x71},
	{0x9203, 0x00},
	{0x9204, 0x71},
	{0x9205, 0x01},
	{0x9371, 0x6a},
	{0x9373, 0x6a},
	{0x9375, 0x64},
	{0x991a, 0x00},
	{0x996b, 0x8c},
	{0x996c, 0x64},
	{0x996d, 0x50},
	{0x9a4c, 0x0d},
	{0x9a4d, 0x0d},
	{0xa001, 0x0a},
	{0xa003, 0x0a},
	{0xa005, 0x0a},
	{0xa006, 0x01},
	{0xa007, 0xc0},
	{0xa009, 0xc0},
	{0x3d8a, 0x01},
	{0x4421, 0x04},
	{0x7b3b, 0x01},
	{0x7b4c, 0x00},
	{0x9905, 0x00},
	{0x9907, 0x00},
	{0x9909, 0x00},
	{0x990b, 0x00},
	{0x9944, 0x3c},
	{0x9947, 0x3c},
	{0x994a, 0x8c},
	{0x994b, 0x50},
	{0x994c, 0x1b},
	{0x994d, 0x8c},
	{0x994e, 0x50},
	{0x994f, 0x1b},
	{0x9950, 0x8c},
	{0x9951, 0x1b},
	{0x9952, 0x0a},
	{0x9953, 0x8c},
	{0x9954, 0x1b},
	{0x9955, 0x0a},
	{0x9a13, 0x04},
	{0x9a14, 0x04},
	{0x9a19, 0x00},
	{0x9a1c, 0x04},
	{0x9a1d, 0x04},
	{0x9a26, 0x05},
	{0x9a27, 0x05},
	{0x9a2c, 0x01},
	{0x9a2d, 0x03},
	{0x9a2f, 0x05},
	{0x9a30, 0x05},
	{0x9a41, 0x00},
	{0x9a46, 0x00},
	{0x9a47, 0x00},
	{0x9c17, 0x35},
	{0x9c1d, 0x31},
	{0x9c29, 0x50},
	{0x9c3b, 0x2f},
	{0x9c41, 0x6b},
	{0x9c47, 0x2d},
	{0x9c4d, 0x40},
	{0x9c6b, 0x00},
	{0x9c71, 0xc8},
	{0x9c73, 0x32},
	{0x9c75, 0x04},
	{0x9c7d, 0x2d},
	{0x9c83, 0x40},
	{0x9c94, 0x3f},
	{0x9c95, 0x3f},
	{0x9c96, 0x3f},
	{0x9c97, 0x00},
	{0x9c98, 0x00},
	{0x9c99, 0x00},
	{0x9c9a, 0x3f},
	{0x9c9b, 0x3f},
	{0x9c9c, 0x3f},
	{0x9ca0, 0x0f},
	{0x9ca1, 0x0f},
	{0x9ca2, 0x0f},
	{0x9ca3, 0x00},
	{0x9ca4, 0x00},
	{0x9ca5, 0x00},
	{0x9ca6, 0x1e},
	{0x9ca7, 0x1e},
	{0x9ca8, 0x1e},
	{0x9ca9, 0x00},
	{0x9caa, 0x00},
	{0x9cab, 0x00},
	{0x9cac, 0x09},
	{0x9cad, 0x09},
	{0x9cae, 0x09},
	{0x9cbd, 0x50},
	{0x9cbf, 0x50},
	{0x9cc1, 0x50},
	{0x9cc3, 0x40},
	{0x9cc5, 0x40},
	{0x9cc7, 0x40},
	{0x9cc9, 0x0a},
	{0x9ccb, 0x0a},
	{0x9ccd, 0x0a},
	{0x9d17, 0x35},
	{0x9d1d, 0x31},
	{0x9d29, 0x50},
	{0x9d3b, 0x2f},
	{0x9d41, 0x6b},
	{0x9d47, 0x42},
	{0x9d4d, 0x5a},
	{0x9d6b, 0x00},
	{0x9d71, 0xc8},
	{0x9d73, 0x32},
	{0x9d75, 0x04},
	{0x9d7d, 0x42},
	{0x9d83, 0x5a},
	{0x9d94, 0x3f},
	{0x9d95, 0x3f},
	{0x9d96, 0x3f},
	{0x9d97, 0x00},
	{0x9d98, 0x00},
	{0x9d99, 0x00},
	{0x9d9a, 0x3f},
	{0x9d9b, 0x3f},
	{0x9d9c, 0x3f},
	{0x9d9d, 0x1f},
	{0x9d9e, 0x1f},
	{0x9d9f, 0x1f},
	{0x9da0, 0x0f},
	{0x9da1, 0x0f},
	{0x9da2, 0x0f},
	{0x9da3, 0x00},
	{0x9da4, 0x00},
	{0x9da5, 0x00},
	{0x9da6, 0x1e},
	{0x9da7, 0x1e},
	{0x9da8, 0x1e},
	{0x9da9, 0x00},
	{0x9daa, 0x00},
	{0x9dab, 0x00},
	{0x9dac, 0x09},
	{0x9dad, 0x09},
	{0x9dae, 0x09},
	{0x9dc9, 0x0a},
	{0x9dcb, 0x0a},
	{0x9dcd, 0x0a},
	{0x9e17, 0x35},
	{0x9e1d, 0x31},
	{0x9e29, 0x50},
	{0x9e3b, 0x2f},
	{0x9e41, 0x6b},
	{0x9e47, 0x2d},
	{0x9e4d, 0x40},
	{0x9e6b, 0x00},
	{0x9e71, 0xc8},
	{0x9e73, 0x32},
	{0x9e75, 0x04},
	{0x9e94, 0x0f},
	{0x9e95, 0x0f},
	{0x9e96, 0x0f},
	{0x9e97, 0x00},
	{0x9e98, 0x00},
	{0x9e99, 0x00},
	{0x9ea0, 0x0f},
	{0x9ea1, 0x0f},
	{0x9ea2, 0x0f},
	{0x9ea3, 0x00},
	{0x9ea4, 0x00},
	{0x9ea5, 0x00},
	{0x9ea6, 0x3f},
	{0x9ea7, 0x3f},
	{0x9ea8, 0x3f},
	{0x9ea9, 0x00},
	{0x9eaa, 0x00},
	{0x9eab, 0x00},
	{0x9eac, 0x09},
	{0x9ead, 0x09},
	{0x9eae, 0x09},
	{0x9ec9, 0x0a},
	{0x9ecb, 0x0a},
	{0x9ecd, 0x0a},
	{0x9f17, 0x35},
	{0x9f1d, 0x31},
	{0x9f29, 0x50},
	{0x9f3b, 0x2f},
	{0x9f41, 0x6b},
	{0x9f47, 0x42},
	{0x9f4d, 0x5a},
	{0x9f6b, 0x00},
	{0x9f71, 0xc8},
	{0x9f73, 0x32},
	{0x9f75, 0x04},
	{0x9f94, 0x0f},
	{0x9f95, 0x0f},
	{0x9f96, 0x0f},
	{0x9f97, 0x00},
	{0x9f98, 0x00},
	{0x9f99, 0x00},
	{0x9f9a, 0x2f},
	{0x9f9b, 0x2f},
	{0x9f9c, 0x2f},
	{0x9f9d, 0x00},
	{0x9f9e, 0x00},
	{0x9f9f, 0x00},
	{0x9fa0, 0x0f},
	{0x9fa1, 0x0f},
	{0x9fa2, 0x0f},
	{0x9fa3, 0x00},
	{0x9fa4, 0x00},
	{0x9fa5, 0x00},
	{0x9fa6, 0x1e},
	{0x9fa7, 0x1e},
	{0x9fa8, 0x1e},
	{0x9fa9, 0x00},
	{0x9faa, 0x00},
	{0x9fab, 0x00},
	{0x9fac, 0x09},
	{0x9fad, 0x09},
	{0x9fae, 0x09},
	{0x9fc9, 0x0a},
	{0x9fcb, 0x0a},
	{0x9fcd, 0x0a},
	{0xa14b, 0xff},
	{0xa151, 0x0c},
	{0xa153, 0x50},
	{0xa155, 0x02},
	{0xa157, 0x00},
	{0xa1ad, 0xff},
	{0xa1b3, 0x0c},
	{0xa1b5, 0x50},
	{0xa1b9, 0x00},
	{0xa24b, 0xff},
	{0xa257, 0x00},
	{0xa2ad, 0xff},
	{0xa2b9, 0x00},
	{0xb21f, 0x04},
	{0xb35c, 0x00},
	{0xb35e, 0x08},
	{0x0114, 0x01}, /* CSI_LANE_MODE: 2 lanes */
	{0x0350, 0x00},
	{0xbcf1, 0x02},
	{0x3ff9, 0x01},
	{0x0220, 0x00},
	{0x0221, 0x11},
	{0x0381, 0x01},
	{0x0383, 0x01},
	{0x0385, 0x01},
	{0x0387, 0x01},
	{0x0902, 0x02},
	{0x3140, 0x02},
	{0x3c00, 0x00},
	{0x9e9a, 0x2f},
	{0x9e9b, 0x2f},
	{0x9e9c, 0x2f},
	{0x9e9d, 0x00},
	{0x9e9e, 0x00},
	{0x9e9f, 0x00},
	{0x0301, 0x05},
	{0x0303, 0x02},
	{0x030b, 0x02}, /* IOP_SYSCK_DIV */
	{0x030d, 0x02}, /* IOP_PREDIV */
	{0x0310, 0x01},
	{0x080a, 0x00},
	{0x080b, 0x7f},
	{0x080c, 0x00},
	{0x080d, 0x4f},
	{0x080e, 0x00},
	{0x080f, 0x77},
	{0x0810, 0x00},
	{0x0811, 0x5f},
	{0x0812, 0x00},
	{0x0813, 0x57},
	{0x0814, 0x00},
	{0x0815, 0x4f},
	{0x0816, 0x01},
	{0x0817, 0x27},
	{0x0818, 0x00},
	{0x0819, 0x3f},
	{0x3e20, 0x01},
	{0x3e37, 0x00},
	{0x3f50, 0x00},
	{IMX477_TABLE_END, 0x00},
};

/* ------------------------------------------------------------------ */
/* Mode: 4056x3040, full resolution, ~10 fps                           */
/* hts = 840MHz / (10 * 3500) = 24000 = 0x5DC0                        */
/* ------------------------------------------------------------------ */
static const struct imx477_reg mode_4056x3040_regs[] = {
	{0x0344, 0x00}, /* X_ADD_STA = 0 */
	{0x0345, 0x00},
	{0x0346, 0x00}, /* Y_ADD_STA = 0 */
	{0x0347, 0x00},
	{0x0348, 0x0f}, /* X_ADD_END = 4055 */
	{0x0349, 0xd7},
	{0x034a, 0x0b}, /* Y_ADD_END = 3039 */
	{0x034b, 0xdf},
	{0x00e3, 0x00},
	{0x00e4, 0x00},
	{0x00fc, 0x0a},
	{0x00fd, 0x0a},
	{0x00fe, 0x0a},
	{0x00ff, 0x0a},
	{0x0900, 0x00}, /* BINNING_MODE = off */
	{0x0901, 0x11},
	{0x3c01, 0x03},
	{0x3c02, 0xa2},
	{0x5748, 0x07},
	{0x5749, 0xff},
	{0x574a, 0x00},
	{0x574b, 0x00},
	{0x7b75, 0x0a},
	{0x7b76, 0x0c},
	{0x7b77, 0x07},
	{0x7b78, 0x06},
	{0x7b79, 0x3c},
	{0x7b53, 0x01},
	{0x9369, 0x5a},
	{0x936b, 0x55},
	{0x936d, 0x28},
	{0x9304, 0x00},
	{0x9305, 0x00},
	{0xa2a9, 0x60},
	{0xa2b7, 0x00},
	{0x0401, 0x00},
	{0x0404, 0x00},
	{0x0405, 0x10},
	{0x0408, 0x00},
	{0x0409, 0x00},
	{0x040a, 0x00},
	{0x040b, 0x00},
	{0x040c, 0x0f}, /* digital_crop_image_width = 4056 */
	{0x040d, 0xd8},
	{0x040e, 0x0b}, /* digital_crop_image_height = 3040 */
	{0x040f, 0xe0},
	{0x034c, 0x0f}, /* X_OUTPUT_SIZE = 4056 */
	{0x034d, 0xd8},
	{0x034e, 0x0b}, /* Y_OUTPUT_SIZE = 3040 */
	{0x034f, 0xe0},
	{0x0305, 0x04},
	{0x0306, 0x01},
	{0x0307, 0x5e},
	{0xe04c, 0x00},
	{0xe04d, 0x7f},
	{0xe04e, 0x00},
	{0xe04f, 0x1f},
	{0x3f56, 0x02},
	{0x3f57, 0xae},
	{IMX477_TABLE_END, 0x00},
};

/* ------------------------------------------------------------------ */
/* Mode: 2028x1520, 2x2 binned, ~40 fps                               */
/* hts = 840MHz / (40 * 2197) = 9573 = 0x2565                        */
/* ------------------------------------------------------------------ */
static const struct imx477_reg mode_2028x1520_regs[] = {
	{0x0344, 0x00},
	{0x0345, 0x00},
	{0x0346, 0x00},
	{0x0347, 0x00},
	{0x0348, 0x0f},
	{0x0349, 0xd7},
	{0x034a, 0x0b},
	{0x034b, 0xdf},
	{0x0900, 0x01}, /* BINNING_MODE = on */
	{0x0901, 0x22}, /* 2x2 binning */
	{0x3c01, 0x03},
	{0x3c02, 0xa2},
	{0x5748, 0x07},
	{0x5749, 0xff},
	{0x574a, 0x00},
	{0x574b, 0x00},
	{0x7b53, 0x01},
	{0x9369, 0x73},
	{0x936b, 0x64},
	{0x936d, 0x5f},
	{0x9304, 0x00},
	{0x9305, 0x00},
	{0xa2a9, 0x60},
	{0xa2b7, 0x00},
	{0x0401, 0x00},
	{0x0404, 0x00},
	{0x0405, 0x20},
	{0x0408, 0x00},
	{0x0409, 0x00},
	{0x040a, 0x00},
	{0x040b, 0x00},
	{0x040c, 0x0f},
	{0x040d, 0xd8},
	{0x040e, 0x0b},
	{0x040f, 0xe0},
	{0x034c, 0x07}, /* X_OUTPUT_SIZE = 2028 */
	{0x034d, 0xec},
	{0x034e, 0x05}, /* Y_OUTPUT_SIZE = 1520 */
	{0x034f, 0xf0},
	{0x0305, 0x04},
	{0x0306, 0x01},
	{0x0307, 0x5e},
	{0xe04c, 0x00},
	{0xe04d, 0x7f},
	{0xe04e, 0x00},
	{0xe04f, 0x1f},
	{0x3f56, 0x01},
	{0x3f57, 0x6c},
	{IMX477_TABLE_END, 0x00},
};

/* ------------------------------------------------------------------ */
/* Mode: 2028x1080, 2x2 binned 16:9 crop, ~50 fps                    */
/* hts = 840MHz / (50 * 2197) = 7658 = 0x1DEA                        */
/* ------------------------------------------------------------------ */
static const struct imx477_reg mode_2028x1080_regs[] = {
	{0x0344, 0x00},
	{0x0345, 0x00},
	{0x0346, 0x01}, /* Y_ADD_STA = 440 */
	{0x0347, 0xb8},
	{0x0348, 0x0f},
	{0x0349, 0xd7},
	{0x034a, 0x0a}, /* Y_ADD_END = 2599 */
	{0x034b, 0x27},
	{0x0900, 0x01},
	{0x0901, 0x22},
	{0x3c01, 0x03},
	{0x3c02, 0xa2},
	{0x5748, 0x07},
	{0x5749, 0xff},
	{0x574a, 0x00},
	{0x574b, 0x00},
	{0x7b53, 0x01},
	{0x9369, 0x73},
	{0x936b, 0x64},
	{0x936d, 0x5f},
	{0x9304, 0x00},
	{0x9305, 0x00},
	{0xa2a9, 0x60},
	{0xa2b7, 0x00},
	{0x0401, 0x00},
	{0x0404, 0x00},
	{0x0405, 0x20},
	{0x0408, 0x00},
	{0x0409, 0x00},
	{0x040a, 0x00},
	{0x040b, 0x00},
	{0x040c, 0x0f},
	{0x040d, 0xd8},
	{0x040e, 0x04}, /* digital_crop_image_height = 1080 */
	{0x040f, 0x38},
	{0x034c, 0x07}, /* X_OUTPUT_SIZE = 2028 */
	{0x034d, 0xec},
	{0x034e, 0x04}, /* Y_OUTPUT_SIZE = 1080 */
	{0x034f, 0x38},
	{0x0305, 0x04},
	{0x0306, 0x01},
	{0x0307, 0x5e},
	{0xe04c, 0x00},
	{0xe04d, 0x7f},
	{0xe04e, 0x00},
	{0xe04f, 0x1f},
	{0x3f56, 0x01},
	{0x3f57, 0x6c},
	{IMX477_TABLE_END, 0x00},
};

/* ------------------------------------------------------------------ */
/* Mode: 1332x990, 2x2 binned + cropped, ~120 fps                     */
/* hts = 840MHz / (120 * 1050) = 6667 = 0x1A0B                       */
/* ------------------------------------------------------------------ */
static const struct imx477_reg mode_1332x990_regs[] = {
	{0x420b, 0x01},
	{0x990c, 0x00},
	{0x990d, 0x08},
	{0x9956, 0x8c},
	{0x9957, 0x64},
	{0x9958, 0x50},
	{0x9a48, 0x06},
	{0x9a49, 0x06},
	{0x9a4a, 0x06},
	{0x9a4b, 0x06},
	{0x9a4c, 0x06},
	{0x9a4d, 0x06},
	{0x0114, 0x01},
	{0x0344, 0x00},
	{0x0345, 0x00},
	{0x0346, 0x02}, /* Y_ADD_STA = 528 */
	{0x0347, 0x10},
	{0x0348, 0x0f},
	{0x0349, 0xd7},
	{0x034a, 0x09}, /* Y_ADD_END = 2511 */
	{0x034b, 0xcf},
	{0x00e3, 0x00},
	{0x00e4, 0x00},
	{0x00fc, 0x0a},
	{0x00fd, 0x0a},
	{0x00fe, 0x0a},
	{0x00ff, 0x0a},
	{0xe013, 0x00},
	{0x0220, 0x00},
	{0x0221, 0x11},
	{0x0381, 0x01},
	{0x0383, 0x01},
	{0x0385, 0x01},
	{0x0387, 0x01},
	{0x0900, 0x01},
	{0x0901, 0x22},
	{0x0902, 0x02},
	{0x3140, 0x02},
	{0x3c00, 0x00},
	{0x3c01, 0x01},
	{0x3c02, 0x9c},
	{0x5748, 0x00},
	{0x5749, 0x00},
	{0x574a, 0x00},
	{0x574b, 0xa4},
	{0x7b75, 0x0e},
	{0x7b76, 0x09},
	{0x7b77, 0x08},
	{0x7b78, 0x06},
	{0x7b79, 0x34},
	{0x7b53, 0x00},
	{0x9369, 0x73},
	{0x936b, 0x64},
	{0x936d, 0x5f},
	{0x9304, 0x03},
	{0x9305, 0x80},
	{0xa2a9, 0x27},
	{0xa2b7, 0x03},
	{0x0401, 0x00},
	{0x0404, 0x00},
	{0x0405, 0x10},
	{0x0408, 0x01},
	{0x0409, 0x5c},
	{0x040a, 0x00},
	{0x040b, 0x00},
	{0x040c, 0x05}, /* digital_crop_image_width = 1332 */
	{0x040d, 0x34},
	{0x040e, 0x03}, /* digital_crop_image_height = 990 */
	{0x040f, 0xde},
	{0x034c, 0x05}, /* X_OUTPUT_SIZE = 1332 */
	{0x034d, 0x34},
	{0x034e, 0x03}, /* Y_OUTPUT_SIZE = 990 */
	{0x034f, 0xde},
	{0x0305, 0x02},
	{0x0306, 0x00},
	{0x0307, 0xaf},
	{0xe04c, 0x00},
	{0xe04d, 0x5f},
	{0xe04e, 0x00},
	{0xe04f, 0x1f},
	{0x3f56, 0x00},
	{0x3f57, 0xbf},
	{IMX477_TABLE_END, 0x00},
};

static const struct imx477_reg imx477_start[] = {
	{0x0100, 0x01},		/* mode select streaming on */
	{IMX477_TABLE_END, 0x00}
};

static const struct imx477_reg imx477_stop[] = {
	{0x0100, 0x00},		/* mode select streaming off */
	{IMX477_TABLE_END, 0x00}
};

enum {
	TEST_PATTERN_DISABLED,
	TEST_PATTERN_SOLID_BLACK,
	TEST_PATTERN_SOLID_WHITE,
	TEST_PATTERN_SOLID_RED,
	TEST_PATTERN_SOLID_GREEN,
	TEST_PATTERN_SOLID_BLUE,
	TEST_PATTERN_COLOR_BAR,
	TEST_PATTERN_FADE_TO_GREY_COLOR_BAR,
	TEST_PATTERN_PN9,
	TEST_PATTERN_MAX
};

static const char *const tp_qmenu[] = {
	"Disabled",
	"Solid Black",
	"Solid White",
	"Solid Red",
	"Solid Green",
	"Solid Blue",
	"Color Bar",
	"Fade to Grey Color Bar",
	"PN9",
};

struct imx477 {
	struct v4l2_subdev subdev;
	struct media_pad pad;
	struct v4l2_ctrl_handler ctrl_handler;
	struct clk *clk;
	struct v4l2_rect crop_rect;
	int hflip;
	int vflip;
	u16 analogue_gain;	/* IMX477 reg value [0, 978] */
	u16 digital_gain;	/* 0x0100 = 1x */
	u16 exposure_time;
	u16 test_pattern;
	u16 test_pattern_solid_color_r;
	u16 test_pattern_solid_color_gr;
	u16 test_pattern_solid_color_b;
	u16 test_pattern_solid_color_gb;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *pixel_rate;
	const struct imx477_mode *cur_mode;
	u32 cfg_num;
	u16 cur_vts;
	u32 module_index;
	const char *module_facing;
	const char *module_name;
	const char *len_name;
};

static const struct imx477_mode supported_modes[] = {
	{
		/* 2x2 binned 40fps — default mode */
		.width = 2028,
		.height = 1520,
		.max_fps = {
			.numerator = 10000,
			.denominator = 400000,
		},
		.hts_def = 9573,
		.vts_def = 2197,
		.reg_list = mode_2028x1520_regs,
	},
	{
		/* Full resolution ~10fps */
		.width = 4056,
		.height = 3040,
		.max_fps = {
			.numerator = 10000,
			.denominator = 100000,
		},
		.hts_def = 24000,
		.vts_def = 3500,
		.reg_list = mode_4056x3040_regs,
	},
	{
		/* 2x2 binned 1080p ~50fps */
		.width = 2028,
		.height = 1080,
		.max_fps = {
			.numerator = 10000,
			.denominator = 500000,
		},
		.hts_def = 7658,
		.vts_def = 2197,
		.reg_list = mode_2028x1080_regs,
	},
	{
		/* 2x2 binned + cropped 120fps */
		.width = 1332,
		.height = 990,
		.max_fps = {
			.numerator = 10000,
			.denominator = 1200000,
		},
		.hts_def = 6667,
		.vts_def = 1050,
		.reg_list = mode_1332x990_regs,
	},
};

static struct imx477 *to_imx477(const struct i2c_client *client)
{
	return container_of(i2c_get_clientdata(client), struct imx477, subdev);
}

static int reg_write(struct i2c_client *client, const u16 addr, const u8 data)
{
	struct i2c_adapter *adap = client->adapter;
	struct i2c_msg msg;
	u8 tx[3];
	int ret;

	msg.addr = client->addr;
	msg.buf = tx;
	msg.len = 3;
	msg.flags = 0;
	tx[0] = addr >> 8;
	tx[1] = addr & 0xff;
	tx[2] = data;
	ret = i2c_transfer(adap, &msg, 1);
	mdelay(2);

	return ret == 1 ? 0 : -EIO;
}

static int reg_read(struct i2c_client *client, const u16 addr)
{
	u8 buf[2] = {addr >> 8, addr & 0xff};
	int ret;
	struct i2c_msg msgs[] = {
		{
			.addr  = client->addr,
			.flags = 0,
			.len   = 2,
			.buf   = buf,
		}, {
			.addr  = client->addr,
			.flags = I2C_M_RD,
			.len   = 1,
			.buf   = buf,
		},
	};

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret < 0) {
		dev_warn(&client->dev, "Reading register %x from %x failed\n",
			 addr, client->addr);
		return ret;
	}

	return buf[0];
}

static int reg_write_table(struct i2c_client *client,
			   const struct imx477_reg table[])
{
	const struct imx477_reg *reg;
	int ret;

	for (reg = table; reg->addr != IMX477_TABLE_END; reg++) {
		ret = reg_write(client, reg->addr, reg->val);
		if (ret < 0)
			return ret;
	}

	return 0;
}

/* V4L2 subdev video operations */
static int imx477_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct imx477 *priv = to_imx477(client);
	u8 reg = 0x00;
	int ret;

	if (!enable)
		return reg_write_table(client, imx477_stop);

	/* Common init registers */
	ret = reg_write_table(client, imx477_common_regs);
	if (ret)
		return ret;

	/* Mode-specific registers */
	ret = reg_write_table(client, priv->cur_mode->reg_list);
	if (ret)
		return ret;

	/* RAW12 output format */
	ret = reg_write(client, 0x0112, 0x0c);
	ret |= reg_write(client, 0x0113, 0x0c);
	if (ret)
		return ret;

	/* Line length (HTS) */
	ret = reg_write(client, 0x0342, (priv->cur_mode->hts_def >> 8) & 0xff);
	ret |= reg_write(client, 0x0343, priv->cur_mode->hts_def & 0xff);
	if (ret)
		return ret;

	/* Frame length (VTS) */
	priv->cur_vts = priv->cur_mode->vts_def;
	ret = reg_write(client, 0x0340, (priv->cur_vts >> 8) & 0xff);
	ret |= reg_write(client, 0x0341, priv->cur_vts & 0xff);
	if (ret)
		return ret;

	/* Image orientation (flip/mirror) */
	if (priv->hflip)
		reg |= 0x1;
	if (priv->vflip)
		reg |= 0x2;
	ret = reg_write(client, 0x0101, reg);
	if (ret)
		return ret;

	/* Test pattern */
	if (priv->test_pattern) {
		ret = reg_write(client, 0x0600, priv->test_pattern >> 8);
		ret |= reg_write(client, 0x0601, priv->test_pattern & 0xff);
		ret |= reg_write(client, 0x0602,
				 priv->test_pattern_solid_color_r >> 8);
		ret |= reg_write(client, 0x0603,
				 priv->test_pattern_solid_color_r & 0xff);
		ret |= reg_write(client, 0x0604,
				 priv->test_pattern_solid_color_gr >> 8);
		ret |= reg_write(client, 0x0605,
				 priv->test_pattern_solid_color_gr & 0xff);
		ret |= reg_write(client, 0x0606,
				 priv->test_pattern_solid_color_b >> 8);
		ret |= reg_write(client, 0x0607,
				 priv->test_pattern_solid_color_b & 0xff);
		ret |= reg_write(client, 0x0608,
				 priv->test_pattern_solid_color_gb >> 8);
		ret |= reg_write(client, 0x0609,
				 priv->test_pattern_solid_color_gb & 0xff);
	} else {
		ret = reg_write(client, 0x0600, 0x00);
		ret |= reg_write(client, 0x0601, 0x00);
	}

	if (ret)
		return ret;

	return reg_write_table(client, imx477_start);
}

/* V4L2 subdev core operations */
static int imx477_s_power(struct v4l2_subdev *sd, int on)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct imx477 *priv = to_imx477(client);

	if (on) {
		dev_dbg(&client->dev, "imx477 power on\n");
		clk_prepare_enable(priv->clk);
	} else {
		dev_dbg(&client->dev, "imx477 power off\n");
		clk_disable_unprepare(priv->clk);
	}

	return 0;
}

/* V4L2 ctrl operations */
static int imx477_s_ctrl_test_pattern(struct v4l2_ctrl *ctrl)
{
	struct imx477 *priv =
	    container_of(ctrl->handler, struct imx477, ctrl_handler);

	switch (ctrl->val) {
	case TEST_PATTERN_DISABLED:
		priv->test_pattern = 0x0000;
		break;
	case TEST_PATTERN_SOLID_BLACK:
		priv->test_pattern = 0x0001;
		priv->test_pattern_solid_color_r = 0x0000;
		priv->test_pattern_solid_color_gr = 0x0000;
		priv->test_pattern_solid_color_b = 0x0000;
		priv->test_pattern_solid_color_gb = 0x0000;
		break;
	case TEST_PATTERN_SOLID_WHITE:
		priv->test_pattern = 0x0001;
		priv->test_pattern_solid_color_r = 0x0fff;
		priv->test_pattern_solid_color_gr = 0x0fff;
		priv->test_pattern_solid_color_b = 0x0fff;
		priv->test_pattern_solid_color_gb = 0x0fff;
		break;
	case TEST_PATTERN_SOLID_RED:
		priv->test_pattern = 0x0001;
		priv->test_pattern_solid_color_r = 0x0fff;
		priv->test_pattern_solid_color_gr = 0x0000;
		priv->test_pattern_solid_color_b = 0x0000;
		priv->test_pattern_solid_color_gb = 0x0000;
		break;
	case TEST_PATTERN_SOLID_GREEN:
		priv->test_pattern = 0x0001;
		priv->test_pattern_solid_color_r = 0x0000;
		priv->test_pattern_solid_color_gr = 0x0fff;
		priv->test_pattern_solid_color_b = 0x0000;
		priv->test_pattern_solid_color_gb = 0x0fff;
		break;
	case TEST_PATTERN_SOLID_BLUE:
		priv->test_pattern = 0x0001;
		priv->test_pattern_solid_color_r = 0x0000;
		priv->test_pattern_solid_color_gr = 0x0000;
		priv->test_pattern_solid_color_b = 0x0fff;
		priv->test_pattern_solid_color_gb = 0x0000;
		break;
	case TEST_PATTERN_COLOR_BAR:
		priv->test_pattern = 0x0002;
		break;
	case TEST_PATTERN_FADE_TO_GREY_COLOR_BAR:
		priv->test_pattern = 0x0003;
		break;
	case TEST_PATTERN_PN9:
		priv->test_pattern = 0x0004;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int imx477_g_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct imx477 *priv = to_imx477(client);
	const struct imx477_mode *mode = priv->cur_mode;

	fi->interval = mode->max_fps;

	return 0;
}

static int imx477_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct imx477 *priv =
	    container_of(ctrl->handler, struct imx477, ctrl_handler);
	struct i2c_client *client = v4l2_get_subdevdata(&priv->subdev);
	u8 reg;
	int ret;
	u32 gain;

	switch (ctrl->id) {
	case V4L2_CID_HFLIP:
		priv->hflip = ctrl->val;
		break;

	case V4L2_CID_VFLIP:
		priv->vflip = ctrl->val;
		break;

	case V4L2_CID_ANALOGUE_GAIN:
	case V4L2_CID_GAIN:
		/*
		 * HAL passes gain * 256 (256 = 1x).
		 * IMX477 analog gain register: reg = 1024 - 1024*256/gain_256
		 * Range: reg [0, 978] → gain [1x, ~22.3x]
		 */
		gain = ctrl->val;
		if (gain < 256)
			gain = 256;
		if (gain > IMX477_ANALOGUE_GAIN_MAX)
			gain = IMX477_ANALOGUE_GAIN_MAX;

		priv->analogue_gain = (u16)(1024 - (1024u * 256u) / gain);
		if (priv->analogue_gain > 978)
			priv->analogue_gain = 978;

		ret = reg_write(client, 0x0204, priv->analogue_gain >> 8);
		ret |= reg_write(client, 0x0205, priv->analogue_gain & 0xff);
		return ret;

	case V4L2_CID_EXPOSURE:
		priv->exposure_time = ctrl->val;

		ret = reg_write(client, 0x0202, priv->exposure_time >> 8);
		ret |= reg_write(client, 0x0203, priv->exposure_time & 0xff);
		return ret;

	case V4L2_CID_TEST_PATTERN:
		return imx477_s_ctrl_test_pattern(ctrl);

	case V4L2_CID_VBLANK:
		if (ctrl->val < priv->cur_mode->vts_def)
			ctrl->val = priv->cur_mode->vts_def;
		priv->cur_vts = ctrl->val;
		ret = reg_write(client, 0x0340, (priv->cur_vts >> 8) & 0xff);
		ret |= reg_write(client, 0x0341, priv->cur_vts & 0xff);
		return ret;

	default:
		return -EINVAL;
	}

	/* If streaming, apply settings immediately */
	reg = reg_read(client, 0x0100);
	if ((reg & 0x1f) == 0x01)
		imx477_s_stream(&priv->subdev, 1);

	return 0;
}

static int imx477_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index != 0)
		return -EINVAL;
	code->code = MEDIA_BUS_FMT_SRGGB12_1X12;

	return 0;
}

static int imx477_get_reso_dist(const struct imx477_mode *mode,
				struct v4l2_mbus_framefmt *framefmt)
{
	return abs(mode->width - framefmt->width) +
	       abs(mode->height - framefmt->height);
}

static const struct imx477_mode *imx477_find_best_fit(
					struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *framefmt = &fmt->format;
	int dist;
	int cur_best_fit = 0;
	int cur_best_fit_dist = -1;
	int i;

	for (i = 0; i < ARRAY_SIZE(supported_modes); i++) {
		dist = imx477_get_reso_dist(&supported_modes[i], framefmt);
		if (cur_best_fit_dist == -1 || dist < cur_best_fit_dist) {
			cur_best_fit_dist = dist;
			cur_best_fit = i;
		}
	}

	return &supported_modes[cur_best_fit];
}

static int imx477_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct imx477 *priv = to_imx477(client);
	const struct imx477_mode *mode;
	s64 h_blank, v_blank, pixel_rate;
	u32 fps = 0;

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY)
		return 0;

	mode = imx477_find_best_fit(fmt);
	fmt->format.code = MEDIA_BUS_FMT_SRGGB12_1X12;
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	priv->cur_mode = mode;
	h_blank = mode->hts_def - mode->width;
	__v4l2_ctrl_modify_range(priv->hblank, h_blank, h_blank, 1, h_blank);
	v_blank = mode->vts_def - mode->height;
	__v4l2_ctrl_modify_range(priv->vblank, v_blank, v_blank, 1, v_blank);
	fps = DIV_ROUND_CLOSEST(mode->max_fps.denominator,
		mode->max_fps.numerator);
	pixel_rate = mode->vts_def * mode->hts_def * fps;
	__v4l2_ctrl_modify_range(priv->pixel_rate, pixel_rate,
					pixel_rate, 1, pixel_rate);

	/* Center crop window on the 4056x3040 pixel array */
	priv->crop_rect.left = 2028 - (mode->width / 2);
	if (priv->crop_rect.left < 0)
		priv->crop_rect.left = 0;
	priv->crop_rect.top = 1520 - (mode->height / 2);
	if (priv->crop_rect.top < 0)
		priv->crop_rect.top = 0;
	priv->crop_rect.width = mode->width;
	priv->crop_rect.height = mode->height;

	return 0;
}

static int imx477_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct imx477 *priv = to_imx477(client);
	const struct imx477_mode *mode = priv->cur_mode;

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY)
		return 0;

	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.code = MEDIA_BUS_FMT_SRGGB12_1X12;
	fmt->format.field = V4L2_FIELD_NONE;

	return 0;
}

static void imx477_get_module_inf(struct imx477 *imx477,
				  struct rkmodule_inf *inf)
{
	memset(inf, 0, sizeof(*inf));
	strlcpy(inf->base.sensor, IMX477_NAME, sizeof(inf->base.sensor));
	strlcpy(inf->base.module, imx477->module_name,
		sizeof(inf->base.module));
	strlcpy(inf->base.lens, imx477->len_name, sizeof(inf->base.lens));
}

static long imx477_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct imx477 *imx477 = to_imx477(client);
	long ret = 0;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		imx477_get_module_inf(imx477, (struct rkmodule_inf *)arg);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}

#ifdef CONFIG_COMPAT
static long imx477_compat_ioctl32(struct v4l2_subdev *sd,
				  unsigned int cmd, unsigned long arg)
{
	void __user *up = compat_ptr(arg);
	struct rkmodule_inf *inf;
	struct rkmodule_awb_cfg *cfg;
	long ret;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		inf = kzalloc(sizeof(*inf), GFP_KERNEL);
		if (!inf) {
			ret = -ENOMEM;
			return ret;
		}

		ret = imx477_ioctl(sd, cmd, inf);
		if (!ret)
			ret = copy_to_user(up, inf, sizeof(*inf));
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
			ret = imx477_ioctl(sd, cmd, cfg);
		kfree(cfg);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}
#endif

static int imx477_g_mbus_config(struct v4l2_subdev *sd, unsigned int pad_id,
				struct v4l2_mbus_config *config)
{
	config->type = V4L2_MBUS_CSI2_DPHY;
	config->flags = V4L2_MBUS_CSI2_2_LANE |
			V4L2_MBUS_CSI2_CHANNEL_0 |
			V4L2_MBUS_CSI2_CONTINUOUS_CLOCK;

	return 0;
}

static int imx477_enum_frame_interval(struct v4l2_subdev *sd,
				       struct v4l2_subdev_pad_config *cfg,
				       struct v4l2_subdev_frame_interval_enum *fie)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct imx477 *priv = to_imx477(client);

	if (fie->index >= priv->cfg_num)
		return -EINVAL;

	if (fie->code != MEDIA_BUS_FMT_SRGGB12_1X12)
		return -EINVAL;

	fie->width = supported_modes[fie->index].width;
	fie->height = supported_modes[fie->index].height;
	fie->interval = supported_modes[fie->index].max_fps;
	return 0;
}

/* Various V4L2 operations tables */
static struct v4l2_subdev_video_ops imx477_subdev_video_ops = {
	.s_stream = imx477_s_stream,
	.g_frame_interval = imx477_g_frame_interval,
};

static struct v4l2_subdev_core_ops imx477_subdev_core_ops = {
	.s_power = imx477_s_power,
	.ioctl = imx477_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = imx477_compat_ioctl32,
#endif
};

static const struct v4l2_subdev_pad_ops imx477_subdev_pad_ops = {
	.enum_mbus_code = imx477_enum_mbus_code,
	.enum_frame_interval = imx477_enum_frame_interval,
	.set_fmt = imx477_set_fmt,
	.get_fmt = imx477_get_fmt,
	.get_mbus_config = imx477_g_mbus_config,
};

static struct v4l2_subdev_ops imx477_subdev_ops = {
	.core = &imx477_subdev_core_ops,
	.video = &imx477_subdev_video_ops,
	.pad = &imx477_subdev_pad_ops,
};

static const struct v4l2_ctrl_ops imx477_ctrl_ops = {
	.s_ctrl = imx477_s_ctrl,
};

static int imx477_video_probe(struct i2c_client *client)
{
	struct v4l2_subdev *subdev = i2c_get_clientdata(client);
	u16 model_id;
	u32 lot_id;
	u16 chip_id;
	int ret;

	ret = imx477_s_power(subdev, 1);
	if (ret < 0)
		return ret;

	/* Chip ID is at 0x0016 (high byte) and 0x0017 (low byte) */
	ret = reg_read(client, 0x0016);
	if (ret < 0) {
		dev_err(&client->dev, "Failure to read Model ID (high byte)\n");
		goto done;
	}
	model_id = ret << 8;

	ret = reg_read(client, 0x0017);
	if (ret < 0) {
		dev_err(&client->dev, "Failure to read Model ID (low byte)\n");
		goto done;
	}
	model_id |= ret;

	ret = reg_read(client, 0x0004);
	if (ret < 0) {
		dev_err(&client->dev, "Failure to read Lot ID (high byte)\n");
		goto done;
	}
	lot_id = ret << 16;

	ret = reg_read(client, 0x0005);
	if (ret < 0) {
		dev_err(&client->dev, "Failure to read Lot ID (mid byte)\n");
		goto done;
	}
	lot_id |= ret << 8;

	ret = reg_read(client, 0x0006);
	if (ret < 0) {
		dev_err(&client->dev, "Failure to read Lot ID (low byte)\n");
		goto done;
	}
	lot_id |= ret;

	ret = reg_read(client, 0x000d);
	if (ret < 0) {
		dev_err(&client->dev, "Failure to read Chip ID (high byte)\n");
		goto done;
	}
	chip_id = ret << 8;

	ret = reg_read(client, 0x000e);
	if (ret < 0) {
		dev_err(&client->dev, "Failure to read Chip ID (low byte)\n");
		goto done;
	}
	chip_id |= ret;

	if (model_id != 0x0477 && model_id != 0x0378) {
		dev_err(&client->dev, "Model ID: 0x%04x not supported!\n",
			model_id);
		ret = -ENODEV;
		goto done;
	}
	dev_info(&client->dev,
		 "Model ID 0x%04x, Lot ID 0x%06x, Chip ID 0x%04x\n",
		 model_id, lot_id, chip_id);
done:
	imx477_s_power(subdev, 0);
	return ret;
}

static int imx477_ctrls_init(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct imx477 *priv = to_imx477(client);
	const struct imx477_mode *mode = priv->cur_mode;
	s64 pixel_rate, h_blank, v_blank;
	int ret;
	u32 fps = 0;

	v4l2_ctrl_handler_init(&priv->ctrl_handler, 10);
	v4l2_ctrl_new_std(&priv->ctrl_handler, &imx477_ctrl_ops,
			  V4L2_CID_HFLIP, 0, 1, 1, 0);
	v4l2_ctrl_new_std(&priv->ctrl_handler, &imx477_ctrl_ops,
			  V4L2_CID_VFLIP, 0, 1, 1, 0);

	/* Gain */
	v4l2_ctrl_new_std(&priv->ctrl_handler, &imx477_ctrl_ops,
			  V4L2_CID_ANALOGUE_GAIN,
			  IMX477_ANALOGUE_GAIN_MIN,
			  IMX477_ANALOGUE_GAIN_MAX,
			  1, IMX477_ANALOGUE_GAIN_DEFAULT);
	v4l2_ctrl_new_std(&priv->ctrl_handler, &imx477_ctrl_ops,
			  V4L2_CID_GAIN,
			  IMX477_DIGITAL_GAIN_MIN,
			  IMX477_DIGITAL_GAIN_MAX, 1,
			  IMX477_DIGITAL_GAIN_DEFAULT);

	/* Exposure */
	v4l2_ctrl_new_std(&priv->ctrl_handler, &imx477_ctrl_ops,
			  V4L2_CID_EXPOSURE,
			  IMX477_DIGITAL_EXPOSURE_MIN,
			  IMX477_DIGITAL_EXPOSURE_MAX, 1,
			  IMX477_DIGITAL_EXPOSURE_DEFAULT);

	/* Blanking */
	h_blank = mode->hts_def - mode->width;
	priv->hblank = v4l2_ctrl_new_std(&priv->ctrl_handler, NULL,
			  V4L2_CID_HBLANK, h_blank, h_blank, 1, h_blank);
	v_blank = mode->vts_def - mode->height;
	priv->vblank = v4l2_ctrl_new_std(&priv->ctrl_handler, NULL,
			  V4L2_CID_VBLANK, v_blank, v_blank, 1, v_blank);

	/* Link frequency and pixel rate */
	v4l2_ctrl_new_int_menu(&priv->ctrl_handler, NULL, V4L2_CID_LINK_FREQ,
			       0, 0, link_freq_menu_items);
	fps = DIV_ROUND_CLOSEST(mode->max_fps.denominator,
		mode->max_fps.numerator);
	pixel_rate = mode->vts_def * mode->hts_def * fps;
	priv->pixel_rate = v4l2_ctrl_new_std(&priv->ctrl_handler, NULL,
			  V4L2_CID_PIXEL_RATE, 0, pixel_rate, 1, pixel_rate);

	v4l2_ctrl_new_std_menu_items(&priv->ctrl_handler, &imx477_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(tp_qmenu) - 1, 0, 0, tp_qmenu);

	priv->subdev.ctrl_handler = &priv->ctrl_handler;
	if (priv->ctrl_handler.error) {
		dev_err(&client->dev, "Error %d adding controls\n",
			priv->ctrl_handler.error);
		ret = priv->ctrl_handler.error;
		goto error;
	}

	ret = v4l2_ctrl_handler_setup(&priv->ctrl_handler);
	if (ret < 0) {
		dev_err(&client->dev, "Error %d setting default controls\n",
			ret);
		goto error;
	}

	return 0;
error:
	v4l2_ctrl_handler_free(&priv->ctrl_handler);
	return ret;
}

static int imx477_probe(struct i2c_client *client,
			const struct i2c_device_id *did)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;
	struct imx477 *priv;
	struct v4l2_subdev *sd;
	char facing[2];
	int ret;

	dev_info(dev, "driver version: %02x.%02x.%02x",
		DRIVER_VERSION >> 16,
		(DRIVER_VERSION & 0xff00) >> 8,
		DRIVER_VERSION & 0x00ff);

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_warn(&adapter->dev,
			 "I2C-Adapter doesn't support I2C_FUNC_SMBUS_BYTE\n");
		return -EIO;
	}
	priv = devm_kzalloc(&client->dev, sizeof(struct imx477), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	ret = of_property_read_u32(node, RKMODULE_CAMERA_MODULE_INDEX,
				   &priv->module_index);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_FACING,
				       &priv->module_facing);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_NAME,
				       &priv->module_name);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_LENS_NAME,
				       &priv->len_name);
	if (ret) {
		dev_err(dev, "could not get module information!\n");
		return -EINVAL;
	}

	priv->clk = devm_clk_get(&client->dev, NULL);
	if (IS_ERR(priv->clk)) {
		dev_info(&client->dev, "Error %ld getting clock\n",
			 PTR_ERR(priv->clk));
		return -EPROBE_DEFER;
	}

	/* Default to 2028x1520 (2x2 binned 40fps) */
	priv->cur_mode = &supported_modes[0];
	priv->cfg_num = ARRAY_SIZE(supported_modes);

	priv->crop_rect.width = priv->cur_mode->width;
	priv->crop_rect.height = priv->cur_mode->height;

	v4l2_i2c_subdev_init(&priv->subdev, client, &imx477_subdev_ops);

	/* Verify sensor presence before anything else */
	ret = imx477_video_probe(client);
	if (ret < 0)
		return ret;

	/*
	 * Power on so v4l2_ctrl_handler_setup can write default register
	 * values without locking the I2C bus (sensor needs clock to ACK).
	 */
	ret = imx477_s_power(&priv->subdev, 1);
	if (ret < 0)
		return ret;

	ret = imx477_ctrls_init(&priv->subdev);
	imx477_s_power(&priv->subdev, 0);
	if (ret < 0)
		return ret;

	priv->subdev.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
		     V4L2_SUBDEV_FL_HAS_EVENTS;

	priv->pad.flags = MEDIA_PAD_FL_SOURCE;
	priv->subdev.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&priv->subdev.entity, 1, &priv->pad);
	if (ret < 0)
		return ret;

	sd = &priv->subdev;
	memset(facing, 0, sizeof(facing));
	if (strcmp(priv->module_facing, "back") == 0)
		facing[0] = 'b';
	else
		facing[0] = 'f';

	snprintf(sd->name, sizeof(sd->name), "m%02d_%s_%s %s",
		 priv->module_index, facing,
		 IMX477_NAME, dev_name(sd->dev));
	ret = v4l2_async_register_subdev_sensor_common(sd);
	if (ret < 0)
		return ret;

	return ret;
}

static int imx477_remove(struct i2c_client *client)
{
	struct imx477 *priv = to_imx477(client);

	v4l2_async_unregister_subdev(&priv->subdev);
	media_entity_cleanup(&priv->subdev.entity);
	v4l2_ctrl_handler_free(&priv->ctrl_handler);

	return 0;
}

static const struct i2c_device_id imx477_id[] = {
	{"imx477", 0},
	{}
};

static const struct of_device_id imx477_of_match[] = {
	{ .compatible = "sony,imx477" },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, imx477_of_match);

MODULE_DEVICE_TABLE(i2c, imx477_id);

static struct i2c_driver imx477_i2c_driver = {
	.driver = {
		.of_match_table = of_match_ptr(imx477_of_match),
		.name = IMX477_NAME,
	},
	.probe = imx477_probe,
	.remove = imx477_remove,
	.id_table = imx477_id,
};

module_i2c_driver(imx477_i2c_driver);
MODULE_DESCRIPTION("Sony IMX477 Camera driver (Raspberry Pi HQ Camera)");
MODULE_AUTHOR("Raspberry Pi (Trading) Ltd / Radxa adaptation");
MODULE_LICENSE("GPL v2");
