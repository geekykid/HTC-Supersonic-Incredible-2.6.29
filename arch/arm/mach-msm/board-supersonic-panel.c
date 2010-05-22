/* linux/arch/arm/mach-msm/board-supersonic-panel.c
 *
 * Copyright (C) 2008 HTC Corporation.
 * Author: Jay Tu <jay_tu@htc.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/leds.h>
#include <linux/i2c.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/gpio.h>

#include <asm/io.h>
#include <asm/mach-types.h>
#include <mach/msm_fb.h>
#include <mach/msm_iomap.h>
#include <mach/vreg.h>
#include <mach/pmic.h>

#include "board-supersonic.h"
#include "devices.h"
#include "proc_comm.h"

#if 1
#define B(s...) printk(s)
#else
#define B(s...) do {} while(0)
#endif

static struct cabc_t {
	struct led_classdev lcd_backlight;
	struct msm_mddi_client_data *client_data;
	struct mutex lock;
	unsigned long status;
} cabc;

enum {
	GATE_ON = 1 << 0,
};

static void suc_set_brightness(struct led_classdev *led_cdev,
				enum led_brightness val)
{
	struct msm_mddi_client_data *client = cabc.client_data;
	unsigned int shrink_br = val;

	printk(KERN_DEBUG "set brightness = %d\n", val);
	if (test_bit(GATE_ON, &cabc.status) == 0)
		return;

	if (val < 30)
		shrink_br = 5;
	else if ((val >= 30) && (val <= 143))
		shrink_br = 104 * (val - 30) / 113 + 5;
	else
		shrink_br = 145 * (val - 144) / 111 + 110;
	mutex_lock(&cabc.lock);
	client->remote_write(client, 0x00, 0x5500);
	client->remote_write(client, shrink_br, 0x5100);
	mutex_unlock(&cabc.lock);
}

static enum led_brightness
suc_get_brightness(struct led_classdev *led_cdev)
{
	struct msm_mddi_client_data *client = cabc.client_data;

	return client->remote_read(client, 0x5100);
}

#define DEFAULT_BRIGHTNESS 100
static void suc_backlight_switch(int on)
{
	enum led_brightness val;

	if (on) {
		printk(KERN_DEBUG "turn on backlight\n");
		set_bit(GATE_ON, &cabc.status);
		val = cabc.lcd_backlight.brightness;

		/* LED core uses get_brightness for default value
		 * If the physical layer is not ready, we should
		 * not count on it */
		if (val == 0)
			val = DEFAULT_BRIGHTNESS;
		suc_set_brightness(&cabc.lcd_backlight, val);
	} else {
		clear_bit(GATE_ON, &cabc.status);
		suc_set_brightness(&cabc.lcd_backlight, 0);
	}
}

static int suc_backlight_probe(struct platform_device *pdev)
{
	int err = -EIO;

	mutex_init(&cabc.lock);
	cabc.client_data = pdev->dev.platform_data;
	cabc.lcd_backlight.name = "lcd-backlight";
	cabc.lcd_backlight.brightness_set = suc_set_brightness;
	cabc.lcd_backlight.brightness_get = suc_get_brightness;
	err = led_classdev_register(&pdev->dev, &cabc.lcd_backlight);
	if (err)
		goto err_register_lcd_bl;
	return 0;

err_register_lcd_bl:
	led_classdev_unregister(&cabc.lcd_backlight);
	return err;
}

/* ------------------------------------------------------------------- */

static struct resource resources_msm_fb[] = {
	{
		.start = MSM_FB_BASE,
		.end = MSM_FB_BASE + MSM_FB_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
};

static struct vreg *vreg_lcd_2v8;
static struct vreg *vreg_lcd_1v8;

#define REG_WAIT (0xffff)

struct nov_regs {
	unsigned reg;
	unsigned val;
} nov_init_seq[] = {
	{0xc000, 0x86},
	{0xc001, 0x00},
	{0xc002, 0x86},
	{0xc003, 0x00},
	{0xc100, 0x40},
	{0xc200, 0x02},
	{0xc202, 0x32},
	{0xe000, 0x0e},
	{0xe001, 0x2a},
	{0xe002, 0x33},
	{0xe003, 0x38},
	{0xe004, 0x1e},
	{0xe005, 0x30},
	{0xe006, 0x64},
	{0xe007, 0x3f},
	{0xe008, 0x21},
	{0xe009, 0x27},
	{0xe00a, 0x88},
	{0xe00b, 0x14},
	{0xe00c, 0x35},
	{0xe00d, 0x56},
	{0xe00e, 0x79},
	{0xe00f, 0x88},
	{0xe010, 0x55},
	{0xe011, 0x57},
	{0xe100, 0x0e},
	{0xe101, 0x2a},
	{0xe102, 0x33},
	{0xe103, 0x3b},
	{0xe104, 0x1e},
	{0xe105, 0x30},
	{0xe106, 0x64},
	{0xe107, 0x3f},
	{0xe108, 0x21},
	{0xe109, 0x27},
	{0xe10a, 0x88},
	{0xe10b, 0x14},
	{0xe10c, 0x35},
	{0xe10d, 0x56},
	{0xe10e, 0x79},
	{0xe10f, 0x88},
	{0xe110, 0x55},
	{0xe111, 0x57},

	{0xe200, 0x0E},
	{0xe201, 0x2A},
	{0xe202, 0x33},
	{0xe203, 0x3B},
	{0xe204, 0x1e},
	{0xe205, 0x30},
	{0xe206, 0x64},
	{0xe207, 0x3F},
	{0xe208, 0x21},
	{0xe209, 0x27},
	{0xe20A, 0x88},
	{0xe20B, 0x14},
	{0xe20C, 0x35},
	{0xe20D, 0x56},
	{0xe20E, 0x79},
	{0xe20F, 0xB8},
	{0xe210, 0x55},
	{0xe211, 0x57},

	{0xe300, 0x0E},
	{0xe301, 0x2A},
	{0xe302, 0x33},
	{0xe303, 0x3B},
	{0xe304, 0x1E},
	{0xe305, 0x30},
	{0xe306, 0x64},
	{0xe307, 0x3F},
	{0xe308, 0x21},
	{0xe309, 0x27},
	{0xe30A, 0x88},
	{0xe30B, 0x14},
	{0xe30C, 0x35},
	{0xe30D, 0x56},
	{0xe30E, 0x79},
	{0xe30F, 0xB8},
	{0xe310, 0x55},
	{0xe311, 0x57},
	{0xe400, 0x0E},
	{0xe401, 0x2A},
	{0xe402, 0x33},
	{0xe403, 0x3B},
	{0xe404, 0x1E},
	{0xe405, 0x30},
	{0xe406, 0x64},
	{0xe407, 0x3F},
	{0xe408, 0x21},
	{0xe409, 0x27},
	{0xe40A, 0x88},
	{0xe40B, 0x14},
	{0xe40C, 0x35},
	{0xe40D, 0x56},
	{0xe40E, 0x79},
	{0xe40F, 0xB8},
	{0xe410, 0x55},
	{0xe411, 0x57},
	{0xe500, 0x0E},
	{0xe501, 0x2A},
	{0xe502, 0x33},
	{0xe503, 0x3B},
	{0xe504, 0x1E},
	{0xe505, 0x30},
	{0xe506, 0x64},
	{0xe507, 0x3F},
	{0xe508, 0x21},
	{0xe509, 0x27},
	{0xe50A, 0x88},
	{0xe50B, 0x14},
	{0xe50C, 0x35},
	{0xe50D, 0x56},
	{0xe50E, 0x79},
	{0xe50F, 0xB8},
	{0xe510, 0x55},
	{0xe511, 0x57},

	{0x3a00, 0x05},

	/* cabc */
	{0x4e00, 0x00},
	{0x5e00, 0x00},
	{0x6a01, 0x00},
	{0x6a02, 0x03},
	{0x5100, 0xff},
	{0x5301, 0x10},
	{0x6A18, 0xff},
	{0x6A17, 0x01},
	{0xF402, 0x14},

	{0x3500, 0x00},
	{0x1100, 0x0},
	{REG_WAIT, 120},
};

static int
supersonic_mddi_init(struct msm_mddi_bridge_platform_data *bridge_data,
		     struct msm_mddi_client_data *client_data)
{
	int i = 0;
	unsigned reg, val;

	client_data->auto_hibernate(client_data, 0);

	for (i = 0; i < ARRAY_SIZE(nov_init_seq); i++) {
		reg = cpu_to_le32(nov_init_seq[i].reg);
		val = cpu_to_le32(nov_init_seq[i].val);
		if (reg == REG_WAIT)
			msleep(val);
		else
			client_data->remote_write(client_data, val, reg);
	}

	client_data->auto_hibernate(client_data, 1);
	return 0;
}

static int
supersonic_mddi_uninit(struct msm_mddi_bridge_platform_data *bridge_data,
			struct msm_mddi_client_data *client_data)
{
	client_data->remote_write(client_data, 0, 0x2800);
	return 0;
}

/* FIXME: remove after XA03 */
static int backlight_control(int on)
{
	struct i2c_adapter *adap = i2c_get_adapter(0);
	struct i2c_msg msg;
	u8 buf[] = {0x90, 0x00, 0x00, 0x08};
	int ret = -EIO, max_retry = 3;

	msg.addr = 0xcc >> 1;
	msg.flags = 0;
	msg.len = sizeof(buf);
	msg.buf = buf;

	if (on == 0)
		buf[0] = 0x91;

	while (max_retry--) {
		ret = i2c_transfer(adap, &msg, 1);
		if (ret != 1)
			msleep(1);
		else {
			ret = 0;
			break;
		}
		ret = -EIO;
	}

	if (ret)
		printk(KERN_ERR "backlight control fail\n");
	return 0;
}

static int
supersonic_panel_blank(struct msm_mddi_bridge_platform_data *bridge_data,
			struct msm_mddi_client_data *client_data)
{
	B(KERN_DEBUG "%s\n", __func__);
	suc_backlight_switch(LED_OFF);
	backlight_control(0);
	return 0;
}

static int
supersonic_panel_unblank(struct msm_mddi_bridge_platform_data *bridge_data,
			struct msm_mddi_client_data *client_data)
{
	B(KERN_DEBUG "%s\n", __func__);
	suc_backlight_switch(LED_FULL);
	client_data->remote_write(client_data, 0x00, 0x2900);
	msleep(100);
	client_data->remote_write(client_data, 0x24, 0x5300);
	backlight_control(1);
	return 0;
}

static struct msm_mddi_bridge_platform_data novatec_client_data = {
	.init = supersonic_mddi_init,
	.uninit = supersonic_mddi_uninit,
	.blank = supersonic_panel_blank,
	.unblank = supersonic_panel_unblank,
	.fb_data = {
		.xres = 480,
		.yres = 800,
		.width = 48,
		.height = 80,
		.output_format = 0,
	},
	.panel_conf = {
		.caps = MSMFB_CAP_CABC,
	},
};

static void
mddi_novatec_power(struct msm_mddi_client_data *client_data, int on)
{
	unsigned id, on_off = 1;

	B(KERN_DEBUG "%s: power %s.\n", __func__, on ? "on" : "off");

	if (on) {
		on_off = 1;
		/* 2V8 */
		id = PM_VREG_PDOWN_SYNT_ID;
		msm_proc_comm(PCOM_VREG_PULLDOWN, &on_off, &id);
		vreg_enable(vreg_lcd_2v8);

		/* 1V8 */
		id = PM_VREG_PDOWN_AUX_ID;
		msm_proc_comm(PCOM_VREG_PULLDOWN, &on_off, &id);
		vreg_enable(vreg_lcd_1v8);
		mdelay(15);

		gpio_set_value(SUPERSONIC_LCD_RST, 1);
		mdelay(1);
		gpio_set_value(SUPERSONIC_LCD_RST, 0);
		mdelay(5);
		gpio_set_value(SUPERSONIC_LCD_RST, 1);
		msleep(50);
	} else {
		on_off = 0;
		gpio_set_value(SUPERSONIC_LCD_RST, 0);
		mdelay(120);

		/* 1V8 */
		id = PM_VREG_PDOWN_AUX_ID;
		msm_proc_comm(PCOM_VREG_PULLDOWN, &on_off, &id);
		vreg_disable(vreg_lcd_1v8);

		/* 2V8 */
		id = PM_VREG_PDOWN_SYNT_ID;
		msm_proc_comm(PCOM_VREG_PULLDOWN, &on_off, &id);
		vreg_disable(vreg_lcd_2v8);
	}
}

static struct msm_mddi_platform_data mddi_pdata = {
	.clk_rate = 384000000,
	.power_client = mddi_novatec_power,
	.fb_resource = resources_msm_fb,
	.num_clients = 1,
	.client_platform_data = {
		{
			.product_id = (0xb9f6 << 16 | 0x5582),
			.name = "mddi_c_b9f6_5582",
			.id = 0,
			.client_data = &novatec_client_data,
			.clk_rate = 0,
		},
	},
};

static struct platform_driver suc_backlight_driver = {
	.probe = suc_backlight_probe,
	.driver = {
		.name = "nov_cabc",
		.owner = THIS_MODULE,
	},
};

static struct msm_mdp_platform_data mdp_pdata = {
	.dma_channel = MDP_DMA_S,
};

int __init supersonic_init_panel(void)
{
	int rc;

	if (!machine_is_supersonic())
		return -1;

	B(KERN_INFO "%s: enter.\n", __func__);

	vreg_lcd_1v8 = vreg_get(0, "gp4");
	if (IS_ERR(vreg_lcd_1v8))
		return PTR_ERR(vreg_lcd_1v8);

	vreg_lcd_2v8 = vreg_get(0, "synt");
	if (IS_ERR(vreg_lcd_2v8))
		return PTR_ERR(vreg_lcd_2v8);

	msm_device_mdp.dev.platform_data = &mdp_pdata;
	rc = platform_device_register(&msm_device_mdp);
	if (rc)
		return rc;

	msm_device_mddi0.dev.platform_data = &mddi_pdata;
	rc = platform_device_register(&msm_device_mddi0);
	if (rc)
		return rc;

	rc = platform_driver_register(&suc_backlight_driver);
	if (rc)
		return rc;

	return 0;
}

device_initcall(supersonic_init_panel);
