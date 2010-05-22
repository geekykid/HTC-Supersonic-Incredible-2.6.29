/*
 * Copyright (C) 2009 Google, Inc.
 * Copyright (C) 2009 HTC Corporation.
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

/* Control bluetooth power for supersonic platform */

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/rfkill.h>
#include <linux/delay.h>
#include <asm/gpio.h>
#include <asm/mach-types.h>

#include "proc_comm.h"
#include "board-supersonic.h"

static struct rfkill *bt_rfk;
static const char bt_name[] = "bcm4329";

static int supersonic_bt_status;

static uint32_t supersonic_bt_init_table[] = {
	PCOM_GPIO_CFG(SUPERSONIC_GPIO_BT_UART1_RTS, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA),	/* BT_RTS */
	PCOM_GPIO_CFG(SUPERSONIC_GPIO_BT_UART1_CTS, 0, GPIO_INPUT, GPIO_PULL_UP, GPIO_8MA),	/* BT_CTS */
	PCOM_GPIO_CFG(SUPERSONIC_GPIO_BT_UART1_RX, 0, GPIO_INPUT, GPIO_PULL_UP, GPIO_8MA),	/* BT_RX */
	PCOM_GPIO_CFG(SUPERSONIC_GPIO_BT_UART1_TX, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA),	/* BT_TX */

	PCOM_GPIO_CFG(SUPERSONIC_GPIO_BT_RESET_N, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA),	/* BT_RESET_N */
	PCOM_GPIO_CFG(SUPERSONIC_GPIO_BT_SHUTDOWN_N, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA),	/* BT_SHUTDOWN_N */

	PCOM_GPIO_CFG(SUPERSONIC_GPIO_BT_HOST_WAKE, 0, GPIO_INPUT, GPIO_PULL_UP, GPIO_4MA),	/* BT_HOST_WAKE */
	PCOM_GPIO_CFG(SUPERSONIC_GPIO_BT_CHIP_WAKE, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA),	/* BT_CHIP_WAKE */
};

static uint32_t supersonic_bt_on_table[] = {
	PCOM_GPIO_CFG(SUPERSONIC_GPIO_BT_UART1_RTS, 2, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA),	/* BT_RTS */
	PCOM_GPIO_CFG(SUPERSONIC_GPIO_BT_UART1_CTS, 2, GPIO_INPUT, GPIO_PULL_UP, GPIO_8MA),	/* BT_CTS */
	PCOM_GPIO_CFG(SUPERSONIC_GPIO_BT_UART1_RX, 2, GPIO_INPUT, GPIO_PULL_UP, GPIO_8MA),	/* BT_RX */
	PCOM_GPIO_CFG(SUPERSONIC_GPIO_BT_UART1_TX, 2, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA),	/* BT_TX */

	PCOM_GPIO_CFG(SUPERSONIC_GPIO_BT_HOST_WAKE, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_4MA),	/* BT_HOST_WAKE */
	PCOM_GPIO_CFG(SUPERSONIC_GPIO_BT_CHIP_WAKE, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA),	/* BT_CHIP_WAKE */

	PCOM_GPIO_CFG(SUPERSONIC_GPIO_BT_RESET_N, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA),	/* BT_RESET_N */
	PCOM_GPIO_CFG(SUPERSONIC_GPIO_BT_SHUTDOWN_N, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA),	/* BT_SHUTDOWN_N */
};

/* BT off and system is sleep/suspend */
static uint32_t supersonic_bt_disable_sleep_table[] = {
	PCOM_GPIO_CFG(SUPERSONIC_GPIO_BT_UART1_RTS, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA),	/* OLNP */
	PCOM_GPIO_CFG(SUPERSONIC_GPIO_BT_UART1_CTS, 0, GPIO_INPUT, GPIO_PULL_UP, GPIO_8MA),	/* OLNP */
	PCOM_GPIO_CFG(SUPERSONIC_GPIO_BT_UART1_RX, 0, GPIO_INPUT, GPIO_PULL_UP, GPIO_8MA),	/* OLNP */
	PCOM_GPIO_CFG(SUPERSONIC_GPIO_BT_UART1_TX, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA),	/* OLNP */

	PCOM_GPIO_CFG(SUPERSONIC_GPIO_BT_HOST_WAKE, 0, GPIO_INPUT, GPIO_PULL_UP, GPIO_4MA),	/* BT_HOST_W, OLNP */
	PCOM_GPIO_CFG(SUPERSONIC_GPIO_BT_CHIP_WAKE, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA),	/* BT_CHIP_WAKE, OLNP */
};

static void config_bt_table(uint32_t *table, int len)
{
	int n;
	unsigned id;
	for(n = 0; n < len; n++) {
		id = table[n];
		msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &id, 0);
	}
}

static void supersonic_config_bt_init(void)
{
	supersonic_bt_status = 0;
	config_bt_table(supersonic_bt_init_table, ARRAY_SIZE(supersonic_bt_init_table));
	mdelay(5);
	gpio_configure(SUPERSONIC_GPIO_BT_RESET_N, GPIOF_DRIVE_OUTPUT | GPIOF_OUTPUT_LOW);
	gpio_configure(SUPERSONIC_GPIO_BT_SHUTDOWN_N, GPIOF_DRIVE_OUTPUT | GPIOF_OUTPUT_LOW);

	gpio_configure(SUPERSONIC_GPIO_BT_UART1_RTS, GPIOF_DRIVE_OUTPUT | GPIOF_OUTPUT_LOW);	/* OLNP */
	gpio_configure(SUPERSONIC_GPIO_BT_UART1_TX, GPIOF_DRIVE_OUTPUT | GPIOF_OUTPUT_LOW);	/* OLNP */

	gpio_configure(SUPERSONIC_GPIO_BT_CHIP_WAKE, GPIOF_DRIVE_OUTPUT | GPIOF_OUTPUT_LOW);	/* OLNP */
}

static void supersonic_config_bt_on(void)
{
	config_bt_table(supersonic_bt_on_table, ARRAY_SIZE(supersonic_bt_on_table));
	mdelay(5);
	gpio_configure(SUPERSONIC_GPIO_BT_RESET_N, GPIOF_DRIVE_OUTPUT | GPIOF_OUTPUT_HIGH);
	gpio_configure(SUPERSONIC_GPIO_BT_SHUTDOWN_N, GPIOF_DRIVE_OUTPUT | GPIOF_OUTPUT_HIGH);

	gpio_configure(SUPERSONIC_GPIO_BT_CHIP_WAKE, GPIOF_DRIVE_OUTPUT | GPIOF_OUTPUT_HIGH);
	supersonic_bt_status = 1;
}

void supersonic_config_bt_disable_sleep(void)
{
	config_bt_table(supersonic_bt_disable_sleep_table, ARRAY_SIZE(supersonic_bt_disable_sleep_table));
	mdelay(5);
	gpio_configure(SUPERSONIC_GPIO_BT_UART1_RTS, GPIOF_DRIVE_OUTPUT | GPIOF_OUTPUT_LOW);	/* OLNP */
	gpio_configure(SUPERSONIC_GPIO_BT_UART1_TX, GPIOF_DRIVE_OUTPUT | GPIOF_OUTPUT_LOW);	/* OLNP */

	gpio_configure(SUPERSONIC_GPIO_BT_CHIP_WAKE, GPIOF_DRIVE_OUTPUT | GPIOF_OUTPUT_LOW);	/* OLNP */
}

static void supersonic_config_bt_off(void)
{
	gpio_configure(SUPERSONIC_GPIO_BT_SHUTDOWN_N, GPIOF_DRIVE_OUTPUT | GPIOF_OUTPUT_LOW);
	gpio_configure(SUPERSONIC_GPIO_BT_RESET_N, GPIOF_DRIVE_OUTPUT | GPIOF_OUTPUT_LOW);
	supersonic_config_bt_disable_sleep();
	supersonic_bt_status = 0;
}

void supersonic_config_bt_disable_active(void)
{
	supersonic_config_bt_disable_sleep();
}

int supersonic_is_bluetooth_off(void)
{
	return !supersonic_bt_status;	//ON:1, OFF:0
}

static int bluetooth_set_power(void *data, enum rfkill_state state)
{
	switch (state) {
		case RFKILL_STATE_UNBLOCKED:
			supersonic_config_bt_on();
			break;
		case RFKILL_STATE_SOFT_BLOCKED:
			supersonic_config_bt_off();
			break;
		default:
			pr_err("%s: bad rfkill state %d\n", __func__, state);
	}

	return 0;
}

static int supersonic_rfkill_probe(struct platform_device *pdev)
{
	int rc = 0;
	enum rfkill_state default_state = RFKILL_STATE_SOFT_BLOCKED;

	supersonic_config_bt_init();	/* bt gpio initial config */

	rfkill_set_default(RFKILL_TYPE_BLUETOOTH, default_state);
	bluetooth_set_power(NULL, default_state);

	bt_rfk = rfkill_allocate(&pdev->dev, RFKILL_TYPE_BLUETOOTH);
	if (!bt_rfk)
		return -ENOMEM;

	bt_rfk->name = bt_name;
	bt_rfk->state = default_state;

	/* userspace cannot take exclusive control */
	bt_rfk->user_claim_unsupported = 1;
	bt_rfk->user_claim = 0;
	bt_rfk->data = NULL;
	bt_rfk->toggle_radio = bluetooth_set_power;

	rc = rfkill_register(bt_rfk);
	if (rc)
		goto err_rfkill_reg;

	return 0;

err_rfkill_reg:
	rfkill_free(bt_rfk);
	return rc;
}

static int supersonic_rfkill_remove(struct platform_device *dev)
{
	rfkill_unregister(bt_rfk);
	rfkill_free(bt_rfk);

	return 0;
}

static struct platform_driver supersonic_rfkill_driver = {
	.probe = supersonic_rfkill_probe,
	.remove = supersonic_rfkill_remove,
	.driver = {
		.name = "supersonic_rfkill",
		.owner = THIS_MODULE,
	},
};

static int __init supersonic_rfkill_init(void)
{
	if (!machine_is_supersonic())
		return 0;

	return platform_driver_register(&supersonic_rfkill_driver);
}

static void __exit supersonic_rfkill_exit(void)
{
	platform_driver_unregister(&supersonic_rfkill_driver);
}

module_init(supersonic_rfkill_init);
module_exit(supersonic_rfkill_exit);
MODULE_DESCRIPTION("supersonic rfkill");
MODULE_AUTHOR("Nick Pelly <npelly@google.com>");
MODULE_LICENSE("GPL");
