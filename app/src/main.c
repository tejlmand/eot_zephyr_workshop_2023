/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/led.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/drivers/gpio.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main, CONFIG_APP_LOG_LEVEL);

/** @brief LED Service UUID (equal to Nordic LBS). */
#define BT_UUID_LED_VAL                                                       \
	BT_UUID_128_ENCODE(0x00001523, 0x1212, 0xefde, 0x1523, 0x785feabcd123)
#define BT_UUID_LED BT_UUID_DECLARE_128(BT_UUID_LED_VAL)

/** @brief LED color set action characteristic UUID. */
#define BT_UUID_LED_ACTION_VAL                                                \
	BT_UUID_128_ENCODE(0x00001525, 0x1212, 0xefde, 0x1523, 0x785feabcd123)
#define BT_UUID_LED_ACTION BT_UUID_DECLARE_128(BT_UUID_LED_ACTION_VAL)

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

/* The devicetree node identifier for the "led1" alias. */
#define LED1_NODE DT_ALIAS(led1)

/* The devicetree node identifier for the "led2" alias. */
#define LED2_NODE DT_ALIAS(led2)

/** @brief Bluetooth advertisement data */
static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME,
		sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

/** @brief Bluetooth service discovery data */
static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_LED_VAL),
};

static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(LED1_NODE, gpios);
static const struct gpio_dt_spec led2 = GPIO_DT_SPEC_GET(LED2_NODE, gpios);

/*******************************************************************************
 * Bluetooth services handling
 ******************************************************************************/

static ssize_t write_led(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			 const void *buf, uint16_t len, uint16_t offset,
			 uint8_t flags)
{
	if (len != 1U) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	if (offset != 0) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	uint8_t val = *((uint8_t *)buf);
	gpio_pin_set_dt(&led2, val);
	return len;
}

/* LED Service Definition */
BT_GATT_SERVICE_DEFINE(led_svc, BT_GATT_PRIMARY_SERVICE(BT_UUID_LED),
		       BT_GATT_CHARACTERISTIC(BT_UUID_LED_ACTION,
					      BT_GATT_CHRC_WRITE,
					      BT_GATT_PERM_WRITE, NULL,
					      write_led, NULL), );

static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err != 0) {
		printk("Connection failed (err %u)\n", err);
		return;
	}
	gpio_pin_set_dt(&led1, true);

	printk("Connected\n");
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	gpio_pin_set_dt(&led1, false);
	printk("Disconnected (reason %u)\n", reason);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};

int main(void)
{
#ifdef CONFIG_USB_DEVICE_STACK
	int ret = usb_enable(NULL);
	if (ret < 0) {
		LOG_ERR("Failed to enable USB");
		return 0;
	}
#endif
	if (!gpio_is_ready_dt(&led0)) {
		return 0;
	}

	ret = gpio_pin_configure_dt(&led0, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return 0;
	}

	if (!gpio_is_ready_dt(&led1)) {
		return 0;
	}

	ret = gpio_pin_configure_dt(&led1, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return 0;
	}

	if (!gpio_is_ready_dt(&led2)) {
		return 0;
	}

	ret = gpio_pin_configure_dt(&led2, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return 0;
	}

	/* enable Bluetooth and start advertising */
	ret = bt_enable(NULL);
	if (ret < 0) {
		printk("Bluetooth init failed (ret %d)\n", ret);
		return 0;
	}

	ret = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), sd,
			      ARRAY_SIZE(sd));
	if (ret < 0) {
		printk("Advertising failed to start (%d)", ret);
		return 0;
	}
	printk("Advertising started (%s)\n", CONFIG_BT_DEVICE_NAME);

	while (1) {
		ret = gpio_pin_toggle_dt(&led0);
		if (ret < 0) {
			return 0;
		}
		k_msleep(SLEEP_TIME_MS);
	}
}
