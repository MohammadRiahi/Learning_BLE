/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 *  @brief LED Button Service (LBS) sample
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

#include "my_lbs.h"

LOG_MODULE_DECLARE(Lesson4_Exercise2);
/* LOG_DBG("Bluetooth initialized\n"); */

static bool notify_DATA_enabled;
//static bool indicate_enabled;
//static bool button_state;
static struct my_lbs_cb lbs_cb;
static uint8_t command_buffer[16]; // Buffer to store 16-byte commands

// Define the default command array
uint8_t default_command[16] = {0x00, 0x04, 0x01, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

/* STEP 4 - Define an indication parameter */
//static struct bt_gatt_indicate_params ind_params;



/* STEP 13 - Define the configuration change callback function for the MESSAGE characteristic */
static void mylbsbc_ccc_message_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	// This can be used for MESSAGE notifications if needed
	// For now, we'll just log it
	LOG_DBG("MESSAGE CCCD changed: %u", value);
}

/* STEP 13 - Define the configuration change callback function for the MYSENSOR characteristic */
static void mylbsbc_ccc_DATA_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	notify_DATA_enabled = (value == BT_GATT_CCC_NOTIFY);
}


static ssize_t write_commands(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf,
			 uint16_t len, uint16_t offset, uint8_t flags)
{
	LOG_DBG("Command write, handle: %u, conn: %p", attr->handle, (void *)conn);

	if (len != 16U) {
		LOG_DBG("Write command: Incorrect data length, expected 16 bytes, got %u", len);
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	if (offset != 0) {
		LOG_DBG("Write command: Incorrect data offset");
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	// Copy the 16-byte command into our buffer
	memcpy(command_buffer, buf, 16);
	
	// Switch on the second byte (command type)
	switch (command_buffer[1]) {
		case CMD_STOP_ALL:
			LOG_INF("Command: STOP_ALL");
			// Handle stop all command
			break;
			
		case CMD_BATTERY_CHECK:
			LOG_INF("Command: BATTERY_CHECK");
			// Handle battery check command
			break;
			
		case CMD_READ_CONFIG:
			LOG_DBG("Command: READ_CONFIG");
			// Handle read config command
			//LOG_DBG("Connected\n");
			break;
			
		case CMD_SET_CONFIG:
			LOG_INF("Command: SET_CONFIG");
			// Handle set config command
			break;
			
		case CMD_STOP_MEASUREMENT:
			LOG_INF("Command: STOP_MEASUREMENT");
			// Handle stop measurement command
			break;
			
		case CMD_START_CONTINUOUS:
			LOG_INF("Command: START_CONTINUOUS");
			// Handle start continuous command
			break;
			
		default:
			LOG_WRN("Unknown command type: 0x%02x", command_buffer[1]);
			return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
	}

	return len;
}

/*static ssize_t read_commands(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
			 uint16_t len, uint16_t offset)
{
	// get a pointer to command_buffer which is passed in the BT_GATT_CHARACTERISTIC() and stored in attr->user_data
	const char *value = attr->user_data;

	LOG_DBG("Command read, handle: %u, conn: %p", attr->handle, (void *)conn);

	// Return the last received command
	return bt_gatt_attr_read(conn, attr, buf, len, offset, value, 16);
}
*/
/* PIBiomed (PBM)  Service Declaration */
BT_GATT_SERVICE_DEFINE(
	my_pbm_svc, BT_GATT_PRIMARY_SERVICE(BT_UUID_PBM),
	BT_GATT_CHARACTERISTIC(BT_UUID_PBM_COMMAND, BT_GATT_CHRC_WRITE, BT_GATT_PERM_WRITE, NULL, write_commands, &command_buffer),
	BT_GATT_CHARACTERISTIC(BT_UUID_PBM_MESSAGE, BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_NONE, NULL, NULL, NULL),
	BT_GATT_CCC(mylbsbc_ccc_message_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	BT_GATT_CHARACTERISTIC(BT_UUID_PBM_DATA, BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_NONE, NULL, NULL, NULL),
	BT_GATT_CCC(mylbsbc_ccc_DATA_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

/* A function to register application callbacks for the LED and Button characteristics  */
int my_lbs_init(struct my_lbs_cb *callbacks)
{
	LOG_DBG("Bluetooth initialized\n");
	if (callbacks) {
		lbs_cb.led_cb = callbacks->led_cb;
		lbs_cb.button_cb = callbacks->button_cb;
	}

	return 0;
}

/* STEP 5 - Define the function to send indications 
int my_lbs_send_button_state_indicate(bool button_state)
{
	if (!indicate_enabled) {
		return -EACCES;
	}
	ind_params.attr = &my_lbs_svc.attrs[2];
	ind_params.func = indicate_cb; // A remote device has ACKed at its host layer (ATT ACK)
	ind_params.destroy = NULL;
	ind_params.data = &button_state;
	ind_params.len = sizeof(button_state);
	return bt_gatt_indicate(NULL, &ind_params);
}
*/
/* STEP 14 - Define the function to send notifications for the MYSENSOR characteristic */
int my_lbs_send_sensor_notify(uint32_t sensor_value)
{
	if (!notify_DATA_enabled) {
		return -EACCES;
	}

	return bt_gatt_notify(NULL, &my_pbm_svc.attrs[7], &sensor_value, sizeof(sensor_value));
}
