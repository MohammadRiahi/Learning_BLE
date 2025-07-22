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

/* Forward declaration of the GATT service */
extern const struct bt_gatt_service_static my_pbm_svc;

static bool notify_DATA_enabled;
static bool notify_MESSAGE_enabled;
//static bool indicate_enabled;
//static bool button_state;
static struct my_lbs_cb lbs_cb;
static uint8_t command_buffer[16]; // Buffer to store 16-byte commands
static uint8_t heartbeat_buffer[8]; // Buffer for heartbeat value
static char message_buffer[120]; // Static buffer for JSON messages
// Define the default command array
uint8_t default_command[16] = {0x00, 0x04, 0x01, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

/* STEP 4 - Define an indication parameter */
//static struct bt_gatt_indicate_params ind_params;



/* STEP 13 - Define the configuration change callback function for the MESSAGE characteristic */
static void mylbsbc_ccc_message_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	notify_MESSAGE_enabled = (value == BT_GATT_CCC_NOTIFY);
	LOG_INF("MESSAGE CCCD changed: %u, notifications %s", value, 
		notify_MESSAGE_enabled ? "enabled" : "disabled");
}

/* STEP 13 - Define the configuration change callback function for the MYSENSOR characteristic */
static void mylbsbc_ccc_DATA_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	notify_DATA_enabled = (value == BT_GATT_CCC_NOTIFY);
}

static ssize_t write_heartbeat(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf,
			 uint16_t len, uint16_t offset, uint8_t flags)
{
	LOG_INF("Heartbeat write, handle: %u, conn: %p", attr->handle, (void *)conn);
	
	if (len != 8U) {
		LOG_ERR("Invalid heartbeat length: %u", len);
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	uint32_t heartbeat_value = sys_get_be32(buf);
	LOG_INF("Heartbeat value: %u", heartbeat_value);

	// Here you can handle the heartbeat value as needed
	// For now, we'll just log it

	return len;
}
static ssize_t write_commands(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf,
			 uint16_t len, uint16_t offset, uint8_t flags)
{
	// Make this VERY visible
	printk("\n\n*** WRITE_COMMANDS FUNCTION CALLED ***\n");
	printk("*** THIS SHOULD BE VERY VISIBLE ***\n\n");
	
	LOG_INF("=== WRITE_COMMANDS CALLED ===");
	LOG_INF("Command write, handle: %u, conn: %p", attr->handle, (void *)conn);

	LOG_INF("Data length: %u bytes", len);
	// -----------------------------------------------------------
	// Log the raw bytes received
	const uint8_t *data = (const uint8_t *)buf;
	LOG_INF("Raw data received:");
	for (int i = 0; i < len; i++) {
		printk("%02x", data[i]);
	}
	printk("\n");
	
	// Also log as hex string for easy comparison
	char hex_str[64];
	for (int i = 0; i < len && i < 16; i++) {
		sprintf(&hex_str[i*2], "%02x", data[i]);
	}
	hex_str[len*2] = '\0';
	LOG_INF("Hex string: %s", hex_str);
// -----------------------------------------------------------
	if (len != 16U) {
		LOG_INF("Write command: Incorrect data length, expected 16 bytes, got %u", len);
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	if (offset != 0) {
		LOG_INF("Write command: Incorrect data offset");
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
			LOG_INF("Command: READ_CONFIG");
			// Update the second byte of default_command with the command type
			default_command[1] = CMD_READ_CONFIG;
			
			// Get current timestamp (using k_uptime_get_32() for milliseconds since boot)
			uint32_t current_timestamp = k_uptime_get_32();
			
			// Use the static message buffer instead of declaring on stack
			// Format JSON message similar to your C++ version
			snprintf(message_buffer, sizeof(message_buffer),
				"{\"ts\":%u,\"Config read\":[%d,%d,%d,%d,%d,%d,%d,%d,%d]}",
				current_timestamp,
				default_command[2], default_command[3], default_command[4], default_command[7],
				default_command[8], default_command[9], default_command[10], default_command[11],
				default_command[12]);
			
			LOG_INF("JSON message created: %s", message_buffer);
			
			// Check if MESSAGE notifications are enabled
			if (!notify_MESSAGE_enabled) {
				LOG_WRN("MESSAGE notifications not enabled by client");
				// Still return success as the command was processed
			} else {
				// Send notification through MESSAGE characteristic
				// The MESSAGE characteristic value attribute is at index 9 in the service
				LOG_INF("Attempting to send notification, message length: %d", strlen(message_buffer));
				int result = bt_gatt_notify(NULL, &my_pbm_svc.attrs[9], message_buffer, strlen(message_buffer));
				if (result == 0) {
					LOG_INF("Message notification sent successfully");
				} else {
					LOG_ERR("Failed to send message notification: %d", result);
					// Add specific error descriptions
					switch (result) {
						case -ENOMEM:
							LOG_ERR("No memory/buffer available for notification");
							break;
						case -ENOTCONN:
							LOG_ERR("Device not connected");
							break;
						case -EINVAL:
							LOG_ERR("Invalid parameters");
							break;
						default:
							LOG_ERR("Unknown error code: %d", result);
							break;
					}
				}
			}
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

static ssize_t read_commands(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
			 uint16_t len, uint16_t offset)
{
	// get a pointer to command_buffer (16-byte array) which is passed in the BT_GATT_CHARACTERISTIC() and stored in attr->user_data
	const uint8_t *command_data = (const uint8_t *)attr->user_data;

	LOG_INF("Command read, handle: %u, conn: %p", attr->handle, (void *)conn);
	
	// Log what we're about to send back to the client
	LOG_INF("Sending command buffer content:");
	for (int i = 0; i < 16; i++) {
		printk("%02x", command_data[i]);
	}
	printk("\n");

	// Return the content of command_buffer (16 bytes)
	return bt_gatt_attr_read(conn, attr, buf, len, offset, command_data, 16);
}

/* PIBiomed (PBM)  Service Declaration */
BT_GATT_SERVICE_DEFINE(
	my_pbm_svc, BT_GATT_PRIMARY_SERVICE(BT_UUID_PBM),
	
	// Command Characteristic with descriptive name
	BT_GATT_CHARACTERISTIC(BT_UUID_PBM_COMMAND, BT_GATT_CHRC_WRITE | BT_GATT_CHRC_READ, BT_GATT_PERM_WRITE | BT_GATT_PERM_READ, read_commands, write_commands, &command_buffer),
	BT_GATT_CUD("COMMAND", BT_GATT_PERM_READ),
		
	// Data Characteristic with descriptive name
	BT_GATT_CHARACTERISTIC(BT_UUID_PBM_DATA, BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_NONE, NULL, NULL, NULL),
	BT_GATT_CUD("DATA", BT_GATT_PERM_READ),
	BT_GATT_CCC(mylbsbc_ccc_DATA_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	
	// Message Characteristic with descriptive name
	BT_GATT_CHARACTERISTIC(BT_UUID_PBM_MESSAGE, BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_NONE, NULL, NULL, NULL),
	BT_GATT_CUD("MESSAGE", BT_GATT_PERM_READ),
	BT_GATT_CCC(mylbsbc_ccc_message_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	// HeartBeat Characteristic with descriptive name
	BT_GATT_CHARACTERISTIC(BT_UUID_PBM_HEARTBEAT, BT_GATT_CHRC_WRITE | BT_GATT_CHRC_READ, BT_GATT_PERM_WRITE, NULL, write_heartbeat, &heartbeat_buffer),
	BT_GATT_CUD("HEARTBEAT", BT_GATT_PERM_READ),
);

/* Advertising Service Declaration - No characteristics, just service UUID 
BT_GATT_SERVICE_DEFINE(
	my_advertising_svc, BT_GATT_PRIMARY_SERVICE(BT_UUID_PBM_ADVERTISING),
);
*/
/* A function to register application callbacks for the LED and Button characteristics  */
int my_lbs_init(struct my_lbs_cb *callbacks)
{
	LOG_INF("LBS service initialization started");
	
	// Initialize command buffer with default command
	memcpy(command_buffer, default_command, 16);
	LOG_INF("Command buffer initialized with default command");
	
	LOG_INF("Service has %d attributes", my_pbm_svc.attr_count);
	
	// Print all attributes to verify service structure
	for (int i = 0; i < my_pbm_svc.attr_count; i++) {
		LOG_INF("Attr[%d]: UUID type %d, read=%p, write=%p", 
			i, my_pbm_svc.attrs[i].uuid->type, 
			(void*)my_pbm_svc.attrs[i].read, 
			(void*)my_pbm_svc.attrs[i].write);
	}
	
	if (callbacks) {
		lbs_cb.led_cb = callbacks->led_cb;
		lbs_cb.button_cb = callbacks->button_cb;
	}
	LOG_INF("LBS service initialization completed");
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

	// The DATA characteristic value attribute is at index 4:
	// [0] Primary Service
	// [1] COMMAND Characteristic Declaration
	// [2] COMMAND Characteristic Value
	// [3] COMMAND CUD Descriptor
	// [4] DATA Characteristic Declaration
	// [5] DATA Characteristic Value
	// [6] DATA CUD Descriptor
	// [7] DATA CCC Descriptor
	// [8] MESSAGE Characteristic Declaration
	// [9] MESSAGE Characteristic Value
	// [10] MESSAGE CUD Descriptor
	// [11] MESSAGE CCC Descriptor
	return bt_gatt_notify(NULL, &my_pbm_svc.attrs[5], &sensor_value, sizeof(sensor_value));
}
