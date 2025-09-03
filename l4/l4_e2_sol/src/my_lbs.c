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
#include <zephyr/drivers/adc.h>
#include <zephyr/device.h>
#include <C:\ncs\v2.6.1\modules\hal\nordic\nrfx\hal\nrf_saadc.h>
//#include <C:\ncs\v3.0.2\zephyr\include\zephyr\dt-bindings\adc\nrf-saadc-v3.h>

#include "my_lbs.h"


#define ADC_RESOLUTION 12
#define ADC_ACQUISITION_TIME ADC_ACQ_TIME_DEFAULT // you can change the acquisition time for higher accuracy
// Note: gain and reference are now configured in app.overlay device tree

LOG_MODULE_DECLARE(Lesson4_Exercise2);
/* LOG_DBG("Bluetooth initialized\n"); */

/* Forward declaration of the GATT service */
extern const struct bt_gatt_service_static my_pbm_svc;
// Sampling parameters 
volatile uint8_t averaging = 1; //Set by the user on the app
volatile uint32_t samplingRate = 2; // Hz

/* Helper function to decode sample rate from code */
static uint32_t decodeSampleRate(uint8_t code) {
    switch (code) {
        case 0: return 100;
        case 1: return 250;
        case 2: return 500;
        case 3: return 1000;
        case 4: return 2000;
        case 5: return 4000;
        default: return 1000;  // default fallback
    }
}

static bool notify_DATA_enabled;
static bool notify_MESSAGE_enabled;
//static bool indicate_enabled;
//static bool button_state;
static struct my_lbs_cb lbs_cb;
static uint8_t command_buffer[16]; // Buffer to store 16-byte commands
static uint8_t heartbeat_buffer[8]; // Buffer for heartbeat value
static char message_buffer[120]; // Static buffer for JSON messages
static char data_buffer[244]; // Buffer for DATA characteristic
// Define the default command array
uint8_t default_command[16] = {0x00, 0x04, 0x01, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

// Simple continuous measurement variables
static bool is_measuring = false;
static struct k_work_delayable continuous_work;

// ADC and data packet configuration
#define DATAPACKET_SIZE 244
#define SENSOR_PIN 0  // ADC channel to use


// Simple ADC read function (placeholder until ADC is properly configured)

static const struct device *adc_dev = DEVICE_DT_GET(DT_NODELABEL(adc));
static int16_t adc_sample_buffer;
static struct adc_channel_cfg channel_cfg;
static struct adc_sequence sequence;

static void adc_setup(uint8_t channel)
{
    // Get device tree configuration for this specific channel
    const struct adc_channel_cfg *ch_cfg;
    
    if (channel == 0) {
        // Use device tree configuration for channel 0
        static const struct adc_channel_cfg ch0_cfg = 
            ADC_CHANNEL_CFG_DT(DT_CHILD(DT_NODELABEL(adc), channel_0));
        ch_cfg = &ch0_cfg;
    } else if (channel == 1) {
        // Use device tree configuration for channel 1  
        static const struct adc_channel_cfg ch1_cfg = 
            ADC_CHANNEL_CFG_DT(DT_CHILD(DT_NODELABEL(adc), channel_1));
        ch_cfg = &ch1_cfg;
    } else {
        LOG_ERR("Invalid channel %d", channel);
        return;
    }
    
    // Setup channel with device tree configuration
    int ret = adc_channel_setup(adc_dev, ch_cfg);
    if (ret) {
        LOG_ERR("ADC channel setup failed with error %d", ret);
        return;
    }

    // Configure sequence
    sequence.channels = BIT(channel);
    sequence.buffer = &adc_sample_buffer;
    sequence.buffer_size = sizeof(adc_sample_buffer);
    sequence.resolution = ADC_RESOLUTION;
    LOG_INF("ADC setup complete for channel %d", channel);
}


static uint16_t read_adc_averaged(uint8_t n)
{
    int32_t sum = 0;
    for (uint8_t i = 0; i < n; i++) {
        if (adc_read(adc_dev, &sequence) == 0) {
            sum += adc_sample_buffer;
            LOG_DBG("Sample %d: %d, Running sum: %d", i, adc_sample_buffer, sum);
        }
    }
    uint16_t average = (uint16_t)(sum / n);
    LOG_INF("ADC Sum: %d, Samples: %d, Average: %d", sum, n, average);
    return average;
}
    /* TODO: Replace with actual ADC reading when CONFIG_ADC is enabled
    // For now, return simulated data
    static uint16_t adc_value = 1000;
    adc_value += (k_uptime_get_32() % 40) - 20; // Add variation
    if (adc_value > 4095) adc_value = 4095;
    if (adc_value < 0) adc_value = 0;
    return adc_value;
	*/
//}

// Get timestamp function
static uint64_t get_timestamp(void)
{
    return k_uptime_get(); // System uptime in milliseconds
}

// Prepare data packet in the specified format
static void prepare_data_packet(uint8_t* data_packet)
{
    // Include timestamp in the packet (6 bytes)
    uint64_t unix_time = get_timestamp();
    for (int i = 0; i < 6; i++) {
        data_packet[i] = (unix_time >> (i * 8)) & 0xFF;
    }

    // Data format configuration
    const uint8_t num_channels = 1;    // 1 channel
    const uint8_t precision = 1;       // 1 => 16-bit
    const uint8_t channel_index = (1 << 0) << 4;  // channel 0 => bit4
    uint8_t bytes_per_sample = precision + 1;     // 2 bytes
    uint8_t encoded_num_channels = num_channels - 1;  // 1 channel -> 0

    uint8_t data_format = (encoded_num_channels & 0x03)
        | ((precision & 0x03) << 2)
        | (channel_index & 0xF0);

    data_packet[6] = data_format;
    data_packet[7] = (DATAPACKET_SIZE - 8) / (num_channels * bytes_per_sample);
    
    int num_samples = data_packet[7]; // Number of samples

    // Fill ADC values
    for (int i = 0; i < num_samples; i++) {
        //adc_setup(1); // Ensure ADC is set up for channel 1
		uint64_t start  = k_uptime_get();
        uint16_t adc_buffer = read_adc_averaged(averaging);
		//uint16_t adc_buffer = 1000;
		data_packet[8 + (i * 2)] = adc_buffer & 0xFF;
        data_packet[9 + (i * 2)] = (adc_buffer >> 8) & 0xFF;
        
		uint64_t elapsed = k_uptime_get() - start;
		if(elapsed<1){
			k_msleep(1 - elapsed); // Ensure at least 1ms per sample for stability
		}
    }
}

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
	LOG_INF("DATA CCCD changed: %u, notifications %s", value, 
		notify_DATA_enabled ? "enabled" : "disabled");
}

// Simple continuous measurement functions
static void continuous_work_handler(struct k_work *work)
{
	if (!is_measuring || !notify_DATA_enabled) {
		return;
	}
	
	// Clear the data buffer and prepare data packet
	memset(data_buffer, 0, sizeof(data_buffer));
	prepare_data_packet((uint8_t*)data_buffer);
	
	// Send data via DATA characteristic with error handling
	int result = bt_gatt_notify(NULL, &my_pbm_svc.attrs[5], data_buffer, DATAPACKET_SIZE);
	if (result == 0) {
		LOG_DBG("Sent data packet (%d bytes)", DATAPACKET_SIZE);
	} else if (result == -ENOMEM) {
		LOG_WRN("BLE buffers full, skipping packet");
		// Don't stop measurement, just skip this packet
	} else {
		LOG_ERR("Failed to send data packet: %d", result);
	}
	
	// Schedule next measurement - realistic BLE timing
	// For 244-byte packets, minimum practical interval is ~50ms
	uint32_t interval_ms = 1000 / samplingRate;
	if (interval_ms < 50) interval_ms = 50; // Minimum 50ms for large packets
	k_work_reschedule(&continuous_work, K_MSEC(interval_ms));
}

static void start_continuous_measurement(void)
{
	if (is_measuring) {
		LOG_WRN("Already measuring");
		return;
	}
	
	is_measuring = true;
	LOG_INF("Starting continuous measurement at %d Hz", samplingRate);
	k_work_schedule(&continuous_work, K_NO_WAIT);
}

static void stop_continuous_measurement(void)
{
	is_measuring = false;
	k_work_cancel_delayable(&continuous_work);
	LOG_INF("Stopped continuous measurement");
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
    
   //LOG_INF("Heartbeat ignored for testing");
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
			LOG_INF("Command: STOP_MEASUREMENT");
			stop_continuous_measurement();
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
			default_command[0] = 0;
			default_command[1] = CMD_SET_CONFIG;
			default_command[2] = command_buffer[2];
			// Clear bytes 3-6 (4 bytes) 
			memset(&default_command[3], 0, 4);
			default_command[7] = command_buffer[7];  // Sample Rate
			samplingRate = decodeSampleRate(default_command[7]);  // Decode sampling rate
			averaging = command_buffer[8];  // Set averaging parameter
			default_command[8] = averaging;
			default_command[9] = command_buffer[9];  // Set averaging parameter
			memset(&default_command[10], 0, 6); // Clear bytes 9-15
			
			current_timestamp = k_uptime_get_32();
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
			
		case CMD_STOP_MEASUREMENT:
			//LOG_INF("Command: STOP_MEASUREMENT");
			//stop_continuous_measurement();
			break;
		case CMD_START_SINGLE:
			LOG_INF("Command: START_SINGLE");
			//adc_setup(SENSOR_PIN); // set up the ADC
			// Send a single data packet
			if (notify_DATA_enabled) {
				memset(data_buffer, 0, sizeof(data_buffer)); // filling the buffer with zeros
				// Prepare the data packet with the current timestamp and ADC values
				prepare_data_packet((uint8_t*)data_buffer);
				int result = bt_gatt_notify(NULL, &my_pbm_svc.attrs[5], data_buffer, DATAPACKET_SIZE);
				if (result == 0) {
					LOG_INF("Single data packet sent (%d bytes)", DATAPACKET_SIZE);
				} else {
					LOG_ERR("Failed to send single data packet: %d", result);
				}
			} else {
				LOG_WRN("DATA notifications not enabled");
			}
			break;
		case CMD_START_CONTINUOUS:
			//adc_setup(SENSOR_PIN); // set up the ADC
			LOG_INF("Command: START_CONTINUOUS");
			start_continuous_measurement();
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
	
	// Initialize continuous measurement work
	adc_setup(SENSOR_PIN); // set up the ADC
	k_work_init_delayable(&continuous_work, continuous_work_handler);
	
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
