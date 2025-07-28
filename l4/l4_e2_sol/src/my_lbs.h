/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef BT_LBS_H_
#define BT_LBS_H_

/**@file
 * @defgroup bt_lbs LED Button Service API
 * @{
 * @brief API for the LED Button Service (LBS).
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <zephyr/types.h>

/** @brief PBM Service and Advertising UUID. */
#define BT_UUID_PBM_VAL                BT_UUID_128_ENCODE(0x0000525F, 0x45D0, 0x4AA9, 0xDB0A, 0x93D9E09A7CFC)
#define BT_UUID_PBM_ADVERTISING_VAL    BT_UUID_128_ENCODE(0x000062C4, 0xB99E, 0x4141, 0x9439, 0xC4F9DB977899)
                                                          
/** @brief COMMAND Characteristic UUID. */
#define BT_UUID_PBM_COMMAND_VAL                                                       \
	BT_UUID_128_ENCODE(0x0100525F, 0x45D0, 0x4AA9, 0xDB0A, 0x93D9E09A7CFC)

/** @brief MESSAGE Characteristic UUID. */
#define BT_UUID_PBM_MESSAGE_VAL BT_UUID_128_ENCODE(0x0300525F, 0x45D0, 0x4AA9, 0xDB0A, 0x93D9E09A7CFC)

/** @brief DATA Characteristic UUID. */
#define BT_UUID_PBM_DATA_VAL                                                 \
	BT_UUID_128_ENCODE(0x0200525F, 0x45D0, 0x4AA9, 0xDB0A, 0x93D9E09A7CFC)

	/** @brief HeartBeat Characteristic UUID. */
#define BT_UUID_PBM_HEARTBEAT_VAL    BT_UUID_128_ENCODE(0x0400525F, 0x45D0, 0x4AA9, 0xDB0A, 0x93D9E09A7CFC)

#define BT_UUID_PBM_ADVERTISING  BT_UUID_DECLARE_128(BT_UUID_PBM_ADVERTISING_VAL)
#define BT_UUID_PBM              BT_UUID_DECLARE_128(BT_UUID_PBM_VAL)
#define BT_UUID_PBM_COMMAND      BT_UUID_DECLARE_128(BT_UUID_PBM_COMMAND_VAL)
#define BT_UUID_PBM_MESSAGE      BT_UUID_DECLARE_128(BT_UUID_PBM_MESSAGE_VAL)
#define BT_UUID_PBM_DATA         BT_UUID_DECLARE_128(BT_UUID_PBM_DATA_VAL)
#define BT_UUID_PBM_HEARTBEAT    BT_UUID_DECLARE_128(BT_UUID_PBM_HEARTBEAT_VAL)
/** @brief Callback type for when an LED state change is received. */
typedef void (*led_cb_t)(const bool led_state);

/** @brief Callback type for when the button state is pulled. */
typedef bool (*button_cb_t)(void);

/** @brief Callback struct used by the LBS Service. */
struct my_lbs_cb {
	/** LED state change callback. */
	led_cb_t led_cb;
	/** Button read callback. */
	button_cb_t button_cb;
};
enum CommandType {
	CMD_STOP_ALL = 0x00,
    CMD_BATTERY_CHECK = 0x01,
    CMD_READ_CONFIG = 0x04,
    CMD_SET_CONFIG = 0x05,
	CMD_START_SINGLE = 0x10,
    CMD_START_CONTINUOUS = 0x11,
	CMD_STOP_MEASUREMENT = 0x12
};
// Remove the global array definition from header file
// This should be defined in a .c file, not here
extern uint8_t default_command[16]; // Declaration only

/** @brief Initialize the LBS Service.
 *
 * This function registers application callback functions with the My LBS
 * Service
 *
 * @param[in] callbacks Struct containing pointers to callback functions
 *			used by the service. This pointer can be NULL
 *			if no callback functions are defined.
 *
 *
 * @retval 0 If the operation was successful.
 *           Otherwise, a (negative) error code is returned.
 */
int my_lbs_init(struct my_lbs_cb *callbacks);

/** @brief Send the button state as indication.
 *
 * This function sends a binary state, typically the state of a
 * button, to all connected peers.
 *
 * @param[in] button_state The state of the button.
 *
 * @retval 0 If the operation was successful.
 *           Otherwise, a (negative) error code is returned.
 */
int my_lbs_send_button_state_indicate(bool button_state);

/** @brief Send the button state as notification.
 *
 * This function sends a binary state, typically the state of a
 * button, to all connected peers.
 *
 * @param[in] button_state The state of the button.
 *
 * @retval 0 If the operation was successful.
 *           Otherwise, a (negative) error code is returned.
 */
int my_lbs_send_button_state_notify(bool button_state);

/** @brief Send the sensor value as notification.
 *
 * This function sends an uint32_t  value, typically the value
 * of a simulated sensor to all connected peers.
 *
 * @param[in] sensor_value The value of the simulated sensor.
 *
 * @retval 0 If the operation was successful.
 *           Otherwise, a (negative) error code is returned.
 */
int my_lbs_send_sensor_notify(uint32_t sensor_value);

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif /* BT_LBS_H_ */
