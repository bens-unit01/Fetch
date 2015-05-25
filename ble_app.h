/* Copyright (c) 2013 Nordic Semiconductor. All Rights Reserved.
 *
 * Use of this source code is governed by a BSD-style license that can be
 * found in the license.txt file.
 */

#ifndef _BLE_APP_H_
#define _BLE_APP_H_

#include <stdint.h>
#include <stdbool.h>
#include "ble_srv_common.h"
#include "ble_app.h"


#define APP_UUID_BASE { 0x80, 0x16, 0xad, 0x28, 0xf0, 0x48, 0x19, 0xb6, 0xa2, 0x4b, 0xf6, 0x37, 0x00, 0x00, 0x00, 0x00 }
#define UUID_QR_SVC 0x0ace
#define UUID_QR_CHR 0x0001

#define APP_CMD_ARM	(0x03)		/* Arm quadrotor */
#define APP_CMD_MOVE	(0x04)		/* movement command */

typedef struct ble_app_s ble_app_t;
typedef void (*ble_app_cmd_handler_t) (ble_app_t *app, uint8_t *cmd, uint8_t cmdlen);

/* quadrotor BLE structure. This contains various status information for the service. */
typedef struct ble_app_s {
	uint16_t service_handle;
	ble_gatts_char_handles_t cmd_char_handles;
	uint8_t uuid_type;
	uint16_t conn_handle;
	ble_app_cmd_handler_t cmd_handler;
} ble_app_t;

typedef struct {
	ble_app_cmd_handler_t cmd_handler;	/* Event handler to be called when LED characteristic is written. */
} ble_app_init_t;

uint32_t ble_app_init(ble_app_t *app, const ble_app_init_t *app_init);

void ble_app_on_ble_evt(ble_app_t *app, ble_evt_t *evt);

uint32_t ble_app_tx_blob(ble_app_t *cam, uint8_t *buf, uint32_t len);

#endif // _BLE_APP_H_
