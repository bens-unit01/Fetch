/* Copyright (c) 2013 Nordic Semiconductor. All Rights Reserved.
 *
 * Use of this source code is governed by a BSD-style license that can be
 * found in the license.txt file.
 */

#include <string.h>
#include "nordic_common.h"
#include "ble_srv_common.h"
#include "app_util.h"
#include "app_timer.h"
#include "ble_bondmngr.h"
#include "ble_app.h"


static void service_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


static void on_connect(ble_app_t *app, ble_evt_t *evt)
{
	app->conn_handle = evt->evt.gap_evt.conn_handle;
}


static void on_disconnect(ble_app_t *app, ble_evt_t *evt)
{
	UNUSED_PARAMETER(evt);
	app->conn_handle = BLE_CONN_HANDLE_INVALID;
}


static void on_write(ble_app_t *app, ble_evt_t *evt)
{
	ble_gatts_evt_write_t *evt_write = &evt->evt.gatts_evt.params.write;

	if ((evt_write->handle == app->cmd_char_handles.value_handle) && (app->cmd_handler != NULL)) {
		app->cmd_handler(app, &evt_write->data[0], evt_write->len);
	}
}


void ble_app_on_ble_evt(ble_app_t *app, ble_evt_t *evt)
{
	switch (evt->header.evt_id) {
	case BLE_GAP_EVT_CONNECTED:
		on_connect(app, evt);
		break;

	case BLE_GAP_EVT_DISCONNECTED:
		on_disconnect(app, evt);
		break;

	case BLE_GATTS_EVT_WRITE:
		on_write(app, evt);
		break;

	default:
		// No implementation needed.
		break;
	};
}


/* defines the command "pipe" characteristic */
static uint32_t cmd_char_add(ble_app_t *app, const ble_app_init_t *app_init)
{
	ble_gatts_char_md_t char_md;
	ble_gatts_attr_t attr_char_value;
	ble_uuid_t ble_uuid;
	ble_gatts_attr_md_t attr_md;

	memset(&char_md, 0, sizeof(char_md));
	char_md.char_props.read = 0;
	char_md.char_props.write = 1;
	char_md.p_char_user_desc = NULL;
	char_md.p_char_pf = NULL;
	char_md.p_user_desc_md = NULL;
	char_md.p_cccd_md = NULL;
	char_md.p_sccd_md = NULL;

	ble_uuid.type = app->uuid_type;
	ble_uuid.uuid = UUID_QR_CHR;

	memset(&attr_md, 0, sizeof(attr_md));
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
	attr_md.vloc = BLE_GATTS_VLOC_STACK;
	attr_md.rd_auth = 0;
	attr_md.wr_auth = 0;
	attr_md.vlen = 0;

	memset(&attr_char_value, 0, sizeof(attr_char_value));
	attr_char_value.p_uuid = &ble_uuid;
	attr_char_value.p_attr_md = &attr_md;
	attr_char_value.init_len = 6;
	attr_char_value.init_offs = 0;
	attr_char_value.max_len = 6;
	attr_char_value.p_value = NULL;

	return sd_ble_gatts_characteristic_add(app->service_handle, &char_md, &attr_char_value, &app->cmd_char_handles);
}


/* initializes the BLE application */
uint32_t ble_app_init(ble_app_t *app, const ble_app_init_t *app_init)
{
	uint32_t   err_code;
	ble_uuid_t ble_uuid;

	// Initialize service structure
	app->conn_handle  = BLE_CONN_HANDLE_INVALID;
	app->cmd_handler = app_init->cmd_handler;

	// Add service
	ble_uuid128_t base_uuid = APP_UUID_BASE;
	err_code = sd_ble_uuid_vs_add(&base_uuid, &app->uuid_type);
	if (err_code != NRF_SUCCESS) {
		return err_code;
	}

	/* add the quadrotor service */
	ble_uuid.type = app->uuid_type;
	ble_uuid.uuid = UUID_QR_SVC;

	err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &app->service_handle);
	if (err_code != NRF_SUCCESS) {
		return err_code;
	}

	err_code = cmd_char_add(app, app_init);
	if (err_code != NRF_SUCCESS) {
		return err_code;
	}

	return NRF_SUCCESS;
}


/* transmits up to 20 bytes of a data blob to the blob characteristic */
uint32_t ble_app_tx_blob(ble_app_t *app, uint8_t *data, uint32_t len)
{
	ble_gatts_hvx_params_t params;
	uint16_t plen;

	/* don't try to do anything if the connection handle is invalid */
	if (app->conn_handle == BLE_CONN_HANDLE_INVALID) {
		return -1;
	}

	if (len > 20) {
		plen = 20;
	} else {
		plen = len;
	}

	memset(&params, 0, sizeof(params));
	params.type = BLE_GATT_HVX_NOTIFICATION;
	params.handle = app->cmd_char_handles.value_handle;
	params.p_data = data;
	params.p_len = &plen;

	return sd_ble_gatts_hvx(app->conn_handle, &params);
}
