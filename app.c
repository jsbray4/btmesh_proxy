/***************************************************************************//**
 * @file
 * @brief Core application logic.
 *******************************************************************************
 * # License
 * <b>Copyright 2024 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * SPDX-License-Identifier: Zlib
 *
 * The licensor of this software is Silicon Laboratories Inc.
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 ******************************************************************************/
#include "app_assert.h"
#include "sl_status.h"
#include "app.h"
#include "sl_main_init.h"
#include "sl_simple_led_instances.h"
#include "sl_simple_button_instances.h"
#include "sl_btmesh_factory_reset.h"
#include <stdio.h>
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "gatt_db.h"


#ifndef ELEMENT_INDEX 
#define ELEMENT_INDEX            0
#endif

#define GENERIC_LEVEL_CLIENT 0x1003
#define GENERIC_ON_OFF_SERVER 0x1000
#define TEMP_ELEMENT 1
#define LIGHT_ELEMENT 2
#define VENDOR_PERIOD 0x4000
#define VENDOR_ID 0x02ff


#include "sl_btmesh_api.h"
#include "sl_bt_api.h"

static uint8_t advertising_set_handle = 0xff;
volatile conn_state_t conn_state = init;
volatile uint8_t connection_handle = CONNECTION_HANDLE_INVALID;
volatile uint32_t service_handle = SERVICE_HANDLE_INVALID;
volatile uint16_t characteristic_handle = CHARACTERISTIC_HANDLE_INVALID;
static uint8_t test = 1;
static xQueueHandle light_queue;
static xQueueHandle blink_queue;
static xSemaphoreHandle reset;
static xSemaphoreHandle erase_bonds;
static uint8_t op_codes [1] = {0};


/*******************************************************************************
 * Application Early Init
 ******************************************************************************/
void app_init_early(void)
{
  /////////////////////////////////////////////////////////////////////////////
  // Put your additional application init code here!                         //
  // This is called once before the OS is initialized if RTOS is used.       //
  // This function precedes permanent memory allocations.                    //
  /////////////////////////////////////////////////////////////////////////////
}
/**************************************************************************//**
 * Application Init.
 *****************************************************************************/
void app_init(void)
{
  /////////////////////////////////////////////////////////////////////////////
  // Put your additional application init code here!                         //
  // This is called once during start-up.                                    //
  /////////////////////////////////////////////////////////////////////////////
  sl_simple_led_init(&sl_led_led0);
  blink_queue = xQueueCreate(1, sizeof(uint8_t));
  app_assert(blink_queue != NULL, "Failed to create blink_queue");
  reset = xSemaphoreCreateBinary();
  app_assert(reset != NULL, "Failed to create reset semaphore");
  erase_bonds = xSemaphoreCreateBinary();
  app_assert(erase_bonds != NULL, "Failed to create erase_bonds semaphore");
  light_queue = xQueueCreate(1, sizeof(uint8_t));
  app_assert(light_queue != NULL, "Failed to create light_queue");
  printf("Application initialized\r\n");
}

void sl_button_on_change(const sl_button_t *handle)
{
  //btn 0 triggers reset; must be done after removing from nrf connect
  if(handle == &sl_button_btn0) {
    if(sl_button_get_state(handle) == SL_SIMPLE_BUTTON_PRESSED) {
      xSemaphoreGiveFromISR(reset, NULL);
    }
  }
  //btn1 erases bonds, for connection with wifi board
  else if(handle == &sl_button_btn1) {
    if(sl_button_get_state(handle) == SL_SIMPLE_BUTTON_PRESSED) {
      xSemaphoreGiveFromISR(erase_bonds, NULL);
    }
  }
}

/**************************************************************************//**
 * Application Process Action.
 *****************************************************************************/
void app_process_action(void)
{
  int8_t numBlink;
  sl_status_t sc;
  int8_t lightState;

  if(xSemaphoreTake(reset, 0) == pdTRUE) { //btn 0 reset
    printf("Factory reset initiated by button press\r\n");
    sl_btmesh_initiate_full_reset();
  }
  if(xSemaphoreTake(erase_bonds, 0) == pdTRUE) { //btn 1 erase bonds
    printf("Erase bonds initiated by button press\r\n");
    sc = sl_bt_sm_delete_bondings();
    app_assert_status(sc);
  }
  //board blinks the passkey during authentication
  if(xQueueReceive(blink_queue, &numBlink, 0) == pdTRUE) {
    vTaskDelay(pdMS_TO_TICKS(3000));
    for(int i = 0; i < numBlink; i++) {
      sl_led_turn_on(&sl_led_led1);
      vTaskDelay(pdMS_TO_TICKS(500));
      sl_led_turn_off(&sl_led_led1);
      vTaskDelay(pdMS_TO_TICKS(500));
    }
      //reset passkey after used
      sc = sl_bt_sm_set_passkey ((rand() % 7) + 3); //generate random 3-9
      app_assert_status(sc);
  }
  if(xQueueReceive(light_queue, &lightState, 0) == pdTRUE) { //change led state
    printf("Received data from light task: %d\r\n", lightState);
    if(lightState == 0) {
      sl_led_turn_off(&sl_led_led0);
    } else {
      sl_led_turn_on(&sl_led_led0);
    }
  }
}

/**************************************************************************//**
 * Bluetooth stack event handler.
 * This overrides the dummy weak implementation.
 *
 * @param[in] evt Event coming from the Bluetooth stack.
 *****************************************************************************/
void sl_bt_on_event(sl_bt_msg_t *evt)
{
  sl_status_t sc;

  switch (SL_BT_MSG_ID(evt->header)) {
    // -------------------------------
    // This event indicates the device has started and the radio is ready.
    // Do not call any stack command before receiving this boot event!
    case sl_bt_evt_system_boot_id:
      //configure security flags
      uint8_t flags = SL_BT_SM_CONFIGURATION_MITM_REQUIRED | SL_BT_SM_CONFIGURATION_PREFER_MITM | SL_BT_SM_CONFIGURATION_SC_ONLY;
      sc = sl_bt_sm_configure(flags, sl_bt_sm_io_capability_displayonly);
      app_assert_status(sc);
      sc = sl_bt_sm_set_passkey ((rand() % 7) + 3); //generate random passkey 3-9
      app_assert_status(sc);

      //configure bonding
      sc = sl_bt_sm_store_bonding_configuration(4, 2); //4 max bonds, replacing oldest if full
      app_assert_status(sc);
      sc = sl_bt_sm_set_bondable_mode(1); //on by default
      app_assert_status(sc);

      //create an advertising set.
      sc = sl_bt_advertiser_create_set(&advertising_set_handle);
      app_assert_status(sc);

      //generate data for advertising
      sc = sl_bt_legacy_advertiser_generate_data(advertising_set_handle,
                                                 sl_bt_advertiser_general_discoverable);
      app_assert_status(sc);
      
      //set advertising interval to 100ms.
      sc = sl_bt_advertiser_set_timing(
        advertising_set_handle,
        160, // min. adv. interval (milliseconds * 1.6)
        160, // max. adv. interval (milliseconds * 1.6)
        0,   // adv. duration
        0);  // max. num. adv. events
      app_assert_status(sc);

      //start advertising and enable connections.
      sc = sl_bt_legacy_advertiser_start(advertising_set_handle,
                                         sl_bt_legacy_advertiser_connectable);
      app_assert_status(sc);

      break;

    // -------------------------------
    // This event indicates that a new connection was opened.
    case sl_bt_evt_connection_opened_id:
      //store connection handle
      printf("Connection opened\r\n");
      connection_handle = evt->data.evt_connection_opened.connection;
      break;

    // -------------------------------
    // This event indicates that a connection was closed.
    case sl_bt_evt_connection_closed_id:
      //reset connection handle + app state
      printf("Connection terminated\r\n");
      connection_handle = CONNECTION_HANDLE_INVALID;
      conn_state = init;

      //generate data for advertising
      sc = sl_bt_legacy_advertiser_generate_data(advertising_set_handle,
                                                 sl_bt_advertiser_general_discoverable);
      app_assert_status(sc);

      //restart advertising
      sc = sl_bt_legacy_advertiser_start(advertising_set_handle,
                                         sl_bt_legacy_advertiser_connectable);
      app_assert_status(sc);
      break;
    
    // This event indicated connection parameters have changed
    case sl_bt_evt_connection_parameters_id:
      //print connection parameters
      // printf("Connection parameters updated:\r\n");
      // printf("\tConnection Handle: %u\r\n", evt->data.evt_connection_parameters.connection);
      // printf("\tInterval: %u * 1.25ms\r\n", evt->data.evt_connection_parameters.interval);
      // printf("\tLatency: %u\r\n", evt->data.evt_connection_parameters.latency);
      // printf("\tTimeout: %u * 10ms\r\n", evt->data.evt_connection_parameters.timeout);
      // printf("\tSecurity Mode: %u\r\n", evt->data.evt_connection_parameters.security_mode);
      
      //triggers in all scenarios; used as confirm bonding state if bond doesn't exist already
      if(evt->data.evt_connection_parameters.security_mode == 0)
        conn_state = opening;
      //connection authenticated
      else if(evt->data.evt_connection_parameters.security_mode == 3)//
      {
        //triggerd if bond already existed
        if(conn_state == opening)
          printf("Previous bond used to reconnect\r\n");
        
        conn_state = running;
      }
      break;
    
    case sl_bt_evt_sm_passkey_display_id:
      conn_state = passkey;
      printf("Passkey for pairing: %06u\r\n", evt->data.evt_sm_passkey_display.passkey);
      xQueueSend(blink_queue, &(evt->data.evt_sm_passkey_display.passkey), 0);
      break;
    
    //triggered when central writes to attribute
    case sl_bt_evt_gatt_server_attribute_value_id:
    {
      sl_bt_evt_gatt_server_attribute_value_t * pkt = &(evt->data.evt_gatt_server_attribute_value);
      uint16_t period;
      // if (pkt->value.len == 2){
      //   printf("GATT WRITE: attr=0x%04x len=%d\r\n", pkt->attribute, pkt->value.len);
      //   uint16_t cccd_value = pkt->value.data[0] | (pkt->value.data[1] << 8);
      //   if (pkt->attribute == gattdb_temp){
      //     if (cccd_value == 0x0001)
      //       printf("TEMP notifications ENABLED\r\n");
      //     else if (cccd_value == 0x0000)
      //       printf("TEMP notifications DISABLED\r\n");
      //   }
      //   else if (pkt->attribute == gattdb_light){
      //     if (cccd_value == 0x0001)
      //       printf("LIGHT notifications ENABLED\r\n");
      //     else if (cccd_value == 0x0000)
      //       printf("LIGHT notifications DISABLED\r\n");
      //   }
      // }
      switch(pkt->attribute)
      {
        case gattdb_led_state: //update all led's
          printf("new state for LED received via GATT: %d\r\n", pkt->value.data[0]);
          sl_btmesh_generic_client_publish(ELEMENT_INDEX, 0x1001, 0, 0, 0, 0, SL_BTMESH_GENERIC_CLIENT_STATE_ON_OFF, pkt->value.len, pkt->value.data);
          break;
        case gattdb_temp_period_ms: //update all temp periods
          printf("new state for temp period received via GATT\r\n");
          sl_btmesh_vendor_model_set_publication (TEMP_ELEMENT, VENDOR_ID, VENDOR_PERIOD, 0, 1, pkt->value.len, pkt->value.data);
          sl_btmesh_vendor_model_publish(TEMP_ELEMENT, VENDOR_ID, VENDOR_PERIOD);
          break;
        case gattdb_light_period_ms: //update all light periods
          printf("new state for light period received via GATT\r\n");
          sl_btmesh_vendor_model_set_publication (LIGHT_ELEMENT, VENDOR_ID, VENDOR_PERIOD, 0, 1, pkt->value.len, pkt->value.data);
          sl_btmesh_vendor_model_publish(LIGHT_ELEMENT, VENDOR_ID, VENDOR_PERIOD);
          break;
        default:
          break;
      }
      break;
    }


    //triggered when bonding/pairing complete
    case sl_bt_evt_sm_bonded_id:
        printf("Pairing process completed\r\n");
        break;

    //triggered when bonding/pairing process fails
    case sl_bt_evt_sm_bonding_failed_id:
      //close the connection
      printf("Bonding/pairing process failed\r\n");
      if(connection_handle != CONNECTION_HANDLE_INVALID) {
          sl_bt_connection_close(connection_handle);
      }
      else{ //edge case, shouldn't occur
          printf("Cannot disconnect: not connected\r\n");
      }
      break;
    ///////////////////////////////////////////////////////////////////////////
    // Add additional event handlers here as your application requires!      //
    ///////////////////////////////////////////////////////////////////////////

    // -------------------------------
    // Default event handler.
    default:
      break;
  }
}

/**************************************************************************//**
 * Bluetooth Mesh stack event handler.
 * This overrides the dummy weak implementation.
 *
 * @param[in] evt Event coming from the Bluetooth Mesh stack.
 *****************************************************************************/
void sl_btmesh_on_event(sl_btmesh_msg_t *evt)
{
  switch (SL_BT_MSG_ID(evt->header)) {
    ///////////////////////////////////////////////////////////////////////////
    // Add additional event handlers here as your application requires!      //
    ///////////////////////////////////////////////////////////////////////////
    case sl_btmesh_evt_node_initialized_id:
      printf("Node initialized\r\n");

      //init vendor models for periods
      sl_btmesh_vendor_model_init(TEMP_ELEMENT, VENDOR_ID, VENDOR_PERIOD, 1, 1, op_codes);
      sl_btmesh_vendor_model_init(LIGHT_ELEMENT, VENDOR_ID, VENDOR_PERIOD, 1, 1, op_codes);
      break;

    case sl_btmesh_evt_node_provisioned_id:
      printf("Node provisioned\r\n");
      break;    

    case sl_btmesh_evt_generic_server_client_request_id:
    {
      printf("Received client request for Generic Server model\r\n");
      sl_btmesh_evt_generic_server_client_request_t * pkt = &(evt->data.evt_generic_server_client_request);
      switch(pkt->model_id)
      {
        case GENERIC_ON_OFF_SERVER: // Generic On Off Server
          //update local model state
          sl_btmesh_generic_server_update(pkt->elem_index, pkt->model_id, 0, pkt->type, pkt->parameters.len, pkt->parameters.data);
          
          //update led state accordingly
          xQueueSend(light_queue, pkt->parameters.data, 0);
          break;
        default:
          break;
      }
      break;
    }

    //event triggered when data is published to group that our clients are subscribed to
    //code assumes only generic level models
    //differentiates which one based on element
    case sl_btmesh_evt_generic_client_server_status_id:
    {
      printf("Received server status for Generic Client model\r\n");
      sl_btmesh_evt_generic_client_server_status_t * pkt = &(evt->data.evt_generic_client_server_status);
      switch(pkt->elem_index)
      {
        case TEMP_ELEMENT:
          uint16_t temp_reading = (uint16_t)pkt->parameters.data[0] | ((uint16_t)pkt->parameters.data[1] << 8);
          printf("Temperature reading: %d\r\n", temp_reading);

          //echo received value to gattdb, notify wifi board
          sl_bt_gatt_server_write_attribute_value(gattdb_temp, 0, pkt->parameters.len, pkt->parameters.data);
          sl_bt_gatt_server_notify_all(gattdb_temp, pkt->parameters.len, pkt->parameters.data);
          break;
        case LIGHT_ELEMENT:
          uint16_t light_reading = (uint16_t)pkt->parameters.data[0] | ((uint16_t)pkt->parameters.data[1] << 8);
          printf("Light reading: %d\r\n", light_reading);

          //echo received value to gattdb, notify wifi board
          sl_bt_gatt_server_write_attribute_value(gattdb_light, 0, pkt->parameters.len, pkt->parameters.data);
          sl_bt_gatt_server_notify_all(gattdb_light, pkt->parameters.len, pkt->parameters.data);
          break;
        default:
          break;
      }
      break;
    }

    // -------------------------------
    // Default event handler.
    default:
      break;
  }
}