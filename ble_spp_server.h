#ifndef BLE_SPP_SERVER_H
#define BLE_SPP_SERVER_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "driver/uart.h"
#include "string.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>


/// SPP Service
static const uint16_t spp_service_uuid = 0xABF0;
/// Characteristic UUID
#define ESP_GATT_UUID_SPP_DATA_RECEIVE      0xABF1
#define ESP_GATT_UUID_SPP_DATA_NOTIFY       0xABF2
#define ESP_GATT_UUID_SPP_COMMAND_RECEIVE   0xABF3
#define ESP_GATT_UUID_SPP_COMMAND_NOTIFY    0xABF4

#define spp_sprintf(s,...)         sprintf((char*)(s), ##__VA_ARGS__)
#define SPP_DATA_MAX_LEN           (512)
#define SPP_CMD_MAX_LEN            (20)
#define SPP_STATUS_MAX_LEN         (20)
#define SPP_DATA_BUFF_MAX_LEN      (2*1024)

#define SPP_PROFILE_NUM             1
#define SPP_PROFILE_APP_IDX         0
#define ESP_SPP_APP_ID              0x56
#define SPP_SVC_INST_ID	            0

static const uint16_t heart_rate_ctrl_points = ESP_GATT_HEART_RATE_CNTL_POINT;
//uint8_t heart_ctrl_points[0] = {0x00};
uint8_t heart_ctrl_points[8];
void uart_task();
///Attributes State Machine
enum{
    SPP_IDX_SVC,

    SPP_IDX_SPP_DATA_RECV_CHAR,
    SPP_IDX_SPP_DATA_RECV_VAL,

    SPP_IDX_SPP_DATA_NOTIFY_CHAR,
    SPP_IDX_SPP_DATA_NTY_VAL,
    SPP_IDX_SPP_DATA_NTF_CFG, 
    
    //CONFIGS ADICIONAIS DO BLUETOOTH NO APP \/

    // SPP_IDX_SPP_COMMAND_CHAR,
    // SPP_IDX_SPP_COMMAND_VAL,

    // SPP_IDX_SPP_STATUS_CHAR,
    // SPP_IDX_SPP_STATUS_VAL,
    // SPP_IDX_SPP_STATUS_CFG,

    SPP_IDX_NB,
};

static uint16_t spp_mtu_size = 23;
static uint16_t spp_conn_id = 0xffff;
static esp_gatt_if_t spp_gatts_if = 0xff;
QueueHandle_t spp_uart_queue = NULL;
static xQueueHandle cmd_cmd_queue = NULL;

static bool enable_data_ntf = false;
static bool is_connected = false;
static esp_bd_addr_t spp_remote_bda = {0x0,};

static uint16_t spp_handle_table[SPP_IDX_NB];

static esp_ble_adv_params_t spp_adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x40,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy  = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};

typedef struct spp_receive_data_node{
    int32_t len;
    uint8_t * node_buff;
    struct spp_receive_data_node * next_node;
}spp_receive_data_node_t;

static spp_receive_data_node_t * temp_spp_recv_data_node_p1 = NULL;
static spp_receive_data_node_t * temp_spp_recv_data_node_p2 = NULL;

typedef struct spp_receive_data_buff{
    int32_t node_num;
    int32_t buff_size;
    spp_receive_data_node_t * first_node;
}spp_receive_data_buff_t;

static spp_receive_data_buff_t SppRecvDataBuff = {
    .node_num   = 0,
    .buff_size  = 0,
    .first_node = NULL
};

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

/* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */
static struct gatts_profile_inst spp_profile_tab[SPP_PROFILE_NUM] = {
    [SPP_PROFILE_APP_IDX] = {
        .gatts_cb = gatts_profile_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
};

/* btc_ble_gatts_args_t */
typedef union {
    //BTC_GATTS_ACT_APP_REGISTER = 0,
    struct app_reg_args {
        uint16_t app_id;
    } app_reg;

    //BTC_GATTS_ACT_APP_UNREGISTER,
    struct app_unreg_args {
        esp_gatt_if_t gatts_if;
    } app_unreg;

    //BTC_GATTS_ACT_CREATE_SERVICE,
    struct create_srvc_args {
        esp_gatt_if_t gatts_if;
        esp_gatt_srvc_id_t service_id;
        uint16_t num_handle;
    } create_srvc;

    //BTC_GATTS_ACT_CREATE_ATTR_TAB
    struct create_attr_tab_args{
        esp_gatt_if_t gatts_if;
        uint8_t srvc_inst_id;
        uint8_t max_nb_attr;
        esp_gatts_attr_db_t *gatts_attr_db;
    }create_attr_tab;

    //BTC_GATTS_ACT_DELETE_SERVICE,
    struct delete_srvc_args {
        uint16_t service_handle;
    } delete_srvc;

    //BTC_GATTS_ACT_START_SERVICE,
    struct start_srvc_args {
        uint16_t service_handle;
    } start_srvc;

    //BTC_GATTS_ACT_STOP_SERVICE,
    struct stop_srvc_args {
        uint16_t service_handle;
    } stop_srvc;

    //BTC_GATTS_ACT_ADD_INCLUDE_SERVICE,
    struct add_incl_srvc_args {
        uint16_t service_handle;
        uint16_t included_service_handle;
    } add_incl_srvc;

    //BTC_GATTS_ACT_ADD_CHAR,
    struct add_char_args {
        uint16_t service_handle;
        esp_bt_uuid_t char_uuid;
        esp_gatt_perm_t perm;
        esp_gatt_char_prop_t property;
        esp_attr_control_t attr_control;
        esp_attr_value_t char_val;
    } add_char;

    //BTC_GATTS_ACT_ADD_CHAR_DESCR,
    struct add_descr_args {
        uint16_t  service_handle;
        esp_bt_uuid_t descr_uuid;
        esp_gatt_perm_t perm;
        esp_attr_control_t attr_control;
        esp_attr_value_t descr_val;
    } add_descr;

    //BTC_GATTS_ACT_SEND_INDICATE,
    struct send_indicate_args {
        uint16_t conn_id;
        uint16_t attr_handle;
        bool need_confirm;
        uint16_t value_len;
        uint8_t *value;
    } send_ind;

    //BTC_GATTS_ACT_SEND_RESPONSE,
    struct send_rsp_args {
        uint16_t conn_id;
        uint32_t trans_id;
        esp_gatt_status_t status;
        esp_gatt_rsp_t *rsp;
    } send_rsp;

    //BTC_GATTS_SET_ATTR_VALUE
    struct set_attr_val_args {
        uint16_t handle;
        uint16_t length;
        uint8_t *value;
    } set_attr_val;

    //BTC_GATTS_ACT_OPEN,
    struct open_args {
        esp_gatt_if_t gatts_if;
        esp_bd_addr_t remote_bda;
        bool is_direct;
    } open;

    //BTC_GATTS_ACT_CLOSE,
    struct close_args {
        uint16_t conn_id;
    } close;

    //BTC_GATTS_ACT_SEND_SERVICE_CHANGE,
    struct send_service_change_args {
        esp_gatt_if_t gatts_if;
        esp_bd_addr_t remote_bda;
    } send_service_change;

} btc_ble_gatts_args_t;

/*
 *  SPP PROFILE ATTRIBUTES
 ****************************************************************************************
 */

#define CHAR_DECLARATION_SIZE   (sizeof(uint8_t))
static const uint16_t primary_service_uuid = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;

static const uint8_t char_prop_read_notify = ESP_GATT_CHAR_PROP_BIT_READ|ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t char_prop_read_write = ESP_GATT_CHAR_PROP_BIT_WRITE_NR|ESP_GATT_CHAR_PROP_BIT_READ;

///SPP Service - data receive characteristic, read&write without response
static const uint16_t spp_data_receive_uuid = ESP_GATT_UUID_SPP_DATA_RECEIVE;
static const uint8_t  spp_data_receive_val[20] = {0x00};

///SPP Service - data notify characteristic, notify&read
static const uint16_t spp_data_notify_uuid = ESP_GATT_UUID_SPP_DATA_NOTIFY;
static const uint8_t  spp_data_notify_val[20] = {0x00};
static const uint8_t  spp_data_notify_ccc[2] = {0x00, 0x00};

///SPP Service - command characteristic, read&write without response
static const uint16_t spp_command_uuid = ESP_GATT_UUID_SPP_COMMAND_RECEIVE;
static const uint8_t  spp_command_val[10] = {0x00};

///SPP Service - status characteristic, notify&read
static const uint16_t spp_status_uuid = ESP_GATT_UUID_SPP_COMMAND_NOTIFY;
static const uint8_t  spp_status_val[10] = {0x00};
static const uint8_t  spp_status_ccc[2] = {0x00, 0x00};

///Full HRS Database Description - Used to add attributes into the database
static const esp_gatts_attr_db_t spp_gatt_db[SPP_IDX_NB] = //ESP_GATT_AUTO_RSP
{
    //SPP -  Service Declaration
    [SPP_IDX_SVC]                      	=
    {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ,
    sizeof(spp_service_uuid), sizeof(spp_service_uuid), (uint8_t *)&spp_service_uuid}},

    //SPP -  data receive characteristic Declaration
    [SPP_IDX_SPP_DATA_RECV_CHAR]            = 
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
    CHAR_DECLARATION_SIZE,CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},

    //SPP -  data receive characteristic Value                    FILA 1
    [SPP_IDX_SPP_DATA_RECV_VAL]             	=
    {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_16, (uint8_t *)&spp_data_receive_uuid, ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE,
    SPP_DATA_MAX_LEN,sizeof(spp_data_receive_val), (uint8_t *)spp_data_receive_val}},

    //SPP -  data notify characteristic Declaration
    [SPP_IDX_SPP_DATA_NOTIFY_CHAR]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
    CHAR_DECLARATION_SIZE,CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_notify}},

    //SPP -  data notify characteristic Value                       FILA 2
    [SPP_IDX_SPP_DATA_NTY_VAL]   =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&spp_data_notify_uuid, ESP_GATT_PERM_READ,
    SPP_DATA_MAX_LEN, sizeof(spp_data_notify_val), (uint8_t *)spp_data_notify_val}},
    
    // CONFIGS ADICIONAIS DO BLUETOOTH NO APP
    //SPP -  data notify characteristic - Client Characteristic Configuration Descriptor  //FILA 3
    [SPP_IDX_SPP_DATA_NTF_CFG]         =                             
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE,
    sizeof(uint16_t),sizeof(spp_data_notify_ccc), (uint8_t *)spp_data_notify_ccc}},

    // //SPP -  command characteristic Declaration
    // [SPP_IDX_SPP_COMMAND_CHAR]            =
    // {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
    // CHAR_DECLARATION_SIZE,CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},

    // //SPP -  command characteristic Value
    // [SPP_IDX_SPP_COMMAND_VAL]                 =
    // {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_16, (uint8_t *)&spp_command_uuid, ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE,
    // SPP_CMD_MAX_LEN,sizeof(spp_command_val), (uint8_t *)spp_command_val}},

    // //SPP -  status characteristic Declaration
    // [SPP_IDX_SPP_STATUS_CHAR]            =
    // {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
    // CHAR_DECLARATION_SIZE,CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_notify}},

    // //SPP -  status characteristic Value
    // [SPP_IDX_SPP_STATUS_VAL]                 =
    // {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_16, (uint8_t *)&spp_status_uuid, ESP_GATT_PERM_READ,
    // SPP_STATUS_MAX_LEN,sizeof(spp_status_val), (uint8_t *)spp_status_val}},

    // //SPP -  status characteristic - Client Characteristic Configuration Descriptor
    // [SPP_IDX_SPP_STATUS_CFG]         =
    // {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE,
    // sizeof(uint16_t),sizeof(spp_status_ccc), (uint8_t *)spp_status_ccc}},

};

#endif