#include "ble_spp_server.h"

#define GATTS_TABLE_TAG "BT"
#define CARAC_DEVICE_NAME "ESP_SPP_SERVER" // Nome nas caracteristicas do GAP
uint8_t appdata[16];
uint8_t printdata[16];

uint16_t c = 0;

static const uint8_t spp_adv_data[23] = {
    /* Flags */
    0x02, 0x01, 0x06,
    /* Lista completa de UUIDs de classe de serviço de 16 bits */
    0x03, 0x03, 0xF0, 0xAB,
    /* Nome local completo na publicidade */
    0x0F, 0x09, 'E', 'S', 'P', '_', 'S', 'P', 'P', '_', 'S', 'E', 'R', 'V', 'E', 'R' // Nome do Dispositivo do BLE
};

static uint8_t find_char_and_desr_index(uint16_t handle)
{
    uint8_t error = 0xff;
    for (int i = 0; i < SPP_IDX_NB; i++)
    {
        if (handle == spp_handle_table[i])
        {
            return i;
        }
    }
    return error;
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    esp_err_t err;
    ESP_LOGE(GATTS_TABLE_TAG, "GAP_EVT, event %d\n", event);

    switch (event)
    {
    case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
        esp_ble_gap_start_advertising(&spp_adv_params);
        break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        // evento de início de publicidade, indica se a publicidade foi iniciada com êxito ou com falha
        if ((err = param->adv_start_cmpl.status) != ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGE(GATTS_TABLE_TAG, "Falha no início da publicidade: %s\n", esp_err_to_name(err));
        }
        break;
    default:
        break;
    }
}

static void sendreceiv(esp_ble_gatts_cb_param_t *param, esp_gatt_if_t gatts_if, uint16_t *dado, uint16_t size)
{
    esp_gatt_rsp_t rsp;
    if (size > 21)
    {
        printf("Max Size 21!\n");
        size = 21;
    }
    memset(&rsp, 0, sizeof(size));              // memoria para evitar lixo
    rsp.attr_value.handle = param->read.handle; // parametro de lida
    rsp.attr_value.len = size;                  // largura da escrita
    for (int i = 0; i < size; i++)
    {
        rsp.attr_value.value[i] = dado[i];
    }
    esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id, ESP_GATT_OK, &rsp);
    sprintf((char *)dado, "%.*s", rsp.attr_value.len, (char *)rsp.attr_value.value);
    printf("Dado: %s\n", (char *)dado);
    return;
}

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    uint8_t res = 0xff;
    uint8_t ress = 0xff;
    esp_ble_gatts_cb_param_t *p_data = (esp_ble_gatts_cb_param_t *)param;
    btc_ble_gatts_args_t args;
    esp_gatt_rsp_t rsp;

    ESP_LOGI(GATTS_TABLE_TAG, "event = %x\n", event);
    switch (event)
    {
    case ESP_GATTS_REG_EVT:
        ESP_LOGI(GATTS_TABLE_TAG, "%s %d\n", __func__, __LINE__);
        esp_ble_gap_set_device_name(CARAC_DEVICE_NAME);

        ESP_LOGI(GATTS_TABLE_TAG, "%s %d\n", __func__, __LINE__);
        esp_ble_gap_config_adv_data_raw((uint8_t *)spp_adv_data, sizeof(spp_adv_data));

        ESP_LOGI(GATTS_TABLE_TAG, "%s %d\n", __func__, __LINE__);
        esp_ble_gatts_create_attr_tab(spp_gatt_db, gatts_if, SPP_IDX_NB, SPP_SVC_INST_ID);
        break;

    case ESP_GATTS_WRITE_EVT: // FUNÇAO DE ESCRITA DO APP PARA O TERMINAL ESP
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_WRITE_EVT : handle = %d\n", res);
        res = find_char_and_desr_index(p_data->write.handle);
        if(res == SPP_IDX_SPP_DATA_NTF_CFG) //config no app de notificação do print, ativada ou desativada,
        {
            if((p_data->write.len == 2)&&(p_data->write.value[0] == 0x01)&&(p_data->write.value[1] == 0x00)){
                ESP_LOGE(GATTS_TABLE_TAG, "Notify ON");
                enable_data_ntf = true;
            }else if((p_data->write.len == 2)&&(p_data->write.value[0] == 0x00)&&(p_data->write.value[1] == 0x00)){
                ESP_LOGE(GATTS_TABLE_TAG, "Notify OFF");
                enable_data_ntf = false;
            }
        }
        else if (res == SPP_IDX_SPP_DATA_RECV_VAL)
        {
            sprintf((char *)&appdata, "%.*s", param->write.len, param->write.value); //Armazena o dado de escrita enviado pelo app em appdata
            printf("Dado: %s\n", (char *)&appdata); //printa o dado newinfo no terminal

            sprintf((char *)&printdata, "%d", c++); //contador para ser mostrado no app a cada mensagem mandada
            //manda o dado printdata no evento 5(SPP_IDX_SPP_DATA_NTY_VAL) que mostra no app o contador
            esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, spp_handle_table[SPP_IDX_SPP_DATA_NTY_VAL],sizeof(printdata), printdata, false);
        }
        break;

    case ESP_GATTS_READ_EVT: // LEITURA DO APP
    {   
        res = find_char_and_desr_index(p_data->read.handle);
        if (res == SPP_IDX_SPP_DATA_RECV_VAL){ 
            uint16_t dado[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9', '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', '0', '1'};
            sendreceiv(param, gatts_if, &dado, 21); //função que printa o 'dado' no app | (chama ESP_GATTS_RESPONSE_EVT 1)
        }

        if (res == SPP_IDX_SPP_DATA_NTY_VAL){ 
            sprintf((char *)&appdata, "%d", c++); //outro contador
            esp_ble_gatts_set_attr_value(param->read.handle, sizeof(&appdata), &appdata); //função que printa o 'dado' no app | chama ESP_GATTS_SET_ATTR_VAL_EVT 2
            printf("Dado a ser lido: %s\n",appdata); //mostra o dado escrito
        }

        if (res == SPP_IDX_SPP_DATA_NTF_CFG){
            printf("SPP_IDX_SPP_DATA_NTF_CFG\n");
        }
        break;
    }
    case ESP_GATTS_SET_ATTR_VAL_EVT: // 2
        printf("ESP_GATTS_SET_ATTR_VAL_EVT\n");
        break;
    case ESP_GATTS_RESPONSE_EVT: // 1
        printf("ESP_GATTS_RESPONSE_EVT\n");
        break;
    case ESP_GATTS_MTU_EVT:
        spp_mtu_size = p_data->mtu.mtu;
        break;
    case ESP_GATTS_CONNECT_EVT:
        spp_conn_id = p_data->connect.conn_id;
        spp_gatts_if = gatts_if;
        is_connected = true;
        memcpy(&spp_remote_bda, &p_data->connect.remote_bda, sizeof(esp_bd_addr_t));
        break;
    case ESP_GATTS_DISCONNECT_EVT:
        is_connected = false;
        enable_data_ntf = false;
        esp_ble_gap_start_advertising(&spp_adv_params);
        break;
    case ESP_GATTS_CREAT_ATTR_TAB_EVT:
    {
        ESP_LOGI(GATTS_TABLE_TAG, "The number handle =%x\n", param->add_attr_tab.num_handle);
        if (param->add_attr_tab.status != ESP_GATT_OK)
        {
            ESP_LOGE(GATTS_TABLE_TAG, "Create attribute table failed, error code=0x%x", param->add_attr_tab.status);
        }
        else if (param->add_attr_tab.num_handle != SPP_IDX_NB)
        {
            ESP_LOGE(GATTS_TABLE_TAG, "Create attribute table abnormally, num_handle (%d) doesn't equal to HRS_IDX_NB(%d)", param->add_attr_tab.num_handle, SPP_IDX_NB);
        }
        else
        {
            memcpy(spp_handle_table, param->add_attr_tab.handles, sizeof(spp_handle_table));
            esp_ble_gatts_start_service(spp_handle_table[SPP_IDX_SVC]);
        }
        break;
    }
    default:
        break;
    }
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    ESP_LOGI(GATTS_TABLE_TAG, "EVT %d, gatts if %d\n", event, gatts_if);

    /* If event is register event, store the gatts_if for each profile */
    if (event == ESP_GATTS_REG_EVT)
    {
        if (param->reg.status == ESP_GATT_OK)
        {
            spp_profile_tab[SPP_PROFILE_APP_IDX].gatts_if = gatts_if;
        }
        else
        {
            ESP_LOGI(GATTS_TABLE_TAG, "Reg app failed, app_id %04x, status %d\n", param->reg.app_id, param->reg.status);
            return;
        }
    }
    do
    {
        int idx;
        for (idx = 0; idx < SPP_PROFILE_NUM; idx++)
        {
            if (gatts_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
                gatts_if == spp_profile_tab[idx].gatts_if)
            {
                if (spp_profile_tab[idx].gatts_cb)
                {
                    spp_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);
}

static void btcheck()
{
    esp_err_t ret;
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();

    // Initialize NVS
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    ret = esp_bt_controller_init(&bt_cfg);
    if (ret)
    {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret)
    {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(GATTS_TABLE_TAG, "%s init bluetooth\n", __func__);
    ret = esp_bluedroid_init();
    if (ret)
    {
        ESP_LOGE(GATTS_TABLE_TAG, "%s init bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }
    ret = esp_bluedroid_enable();
    if (ret)
    {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }
    return;
}

void app_main(void)
{
    btcheck();
    esp_ble_gatts_register_callback(gatts_event_handler);
    esp_ble_gap_register_callback(gap_event_handler);
    esp_ble_gatts_app_register(ESP_SPP_APP_ID);
    return;
}
