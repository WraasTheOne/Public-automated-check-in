#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"

#include "sdkconfig.h"

#define GATTS_TAG "GATTS_SIMPLE"
#define SERVICE_UUID   0x00FF
#define CHAR_UUID      0xFF01
#define DEVICE_NAME    "ESP_SIMPLE_GATT"

// Define global variable to store connected device address
static esp_bd_addr_t connected_device_addr;
static bool device_connected = false;

static uint8_t char_value[4] = {0xde, 0xad, 0xbe, 0xef};

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

static esp_gatt_if_t gatt_if_instance;
static uint16_t service_handle;
static uint16_t char_handle;

static esp_attr_value_t char_val = {
    .attr_max_len = sizeof(char_value),
    .attr_len     = sizeof(char_value),
    .attr_value   = char_value,
};

static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static esp_ble_adv_params_t adv_params = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x40,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

// Function to read RSSI of the connected device
void read_rssi_task(void *pvParameter) {
    while (1) {
        if (device_connected) {
            // Read the RSSI of the connected device
            esp_ble_gap_read_rssi(connected_device_addr);
        }
        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay of 1000 ms (1 second)
    }
}

// GAP event handler
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        esp_ble_gap_start_advertising(&adv_params);
        break;

    case ESP_GAP_BLE_READ_RSSI_COMPLETE_EVT:
        if (param->read_rssi_cmpl.status == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(GATTS_TAG, "RSSI of connected device: %d", param->read_rssi_cmpl.rssi);
        } else {
            ESP_LOGE(GATTS_TAG, "Failed to read RSSI, status = %d", param->read_rssi_cmpl.status);
        }
        break;

    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
        // Store the connected device address and set connected flag
        memcpy(connected_device_addr, param->update_conn_params.bda, sizeof(esp_bd_addr_t));
        device_connected = true;
        ESP_LOGI(GATTS_TAG, "Connected device address: %02x:%02x:%02x:%02x:%02x:%02x",
                 connected_device_addr[0], connected_device_addr[1],
                 connected_device_addr[2], connected_device_addr[3],
                 connected_device_addr[4], connected_device_addr[5]);
        break;

    default:
        break;
    }
}

// GATT server event handler
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    ESP_LOGI(GATTS_TAG, "It's a GATT Event: %d", event);
    switch (event) {
    case 15:
        ESP_LOGI(GATTS_TAG, "Device disconnected, restarting advertising...");
        device_connected = false; // Set flag to false when device disconnects
        esp_ble_gap_start_advertising(&adv_params);
        break;

    case ESP_GATTS_REG_EVT:
        ESP_LOGI(GATTS_TAG, "GATT_REG_EVT, service %d", param->reg.app_id);
        esp_ble_gap_set_device_name(DEVICE_NAME);
        esp_ble_gap_config_adv_data(&adv_data);
        
        esp_ble_gatts_create_service(gatts_if, &((esp_gatt_srvc_id_t){
            .is_primary = true,
            .id.inst_id = 0x00,
            .id.uuid.len = ESP_UUID_LEN_16,
            .id.uuid.uuid.uuid16 = SERVICE_UUID,
        }), 4);
        gatt_if_instance = gatts_if;
        break;

    case ESP_GATTS_CREATE_EVT:
        service_handle = param->create.service_handle;
        esp_ble_gatts_start_service(service_handle);
        esp_ble_gatts_add_char(service_handle, &((esp_bt_uuid_t){
            .len = ESP_UUID_LEN_16,
            .uuid.uuid16 = CHAR_UUID,
        }), ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE, &char_val, NULL);
        break;

    case ESP_GATTS_ADD_CHAR_EVT:
        char_handle = param->add_char.attr_handle;
        ESP_LOGI(GATTS_TAG, "CHAR_HANDLE: %d", char_handle);
        break;

    case ESP_GATTS_WRITE_EVT:
        ESP_LOGI(GATTS_TAG, "Write Event: handle = %d, value =", param->write.handle);
        esp_log_buffer_hex(GATTS_TAG, param->write.value, param->write.len);
        esp_ble_gatts_send_response(gatt_if_instance, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
        break;

    case ESP_GATTS_READ_EVT:
        ESP_LOGI(GATTS_TAG, "Read Event: handle = %d", param->read.handle);
        esp_gatt_rsp_t rsp = {0};
        rsp.attr_value.handle = char_handle;
        rsp.attr_value.len = sizeof(char_value);
        memcpy(rsp.attr_value.value, char_value, sizeof(char_value));
        esp_ble_gatts_send_response(gatt_if_instance, param->read.conn_id, param->read.trans_id, ESP_GATT_OK, &rsp);
        break;

    default:
        break;
    }
}

void app_main(void) {
    esp_err_t ret;

    // Initialize NVS
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));

    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    // Register the GAP and GATT callbacks
    ESP_ERROR_CHECK(esp_ble_gatts_register_callback(gatts_event_handler));
    ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_event_handler));

    ESP_ERROR_CHECK(esp_ble_gatts_app_register(0));

    // Create a FreeRTOS task to read RSSI periodically
    xTaskCreate(&read_rssi_task, "read_rssi_task", 2048, NULL, 5, NULL);
    // Start advertising
    esp_ble_gap_set_device_name(DEVICE_NAME);
    esp_ble_gap_config_adv_data(&adv_data);
}