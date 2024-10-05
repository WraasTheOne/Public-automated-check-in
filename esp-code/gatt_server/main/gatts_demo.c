#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_event.h"
#include "esp_wifi.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"

#define GATTS_TAG "GATTS_SIMPLE"
#define SERVICE_UUID   0x00FF
#define CHAR_UUID      0xFF01
#define DEVICE_NAME    "ESP_SIMPLE_GATT2"
/** DEFINES **/
#define WIFI_SUCCESS 1 << 0
#define WIFI_FAILURE 1 << 1
#define TCP_SUCCESS 1 << 0
#define TCP_FAILURE 1 << 1
#define MAX_FAILURES 10

/** GLOBALS **/

typedef enum {
    DATA_TYPE_INT,
    DATA_TYPE_STRING
} data_type_t;

// event group to contain status information
static EventGroupHandle_t wifi_event_group;

// retry tracker
static int s_retry_num = 0;

// task tag
static const char *TAG = "WIFI";
/** FUNCTIONS **/

//event handler for wifi events
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
	if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
	{
		ESP_LOGI(TAG, "Connecting to AP...");
		esp_wifi_connect();
	} else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
	{
		if (s_retry_num < MAX_FAILURES)
		{
			ESP_LOGI(TAG, "Reconnecting to AP...");
			esp_wifi_connect();
			s_retry_num++;
		} else {
			xEventGroupSetBits(wifi_event_group, WIFI_FAILURE);
		}
	}
}

//event handler for ip events
static void ip_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
	if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
	{
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "STA IP: " IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(wifi_event_group, WIFI_SUCCESS);
    }

}

// connect to wifi and return the result
esp_err_t connect_wifi()
{
	int status = WIFI_FAILURE;

	/** INITIALIZE ALL THE THINGS **/
	//initialize the esp network interface
	ESP_ERROR_CHECK(esp_netif_init());

	//initialize default esp event loop
	ESP_ERROR_CHECK(esp_event_loop_create_default());

	//create wifi station in the wifi driver
	esp_netif_create_default_wifi_sta();

	//setup wifi station with the default wifi configuration
	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    /** EVENT LOOP CRAZINESS **/
	wifi_event_group = xEventGroupCreate();

    esp_event_handler_instance_t wifi_handler_event_instance;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &wifi_handler_event_instance));

    esp_event_handler_instance_t got_ip_event_instance;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &ip_event_handler,
                                                        NULL,
                                                        &got_ip_event_instance));

    /** START THE WIFI DRIVER **/
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = "SanderPhone",
            .password = "123456789",
	     .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .capable = true,
                .required = false
            },
        },
    };

    // set the wifi controller to be a station
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );

    // set the wifi config
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );

    // start the wifi driver
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "STA initialization complete");

    /** NOW WE WAIT **/
    EventBits_t bits = xEventGroupWaitBits(wifi_event_group,
            WIFI_SUCCESS | WIFI_FAILURE,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_SUCCESS) {
        ESP_LOGI(TAG, "Connected to ap");
        status = WIFI_SUCCESS;
    } else if (bits & WIFI_FAILURE) {
        ESP_LOGI(TAG, "Failed to connect to ap");
        status = WIFI_FAILURE;
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
        status = WIFI_FAILURE;
    }

    /* The event will not be processed after unregister */
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, got_ip_event_instance));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_handler_event_instance));
    vEventGroupDelete(wifi_event_group);

    return status;
}

// connect to the server and return the result

esp_err_t connect_tcp_server(void* data, data_type_t type) {

	struct sockaddr_in serverInfo = {0};
	char sendBuffer[64];  // Buffer to hold the message to send

	serverInfo.sin_family = AF_INET;
	serverInfo.sin_addr.s_addr = inet_addr("192.168.5.45"); // Target IP address
	serverInfo.sin_port = htons(12345); // Target port

	// Create a socket
	int sock = socket(AF_INET, SOCK_STREAM, 0);
	if (sock < 0)
	{
		ESP_LOGE(TAG, "Failed to create a socket..?");
		return TCP_FAILURE;
	}

	// Connect to the server
	if (connect(sock, (struct sockaddr *)&serverInfo, sizeof(serverInfo)) != 0)
	{
		ESP_LOGE(TAG, "Failed to connect to %s!", inet_ntoa(serverInfo.sin_addr.s_addr));
		close(sock);
		return TCP_FAILURE;
	}

     if (type == DATA_TYPE_INT) {
        // Assume data is a pointer to an int8_t
        int8_t value = *(int8_t*)data;
        snprintf(sendBuffer, sizeof(sendBuffer), "The value is: %d This is right", value);
    } else if (type == DATA_TYPE_STRING) {
        // Assume data is a pointer to a null-terminated char array
        snprintf(sendBuffer, sizeof(sendBuffer), "The message is: %s \n", (char*)data);
    }
    // Send the message to the server
    int w = write(sock, sendBuffer, strlen(sendBuffer));
    if (w < 0) {
        ESP_LOGE(TAG, "Failed to send data to server");
        close(sock);
        return TCP_FAILURE;
    }

	ESP_LOGI(TAG, "Sent data to server: %s", sendBuffer);

	// Close the socket after sending the data
	close(sock);
	return TCP_SUCCESS;
}

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
            connect_tcp_server(&param->read_rssi_cmpl.rssi, DATA_TYPE_INT);
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
        esp_ble_gap_read_rssi(connected_device_addr);
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

    xTaskCreate(&read_rssi_task, "read_rssi_task", 2048, NULL, 5, NULL);

    // Create a FreeRTOS task to read RSSI periodically
    esp_ble_gap_set_device_name(DEVICE_NAME);
    esp_ble_gap_config_adv_data(&adv_data);
    esp_err_t status = WIFI_FAILURE;
    status = connect_wifi();
	if (WIFI_SUCCESS != status)
	{
		ESP_LOGI(TAG, "Failed to associate to AP, dying...");
		return;
	}

	if (TCP_SUCCESS != status)
	{
		ESP_LOGI(TAG, "Failed to connect to remote server, dying...");
		return;
	}
}