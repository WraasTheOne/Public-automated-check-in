
//thies are the libraries that are used in the code wifi and esp32
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


//the tag is used to identify the log messages
#define GATTS_TAG "GATTS_SIMPLE"
//the service uuid and the characteristic uuid
#define SERVICE_UUID   0x00FF
//the characteristic uuid
#define CHAR_UUID      0xFF01
//the device name
#define DEVICE_NAME    "ESP_SIMPLE_GATT"
/** DEFINES **/
#define WIFI_SUCCESS 1 << 0
#define WIFI_FAILURE 1 << 1
#define TCP_SUCCESS 1 << 0
#define TCP_FAILURE 1 << 1
#define MAX_FAILURES 10


//the data type that i can send to the netcat server that listens to the port 12345
typedef enum {
    DATA_TYPE_INT,
    DATA_TYPE_STRING 
} data_type_t;





