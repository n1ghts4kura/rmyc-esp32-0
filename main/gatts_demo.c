/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 * 
 * @author *espressif coders*, n1ghts4kura
 * @date 25/5/31
 * @link github.com/n1ghts4kura
 * 
 * Thanks to the great coders.
 */


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gatt_common_api.h"
#include "driver/uart.h"
#include "driver/gpio.h"

#include "sdkconfig.h"

#define GATTS_TAG "GATTS_APP"

#define DELAY(x) \
    do { \
        vTaskDelay((x) / portTICK_PERIOD_MS); \
    } while (0);

// 信息数据队列实现 蓝牙
#define MSG_LEN_MAX 256  // 256 / 2 = 128个ascii字
typedef struct ble_queue_node_t {
    // esp_gatt_value_t value;
    uint8_t *value;
    struct ble_queue_node_t *next;
} ble_queue_node_t ;
typedef struct ble_queue_t {
    ble_queue_node_t *front;
    ble_queue_node_t *rear;
} ble_queue_t ;
static ble_queue_t ble_queue;
static void ble_queue_init() {
    ble_queue.front = NULL;
    ble_queue.rear  = NULL;
}
static bool ble_queue_is_empty() {
    return ble_queue.front == NULL;
}
static bool ble_queue_append(uint8_t *val) {
    ble_queue_node_t *new_node = (ble_queue_node_t *)malloc(sizeof(ble_queue_node_t));
    if (!new_node) {
        ESP_LOGE(GATTS_TAG, "failed to append to ble_queue! (memory)\n");
        return false;
    }
    new_node->value = NULL;
    memcpy(new_node->value, (const uint8_t *)val, MSG_LEN_MAX); // Check if memory go die. LOL
    new_node->next = NULL;
    if (ble_queue_is_empty()) {
        ble_queue.front = new_node;
        ble_queue.rear  = new_node;
    } else {
        ble_queue.rear->next = new_node;
        ble_queue.rear = new_node;
    }
    return true;
}
static bool ble_queue_pop(uint8_t *val) {
    if (ble_queue_is_empty()) {
        ESP_LOGE(GATTS_TAG, "failed to pop ble_queue! (empty queue)\n");
        return false;
    }
    ble_queue_node_t *tmp = ble_queue.front;
    memcpy(val, (const uint8_t *)tmp->value, MSG_LEN_MAX); // Check if memory go die here. LOL
    ble_queue.front = ble_queue.front->next;
    if (ble_queue.front == NULL) {
        ble_queue.rear = NULL;
    }
    free(tmp);
    return true;
}

// 信息数据队列实现 UART
typedef struct uart_queue_node_t {
    uint8_t *value;
    struct uart_queue_node_t *next;
} uart_queue_node_t ;
typedef struct uart_queue_t {
    uart_queue_node_t *front;
    uart_queue_node_t *rear;
} uart_queue_t ;
static uart_queue_t uart_queue;
static void uart_queue_init() {
    uart_queue.front = NULL;
    uart_queue.rear  = NULL;
}
static bool uart_queue_is_empty() {
    return uart_queue.front == NULL;
}
static bool uart_queue_append(uint8_t *val) {
    uart_queue_node_t *new_node = (uart_queue_node_t *)malloc(sizeof(uart_queue_node_t));
    if (!new_node) {
        ESP_LOGE(GATTS_TAG, "failed to append to uart_queue! (memory)\n");
        return false;
    }
    new_node->value = NULL;
    memcpy(new_node->value, (const uint8_t *)val, MSG_LEN_MAX); // Check if memory go die LOL.
    new_node->next = NULL;
    if (uart_queue_is_empty()) {
        uart_queue.front = new_node;
        uart_queue.rear  = new_node;
    } else {
        uart_queue.rear->next = new_node;
        uart_queue.rear = new_node;
    }
    return true;
}
static bool uart_queue_pop(uint8_t *val) {
    if (uart_queue_is_empty()) {
        ESP_LOGE(GATTS_TAG, "failed to pop uart_queue! (empty queue)\n");
        return false;
    }
    uart_queue_node_t *tmp = uart_queue.front;
    memcpy(val, tmp->value, MSG_LEN_MAX); // Check
    uart_queue.front = uart_queue.front->next;
    if (uart_queue.front == NULL) {
        uart_queue.rear = NULL;
    }
    free(tmp);
    return true;
}

// UART 配置
QueueHandle_t hw_uart_queue;
const uart_port_t uart_num = UART_NUM_2;
const int uart_buffer_size = (1024 * 2);
uart_config_t uart_config = {
    .baud_rate = 115200,
    .data_bits = UART_DATA_8_BITS,
    .parity    = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    // .rx_flow_ctrl_thresh
};
static void hw_uart_init() {
    uart_set_pin(uart_num, GPIO_NUM_4, GPIO_NUM_5, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
    ESP_ERROR_CHECK(
        uart_driver_install(
            uart_num,
            uart_buffer_size,
            uart_buffer_size,
            10,
            &hw_uart_queue,
            0
        )
    );
    uart_set_mode(uart_num, UART_MODE_UART);
}

// 任务定义
void task_uart_send() {
    while(true) {
        if (ble_queue_is_empty()) {
            DELAY(500);
            continue;
        }
        char send_data[MSG_LEN_MAX] = {""};
        ble_queue_pop((uint8_t *)send_data);
        uart_write_bytes(uart_num, (const char *)send_data, strlen(send_data));
        ESP_LOGI(GATTS_TAG, "Uart sending: [%s]\n", send_data);
        free(send_data);
    }
}

void task_uart_recv() {
    while(true) {
        int length = 0;
        char recv_data[MSG_LEN_MAX] = {""};
        uart_get_buffered_data_len(uart_num, (size_t *)length);
        if (length > 0) {
            uart_read_bytes(uart_num, recv_data, length, 100);
            uart_queue_append((uint8_t *)recv_data);
            ESP_LOGI(GATTS_TAG, "Uart recving: [%s]\n", recv_data);
            DELAY(500);
        }
        uart_flush(uart_num);
        free(recv_data);
    }
}


static void my_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

// GATTS 服务定义
#define GATTS_SERVICE_UUID_TEST_A   0x00FF // Service UUID
#define GATTS_CHAR_UUID_TEST_A      0xFF01 // Characteristic UUID
#define GATTS_DEMO_CHAR_VAL_LEN_MAX 0x40   // Characteristic 名称最长长度
#define GATTS_DESCR_UUID_TEST_A     0x3333 // Descriptor UUID
#define GATTS_NUM_HANDLE_TEST_A     1      // Max connecting device number

// ESP32 蓝牙设备名称
// 若有多个设备同时使用 记得修改不同esp32设备的名称
static char test_device_name[ESP_BLE_ADV_NAME_LEN_MAX] = "RMYC_ESP32_1";

// 设备制造商的数据
// *Currently unused* *暂时未使用*
// #define TEST_MANUFACTURER_DATA_LEN  17

// 蓝牙写缓冲区大小 (byte)
#define PREPARE_BUF_MAX_SIZE 1024

// Characteristic 初始数据
static uint8_t char1_str[] = {0x11,0x22,0x33};
// Characteristic 状态变量
static esp_gatt_char_prop_t chrc_property = 0;
// Characteristic 属性结构
static esp_attr_value_t gatts_demo_char1_val =
{
    .attr_max_len = GATTS_DEMO_CHAR_VAL_LEN_MAX,
    .attr_len     = sizeof(char1_str),
    .attr_value   = char1_str,
};
// 蓝牙广播配置 标志变量
/**
 * 初始值为0
 * 广播数据先设置完成 此时调用adv_config_flag 值为?
 * 然后不知道....自己看代码吧 应该会用就行
 */
static uint8_t adv_config_done = 0;
#define adv_config_flag      (1 << 0) // 设置广播数据完成
#define scan_rsp_config_flag (1 << 1) // 开始扫描
// Service UUID
static uint8_t adv_service_uuid128[32] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    //first uuid, 16bit, [12],[13] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xEE, 0x00, 0x00, 0x00,
    //second uuid, 32bit, [12], [13], [14], [15] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00,
};

// The length of adv data must be less than 31 bytes
// static uint8_t test_manufacturer[TEST_MANUFACTURER_DATA_LEN] =  {0x12, 0x23, 0x45, 0x56};
// adv data
// 蓝牙广播时发送的数据
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = false,
    .min_interval = 0x0006, //slave connection min interval, Time = min_interval * 1.25 msec
    .max_interval = 0x0010, //slave connection max interval, Time = max_interval * 1.25 msec
    .appearance = 0x00,
    .manufacturer_len = 0, //TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data =  NULL, //&test_manufacturer[0],
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(adv_service_uuid128),
    .p_service_uuid = adv_service_uuid128,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};
// scan response data
// 如何响应 扫描操作 的一个struct
static esp_ble_adv_data_t scan_rsp_data = {
    .set_scan_rsp = true,
    .include_name = true,
    .include_txpower = true,
    //.min_interval = 0x0006,
    //.max_interval = 0x0010,
    .appearance = 0x00,
    .manufacturer_len = 0, //TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data =  NULL, //&test_manufacturer[0],
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(adv_service_uuid128),
    .p_service_uuid = adv_service_uuid128,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};
// 广播参数配置
static esp_ble_adv_params_t adv_params = {
    .adv_int_min        = 0x20, // 广播间隔时间 Adv interval time
    .adv_int_max        = 0x40, // ^^^
    .adv_type           = ADV_TYPE_IND, // 广播模式
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC, 
    //.peer_addr            =
    //.peer_addr_type       =
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY, // 连接设备过滤 默认为不过滤
};

#define PROFILE_NUM 1
#define PROFILE_APP_ID 0
// GATTS服务 APP定义(吧?)
struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb; // 响应连接等事件的函数
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

/* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */
static struct gatts_profile_inst gl_profile_tab[PROFILE_NUM] = {
    [PROFILE_APP_ID] = {
        .gatts_cb = my_profile_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
};
// 蓝牙写缓冲区
typedef struct {
    uint8_t                 *prepare_buf;
    int                     prepare_len;
} prepare_type_env_t;
static prepare_type_env_t prepare_write_env;

void example_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param);
void example_exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param);

// GAP 响应器
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT: // 广播数据设置完毕
        adv_config_done &= (~adv_config_flag);
        if (adv_config_done == 0){
            esp_ble_gap_start_advertising(&adv_params); // 开始广播
        }
        break;
    case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT: // 扫描操作响应数据设置完毕
        adv_config_done &= (~scan_rsp_config_flag);
        if (adv_config_done == 0){
            esp_ble_gap_start_advertising(&adv_params); // 开始广播
        }
        break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT: // 启动广播完成
        //advertising start complete event to indicate advertising start successfully or failed
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(GATTS_TAG, "Advertising start failed, status %d", param->adv_start_cmpl.status);
            break;
        }
        ESP_LOGI(GATTS_TAG, "Advertising start successfully");
        break;
    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT: // 停止广播完成
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(GATTS_TAG, "Advertising stop failed, status %d", param->adv_stop_cmpl.status);
            break;
        }
        ESP_LOGI(GATTS_TAG, "Advertising stop successfully");
        break;
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT: // 连接参数变动
         ESP_LOGI(GATTS_TAG, "Connection params update, status %d, conn_int %d, latency %d, timeout %d",
                  param->update_conn_params.status,
                  param->update_conn_params.conn_int,
                  param->update_conn_params.latency,
                  param->update_conn_params.timeout);
        break;
    case ESP_GAP_BLE_SET_PKT_LENGTH_COMPLETE_EVT: // IDK xD
        ESP_LOGI(GATTS_TAG, "Packet length update, status %d, rx %d, tx %d",
                  param->pkt_data_length_cmpl.status,
                  param->pkt_data_length_cmpl.params.rx_len,
                  param->pkt_data_length_cmpl.params.tx_len);
        break;
    default:
        break;
    }
}

// 将缓冲区数据 输出
void example_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param){
    esp_gatt_status_t status = ESP_GATT_OK;
    if (param->write.need_rsp){
        if (param->write.is_prep) {
            if (param->write.offset > PREPARE_BUF_MAX_SIZE) {
                status = ESP_GATT_INVALID_OFFSET;
            } else if ((param->write.offset + param->write.len) > PREPARE_BUF_MAX_SIZE) {
                status = ESP_GATT_INVALID_ATTR_LEN;
            }
            if (status == ESP_GATT_OK && prepare_write_env->prepare_buf == NULL) {
                prepare_write_env->prepare_buf = (uint8_t *)malloc(PREPARE_BUF_MAX_SIZE*sizeof(uint8_t));
                prepare_write_env->prepare_len = 0;
                if (prepare_write_env->prepare_buf == NULL) {
                    ESP_LOGE(GATTS_TAG, "Gatt_server prep no mem");
                    status = ESP_GATT_NO_RESOURCES;
                }
            }

            esp_gatt_rsp_t *gatt_rsp = (esp_gatt_rsp_t *)malloc(sizeof(esp_gatt_rsp_t));
            if (gatt_rsp) {
                gatt_rsp->attr_value.len = param->write.len;
                gatt_rsp->attr_value.handle = param->write.handle;
                gatt_rsp->attr_value.offset = param->write.offset;
                gatt_rsp->attr_value.auth_req = ESP_GATT_AUTH_REQ_NONE;
                memcpy(gatt_rsp->attr_value.value, param->write.value, param->write.len);
                esp_err_t response_err = esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, gatt_rsp);
                if (response_err != ESP_OK){
                    ESP_LOGE(GATTS_TAG, "Send response error\n");
                }
                free(gatt_rsp);
            } else {
                ESP_LOGE(GATTS_TAG, "malloc failed, no resource to send response error\n");
                status = ESP_GATT_NO_RESOURCES;
            }
            if (status != ESP_GATT_OK){
                return;
            }
            memcpy(prepare_write_env->prepare_buf + param->write.offset,
                   param->write.value,
                   param->write.len);
            prepare_write_env->prepare_len += param->write.len;

        }else{
            esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, NULL);
        }
    }
}

// 监测缓冲区写入事件
void example_exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param){
    if (param->exec_write.exec_write_flag == ESP_GATT_PREP_WRITE_EXEC){
        ESP_LOG_BUFFER_HEX(GATTS_TAG, prepare_write_env->prepare_buf, prepare_write_env->prepare_len);
    }else{
        ESP_LOGI(GATTS_TAG,"Prepare write cancel");
    }
    if (prepare_write_env->prepare_buf) {
        free(prepare_write_env->prepare_buf);
        prepare_write_env->prepare_buf = NULL;
    }
    prepare_write_env->prepare_len = 0;
}

/**
 * 最核心代码
 * 处理连接
 */
static void my_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    switch (event) {
    case ESP_GATTS_REG_EVT: // app开始注册
        ESP_LOGI(GATTS_TAG, "GATT server register, status %d, app_id %d, gatts_if %d", param->reg.status, param->reg.app_id, gatts_if);
        gl_profile_tab[PROFILE_APP_ID].service_id.is_primary = true;
        gl_profile_tab[PROFILE_APP_ID].service_id.id.inst_id = 0x00;
        gl_profile_tab[PROFILE_APP_ID].service_id.id.uuid.len = ESP_UUID_LEN_16;
        gl_profile_tab[PROFILE_APP_ID].service_id.id.uuid.uuid.uuid16 = GATTS_SERVICE_UUID_TEST_A;

        esp_err_t set_dev_name_ret = esp_ble_gap_set_device_name(test_device_name);
        if (set_dev_name_ret){
            ESP_LOGE(GATTS_TAG, "set device name failed, error code = %x", set_dev_name_ret);
        }
        esp_err_t ret = esp_ble_gap_config_adv_data(&adv_data);
        if (ret){
            ESP_LOGE(GATTS_TAG, "config adv data failed, error code = %x", ret);
        }
        adv_config_done |= adv_config_flag;
        //config scan response data
        ret = esp_ble_gap_config_adv_data(&scan_rsp_data);
        if (ret){
            ESP_LOGE(GATTS_TAG, "config scan response data failed, error code = %x", ret);
        }
        adv_config_done |= scan_rsp_config_flag;
        esp_ble_gatts_create_service(gatts_if, &gl_profile_tab[PROFILE_APP_ID].service_id, GATTS_NUM_HANDLE_TEST_A);
        break;
    case ESP_GATTS_READ_EVT: { // client端获取数据 server端发送数据
        ESP_LOGI(GATTS_TAG, "Characteristic read, conn_id %d, trans_id %" PRIu32 ", handle %d", param->read.conn_id, param->read.trans_id, param->read.handle);
        esp_gatt_rsp_t rsp;
        memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
        rsp.attr_value.handle = param->read.handle;
       
        // TODO
        char data[MSG_LEN_MAX] = "";
        uart_queue_pop((uint8_t *)data);
        rsp.attr_value.len = strlen(data);
        memcpy(rsp.attr_value.value, (const uint8_t *)data, rsp.attr_value.len);

        esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id,
                                    ESP_GATT_OK, &rsp);

        free(data);
        break;
    }
    case ESP_GATTS_WRITE_EVT: { // client端写入数据 server端获取信息
        ESP_LOGI(GATTS_TAG, "Characteristic write, conn_id %d, trans_id %" PRIu32 ", handle %d", param->write.conn_id, param->write.trans_id, param->write.handle);

        char data[MSG_LEN_MAX] = "";
        memcpy(data, (const char *)param->write.value, param->write.len); 
        ble_queue_append((uint8_t *)data); 
        free(data);

        if (!param->write.is_prep){
            ESP_LOGI(GATTS_TAG, "value len %d, value ", param->write.len);
            ESP_LOG_BUFFER_HEX(GATTS_TAG, param->write.value, param->write.len);
            if (gl_profile_tab[PROFILE_APP_ID].descr_handle == param->write.handle && param->write.len == 2){
                uint16_t descr_value = param->write.value[1]<<8 | param->write.value[0];
                if (descr_value == 0x0001){
                    if (chrc_property & ESP_GATT_CHAR_PROP_BIT_NOTIFY){
                        ESP_LOGI(GATTS_TAG, "Notification enable");
                        uint8_t notify_data[15];
                        for (int i = 0; i < sizeof(notify_data); ++i)
                        {
                            notify_data[i] = i%0xff;
                        }
                        //the size of notify_data[] need less than MTU size
                        esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, gl_profile_tab[PROFILE_APP_ID].char_handle,
                                                sizeof(notify_data), notify_data, false);
                    }
                }else if (descr_value == 0x0002){
                    if (chrc_property & ESP_GATT_CHAR_PROP_BIT_INDICATE){
                        ESP_LOGI(GATTS_TAG, "Indication enable");
                        uint8_t indicate_data[15];
                        for (int i = 0; i < sizeof(indicate_data); ++i)
                        {
                            indicate_data[i] = i%0xff;
                        }
                        //the size of indicate_data[] need less than MTU size
                        esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, gl_profile_tab[PROFILE_APP_ID].char_handle,
                                                sizeof(indicate_data), indicate_data, true);
                    }
                }
                else if (descr_value == 0x0000){
                    ESP_LOGI(GATTS_TAG, "Notification/Indication disable");
                }else{
                    ESP_LOGE(GATTS_TAG, "Unknown descriptor value");
                    ESP_LOG_BUFFER_HEX(GATTS_TAG, param->write.value, param->write.len);
                }

            }
        }
        example_write_event_env(gatts_if, &prepare_write_env, param);
        break;
    }
    case ESP_GATTS_EXEC_WRITE_EVT: // client 尝试执行写入操作
        ESP_LOGI(GATTS_TAG,"Execute write");
        esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
        example_exec_write_event_env(&prepare_write_env, param);
        break;
    case ESP_GATTS_MTU_EVT:
        ESP_LOGI(GATTS_TAG, "MTU exchange, MTU %d", param->mtu.mtu);
        break;
    case ESP_GATTS_UNREG_EVT:
        break;
    case ESP_GATTS_CREATE_EVT: // service创建
        ESP_LOGI(GATTS_TAG, "Service create, status %d, service_handle %d", param->create.status, param->create.service_handle);
        gl_profile_tab[PROFILE_APP_ID].service_handle = param->create.service_handle;
        gl_profile_tab[PROFILE_APP_ID].char_uuid.len = ESP_UUID_LEN_16;
        gl_profile_tab[PROFILE_APP_ID].char_uuid.uuid.uuid16 = GATTS_CHAR_UUID_TEST_A;
        
        printf("CHRC UUID: %d\n", gl_profile_tab[PROFILE_APP_ID].char_uuid.uuid.uuid16);

        esp_ble_gatts_start_service(gl_profile_tab[PROFILE_APP_ID].service_handle);
        chrc_property = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
        esp_err_t add_char_ret = esp_ble_gatts_add_char(gl_profile_tab[PROFILE_APP_ID].service_handle, &gl_profile_tab[PROFILE_APP_ID].char_uuid,
                                                        ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                                        chrc_property,
                                                        &gatts_demo_char1_val, NULL);
        if (add_char_ret){
            ESP_LOGE(GATTS_TAG, "add char failed, error code =%x",add_char_ret);
        }
        break;
    case ESP_GATTS_ADD_INCL_SRVC_EVT:
        break;
    case ESP_GATTS_ADD_CHAR_EVT: { // 添加 Characteristic
        uint16_t length = 0;
        const uint8_t *prf_char;

        ESP_LOGI(GATTS_TAG, "Characteristic add, status %d, attr_handle %d, service_handle %d",
                param->add_char.status, param->add_char.attr_handle, param->add_char.service_handle);
        gl_profile_tab[PROFILE_APP_ID].char_handle = param->add_char.attr_handle;
        gl_profile_tab[PROFILE_APP_ID].descr_uuid.len = ESP_UUID_LEN_16;
        gl_profile_tab[PROFILE_APP_ID].descr_uuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
        esp_err_t get_attr_ret = esp_ble_gatts_get_attr_value(param->add_char.attr_handle,  &length, &prf_char);
        if (get_attr_ret == ESP_FAIL){
            ESP_LOGE(GATTS_TAG, "ILLEGAL HANDLE");
        }

        ESP_LOGI(GATTS_TAG, "the gatts demo char length = %x", length);
        for(int i = 0; i < length; i++){
            ESP_LOGI(GATTS_TAG, "prf_char[%x] =%x",i,prf_char[i]);
        }
        esp_err_t add_descr_ret = esp_ble_gatts_add_char_descr(gl_profile_tab[PROFILE_APP_ID].service_handle, &gl_profile_tab[PROFILE_APP_ID].descr_uuid,
                                                                ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, NULL, NULL);
        if (add_descr_ret){
            ESP_LOGE(GATTS_TAG, "add char descr failed, error code =%x", add_descr_ret);
        }
        break;
    }
    case ESP_GATTS_ADD_CHAR_DESCR_EVT: // 添加 Descriptor
        gl_profile_tab[PROFILE_APP_ID].descr_handle = param->add_char_descr.attr_handle;
        ESP_LOGI(GATTS_TAG, "Descriptor add, status %d, attr_handle %d, service_handle %d",
                 param->add_char_descr.status, param->add_char_descr.attr_handle, param->add_char_descr.service_handle);
        break;
    case ESP_GATTS_DELETE_EVT:
        break;
    case ESP_GATTS_START_EVT: // 启动 Service
        ESP_LOGI(GATTS_TAG, "Service start, status %d, service_handle %d",
                 param->start.status, param->start.service_handle);
        break;
    case ESP_GATTS_STOP_EVT:
        break;
    case ESP_GATTS_CONNECT_EVT: { // 连接操作
        esp_ble_conn_update_params_t conn_params = {0};
        memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
        /* For the IOS system, please reference the apple official documents about the ble connection parameters restrictions. */
        conn_params.latency = 0;
        conn_params.max_int = 0x20;    // max_int = 0x20*1.25ms = 40ms
        conn_params.min_int = 0x10;    // min_int = 0x10*1.25ms = 20ms
        conn_params.timeout = 400;    // timeout = 400*10ms = 4000ms
        ESP_LOGI(GATTS_TAG, "Connected, conn_id %u, remote "ESP_BD_ADDR_STR"",
                 param->connect.conn_id, ESP_BD_ADDR_HEX(param->connect.remote_bda));
        gl_profile_tab[PROFILE_APP_ID].conn_id = param->connect.conn_id;
        //start sent the update connection parameters to the peer device.
        esp_ble_gap_update_conn_params(&conn_params);
        break;
    }
    case ESP_GATTS_DISCONNECT_EVT: // 断
        ESP_LOGI(GATTS_TAG, "Disconnected, remote "ESP_BD_ADDR_STR", reason 0x%02x",
                 ESP_BD_ADDR_HEX(param->disconnect.remote_bda), param->disconnect.reason);
        esp_ble_gap_start_advertising(&adv_params);
        break;
    case ESP_GATTS_CONF_EVT:
        ESP_LOGI(GATTS_TAG, "Confirm receive, status %d, attr_handle %d", param->conf.status, param->conf.handle);
        if (param->conf.status != ESP_GATT_OK){
            ESP_LOG_BUFFER_HEX(GATTS_TAG, param->conf.value, param->conf.len);
        }
        break;
    case ESP_GATTS_OPEN_EVT:
    case ESP_GATTS_CANCEL_OPEN_EVT:
    case ESP_GATTS_CLOSE_EVT:
    case ESP_GATTS_LISTEN_EVT:
    case ESP_GATTS_CONGEST_EVT:
    default:
        break;
    }
}

// GATTS服务 事件处理器
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    /* If event is register event, store the gatts_if for each profile */
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            gl_profile_tab[param->reg.app_id].gatts_if = gatts_if;
        } else {
            ESP_LOGI(GATTS_TAG, "Reg app failed, app_id %04x, status %d",
                    param->reg.app_id,
                    param->reg.status);
            return;
        }
    }

    /* If the gatts_if equal to profile A, call profile A cb handler,
     * so here call each profile's callback */
    do {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++) {
            if (gatts_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
                    gatts_if == gl_profile_tab[idx].gatts_if) {
                if (gl_profile_tab[idx].gatts_cb) {
                    gl_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);
}

void app_main(void)
{
    esp_err_t ret;

    // Initialize NVS.
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s initialize controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s init bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }
    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s enable bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }
    // Note: Avoid performing time-consuming operations within callback functions.
    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret){
        ESP_LOGE(GATTS_TAG, "gatts register error, error code = %x", ret);
        return;
    }
    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret){
        ESP_LOGE(GATTS_TAG, "gap register error, error code = %x", ret);
        return;
    }
    ret = esp_ble_gatts_app_register(PROFILE_APP_ID);
    if (ret){
        ESP_LOGE(GATTS_TAG, "gatts app register error, error code = %x", ret);
        return;
    }
    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
    if (local_mtu_ret){
        ESP_LOGE(GATTS_TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
    }

    hw_uart_init();
    ble_queue_init();
    uart_queue_init();

    xTaskCreate(task_uart_send, "", 1000, NULL, 0, NULL);
    xTaskCreate(task_uart_recv, "", 1000, NULL, 0, NULL);

    return;
}
