/**
 * hw_uart.h
 * @author n1ghts4kura
 * @date 25/6/13 - ___
 * 
 * @brief 若出现 *sdkconfig.h* 中的 *CONFIG_LOG_MAXIMUM_LEVEL* 可无需在意
 */

#ifndef HW_UART_H
#define HW_UART_H

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"

#include "sdkconfig.h"

#include "constant.h"

// UART设置
#define UART_PORT_NUM UART_NUM_2
#define UART_BUFFER_SIZE 2048

static QueueHandle_t hw_uart_queue;

uart_config_t uart_config = {
    .baud_rate = 115200,
    .data_bits = UART_DATA_8_BITS,
    .parity    = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .source_clk = UART_SCLK_DEFAULT,
};

void hw_uart_init() {
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
    uart_set_pin(UART_PORT_NUM, GPIO_NUM_4, GPIO_NUM_5, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    ESP_ERROR_CHECK(
        uart_driver_install(
            UART_PORT_NUM,
            UART_BUFFER_SIZE,
            UART_BUFFER_SIZE,
            10,
            &hw_uart_queue,
            0
        )
    );
}

// 1. 改进的UART写入函数 - 添加参数验证和更好的错误处理
int hw_uart_write(const char* string) {
    if (!string) {
        ESP_LOGE("UART", "String is NULL");
        return 0;
    }
    
    const size_t len = strlen(string);
    if (len == 0) {
        ESP_LOGW("UART", "Empty string");
        return 0;
    }
    
    // 对于文本数据，确保有足够的缓冲区空间
    if (len >= QUEUE_MSG_LENGTH) {
        ESP_LOGW("UART", "String too long: %zu bytes, truncating to %d", 
                 len, QUEUE_MSG_LENGTH - 1);
    }
    
    const int sent = uart_write_bytes(UART_PORT_NUM, string, len);
    
    if (sent != len) {
        ESP_LOGE("UART", "Partial write: %d/%zu bytes", sent, len);
    } else {
        ESP_LOGD("UART", "Successfully sent %d bytes", sent);
    }
    return sent;
}

// 2. 改进的UART读取函数 - 更好的字符串处理
bool hw_uart_read(char* buffer) {
    if (!buffer) {
        ESP_LOGE("UART", "Buffer is NULL");
        return false;
    }
    
    size_t length = 0;
    esp_err_t err = uart_get_buffered_data_len(UART_PORT_NUM, &length);
    
    if (err != ESP_OK) {
        ESP_LOGE("UART", "Failed to get buffered data length: %s", esp_err_to_name(err));
        return false;
    }
    
    if (length <= 0) {
        return false;
    }
    
    // 确保不会缓冲区溢出，为字符串终止符留空间
    if (length >= (QUEUE_MSG_LENGTH - 1)) {
        ESP_LOGW("UART", "Data too long: %zu bytes, truncating to %d", 
                 length, QUEUE_MSG_LENGTH - 2);
        length = QUEUE_MSG_LENGTH - 1;
    }
    
    int result = uart_read_bytes(UART_PORT_NUM, (uint8_t*)buffer, 
                                length, pdMS_TO_TICKS(20));
    if (result > 0) {
        buffer[result] = '\0';  // 确保字符串终止
        
        // 移除可能的换行符和回车符（清理文本数据）
        char* end = buffer + result - 1;
        while (end >= buffer && (*end == '\n' || *end == '\r' || *end == ' ')) {
            *end = '\0';
            end--;
        }
        
        // 检查是否还有有效内容
        if (strlen(buffer) > 0) {
            ESP_LOGD("UART", "Received clean text: [%s]", buffer);
            return true;
        }
    }
    return false;
}

#endif