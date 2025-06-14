/**
 * hw_uart.h
 * @author n1ghts4kura
 * @date 25/6/13 - ___
 * 
 * 硬件UART 操控
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
#include "my_queue.h"

// UART设置
#define UART_PORT_NUM UART_NUM_2 // UART 端口号
#define UART_TX_PORT  GPIO_NUM_4 // UART TX端口
#define UART_RX_PORT  GPIO_NUM_5 // UART RX端口
#define UART_BUFFER_SIZE 2048    // UART缓冲区大小

// 硬件UART 操控句柄
static QueueHandle_t hw_uart_queue;
// UART 配置
uart_config_t uart_config = {
    // 波特率
    .baud_rate = 115200,
    // 数据位
    .data_bits = UART_DATA_8_BITS,
    // 校验
    .parity    = UART_PARITY_DISABLE,
    // 停止位
    .stop_bits = UART_STOP_BITS_1,
    // 流控制
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    // TODO: 不知道
    .source_clk = UART_SCLK_DEFAULT,
};

// 初始化硬件UART
void hw_uart_init() {
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config)); // 注入参数
    uart_set_pin(UART_PORT_NUM, GPIO_NUM_4, GPIO_NUM_5, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE); // 设置UART借口
    ESP_ERROR_CHECK(
        uart_driver_install( // UART 驱动安装
            UART_PORT_NUM,
            UART_BUFFER_SIZE,
            UART_BUFFER_SIZE,
            10,
            &hw_uart_queue,
            0
        )
    );
}

/**
 * UART写入函数
 * 
 * @param string 字符串
 * @return 实际写入的数据大小 (ps: 计量单位不知道，自己琢磨一下)
 */
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
        ESP_LOGI("UART", "Successfully sent %d bytes", sent);
    }
    return sent;
}

/**
 * UART读取函数
 * 
 * @param buffer 获取的数据缓冲区
 * @return 是否获取到了数据
 */
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
            ESP_LOGI("UART", "Received clean text: [%s]", buffer);
            return true;
        }
    }
    return false;
}

#endif