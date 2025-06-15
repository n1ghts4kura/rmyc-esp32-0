/**
 * hw_uart.h
 * 
 */

#ifndef HW_UART_H
#define HW_UART_H

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"

#include "constants.h"

#define HW_UART_TAG "hw_uart module"

#define UART_PORT_NUM UART_NUM_2
#define UART_TX_PORT  GPIO_NUM_4
#define UART_RX_PORT  GPIO_NUM_5
#define UART_RTS_PORT UART_PIN_NO_CHANGE
#define UART_CTS_PORT UART_PIN_NO_CHANGE
#define UART_BUFFER_SIZE 2048
QueueHandle_t hw_uart_handle;
uart_config_t uart_config = {
    .baud_rate = 115200,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .source_clk = UART_SCLK_DEFAULT,
};

void hw_uart_init() {
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
    uart_set_pin(UART_PORT_NUM, UART_TX_PORT, UART_RX_PORT, UART_RTS_PORT, UART_CTS_PORT);
    ESP_ERROR_CHECK(uart_driver_install(
        UART_PORT_NUM,
        UART_BUFFER_SIZE, UART_BUFFER_SIZE,
        15,
        &hw_uart_handle,
        0
    ));
}

int hw_uart_write(char *data) {
    if (!data) {
        ESP_LOGI(HW_UART_TAG, "Uart wrote empty string, now quitting...");
        return 0;
    }

    data[MSG_LEN - 1] = '\0';
    const int sent = uart_write_bytes(UART_PORT_NUM, data, strlen(data));
    ESP_LOGI(HW_UART_TAG, "Uart wrote [%s], ( %d / %d )", data, sent, MSG_LEN);

    return sent;
}

bool hw_uart_read(char *data) {
    size_t len = 0;
    esp_err_t rsp = uart_get_buffered_data_len(UART_PORT_NUM, &len);

    if (rsp != ESP_OK) {
        ESP_LOGE(HW_UART_TAG, "failed to get buffered data length cuz: %s", esp_err_to_name(rsp));
        return false;
    }

    if (len <= 0) return false;

    if (len >= (MSG_LEN - 1)) {
        ESP_LOGW(HW_UART_TAG, "Uart receive too long: for ( %d / %d )", len, MSG_LEN - 1);
        len = MSG_LEN - 1;
    }

    int result = uart_read_bytes(UART_PORT_NUM, (uint8_t *)data, len, pdMS_TO_TICKS(20));

    if (result > 0) {
        data[result] = '\0';

        char* end = data + result - 1;
        while (end >= data && (*end == '\n' || *end == '\r' || *end == ' ')) {
            *end = '\0';
            end--;
        }

        if (strlen(data) > 0) {
            ESP_LOGI(HW_UART_TAG, "Uart receive clean text: [%s]", data);
            return true;
        }
    }

    return false;
}

#endif