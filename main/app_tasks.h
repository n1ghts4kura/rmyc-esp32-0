/**
 * app_tasks.h
 * 
 * @author n1ghts4kura
 * @date 25/6/14
 * 
 * 定义了APP的两个重要任务
 */

#ifndef APP_TASK_H
#define APP_TASK_H

#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "esp_log.h"
#include "sdkconfig.h"

#include "constants.h"
#include "my_queue.h"
#include "hw_uart.h"

/**
 * 将缓存于*queue_pi2esp*队列的数据 发送到下位机(robomaster ep)
 */
void task1(void* p) {
    my_queue_t *queue_pi2esp = (my_queue_t *)p;

    uint8_t data[QUEUE_MSG_LENGTH] = {0};

    while (true) {
        if (queue_pop(queue_pi2esp, data)) {
            ESP_LOGI("task1", "queue_pi2esp data: [%s]", (const char *)data);
            // 成功获取数据 发送
            hw_uart_write((const char *)data);
            memset(data, 0, QUEUE_MSG_LENGTH);
            vTaskDelay(pdMS_TO_TICKS(80));
        }
        else {
            // 否则
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }
}

/**
 * 将下位机(robomaster ep)发送的数据 缓存到*queue_bot2esp*
 */
void task2(void *p) {
    my_queue_t *queue_bot2esp = (my_queue_t *)p;
    uint8_t data[QUEUE_MSG_LENGTH] = {0};

    while (true) {
        if (hw_uart_read((char *)data)) {
            ESP_LOGI("task2", "received from hw_uart, data: [%s]", (const char *)data);
            // uart成功获取信息 缓存
            queue_append(queue_bot2esp, data);
            memset(data, 0, QUEUE_MSG_LENGTH);
            vTaskDelay(pdMS_TO_TICKS(80));
        }
        else {
            // 否则
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }
}

#endif