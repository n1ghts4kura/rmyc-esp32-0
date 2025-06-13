#ifndef UART_QUEUE_H
#define UART_QUEUE_H

#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "constant.h"
#include "esp_system.h"
#include "esp_log.h"

#include "sdkconfig.h"

#include "constant.h"

// static const char* BLE_QUEUE_TAG = "uart_queue";  // 改为 const
#define UART_QUEUE_TAG "uart_queue"

typedef struct uart_queue_node_t {
    uint8_t value[QUEUE_MSG_LENGTH];  // 固定大小数组
    struct uart_queue_node_t *next;
} uart_queue_node_t;

typedef struct uart_queue_t {
    uart_queue_node_t *front;
    uart_queue_node_t *rear;
    SemaphoreHandle_t mutex;  // 添加互斥锁
} uart_queue_t;

// 初始化队列（线程安全）
void uart_queue_init(uart_queue_t *q) {
    q->front = NULL;
    q->rear = NULL;
    q->mutex = xSemaphoreCreateMutex();
    if (q->mutex == NULL) {
        ESP_LOGE(UART_QUEUE_TAG, "Mutex creation failed!");
    }
}

// 销毁队列（释放所有资源）
void uart_queue_destroy(uart_queue_t *q) {
    xSemaphoreTake(q->mutex, portMAX_DELAY);
    
    while (q->front != NULL) {
        uart_queue_node_t *temp = q->front;
        q->front = q->front->next;
        free(temp);
    }
    q->rear = NULL;
    
    xSemaphoreGive(q->mutex);
    vSemaphoreDelete(q->mutex);
}

bool uart_queue_empty(uart_queue_t *q) {
    xSemaphoreTake(q->mutex, portMAX_DELAY);
    bool empty = (q->front == NULL);
    xSemaphoreGive(q->mutex);
    return empty;
}

bool uart_queue_append(uart_queue_t *q, uint8_t val[QUEUE_MSG_LENGTH]) {
    xSemaphoreTake(q->mutex, portMAX_DELAY);
    
    uart_queue_node_t *new_node = (uart_queue_node_t*)malloc(sizeof(uart_queue_node_t));
    if (!new_node) {
        ESP_LOGE(UART_QUEUE_TAG, "Memory allocation failed");
        xSemaphoreGive(q->mutex);
        return false;
    }

    memcpy(new_node->value, val, QUEUE_MSG_LENGTH);
    new_node->next = NULL;

    if (!q->rear) {
        q->front = q->rear = new_node;
    } else {
        q->rear->next = new_node;
        q->rear = new_node;
    }
    
    xSemaphoreGive(q->mutex);
    return true;
}

bool uart_queue_pop(uart_queue_t *q, uint8_t val[QUEUE_MSG_LENGTH]) {
    xSemaphoreTake(q->mutex, portMAX_DELAY);
    
    if (!q->front) {
        xSemaphoreGive(q->mutex);
        return false;
    }

    memcpy(val, q->front->value, QUEUE_MSG_LENGTH);
    uart_queue_node_t *temp = q->front;
    q->front = q->front->next;
    
    if (!q->front) {
        q->rear = NULL;
    }
    
    free(temp);  // 正确释放节点
    xSemaphoreGive(q->mutex);
    return true;
}

#endif // BLE_QUEUE_H