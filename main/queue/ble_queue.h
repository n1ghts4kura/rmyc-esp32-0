#ifndef BLE_QUEUE_H
#define BLE_QUEUE_H

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

// static const char* BLE_QUEUE_TAG = "ble_queue";  // 改为 const
#define BLE_QUEUE_TAG "ble_queue"

typedef struct ble_queue_node_t {
    uint8_t value[QUEUE_MSG_LENGTH];  // 固定大小数组
    struct ble_queue_node_t *next;
} ble_queue_node_t;

typedef struct ble_queue_t {
    ble_queue_node_t *front;
    ble_queue_node_t *rear;
    SemaphoreHandle_t mutex;  // 添加互斥锁
} ble_queue_t;

// 初始化队列（线程安全）
void ble_queue_init(ble_queue_t *q) {
    q->front = NULL;
    q->rear = NULL;
    q->mutex = xSemaphoreCreateMutex();
    if (q->mutex == NULL) {
        ESP_LOGE(BLE_QUEUE_TAG, "Mutex creation failed!");
    }
}

// 销毁队列（释放所有资源）
void ble_queue_destroy(ble_queue_t *q) {
    xSemaphoreTake(q->mutex, portMAX_DELAY);
    
    while (q->front != NULL) {
        ble_queue_node_t *temp = q->front;
        q->front = q->front->next;
        free(temp);
    }
    q->rear = NULL;
    
    xSemaphoreGive(q->mutex);
    vSemaphoreDelete(q->mutex);
}

bool ble_queue_empty(ble_queue_t *q) {
    xSemaphoreTake(q->mutex, portMAX_DELAY);
    bool empty = (q->front == NULL);
    xSemaphoreGive(q->mutex);
    return empty;
}

bool ble_queue_append(ble_queue_t *q, uint8_t val[QUEUE_MSG_LENGTH]) {
    xSemaphoreTake(q->mutex, portMAX_DELAY);
    
    ble_queue_node_t *new_node = (ble_queue_node_t *)malloc(sizeof(ble_queue_node_t));
    if (!new_node) {
        ESP_LOGE(BLE_QUEUE_TAG, "Memory allocation failed");
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

bool ble_queue_pop(ble_queue_t *q, uint8_t val[QUEUE_MSG_LENGTH]) {
    xSemaphoreTake(q->mutex, portMAX_DELAY);
    
    if (!q->front) {
        xSemaphoreGive(q->mutex);
        return false;
    }

    memcpy(val, q->front->value, QUEUE_MSG_LENGTH);
    ble_queue_node_t *temp = q->front;
    q->front = q->front->next;
    
    if (!q->front) {
        q->rear = NULL;
    }
    
    free(temp);  // 正确释放节点
    xSemaphoreGive(q->mutex);
    return true;
}

#endif // BLE_QUEUE_H