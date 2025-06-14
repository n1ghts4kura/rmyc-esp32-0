/**
 * queue.h
 * 
 * @author n1ghts4kura
 * @date 25/6/13 - ___
 * 
 * 队列的实现 使用链表
 */

#ifndef MY_QUEUE_H
#define MY_QUEUE_H

#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "esp_log.h"
#include "sdkconfig.h"

#include "constants.h"

#define QUEUE_TAG "queue module"

// 链表节点
typedef struct my_queue_node_t {
    uint8_t value[QUEUE_MSG_LENGTH];  // 固定大小数组
    struct my_queue_node_t *next;
} my_queue_node_t;

// 链表(队列) 定义
typedef struct my_queue_t {
    my_queue_node_t *front;
    my_queue_node_t *rear;
    SemaphoreHandle_t mutex; // 线程锁 防止多线程访问队列时出现错误
} my_queue_t;

/** 
 * 初始化队列
 * 
 * @param q
 */
void queue_init(my_queue_t *q) {
    q->front = NULL;
    q->rear = NULL;
    q->mutex = xSemaphoreCreateMutex();
    if (q->mutex == NULL) {
        ESP_LOGE(QUEUE_TAG, "Mutex creation failed!");
    }
}

/**
 * 删除队列
 * 
 * @param q
 */
void queue_destroy(my_queue_t *q) {
    xSemaphoreTake(q->mutex, portMAX_DELAY);
    
    while (q->front != NULL) {
        my_queue_node_t *temp = q->front;
        q->front = q->front->next;
        free(temp);
    }
    q->rear = NULL;
    
    xSemaphoreGive(q->mutex);
    vSemaphoreDelete(q->mutex);
}

/**
 * 判断队列是否为空
 * 
 * @param q
 * @return 是 / 否
 */
bool is_queue_empty(my_queue_t *q) {
    xSemaphoreTake(q->mutex, portMAX_DELAY);
    bool empty = (q->front == NULL);
    xSemaphoreGive(q->mutex);
    return empty;
}

/**
 * 向队列中添加数据 到队尾
 * 
 * @param q
 * @param val 数据 长度为 QUEUE_MSG_LENGTH
 * @return 是否成功
 */
bool queue_append(my_queue_t *q, uint8_t val[QUEUE_MSG_LENGTH]) {
    xSemaphoreTake(q->mutex, portMAX_DELAY);
    
    my_queue_node_t *new_node = (my_queue_node_t*)malloc(sizeof(my_queue_node_t));
    if (!new_node) {
        ESP_LOGE(QUEUE_TAG, "Memory allocation failed");
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

/**
 * 从队列头中获得最早的数据
 * 
 * @param q
 * @param val 数据指针
 * @return 是否成功
 */
bool queue_pop(my_queue_t *q, uint8_t val[QUEUE_MSG_LENGTH]) {
    xSemaphoreTake(q->mutex, portMAX_DELAY);
    
    if (!q->front) {
        xSemaphoreGive(q->mutex);
        return false;
    }

    memcpy(val, q->front->value, QUEUE_MSG_LENGTH);
    my_queue_node_t *temp = q->front;
    q->front = q->front->next;
    
    if (!q->front) {
        q->rear = NULL;
    }
    
    free(temp);  // 正确释放节点
    xSemaphoreGive(q->mutex);
    return true;
}

#endif // MY_QUEUE_H