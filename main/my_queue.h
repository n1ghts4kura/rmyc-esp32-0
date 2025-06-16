/**
 * my_queue.h
 * 
 */

#ifndef MY_QUEUE_H
#define MY_QUEUE_H

#include <stdlib.h>
#include <inttypes.h>
#include "freertos/task.h"
#include "freertos/FreeRTOS.h"

#include "constants.h"

#define QUEUE_TAG "queue module"

typedef struct my_queue_node_t {
    uint8_t val[MSG_LEN];
    struct my_queue_node_t *next;    
} my_queue_node_t;

typedef struct my_queue_t {
    my_queue_node_t *front;
    my_queue_node_t *rear;
    // SemaphoreHandle_t lock;
} my_queue_t;

void queue_init(my_queue_t *q) {
    q->front = NULL;
    q->rear = NULL;
}

// #define LOCK_TAKE(queue) xSemaphoreTake( (queue)->lock , portMAX_DELAY )
// #define LOCK_GIVE(queue) xSemaphoreGive( (queue)->lock )

bool is_queue_empty(my_queue_t *q) {
    bool rst = q->front == NULL;
    return rst;
}

bool queue_append(my_queue_t *q, uint8_t val[MSG_LEN]) {
    my_queue_node_t *new_node = (my_queue_node_t *)malloc(sizeof(my_queue_node_t));
    new_node->next = NULL;
    memcpy(new_node->val, val, MSG_LEN);

    if (q->front == NULL) {
        q->front = q->rear = new_node;
    } else {
        q->rear->next = new_node;
        q->rear = new_node;
    }

    return true;
}

bool queue_pop(my_queue_t *q, uint8_t val[MSG_LEN]) {
    if (is_queue_empty(q)) {
        return false;
    }

    memcpy(val, q->front->val, MSG_LEN);
    my_queue_node_t *pop = q->front;
    q->front = q->front->next;
    if (!q->front) {
        q->rear = NULL;
    }
    free(pop);
    
    return true;
}

#endif