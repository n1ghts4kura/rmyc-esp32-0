/**
 * my_queue.h
 * 
 * @author n1ghts4kura
 * @date 2025 q1->q2
 * 
 * Queues used to temporarily store data.
 */

#ifndef MY_QUEUE_H
#define MY_QUEUE_H

#include <stdlib.h>
#include <inttypes.h>
#include "freertos/task.h"
#include "freertos/FreeRTOS.h"

#include "constants.h"

#define QUEUE_TAG "queue module"

/**
 * The node of the queue.
 */
typedef struct my_queue_node_t {
    uint8_t val[MSG_LEN];
    struct my_queue_node_t *next;    
} my_queue_node_t;

/**
 * The definition of the queue.
 */
typedef struct my_queue_t {
    my_queue_node_t *front;
    my_queue_node_t *rear;
    // SemaphoreHandle_t lock;
} my_queue_t;

/**
 * Init one queue.
 *
 * @param q queue
 */
void queue_init(my_queue_t *q) {
    q->front = NULL;
    q->rear = NULL;
}

// #define LOCK_TAKE(queue) xSemaphoreTake( (queue)->lock , portMAX_DELAY )
// #define LOCK_GIVE(queue) xSemaphoreGive( (queue)->lock )

/**
 * Check if a queue is empty.
 * 
 * @param q queue
 * @return if empty.
 */
bool is_queue_empty(my_queue_t *q) {
    bool rst = q->front == NULL;
    return rst;
}

/**
 * Append data to the rear side of a queue.
 * 
 * @param q queue
 * @param val data
 * @return if append successfully
 */
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

/**
 * Pop data from the front size of a queue.
 * 
 * @param q queue
 * @param val the target var
 * @return if pop successfully
 */
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