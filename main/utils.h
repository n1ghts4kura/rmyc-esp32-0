/**
 * utils.h
 * 
 * @author n1ghts4kura
 * @date 2025 q1->q2
 * 
 * Some useful methods.
 */

#ifndef UTILS_H
#define UTILS_H

#include <inttypes.h>

void replace_enter(uint8_t *val) {
    /**
     * Replace all '\n' with ' ' for output beauty.
     */
    do {
        char *string = strchr((const char *)val, '\n');
        if (string) {
            *string = ' ';
        } else {
            break;
        }
    } while (1); 
}

#endif