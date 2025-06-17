/**
 * utils.h
 * 
 * @author n1ghts4kura
 * @date 23/6/13 - 6/14
 * 
 * 杂项
 */

#ifndef UTILS_H
#define UTILS_H

#include "esp_gatts_api.h"
#include "esp_gatt_defs.h"

/**
 * 生成空返回 (针对read操作)
 * 
 * @param param
 * @return 操作是否成功
 */
esp_gatt_rsp_t get_empty_gatt_read_rsp(esp_ble_gatts_cb_param_t *param) {
    esp_gatt_rsp_t rsp = {
        .attr_value = {
            .len = 0,
            .handle = param->read.handle,
            .offset = 0,
            .auth_req = ESP_GATT_AUTH_REQ_NONE
        }
    };
    return rsp;
}

/**
 * 生成空返回 (针对write操作)
 * 
 * @param param
 * @return 操作是否成功k
 */
esp_gatt_rsp_t get_empty_gatt_write_rsp(esp_ble_gatts_cb_param_t *param) {
    esp_gatt_rsp_t rsp = {
        .attr_value = {
            .len = 0,
            .handle = param->write.handle,
            .offset = 0,
            .auth_req = ESP_GATT_AUTH_REQ_NONE
        }
    };
    return rsp;
}

#endif
