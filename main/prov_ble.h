#pragma once

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct
    {
        char ssid[33];
        char pass[65];

        char host_ip[16]; // "192.168.1.50"
        uint16_t port_video;
        uint16_t port_tele;
        uint16_t port_ws;

        int64_t unix_ms_utc; // UTC epoch ms
    } prov_session_cfg_t;

    /**
     * Пользовательский callback: применить конфиг.
     * Тут обычно:
     *  - установить время (settimeofday)
     *  - запустить Wi-Fi STA
     *  - дождаться IP (или запустить дальше свои сервисы)
     *
     * ВАЖНО: если вернёшь ESP_OK -> BLE будет выключен.
     */
    typedef esp_err_t (*prov_apply_cb_t)(const prov_session_cfg_t *cfg);

    typedef struct
    {
        const char *adv_name_prefix; // "Train-"
        uint16_t max_cfg_len;        // напр. 2048
        prov_apply_cb_t apply_cb;    // обязательный
    } prov_ble_cfg_t;

    esp_err_t prov_ble_start(const prov_ble_cfg_t *cfg);
    void prov_ble_stop(void);

#ifdef __cplusplus
}
#endif