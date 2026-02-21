#pragma once
#include <stdbool.h>
#include "esp_err.h"

typedef void (*ws_on_text_cb_t)(const char *txt, int len);

typedef struct
{
    const char *uri;        // ws://ip:port/ws or wss://domain/ws
    const char *device_id;  // "train-01"
    const char *fw_version; // "1.0.0"
} ws_client_cfg_t;

esp_err_t ws_client_start(const ws_client_cfg_t *cfg, ws_on_text_cb_t on_text);
bool ws_client_is_connected(void);
esp_err_t ws_client_send_text(const char *txt);