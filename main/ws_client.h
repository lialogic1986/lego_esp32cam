#pragma once
#include <stdbool.h>
#include "esp_err.h"

typedef void (*ws_on_text_cb_t)(const char *txt, int len);

esp_err_t ws_client_start(char *ipaddr, uint16_t port);
bool ws_client_is_connected(void);
esp_err_t ws_client_send_text(const char *txt);