#pragma once
#include "esp_err.h"

esp_err_t ping_mod_start_gateway(const char *cmd_id, int count, int interval_ms, int timeout_ms, int payload_size);