#pragma once
#include "esp_err.h"

esp_err_t fota_start(const char *cmd_id, const char *url, const char *sha256_hex, int expected_size);