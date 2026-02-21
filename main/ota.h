#pragma once
#include "esp_err.h"
esp_err_t ota_check_and_update(const char *manifest_url, const char *current_version);
