#include "ota.h"
#include "esp_https_ota.h"
#include "esp_log.h"
#include "esp_http_client.h"
#include "cJSON.h"
#include "mbedtls/sha256.h"
#include <string.h>

static const char *TAG = "OTA";

static esp_err_t http_get_json(const char *url, char *buf, size_t buflen)
{
    esp_http_client_config_t cfg = {
        .url = url,
        .timeout_ms = 5000,
    };
    esp_http_client_handle_t c = esp_http_client_init(&cfg);
    if (!c)
        return ESP_FAIL;

    esp_err_t err = esp_http_client_open(c, 0);
    if (err != ESP_OK)
    {
        esp_http_client_cleanup(c);
        return err;
    }

    int total = esp_http_client_fetch_headers(c);
    (void)total;

    int r = esp_http_client_read(c, buf, buflen - 1);
    if (r < 0)
    {
        esp_http_client_close(c);
        esp_http_client_cleanup(c);
        return ESP_FAIL;
    }

    buf[r] = 0;

    esp_http_client_close(c);
    esp_http_client_cleanup(c);
    return ESP_OK;
}

static bool version_is_newer(const char *cur, const char *next)
{
    // простой semver compare: a.b.c
    int ca = 0, cb = 0, cc = 0, na = 0, nb = 0, nc = 0;
    sscanf(cur, "%d.%d.%d", &ca, &cb, &cc);
    sscanf(next, "%d.%d.%d", &na, &nb, &nc);
    if (na != ca)
        return na > ca;
    if (nb != cb)
        return nb > cb;
    return nc > cc;
}

esp_err_t ota_check_and_update(const char *manifest_url, const char *current_version)
{
    char json[2048];
    esp_err_t err = http_get_json(manifest_url, json, sizeof(json));
    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "manifest get failed: %s", esp_err_to_name(err));
        return err;
    }

    cJSON *root = cJSON_Parse(json);
    if (!root)
        return ESP_FAIL;

    const cJSON *v = cJSON_GetObjectItem(root, "version");
    const cJSON *u = cJSON_GetObjectItem(root, "url");
    const cJSON *s = cJSON_GetObjectItem(root, "sha256");

    if (!cJSON_IsString(v) || !cJSON_IsString(u) || !cJSON_IsString(s))
    {
        cJSON_Delete(root);
        return ESP_FAIL;
    }

    if (!version_is_newer(current_version, v->valuestring))
    {
        ESP_LOGI(TAG, "no update (cur=%s, srv=%s)", current_version, v->valuestring);
        cJSON_Delete(root);
        return ESP_OK;
    }

    ESP_LOGW(TAG, "update available: %s -> %s", current_version, v->valuestring);

    esp_http_client_config_t http_cfg = {
        .url = u->valuestring,
        .timeout_ms = 15000,
    };

    esp_https_ota_config_t ota_cfg = {
        .http_config = &http_cfg,
    };

    err = esp_https_ota(&ota_cfg);
    if (err == ESP_OK)
    {
        ESP_LOGW(TAG, "OTA OK, rebooting...");
        cJSON_Delete(root);
        esp_restart();
        return ESP_OK;
    }
    else
    {
        ESP_LOGE(TAG, "OTA failed: %s", esp_err_to_name(err));
        cJSON_Delete(root);
        return err;
    }
}
