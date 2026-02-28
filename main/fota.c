#include "fota.h"
#include "ws_client.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "esp_log.h"
#include "esp_http_client.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "mbedtls/sha256.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "camera_task.h"
#include "udp_tx.h"

static const char *TAG = "FOTA";

static bool s_busy = false;

typedef struct
{
    char cmd_id[24];
    char url[256];
    char sha256_hex[65];
    int expected_size;
} fota_job_t;

static void ws_sendf(const char *fmt, ...)
{
    char buf[512];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    (void)ws_client_send_text(buf);
}

static void cmd_ok(const char *id, const char *result_obj_or_null)
{
    char msg[512];
    if (result_obj_or_null)
    {
        snprintf(msg, sizeof(msg),
                 "{\"type\":\"cmd_result\",\"id\":\"%s\",\"ok\":true,\"result\":%s}", id, result_obj_or_null);
    }
    else
    {
        snprintf(msg, sizeof(msg),
                 "{\"type\":\"cmd_result\",\"id\":\"%s\",\"ok\":true}", id);
    }
    (void)ws_client_send_text(msg);
}

static void cmd_err(const char *id, const char *err)
{
    char msg[256];
    snprintf(msg, sizeof(msg),
             "{\"type\":\"cmd_result\",\"id\":\"%s\",\"ok\":false,\"error\":\"%s\"}",
             id, err ? err : "error");
    (void)ws_client_send_text(msg);
}

static void sha256_hex(const unsigned char *in32, char out65[65])
{
    static const char *hex = "0123456789abcdef";
    for (int i = 0; i < 32; i++)
    {
        out65[i * 2 + 0] = hex[(in32[i] >> 4) & 0xF];
        out65[i * 2 + 1] = hex[(in32[i] >> 0) & 0xF];
    }
    out65[64] = 0;
}

static void fota_task(void *arg)
{
    fota_job_t job = *(fota_job_t *)arg;
    free(arg);

    ESP_LOGI(TAG, "FOTA start url=%s size=%d", job.url, job.expected_size);
    ws_sendf("{\"type\":\"ota_status\",\"stage\":\"task_started\"}");

    ws_sendf("{\"type\":\"ota_status\",\"stage\":\"pre_stop\"}");
    udp_tx_stop();
    camera_task_stop();
    ws_sendf("{\"type\":\"ota_status\",\"stage\":\"stopped_stream\"}");

    ws_sendf("{\"type\":\"ota_status\",\"stage\":\"start\",\"url\":\"%s\"}", job.url);

    esp_http_client_config_t cfg = {
        .url = job.url,
        .timeout_ms = 15000,
        .buffer_size = 8 * 1024,
        .buffer_size_tx = 1024,
        .keep_alive_enable = true,
    };

    esp_http_client_handle_t cli = esp_http_client_init(&cfg);
    if (!cli)
    {
        cmd_err(job.cmd_id, "http_init_fail");
        s_busy = false;
        vTaskDelete(NULL);

        return;
    }

    ESP_LOGI(TAG, "S1 http_init");
    ws_sendf("{\"type\":\"ota_status\",\"stage\":\"S1_http_init\"}");

    esp_err_t err = esp_http_client_open(cli, 0);
    if (err != ESP_OK)
    {
        esp_http_client_cleanup(cli);
        cmd_err(job.cmd_id, "http_open_fail");
        s_busy = false;
        vTaskDelete(NULL);

        return;
    }

    ESP_LOGI(TAG, "S2 http_open");
    ws_sendf("{\"type\":\"ota_status\",\"stage\":\"S2_http_open\"}");

    int content_len = esp_http_client_fetch_headers(cli);
    if (content_len <= 0 && job.expected_size > 0)
    {
        content_len = job.expected_size;
    }

    ESP_LOGI(TAG, "S3 fetch_headers");
    ws_sendf("{\"type\":\"ota_status\",\"stage\":\"S3_headers\"}");

    const esp_partition_t *update_part = esp_ota_get_next_update_partition(NULL);
    if (!update_part)
    {
        esp_http_client_close(cli);
        esp_http_client_cleanup(cli);
        cmd_err(job.cmd_id, "no_ota_partition");
        s_busy = false;

        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "esp_ota_get_next_update_partition");
    ws_sendf("{\"type\":\"ota_status\",\"stage\":\"esp_ota_get_next_update_partition\"}");

    esp_ota_handle_t ota = 0;
    err = esp_ota_begin(update_part, OTA_SIZE_UNKNOWN, &ota);
    if (err != ESP_OK)
    {
        esp_http_client_close(cli);
        esp_http_client_cleanup(cli);
        cmd_err(job.cmd_id, "ota_begin_fail");
        s_busy = false;

        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "S4 ota_begin");
    ws_sendf("{\"type\":\"ota_status\",\"stage\":\"S4_ota_begin\"}");

    mbedtls_sha256_context sha;
    mbedtls_sha256_init(&sha);
    mbedtls_sha256_starts(&sha, 0);

    uint8_t *buf = (uint8_t *)malloc(16 * 1024);
    if (!buf)
    {
        esp_ota_abort(ota);
        esp_http_client_close(cli);
        esp_http_client_cleanup(cli);
        cmd_err(job.cmd_id, "no_mem");
        s_busy = false;

        vTaskDelete(NULL);
        return;
    }

    int written = 0;
    int last_pct = -1;

    while (1)
    {
        int r = esp_http_client_read(cli, (char *)buf, 8 * 1024);
        if (r < 0)
        {
            err = ESP_FAIL;
            break;
        }
        if (r == 0)
        {
            // EOF
            err = ESP_OK;
            break;
        }

        mbedtls_sha256_update(&sha, buf, r);

        err = esp_ota_write(ota, buf, r);
        if (err != ESP_OK)
        {
            break;
        }
        written += r;

        if (content_len > 0)
        {
            int pct = (int)((written * 100LL) / content_len);
            if (pct != last_pct && (pct % 5 == 0 || pct == 100))
            {
                last_pct = pct;
                ws_sendf("{\"type\":\"ota_status\",\"stage\":\"download\",\"progress\":%d,\"written\":%d}", pct, written);
            }
        }
        else
        {
            // если длины нет — просто иногда пиши
            if ((written & (128 * 1024 - 1)) == 0)
            {
                ws_sendf("{\"type\":\"ota_status\",\"stage\":\"download\",\"written\":%d}", written);
            }
        }
        vTaskDelay(1); // 1 tick, чтобы дать жить системе
    }

    free(buf);
    esp_http_client_close(cli);
    esp_http_client_cleanup(cli);

    if (err != ESP_OK)
    {
        esp_ota_abort(ota);
        cmd_err(job.cmd_id, "ota_write_fail");
        s_busy = false;

        vTaskDelete(NULL);
        return;
    }

    unsigned char sum[32];
    mbedtls_sha256_finish(&sha, sum);
    mbedtls_sha256_free(&sha);

    char got_hex[65];
    sha256_hex(sum, got_hex);

    ws_sendf("{\"type\":\"ota_status\",\"stage\":\"verify\",\"sha256\":\"%s\"}", got_hex);

    if (job.sha256_hex[0] && strcasecmp(got_hex, job.sha256_hex) != 0)
    {
        esp_ota_abort(ota);
        cmd_err(job.cmd_id, "sha256_mismatch");
        s_busy = false;

        vTaskDelete(NULL);
        return;
    }

    if (job.expected_size > 0 && written != job.expected_size)
    {
        // не всегда обязательно, но полезно как защита от обрыва
        ws_sendf("{\"type\":\"ota_status\",\"stage\":\"warn\",\"msg\":\"size_mismatch\",\"written\":%d,\"expected\":%d}",
                 written, job.expected_size);
        // можно считать ошибкой, но я оставлю предупреждением
    }

    err = esp_ota_end(ota);
    if (err != ESP_OK)
    {
        cmd_err(job.cmd_id, "ota_end_fail");
        s_busy = false;

        vTaskDelete(NULL);
        return;
    }

    err = esp_ota_set_boot_partition(update_part);
    if (err != ESP_OK)
    {
        cmd_err(job.cmd_id, "set_boot_fail");
        s_busy = false;
        vTaskDelete(NULL);

        return;
    }

    ws_sendf("{\"type\":\"ota_status\",\"stage\":\"done\",\"written\":%d}", written);

    char result[128];
    snprintf(result, sizeof(result), "{\"written\":%d}", written);
    cmd_ok(job.cmd_id, result);

    // reboot (можно сделать опцией, но сейчас “просто”)
    vTaskDelay(pdMS_TO_TICKS(800));
    esp_restart();
}

esp_err_t fota_start(const char *cmd_id, const char *url, const char *sha256_hex, int expected_size)
{
    if (!cmd_id || !url)
        return ESP_ERR_INVALID_ARG;
    if (s_busy)
        return ESP_ERR_INVALID_STATE;

    s_busy = true;

    fota_job_t *job = (fota_job_t *)calloc(1, sizeof(fota_job_t));
    if (!job)
    {
        s_busy = false;
        return ESP_ERR_NO_MEM;
    }

    strncpy(job->cmd_id, cmd_id, sizeof(job->cmd_id) - 1);
    strncpy(job->url, url, sizeof(job->url) - 1);
    if (sha256_hex)
        strncpy(job->sha256_hex, sha256_hex, sizeof(job->sha256_hex) - 1);
    job->expected_size = expected_size;

    BaseType_t ok = xTaskCreate(fota_task, "fota", 16384, job, 6, NULL);
    if (ok != pdPASS)
    {
        free(job);
        s_busy = false;
        return ESP_FAIL;
    }
    return ESP_OK;
}