#include "ws_client.h"

#include <string.h>
#include <stdio.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_websocket_client.h"
#include "conn_mgr.h"

static const char *TAG = "WS";

static esp_websocket_client_handle_t s_client;
static ws_on_text_cb_t s_on_text;
static bool s_connected;

static char s_device_id[32];
static char s_fw[24];

typedef struct
{
    char buf[384];
} ws_msg_t;

static QueueHandle_t s_txq;

static void send_hello(void)
{
    char msg[256];
    int n = snprintf(msg, sizeof(msg),
                     "{\"type\":\"hello\",\"device_id\":\"%s\",\"fw\":\"%s\",\"caps\":[\"log\",\"cmd\",\"ota\"],\"ts\":%lld}",
                     s_device_id, s_fw, (long long)(esp_timer_get_time() / 1000));
    if (n > 0)
        ws_client_send_text(msg);
}

static void hb_task(void *arg)
{
    (void)arg;
    while (1)
    {
        if (s_connected)
        {
            // heap/rssi можно добавить позже (через esp_wifi_sta_get_ap_info)
            char msg[256];
            int n = snprintf(msg, sizeof(msg),
                             "{\"type\":\"hb\",\"uptime_ms\":%lld,\"ts\":%lld}",
                             (long long)(esp_timer_get_time() / 1000),
                             (long long)(esp_timer_get_time() / 1000));
            if (n > 0)
                ws_client_send_text(msg);
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

static void tx_task(void *arg)
{
    (void)arg;
    ws_msg_t m;
    while (1)
    {
        if (xQueueReceive(s_txq, &m, portMAX_DELAY) == pdTRUE)
        {
            if (s_client && s_connected)
            {
                int r = esp_websocket_client_send_text(s_client, m.buf, (int)strlen(m.buf), pdMS_TO_TICKS(1000));
                if (r < 0)
                {
                    ESP_LOGW(TAG, "send failed");
                }
            }
        }
    }
}

static void ws_event(void *args, esp_event_base_t base, int32_t id, void *data)
{
    (void)args;
    (void)base;
    esp_websocket_event_data_t *ev = (esp_websocket_event_data_t *)data;

    switch (id)
    {
    case WEBSOCKET_EVENT_CONNECTED:
        s_connected = true;
        ESP_LOGI(TAG, "connected");
        send_hello();
        break;

    case WEBSOCKET_EVENT_DISCONNECTED:
        s_connected = false;
        ESP_LOGW(TAG, "disconnected");
        break;

    case WEBSOCKET_EVENT_DATA:
        if (ev && ev->op_code == 0x1 && ev->data_ptr && ev->data_len > 0)
        {
            if (s_on_text)
                s_on_text((const char *)ev->data_ptr, ev->data_len);
        }
        break;

    case WEBSOCKET_EVENT_ERROR:
        s_connected = false;
        ESP_LOGE(TAG, "error");
        break;

    default:
        break;
    }
}

static void reconnect_task(void *arg)
{
    (void)arg;
    int backoff_ms = 500;
    while (1)
    {
        if (!is_wifi_connected())
        {
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        if (s_client && !s_connected)
        {
            if (!esp_websocket_client_is_connected(s_client))
            {
                ESP_LOGW(TAG, "reconnect...");
                esp_websocket_client_stop(s_client);
                esp_websocket_client_start(s_client);
            }
            vTaskDelay(pdMS_TO_TICKS(backoff_ms));
            if (backoff_ms < 8000)
                backoff_ms *= 2;
        }
        else
        {
            backoff_ms = 500;
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }
}

esp_err_t ws_client_start(const ws_client_cfg_t *cfg, ws_on_text_cb_t on_text)
{
    if (!cfg || !cfg->uri || !cfg->device_id || !cfg->fw_version)
        return ESP_ERR_INVALID_ARG;

    strncpy(s_device_id, cfg->device_id, sizeof(s_device_id) - 1);
    strncpy(s_fw, cfg->fw_version, sizeof(s_fw) - 1);
    s_on_text = on_text;

    s_txq = xQueueCreate(16, sizeof(ws_msg_t));
    if (!s_txq)
        return ESP_ERR_NO_MEM;

    esp_websocket_client_config_t ws_cfg = {
        .uri = cfg->uri,
        // Для wss с нормальным сертификатом позже добавишь:
        // .cert_pem = server_cert_pem,
        .reconnect_timeout_ms = 0, // мы сами делаем reconnect
        .network_timeout_ms = 8000,
    };

    s_client = esp_websocket_client_init(&ws_cfg);
    if (!s_client)
        return ESP_FAIL;

    esp_websocket_register_events(s_client, WEBSOCKET_EVENT_ANY, ws_event, NULL);

    esp_err_t err = esp_websocket_client_start(s_client);
    if (err != ESP_OK)
        return err;

    xTaskCreate(tx_task, "ws_tx", 4096, NULL, 8, NULL);
    xTaskCreate(hb_task, "ws_hb", 4096, NULL, 5, NULL);
    xTaskCreate(reconnect_task, "ws_reconn", 4096, NULL, 6, NULL);

    return ESP_OK;
}

bool ws_client_is_connected(void)
{
    return s_connected;
}

esp_err_t ws_client_send_text(const char *txt)
{
    if (!txt)
        return ESP_ERR_INVALID_ARG;
    if (!s_txq)
        return ESP_ERR_INVALID_STATE;

    ws_msg_t m = {0};
    strncpy(m.buf, txt, sizeof(m.buf) - 1);

    if (xQueueSend(s_txq, &m, 0) != pdTRUE)
    {
        // очередь переполнена — дропаем (prod-friendly, чтобы не тормозить)
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}