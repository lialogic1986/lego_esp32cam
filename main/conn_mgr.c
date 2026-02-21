#include "conn_mgr.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

#include "esp_netif.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_system.h" // strlcpy
#include "esp_ota_ops.h"
#include "esp_log.h"

static const char *TAG = "CONN";

#define WIFI_SSID "lialogic_home"
#define WIFI_PASS "02121986"

static bool s_wifi_inited = false;

static void ota_mark_valid_if_pending(void)
{
    esp_ota_img_states_t st;
    const esp_partition_t *running = esp_ota_get_running_partition();
    if (esp_ota_get_state_partition(running, &st) == ESP_OK &&
        st == ESP_OTA_IMG_PENDING_VERIFY)
    {
        ESP_LOGW("OTA", "Marking app valid (cancel rollback)");
        esp_ota_mark_app_valid_cancel_rollback();
    }
}

void conn_task_init(void)
{
    if (s_wifi_inited)
    {
        ESP_LOGW(TAG, "conn_task_init called twice, skipping");
        return;
    }
    s_wifi_inited = true;

    ESP_ERROR_CHECK(esp_netif_init());

    // если вдруг уже создано (в будущем), не падаем
    esp_err_t err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
    {
        ESP_ERROR_CHECK(err);
    }

    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wc = {0};
    strlcpy((char *)wc.sta.ssid, WIFI_SSID, sizeof(wc.sta.ssid));
    strlcpy((char *)wc.sta.password, WIFI_PASS, sizeof(wc.sta.password));
    wc.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wc));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi init done (ssid=%s)", WIFI_SSID);
}

static void conn_task(void *arg)
{
    (void)arg;
    static bool first = true;

    ESP_LOGI(TAG, "WiFi start; connecting to AP...");
    ESP_ERROR_CHECK(esp_wifi_connect());

    while (1)
    {
        wifi_ap_record_t ap;
        esp_err_t e = esp_wifi_sta_get_ap_info(&ap);
        if (e == ESP_OK)
        {
            ESP_LOGI(TAG, "WiFi OK, RSSI=%d", ap.rssi);
            if (first)
            {
                ota_mark_valid_if_pending();
                first = false;
            }
        }
        else
        {
            ESP_LOGW(TAG, "WiFi not connected, reconnect...");
            (void)esp_wifi_connect();
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

bool is_wifi_connected()
{
    wifi_ap_record_t ap;
    esp_err_t e = esp_wifi_sta_get_ap_info(&ap);
    return e == ESP_OK;
}

void conn_mgr_start(void)
{
    xTaskCreatePinnedToCore(conn_task, "conn_mgr", 8192, NULL, 4, NULL, 0);
}
