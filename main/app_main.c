#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include <sys/time.h>
#include "esp_log.h"
#include "nvs_flash.h"

#include "app_shared.h"
#include "conn_mgr.h"
#include "camera_task.h"
#include "udp_tx.h"
#include "rfid_task.h"
#include "driver/gpio.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_ota_ops.h"
#include "ws_client.h"
#include "fota.h"
#include "remote_console.h"
#include "log_stream.h"
#include "prov_ble.h"

static const char *TAG = "APP";

QueueHandle_t g_frame_q = NULL;
static bool provisioning_ok = false;
static prov_session_cfg_t sConfig = {
    .ssid = WIFI_SSID,
    .pass = WIFI_PASS,
    .host_ip = DEST_IP_STR,
    .port_video = VIDEO_PORT,
    .port_tele = WS_PORT,
    .port_ws = WS_PORT,
};

static void set_time_utc_ms(int64_t unix_ms)
{
    struct timeval tv;
    tv.tv_sec = (time_t)(unix_ms / 1000);
    tv.tv_usec = (suseconds_t)((unix_ms % 1000) * 1000);
    settimeofday(&tv, NULL);
}

static esp_err_t my_apply_cb(const prov_session_cfg_t *cfg)
{
    memcpy(&sConfig, cfg, sizeof(prov_session_cfg_t));

    ESP_LOGI(TAG, "CFG: ssid=%s host=%s video=%u tele=%u ws=%u time(ms)=%" PRId64,
             cfg->ssid, cfg->host_ip, cfg->port_video, cfg->port_tele, cfg->port_ws, cfg->unix_ms_utc);

    set_time_utc_ms(cfg->unix_ms_utc);

    provisioning_ok = true;

    return ESP_OK;
}

static void logs_tune(void)
{
    esp_log_level_set("*", ESP_LOG_INFO);     // твои INFO видны
    esp_log_level_set("wifi", ESP_LOG_ERROR); // wifi только ERROR
    esp_log_level_set("phy", ESP_LOG_ERROR);
}

static void on_ws_text(const char *txt, int len)
{
    remote_console_on_ws_text(txt, len);
}

static void flash_init(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }
    else
    {
        ESP_ERROR_CHECK(err);
    }
}

static void ota_mark_valid_if_needed(void)
{
    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_ota_img_states_t st;
    if (esp_ota_get_state_partition(running, &st) == ESP_OK)
    {
        if (st == ESP_OTA_IMG_PENDING_VERIFY)
        {
            // мы успешно запустились -> фиксируем, чтобы не было rollback
            esp_err_t e = esp_ota_mark_app_valid_cancel_rollback();
            ESP_LOGI(TAG, "OTA pending verify -> mark valid: %s", esp_err_to_name(e));
        }
    }
}

void app_main(void)
{
    logs_tune();
    ESP_LOGI(TAG, "lego_cam_udp starting...");

    flash_init();

    prov_ble_cfg_t c = {
        .adv_name_prefix = "Train-",
        .max_cfg_len = 2048,
        .apply_cb = my_apply_cb,
    };

    ESP_ERROR_CHECK(prov_ble_start(&c));

    while (!provisioning_ok)
    {
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    conn_task_init(sConfig.ssid, sConfig.pass);
    camera_task_init();
    udp_tx_task_init(sConfig.host_ip, sConfig.port_video);
    // rfid_task_init();

    // Очередь кадров: 3 элемента — достаточно
    g_frame_q = xQueueCreate(3, sizeof(frame_item_t));
    if (!g_frame_q)
    {
        ESP_LOGE(TAG, "Failed to create frame queue");
        return;
    }
    QueueHandle_t rfid_evt_q = xQueueCreate(4, sizeof(rfid_evt_t));
    if (!rfid_evt_q)
    {
        ESP_LOGE("APP", "Failed to create rfid_evt_q");
        // можно reboot/abort, но лучше просто вернуть
        return;
    }

    conn_mgr_start();
    camera_task_start(g_frame_q);
    udp_tx_start(g_frame_q);
    //  rfid_task_start(rfid_evt_q);

    ESP_LOGI(TAG, "Tasks started");
    while (!is_wifi_connected())
    {
        vTaskDelay(pdMS_TO_TICKS(3000));
    };

    ota_mark_valid_if_needed();

    ESP_ERROR_CHECK(ws_client_start(sConfig.host_ip, sConfig.port_ws));
    ESP_LOGI(TAG, "ws client started");

    log_stream_start();
    ESP_LOGI(TAG, "log stream started");

    // app_main может завершиться — задачи продолжат жить
}
