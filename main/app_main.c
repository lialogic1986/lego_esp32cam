#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_log.h"

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
#include "ota.h"
#include "remote_console.h"
#include "log_stream.h"

static const char *TAG = "APP";

QueueHandle_t g_frame_q = NULL;

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

void app_main(void)
{
    logs_tune();
    ESP_LOGI(TAG, "lego_cam_udp starting...");

    conn_task_init();
    camera_task_init();
    udp_tx_task_init();
    rfid_task_init();

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
    rfid_task_start(rfid_evt_q);

    ESP_LOGI(TAG, "Tasks started");
    while (!is_wifi_connected())
    {
        vTaskDelay(pdMS_TO_TICKS(3000));
    };

    ws_client_cfg_t cfg = {
        .uri = "ws://" DEST_IP_STR ":8000/ws/train-01", // поменяешь
        .device_id = "train-01",
        .fw_version = "0.1.0",
    };

    ESP_ERROR_CHECK(ws_client_start(&cfg, on_ws_text));
    ESP_LOGI(TAG, "ws client started");

    log_stream_start();
    ESP_LOGI(TAG, "log stream started");

    // app_main может завершиться — задачи продолжат жить
}
