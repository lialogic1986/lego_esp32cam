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

static const char *TAG = "APP";

QueueHandle_t g_frame_q = NULL;

void app_main(void)
{
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
    // app_main может завершиться — задачи продолжат жить
}
