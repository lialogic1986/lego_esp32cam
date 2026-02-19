// rfid_task.c — polling RC522 UID без прерываний
// - читает UID (4/7 bytes)
// - debouce/same-UID suppression
// - отправляет событие в очередь (опционально)
//
// Требует: rc522_init(), rc522_get_uid(uid,&len)

#include "rfid_task.h"
#include "rc522.h"

#include <string.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "RFID";

// Настройки опроса (под поезд обычно 5–10мс)
#ifndef RFID_POLL_MS
#define RFID_POLL_MS 10
#endif

// Если тот же UID держится в поле — не спамим чаще этого интервала
#ifndef RFID_SAME_UID_MIN_PERIOD_MS
#define RFID_SAME_UID_MIN_PERIOD_MS 200
#endif

// Через сколько считать, что метка "ушла" (нет ответов)
#ifndef RFID_TAG_LOST_MS
#define RFID_TAG_LOST_MS 350
#endif

static QueueHandle_t s_evt_q = NULL;

static void uid_to_str(const uint8_t *uid, uint8_t len, char *out, size_t out_sz)
{
    size_t p = 0;
    for (uint8_t i = 0; i < len; i++)
    {
        int n = snprintf(out + p, out_sz - p, (i ? ":%02X" : "%02X"), uid[i]);
        if (n < 0)
            break;
        p += (size_t)n;
        if (p >= out_sz)
            break;
    }
}

void rfid_task_init(void)
{
    ESP_LOGI(TAG, "Init RC522...");
    if (!rc522_init())
    {
        ESP_LOGE(TAG, "RC522 init failed");
        // Не перезагружаемся — пусть система живёт, а ты увидишь ошибку в логах
        while (1)
            vTaskDelay(pdMS_TO_TICKS(1000));
    }
    ESP_LOGI(TAG, "RC522 init OK");
}

static void rfid_task(void *arg)
{
    (void)arg;

    uint8_t last_uid[10] = {0};
    uint8_t last_len = 0;
    bool have_last = false;

    int64_t last_seen_ms = 0;   // когда в последний раз успешно читали метку
    int64_t last_report_ms = 0; // когда в последний раз репортили этот UID

    while (1)
    {
        uint8_t uid[10] = {0};
        uint8_t len = 0;

        bool got = rc522_get_uid(uid, &len);
        int64_t now_ms = esp_timer_get_time() / 1000;

        if (got)
        {
            last_seen_ms = now_ms;

            bool same = have_last && (len == last_len) && (memcmp(uid, last_uid, len) == 0);

            // Если новый UID — репортим сразу
            // Если тот же — репортим не чаще RFID_SAME_UID_MIN_PERIOD_MS
            if (!same || (now_ms - last_report_ms) >= RFID_SAME_UID_MIN_PERIOD_MS)
            {
                char s[64] = {0};
                uid_to_str(uid, len, s, sizeof(s));
                ESP_LOGI(TAG, "UID=%s (len=%u)", s, (unsigned)len);

                // Отправка события в очередь (если подключена)
                if (s_evt_q)
                {
                    rfid_evt_t evt = {
                        .ts_ms = now_ms,
                        .uid_len = len,
                    };
                    memcpy(evt.uid, uid, len);

                    // Не блокируемся: если очередь полна — дропаем событие
                    (void)xQueueSend(s_evt_q, &evt, 0);
                }

                // обновляем "последний"
                memcpy(last_uid, uid, len);
                last_len = len;
                have_last = true;
                last_report_ms = now_ms;
            }
        }
        else
        {
            // если давно не видели метку — считаем что "ушла"
            if (have_last && (now_ms - last_seen_ms) >= RFID_TAG_LOST_MS)
            {
                ESP_LOGI(TAG, "Tag left");
                have_last = false;
                last_len = 0;
                memset(last_uid, 0, sizeof(last_uid));
            }
        }

        vTaskDelay(pdMS_TO_TICKS(RFID_POLL_MS));
    }
}

void rfid_task_start(QueueHandle_t rfid_evt_q)
{
    s_evt_q = rfid_evt_q;

    // Можно pin на core0 (wifi/udp часто на core0), но RFID лёгкий — можно и туда.
    // Если хочешь меньше влияния на Wi-Fi — pin на core1.
    xTaskCreatePinnedToCore(rfid_task, "rfid_task", 12000, NULL, 5, NULL, 0);
}
