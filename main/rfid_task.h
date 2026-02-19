#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

// ===== Событие RFID (для очереди) =====
typedef struct
{
    int64_t ts_ms;
    uint8_t uid[10];
    uint8_t uid_len; // 4 или 7 (обычно)
} rfid_evt_t;

void rfid_task_init(void);
void rfid_task_start(QueueHandle_t rfid_evt_q);
