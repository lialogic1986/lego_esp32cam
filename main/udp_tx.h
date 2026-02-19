#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

void udp_tx_task_init(void);
void udp_tx_start(QueueHandle_t frame_q);
