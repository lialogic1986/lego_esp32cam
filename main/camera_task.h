#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

void camera_task_init(void);
void camera_task_start(QueueHandle_t frame_q);
