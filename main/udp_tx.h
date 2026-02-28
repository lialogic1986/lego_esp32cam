#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

void udp_tx_task_init(const char *ipaddr, uint16_t video_port);
void udp_tx_start(QueueHandle_t frame_q);
void udp_tx_stop(void);
bool udp_tx_is_running(void);
