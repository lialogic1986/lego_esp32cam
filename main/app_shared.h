#pragma once

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_camera.h"

// ========= Настройки сети =========
// Впиши IP твоего Linux ПК (куда шлём UDP)
#define DEST_IP_STR "192.168.0.102"
#define VIDEO_PORT 5000

// Размер payload в UDP (безопасно для MTU)
#define UDP_PAYLOAD 1200

// ========= Протокол фрагментации =========
typedef struct __attribute__((packed))
{
    uint8_t magic0; // 'M'
    uint8_t magic1; // 'J'
    uint16_t frameId;
    uint16_t chunkId;
    uint16_t chunksTotal;
    uint16_t payloadLen;
    uint16_t reserved;
} mj_hdr_t;

// Элемент очереди кадров (важно: без копирования JPEG!)
typedef struct
{
    camera_fb_t *fb;
    uint16_t frame_id;
} frame_item_t;

// Глобальная очередь (создаётся в app_main)
extern QueueHandle_t g_frame_q;
