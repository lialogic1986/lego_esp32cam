#include "udp_tx.h"
#include "app_shared.h"

#include <string.h>
#include <sys/socket.h>
#include "lwip/inet.h"
#include "lwip/sockets.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "UDP_TX";

static QueueHandle_t s_frame_q = NULL;

static TaskHandle_t s_udp_th = NULL;
static volatile bool s_udp_run = false;

static uint32_t s_tx_frames = 0;
static uint32_t s_tx_pkts = 0;
static uint64_t s_tx_bytes = 0;
static uint32_t s_tx_err = 0;

static int64_t s_stat_t0_us = 0;

static void tx_stats_init(void)
{
    s_stat_t0_us = esp_timer_get_time();
    s_tx_frames = 0;
    s_tx_pkts = 0;
    s_tx_bytes = 0;
    s_tx_err = 0;
}

static void tx_stats_on_packet(int sent_bytes, int err)
{
    if (err)
    {
        s_tx_err++;
        return;
    }
    s_tx_pkts++;
    s_tx_bytes += (uint64_t)sent_bytes;
}

static void tx_stats_on_frame_done(void)
{
    s_tx_frames++;

    int64_t now = esp_timer_get_time();
    int64_t dt = now - s_stat_t0_us;
    if (dt >= 1000000)
    { // ~1s
        // точнее, чем просто "за секунду"
        float sec = (float)dt / 1000000.0f;
        float fps = (float)s_tx_frames / sec;
        float pps = (float)s_tx_pkts / sec;
        float kbps = (float)(s_tx_bytes * 8ULL) / 1000.0f / sec;

        ESP_LOGI("UDP_TX", "TX fps=%.1f pps=%.0f bitrate=%.0f kbps err=%lu",
                 fps, pps, kbps, (unsigned long)s_tx_err);

        // reset window
        s_stat_t0_us = now;
        s_tx_frames = 0;
        s_tx_pkts = 0;
        s_tx_bytes = 0;
        s_tx_err = 0;
    }
}

static void send_jpeg_udp(int sock, const struct sockaddr_in *to,
                          const uint8_t *jpeg, size_t len, uint16_t frame_id)
{
    uint16_t chunks_total = (len + UDP_PAYLOAD - 1) / UDP_PAYLOAD;

    for (uint16_t cid = 0; cid < chunks_total; cid++)
    {
        size_t off = (size_t)cid * UDP_PAYLOAD;
        size_t plen = len - off;
        if (plen > UDP_PAYLOAD)
            plen = UDP_PAYLOAD;

        uint8_t pkt[sizeof(mj_hdr_t) + UDP_PAYLOAD];

        mj_hdr_t hdr = {
            .magic0 = 'M',
            .magic1 = 'J',
            .frameId = htons(frame_id),
            .chunkId = htons(cid),
            .chunksTotal = htons(chunks_total),
            .payloadLen = htons((uint16_t)plen),
            .reserved = 0};

        memcpy(pkt, &hdr, sizeof(hdr));
        memcpy(pkt + sizeof(hdr), jpeg + off, plen);

        size_t sent = sendto(sock, pkt, sizeof(hdr) + plen, 0,
                             (const struct sockaddr *)to, sizeof(*to));

        if (sent < 0)
        {
            tx_stats_on_packet(0, 1);
            return;
        }
        else
        {
            tx_stats_on_packet(sent, 0);
        }
    }
    tx_stats_on_frame_done();
}

int sock = 0;
struct sockaddr_in to = {0};

void udp_tx_task_init(const char *ipaddr, uint16_t video_port)
{
    sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0)
    {
        ESP_LOGE(TAG, "socket() failed");
        vTaskDelete(NULL);
        return;
    }

    to.sin_family = AF_INET;
    to.sin_port = htons(video_port);
    inet_pton(AF_INET, ipaddr, &to.sin_addr);

    tx_stats_init();

    ESP_LOGI(TAG, "UDP TX -> %s:%d", ipaddr, video_port);
}

static void udp_tx_task(void *arg)
{
    s_udp_run = true;
    while (1)
    {
        if (!s_udp_run)
        {
            break;
        }

        frame_item_t item;
        if (xQueueReceive(s_frame_q, &item, portMAX_DELAY) != pdTRUE)
        {
            continue;
        }

        camera_fb_t *fb = item.fb;
        if (!fb)
            continue;

        send_jpeg_udp(sock, &to, fb->buf, fb->len, item.frame_id);

        // ESP_LOGI(TAG, "UDP jpeg frame: %d", item.frame_id);

        // ВАЖНО: вернуть буфер в камеру, иначе всё станет колом.
        esp_camera_fb_return(fb);
    }

    vTaskDelete(NULL);
    close(sock);
    s_udp_th = NULL;
}

void udp_tx_start(QueueHandle_t frame_q)
{
    if (s_udp_th)
        return;
    s_frame_q = frame_q;
    xTaskCreatePinnedToCore(udp_tx_task, "udp_tx", 8192, NULL, 6, NULL, 1);
}

void udp_tx_stop(void)
{
    s_udp_run = false;
    vTaskDelay(pdMS_TO_TICKS(20));
    while (s_udp_th != NULL)
    {
        vTaskDelay(10);
    }
}

bool udp_tx_is_running(void)
{
    return s_udp_th != NULL;
}