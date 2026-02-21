#include "ping_mod.h"
#include "ws_client.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>

#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"

#include "lwip/sockets.h"
#include "lwip/inet.h"
#include "lwip/ip.h"
#include "lwip/icmp.h"

static const char *TAG = "PINGMOD";

static bool s_running;

typedef struct
{
    char cmd_id[24];
    uint32_t sent, recv;
    uint32_t min_ms, max_ms, sum_ms;
    uint16_t ident;
} ping_stats_t;

static ping_stats_t s_ps;

static uint16_t icmp_checksum(const void *data, size_t len)
{
    const uint16_t *p = (const uint16_t *)data;
    uint32_t sum = 0;
    while (len > 1)
    {
        sum += *p++;
        len -= 2;
    }
    if (len == 1)
        sum += *(const uint8_t *)p;

    sum = (sum >> 16) + (sum & 0xFFFF);
    sum += (sum >> 16);
    return (uint16_t)~sum;
}

static esp_err_t get_gateway_ipv4(uint32_t *out_addr_net_order)
{
    if (!out_addr_net_order)
        return ESP_ERR_INVALID_ARG;

    esp_netif_t *n = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (!n)
        return ESP_FAIL;

    esp_netif_ip_info_t ip;
    if (esp_netif_get_ip_info(n, &ip) != ESP_OK)
        return ESP_FAIL;

    *out_addr_net_order = ip.gw.addr; // already network order
    return ESP_OK;
}

static void ws_sendf(const char *fmt, ...)
{
    char buf[512];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    (void)ws_client_send_text(buf);
}

static void finish_ok(uint32_t total_sent, uint32_t total_recv)
{
    uint32_t loss = (total_sent > 0) ? (100U * (total_sent - total_recv) / total_sent) : 0;
    uint32_t avg = (s_ps.recv > 0) ? (s_ps.sum_ms / s_ps.recv) : 0;

    ws_sendf(
        "{\"type\":\"cmd_result\",\"id\":\"%s\",\"ok\":true,\"result\":"
        "{\"sent\":%u,\"recv\":%u,\"loss_pct\":%u,\"min_ms\":%u,\"avg_ms\":%u,\"max_ms\":%u}}",
        s_ps.cmd_id,
        (unsigned)total_sent, (unsigned)total_recv, (unsigned)loss,
        (unsigned)s_ps.min_ms, (unsigned)avg, (unsigned)s_ps.max_ms);
}

static void finish_err(const char *err)
{
    ws_sendf("{\"type\":\"cmd_result\",\"id\":\"%s\",\"ok\":false,\"error\":\"%s\"}",
             s_ps.cmd_id, err ? err : "error");
}

static void ping_task(void *arg)
{
    (void)arg;

    uint32_t gw_net;
    if (get_gateway_ipv4(&gw_net) != ESP_OK)
    {
        finish_err("no_gateway");
        s_running = false;
        vTaskDelete(NULL);
        return;
    }

    struct sockaddr_in to = {0};
    to.sin_family = AF_INET;
    to.sin_addr.s_addr = gw_net;

    char ipbuf[16];
    inet_ntoa_r(to.sin_addr, ipbuf, sizeof(ipbuf));

    int sock = lwip_socket(AF_INET, SOCK_RAW, IP_PROTO_ICMP);
    if (sock < 0)
    {
        ESP_LOGE(TAG, "socket RAW ICMP failed errno=%d", errno);
        finish_err("sock_raw_fail");
        s_running = false;
        vTaskDelete(NULL);
        return;
    }

    ws_sendf("{\"type\":\"ping_status\",\"stage\":\"start\",\"target\":\"%s\"}", ipbuf);

    const int count = (int)((uintptr_t)arg); // not used; we’ll read from globals below
    (void)count;

    // параметры берём из статиков, заданных в start()
    // (простая схема: start() заполняет s_ps и доп. статики)
    // чтобы не плодить структуры — зашьём в локальные переменные:
    // (мы их сохраним в статических ниже)
    extern int g_ping_count, g_ping_interval_ms, g_ping_timeout_ms, g_ping_payload_size;
    int N = g_ping_count;
    int interval_ms = g_ping_interval_ms;
    int timeout_ms = g_ping_timeout_ms;
    int payload_size = g_ping_payload_size;

    if (payload_size < 0)
        payload_size = 0;
    if (payload_size > 256)
        payload_size = 256;

    uint8_t txbuf[sizeof(struct icmp_echo_hdr) + 256];
    uint8_t rxbuf[1280];

    for (int i = 0; i < N; i++)
    {
        struct icmp_echo_hdr *hdr = (struct icmp_echo_hdr *)txbuf;
        ICMPH_TYPE_SET(hdr, ICMP_ECHO);
        ICMPH_CODE_SET(hdr, 0);
        hdr->chksum = 0;
        hdr->id = htons(s_ps.ident);
        hdr->seqno = htons((uint16_t)i);

        // payload: timestamp (us) + padding
        int64_t t0 = esp_timer_get_time();
        memcpy(txbuf + sizeof(*hdr), &t0, sizeof(t0));
        for (int k = (int)sizeof(t0); k < payload_size; k++)
        {
            txbuf[sizeof(*hdr) + k] = (uint8_t)k;
        }

        size_t pkt_len = sizeof(*hdr) + (size_t)payload_size;
        hdr->chksum = icmp_checksum(txbuf, pkt_len);

        s_ps.sent++;

        int sent = lwip_sendto(sock, txbuf, pkt_len, 0, (struct sockaddr *)&to, sizeof(to));
        if (sent < 0)
        {
            ws_sendf("{\"type\":\"ping_status\",\"seq\":%u,\"send_err\":true,\"errno\":%d}",
                     (unsigned)i, errno);
            vTaskDelay(pdMS_TO_TICKS(interval_ms));
            continue;
        }

        // wait reply
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(sock, &rfds);
        struct timeval tv;
        tv.tv_sec = timeout_ms / 1000;
        tv.tv_usec = (timeout_ms % 1000) * 1000;

        int sel = lwip_select(sock + 1, &rfds, NULL, NULL, &tv);
        if (sel <= 0)
        {
            ws_sendf("{\"type\":\"ping_status\",\"seq\":%u,\"timeout\":true}", (unsigned)i);
            vTaskDelay(pdMS_TO_TICKS(interval_ms));
            continue;
        }

        struct sockaddr_in from;
        socklen_t fromlen = sizeof(from);
        int rlen = lwip_recvfrom(sock, rxbuf, sizeof(rxbuf), 0, (struct sockaddr *)&from, &fromlen);
        if (rlen <= 0)
        {
            ws_sendf("{\"type\":\"ping_status\",\"seq\":%u,\"recv_err\":true,\"errno\":%d}",
                     (unsigned)i, errno);
            vTaskDelay(pdMS_TO_TICKS(interval_ms));
            continue;
        }

        // Packet includes IP header -> find ICMP
        struct ip_hdr *iph = (struct ip_hdr *)rxbuf;
        int iphdr_len = IPH_HL(iph) * 4;
        if (rlen < iphdr_len + (int)sizeof(struct icmp_echo_hdr))
        {
            vTaskDelay(pdMS_TO_TICKS(interval_ms));
            continue;
        }

        struct icmp_echo_hdr *rh = (struct icmp_echo_hdr *)(rxbuf + iphdr_len);
        if (ICMPH_TYPE(rh) == ICMP_ER &&
            ntohs(rh->id) == s_ps.ident &&
            ntohs(rh->seqno) == (uint16_t)i)
        {

            int64_t t1 = esp_timer_get_time();
            int64_t t0_rx = 0;
            if (rlen >= iphdr_len + (int)sizeof(*rh) + (int)sizeof(t0_rx))
            {
                memcpy(&t0_rx, (uint8_t *)rh + sizeof(*rh), sizeof(t0_rx));
            }
            uint32_t rtt_ms = (uint32_t)((t1 - t0_rx) / 1000);

            s_ps.recv++;
            s_ps.sum_ms += rtt_ms;
            if (s_ps.min_ms == 0 || rtt_ms < s_ps.min_ms)
                s_ps.min_ms = rtt_ms;
            if (rtt_ms > s_ps.max_ms)
                s_ps.max_ms = rtt_ms;

            ws_sendf("{\"type\":\"ping_status\",\"seq\":%u,\"ms\":%u}", (unsigned)i, (unsigned)rtt_ms);
        }
        else
        {
            // not our reply, ignore
        }

        vTaskDelay(pdMS_TO_TICKS(interval_ms));
    }

    lwip_close(sock);

    finish_ok(s_ps.sent, s_ps.recv);
    s_running = false;
    vTaskDelete(NULL);
}

// глобальные параметры (простая схема)
int g_ping_count = 4;
int g_ping_interval_ms = 1000;
int g_ping_timeout_ms = 1000;
int g_ping_payload_size = 56;

esp_err_t ping_mod_start_gateway(const char *cmd_id, int count, int interval_ms, int timeout_ms, int payload_size)
{
    if (!cmd_id)
        return ESP_ERR_INVALID_ARG;
    if (s_running)
        return ESP_ERR_INVALID_STATE;

    memset(&s_ps, 0, sizeof(s_ps));
    strncpy(s_ps.cmd_id, cmd_id, sizeof(s_ps.cmd_id) - 1);
    s_ps.ident = (uint16_t)(esp_random() & 0xFFFF);

    g_ping_count = (count > 0) ? count : 4;
    g_ping_interval_ms = (interval_ms > 0) ? interval_ms : 1000;
    g_ping_timeout_ms = (timeout_ms > 0) ? timeout_ms : 1000;
    g_ping_payload_size = (payload_size >= 0) ? payload_size : 56;

    s_running = true;
    xTaskCreate(ping_task, "ping_task", 4096, NULL, 6, NULL);
    return ESP_OK;
}