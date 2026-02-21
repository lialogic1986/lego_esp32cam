#include "log_stream.h"
#include "ws_client.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "LSTR";

static int (*s_prev_vprintf)(const char *fmt, va_list ap);

static int ws_vprintf(const char *fmt, va_list ap)
{
    // Сначала печатаем в UART как обычно
    int r = 0;
    if (s_prev_vprintf)
    {
        va_list ap2;
        va_copy(ap2, ap);
        r = s_prev_vprintf(fmt, ap2);
        va_end(ap2);
    }

    if (!ws_client_is_connected())
        return r;

    char line[256];
    vsnprintf(line, sizeof(line), fmt, ap);

    // Уберём переводы строки в конце (чтобы JSON был аккуратный)
    size_t n = strlen(line);
    while (n && (line[n - 1] == '\n' || line[n - 1] == '\r'))
    {
        line[n - 1] = 0;
        n--;
    }

    // Очень простой JSON-escape: заменим кавычки
    for (char *p = line; *p; ++p)
    {
        if (*p == '\"')
            *p = '\'';
    }

    char msg[384];
    snprintf(msg, sizeof(msg),
             "{\"type\":\"log\",\"ts\":%lld,\"msg\":\"%s\"}",
             (long long)(esp_timer_get_time() / 1000), line);

    ws_client_send_text(msg);
    return r;
}

void log_stream_start(void)
{
    ESP_LOGI(TAG, "log stream start");
    s_prev_vprintf = esp_log_set_vprintf(ws_vprintf);
}