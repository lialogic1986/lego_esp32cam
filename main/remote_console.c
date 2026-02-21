#include "remote_console.h"
#include "ws_client.h"
#include "freertos/FreeRTOS.h"

#include <string.h>
#include <stdio.h>
#include "esp_log.h"
#include "esp_system.h"
#include "cJSON.h"

static const char *TAG = "RCMD";

// Заглушки под твой проект (поезда/мотор/стрелки)
static void apply_speed(int speed)
{
    (void)speed;
    // TODO: дернуть твой motor control
}

static void reply_ok(const char *id, const char *json_result_obj_or_null)
{
    char msg[384];
    if (json_result_obj_or_null)
    {
        snprintf(msg, sizeof(msg),
                 "{\"type\":\"cmd_result\",\"id\":\"%s\",\"ok\":true,\"result\":%s}",
                 id, json_result_obj_or_null);
    }
    else
    {
        snprintf(msg, sizeof(msg),
                 "{\"type\":\"cmd_result\",\"id\":\"%s\",\"ok\":true}",
                 id);
    }
    ws_client_send_text(msg);
}

static void reply_err(const char *id, const char *err)
{
    char msg[384];
    snprintf(msg, sizeof(msg),
             "{\"type\":\"cmd_result\",\"id\":\"%s\",\"ok\":false,\"error\":\"%s\"}",
             id, err ? err : "error");
    ws_client_send_text(msg);
}

void remote_console_on_ws_text(const char *txt, int len)
{
    if (!txt || len <= 0)
        return;

    // cJSON ожидает 0-terminated
    char *buf = (char *)malloc((size_t)len + 1);
    if (!buf)
        return;
    memcpy(buf, txt, (size_t)len);
    buf[len] = 0;

    cJSON *root = cJSON_Parse(buf);
    free(buf);
    if (!root)
        return;

    cJSON *type = cJSON_GetObjectItem(root, "type");
    if (!cJSON_IsString(type))
    {
        cJSON_Delete(root);
        return;
    }

    if (strcmp(type->valuestring, "cmd") == 0)
    {
        cJSON *id = cJSON_GetObjectItem(root, "id");
        cJSON *name = cJSON_GetObjectItem(root, "name");
        cJSON *args = cJSON_GetObjectItem(root, "args");

        if (!cJSON_IsString(id) || !cJSON_IsString(name))
        {
            cJSON_Delete(root);
            return;
        }

        ESP_LOGI(TAG, "cmd %s (%s)", name->valuestring, id->valuestring);

        if (strcmp(name->valuestring, "reboot") == 0)
        {
            reply_ok(id->valuestring, NULL);
            vTaskDelay(pdMS_TO_TICKS(100));
            esp_restart();
        }
        else if (strcmp(name->valuestring, "speed_set") == 0)
        {
            int spd = 0;
            if (cJSON_IsObject(args))
            {
                cJSON *v = cJSON_GetObjectItem(args, "speed");
                if (cJSON_IsNumber(v))
                    spd = v->valueint;
            }
            apply_speed(spd);

            char res[64];
            snprintf(res, sizeof(res), "{\"applied\":%d}", spd);
            reply_ok(id->valuestring, res);
        }
        else if (strcmp(name->valuestring, "log_level") == 0)
        {
            // args: {"tag":"*", "level":3}
            // упрощенно: global level
            int lvl = 3;
            if (cJSON_IsObject(args))
            {
                cJSON *v = cJSON_GetObjectItem(args, "level");
                if (cJSON_IsNumber(v))
                    lvl = v->valueint;
            }
            // 0..5 => NONE/ERROR/WARN/INFO/DEBUG/VERBOSE
            esp_log_level_set("*", (esp_log_level_t)lvl);
            reply_ok(id->valuestring, NULL);
        }
        else if (strcmp(name->valuestring, "ota_check") == 0)
        {
            // Сюда позже повесим вызов ota_check_and_update()
            reply_ok(id->valuestring, NULL);
        }
        else
        {
            reply_err(id->valuestring, "unknown_cmd");
        }
    }

    cJSON_Delete(root);
}