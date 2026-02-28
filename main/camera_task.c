#include "camera_task.h"
#include "app_shared.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_camera.h"
#include "esp_timer.h"

static const char *TAG = "CAM";

// ======== ESP32-CAM AI Thinker pin map ========
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27

#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22

static uint16_t s_frame_id = 0;
static QueueHandle_t s_frame_q = NULL;

static TaskHandle_t s_cam_th = NULL;
static volatile bool s_cam_run = false;

static esp_err_t cam_init(void)
{
    camera_config_t c = {0};
    c.ledc_channel = LEDC_CHANNEL_0;
    c.ledc_timer = LEDC_TIMER_0;

    c.pin_d0 = Y2_GPIO_NUM;
    c.pin_d1 = Y3_GPIO_NUM;
    c.pin_d2 = Y4_GPIO_NUM;
    c.pin_d3 = Y5_GPIO_NUM;
    c.pin_d4 = Y6_GPIO_NUM;
    c.pin_d5 = Y7_GPIO_NUM;
    c.pin_d6 = Y8_GPIO_NUM;
    c.pin_d7 = Y9_GPIO_NUM;

    c.pin_xclk = XCLK_GPIO_NUM;
    c.pin_pclk = PCLK_GPIO_NUM;
    c.pin_vsync = VSYNC_GPIO_NUM;
    c.pin_href = HREF_GPIO_NUM;
    c.pin_sscb_sda = SIOD_GPIO_NUM;
    c.pin_sscb_scl = SIOC_GPIO_NUM;
    c.pin_pwdn = PWDN_GPIO_NUM;
    c.pin_reset = RESET_GPIO_NUM;

    c.xclk_freq_hz = 20000000;
    c.pixel_format = PIXFORMAT_JPEG;

    // Стартовые настройки (можно крутить):
    c.frame_size = FRAMESIZE_QVGA; // если сеть слабая: FRAMESIZE_QVGA
    c.jpeg_quality = 12;           // больше число => хуже качество, меньше трафик
    c.fb_count = 2;                // 2 буфера достаточно (это и есть "двойной буфер")

    esp_err_t err = esp_camera_init(&c);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_camera_init failed: 0x%x", err);
        return err;
    }
    ESP_LOGI(TAG, "Camera init OK");
    return ESP_OK;
}

static esp_err_t camera_deinit_safe(void)
{
    ESP_LOGW(TAG, "camera_deinit_safe: stopping camera...");

    // 1️⃣ Остановить драйвер захвата
    esp_err_t err = esp_camera_deinit();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_camera_deinit failed: %s", esp_err_to_name(err));
        return err;
    }

    // 2️⃣ Небольшая пауза чтобы:
    // - освободились DMA
    // - закрылись I2C/VSYNC/interrupt
    // - WiFi/flash не словили конфликт
    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_LOGW(TAG, "camera_deinit_safe: done");
    return ESP_OK;
}

void camera_task_init(void)
{
    ESP_ERROR_CHECK(cam_init());
}

static void camera_task(void *arg)
{
    uint32_t cam_frame_cnt = 0;
    int64_t cam_t0 = esp_timer_get_time();

    s_cam_run = true;
    while (1)
    {
        if (!s_cam_run)
        {
            break;
        }

        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb)
        {
            ESP_LOGW(TAG, "fb_get failed");
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }

        cam_frame_cnt++;

        int64_t now = esp_timer_get_time();
        int64_t dt = now - cam_t0;

        if (dt >= 1000000) // 1 сек
        {
            float sec = (float)dt / 1000000.0f;
            float fps = cam_frame_cnt / sec;

            ESP_LOGI("CAM", "CAM fps=%.1f frame_size=%u",
                     fps,
                     fb->len);

            cam_frame_cnt = 0;
            cam_t0 = now;
        }

        if (fb->format != PIXFORMAT_JPEG)
        {
            ESP_LOGW(TAG, "Non-JPEG frame, returning");
            esp_camera_fb_return(fb);
            continue;
        }

        frame_item_t item = {
            .fb = fb,
            .frame_id = ++s_frame_id};

        // Если очередь заполнена — лучше выкинуть самый старый кадр, чем блокироваться.
        // Пытаемся отправить быстро; если не получилось — возвращаем fb.
        if (xQueueSend(s_frame_q, &item, 0) != pdTRUE)
        {
            // Очередь занята — кадр выкидываем (важно вернуть!)
            esp_camera_fb_return(fb);
        }

        // Лёгкий yield
        vTaskDelay(1);
    }

    vTaskDelete(NULL);
    camera_deinit_safe();
    s_cam_th = NULL;
}

void camera_task_start(QueueHandle_t frame_q)
{
    if (s_cam_th)
        return; // уже запущено

    s_frame_q = frame_q;
    xTaskCreatePinnedToCore(camera_task, "camera_task", 8192, NULL, 5, NULL, 1); // 1
}

void camera_task_stop(void)
{
    s_cam_run = false;

    while (s_cam_th != NULL)
    {
        vTaskDelay(10);
    }
}

bool camera_task_is_running(void)
{
    return s_cam_th != NULL;
}
