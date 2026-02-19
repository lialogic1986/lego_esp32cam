// rc522.c — полный файл с фиксами ожидания по ComIrqReg + расширенная отладка
// ESP-IDF v5.x, SPI2_HOST (HSPI), пины под твою схему:
// SCK=14, MOSI=15, MISO=2, CS=13, RST=12
//
// Включить/выключить отладку: RC522_DEBUG 1/0

#include "rc522.h"

#include <string.h>

#include "esp_log.h"
#include "esp_err.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/spi_master.h"
#include "driver/gpio.h"

static const char *TAG = "RC522";

#define RC522_DEBUG 0

#if RC522_DEBUG
#define DLOGI(...) ESP_LOGI(TAG, __VA_ARGS__)
#define DLOGW(...) ESP_LOGW(TAG, __VA_ARGS__)
#define DLOGE(...) ESP_LOGE(TAG, __VA_ARGS__)
#else
#define DLOGI(...)
#define DLOGW(...)
#define DLOGE(...)
#endif

// ===== Pin map (твоя схема) =====
#define RC522_SCK GPIO_NUM_14
#define RC522_MOSI GPIO_NUM_15
#define RC522_MISO GPIO_NUM_2
#define RC522_CS GPIO_NUM_13
#define RC522_RST GPIO_NUM_12

// ===== RC522 regs =====
#define CommandReg 0x01
#define ComIEnReg 0x02
#define DivIEnReg 0x03
#define ComIrqReg 0x04
#define DivIrqReg 0x05
#define ErrorReg 0x06
#define Status1Reg 0x07
#define Status2Reg 0x08
#define FIFODataReg 0x09
#define FIFOLevelReg 0x0A
#define ControlReg 0x0C
#define BitFramingReg 0x0D
#define CollReg 0x0E

#define ModeReg 0x11
#define TxModeReg 0x12
#define RxModeReg 0x13
#define TxControlReg 0x14
#define TxASKReg 0x15

#define CRCResultRegH 0x21
#define CRCResultRegL 0x22

#define TModeReg 0x2A
#define TPrescalerReg 0x2B
#define TReloadRegH 0x2C
#define TReloadRegL 0x2D

#define VersionReg 0x37

// ===== Commands =====
#define PCD_Idle 0x00
#define PCD_CalcCRC 0x03
#define PCD_Transceive 0x0C
#define PCD_SoftReset 0x0F

// ===== PICC =====
#define PICC_REQA 0x26

// ISO14443A cascade levels
#define PICC_ANTICOLL_CL1 0x93
#define PICC_ANTICOLL_CL2 0x95
#define PICC_SEL 0x70

static spi_device_handle_t s_spi = NULL;

static inline uint8_t addr_read(uint8_t reg) { return (uint8_t)(0x80 | (reg << 1)); }
static inline uint8_t addr_write(uint8_t reg) { return (uint8_t)((reg << 1) & 0x7E); }

static TaskHandle_t s_irq_task = NULL;

void rc522_set_irq_task(TaskHandle_t task_to_notify)
{
    s_irq_task = task_to_notify;
}

void rc522_irq_notify_from_isr(void)
{
    BaseType_t hp = pdFALSE;
    if (s_irq_task)
    {
        vTaskNotifyGiveFromISR(s_irq_task, &hp);
    }
    if (hp)
    {
        portYIELD_FROM_ISR();
    }
}

uint8_t rc522_read_reg(uint8_t reg)
{
    uint8_t tx[2] = {addr_read(reg), 0x00};
    uint8_t rx[2] = {0};

    spi_transaction_t t = {0};
    t.length = 8 * sizeof(tx);
    t.tx_buffer = tx;
    t.rx_buffer = rx;

    if (!s_spi)
    {
        DLOGE("SPI not init");
        return 0x00;
    }

    esp_err_t err = spi_device_transmit(s_spi, &t);
    if (err != ESP_OK)
    {
        DLOGE("spi_device_transmit(read 0x%02X) failed: %s", reg, esp_err_to_name(err));
        return 0x00;
    }
    return rx[1];
}

void rc522_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = {addr_write(reg), val};

    spi_transaction_t t = {0};
    t.length = 8 * sizeof(tx);
    t.tx_buffer = tx;

    if (!s_spi)
    {
        DLOGE("SPI not init");
        return;
    }

    esp_err_t err = spi_device_transmit(s_spi, &t);
    if (err != ESP_OK)
    {
        DLOGE("spi_device_transmit(write 0x%02X) failed: %s", reg, esp_err_to_name(err));
    }
}

void rc522_set_bitmask(uint8_t reg, uint8_t mask)
{
    rc522_write_reg(reg, rc522_read_reg(reg) | mask);
}

void rc522_clear_bitmask(uint8_t reg, uint8_t mask)
{
    rc522_write_reg(reg, rc522_read_reg(reg) & (uint8_t)(~mask));
}

static void rc522_soft_reset(void)
{
    rc522_write_reg(CommandReg, PCD_SoftReset);
    vTaskDelay(pdMS_TO_TICKS(50));
}

static void rc522_antenna_on(void)
{
    uint8_t v = rc522_read_reg(TxControlReg);
    if ((v & 0x03) != 0x03)
    {
        rc522_write_reg(TxControlReg, v | 0x03);
    }
}

static bool rc522_spi_init(void)
{
    // RST pin
    gpio_config_t rst = {
        .pin_bit_mask = 1ULL << RC522_RST,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE};
    ESP_ERROR_CHECK(gpio_config(&rst));

    // HW reset pulse
    gpio_set_level(RC522_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(30));
    gpio_set_level(RC522_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(30));

    spi_bus_config_t buscfg = {
        .sclk_io_num = RC522_SCK,
        .mosi_io_num = RC522_MOSI,
        .miso_io_num = RC522_MISO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 64};

    // HSPI = SPI2_HOST
    esp_err_t err = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
    {
        ESP_LOGE(TAG, "spi_bus_initialize failed: %s", esp_err_to_name(err));
        return false;
    }

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1000000, // 1 MHz для надёжного старта
        .mode = 0,
        .spics_io_num = RC522_CS,
        .queue_size = 1};

    if (!s_spi)
    {
        err = spi_bus_add_device(SPI2_HOST, &devcfg, &s_spi);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "spi_bus_add_device failed: %s", esp_err_to_name(err));
            return false;
        }
    }

    return true;
}

static void dump_fail(const char *where, uint8_t irq, uint8_t err, uint8_t fifo)
{
#if RC522_DEBUG
    DLOGW("%s fail: ComIrqReg=0x%02X ErrorReg=0x%02X FIFO=%u Status1=0x%02X Status2=0x%02X Control=0x%02X",
          where, irq, err, fifo,
          rc522_read_reg(Status1Reg),
          rc522_read_reg(Status2Reg),
          rc522_read_reg(ControlReg));
#else
    (void)where;
    (void)irq;
    (void)err;
    (void)fifo;
#endif
}
#include "hal/gpio_hal.h"
#include "soc/gpio_struct.h"
static void debug_gpio4_mode(void)
{
    // GPIO enable bit = output enable
    int out_en = (GPIO.enable >> 4) & 0x1; // для GPIO0..31
    int in_val = (GPIO.in >> 4) & 0x1;

    ESP_LOGI("GPIO4", "out_en=%d in=%d", out_en, in_val);
}

// ВНИМАНИЕ: эта функция ждёт GPIO IRQ через task notify.
// rfid_task должен:
//  - вызвать rc522_set_irq_task(xTaskGetCurrentTaskHandle())
//  - настроить GPIO4 interrupt и в ISR вызывать rc522_irq_notify_from_isr()
static bool rc522_transceive_irq(const uint8_t *tx, uint8_t tx_len,
                                 uint8_t *rx, uint8_t *rx_len,
                                 uint8_t tx_last_bits, uint16_t timeout_ms,
                                 const char *dbg_name)
{
    // stop
    rc522_write_reg(0x01 /*CommandReg*/, 0x00 /*PCD_Idle*/);

    // clear irq flags (это также отпускает IRQ pin вверх)
    rc522_write_reg(0x04 /*ComIrqReg*/, 0x7F);

    // flush FIFO
    rc522_set_bitmask(0x0A /*FIFOLevelReg*/, 0x80);

    // bit framing: TxLastBits in low 3 bits
    rc522_write_reg(0x0D /*BitFramingReg*/, (tx_last_bits & 0x07));

    // write FIFO
    for (uint8_t i = 0; i < tx_len; i++)
    {
        rc522_write_reg(0x09 /*FIFODataReg*/, tx[i]);
    }

    // start transceive
    rc522_write_reg(0x01 /*CommandReg*/, 0x0C /*PCD_Transceive*/);
    rc522_set_bitmask(0x0D /*BitFramingReg*/, 0x80); // StartSend

    // ждём IRQ с таймаутом
    uint32_t got = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(timeout_ms));
    if (got == 0)
    {
        debug_gpio4_mode();
        // timeout: почистим флаги, чтобы IRQ отпустился
        rc522_write_reg(0x04 /*ComIrqReg*/, 0x7F);
        if (dbg_name)
        {
            ESP_LOGW("RC522", "%s: IRQ wait timeout", dbg_name);
        }

        return false;
    }

    uint8_t irq = rc522_read_reg(0x04 /*ComIrqReg*/);
    uint8_t err = rc522_read_reg(0x06 /*ErrorReg*/);

    // сразу очистим, чтобы IRQ pin отпустился для следующего события
    rc522_write_reg(0x04 /*ComIrqReg*/, 0x7F);

    // TimerIRq (bit0) = timeout
    if (irq & 0x01)
    {
        if (dbg_name)
        {
            ESP_LOGW("RC522", "%s: TimerIRq (timeout) irq=0x%02X err=0x%02X", dbg_name, irq, err);
        }
        return false;
    }

    // базовые ошибки
    if (err & 0x1B)
    {
        if (dbg_name)
        {
            ESP_LOGW("RC522", "%s: error irq=0x%02X err=0x%02X", dbg_name, irq, err);
        }
        return false;
    }

    if (!rx || !rx_len || *rx_len == 0)
        return false;

    uint8_t fifo = rc522_read_reg(0x0A /*FIFOLevelReg*/);
    if (fifo == 0)
        return false;

    uint8_t n = fifo;
    if (n > *rx_len)
        n = *rx_len;

    for (uint8_t i = 0; i < n; i++)
    {
        rx[i] = rc522_read_reg(0x09 /*FIFODataReg*/);
    }
    *rx_len = n;
    return true;
}

// Надёжный обмен: стартуем Transceive и ждём RxIRq/IdleIRq/TimerIRq с таймаутом
static bool rc522_transceive(const uint8_t *tx, uint8_t tx_len,
                             uint8_t *rx, uint8_t *rx_len,
                             uint8_t tx_last_bits, uint16_t timeout_ms,
                             const char *dbg_name)
{
    rc522_write_reg(CommandReg, PCD_Idle);
    rc522_write_reg(ComIrqReg, 0x7F);      // clear all irq flags
    rc522_set_bitmask(FIFOLevelReg, 0x80); // flush FIFO

    // lower 3 bits = TxLastBits
    rc522_write_reg(BitFramingReg, (tx_last_bits & 0x07));

    for (uint8_t i = 0; i < tx_len; i++)
    {
        rc522_write_reg(FIFODataReg, tx[i]);
    }

    rc522_write_reg(CommandReg, PCD_Transceive);
    rc522_set_bitmask(BitFramingReg, 0x80); // StartSend

    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);
    uint8_t irq = 0;
    while (xTaskGetTickCount() < deadline)
    {
        irq = rc522_read_reg(ComIrqReg);
        if (irq & 0x20)
            break; // RxIRq
        if (irq & 0x10)
            break; // IdleIRq
        if (irq & 0x01)
            break; // TimerIRq
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    uint8_t err = rc522_read_reg(ErrorReg);
    uint8_t fifo = rc522_read_reg(FIFOLevelReg);

    // TimerIRq -> timeout
    if (irq & 0x01)
    {
        dump_fail(dbg_name ? dbg_name : "transceive", irq, err, fifo);
        return false;
    }

    // Error bits: (BufferOvfl 0x10), (ParityErr 0x08), (ProtocolErr 0x01), (CollErr 0x20)
    // Тут базово фильтруем 0x1B как раньше; CollErr отдельно может быть допустим при anticoll,
    // но на первых шагах проще считать ошибкой.
    if (err & 0x1B)
    {
        dump_fail(dbg_name ? dbg_name : "transceive", irq, err, fifo);
        return false;
    }

    if (!rx || !rx_len || *rx_len == 0)
    {
        dump_fail(dbg_name ? dbg_name : "transceive", irq, err, fifo);
        return false;
    }

    if (fifo == 0)
    {
        dump_fail(dbg_name ? dbg_name : "transceive", irq, err, fifo);
        return false;
    }

    uint8_t want = *rx_len;
    uint8_t n = fifo > want ? want : fifo;
    for (uint8_t i = 0; i < n; i++)
    {
        rx[i] = rc522_read_reg(FIFODataReg);
    }
    *rx_len = n;
    return true;
}

static bool rc522_calc_crc(const uint8_t *data, uint8_t len, uint8_t *crc_l, uint8_t *crc_h)
{
    if (!crc_l || !crc_h)
        return false;

    rc522_write_reg(CommandReg, PCD_Idle);
    rc522_write_reg(DivIrqReg, 0x04);      // clear CRCIRq
    rc522_set_bitmask(FIFOLevelReg, 0x80); // flush FIFO

    for (uint8_t i = 0; i < len; i++)
        rc522_write_reg(FIFODataReg, data[i]);

    rc522_write_reg(CommandReg, PCD_CalcCRC);

    // wait CRCIRq
    for (int i = 0; i < 50; i++)
    {
        uint8_t n = rc522_read_reg(DivIrqReg);
        if (n & 0x04)
        {
            *crc_l = rc522_read_reg(CRCResultRegL);
            *crc_h = rc522_read_reg(CRCResultRegH);
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

#if RC522_DEBUG
    DLOGW("CRC calc timeout");
#endif
    return false;
}

bool rc522_init(void)
{
    if (!rc522_spi_init())
        return false;

    uint8_t v = rc522_read_reg(VersionReg);
    ESP_LOGI(TAG, "VersionReg=0x%02X (expected usually 0x91/0x92)", v);
    if (v == 0x00 || v == 0xFF)
        return false;

    rc522_soft_reset();

    // Timer setup (часто используемые стабильные значения)
    rc522_write_reg(TModeReg, 0x8D);
    rc522_write_reg(TPrescalerReg, 0x3E);
    rc522_write_reg(TReloadRegL, 30);
    rc522_write_reg(TReloadRegH, 0);

    rc522_write_reg(TxASKReg, 0x40); // force 100% ASK
    rc522_write_reg(ModeReg, 0x3D);  // CRC preset 0x6363

    rc522_antenna_on();

    // Clear IRQ flags
    rc522_write_reg(ComIrqReg, 0x7F);
    rc522_write_reg(DivIrqReg, 0x7F);

    // Включаем источники прерываний, чтобы RC522 дёргал пин IRQ.
    // Важно: bit7 (Set1) должен быть 1, иначе IRQ pin может не работать.
    // RxIEn(0x20) + IdleIEn(0x10) + ErrIEn(0x02) + TimerIEn(0x01)
    rc522_write_reg(ComIEnReg, 0x80 | 0x20 | 0x10 | 0x02 | 0x01);

    // Очистим флаги, чтобы IRQ pin был отпущен (high)
    rc522_write_reg(ComIrqReg, 0x7F);

#if RC522_DEBUG
    DLOGI("Init done. TxControl=0x%02X", rc522_read_reg(TxControlReg));
#endif
    return true;
}

// REQA через IRQ (7 bits)
static bool rc522_request_a_irq(uint8_t atqa[2])
{
    uint8_t tx[1] = {0x26 /*PICC_REQA*/};
    uint8_t rx[4] = {0};
    uint8_t rx_len = sizeof(rx);

    bool ok = rc522_transceive_irq(tx, 1, rx, &rx_len, 0x07, 30, "REQA");
    if (!ok || rx_len < 2)
        return false;

    atqa[0] = rx[0];
    atqa[1] = rx[1];
    return true;
}

// REQA (ATQA) — ВАЖНО: теперь ждём корректно по ComIrqReg (не просто delay 2ms)
bool rc522_request_a(uint8_t atqa[2])
{
    if (!atqa)
        return false;

    uint8_t tx[1] = {PICC_REQA};
    uint8_t rx[4] = {0};
    uint8_t rx_len = sizeof(rx);

    // REQA: 7 valid bits in last byte
    bool ok = rc522_transceive(tx, 1, rx, &rx_len, 0x07, 20, "REQA");
    if (!ok || rx_len < 2)
        return false;

    atqa[0] = rx[0];
    atqa[1] = rx[1];

#if RC522_DEBUG
    DLOGI("ATQA=%02X %02X (rx_len=%u)", atqa[0], atqa[1], rx_len);
#endif
    return true;
}

static bool anticoll_select_level_irq(uint8_t sel_code, uint8_t *uid_part_out, uint8_t *sak_out, bool *more_levels)
{
    // ANTICOLL: [SEL, 0x20]
    uint8_t tx1[2] = {sel_code, 0x20};
    uint8_t rx1[10] = {0};
    uint8_t rx1_len = sizeof(rx1);

    if (!rc522_transceive_irq(tx1, sizeof(tx1), rx1, &rx1_len, 0, 30, "ANTICOLL"))
        return false;
    if (rx1_len < 5)
        return false;

    uint8_t bcc = rx1[0] ^ rx1[1] ^ rx1[2] ^ rx1[3];
    if (bcc != rx1[4])
        return false;

    // SELECT frame + CRC (используй твою rc522_calc_crc)
    uint8_t sel_frame[7] = {sel_code, 0x70, rx1[0], rx1[1], rx1[2], rx1[3], rx1[4]};
    uint8_t crc_l = 0, crc_h = 0;
    if (!rc522_calc_crc(sel_frame, sizeof(sel_frame), &crc_l, &crc_h))
        return false;

    uint8_t tx2[9] = {sel_code, 0x70, rx1[0], rx1[1], rx1[2], rx1[3], rx1[4], crc_l, crc_h};
    uint8_t rx2[5] = {0};
    uint8_t rx2_len = sizeof(rx2);

    if (!rc522_transceive_irq(tx2, sizeof(tx2), rx2, &rx2_len, 0, 30, "SELECT"))
        return false;
    if (rx2_len < 1)
        return false;

    uint8_t sak = rx2[0];
    if (sak_out)
        *sak_out = sak;

    if (rx1[0] == 0x88)
    {
        uid_part_out[0] = rx1[1];
        uid_part_out[1] = rx1[2];
        uid_part_out[2] = rx1[3];
    }
    else
    {
        uid_part_out[0] = rx1[0];
        uid_part_out[1] = rx1[1];
        uid_part_out[2] = rx1[2];
        uid_part_out[3] = rx1[3];
    }

    if (more_levels)
        *more_levels = ((sak & 0x04) != 0);
    return true;
}

static bool anticoll_select_level(uint8_t sel_code, uint8_t *uid_part_out, uint8_t *sak_out, bool *more_levels)
{
    // ANTICOLL: [SEL, 0x20] -> expect 5 bytes (uid0..uid3 + bcc)
    uint8_t tx1[2] = {sel_code, 0x20};
    uint8_t rx1[10] = {0};
    uint8_t rx1_len = sizeof(rx1);

    if (!rc522_transceive(tx1, sizeof(tx1), rx1, &rx1_len, 0, 30, "ANTICOLL"))
        return false;
    if (rx1_len < 5)
    {
        dump_fail("ANTICOLL(short)", rc522_read_reg(ComIrqReg), rc522_read_reg(ErrorReg), rc522_read_reg(FIFOLevelReg));
        return false;
    }

    // verify BCC
    uint8_t bcc = rx1[0] ^ rx1[1] ^ rx1[2] ^ rx1[3];
    if (bcc != rx1[4])
    {
#if RC522_DEBUG
        DLOGW("BCC mismatch: got=%02X exp=%02X (SEL=%02X)", rx1[4], bcc, sel_code);
#endif
        return false;
    }

    // SELECT frame: [SEL, 0x70, uid0..uid3, bcc, crcA(2)]
    uint8_t sel_frame[7] = {sel_code, PICC_SEL, rx1[0], rx1[1], rx1[2], rx1[3], rx1[4]};
    uint8_t crc_l = 0, crc_h = 0;
    if (!rc522_calc_crc(sel_frame, sizeof(sel_frame), &crc_l, &crc_h))
        return false;

    uint8_t tx2[9] = {sel_code, PICC_SEL, rx1[0], rx1[1], rx1[2], rx1[3], rx1[4], crc_l, crc_h};
    uint8_t rx2[5] = {0};
    uint8_t rx2_len = sizeof(rx2);

    if (!rc522_transceive(tx2, sizeof(tx2), rx2, &rx2_len, 0, 30, "SELECT"))
        return false;
    if (rx2_len < 1)
        return false;

    uint8_t sak = rx2[0];
    if (sak_out)
        *sak_out = sak;

    // UID part:
    // If first byte is 0x88 => cascade tag => uid bytes are rx1[1..3] (3 bytes)
    if (rx1[0] == 0x88)
    {
        uid_part_out[0] = rx1[1];
        uid_part_out[1] = rx1[2];
        uid_part_out[2] = rx1[3];
    }
    else
    {
        uid_part_out[0] = rx1[0];
        uid_part_out[1] = rx1[1];
        uid_part_out[2] = rx1[2];
        uid_part_out[3] = rx1[3];
    }

    if (more_levels)
        *more_levels = ((sak & 0x04) != 0);

#if RC522_DEBUG
    DLOGI("SELECT SAK=0x%02X more=%d SEL=0x%02X", sak, (int)((sak & 0x04) != 0), sel_code);
#endif
    return true;
}

bool rc522_get_uid_irq(uint8_t *uid, uint8_t *uid_len)
{
    if (!uid || !uid_len)
        return false;

    uint8_t atqa[2];
    if (!rc522_request_a_irq(atqa))
        return false;

    // Level 1
    uint8_t sak1 = 0;
    bool more = false;
    uint8_t part1[4] = {0};

    if (!anticoll_select_level_irq(0x93 /*CL1*/, part1, &sak1, &more))
        return false;

    if (!more)
    {
        memcpy(uid, part1, 4);
        *uid_len = 4;
        return true;
    }

    // Level 2 (7-byte UID типично для наклеек)
    uint8_t sak2 = 0;
    bool more2 = false;
    uint8_t part2[4] = {0};

    if (!anticoll_select_level_irq(0x95 /*CL2*/, part2, &sak2, &more2))
        return false;

    uid[0] = part1[0];
    uid[1] = part1[1];
    uid[2] = part1[2];
    uid[3] = part2[0];
    uid[4] = part2[1];
    uid[5] = part2[2];
    uid[6] = part2[3];
    *uid_len = 7;

    (void)more2;
    return true;
}

bool rc522_get_uid(uint8_t *uid, uint8_t *uid_len)
{
    if (!uid || !uid_len)
        return false;

    uint8_t atqa[2];
    if (!rc522_request_a(atqa))
        return false;

    // Level 1
    uint8_t sak1 = 0;
    bool more = false;
    uint8_t part1[4] = {0};

    if (!anticoll_select_level(PICC_ANTICOLL_CL1, part1, &sak1, &more))
        return false;

    if (!more)
    {
        memcpy(uid, part1, 4);
        *uid_len = 4;
#if RC522_DEBUG
        DLOGI("UID4=%02X:%02X:%02X:%02X", uid[0], uid[1], uid[2], uid[3]);
#endif
        return true;
    }

    // Level 2 (типично для наклеек NTAG/Ultralight: 7-byte UID)
    uint8_t sak2 = 0;
    bool more2 = false;
    uint8_t part2[4] = {0};

    if (!anticoll_select_level(PICC_ANTICOLL_CL2, part2, &sak2, &more2))
        return false;

    // Compose 7-byte UID: 3 bytes from L1 + 4 bytes from L2
    uid[0] = part1[0];
    uid[1] = part1[1];
    uid[2] = part1[2];
    uid[3] = part2[0];
    uid[4] = part2[1];
    uid[5] = part2[2];
    uid[6] = part2[3];
    *uid_len = 7;

#if RC522_DEBUG
    DLOGI("UID7=%02X:%02X:%02X:%02X:%02X:%02X:%02X",
          uid[0], uid[1], uid[2], uid[3], uid[4], uid[5], uid[6]);
#endif

    // 10-byte UID (level3) пока не делаем
    (void)more2;
    return true;
}
