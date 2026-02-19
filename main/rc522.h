#pragma once
#include <stdint.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

bool rc522_init(void);

uint8_t rc522_read_reg(uint8_t reg);
void rc522_write_reg(uint8_t reg, uint8_t val);

void rc522_set_bitmask(uint8_t reg, uint8_t mask);
void rc522_clear_bitmask(uint8_t reg, uint8_t mask);

bool rc522_get_uid(uint8_t *uid, uint8_t *uid_len);     // polling версия (как у тебя сейчас)
bool rc522_get_uid_irq(uint8_t *uid, uint8_t *uid_len); // IRQ версия

// Для IRQ-модели:
void rc522_set_irq_task(TaskHandle_t task_to_notify);
void rc522_irq_notify_from_isr(void);
