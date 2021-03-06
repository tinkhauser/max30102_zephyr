/*
 * Copyright (c) 2021 EVERGREEN FUND 501(c)(3)
 * Copyright (c) 2021, 2022 Jacob Tinkhauser
 *
 * SPDX-License-Identifier: AGPL-3.0-or-later
 */

#ifndef _MAX30102_H
#define _MAX30102_H

#include <drivers/gpio.h>
#include <drivers/i2c.h>
#include <drivers/sensor.h>

#define DT_DRV_COMPAT maxim_max30102

#define MAX30102_REG_INT_STS1 0x00
#define MAX30102_REG_INT_STS2 0x01
#define MAX30102_REG_INT_EN1 0x02
#define MAX30102_REG_INT_EN2 0x03
#define MAX30102_REG_FIFO_WR 0x04
#define MAX30102_REG_FIFO_OVF 0x05
#define MAX30102_REG_FIFO_RD 0x06
#define MAX30102_REG_FIFO_DATA 0x07
#define MAX30102_REG_FIFO_CFG 0x08
#define MAX30102_REG_MODE_CFG 0x09
#define MAX30102_REG_SPO2_CFG 0x0a
#define MAX30102_REG_LED1_PA 0x0c
#define MAX30102_REG_LED2_PA 0x0d
/* #define MAX30102_REG_LED3_PA		0x0e */
#define MAX30102_REG_PILOT_PA 0x10
#define MAX30102_REG_MULTI_LED 0x11
#define MAX30102_REG_TINT 0x1f
#define MAX30102_REG_TFRAC 0x20
#define MAX30102_REG_TEMP_CFG 0x21
#define MAX30102_REG_PROX_INT 0x30
#define MAX30102_REG_REV_ID 0xfe
#define MAX30102_REG_PART_ID 0xff

#define MAX30102_INT_PPG_MASK (1 << 6)

#define MAX30102_FIFO_CFG_SMP_AVE_SHIFT 5
#define MAX30102_FIFO_CFG_FIFO_FULL_SHIFT 0
#define MAX30102_FIFO_CFG_ROLLOVER_EN_MASK (1 << 4)

#define MAX30102_MODE_CFG_SHDN_MASK (1 << 7)
#define MAX30102_MODE_CFG_RESET_MASK (1 << 6)

#define MAX30102_SPO2_ADC_RGE_SHIFT 5
#define MAX30102_SPO2_SR_SHIFT 2
#define MAX30102_SPO2_PW_SHIFT 0

#define MAX30102_PART_ID 0x15

#define MAX30102_BYTES_PER_CHANNEL 3
#define MAX30102_MAX_NUM_CHANNELS 2
#define MAX30102_MAX_BYTES_PER_SAMPLE (MAX30102_MAX_NUM_CHANNELS * \
                                       MAX30102_BYTES_PER_CHANNEL)

#define MAX30102_SLOT_LED_MASK 0x03

#define MAX30102_FIFO_DATA_BITS 18
#define MAX30102_FIFO_DATA_MASK ((1 << MAX30102_FIFO_DATA_BITS) - 1)

#define MAX30102_INTR_2_DIE_TEMP_RDY_EN 2
#define MAX30102_TEMP_CFG_TEMP_EN 1

/* Interrupt Status 1 Register */
#define MAX30102_INTR_STATUS_1_A_FULL(x) (((x) >> 7) & 0x1)
#define MAX30102_INTR_STATUS_1_PPG_RDY(x) (((x) >> 6) & 0x1)
#define MAX30102_INTR_STATUS_1_ALC_OVF(x) (((x) >> 5) & 0x1)
#define MAX30102_INTR_STATUS_1_PWR_RDY(x) (((x) >> 0) & 0x1)

/* Interrupt Status 2 Register */
#define MAX30102_INTR_STATUS_2_DIE_TEMP_RDY(x) (((x) >> 1) & 0x1)

enum max30102_mode {
    MAX30102_MODE_HEART_RATE = 2,
    MAX30102_MODE_SPO2 = 3,
    MAX30102_MODE_MULTI_LED = 7,
};

enum max30102_slot {
    MAX30102_SLOT_DISABLED = 0,
    MAX30102_SLOT_RED_LED1_PA,
    MAX30102_SLOT_IR_LED2_PA,
    MAX30102_SLOT_GREEN_LED3_PA,
    MAX30102_SLOT_RED_PILOT_PA,
    MAX30102_SLOT_IR_PILOT_PA,
    MAX30102_SLOT_GREEN_PILOT_PA,
};

enum max30102_led_channel {
    MAX30102_LED_CHANNEL_RED = 0,
    MAX30102_LED_CHANNEL_IR,
    MAX30102_LED_CHANNEL_GREEN,
};

enum max30102_pw {
    MAX30102_PW_15BITS = 0,
    MAX30102_PW_16BITS,
    MAX30102_PW_17BITS,
    MAX30102_PW_18BITS,
};

enum max30102_intr_1 {
    MAX30102_INTR_1_ALC_OVF_EN = 32,
    MAX30102_INTR_1_PPG_RDY_EN = 64,
    MAX30102_INTR_1_A_FULL_EN = 128,
};

uint32_t reverse_32(uint32_t value);
uint8_t max30102_get_interrupt_status_1(const struct device *dev, uint8_t *status);
uint8_t max30102_get_interrupt_status_2(const struct device *dev, uint8_t *status);

struct max30102_config {
    const char *i2c_label;
    uint16_t i2c_addr;
    uint8_t fifo;
    uint8_t spo2;
    uint8_t led_pa[MAX30102_MAX_NUM_CHANNELS];
    enum max30102_mode mode;
    enum max30102_slot slot[4];
    enum max30102_intr_1 intr_1;
    uint8_t intr_2;
    uint8_t temp_cfg;

#ifdef CONFIG_MAX30102_TRIGGER
    const struct device *intb_gpio;
    gpio_pin_t intb_pin;
    gpio_dt_flags_t intb_flags;
#endif
};

struct max30102_data {
    const struct device *i2c;
    uint32_t raw[MAX30102_MAX_NUM_CHANNELS];
    uint8_t map[MAX30102_MAX_NUM_CHANNELS];
    uint8_t num_channels;

    /* Die temperature */
    int8_t tint;
    uint8_t tfrac;

#ifdef CONFIG_MAX30102_TRIGGER
    struct gpio_callback gpio_cb; /* TODO what does this do? */

    struct k_mutex trigger_mutex;
    /* TODO add handler for temp */
    sensor_trigger_handler_t drdy_handler;
    struct sensor_trigger drdy_trigger;
    const struct device *dev;

#ifdef CONFIG_MAX30102_TRIGGER_OWN_THREAD
    K_THREAD_STACK_MEMBER(thread_stack, CONFIG_MAX30102_THREAD_STACK_SIZE);
    struct k_sem gpio_sem;
    struct k_thread thread;
#elif CONFIG_MAX30102_TRIGGER_GLOBAL_THREAD
    struct k_work work;
#endif
#endif /* CONFIG_MAX30102_TRIGGER */
};

int max30102_trigger_set(const struct device *dev,
                         const struct sensor_trigger *trig,
                         sensor_trigger_handler_t handler);

int max30102_init_interrupt(const struct device *dev);
#endif /*_MAX30102_H*/