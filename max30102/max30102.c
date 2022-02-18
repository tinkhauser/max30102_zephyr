/* max30102.c - Driver for Maxim MAX30102 */

/*
 * Copyright (c) 2021, 2022 EVERGREEN FUND 501(c)(3)
 *
 * SPDX-License-Identifier: AGPL-3.0-or-later
 */

#include "max30102.h"

#include <devicetree.h>
#include <drivers/gpio.h>
#include <drivers/sensor.h>
#include <init.h>
#include <kernel.h>
#include <logging/log.h>
#include <pm/device.h>
#include <sys/__assert.h>
#include <sys/byteorder.h>
LOG_MODULE_REGISTER(MAX30102, CONFIG_SENSOR_LOG_LEVEL);

#if DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 0
#warning "MAX30102 driver enabled without any devices"
#endif

/**
 * @brief Read interrupt status register 1 (0x00) on MAX30102
 *
 * @param dev device structure
 * @param status interrupt register status 1
 * @return 0 in case of success, negative error code otherwise.
 */
uint8_t max30102_get_interrupt_status_1(const struct device *dev, uint8_t *status) {
    struct max30102_data *data = dev->data;
    const struct max30102_config *config = dev->config;

    if (i2c_reg_read_byte(data->i2c, config->i2c_addr,
                          MAX30102_REG_INT_STS1, status)) {
        LOG_ERR("Couldn't read INT_STS1");
        return -EIO;
    }

    LOG_INF("STS1 status %d", *status);
    return 0;
}

/**
 * @brief Read interrupt status register 2 (0x01) on MAX30102
 *
 * @param dev device structure
 * @param status interrupt register status 2
 */
uint8_t max30102_get_interrupt_status_2(const struct device *dev, uint8_t *status) {
    struct max30102_data *data = dev->data;
    const struct max30102_config *config = dev->config;

    if (i2c_reg_read_byte(data->i2c, config->i2c_addr,
                          MAX30102_REG_INT_STS2, status)) {
        LOG_ERR("Couldn't read INT_STS2");
        return -EIO;
    }

    LOG_INF("STS2 status %d", *status);
    return 0;
}

/* TODO check for ALC overflow */
static int max30102_sample_fetch(const struct device *dev, enum sensor_channel chan) {
    struct max30102_data *data = dev->data;
    const struct max30102_config *config = dev->config;
    uint8_t fifo_size = 1; /* TODO get this value from FIFO_A_FULL */
    uint8_t buffer[MAX30102_MAX_BYTES_PER_SAMPLE * fifo_size];
    uint32_t fifo_data;
    int fifo_chan;
    int num_bytes;
    int i;

    /* TODO read number of sample according to FIFO_A_FULL value if in trigger mode */
    /* Read all the active channels for one 18-bit samples */
    num_bytes = data->num_channels * MAX30102_BYTES_PER_CHANNEL * fifo_size;
    if (i2c_burst_read(data->i2c, config->i2c_addr,
                       MAX30102_REG_FIFO_DATA, buffer, num_bytes)) {
        LOG_ERR("Could not fetch sample");
        return -EIO;
    }

    fifo_chan = 0;
    for (i = 0; i < num_bytes; i++) {
        // Each channel is 18-bits
        fifo_data = (buffer[i * MAX30102_BYTES_PER_CHANNEL] << 16) |
                    (buffer[(i * MAX30102_BYTES_PER_CHANNEL) + 1] << 8) |
                    (buffer[(i * MAX30102_BYTES_PER_CHANNEL) + 2]);
        fifo_data &= MAX30102_FIFO_DATA_MASK;

        /* Save the raw data per channel*/
        data->raw[fifo_chan++] = fifo_data;
    }

    /* Read Status 1 register to clear interrupt */
    /* TODO #ifdef CONFIG_MAX30102_INTERUPT */
    uint8_t status;
    if (i2c_reg_read_byte(data->i2c, config->i2c_addr,
                          MAX30102_REG_INT_STS1, &status)) {
        LOG_ERR("Couldn't read INT_STS1");
        return -EIO;
    }

    if (chan == SENSOR_CHAN_DIE_TEMP) {
        uint8_t tint = 0;
        uint8_t tfrac = 0;

        if (i2c_reg_read_byte(data->i2c, config->i2c_addr,
                              MAX30102_REG_TINT, &tint)) {
            LOG_ERR("Couldn't read TINT");
            return -EIO;
        }

        if (i2c_reg_read_byte(data->i2c, config->i2c_addr,
                              MAX30102_REG_TFRAC, &tfrac)) {
            LOG_ERR("Couldn't read TFRAC");
            return -EIO;
        }

        data->tint = tint;
        data->tfrac = tfrac;
    }
    return 0;
}

static int max30102_channel_get(const struct device *dev,
                                enum sensor_channel chan,
                                struct sensor_value *val) {
    struct max30102_data *data = dev->data;
    enum max30102_led_channel led_chan;
    int fifo_chan;

    switch (chan) {
        case SENSOR_CHAN_RED:
            led_chan = MAX30102_LED_CHANNEL_RED;
            /* Check if the led channel is active by looking up the associated fifo
             * channel. If the fifo channel isn't valid, then the led channel
             * isn't active.
             */
            fifo_chan = data->map[led_chan];
            if (fifo_chan >= MAX30102_MAX_NUM_CHANNELS) {
                LOG_ERR("Inactive sensor channel");
                return -ENOTSUP;
            }

            val->val1 = data->raw[fifo_chan];
            val->val2 = 0;
            break;

        case SENSOR_CHAN_IR:
            led_chan = MAX30102_LED_CHANNEL_IR;
            /* Check if the led channel is active by looking up the associated fifo
             * channel. If the fifo channel isn't valid, then the led channel
             * isn't active.
             */
            fifo_chan = data->map[led_chan];
            if (fifo_chan >= MAX30102_MAX_NUM_CHANNELS) {
                LOG_ERR("Inactive sensor channel");
                return -ENOTSUP;
            }

            val->val1 = data->raw[fifo_chan];
            val->val2 = 0;
            break;

        case SENSOR_CHAN_DIE_TEMP:
            val->val1 = data->tint;
            val->val2 = (data->tfrac * 62500); /* 0.0625 * 10^6 */
            break;

        default:
            LOG_ERR("Unsupported sensor channel");
            return -ENOTSUP;
    }

    return 0;
}

static const struct sensor_driver_api max30102_driver_api = {
    .sample_fetch = max30102_sample_fetch,
    .channel_get = max30102_channel_get,
    .trigger_set = max30102_trigger_set,
};

static int max30102_init(const struct device *dev) {
    const struct max30102_config *config = dev->config;
    struct max30102_data *data = dev->data;
    uint8_t part_id;
    uint8_t mode_cfg;
    uint32_t led_chan;
    int fifo_chan;

    /* Get the I2C device */
    data->i2c = device_get_binding(config->i2c_label);
    if (!data->i2c) {
        LOG_ERR("Could not find I2C device");
        return -EINVAL;
    }
    /* Check the part id to make sure this is MAX30102 */
    if (i2c_reg_read_byte(data->i2c, config->i2c_addr,
                          MAX30102_REG_PART_ID, &part_id)) {
        LOG_ERR("Could not get Part ID");
        return -EIO;
    }
    if (part_id != MAX30102_PART_ID) {
        LOG_ERR("Got Part ID 0x%02x, expected 0x%02x",
                part_id, MAX30102_PART_ID);
        return -EIO;
    }

    /* Reset the sensor */
    if (i2c_reg_write_byte(data->i2c, config->i2c_addr,
                           MAX30102_REG_MODE_CFG,
                           MAX30102_MODE_CFG_RESET_MASK)) {
        return -EIO;
    }

    /* Wait for reset to be cleared */
    do {
        if (i2c_reg_read_byte(data->i2c, config->i2c_addr,
                              MAX30102_REG_MODE_CFG, &mode_cfg)) {
            LOG_ERR("Could read mode cfg after reset");
            return -EIO;
        }
    } while (mode_cfg & MAX30102_MODE_CFG_RESET_MASK);

    /* Write the FIFO configuration register */
    if (i2c_reg_write_byte(data->i2c, config->i2c_addr,
                           MAX30102_REG_FIFO_CFG, config->fifo)) {
        return -EIO;
    }

    /* Write the mode configuration register */
    if (i2c_reg_write_byte(data->i2c, config->i2c_addr,
                           MAX30102_REG_MODE_CFG, config->mode)) {
        return -EIO;
    }

    /* Write the SpO2 configuration register */
    if (i2c_reg_write_byte(data->i2c, config->i2c_addr,
                           MAX30102_REG_SPO2_CFG, config->spo2)) {
        return -EIO;
    }

    /* Write the LED pulse amplitude registers */
    if (i2c_reg_write_byte(data->i2c, config->i2c_addr,
                           MAX30102_REG_LED1_PA, config->led_pa[0])) {
        return -EIO;
    }
    if (i2c_reg_write_byte(data->i2c, config->i2c_addr,
                           MAX30102_REG_LED2_PA, config->led_pa[1])) {
        return -EIO;
    }

#ifdef CONFIG_MAX30102_TRIGGER
    if (max30102_init_interrupt(dev)) {
        LOG_ERR("Failed to initalize interrupt");
        return -EIO;
    }
#endif

    /*
     * Write to the temperature config register
     * TEMP_EN bit is self-clearing
     */
    if (i2c_reg_write_byte(data->i2c, config->i2c_addr,
                           MAX30102_REG_TEMP_CFG, config->temp_cfg)) {
        return -EIO;
    }
    LOG_INF("Wrote to temperature config");

#ifdef CONFIG_MAX30102_MULTI_LED_MODE
    uint8_t multi_led[2];

    /* Write the multi-LED mode control registers */
    multi_led[0] = (config->slot[1] << 4) | (config->slot[0]);
    multi_led[1] = (config->slot[3] << 4) | (config->slot[2]);

    if (i2c_reg_write_byte(data->i2c, config->i2c_addr,
                           MAX30102_REG_MULTI_LED, multi_led[0])) {
        return -EIO;
    }
    if (i2c_reg_write_byte(data->i2c, config->i2c_addr,
                           MAX30102_REG_MULTI_LED + 1, multi_led[1])) {
        return -EIO;
    }
#endif

    /* Initialize the channel map and active channel count */
    data->num_channels = 0U;
    for (led_chan = 0U; led_chan < MAX30102_MAX_NUM_CHANNELS; led_chan++) {
        data->map[led_chan] = MAX30102_MAX_NUM_CHANNELS;
    }

    /* Count the number of active channels and build a map that translates
     * the LED channel number (red/ir) to the fifo channel number.
     */
    for (fifo_chan = 0; fifo_chan < MAX30102_MAX_NUM_CHANNELS; fifo_chan++) {
        led_chan = (config->slot[fifo_chan] & MAX30102_SLOT_LED_MASK) - 1;
        if (led_chan < MAX30102_MAX_NUM_CHANNELS) {
            data->map[led_chan] = fifo_chan;
            data->num_channels++;
        }
    }
#define MAX30102_INTB_PROPS(n)                                            \
    .intb_gpio = DEVICE_DT_GET(DT_GPIO_CTLR(DT_DRV_INST(n), intb_gpios)), \
    .intb_pin = DT_INST_GPIO_PIN(n, intb_gpios),                          \
    .intb_flags = DT_INST_GPIO_FLAGS(n, intb_gpios),

#define MAX30102_INTB(n)                \
    IF_ENABLED(CONFIG_MAX30102_TRIGGER, \
               (MAX30102_INTB_PROPS(n)))

    /* TODO:
    - Read temp from sensor - done
    - Calcuate temp increase from current settings
    - Send temp over ble
    */

    return 0;
}

#define MAX30102_DEFINE(inst)                                                    \
    static struct max30102_data max30102_data_##inst;                            \
                                                                                 \
    static struct max30102_config max30102_config_##inst = {                     \
        .i2c_label = DT_INST_BUS_LABEL(0),                                       \
        .i2c_addr = DT_INST_REG_ADDR(0),                                         \
        .fifo = DT_INST_PROP(inst, smp_ave) << MAX30102_FIFO_CFG_SMP_AVE_SHIFT | \
                COND_CODE_1(DT_INST_PROP(inst, fifo_rollover_en),                \
                            MAX30102_FIFO_CFG_ROLLOVER_EN_MASK |, ())            \
                        DT_INST_PROP(inst, fifo_a_full)                          \
                    << MAX30102_FIFO_CFG_FIFO_FULL_SHIFT,                        \
                                                                                 \
        .mode = DT_INST_PROP(inst, mode),                                        \
        .slot[0] = MAX30102_SLOT_RED_LED1_PA,                                    \
        .slot[1] = MAX30102_SLOT_IR_LED2_PA,                                     \
        .slot[2] = MAX30102_SLOT_DISABLED,                                       \
        .slot[3] = MAX30102_SLOT_DISABLED, /* TODO allow other modes */          \
                                                                                 \
        .spo2 = DT_INST_PROP(inst, adc_rge) << MAX30102_SPO2_ADC_RGE_SHIFT |     \
                DT_INST_PROP(inst, sr) << MAX30102_SPO2_SR_SHIFT |               \
                MAX30102_PW_18BITS << MAX30102_SPO2_PW_SHIFT,                    \
                                                                                 \
        .led_pa[0] = DT_INST_PROP(inst, led1_pa),                                \
        .led_pa[1] = DT_INST_PROP(inst, led2_pa),                                \
        MAX30102_INTB(inst)                                                      \
                                                                                 \
    };                                                                           \
                                                                                 \
    DEVICE_DT_INST_DEFINE(inst, max30102_init, NULL,                             \
                          &max30102_data_##inst, &max30102_config_##inst,        \
                          POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,              \
                          &max30102_driver_api);

/* Create the struct device for every status "okay" node in the devicetree. */
DT_INST_FOREACH_STATUS_OKAY(MAX30102_DEFINE)
