/*
 * Copyright (c) 2022 EVERGREEN FUND 501(c)(3)
 * Copyright (c) 2022 Jacob Tinkhauser
 *
 * SPDX-License-Identifier: AGPL-3.0-or-later
 */

#define DT_DRV_COMPAT maxim_max30102

#include <drivers/gpio.h>
#include <drivers/sensor.h>
#include <kernel.h>
#include <logging/log.h>

#include "max30102.h"

LOG_MODULE_DECLARE(MAX30102, CONFIG_SENSOR_LOG_LEVEL);

static void max30102_thread_cb(const struct device *dev) {
    struct max30102_data *data = dev->data;

    /* Clear status */
    uint8_t status_1, status_2;
    int ret = max30102_get_interrupt_status_1(dev, &status_1);
    ret = max30102_get_interrupt_status_2(dev, &status_2);
    if (ret) {
        LOG_ERR("Unable to get statuses");
        return;
    }

    k_mutex_lock(&data->trigger_mutex, K_FOREVER);
    if (data->drdy_handler != NULL && MAX30102_INTR_STATUS_1_A_FULL(status_1)) {
        data->drdy_handler(dev, &data->drdy_trigger);
    }
    k_mutex_unlock(&data->trigger_mutex);
}

int max30102_trigger_set(const struct device *dev,
                         const struct sensor_trigger *trig,
                         sensor_trigger_handler_t handler) {
    struct max30102_data *data = dev->data;
    const struct max30102_config *cfg = dev->config;
    int ret = 0;

    gpio_pin_interrupt_configure(cfg->intb_gpio, cfg->intb_pin, GPIO_INT_DISABLE);

    /* TODO add handler for die temp */
    switch (trig->type) {
        case SENSOR_TRIG_DATA_READY:
            k_mutex_lock(&data->trigger_mutex, K_FOREVER);
            data->drdy_handler = handler;
            data->drdy_trigger = *trig;
            k_mutex_unlock(&data->trigger_mutex);

            // int_mask = FDC2X1X_ERROR_CONFIG_DRDY_2INT_MSK;
            break;
        default:
            LOG_ERR("Unsupported sensor trigger");
            ret = -ENOTSUP;
            goto out;
    }

    if (handler) {
        // int_en = int_mask;
        //  data->int_config |= int_mask;
    } else {
        // int_en = 0U;
    }

    /* Clear INTB pin by reading STATUS register */
    uint8_t status_1, status_2;
    max30102_get_interrupt_status_1(dev, &status_1);
    max30102_get_interrupt_status_2(dev, &status_2);
out:
    gpio_pin_interrupt_configure(cfg->intb_gpio, cfg->intb_pin,
                                 GPIO_INT_EDGE_FALLING);

    return ret;
}

static void max30102_gpio_callback(const struct device *port,
                                   struct gpio_callback *cb, uint32_t pin) {
    struct max30102_data *data =
        CONTAINER_OF(cb, struct max30102_data, gpio_cb);

    ARG_UNUSED(port);
    ARG_UNUSED(pin);

#ifdef CONFIG_MAX30102_TRIGGER_OWN_THREAD
    k_sem_give(&data->gpio_sem);
#elif CONFIG_MAX30102_TRIGGER_GLOBAL_THREAD
    k_work_submit(&data->work);
#endif
}

#ifdef CONFIG_MAX30102_TRIGGER_OWN_THREAD
static void max30102_thread(struct max30102_data *data) {
    while (1) {
        k_sem_take(&data->gpio_sem, K_FOREVER);
        max30102_thread_cb(data->dev);
    }
}

#elif CONFIG_MAX30102_TRIGGER_GLOBAL_THREAD
static void max30102_work_cb(struct k_work *work) {
    struct max30102_data *data = CONTAINER_OF(work, struct max30102_data, work);

    max30102_thread_cb(data->dev);
}
#endif

int max30102_init_interrupt(const struct device *dev) {
    struct max30102_data *data = dev->data;
    const struct max30102_config *cfg = dev->config;
    int ret;

    k_mutex_init(&data->trigger_mutex);

    /* setup data ready gpio interrupt */
    if (!device_is_ready(cfg->intb_gpio)) {
        LOG_ERR("Cannot get pointer to interrupt device");
        return -EINVAL;
    }

    /* Write to the interrupt enable register 1 */
    ret = i2c_reg_write_byte(data->i2c, cfg->i2c_addr,
                             MAX30102_REG_INT_EN1, MAX30102_INTR_1_A_FULL_EN | MAX30102_INTR_1_ALC_OVF_EN);
    LOG_INF("Wrote to interrupt enable 1");

    /* Write to the interrupt enable register 2 */
    ret = i2c_reg_write_byte(data->i2c, cfg->i2c_addr, MAX30102_REG_INT_EN2,
                             MAX30102_INTR_2_DIE_TEMP_RDY_EN);
    LOG_INF("Wrote to interrupt enable 2");

    gpio_pin_configure(cfg->intb_gpio, cfg->intb_pin, GPIO_INPUT | cfg->intb_flags);
    gpio_init_callback(&data->gpio_cb, max30102_gpio_callback, BIT(cfg->intb_pin));

    if (gpio_add_callback(cfg->intb_gpio, &data->gpio_cb)) {
        LOG_ERR("Failed to set gpio callback");
        return -EIO;
    }

    data->dev = dev;

#ifdef CONFIG_MAX30102_TRIGGER_OWN_THREAD
    k_sem_init(&data->gpio_sem, 0, K_SEM_MAX_LIMIT);

    k_thread_create(&data->thread, data->thread_stack,
                    CONFIG_MAX30102_THREAD_STACK_SIZE,
                    (k_thread_entry_t)max30102_thread,
                    data, NULL, NULL,
                    K_PRIO_COOP(CONFIG_MAX30102_THREAD_PRIORITY),
                    0, K_NO_WAIT);
#elif defined(CONFIG_MAX30102_TRIGGER_GLOBAL_THREAD)
    data->work.handler = max30102_work_cb;
#endif

    return 0;
}