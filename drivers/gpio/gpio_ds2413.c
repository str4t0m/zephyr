/*
 * Copyright (c) 2022 Thomas Stranger
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * Driver for DS2413 1-Wire Dual channel open drain switch.
 * A datasheet is available at:
 * https://datasheets.maximintegrated.com/en/ds/DS2413.pdf
 */
#define DT_DRV_COMPAT maxim_ds2413

#include <device.h>
#include <drivers/w1.h>
#include <drivers/gpio.h>
#include <kernel.h>
#include <sys/__assert.h>
#include <logging/log.h>

#include "gpio_ds2413.h"

LOG_MODULE_REGISTER(DS2413, CONFIG_GPIO_LOG_LEVEL);

static int ds2413_port_get_raw(const struct device *dev, uint32_t *value)
{
	struct ds2413_data *data = dev->data;
	const struct device *bus = ds2413_bus(dev);
	uint8_t write_buf = DS2413_CMD_PIO_READ;
	uint8_t tmp = 0;
	struct ds2413_pio_status *tmp_ptr = (struct ds2413_pio_status *)&tmp;
	int tries = 3;
	int ret = 0;

	/* Can't do bus operations from an ISR */
	if (k_is_in_isr()) {
		return -EWOULDBLOCK;
	}

	k_sem_take(&data->lock, K_FOREVER);

	do {
		ret = w1_write_read(bus, &data->config,  &write_buf, 1, &tmp, 1);
		if (ret != 0) {
			continue;
		}

		if ((tmp & 0xF0) != ((~tmp << 4) & 0xF0)) {
			/* complement mismatch */
			ret = -EIO;
		} else {
			*value = tmp_ptr->pin_a_state | (tmp_ptr->pin_b_state << 1);
			LOG_DBG("Pin A: %u/%u  Pin B: %u/%u  (state/latch)",
				(uint8_t)tmp_ptr->pin_a_state, (uint8_t)tmp_ptr->pin_a_latch_state,
				(uint8_t)tmp_ptr->pin_b_state, (uint8_t)tmp_ptr->pin_b_latch_state);
		}
	} while (ret != 0 && --tries > 0);

	w1_reset_bus(bus);

	k_sem_give(&data->lock);
	return ret;
}

static int ds2413_port_set_masked_raw_not_locked(const struct device *dev,
				      uint32_t mask, uint32_t value)
{
	struct ds2413_data *data = dev->data;
	const struct device *bus = ds2413_bus(dev);
	uint8_t write_buf[3] = {DS2413_CMD_PIO_WRITE, 0, 0};
	uint8_t read_buf[2];
	struct ds2413_pio_status *rsp = (struct ds2413_pio_status *)&read_buf[1];
	uint8_t state;
	int tries = 3;
	int ret = 0;

	state = data->reg_cache.gpio;
	state = (state & ~mask) | (mask & value);
	write_buf[1] = state;
	write_buf[2] = ~state;

	do {
		w1_write_read(bus, &data->config, write_buf, 3, read_buf, 2);
		if (read_buf[0] != 0xAA) {
			LOG_WRN("write failed, no confirmation byte");
			ret = -EIO;
			continue;
		}

		if (((bool)(state & 0x01) != rsp->pin_a_latch_state) ||
		    ((bool)(state & 0x02) != rsp->pin_b_latch_state)) {
			LOG_DBG("state not applied: %x", state);
			/* LOG_DBG("state state is: %u/%u", */
			/*	rsp->pin_a_latch_state, rsp->pin_b_latch_state); */
			ret = -EIO;
		} else {
			LOG_DBG("pin state: %x", state);
			data->reg_cache.gpio = state;
		}
	} while (ret != 0 && --tries > 0);

	w1_reset_bus(bus);
	return ret;
}

static int ds2413_port_set_masked_raw(const struct device *dev,
				      uint32_t mask, uint32_t value)
{
	struct ds2413_data *data = dev->data;
	int ret;

	/* Can't do bus operations from an ISR */
	if (k_is_in_isr()) {
		return -EWOULDBLOCK;
	}

	k_sem_take(&data->lock, K_FOREVER);
	ret = ds2413_port_set_masked_raw_not_locked(dev, mask, value);
	k_sem_give(&data->lock);
	return ret;
}

static int ds2413_port_set_bits_raw(const struct device *dev, uint32_t mask)
{
	return ds2413_port_set_masked_raw(dev, mask, mask);
}

static int ds2413_port_clear_bits_raw(const struct device *dev,
					uint32_t mask)
{
	return ds2413_port_set_masked_raw(dev, mask, 0);
}

static int ds2413_port_toggle_bits(const struct device *dev, uint32_t mask)
{

	struct ds2413_data *data = dev->data;
	uint8_t value;
	int ret = 0;

	/* Can't do bus operations from an ISR */
	if (k_is_in_isr()) {
		return -EWOULDBLOCK;
	}

	k_sem_take(&data->lock, K_FOREVER);

	value = data->reg_cache.gpio;
	value ^= mask;

	ret = ds2413_port_set_masked_raw_not_locked(dev, mask, value);

	k_sem_give(&data->lock);
	return ret;
}

static int ds2413_config(const struct device *dev,
				gpio_pin_t pin, gpio_flags_t flags)
{
	if ((flags & NOT_SUPPORTED_FLAGS) != 0) {
		return -ENOTSUP;
	}
	if ((flags & GPIO_OPEN_DRAIN) != GPIO_OPEN_DRAIN) {
		return -ENOTSUP;
	}

	/* Can't do bus operations from an ISR */
	if (k_is_in_isr()) {
		return -EWOULDBLOCK;
	}

	if (flags & GPIO_OUTPUT) {
		if ((flags & GPIO_OUTPUT_INIT_HIGH)) {
			return ds2413_port_set_bits_raw(dev, BIT(pin));
		} else if ((flags & GPIO_OUTPUT_INIT_LOW)) {
			return ds2413_port_clear_bits_raw(dev, BIT(pin));
		}
	}

	return 0;
}

static int ds2413_pin_interrupt_configure(const struct device *dev,
					    gpio_pin_t pin,
					    enum gpio_int_mode mode,
					    enum gpio_int_trig trig)
{
	return -ENOTSUP;
}

static const struct gpio_driver_api api_table = {
	.pin_configure = ds2413_config,
	.port_get_raw = ds2413_port_get_raw,
	.port_set_masked_raw = ds2413_port_set_masked_raw,
	.port_set_bits_raw = ds2413_port_set_bits_raw,
	.port_clear_bits_raw = ds2413_port_clear_bits_raw,
	.port_toggle_bits = ds2413_port_toggle_bits,
	.pin_interrupt_configure = ds2413_pin_interrupt_configure,
};

/* TODO init rom and introduce method to allow updates */
static int ds2413_init(const struct device *dev)
{
	const struct ds2413_config *cfg = dev->config;
	struct ds2413_data *data = dev->data;

	if (device_is_ready(cfg->bus) ==  0) {
		LOG_DBG("w1 bus is not ready");
		return -ENODEV;
	}

	/* set most significat bits to 1. */
	data->reg_cache.gpio = 0xFC;

	w1_uint64_to_rom(0ULL, &data->config.rom);
	data->config.overdrive = cfg->overdrive_speed;
	k_sem_init(&data->lock, 1, 1);

	return 0;
}

/* TODO make family code optional? */
#define DS2413_CONFIG_INIT(inst)					\
	{								\
		.common = {						\
			.port_pin_mask = 0b11				\
		},							\
		.bus = DEVICE_DT_GET(DT_INST_BUS(inst)),		\
		.family = (uint8_t)DT_INST_PROP(inst, family_code),	\
		.overdrive_speed = DT_INST_PROP(inst, overdrive_speed)	\
	}

#define DS2413_DEFINE(inst)						\
	static struct ds2413_data ds2413_data_##inst;			\
	static const struct ds2413_config ds2413_config_##inst =	\
		DS2413_CONFIG_INIT(inst);				\
	DEVICE_DT_INST_DEFINE(inst,					\
			      ds2413_init,				\
			      NULL,					\
			      &ds2413_data_##inst,			\
			      &ds2413_config_##inst,			\
			      POST_KERNEL,				\
			      CONFIG_GPIO_DS2413_INIT_PRIORITY,		\
			      &api_table);

DT_INST_FOREACH_STATUS_OKAY(DS2413_DEFINE)
