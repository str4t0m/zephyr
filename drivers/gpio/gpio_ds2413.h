/*
 * Copyright (c) 2021 Thomas Stranger
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_GPIO_DS2413_H_
#define ZEPHYR_DRIVERS_GPIO_DS2413_H_

#include <device.h>
#include <devicetree.h>
#include <kernel.h>
#include <drivers/gpio.h>
#include <drivers/w1.h>

#define DS2413_CMD_PIO_WRITE   0x5A
#define DS2413_CMD_PIO_READ    0xF5

#define NOT_SUPPORTED_FLAGS (GPIO_PULL_DOWN | GPIO_PULL_UP | GPIO_VOLTAGE_MASK)

struct ds2413_pio_status {
	uint8_t pin_a_state:1;
	uint8_t pin_a_latch_state:1;
	uint8_t pin_b_state:1;
	uint8_t pin_b_latch_state:1;
	/* comlements to the pin states: */
	uint8_t npin_a_state:1;
	uint8_t npin_a_latch_state:1;
	uint8_t npin_b_state:1;
	uint8_t npin_b_latch_state:1;
};

struct ds2413_config {
	/* gpio_driver_data needs to be first */
	struct gpio_driver_config common;

	const struct device *bus;
	uint8_t family;
	bool overdrive_speed;
};

struct ds2413_data {
	/* gpio_driver_data needs to be first */
	struct gpio_driver_config data;

	struct w1_slave_config config;
	struct k_sem lock;

	struct {
		/* uint16_t iodir; */
		uint8_t gpio;
		/* uint16_t olat; */
	} reg_cache;
};

static inline const struct device *ds2413_bus(const struct device *dev)
{
	const struct ds2413_config *dcp = dev->config;

	return dcp->bus;
}

#endif /* ZEPHYR_DRIVERS_GPIO_DS2413_H_ */
