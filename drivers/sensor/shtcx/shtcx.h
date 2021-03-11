/*
 * Copyright (c) 2021 Thomas Stranger
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_SHTCX_SHTCX_H_
#define ZEPHYR_DRIVERS_SENSOR_SHTCX_SHTCX_H_

#include <device.h>
#include <devicetree.h>
#include <kernel.h>
#include <drivers/gpio.h>

/* common cmds */
#define SHTCX_CMD_READ_ID              0xEFC8
#define SHTCX_CMD_SOFT_RESET           0x805D
/* shtc3 only: */
#define SHTCX_CMD_SLEEP                0xB098
#define SHTCX_CMD_WAKEUP               0x3517

#define SHTCX_POWER_UP_TIME_US   240U
/* Soft reset time is 230us for shtc1 and 240us for shtc3 */
#define SHTCX_SOFT_RESET_TIME_US 240U

#if CONFIG_SHTCX_MODE_NORMAL
	#define SHTCX_MODE        0
#elif CONFIG_SHTCX_MODE_LOW_POWER
	#define SHTCX_MODE        1
#endif

#define SHTCX_MAX_READ_LEN 6
#define SHTCX_WORD_LEN 2
#define SHTCX_CRC8_LEN 1

#define SHTC3_ID_MASK    0x083F
#define SHTC3_ID_VALUE   0x0807
#define SHTC1_ID_MASK    0x083F
#define SHTC1_ID_VALUE   0x0007

#define CHIP_SHTC1 0
#define CHIP_SHTC3 1

#define TO_CHIP(inst) \
	CONCAT(CHIP_, DT_ENUM_UPPER_TOKEN(DT_DRV_INST(inst), chip))

enum shtcx_chip {
	SHTC1 = CHIP_SHTC1,
	SHTC3 = CHIP_SHTC3,
};

struct shtcx_sample {
	uint16_t temp;
	uint16_t humidity;
} __packet;

struct shtcx_config {
	const struct device *bus;
	uint8_t base_address;
	enum shtcx_chip chip;
	bool clock_stretching;
};

struct shtcx_data {
	struct shtcx_sample sample;
};

static inline uint8_t shtcx_i2c_address(const struct device *dev)
{
	const struct shtcx_config *dcp = dev->config;

	return dcp->base_address;
}

static inline const struct device *shtcx_i2c_bus(const struct device *dev)
{
	const struct shtcx_config *dcp = dev->config;

	return dcp->bus;
}

#endif /* ZEPHYR_DRIVERS_SENSOR_SHTCX_SHTCX_H_ */
