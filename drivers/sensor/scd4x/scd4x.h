/*
 * Copyright (c) 2021 Thomas Stranger
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_SCD4X_SCD4X_H_
#define ZEPHYR_DRIVERS_SENSOR_SCD4X_SCD4X_H_

#include <device.h>
#include <kernel.h>
#include <drivers/gpio.h>

/* Basic commands */
#define SCD4X_CMD_MEASURE_HPM_START      0x21B1
#define SCD4X_CMD_MEASUREMENT_READ       0xEC05
#define SCD4X_CMD_MEASURE_PERIODIC_STOP  0x3F86
/* on-chip output signal compensation */
#define SCD4X_CMD_TEMP_OFFSET_SET	 0x241D
#define SCD4X_CMD_TEMP_OFFSET_GET        0x2318
#define SCD4X_CMD_SENSOR_ALTITUDE_SET    0x2427
#define SCD4X_CMD_SENSOR_ALTITUDE_GET    0x2322
#define SCD4X_CMD_AMBIENT_PRESSURE_SET   0xE000
/* field calibration */
#define SCD4X_CMD_RECALIB_FORCED         0x362F
#define SCD4X_CMD_CALIB_AUTO_SET         0x2416
#define SCD4X_CMD_CALIB_AUTO_GET         0x2313
/* low power */
#define SCD4X_CMD_MEASURE_LPM_START      0x21AC
#define SCD4X_CMD_DATA_READY_GET         0xE4B8
/* advanced features */
#define SCD4X_CMD_PERSIST_SETTINGS       0x3615
#define SCD4X_CMD_SERIAL_GET             0x3682
#define SCD4X_CMD_SELFTEST               0x3639
#define SCD4X_CMD_FACTORY_RST            0x3632
#define SCD4X_CMD_REINIT                 0x3646
/* low power single shot */
#define SCD4X_CMD_MEASURE_SINGLE_SHOT    0x219D
#define SCD4X_CMD_MEASURE_SINGLE_SHOT_RHT_ONLY 0x2196
/* cmds not listed in data sheet */
#define SCD4X_CMD_WAKEUP                 0x36F6
#define SCD4X_CMD_POWERDOWN              0x36e0

#define SCD4X_POWERUP_TIME_MS    1000
#define SCD4X_SOFT_RESET_TIME_MS 1000
#define SCD4X_MEASURE_WAIT_SINGLE_SHOT_MS 1350

#define SCD4X_MAX_READ_LEN 9
#define SCD4X_WORD_LEN 2
#define SCD4X_CRC8_LEN 1

/* data is not ready if last 11-bits are all 0 */
#define SCD4X_DATA_IS_READY_MASK 0x7FF

enum scd4x_measure_mode {
	SCD4X_MEASURE_PERIODIC_HPM,
	SCD4X_MEASURE_PERIODIC_LPM,
	/* only scd41: */
	SCD4X_MEASURE_ON_DEMAND,
};

#if defined(CONFIG_SCD4X_MEASURE_PERIODIC_HPM)
	#define INITIAL_MEASURE_MODE SCD4X_MEASURE_PERIODIC_HPM
#elif defined(CONFIG_SCD4X_MEASURE_PERIODIC_LPM)
	#define INITIAL_MEASURE_MODE SCD4X_MEASURE_PERIODIC_LPM
#elif defined(CONFIG_SCD4X_MEASURE_SINGLE_SHOT)
	#define INITIAL_MEASURE_MODE SCD4X_MEASURE_ON_DEMAND
#endif

struct scd4x_sample {
	uint16_t co2;
	uint16_t temp;
	uint16_t humidity;
} __packed;

struct scd4x_config {
	const struct device *bus;
	const uint8_t i2c_address;
	const bool single_shot_support;
};

struct scd4x_data {
	struct scd4x_sample sample;
	enum scd4x_measure_mode measure_mode;
};

static inline uint8_t scd4x_i2c_address(const struct device *dev)
{
	const struct scd4x_config *dcp = dev->config;

	return dcp->i2c_address;
}

static inline const struct device *scd4x_i2c_bus(const struct device *dev)
{
	const struct scd4x_config *dcp = dev->config;

	return dcp->bus;
}

int scd4x_write_command(const struct device *bus, uint16_t cmd);

int scd4x_write_reg(const struct device *dev, uint16_t cmd, uint16_t val);

int scd4x_read_words(const struct device *dev, uint16_t cmd, uint16_t *data,
		     uint16_t num_words, uint16_t max_duration_ms);

static int scd4x_attr_set(const struct device *dev,
		      enum sensor_channel chan,
		      enum sensor_attribute attr,
		      const struct sensor_value *val);

static int scd4x_attr_get(const struct device *dev,
			     enum sensor_channel chan,
			     enum sensor_attribute attr,
			     struct sensor_value *val);

#endif /* ZEPHYR_DRIVERS_SENSOR_SCD4X_SCD4X_H_ */
