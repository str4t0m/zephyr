/*
 * Copyright (c) 2021 Thomas Stranger
 *
 *SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT sensirion_scd4x

/* @file
 * @brief driver for scd4x c02 sensor
 */

#include <device.h>
#include <drivers/sensor.h>
#include <drivers/i2c.h>
#include <sys/__assert.h>
#include <sys/byteorder.h>
#include <init.h>
#include <kernel.h>
#include <string.h>
#include <logging/log.h>
#include <sys/__assert.h>

#include "scd4x.h"

LOG_MODULE_REGISTER(SCD4X, CONFIG_SENSOR_LOG_LEVEL);


/*
 * CRC algorithm parameters were taken from the
 * "Checksum Calculation" section of the datasheet.
 * reflect input/output: flase/false
 * final xor: false
 */
static uint8_t scd4x_compute_crc(uint16_t value)
{
	uint8_t buf[2] = { value >> 8, value & 0xFF };
	uint8_t crc = 0xFF;
	uint8_t polynom = 0x31;
	int i, j;

	for (i = 0; i < 2; ++i) {
		crc = crc ^ buf[i];
		for (j = 0; j < 8; ++j) {
			if (crc & 0x80) {
				crc = (crc << 1) ^ polynom;
			} else {
				crc = crc << 1;
			}
		}
	}

	return crc;
}

int scd4x_write_command(const struct device *dev, uint16_t cmd)
{
	uint8_t tx_buf[2];

	sys_put_be16(cmd, &tx_buf[0]);
	return i2c_write(scd4x_i2c_bus(dev), tx_buf, sizeof(tx_buf),
			 scd4x_i2c_address(dev));
}

int scd4x_write_reg(const struct device *dev, uint16_t cmd, uint16_t val)
{
	uint8_t tx_buf[5];

	sys_put_be16(cmd, &tx_buf[0]);
	sys_put_be16(val, &tx_buf[2]);
	tx_buf[4] = scd4x_compute_crc(val);

	return i2c_write(scd4x_i2c_bus(dev), tx_buf, sizeof(tx_buf),
			 scd4x_i2c_address(dev));
}

/*
 * Initiates the read, waits the required sensor execution time,
 * then reads the defined number of words into given destination.
 * The crc is already checked for each word, hence not returned.
 */
int scd4x_read_words(const struct device *dev, uint16_t cmd, uint16_t *data,
		     uint16_t num_words, uint16_t max_duration_ms)
{
	int status = 0;
	uint32_t raw_len = num_words * (SCD4X_WORD_LEN + SCD4X_CRC8_LEN);
	uint16_t temp16;
	uint8_t rx_buf[SCD4X_MAX_READ_LEN];
	int dst = 0;

	status = scd4x_write_command(dev, cmd);
	if (status != 0) {
		LOG_DBG("Failed to initiate read");
		return -EIO;
	}

	k_sleep(K_MSEC(max_duration_ms));

	status = i2c_read(scd4x_i2c_bus(dev), rx_buf, raw_len,
			  scd4x_i2c_address(dev));
	if (status != 0) {
		LOG_DBG("Failed to read data");
		return -EIO;
	}

	for (int i = 0; i < raw_len; i += (SCD4X_WORD_LEN + SCD4X_CRC8_LEN)) {
		temp16 = sys_get_be16(&rx_buf[i]);
		if (scd4x_compute_crc(temp16) != rx_buf[i+2]) {
			LOG_DBG("invalid received invalid crc");
			return -EIO;
		}

		data[dst++] = temp16;
	}
	return 0;
}

/* conversion according to: val = -45 + 175 * sample / (2^16) */
static void scd4x_temperature_from_raw(uint16_t raw, struct sensor_value *val)
{
	uint64_t tmp;

	tmp = (uint64_t)raw * 175U;
	val->val1 = (int32_t)(tmp >> 16) - 45;
	val->val2 = ((tmp % 0x10000) * 1000000U) >> 16;
}

/* TODO check, valid and size reduction */
/* conversion according to: raw = (val + 45) * 2^16 / 175 */
static uint16_t scd4x_temperature_to_raw(const struct sensor_value *val)
{
	uint64_t tmp;

	tmp = (uint64_t)val->val1 * 1000000U + val->val2;
	return (tmp << 16) / 175 / 1000000;

	tmp = ((uint64_t)val->val2 << 16) / 1000000U;
	tmp += (val->val1 + 45) << 16;
	tmp = tmp / 175U;
	return (uint16_t)tmp;
}

/* conversion according to: val = 100 * sample / (2^16) */
static void scd4x_humidity_from_raw(uint16_t raw, struct sensor_value *val)
{
	uint32_t tmp;

	tmp = (uint32_t)raw * 100U;
	val->val1 = tmp >> 16;
	/* x * 1.000.000 / 65.536 == x * 15.625 / 1.024 */
	val->val2 = (tmp % 0x10000) * 15625U >> 10;
}

static void scd4x_wakeup(const struct device *dev)
{
	scd4x_write_command(dev, SCD4X_CMD_WAKEUP);
	/* wakeup cmd is not acknowledged */
	k_sleep(K_MSEC(20));
}

static int scd4x_reinit(const struct device *dev)
{
	if (scd4x_write_command(dev, SCD4X_CMD_REINIT) < 0) {
		LOG_DBG("reinit fail");
		return -EIO;
	}
	k_sleep(K_MSEC(20));
	return 0;
}

static int scd4x_persist_settings(const struct device *dev)
{
	if (scd4x_write_command(dev, SCD4X_CMD_PERSIST_SETTINGS) < 0) {
		LOG_DBG("settings could not applied permanently");
		return -EIO;
	}
	k_sleep(K_MSEC(800));
	return 0;
}

static int scd4x_measure_mode_stop(const struct device *dev)
{
	struct scd4x_data *data = dev->data;

	if (data->measure_mode == SCD4X_MEASURE_ON_DEMAND) {
		return 0;
	}
	if (scd4x_write_command(dev, SCD4X_CMD_MEASURE_PERIODIC_STOP) < 0) {
		LOG_DBG("stopping periodic mode failed");
		return -EIO;
	}
	data->measure_mode = SCD4X_MEASURE_ON_DEMAND;
	k_sleep(K_MSEC(500));
	return 0;
}

/*
 * The sensor must be in idle mode(on-demand),
 * i.e. no periodic measurement mode executed, before a new mode can be started.
 */
static int scd4x_measure_mode_start(const struct device *dev,
				     enum scd4x_measure_mode mode)
{

	if (mode == SCD4X_MEASURE_ON_DEMAND) {
	} else if (mode == SCD4X_MEASURE_PERIODIC_HPM) {
		if (scd4x_write_command(dev, SCD4X_CMD_MEASURE_HPM_START)) {
			LOG_DBG("Failed to start periodic hpm!");
			return -EIO;
		}
	} else if (mode == SCD4X_MEASURE_PERIODIC_LPM) {
		if (scd4x_write_command(dev, SCD4X_CMD_MEASURE_LPM_START)) {
			LOG_DBG("Failed to start periodic lpm!");
			return -EIO;
		}
	} else {
		LOG_DBG("Unsupported Mode");
		return -ENOTSUP;
	}

	((struct scd4x_data *)dev->data)->measure_mode = mode;
	return 0;
}

#if defined(CONFIG_SCD4X_FORCED_CALIBRATION_SUPPORT)
/*
 * Forced calibration can only be executed after the sensor has operated
 * in periodic mode with a constant CO2 concentration for more that 3 min.
 * The periodic mode is stopped for the calibration and enabled again after
 * the process is finished.
 */
static int scd4x_force_recalibration(const struct device *dev,
				     const struct sensor_value *val)
{
	struct scd4x_data *data = dev->data;
	enum scd4x_measure_mode mode;
	uint16_t co2_target;
	int16_t co2_correction;
	int ret = 0;

	__ASSERT_NO_MSG(data->measure_mode != SCD4X_MEASURE_ON_DEMAND);

	mode = data->measure_mode;
	if (scd4x_measure_mode_stop(dev) < 0) {
		LOG_DBG("Couldn't stop periodic mode");
		ret = -EIO;
		goto out;
	}

	co2_target = (uint32_t)val->val1;
	if (scd4x_write_reg(dev, SCD4X_CMD_RECALIB_FORCED, co2_target) < 0) {
		LOG_DBG("writing forced recalibration failed");
		ret = -EIO;
		goto restore_mode;
	}
	k_sleep(K_MSEC(400));

	if (scd4x_read_words(dev, SCD4X_CMD_RECALIB_FORCED, &co2_correction, 1, 400) < 0) {
		LOG_DBG("Could not read frc correction value");
		ret = -EIO;
		goto restore_mode;
	}
	if (co2_correction == 0xffff) {
		LOG_DBG("frc failed");
		ret = -EIO;
		goto restore_mode;
	}
	co2_correction -= 0x8000;
	LOG_DBG("FRC of %d successfully applied", co2_correction);

restore_mode:
	if (scd4x_measure_mode_start(dev, mode) < 0) {
		LOG_DBG("Could not restore Measure Mode");
		ret = -EBUSY;
	}
out:
	return ret;
}
#endif /* defined(CONFIG_SCD4X_FORCED_CALIBRATION_SUPPORT) */

static int scd4x_sample_fetch(const struct device *dev,
				 enum sensor_channel chan)
{
	const struct scd4x_config *config = dev->config;
	struct scd4x_data *data = dev->data;
	uint16_t tmp16;

	if (chan != SENSOR_CHAN_ALL) {
		LOG_ERR("Unsupported sensor channel");
		return -ENOTSUP;
	}

	if (data->measure_mode == SCD4X_MEASURE_ON_DEMAND) {
		if (!config->single_shot_support) {
			return -ENOTSUP;
		}
		if (scd4x_write_command(dev, SCD4X_CMD_MEASURE_SINGLE_SHOT) < 0) {
			LOG_DBG("Failed to initiate single shot measurement");
			return -EIO;
		}
		k_sleep(K_MSEC(SCD4X_MEASURE_WAIT_SINGLE_SHOT_MS));
	} else {
		if (scd4x_read_words(dev, SCD4X_CMD_DATA_READY_GET, &tmp16, 1, 1) < 0) {
			LOG_DBG("Query data ready status failed");
			return -EIO;
		}
		if ((tmp16 & SCD4X_DATA_IS_READY_MASK) == 0) {
			LOG_DBG("No new data ready");
			return -EAGAIN;
		}
	}

	if (scd4x_read_words(dev, SCD4X_CMD_MEASUREMENT_READ,
			     (uint16_t *)&data->sample, 3, 1) < 0) {
		LOG_DBG("Failed to read data sample");
		return -EIO;
	}

	return 0;
}



static int scd4x_channel_get(const struct device *dev,
				enum sensor_channel chan,
				struct sensor_value *val)
{
	struct scd4x_data *data = dev->data;

	switch (chan) {
	case SENSOR_CHAN_CO2:
		val->val1 = data->sample.co2;
		val->val2 = 0;
		break;
	case SENSOR_CHAN_AMBIENT_TEMP:
		scd4x_temperature_from_raw(data->sample.temp, val);
		break;
	case SENSOR_CHAN_HUMIDITY:
		scd4x_humidity_from_raw(data->sample.humidity, val);
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

static int scd4x_attr_set(const struct device *dev,
		      enum sensor_channel chan,
		      enum sensor_attribute attr,
		      const struct sensor_value *val)
{
	struct scd4x_data *data = dev->data;
	uint16_t val16;
	int status;

	if (data->measure_mode != SCD4X_MEASURE_ON_DEMAND &&
	    (chan == SENSOR_CHAN_AMBIENT_TEMP || chan == SENSOR_CHAN_ALTITUDE ||
	     (chan == SENSOR_CHAN_ALL && attr == SENSOR_ATTR_CONFIG_PERSIST))) {
		LOG_DBG("Attr set not supported during periodic measure mode");
		return -EAGAIN;
	}

	switch (chan) {
	case SENSOR_CHAN_AMBIENT_TEMP:
		/* Temp offset for rh and t accuracy only, has no influence on
		 * c02.
		 * The offset depends on measurement mode, self-heating, ambient
		 * temp, air flow and the position of the sensor in the device.
		 */
		if (attr == SENSOR_ATTR_OFFSET) {
			val16 = scd4x_temperature_to_raw(val);
			status = scd4x_write_reg(dev, SCD4X_CMD_TEMP_OFFSET_SET,
						 val16);
		} else {
			status = -ENOTSUP;
		}
		break;
	case SENSOR_CHAN_PRESS:
		if (attr == SENSOR_ATTR_CALIB_TARGET) {
			val16 = val->val1 / 100;
			status = scd4x_write_reg(dev,
						 SCD4X_CMD_AMBIENT_PRESSURE_SET,
						 val16);
		} else {
			status = -ENOTSUP;
		}
		break;
	case SENSOR_CHAN_ALTITUDE:
		if (attr == SENSOR_ATTR_CALIB_TARGET) {
			val16 = (uint16_t)val->val1;
			status = scd4x_write_reg(dev,
						 SCD4X_CMD_SENSOR_ALTITUDE_SET,
						 val16);
		} else {
			status = -ENOTSUP;
		}
		break;
	case SENSOR_CHAN_CO2:
		/* forced recalibration */
		if (attr == SENSOR_ATTR_CALIB_TARGET) {
			/* special cases 0/0 and 0/1 disable/enable auto calibration */
			if (val->val1 == 0 && (val->val2 <= 1)) {
				status = scd4x_write_reg(dev,
							 SCD4X_CMD_CALIB_AUTO_SET,
							 (uint16_t)val->val2);
			} else {
#if defined(CONFIG_SCD4X_FORCED_CALIBRATION_SUPPORT)
				status = scd4x_force_recalibration(dev, val);
#else
				status = -ENOTSUP;
#endif /* defined(CONFIG_SCD4X_FORCED_CALIBRATION_SUPPORT) */
			}
		} else {
			status = -ENOTSUP;
		}
		break;
	case SENSOR_CHAN_ALL:
		/* the eeprom is rated to persist at least 2k write cycles */
		if (attr == SENSOR_ATTR_CONFIG_PERSIST) {
			status = scd4x_persist_settings(dev);
#if defined(CONFIG_SCD4X_MEASURE_MODE_RUNTIME_CHANGE)
		} else if (attr == SENSOR_ATTR_SAMPLING_FREQUENCY) {
			if (val->val1 < 0) {
				status = -ENOTSUP;
				goto out;
			}

			status = scd4x_measure_mode_stop(dev);
			if (val->val1 == 0 && val->val2 == 0) {
				if (scd4x_measure_mode_start(dev,
						  SCD4X_MEASURE_ON_DEMAND)) {
					status = -EAGAIN;
				}
			} else if (val->val1 > 0 || val->val2 > 33333) {
				if (scd4x_measure_mode_start(dev,
						  SCD4X_MEASURE_PERIODIC_HPM)) {
					status = -EAGAIN;
				}
			} else {
				if (scd4x_measure_mode_start(dev,
						  SCD4X_MEASURE_PERIODIC_LPM)) {
					status = -EAGAIN;
				}
			}
#endif /* defined(CONFIG_SCD4X_MEASURE_MODE_RUNTIME_CHANGE) */
		} else {
			status = -ENOTSUP;
		}
		break;
	default:
		LOG_ERR("Channel not supported: %d", chan);
		status = -ENOTSUP;
		goto out; /* avoid compile warning out not used */
	}

out:
	return status;
}

static int scd4x_attr_get(const struct device *dev,
			     enum sensor_channel chan,
			     enum sensor_attribute attr,
			     struct sensor_value *val)
{
	const struct scd4x_data *data = dev->data;
	int status = 0;
	uint16_t val16;

	__ASSERT_NO_MSG(val != NULL);
	if (data->measure_mode != SCD4X_MEASURE_ON_DEMAND &&
	    (chan == SENSOR_CHAN_AMBIENT_TEMP || chan == SENSOR_CHAN_ALTITUDE)) {
		LOG_DBG("Attr get not supported during periodic measure mode");
		return -EAGAIN;
	}

	switch (chan) {
	case SENSOR_CHAN_AMBIENT_TEMP:
		if (attr == SENSOR_ATTR_OFFSET) {
			status = scd4x_read_words(dev, SCD4X_CMD_TEMP_OFFSET_GET,
						 &val16, 1, 1);
			scd4x_temperature_from_raw(val16, val);
		} else {
			status = -ENOTSUP;
		}
		break;
	case SENSOR_CHAN_ALTITUDE:
		if (attr == SENSOR_ATTR_CALIB_TARGET) {
			status = scd4x_read_words(dev,
						 SCD4X_CMD_SENSOR_ALTITUDE_GET,
						 &val16, 1, 1);
			val->val1 = val16;
			val->val2 = 0;
		} else {
			status = -ENOTSUP;
		}
		break;
	case SENSOR_CHAN_CO2:
		if (attr == SENSOR_ATTR_CALIB_TARGET) {
			status = scd4x_read_words(dev, SCD4X_CMD_CALIB_AUTO_GET,
						 &val16, 1, 1);
			if (val16 == 1) {
				/* Automatic self calibration enabled */
				val->val1 = 0;
				val->val2 = 1;
			} else {
				/* Automatic self calibration disabled */
				val->val1 = 0;
				val->val2 = 0;
			}
		} else {
			status = -ENOTSUP;
		}
		break;
#if defined(CONFIG_SCD4X_MEASURE_MODE_RUNTIME_CHANGE)
	case SENSOR_CHAN_ALL:
		if (attr == SENSOR_ATTR_SAMPLING_FREQUENCY) {
			val->val1 = 0;
			if (data->measure_mode == SCD4X_MEASURE_ON_DEMAND) {
				val->val2 = 0;
			} else if (data->measure_mode == SCD4X_MEASURE_PERIODIC_HPM) {
				val->val2 = 200000; /* 1/5s */
			} else if (data->measure_mode == SCD4X_MEASURE_PERIODIC_LPM) {
				val->val2 = 333333; /* 1/30s */
			} else {
				status = -EIO;
			}
		} else {
			status = -ENOTSUP;
		}
		break;
#endif /* defined(CONFIG_SCD4X_MEASURE_MODE_RUNTIME_CHANGE) */
	default:
		LOG_ERR("Not supported");
		return -EINVAL;
	}

	return status;
}

static int scd4x_init(const struct device *dev)
{
	const struct scd4x_config *config = dev->config;
	struct scd4x_data *data = dev->data;
	uint16_t serial[3];
#if defined(CONFIG_SCD4X_SELFTEST_AT_INIT)
	uint16_t selftest_result = 0;
#endif

	/* Initialize time 1000ms after sensor has vdd_min=2.25V */
	k_sleep(K_MSEC(1000));

	if (device_is_ready(config->bus) == 0) {
		LOG_DBG("I2C bus not ready");
		return -ENODEV;
	}

	data->sample.co2 = 0U;
	data->sample.temp = 0U;
	data->sample.humidity = 0U;
	data->measure_mode = SCD4X_MEASURE_PERIODIC_HPM;

	scd4x_wakeup(dev);
	scd4x_measure_mode_stop(dev);
	scd4x_reinit(dev);

	if (scd4x_read_words(dev, SCD4X_CMD_SERIAL_GET, &serial[0], 3, 1)) {
		LOG_ERR("Failed reading chip serial number");
		return -EIO;
	}
	LOG_DBG("Serial is: 0x%04x%04x%04x", serial[0], serial[1], serial[2]);

#if defined(CONFIG_SCD4X_FACTORY_RESET_AT_INIT)
	if (scd4x_write_command(dev, SCD4X_CMD_FACTORY_RST) < 0) {
		LOG_DBG("Factory reset could not be executed");
		return -EIO;
	}
	LOG_DBG("Sensor parameters reset to factory default values");
	k_sleep(K_MSEC(1200));
#endif
#if defined(CONFIG_SCD4X_SELFTEST_AT_INIT)
	if (scd4x_read_words(dev, SCD4X_CMD_SELFTEST, &selftest_result, 1, 5500)
		< 0) {
		LOG_DBG("selftest result could not be read");
		return -EIO;
	}
	if (selftest_result != 0) {
		LOG_DBG("Selftest detected sensor malfunction");
	} else {
		LOG_DBG("Sensor Selftest OK");
	}
#endif

	if (scd4x_measure_mode_start(dev, INITIAL_MEASURE_MODE) < 0) {
		LOG_ERR("Failed to init measure mode");
		return -EIO;
	}

	LOG_DBG("SCD4x initialized");
	return 0;
}

static const struct sensor_driver_api scd4x_driver_api = {
	.sample_fetch = scd4x_sample_fetch,
	.channel_get = scd4x_channel_get,
	.attr_set = scd4x_attr_set,
	.attr_get = scd4x_attr_get,
};

#define SCD4X_CONFIG(inst)						  \
	{								  \
		.bus = DEVICE_DT_GET(DT_INST_BUS(inst)),		  \
		.i2c_address = DT_INST_REG_ADDR(inst),			  \
		.single_shot_support = DT_INST_PROP(inst,		  \
						    single_shot_support), \
	}

#define SCD4X_DEFINE(inst)						\
	static struct scd4x_data scd4x_data_##inst;			\
	static const struct scd4x_config scd4x_config_##inst =		\
		SCD4X_CONFIG(inst);					\
	DEVICE_DT_INST_DEFINE(inst,					\
			      scd4x_init,				\
			      device_pm_control_nop,			\
			      &scd4x_data_##inst,			\
			      &scd4x_config_##inst,			\
			      POST_KERNEL,				\
			      CONFIG_SENSOR_INIT_PRIORITY,		\
			      &scd4x_driver_api);

DT_INST_FOREACH_STATUS_OKAY(SCD4X_DEFINE)
