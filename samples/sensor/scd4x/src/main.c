/*
 * Copyright (c) 2021 Thomas Stranger
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/sensor.h>

#define SCD4X DT_INST(0, sensirion_scd4x)

#if DT_NODE_HAS_STATUS(SCD4X, okay)
#define SCD4X_LABEL DT_LABEL(SCD4X)
#else
#error Your devicetree has no enabled nodes with compatible "sensirion,scd4x"
#define SCD4X_LABEL "<none>"
#endif

#if CONFIG_SCD4X_MEASURE_PERIODIC_HPM
#define SAMPLING_INTERVAL_S 5
#define TEMP_OFFSET         3000
#elif CONFIG_SCD4X_MEASURE_PERIODIC_LPM
#define SAMPLING_INTERVAL_S 30
#define TEMP_OFFSET         3000
#elif CONFIG_SCD4X_MEASURE_SINGLE_SHOT
#define SAMPLING_INTERVAL_S 300
#define TEMP_OFFSET         1700
#else
#error
#endif

#if defined(CONFIG_APP_ENABLE_AUTO_SELF_CALIBRATION)
#define AUTO_SELF_CALIBRATION 1
#else
#define AUTO_SELF_CALIBRATION 0
#endif

#if defined(CONFIG_APP_CALIBRATE_ENVIRONMENT)
static int calibrate_environment(const struct device *dev)
{
	struct sensor_value temp_offset;
	struct sensor_value sensor_altitude;

	temp_offset.val1 = CONFIG_APP_TEMPERATURE_OFFSET / 1000;
	temp_offset.val2 = (CONFIG_APP_TEMPERATURE_OFFSET % 1000) * 1000U;
	if (sensor_attr_set(dev, SENSOR_CHAN_AMBIENT_TEMP, SENSOR_ATTR_OFFSET,
			&temp_offset) != 0) {
		printk("Couldn't set temperature offset\n");
		return -1;
	}
	sensor_attr_get(dev, SENSOR_CHAN_AMBIENT_TEMP, SENSOR_ATTR_OFFSET,
			&temp_offset);

	/*
	 * pressure compensation can be done either by defining the altitude
	 * or by providing the pressure value directly. Only the second one
	 * can be done continuously during a peridic measure mode.
	 */
	sensor_altitude.val1 = CONFIG_APP_ALTITUDE;
	sensor_altitude.val2 = 0;
	if (sensor_attr_set(dev, SENSOR_CHAN_ALTITUDE, SENSOR_ATTR_CALIB_TARGET,
			&sensor_altitude)) {
		printk("Couldn't set sensor sensor_altitude\n");
		return -1;
	}
	printk("Temperatureoffset and Altitude calibrated successfully\n");
	return 0;
}
#endif /* defined(CONFIG_APP_CALIBRATE_ENVIRONMENT) */

static int automatic_self_calibration_set(const struct device *dev, bool enabled)
{
	struct sensor_value asc_enabled;

	asc_enabled.val1 = 0;
	asc_enabled.val2 = enabled;
	if (sensor_attr_set(dev, SENSOR_CHAN_CO2, SENSOR_ATTR_CALIB_TARGET,
			&asc_enabled)) {
		printk("Couldn't set Automatic Self-calibration target\n");
		return -1;
	}
	printk("Automatic Self-calibration enabled=%d\n", enabled);
	return 0;
}

#if defined(CONFIG_APP_PERSIST_SETTINGS)
static int persist_settings(const struct device *dev)
{
	struct sensor_value unused_val;

	unused_val.val1 = 0;
	unused_val.val2 = 0;
	if (sensor_attr_set(dev, SENSOR_CHAN_ALL, SENSOR_ATTR_CONFIG_PERSIST,
			&unused_val)) {
		printk("Couldn't apply configuration permanently\n");
		return -1;
	}
	printk("Settings persisted successfully\n");
	return 0;
}
#endif /* defined(CONFIG_APP_PERSIST_SETTINGS) */

#if defined(CONFIG_APP_FORCE_CO2_CALIBRATION)
/* turns off periodic measure mode and keeps it disabled */
static int co2_calibration_force(const struct device *dev)
{
	struct sensor_value val;
	struct sensor_value co2_forced;
	int stabilization_time = CONFIG_APP_CO2_CALIBRATION_STABILIZATION_TIME;

	printk("Enabling high performance mode by setting short measure period\n");
	val.val1 = 0;
	val.val2 = 200 * 1000; /* 1/5s = 200mHz */
	if (sensor_attr_set(dev, SENSOR_CHAN_ALL, SENSOR_ATTR_SAMPLING_FREQUENCY,
			&val)) {
		printk("Couldn't set hpm mode\n");
		return -1;
	}

	printk("Stabilze measurement by keeping sensor in constant co2"
	       " concentration for %ds\n", stabilization_time);
	for (int i = 0; i < 10; ++i) {
		printk("%d%%...", i*10);
		k_sleep(K_SECONDS(stabilization_time/10));
	}
	printk("100%%\n\n");

	printk("Setting CO2 reference value to %d\n",
	       CONFIG_APP_CO2_REFERENCE_VALUE);
	co2_forced.val1 = CONFIG_APP_CO2_REFERENCE_VALUE;
	co2_forced.val2 = 0;
	if (sensor_attr_set(dev, SENSOR_CHAN_CO2, SENSOR_ATTR_CALIB_TARGET,
			&co2_forced)) {
		printk("Couldn't set sensor sensor_altitude\n");
		return -1;
	}

	printk("CO2 calibration complete\n\n");
	return 0;
}
#endif /* defined(CONFIG_APP_FORCE_CO2_CALIBRATION) */

/* #if defined(CONFIG_APP_CALIBRATE_ENVIRONMENT) */
static int periodic_measure_mode_disable(const struct device *dev)
{
	struct sensor_value val;

	val.val1 = 0;
	val.val2 = 0;

	if (sensor_attr_set(dev, SENSOR_CHAN_ALL,
			    SENSOR_ATTR_SAMPLING_FREQUENCY, &val)) {
		printk("Couldn't disable periodic measure mode\n");
		return -1;
	}
	printk("Periodic measure mode disabled\n");
	return 0;
}
/* #endif /1* defined(CONFIG_APP_CALIBRATE_ENVIRONMENT) *1/ */

static int measure_mode_restore_default(const struct device *dev)
{
	struct sensor_value val;

#if defined(CONFIG_SCD4X_MEASURE_SINGLE_SHOT)
	val.val1 = 0;
	val.val2 = 0;
#else
	val.val1 = 0;
	val.val2 = 1000000 / SAMPLING_INTERVAL_S;
#endif
	if (sensor_attr_set(dev, SENSOR_CHAN_ALL,
			    SENSOR_ATTR_SAMPLING_FREQUENCY, &val)) {
		printk("Couldn't restore the configured measure mode\n");
		return -1;
	}
	printk("Default measure mode restored\n");
	return 0;
}

static int print_sensor_attributes(const struct device *dev)
{
	struct sensor_value val;

	printk("\nCurrent SCD4x configuration:\n");

	if (sensor_attr_get(dev, SENSOR_CHAN_AMBIENT_TEMP,
			    SENSOR_ATTR_OFFSET, &val)) {
		printk("Couldn't read temp offset\n");
		return -1;
	}
	printk("Ambient temperature offset: % 6d.%06d\n", val.val1, val.val2);

	if (sensor_attr_get(dev, SENSOR_CHAN_ALTITUDE,
			    SENSOR_ATTR_CALIB_TARGET, &val)) {
		printk("Couldn't read current altitude\n");
		return -1;
	}
	printk("Sensor altitude:            % 6d.0\n", val.val1);

	if (sensor_attr_get(dev, SENSOR_CHAN_CO2,
			    SENSOR_ATTR_CALIB_TARGET, &val)) {
		printk("Couldn't read automatic self-calibration mode\n");
		return -1;
	}
	printk("Auto self calib enabled: %d\n\n", val.val2);
	return 0;
}

void main(void)
{
	const struct device *dev = device_get_binding(SCD4X_LABEL);

	if (dev == NULL) {
		printk("No device \"%s\" found; did initialization fail?\n",
		       SCD4X_LABEL);
		return;
	}
	printk("Found device \"%s\"\n", SCD4X_LABEL);

	periodic_measure_mode_disable(dev);
	print_sensor_attributes(dev);
#if defined(CONFIG_APP_CALIBRATE_ENVIRONMENT)
	calibrate_environment(dev);
#endif /* defined(CONFIG_APP_CALIBRATE_ENVIRONMENT)*/
	automatic_self_calibration_set(dev, AUTO_SELF_CALIBRATION);
#if defined(CONFIG_APP_PERSIST_SETTINGS)
	persist_settings(dev);
#endif /* defined(CONFIG_APP_PERSIST_SETTINGS) */
#if defined(CONFIG_APP_FORCE_CO2_CALIBRATION)
	co2_calibration_force(dev);
#endif
	measure_mode_restore_default(dev);
	k_sleep(K_SECONDS(7));

	while (1) {
		struct sensor_value co2, temp, humidity;

		sensor_sample_fetch(dev);
		sensor_channel_get(dev, SENSOR_CHAN_CO2, &co2);
		sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &temp);
		sensor_channel_get(dev, SENSOR_CHAN_HUMIDITY, &humidity);

		printk("CO2: %d; temp: %d.%06d; humidity: %d.%06d\n",
		      co2.val1, temp.val1, temp.val2,
		      humidity.val1, humidity.val2);

		k_sleep(K_SECONDS(SAMPLING_INTERVAL_S));
	}
}
