/*
 * Copyright (c) 2021 Thomas Stranger
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/sensor.h>
#include <sys/printk.h>

static const struct device *get_shtcx_device(void)
{
	const struct device *dev = DEVICE_DT_GET_ANY(sensirion_shtcx);

	if (dev == NULL) {
		/* No such node, or the node does not have status "okay". */
		printk("\nError: no device found.\n");
		return NULL;
	}

	if (!device_is_ready(dev)) {
		printk("\nError: Device \"%s\" is not ready; "
		       "check the driver initialization logs for errors.\n",
		       dev->name);
		return NULL;
	}

	printk("Found device \"%s\", getting sensor data\n", dev->name);
	return dev;
}

void main(void)
{
	const struct device *dev = get_shtcx_device();

	if (dev == NULL) {
		return;
	}

	struct sensor_value temp, humidity;

	while (1) {
		sensor_sample_fetch(dev);
		sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &temp);
		sensor_channel_get(dev, SENSOR_CHAN_HUMIDITY, &humidity);

		printk("Temp = %d.%06d C, RH = %d.%06d %%\n",
		       temp.val1, temp.val2, humidity.val1, humidity.val2);

		k_sleep(K_SECONDS(2));
	}
}
