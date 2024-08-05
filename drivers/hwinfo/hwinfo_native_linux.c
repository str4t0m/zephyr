
/*
 * Copyright (c) 2024, Thomas Stranger
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/hwinfo.h>
#include <zephyr/sys/util.h>
#include <zephyr/types.h>

#include "hwinfo_native_linux_bottom.h"

/*
 * HWINFO driver for POSIX ARCH based boards.
 * It can only be used on linux
 *
 */

ssize_t z_impl_hwinfo_get_device_id(uint8_t *buffer, size_t length)
{
	uint8_t hex_id[HWINFO_NP_MACHINE_ID_LEN];

	if ((buffer == NULL) || (length == 0)) {
		return 0;
	}

	if (length > (HWINFO_NP_MACHINE_ID_LEN / 2)) {
		length = HWINFO_NP_MACHINE_ID_LEN / 2;
	}

	if (np_machine_id_get(hex_id) != 0) {
		return -EIO;
	}

	if (hex2bin(hex_id, length * 2, buffer, length) != length) {
		return -EINVAL;
	}

	return length;
}
