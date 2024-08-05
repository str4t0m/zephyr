/*
 * Copyright (c) 2024 Thomas Stranger
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "hwinfo_native_linux_bottom.h"

#include <stdio.h>
#include <stdbool.h>

#ifndef __linux
#error "This driver can only be built on Linux systems"
#endif

#define HWINFO_NP_DBUS_MACHINE_ID_FILE "/var/lib/dbus/machine-id"
#define HWINFO_NP_MACHINE_ID_FILE      "/etc/machine-id"

/**
 * @brief Gets the hexadecimal 32 character machine id
 *
 * @param machine_id  buffer where the id is stored
 *
 * @retval 0  On successful read of all 32 characters of the id
 * @retval -1 If the file coulnd't be opened
 * @retval -2 In case of errors during the read
 */
int np_machine_id_get(uint8_t *machine_id)
{
	FILE *id_file;
	int res;

	id_file = fopen(HWINFO_NP_DBUS_MACHINE_ID_FILE, "r");
	if (id_file == 0) {
		id_file = fopen(HWINFO_NP_MACHINE_ID_FILE, "r");
		if (id_file == 0) {
			return -1;
		}
	}

	res = fread(machine_id, 1, HWINFO_NP_MACHINE_ID_LEN, id_file);
	if (res != HWINFO_NP_MACHINE_ID_LEN) {
		(void)fclose(id_file);
		return -2;
	}

	(void)fclose(id_file);
	return 0;
}
