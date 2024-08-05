/*
 * Copyright (c) 2024, Thomas Stranger
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * API for the "Bottom" part of native linux hwinfo driver
 * Native simulator targets build this in the runner context, using
 * the host C library and host include paths.
 */

#ifndef DRIVERS_HWINFO_HWINFO_NATIVE_LINUX_BOTTOM_H
#define DRIVERS_HWINFO_HWINFO_NATIVE_LINUX_BOTTOM_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define HWINFO_NP_MACHINE_ID_LEN  32

/* native linux driver internal interface */
int np_machine_id_get(uint8_t *machine_id);

#ifdef __cplusplus
}
#endif

#endif /* DRIVERS_HWINFO_HWINFO_NATIVE_LINUX_BOTTOM_H */
