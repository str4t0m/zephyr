# Copyright (c) 2019 Carlo Caione <ccaione@baylibre.com>
# SPDX-License-Identifier: Apache-2.0

set(EMU_PLATFORM qemu)
set(QEMU_ARCH aarch64)

set(QEMU_CPU_TYPE_${ARCH} cortex-a53)

if(CONFIG_ARMV8_A_NS)
set(QEMU_MACH virt,gic-version=3)
else()
set(QEMU_MACH virt,secure=on,gic-version=3)
endif()

set(QEMU_FLAGS_${ARCH}
  -cpu ${QEMU_CPU_TYPE_${ARCH}}
  -nographic
  -machine ${QEMU_MACH}
  )
board_set_debugger_ifnset(qemu)
