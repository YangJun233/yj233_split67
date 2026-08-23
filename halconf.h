// Copyright 2024 yangjun
// SPDX-License-Identifier: GPL-2.0-or-later

#pragma once

// Pull in the ChibiOS base halconf first (defines _CHIBIOS_HAL_CONF_, the version
// guard, and all the HAL_USE_* defaults), THEN override. Same order as mcuconf.h.
#include_next <halconf.h>

// Enable the I2C HAL for the Azoteq TPS43 touchpad.
#undef HAL_USE_I2C
#define HAL_USE_I2C TRUE
