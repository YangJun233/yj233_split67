// Copyright 2024 yangjun
// SPDX-License-Identifier: GPL-2.0-or-later

#pragma once

#include_next <mcuconf.h>

// Enable I2C0 (GP0/GP1) for the Azoteq TPS43 touchpad.
#undef RP_I2C_USE_I2C0
#define RP_I2C_USE_I2C0 TRUE

#undef RP_I2C_USE_I2C1
#define RP_I2C_USE_I2C1 FALSE
