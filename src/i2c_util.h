#pragma once
#ifndef LIB_I2C_UTIL_H
#define LIB_I2C_UTIL_H

#include "Arduino.h"

namespace i2c_util {

void i2crecovery(uint8_t sdaPin, uint8_t sclPin);
}

#endif