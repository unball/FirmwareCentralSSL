#ifndef I2CDRIVER_H
#define I2CDRIVER_H

#include <Arduino.h>
#include <stdint-gcc.h>
#include <Wire.h>
#include "Utils.hpp"
#include "../../include/constants.h"
#include "../../include/pins.h"

namespace I2CDriver
{
    void setup(boolean isDebugModeActive);
    void sendInfoToDriver(uint8_t driverNumber, float* wheelsVelocities);

    static boolean isModuleDebugModeActive;
    static char* moduleName = "I2CDriver";
}

#endif // I2CDRIVER_H