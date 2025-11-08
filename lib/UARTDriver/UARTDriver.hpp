#ifndef UARTDRIVER_H
#define UARTDRIVER_H

#include <Arduino.h>
#include <stdint-gcc.h>
#include <Wire.h>
#include "Utils.hpp"
#include "../../include/constants.h"
#include "../../include/pins.h"

namespace UARTDriver
{
    void setup(boolean isDebugModeActive);
    void sendInfoToDriver(float* wheelsVelocities);

    static HardwareSerial driver0Serial(1);
    static HardwareSerial driver1Serial(2);

    static boolean isModuleDebugModeActive;
    static char* moduleName = "UARTDriver";
}

#endif // UARTDRIVER_H