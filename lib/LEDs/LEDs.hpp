#ifndef LEDs_H
#define LEDs_H

#include <Arduino.h>
#include "Utils.hpp"
#include "../../include/pins.h"
#include "../../include/constants.h"

namespace LEDs
{
    void setup(boolean isDebugModeActive);
    void turnLEDOnOff(boolean state, uint8_t pin);

    static boolean isModuleDebugModeActive;
    static char* moduleName = "LEDs";
}

#endif // LEDs_H