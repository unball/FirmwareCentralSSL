#ifndef BATTERY_MEASUREMENT_H
#define BATTERY_MEASUREMENT_H

#include <Arduino.h>
#include <Wire.h>
#include "../../include/pins.h"
#include "../../include/constants.h"
#include "Utils.hpp"
#include "LEDs.hpp"


namespace BatteryMeasurement
{
    void setup(boolean isDebugModeActive);
    float getBatteryLevelCharge();
    void alertLowBattery(float batteryLevelRead);

    static boolean isModuleDebugModeActive;
    static char* moduleName = "BatteryMeasurement";
}

#endif // BATTERY_MEASUREMENT_H