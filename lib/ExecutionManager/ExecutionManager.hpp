
#ifndef EXECUTION_MANAGER_H
#define EXECUTION_MANAGER_H

#include <Arduino.h>
#include "../../include/pins.h"
#include "../../include/constants.h"

namespace ExecutionManager {
    void setDebugModeAll(boolean isDebugModeActive = false);

    static boolean DEBUG_BATTERY_MEASUREMENT = true;
    static boolean DEBUG_LEDs = true;
    static boolean DEBUG_I2CDRIVERS = true;
    static boolean DEBUG_ROBOT_MOVE = true;
    static boolean DEBUG_ESPNOW = true;
    static boolean DEBUG_IR_SENSOR = true;
    static boolean DEBUG_IMU_SENSOR = true;
}


#endif // EXECUTION_MANAGER_H