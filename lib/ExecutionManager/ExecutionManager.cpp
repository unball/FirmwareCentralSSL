#include "ExecutionManager.hpp"

namespace ExecutionManager {

    void setDebugModeAll(boolean isDebugModeActive = false){
        if(isDebugModeActive){
            DEBUG_BATTERY_MEASUREMENT = true;
            DEBUG_LEDs = true;
            DEBUG_I2CDRIVERS = true;
            DEBUG_ROBOT_MOVE = true;
            DEBUG_ESPNOW = true;
            DEBUG_IR_SENSOR = true;
            DEBUG_IMU_SENSOR = true;
        }
    }

}
