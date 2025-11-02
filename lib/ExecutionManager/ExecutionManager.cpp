#include "ExecutionManager.hpp"

namespace ExecutionManager {

    void setDebugModeAll(boolean isDebugModeActive){
        if(isDebugModeActive){
            DEBUG_BATTERY_MEASUREMENT = true;
            DEBUG_LEDs = true;
            DEBUG_I2CDRIVERS = true;
            DEBUG_ROBOT_MOVE = true;
            DEBUG_ESPNOW = true;
            DEBUG_IR_SENSOR = true;
            DEBUG_IMU_SENSOR = true;
        }else{
            DEBUG_BATTERY_MEASUREMENT = false;
            DEBUG_LEDs = false;
            DEBUG_I2CDRIVERS = false;
            DEBUG_ROBOT_MOVE = false;
            DEBUG_ESPNOW = false;
            DEBUG_IR_SENSOR = false;
            DEBUG_IMU_SENSOR = false;
        }
    }

    void executeAll(boolean isExecuteAllActive){
        if(isExecuteAllActive){
            static boolean EXECUTE_I2CDRIVERS = true;
            static boolean EXECUTE_BATTERY_MEASUREMENT = true;
            static boolean EXECUTE_ROBOT_MOVE = true;
            static boolean EXECUTE_ESP_NOW = true;
            static boolean EXECUTE_IMU_SENSOR = true;
            static boolean EXECUTE_IR_SENSOR = true;
        }

    }

}
