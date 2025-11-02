
#ifndef EXECUTION_MANAGER_H
#define EXECUTION_MANAGER_H

#include <Arduino.h>
#include "../../include/pins.h"
#include "../../include/constants.h"

namespace ExecutionManager {
    void setDebugModeAll(boolean isDebugModeActive = false);
    void executeAll(boolean isExecuteAllActive = false);

    static boolean DEBUG_BATTERY_MEASUREMENT = false;
    static boolean DEBUG_LEDs = false;
    static boolean DEBUG_I2CDRIVERS = false;
    static boolean DEBUG_ROBOT_MOVE = false;
    static boolean DEBUG_ESPNOW = false;
    static boolean DEBUG_IR_SENSOR = false;
    static boolean DEBUG_IMU_SENSOR = false;

    static boolean EXECUTE_I2CDRIVERS = false;
    static boolean EXECUTE_BATTERY_MEASUREMENT = false;
    static boolean EXECUTE_ROBOT_MOVE = false;
    static boolean EXECUTE_ESP_NOW = false;
    static boolean EXECUTE_IMU_SENSOR = false;
    static boolean EXECUTE_IR_SENSOR = false;

    static uint32_t SAMPLE_TIME_BATTERY_MEASUREMENT = 500; // ms
    static uint32_t SAMPLE_TIME_IR_SENSOR = 200; // ms
    static uint32_t SAMPLE_TIME_ESP_NOW = 500; // ms
    static uint32_t SAMPLE_TIME_IMU_SENSOR = 500; // ms
    static uint32_t SAMPLE_TIME_ROBOT_MOVE = 500; // ms
    static uint32_t SAMPLE_TIME_I2CDRIVERS = 500; // ms
}


#endif // EXECUTION_MANAGER_H