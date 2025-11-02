#ifndef UTILS_H
#define UTILS_H

#include <Arduino.h>
#include "../../include/pins.h"
#include "../../include/constants.h"

namespace Utils
{
    float getADCReadingsAverage(uint8_t pin, float valueVoltage);
    void printMessageSetupDebug(boolean isDebugModeActive, char* moduleName);
    void printMessageLoopDebug(boolean isDebugModeActive, char* moduleName, char* variables, float var1, float var2, float var3);
    void printMessageLoopDebug(boolean isDebugModeActive, char* moduleName, char* variables, float var1);
    void printMessageLoopDebug(boolean isDebugModeActive, char* moduleName, char* message);
    void printMessageLoopDebug(boolean isDebugModeActive, char* moduleName, uint8_t pin, char* message);
}

#endif // UTILS_H