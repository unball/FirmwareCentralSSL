#include "BatteryMeasurement.hpp"

namespace BatteryMeasurement
{
    void setIsModuleDebugModeActive(boolean isDebugModeActive){
        isModuleDebugModeActive = isDebugModeActive;
    }

    void setup(boolean isDebugModeActive)
    {
        setIsModuleDebugModeActive(isDebugModeActive);
        
        pinMode(pins::BATTERY_MEASUREMENT, INPUT);

        Utils::printMessageSetupDebug(isModuleDebugModeActive, moduleName);
    }

    float getBatteryLevelCharge(){
        float batteryVoltageValue = Utils::getADCReadingsAverage(pins::BATTERY_MEASUREMENT, constants::MEASUREMENT_BATTERY_MAX_VALUE);
        
        Utils::printMessageLoopDebug(isModuleDebugModeActive, moduleName, "battery voltage value", batteryVoltageValue);

        return batteryVoltageValue;
    }

    void alertLowBattery(float batteryLevelRead){

        float percentage = (batteryLevelRead/constants::MEASUREMENT_BATTERY_MAX_VALUE) * 100;

        Utils::printMessageLoopDebug(isModuleDebugModeActive, moduleName, "battery voltage percentage", percentage);

        if(percentage <= 98.5){
            LEDs::turnLEDOnOff(true,pins::LED_RGB_RED);
            Utils::printMessageLoopDebug(isModuleDebugModeActive, moduleName,"Red LED should be turned on.");
        }else{
            LEDs::turnLEDOnOff(false,pins::LED_RGB_RED);
            Utils::printMessageLoopDebug(isModuleDebugModeActive, moduleName,"Red LED should be turned off.");
        }

    }
}