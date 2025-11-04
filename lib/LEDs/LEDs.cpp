#include "LEDs.hpp"

namespace LEDs
{  
    void setIsModuleDebugModeActive(boolean isDebugModeActive){
        isModuleDebugModeActive = isDebugModeActive;
    }

    void setup(boolean isDebugModeActive){

        setIsModuleDebugModeActive(isDebugModeActive);

        pinMode(pins::LED_RGB_RED, OUTPUT);
        pinMode(pins::LED_RGB_BLUE, OUTPUT);
        pinMode(pins::LED_RGB_GREEN, OUTPUT);
        pinMode(pins::LED_BOARD, OUTPUT);

        Utils::printMessageSetupDebug(isModuleDebugModeActive, moduleName);
    }

    void turnLEDOnOff(boolean state, uint8_t pin){

        if(state){
            digitalWrite(pin, HIGH);
            Utils::printMessageLoopDebug(isModuleDebugModeActive, moduleName, pin, "LED should be turned on. GPIO ");
        }else{
            digitalWrite(pin, LOW);
            Utils::printMessageLoopDebug(isModuleDebugModeActive, moduleName, pin, "LED should be turned off. GPIO ");
        }
    }

}