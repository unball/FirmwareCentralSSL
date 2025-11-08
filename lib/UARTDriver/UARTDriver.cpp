#include "UARTDriver.hpp"

namespace UARTDriver
{
    void setIsModuleDebugModeActive(boolean isDebugModeActive){
        isModuleDebugModeActive = isDebugModeActive;
    }

    void setup(boolean isDebugModeActive){ 
        
        setIsModuleDebugModeActive(isDebugModeActive);
        
        driver0Serial.begin(9600,SERIAL_8N1,pins::UART_RX_0, pins::UART_TX_0);
        driver1Serial.begin(9600, SERIAL_8N2, pins::UART_RX_1, pins::UART_TX_1);

        Utils::printMessageSetupDebug(isModuleDebugModeActive, moduleName);
    
    }

    void sendInfoToDriver(float* wheelsVelocities){
        
        String wheelsDriver0 = "D0" + String(wheelsVelocities[0],2) + "," + String(wheelsVelocities[1],2); 
        String wheelsDriver1 = "D1" + String(wheelsVelocities[2],2) + "," + String(wheelsVelocities[3],2); 

        driver0Serial.println(wheelsDriver0);
        driver1Serial.println(wheelsDriver1);

    }

}