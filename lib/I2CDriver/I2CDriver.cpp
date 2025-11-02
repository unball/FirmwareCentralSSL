#include "I2CDriver.hpp"

namespace I2CDriver
{
    void setIsModuleDebugModeActive(boolean isDebugModeActive){
        isModuleDebugModeActive = isDebugModeActive;
    }

    void setup(boolean isDebugModeActive){ 
        
        setIsModuleDebugModeActive(isDebugModeActive);
        
        Wire.begin(pins::I2C_SDA, pins::I2C_SCL, constants::I2C_FREQUENCY);
    
        Utils::printMessageSetupDebug(isModuleDebugModeActive, moduleName);
    
    }

    void sendInfoToDriver(uint8_t address, float wheelVelocity0, float wheelVelocity1){
        
        Wire.beginTransmission(address);

        Wire.write(reinterpret_cast<uint8_t *>(&wheelVelocity0), sizeof(float));
        Wire.write(reinterpret_cast<uint8_t *>(&wheelVelocity1), sizeof(float));

        Wire.endTransmission();
        Utils::printMessageLoopDebug(isModuleDebugModeActive, moduleName, "Driver address and wheels velocities [rad/s] ", address, wheelVelocity0, wheelVelocity1);
    }

}