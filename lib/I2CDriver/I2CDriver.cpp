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

    void sendInfoToDriver(uint8_t driverNumber, float* wheelsVelocities){
        
        uint8_t address;
        uint8_t firstWheel;
        uint8_t secondWheel;

        if(driverNumber == 0){
            address = constants::I2C_DRIVER_ADDRESS_0;
            firstWheel = 0;
            secondWheel = 1;
        }else{
            address = constants::I2C_DRIVER_ADDRESS_1;
            firstWheel = 2;
            secondWheel = 3;
        }


        Wire.beginTransmission(address);

        Wire.write(reinterpret_cast<uint8_t *>(&wheelsVelocities[firstWheel]), sizeof(float));
        Wire.write(reinterpret_cast<uint8_t *>(&wheelsVelocities[secondWheel]), sizeof(float));

        Wire.endTransmission();
        Utils::printMessageLoopDebug(isModuleDebugModeActive, moduleName, "Driver address and wheels velocities [rad/s] ", address, wheelsVelocities[firstWheel], wheelsVelocities[secondWheel]);
    }

}