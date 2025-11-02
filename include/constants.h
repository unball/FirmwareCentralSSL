
#ifndef CONSTANTS_H
#define CONSTANTS_H

namespace constants {

    constexpr uint8_t ROBOT_NUMBER = 0;

    // Execution parameters
    constexpr boolean DEBUG_EXECUTION_MODE = true;

    // ESP32 parameters
    constexpr uint16_t RESOLUTION_ADC_ESP32 = 4095;

    // Battery Measurement parameters
    constexpr float MEASUREMENT_BATTERY_MAX_VALUE = 2.15;
    constexpr float MEASUREMENT_BATTERY_MAX_VALUE_TRANSFORMATION = 12.2;

    // Drivers Comunication parameters
    constexpr uint32_t I2C_FREQUENCY = 400000UL;
    constexpr boolean I2C_DRIVER_ADDRESS_0 = 0x76;
    constexpr boolean I2C_DRIVER_ADDRESS_1 = 0x55;

}


#endif // CONSTANTS_H