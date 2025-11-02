
#ifndef CONSTANTS_H
#define CONSTANTS_H

namespace constants {

    // Robot parameters
    constexpr uint8_t ROBOT_NUMBER = 0;
    constexpr float ROBOT_WHEELS_RADIUS = 0; // mm?
    constexpr float ROBOT_WHEELS_ANGLE_FRONT = 40; // graus
    constexpr float ROBOT_WHEELS_ANGLE_BACK = 45; // graus
    
    // Execution parameters
    constexpr int SERIAL_FREQUENCY = 115200;

    // ESP32 parameters
    constexpr uint16_t RESOLUTION_ADC_ESP32 = 4095;

    // Battery Measurement parameters
    constexpr float MEASUREMENT_BATTERY_MAX_VALUE = 2.15;
    constexpr float MEASUREMENT_BATTERY_MAX_VALUE_TRANSFORMATION = 12.2;

    // Drivers Comunication parameters
    constexpr uint32_t I2C_FREQUENCY = 400000UL;
    constexpr boolean I2C_DRIVER_ADDRESS_0 = 0x76;
    constexpr boolean I2C_DRIVER_ADDRESS_1 = 0x55;

    // WiFi Comunication parameters
    constexpr uint32_t COMMUNICATION_TIMEOUT = 1000; // ms
    constexpr uint32_t RESET_ESP32_TIMEOUT = 2000; // ms
}


#endif // CONSTANTS_H