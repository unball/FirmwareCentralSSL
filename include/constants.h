
#ifndef CONSTANTS_H
#define CONSTANTS_H

namespace constants {

    // Robot parameters
    constexpr uint8_t ROBOT_NUMBER = 0;
    constexpr float ROBOT_WHEELS_RADIUS = 0.09; // m
    constexpr float ROBOT_WHEELS_ANGLE_FRONT = 0.698132; // rad
    constexpr float ROBOT_WHEELS_ANGLE_BACK = 0.785398; // rad
    constexpr float ROBOT_MAX_WHEEL_VELOCITY = 50; // rad/s

    constexpr float ANGULAR_SPEED = 30.0f;
    constexpr float SPEED = 30.0f;

    // Execution parameters
    constexpr int SERIAL_FREQUENCY = 115200;

    // ESP32 parameters
    constexpr uint16_t RESOLUTION_ADC_ESP32 = 4095;

    // Battery Measurement parameters
    constexpr float MEASUREMENT_BATTERY_MAX_VALUE = 2.15;
    constexpr float MEASUREMENT_BATTERY_MAX_VALUE_TRANSFORMATION = 12.2;

    // Drivers Comunication parameters
    constexpr uint32_t UART_FREQUENCY = 9600;

    // WiFi Comunication parameters
    constexpr uint32_t COMMUNICATION_TIMEOUT = 5000; // ms
    constexpr uint32_t RESET_ESP32_TIMEOUT = 10000; // ms
}


#endif // CONSTANTS_H