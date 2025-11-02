
#ifndef PINS_HPP
#define PINS_HPP

namespace pins {
    
    // Battery Measurement GPIO
    constexpr uint8_t BATTERY_MEASUREMENT = 34;

    // RGB LED GPIO
    constexpr uint8_t LED_RGB_RED = 22;
    constexpr uint8_t LED_RGB_GREEN = 21;
    constexpr uint8_t LED_RGB_BLUE = 19;

    // Drivers Comunication GPIO
    constexpr uint8_t I2C_SDA = 13;
    constexpr uint8_t I2C_SCL = 15;
}


#endif // PINS_HPP