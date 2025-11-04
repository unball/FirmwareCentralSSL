
#ifndef PINS_HPP
#define PINS_HPP

namespace pins {
    
    // Battery Measurement GPIO
    constexpr uint8_t BATTERY_MEASUREMENT = 34;

    // RGB LED GPIO
    constexpr uint8_t LED_RGB_RED = 22;
    constexpr uint8_t LED_RGB_GREEN = 21;
    constexpr uint8_t LED_RGB_BLUE = 19;
    constexpr uint8_t LED_BOARD = 2;
    

    // Drivers Comunication GPIO
    constexpr uint8_t UART_RX_0 = 13;
    constexpr uint8_t UART_TX_0 = 15;
    constexpr uint8_t UART_RX_1 = 17;
    constexpr uint8_t UART_TX_1 = 16;
}


#endif // PINS_HPP