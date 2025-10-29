#ifndef LEDs_H
#define LEDs_H

#include <Arduino.h>
#include "../../include/pins.h"
#include "../../include/constantes.h"

namespace LEDs
{
    void setup();
    void turnLEDOnOff(boolean state, uint8_t pin);
}

#endif // LEDs_H