#ifndef I2CDRIVER_H
#define I2CDRIVER_H

#include <Arduino.h>
#include <stdint-gcc.h>
#include <Wire.h>
#include "../../include/constantes.h"
#include "../../include/pins.h"

namespace I2CDriver
{
    void setup();
    void sendInfoToDriver(uint8_t addr, float u1, float u2);
}

#endif // I2CDRIVER_H