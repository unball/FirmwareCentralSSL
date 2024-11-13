#ifndef MESTRE_H
#define MESTRE_H

#include <Arduino.h>
#include <stdint-gcc.h>
#include <Wire.h>
#include "../../include/config.h"

namespace Mestre
{
    void loop();
    void setup();
    void send_speed_2_driver_1();
}

#endif