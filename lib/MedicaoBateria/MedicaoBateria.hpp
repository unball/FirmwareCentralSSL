#ifndef MEDICAO_BATERIA_H
#define MEDICAO_BATERIA_H

#include <Arduino.h>
#include <stdint-gcc.h>
#include <Wire.h>
#include "../../include/pins.h"
#include "../../include/constantes.h"
#include "Utils.hpp"


namespace MedicaoBateria
{
    void setup();
    float getBatteryCharge();
    float sendDataDrivers(float batteryLevelRead);
    float alertLowBattery(float batteryLevelRead);
}

#endif // MEDICAO_BATERIA_H