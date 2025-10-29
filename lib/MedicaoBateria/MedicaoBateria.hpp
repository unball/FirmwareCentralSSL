#ifndef MEDICAO_BATERIA_H
#define MEDICAO_BATERIA_H

#include <Arduino.h>
#include <Wire.h>
#include "../../include/pins.h"
#include "../../include/constantes.h"
#include "Utils.hpp"
#include "LEDs.hpp"


namespace MedicaoBateria
{
    void setup();
    float getBatteryCharge();
    float sendDataDrivers(float batteryLevelRead);
    void alertLowBattery(float batteryLevelRead);
}

#endif // MEDICAO_BATERIA_H