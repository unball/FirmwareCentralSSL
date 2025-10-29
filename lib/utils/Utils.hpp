#ifndef UTILS_H
#define UTILS_H

#include <Arduino.h>
#include "../../include/pins.h"
#include "../../include/constantes.h"

namespace Utils
{
    float getADCReadingsAverage(uint8_t pin);
}

#endif // UTILS_H