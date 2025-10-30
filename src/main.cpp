#include "Arduino.h"
#include "MedicaoBateria.hpp"
#include "LEDs.hpp"
#include "I2CDriver.hpp"

#define I2C_DEV_ADDR_0 0x76
#define I2C_DEV_ADDR_1 0x55

void setup()
{
  Serial.begin(115200);
  Serial.println("Setup");

  MedicaoBateria::setup();
  LEDs::setup();
  I2CDriver::setup();

}

void loop()
{

  // Le bateria a cada X ms
  // Envia bateria pro driver -> se n teve atualização no valor, envia valor passado
  // Se bateria < 96% => acende LED pra alertar bateria baixa
  // float bat = MedicaoBateria::getBatteryCharge();
  // float driver = MedicaoBateria::sendDataDrivers(bat);
  // MedicaoBateria::alertLowBattery(bat);

  unsigned long t0 = micros();

  I2CDriver::sendInfoToDriver(I2C_DEV_ADDR_0, 10, 0.001);

  I2CDriver::sendInfoToDriver(I2C_DEV_ADDR_1, 10, 0.001);

  delay(500);

}