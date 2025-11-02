#include "Arduino.h"
#include "BatteryMeasurement.hpp"
#include "LEDs.hpp"
#include "I2CDriver.hpp"

const bool activeAllDebug = constants::DEBUG_EXECUTION_MODE;

void setup()
{
  Serial.begin(115200);

  BatteryMeasurement::setup(activeAllDebug);
  LEDs::setup(activeAllDebug);
  I2CDriver::setup(activeAllDebug);
  // setup wiFi
  // setup sensor IR
  // setup IMU
  // setup RobotMove
}

void loop()
{


}