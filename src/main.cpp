#include "Arduino.h"
#include "BatteryMeasurement.hpp"
#include "LEDs.hpp"
#include "I2CDriver.hpp"
#include "RobotMove.hpp"
#include "EspNow.hpp"

const bool activeAllDebug = constants::DEBUG_EXECUTION_MODE;

void setup()
{
  Serial.begin(115200);

  BatteryMeasurement::setup(activeAllDebug);
  LEDs::setup(activeAllDebug);
  I2CDriver::setup(activeAllDebug);
  RobotMove::setup(activeAllDebug);
  EspNow::setup(activeAllDebug, constants::ROBOT_NUMBER);
  // setup sensor IR
  // setup IMU
}

void loop()
{


}