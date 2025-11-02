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
  // a cada 500ms verifica nivel da bateria 
    // alerta se for necessario

  // verifica estado sensor IR a cada X s
    // ativa chute se tiver nas condições

  // recebe mensagem do WiFi a cada X s
  // recebe velocidade angular da IMU
  // executa controle a cada X s
    // calcula velocidade das rodas

  // envia velocidade para as rodas

}