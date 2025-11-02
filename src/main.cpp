#include "Arduino.h"
#include "BatteryMeasurement.hpp"
#include "LEDs.hpp"
#include "I2CDriver.hpp"
#include "RobotMove.hpp"
#include "EspNow.hpp"
#include "ExecutionManager.hpp"

void setup()
{
  Serial.begin(constants::SERIAL_FREQUENCY);

  ExecutionManager::setDebugModeAll();

  BatteryMeasurement::setup(ExecutionManager::DEBUG_BATTERY_MEASUREMENT);
  LEDs::setup(ExecutionManager::DEBUG_LEDs);
  I2CDriver::setup(ExecutionManager::DEBUG_I2CDRIVERS);
  RobotMove::setup(ExecutionManager::DEBUG_ROBOT_MOVE);
  EspNow::setup(ExecutionManager::DEBUG_ESPNOW, constants::ROBOT_NUMBER);
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