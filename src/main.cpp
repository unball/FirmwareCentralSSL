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
  ExecutionManager::executeAll();

  BatteryMeasurement::setup(ExecutionManager::DEBUG_BATTERY_MEASUREMENT);
  LEDs::setup(ExecutionManager::DEBUG_LEDs);
  I2CDriver::setup(ExecutionManager::DEBUG_I2CDRIVERS);
  RobotMove::setup(ExecutionManager::DEBUG_ROBOT_MOVE);
  EspNow::setup(ExecutionManager::DEBUG_ESPNOW, constants::ROBOT_NUMBER);
  // setup sensor IR
  // setup IMU
}

float batteryVoltageLevel = 0;
float* wheels_velocities;
float* wheels_velocities_driver_0;
float* wheels_velocities_driver_1;

boolean isBallClose;

uint32_t previousTimestamp_battery_measurement = 0;
uint32_t previousTimestamp_ir_sensor = 0;
uint32_t previousTimestamp_imu_sensor = 0;
uint32_t previousTimestamp_robot_move = 0;
uint32_t previousTimestamp_i2cdriver = 0;
uint32_t previousTimestamp_espnow = 0;

uint32_t actualTimestamp = 0;

void loop()
{

  actualTimestamp = micros();

  if(ExecutionManager::EXECUTE_BATTERY_MEASUREMENT){
    
    if(actualTimestamp - previousTimestamp_battery_measurement > ExecutionManager::SAMPLE_TIME_BATTERY_MEASUREMENT){
      // verifica nivel da bateria 
      // alerta se for necessario

      previousTimestamp_battery_measurement = actualTimestamp;
    }

  }

  // TODO: 
  if(ExecutionManager::EXECUTE_IR_SENSOR){
    if(actualTimestamp - previousTimestamp_ir_sensor > ExecutionManager::SAMPLE_TIME_IR_SENSOR){
      // verifica estado sensor IR 
      // ativa chute se tiver nas condições

      previousTimestamp_ir_sensor = actualTimestamp;
    }
  }

  if(ExecutionManager::EXECUTE_ESP_NOW){
    if(actualTimestamp - previousTimestamp_espnow > ExecutionManager::SAMPLE_TIME_ESP_NOW){
      // recebe mensagem do WiFi
        // EspNow::message; -> é pra ter os valores recebidos 
      // atualiza valores
        // RobotMove::robotVelocities -> valores atualizados

      previousTimestamp_espnow = actualTimestamp;
    }
  }

  // TODO: 
  if(ExecutionManager::EXECUTE_IMU_SENSOR){
    if(actualTimestamp - previousTimestamp_imu_sensor > ExecutionManager::SAMPLE_TIME_IMU_SENSOR){
      // recebe velocidade angular da IMU
      // atualiza valores para controle do robo

      previousTimestamp_imu_sensor = actualTimestamp;
    }
  }

  if(ExecutionManager::EXECUTE_ROBOT_MOVE){
    if(actualTimestamp - previousTimestamp_robot_move > ExecutionManager::SAMPLE_TIME_ROBOT_MOVE){

      wheels_velocities = RobotMove::calculateWheelVelocities();

      previousTimestamp_robot_move = actualTimestamp;
    }
  }

  if(ExecutionManager::EXECUTE_I2CDRIVERS){
    if(actualTimestamp - previousTimestamp_i2cdriver > ExecutionManager::SAMPLE_TIME_I2CDRIVERS){
      // envia velocidade para as rodas
      I2CDriver::sendInfoToDriver(0, wheels_velocities);
      I2CDriver::sendInfoToDriver(1, wheels_velocities);

      previousTimestamp_i2cdriver = actualTimestamp;
    }
  }

}