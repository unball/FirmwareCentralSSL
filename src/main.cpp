#include "Arduino.h"
#include "BatteryMeasurement.hpp"
#include "LEDs.hpp"
#include "UARTDriver.hpp"
#include "RobotMove.hpp"
#include "EspNow.hpp"
#include "ExecutionManager.hpp"

#define TRANSMITTER_CODE false

#if TRANSMITTER_CODE
  void setup(){
    Serial.begin(constants::SERIAL_FREQUENCY);

    EspNow::Transmitter::setupTransmitter(ExecutionManager::DEBUG_TRANSMITTER);
    LEDs::setup(ExecutionManager::DEBUG_LEDs);

  }

  void loop()
  {

    EspNow::message_t message = {
      .robotId = 0,
      .linearVelocity_x = 0,
      .linearVelocity_y = 0,
      .angularVelocity = 60,
      .checksum = 60,
    }; 

    EspNow::Transmitter::executeTransmitter(message);
  }

#else

  void setup()
  {
    Serial.begin(constants::SERIAL_FREQUENCY);

    BatteryMeasurement::setup(ExecutionManager::DEBUG_BATTERY_MEASUREMENT);
    LEDs::setup(ExecutionManager::DEBUG_LEDs);
    UARTDriver::setup(ExecutionManager::DEBUG_I2CDRIVERS);
    RobotMove::setup(ExecutionManager::DEBUG_ROBOT_MOVE);
    EspNow::setup(ExecutionManager::DEBUG_ESPNOW, constants::ROBOT_NUMBER);
    // setup sensor IR
    // setup IMU
  }

  uint32_t previousTimestamp_robot_move = 0;
  uint32_t previousTimestamp_i2c_driver = 0;
  float* wheelsVelocities = new float[4] {0,0,0,0};

  uint32_t actualTimestamp = 0;

  void loop()
  {

    actualTimestamp = micros();

    
    
    if(ExecutionManager::EXECUTE_ROBOT_MOVE){
      if(actualTimestamp - previousTimestamp_robot_move > ExecutionManager::SAMPLE_TIME_ROBOT_MOVE){
        
        wheelsVelocities = RobotMove::calculateWheelVelocities();
        
        previousTimestamp_robot_move = actualTimestamp;
      }
    }

    if(EspNow::isCommunicationLost()){
      LEDs::turnLEDOnOff(false, pins::LED_BOARD);
      wheelsVelocities = new float[4] {0,0,0,0};
    }

    if(ExecutionManager::EXECUTE_I2CDRIVERS){
      if(actualTimestamp - previousTimestamp_i2c_driver > ExecutionManager::SAMPLE_TIME_I2CDRIVERS){

        UARTDriver::sendInfoToDriver(wheelsVelocities);

        previousTimestamp_i2c_driver = actualTimestamp;
      }
    }
  }
#endif

