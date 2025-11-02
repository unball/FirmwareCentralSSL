#ifndef WIFI_H
#define WIFI_H

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h> 
#include "../../include/pins.h"
#include "../../include/constants.h"
#include "Utils.hpp"
#include "RobotMove.hpp"

namespace EspNow
{   

    struct messageReceived{
        int8_t robotId;
        float linearVelocity_x;
        float linearVelocity_y;
        float angularVelocity;
        int32_t checksum;
    };

    void setup(boolean isDebugModeActive, uint8_t robotNumber);
    void onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len);
    void demultiplexReceivedMessage(float linearVelocity_x,float linearVelocity_y, float angularVelocity);
    bool isCommunicationLost();
    void tokenize(const uint8_t *data,int len);

    static uint8_t robotNumberId;
    static messageReceived message;

    static uint32_t lastReceived;
    static uint32_t comunicationTimeout = 1000;
    static uint32_t resetTimeout = 2000;

    static boolean isModuleDebugModeActive;
    static char* moduleName = "WiFi";
}

#endif // WIFI_H