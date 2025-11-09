#ifndef ESP_NOW_H
#define ESP_NOW_H

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h> 
#include "../../include/pins.h"
#include "../../include/constants.h"
#include "Utils.hpp"
#include "RobotMove.hpp"
#include "LEDs.hpp"

namespace EspNow
{   

    void setup(boolean isDebugModeActive, uint8_t robotNumber);
    void onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len);
    void demultiplexReceivedMessage(float linearVelocity_x,float linearVelocity_y, float angularVelocity);
    bool isCommunicationLost();

    struct message_t{
        int8_t robotId;
        float linearVelocity_x;
        float linearVelocity_y;
        float angularVelocity;
        int32_t checksum;
    };
    static uint8_t robotNumberId;
    static message_t message;
    
    struct keyboard_state_t {
        unsigned int x:2;
        unsigned int y:2;
        unsigned int clockwise_rotation:2;
    };
    static keyboard_state_t keyboardState = {0};

    static uint32_t comunicationTimeout = constants::COMMUNICATION_TIMEOUT;
    static uint32_t resetTimeout = constants::RESET_ESP32_TIMEOUT;

    namespace Transmitter{
        static uint8_t broadcastAddress[] = {0xA0, 0xDD, 0x6C, 0x04, 0xBA, 0x0C};
        static esp_now_peer_info_t peerInfo;

        void setupTransmitter(boolean isDebugModeActive);
        void executeTransmitter();
        void onDataSent(const uint8_t *macAddress, esp_now_send_status_t status);
    }

    static boolean isModuleDebugModeActive;
    static char* moduleName = "WiFi - EspNow ";
}

#endif // ESP_NOW_H