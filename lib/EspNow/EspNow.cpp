#include "EspNow.hpp"
#include "ExecutionManager.hpp"

namespace EspNow{

    volatile static uint32_t lastTimeMessageReceived = 0;

    void setIsModuleDebugModeActive(boolean isDebugModeActive){
        isModuleDebugModeActive = isDebugModeActive;
    }

    void onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len){
        memcpy(&keyboardState, incomingData, sizeof(keyboard_state_t));
        lastTimeMessageReceived = millis();

        if(keyboardState.restart == 1){
            RobotMove::setRobotVelocities(0,0,0);
            LEDs::turnLEDOnOff(false, pins::LED_BOARD);
            ESP.restart();
            return;
        }

        // float calculateReceivedChecksum = message.linearVelocity_x + message.linearVelocity_y + message.angularVelocity;
        // if (message.checksum == calculateReceivedChecksum) {
            message = {
                .robotId = 0,
                .command = 0,
                .linearVelocity_x = keyboardState.x == 2 ? constants::SPEED
                                : (keyboardState.x == 0 ? -constants::SPEED : 0),
                .linearVelocity_y = keyboardState.y == 2 ? constants::SPEED
                                : (keyboardState.y == 0 ? -constants::SPEED : 0),
                .angularVelocity = keyboardState.clockwise_rotation == 2 ? constants::ANGULAR_SPEED
                                : (keyboardState.clockwise_rotation == 0 ? -constants::ANGULAR_SPEED : 0),
                .checksum = 0,
            };

            

            demultiplexReceivedMessage(message.linearVelocity_x, message.linearVelocity_y, message.angularVelocity);
            LEDs::turnLEDOnOff(true, pins::LED_BOARD);
        // } else {
        //     Utils::printMessageLoopDebug(isModuleDebugModeActive, moduleName, "Erro checksum");
        // }
    }

    void setup(boolean isDebugModeActive, uint8_t robotNumber){

        setIsModuleDebugModeActive(isDebugModeActive);

        robotNumberId = robotNumber;
        WiFi.disconnect();
        WiFi.mode(WIFI_STA);
        
        // if(isDebugModeActive){
        //     WiFi.begin();
        //     String message = "MacAdress => " + WiFi.macAddress();
        //     Utils::printMessageSetupDebug(isModuleDebugModeActive, moduleName, message);
        // }

        if(esp_now_init() != ESP_OK){
            Utils::printMessageSetupDebug(isModuleDebugModeActive, moduleName, "Erro ao inicializar ESP NOW");
        }

        esp_now_register_recv_cb(esp_now_recv_cb_t(onDataRecv));

        Utils::printMessageSetupDebug(isModuleDebugModeActive, moduleName);
    }

    void demultiplexReceivedMessage(float linearVelocity_x,float linearVelocity_y, float angularVelocity){

        if(message.robotId == robotNumberId){
            Utils::printMessageLoopDebug(isModuleDebugModeActive, moduleName, " Velocidade do robô recebida: ", linearVelocity_x, linearVelocity_y, angularVelocity);
            RobotMove::setRobotVelocities(linearVelocity_x, linearVelocity_y, angularVelocity);
        }

    }

    bool isCommunicationLost(){

        if((millis() - lastTimeMessageReceived) > comunicationTimeout){

            Utils::printMessageLoopDebug(isModuleDebugModeActive, moduleName, "Lost espNow Communication");

            // if((millis() - lastTimeMessageReceived) > resetTimeout){
            //     Utils::printMessageLoopDebug(isModuleDebugModeActive, moduleName, "ESP will restart");
			// 	ESP.restart();
            // }

            return true;
        }

        return false;

    }

    void Transmitter::setupTransmitter(boolean isDebugModeActive){

        WiFi.mode(WIFI_STA);
 
        if (esp_now_init() != ESP_OK) {
            Serial.println("Error initializing ESP-NOW");
            return;
        }

        esp_now_register_send_cb(Transmitter::onDataSent);

        peerInfo.channel = 0;  
        peerInfo.encrypt = false;
        memcpy(peerInfo.peer_addr, broadcastAddress, 6);
        if (esp_now_add_peer(&peerInfo) != ESP_OK){
            Serial.println("Failed to add peer");
            return;
        }

    }

    void Transmitter::executeTransmitter(){
        if (Serial.available()) {
            uint32_t actualTimestamp = micros();

            keyboard_state_t newKeyboardState = {0};
            Serial.readBytes((uint8_t *) &newKeyboardState, sizeof(newKeyboardState));

            int changed = 0;
            if(keyboardState.x != newKeyboardState.x) changed = 1;
            if(keyboardState.y != newKeyboardState.y) changed = 1;
            if(keyboardState.clockwise_rotation != newKeyboardState.clockwise_rotation) changed = 1;
            if(keyboardState.restart != newKeyboardState.restart) changed = 1;

            keyboardState = newKeyboardState;

            if(changed || (actualTimestamp - previousTimestampCommandSent) > ExecutionManager::SAMPLE_TIME_ROBOT_MOVE / 2){
                esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &keyboardState, sizeof(keyboardState));
        
                if (result == ESP_OK) {
                    LEDs::turnLEDOnOff(true, pins::LED_BOARD);
                    Serial.println("Sent with success");
                    previousTimestampCommandSent = actualTimestamp;
                }
                else {
                    LEDs::turnLEDOnOff(false, pins::LED_BOARD);
                    Serial.println("Error sending the data");
                }
            }
        }
        delay(3);
    }

    
    void Transmitter::onDataSent(const uint8_t *macAddress, esp_now_send_status_t status){
        char macStr[18];
        Serial.print("Packet from: ");
        // Copies the sender mac address to a string
        snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
                macAddress[0], macAddress[1], macAddress[2], macAddress[3], macAddress[4], macAddress[5]);
        Serial.print(macStr);
        Serial.print(" send status:\t");
        Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
    }

}
