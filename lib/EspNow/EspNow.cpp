#include "EspNow.hpp"

namespace EspNow{

    void setIsModuleDebugModeActive(boolean isDebugModeActive){
        isModuleDebugModeActive = isDebugModeActive;
    }

    void onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len){

        tokenize(incomingData, len);
        lastTimeMessageReceived = micros();

        float calculateReceivedChecksum = message.linearVelocity_x + message.linearVelocity_y + message.angularVelocity;
        if(message.checksum == calculateReceivedChecksum){
            demultiplexReceivedMessage(message.linearVelocity_x, message.linearVelocity_y, message.angularVelocity);
        }else{
            Utils::printMessageLoopDebug(isModuleDebugModeActive, moduleName, "Erro checksum");
        }

    }

    void setup(boolean isDebugModeActive, uint8_t robotNumber){

        setIsModuleDebugModeActive(isDebugModeActive);

        robotNumberId = robotNumber;
        WiFi.disconnect();
        WiFi.mode(WIFI_STA);

        if(isDebugModeActive){
            String message = "MacAdress => " + WiFi.macAddress();
            Utils::printMessageSetupDebug(isModuleDebugModeActive, moduleName, message);
        }

        if(esp_now_init() != ESP_OK){
            Utils::printMessageSetupDebug(isModuleDebugModeActive, moduleName, "Erro ao inicializar ESP NOW");
        }

        esp_now_register_recv_cb(esp_now_recv_cb_t(onDataRecv));

        Utils::printMessageSetupDebug(isModuleDebugModeActive, moduleName);
    }

    void tokenize(const uint8_t *data,int len){

        if(data == NULL){
            return;
        }

        char str[len+1];
        memcpy(str,data,len);
        if(str[0]!='['){
            return;
        }
        str[len]='\0';

        double tokens[5];
        int i = 0;
        char* result;
        result = strtok(str, ",");
        while(result != nullptr){
            tokens[i] = std::atof(result);
            result = strtok(nullptr, ",");
            i++;

            if(i >= 5){
                break;
            }
        }

        message.robotId = tokens[0];
        message.linearVelocity_x = tokens[1];
        message.linearVelocity_y = tokens[2];
        message.angularVelocity = tokens[3];
        message.checksum = tokens[4];
    }

    void demultiplexReceivedMessage(float linearVelocity_x,float linearVelocity_y, float angularVelocity){

        if(message.robotId == robotNumberId){
            Utils::printMessageLoopDebug(isModuleDebugModeActive, moduleName, "Velocidade do robô recebida: ", linearVelocity_x, linearVelocity_y, angularVelocity);
            RobotMove::setRobotVelocities(linearVelocity_x, linearVelocity_y, angularVelocity);
        }

    }

    bool isCommunicationLost(){

        if((micros() - lastTimeMessageReceived) > comunicationTimeout){
            Utils::printMessageLoopDebug(isModuleDebugModeActive, moduleName, "Lost espNow Communication");

            if((micros() - lastTimeMessageReceived) > resetTimeout){
                Utils::printMessageLoopDebug(isModuleDebugModeActive, moduleName, "ESP will restart");
				ESP.restart();
            }

            return true;
        }

        return false;

    }
}
