#include "wifi.hpp"
#include "esp_wifi.h"

namespace Wifi {

    const int communicationTimeout = 500000;
    const int resetTimeout = 4000000;     
    volatile uint32_t lastReceived = 0;

    rcv_message temp_msg;
    rcv_message msg;

    void setup() {
        WiFi.mode(WIFI_STA);

         Serial.println(WiFi.softAPmacAddress()); // Inclua essa linha no projeto para mostrar o MAC do ESP32 no monitor serial
        esp_wifi_set_promiscuous(true);
        esp_wifi_set_channel(14, WIFI_SECOND_CHAN_NONE);
        esp_wifi_set_promiscuous(false);

        if (esp_now_init() != ESP_OK) {
            Serial.println("Erro ao inicializar ESP-NOW");
            return;
        }

        esp_now_register_recv_cb(OnDataRecv);
    }

    void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
        memcpy(&temp_msg, incomingData, sizeof(temp_msg));
        lastReceived = micros();

        // Verificar checksum
        int16_t calculatedChecksum = temp_msg.vx + temp_msg.vy + temp_msg.w;
        if (temp_msg.checksum == calculatedChecksum) {
            msg = temp_msg;

            if (msg.vx == 0 && msg.vy == 1 && msg.w == 0) {
                Serial.println("Comando recebido: Frente (W)");
            } else if (msg.vx == -1 && msg.vy == 0 && msg.w == -1) {
                Serial.println("Comando recebido: Esquerda (A)");
            } else if (msg.vx == 0 && msg.vy == -1 && msg.w == 0) {
                Serial.println("Comando recebido: Ré (S)");
            } else if (msg.vx == 1 && msg.vy == 0 && msg.w == 0) {
                Serial.println("Comando recebido: Direita (D)");
            } else {
                Serial.println("Comando desconhecido recebido");
            }
        } else {
            Serial.println("Erro de checksum");
        }
    }

    void receiveData(int16_t *vx, int16_t *vy, int16_t *w) {
        *vx = msg.vx;
        *vy = msg.vy;
        *w = msg.w;
    }

    bool isCommunicationLost() {
        return (micros() - lastReceived) > communicationTimeout;
    }
}
