#include "wifi.hpp"


/* Wi-Fi */ 
const int communicationTimeout = 500000;
const int resetTimeout = 4000000;
const float MAX_POWER = 10.0;

namespace Wifi{

    uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

    rcv_message temp_msg;

    volatile static uint32_t lastReceived;

    uint8_t robotNumber;

    void setup(){

        WiFi.mode(WIFI_STA);
        WiFi.disconnect();
        if (esp_now_init() != ESP_OK) {
            #if WEMOS_DEBUG
            Serial.println("Erro ao inicializar o ESP-NOW");
            #endif
            return;
        }
        WiFi.setTxPower(WIFI_POWER_19_5dBm);
        esp_err_t error = esp_wifi_set_channel(14, WIFI_SECOND_CHAN_NONE);

        esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));            
    }

    // Callback function, execute when message is received via Wi-Fi
    void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len){
        memcpy(&temp_msg, incomingData, sizeof(temp_msg));
	    lastReceived = micros();

    }
    /// @brief Receive data copying from temp struct to global struct
    /// @param v reference to the linear velocity
    /// @param w reference to the angular velocity
    void receiveData(int16_t *vx, int16_t *vy, int16_t *w){
            // Demultiplexing and decoding the velocities and constants
            *vx  = temp_msg.vx;
            *vy  = temp_msg.vy;
            *w  = temp_msg.w;
    }

    bool isCommunicationLost(){
        if((micros() - lastReceived) > communicationTimeout){
			// Communication probably failed
			if((micros() - lastReceived) > resetTimeout)
				ESP.restart();
            return true;
		}
        return false;
    }

}