#include "LEDs.hpp"

namespace LEDs
{

    void setup(){
        pinMode(pins::LED_RGB_VERMELHO, OUTPUT);
        pinMode(pins::LED_RGB_AZUL, OUTPUT);
        pinMode(pins::LED_RGB_VERDE, OUTPUT);
    }

    void turnLEDOnOff(boolean state, uint8_t pin){

        if(state){
            digitalWrite(pin, HIGH);
        }else{
            digitalWrite(pin, LOW);
        }
    }

}