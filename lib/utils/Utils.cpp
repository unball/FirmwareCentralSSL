#include "Utils.hpp"

namespace Utils
{
    float getADCReadingsAverage(uint8_t pin, float valueVoltage){

        float sum = 0;
        int numberSamples = 100;
        float value;

        for(int i=0; i<numberSamples; i++){
            uint16_t ADCReading = analogRead(pin);
            value = (ADCReading/constantes::RESOLUCAO_ADC_ESP32)*valueVoltage;
            sum += value;
        }

        return sum/numberSamples;

    }

}