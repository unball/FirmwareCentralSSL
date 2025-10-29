#include "Utils.hpp"

namespace Utils
{
    float getADCReadingsAverage(uint8_t pin){

        float sum = 0;
        int numberSamples = 10;

        for(int i=0; i<numberSamples; i++){
            uint16_t reading = analogRead(pin);
            sum += reading;
        }

        return sum/numberSamples;

    }

}