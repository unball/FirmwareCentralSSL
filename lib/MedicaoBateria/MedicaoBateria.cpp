#include "MedicaoBateria.hpp"

namespace MedicaoBateria
{

    void setup()
    {
       pinMode(pins::MEDICAO_BATERIA, INPUT);
    }

    float getBatteryCharge(){
        float batteryADCValue = Utils::getADCReadingsAverage(pins::MEDICAO_BATERIA);
        float batteryValue = 0;

        batteryValue = (batteryADCValue/constantes::RESOLUCAO_ADC_ESP32) * constantes::MEDICAO_BAT_MAX_VALUE;

        return batteryValue;
    }

    float sendDataDrivers(float batteryLevelRead)
    {
        // Tensão para o driver
        float batteryLevelToDriver = (constantes::MEDICAO_BAT_MAX_VALUE_TRANSFORMATION/constantes::MEDICAO_BAT_MAX_VALUE) * batteryLevelRead;

        return batteryLevelToDriver;
    }

    float alertLowBattery(float batteryLevelRead){
        // % da bateria
        float percentage = (batteryLevelRead/constantes::MEDICAO_BAT_MAX_VALUE) * 100;

        return percentage;

    }
}