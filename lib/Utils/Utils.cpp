#include "Utils.hpp"

namespace Utils
{
    float getADCReadingsAverage(uint8_t pin, float valueVoltage){

        float sum = 0;
        int numberSamples = 100;
        float value;

        for(int i=0; i<numberSamples; i++){
            uint16_t ADCReading = analogRead(pin);
            value = (ADCReading/constants::RESOLUTION_ADC_ESP32)*valueVoltage;
            sum += value;
        }

        return sum/numberSamples;

    }

    void printMessageSetupDebug(boolean isDebugModeActive, char* moduleName){
        
        if(isDebugModeActive){
            Serial.print("Setup: ");
            Serial.println(moduleName);
        }

    }

    void printMessageSetupDebug(boolean isDebugModeActive, char* moduleName, char* message){
        
        if(isDebugModeActive){
            Serial.print("Setup Error: ");
            Serial.print("\t");
            Serial.print(moduleName);
            Serial.print("\t - ");
            Serial.println(message);

        }

    }

    void printMessageLoopDebug(boolean isDebugModeActive, char* moduleName, char* variables, float var1, float var2, float var3){
        
        if(isDebugModeActive){
            Serial.print("Executing ");
            Serial.print("\t");
            Serial.print(moduleName);
            Serial.print("-");
            Serial.print(variables);
            Serial.print(": ");
            Serial.print(var1,4);
            Serial.print("\t");
            Serial.print(var2,4);
            Serial.print("\t");
            Serial.println(var3,4);
        }

    }

    void printMessageLoopDebug(boolean isDebugModeActive, char* moduleName, char* variables, float var1){
        
        if(isDebugModeActive){
            Serial.print("Executing ");
            Serial.print("\t");
            Serial.print(moduleName);
            Serial.print("-");
            Serial.print(variables);
            Serial.print(": ");
            Serial.println(var1,4);
        }

    }

    void printMessageLoopDebug(boolean isDebugModeActive, char* moduleName, char* message){
        
        if(isDebugModeActive){
            Serial.print("Executing ");
            Serial.print("\t");
            Serial.print(moduleName);
            Serial.print(": ");
            Serial.println(message);
        }
        
    }

    void printMessageLoopDebug(boolean isDebugModeActive, char* moduleName, uint8_t pin, char* message){
        
        if(isDebugModeActive){
            Serial.print("Executing ");
            Serial.print("\t");
            Serial.print(moduleName);
            Serial.print(": ");
            Serial.print(message);
            Serial.print(" ");
            Serial.println(pin);
        }
        
    }

    void printMessageLoopDebug(boolean isDebugModeActive, char* moduleName, char* variables, float var1, float var2, float var3, float var4){
        
        if(isDebugModeActive){
            Serial.print("Executing ");
            Serial.print("\t");
            Serial.print(moduleName);
            Serial.print("-");
            Serial.print(variables);
            Serial.print(": ");
            Serial.print(var1,4);
            Serial.print("\t");
            Serial.print(var2,4);
            Serial.print("\t");
            Serial.print(var3,4);
            Serial.print("\t");
            Serial.println(var4,4);


        }

    }

}