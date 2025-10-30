#include "I2CDriver.hpp"

#define I2C_DEV_ADDR 0x55

namespace I2CDriver
{

    void setup()
    {
        Wire.begin(pins::I2C_SDA, pins::I2C_SCL, constantes::I2C_FREQUENCY); // Inicializa o I2C com pinos SDA = 21, SCL = 22 e frequência = 400k
        Serial.println("\nI2C Mestre comunicando com escravo");
    }

    void sendInfoToDriver(uint8_t addr, float u1, float u2)
    {
        Wire.beginTransmission(addr);

        Wire.write(reinterpret_cast<uint8_t *>(&u1), sizeof(float));
        Wire.write(reinterpret_cast<uint8_t *>(&u2), sizeof(float));

        Wire.endTransmission();
        Serial.print("Enviando para driver no endereço ");
        Serial.print(addr, HEX);
        Serial.print(": u1 = ");
        Serial.print(u1,3);
        Serial.print(", u2 = ");
        Serial.println(u2,3);
    }

}