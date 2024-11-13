#include "Mestre.hpp"

namespace Receptor
{

    uint16_t velocidade_M0 = 0;
    uint16_t velocidade_M1 = 0;

    void setup()
    {
        Wire.begin(sda1, scl1, freq); // Inicializa o I2C com pinos SDA = 21, SCL = 22 e frequência = 400k
        Serial.begin(115200);
        Serial.println("\nI2C Mestre comunicando com múltiplos escravos");
    }

    void send_speed_2_driver(uint8_t addr, float u1, float u2)
    {
        Wire.beginTransmission(addr);

        Wire.write(reinterpret_cast<uint8_t *>(&u1), sizeof(float));
        Wire.write(reinterpret_cast<uint8_t *>(&u2), sizeof(float));

        Wire.endTransmission();
        Serial.print("Enviando para driver no endereço ");
        Serial.print(addr, HEX);
        Serial.print(": u1 = ");
        Serial.print(u1);
        Serial.print(", u2 = ");
        Serial.println(u2);
    }

    void loop()
    {
    }