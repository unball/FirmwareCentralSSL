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

    void loop()
    {
        delay(2000);

        // Enviar para o primeiro driver no endereço 8
        Wire.beginTransmission(addr_driver1);
        // Wire.write(o que botar aqui?); // Envia o valor 0 para o escravo no endereço 8

        // Wire.write(reinterpret_cast<uint8_t *>(&u1), sizeof(float));
        // Wire.write(reinterpret_cast<uint8_t *>(&u2), sizeof(float));

        Wire.endTransmission();
        Serial.println("Fim da transmissão driver 1");

        delay(2000);

        // Enviar para o segundo driver no endereço 9
        Wire.beginTransmission(9);
        Wire.write(1); // Envia o valor 1 para o escravo no endereço 9
        Wire.endTransmission();
        Serial.println("Fim da transmissão driver 2");

        delay(2000);
    }
}