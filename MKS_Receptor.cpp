#include <Arduino.h>
#include <Wire.h>

TwoWire I2Cone = TwoWire(0); // Canal I2C especificado (0)
int recebido = -1;           // Variável para armazenar o valor recebido

void receiveEvent(int howMany)
{
  if (I2Cone.available())
  {
    recebido = I2Cone.read(); // Lê o valor enviado pelo mestre
    Serial.print("Recebido: ");
    Serial.println(recebido);
  }
}

void setup()
{
  I2Cone.begin(4, 19, 18, 400000); // Configura SDA = 4, SCL = 19, 400 kHz
  I2Cone.onReceive(receiveEvent);  // Configura a função de callback para receber dados
  Serial.begin(115200);
  Serial.println("Escravo I2C Iniciado");
}

void loop()
{
  // O loop fica vazio porque a função receiveEvent trata a recepção de dados.
}

