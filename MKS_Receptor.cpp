#include <Wire.h>
#include <Arduino.h>

void receiveEvent(int howMany)
{
  while (Wire.available())
  {
    char receivedChar = Wire.read(); // Lê o dado recebido
    Serial.print("Escravo 8 recebeu: ");
    Serial.println(receivedChar);
  }
}

void setup()
{
  Wire.begin(8, 19, 18, 400000); // Configura este ESP32 como escravo no endereço 8
  Wire.onReceive(receiveEvent);  // Registra o evento de recebimento
  Serial.begin(115200);
  Serial.println("Escravo 8 pronto para receber dados");
}

void loop()
{
  delay(100); // Mantém o loop rodando
}
