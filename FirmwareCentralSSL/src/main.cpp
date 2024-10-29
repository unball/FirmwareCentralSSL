#include "Arduino.h"
#include "Wire.h"

#define I2C_DEV_ADDR 0x55

uint32_t i = 0;

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Wire.begin();  // Inicia comunicação I2C
}

void loop() {
  delay(5000);

  // Envia mensagem para o periférico
  Wire.beginTransmission(I2C_DEV_ADDR);
  Wire.printf("Hello World! %lu", i++);
  uint8_t error = Wire.endTransmission(true);
  Serial.printf("Fim da transmissão: %u\n", error);

  // Lê 16 bytes do periférico
  uint8_t bytesReceived = Wire.requestFrom(I2C_DEV_ADDR, 16);
  Serial.printf("Bytes recebidos: %u\n", bytesReceived);

  if (bytesReceived > 0) {
    uint8_t temp[bytesReceived];
    Wire.readBytes(temp, bytesReceived);
    log_print_buf(temp, bytesReceived);  // Exibe buffer
  }
}
