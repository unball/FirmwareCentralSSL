#include "Arduino.h"
#include "Wire.h"
#include "Mestre.hpp"

#define I2C_DEV_ADDR 0x55

uint32_t i = 0;

// Encontrar constantes
uint32_t l = 10;
uint32_t w = 10;
uint32_t r = 3.5;

void setup()
{
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Wire.begin(); // Inicia comunicação I2C
  // TODO: Fazer setup do WiFi
}

void loop()
{
  // TODO: Remover delay
  delay(5000);

  // TODO: Recebe velocidades x,y e w do wifi

  // TODO: Traduz velocidades x, y e w para velocidade das rodas u1,u2,u3 e u4
  //  calc_speed_2_motor(colocar argumentos)

  // TODO: Envia velocidade das rodas para drivers
  //  send_speed_2_drivers(float* goal_velocity_motor)

  // TODO: Colocar comunicação abaixo na função send_speed_2_drivers
  //  Envia mensagem para o periférico
  /*Wire.beginTransmission(I2C_DEV_ADDR);
  Wire.printf("Hello World! %lu", i++);
  uint8_t error = Wire.endTransmission(true);
  Serial.printf("Fim da transmissão: %u\n", error);

  // OBS.: Não precisa ler do periferico por enquanto
  //  Lê 16 bytes do periférico
  uint8_t bytesReceived = Wire.requestFrom(I2C_DEV_ADDR, 16);
  Serial.printf("Bytes recebidos: %u\n", bytesReceived);

  if (bytesReceived > 0)
  {
    uint8_t temp[bytesReceived];
    Wire.readBytes(temp, bytesReceived);
    log_print_buf(temp, bytesReceived); // Exibe buffer
  }*/

  // valores arbitários
  float x_dot = 1.0;
  float y_dot = 2.0;
  float theta_dot = 3.0;

  std::array<float, 4> speeds = calc_speed_2_motor(x_dot, y_dot, theta_dot);
  float u1 = speeds[0];
  float u2 = speeds[1];
  float u3 = speeds[2];
  float u4 = speeds[3];

  Mestre::setup();
  Mestre::send_speed_2_driver(addr_driver1, u1, u2);
}

std::array<float, 4> calc_speed_2_motor(float x_dot, float y_dot, float theta_dot)
{

  std::array<float, 4> speeds;

  speeds[0] = (1 / r) * (theta_dot * (-l - w) + x_dot - y_dot); // u1
  speeds[1] = (1 / r) * (theta_dot * (l + w) + x_dot + y_dot);  // u2
  speeds[2] = (1 / r) * (theta_dot * (l + w) + x_dot - y_dot);  // u3
  speeds[3] = (1 / r) * (theta_dot * (-l - w) + x_dot + y_dot); // u4

  return speeds;
}
