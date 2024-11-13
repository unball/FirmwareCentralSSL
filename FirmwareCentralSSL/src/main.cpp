#include "Arduino.h"
#include "Wire.h"

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
  Wire.beginTransmission(I2C_DEV_ADDR);
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
  }
}

std::array<float, 4> calc_speed_2_motor(float x_dot, float y_dot, float theta_dot)
{

  std::array<float, 4> speeds;

  speeds[0] = (1 / r) * (theta_dot * (-l - w) + x_dot - y_dot);
  speeds[1] = (1 / r) * (theta_dot * (l + w) + x_dot + y_dot);
  speeds[2] = (1 / r) * (theta_dot * (l + w) + x_dot - y_dot);
  speeds[3] = (1 / r) * (theta_dot * (-l - w) + x_dot + y_dot);

  return speeds;
}

void send_speed_2_drivers(float *goal_velocity_motor)
{
  // Enviar velocidades para drivers

  // Definir qual será o driver responsavel por u1 e u2, e qual sera responsavel por u3 e u4
}
