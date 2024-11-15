#include "Arduino.h"
#include "Wire.h"
#include "Mestre.hpp"

#define I2C_DEV_ADDR 0x55

uint32_t i = 0;

// Encontrar constantes (tinham sido declaradas como uint32_t??)
float l = 10;
float w = 10;
float r = 3.5;

float u1 = 0;
float u2 = 0;
float u3 = 0;
float u4 = 0;
std::array<float, 4> speeds_result;

void detectKeyPressAndSend() {
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');  // Lê até uma nova linha ou ENTER

        input.trim();  // Remove espaços e quebras de linha extras

        // Interpreta o comando
        if (input == "W" || input == "w") {  // Frente
            speeds_result = calc_speed_2_motor(1, 0, 0);
            Serial.println("Comando: Frente");
        } else if (input == "A" || input ==  "a") {  // Esquerda
            speeds_result = calc_speed_2_motor(0, 1, 0);
            Serial.println("Comando: Esquerda");
        } else if (input == "S" || input ==  "s") {  // Ré
            speeds_result = calc_speed_2_motor(-1, 0, 0);
            Serial.println("Comando: Ré");
        } else if (input == "D" || input ==  "d") {  // Direita
            speeds_result = calc_speed_2_motor(0, -1, 0);
            Serial.println("Comando: Direita");
        } else {
            Serial.println("Para");
            speeds_result = calc_speed_2_motor(0, 0, 0);
        }

    }
}

std::array<float, 4> calc_speed_2_motor(float x_dot, float y_dot, float theta_dot)
{
  // Serial.printf("Antes das contas l: %f, w: %f, r: %f, x_dot: %f, y_dot: %f, theta_dot: %f\n", l, w, r, x_dot, y_dot, theta_dot);

  std::array<float, 4> speeds;

  speeds[0] = (1.0f / r) * (theta_dot * (-l - w) + x_dot - y_dot); // u1
  speeds[1] = (1.0f / r) * (theta_dot * (l + w) + x_dot + y_dot);  // u2
  speeds[2] = (1.0f / r) * (theta_dot * (l + w) + x_dot - y_dot);  // u3
  speeds[3] = (1.0f / r) * (theta_dot * (-l - w) + x_dot + y_dot); // u4

  // Serial.printf("Depois da conta l: %f, w: %f, r: %f, x_dot: %f, y_dot: %f, theta_dot: %f\n", l, w, r, x_dot, y_dot, theta_dot);

  // Serial.printf("u1 (antes): %f, u2 (antes): %f\n", speeds[0], speeds[1]);

  return speeds;
}

void setup()
{
  Mestre::setup();
  // Inicia comunicação I2C
  // TODO: Fazer setup do WiFi
}

void loop()
{
  // TODO: Remover delay
  delay(5000);

  detectKeyPressAndSend();
  u1 = speeds_result[0];
  u2 = speeds_result[1];
  u3 = speeds_result[2];
  u4 = speeds_result[3];

  Mestre::send_speed_2_driver(addr_driver1, u1, u2);
  Mestre::send_speed_2_driver(addr_driver2, u3, u4);

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
  float x_dot = 0.0;
  float y_dot = 0.0;
  float theta_dot = 0.0;

  // supostamente faz um quadrado
  float dir_x[] = {1,
                   0,
                   -1,
                   0};

  float dir_y[] = {0,
                   1,
                   0,
                   -1};

  for (int i = 0; i < 4; i++)
  {
    std::array<float, 4> speeds_result = calc_speed_2_motor(dir_x[i], dir_y[i], theta_dot);
    float u1 = speeds_result[0];
    float u2 = speeds_result[1];
    float u3 = speeds_result[2];
    float u4 = speeds_result[3];

    Mestre::send_speed_2_driver(addr_driver1, u1, u2);
    Mestre::send_speed_2_driver(addr_driver2, u3, u4);

    delay(1500);
  }

  // Serial.printf("x_dot: %f, y_dot: %f, theta_dot: %f\n", x_dot, y_dot, theta_dot);
  // Serial.printf("u1 (depois): %f, u2 (depois): %f\n", u1, u2);
}
