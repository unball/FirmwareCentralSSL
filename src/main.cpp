#include "Arduino.h"
#include "Wire.h"
#include "MedicaoBateria.hpp"

void setup()
{
  Serial.begin(115200);
  Serial.println("Setup");
}

void loop()
{

  // Le bateria a cada X ms
  // Envia bateria pro driver -> se n teve atualização no valor, envia valor passado
  // Se bateria < 96% => acende LED pra alertar bateria baixa
}
