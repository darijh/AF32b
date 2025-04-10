#include <Arduino.h>
#include <hardware.h>
#include <regmap.h>

void setup()
{
  analogWriteResolution(12);
  pinMode(HW_STS, OUTPUT);
  pinMode(HW_SPV_A, OUTPUT);
  pinMode(HW_SPI_A, OUTPUT);
  pinMode(HW_SPV_B, OUTPUT);
  pinMode(HW_SPI_B, OUTPUT);
  pinMode(HW_OK, OUTPUT);
  pinMode(HW_FAIL, OUTPUT);
  DBG_PORT.begin(115200);
  while (!DBG_PORT)
  {
  }
}

void loop() {}