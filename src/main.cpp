#include <Arduino.h>
#include <hardware.h>
#include <regmap.h>

void setup() {
  pinMode(HW_STS, OUTPUT);
  DBG_PORT.begin(115200);
  while (!DBG_PORT) {
  }
}

void loop() {}