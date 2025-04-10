#include <Arduino.h>
#include <EEPROM.h>

#include <hardware.h>
#include <regmap.h>

uint16_t duty(uint8_t duty) { return map(duty, 0, 100, 0, 4095); }

void setup() {
  analogReadResolution(12);
  analogWriteResolution(12);
  pinMode(HW_ID_B0, INPUT_PULLUP);
  pinMode(HW_ID_B1, INPUT_PULLUP);
  pinMode(HW_ID_B2, INPUT_PULLUP);
  pinMode(HW_ID_B3, INPUT_PULLUP);
  pinMode(HW_CTRL, INPUT_PULLUP);
  pinMode(HW_SPV_A, OUTPUT);
  pinMode(HW_SPI_A, OUTPUT);
  pinMode(HW_SPV_B, OUTPUT);
  pinMode(HW_SPI_B, OUTPUT);
  pinMode(HW_STS, OUTPUT);
  pinMode(HW_OK, OUTPUT);
  pinMode(HW_FAIL, OUTPUT);
  DBG_PORT.begin(115200);
  delay(5000);
  analogWrite(HW_SPV_A, duty(20));
  analogWrite(HW_SPI_A, duty(40));
  analogWrite(HW_SPV_B, duty(60));
  analogWrite(HW_SPI_B, duty(80));
}

void loop() {}