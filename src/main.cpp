#include <Arduino.h>

// CLOUD LIBS
#include <AinHandler.h>
#include <SlaveRtu.h>
#include <Horometer.h>
#include <EepromManager.h>

// LOCAL LIBS
#include <PowerController.h>
#include <PeriodicTask.h>

// CONSTANTS
#include <hardware.h>
#include <regmap.h>
#include <serial_number.h>

// GLOBAL VARIABLES
int16_t regs[REGS_SIZE]; // Array de registros

// OBJECTS
PowerController regulator_a(VKp, VKi, VKd, IKp, IKi, IKd);
PowerController regulator_b(VKp, VKi, VKd, IKp, IKi, IKd);
Slave<int16_t, HardwareSerial> modbus_slave;
AinHandler vout_a, iout_a, vout_b, iout_b;
HardwareSerial Serial3(PB11, PB10); // RX = PB11, TX = PB10
EepromManager eepr(eeprom_manager::NATIVE);
Horometer hor_a, hor_b;
PeriodicTask one_second_task(1000);  // Tarea periódica de 1 segundo
PeriodicTask one_hour_task(3600000); // Tarea periódica de 1 hora
PeriodicTask one_day_task(86400000); // Tarea periódica de 1 día

// CODE
#include <hardware_config.h>
#include <core.h>

void setup()
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  CoreInit();
  Config();
}

void loop()
{
  modbus_slave.Poll(regs, REGS_SIZE);
  ADCReadings();
  Regulation();
  digitalWrite(HW_STS, modbus_slave.active);
  digitalWrite(HW_FAIL, (regs[MB_ALARM]) ? HIGH : LOW);
  digitalWrite(HW_OK, (regs[MB_ALARM]) ? LOW : HIGH);
}