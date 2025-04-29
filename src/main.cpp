#include <Arduino.h>

// CLOUD LIBS
#include <AinHandler.h>
#include <Horometer.h>
#include <SlaveRtu.h>

// LOCAL LIBS
#include <EepromSTM32.h>
#include <PeriodicTask.h>
#include <PowerController.h>

// CONSTANTS
#include <hardware.h>
#include <regmap.h>
#include <serial_number.h>

// GLOBAL VARIABLES
int16_t regs[REGS_SIZE]; // Array de registros
int16_t run_regs[RW_SIZE];
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
namespace asciistatus
{
  enum status
  {
    D = 68,
    H = 72,
    L = 76,
    N = 78
  };
};

// OBJECTS
HardwareSerial Serial3(PB11, PB10); // RX = PB11, TX = PB10
EepromSTM32 eepr;
Slave<int16_t, HardwareSerial> modbus_slave;
AinHandler vout_a, iout_a, vout_b, iout_b; // AinHandler(READS);
PowerController
    regulator_a(VKp, VKi, VKd, IKp, IKi,
                IKd); // PowerController(VKp, VKi, VKd, IKp, IKi, IKd);
PowerController
    regulator_b(VKp, VKi, VKd, IKp, IKi,
                IKd); // PowerController(VKp, VKi, VKd, IKp, IKi, IKd);
Horometer hor_a, hor_b;
PeriodicTask one_second_task(1000);  // Tarea periódica de 1 segundo
PeriodicTask one_hour_task(3600000); // Tarea periódica de 1 hora
PeriodicTask one_day_task(86400000); // Tarea periódica de 1 día

// CODE
#include <core.h>
#include <hardware_config.h>

void setup()
{
  HAL_Init();           // Inicializa la HAL
  SystemClock_Config(); // Configura el reloj del sistema
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  CoreInit();
}

void loop()
{
  one_second_task.run(millis());      // Ejecuta la tarea de 1 segundo
  one_hour_task.run(millis());        // Ejecuta la tarea de 1 hora
  one_day_task.run(millis());         // Ejecuta la tarea de 1 día
  modbus_slave.Poll(regs, REGS_SIZE); // Polling del Modbus
  ADCReadings();                      // Lee los valores de ADC
}