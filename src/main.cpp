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
Horometer hor_a, hor_b;
PeriodicTask one_second_task(1000);                        // Tarea periódica de 1 segundo
PeriodicTask one_hour_task(3600000);                       // Tarea periódica de 1 hora
PeriodicTask one_day_task(86400000);                       // Tarea periódica de 1 día
PowerController regulator_a(VKp, VKi, VKd, IKp, Iki, Ikd); // Regulador A
PowerController regulator_b(VKp, VKi, VKd, IKp, Iki, Ikd); // Regulador B

// CODE
void CoreInit();
void Config();
uint8_t Get_ID(); // Lee el ID del hardware
uint16_t Read_ADC_Channel_Raw(ADC_HandleTypeDef *hadc, uint32_t channel,
                              uint32_t samplingTime);
void HandleAins(AinHandler &ain, int16_t *value_reg, int16_t *status_reg,
                uint8_t alarm_bit, int16_t *alarm_reg,
                uint32_t &ml_alarm); // Maneja los AINs
void ADCReadings(uint32_t interval = 0);
float Read_VDDA();
float Read_Temp();
uint16_t Set_Duty(int16_t duty_percent);
void Regulation();
void Factory_Reset();
void Modbus_Listener();
void OneSecondTask();
void OneHourTask();
void OneDayTask();
void HorACallback();
void HorBCallback();
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
  ADCReadings(READ_INTERVAL);         // Lee los valores de ADC
}