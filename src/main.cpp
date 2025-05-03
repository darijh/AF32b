#include <Arduino.h> // // Librería de Arduino

// CLOUD LIBS
#include <AinHandler.h> // // Librería para manejar entradas analógicas
#include <Horometer.h>  // // Librería para manejar horómetros
#include <SlaveRtu.h>   // // Librería Modbus RTU para Arduino

// LOCAL LIBS
#include <EepromSTM32.h>  // // Librería para manejar la EEPROM
#include <PeriodicTask.h> // // Librería para manejar tareas periódicas
#include <PID.h>          // // Librería para manejar PID

// CONSTANTS
#include <hardware.h>      // // Constantes de hardware
#include <regmap.h>        // // Mapa de registros Modbus
#include <serial_number.h> // // Ajustes segun el número de serie

// GLOBAL VARIABLES
int16_t regs[REGS_SIZE];   // Array de registros de Modbus
int16_t run_regs[RW_SIZE]; // Array de registros de ejecución
TIM_HandleTypeDef htim1;   // Timer 1 para PWM
TIM_HandleTypeDef htim3;   // Timer 3 para PWM
ADC_HandleTypeDef hadc1;   // ADC1 para Vout_A e Iout_A
ADC_HandleTypeDef hadc2;   // ADC2 para Vout_B e Iout_B
namespace asciistatus      // Definición de los estados ASCII
{
  enum status
  {
    D = 68, // Desconectado
    H = 72, // Alto
    L = 76, // Bajo
    N = 78  // Normal
  };
};

// OBJECTS
HardwareSerial Serial3(PB11, PB10);                                // RX = PB11, TX = PB10
EepromSTM32 eepr;                                                  // Objeto para la EEPROM
Slave<int16_t, HardwareSerial> modbus_slave;                       // Objeto Modbus
AinHandler vout_a, iout_a, vout_b, iout_b;                         // AinHandler(READS);
Horometer hor_a, hor_b;                                            // Objeto Horometro
PeriodicTask one_second_task(1000);                                // Tarea periódica de 1 segundo
PeriodicTask one_hour_task(3600000);                               // Tarea periódica de 1 hora
PeriodicTask one_day_task(86400000);                               // Tarea periódica de 1 día
PID pid_v_a(VKp, VKi, VKd, (READS * READ_INTERVAL * pow(10, -6))); // PID para Vout_A
PID pid_i_a(IKp, Iki, Ikd, (READS * READ_INTERVAL * pow(10, -6))); // PID para Iout_A
PID pid_v_b(VKp, VKi, VKd, (READS * READ_INTERVAL * pow(10, -6))); // PID para Vout_B
PID pid_i_b(IKp, Iki, Ikd, (READS * READ_INTERVAL * pow(10, -6))); // PID para Iout_B

// CORE
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
  MX_GPIO_Init();       // Inicializa los GPIOs
  MX_TIM1_Init();       // Inicializa el timer 1
  MX_TIM3_Init();       // Inicializa el timer 3
  MX_ADC1_Init();       // Inicializa el ADC1
  MX_ADC2_Init();       // Inicializa el ADC2
  CoreInit();           // Inicializa el núcleo
}

void loop()
{
  one_second_task.run(millis());      // Ejecuta la tarea de 1 segundo
  one_hour_task.run(millis());        // Ejecuta la tarea de 1 hora
  one_day_task.run(millis());         // Ejecuta la tarea de 1 día
  modbus_slave.Poll(regs, REGS_SIZE); // Polling del Modbus
  ADCReadings(READ_INTERVAL);         // Lee los ADCs
}