
#include <Arduino.h>

#include "stm32f1xx_hal.h"
#include <AinHandler.h>

#include <hardware.h>
#include <hardware_config.h>
#include <regmap.h>
#include <serial_number.h>

#include <core.h>
namespace asciistatus {
enum status { D = 68, H = 72, L = 76, N = 78 };
};
uint16_t regs[REGS_SIZE]; // Array de registros
AinHandler vout_a, iout_a, vout_b, iout_b;
void Config(); // Prototipo de la función de configuración

void setup() {
  HAL_Init();           // Inicializa HAL y SysTick
  SystemClock_Config(); // Configura relojes según CubeMX
  DBG_PORT.begin();     // inicializa USB CDC
  while (!DBG_PORT)
    ;
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  HAL_ADCEx_Calibration_Start(&hadc1); // Calibración de ADC1
  HAL_ADCEx_Calibration_Start(&hadc2); // Calibración de ADC2
  // Arranca PWM en TIM1 canales 1 y 2
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  // Arranca PWM en TIM3 canales 1 y 2
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  Config();
}

void loop() {
  vout_a.Sample(
      Read_ADC_Channel_Raw(&hadc2, ADC_CHANNEL_0, ADC_SAMPLETIME_28CYCLES_5));
  iout_a.Sample(
      Read_ADC_Channel_Raw(&hadc2, ADC_CHANNEL_1, ADC_SAMPLETIME_28CYCLES_5));
  vout_b.Sample(
      Read_ADC_Channel_Raw(&hadc2, ADC_CHANNEL_4, ADC_SAMPLETIME_28CYCLES_5));
  iout_b.Sample(
      Read_ADC_Channel_Raw(&hadc2, ADC_CHANNEL_8, ADC_SAMPLETIME_28CYCLES_5));
  vout_a.Run();
  vout_b.Run();
  iout_a.Run();
  iout_b.Run();

  char estado;
  static uint32_t ml_alarm = 0;

  // vout_a
  regs[MB_V_VAL_A] = max(vout_a.GetEU_AVG(), (double)0);
  estado = vout_a.GetStatus(regs[MB_V_VAL_A]);
  if (estado == asciistatus::H || estado == asciistatus::L) {
    ml_alarm = millis();
    bitSet(regs[MB_ALARM], VOUT_A_BIT);
    regs[MB_V_STS_A] = int(estado);
  } else {
    if (bitRead(regs[MB_ALARM], VOUT_A_BIT)) {
      if (millis() > ml_alarm + 1000) {
        bitClear(regs[MB_ALARM], VOUT_A_BIT);
        regs[MB_V_STS_A] = int(estado);
      }
    } else {
      regs[MB_V_STS_A] = int(estado);
    }
  }

  // iout_a
  regs[MB_I_VAL_A] = max(iout_a.GetEU_AVG(), (double)0);
  estado = iout_a.GetStatus(regs[MB_I_VAL_A]);
  if (estado == asciistatus::H || estado == asciistatus::L) {
    ml_alarm = millis();
    bitSet(regs[MB_ALARM], IOUT_A_BIT);
    regs[MB_I_STS_A] = int(estado);
  } else {
    if (bitRead(regs[MB_ALARM], IOUT_A_BIT)) {
      if (millis() > ml_alarm + 1000) {
        bitClear(regs[MB_ALARM], IOUT_A_BIT);
        regs[MB_I_STS_A] = int(estado);
      }
    } else {
      regs[MB_I_STS_A] = int(estado);
    }
  }

  // vout_b
  regs[MB_V_VAL_B] = max(vout_b.GetEU_AVG(), (double)0);
  estado = vout_b.GetStatus(regs[MB_V_VAL_B]);
  if (estado == asciistatus::H || estado == asciistatus::L) {
    ml_alarm = millis();
    bitSet(regs[MB_ALARM], VOUT_B_BIT);
    regs[MB_V_STS_B] = int(estado);
  } else {
    if (bitRead(regs[MB_ALARM], VOUT_B_BIT)) {
      if (millis() > ml_alarm + 1000) {
        bitClear(regs[MB_ALARM], VOUT_B_BIT);
        regs[MB_V_STS_B] = int(estado);
      }
    } else {
      regs[MB_V_STS_B] = int(estado);
    }
  }

  // iout_b
  regs[MB_I_VAL_B] = max(iout_b.GetEU_AVG(), (double)0);
  estado = iout_b.GetStatus(regs[MB_I_VAL_B]);
  if (estado == asciistatus::H || estado == asciistatus::L) {
    ml_alarm = millis();
    bitSet(regs[MB_ALARM], IOUT_B_BIT);
    regs[MB_I_STS_B] = int(estado);
  } else {
    if (bitRead(regs[MB_ALARM], IOUT_B_BIT)) {
      if (millis() > ml_alarm + 1000) {
        bitClear(regs[MB_ALARM], IOUT_B_BIT);
        regs[MB_I_STS_B] = int(estado);
      }
    } else {
      regs[MB_I_STS_B] = int(estado);
    }
  }
  static uint32_t ml = 0;
  if (millis() > ml + 1000) {
    ml = millis();
    DBG_PORT.print("Vout_a: ");
    DBG_PORT.print(vout_a.GetEU_AVG(), 2);
    DBG_PORT.print(" Iout_a: ");
    DBG_PORT.print(iout_a.GetEU_AVG(), 2);
    DBG_PORT.print(" Vout_b: ");
    DBG_PORT.print(vout_b.GetEU_AVG(), 2);
    DBG_PORT.print(" Iout_b: ");
    DBG_PORT.println(iout_b.GetEU_AVG(), 2);
  }
}

void Config() {
  for (uint8_t i = 0; i < REGS_SIZE; i++) {
    regs[i] = 0;
  }
  uint8_t id = Calculate_ID();
  uint8_t duty = 10;
  // funciones de pwm pendientes
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, Set_Duty(duty));
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, Set_Duty(duty));
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, Set_Duty(duty));
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, Set_Duty(duty));
  //

  // Vout_a
  vout_a.SetReads(READS);
  vout_a.SetEnable(true);
  vout_a.SetHiLimitEnable(true);
  vout_a.SetLoLimitEnable(true);
  vout_a.SetHiLimit(regs[MB_VHL_A]);
  vout_a.SetLoLimit(regs[MB_VLL_A]);
  vout_a.SetConstants(VOUT_MULT_A, VOUT_OFFSET_A);

  // Iout_a
  iout_a.SetReads(READS);
  iout_a.SetEnable(true);
  iout_a.SetHiLimitEnable(true);
  iout_a.SetLoLimitEnable(true);
  iout_a.SetHiLimit(regs[MB_IHL_A]);
  iout_a.SetLoLimit(regs[MB_ILL_A]);
  iout_a.SetConstants(IOUT_MULT_A, IOUT_OFFSET_A);

  // Vout_b
  vout_b.SetReads(READS);
  vout_b.SetEnable(true);
  vout_b.SetHiLimitEnable(true);
  vout_b.SetLoLimitEnable(true);
  vout_b.SetHiLimit(regs[MB_VHL_B]);
  vout_b.SetLoLimit(regs[MB_VLL_B]);
  vout_b.SetConstants(VOUT_MULT_B, VOUT_OFFSET_B);

  // Iout_b
  iout_b.SetReads(READS);
  iout_b.SetEnable(true);
  iout_b.SetHiLimitEnable(true);
  iout_b.SetLoLimitEnable(true);
  iout_b.SetHiLimit(regs[MB_IHL_B]);
  iout_b.SetLoLimit(regs[MB_ILL_B]);
  iout_b.SetConstants(IOUT_MULT_B, IOUT_OFFSET_B);
}