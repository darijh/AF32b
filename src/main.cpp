#include <Arduino.h>

#include "stm32f1xx_hal.h"
#include <hardware.h>
#include <hardware_config.h>

#include <core.h>

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
  // funciones de pwm pendientes
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, Set_Duty(10));
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, Set_Duty(10));
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, Set_Duty(10));
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, Set_Duty(10));
  //
  uint8_t id = Calculate_ID();
}

void loop() {}
