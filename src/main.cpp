
#include <Arduino.h>

#include "stm32f1xx_hal.h"
#include <AinHandler.h>
#include <PID.h>

#include <hardware.h>
#include <hardware_config.h>
#include <regmap.h>
#include <serial_number.h>

#include <core.h>
namespace asciistatus {
enum status { D = 68, H = 72, L = 76, N = 78 };
};

#define Kp 2.4
#define Ki 9.6
#define Kd 0.15
PID PID_A(Kp, Ki, Kd, false, 0, 4095, 10);

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
#ifdef CALIBRACION
  if (DBG_PORT.available()) {
    String input = DBG_PORT.readStringUntil('\n');
    input.trim();
    int newDuty = input.toInt();
    if (newDuty >= 0 && newDuty <= 100) // Valid range for duty cycle
    {
      uint8_t duty = static_cast<uint8_t>(newDuty);
      // funciones de pwm pendientes
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, Set_Duty(duty)); // VSET_A
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, Set_Duty(duty)); // ISET_A
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, Set_Duty(duty)); // VSET_B
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, Set_Duty(duty)); // ISET_B
      //
      DBG_PORT.print("Duty updated to: ");
      DBG_PORT.println(duty);
    } else {
      DBG_PORT.println(
          "Invalid duty value. Please enter a value between 0 and 100.");
    }
  }
  static uint32_t ml = 0;
  if (millis() > ml + 1000) {
    ml = millis();
    /*     DBG_PORT.print("Vout_a: ");
        DBG_PORT.print(vout_a.GetADC_AVG());
        DBG_PORT.print(" Iout_a: ");
        DBG_PORT.print(iout_a.GetADC_AVG());
        DBG_PORT.print(" Vout_b: ");
        DBG_PORT.print(vout_b.GetADC_AVG());
        DBG_PORT.print(" Iout_b: ");
        DBG_PORT.println(iout_b.GetADC_AVG()); */
    DBG_PORT.print("Vout_a: ");
    DBG_PORT.print(regs[MB_V_VAL_A] / 100.0);
    DBG_PORT.print(" Iout_a: ");
    DBG_PORT.print(regs[MB_I_VAL_A]);
    DBG_PORT.print(" Vout_b: ");
    DBG_PORT.print(regs[MB_V_VAL_B] / 100.0);
    DBG_PORT.print(" Iout_b: ");
    DBG_PORT.println(regs[MB_I_VAL_B]);
  }
#endif
}

void Config() {
  for (uint8_t i = 0; i < REGS_SIZE; i++) {
    regs[i] = 0;
  }
  uint8_t id = Calculate_ID();

  // Vout_a
  vout_a.SetReads(READS);
  vout_a.SetEnable(true);
  vout_a.SetHiLimitEnable(true);
  vout_a.SetLoLimitEnable(true);
  vout_a.SetHiLimit(regs[MB_VHL_A]);
  vout_a.SetLoLimit(regs[MB_VLL_A]);
  vout_a.SetConstants(VOUT_A_MULT, VOUT_A_OFFSET);

  // Iout_a
  iout_a.SetReads(READS);
  iout_a.SetEnable(true);
  iout_a.SetHiLimitEnable(true);
  iout_a.SetLoLimitEnable(true);
  iout_a.SetHiLimit(regs[MB_IHL_A]);
  iout_a.SetLoLimit(regs[MB_ILL_A]);
  iout_a.SetConstants(IOUT_A_MULT, IOUT_A_OFFSET);

  // Vout_b
  vout_b.SetReads(READS);
  vout_b.SetEnable(true);
  vout_b.SetHiLimitEnable(true);
  vout_b.SetLoLimitEnable(true);
  vout_b.SetHiLimit(regs[MB_VHL_B]);
  vout_b.SetLoLimit(regs[MB_VLL_B]);
  vout_b.SetConstants(VOUT_B_MUL, VOUT_B_OFFSET);

  // Iout_b
  iout_b.SetReads(READS);
  iout_b.SetEnable(true);
  iout_b.SetHiLimitEnable(true);
  iout_b.SetLoLimitEnable(true);
  iout_b.SetHiLimit(regs[MB_IHL_B]);
  iout_b.SetLoLimit(regs[MB_ILL_B]);
  iout_b.SetConstants(IOUT_B_MULT, IOUT_B_OFFSET);
}