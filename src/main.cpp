#include <Arduino.h>
#include <EEPROM.h>

#include <hardware.h>
#include <regmap.h>

void setupPWM_TIM1_TIM3(uint8_t resolutionBits, uint32_t freqHz) {
  if (resolutionBits > 16)
    resolutionBits = 16;

  // Habilitar clocks
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_TIM1EN;
  RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

  // Configurar PA6 y PA7 (TIM3_CH1 y CH2)
  GPIOA->CRL &= ~(GPIO_CRL_CNF6 | GPIO_CRL_CNF7);
  GPIOA->CRL |=
      (GPIO_CRL_CNF6_1 | GPIO_CRL_MODE6 | GPIO_CRL_CNF7_1 | GPIO_CRL_MODE7);

  // Configurar PA8 y PA9 (TIM1_CH1 y CH2)
  GPIOA->CRH &= ~(GPIO_CRH_CNF8 | GPIO_CRH_CNF9);
  GPIOA->CRH |=
      (GPIO_CRH_CNF8_1 | GPIO_CRH_MODE8 | GPIO_CRH_CNF9_1 | GPIO_CRH_MODE9);

  // Timer clock del sistema actual
  uint32_t timer_clk = SystemCoreClock;
  uint32_t arr = (1UL << resolutionBits) - 1;

  // Calcular prescaler con redondeo
  float idealPSC = (float)timer_clk / (freqHz * (arr + 1)) - 1;
  uint16_t prescaler = (uint16_t)(idealPSC + 0.5f); // Redondeo

  // Configurar TIM3
  TIM3->PSC = prescaler;
  TIM3->ARR = arr;
  TIM3->CCMR1 = 0;
  TIM3->CCMR1 |= (6 << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE;
  TIM3->CCMR1 |= (6 << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;
  TIM3->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E;
  TIM3->CR1 |= TIM_CR1_ARPE | TIM_CR1_CEN;
  TIM3->CCR1 = 0;
  TIM3->CCR2 = 0;

  // Configurar TIM1
  TIM1->PSC = prescaler;
  TIM1->ARR = arr;
  TIM1->CCMR1 = 0;
  TIM1->CCMR1 |= (6 << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE;
  TIM1->CCMR1 |= (6 << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;
  TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E;
  TIM1->BDTR |= TIM_BDTR_MOE; // Habilitar salidas principales
  TIM1->CR1 |= TIM_CR1_ARPE | TIM_CR1_CEN;
  TIM1->CCR1 = 0;
  TIM1->CCR2 = 0;
}

void setDuty(uint8_t pin, float porcentaje) {
  if (porcentaje < 0)
    porcentaje = 0;
  if (porcentaje > 100)
    porcentaje = 100;

  uint32_t arr = TIM3->ARR; // ARR es igual en todos los timers configurados
  uint32_t valor = (porcentaje / 100.0f) * arr;

  switch (pin) {
  case PA6:
    TIM3->CCR1 = valor;
    break;
  case PA7:
    TIM3->CCR2 = valor;
    break;
  case PA8:
    TIM1->CCR1 = valor;
    break;
  case PA9:
    TIM1->CCR2 = valor;
    break;
  default:
    // Pin no válido
    break;
  }
}

void setup() {
  /*   Serial.begin(115200);
    while (!Serial)
      ; // Esperar a que abra el puerto
    delay(5000); */
  pinMode(HW_ID_B0, INPUT_PULLUP);
  pinMode(HW_ID_B1, INPUT_PULLUP);
  pinMode(HW_ID_B2, INPUT_PULLUP);
  pinMode(HW_ID_B3, INPUT_PULLUP);
  pinMode(HW_CTRL, INPUT_PULLUP);
  pinMode(HW_STS, OUTPUT);
  pinMode(HW_OK, OUTPUT);
  pinMode(HW_FAIL, OUTPUT);
  setupPWM_TIM1_TIM3(12, 10000); // 12 bits, 1 kHz
  setDuty(PA6, 10);
  setDuty(PA7, 30);
  setDuty(PA8, 60);
  setDuty(PA9, 90);
}

void loop() {}