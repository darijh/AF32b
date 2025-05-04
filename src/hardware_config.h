/**
 * @brief Configura el reloj del sistema.
 *
 * Configura el oscilador HSE, habilita el PLL y ajusta los prescalers para
 * generar una frecuencia de 48 MHz. También configura el SysTick para
 * interrupciones de 1 ms.
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0}; // Estructura para la configuración del oscilador
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0}; // Estructura para la configuración del reloj

  // Configuración del oscilador HSE y PLL
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE; // Tipo de oscilador HSE
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;                   // HSE encendido
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;    // Predivisor HSE = 1
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;               // Habilita PLL
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;       // Fuente PLL = HSE
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;               // Multiplicador PLL = 6
  HAL_RCC_OscConfig(&RCC_OscInitStruct);                     // Configura el oscilador

  // Configuración del reloj del sistema
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2; // Tipos de reloj
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;                // Fuente SYSCLK = PLL
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;                       // Prescaler AHB = 1
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;                        // Prescaler APB1 = 2
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;                        // Prescaler APB2 = 1
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);                // Configura el reloj

  // Configuración adicional para USB y SysTick
  RCC->CFGR |= RCC_CFGR_USBPRE;                        // Ajuste USB prescaler para 48 MHz USB
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);    // Configura SysTick a 1 ms
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK); // Fuente SysTick = HCLK
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);           // Prioridad SysTick_IRQn (nivel más bajo)
}

/**
 * @brief Configura los pines GPIO.
 *
 * Inicializa los pines GPIO necesarios para el funcionamiento del sistema,
 * incluyendo entradas analógicas, salidas digitales y pines de control.
 */
void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0}; // Estructura para la configuración de GPIO

  // Habilita los relojes para los puertos GPIO
  __HAL_RCC_GPIOA_CLK_ENABLE(); // Habilita reloj GPIOA
  __HAL_RCC_GPIOB_CLK_ENABLE(); // Habilita reloj GPIOB
  __HAL_RCC_GPIOC_CLK_ENABLE(); // Habilita reloj GPIOC

  // Configuración de pines analógicos
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4; // PA0, PA1, PA4
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;                    // Modo analógico
  GPIO_InitStruct.Pull = GPIO_NOPULL;                         // Sin resistencia de pull-up/pull-down
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);                     // Inicializa GPIOA

  GPIO_InitStruct.Pin = GPIO_PIN_0;       // PB0: ADC2_IN0
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct); // Inicializa GPIOB (PB0: ADC2_IN0)

  // Configuración de pines en modo alternativo
  GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9; // PA6, PA7, PA8, PA9
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;                                  // Modo alternativo push-pull
  GPIO_InitStruct.Pull = GPIO_NOPULL;                                      // Sin resistencia de pull-up/pull-down
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;                             // Velocidad baja
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);                                  // Inicializa GPIOA (PA6, PA7, PA8, PA9)

  // Configuración de pines digitales
  pinMode(HW_ID_B3, INPUT_PULLUP); // PB3: ID_B3 (entrada con pull-up)
  pinMode(HW_ID_B2, INPUT_PULLUP); // PB2: ID_B2 (entrada con pull-up)
  pinMode(HW_ID_B1, INPUT_PULLUP); // PB1: ID_B1 (entrada con pull-up)
  pinMode(HW_ID_B0, INPUT_PULLUP); // PB0: ID_B0 (entrada con pull-up)
  pinMode(HW_CTRL, INPUT_PULLUP);  // PA0: CTRL (entrada con pull-up)
  pinMode(HW_STS, OUTPUT);         // PA1: STS (salida)
  pinMode(HW_OK, OUTPUT);          // PA2: OK (salida)
  pinMode(HW_FAIL, OUTPUT);        // PA3: FAIL (salida)

  // Configuración para depuración
#if defined(CALIBRACION) || defined(DEBUG)
  DBG_PORT.begin(); // Inicializa USB CDC para depuración
#endif
}

/**
 * @brief Configura el temporizador TIM1 para PWM.
 *
 * Inicializa el temporizador TIM1 con un periodo de 12 bits (4095) y configura
 * dos canales para generar señales PWM.
 */
void MX_TIM1_Init(void)
{
  TIM_OC_InitTypeDef sConfigOC = {0}; // Estructura para la configuración de salida de TIM

  __HAL_RCC_TIM1_CLK_ENABLE(); // Habilita reloj TIM1

  // Configuración básica del temporizador
  htim1.Instance = TIM1;                                        // Instancia TIM1
  htim1.Init.Prescaler = 0;                                     // Prescaler = 0 (sin división)
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;                  // Modo de conteo ascendente
  htim1.Init.Period = 4095;                                     // Periodo = 4095 (12 bits)
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;            // División de reloj = 1
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE; // Pre-carga automática habilitada
  HAL_TIM_PWM_Init(&htim1);                                     // Inicializa TIM1 para PWM

  // Configuración de los canales PWM
  sConfigOC.OCMode = TIM_OCMODE_PWM1;                           // Modo PWM1
  sConfigOC.Pulse = 0;                                          // Pulso inicial = 0
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;                   // Polaridad alta
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;                    // Modo rápido deshabilitado
  HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1); // Configura canal 1
  HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2); // Configura canal 2

  // Inicia los canales PWM
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); // Inicia PWM en canal 1
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2); // Inicia PWM en canal 2
}

/**
 * @brief Configura el temporizador TIM3 para PWM.
 *
 * Inicializa el temporizador TIM3 con un periodo de 12 bits (4095) y configura
 * dos canales para generar señales PWM.
 */
void MX_TIM3_Init(void)
{
  TIM_OC_InitTypeDef sConfigOC = {0}; // Estructura para la configuración de salida de TIM

  __HAL_RCC_TIM3_CLK_ENABLE(); // Habilita el reloj para TIM3

  // Configuración básica del temporizador
  htim3.Instance = TIM3;                                        // Instancia TIM3
  htim3.Init.Prescaler = 0;                                     // Prescaler = 0 (sin división)
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;                  // Modo de conteo ascendente
  htim3.Init.Period = 4095;                                     // Periodo = 4095 (12 bits)
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;            // División de reloj = 1
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE; // Pre-carga automática habilitada
  HAL_TIM_PWM_Init(&htim3);                                     // Inicializa TIM3 para PWM

  // Configuración de los canales PWM
  sConfigOC.OCMode = TIM_OCMODE_PWM1;                           // Modo PWM1
  sConfigOC.Pulse = 0;                                          // Pulso inicial = 0
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;                   // Polaridad alta
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;                    // Modo rápido deshabilitado
  HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1); // Configura canal 1
  HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2); // Configura canal 2

  // Inicia los canales PWM
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); // Inicia PWM en canal 1
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); // Inicia PWM en canal 2
}

/**
 * @brief Configura el ADC1.
 *
 * Inicializa el ADC1 para realizar conversiones de un solo canal con
 * alineación de datos a la derecha.
 */
void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0}; // Estructura de configuración del ADC

  hadc1.Instance = ADC1;                            // Instancia ADC1
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;       // Desactiva el modo escaneo
  hadc1.Init.ContinuousConvMode = DISABLE;          // Modo de conversión única
  hadc1.Init.DiscontinuousConvMode = DISABLE;       // Desactiva el modo de conversión discontinua
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START; // Conversión por software
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;       // Alineación de datos a la derecha
  hadc1.Init.NbrOfConversion = 1;                   // Solo un canal por conversión
  HAL_ADC_Init(&hadc1);                             // Inicializa ADC1
  HAL_ADCEx_Calibration_Start(&hadc1);              // Calibración de ADC1
}

/**
 * @brief Configura el ADC2.
 *
 * Inicializa el ADC2 para realizar conversiones de un solo canal con
 * alineación de datos a la derecha.
 */
void MX_ADC2_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};             // Estructura de configuración del ADC
  hadc2.Instance = ADC2;                            // Instancia ADC2
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;       // Desactiva el modo escaneo
  hadc2.Init.ContinuousConvMode = DISABLE;          // Modo de conversión única
  hadc2.Init.DiscontinuousConvMode = DISABLE;       // Desactiva el modo de conversión discontinua
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START; // Conversión por software
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;       // Alineación de datos a la derecha
  hadc2.Init.NbrOfConversion = 1;                   // Solo un canal por conversión
  HAL_ADC_Init(&hadc2);                             // Inicializa ADC2
  HAL_ADCEx_Calibration_Start(&hadc2);              // Calibración de ADC2
}