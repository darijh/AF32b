// Handles globales
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  // HSE ON, PLL = HSE ×6 → 48 MHz
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  // Selecciona PLL como SYSCLK, prescalers AHB/1, APB1/2, APB2/1
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);

  // Ajuste USB prescaler para 48 MHz USB
  RCC->CFGR |= RCC_CFGR_USBPRE;

  // SysTick cada 1 ms
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  // Prioridad SysTick_IRQn (nivel más bajo)
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  // Habilita relojes de GPIOA, GPIOB y GPIOC
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  // Remapea SWJ: deshabilita JTAG, habilita SWD
  __HAL_AFIO_REMAP_SWJ_NOJTAG();

  // Pines analógicos (ADC1_IN0, ADC1_IN1, ADC1_IN4, ADC2_IN0)
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct); // PA0, PA1, PA4
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  HAL_GPIO_Init(GPIOB,
                &GPIO_InitStruct); // PB0 :contentReference[oaicite:1]{index=1}

  // AF_PP para PWM: TIM3_CH1/CH2 (PA6, PA7), TIM1_CH1/CH2 (PA8, PA9)
  GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA,
                &GPIO_InitStruct); // PA6, PA7, PA8, PA9
                                   // :contentReference[oaicite:4]{index=4}
  pinMode(HW_ID_B3, INPUT_PULLUP);
  pinMode(HW_ID_B2, INPUT_PULLUP);
  pinMode(HW_ID_B1, INPUT_PULLUP);
  pinMode(HW_ID_B0, INPUT_PULLUP);
  pinMode(HW_CTRL, INPUT_PULLUP);
  pinMode(HW_STS, OUTPUT);
  pinMode(HW_OK, OUTPUT);
  pinMode(HW_FAIL, OUTPUT);
#if defined(CALIBRACION) || defined(DEBUG)
  DBG_PORT.begin(); // inicializa USB CDC
  while (!DBG_PORT)
    ;
#endif
}

void MX_TIM1_Init(void)
{
  TIM_OC_InitTypeDef sConfigOC = {0};

  // Habilita reloj TIM1
  __HAL_RCC_TIM1_CLK_ENABLE();

  // Configura instancia y parámetros básicos
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 4095;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  HAL_TIM_PWM_Init(&htim1);

  // Configura canal 1 en PWM1
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0; // duty=0% al inicio
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1);

  // Configura canal 2 en PWM1
  HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2);

  // Arranca PWM en TIM1 canales 1 y 2
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
}

void MX_TIM3_Init(void)
{
  TIM_OC_InitTypeDef sConfigOC = {0};

  // Habilita reloj TIM3
  __HAL_RCC_TIM3_CLK_ENABLE();

  // Configura instancia y parámetros básicos
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 4095;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  HAL_TIM_PWM_Init(&htim3);

  // Configura canal 1 en PWM1
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);

  // Configura canal 2 en PWM1
  HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2);

  // Arranca PWM en TIM3 canales 1 y 2
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
}

void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  /** Configuración común del ADC1 */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE; // Desactiva el modo escaneo
  hadc1.Init.ContinuousConvMode = DISABLE;    // Modo de conversión única
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START; // Conversión por software
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1; // Solo un canal por conversión
  HAL_ADC_Init(&hadc1);

  HAL_ADCEx_Calibration_Start(&hadc1); // Calibración de ADC1
}

void MX_ADC2_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  /** Configuración común del ADC2 */
  hadc2.Instance = ADC2;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE; // Desactiva el modo escaneo
  hadc2.Init.ContinuousConvMode = DISABLE;    // Modo de conversión única
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START; // Conversión por software
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1; // Solo un canal por conversión
  HAL_ADC_Init(&hadc2);

  HAL_ADCEx_Calibration_Start(&hadc2); // Calibración de ADC2
}