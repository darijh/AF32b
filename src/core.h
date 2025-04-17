// Función para establecer el duty cycle en porcentaje
uint16_t Set_Duty(float duty_percent)
{
  return map(duty_percent, 0, 100, 0,
             4095); // Mapea el porcentaje a un valor entre 0 y 4095
}

uint8_t Get_ID()
{
  uint8_t id = 0;

  // Leer el estado de los pines y construir el ID en binario
  if (!digitalRead(HW_ID_B0))
  {
    id |= (1 << 0);
  }
  if (!digitalRead(HW_ID_B1))
  {
    id |= (1 << 1);
  }
  if (!digitalRead(HW_ID_B2))
  {
    id |= (1 << 2);
  }
  if (!digitalRead(HW_ID_B3))
  {
    id |= (1 << 3);
  }

  return id;
}

uint16_t Read_ADC_Channel_Raw(ADC_HandleTypeDef *hadc, uint32_t channel,
                              uint32_t samplingTime)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  // Configura el canal y el tiempo de muestreo
  sConfig.Channel = channel;
  sConfig.Rank = ADC_REGULAR_RANK_1; // Siempre el primer rank
  sConfig.SamplingTime = samplingTime;
  if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK)
  {
    // Manejo de error
    return 0;
  }

  // Inicia la conversión
  HAL_ADC_Start(hadc);

  // Espera a que la conversión termine
  HAL_ADC_PollForConversion(hadc, HAL_MAX_DELAY);

  // Lee el valor convertido
  uint16_t adc_value = HAL_ADC_GetValue(hadc);

  // Detiene el ADC
  HAL_ADC_Stop(hadc);
  return adc_value;
}

float Read_VDDA()
{
  uint16_t adc_value = Read_ADC_Channel_Raw(&hadc1, ADC_CHANNEL_VREFINT,
                                            ADC_SAMPLETIME_239CYCLES_5);
  float vdda = 4095 * 1200.0f / adc_value;
  return vdda;
}

float Read_ADC(uint32_t channel)
{
  uint16_t adc_value =
      Read_ADC_Channel_Raw(&hadc2, channel, ADC_SAMPLETIME_28CYCLES_5);
  float val = adc_value * Read_VDDA() / 4095.0f; // Convierte a mV
  return val;
}

float Read_Temp()
{
  uint16_t adc_value = Read_ADC_Channel_Raw(&hadc1, ADC_CHANNEL_TEMPSENSOR,
                                            ADC_SAMPLETIME_239CYCLES_5);
  float temp = 25 + (1430 - (adc_value * Read_VDDA() / 4095.0f)) / 4.3f;
  return temp;
}

void Set_Out_A()
{

  uint16_t duty_a = PID_A.compute(vout_a.GetEU_AVG(), regs[MB_VSET_A]);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, Set_Duty(duty_a)); // VSET_A
  static uint32_t ml = 0;
  if (millis() > ml + 1000)
  {
    ml = millis();
    Serial.print("Vout_a: ");
    Serial.print(vout_a.GetEU_AVG() / 100.0f, 2);
    Serial.print(" V | Iout_a: ");
    Serial.print(iout_a.GetEU_AVG() / 100.0f, 2);
    Serial.print(" A | Vset_a: ");
    Serial.print(regs[MB_VSET_A] / 100.0f, 2);
    Serial.print(" V | PWM: ");
    Serial.print(duty_a);
    Serial.print(" | Saturado: ");
    Serial.println(PID_A.isSaturated() ? "SI" : "NO");
  }
}

void Modbus_Listener()
{
  modbus_slave.ChangesProcessed();
}

void Config()
{
  DBG_PORT.begin(); // inicializa USB CDC
  while (!DBG_PORT)
    ;
  HAL_ADCEx_Calibration_Start(&hadc1); // Calibración de ADC1
  HAL_ADCEx_Calibration_Start(&hadc2); // Calibración de ADC2
  // Arranca PWM en TIM1 canales 1 y 2
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  // Arranca PWM en TIM3 canales 1 y 2
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

  for (uint8_t i = 0; i < REGS_SIZE; i++)
  {
    regs[i] = 0;
  }
  modbus_slave.Enable(true);
  modbus_slave.Begin(&COM_PORT, BAUDRATE, Get_ID());
  modbus_slave.SetWriteListener(Modbus_Listener);

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