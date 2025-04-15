// Función para establecer el duty cycle en porcentaje
uint16_t Set_Duty(float duty_percent) {
  return map(duty_percent, 0, 100, 0,
             4095); // Mapea el porcentaje a un valor entre 0 y 4095
}

uint8_t Calculate_ID() {
  uint8_t id = 0;

  // Leer el estado de los pines y construir el ID en binario
  if (!digitalRead(HW_ID_B0)) {
    id |= (1 << 0);
  }
  if (!digitalRead(HW_ID_B1)) {
    id |= (1 << 1);
  }
  if (!digitalRead(HW_ID_B2)) {
    id |= (1 << 2);
  }
  if (!digitalRead(HW_ID_B3)) {
    id |= (1 << 3);
  }

  return id;
}

uint16_t Read_ADC_Channel_Raw(ADC_HandleTypeDef *hadc, uint32_t channel,
                              uint32_t samplingTime) {
  ADC_ChannelConfTypeDef sConfig = {0};

  // Configura el canal y el tiempo de muestreo
  sConfig.Channel = channel;
  sConfig.Rank = ADC_REGULAR_RANK_1; // Siempre el primer rank
  sConfig.SamplingTime = samplingTime;
  if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK) {
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

float Read_VDDA() {
  uint16_t adc_value = Read_ADC_Channel_Raw(&hadc1, ADC_CHANNEL_VREFINT,
                                            ADC_SAMPLETIME_239CYCLES_5);
  float vdda = 4095 * 1200.0f / adc_value;
  return vdda;
}

float Read_ADC(uint32_t channel) {
  uint16_t adc_value =
      Read_ADC_Channel_Raw(&hadc2, channel, ADC_SAMPLETIME_28CYCLES_5);
  float val = adc_value * Read_VDDA() / 4095.0f; // Convierte a mV
  return val;
}

float Read_Temp() {
  uint16_t adc_value = Read_ADC_Channel_Raw(&hadc1, ADC_CHANNEL_TEMPSENSOR,
                                            ADC_SAMPLETIME_239CYCLES_5);
  float temp = 25 + (1430 - (adc_value * Read_VDDA() / 4095.0f)) / 4.3f;
  return temp;
}

void Set_Vout_A(
    uint16_t set_point) { // pendiente de avanzar con mapa de registros
  float error = set_point / 100.0 - Read_ADC(PA0) * 1000;
}