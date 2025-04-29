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

void CoreInit() {
  eepr.begin(REGS_SIZE); // Inicializa el gestor de EEPROM
  hor_a.Begin(eepr.read(MB_HS_A), eepr.read(MB_ROLLS_A));
  hor_b.Begin(eepr.read(MB_HS_B), eepr.read(MB_ROLLS_B));
  modbus_slave.Begin(&COM_PORT, BAUDRATE,
                     Get_ID()); // Inicializa el puerto serie
  modbus_slave.Enable(true);    // Habilita el puerto serie
  modbus_slave.SetWriteListener(
      Modbus_Listener);              // Establece el listener de escritura
  hor_a.SetHsListener(HorACallback); // Establece el listener de Horometro A
  hor_b.SetHsListener(HorBCallback); // Establece el listener de Horometro B
  one_second_task.setCallback(
      OneSecondTask); // Establece el callback de la tarea de 1 segundo
  one_hour_task.setCallback(
      OneHourTask); // Establece el callback de la tarea de 1 hora
  one_day_task.setCallback(OneDayTask); // Establece el callback de la tarea de
                                        // 1 día Vout_a
  vout_a.SetReads(READS);
  vout_a.SetEnable(true);
  vout_a.SetHiLimitEnable(true);
  vout_a.SetLoLimitEnable(true);
  vout_a.SetConstants(VOUT_A_MULT, VOUT_A_OFFSET);

  // Iout_a
  iout_a.SetReads(READS);
  iout_a.SetEnable(true);
  iout_a.SetHiLimitEnable(true);
  iout_a.SetLoLimitEnable(true);
  iout_a.SetConstants(IOUT_A_MULT, IOUT_A_OFFSET);

  // Vout_b
  vout_b.SetReads(READS);
  vout_b.SetEnable(true);
  vout_b.SetHiLimitEnable(true);
  vout_b.SetLoLimitEnable(true);
  vout_b.SetConstants(VOUT_B_MUL, VOUT_B_OFFSET);

  // Iout_b
  iout_b.SetReads(READS);
  iout_b.SetEnable(true);
  iout_b.SetHiLimitEnable(true);
  iout_b.SetLoLimitEnable(true);
  iout_b.SetConstants(IOUT_B_MULT, IOUT_B_OFFSET);
  iout_b.SetMedListener(Regulation);

  Config();
}

void Config() {
  memset(regs, 0, sizeof(regs)); // Limpia el array de registros
  eepr.read(0, RW_SIZE, regs);   // Lee los registros de la EEPROM
  for (uint8_t i = 0; i < RW_SIZE; i++) {
    run_regs[i] = regs[i]; // Copia los registros a run_regs
  }

  // Vout_a
  vout_a.SetHiLimit(run_regs[MB_VHL_A]);
  vout_a.SetLoLimit(run_regs[MB_VLL_A]);

  // Iout_a
  iout_a.SetHiLimit(run_regs[MB_IHL_A]);
  iout_a.SetLoLimit(run_regs[MB_ILL_A]);

  // Vout_b
  vout_b.SetHiLimit(run_regs[MB_VHL_B]);
  vout_b.SetLoLimit(run_regs[MB_VLL_B]);

  // Iout_b
  iout_b.SetHiLimit(run_regs[MB_IHL_B]);
  iout_b.SetLoLimit(run_regs[MB_ILL_B]);

  regs[MB_V_DUTY_A] = 0;
  regs[MB_I_DUTY_A] = 0;
  regs[MB_V_DUTY_B] = 0;
  regs[MB_I_DUTY_B] = 0;

  /*   regulator_a.setEnable(false);
    regulator_b.setEnable(false);
  regulator_a.setEnable(run_regs[MB_ENALBLE]);
  regulator_b.setEnable(run_regs[MB_ENALBLE]);
  regulator_a.setSetpoints(run_regs[MB_VSET_A], run_regs[MB_ISET_A],
                           regs[MB_IRED_A]);
  regulator_b.setSetpoints(run_regs[MB_VSET_B], run_regs[MB_ISET_B],
                           regs[MB_IRED_B]);*/
}

uint8_t Get_ID() {
  uint8_t id = 0;
  bitWrite(id, 0, !digitalRead(HW_ID_B0));
  bitWrite(id, 1, !digitalRead(HW_ID_B1));
  bitWrite(id, 2, !digitalRead(HW_ID_B2));
  bitWrite(id, 3, !digitalRead(HW_ID_B3));
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
  HAL_ADC_PollForConversion(hadc, 100);

  // Lee el valor convertido
  uint16_t adc_value = HAL_ADC_GetValue(hadc);

  // Detiene el ADC
  HAL_ADC_Stop(hadc);
  return adc_value;
}

float Read_ADC(uint32_t channel) {
  uint16_t adc_value =
      Read_ADC_Channel_Raw(&hadc2, channel, ADC_SAMPLETIME_28CYCLES_5);
  float val = adc_value * Read_VDDA() / 4095.0f; // Convierte a mV
  return val;
}

void HandleAins(AinHandler &ain, int16_t *value_reg, int16_t *status_reg,
                uint8_t alarm_bit, int16_t *alarm_reg, uint32_t &ml_alarm) {
  *value_reg = max(ain.GetEU_AVG(), 0.0);  // Lee el valor en EU
  char status = ain.GetStatus(*value_reg); // Lee el estado del AIN

  if (status == asciistatus::H || status == asciistatus::L) {
    ml_alarm = millis();
    bitSet(*alarm_reg, alarm_bit);
    *status_reg = int(status);
  } else {
    if (bitRead(*alarm_reg, alarm_bit)) {
      if (millis() > ml_alarm + 1000) {
        bitClear(*alarm_reg, alarm_bit);
        *status_reg = int(status);
      }
    } else {
      *status_reg = int(status);
    }
  }
}

void ADCReadings(uint32_t interval) {
  static uint32_t ml_sample = 0;
  if (millis() > ml_sample + interval) {
    ml_sample = millis();
    vout_a.Sample(
        Read_ADC_Channel_Raw(&hadc2, HW_VOUT_A, ADC_SAMPLETIME_28CYCLES_5));
    iout_a.Sample(
        Read_ADC_Channel_Raw(&hadc2, HW_IOUT_A, ADC_SAMPLETIME_28CYCLES_5));
    vout_b.Sample(
        Read_ADC_Channel_Raw(&hadc2, HW_VOUT_B, ADC_SAMPLETIME_28CYCLES_5));
    iout_b.Sample(
        Read_ADC_Channel_Raw(&hadc2, HW_IOUT_B, ADC_SAMPLETIME_28CYCLES_5));
  }
  vout_a.Run();
  vout_b.Run();
  iout_a.Run();
  iout_b.Run();
}

float Read_VDDA() {
  uint16_t adc_value = Read_ADC_Channel_Raw(&hadc1, ADC_CHANNEL_VREFINT,
                                            ADC_SAMPLETIME_239CYCLES_5);
  float vdda = 4095 * 1200.0f / adc_value;
  return vdda;
}

float Read_Temp() {
  uint16_t adc_value = Read_ADC_Channel_Raw(&hadc1, ADC_CHANNEL_TEMPSENSOR,
                                            ADC_SAMPLETIME_239CYCLES_5);
  float temp = 25 + (1430 - (adc_value * Read_VDDA() / 4095.0f)) / 4.3f;
  return temp;
}

uint16_t Set_Duty(int16_t duty_percent) {
  return map(duty_percent, 0, 10000, 0,
             4095); // Mapea el porcentaje a un valor entre 0 y 4095
}

void Regulation() {
  static uint32_t ml_alarm_vout_a = 0, ml_alarm_iout_a = 0;
  static uint32_t ml_alarm_vout_b = 0, ml_alarm_iout_b = 0;
  HandleAins(vout_a, &regs[MB_V_VAL_A], &regs[MB_V_STS_A], VOUT_A_BIT,
             &regs[MB_ALARM], ml_alarm_vout_a);
  HandleAins(iout_a, &regs[MB_I_VAL_A], &regs[MB_I_STS_A], IOUT_A_BIT,
             &regs[MB_ALARM], ml_alarm_iout_a);
  HandleAins(vout_b, &regs[MB_V_VAL_B], &regs[MB_V_STS_B], VOUT_B_BIT,
             &regs[MB_ALARM], ml_alarm_vout_b);
  HandleAins(iout_b, &regs[MB_I_VAL_B], &regs[MB_I_STS_B], IOUT_B_BIT,
             &regs[MB_ALARM], ml_alarm_iout_b);
  if (regs[MB_ENALBLE] == 1) {
    /*       regulator_a.update(regs[MB_V_VAL_A], regs[MB_I_VAL_A],
       regs[MB_IRED_STS]); regulator_b.update(regs[MB_V_VAL_B],
       regs[MB_I_VAL_B], regs[MB_IRED_STS]); regs[MB_V_DUTY_A] =
       regulator_a.getVoltageDuty(); regs[MB_I_DUTY_A] =
       regulator_a.getCurrentDuty(); regs[MB_V_DUTY_B] =
       regulator_b.getVoltageDuty(); regs[MB_I_DUTY_B] =
       regulator_b.getCurrentDuty(); bitWrite(regs[MB_ALARM], REG_A_BIT,
       regulator_a.isNotReachingSetpoint()); bitWrite(regs[MB_ALARM], REG_B_BIT,
       regulator_b.isNotReachingSetpoint()); */
    int16_t i_setpoint_a =
        regs[MB_IRED_STS] ? regs[MB_IRED_A] : regs[MB_ISET_A];
    int16_t i_setpoint_b =
        regs[MB_IRED_STS] ? regs[MB_IRED_B] : regs[MB_ISET_B];
    int16_t v_error_a = regs[MB_VSET_A] - regs[MB_V_VAL_A];
    int16_t i_error_a = i_setpoint_a - regs[MB_I_VAL_A];
    int16_t v_error_b = regs[MB_VSET_B] - regs[MB_V_VAL_B];
    int16_t i_error_b = i_setpoint_b - regs[MB_I_VAL_B];
    int16_t v_error_a_rel = abs(v_error_a) * 100 / regs[MB_VSET_A];
    int16_t i_error_a_rel = abs(i_error_a) * 100 / i_setpoint_a;
    int16_t v_error_b_rel = abs(v_error_b) * 100 / regs[MB_VSET_B];
    int16_t i_error_b_rel = abs(i_error_b) * 100 / i_setpoint_b;
    static uint32_t ml_alarm_a = millis();
    static uint32_t ml_alarm_b = millis();
    if (v_error_a_rel > 10 && i_error_a_rel > 10) {
      if (millis() > ml_alarm_a + 60000) {
        bitSet(regs[MB_ALARM], REG_A_BIT);
      }
    } else {
      ml_alarm_a = millis();
      bitClear(regs[MB_ALARM], REG_A_BIT);
    }
    if (v_error_b_rel > 10 && i_error_b_rel > 10) {
      if (millis() > ml_alarm_b + 60000) {
        bitSet(regs[MB_ALARM], REG_B_BIT);
      }
    } else {
      ml_alarm_b = millis();
      bitClear(regs[MB_ALARM], REG_B_BIT);
    }

    regs[MB_V_DUTY_A] += constrain(v_error_a * 0.1, 0, 10000);
    regs[MB_I_DUTY_A] += constrain(i_error_a * 0.1, 0, 10000);
    regs[MB_V_DUTY_B] += constrain(v_error_b * 0.1, 0, 10000);
    regs[MB_I_DUTY_B] += constrain(i_error_b * 0.1, 0, 10000);
  }

  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, Set_Duty(regs[MB_V_DUTY_A]));
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, Set_Duty(regs[MB_I_DUTY_A]));
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, Set_Duty(regs[MB_V_DUTY_B]));
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, Set_Duty(regs[MB_I_DUTY_B]));
}

void Factory_Reset() {
  memset(regs, 0, sizeof(regs));
  regs[MB_VHL_A] = MB_DEF_VHL;
  regs[MB_IHL_A] = MB_DEF_IHL;
  regs[MB_VLL_A] = MB_DEF_VLL;
  regs[MB_ILL_A] = MB_DEF_ILL;
  regs[MB_VHL_B] = MB_DEF_VHL;
  regs[MB_IHL_B] = MB_DEF_IHL;
  regs[MB_VLL_B] = MB_DEF_VLL;
  regs[MB_ILL_B] = MB_DEF_ILL;
  eepr.write(regs, REGS_SIZE, 0, RW_SIZE, 0);
  eepr.write(MB_HS_A, 0);    // Reset de Horometro
  eepr.write(MB_ROLLS_A, 0); // Reset de Horometro
  eepr.write(MB_HS_B, 0);    // Reset de Horometro
  eepr.write(MB_ROLLS_B, 0); // Reset de Horometro
  hor_a.Reset();
  hor_b.Reset();
}

void Modbus_Listener() {
  modbus_slave.ChangesProcessed();
  if (regs[MB_APPLY]) {
    if (regs[MB_FACTORY_RESET]) {
      Factory_Reset();
    } else {
      regs[MB_APPLY] = 0;
      eepr.write(regs, REGS_SIZE, 0, RW_SIZE, 0);
    }
    Config();
  }
  if (regs[MB_CANCEL]) {
    Config();
  }
}

void OneSecondTask() {
  digitalWrite(HW_STS, modbus_slave.active);
  digitalWrite(HW_FAIL, (regs[MB_ALARM]) ? HIGH : LOW);
  digitalWrite(HW_OK, (regs[MB_ALARM]) ? LOW : HIGH);
  regs[MB_TEMP] = Read_Temp() * 100.0f;        // Lee la temperatura
  if (run_regs[MB_IRED_NC_MODE]) {             // Modo NC
    regs[MB_IRED_STS] = !digitalRead(HW_CTRL); // Lee el estado del control
  } else {                                     // Modo NO
    regs[MB_IRED_STS] = digitalRead(HW_CTRL);  // Lee el estado del control
  }

  // HOROMETRO A
  hor_a.Count();
  bool hor_a_status =
      (regs[MB_IRED_STS] == 0 && regs[MB_I_VAL_A] > IMAX * 0.01f) ? true
                                                                  : false;
  hor_a.SetEnable(hor_a_status);
  regs[MB_HS_STS_A] = hor_a_status;
  regs[MB_HS_A] = hor_a.GetHs();
  regs[MB_DHS_A] = hor_a.GetDh();
  regs[MB_ROLLS_A] = hor_a.GetRolls();

  // HOROMETRO B
  hor_b.Count();
  bool hor_b_status =
      (regs[MB_IRED_STS] == 0 && regs[MB_I_VAL_B] > IMAX * 0.01f) ? true
                                                                  : false;
  hor_b.SetEnable(hor_b_status);
  regs[MB_HS_STS_B] = hor_b_status;
  regs[MB_HS_B] = hor_b.GetHs();
  regs[MB_DHS_B] = hor_b.GetDh();
  regs[MB_ROLLS_B] = hor_b.GetRolls();

#ifdef CALIBRACION
  DBG_PORT.print("Vout_a: ");
  DBG_PORT.print(vout_a.GetADC_AVG());
  DBG_PORT.print(" Iout_a: ");
  DBG_PORT.print(iout_a.GetADC_AVG());
  DBG_PORT.print(" Vout_b: ");
  DBG_PORT.print(vout_b.GetADC_AVG());
  DBG_PORT.print(" Iout_b: ");
  DBG_PORT.println(iout_b.GetADC_AVG());
#else
#ifdef DEBUG
  DBG_PORT.print("Vout_A: ");
  DBG_PORT.print(regs[MB_V_VAL_A] / 100.0f, 2);
  DBG_PORT.print(" V | Iout_A: ");
  DBG_PORT.print(regs[MB_I_VAL_A]);
  DBG_PORT.println(" mA");
  DBG_PORT.print("V_Duty_A: ");
  DBG_PORT.print(regs[MB_V_DUTY_A]);
  DBG_PORT.print(" | I_Duty_A: ");
  DBG_PORT.println(regs[MB_I_DUTY_A]);
  /* DBG_PORT.print("Current Mode: ");
  DBG_PORT.println((regulator_a.isCurrentMode()) ? "ON" : "OFF"); */
  /*   DBG_PORT.print("Vout_B: ");
    DBG_PORT.print(regs[MB_V_VAL_B] / 100.0f, 2);
    DBG_PORT.print(" V | Iout_B: ");
    DBG_PORT.print(regs[MB_I_VAL_B]);
    DBG_PORT.println(" mA");
    DBG_PORT.print("V_Duty_B: ");
    DBG_PORT.print(regs[MB_V_DUTY_B]);
    DBG_PORT.print(" | I_Duty_B: ");
    DBG_PORT.println(regs[MB_I_DUTY_B]);
    DBG_PORT.print("Current Mode: ");
    DBG_PORT.println((regulator_b.isCurrentMode()) ? "ON" : "OFF"); */
  /* DBG_PORT.print("Ired_STS_B: ");
  DBG_PORT.print((regs[MB_IRED_STS]) ? "ON" : "OFF");
  DBG_PORT.print(" | ID: ");
  DBG_PORT.println(Get_ID()); */
  DBG_PORT.println("=====================================");
#endif
#endif
}

void OneHourTask() {
  HAL_ADCEx_Calibration_Start(&hadc1); // Calibración de ADC1
  HAL_ADCEx_Calibration_Start(&hadc2); // Calibración de ADC2
}

void OneDayTask() {
  NVIC_SystemReset(); // Reinicia el microcontrolador
}

void HorACallback() {
  eepr.write(MB_HS_A, hor_a.GetHs());
  eepr.write(MB_ROLLS_A, hor_a.GetRolls());
}

void HorBCallback() {
  eepr.write(MB_HS_B, hor_b.GetHs());
  eepr.write(MB_ROLLS_B, hor_b.GetRolls());
}