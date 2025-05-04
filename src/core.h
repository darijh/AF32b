/**
 * @brief Inicializa el sistema configurando los módulos principales.
 */
void CoreInit()
{
  eepr.begin(REGS_SIZE);                                  // Inicializa el gestor de EEPROM
  hor_a.Begin(eepr.read(MB_HS_A), eepr.read(MB_ROLLS_A)); // Inicializa el horometro A
  hor_b.Begin(eepr.read(MB_HS_B), eepr.read(MB_ROLLS_B)); // Inicializa el horometro B
  modbus_slave.Begin(&COM_PORT, BAUDRATE,
                     Get_ID()); // Inicializa el modbus
  modbus_slave.Enable(true);    // Habilita el modbus
  modbus_slave.SetWriteListener(
      Modbus_Listener);              // Establece el listener de escritura
  hor_a.SetHsListener(HorACallback); // Establece el listener de Horometro A
  hor_b.SetHsListener(HorBCallback); // Establece el listener de Horometro B
  one_second_task.setCallback(
      OneSecondTask); // Establece el callback de la tarea de 1 segundo
  one_hour_task.setCallback(
      OneHourTask);                                // Establece el callback de la tarea de 1 hora
  one_day_task.setCallback(OneDayTask);            // Establece el callback de la tarea de 1 día
  vout_a.SetReads(READS);                          // Establece el número de lecturas
  vout_a.SetEnable(true);                          // Habilita el AIN
  vout_a.SetHiLimitEnable(true);                   // Habilita el límite superior
  vout_a.SetLoLimitEnable(true);                   // Habilita el límite inferior
  vout_a.SetConstants(VOUT_A_MULT, VOUT_A_OFFSET); // Establece los constantes de calibración

  // Iout_a
  iout_a.SetReads(READS);                          // Establece el número de lecturas
  iout_a.SetEnable(true);                          // Habilita el AIN
  iout_a.SetHiLimitEnable(true);                   // Habilita el límite superior
  iout_a.SetLoLimitEnable(true);                   // Habilita el límite inferior
  iout_a.SetConstants(IOUT_A_MULT, IOUT_A_OFFSET); // Establece los constantes de calibración

  // Vout_b
  vout_b.SetReads(READS);                         // Establece el número de lecturas
  vout_b.SetEnable(true);                         // Habilita el AIN
  vout_b.SetHiLimitEnable(true);                  // Habilita el límite superior
  vout_b.SetLoLimitEnable(true);                  // Habilita el límite inferior
  vout_b.SetConstants(VOUT_B_MUL, VOUT_B_OFFSET); // Establece los constantes de calibración

  // Iout_b
  iout_b.SetReads(READS);                          // Establece el número de lecturas
  iout_b.SetEnable(true);                          // Habilita el AIN
  iout_b.SetHiLimitEnable(true);                   // Habilita el límite superior
  iout_b.SetLoLimitEnable(true);                   // Habilita el límite inferior
  iout_b.SetConstants(IOUT_B_MULT, IOUT_B_OFFSET); // Establece los constantes de calibración
  iout_b.SetMedListener(Regulation);               // Establece el listener de medición

  pid_v_a.setOutputLimits(MIN_DUTY, MAX_DUTY); // Limites de salida
  pid_i_a.setOutputLimits(MIN_DUTY, MAX_DUTY); // Limites de salida
  pid_v_b.setOutputLimits(MIN_DUTY, MAX_DUTY); // Limites de salida
  pid_i_b.setOutputLimits(MIN_DUTY, MAX_DUTY); // Limites de salida

  Config(); // Configura el sistema
}

/**
 * @brief Configura el sistema con valores iniciales o restaurados.
 */
void Config()
{
  memset(regs, 0, sizeof(regs)); // Limpia el array de registros
  eepr.read(0, RW_SIZE, regs);   // Lee los registros de la EEPROM
  for (uint8_t i = 0; i < RW_SIZE; i++)
  {
    run_regs[i] = regs[i]; // Copia los registros a run_regs
  }

  // Vout_a
  vout_a.SetHiLimit(run_regs[MB_VHL_A]); // Límite superior
  vout_a.SetLoLimit(run_regs[MB_VLL_A]); // Límite inferior

  // Iout_a
  iout_a.SetHiLimit(run_regs[MB_IHL_A]); // Límite superior
  iout_a.SetLoLimit(run_regs[MB_ILL_A]); // Límite inferior

  // Vout_b
  vout_b.SetHiLimit(run_regs[MB_VHL_B]); // Límite superior
  vout_b.SetLoLimit(run_regs[MB_VLL_B]); // Límite inferior

  // Iout_b
  iout_b.SetHiLimit(run_regs[MB_IHL_B]); // Límite superior
  iout_b.SetLoLimit(run_regs[MB_ILL_B]); // Límite inferior

  regs[MB_V_DUTY_A] = MIN_DUTY; // Inicializa el duty cycle
  regs[MB_I_DUTY_A] = MIN_DUTY; // Inicializa el duty cycle
  regs[MB_V_DUTY_B] = MIN_DUTY; // Inicializa el duty cycle
  regs[MB_I_DUTY_B] = MIN_DUTY; // Inicializa el duty cycle
  pid_v_a.reset();              // Resetea el PID
  pid_i_a.reset();              // Resetea el PID
  pid_v_b.reset();              // Resetea el PID
  pid_i_b.reset();              // Resetea el PID
}

/**
 * @brief Lee el ID del hardware desde los dip switches.
 * @return ID del hardware.
 */
uint8_t Get_ID()
{
  uint8_t id = 0;                          // ID del hardware
  bitWrite(id, 0, !digitalRead(HW_ID_B0)); // setea el bit 0 segun el estado del dip switch
  bitWrite(id, 1, !digitalRead(HW_ID_B1)); // setea el bit 1 segun el estado del dip switch
  bitWrite(id, 2, !digitalRead(HW_ID_B2)); // setea el bit 2 segun el estado del dip switch
  bitWrite(id, 3, !digitalRead(HW_ID_B3)); // setea el bit 3 segun el estado del dip switch
  return id;                               // retorna el id
}

/**
 * @brief Lee un canal del ADC en modo crudo.
 * @param hadc Manejador del ADC.
 * @param channel Canal a leer.
 * @param samplingTime Tiempo de muestreo.
 * @return Valor crudo leído del ADC.
 */
uint16_t Read_ADC_Channel_Raw(ADC_HandleTypeDef *hadc, uint32_t channel,
                              uint32_t samplingTime)
{
  ADC_ChannelConfTypeDef sConfig = {0};                // Estructura de configuración del ADC
  sConfig.Channel = channel;                           // Canal a leer
  sConfig.Rank = ADC_REGULAR_RANK_1;                   // Siempre el primer rank
  sConfig.SamplingTime = samplingTime;                 // Tiempo de muestreo
  if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK) // Manejo de error
  {
    return 0; // Error en la configuración del canal
  }

  HAL_ADC_Start(hadc);                         // Inicia la conversión
  HAL_ADC_PollForConversion(hadc, 100);        // Espera a que la conversión termine
  uint16_t adc_value = HAL_ADC_GetValue(hadc); // Lee el valor convertido
  HAL_ADC_Stop(hadc);                          // Detiene el ADC
  return adc_value;                            // Retorna el valor convertido
}

/**
 * @brief Lee el valor del ADC y lo convierte a milivoltios.
 * @param channel Canal del ADC a leer.
 * @return Valor convertido en milivoltios.
 */
float Read_ADC(uint32_t channel)
{
  uint16_t adc_value =
      Read_ADC_Channel_Raw(&hadc2, channel, ADC_SAMPLETIME_28CYCLES_5); // Lee el valor del ADC
  float val = adc_value * Read_VDDA() / 4095.0f;                        // Convierte a mV
  return val;                                                           // Retorna el valor convertido
}

/**
 * @brief Maneja los AIN y actualiza los registros correspondientes.
 * @param ain Referencia al objeto AinHandler.
 * @param value_reg Registro de valor.
 * @param status_reg Registro de estado.
 * @param alarm_bit Bit de alarma.
 * @param alarm_reg Registro de alarma.
 * @param ml_alarm Tiempo de alarma.
 */
void HandleAins(AinHandler &ain, int16_t *value_reg, int16_t *status_reg,
                uint8_t alarm_bit, int16_t *alarm_reg, uint32_t &ml_alarm)
{
  *value_reg = max(ain.GetEU_AVG(), 0.0);  // obtiene el valor en EU
  char status = ain.GetStatus(*value_reg); // obtiene el estado del AIN

  if (status == asciistatus::H || status == asciistatus::L) // Si el estado es H o L
  {
    ml_alarm = millis();           // Guarda el tiempo de alarma
    bitSet(*alarm_reg, alarm_bit); // Setea el bit de alarma
    *status_reg = int(status);     // Actualiza el estado
  }
  else // Si el estado es normal
  {
    if (bitRead(*alarm_reg, alarm_bit)) // Si el bit de alarma está seteado
    {
      if (millis() > ml_alarm + 1000) // Si ha pasado 1 segundo desde la alarma
      {
        bitClear(*alarm_reg, alarm_bit); // Limpia el bit de alarma
        *status_reg = int(status);       // Actualiza el estado
      }
    }
    else // Si el bit de alarma no está seteado
    {
      *status_reg = int(status); // Actualiza el estado
    }
  }
}

/**
 * @brief Lee los valores del ADC y actualiza los AIN.
 * @param interval Intervalo de tiempo entre lecturas.
 */
void ADCReadings(uint32_t interval)
{
  static uint32_t ml_sample = 0;       // Guarda el tiempo de la última lectura
  if (micros() > ml_sample + interval) // Si ha pasado el intervalo
  {
    ml_sample = micros(); // Actualiza el tiempo de la última lectura
    vout_a.Sample(
        Read_ADC_Channel_Raw(&hadc2, HW_VOUT_A, ADC_SAMPLETIME_28CYCLES_5)); // Lee el valor del ADC
    iout_a.Sample(
        Read_ADC_Channel_Raw(&hadc2, HW_IOUT_A, ADC_SAMPLETIME_28CYCLES_5)); // Lee el valor del ADC
    vout_b.Sample(
        Read_ADC_Channel_Raw(&hadc2, HW_VOUT_B, ADC_SAMPLETIME_28CYCLES_5)); // Lee el valor del ADC
    iout_b.Sample(
        Read_ADC_Channel_Raw(&hadc2, HW_IOUT_B, ADC_SAMPLETIME_28CYCLES_5)); // Lee el valor del ADC
  }
  vout_a.Run(); // Procesa el valor del ADC
  vout_b.Run(); // Procesa el valor del ADC
  iout_a.Run(); // Procesa el valor del ADC
  iout_b.Run(); // Procesa el valor del ADC
}

/**
 * @brief Lee el valor de VDDA y lo convierte a milivoltios.
 * @return Valor de VDDA en milivoltios.
 */
float Read_VDDA()
{
  uint16_t adc_value = Read_ADC_Channel_Raw(&hadc1, ADC_CHANNEL_VREFINT,
                                            ADC_SAMPLETIME_239CYCLES_5); // Lee el valor del ADC
  float vdda = 4095 * 1200.0f / adc_value;                               // Convierte a mV
  return vdda;
}

/**
 * @brief Lee la temperatura del microcontrolador y la convierte a grados Celsius.
 * @return Temperatura en grados Celsius.
 */
float Read_Temp()
{
  uint16_t adc_value = Read_ADC_Channel_Raw(&hadc1, ADC_CHANNEL_TEMPSENSOR,
                                            ADC_SAMPLETIME_239CYCLES_5); // Lee el valor del ADC
  float temp = 25 + (1430 - (adc_value * Read_VDDA() / 4095.0f)) / 4.3f; // Convierte a °C
  return temp;
}

/**
 * @brief Convierte un porcentaje de duty cycle a un valor de PWM.
 * @param duty_percent Porcentaje de duty cycle (0-10000).
 * @return Valor de PWM correspondiente (0-4095).
 */
uint16_t Set_Duty(int16_t duty_percent)
{
  duty_percent = constrain(duty_percent, 0, 10000); // Limita el rango
  return map(duty_percent, 0, 10000, 0,
             4095); // Mapea el porcentaje a un valor entre 0 y 4095
}

/**
 * @brief Configura el sistema de regulación de voltaje y corriente.
 */
void Regulation()
{
  static uint32_t ml_alarm_vout_a = 0, ml_alarm_iout_a = 0; // Variables para almacenar el tiempo de alarma
  static uint32_t ml_alarm_vout_b = 0, ml_alarm_iout_b = 0; // Variables para almacenar el tiempo de alarma
  HandleAins(vout_a, &regs[MB_V_VAL_A], &regs[MB_V_STS_A], VOUT_A_BIT,
             &regs[MB_ALARM], ml_alarm_vout_a); // Maneja el AIN
  HandleAins(iout_a, &regs[MB_I_VAL_A], &regs[MB_I_STS_A], IOUT_A_BIT,
             &regs[MB_ALARM], ml_alarm_iout_a); // Maneja el AIN
  HandleAins(vout_b, &regs[MB_V_VAL_B], &regs[MB_V_STS_B], VOUT_B_BIT,
             &regs[MB_ALARM], ml_alarm_vout_b); // Maneja el AIN
  HandleAins(iout_b, &regs[MB_I_VAL_B], &regs[MB_I_STS_B], IOUT_B_BIT,
             &regs[MB_ALARM], ml_alarm_iout_b); // Maneja el AIN

  if (run_regs[MB_ENALBLE] == 1) // Si la regulación está habilitada
  {
    static bool prev_i_red_sts = false;      // Variable para almacenar el estado anterior de la entrada de reduccion de corriente
    if (regs[MB_IRED_STS] != prev_i_red_sts) // Si el estado ha cambiado
    {
      pid_i_a.reset(); // Resetea el PID
      pid_i_b.reset(); // Resetea el PID
    }
    prev_i_red_sts = regs[MB_IRED_STS]; // Actualiza el estado anterior
    double i_setpoint_a =
        regs[MB_IRED_STS] ? run_regs[MB_IRED_A] : run_regs[MB_ISET_A]; // Establece el setpoint de corriente
    double i_setpoint_b =
        regs[MB_IRED_STS] ? run_regs[MB_IRED_B] : run_regs[MB_ISET_B];          // Establece el setpoint de corriente
    regs[MB_V_DUTY_A] = pid_v_a.compute(regs[MB_V_VAL_A], run_regs[MB_VSET_A]); // Calcula el duty cycle de la tensión de la salida A
    regs[MB_I_DUTY_A] = pid_i_a.compute(regs[MB_I_VAL_A], i_setpoint_a);        // Calcula el duty cycle de la corriente de la salida A
    regs[MB_V_DUTY_B] = pid_v_b.compute(regs[MB_V_VAL_B], run_regs[MB_VSET_B]); // Calcula el duty cycle de la tensión de la salida B
    regs[MB_I_DUTY_B] = pid_i_b.compute(regs[MB_I_VAL_B], i_setpoint_b);        // Calcula el duty cycle de la corriente de la salida B
    if (pid_v_a.getAlarm() && pid_i_a.getAlarm())                               // Si ambos PID estan en alarma
    {
      bitSet(regs[MB_ALARM], REG_A_BIT); // Setea Alarma
    }
    else
    {
      bitClear(regs[MB_ALARM], REG_A_BIT); // Resetea Alarma
    }
    if (pid_v_b.getAlarm() && pid_i_b.getAlarm()) // Si ambos PID estan en alarma
    {
      bitSet(regs[MB_ALARM], REG_B_BIT); // Setea Alarma
    }
    else
    {
      bitClear(regs[MB_ALARM], REG_B_BIT); // Resetea Alarma
    }
  }
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, Set_Duty(regs[MB_V_DUTY_A])); // Configura el pwm segun el duty de voltaje en salida A
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, Set_Duty(regs[MB_I_DUTY_A])); // Configura el pwm segun el duty de corriente en salida A
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, Set_Duty(regs[MB_V_DUTY_B])); // Configura el pwm segun el duty de voltaje en salida B
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, Set_Duty(regs[MB_I_DUTY_B])); // Configura el pwm segun el duty de corriente en salida B
}

/**
 * @brief // Resetea la configuración de fábrica.
 */
void Factory_Reset()
{
  memset(regs, 0, sizeof(regs));              // Reinicia todos los registro
  regs[MB_VHL_A] = MB_DEF_VHL;                // Establece el valor por defecto
  regs[MB_IHL_A] = MB_DEF_IHL;                // Establece el valor por defecto
  regs[MB_VLL_A] = MB_DEF_VLL;                // Establece el valor por defecto
  regs[MB_ILL_A] = MB_DEF_ILL;                // Establece el valor por defecto
  regs[MB_VHL_B] = MB_DEF_VHL;                // Establece el valor por defecto
  regs[MB_IHL_B] = MB_DEF_IHL;                // Establece el valor por defecto
  regs[MB_VLL_B] = MB_DEF_VLL;                // Establece el valor por defecto
  regs[MB_ILL_B] = MB_DEF_ILL;                // Establece el valor por defecto
  eepr.write(regs, REGS_SIZE, 0, RW_SIZE, 0); // Guarda en EEPROM los registros RW
  eepr.write(MB_HS_A, 0);                     // Reset de Horometro en EEPROM
  eepr.write(MB_ROLLS_A, 0);                  // Reset de Horometro en EEPROM
  eepr.write(MB_HS_B, 0);                     // Reset de Horometro en EEPROM
  eepr.write(MB_ROLLS_B, 0);                  // Reset de Horometro en EEPROM
  hor_a.Reset();                              // Reset Horometro A
  hor_b.Reset();                              // Reset Horometro B
}

/**
 * @brief Listener para manejar cambios en los registros Modbus.
 */
void Modbus_Listener()
{
  modbus_slave.ChangesProcessed(); // Procesa los cambios
  if (regs[MB_APPLY])              // Si se aplica la configuración
  {
    if (regs[MB_FACTORY_RESET]) // Si se aplica el reset de fábrica

    {
      Factory_Reset(); // Resetea la configuración de fábrica
    }
    else // Si no se aplica el reset de fábrica
    {
      regs[MB_APPLY] = 0;                         // Limpia el flag de aplicar configuración
      eepr.write(regs, REGS_SIZE, 0, RW_SIZE, 0); // Guarda en EEPROM los registros RW
    }
    Config(); // Configura el sistema
  }
  if (regs[MB_CANCEL]) // Si se cancela la configuración
  {
    Config(); // Configura el sistema
  }
}

/**
 * @brief Tareas de 1 segundo.
 */
void OneSecondTask()
{
  digitalWrite(HW_STS, modbus_slave.active);            // Enciende el led de estado
  digitalWrite(HW_FAIL, (regs[MB_ALARM]) ? HIGH : LOW); // Enciende el led de fallo
  digitalWrite(HW_OK, (regs[MB_ALARM]) ? LOW : HIGH);   // Enciende el led de OK
  regs[MB_TEMP] = Read_Temp() * 100.0f;                 // Lee la temperatura
  if (run_regs[MB_IRED_NC_MODE])
  {                                            // Modo NC
    regs[MB_IRED_STS] = !digitalRead(HW_CTRL); // Lee el estado del control
  }
  else
  {                                           // Modo NO
    regs[MB_IRED_STS] = digitalRead(HW_CTRL); // Lee el estado del control
  }

  // HOROMETRO A
  hor_a.Count(); // Cuenta el horometro
  bool hor_a_status =
      (regs[MB_IRED_STS] == 0 && regs[MB_I_VAL_A] > IMAX * 0.01f); // Establece el estado del horometro
  hor_a.SetEnable(hor_a_status);                                   // Habilita el horometro
  regs[MB_HS_STS_A] = hor_a_status;                                // Establece el estado del horometro
  regs[MB_HS_A] = hor_a.GetHs();                                   // Establece el valor del horometro
  regs[MB_DHS_A] = hor_a.GetDh();                                  // Establece el valor del horometro
  regs[MB_ROLLS_A] = hor_a.GetRolls();                             // Establece el valor del horometro

  // HOROMETRO B
  hor_b.Count(); // Cuenta el horometro
  bool hor_b_status =
      (regs[MB_IRED_STS] == 0 && regs[MB_I_VAL_B] > IMAX * 0.01f); // Establece el estado del horometro
  hor_b.SetEnable(hor_b_status);                                   // Habilita el horometro
  regs[MB_HS_STS_B] = hor_b_status;                                // Establece el estado del horometro
  regs[MB_HS_B] = hor_b.GetHs();                                   // Establece el valor del horometro
  regs[MB_DHS_B] = hor_b.GetDh();                                  // Establece el valor del horometro
  regs[MB_ROLLS_B] = hor_b.GetRolls();                             // Establece el valor del horometro

#ifdef CALIBRACION                       // Si se habilita la calibración
  DBG_PORT.print("Vout_a: ");            // Debug
  DBG_PORT.print(vout_a.GetADC_AVG());   // Debug
  DBG_PORT.print(" Iout_a: ");           // Debug
  DBG_PORT.print(iout_a.GetADC_AVG());   // Debug
  DBG_PORT.print(" Vout_b: ");           // Debug
  DBG_PORT.print(vout_b.GetADC_AVG());   // Debug
  DBG_PORT.print(" Iout_b: ");           // Debug
  DBG_PORT.println(iout_b.GetADC_AVG()); // Debug
#else
#ifdef DEBUG // Si se habilita el debug
  DBG_PORT.print("Vout_A: ");                                // Debug
  DBG_PORT.print(regs[MB_V_VAL_A] / 100.0f, 2);              // Debug
  DBG_PORT.print(" V | Iout_A: ");                           // Debug
  DBG_PORT.print(regs[MB_I_VAL_A]);                          // Debug
  DBG_PORT.println(" mA");                                   // Debug
  DBG_PORT.print("V_Duty_A: ");                              // Debug
  DBG_PORT.print(regs[MB_V_DUTY_A]);                         // Debug
  DBG_PORT.print(" | I_Duty_A: ");                           // Debug
  DBG_PORT.println(regs[MB_I_DUTY_A]);                       // Debug
  DBG_PORT.print("Vout_B: ");                                // Debug
  DBG_PORT.print(regs[MB_V_VAL_B] / 100.0f, 2);              // Debug
  DBG_PORT.print(" V | Iout_B: ");                           // Debug
  DBG_PORT.print(regs[MB_I_VAL_B]);                          // Debug
  DBG_PORT.println(" mA");                                   // Debug
  DBG_PORT.print("V_Duty_B: ");                              // Debug
  DBG_PORT.print(regs[MB_V_DUTY_B]);                         // Debug
  DBG_PORT.print(" | I_Duty_B: ");                           // Debug
  DBG_PORT.println(regs[MB_I_DUTY_B]);                       // Debug
  DBG_PORT.print("Ired_STS_B: ");                            // Debug
  DBG_PORT.print((regs[MB_IRED_STS]) ? "ON" : "OFF");        // Debug
  DBG_PORT.print(" | ID: ");                                 // Debug
  DBG_PORT.println(Get_ID());                                // Debug
  DBG_PORT.println("====================================="); // Debug
#endif
#endif
}

/**
 * @brief Tareas de 1 hora.
 */
void OneHourTask()
{
  HAL_ADCEx_Calibration_Start(&hadc1); // Calibración de ADC1
  HAL_ADCEx_Calibration_Start(&hadc2); // Calibración de ADC2
}

/**
 * @brief Tareas de 1 día.
 */
void OneDayTask()
{
  NVIC_SystemReset(); // Reinicia el microcontrolador
}

/**
 * @brief Callback para el horómetro A.
 */
void HorACallback()
{
  eepr.write(MB_HS_A, hor_a.GetHs());       // Guarda el valor del horometro A en EEPROM
  eepr.write(MB_ROLLS_A, hor_a.GetRolls()); // Guarda el valor del horometro A en EEPROM
}

/**
 * @brief Callback para el horómetro B.
 */
void HorBCallback()
{
  eepr.write(MB_HS_B, hor_b.GetHs());       // Guarda el valor del horometro B en EEPROM
  eepr.write(MB_ROLLS_B, hor_b.GetRolls()); // Guarda el valor del horometro B en EEPROM
}