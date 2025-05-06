#include "PID.h"

/**
 * @brief Constructor de la clase PID.
 * @param kp Ganancia proporcional.
 * @param ki Ganancia integral.
 * @param kd Ganancia derivativa.
 * @param time_step Intervalo de tiempo entre cálculos (en segundos).
 */
PID::PID(double kp, double ki, double kd)
    : _kp(kp), _ki(ki), _kd(kd), _integral(0), _prevError(0), _outputMin(0),
      _outputMax(10000), _firstCompute(true), _enable(0), _alarm(0), _alarm_timeout(20000), _alarm_threshold(10), _ml_alarm(0) {}

/**
 * @brief Establece los límites de la salida del controlador.
 * @param min Valor mínimo de la salida.
 * @param max Valor máximo de la salida.
 */
void PID::setOutputLimits(double min, double max, double threshold)
{
  if (max > min) // Verifica que el límite máximo sea mayor que el mínimo.
  {
    _outputMin = min;
    _outputMax = max;
  }
  _spThreshold = threshold; // Establece el umbral de error relativo.
}

/**
 * @brief Habilita o deshabilita el controlador PID.
 * @param enable `true` para habilitar, `false` para deshabilitar.
 */
void PID::setEnable(bool enable)
{
  _enable = enable;
  if (!enable)
  {
    reset(); // Reinicia el estado interno si se deshabilita el controlador.
  }
}

/**
 * @brief Configura los parámetros de la alarma.
 * @param timeout Tiempo de espera para activar la alarma (en milisegundos).
 * @param threshold Umbral de error relativo para activar la alarma (en porcentaje).
 */
void PID::AlarmConfig(unsigned long timeout, double threshold)
{
  _alarm_timeout = timeout;
  _alarm_threshold = threshold;
}

/**
 * @brief Calcula la salida del controlador PID.
 * @param input Valor actual de la entrada.
 * @param setpoint Valor objetivo (setpoint).
 * @return Salida del controlador PID.
 */
double PID::compute(double input, double setpoint)
{
  if (!_enable)
  {
    return 0; // Si el controlador no está habilitado, devuelve 0.
  }

  // Calcula el error entre el setpoint y la entrada.
  double error = setpoint - input;
  double relative_error = 0;

  // Calcula el error relativo si el setpoint no es 0.
  if (setpoint)
  {
    relative_error = abs(error) * 100 / setpoint;
  }

  // Actualiza el estado de la alarma según el error relativo.
  if (relative_error < _alarm_threshold)
  {
    _ml_alarm = millis(); // Marca de tiempo de la última vez que el error estuvo dentro del umbral.
    _alarm = false;       // Desactiva la alarma.
  }
  else if ((millis() - _ml_alarm) > _alarm_timeout)
  {
    _alarm = true; // Activa la alarma si el error persiste más allá del tiempo de espera.
  }

  _atSetPoint = (relative_error < _spThreshold); // Verifica si el controlador está en el punto de ajuste.

  if (!_stop)
  {
    // Calcula la parte integral, limitada por los valores de salida.
    _integral = constrain(_integral + (error * _ki), _outputMin, _outputMax);
  }

  // Calcula la parte derivativa.
  double derivative = 0;
  if (!_firstCompute)
  {
    derivative = error - _prevError; // Derivada del error.
  }
  else
  {
    _firstCompute = false; // Marca que el primer cálculo ya se realizó.
  }
  _prevError = error; // Actualiza el error previo.

  // Calcula la salida del controlador PID.
  double output = _kp * error + _integral + _kd * derivative;

  // Limita la salida a los valores establecidos.
  if (output > _outputMax)
    output = _outputMax;
  else if (output < _outputMin)
    output = _outputMin;

  return output;
}

/**
 * @brief Detiene el controlador PID.
 * @param stop `true` para detener, `false` para continuar.
 */
void PID::stop(bool stop)
{
  _stop = stop; // Establece el estado de detención del controlador.
}

/**
 * @brief Obtiene el estado de la alarma.
 * @return `true` si la alarma está activa, `false` en caso contrario.
 */
bool PID::getAlarm()
{
  return _alarm;
}

/***
 * @brief Obtiene el valor integral del controlador PID.
 * @return Valor integral del controlador PID.
 */
double PID::getIntegral()
{
  return _integral; // Devuelve el valor integral del controlador PID.
}

/**
 * @brief Verifica si el controlador PID está en el punto de ajuste.
 * @return `true` si está en el punto de ajuste, `false` en caso contrario.
 */
bool PID::atSetPoint()
{
  return _atSetPoint; // Devuelve si el controlador está en el punto de ajuste.
}

/**
 * @brief Reinicia el estado interno del controlador PID.
 */
void PID::reset()
{
  _integral = 0;        // Reinicia la parte integral.
  _prevError = 0;       // Reinicia el error previo.
  _firstCompute = true; // Marca que el próximo cálculo será el primero.
}