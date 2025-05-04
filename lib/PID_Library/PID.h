#ifndef PID_H
#define PID_H
#include <Arduino.h>

/**
 * @class PID
 * @brief Implementación de un controlador PID con soporte para alarmas.
 */
class PID
{
public:
  /**
   * @brief Constructor de la clase PID.
   * @param kp Ganancia proporcional.
   * @param ki Ganancia integral.
   * @param kd Ganancia derivativa.
   * @param time_step Intervalo de tiempo entre cálculos (en segundos).
   */
  PID(double kp, double ki, double kd, double time_step);

  /**
   * @brief Establece los límites de la salida del controlador.
   * @param min Valor mínimo de la salida.
   * @param max Valor máximo de la salida.
   */
  void setOutputLimits(double min, double max);

  /**
   * @brief Habilita o deshabilita el controlador PID.
   * @param enable `true` para habilitar, `false` para deshabilitar.
   */
  void setEnable(bool enable);

  /**
   * @brief Configura los parámetros de la alarma.
   * @param timeout Tiempo de espera para activar la alarma (en milisegundos).
   * @param threshold Umbral de error relativo para activar la alarma (en porcentaje).
   */
  void AlarmConfig(unsigned long timeout, double threshold);

  /**
   * @brief Calcula la salida del controlador PID.
   * @param input Valor actual de la entrada.
   * @param setpoint Valor objetivo (setpoint).
   * @return Salida del controlador PID.
   */
  double compute(double input, double setpoint);

  /**
   * @brief Obtiene el estado de la alarma.
   * @return `true` si la alarma está activa, `false` en caso contrario.
   */
  bool getAlarm();

  /**
   * @brief Reinicia el estado interno del controlador PID.
   */
  void reset();

private:
  // Ganancias del controlador PID
  double _kp; ///< Ganancia proporcional.
  double _ki; ///< Ganancia integral.
  double _kd; ///< Ganancia derivativa.

  // Intervalo de tiempo entre cálculos
  double _time_step; ///< Intervalo de tiempo (en segundos).

  // Estado interno del controlador
  double _integral;   ///< Acumulador de la parte integral.
  double _prevError;  ///< Error en el cálculo anterior.
  double _outputMin;  ///< Límite inferior de la salida.
  double _outputMax;  ///< Límite superior de la salida.
  bool _firstCompute; ///< Indica si es el primer cálculo del controlador.

  // Configuración y estado del controlador
  bool _enable; ///< Indica si el controlador está habilitado.
  bool _alarm;  ///< Estado de la alarma.

  // Configuración de la alarma
  unsigned long _alarm_timeout; ///< Tiempo de espera para activar la alarma (en milisegundos).
  double _alarm_threshold;      ///< Umbral de error relativo para activar la alarma (en porcentaje).
  unsigned long _ml_alarm;      ///< Marca de tiempo de la última vez que el error estuvo dentro del umbral.
};

#endif // PID_H