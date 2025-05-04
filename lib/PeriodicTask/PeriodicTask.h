#ifndef PERIODIC_TASK_H
#define PERIODIC_TASK_H

#include <functional> // Necesario para std::function

/**
 * @class PeriodicTask
 * @brief Clase para ejecutar tareas periódicas basadas en un intervalo de tiempo.
 */
class PeriodicTask
{
public:
  /**
   * @brief Constructor de la clase PeriodicTask.
   * @param intervalMs Intervalo de tiempo en milisegundos entre ejecuciones.
   */
  PeriodicTask(unsigned long intervalMs)
      : interval(intervalMs), lastExecution(0), cb(nullptr) {}

  /**
   * @brief Método que debe llamarse periódicamente (por ejemplo, desde loop()).
   * @param currentMillis Tiempo actual en milisegundos (normalmente obtenido con millis()).
   *
   * Este método verifica si ha pasado el intervalo de tiempo especificado
   * desde la última ejecución. Si es así, ejecuta la función callback asociada.
   */
  void run(unsigned long currentMillis)
  {
    if (cb && (currentMillis - lastExecution >= interval)) // Verifica si ha pasado el intervalo
    {
      lastExecution = currentMillis; // Actualiza el tiempo de la última ejecución
      cb();                          // Ejecuta la función callback
    }
  }

  /**
   * @brief Establece la función callback que se ejecutará periódicamente.
   * @param newCallback Función que se ejecutará cuando se cumpla el intervalo.
   */
  void setCallback(std::function<void()> newCallback) { cb = newCallback; }

private:
  unsigned long interval;      ///< Intervalo de tiempo en milisegundos entre ejecuciones.
  std::function<void()> cb;    ///< Función callback que se ejecutará periódicamente.
  unsigned long lastExecution; ///< Marca de tiempo de la última ejecución en milisegundos.
};

#endif // PERIODIC_TASK_H