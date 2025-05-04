#include <EEPROM.h>

/**
 * @class EepromSTM32
 * @brief Clase para manejar operaciones de lectura y escritura en la EEPROM de un STM32.
 */
class EepromSTM32
{
public:
  /**
   * @brief Inicializa la clase para manejar un número específico de registros.
   * @param numRegisters Número de registros de tipo int16_t que se desean manejar.
   * @return `true` si hay suficiente espacio en la EEPROM, `false` en caso contrario.
   */
  bool begin(uint16_t numRegisters)
  {
    uint32_t required = uint32_t(numRegisters) * sizeof(int16_t);
    if (required > EEPROM.length())
    {
      return false; // No hay suficiente espacio en la EEPROM.
    }
    _count = numRegisters;
    return true;
  }

  /**
   * @brief Lee un único registro de la EEPROM.
   * @param index Índice del registro a leer.
   * @return Valor del registro leído. Devuelve 0 si el índice está fuera de rango.
   */
  int16_t read(uint16_t index)
  {
    if (index >= _count)
    {
      return 0; // Índice fuera de rango.
    }
    int16_t val;
    EEPROM.get(index * sizeof(int16_t), val); // Lee el valor desde la EEPROM.
    return val;
  }

  /**
   * @brief Escribe un único registro en la EEPROM.
   * @param index Índice del registro a escribir.
   * @param value Valor que se desea escribir.
   * @return `true` si la operación fue exitosa, `false` si el índice es inválido.
   */
  bool write(uint16_t index, int16_t value)
  {
    if (index >= _count)
    {
      return false; // Índice fuera de rango.
    }
    EEPROM.put(index * sizeof(int16_t), value); // Escribe el valor en la EEPROM.
    return true;
  }

  /**
   * @brief Lee múltiples registros desde la EEPROM.
   * @param startIndex Índice inicial desde donde se comenzará a leer.
   * @param count Número de registros a leer.
   * @param dst Puntero al arreglo donde se almacenarán los valores leídos.
   * @return `true` si la operación fue exitosa, `false` si los índices están fuera de rango.
   */
  bool read(uint16_t startIndex, uint16_t count, int16_t *dst)
  {
    if (startIndex + count > _count)
    {
      return false; // Rango fuera de los límites.
    }
    for (uint16_t i = 0; i < count; i++)
    {
      EEPROM.get((startIndex + i) * sizeof(int16_t), dst[i]); // Lee cada registro.
    }
    return true;
  }

  /**
   * @brief Escribe múltiples registros en la EEPROM.
   * @param src Puntero al arreglo de valores que se desean escribir.
   * @param arraySize Tamaño total del arreglo `src`.
   * @param arrayOffset Índice inicial en el arreglo `src` desde donde se comenzará a escribir.
   * @param count Número de registros a escribir.
   * @param eepromStartIndex Índice inicial en la EEPROM donde se comenzará a escribir.
   * @return `true` si la operación fue exitosa, `false` si los índices están fuera de rango.
   */
  bool write(int16_t *src, uint16_t arraySize, uint16_t arrayOffset,
             uint16_t count, uint16_t eepromStartIndex)
  {
    if (arrayOffset + count > arraySize || eepromStartIndex + count > _count)
    {
      return false; // Rango fuera de los límites.
    }
    for (uint16_t i = 0; i < count; i++)
    {
      EEPROM.put((eepromStartIndex + i) * sizeof(int16_t),
                 src[arrayOffset + i]); // Escribe cada registro.
    }
    return true;
  }

  /**
   * @brief Obtiene el número total de registros configurados.
   * @return Número total de registros configurados.
   */
  uint16_t size() const { return _count; }

private:
  uint16_t _count = 0; ///< Número total de registros configurados.
};