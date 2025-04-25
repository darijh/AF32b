#include <EEPROM.h>

class EepromSTM32 {
public:
  // Inicializa la clase para numRegisters de int16_t.
  // Devuelve false si no hay suficiente espacio en EEPROM.
  bool begin(uint16_t numRegisters) {
    uint32_t required = uint32_t(numRegisters) * sizeof(int16_t);
    if (required > EEPROM.length()) {
      return false;
    }
    _count = numRegisters;
    return true;
  }

  // Lee un único elemento. Si el índice está fuera de rango, devuelve 0.
  int16_t read(uint16_t index) {
    if (index >= _count) {
      return 0;
    }
    int16_t val;
    EEPROM.get(index * sizeof(int16_t), val);
    return val;
  }

  // Escribe un único elemento. Devuelve false si el índice es inválido.
  bool write(uint16_t index, int16_t value) {
    if (index >= _count) {
      return false;
    }
    EEPROM.put(index * sizeof(int16_t), value);
    return true;
  }

  // Lee 'count' elementos desde 'startIndex' y los copia en dst[].
  // Devuelve false si sale del rango.
  bool read(uint16_t startIndex, uint16_t count, int16_t *dst) {
    if (startIndex + count > _count) {
      return false;
    }
    for (uint16_t i = 0; i < count; i++) {
      EEPROM.get((startIndex + i) * sizeof(int16_t), dst[i]);
    }
    return true;
  }

  // Escribe 'count' valores desde src[arrayOffset…] en EEPROM
  // a partir de 'eepromStartIndex'. Comprueba límites con arraySize y _count.
  bool write(int16_t *src, uint16_t arraySize, uint16_t arrayOffset,
             uint16_t count, uint16_t eepromStartIndex) {
    if (arrayOffset + count > arraySize || eepromStartIndex + count > _count) {
      return false;
    }
    for (uint16_t i = 0; i < count; i++) {
      EEPROM.put((eepromStartIndex + i) * sizeof(int16_t),
                 src[arrayOffset + i]);
    }
    return true;
  }

  // Número total de registros configurados
  uint16_t size() const { return _count; }

private:
  uint16_t _count = 0;
};