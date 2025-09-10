#pragma once
#include <Arduino.h>
#include <Wire.h>

class Adp910Driver {
public:
  // Constructor (puedes cambiar addr si hiciera falta)
  explicit Adp910Driver(uint8_t i2c_addr = 0x25);

  // Inicializa el bus I²C, fija pines/clock y entra en modo continuo (0x361E)
  // Descarta 3 tramas para estabilizar.
  // Devuelve true si pudo escribir el comando inicial.
  bool begin(TwoWire& wire, int sda_pin, int scl_pin, uint32_t i2c_hz = 25000);

  // Lee una muestra del stream continuo con CRC y conversión.
  // Devuelve true si la lectura fue válida.
  bool readOnce(float& dp_Pa, float& t_C);

  // Calibración temporal: promedia durante 'ms' (por defecto 5000 ms)
  // y guarda un offset en RAM (no persistente).
  void calibrateMs(uint32_t ms = 5000);

  // Helpers
  void  setOffsetPa(float off);
  float offsetPa() const;

  // Re-armar modo continuo (normalmente no hace falta; la librería lo usa internamente).
  bool startContinuous();

  // Accesores de configuración
  uint8_t  address() const;
  int      sdaPin()  const;
  int      sclPin()  const;
  uint32_t i2cHz()   const;

private:
  static uint8_t crc8_(const uint8_t* d, uint8_t n);
  bool writeCmd_(uint16_t cmd);

private:
  // Config
  uint8_t  addr_ = 0x25;
  int      sda_  = -1;
  int      scl_  = -1;
  uint32_t hz_   = 25000;
  TwoWire* bus_  = nullptr;

  // Estado
  float offsetPa_ = 0.0f;

  // Constantes del protocolo
  static constexpr uint16_t CMD_CONT_ = 0x361E; // continuous mode
};