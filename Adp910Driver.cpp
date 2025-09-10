#include "Adp910Driver.h"

Adp910Driver::Adp910Driver(uint8_t i2c_addr) : addr_(i2c_addr) {}

bool Adp910Driver::begin(TwoWire& wire, int sda_pin, int scl_pin, uint32_t i2c_hz) {
  bus_ = &wire;
  sda_ = sda_pin;
  scl_ = scl_pin;
  hz_  = i2c_hz;

  bus_->begin(sda_, scl_);
  bus_->setClock(hz_);
  bus_->setTimeOut(50);
  delay(60); // power-up del sensor

  const bool ok = startContinuous();
  delay(20);   // primera respuesta disponible

  // Descarta 3 tramas iniciales
  float dp, t;
  for (int i = 0; i < 3; ++i) { readOnce(dp, t); delay(10); }

  return ok;
}

bool Adp910Driver::startContinuous() {
  return writeCmd_(CMD_CONT_);
}

bool Adp910Driver::readOnce(float& dp_Pa, float& t_C) {
  if (!bus_) return false;

  const uint8_t N = 6; // DP_H,DP_L,CRC, T_H,T_L,CRC
  int got = bus_->requestFrom((int)addr_, (int)N);
  if (got != N) return false;

  uint8_t raw[N];
  for (uint8_t i = 0; i < N; ++i) raw[i] = bus_->read();

  if (crc8_(raw + 0, 2) != raw[2]) return false; // CRC DP
  if (crc8_(raw + 3, 2) != raw[5]) return false; // CRC T

  const int16_t dp_raw = (int16_t)((raw[0] << 8) | raw[1]);
  const int16_t t_raw  = (int16_t)((raw[3] << 8) | raw[4]);

  dp_Pa = dp_raw / 60.0f;   // ±500 Pa => scale 60
  t_C   = t_raw  / 200.0f;  // °C => scale 200
  return true;
}

void Adp910Driver::calibrateMs(uint32_t ms) {
  Serial.println(F("Calibrando (mantén ambas tomas a la misma presión)..."));
  const uint32_t T_END = millis() + ms;
  double acc = 0.0; uint32_t ok = 0;

  while ((int32_t)(T_END - millis()) > 0) {
    float dp, t;
    if (readOnce(dp, t)) { acc += (double)dp; ok++; }
    delay(50); // ~20 Hz
  }

  if (ok > 0) {
    offsetPa_ = (float)(acc / (double)ok);
    Serial.print(F("Offset (no persistente) = "));
    Serial.print(offsetPa_, 4); Serial.println(F(" Pa"));
  } else {
    offsetPa_ = 0.0f;
    Serial.println(F("Calibración sin lecturas válidas. Offset=0.0 Pa"));
  }
}

void Adp910Driver::setOffsetPa(float off) { offsetPa_ = off; }
float Adp910Driver::offsetPa() const { return offsetPa_; }

uint8_t  Adp910Driver::address() const { return addr_; }
int      Adp910Driver::sdaPin()  const { return sda_; }
int      Adp910Driver::sclPin()  const { return scl_; }
uint32_t Adp910Driver::i2cHz()   const { return hz_; }

uint8_t Adp910Driver::crc8_(const uint8_t* d, uint8_t n){
  uint8_t c = 0xFF; // poly 0x31, init 0xFF, no reflect, xorout 0x00
  for(uint8_t i=0;i<n;i++){ c ^= d[i]; for(uint8_t b=0;b<8;b++) c = (c & 0x80) ? (uint8_t)((c<<1)^0x31) : (uint8_t)(c<<1); }
  return c;
}

bool Adp910Driver::writeCmd_(uint16_t cmd){
  if (!bus_) return false;
  bus_->beginTransmission(addr_);
  bus_->write((uint8_t)(cmd >> 8));
  bus_->write((uint8_t)(cmd & 0xFF));
  return (bus_->endTransmission() == 0);
}