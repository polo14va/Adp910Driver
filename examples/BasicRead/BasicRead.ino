#include <Wire.h>
#include <Adp910Driver.h>

// Pines que te funcionan hoy en tu Nano ESP32-C6 DevKitM1
constexpr int SDA_PIN = 8;
constexpr int SCL_PIN = 9;

Adp910Driver adp; // addr por defecto 0x25

void setup() {
  Serial.begin(115200);
  delay(80);

  // Inicia I2C + modo continuo + descarta 3 tramas
  adp.begin(Wire, SDA_PIN, SCL_PIN, 25000);

  // Calibración no persistente de 5 s
  adp.calibrateMs(5000);
}

void loop() {
  float dp, t;
  if (adp.readOnce(dp, t)) {
    // Aplica offset manualmente si quieres (o guárdalo y réstalo tú)
    float dp_corr = dp - adp.offsetPa();

    Serial.print("DP="); Serial.print(dp_corr, 3); Serial.print(" Pa  (raw=");
    Serial.print(dp, 3); Serial.print(")  T=");
    Serial.print(t, 2); Serial.println(" °C");
  } else {
    Serial.println("Lectura/CRC fallida");
  }
  delay(1000);
}