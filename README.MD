# Adp910Driver

Arduino driver for the **ASAIR ADP910 differential pressure sensor** over I²C.

This library encapsulates the low-level protocol (continuous mode `0x361E`, CRC-8 checks, conversion to Pa and °C) and provides a simple interface for initialization, reading, and non-persistent calibration.

---

## ✨ Features
- Supports **continuous measurement mode** (`0x361E`).
- Reads **ΔP (Pa)** and **temperature (°C)** with CRC-8 verification.
- Provides a **5-second calibration** routine to compute and apply an offset (not persistent).
- Configurable I²C pins and clock (default: 25 kHz for robustness).
- Easy to integrate in larger Arduino/ESP32 projects.

---

## 📦 Installation
1. Clone this repository or download the ZIP:
   ```bash
   git clone https://github.com/polo14va/Adp910Driver.git

	2.	Copy the Adp910Driver folder into your Arduino libraries/ folder.
	3.	Restart the Arduino IDE.
	4.	The library should now appear under File → Examples → Adp910Driver.

---

🚀 Basic Usage

#include <Wire.h>
#include <Adp910Driver.h>

// Example pins for ESP32-C6 DevKitM1
constexpr int SDA_PIN = 8;
constexpr int SCL_PIN = 9;

Adp910Driver adp; // default I²C address 0x25

void setup() {
  Serial.begin(115200);
  delay(80);

  // Initialize I²C and enter continuous mode
  adp.begin(Wire, SDA_PIN, SCL_PIN, 25000);

  // Optional: run a 5-second calibration (non-persistent offset)
  adp.calibrateMs(5000);
}

void loop() {
  float dp, t;
  if (adp.readOnce(dp, t)) {
    float dp_corr = dp - adp.offsetPa();
    Serial.print("DP="); Serial.print(dp_corr, 3); Serial.print(" Pa  T=");
    Serial.print(t, 2); Serial.println(" °C");
  } else {
    Serial.println("Read/CRC failed");
  }
  delay(1000);
}


⸻

📂 Examples
	•	BasicRead – Initialize the sensor, run a 5-second calibration, and print ΔP and temperature every second.

---

📖 API Reference
	•	bool begin(TwoWire& wire, int sda_pin, int scl_pin, uint32_t i2c_hz = 25000)
Initializes the sensor and enters continuous mode.
	•	bool readOnce(float& dp_Pa, float& t_C)
Reads one ΔP [Pa] and temperature [°C] sample.
	•	void calibrateMs(uint32_t ms = 5000)
Runs calibration for the given time and stores offset.
	•	float offsetPa() const
Returns current offset in Pa.
	•	void setOffsetPa(float off)
Sets offset manually.

---

🧑‍💻 Author

Developed by Pedro Martínez

---

📜 License

MIT License

---