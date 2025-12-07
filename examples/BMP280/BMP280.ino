#include <BMP280.h>

// If using SPI

// #define BMP_SCK  (18)
// #define BMP_MISO (19)
// #define BMP_MOSI (23)
#define BMP_CS   (5)

BMP280 bmp(BMP_CS);
// BMP280 bmp(BMP_SCK, BMP_MISO, BMP_MOSI, BMP_CS);

// If using I2C

// #define BMP_SDA  (21)
// #define BMP_SCL  (22)

// BMP280 bmp; // Default constructor uses I2C with default pins
// BMP280 bmp(BMP_SDA, BMP_SCL); // Custom I2C pins

void setup() {
  Serial.begin(115200);
  delay(100);

  Serial.println("Initializing BMP280 (SPI)...");
  if (!bmp.begin()) {
    Serial.println("Error: BMP280 not found or invalid CHIP_ID.");
    while (true) {
      delay(1000);
    }
  }

  Serial.println("BMP280 successfully detected!");
}

void loop() {
  float temperature = bmp.readTemperature();
  float pressure = bmp.readPressure();
  float altitude = bmp.readAltitude();

  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println(" °C");

  Serial.print("Pressure: ");
  Serial.print(pressure);
  Serial.println(" Pa");

  Serial.print("Altitude: ");
  Serial.print(altitude);
  Serial.println(" m");

  Serial.println("---");
  delay(1000);
}