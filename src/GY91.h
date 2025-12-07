#pragma once

#include <MPU9250.h>
#include <BMP280.h>

class GY91 {
public:
    // Constructor: I2C using default pins (SDA/SCL default)
    GY91();

    // Constructor: I2C or SPI with 2 pins
    // For I2C (useI2C = true): pin1 = SDA, pin2 = SCL
    // For SPI (useI2C = false): pin1 = MPU9250 CS, pin2 = BMP280 CS (uses hardware SPI default pins)
    GY91(uint8_t pin1, uint8_t pin2, bool useI2C);

    // Constructor: SPI providing all pins (SCK, MISO, MOSI, CS for MPU9250, CS for BMP280)
    // For hardware SPI, use default SPI pins (typically SCK=18, MISO=19, MOSI=23 on ESP32)
    GY91(uint8_t sckPin, uint8_t misoPin, uint8_t mosiPin, uint8_t mpuCsPin, uint8_t bmpCsPin);

    // Initialize both sensors
    // For I2C: optionally specify I2C addresses (MPU9250 default 0x68, BMP280 default 0x76)
    bool begin(uint8_t mpuI2cAddr = MPU9250_I2C_ADDR, uint8_t bmpI2cAddr = BMP280_I2C_ADDR,
               uint8_t mpuChipId = MPU9250_CHIP_ID, uint8_t bmpChipId = BMP280_CHIP_ID);

    // Read all data from both sensors
    void readAll(AccelData &accel, GyroData &gyro, MagData &mag, float &altitude);

    // Access to individual sensors for specific methods
    MPU9250& getMPU9250() { return _mpu9250; }
    BMP280& getBMP280() { return _bmp280; }

private:
    MPU9250 _mpu9250;
    BMP280 _bmp280;
};