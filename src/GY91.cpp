#include "GY91.h"

// Constructor: I2C using default pins (SDA/SCL default)
GY91::GY91() : _mpu9250(), _bmp280() {
}

// Constructor: I2C or SPI with 2 pins
GY91::GY91(uint8_t pin1, uint8_t pin2, bool useI2C) {
    if (useI2C) {
        // I2C: pin1 = SDA, pin2 = SCL
        _mpu9250 = MPU9250(pin1, pin2);
        _bmp280 = BMP280(pin1, pin2);
    } else {
        // SPI: pin1 = MPU9250 CS, pin2 = BMP280 CS (hardware SPI)
        _mpu9250 = MPU9250(pin1);
        _bmp280 = BMP280(pin2);
    }
}

// Constructor: SPI providing all pins (SCK, MISO, MOSI, CS for MPU9250, CS for BMP280)
// For hardware SPI, use default SPI pins (typically SCK=18, MISO=19, MOSI=23 on ESP32)
GY91::GY91(uint8_t sckPin, uint8_t misoPin, uint8_t mosiPin, uint8_t mpuCsPin, uint8_t bmpCsPin)
    : _mpu9250(sckPin, misoPin, mosiPin, mpuCsPin), _bmp280(sckPin, misoPin, mosiPin, bmpCsPin) {
}

// Initialize both sensors
bool GY91::begin(uint8_t mpuI2cAddr, uint8_t bmpI2cAddr, uint8_t mpuChipId, uint8_t bmpChipId) {
    // Initialize MPU9250
    if (!_mpu9250.begin(mpuI2cAddr, mpuChipId)) {
        return false;
    }
    
    // Initialize BMP280
    if (!_bmp280.begin(bmpI2cAddr, bmpChipId)) {
        return false;
    }
    
    return true;
}

// Read all data from both sensors
void GY91::readAll(AccelData &accel, GyroData &gyro, MagData &mag, float &altitude) {
    // Read MPU9250 data (accelerometer, gyroscope, magnetometer)
    _mpu9250.readAll(accel, gyro, mag);
    
    // Read altitude from BMP280 (temperature and pressure are read internally)
    altitude = _bmp280.readAltitude();
}