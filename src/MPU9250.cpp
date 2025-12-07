#include "MPU9250.h"

// Constructors
MPU9250::MPU9250() : _useI2C(true), _useHardwareSPI(false) {
    // I2C with default pins
}

MPU9250::MPU9250(uint8_t sdaPin, uint8_t sclPin) 
    : _useI2C(true), _useHardwareSPI(false), _pinSDA(sdaPin), _pinSCL(sclPin) {
}

MPU9250::MPU9250(uint8_t csPin) 
    : _useI2C(false), _useHardwareSPI(true), _pinCS(csPin) {
}

MPU9250::MPU9250(uint8_t sckPin, uint8_t misoPin, uint8_t mosiPin, uint8_t csPin)
    : _useI2C(false), _useHardwareSPI(false), _pinCS(csPin), _pinSCK(sckPin), 
      _pinMISO(misoPin), _pinMOSI(mosiPin) {
}

// Initialize device
bool MPU9250::begin(uint8_t i2cAddr, uint8_t chipId) {
    if (_useI2C) {
        _i2cAddress = i2cAddr;
        if (_pinSDA != 0xFF && _pinSCL != 0xFF) {
            Wire.begin(_pinSDA, _pinSCL);
        } else {
            Wire.begin();
        }
    } else {
        if (_useHardwareSPI) {
            SPI.begin();
            pinMode(_pinCS, OUTPUT);
            deselect();
        } else {
            pinMode(_pinCS, OUTPUT);
            pinMode(_pinSCK, OUTPUT);
            pinMode(_pinMOSI, OUTPUT);
            pinMode(_pinMISO, INPUT);
            deselect();
            digitalWrite(_pinSCK, LOW);
            digitalWrite(_pinMOSI, LOW);
        }
    }

    // Reset device
    write8(MPU9250_REG_PWR_MGMT_1, 0x80);
    delay(100);
    write8(MPU9250_REG_PWR_MGMT_1, 0x00);
    delay(100);

    // Check CHIP_ID
    _chipId = read8(MPU9250_REG_WHO_AM_I);
    if (_chipId != chipId) {
        return false;
    }

    // Wake up device
    write8(MPU9250_REG_PWR_MGMT_1, 0x00);

    return true;
}

// Configuration functions
void MPU9250::setAccelerometerRange(uint8_t range) {
    uint8_t reg = read8(MPU9250_REG_ACCEL_CONFIG);
    reg &= ~0x18;  // Clear AFS bits
    reg |= range;
    write8(MPU9250_REG_ACCEL_CONFIG, reg);
    
    // Update scale factor
    switch (range) {
        case MPU9250_ACCEL_FS_2G:  _accelScale = 16384.0f; break;
        case MPU9250_ACCEL_FS_4G:  _accelScale = 8192.0f; break;
        case MPU9250_ACCEL_FS_8G:  _accelScale = 4096.0f; break;
        case MPU9250_ACCEL_FS_16G: _accelScale = 2048.0f; break;
    }
}

void MPU9250::setGyroRange(uint8_t range) {
    uint8_t reg = read8(MPU9250_REG_GYRO_CONFIG);
    reg &= ~0x18;  // Clear FS bits
    reg |= range;
    write8(MPU9250_REG_GYRO_CONFIG, reg);
    
    // Update scale factor
    switch (range) {
        case MPU9250_GYRO_FS_250DPS:  _gyroScale = 131.0f; break;
        case MPU9250_GYRO_FS_500DPS:  _gyroScale = 65.5f; break;
        case MPU9250_GYRO_FS_1000DPS: _gyroScale = 32.8f; break;
        case MPU9250_GYRO_FS_2000DPS: _gyroScale = 16.4f; break;
    }
}

void MPU9250::setFilterBandwidth(uint8_t bandwidth) {
    uint8_t reg = read8(MPU9250_REG_CONFIG);
    reg &= ~0x07;  // Clear DLPF bits
    reg |= bandwidth;
    write8(MPU9250_REG_CONFIG, reg);
}

void MPU9250::setGyroFilterBandwidth(uint8_t bandwidth) {
    setFilterBandwidth(bandwidth);
}

void MPU9250::setAccelFilterBandwidth(uint8_t bandwidth) {
    uint8_t reg = read8(MPU9250_REG_ACCEL_CONFIG_2);
    reg &= ~0x07;  // Clear DLPF bits
    reg |= bandwidth;
    write8(MPU9250_REG_ACCEL_CONFIG_2, reg);
}

void MPU9250::setSampleRateDivider(uint8_t divider) {
    write8(MPU9250_REG_SMPLRT_DIV, divider);
}

// Read accelerometer data
AccelData MPU9250::readAccel() {
    AccelData data;
    readAccel(data.x, data.y, data.z);
    return data;
}

void MPU9250::readAccel(float &x, float &y, float &z) {
    uint8_t buffer[6];
    readRegisters(MPU9250_REG_ACCEL_XOUT_H, buffer, 6);
    
    int16_t rawX = (int16_t)((buffer[0] << 8) | buffer[1]);
    int16_t rawY = (int16_t)((buffer[2] << 8) | buffer[3]);
    int16_t rawZ = (int16_t)((buffer[4] << 8) | buffer[5]);
    
    x = rawX / _accelScale;
    y = rawY / _accelScale;
    z = rawZ / _accelScale;
}

// Read gyroscope data
GyroData MPU9250::readGyro() {
    GyroData data;
    readGyro(data.x, data.y, data.z);
    return data;
}

void MPU9250::readGyro(float &x, float &y, float &z) {
    uint8_t buffer[6];
    readRegisters(MPU9250_REG_GYRO_XOUT_H, buffer, 6);
    
    int16_t rawX = (int16_t)((buffer[0] << 8) | buffer[1]);
    int16_t rawY = (int16_t)((buffer[2] << 8) | buffer[3]);
    int16_t rawZ = (int16_t)((buffer[4] << 8) | buffer[5]);
    
    x = rawX / _gyroScale;
    y = rawY / _gyroScale;
    z = rawZ / _gyroScale;
}

// Magnetometer functions
bool MPU9250::initMagnetometer() {
    enableI2CMaster();
    
    // Reset magnetometer
    writeAK8963Register(MPU9250_AK8963_REG_CNTL2, 0x01);
    delay(100);
    
    // Check WHO_AM_I
    uint8_t whoAmI = readAK8963Register(MPU9250_AK8963_REG_WIA);
    if (whoAmI != MPU9250_AK8963_WHO_AM_I) {
        return false;
    }
    
    // Read calibration data
    getMagnetometerCalibration();
    
    // Set 16-bit mode and continuous measurement mode 1 (8Hz)
    uint8_t cntl1 = readAK8963Register(MPU9250_AK8963_REG_CNTL1);
    cntl1 &= ~0x0F;  // Clear mode bits
    cntl1 |= MPU9250_AK8963_BIT_16BIT | MPU9250_AK8963_MODE_CONT_MEAS_1;
    writeAK8963Register(MPU9250_AK8963_REG_CNTL1, cntl1);
    delay(10);
    
    // Enable continuous reading of magnetometer data
    write8(MPU9250_REG_I2C_SLV0_ADDR, MPU9250_AK8963_I2C_ADDR | 0x80);  // Read mode
    write8(MPU9250_REG_I2C_SLV0_REG, MPU9250_AK8963_REG_HXL);
    write8(MPU9250_REG_I2C_SLV0_CTRL, 0x80 | 0x08);  // Enable + 8 bytes
    delay(10);
    
    return true;
}

MagData MPU9250::readMag() {
    MagData data;
    readMag(data.x, data.y, data.z);
    return data;
}

void MPU9250::readMag(float &x, float &y, float &z) {
    uint8_t buffer[6];
    readAK8963Data(buffer);
    
    int16_t rawX = (int16_t)((buffer[1] << 8) | buffer[0]);
    int16_t rawY = (int16_t)((buffer[3] << 8) | buffer[2]);
    int16_t rawZ = (int16_t)((buffer[5] << 8) | buffer[4]);
    
    // Scale factor: 4912.0 / 32760.0 (for ±4800µT range in 16-bit mode)
    const float scaleFactor = 4912.0f / 32760.0f;
    
    x = rawX * scaleFactor * _magCorrFactorX;
    y = rawY * scaleFactor * _magCorrFactorY;
    z = rawZ * scaleFactor * _magCorrFactorZ;
}

// Read all sensor data (accelerometer, gyroscope, magnetometer)
void MPU9250::readAll(AccelData &accel, GyroData &gyro, MagData &mag) {
    readAccel(accel.x, accel.y, accel.z);
    readGyro(gyro.x, gyro.y, gyro.z);
    readMag(mag.x, mag.y, mag.z);
}

// Private helper functions
void MPU9250::select() {
    digitalWrite(_pinCS, LOW);
}

void MPU9250::deselect() {
    digitalWrite(_pinCS, HIGH);
}

uint8_t MPU9250::spiTransfer(uint8_t value) {
    if (_useHardwareSPI) {
        return SPI.transfer(value);
    }
    
    // Software SPI (mode 0: CPOL=0, CPHA=0)
    uint8_t received = 0;
    for (uint8_t i = 0; i < 8; i++) {
        digitalWrite(_pinMOSI, (value & 0x80) ? HIGH : LOW);
        value <<= 1;
        
        digitalWrite(_pinSCK, HIGH);
        
        received <<= 1;
        if (digitalRead(_pinMISO)) {
            received |= 0x01;
        }
        
        digitalWrite(_pinSCK, LOW);
    }
    return received;
}

uint8_t MPU9250::read8(uint8_t reg) {
    if (_useI2C) {
        Wire.beginTransmission(_i2cAddress);
        Wire.write(reg);
        Wire.endTransmission(false);
        Wire.requestFrom(_i2cAddress, (uint8_t)1);
        if (Wire.available()) {
            return Wire.read();
        }
        return 0;
    } else {
        // SPI read: set MSB to 1
        uint8_t value = 0;
        if (_useHardwareSPI) {
            static const SPISettings settings(8000000, MSBFIRST, SPI_MODE0);
            SPI.beginTransaction(settings);
            select();
            SPI.transfer(reg | 0x80);
            value = SPI.transfer(0x00);
            deselect();
            SPI.endTransaction();
        } else {
            select();
            spiTransfer(reg | 0x80);
            value = spiTransfer(0x00);
            deselect();
        }
        return value;
    }
}

void MPU9250::write8(uint8_t reg, uint8_t value) {
    if (_useI2C) {
        Wire.beginTransmission(_i2cAddress);
        Wire.write(reg);
        Wire.write(value);
        Wire.endTransmission();
    } else {
        // SPI write: MSB = 0
        if (_useHardwareSPI) {
            static const SPISettings settings(8000000, MSBFIRST, SPI_MODE0);
            SPI.beginTransaction(settings);
            select();
            SPI.transfer(reg & 0x7F);
            SPI.transfer(value);
            deselect();
            SPI.endTransaction();
        } else {
            select();
            spiTransfer(reg & 0x7F);
            spiTransfer(value);
            deselect();
        }
    }
}

int16_t MPU9250::read16(uint8_t reg) {
    uint8_t buffer[2];
    readRegisters(reg, buffer, 2);
    return (int16_t)((buffer[0] << 8) | buffer[1]);
}

void MPU9250::readRegisters(uint8_t reg, uint8_t *buffer, uint8_t length) {
    if (_useI2C) {
        Wire.beginTransmission(_i2cAddress);
        Wire.write(reg);
        Wire.endTransmission(false);
        Wire.requestFrom(_i2cAddress, length);
        for (uint8_t i = 0; i < length && Wire.available(); i++) {
            buffer[i] = Wire.read();
        }
    } else {
        if (_useHardwareSPI) {
            static const SPISettings settings(8000000, MSBFIRST, SPI_MODE0);
            SPI.beginTransaction(settings);
            select();
            SPI.transfer(reg | 0x80);
            for (uint8_t i = 0; i < length; i++) {
                buffer[i] = SPI.transfer(0x00);
            }
            deselect();
            SPI.endTransaction();
        } else {
            select();
            spiTransfer(reg | 0x80);
            for (uint8_t i = 0; i < length; i++) {
                buffer[i] = spiTransfer(0x00);
            }
            deselect();
        }
    }
}

// Magnetometer helper functions (I2C master mode)
void MPU9250::enableI2CMaster() {
    // Enable I2C master mode
    uint8_t userCtrl = read8(MPU9250_REG_USER_CTRL);
    userCtrl |= 0x20;  // Set I2C_MST_EN bit
    write8(MPU9250_REG_USER_CTRL, userCtrl);
    delay(10);
    
    // Configure I2C master clock to 400kHz
    write8(MPU9250_REG_I2C_MST_CTRL, 0x0D);
    delay(10);
}

uint8_t MPU9250::readAK8963Register(uint8_t reg) {
    // Configure I2C slave 0 to read from AK8963
    write8(MPU9250_REG_I2C_SLV0_ADDR, MPU9250_AK8963_I2C_ADDR | 0x80);  // Read mode
    write8(MPU9250_REG_I2C_SLV0_REG, reg);
    write8(MPU9250_REG_I2C_SLV0_CTRL, 0x80 | 0x01);  // Enable + 1 byte
    delay(10);
    
    // Read the data from external sensor data register
    uint8_t value = read8(MPU9250_REG_EXT_SLV_SENS_DATA_00);
    
    // Re-enable continuous reading of magnetometer data
    write8(MPU9250_REG_I2C_SLV0_ADDR, MPU9250_AK8963_I2C_ADDR | 0x80);
    write8(MPU9250_REG_I2C_SLV0_REG, MPU9250_AK8963_REG_HXL);
    write8(MPU9250_REG_I2C_SLV0_CTRL, 0x80 | 0x08);  // Enable + 8 bytes
    delay(10);
    
    return value;
}

void MPU9250::writeAK8963Register(uint8_t reg, uint8_t value) {
    // Configure I2C slave 0 to write to AK8963
    write8(MPU9250_REG_I2C_SLV0_ADDR, MPU9250_AK8963_I2C_ADDR);  // Write mode
    write8(MPU9250_REG_I2C_SLV0_REG, reg);
    write8(MPU9250_REG_I2C_SLV0_DO, value);
    write8(MPU9250_REG_I2C_SLV0_CTRL, 0x80 | 0x01);  // Enable + 1 byte
    delay(10);
}

void MPU9250::readAK8963Data(uint8_t *buffer) {
    readRegisters(MPU9250_REG_EXT_SLV_SENS_DATA_00, buffer, 6);
}

void MPU9250::getMagnetometerCalibration() {
    // Set to FUSE ROM access mode to read calibration data
    uint8_t cntl1 = readAK8963Register(MPU9250_AK8963_REG_CNTL1);
    cntl1 &= ~0x0F;  // Clear mode bits
    cntl1 |= MPU9250_AK8963_MODE_FUSE_ROM;
    writeAK8963Register(MPU9250_AK8963_REG_CNTL1, cntl1);
    delay(10);
    
    // Read calibration values
    uint8_t asaX = readAK8963Register(MPU9250_AK8963_REG_ASAX);
    uint8_t asaY = readAK8963Register(MPU9250_AK8963_REG_ASAY);
    uint8_t asaZ = readAK8963Register(MPU9250_AK8963_REG_ASAZ);
    
    // Calculate correction factors: (0.5 * (ASA - 128) / 128) + 1
    _magCorrFactorX = (0.5f * ((float)asaX - 128.0f) / 128.0f) + 1.0f;
    _magCorrFactorY = (0.5f * ((float)asaY - 128.0f) / 128.0f) + 1.0f;
    _magCorrFactorZ = (0.5f * ((float)asaZ - 128.0f) / 128.0f) + 1.0f;
}

