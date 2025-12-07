#include <MPU9250.h>

// If using SPI

// #define MPU_SCK  (18)
// #define MPU_MISO (19)
// #define MPU_MOSI (23)
#define MPU_CS   (5)

MPU9250 mpu(MPU_CS); // Default constructor uses SPI with hardware default pins
// MPU9250 mpu(MPU_SCK, MPU_MISO, MPU_MOSI, MPU_CS); // Custom SPI pins

// If using I2C

// #define MPU_SDA  (21)
// #define MPU_SCL  (22)

// MPU9250 mpu; // Default constructor uses I2C with default pins
// MPU9250 mpu(MPU_SDA, MPU_SCL); // Custom I2C pins

void setup() {
  Serial.begin(115200);
  delay(100);

  Serial.println("Initializing MPU9250...");
  if (!mpu.begin()) {
    Serial.println("Error: MPU9250 not found or invalid CHIP_ID.");
    while (true) {
      delay(1000);
    }
  }

  Serial.println("MPU9250 successfully detected!");
  
  // Configure sensor settings
  Serial.println("Configuring sensor...");
  
  // Set accelerometer range to ±4g
  mpu.setAccelerometerRange(MPU9250_ACCEL_FS_4G);
  Serial.println("  Accelerometer range: ±4g");
  
  // Set gyroscope range to ±500°/s
  mpu.setGyroRange(MPU9250_GYRO_FS_500DPS);
  Serial.println("  Gyroscope range: ±500°/s");
  
  // Set gyroscope filter bandwidth to 41Hz
  mpu.setFilterBandwidth(MPU9250_DLPF_41HZ);
  Serial.println("  Gyroscope filter: 41Hz");
  
  // Set accelerometer filter bandwidth to 20Hz
  mpu.setAccelFilterBandwidth(MPU9250_ACCEL_DLPF_20HZ);
  Serial.println("  Accelerometer filter: 20Hz");
  
  // Set sample rate divider (1kHz / (1 + 4) = 200Hz)
  mpu.setSampleRateDivider(4);
  Serial.println("  Sample rate: 200Hz");
  
  // Initialize magnetometer
  Serial.println("Initializing magnetometer...");
  if (!mpu.initMagnetometer()) {
    Serial.println("  Warning: Magnetometer initialization failed.");
  } else {
    Serial.println("  Magnetometer initialized successfully!");
  }
  
  Serial.println();
  Serial.println("Reading accelerometer, gyroscope, and magnetometer data...");
  Serial.println();
}

void loop() {
  Serial.println("=== Method 1: Individual readings ===");
  
  // Read accelerometer data individually
  AccelData accel = mpu.readAccel();
  
  // Read gyroscope data individually
  GyroData gyro = mpu.readGyro();
  
  // Read magnetometer data individually
  MagData mag = mpu.readMag();

  // Print accelerometer data (in g)
  Serial.print("Accel (g): ");
  Serial.print("X=");
  Serial.print(accel.x, 3);
  Serial.print("  Y=");
  Serial.print(accel.y, 3);
  Serial.print("  Z=");
  Serial.print(accel.z, 3);

  // Print gyroscope data (in deg/s)
  Serial.print("  |  Gyro (deg/s): ");
  Serial.print("X=");
  Serial.print(gyro.x, 2);
  Serial.print("  Y=");
  Serial.print(gyro.y, 2);
  Serial.print("  Z=");
  Serial.print(gyro.z, 2);

  // Print magnetometer data (in µT)
  Serial.print("  |  Mag (µT): ");
  Serial.print("X=");
  Serial.print(mag.x, 2);
  Serial.print("  Y=");
  Serial.print(mag.y, 2);
  Serial.print("  Z=");
  Serial.println(mag.z, 2);

  delay(500);
  
  Serial.println();
  Serial.println("=== Method 2: Read all at once ===");
  
  // Declare data structures
  AccelData accelAll;
  GyroData gyroAll;
  MagData magAll;
  
  // Read all sensor data at once
  mpu.readAll(accelAll, gyroAll, magAll);

  // Print accelerometer data (in g)
  Serial.print("Accel (g): ");
  Serial.print("X=");
  Serial.print(accelAll.x, 3);
  Serial.print("  Y=");
  Serial.print(accelAll.y, 3);
  Serial.print("  Z=");
  Serial.print(accelAll.z, 3);

  // Print gyroscope data (in deg/s)
  Serial.print("  |  Gyro (deg/s): ");
  Serial.print("X=");
  Serial.print(gyroAll.x, 2);
  Serial.print("  Y=");
  Serial.print(gyroAll.y, 2);
  Serial.print("  Z=");
  Serial.print(gyroAll.z, 2);

  // Print magnetometer data (in µT)
  Serial.print("  |  Mag (µT): ");
  Serial.print("X=");
  Serial.print(magAll.x, 2);
  Serial.print("  Y=");
  Serial.print(magAll.y, 2);
  Serial.print("  Z=");
  Serial.println(magAll.z, 2);

  Serial.println();
  delay(500);  // 2Hz output rate
}