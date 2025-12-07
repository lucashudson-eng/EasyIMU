#include <GY91.h>

// If using SPI

// #define GY_SCK  (18)
// #define GY_MISO (19)
// #define GY_MOSI (23)
#define MPU_CS   (5)
#define BMP_CS   (4)

GY91 gy91(MPU_CS, BMP_CS, false); // SPI with hardware default pins
// GY91 gy91(GY_SCK, GY_MISO, GY_MOSI, MPU_CS, BMP_CS); // Custom SPI pins

// If using I2C

// #define GY_SDA  (21)
// #define GY_SCL  (22)

// GY91 gy91; // Default constructor uses I2C with default pins
// GY91 gy91(GY_SDA, GY_SCL, true); // Custom I2C pins

void setup() {
  Serial.begin(115200);
  delay(100);

  Serial.println("Initializing GY-91...");
  if (!gy91.begin()) {
    Serial.println("Error: GY-91 sensors not found or invalid CHIP_ID.");
    while (true) {
      delay(1000);
    }
  }

  Serial.println("GY-91 successfully detected!");
  
  // Configure MPU9250 sensor settings
  Serial.println("Configuring MPU9250...");
  
  // Set accelerometer range to ±4g
  gy91.getMPU9250().setAccelerometerRange(MPU9250_ACCEL_FS_4G);
  Serial.println("  Accelerometer range: ±4g");
  
  // Set gyroscope range to ±500°/s
  gy91.getMPU9250().setGyroRange(MPU9250_GYRO_FS_500DPS);
  Serial.println("  Gyroscope range: ±500°/s");
  
  // Set gyroscope filter bandwidth to 41Hz
  gy91.getMPU9250().setFilterBandwidth(MPU9250_DLPF_41HZ);
  Serial.println("  Gyroscope filter: 41Hz");
  
  // Set accelerometer filter bandwidth to 20Hz
  gy91.getMPU9250().setAccelFilterBandwidth(MPU9250_ACCEL_DLPF_20HZ);
  Serial.println("  Accelerometer filter: 20Hz");
  
  // Set sample rate divider (1kHz / (1 + 4) = 200Hz)
  gy91.getMPU9250().setSampleRateDivider(4);
  Serial.println("  Sample rate: 200Hz");
  
  // Initialize magnetometer
  Serial.println("Initializing magnetometer...");
  if (!gy91.getMPU9250().initMagnetometer()) {
    Serial.println("  Warning: Magnetometer initialization failed.");
  } else {
    Serial.println("  Magnetometer initialized successfully!");
  }
  
  Serial.println();
  Serial.println("Reading all sensor data...");
  Serial.println();
}

void loop() {
  Serial.println("=== Method 1: Individual readings ===");
  
  // Read MPU9250 data individually
  AccelData accel = gy91.getMPU9250().readAccel();
  GyroData gyro = gy91.getMPU9250().readGyro();
  MagData mag = gy91.getMPU9250().readMag();
  
  // Read BMP280 data individually
  float altitude = gy91.getBMP280().readAltitude();

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
  Serial.print(mag.z, 2);

  // Print altitude (in m)
  Serial.print("  |  Altitude: ");
  Serial.print(altitude, 2);
  Serial.println(" m");

  delay(500);
  
  Serial.println();
  Serial.println("=== Method 2: Read all at once ===");
  
  // Declare data structures
  AccelData accelAll;
  GyroData gyroAll;
  MagData magAll;
  float altitudeAll;
  
  // Read all sensor data at once
  gy91.readAll(accelAll, gyroAll, magAll, altitudeAll);

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
  Serial.print(magAll.z, 2);

  // Print altitude (in m)
  Serial.print("  |  Altitude: ");
  Serial.print(altitudeAll, 2);
  Serial.println(" m");

  Serial.println();
  delay(500);  // 2Hz output rate
}
