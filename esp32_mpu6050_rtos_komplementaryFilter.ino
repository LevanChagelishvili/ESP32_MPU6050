#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu6050;
Adafruit_Sensor *mpu_accel, *mpu_gyro;

typedef struct {
  float accX;
  float accY;
  float accZ;
  float gyroX;
  float gyroY;
  float gyroZ;
  float roll;
  float pitch;
} MPU6050_struct;

MPU6050_struct mpu;

SemaphoreHandle_t xMutexMPU6050 = NULL;
TickType_t timeOut = 1000;

const float alpha = 0.5; // Complementary filter constant

void mpu6050Task(void *pvParam) {
  mpu6050.begin();
  mpu_accel = mpu6050.getAccelerometerSensor();
  mpu_gyro = mpu6050.getGyroSensor();

  // Calibration
  float gyroXOffset = 0, gyroYOffset = 0, gyroZOffset = 0;
  float accelXOffset = 0, accelYOffset = 0, accelZOffset = 0;
  int numReadings = 1000;

  for (int i = 0; i < numReadings; i++) {
    sensors_event_t accel, gyro;
    mpu_accel->getEvent(&accel);
    mpu_gyro->getEvent(&gyro);
    
    gyroXOffset += gyro.gyro.x;
    gyroYOffset += gyro.gyro.y;
    gyroZOffset += gyro.gyro.z;
    accelXOffset += accel.acceleration.x;
    accelYOffset += accel.acceleration.y;
    accelZOffset += accel.acceleration.z;

    vTaskDelay(10);
  }

  gyroXOffset /= numReadings;
  gyroYOffset /= numReadings;
  gyroZOffset /= numReadings;
  accelXOffset /= numReadings;
  accelYOffset /= numReadings;
  accelZOffset /= numReadings;

  sensors_event_t accel, gyro;
  unsigned long lastTime = 0;
  const float gyroSensitivity = 131; // Gyroscope sensitivity for 250deg/s range

  while (1) {
    if (xSemaphoreTake(xMutexMPU6050, timeOut) == pdPASS) {
      mpu_accel->getEvent(&accel);
      mpu_gyro->getEvent(&gyro);

      // Subtracting the offsets to get calibrated data
      mpu.accX = accel.acceleration.x - accelXOffset;
      mpu.accY = accel.acceleration.y - accelYOffset;
      mpu.accZ = accel.acceleration.z - accelZOffset;
      mpu.gyroX = (gyro.gyro.x - gyroXOffset) / gyroSensitivity;
      mpu.gyroY = (gyro.gyro.y - gyroYOffset) / gyroSensitivity;
      mpu.gyroZ = (gyro.gyro.z - gyroZOffset) / gyroSensitivity;

      // Debugging: Print calibrated accelerometer data
      /*
      Serial.print("Calibrated Acc X: "); Serial.print(mpu.accX);
      Serial.print(" Y: "); Serial.print(mpu.accY);
      Serial.print(" Z: "); Serial.println(mpu.accZ);
      */
      unsigned long currentTime = millis();
      float elapsedTime = (currentTime - lastTime) / 1000.0; // Convert to seconds
      lastTime = currentTime;

      // Calculate roll and pitch from the accelerometer data
      float rollAcc = atan2(mpu.accY, mpu.accZ) * 180 / PI;
      float pitchAcc = atan2(-mpu.accX, sqrt(mpu.accY * mpu.accY + mpu.accZ * mpu.accZ)) * 180 / PI;

      // Complementary filter
      mpu.roll = alpha * (mpu.roll + mpu.gyroX * elapsedTime) + (1 - alpha) * rollAcc;
      mpu.pitch = alpha * (mpu.pitch + mpu.gyroY * elapsedTime) + (1 - alpha) * pitchAcc;

      // Print roll and pitch
      
      Serial.print(mpu.roll);
      Serial.print("\t");
      Serial.print(mpu.pitch);
      Serial.print("\t\n");
      xSemaphoreGive(xMutexMPU6050);
    }
    vTaskDelay(500);
  }
}

void setup() {
  Serial.begin(115200);
  xMutexMPU6050 = xSemaphoreCreateMutex();
  xTaskCreate(mpu6050Task, "MPU6050", 1024 * 8, NULL, 1, NULL);
  vTaskDelay(1000);
}

void loop() {}
