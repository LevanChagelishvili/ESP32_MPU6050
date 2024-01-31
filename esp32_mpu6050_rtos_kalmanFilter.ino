#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Kalman.h>

Adafruit_MPU6050 mpu6050;
Adafruit_Sensor *mpu_accel, *mpu_gyro;
Kalman kalmanRoll;
Kalman kalmanPitch;

struct MPU6050Data {
  float accX, accY, accZ;
  float gyroX, gyroY, gyroZ;
  float roll, pitch;
} mpu;

SemaphoreHandle_t xMutexMPU6050 = NULL;
const float gyroSensitivity = 131.0;

void setup() {
  Serial.begin(115200);
  if (!mpu6050.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  mpu_accel = mpu6050.getAccelerometerSensor();
  mpu_gyro = mpu6050.getGyroSensor();
  calibrateSensor();
  initializeKalmanFilters();

  xMutexMPU6050 = xSemaphoreCreateMutex();
  xTaskCreate(mpu6050Task, "MPU6050", 2048, NULL, 1, NULL);
}

void loop() {
  // Empty loop since FreeRTOS is handling the task
}

void calibrateSensor() {
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

    delay(10);
  }

  gyroXOffset /= numReadings;
  gyroYOffset /= numReadings;
  gyroZOffset /= numReadings;
  accelXOffset /= numReadings;
  accelYOffset /= numReadings;
  accelZOffset /= numReadings;

  mpu.accX = accelXOffset;
  mpu.accY = accelYOffset;
  mpu.accZ = accelZOffset;
  mpu.gyroX = gyroXOffset;
  mpu.gyroY = gyroYOffset;
  mpu.gyroZ = gyroZOffset;
}

void initializeKalmanFilters() {
  sensors_event_t accel;
  mpu_accel->getEvent(&accel);

  float rollStart = atan2(accel.acceleration.y, accel.acceleration.z) * 180 / PI;
  float pitchStart = atan2(-accel.acceleration.x, sqrt(accel.acceleration.y * accel.acceleration.y + accel.acceleration.z * accel.acceleration.z)) * 180 / PI;

  kalmanRoll.setAngle(rollStart);
  kalmanPitch.setAngle(pitchStart);
}

void mpu6050Task(void *pvParam) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = 10; // Delay for task, 10ms
  unsigned long lastTime = 0; // Initialize lastTime variable

  while (1) {
    if (xSemaphoreTake(xMutexMPU6050, portMAX_DELAY)) {
      sensors_event_t accel, gyro;
      mpu_accel->getEvent(&accel);
      mpu_gyro->getEvent(&gyro);

      mpu.accX = accel.acceleration.x - mpu.accX;
      mpu.accY = accel.acceleration.y - mpu.accY;
      mpu.accZ = accel.acceleration.z - mpu.accZ;
      mpu.gyroX = (gyro.gyro.x - mpu.gyroX) / gyroSensitivity;
      mpu.gyroY = (gyro.gyro.y - mpu.gyroY) / gyroSensitivity;
      mpu.gyroZ = (gyro.gyro.z - mpu.gyroZ) / gyroSensitivity;

      unsigned long currentTime = millis();
      float elapsedTime = (currentTime - lastTime) / 1000.0;
      lastTime = currentTime;

      float rollAcc = atan2(mpu.accY, mpu.accZ) * 180 / PI;
      float pitchAcc = atan2(-mpu.accX, sqrt(mpu.accY * mpu.accY + mpu.accZ * mpu.accZ)) * 180 / PI;

      mpu.roll = kalmanRoll.getAngle(rollAcc, mpu.gyroX, elapsedTime);
      mpu.pitch = kalmanPitch.getAngle(pitchAcc, mpu.gyroY, elapsedTime);

      Serial.print(mpu.roll);
      Serial.print("\t");
      Serial.println(mpu.pitch);

      xSemaphoreGive(xMutexMPU6050);
    }
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}
