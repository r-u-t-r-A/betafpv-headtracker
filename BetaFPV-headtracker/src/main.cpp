#include <Arduino.h>
#include <SPI.h>
#include "SparkFun_BMI270_Arduino_Library.h"

#define GYRO_CS PA4
#define GYRO_INT1 PA1
#define GYRO_CLOCK 100000
// Create a new sensor object
BMI270 imu;

void map_data() {
  // Get measurements from the sensor. This must be called before accessing
    // the sensor data, otherwise it will never update
    imu.getSensorData();
  Serial.print("Rotation in deg/sec");
    Serial.print("\t");
    Serial.print("X: ");
    Serial.print(imu.data.gyroX, 3);
    Serial.print("\t");
    Serial.print("Y: ");
    Serial.print(imu.data.gyroY, 3);
    Serial.print("\t");
    Serial.print("Z: ");
    Serial.println(imu.data.gyroZ, 3);
}

void setup() {
  
SerialUSB.begin(115200);
SerialUSB.println("headtracker booting");
  // Initialize the SPI library
    SPI.begin();

    // Check if sensor is connected and initialize
    // Clock frequency is optional (defaults to 100kHz)
    while(imu.beginSPI(GYRO_CS, GYRO_CLOCK) != BMI2_OK)
    {
        // Not connected, inform user
        Serial.println("Error: BMI270 not connected, check wiring and CS pin!");

        // Wait a bit to see if connection is established
        delay(1000);
      return;
    }

    Serial.println("BMI270 connected!");
  
}

void loop() {
  map_data();
  delay(20);
}
