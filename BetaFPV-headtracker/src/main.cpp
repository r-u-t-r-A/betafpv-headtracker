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
  SerialUSB.print("Rotation in deg/sec");
    SerialUSB.print("\t");
    SerialUSB.print("X: ");
    SerialUSB.print(imu.data.gyroX, 3);
    SerialUSB.print("\t");
    SerialUSB.print("Y: ");
    SerialUSB.print(imu.data.gyroY, 3);
    SerialUSB.print("\t");
    SerialUSB.print("Z: ");
    SerialUSB.println(imu.data.gyroZ, 3);
    
}

void setup() {
  
  SerialUSB.begin(115200);
  SerialUSB.println("headtracker booting");
  SPI.setMISO(PA6);
  SPI.setMOSI(PA7);
  SPI.setSCLK(PA5);
  // Initialize the SPI library
    SPI.begin();

    // Check if sensor is connected and initialize
    // Clock frequency is optional (defaults to 100kHz)
    while(imu.beginSPI(GYRO_CS, GYRO_CLOCK) != BMI2_OK)
    {
        // Not connected, inform user
        SerialUSB.println("Error: BMI270 not connected, check wiring and CS pin!");

        // Wait a bit to see if connection is established
        delay(1000);
      return;
    }

    SerialUSB.println("BMI270 connected!");
  
}

void loop() {
  map_data();
  delay(20);
}
