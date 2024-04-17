#include <Arduino.h>
#include <SPI.h>
#include "SparkFun_BMI270_Arduino_Library.h"

#define GYRO_CS PA4
#define GYRO_INT1 PA1
// Create a new sensor object
BMI270 imu;

void map_data() {
  
}

void setup() {
SerialUSB.begin(115200);
SerialUSB.println("headtracker booting");
  
}

void loop() {
  // put your main code here, to run repeatedly:
}
