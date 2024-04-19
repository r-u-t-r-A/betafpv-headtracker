#include <Arduino.h>
#include <SPI.h>
#include "SparkFun_BMI270_Arduino_Library.h"
#include "crsf.c"

#define debug

//#define ELRS_Serial Serial1
HardwareSerial Serial1(USART1);
#define GYRO_CS PA4
#define GYRO_INT1 PA1
#define GYRO_CLOCK 100000

#define max_x_ang 90
#define min_x_ang -90
#define max_y_ang 90
#define min_y_ang -90
#define max_z_ang 90
#define min_z_ang -90
#define control_protocol 2
// Create a new sensor object
BMI270 imu;

#define CALIB_COUNT 1024

bool calibration = false;

float gyroCalibX, gyroCalibY, gyroCalibZ;

float gyroAngleX, gyroAngleY, gyroAngleZ;
float elapsedTime, currentTime, previousTime;

int i = 0;

void map_data() {
  previousTime = currentTime;        // Previous time is stored before the actual time read
  currentTime = millis();            // Current time actual time read
  elapsedTime = (currentTime - previousTime) / 1000; // Divide by 1000 to get seconds
  // Get measurements from the sensor. This must be called before accessing
  // the sensor data, otherwise it will never update
  imu.getSensorData();

  if (calibration == true) {
  
    #ifdef debug
      SerialUSB.println("calib start");
    #endif
  
    if (i < CALIB_COUNT) {
  
      i++;
      gyroCalibX = gyroCalibX + imu.data.gyroX;
      gyroCalibY = gyroCalibY + imu.data.gyroY;
      gyroCalibZ = gyroCalibZ + imu.data.gyroZ;
      gyroAngleX = 0;
      gyroAngleY = 0;
      gyroAngleZ = 0;
  
    } else {
  
      gyroCalibX = gyroCalibX / CALIB_COUNT;
      gyroCalibY = gyroCalibY / CALIB_COUNT;
      gyroCalibZ = gyroCalibZ / CALIB_COUNT;
      i = 0;
      calibration = false; 
  
      #ifdef debug
        SerialUSB.print("calib:");
        SerialUSB.print("\t");
        SerialUSB.print("X: ");
        SerialUSB.print(gyroCalibX, 3);
        SerialUSB.print("\t");
        SerialUSB.print("Y: ");
        SerialUSB.print(gyroCalibY, 3);
        SerialUSB.print("\t");
        SerialUSB.print("Z: ");
        SerialUSB.println(gyroCalibZ, 3);
      #endif
    }

  }
  #ifdef debug
    SerialUSB.print("Rotation in deg/sec");
    SerialUSB.print("\t");
    SerialUSB.print("X: ");
    SerialUSB.print((imu.data.gyroX - gyroCalibX), 3);
    SerialUSB.print("\t");
    SerialUSB.print("Y: ");
    SerialUSB.print((imu.data.gyroY - gyroCalibY), 3);
    SerialUSB.print("\t");
    SerialUSB.print("Z: ");
    SerialUSB.print((imu.data.gyroZ - gyroCalibZ), 3);
    SerialUSB.print("\t");
  #endif

  gyroAngleX = gyroAngleX + (imu.data.gyroX - gyroCalibX) * elapsedTime; // deg/s * s = deg
  gyroAngleY = gyroAngleY + (imu.data.gyroY - gyroCalibY) * elapsedTime;
  gyroAngleZ = gyroAngleZ + (imu.data.gyroZ - gyroCalibZ) * elapsedTime;
  
  gyroAngleX = constrain(gyroAngleX, min_x_ang, max_x_ang);
  gyroAngleY = constrain(gyroAngleY, min_y_ang, max_y_ang);
  gyroAngleZ = constrain(gyroAngleZ, min_z_ang, max_z_ang);
  
  #ifdef debug
    SerialUSB.print("Position in deg:");
    SerialUSB.print("\t");
    SerialUSB.print("X: ");
    SerialUSB.print(gyroAngleX, 3);
    SerialUSB.print("\t");
    SerialUSB.print("Y: ");
    SerialUSB.print(gyroAngleY, 3);
    SerialUSB.print("\t");
    SerialUSB.print("Z: ");
    SerialUSB.println(gyroAngleZ, 3);
  #endif

    rcChannels[0] = map(gyroAngleX, min_x_ang, max_x_ang, RC_CHANNEL_MIN, RC_CHANNEL_MAX);
    rcChannels[1] = map(gyroAngleY, min_y_ang, max_y_ang, RC_CHANNEL_MIN, RC_CHANNEL_MAX);
    rcChannels[2] = map(gyroAngleZ, min_z_ang, max_z_ang, RC_CHANNEL_MIN, RC_CHANNEL_MAX);

    crsfPreparePacket(crsfPacket, rcChannels);
    ELRS_Serial.write(crsfPacket, CRSF_PACKET_SIZE); //Send data over CRSF to tx module
}

void setup() {
  
  SerialUSB.begin(115200);
  SerialUSB.println("headtracker booting");
  ELRS_Serial.begin(115200);
  // Initialize the SPI library
  SPI.setMISO(PA6);
  SPI.setMOSI(PA7);
  SPI.setSCLK(PA5);
  SPI.begin();

  // Check if sensor is connected and initialize
  // Clock frequency is optional (defaults to 100kHz)
  while(imu.beginSPI(GYRO_CS, GYRO_CLOCK) != BMI2_OK) {
    // Not connected, inform user
    SerialUSB.println("Error: BMI270 not connected, check wiring and CS pin!");

    // Wait a bit to see if connection is established
    delay(1000);
    return;
  }
  int8_t err = BMI2_OK;

    // Set accelerometer config
    bmi2_sens_config accelConfig;
    accelConfig.type = BMI2_ACCEL;
    accelConfig.cfg.acc.odr = BMI2_ACC_ODR_50HZ;
    accelConfig.cfg.acc.bwp = BMI2_ACC_OSR4_AVG1;
    accelConfig.cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;
    accelConfig.cfg.acc.range = BMI2_ACC_RANGE_2G;
    err = imu.setConfig(accelConfig);

    // Set gyroscope config
    bmi2_sens_config gyroConfig;
    gyroConfig.type = BMI2_GYRO;
    gyroConfig.cfg.gyr.odr = BMI2_GYR_ODR_50HZ;
    gyroConfig.cfg.gyr.bwp = BMI2_GYR_OSR4_MODE;
    gyroConfig.cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;
    gyroConfig.cfg.gyr.ois_range = BMI2_GYR_OIS_250;
    gyroConfig.cfg.gyr.range = BMI2_GYR_RANGE_125;
    gyroConfig.cfg.gyr.noise_perf = BMI2_PERF_OPT_MODE;
    err = imu.setConfig(gyroConfig);

    // Check whether the config settings above were valid
    while(err != BMI2_OK)
    {
        // Not valid, determine which config was the problem
        if(err == BMI2_E_ACC_INVALID_CFG)
        {
            SerialUSB.println("Accelerometer config not valid!");
        }
        else if(err == BMI2_E_GYRO_INVALID_CFG)
        {
            SerialUSB.println("Gyroscope config not valid!");
        }
        else if(err == BMI2_E_ACC_GYR_INVALID_CFG)
        {
            SerialUSB.println("Both configs not valid!");
        }
        else
        {
            SerialUSB.print("Unknown error: ");
            SerialUSB.println(err);
        }
        delay(1000);
    }

    SerialUSB.println("Configuration valid! Beginning measurements");
    delay(1000);
    SerialUSB.println("BMI270 connected!");
  
}

void loop() {
  map_data();
  delay(10);
  char read = SerialUSB.read();
  switch (read)
  {
  case 'r':
  SerialUSB.println("reset");
     gyroAngleX = 0;
    gyroAngleY = 0;
    gyroAngleZ = 0;
    break;
  case 'c':
  SerialUSB.println("calib");
    calibration = true;
    break;
  case 'w':
  SerialUSB.println("wifi");
    buildElrsPacket(crsfCmdPacket, ELRS_LUA_COMMAND_ENABLE_WIFI, 4);
    ELRS_Serial.write(crsfCmdPacket, CRSF_CMD_PACKET_SIZE);
    break;
 
  }
  
}
