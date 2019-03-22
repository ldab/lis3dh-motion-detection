/******************************************************************************
lis3dh-motion-detection.h
LIS3DH Arduino
Leonardo Bispo
Mar 03, 2019
https://github.com/ldab/lis3dh-motion-detection
Resources:
Uses Wire.h for i2c operation

Inspired by https://github.com/sparkfun/SparkFun_LIS3DH_Arduino_Library

Distributed as-is; no warranty is given.
******************************************************************************/

// Accelerometer provides different Power modes by changing output bit resolution
//#define LOW_POWER
//#define NORMAL_MODE
#define HIGH_RESOLUTION

// Enable Serial debbug on Serial UART to see registers wrote
#define LIS3DH_DEBUG Serial

#include "lis3dh-motion-detection.h"
#include "Wire.h"

uint16_t sampleRate = 1;  // HZ - Samples per second - 1, 10, 25, 50, 100, 200, 400, 1600, 5000
uint8_t accelRange = 2;   // Accelerometer range = 2, 4, 8, 16g

uint16_t errorsAndWarnings = 0;

LIS3DH myIMU(0x19); //Default address is 0x19.

void setup() {
  // put your setup code here, to run once:
  
  Serial.begin(115200);
  delay(1000); //wait until serial is open...
  
  if( myIMU.begin(sampleRate, 1, 1, 1, accelRange) != 0 )
  {
    Serial.print("Failed to initialize IMU.\n");
  }
  else
  {
    Serial.print("IMU initialized.\n");
  }
  
  //Detection threshold can be from 1 to 127 and depends on the Range
  //chosen above, change it and test accordingly to your application
  //Duration = timeDur x Seconds / sampleRate
  myIMU.intConf(INT_1, DET_MOVE, 13, 2);
  myIMU.intConf(INT_2, DET_STOP, 13, 10);

  uint8_t readData = 0;

  // Confirm configuration:
  myIMU.readRegister(&readData, LIS3DH_INT1_CFG);
  myIMU.readRegister(&readData, LIS3DH_INT2_CFG);

  // Get the ID:
  myIMU.readRegister(&readData, LIS3DH_WHO_AM_I);
  Serial.print("Who am I? 0x");
  Serial.println(readData, HEX);

}


void loop()
{

  int16_t dataHighres = 0;

  if( myIMU.readRegisterInt16( &dataHighres, LIS3DH_OUT_X_L ) != 0 )
  {
    errorsAndWarnings++;
  }
  Serial.print(" Acceleration X RAW = ");
  Serial.println(dataHighres);

  if( myIMU.readRegisterInt16( &dataHighres, LIS3DH_OUT_Z_L ) != 0 )
  {
    errorsAndWarnings++;
  }
  Serial.print(" Acceleration Z RAW = ");
  Serial.println(dataHighres);

  // Read accelerometer data in mg as Float
  Serial.print(" Acceleration Z float = ");
  Serial.println( myIMU.axisAccel( Z ), 4);

  delay(1000);

}