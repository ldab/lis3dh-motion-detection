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
#define LOW_POWER
//#define NORMAL_MODE
//#define HIGH_RESOLUTION

#define VERBOSE_SERIAL

#include "lis3dh-motion-detection.h"
#include "Wire.h"

uint16_t sampleRate = 10; //HZ - Samples per second

uint16_t errorsAndWarnings = 0;

LIS3DH myIMU(0x19); //Default address is 0x19.

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(1000); //wait until serial is open...
  
  if( myIMU.begin(sampleRate, 1, 1, 1, 2) != 0 )
  {
    Serial.print("Error at begin().\n");
  }
  else
  {
    Serial.print("\nbegin() passed.\n");
  }
  
  //Detection threshold can be from 1 to 127 and depends on the Range
  //chosen above, change it and test accordingly to your application
  //Duration = timeDur x Seconds / sampleRate
  myIMU.intConf(1, DET_MOVE, 50, 4);
  myIMU.intConf(2, DET_STOP, 50, 3);


//Setup the accelerometer******************************
  uint8_t dataToWrite = 0; //Start Fresh!
  dataToWrite |= 0x4 << 4; //ODR of 50Hz
  dataToWrite |= 0x7; //Enable all axes

  //Now, write the patched together data
  errorsAndWarnings += myIMU.writeRegister(LIS3DH_CTRL_REG1, dataToWrite);

  dataToWrite = 0x80;
  errorsAndWarnings += myIMU.writeRegister(LIS3DH_CTRL_REG4, dataToWrite);  

//  //Test interrupt configuration profile on int1
//  {
//	dataToWrite = 0x40; //INT1 src
//	errorsAndWarnings += myIMU.writeRegister(LIS3DH_CTRL_REG3, dataToWrite);
//	dataToWrite = 0x08; //latch output int
//	errorsAndWarnings += myIMU.writeRegister(LIS3DH_CTRL_REG5, dataToWrite);
//	dataToWrite = 0x40; //
//	//errorsAndWarnings += myIMU.writeRegister(LIS3DH_REFERENCE, dataToWrite);
//	dataToWrite = 0x0A; //High X and high Y only
//	errorsAndWarnings += myIMU.writeRegister(LIS3DH_INT1_CFG, dataToWrite);
//	dataToWrite = 0x3F; // half amplitude?
//	errorsAndWarnings += myIMU.writeRegister(LIS3DH_INT1_THS, dataToWrite);
//	dataToWrite = 0x01; //duration?
//	errorsAndWarnings += myIMU.writeRegister(LIS3DH_INT1_DURATION, dataToWrite);
//  }

  
  //Test interrupt configuration profile on int2
  {
	dataToWrite = 0x40; //INT2 src
	errorsAndWarnings += myIMU.writeRegister(LIS3DH_CTRL_REG6, dataToWrite);
	dataToWrite = 0x08; //latch output int
	errorsAndWarnings += myIMU.writeRegister(LIS3DH_CTRL_REG5, dataToWrite);
	dataToWrite = 0x40; //
	//errorsAndWarnings += myIMU.writeRegister(LIS3DH_REFERENCE, dataToWrite);
	dataToWrite = 0x0A; //High X and high Y only
	errorsAndWarnings += myIMU.writeRegister(LIS3DH_INT1_CFG, dataToWrite);
	dataToWrite = 0x3F; // half amplitude?
	errorsAndWarnings += myIMU.writeRegister(LIS3DH_INT1_THS, dataToWrite);
	dataToWrite = 0x01; //duration?
	errorsAndWarnings += myIMU.writeRegister(LIS3DH_INT1_DURATION, dataToWrite);
  }

  //Get the ID:
  uint8_t readData = 0;
  Serial.print("\nReading LIS3DH_WHO_AM_I: 0x");
  myIMU.readRegister(&readData, LIS3DH_WHO_AM_I);
  Serial.println(readData, HEX);

}


void loop()
{
  float accel;
  uint8_t readData = 0;

  Serial.print(" Acceleration = ");
  //Read a register into the Acceleration variable.
  if( myIMU.readRegister( &readData, LIS3DH_OUT_X_L ) != 0 )
  {
    errorsAndWarnings++;
  }
  
  Serial.println();
  Serial.print("Total reported Errors and Warnings: ");
  Serial.println(errorsAndWarnings);

  delay(1000);
}