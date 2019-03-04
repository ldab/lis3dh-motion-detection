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

#include "lis3dh-motion-detection.h"
#include "stdint.h"

#include "Wire.h"

//****************************************************************************//
//
//  LIS3DHCore functions.
//
//  Construction arguments:
//  ( uint8_t inputArg ),
//
//  Default construction is I2C mode, address 0x19.
//
//****************************************************************************//
LIS3DH::LIS3DH( uint8_t inputArg )
{
  I2CAddress = inputArg;
}

status_t LIS3DH::begin( uint16_t accSample,
						uint8_t xAcc,
						uint8_t yAcc,
						uint8_t zAcc,
						uint8_t accSens )
{
	status_t returnError = IMU_SUCCESS;

  	Wire.begin();
  
	//Spin for a few ms
	volatile uint8_t temp = 0;
	for( uint16_t i = 0; i < 10000; i++ )
	{
		temp++;
	}

	//Check the ID register to determine if the operation was a success.
	uint8_t readCheck;
	readRegister(&readCheck, LIS3DH_WHO_AM_I);
	if( readCheck != 0x33 )
	{
		returnError = IMU_HW_ERROR;
	}

	accelSampleRate = accSample;
	xAccelEnabled = xAcc;
	yAccelEnabled = yAcc;
	zAccelEnabled = zAcc;
	accelRange = accSens;

	applySettings();

	return returnError;
}

//****************************************************************************//
//
//  ReadRegisterRegion
//
//  Parameters:
//    *outputPointer -- Pass &variable (base address of) to save read data to
//    offset -- register to read
//    length -- number of bytes to read
//
//  Note:  Does not know if the target memory space is an array or not, or
//    if there is the array is big enough.  if the variable passed is only
//    two bytes long and 3 bytes are requested, this will over-write some
//    other memory!
//
//****************************************************************************//
status_t LIS3DH::readRegisterRegion(uint8_t *outputPointer , uint8_t offset, uint8_t length)
{
	status_t returnError = IMU_SUCCESS;

	//define pointer that will point to the external space
	uint8_t i = 0;
	uint8_t c = 0;

  Wire.beginTransmission(I2CAddress);
  offset |= 0x80; //turn auto-increment bit on, bit 7 for I2C
  Wire.write(offset);
  if( Wire.endTransmission() != 0 )
  {
    returnError = IMU_HW_ERROR;
  }
  else  //OK, all worked, keep going
  {
    // request 6 bytes from slave device
    Wire.requestFrom(I2CAddress, length);
    while ( (Wire.available()) && (i < length))  // slave may send less than requested
    {
      c = Wire.read(); // receive a byte as character
      *outputPointer = c;
      outputPointer++;
      i++;
    }
  }

	return returnError;
}

//****************************************************************************//
//
//  ReadRegister
//
//  Parameters:
//    *outputPointer -- Pass &variable (address of) to save read data to
//    offset -- register to read
//
//****************************************************************************//
status_t LIS3DH::readRegister(uint8_t* outputPointer, uint8_t offset) {
	//Return value
	uint8_t result;
	uint8_t numBytes = 1;
	status_t returnError = IMU_SUCCESS;

  Wire.beginTransmission(I2CAddress);
  Wire.write(offset);
  if( Wire.endTransmission() != 0 )
  {
    returnError = IMU_HW_ERROR;
  }
  Wire.requestFrom(I2CAddress, numBytes);
  while ( Wire.available() ) // slave may send less than requested
  {
    result = Wire.read(); // receive a byte as a proper uint8_t
  }

	*outputPointer = result;
	return returnError;
}

//****************************************************************************//
//
//  readRegisterInt16
//
//  Parameters:
//    *outputPointer -- Pass &variable (base address of) to save read data to
//    offset -- register to read
//
//****************************************************************************//
status_t LIS3DH::readRegisterInt16( int16_t* outputPointer, uint8_t offset )
{
	{
		//offset |= 0x80; //turn auto-increment bit on
		uint8_t myBuffer[2];
		status_t returnError = readRegisterRegion(myBuffer, offset, 2);  //Does memory transfer
		int16_t output = (int16_t)myBuffer[0] | int16_t(myBuffer[1] << 8);
		*outputPointer = output;
		return returnError;
	}

}

//****************************************************************************//
//
//  writeRegister
//
//  Parameters:
//    offset -- register to write
//    dataToWrite -- 8 bit data to write to register
//
//****************************************************************************//
status_t LIS3DH::writeRegister(uint8_t offset, uint8_t dataToWrite) {
	status_t returnError = IMU_SUCCESS;

  //Write the byte
  Wire.beginTransmission(I2CAddress);
  Wire.write(offset);
  Wire.write(dataToWrite);
  if( Wire.endTransmission() != 0 )
  {
    returnError = IMU_HW_ERROR;
  }

	return returnError;
}

//****************************************************************************//
//
//  Apply settings passed to .begin();
//
//****************************************************************************//
void LIS3DH::applySettings( void )
{
	uint8_t dataToWrite = 0;  //Temporary variable
	
	//Build CTRL_REG1
	//  Convert ODR
	switch(accelSampleRate)
	{
		case 1:
		dataToWrite |= (0x01 << 4);
		break;
		case 10:
		dataToWrite |= (0x02 << 4);
		break;
		case 25:
		dataToWrite |= (0x03 << 4);
		break;
		case 50:
		dataToWrite |= (0x04 << 4);
		break;
		case 100:
		dataToWrite |= (0x05 << 4);
		break;
		case 200:
		dataToWrite |= (0x06 << 4);
		break;
		default:
		case 400:
		dataToWrite |= (0x07 << 4);
		break;
		case 1600:
		dataToWrite |= (0x08 << 4);
		break;
		case 5000:
		dataToWrite |= (0x09 << 4);
		break;
	}
	
	// page 16 set CTRL_REG1[3](LPen bit)
	#ifdef LOW_POWER
		dataToWrite |= 0x08;
	#else
		dataToWrite |= 0xF7;
	#endif

	dataToWrite |= (zAccelEnabled & 0x01) << 2;
	dataToWrite |= (yAccelEnabled & 0x01) << 1;
	dataToWrite |= (xAccelEnabled & 0x01);
	//Now, write the patched together data
#ifdef VERBOSE_SERIAL
	Serial.print("LIS3DH_CTRL_REG1: 0x");
	Serial.println(dataToWrite, HEX);
#endif
	writeRegister(LIS3DH_CTRL_REG1, dataToWrite);

	//Build CTRL_REG4
	dataToWrite = 0; //Start Fresh!
	//  Convert scaling
	switch(accelRange)
	{
		case 2:
		dataToWrite |= (0x00 << 4);
		break;
		case 4:
		dataToWrite |= (0x01 << 4);
		break;
		case 8:
		dataToWrite |= (0x02 << 4);
		break;
		default:
		case 16:
		dataToWrite |= (0x03 << 4);
		break;
	}

// page 16 set CTRL_REG4[3](HR bit)
#ifdef HIGH_RESOLUTION
	dataToWrite |= 0x08; //set high resolution
#else
	dataToWrite |= 0xF7; //CTRL_REG4[3](HR bit) to 0
#endif

#ifdef VERBOSE_SERIAL
	Serial.print("LIS3DH_CTRL_REG4: 0x");
	Serial.println(dataToWrite, HEX);
#endif
	//Now, write the patched together data
	writeRegister(LIS3DH_CTRL_REG4, dataToWrite);

}

//****************************************************************************//
//
//  Configure interrupts 1 or 2, stop or move, threshold and duration
//	Durationsteps and maximum values depend on the ODR chosen.
//
//****************************************************************************//
status_t LIS3DH::intConf(uint8_t interrupt,
						event_t moveType, 
						uint8_t threshold,
						uint8_t timeDur)
{
	uint8_t dataToWrite = 0;  //Temporary variable
	status_t returnError = IMU_SUCCESS;

	uint8_t regToWrite;
	regToWrite = (interrupt==1) ? LIS3DH_INT1_CFG : LIS3DH_INT2_CFG;

	//Build INT_CFG 0x30 or 0x34
	//Detect movement or stop
	if(moveType)	dataToWrite |= 0x2A;
	else 			dataToWrite |= 0x15;

	#ifdef VERBOSE_SERIAL
		Serial.print("LIS3DH_INT_CFG: 0x");
		Serial.println(dataToWrite, HEX);
	#endif

	returnError = writeRegister(regToWrite, dataToWrite);
	
	//Build INT_THS 0x32 or 0x36
	regToWrite += 2;
	returnError = writeRegister(regToWrite, threshold);

	//Build INT_DURATION 0x33 or 0x37
	regToWrite++;
	returnError = writeRegister(regToWrite, timeDur);

	//Attach configuration to Interrupt X
	if(interrupt==1)
	{
		returnError = writeRegister(LIS3DH_CTRL_REG3, 0x40);
	}
	else
	{
		returnError = writeRegister(LIS3DH_CTRL_REG6, 0x20);
	}
	
	return returnError;
}
