/******************************************************************************
lis3dh-motion-detection.h
LIS3DH Arduino
Leonardo Bispo
Mar 03, 2019
https://github.com/ldab/lis3dh-motion-detection
Resources:
Uses Wire.h for i2c operation

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
LIS3DHCore::LIS3DHCore( uint8_t inputArg )
{
  I2CAddress = inputArg;
}

status_t LIS3DHCore::beginCore(void)
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
status_t LIS3DHCore::readRegisterRegion(uint8_t *outputPointer , uint8_t offset, uint8_t length)
{
	status_t returnError = IMU_SUCCESS;

	//define pointer that will point to the external space
	uint8_t i = 0;
	uint8_t c = 0;
	uint8_t tempFFCounter = 0;

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
status_t LIS3DHCore::readRegister(uint8_t* outputPointer, uint8_t offset) {
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
status_t LIS3DHCore::readRegisterInt16( int16_t* outputPointer, uint8_t offset )
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
status_t LIS3DHCore::writeRegister(uint8_t offset, uint8_t dataToWrite) {
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
//  Main user class -- wrapper for the core class + maths
//
//  Construct with same rules as the core ( uint8_t busType, uint8_t inputArg )
//
//****************************************************************************//
LIS3DH::LIS3DH( uint8_t inputArg ) : LIS3DHCore( inputArg )
{
	//Construct with these default settings
	//ADC stuff
	settings.adcEnabled = 1;
	
	//Temperature settings
	settings.tempEnabled = 1;

	//Accelerometer settings
	settings.accelSampleRate = 50;  //Hz.  Can be: 0,1,10,25,50,100,200,400,1600,5000 Hz
	settings.accelRange = 2;      //Max G force readable.  Can be: 2, 4, 8, 16

	settings.xAccelEnabled = 1;
	settings.yAccelEnabled = 1;
	settings.zAccelEnabled = 1;

	//FIFO control settings
	settings.fifoEnabled = 0;
	settings.fifoThreshold = 20;  //Can be 0 to 32
	settings.fifoMode = 0;  //FIFO mode.
  
	allOnesCounter = 0;
	nonSuccessCounter = 0;

}

//****************************************************************************//
//
//  Begin
//
//  This starts the lower level begin, then applies settings
//
//****************************************************************************//
status_t LIS3DH::begin( void )
{
	//Begin the inherited core.  This gets the physical wires connected
	status_t returnError = beginCore();

	applySettings();
	
	return returnError;
}

//****************************************************************************//
//
//  Configuration section
//
//  This uses the stored SensorSettings to start the IMU
//  Use statements such as "myIMU.settings.commInterface = SPI_MODE;" or
//  "myIMU.settings.accelEnabled = 1;" to configure before calling .begin();
//
//****************************************************************************//
void LIS3DH::applySettings( void )
{
	uint8_t dataToWrite = 0;  //Temporary variable

	//Build TEMP_CFG_REG
	dataToWrite = 0; //Start Fresh!
	dataToWrite = ((settings.tempEnabled & 0x01) << 6) | ((settings.adcEnabled & 0x01) << 7);
	//Now, write the patched together data
#ifdef VERBOSE_SERIAL
	Serial.print("LIS3DH_TEMP_CFG_REG: 0x");
	Serial.println(dataToWrite, HEX);
#endif
	writeRegister(LIS3DH_TEMP_CFG_REG, dataToWrite);
	
	//Build CTRL_REG1
	dataToWrite = 0; //Start Fresh!
	//  Convert ODR
	switch(settings.accelSampleRate)
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
	
	dataToWrite |= (settings.zAccelEnabled & 0x01) << 2;
	dataToWrite |= (settings.yAccelEnabled & 0x01) << 1;
	dataToWrite |= (settings.xAccelEnabled & 0x01);
	//Now, write the patched together data
#ifdef VERBOSE_SERIAL
	Serial.print("LIS3DH_CTRL_REG1: 0x");
	Serial.println(dataToWrite, HEX);
#endif
	writeRegister(LIS3DH_CTRL_REG1, dataToWrite);

	//Build CTRL_REG4
	dataToWrite = 0; //Start Fresh!
	//  Convert scaling
	switch(settings.accelRange)
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
	dataToWrite |= 0x80; //set block update
	dataToWrite |= 0x08; //set high resolution
#ifdef VERBOSE_SERIAL
	Serial.print("LIS3DH_CTRL_REG4: 0x");
	Serial.println(dataToWrite, HEX);
#endif
	//Now, write the patched together data
	writeRegister(LIS3DH_CTRL_REG4, dataToWrite);

}
//****************************************************************************//
//
//  Accelerometer section
//
//****************************************************************************//
int16_t LIS3DH::readRawAccelX( void )
{
	int16_t output;
	status_t errorLevel = readRegisterInt16( &output, LIS3DH_OUT_X_L );
	if( errorLevel != IMU_SUCCESS )
	{
		if( errorLevel == IMU_ALL_ONES_WARNING )
		{
			allOnesCounter++;
		}
		else
		{
			nonSuccessCounter++;
		}
	}
	return output;
}
float LIS3DH::readFloatAccelX( void )
{
	float output = calcAccel(readRawAccelX());
	return output;
}

int16_t LIS3DH::readRawAccelY( void )
{
	int16_t output;
	status_t errorLevel = readRegisterInt16( &output, LIS3DH_OUT_Y_L );
	if( errorLevel != IMU_SUCCESS )
	{
		if( errorLevel == IMU_ALL_ONES_WARNING )
		{
			allOnesCounter++;
		}
		else
		{
			nonSuccessCounter++;
		}
	}
	return output;
}

float LIS3DH::readFloatAccelY( void )
{
	float output = calcAccel(readRawAccelY());
	return output;
}

int16_t LIS3DH::readRawAccelZ( void )
{
	int16_t output;
	status_t errorLevel = readRegisterInt16( &output, LIS3DH_OUT_Z_L );
	if( errorLevel != IMU_SUCCESS )
	{
		if( errorLevel == IMU_ALL_ONES_WARNING )
		{
			allOnesCounter++;
		}
		else
		{
			nonSuccessCounter++;
		}
	}
	return output;

}

float LIS3DH::readFloatAccelZ( void )
{
	float output = calcAccel(readRawAccelZ());
	return output;
}

float LIS3DH::calcAccel( int16_t input )
{
	float output;
	switch(settings.accelRange)
	{
		case 2:
		output = (float)input / 15987;
		break;
		case 4:
		output = (float)input / 7840;
		break;
		case 8:
		output = (float)input / 3883;
		break;
		case 16:
		output = (float)input / 1280;
		break;
		default:
		output = 0;
		break;
	}
	return output;
}

//****************************************************************************//
//
//  Accelerometer section
//
//****************************************************************************//
uint16_t LIS3DH::read10bitADC1( void )
{
	int16_t intTemp;
	uint16_t uintTemp;
	readRegisterInt16( &intTemp, LIS3DH_OUT_ADC1_L );
	intTemp = 0 - intTemp;
	uintTemp = intTemp + 32768;
	return uintTemp >> 6;
}

uint16_t LIS3DH::read10bitADC2( void )
{
	int16_t intTemp;
	uint16_t uintTemp;
	readRegisterInt16( &intTemp, LIS3DH_OUT_ADC2_L );
	intTemp = 0 - intTemp;
	uintTemp = intTemp + 32768;
	return uintTemp >> 6;
}

uint16_t LIS3DH::read10bitADC3( void )
{
	int16_t intTemp;
	uint16_t uintTemp;
	readRegisterInt16( &intTemp, LIS3DH_OUT_ADC3_L );
	intTemp = 0 - intTemp;
	uintTemp = intTemp + 32768;
	return uintTemp >> 6;
}

//****************************************************************************//
//
//  FIFO section
//
//****************************************************************************//
void LIS3DH::fifoBegin( void )
{
	uint8_t dataToWrite = 0;  //Temporary variable

	//Build LIS3DH_FIFO_CTRL_REG
	readRegister( &dataToWrite, LIS3DH_FIFO_CTRL_REG ); //Start with existing data
	dataToWrite &= 0x20;//clear all but bit 5
	dataToWrite |= (settings.fifoMode & 0x03) << 6; //apply mode
	dataToWrite |= (settings.fifoThreshold & 0x1F); //apply threshold
	//Now, write the patched together data
#ifdef VERBOSE_SERIAL
	Serial.print("LIS3DH_FIFO_CTRL_REG: 0x");
	Serial.println(dataToWrite, HEX);
#endif
	writeRegister(LIS3DH_FIFO_CTRL_REG, dataToWrite);

	//Build CTRL_REG5
	readRegister( &dataToWrite, LIS3DH_CTRL_REG5 ); //Start with existing data
	dataToWrite &= 0xBF;//clear bit 6
	dataToWrite |= (settings.fifoEnabled & 0x01) << 6;
	//Now, write the patched together data
#ifdef VERBOSE_SERIAL
	Serial.print("LIS3DH_CTRL_REG5: 0x");
	Serial.println(dataToWrite, HEX);
#endif
	writeRegister(LIS3DH_CTRL_REG5, dataToWrite);
}

void LIS3DH::fifoClear( void ) {
	//Drain the fifo data and dump it
	while( (fifoGetStatus() & 0x20 ) == 0 ) {
		readRawAccelX();
		readRawAccelY();
		readRawAccelZ();
	}
}

void LIS3DH::fifoStartRec( void )
{
	uint8_t dataToWrite = 0;  //Temporary variable
	
	//Turn off...
	readRegister( &dataToWrite, LIS3DH_FIFO_CTRL_REG ); //Start with existing data
	dataToWrite &= 0x3F;//clear mode
#ifdef VERBOSE_SERIAL
	Serial.print("LIS3DH_FIFO_CTRL_REG: 0x");
	Serial.println(dataToWrite, HEX);
#endif
	writeRegister(LIS3DH_FIFO_CTRL_REG, dataToWrite);	
	//  ... then back on again
	readRegister( &dataToWrite, LIS3DH_FIFO_CTRL_REG ); //Start with existing data
	dataToWrite &= 0x3F;//clear mode
	dataToWrite |= (settings.fifoMode & 0x03) << 6; //apply mode
	//Now, write the patched together data
#ifdef VERBOSE_SERIAL
	Serial.print("LIS3DH_FIFO_CTRL_REG: 0x");
	Serial.println(dataToWrite, HEX);
#endif
	writeRegister(LIS3DH_FIFO_CTRL_REG, dataToWrite);
}

uint8_t LIS3DH::fifoGetStatus( void )
{
	//Return some data on the state of the fifo
	uint8_t tempReadByte = 0;
	readRegister(&tempReadByte, LIS3DH_FIFO_SRC_REG);
#ifdef VERBOSE_SERIAL
	Serial.print("LIS3DH_FIFO_SRC_REG: 0x");
	Serial.println(tempReadByte, HEX);
#endif
	return tempReadByte;  
}

void LIS3DH::fifoEnd( void )
{
	uint8_t dataToWrite = 0;  //Temporary variable

	//Turn off...
	readRegister( &dataToWrite, LIS3DH_FIFO_CTRL_REG ); //Start with existing data
	dataToWrite &= 0x3F;//clear mode
#ifdef VERBOSE_SERIAL
	Serial.print("LIS3DH_FIFO_CTRL_REG: 0x");
	Serial.println(dataToWrite, HEX);
#endif
	writeRegister(LIS3DH_FIFO_CTRL_REG, dataToWrite);	
}