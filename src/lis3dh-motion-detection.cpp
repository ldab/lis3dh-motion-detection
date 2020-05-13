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

	_DEBBUG("Configuring IMU");

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

	_DEBBUG("Apply settings");
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
	uint8_t result = 0;
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

		_DEBBUG("Read register 0x", offset, " = ", result);

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

		_DEBBUG("12 bit from 0x", offset, " = ", output);
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

	_DEBBUG("Write register 0x", offset, " = ", dataToWrite);
	return returnError;

	delay(100);             // some users reported hangs without delay.
}

// Read tap (aka: 'click')
uint8_t LIS3DH::readTap(){
	uint8_t t;
	
	readRegister(&t, LIS3DH_CLICK_SRC); // read
	return t;
}

uint8_t LIS3DH::readEvent(){
	uint8_t t;
	
	readRegister(&t, LIS3DH_INT1_SRC);
	return t;
}

void LIS3DH::configTap(bool Zdouble, bool Zsingle, bool Ydouble, bool Ysingle, bool Xdouble, bool Xsingle, uint8_t threshold){
	uint8_t r = 0;

	writeRegister(LIS3DH_CTRL_REG3, 0x80); 				// Tap on INT1
	readRegister(&r, LIS3DH_CTRL_REG5);
	writeRegister(LIS3DH_CTRL_REG5, r | 0b1000); 			// latch INT1

	readRegister(&r, LIS3DH_CTRL_REG2);
	writeRegister(LIS3DH_CTRL_REG2, r | 0x04); 			// Hi pass enable

	writeRegister(LIS3DH_CLICK_CFG, Zdouble << 5 | Zsingle << 4 | Ydouble << 3 | Ysingle << 2 | Xdouble << 1 | Xsingle); //
	// writeRegister(LIS3DH_CLICK_SRC, 0b00110000); 		// single and double tap enable READ ONLY!
	writeRegister(LIS3DH_CLICK_THS, 0b01111111 & threshold); 	// pro-latch off
	writeRegister(LIS3DH_TIME_LIMIT, 0x40); 			// time limit
	writeRegister(LIS3DH_TIME_WINDOW, 0x40);			// arbitrary
}

void LIS3DH::autoSleep(uint8_t threshold, uint8_t time){
	writeRegister(LIS3DH_ACT_THS, threshold);
	writeRegister(LIS3DH_ACT_DUR, time);
}

// p. 23 of DOCid1898
void LIS3DH::wakeUpInertialAN(){
	status_t returnError = IMU_SUCCESS;

	// basic setup
	writeRegister(LIS3DH_CTRL_REG1, 0xA7); // Enable, ODR100, XYZ
	writeRegister(LIS3DH_CTRL_REG2, 0x00); // Hi pass disable
	writeRegister(LIS3DH_CTRL_REG3, 0x40); // Int to INT1 pad
	writeRegister(LIS3DH_CTRL_REG4, 0x00); // 2G
	writeRegister(LIS3DH_CTRL_REG5, 0x08); // latched

	// config INT
	writeRegister(LIS3DH_INT1_THS, 0x10);		// Threshold 250mg
	writeRegister(LIS3DH_INT1_DURATION, 0x40);	// Duration [default: 0] MAX 0x40
	writeRegister(LIS3DH_INT1_CFG, 0b1010);		// XH + YH interrupt
}

// Read axis acceleration as Float
float LIS3DH::axisAccel( axis_t _axis)
{
	int16_t outRAW;
	uint8_t regToRead = 0;
	switch (_axis)
	{
		case 0:
			// X axis
			regToRead = LIS3DH_OUT_X_L;
			break;
		case 1:
			// Y axis
			regToRead = LIS3DH_OUT_Y_L;
			break;
		case 2:
			// Z axis
			regToRead = LIS3DH_OUT_Z_L;
			break;
	
		default:
			// Not valid axis return NAN
			return NAN;
			break;
	}

	readRegisterInt16( &outRAW, regToRead );

	float outFloat;

	switch( accelRange )
	{
		case 2:
		outFloat = (float)outRAW / 15987;
		break;
		case 4:
		outFloat = (float)outRAW / 7840;
		break;
		case 8:
		outFloat = (float)outRAW / 3883;
		break;
		case 16:
		outFloat = (float)outRAW / 1280;
		break;
		default:
		outFloat = 0;
		break;
	}

	return outFloat;

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

	// page 16 set CTRL_REG1[3](LPen bit)
	#ifdef LOW_POWER
		dataToWrite |= 0x08;
	#endif

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

	dataToWrite |= (zAccelEnabled & 0x01) << 2;
	dataToWrite |= (yAccelEnabled & 0x01) << 1;
	dataToWrite |= (xAccelEnabled & 0x01);
	//Now, write the patched together data
	_DEBBUG ("LIS3DH_CTRL_REG1: 0x", dataToWrite);

	writeRegister(LIS3DH_CTRL_REG1, dataToWrite);

	//Build CTRL_REG4
	dataToWrite = 0; //Start Fresh!
	
	// page 16 set CTRL_REG4[3](HR bit)
#ifdef HIGH_RESOLUTION
	dataToWrite |= 0x08; //set high resolution
#endif

	//  Convert scaling
	switch(accelRange)
	{	
		default:
		case 2:
		dataToWrite |= (0x00 << 4);
		break;
		case 4:
		dataToWrite |= (0x01 << 4);
		break;
		case 8:
		dataToWrite |= (0x02 << 4);
		break;
		case 16:
		dataToWrite |= (0x03 << 4);
		break;
	}

	//Now, write the patched together data
	_DEBBUG ("LIS3DH_CTRL_REG4: 0x", dataToWrite);
	writeRegister(LIS3DH_CTRL_REG4, dataToWrite);

	delay(100);
}

//****************************************************************************//
//
//  Configure interrupts 1 or 2, stop or move, threshold and duration
//	Durationsteps and maximum values depend on the ODR chosen.
//
//****************************************************************************//
status_t LIS3DH::intConf(interrupt_t interrupt,
						event_t moveType, 
						uint8_t threshold,
						uint8_t timeDur,
						bool polarity)
{
	uint8_t dataToWrite = 0;  //Temporary variable
	status_t returnError = IMU_SUCCESS;

	if ( interrupt == INT_0 ) { // disable INTerrupts and exit
		writeRegister(LIS3DH_CTRL_REG3, 0x00);
		returnError = writeRegister(LIS3DH_CTRL_REG6, 0x00);
		return returnError;
	}

	uint8_t regToWrite = 0;
	regToWrite = (interrupt == INT_1) ? LIS3DH_INT1_CFG : LIS3DH_INT2_CFG; // 0x30 or 0x34

	//Build INT_CFG 0x30 or 0x34
	//Detect movement or stop
	//bike straight  : ZL YH XL
	//bike fall right: ZL YH XH
	//bile fall left : ZL YL XH
	if(moveType == 0)	dataToWrite |= 0x0A; // b1010 phone on table
	if(moveType == 1)	dataToWrite |= 0x05; // b0101 phone upside down on table
	if(moveType == 2)	dataToWrite = 0b00000011; // bike fall
	

	_DEBBUG ("LIS3DH_INT_CFG: 0x", dataToWrite);
	returnError = writeRegister(regToWrite, dataToWrite);

	//Build INT_THS 0x32 or 0x36
	regToWrite += 2;
	returnError = writeRegister(regToWrite, threshold);

	//Build INT_DURATION 0x33 or 0x37
	regToWrite++;

	//float _seconds = float(timeDur/1.0f/accelSampleRate);
	//_DEBBUG ("Event Duration is: ", _seconds, "sec");

	returnError = writeRegister(regToWrite, timeDur);
	
	readRegister(&dataToWrite, LIS3DH_CTRL_REG5);
	returnError = writeRegister(LIS3DH_CTRL_REG5, dataToWrite | 0b1000); // latch INT1

	//Attach configuration to Interrupt X
	if(interrupt == 1)
	{
		returnError = writeRegister(LIS3DH_CTRL_REG3, 0x40);
	}
	else
	{
		returnError = writeRegister(LIS3DH_CTRL_REG6, 0x20);
	}

	// Polarity
	readRegister(&dataToWrite, LIS3DH_CTRL_REG6);
	returnError = writeRegister(LIS3DH_CTRL_REG6, dataToWrite | polarity ); //
	
	return returnError;
}

void LIS3DH::setClickAda(uint8_t c, uint8_t clickthresh, uint8_t timelimit, uint8_t timelatency, uint8_t timewindow) {
  if (!c) {    //disable int    
	uint8_t r;
       	readRegister(&r, LIS3DH_CTRL_REG3);
	r &= ~(0x80); // turn off I1_CLICK    
	writeRegister(LIS3DH_CTRL_REG3, r);
	writeRegister(LIS3DH_CLICK_CFG, 0);
	return;
}  // else...  
	writeRegister(LIS3DH_CTRL_REG3, 0x80);			// turn on int1 click  
	writeRegister(LIS3DH_CTRL_REG5, 0x08);			// latch interrupt on int1  if (c == 1)  
	writeRegister(LIS3DH_CLICK_CFG, 0x15);			// turn on all axes & singletap  if (c == 2)  
	writeRegister(LIS3DH_CLICK_CFG, 0x2A);			// turn on all axes & doubletap
	writeRegister(LIS3DH_CLICK_THS, clickthresh);		// arbitrary
	writeRegister(LIS3DH_TIME_LIMIT, timelimit);		// arbitrary
	writeRegister(LIS3DH_TIME_LATENCY, timelatency);	// arbitrary
	writeRegister(LIS3DH_TIME_WINDOW, timewindow);		// arbitrary
}

// he needs delays between i2c
void LIS3DH::motionSTforum(){
	writeRegister(LIS3DH_CTRL_REG1, 0x27);
	writeRegister(LIS3DH_CTRL_REG2, 0x01); // Hi-Filter for AOI on INT1
	writeRegister(LIS3DH_CTRL_REG3, 0x40); // Interrupt driven to INT1 pad,  AOI1 interrupt on INT1.  ---0100
	writeRegister(LIS3DH_CTRL_REG4, 0x00); // FS = 2g  Full scale selection 
	
	writeRegister(LIS3DH_CTRL_REG5, 0x08); //latched  0000 1000 ---0x08 + Disable FIFO temporarily;  0: interrupt request not latched;---0x00

	writeRegister(LIS3DH_INT1_THS, 0x08); // 0010 1010 beyond thresh hold  ; 0001 0101  below thresh hold. event
	writeRegister(LIS3DH_INT1_CFG, 0x2a); // 0010 1010 enable HIGH events?
}
