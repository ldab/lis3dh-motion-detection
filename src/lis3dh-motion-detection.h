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

#ifndef __LIS3DH_IMU_H__
#define __LIS3DH_IMU_H__

#include "stdint.h"

#if defined(ARDUINO) && ARDUINO >= 100
    #include "Arduino.h"
#else
    #include "WProgram.h"
#endif

#if defined LOW_POWER && defined NORMAL_MODE
	#error Please choose between the 3 resolution types
#elif defined NORMAL_MODE && defined HIGH_RESOLUTION
	#error Please choose between the 3 resolution types
#elif defined LOW_POWER && defined HIGH_RESOLUTION
	#error Please choose between the 3 resolution types
#endif

//#define LIS3DH_DEBUG

#ifdef LIS3DH_DEBUG
	namespace {
  		template<typename T>
  		static void _DEBBUG(T last) {
    	Serial.println(last);
		}
		
		template<typename T, typename... Args>
		static void _DEBBUG(T head, Args... tail) {
			Serial.print(head);
			Serial.print(' ');
			_DEBBUG(tail...);
		}
	}
#else
  	#define _DEBBUG(...)
#endif

//Print variable name
#define getName(var)  #var

// Return values 
typedef enum
{
	IMU_SUCCESS,
	IMU_HW_ERROR,
	IMU_NOT_SUPPORTED,
	IMU_GENERIC_ERROR,
	IMU_OUT_OF_BOUNDS,
	IMU_ALL_ONES_WARNING,
	//...
} status_t;

typedef enum
{
	DET_STOP,
	DET_MOVE,
	BIKE_FALL,
} event_t;

typedef enum
{
	INT_0,
	INT_1,
	INT_2,
} interrupt_t;

typedef enum
{
	X = 0,
	Y,
	Z,
} axis_t;

class LIS3DH
{
public:
	LIS3DH( uint8_t );
	
	status_t begin( uint16_t accSample,
					uint8_t xAcc,
					uint8_t yAcc,
					uint8_t zAcc,
					uint8_t accSens );
	
	// The following utilities read and write to the IMU

	// readRegister reads one 8-bit register
	status_t readRegister(uint8_t* outputPointer, uint8_t offset);
	
	// Reads two 8-bit regs, LSByte then MSByte order, and concatenates them.
	// Acts as a 16-bit read operation
	status_t readRegisterInt16(int16_t*, uint8_t offset );
	
	// Writes an 8-bit byte;
	status_t writeRegister(uint8_t, uint8_t);

	// Configure Interrupts
	// INT1 or 2, Move or Stop, Detection Sensivity and duration cycles = from 1 to 127
	status_t intConf(interrupt_t interrupt,
					event_t moveType = DET_MOVE,
					uint8_t threshold = 13,
					uint8_t timeDur = 2,
					bool polarity = 0);
	
	// Read axis acceleration as Float
	float axisAccel( axis_t _axis);
	uint8_t readTap();
	uint8_t readEvent();
	void wakeUpInertialAN();

	void configTap(bool Zdouble = 1, bool Zsingle = 1, bool Ydouble = 1, bool Ysingle = 1, bool Xdouble = 1, bool Xsingle = 1, uint8_t threshold = 10);
	void autoSleep(uint8_t threshold = 40, uint8_t time = 60);

	void motionSTforum();
	void setClickAda(uint8_t c = 1, uint8_t clickthresh = 40, uint8_t timelimit = 10, uint8_t timelatency = 20, uint8_t timewindow = 255);
	
private:
	uint8_t I2CAddress;
	uint16_t accelSampleRate; //Can be 1, 10, 25, 50, 100, 200, 400, 1600, 5000
	uint8_t xAccelEnabled;
	uint8_t yAccelEnabled;
	uint8_t zAccelEnabled;
	uint8_t accelRange; //Accelerometer range = 2, 4, 8, 16g

	//Apply settings at .begin()
	void applySettings( void );

	//ReadRegisterRegion takes a uint8 array address as input and reads
	//  a chunk of memory into that array.
	status_t readRegisterRegion(uint8_t*, uint8_t, uint8_t );
};

//Device Registers
#define LIS3DH_STATUS_REG_AUX         0x07
#define LIS3DH_WHO_AM_I               0x0F

#define LIS3DH_CTRL_REG1              0x20
#define LIS3DH_CTRL_REG2              0x21
#define LIS3DH_CTRL_REG3              0x22
#define LIS3DH_CTRL_REG4              0x23
#define LIS3DH_CTRL_REG5              0x24
#define LIS3DH_CTRL_REG6              0x25
#define LIS3DH_REFERENCE              0x26
#define LIS3DH_STATUS_REG2            0x27
#define LIS3DH_OUT_X_L                0x28
#define LIS3DH_OUT_X_H                0x29
#define LIS3DH_OUT_Y_L                0x2A
#define LIS3DH_OUT_Y_H                0x2B
#define LIS3DH_OUT_Z_L                0x2C
#define LIS3DH_OUT_Z_H                0x2D
#define LIS3DH_FIFO_CTRL_REG          0x2E
#define LIS3DH_FIFO_SRC_REG           0x2F
#define LIS3DH_INT1_CFG               0x30
#define LIS3DH_INT1_SRC               0x31
#define LIS3DH_INT1_THS               0x32
#define LIS3DH_INT1_DURATION          0x33
#define LIS3DH_INT2_CFG               0x34
#define LIS3DH_INT2_SRC               0x35
#define LIS3DH_INT2_THS               0x36
#define LIS3DH_INT2_DURATION          0x37

#define LIS3DH_CLICK_CFG              0x38
#define LIS3DH_CLICK_SRC              0x39
#define LIS3DH_CLICK_THS              0x3A
#define LIS3DH_TIME_LIMIT             0x3B
#define LIS3DH_TIME_LATENCY           0x3C
#define LIS3DH_TIME_WINDOW            0x3D

#define LIS3DH_ACT_THS                0x3E
#define LIS3DH_ACT_DUR                0x3F

#endif // End of __LIS3DH_IMU_H__ definition check
