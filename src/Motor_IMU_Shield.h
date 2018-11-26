/*
 * Motor_IMU_Shield.h - Library For Using Brushless DC Motor Driver Board With
 * Integrated IMU
 * Created by Michael Lazernik and Dillon Haughton
 * MIT License
 */

#ifndef Motor_IMU_Shield_h
#define Motor_IMU_Shield_h

#include "Arduino.h"	//Arduino Functions
#include "SPI.h"	//SPI Library
#include "Wire.h"	//I2C Library
#include "EEPROM.h"	//EEPROM Library

/*************************************************
 * A4963 Motor Controller Registers              *
 * 3 Most Significant Bits Of SPI Communications *
 *************************************************/
#define A4963_CONF0 0x0000	//12-bit Timing Configuration Register
#define A4963_CONF1 0x0001	//12-bit Voltage Configuration Register
#define A4963_CONF2 0x0002	//12-bit PWM Configuration Register
#define A4963_CONF3 0x0003	//12-bit Hold Configuration Register
#define A4963_CONF4 0x0004	//12-bit Start Configuration Register
#define A4963_CONF5 0x0005	//12-bit Other Configuration Register
#define A4963_FAULT 0x0006	//12-bit Fault Mask Register
#define A4963_RUN 0x0007	//12-bit Run Register

/*******************
 * Motor Terminals *
 *******************/
#define MOTOR_A 3		//Motor Terminal A SS on Pin 3
#define MOTOR_B 4		//Motor Terminal B SS on Pin 4
#define MOTOR_C 5		//Motor Terminal C SS on Pin 5
#define MOTOR_D 6		//Motor Terminal D SS on Pin 6

/*****************************
 * Motor Control Mode Values *
 *****************************/
#define INDIRECT_SPEED 0x0000	//Default
#define DIRECT_SPEED 0x0001
#define CLOSED_LOOP_CURRENT 0x0002
#define CLOSED_LOOP_SPEED 0x0003

/**************************
 * Motor Direction Values *
 **************************/
#define FORWARD 0x0000		//Default
#define REVERSE 0x0001

/********************
 * BNO055 Addresses *
 ********************/
#define JUMPERED 0X28
#define CLOSED 0X28
#define NOT_JUMPERED 0X29
#define OPEN 0x29

/**************************
 * BNO055 Operating Modes *
 **************************/
#define CONFIG_MODE 0x00
#define ACC_ONLY_MODE 0x01
#define MAG_ONLY_MODE 0x02
#define GYRO_ONLY_MODE 0x03
#define ACC_MAG_MODE 0x04
#define ACC_GYRO_MODE 0x05
#define MAG_GYRO_MODE 0x06
#define AMG_MODE 0x07
#define IMU_MODE 0x08
#define COMPASS_MODE 0x09
#define M4G_MODE 0x0A
#define NDOF_FMC_OFF_MODE 0x0B
#define NDOF_MODE 0x0C

/*********************
 * BNO055 Data Types *
 *********************/
#define ACCELEROMETER 0
#define MAGNETOMETER 1
#define GYROSCOPE 2
#define EULER_ANGLES 3
#define QUATERNION 4
#define LINEAR_ACCELERATION 5
#define GRAVITY_VECTOR 6
#define TEMPERATURE 7

struct vector {
	double x;
	double y;
	double z;
};
struct euler {
	double yaw;
	double roll;
	double pitch;
};
struct quaternion {
	double w;
	double x;
	double y;
	double z;
};


struct Motor_IMU_Shield {
	public:
		void begin(uint8_t imuAddr, int serial = 9600);
		volatile boolean MOTOR_FAULT;
		volatile boolean IMU_FAULT;
	private:
		void badAddress()__attribute__((error("Invalid IMU Address")));
};

class Motor{
	public:
		Motor();
		~Motor();
		Motor* attach(uint8_t motorTerminal, float maxCurrent = 20.0, uint8_t numPoles = 6, uint16_t maxSpeed = 32767);
		uint16_t registerRead(uint16_t addr);
                void registerWrite(uint16_t addr, uint16_t data);
                boolean faultCheck();
		void setMode(uint16_t controlMode);
		void setSpeed(uint8_t speed);
		int getSpeed();
		void setDirection(uint8_t dir);
		void restart();
		void coast();
		static void faultInterrupt();
	private:
		uint8_t _cs;
};

class IMU {
	public:
		IMU();
		~IMU();
		IMU* attach();
		uint8_t registerRead(uint8_t page, uint8_t reg);
		void registerWrite(uint8_t page, uint8_t reg, uint8_t value);
		boolean calibrate();
		void saveCalibration();
		void eepromClear();
		boolean restoreCalibration();
		void setMode(uint8_t mode);
		void suspend();
		void wake();
		uint16_t status();
		struct dat{
			union {
				vector accelerometer;
				vector magnetometer;
				vector gyroscope;
				euler eulerData;
				quaternion quaternionData;
				vector linearAcceleration;
				vector gravityVector;
				int8_t temperature;
			};
			 void update(uint8_t dataType);
		} data;
		struct calibrationData {
			uint16_t magRadius;
			uint16_t accRadius;
			uint16_t gyroZOffset;
			uint16_t gyroYOffset;
			uint16_t gyroXOffset;
			uint16_t magZOffset;
			uint16_t magYOffset;
			uint16_t magXOffset;
			uint16_t accZOffset;
			uint16_t accYOffset;
			uint16_t accXOffset;
		};
		calibrationData getCalibration();
	private:
		static void _errorHandler();
		static uint8_t _addr;
		static uint8_t _page;
};

#endif
