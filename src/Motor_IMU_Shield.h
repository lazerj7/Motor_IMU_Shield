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

class Motor{
	public:
		Motor(uint8_t motorTerminal, float maxCurrent = 20.0, uint8_t numPoles = 6, uint16_t maxSpeed = 32767);
		~Motor();
		uint16_t registerRead(uint16_t addr);
                void registerWrite(uint16_t addr, uint16_t data);
                boolean faultCheck();
		boolean setMode(uint16_t controlMode);
		boolean setSpeed(uint8_t speed);
		int getSpeed();
		boolean setDirection(uint8_t dir);
		boolean restart();
		boolean coast();
		static void faultInterrupt();
	private:
		static void badTerminal()  __attribute__((error("Invalid Motor Terminal!")));
		uint16_t _outputBuffer;
		uint16_t _inputBuffer;
		uint8_t _cs;
		struct registers {
			uint16_t fault;
			uint16_t conf0;
			uint16_t conf1;
			uint16_t conf2;
			uint16_t conf3;
			uint16_t conf4;
			uint16_t conf5;
			uint16_t run;
		} _registers;
		static Motor* _motors[4];
};

class IMU {
	public:
		IMU(uint8_t addr, boolean serial = false, boolean interrupt = true);
		~IMU();
		uint8_t registerRead(uint8_t page, uint8_t reg);
		void registerWrite(uint8_t page, uint8_t reg, uint8_t value);
		boolean calibrate();
		void saveCalibration();
		void eepromClear();
		boolean restoreCalibration();
		void setMode(uint8_t mode);
		void suspend();
		void wake();
		static void update();
		uint16_t status();
		struct cal {
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
		} calibration;
		struct acc {
			volatile uint16_t x;
			volatile uint16_t y;
			volatile uint16_t z;
		} accelerometer;
		struct mag {
			volatile uint16_t x;
			volatile uint16_t y;
			volatile uint16_t z;
		} magnetometer;
		struct gyr {
			volatile uint16_t x;
			volatile uint16_t y;
			volatile uint16_t z;
		} gyroscope;
		struct eul {
			volatile uint16_t yaw;
			volatile uint16_t roll;
			volatile uint16_t pitch;
		} euler;
		struct qua {
			volatile uint16_t w;
			volatile uint16_t x;
			volatile uint16_t y;
			volatile uint16_t z;
		} quaternion;
		struct linacc {
			volatile uint16_t x;
			volatile uint16_t y;
			volatile uint16_t z;
		};
	       	linacc linearAcceleration;
		struct grav {
			volatile uint16_t x;
			volatile uint16_t y;
			volatile uint16_t z;
		} gravityVector;
		volatile uint16_t temperature;
	private:
		static void badAddress()  __attribute__((error("Invalid IMU Address!")));
		static IMU* _instance;
		uint8_t _addr;
		uint8_t _page;
		uint8_t _mode;
		uint8_t _useSerial;
		uint8_t _noInt;
};

#endif
