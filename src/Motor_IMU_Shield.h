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

#define MOTOR_A 3		//Motor Terminal A SS on Pin 3
#define MOTOR_B 4		//Motor Terminal B SS on Pin 4
#define MOTOR_C 5		//Motor Terminal C SS on Pin 5
#define MOTOR_D 6		//Motor Terminal D SS on Pin 6

/***********************
 * Control Mode Values *
 ***********************/
#define INDIRECT_SPEED 0x0000	//Default
#define DIRECT_SPEED 0x0001
#define CLOSED_LOOP_CURRENT 0x0002
#define CLOSED_LOOP_SPEED 0x0003

/********************
 * Direction Values *
 ********************/
#define FORWARD 0x0000		//Default
#define REVERSE 0x0001

class Motor{
	public:
		Motor(uint8_t motorTerminal, float maxCurrent = 20.0, uint8_t numPoles = 6, uint16_t maxSpeed = 32767);
		uint16_t registerRead(uint16_t addr);
                void registerWrite(uint16_t addr, uint16_t data);
                boolean faultCheck();
		boolean setMode(uint16_t controlMode);
		boolean setSpeed(uint8_t speed);
		int getSpeed();
		boolean setDirection(uint8_t dir);
		boolean restart();
		boolean coast();
		boolean brake();
	private:
		uint16_t _outputBuffer;
		uint16_t _inputBuffer;
		uint8_t _portMask;
		struct _registers {
			uint16_t fault;
			uint16_t conf0;
			uint16_t conf1;
			uint16_t conf2;
			uint16_t conf3;
			uint16_t conf4;
			uint16_t conf5;
			uint16_t run;
		}
};

#endif
