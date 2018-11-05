/*
 * Motor_IMU_Shield.cpp - Library For Using Brushless DC Motor Driver Board With
 * Integrated IMU
 * Created by Michael Lazernik and Dillon Haughton
 * MIT License
 */

#include "Arduino.h"
#include "Motor_IMU_Shield.h"

#define SPI_SETTINGS SPISettings(10000000,MSBFIRST,SPI_MODE3)

/*****************
 * Read Function *
 *****************/
uint16_t registerRead(uint16_t addr) {
	_outputBuffer = addr << 13;
	SPI.begin();
        SPI.beginTransaction(SPI_SETTINGS);
        PORTD &= ~_portMask;
        _inputBuffer = SPI.transfer16(_outputBuffer);
        PORTD |= _portMask;
	//Check for error and store faults if present
	//Faults must be cleared by error handler or else will remain
	if (_inputBuffer >> 15) {
		PORTD &= ~_portMask;
		_registers.fault = SPI.transfer16(A4963_FAULT << 13);
		PORTD |= _portMask;
	}
	SPI.endTransaction();
	SPI.end();
	return (_inputBuffer & 0x0FFF);
}

/******************
 * Write Function *
 ******************/
void registerWrite(uint16_t addr, uint16_t data) {
	_outputBuffer = (addr << 13) | 0x1000 | data;
	SPI.begin();
	SPI.beginTransaction(SPI_SETTINGS);
	PORTD &= ~_portMask;
	_registers.fault = SPI.transfer16(_outputBuffer);
	PORTD |= _portMask;
	SPI.endTransaction();
	SPI.end();
}

/***************
 * Error Check *
 ***************/
boolean faultCheck() {
	if (_registers.fault) {
	return true;
	}

	SPI.begin();
        SPI.beginTransaction(SPI_SETTINGS);
	PORTD &= ~_portMask;

	if (digitalRead(MISO)) {
		_registers.fault = SPI.transfer16(A4963_FAULT << 13);
		PORTD |= _portMask;
		SPI.endTransaction();
		SPI.end();
		return true;
	}

	PORTD |= _portMask;
	SPI.endTransaction();
	SPI.end();
	_registers.fault = 0x0000;
	return false;
}

/*********************
 * Motor Constructor *
 *********************/
Motor::Motor(uint8_t motorTerminal, float maxCurrent, uint8_t numPoles, uint16_t maxSpeed) {
	pinMode(motorTerminal, OUTPUT);
	digitalWrite(motorTerminal, HIGH);

	/**********************************
	 * Set Mask For Port Manipulation *
	 **********************************/
	_portMask = (0x01 << motorTerminal);

	/**********************
	 * Get Current Config *
	 **********************/
	_registers.conf0 = registerRead(A4963_CONF0);
	_registers.conf1 = registerRead(A4963_CONF1);
	_registers.conf2 = registerRead(A4963_CONF2);
	_registers.conf3 = registerRead(A4963_CONF3);
	_registers.conf4 = registerRead(A4963_CONF4);
	_registers.conf5 = registerRead(A4963_CONF5);
	_registers.run = _read(A4963_RUN);
	//Check for error during reads
	//We expect a POR flag, so ignore it but any other flag
	//should be treated as an actual error
	if (_registers.fault & 0x3FFF) {
		//TODO error handling
		//clear fault
		_registers.fault = 0x0000;
	}

	/*********************
	 * Set Current Limit *
	 *********************/
	//Limit Max Current to 20 Amps
	if (maxCurrent > 20.0) {
		maxCurrent = 20.0;
	}
	//Min Value for Max Current 2.5 Amps
	else if (maxCurrent < 2.5) {
		maxCurrent = 2.5;
	}
	
	_registers.conf1 &= 0xFC3F; 
	_registers.conf1 |= ( ( (uint16_t) ( ( (maxCurrent * 0.005) / 12.5 ) - 1.0 ) ) << 6 );
	registerWrite(A4963_CONF1, _outputBuffer);
	if (faultCheck()) {
		//TODO error handling
	}
	
	/*****************
	 * Set Max Speed *
	 *****************/
	//Max Value in Electrical Cycle Frequency is 3276.7
	if ( ( ( ( (double) numPoles ) * ( (double) maxSpeed) ) / 60.0 ) > 3276.7 ) {
		maxSpeed = (uint16_t) ( 196602.0 / (double) numPoles );
	}
	
	_registers.conf5 &= 0xFF8F
	_registers.conf5 |= ( (uint16_t) ( round( (log( ( ( ((double) numPoles * (double) maxSpeed) / 6.0 ) + 1.0 ) ) / log(2.0) ) - 8.0 ) ) ) << 4;
	registerWrite(A4963_CONF5, _registers.conf5);
	if (faultCheck()) {
		//TODO error handling
	}
}

/****************
 * Set Run Mode *
 ****************/
boolean Motor::setMode(uint16_t controlMode) {
	//Only acceptable values or between 0x0000 and 0x0003
	//If outside this range, set to default indirect speed mode
	if (controlMode > 0x0003) {
		controlMode = 0x0000;
		}
	
	_registers.run &= 0xF3FF;
	_registers.run |= controlMode << 10;
	registerWrite(A4963_RUN, _registers.run);
	if (faultCheck()) {
		//TODO error handling
	}

	return true;
}

/***********************************************
 * Set Motor Speed 			       *
 * (Also sets motor to run and disables brake) *
 ***********************************************/
boolean Motor::setSpeed(uint8_t speed) {
	if (speed > 100) {
		speed = 100;
	}
	else if (speed < 7) {
		speed = 7;
	}

	_registers.run &= 0xFE0B
	_registers.run |= 0x0001 | ( (uint16_t) ( (speed - 7) / 3 ) ) << 4;
	registerWrite(A4963_RUN, _registers.run);
	if (faultCheck()) {
		//TODO error handling
	}

	return true;
}

/*******************
 * Get Motor Speed *
 *******************/
int Motor::getSpeed() {
	_registers.run = readRegister(A4963_RUN);
	if (faultCheck()) {
		//TODO error handling
	}
	return (int) ( ( 3 * ( ( _registers.run & 0x01F0 ) >> 4 ) ) + 7 );
}

/***********************
 * Set Motor Direction *
 ***********************/
boolean setDirection(uint8_t dir) {
	_registers.run &= 0xFFFD;
	_registers.run |= dir << 1;
	registerWrite(A4963_RUN, _registers.run);
	if (faultCheck()) {
		//TODO error handling
	}
	return true;
}

/********************************
 * Restart Motor After Coasting *
 ********************************/
boolean restart() {
	_registers.run |= 0x0001;
	registerWrite(A4963_RUN, _registers.run);
	if (faultCheck()) {
		//TODO error handling
	}

	return true;
}

/***************
 * Coast Motor *
 ***************/
boolean coast() {
	_registers.run &= 0xFFFE;
	registerWrite(A4963_RUN, _registers.run);
	if (faultCheck()) {
		//TODO error handling
	}
	
	return true;
}

/********************************************************************
 * Brake Motor                                                      *
 * (Sets speed to zero to Brake. Restart will Not Work After Brake. *
 *  Must call setSpeed to restart motor.)                           *
 ********************************************************************/
boolean brake() {
	_registers.run &= 0xFFFE;
	_register.run |= 0x0001;
	registerWrite(A4963_RUN, _registers.run);
	if (faultCheck()) {
		//TODO error handling
	}

	return true;
}
