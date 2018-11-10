/*
 * Motor_IMU_Shield.cpp - Library For Using Brushless DC Motor Driver Board With
 * Integrated IMU
 * Created by Michael Lazernik and Dillon Haughton
 * MIT License
 */

#include "Arduino.h"
#include "Motor_IMU_Shield.h"

#define SPI_SETTINGS SPISettings(10000000,MSBFIRST,SPI_MODE3)

//Initialize array of object pointers
Motor* Motor::_motors[4] = {NULL, NULL, NULL, NULL};

/*****************
 * Read Function *
 *****************/
uint16_t Motor::registerRead(uint16_t addr) {
	_outputBuffer = addr << 13;
        SPI.beginTransaction(SPI_SETTINGS);
        digitalWrite(_cs, LOW);
        _inputBuffer = SPI.transfer16(_outputBuffer);
        digitalWrite(_cs, HIGH);
	//Check for error and store faults if present
	//Faults must be cleared by error handler or else will remain
	if (_inputBuffer >> 15) {
		digitalWrite(_cs, LOW);
		_registers.fault = SPI.transfer16(A4963_FAULT << 13);
		digitalWrite(_cs, HIGH);
	}
	SPI.endTransaction();
	return (_inputBuffer & 0x0FFF);
}

/******************
 * Write Function *
 ******************/
void Motor::registerWrite(uint16_t addr, uint16_t data) {
	Serial.print("Write: ");
	_outputBuffer = (addr << 13) | 0x1000 | data;
	Serial.println(_outputBuffer, BIN);
	SPI.beginTransaction(SPI_SETTINGS);
	digitalWrite(_cs, LOW);
	_registers.fault = SPI.transfer16(_outputBuffer);
	digitalWrite(_cs, HIGH);
	SPI.endTransaction();
}

/***************
 * Error Check *
 ***************/
boolean Motor::faultCheck() {
	if (_registers.fault) {
	return true;
	}

        SPI.beginTransaction(SPI_SETTINGS);
	digitalWrite(_cs, LOW);

	if (digitalRead(MISO)) {
		_registers.fault = SPI.transfer16(A4963_FAULT << 13);
		digitalWrite(_cs, HIGH);
		SPI.endTransaction();
		return true;
	}

	digitalWrite(_cs, HIGH);
	SPI.endTransaction();
	_registers.fault = 0x0000;
	return false;
}

/*********************
 * Motor Constructor *
 *********************/
Motor::Motor(uint8_t motorTerminal, float maxCurrent, uint8_t numPoles, uint16_t maxSpeed) {
	if (motorTerminal != MOTOR_A && motorTerminal != MOTOR_B && motorTerminal != MOTOR_C && motorTerminal != MOTOR_D) {
                badTerminal();
        }
        
        //if first instance
        if (&_motors[0] == NULL && &_motors[1] == NULL && &_motors[2] == NULL && &_motors[3] == NULL) {
                SPI.usingInterrupt(digitalPinToInterrupt(2));
                attachInterrupt(digitalPinToInterrupt(2), Motor::faultInterrupt, FALLING);
        SPI.begin();
        }

	_cs = motorTerminal;
	switch(_cs) {
		case MOTOR_A:
			_motors[0] = this;
			break;
		case MOTOR_B:
			_motors[1] = this;
			break;
		case MOTOR_C:
			_motors[2] = this;
			break;
		case MOTOR_D:
			_motors[3] = this;
			break;
	}

	pinMode(_cs, OUTPUT);
	digitalWrite(_cs, HIGH);

	/**********************
	 * Get Current Config *
	 **********************/
	_registers.conf0 = registerRead(A4963_CONF0);
	_registers.conf1 = registerRead(A4963_CONF1);
	_registers.conf2 = registerRead(A4963_CONF2);
	_registers.conf3 = registerRead(A4963_CONF3);
	_registers.conf4 = registerRead(A4963_CONF4);
	_registers.conf5 = registerRead(A4963_CONF5);
	_registers.run = registerRead(A4963_RUN);
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
	_registers.conf1 |= ( ( (uint16_t) round( ( ( (maxCurrent * 0.005) / 0.0125 ) - 1.0 ) ) << 6 ) );
	
	registerWrite(A4963_CONF1, _registers.conf1);
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
	
	_registers.conf5 &= 0xFF8F;
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

	_registers.run &= 0xFE0B;
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
	_registers.run = registerRead(A4963_RUN);
	if (faultCheck()) {
		//TODO error handling
	}
	return (int) ( ( 3 * ( ( _registers.run & 0x01F0 ) >> 4 ) ) + 7 );
}

/***********************
 * Set Motor Direction *
 ***********************/
boolean Motor::setDirection(uint8_t dir) {
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
boolean Motor::restart() {
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
boolean Motor::coast() {
	_registers.run &= 0xFFFE;
	registerWrite(A4963_RUN, _registers.run);
	if (faultCheck()) {
		//TODO error handling
	}
	
	return true;
}

/**************
 * Destructor *
 **************/
Motor::~Motor() {
	switch(_cs) {
                case MOTOR_A:
                        _motors[0] = NULL;
                        break;
                case MOTOR_B:
                        _motors[1] = NULL;
                        break;
                case MOTOR_C:
                        _motors[2] = NULL;
                        break;
                case MOTOR_D:
                        _motors[3] = NULL;
                        break;
        }
	if (_motors[0] == NULL && _motors[1] == NULL && _motors[2] == NULL && _motors[3] == NULL) {
		SPI.end();
		detachInterrupt(digitalPinToInterrupt(2));
	}
}

/*******************
 * Fault Interrupt *
 *******************/
void Motor::faultInterrupt() {
	//TODO fault interrupt
}
