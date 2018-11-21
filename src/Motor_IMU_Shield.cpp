/*
 * Motor_IMU_Shield.cpp - Library For Using Brushless DC Motor Driver Board With
 * Integrated IMU
 * Created by Michael Lazernik and Dillon Haughton
 * MIT License
 */

#include "Arduino.h"
#include "Motor_IMU_Shield.h"

#define SPI_SETTINGS SPISettings(10000000,MSBFIRST,SPI_MODE3)
#define I2C_CLK 400000
#define IMU_INTERRUPT_FREQUENCY 60

//Initialize array of object pointers
Motor* Motor::_motors[4] = {nullptr, nullptr, nullptr, nullptr};

//Initialize IMU instance
IMU* IMU::_instance = nullptr;

/***********************
 * Motor Read Function *
 ***********************/
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

/************************
 * Motor Write Function *
 ************************/
void Motor::registerWrite(uint16_t addr, uint16_t data) {
	_outputBuffer = (addr << 13) | 0x1000 | data;
	SPI.beginTransaction(SPI_SETTINGS);
	digitalWrite(_cs, LOW);
	_registers.fault = SPI.transfer16(_outputBuffer);
	digitalWrite(_cs, HIGH);
	SPI.endTransaction();
}

/*********************
 * Motor Error Check *
 *********************/
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
        if (_motors[0] == nullptr && _motors[1] == nullptr && _motors[2] == nullptr && _motors[3] == nullptr) {
                SPI.usingInterrupt(digitalPinToInterrupt(2));
                attachInterrupt(digitalPinToInterrupt(2), Motor::faultInterrupt, FALLING);
        	SPI.begin();
       }

	_cs = motorTerminal;
	switch(_cs) {
		case MOTOR_A:
			if (_motors[0] != nullptr) {
				delete _motors[0];
			}
			_motors[0] = this;
			break;
		case MOTOR_B:
			if (_motors[1] != nullptr) {
                                delete _motors[1];
                        }
			_motors[1] = this;
			break;
		case MOTOR_C:
			if (_motors[2] != nullptr) {
                                delete _motors[2];
                        }
			_motors[2] = this;
			break;
		case MOTOR_D:
			if (_motors[3] != nullptr) {
                                delete _motors[3];
                        }
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

/**********************
 * Motor Set Run Mode *
 **********************/
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

/****************************
 * Set Motor Speed          *
 * (Also sets motor to run) *
 ****************************/
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

/********************
 * Motor Destructor *
 ********************/
Motor::~Motor() {
	switch(_cs) {
                case MOTOR_A:
                        _motors[0] = nullptr;
                        break;
                case MOTOR_B:
                        _motors[1] = nullptr;
                        break;
                case MOTOR_C:
                        _motors[2] = nullptr;
                        break;
                case MOTOR_D:
                        _motors[3] = nullptr;
                        break;
        }
	if (_motors[0] == nullptr && _motors[1] == nullptr && _motors[2] == nullptr && _motors[3] == nullptr) {
		SPI.end();
		detachInterrupt(digitalPinToInterrupt(2));
	}
}

/*************************
 * Motor Fault Interrupt *
 *************************/
void Motor::faultInterrupt() {
	//TODO fault interrupt
}

/********************************************************************************
 * IMU Constructor                                                              *
 * NOTE: Confirmed Arduino Documentation is wrong. Wire.requestFrom returns the *
 *       number of bytes REQUESTED, not the number of bytes RECIEVED. Therefore *
 *       it is not useful to test this return value. Code amended, multiple     *
 *       people have pointed this out online but Arduino doesn't seem to care   *
 *       to fix their documentation........                                     *
 ********************************************************************************/
IMU::IMU (uint8_t addr, boolean serial, boolean interrupt) {
	//check addr
	if(addr != JUMPERED && addr != NOT_JUMPERED) {
		badAddress();
	}

	//delete previous instance if it exists
	if (_instance != nullptr) {
		delete _instance;
	}

	//Serial Communication
	_useSerial = serial;
	if (_useSerial) {
		Serial.begin(9600);
	}

	Wire.begin();
	Wire.setClock(I2C_CLK);

	_instance = this;

	//Store address
	_addr = addr;

	//get current register page
	Wire.beginTransmission(_addr);
	Wire.write(0x07);
	if (!Wire.endTransmission(0)) {
		//TODO error handling
	}
	if (Wire.requestFrom((int) _addr, 1, 1)) {
		_page = Wire.read();
	}
	else {
		//TODO error handling
	}

	//Set Normal Power Mode
	registerWrite(0x00, 0x3E, 0x00);
	
	//enter config mode (should be default but just in case...)
	registerWrite(0x00, 0x3D, 0x00);

	//self test
	registerWrite(0x00, 0x3F, 0x01);
	delayMicroseconds(50);
	if (registerRead(0x00, 0x36) != 0x0F) {
		//TODO error handling
	}

	//set units to celsius for temp, degrees for angles, m/s^2 for acceleration
	registerWrite(0x00, 0x3B, 0x00);

	//disable interrupts (since interrupt pin isn't connected)
	registerWrite(0x01, 0x10, 0x00);
	registerWrite(0x01, 0x0F, 0x00);

	//Set axis to defaults (again, just in case...)
	registerWrite(0x00, 0x42, 0x00);
	registerWrite(0x00, 0x41, 0x24);

	//Set Up Timer2 Interrupt to read orientation data at ~IMU_INTERRUPT_FREQUENCY
	uint8_t compare;
	uint8_t prescaler;
	if (interrupt) {
		if (F_CPU/(uint32_t) IMU_INTERRUPT_FREQUENCY < 256) {
			prescaler = 0x01;
			compare = (uint8_t) (F_CPU/(uint32_t) IMU_INTERRUPT_FREQUENCY);
		}
		else if (F_CPU/((uint32_t) IMU_INTERRUPT_FREQUENCY * 8) < 256) {
			prescaler = 0x02;
			compare = (uint8_t) (F_CPU/((uint32_t) IMU_INTERRUPT_FREQUENCY * 8));
		}
		else if (F_CPU/((uint32_t) IMU_INTERRUPT_FREQUENCY * 32) < 256) {
			prescaler = 0x03;
			compare = (uint8_t) (F_CPU/((uint32_t) IMU_INTERRUPT_FREQUENCY * 32));
		}
		else if (F_CPU/((uint32_t) IMU_INTERRUPT_FREQUENCY * 64) < 256) {
			prescaler = 0x04;
			compare = (uint8_t) (F_CPU/((uint32_t) IMU_INTERRUPT_FREQUENCY * 64));
		}
		else if (F_CPU/((uint32_t) IMU_INTERRUPT_FREQUENCY * 128) < 256) {
			prescaler = 0x05;
			compare = (uint8_t) (F_CPU/((uint32_t) IMU_INTERRUPT_FREQUENCY * 128));
		}
		else if (F_CPU/((uint32_t) IMU_INTERRUPT_FREQUENCY * 256) < 256) {
			prescaler = 0x06;
			compare = (uint8_t) (F_CPU/((uint32_t) IMU_INTERRUPT_FREQUENCY * 256));
		}
		else if (F_CPU/((uint32_t) IMU_INTERRUPT_FREQUENCY * 1024) < 256) {
			prescaler = 0x07;
			compare = (uint8_t) (F_CPU/((uint32_t) IMU_INTERRUPT_FREQUENCY * 1024));
		}
		else {
			compare = 0;
		}
	}

	if (compare == 0) {
		interrupt = false;
	}

	if (interrupt) {
		//configure Timer2
		noInterrupts();
		TCCR2A = (TCCR2A & 0x0C) | 0x02;	//Set CTC Mode
		TCCR2B &= 0xF0;
		TCCR2B |= prescaler;	//Set Prescaler
		OCR2A = compare;	//Set Compare Value
		TIMSK2 = (TIMSK2 & 0xFD) | 0x02; //Enable Timer Compare Interrupt
		TCNT2 = 0;	//Restart Timer at 0	
		interrupts();
	}
}

/**************************
 * IMU Read From Register *
 **************************/
uint8_t IMU::registerRead(uint8_t page, uint8_t reg) {
	if (_page != page) {
		Wire.beginTransmission(_addr);
		Wire.write(0x07);
		Wire.write((int) page);
		if (!Wire.endTransmission(1)) {
			//TODO error handling
		}
		else {
		_page = page;
		}
	}

	Wire.beginTransmission(_addr);
	Wire.write((int) reg);
	if (!Wire.endTransmission(0)) {
		//TODO error handling
	}
	if (Wire.requestFrom((int) _addr, 1, 1)) {
	}
	else {
		//TODO error handling
	}
	return (uint8_t) Wire.read();
}

/*************************
 * IMU Write To Register *
 *************************/
void IMU::registerWrite(uint8_t page, uint8_t reg, uint8_t value) {
	if (_page != page) {
                Wire.beginTransmission(_addr);
                Wire.write(0x07);
                Wire.write((int) page);
               if (!Wire.endTransmission(1)) {
		       //TODO error handling
	       }
	       else {
                _page = page;
	       }
        }
	
	Wire.beginTransmission(_addr);
	Wire.write((int) reg);
	Wire.write((int) value);
	if (!Wire.endTransmission(1)) {
		//TODO error handling
	}
}

/*****************
 * IMU Calibrate *
 *****************/
boolean IMU::calibrate() {
	if(_mode < 0x08) {
		if (_useSerial) {
			Serial.println("IMU ERROR: Can Only Perform Calibration When In a Fusion Mode!");
		}
		return false;
	}

	unsigned long started = millis();
	unsigned long time = 0;
	uint8_t status = 0;

	if (_useSerial) {
		Serial.println("IMU Calibration Started.");
		Serial.println("To calibrate gyroscope set device on solid surface and ensure no movement.");
	}

	while (status < 3 && time < 60000) {
		status = (registerRead(0x00, 0x35) >> 4) & 0x03;
		time = millis() - started;
	}

	if (status != 3) {
		if (_useSerial) {
			Serial.println("IMU ERROR: Gyroscope Calibration Failed!");
			Serial.println("IMU ERROR: Calibration Failed!");
		}
		return false;
	}

	if (_useSerial) {
		Serial.println("Gyroscope Calibrated!");
		Serial.println("To calibrate accelerometer place device in up to 6 different stable positions, pausing for a few seconds in each position, with slow movement between each position. Ensure the device is laying at least once perpindicular to x, y, and z axis.");
	}

	status = 0;
	time = 0;
	started = millis();
	while (status < 3 && time < 60000) {
		status = (registerRead(0x00, 0x35) >> 2) & 0x03;
		time = millis() - started;
	}

	if (status != 3) {
		if (_useSerial) {
			Serial.println("IMU ERROR: Accelerometer Calibration Failed!");
			Serial.println("IMU ERROR: Calibration Failed!");
		}
		return false;
	}

	if (_useSerial) {
		Serial.println("Accelerometer Calibrated!");
		Serial.println("To calibrate magnetometer, make some random movements with device");
	}

	status = 0;
	time = 0;
	started = millis();
	while (status < 3 && time < 60000) {
		status = registerRead(0x00, 0x35) & 0x03;
		time = millis() - started;
	}

	if (status != 3) {
		if (_useSerial) {
			Serial.println("IMU ERROR: Magnetometer calibration failed!");
			Serial.println("IMU ERROR: Calibration Failed!");
		}
		return false;
	}

	if (_useSerial) {
		Serial.println("Magnetometer Calibrated!");
	}

	status = registerRead(0x00, 0x35) >> 6;

	if (status == 3) {
		if (_useSerial) {
			Serial.println("IMU Calibrated!");
		}

		uint8_t lsb;
        	uint16_t msb;
        	uint16_t* cal = (uint16_t*) &calibration;
        	Wire.beginTransmission(_addr);
        	Wire.write(0x07);
        	Wire.write(0x00);
        	if (!Wire.endTransmission(1)) {
			//TODO error handling
		}
		else {
			_page = 0x00;
		}
        	Wire.beginTransmission(_addr);
        	Wire.write(0x55);
        	if (!Wire.endTransmission(0)) {
			//TODO error handling
		}
        	if (Wire.requestFrom((int) _addr, 22, 1)) {
        		for (int i = 11; i >= 0; i--) {
        		lsb = Wire.read();
        		msb = Wire.read();
        		*(cal + i) = (msb << 8) | lsb;
        		}
		}
		else {
			//TODO error handling
		}

		return true;
	}
	else {
		if (_useSerial) {
			Serial.println("IMU ERROR: Calibration Failed!");
		}

		return false;
	}
}

/**********************************
 * IMU Save Calibration To EEPROM *
 **********************************/
void IMU::saveCalibration() {
	Wire.beginTransmission(_addr);
        Wire.write(0x07);
        Wire.write(0x01);
        if (!Wire.endTransmission(1)) {
                //TODO error handling
        }
        else {
                _page = 0x01;
        }
        Wire.beginTransmission(_addr);
        Wire.write(0x50);
        if (!Wire.endTransmission(0)) {
                //TODO error handling
        }
        if (Wire.requestFrom((int) _addr, 16, 1)) {
		uint8_t id[16];
		for (int i = 16; i >= 0; i--) {
			id[i] = Wire.read();
		}
		EEPROM.put(0, id);
		EEPROM.put(sizeof(id), calibration);
	}
	else {
		//TODO error handling
	}
}

/****************
 * Clear EEPROM *
 ****************/
void IMU::eepromClear() {
	for (unsigned int i = 0; i < EEPROM.length(); i++) {
		EEPROM.write(i, 0);
	}
}

/***************************************
 * IMU Restore Calibration From EEPROM *
 ***************************************/
boolean IMU::restoreCalibration() {
	uint8_t id[16];
	EEPROM.get(0, id);
	Wire.beginTransmission(_addr);
        Wire.write(0x07);
        Wire.write(0x01);
        if (!Wire.endTransmission(1)) {
                //TODO error handling
        }
        else {
                _page = 0x01;
        }
        Wire.beginTransmission(_addr);
        Wire.write(0x50);
        if (!Wire.endTransmission(0)) {
                //TODO error handling
        }
        if (Wire.requestFrom((int) _addr, 16, 1)) {
        	for (int i = 16; i >= 0; i--) {
                	if (id[i] != Wire.read()) {
				return false;
			}
		}
	}
	else {
		//TODO error handling
	}

	uint8_t msb;
	uint8_t lsb;
	uint16_t* cal = (uint16_t*) &calibration;
	uint8_t startAddr = 0x6A;

	EEPROM.get(sizeof(id), calibration);
	Wire.beginTransmission(_addr);
	Wire.write(0x07);
	Wire.write(0x00);
	if (!Wire.endTransmission(1)) {
		//TODO error handling
	}
	else {
		_page = 0x00;
	}
	for(int i = 0; i < 11; i++) {
		msb = (uint8_t) ( (*(cal + i)) >> 8 );
		lsb = (uint8_t) ( (*(cal + i)) & 0x0F );
		Wire.beginTransmission(_addr);
		Wire.write(startAddr--);
		Wire.write(msb);
		if (!Wire.endTransmission(1)) {
			//TODO error handling
		}
		Wire.beginTransmission(_addr);
		Wire.write(startAddr--);
		Wire.write(lsb);
		if (!Wire.endTransmission(1)) {
			//TODO error handling
		}
	}
	return true;
}

/**************************
 * IMU Set Operating Mode *
 **************************/
void IMU::setMode(uint8_t mode) {
	//Check for valid mode
	if (mode > 0x0C) {
		return;
	}

	registerWrite(0x00, 0x3D, mode);
}

/********************************
 * IMU Enter Suspend Power Mode *
 ********************************/
void IMU::suspend() {
	registerWrite(0x00, 0x3E, 0x02);
}

/*************************
 * IMU Wake From Suspend *
 *************************/
void IMU::wake() {
	registerWrite(0x00, 0x3E, 0x00);
}

/******************************************
 * IMU Status                             *
 * MSB gives status, LSB gives error code *
 ******************************************/
uint16_t IMU::status() {
	uint16_t ret = (uint16_t) registerRead(0x00, 0x39);
	ret <<= 8;
	ret |= (uint16_t) registerRead(0x00, 0x3A);
	return ret;
}

/*****************************
 * IMU Read Orientation Data *
 *****************************/
void IMU::update() {
	//TODO delete below me
	Serial.println(micros(), DEC);
	Serial.flush();
	//TODO delete above me

	uint8_t lsb;
        uint16_t msb;
        Wire.beginTransmission(_instance->_addr);
        Wire.write(0x07);
        Wire.write(0x00);
        if (!Wire.endTransmission(1)) {
		//TODO error handling
	}
	else {
		_instance->_page = 0x00;
	}
	Wire.end();
        Wire.beginTransmission(_instance->_addr);
        Wire.write(0x08);
        if (!Wire.endTransmission(0)) {
		//TODO error handling
	}
        if (Wire.requestFrom((int) _instance->_addr, 32, 1)) {
        	lsb = Wire.read();
		msb = Wire.read();
		_instance->accelerometer.x = (msb << 8) | lsb;

		lsb = Wire.read();
		msb = Wire.read();
		_instance->accelerometer.y = (msb << 8) | lsb;

		lsb = Wire.read();
		msb = Wire.read();
		_instance->accelerometer.z = (msb << 8) | lsb;
	
		lsb = Wire.read();
		msb = Wire.read();
		_instance->magnetometer.x = (msb << 8) | lsb;

		lsb = Wire.read();
		msb = Wire.read();
		_instance->magnetometer.y = (msb << 8) | lsb;

		lsb = Wire.read();
		msb = Wire.read();
		_instance->magnetometer.z = (msb << 8) | lsb;
	
		lsb = Wire.read();
		msb = Wire.read();
		_instance->gyroscope.x = (msb << 8) | lsb;

		lsb = Wire.read();
		msb = Wire.read();
		_instance->gyroscope.y = (msb << 8) | lsb;
	
		lsb = Wire.read();
		msb = Wire.read();
		_instance->gyroscope.z = (msb << 8) | lsb;
	
		lsb = Wire.read();
		msb = Wire.read();
		_instance->euler.yaw = (msb << 8) | lsb;
		
		lsb = Wire.read();
		msb = Wire.read();
		_instance->euler.roll = (msb << 8) | lsb;

		lsb = Wire.read();
		msb = Wire.read();
		_instance->euler.pitch = (msb << 8) | lsb;

		lsb = Wire.read();
		msb = Wire.read();
		_instance->quaternion.w = (msb << 8) | lsb;

		lsb = Wire.read();
		msb = Wire.read();
		_instance->quaternion.x = (msb << 8) | lsb;

		lsb = Wire.read();
		msb = Wire.read();
		_instance->quaternion.y = (msb << 8) | lsb;

		lsb = Wire.read();
		msb = Wire.read();
		_instance->quaternion.z = (msb << 8) | lsb;
	}
	else {
		//TODO error handling
	}

	if (Wire.requestFrom((int) _instance->_addr, 14, 1)) {	
		lsb = Wire.read();
		msb = Wire.read();
		_instance->linearAcceleration.x = (msb << 8) | lsb;

		lsb = Wire.read();
		msb = Wire.read();
		_instance->linearAcceleration.y = (msb << 8) | lsb;

		lsb = Wire.read();
		msb = Wire.read();
		_instance->linearAcceleration.z = (msb << 8) | lsb;

		lsb = Wire.read();
		msb = Wire.read();
		_instance->gravityVector.x = (msb << 8) | lsb;

		lsb = Wire.read();
		msb = Wire.read();
		_instance->gravityVector.y = (msb << 8) | lsb;

		lsb = Wire.read();
		msb = Wire.read();
		_instance->gravityVector.z = (msb << 8) | lsb;

		lsb = Wire.read();
		msb = Wire.read();
		_instance->temperature = (msb << 8) | lsb;
	}
	else {
		//TODO error handling
	}
	//TODO delete below me
	Serial.println(micros(), DEC);
	Serial.flush();
	//TODO delete above me
}

/**************************************
 * Timer Interrupt To Update IMU Data *
 **************************************/
ISR(TIMER2_COMPA_vect) {
	IMU::_interruptHelper helper;
}

/******************
 * IMU Destructor *
 ******************/
IMU::~IMU() {
	_instance = nullptr;
	Wire.end();
}
