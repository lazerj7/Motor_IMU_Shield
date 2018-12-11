/*
 * Motor_IMU_Shield.cpp - Library For Using Brushless DC Motor Driver Board With
 * Integrated IMU
 * Created by Michael Lazernik and Dillon Haughton
 * MIT License
 */

#include "Motor_IMU_Shield.h"

#define SPI_SETTINGS SPISettings(1000000,MSBFIRST,SPI_MODE3)
#define I2C_CLK 100000

namespace {
	struct {
		unsigned int motors: 4;
		unsigned int serial: 1;
		unsigned int spi: 1;
		unsigned int imu: 1;
		unsigned int imuAddr: 1;
	} state;

	volatile boolean* motorFaultPointer;
	volatile boolean* imuFaultPointer;
}

uint8_t IMU::_page = 5;
uint8_t IMU::_addr = 0;

/*********************
 * Initialize Shield *
 *********************/
void Motor_IMU_Shield::begin(uint8_t imuAddr, long serial) {
	state.spi = state.imu = state.motors = 0;
	MOTOR_FAULT = false;
	IMU_FAULT = false;
	motorFaultPointer = &MOTOR_FAULT;
	imuFaultPointer = &IMU_FAULT;
	if (serial) {
		Serial.begin(serial);
		state.serial = 1;
	}
	else {
		state.serial = 0;
	}

	SPI.begin();
	for (int i = MOTOR_A; i <= MOTOR_D; i++) {
		digitalWrite(i, HIGH);
		pinMode(i, OUTPUT);
		SPI.beginTransaction(SPI_SETTINGS);
		digitalWrite(i, LOW);
		SPI.transfer16( (A4963_RUN << 13) | 0x1008 );
		digitalWrite(i, HIGH);
		SPI.endTransaction();
	}
	SPI.end();
	if (imuAddr == NOT_JUMPERED) {
		state.imuAddr = 0;
	}
	else if (imuAddr == JUMPERED) {
		state.imuAddr = 1;
	}
	else {
		badAddress();
	}

	Wire.begin();
	Wire.setClock(I2C_CLK);
	Wire.beginTransmission(imuAddr);
	Wire.write(0x07);
	Wire.write(0x00);
	if (Wire.endTransmission(1) != 0) {
		delayMicroseconds(450);
		Wire.beginTransmission(imuAddr);
		Wire.write(0x07);
		Wire.write(0x00);
		Wire.endTransmission(1);
	}
	Wire.beginTransmission(imuAddr);
	Wire.write(0x3E);
	Wire.write(0x02);
	Wire.endTransmission(1);
	Wire.end();
}

/*********************
 * Motor Constructor *
 *********************/
Motor::Motor() {
	_cs = 0;
}

/****************
 * Attach Motor *
 ****************/
Motor* Motor::attach(uint8_t motorTerminal, float maxCurrent, uint8_t numPoles, uint32_t maxSpeed) {
	if (motorTerminal < MOTOR_A || motorTerminal > MOTOR_D) {
		return nullptr;
	}

	if (!state.spi) {
		SPI.usingInterrupt(digitalPinToInterrupt(2));
		//attachInterrupt(digitalPinToInterrupt(2), Motor::faultInterrupt, FALLING);
		SPI.begin();
		state.spi = 1;	
	}

	state.motors |= 1 << (motorTerminal - MOTOR_A);

	uint16_t buffer;
	uint16_t fault;

	_cs = motorTerminal;
	digitalWrite(_cs, HIGH);
	pinMode(_cs, OUTPUT);

	/**********************
	 * Set Default Config *
	 **********************/
	buffer = (A4963_CONF0 << 13) | 0x1000 | 0x0214;
	SPI.beginTransaction(SPI_SETTINGS);
	digitalWrite(_cs, LOW);
	fault = SPI.transfer16(buffer);
	digitalWrite(_cs, HIGH);
	SPI.endTransaction();
	if (fault & 0x4000) {
		faultInterrupt();
	}
	if (registerRead(A4963_CONF0) != 0x0214) {
		if (state.serial) {
			Serial.print(F("Failed to Communicate With Motor Controller "));
			switch (_cs) {
				case MOTOR_A:
					Serial.println(F("A"));
					break;
				case MOTOR_B:
					Serial.println(F("B"));
					break;
				case MOTOR_C:
					Serial.println(F("C"));
					break;
				case MOTOR_D:
					Serial.println(F("D"));
					break;
			}
		}
		return nullptr;
	}
	registerWrite(A4963_CONF1, 0x001F);
	registerWrite(A4963_CONF2, 0x0880);
	registerWrite(A4963_CONF3, 0x0809);
	registerWrite(A4963_CONF4, 0x0A0F);
	registerWrite(A4963_CONF5, 0x0308);
	registerWrite(A4963_RUN, 0x000A);
	registerWrite(A4963_FAULT, 0x0000);



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
	
	buffer = 0x001F;	
	buffer |= ( ( (uint16_t) round( ( ( (maxCurrent * 0.005) / 0.0125 ) - 1.0 ) ) << 6 ) );
	
	registerWrite(A4963_CONF1, buffer);
	
	/*****************
	 * Set Max Speed *
	 *****************/
	//Max Value in Electrical Cycle Frequency is 3276.7
	if ( ( ( ( (double) numPoles ) * ( (double) maxSpeed) ) / 60.0 ) > 3276.7 ) {
		maxSpeed = (uint16_t) ( 196602.0 / (double) numPoles );
	}
	
	buffer = 0x0708;
	buffer |= ( (uint16_t) ( round( (log( ( ( ((double) numPoles * (double) maxSpeed) / 6.0 ) + 1.0 ) ) / log(2.0) ) - 8.0 ) ) ) << 4;
	registerWrite(A4963_CONF5, buffer);

	return this;
}

/***********************
 * Motor Read Function *
 ***********************/
uint16_t Motor::registerRead(uint16_t addr) {
	uint16_t inputBuffer;
	uint16_t outputBuffer;
	outputBuffer = addr << 13;
        SPI.beginTransaction(SPI_SETTINGS);
        digitalWrite(_cs, LOW);
        inputBuffer = SPI.transfer16(outputBuffer);
        digitalWrite(_cs, HIGH);
	//Check for error and store faults if present
	if (inputBuffer >> 15) {
		faultInterrupt();
	}
	SPI.endTransaction();
	return (inputBuffer & 0x0FFF);
}

/************************
 * Motor Write Function *
 ************************/
void Motor::registerWrite(uint16_t addr, uint16_t data) {
	uint16_t inputBuffer;
	uint16_t outputBuffer;
	outputBuffer = (addr << 13) | 0x1000 | data;
	SPI.beginTransaction(SPI_SETTINGS);
	digitalWrite(_cs, LOW);
	inputBuffer = SPI.transfer16(outputBuffer);
	digitalWrite(_cs, HIGH);
	SPI.endTransaction();
	if (inputBuffer >> 15) {
		faultInterrupt();
	}
}

/*********************
 * Motor Error Check *
 *********************/
boolean Motor::faultCheck() {
	digitalWrite(_cs, LOW);

	if (digitalRead(MISO)) {
		digitalWrite(_cs, HIGH);
		faultInterrupt();
		return true;
	}
	digitalWrite(_cs, HIGH);
	return false;
}

/**********************
 * Motor Set Run Mode *
 **********************/
void Motor::setMode(uint16_t controlMode) {
	uint16_t buffer;

	//Only acceptable values or between 0x0000 and 0x0003
	//If outside this range, set to default indirect speed mode
	if (controlMode > 0x0003) {
		controlMode = 0x0000;
		}
	buffer = registerRead(A4963_RUN);
	buffer &= 0xF3FF;
	buffer |= controlMode << 10;
	registerWrite(A4963_RUN, buffer);
}

/****************************
 * Set Motor Speed          *
 * (Also sets motor to run) *
 ****************************/
void Motor::setSpeed(uint8_t speed) {
	uint16_t buffer;

	if (speed > 100) {
		speed = 100;
	}
	else if (speed < 7) {
		speed = 7;
	}

	buffer = registerRead(A4963_RUN);
	buffer &= 0xFE0B;
	buffer |= 0x0001 | ( (uint16_t) ( (speed - 7) / 3 ) ) << 4;
	registerWrite(A4963_RUN, buffer);
}

/*******************
 * Get Motor Speed *
 *******************/
int Motor::getSpeed() {
	uint16_t buffer;
	uint8_t speed;

	buffer = registerRead(A4963_RUN);
	speed = ((int) ( ( 3* ( ( buffer & 0x01F0 ) >> 4 ) ) + 7 ));
	if (speed <= 7) {
		return 0;
	}
	return speed;
}

/***********************
 * Set Motor Direction *
 ***********************/
void Motor::setDirection(uint8_t dir) {
	uint16_t buffer;

	buffer = registerRead(A4963_RUN);
	buffer &= 0xFFFD;
	buffer |= dir << 1;
	registerWrite(A4963_RUN, buffer);
}

/********************************
 * Restart Motor After Coasting *
 ********************************/
void Motor::restart() {
	uint16_t buffer;

	buffer = registerRead(A4963_RUN);
	buffer |= 0x0001;
	registerWrite(A4963_RUN, buffer);
}

/***************
 * Coast Motor *
 ***************/
void Motor::coast() {
	uint16_t buffer;

	buffer = registerRead(A4963_RUN);
	buffer &= 0xFFFE;
	registerWrite(A4963_RUN, buffer);
}

/********************
 * Motor Destructor *
 ********************/
Motor::~Motor() {
	state.motors &= ~(1 << (_cs - MOTOR_A)); 

	if (!state.motors) {
		SPI.end();
		state.spi = 0;
		detachInterrupt(digitalPinToInterrupt(2));
	}
}

/*************************
 * Motor Fault Interrupt *
 *************************/
void Motor::faultInterrupt() {
	/*
	//detachInterrupt(digitalPinToInterrupt(2));
	*motorFaultPointer = true;
	if (state.serial) {
		uint16_t buffer;
		uint16_t fault;
		
		//interrupts();

		for (int i = MOTOR_A; i <= MOTOR_D; i++) {
			buffer = A4963_FAULT << 13;
			SPI.beginTransaction(SPI_SETTINGS);
			digitalWrite(i, LOW);
			fault = SPI.transfer16(buffer);
			digitalWrite(i, HIGH);
			SPI.endTransaction();

			if ((fault >> 15) != 0x0000) {
				fault &= 0x6FFF;
				char motorController[] = "Motor Controller   Error: ";
				switch (i) {
					case MOTOR_A:
						motorController[17] = 'A';
						break;
					case MOTOR_B:
						motorController[17] = 'B';
						break;
					case MOTOR_C:
						motorController[17] = 'C';
						break;
					case MOTOR_D:
						motorController[17] = 'D';
						break;
				}
				for (int j = 14; j >= 0; j--) {
					if ((fault >> j) != 0x0000) {
						fault &= (~(0x0001 << j));
						Serial.print(motorController);
						switch (j) {
							case 14:
								Serial.println(F("Power On Reset"));
								break;
							case 13:
								Serial.println(F("Serial Transfer Error"));
								break;
							case 11:
								Serial.println(F("High Temperature Warning"));
								break;
							case 10:
								Serial.println(F("Overtemperature Shutdown"));
								break;
							case 9:
								Serial.println(F("Loss of Synchronization"));
								break;
							case 7:
								Serial.println(F("Low Supply Voltage"));
								break;
							case 5:
								Serial.println(F("Phase U High Side MOSFET Fault"));
								break;
							case 4:
								Serial.println(F("Phase U Low Side MOSFET Fault"));
								break;
							case 3:
								Serial.println(F("Phase V High Side MOSFET Fault"));
								break;
							case 2:
								Serial.println(F("Phase V Low Side MOSFET Fault"));
								break;
							case 1:
								Serial.println(F("Phase W High Side MOSFET Fault"));
								break;
							case 0:
								Serial.println(F("Phase W Low Side MOSFET Fault"));
								break;
						}
					}
				}
			}
		}
	}
	//noInterrupts();
	//attachInterrupt(digitalPinToInterrupt(2), Motor::faultInterrupt, FALLING);
*/
}

/********************************************************************************
 * IMU Constructor                                                              *
 * NOTE: Confirmed Arduino Documentation is wrong. Wire.requestFrom returns the *
 *       number of bytes REQUESTED, not the number of bytes RECIEVED. Therefore *
 *       it is not useful to test this return value. Code amended, multiple     *
 *       people have pointed this out online but Arduino doesn't seem to care   *
 *       to fix their documentation........                                     *
 ********************************************************************************/
IMU::IMU() {}

/**************
 * Attach IMU *
 **************/
IMU* IMU::attach () {
	uint8_t selfTest;
	uint16_t timeStart;
	uint16_t time;

	Wire.begin();
	Wire.setClock(I2C_CLK);

	//Store address
	if (state.imuAddr) {
		_addr = JUMPERED;
	}
	else {
		_addr = NOT_JUMPERED;
	}

	//See if IMU is there
	Wire.beginTransmission(_addr);
	Wire.write(0x07);
	Wire.write(0x00);
	if (Wire.endTransmission(1) != 0) {
		delayMicroseconds(450);
		Wire.beginTransmission(_addr);
		Wire.write(0x07);
		Wire.write(0x00);
		Wire.endTransmission(1);
	}
	Wire.beginTransmission(_addr);
	Wire.write(0x00);
	Wire.endTransmission(0);
	Wire.requestFrom((int) _addr, 1, 1);
	if (Wire.read() != 0xA0) {
		if (state.serial) {
			Serial.println(F("Failed to Communicate with BNO055"));
			Serial.println(F("All Further Attempted Interaction With BNO055 results in undefined behavior!!!!!!!!!"));
		}
		return nullptr;
	}
	else {
		_page = 0x00;
	}

	//Set Normal Power Mode
	registerWrite(0x00, 0x3E, 0x00);
	
	//enter config mode (should be default but just in case...)
	registerWrite(0x00, 0x3D, CONFIG_MODE);

	//self test
	if (state.serial) {
		Serial.println(F("Performing BNO055 Self Test. May Take Up to 5 Seconds. Please Wait...."));
	}
	registerWrite(0x00, 0x3F, 0x01);
	time = 0;
	timeStart = millis();
	while(registerRead(0x00, 0x36) != 0x0F && time < 5000) {
		time = millis() - timeStart;
		delay(100);
	}
	if ((selfTest = registerRead(0x00, 0x36)) != 0x0F) {
		if (state.serial) {
			Serial.print(F("BNO055 FAILED Self Test With Error Code: "));
			Serial.println(selfTest, HEX);
		}
		return nullptr;
	}
	else {
		if (state.serial) {
			Serial.println(F("BNO055 Self Test Passed"));
		}
	}

	//set units to celsius for temp, degrees for angles, m/s^2 for acceleration
	registerWrite(0x00, 0x3B, 0x00);

	//disable interrupts (since interrupt pin isn't connected)
	registerWrite(0x01, 0x10, 0x00);
	registerWrite(0x01, 0x0F, 0x00);

	//Set axes
	registerWrite(0x00, 0x42, 0x02);
	registerWrite(0x00, 0x41, 0x21);

	return this;
}


/**************************
 * IMU Read From Register *
 **************************/
uint8_t IMU::registerRead(uint8_t page, uint8_t reg) {
	if (_page != page) {
		Wire.beginTransmission(_addr);
		Wire.write(0x07);
		Wire.write((int) page);
		if (Wire.endTransmission(1) != 0) {
			IMU::_errorHandler();
		}
		else {
		_page = page;
		}
	}

	Wire.beginTransmission(_addr);
	Wire.write((int) reg);
	if (Wire.endTransmission(0) != 0) {
		IMU::_errorHandler();
	}

	if (Wire.requestFrom((int) _addr, 1, 1)) {
	}
	else {
		IMU::_errorHandler();
	}
	return (uint8_t) Wire.read();
}

/*************************
 * IMU Write To Register *
 *************************/
void IMU::registerWrite(uint8_t page, uint8_t reg, uint8_t value) {
	if (IMU::_page != page) {
                Wire.beginTransmission(_addr);
                Wire.write(0x07);
                Wire.write((int) page);
               if (Wire.endTransmission(1) != 0) {
		       IMU::_errorHandler();
	       }
	       else {
                _page = page;
	       }
        }
	
	Wire.beginTransmission(_addr);
	Wire.write((int) reg);
	Wire.write((int) value);
	if (Wire.endTransmission(1) != 0) {
		IMU::_errorHandler();
	}
}

/*****************
 * IMU Calibrate *
 *****************/
boolean IMU::calibrate() {
	//Enter NDOF Mode
	uint8_t mode = registerRead(0x00, 0x3D);
	registerWrite(0x00, 0x3D, 0x0C);

	unsigned long started = millis();
	unsigned long time = 0;
	uint8_t status = 0;

	if (state.serial) {
		Serial.println(F("IMU Calibration Started."));
		Serial.println(F("To calibrate gyroscope set device on solid surface and ensure no movement."));
	}

	while (status < 3 && time < 60000) {
		status = (registerRead(0x00, 0x35) >> 4) & 0x03;
		time = millis() - started;
	}

	if (status != 3) {
		if (state.serial) {
			Serial.println(F("IMU ERROR: Gyroscope Calibration Failed!"));
			Serial.println(F("IMU ERROR: Calibration Failed!"));
		}
		registerWrite(0x00, 0x3D, mode);
		return false;
	}

	if (state.serial) {
		Serial.println(F("Gyroscope Calibrated!"));
		Serial.println(F("To calibrate accelerometer place device in up to 6 different stable positions, pausing for a few seconds in each position, with slow movement between each position. Ensure the device is laying at least once perpindicular to x, y, and z axis."));
	}

	status = 0;
	time = 0;
	started = millis();
	while (status < 3 && time < 60000) {
		status = (registerRead(0x00, 0x35) >> 2) & 0x03;
		time = millis() - started;
	}

	if (status != 3) {
		if (state.serial) {
			Serial.println(F("IMU ERROR: Accelerometer Calibration Failed!"));
			Serial.println(F("IMU ERROR: Calibration Failed!"));
		}
		registerWrite(0x00, 0x3D, mode);
		return false;
	}

	if (state.serial) {
		Serial.println(F("Accelerometer Calibrated!"));
		Serial.println(F("To calibrate magnetometer, make some random movements with device"));
	}

	status = 0;
	time = 0;
	started = millis();
	while (status < 3 && time < 60000) {
		status = registerRead(0x00, 0x35) & 0x03;
		time = millis() - started;
	}

	if (status != 3) {
		if (state.serial) {
			Serial.println(F("IMU ERROR: Magnetometer calibration failed!"));
			Serial.println(F("IMU ERROR: Calibration Failed!"));
		}
		registerWrite(0x00, 0x3D, mode);
		return false;
	}

	if (state.serial) {
		Serial.println(F("Magnetometer Calibrated!"));
	}

	delay(500);

	status = registerRead(0x00, 0x35) <<2;

	if (status == 0xFC) {
		if (state.serial) {
			Serial.println(F("IMU Calibrated!"));
		}
		registerWrite(0x00, 0x3D, mode);
		return true;
	}
	else {
		if (state.serial) {
			Serial.println(F("IMU ERROR: Calibration Failed!"));
		}
		registerWrite(0x00, 0x3D, mode);
		return false;
	}
}

/**********************************
 * IMU Save Calibration To EEPROM *
 **********************************/
void IMU::saveCalibration() {
	calibrationData calibration;
	Wire.beginTransmission(_addr);
        Wire.write(0x07);
        Wire.write(0x01);
        if (Wire.endTransmission(1) != 0) {
                IMU::_errorHandler();
        }
        else {
                _page = 0x01;
        }
        Wire.beginTransmission(_addr);
        Wire.write(0x50);
        if (Wire.endTransmission(0) != 0) {
                IMU::_errorHandler();
        }
        if (Wire.requestFrom((int) _addr, 16, 1)) {
		uint8_t id[16];
		for (int i = 15; i >= 0; i--) {
			id[i] = Wire.read();
		}
		if (state.serial) {
			Serial.println(F("Saving calibration information to EEPROM. Please Wait."));
		}
		for (int i = 0; i < 16; i++) {
			EEPROM.write((i * sizeof(uint8_t)), id[i]);
		}
		calibration = getCalibration();
		EEPROM.put((16 * sizeof(uint8_t)), calibration);
		if (state.serial) {
			Serial.println(F("Calibration Data Saved to EEPROM."));
		}
	}
	else {
		IMU::_errorHandler();
	}
}

/****************
 * Clear EEPROM *
 ****************/
void IMU::eepromClear() {
	if (state.serial) {
		Serial.println(F("Clearing EEPROM. Please Wait."));
	}
	for (unsigned int i = 0; i < EEPROM.length(); i++) {
		EEPROM.write(i, 0);
	}
	if (state.serial) {
		Serial.println(F("EEPROM Cleared."));
	}
}

/***************************************
 * IMU Restore Calibration From EEPROM *
 ***************************************/
boolean IMU::restoreCalibration() {
	uint8_t mode;
	uint8_t id[16];
	
	mode = registerRead(0x00, 0x3D);
	registerWrite(0x00, 0x3D, CONFIG_MODE); 

	for(int i = 0; i < 16; i ++) {
		id[i] = EEPROM.read((i * sizeof(uint8_t)));
	}
	Wire.beginTransmission(_addr);
        Wire.write(0x07);
        Wire.write(0x01);
        if (Wire.endTransmission(1) != 0) {
                IMU::_errorHandler();
        }
        else {
                _page = 0x01;
        }
        Wire.beginTransmission(_addr);
        Wire.write(0x50);
        if (Wire.endTransmission(0) != 0) {
                IMU::_errorHandler();
        }
        if (Wire.requestFrom((int) _addr, 16, 1)) {
        	for (int i = 15; i >= 0; i--) {
                	if (id[i] != Wire.read()) {
				return false;
			}
		}
	}
	else {
		IMU::_errorHandler();
	}

	uint8_t msb;
	uint8_t lsb;
	calibrationData calibration;
	uint16_t* cal = (uint16_t*) &calibration;
	uint8_t startAddr = 0x6A;

	EEPROM.get((16 * sizeof(uint8_t)), calibration);
	Wire.beginTransmission(_addr);
	Wire.write(0x07);
	Wire.write(0x00);
	if (Wire.endTransmission(1) != 0) {
		IMU::_errorHandler();
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
		if (Wire.endTransmission(1) != 0) {
			IMU::_errorHandler();
		}
		Wire.beginTransmission(_addr);
		Wire.write(startAddr--);
		Wire.write(lsb);
		if (Wire.endTransmission(1) != 0) {
			IMU::_errorHandler();
		}
	}
	registerWrite(0x00, 0x3D, mode);
	return true;
}

/****************************
 * IMU Get Calibration Data *
 ****************************/
IMU::calibrationData IMU::getCalibration() {
	calibrationData calibration;
	uint8_t lsb;
        uint16_t msb;
	uint8_t i;
	uint16_t* startAddr = (uint16_t*) &calibration;

        Wire.beginTransmission(_addr);
        Wire.write(0x07);
        Wire.write(0x00);
        if (Wire.endTransmission(1) != 0) {
		IMU::_errorHandler();
	}
	else {
		_page = 0x00;
	}
	Wire.beginTransmission(_addr);
        Wire.write(0x55);
        if (Wire.endTransmission(0) != 0) {
		IMU::_errorHandler();
	}
        if (Wire.requestFrom((int) _addr, 22, 0)) {
		for (i = 0; i < 22; i++) {
			lsb = Wire.read();
			msb = (Wire.read()) << 8;
			*(startAddr + i) = msb | lsb;
		}
        }
	else {
		IMU::_errorHandler();
	}
	return calibration;
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
void IMU::dat::update(uint8_t dataType) {
	uint8_t lsb;
        uint8_t msb;
	int16_t buffer;
	
        Wire.beginTransmission(_addr);
        Wire.write(0x07);
        Wire.write(0x00);
        if (Wire.endTransmission(1) != 0) {
		IMU::_errorHandler();
	}
	else {
		_page = 0x00;
	}


	switch (dataType) {
		case ACCELEROMETER:
			Wire.beginTransmission(_addr);
			Wire.write(0x08);
			if (Wire.endTransmission(0) != 0) {
				IMU::_errorHandler();
			}
			if (Wire.requestFrom((int) _addr, 6, 0)) {
				for (int i = 0; i < 3; i++) {
					lsb = Wire.read();
					msb = Wire.read();
					buffer = ((int16_t) lsb) | (((int16_t) msb) << 8);
					*( ( (double*) &(accelerometer.x) ) + i) = ((double) buffer)/ 100.0;
				}
			}
			else {
				IMU::_errorHandler();
			}
			break;
		case MAGNETOMETER:
			Wire.beginTransmission(_addr);
			Wire.write(0x0E);
			if (Wire.endTransmission(0) != 0) {
				IMU::_errorHandler();
			}
			if (Wire.requestFrom((int) _addr, 6, 0)) {
				for (int i = 0; i < 3; i++) {
					lsb = Wire.read();
					msb = Wire.read();
					buffer = ((int16_t) lsb) | (((int16_t) msb) << 8);
					*( ( (double*) &(magnetometer.x) ) + i) = ((double) buffer)/ 16.0;
				}
			}
			else {
				IMU::_errorHandler();
			}
			break;
		case GYROSCOPE:
			Wire.beginTransmission(_addr);
			Wire.write(0x14);
			if (Wire.endTransmission(0) != 0) {
				IMU::_errorHandler();
			}
			if (Wire.requestFrom((int) _addr, 6, 0)) {
				for (int i = 0; i < 3; i++) {
					lsb = Wire.read();
					msb = Wire.read();
					buffer = ((int16_t) lsb) | (((int16_t) msb) << 8);
					*( ( (double*) &(gyroscope.x) ) + i) = ((double) buffer)/ 16.0;
				}
			}
			else {
				IMU::_errorHandler();
			}
			break;
		case EULER_ANGLES:
			Wire.beginTransmission(_addr);
			Wire.write(0x1A);
			if (Wire.endTransmission(0) != 0) {
				IMU::_errorHandler();
			}
			if (Wire.requestFrom((int) _addr, 6, 0)) {
				for (int i = 0; i < 3; i++) {
					lsb = Wire.read();
					msb = Wire.read();
					buffer = ((int16_t) lsb) | (((int16_t) msb) << 8);
					*( ( (double*) &(eulerData.yaw) ) + i) = ((double) buffer)/ 16.0;
				}
			}
			else {
				IMU::_errorHandler();
			}
			break;
		case QUATERNION:
			Wire.beginTransmission(_addr);
			Wire.write(0x20);
			if (Wire.endTransmission(0) != 0) {
				IMU::_errorHandler();
			}
			if (Wire.requestFrom((int) _addr, 6, 0)) {
				for (int i = 0; i < 3; i++) {
					lsb = Wire.read();
					msb = Wire.read();
					buffer = ((int16_t) lsb) | (((int16_t) msb) << 8);
					*( ( (double*) &(quaternionData.w) ) + i) = ((double) buffer)/ 16384.0;
				}
			}
			else {
				IMU::_errorHandler();
			}
			break;
		case LINEAR_ACCELERATION:
			Wire.beginTransmission(_addr);
			Wire.write(0x28);
			if (Wire.endTransmission(0) != 0) {
				IMU::_errorHandler();
			}
			if (Wire.requestFrom((int) _addr, 6, 0)) {
				for (int i = 0; i < 3; i++) {
					lsb = Wire.read();
					msb = Wire.read();
					buffer = ((int16_t) lsb) | (((int16_t) msb) << 8);
					*( ( (double*) &(linearAcceleration.x) ) + i) = ((double) buffer)/ 100.0;
				}
			}
			else {
				IMU::_errorHandler();
			}
			break;
		case GRAVITY_VECTOR:
			Wire.beginTransmission(_addr);
			Wire.write(0x2E);
			if (Wire.endTransmission(0) != 0) {
				IMU::_errorHandler();
			}
			if (Wire.requestFrom((int) _addr, 6, 0)) {
				for (int i = 0; i < 3; i++) {
					lsb = Wire.read();
					msb = Wire.read();
					buffer = ((int16_t) lsb) | (((int16_t) msb) << 8);
					*( ( (double*) &(gravityVector.x) ) + i) = ((double) buffer)/ 100.0;
				}
			}
			else {
				IMU::_errorHandler();
			}
			break;
		case TEMPERATURE:
			Wire.beginTransmission(_addr);
			Wire.write(0x34);
			if (Wire.endTransmission(0) != 0) {
				IMU::_errorHandler();
			}
			if (Wire.requestFrom((int) _addr, 1, 0)) {
				buffer = (int16_t) Wire.read();
				temperature = (double) buffer;
			}
			else {
				IMU::_errorHandler();
			}
			break;
	}
}

/***************************
 * IMU Communication Error *
 ***************************/
void IMU::_errorHandler() {
	*imuFaultPointer = true;
	if (state.serial) {
		Serial.println(F("Error Communicating With BNO055 IMU"));
	}
}

/******************
 * IMU Destructor *
 ******************/
IMU::~IMU() {
	Wire.end();
}
