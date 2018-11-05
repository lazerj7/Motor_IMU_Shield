# Motor_IMU_Shield
This is an Arduino library for an Arduino "Shield" designed for the Arduino Uno (Should be Widely Compatibile with other Arduinos i.e. Mega) which provides 4 Brushless DC Motor Drivers and an integrated Inertial Measurement Unit (IMU). The Motor Controllers are Allegro MicroSystems A4963's and the IMU is a Bosch Sensortec BNO055. The PCB directory contains a PDF Schematic for the Shield as well as a KiCad Project for the Shield and the Gerber files for the Shield.
##Using The Library
###Library Functions
'''
Motor motorName(int motorTerminal, [float maxCurrent], [int numPoles], [int maxSpeed])
'''
Create a Motor object called motorName.
motorTerminal is the only required argument and can be MOTOR_A, MOTOR_B, MOTOR_C, or MOTOR_D corresponding to which terminal the motor is plugged into on the shield.
maxCurrent is an optional floating point value to specify the max current of the motor in amps. Defaults to maximum value of 20 amps if ommitted, any value greater than 20 amps defaults to 20 amps, and any value less than 2.5 amps defaults to 2.5 amps.
numPoles is an optional integer value giving the number of pole pairs in the motor. This is only necessary for calculating a value for the maximum speed of the motor and is only relevent in Closed Loop Speed Mode.
maxSpeed is an optional integer value representing the maximum speed of the motor in RPM. It is only relevent in Closed Loop Speed Mode.
