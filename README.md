# Motor_IMU_Shield

This is an Arduino library for an Arduino "Shield" designed for the Arduino Uno (Should be Widely Compatibile with other Arduinos i.e. Mega) which provides 4 Brushless DC Motor Drivers and an integrated Inertial Measurement Unit (IMU). The Motor Controllers are Allegro MicroSystems A4963's and the IMU is a Bosch Sensortec BNO055. The PCB directory contains a PDF Schematic for the Shield as well as a KiCad Project for the Shield and the Gerber files for the Shield.

## Using The Library

### Library Functions

*
```cpp
Motor motorName(int motorTerminal, [float maxCurrent], [int numPoles], [int maxSpeed])
```

Create a Motor object called motorName.

motorTerminal is the only required argument and can be MOTOR_A, MOTOR_B, MOTOR_C, or MOTOR_D corresponding to which terminal the motor is plugged into on the shield.

maxCurrent is an optional floating point value to specify the max current of the motor in amps. Defaults to maximum value of 20 amps if ommitted, any value greater than 20 amps defaults to 20 amps, and any value less than 2.5 amps defaults to 2.5 amps.

numPoles is an optional integer value giving the number of pole pairs in the motor. This is only necessary for calculating a value for the maximum speed of the motor and is only relevent in Closed Loop Speed Mode.

maxSpeed is an optional integer value representing the maximum speed of the motor in RPM. It is only relevent in Closed Loop Speed Mode.

*
```cpp
motorName.setMode(int controlMode)
```

Sets Operating Mode For The Motor.

Returns a boolean true if successful or false if an error occurred.

Takes an argument of INDIRECT_SPEED (default value), DIRECT_SPEED, CLOSED_LOOP_CURRENT, or CLOSED_LOOP_SPEED corresponding to the desired operating mode.

*
```cpp
motorName.setDirection(int dir)
```

Sets Rotation Direction for the motor.

Returns a boolean true if successful or false if an error occurred.

Takes an argument of FORWARD or REVERSE.

*
```cpp
motorName.setSpeed(int speed)
```

Sets the Rotational speed of the motor. In non-closed-loop modes corresponds to PWM duty cycle. In closed loop modes corresponds to a percent of max speed or max current. Also disables motor brake and sets motor to run.

Returns a boolean true if successful or false if an error occurred.

Takes an integer corresponding to either PWM duty cylce (in percent) or percent of max speed or max current.

*
```cpp
motorName.getSpeed()
```

Returns an integer representing the current speed setting of the motor.

Does not take an argument.

*
```cpp
motorName.coast()
```

Disables motor outputs and coasts motor.

Returns boolean true if successful or false is an error occurred.

Does not take an argument.

NOTE: Motor can be resumed at previous speed setting by calling motorName.restart() or with a different speed using motorName.setSpeed(int speed).

*
```cpp
motorName.restart()
```

Restarts motor at previous speed setting following motorName.coast().

Returns boolean true if successful or false if an error occurred.

Does not take an argument.

*
```cpp
motorName.faultCheck();
```

Returns boolean true if a motor fault has been detected or false if no motor faults detected.

Does not take an argument.

### Advanced Library Functions

NOTE: No error checking is performed in these functions. It is recommended that you call motorName.faultCheck() after a registerRead or registerWrite and implement some form of error handling.

*
```cpp
motorName.registerRead(int addr)
```

Read from one of the A4963 motor controller registers.

Returns the register value as an int.

Registers are defined as A4963_CONF0, A4963_CONF1, A4963_CONF2, A4963_CONF3, A4963_CONF4, A4963_CONF5, A4963_FAULT, and A4963_RUN.

See A4963 datasheet for more detail on registers.

*
```cpp
motorName.registerWrite(int addr, int data)
```

Write to one of the A4963 motor controller registers.

No return value.

Register addresses defined as in registerRead, data is the data to be written to the register.
See A4963 datasheet for more detail on registers.
