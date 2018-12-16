# Motor_IMU_Shield

This is an Arduino library for an Arduino "Shield" designed for the Arduino Uno (Should be Widely Compatibile with other Arduinos i.e. Mega) which provides 4 Brushless DC Motor Drivers and an integrated Inertial Measurement Unit (IMU). The Motor Controllers are Allegro MicroSystems A4963's and the IMU is a Bosch Sensortec BNO055. PCB files are available at [https://github.com/lazerj7/Motor_IMU_Shield_PCB](https://github.com/lazerj7/Motor_IMU_Shield_PCB).

## Contents

1. [Installing The Library](#installing-the-library)
   * [Linux](#linux)
     * [Command Line](#command-line)
     * [Download Zip](#download-zip)
   * [Mac](#mac)
   * [Windows](#windows)
1. [Using The Library](#using-the-library)
   * [Library Functions](#library-functions)
   * [Library Variables](#library-variables)

## Installing The Library

### Linux

#### Command Line

1. Locate your Sketchbook Directory and the libraries Directory Inside
   * Usually your sketchbook is located in your home directory and called either Arduino or Sketchbook.
  
   * To find you sketchbook open the Arduino IDE, click File, then Preferences. On the Settings tab you should see your Sketchbook location.
  
   * Your sketchbook directory should have a libraries Directory inside. If not, make one. Note the path to the libraries Directory, i.e.:
     ```
     ~/Arduino/libraries
     ```
     or
     ```
     /home/<user name>/Arduino/libraries
     ```
  
1. Navigate to you libraries directory in a terminal. i.e.:

   ```
   cd ~/Arduino/libraries
   ```

1. Clone the repository:

   ```
   git clone https://github.com/lazerj7/Motor_IMU_Shield
   ```

1. That's it! You should now have a directory callled Motor_IMU_Shield in your libraries directory with all the library files inside.

#### Download Zip

1. Locate your Sketchbook Directory and the libraries Directory Inside
   * Usually your sketchbook is located in your home directory and called either Arduino or Sketchbook.
  
   * To find you sketchbook open the Arduino IDE, click File, then Preferences. On the Settings tab you should see your Sketchbook location.
  
   * Your sketchbook directory should have a libraries Directory inside. If not, make one. Note the path to the libraries Directory.
  
1. In the upper right of the repository screen on GitHub, click 'Clone or Download', and then click 'Download ZIP'

1. Extract the .zip file into your libraries Directory.

1. Done!

### Mac

1. Locate your Sketchbook Directory and the libraries Directory Inside

   * Usually your sketchbook is located in your Documents directory and called Arduino. It should have a libraries Directory inside. If not, make one.

   * To find you sketchbook open the Arduino IDE, click File, then Preferences. On the Settings tab you should see your Sketchbook location.
  
   * Your sketchbook directory should have a libraries Directory inside. If not, make one. Note the path to the libraries Directory.
  
1. In the upper right of the repository screen on GitHub, click 'Clone or Download', and then click 'Download ZIP'

1. Extract the .zip file into your libraries Directory.

1. Done!

### Windows

1. Locate your Sketchbook Directory and the libraries Directory Inside

   * Usually your sketchbook is located in your Documents directory and called Arduino. It should have a libraries Directory inside. If not, make one.

   * To find you sketchbook open the Arduino IDE, click File, then Preferences. On the Settings tab you should see your Sketchbook location.
  
   * Your sketchbook directory should have a libraries Directory inside. If not, make one. Note the path to the libraries Directory.
  
1. In the upper right of the repository screen on GitHub, click 'Clone or Download', and then click 'Download ZIP'

1. Extract the .zip file into your libraries Directory.

1. Done!

## Using The Library

### Library Functions

  * ```cpp
    <shield>.begin(int imu_address, long serial_baud_rate);
    ```
    Initializes Motor/IMU Shield. <shield> is the name of a Motor_IMU_Shield instance. See [Library Variables](#library-variables) for information on Motor_IMU_Shield instances.
      * Arguments
        * ```cpp
          int imu_address
          ```
          The address of the BNO055 IMU. Determined by the address jumper on the board. Can be given using one of the pre-defined constants:
            * JUMPERED
            * NOT_JUMPERED
            * CLOSED
            * OPEN
        * ```cpp
          long serial_baud_rate
          ```
          The baud rate for USB serial communication. Set this to 0 to disable serial communication.
  <br/>
  <br/>
  
  * ```cpp
    <motor>.attach(int motorTerminal, [float maxCurrent], [int numPoles], [long maxSpeed])
    ```
    Attaches a motor instance to a motor terminal on the shield. <motor> is the name of a motor instance. See [Library Variables](#library-variables) for information on motor instances.

      * Arguments
        * ```cpp
          int motorTerminal
          ```
          The only required argument and can be MOTOR_A, MOTOR_B, MOTOR_C, or MOTOR_D corresponding to which terminal the motor is plugged into on the shield.

        * ```cpp
          float maxCurrent
          ```
          An optional floating point value to specify the max current of the motor in amps. Defaults to maximum value of 20 amps if ommitted, any value greater than 20 amps defaults to 20 amps, and any value less than 2.5 amps defaults to 2.5 amps.
        * ```cpp
          int numPoles
          ```
          An optional integer value giving the number of pole pairs in the motor. This is only necessary for calculating a value for the maximum speed of the motor and is only relevent in Closed Loop Speed Mode.
        * ```cpp
          long maxSpeed
          ```
          An optional long value representing the maximum speed of the motor in RPM. It is only relevent in Closed Loop Speed Mode.
<br/>
<br/>

  * ```cpp
    <motor>.setMode(int controlMode)
    ```
    Sets the operating mode for a motor instance. <motor> is the name of a motor instance. See [Library Variables](#library-variables) for information on motor instances.
    * Arguments
      * ```cpp
        int controlMode
        ```
        Specifies the conrol mode to set. Given using one of the predefined constants:
          * INDIRECT_SPEED (default value)
          * DIRECT_SPEED
          * CLOSED_LOOP_CURRENT
          * CLOSED_LOOP_SPEED
  <br/>
  <br/>
  
  * ```cpp
    <motor>.setDirection(int dir)
    ```
    Sets Rotation Direction for the motor. <motor> is the name of a motor instance. See [Library Variables](#library-variables) for information on motor instances.
    * Arguments
      * ```cpp
        int dir
        ```
        An integer representing the direction of rotation. Can be given by either of the constanst FORWARD or REVERSE.
  <br/>
  <br/>
  
  * ```cpp
    <motor>.setSpeed(int speed)
    ```
    Sets the Rotational speed of a motor instance. In non-closed-loop modes corresponds to PWM duty cycle. In closed loop modes corresponds to a percent of max speed or max current. Also disables motor brake and sets motor to run. <motor> is the name of a motor instance. See [Library Variables](#library-variables) for information on motor instances.
    * Arguments
      * ```cpp
        int speed
        ```
        An integer corresponding to either PWM duty cylce (in percent) or percent of max speed or max current.
   <br/>
   <br/>
    
  * ```cpp
    <motor>.getSpeed()
    ```
    Gets the current speed setting of a motor instance. <motor> is the name of a motor instance. See [Library Variables](#library-variables) for information on motor instances.
    * Returns
      An int representing either the PWM duty cycle or percent of max speed or max current. (depends on operating mode)
  <br/>
  <br/>
  
```cpp
motorName.coast()
```

Disables motor outputs and coasts motor.

Returns boolean true if successful or false is an error occurred.

Does not take an argument.

NOTE: Motor can be resumed at previous speed setting by calling motorName.restart() or with a different speed using motorName.setSpeed(int speed).

```cpp
motorName.restart()
```

Restarts motor at previous speed setting following motorName.coast().

Returns boolean true if successful or false if an error occurred.

Does not take an argument.

```cpp
motorName.faultCheck();
```

Returns boolean true if a motor fault has been detected or false if no motor faults detected.

Does not take an argument.

### Advanced Library Functions

NOTE: No error checking is performed in these functions. It is recommended that you call motorName.faultCheck() after a registerRead or registerWrite and implement some form of error handling.

```cpp
motorName.registerRead(int addr)
```

Read from one of the A4963 motor controller registers.

Returns the register value as an int.

Registers are defined as A4963_CONF0, A4963_CONF1, A4963_CONF2, A4963_CONF3, A4963_CONF4, A4963_CONF5, A4963_FAULT, and A4963_RUN.

See A4963 datasheet for more detail on registers.

```cpp
motorName.registerWrite(int addr, int data)
```

Write to one of the A4963 motor controller registers.

No return value.

Register addresses defined as in registerRead, data is the data to be written to the register.

See A4963 datasheet for more detail on registers.
