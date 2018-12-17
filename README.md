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
    Initializes Motor/IMU Shield. `<shield>` is the name of a Motor_IMU_Shield instance. See [Library Variables](#library-variables) for information on Motor_IMU_Shield instances.
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
    Attaches a motor instance to a motor terminal on the shield. `<motor>` is the name of a motor instance. See [Library Variables](#library-variables) for information on motor instances.

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
    Sets the operating mode for a motor instance. `<motor>` is the name of a motor instance. See [Library Variables](#library-variables) for information on motor instances.
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
    Sets Rotation Direction for the motor. `<motor>` is the name of a motor instance. See [Library Variables](#library-variables) for information on motor instances.
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
    Sets the Rotational speed of a motor instance. In non-closed-loop modes corresponds to PWM duty cycle. In closed loop modes corresponds to a percent of max speed or max current. Also disables motor brake and sets motor to run. `<motor>` is the name of a motor instance. See [Library Variables](#library-variables) for information on motor instances.
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
    Gets the current speed setting of a motor instance. `<motor>` is the name of a motor instance. See [Library Variables](#library-variables) for information on motor instances.
    * Returns: an integer representing either the PWM duty cycle or percent of max speed or max current. (depends on operating mode)
  <br/>
  <br/>
  
  * ```cpp
    <motor>.coast()
    ```
    Disables motor outputs and coasts motor. `<motor>` is the name of a motor instance. See [Library Variables](#library-variables) for information on motor instances.
NOTE: Motor can be resumed at previous speed setting by calling `<motor>.restart()` or with a different speed using `<motor>.setSpeed(int speed)`.
  <br/>
  <br/>
  
  * ```cpp
    <motor>.restart()
    ```
    Restarts motor at previous speed setting following `<motor>.coast()`. `<motor>` is the name of a motor instance. See [Library Variables](#library-variables) for information on motor instances.
  <br/>
  <br/>

  * ```cpp
    <motor>.faultCheck();
    ```
    Checks for motor controller faults. `<motor>` is the name of a motor instance. See [Library Variables](#library-variables) for information on motor instances.
      * Returns: a boolean true if a motor fault has been detected or false if no motor faults detected.
  <br/>
  <br/>
  
  * ```cpp
    <imu>.attach()
    ```
    Initializes BNO055 IMU. `<imu>` is the name of the IMU instance. See [Library Variables](#library-variables) for information on IMU instances.
  <br/>
  <br/>
  
  * ```cpp
    <imu>.calibrate()
    ```
    Performs IMU calibration. `<imu>` is the name of the IMU instance. See [Library Variables](#library-variables) for information on IMU instances. It is highly suggested that you have serial communication enabled if performing calibration, as the calibration routine prints instructions and status information over serial.
      * Returns
          A boolean true if calibration was successful or false if calibration failed.
  <br/>
  <br/>
  
  * ```cpp
    <imu>.saveCalibration()
    ```
    Saves calibration data to EEPROM. `<imu>` is the name of the IMU instance. See [Library Variables](#library-variables) for information on IMU instances. 
    NOTE: EEPROM has a limited number of writes so this should not be done frequently.
  <br/>
  <br/>
  
  * ```cpp
    <imu>.eepromClear()
    ```
    Clears EEPROM by writing 0's. `<imu>` is the name of the IMU instance. See [Library Variables](#library-variables) for information on IMU instances.
    NOTE: EEPROM has a limited number of writes so this should not be done frequently.
  <br/>
  <br/>
  
  * ```cpp
    <imu>.restoreCalibration()
    ```
    Restores calibration settings previously stored in EEPROM. `<imu>` is the name of the IMU instance. See [Library Variables](#library-variables) for information on IMU instances.
      * Returns: a boolean true if successful in restoring calibration, or false if failed to restore calibration.
  <br/>
  <br/>
  
  * ```cpp
    <imu>.setMode(int mode)
    ```
    Sets operating mode for BNO055 IMU. `<imu>` is the name of the IMU instance. See [Library Variables](#library-variables) for information on IMU instances.
      * Arguments
        * ```cpp
          int mode
          ```
          The desired operating mode. Given by one of the following predefined constants:
            * CONFIG_MODE
            * ACC_ONLY_MODE
            * MAG_ONLY_MODE
            * GYRO_ONLY_MODE
            * ACC_MAG_MODE
            * ACC_GYRO_MODE
            * MAG_GYRO_MODE
            * AMG_MODE
            * IMU_MODE
            * COMPASS_MODE
            * M4G_MODE
            * NDOF_FMC_OFF_MODE
            * NDOF_MODE
  <br/>
  <br/>
  
  * ```cpp
    <imu>.data.update(int dataType)
    ```
    Updates IMU data. Will only store one type of data at a time. If you want multiple types of data store data in a seperate variable in between calls to `update()`. Updated data is stored in `<imu>.data.<dataType>`, see [Library Variables](#library-variables) for more info. Referring to a different data type than the most recent updata data type is undefined. `<imu>` is the name of the IMU instance. See [Library Variables](#library-variables) for information on IMU instances.
      * Arguments
        * ```cpp
          int dataType
          ```
          The dataType to update. Given by one of the following predefined constants:
            * ACCELEROMETER
            * MAGNETOMETER
            * GYROSCOPE
            * EULER_ANGLES
            * QUATERNION
            * LINEAR_ACCELERATION
            * GRAVITY_VECTOR
            * TEMPERATURE
  <br/>
  <br/>
  
### Library Variables

  * ```cpp
    Motor_IMU_Shield shieldName;
    ```
    A variable representing an instance of the shield. Must be initialized with `shieldName.begin(int imu_address, long serial_baud_rate)` before using. Should also be initialized before initializing IMU or Motor instances.
  <br/>
  <br/>
  
  * ```cpp
    Motor motorName;
    ```
    A variable representing a motor instance. Must be attached to a motor terminal using `motorName.attach(int motorTerminal, [float maxCurrent], [int numPoles], [long maxSpeed])` before using.
  <br/>
  <br/>
  
  * ```cpp
    IMU imuName;
    ```
    A variable representing an IMU instance. Must be initialized using `imuName.attach()` before using.
  <br/>
  <br/>
  
  * ```cpp
    imuName.data.<dataType>
    ```
    IMU data. Must call `imuName.data.update(int dataType)` before accessing. Only the data type corresponding to what was specified with `imuName.data.update(int dataType)` is valid:
      * ```cpp
        imuName.data.update(ACCELEROMETER);
        ```
          * Valid Data is:
            ```cpp
            vector imuName.data.accelerometer;
            ```
      * ```cpp
        imuName.data.update(MAGNETOMETER);
        ```
          * Valid Data is:
            ```cpp
            vector imuName.data.magnetometer;
            ```
      * ```cpp
        imuName.data.update(GYROSCOPE);
        ```
          * Valid Data is:
            ```cpp
            vector imuName.data.gyroscope;
            ```
      * ```cpp
        imuName.data.update(EULER_ANGLES);
        ```
          * Valid Data is:
            ```cpp
            euler imuName.data.eulerData;
      * ```cpp
        imuName.data.update(QUATERNION);
        ```
          * Valid Data is:
            ```cpp
            quaternion imuName.data.quaternionData;
            ```
      * ```cpp
        imuName.data.update(LINEAR_ACCELERATION);
        ```
          * Valid Data is:
            ```cpp
            vector imuName.data.linearAcceleration;
            ```
      * ```cpp
        imuName.data.update(GRAVITY_VECTOR);
        ```
          * Valid Data is:
            ```cpp
            vector imuName.data.gravityVector;
            ```
      * ```cpp
        imuName.data.update(TEMPERATURE);
        ```
          * Valid Data is:
            ```cpp
            double imuName.data.temperature;
            ```
  <br/>
  <br/>
  
  * ```cpp
    vector vectorName;
    ```
    A vector struct containing:
      * ```cpp
        double vectorName.x;
        ```
      * ```cpp
        double vectorName.y;
        ```
      * ```cpp
        double vectorName.z;
        ```
  <br/>
  <br/>
  
  * ```cpp
    euler eulerName;
    ```
    A struct of euler angles containing:
      * ```cpp
        double yaw;
        ```
      * ```cpp
        double roll;
        ```
      * ```cpp
        double pitch;
        ```
  <br/>
  <br/>
  
  * ```cpp
    quaternion quaternionName;
    ```
    A quaternion struct containing:
      * ```cpp
        double quaternionName.w;
        ```
      * ```cpp
        double quaternionName.x;
        ```
      * ```cpp
        double quaternionName.y;
        ```
      * ``cpp
        double quaternionName.z;
        ```
        
