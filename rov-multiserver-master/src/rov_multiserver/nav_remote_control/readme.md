# Navigation Remote Control Protocol

Navigation RC is a small protocol running over Insytech Communication Bus. 
Remote devices can use this protocol in order to execute navigation orders.

Most command arguments have a close match with mavlink commands and messages
definitions:

* [Dialect: ArduPilotMega](https://mavlink.io/en/messages/ardupilotmega.html)
* [MAVLINK Common Message Set](https://mavlink.io/en/messages/common.html)

## Common Definitions

### Topics

Main topic for triggering command actions:

* Topic: **/vehicle/navigation/rc/**
* Keys:
  * cmd: Command id string identifier
  * args (optional): List of command arguments

Topic for delivering human-readable feedback in response to incoming commands:

* Topic: **/vehicle/navigation_subsystem/feedback/**
* Keys:
  * msg: Human-readable string response

## Available Remote Orders

### set_preflight_mode
* **Definition:** Put vehicle in preflight mode

### set_manual_mode
* **Definition:** Put vehicle in manual mode 

### set_armed_mode
* **Definition:** Put vehicle motors in armed mode

### set_disarmed_mode
* **Definition:** Disarming vehicle motors

### start_accel_calib
* **Definition:** Start accelerometer calibration routine in vehicle
* **Afterwards:** Routine completion requires delivering a specific set of 
                  vehicle positions (see **set_vehicle_position**).

### set_vehicle_position
* **Definition:** Defines current vehicle position for accelerometer calibration
                  usage.
* **Arg0 vehicle_position_id:** Position integer id in range 1 to 6 according to available 
                      rov positions. After last position routine calibration routine
                      is ended.
  * Available rov positions: 
    ```
    LEVEL,
    LEFT,
    RIGHT,
    NOSE_DOWN,
    NOSE_UP,
    BACK;
    ```

### start_magnetometer_calibration
* **Definition:** Starts magnetometer calibration routine

### stop_magnetometer_calibration
* **Definition:** Stops magnetometer calibration routine

### start_motor_test
* **Definition:** Start independent motor test
* **Arg0 motor_id:** Motor identification index starting at 0
* **Arg1 motor_throttle:** Desired motor throttle in 0-100 range where 0 is full
reverse 50 is idle and 100 is full forward.

### stop_motor_test
* **Definition:** Stops motor test

### set_motor_direction
* **Definition:** Setting motor rotation direction without changing wires
* **Arg0 motor_id:** Motor identification index starting at 0
* **Arg1 direction_id:** 1 for forward movement, -1 for reverse direction

### set_board_orientation
* **Definition:** Defines current autopilot board orientation relative to vehicle
* **Arg0 board_orientation_id:** Orientation integer id according to following list 
of available board orientations.
  * Avialable board orientations:
  ```
  NONE = 0,
  YAW180 = 4,
  ROLL180 = 8,
  PITCH180 = 12,
  ```

### reboot_autopilot
* **Definition:** Force an autopilot reboot.


