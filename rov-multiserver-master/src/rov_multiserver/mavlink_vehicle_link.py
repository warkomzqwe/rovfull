# -*- coding: utf-8 -*-
"""
Mavlink Vehicle Link module.

The purpose of this module is to implement the VehicleLink interface for
vehicles that use `MAVLink <https://mavlink.io/en/>`_ to communicate
(probably a vehicle using `ArduSub <https://www.ardusub.com/>`_).

Events
======

'vehicle-connected'
~~~~~~~~~~~~~~~~~~~
It is triggered when a connection to the vehicle has been established.

Callback input arguments
------------------------
uri : str
    String with the connection URI.

'vehicle-diconnected'
~~~~~~~~~~~~~~~~~~~~~
It is triggered when the connection to the vehicle has been lost.

'new-vehicle-message'
~~~~~~~~~~~~~~~~~~~~~
It is triggered every time a tracked and valid message from the vehicle
has been acquired.

Callback input arguments
------------------------
message : dict
    Received message as a python dict.
timestamp : int
    Timestamp of message reception.
"""
from time import time, sleep
from typing import Optional, Callable, Any, Union

from pymavlink import mavutil
from pymessaginglib.MessagingSystem import receiver
from pymessaginglib.Topic import TopicObserver, Topic, topic_factory
from rov_event_bus.bus import *
from rov_logger.logger import get_rov_logger

from rov_multiserver.arm_controller import ArmController
from rov_multiserver.full_axes_speed_smoother import SpeedSmootherLoop
from rov_multiserver.messaging_bus.outgoing_messages import send_mag_calibration_feedback, send_hud_notification, \
    report_vehicle_armed, send_nav_feedback_msg
from rov_multiserver.motor_test.motor_test_feedback import deliver_motor_test_feedback
from rov_multiserver.motor_test.motor_test_service import MotorTestService
from rov_multiserver.nav_remote_control.nav_remote_control_service import NavRemoteControlService
from rov_multiserver.pwm import PwmOutput
from rov_multiserver.servo_lock.servo_lock_service import ServoLockService
from rov_multiserver.speed_smoother_old import SpeedSmoother
from rov_multiserver.utils.debug import mavlink_dump
from rov_multiserver.vehicle_data import VehicleNavigationMode, board_rotations
from rov_multiserver.vehicle_link import VehicleLink

logger = get_rov_logger()

# Module defaults
DEFAULT_CAM_TILT_PWM_REVERSE = True
DEFAULT_CAM_TILT_PWM_MAX = 2200
DEFAULT_CAM_TILT_PWM_MIN = 500
DEFAULT_CAM_TILT_PWM_CENTER = 1350
USE_PIXHAWK_ELECTRICAL_MEASUREMENTS = True
DEFAULT_BATTERY_CURRENT_LIMIT = 28  # 28A max current with battery
DEFAULT_EXTERNAL_CURRENT_LIMIT = 80  # No current limit
DEFAULT_CURRENT_LIMIT = DEFAULT_BATTERY_CURRENT_LIMIT  # conservative default
CURRENT_LIMIT_SET_MIN_INTERVAL = 0.1  # 100 ms
USE_OLD_SPEED_SMOOTHER = False
USE_NEW_SPEED_SMOOTHER = True
# New SpeedSmoother configuration
SPEED_SMOOTHER_MIN_FREQ = 1.0
SPEED_SMOOTHER_MAX_FREQ = 10.0
SPEED_ACCEL_RATE = 25.0
SPEED_ACCEL_THRS = 90.0
SPEED_ACCEL_START_THRS = 50.0
SPEED_DECEL_RATE = 200.0
SPEED_DECEL_THRS = 20.0
SPEED_DECEL_START_THRS = 60


def get_last_instance():
    if MavlinkVehicleLink.last_instance is None:
        return MavlinkVehicleLink()
    else:
        return MavlinkVehicleLink.last_instance


class PowerSourceDetectorListener(TopicObserver):
    def __init__(self, new_power_source_callback: Callable[[str], None]):
        self._callback = new_power_source_callback

    def on_message_arrival(self, dic: dict, ip: str):
        if 'active_power_source' in dic:
            self._callback(dic['active_power_source'])


def skip_none(f):
    """
    Decorator to avoid checking for valid vehicle link everytime.

    Requirement:
    A class method like,

    def is_vehicle_com_none(self):
        pass

    Usage:
    @skip_none
    def some_method(self):
        pass  # do something
    """

    def _skip_none(*args):
        """
        If vehicle communication is non existent:
        """
        if args[0].is_vehicle_com_none():
            logger.warning("Vehicle communication is lost.")
            return

        else:
            return f(*args)

    return _skip_none


class MavlinkVehicleLink(VehicleLink):
    """
    Mavlink implementation of the VehicleLink class.

    TODO: Document.

    """

    _vehicle = None
    last_instance = None

    def __init__(self, timeout_threshold=3.0, degree_unit_range_fix=False,
                 *args, **kwargs):
        """Constructor Method."""
        self.__net_cleaning_mode_active = False
        self._curr_power_source = 'battery'
        self.__current_limit_now = None
        self.__current_limit: Optional[int] = DEFAULT_CURRENT_LIMIT
        self.__last_rc = [1500, 1500, 1500, 1500, 1500, 1500, 65535, 65535]
        self.__degree_unit_range_fix = degree_unit_range_fix
        self.__cam_tilt_pwm_reverse = DEFAULT_CAM_TILT_PWM_REVERSE
        self.__cam_tilt_pwm_max = DEFAULT_CAM_TILT_PWM_MAX
        self.__cam_tilt_pwm_min = DEFAULT_CAM_TILT_PWM_MIN
        self.__cam_tilt_pwm_center = DEFAULT_CAM_TILT_PWM_CENTER
        MavlinkVehicleLink.last_instance = self
        super(MavlinkVehicleLink, self).__init__(*args, **kwargs)
        self.__net_cleaning_mode_gain = \
            self.settings.net_cleaning_mode_initial_gain
        self._connected = False
        # DELETE ME: Store RC Values
        self.__last_value = {
            'voltage': None, 'current': None, 'rc_heave': None, 'rc_yaw': None,
            'rc_surge': None, 'rc_sway': None, 'pitch': None, 'roll': None,
            'yaw': None, 'pitchspeed': None, 'rollspeed': None,
            'yawspeed': None
        }
        self.__last_time_ = 0.0
        self.__MIN_INTERVAL = 0.2
        keys = list(self.__last_value)
        keys.insert(0, 'time')
        if USE_OLD_SPEED_SMOOTHER:
            self.vertical_smoother = SpeedSmoother(self._do_move_vertical)
            self.lateral_smoother = SpeedSmoother(self._do_move_lateral)
            self.horizontal_smoother = SpeedSmoother(self._do_move_horizontal)
            self.yaw_smoother = SpeedSmoother(self._do_turn_yaw)
            self.vertical_smoother.start()
            self.horizontal_smoother.start()
            self.lateral_smoother.start()
            self.yaw_smoother.start()
        if USE_NEW_SPEED_SMOOTHER:
            self.__movements = SpeedSmootherLoop(
                self._update_all_axes, SPEED_SMOOTHER_MIN_FREQ,
                SPEED_SMOOTHER_MAX_FREQ, self.acceleration_rate,
                self.acceleration_stop_threshold,
                self.acceleration_start_threshold, self.deceleration_rate,
                self.deceleration_stop_threshold,
                self.deceleration_start_threshold)
        self.__calculate_pwm_linear_coeffs()
        arm_pwm = MavlinkVehicleLinkPwmOutput(11, self)
        self.__arm_controller = ArmController(arm_pwm)
        self._current_limit_policy = {
            'external': DEFAULT_EXTERNAL_CURRENT_LIMIT,
            'battery': DEFAULT_BATTERY_CURRENT_LIMIT}
        self.__last_current_limit_set_time = 0.0

        # Internal functionality control of spamming axes
        self.update_all_axes = True

        # Starting servo locking service
        # TODO Move this service to main script and pass vehicle link as parameter.
        #      Also, add type hints to servo_lock_service.py (vehicle_com: MavlinkVehicleLink).
        #      Issue is outside from here i'm not sure how to check or wait for vehicle communication.
        self.servo_lock_service = ServoLockService(vehicle_com=self)

        # Starting Remote Control Service
        remote_control_service = NavRemoteControlService(vehicle_com=self)

        # Starting Motor Test Service
        self.motor_test_service = MotorTestService(vehicle_link=self)

    def is_vehicle_com_none(self):
        return self._vehicle is None

    def set_net_cleaning_mode(self, activate: bool):
        self.__net_cleaning_mode_active = activate
        speed = self.__net_cleaning_mode_gain if activate else 0.0
        self.move_vertical(speed * 100.0, False)

    def is_net_cleaning_mode_on(self) -> bool:
        return self.__net_cleaning_mode_active

    def set_net_cleaning_mode_gain(self, new_gain: Union[float, int]):
        self.__net_cleaning_mode_gain = new_gain
        if self.is_net_cleaning_mode_on():
            self.activate_net_cleaning_mode()  # update current speed
        event_bus.trigger(
            'new-user-notification',
            f"Net-cleaning mode gain: {new_gain * 100.0} %", 2, 2)

    def get_net_cleaning_mode_gain(self) -> float:
        return self.__net_cleaning_mode_gain

    def _on_setting_update_callback(self, setting_name: str,
                                    setting_value: Any):
        VehicleLink._on_setting_update_callback(self, setting_name,
                                                setting_value)
        if USE_NEW_SPEED_SMOOTHER:  # just if settings are enabled
            updated = True
            if setting_name == 'speed_smoother_acceleration_rate':
                self.__movements.acceleration_rate = setting_value
            elif setting_name == 'speed_smoother_acceleration_start_threshold':
                self.__movements.acceleration_start_threshold = setting_value
            elif setting_name == 'speed_smoother_acceleration_stop_threshold':
                self.__movements.acceleration_stop_threshold = setting_value
            elif setting_name == 'speed_smoother_deceleration_rate':
                self.__movements.deceleration_rate = setting_value
            elif setting_name == 'speed_smoother_deceleration_start_threshold':
                self.__movements.deceleration_start_threshold = setting_value
            elif setting_name == 'speed_smoother_deceleration_stop_threshold':
                self.__movements.deceleration_stop_threshold = setting_value
            else:
                updated = False
            if updated:
                logger.debug(f"Updated {setting_name} with {setting_value} on "
                             "SpeedSmootherLoop")
                return
        if setting_name == 'battery_current_limit':
            self._current_limit_policy['battery'] = setting_value
            if self._curr_power_source == 'battery':
                self.current_limit = setting_value
            if setting_value == 0:
                setting_value = 'no limit'  # Just for debugging
        elif setting_name == 'external_power_current_limit':
            self._current_limit_policy['external'] = setting_value
            if self._curr_power_source == 'external':
                self.current_limit = setting_value
            if setting_value == 0:
                setting_value = 'no limit'  # Just for debugging
        else:
            return
        logger.debug(f"Updated {setting_name}: {setting_value}")

    def input_gain_changed(self, value):
        """
        Changes the input gain.

        Parameters
        ----------
        value : int, float
            numeric value. It shoud be >= 0 and <= 100.

        """
        logger.info("Input Gain changed to %d%%", int(value * 100))
        self.state.input_gain = value
        value = int(round(value, 1) * 100)
        # self._send_blinking_text('GAIN: ' + str(value) + "%")

    def __initialize_vehicle(self):
        """
        Setups the vehicle for use.

        -   It should be called after establishing a connection with the
            vehicle.
        -   It sets the SERVOn_FUNCTION parameters of the 9nth and 10nth
            servos (lights and camera tilt) to 0 (disabled). This is
            necessary to use the RC override functions on this servos.
            This change will be stored on RAM and will be RESET on
            system reboot.

        """
        msg = "Initializing vehicle..."
        logger.info(msg)
        send_nav_feedback_msg(msg)
        # Configure Light Servo
        logger.debug("Setting light servo channel")
        self._vehicle.mav.param_set_send(
            self._vehicle.target_system, self._vehicle.target_component,
            b'SERVO9_FUNCTION',
            0,  # disabled (pwm can be overriden)
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

        message = self._vehicle.recv_match(type="PARAM_VALUE",
                                           blocking=True, timeout=1)
        if message is None:
            logger.debug("Timeout waiting for light servo channel set "
                         "response")
            return False

        # Configure Camera Tilt Servo
        logger.debug("Setting camera tilt servo channel")
        self._vehicle.mav.param_set_send(
            self._vehicle.target_system, self._vehicle.target_component,
            b'SERVO10_FUNCTION',
            0,  # disabled (pwm can be overriden)
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

        message = self._vehicle.recv_match(type="PARAM_VALUE",
                                           blocking=True, timeout=1)
        if message is None:
            logger.debug("Timeout waiting for camera tilt servo channel set "
                         "response")
            return False

        # Configure Arm Channel
        logger.debug("Setting arm servo channel")
        self._vehicle.mav.param_set_send(
            self._vehicle.target_system, self._vehicle.target_component,
            b'SERVO11_FUNCTION',
            0,  # disabled (pwm can be overridden)
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

        message = self._vehicle.recv_match(type="PARAM_VALUE",
                                           blocking=True, timeout=1)
        if message is None:
            logger.debug("Timeout waiting for arm servo channel set response")
            return False

        # Set current limit
        logger.debug("Setting initial current limit")
        self.set_current_limit_on_autopilot(DEFAULT_CURRENT_LIMIT)
        message = self._vehicle.recv_match(type="PARAM_VALUE",
                                           blocking=True, timeout=1)
        if message is None:
            logger.debug("Timeout waiting for current limit set response")
            return False
        elif message.param_id == 'MOT_BAT_CURR_MAX' \
                and hasattr(message, 'param_value'):
            self.__current_limit_now = int(message.param_value)

        logger.debug('Attaching a listener for "/vehicle/power_source/" '
                     'topic.')
        topic: Topic = topic_factory.create_topic('/vehicle/power_source/')
        listener = PowerSourceDetectorListener(self._on_new_power_source)
        topic.attach(listener)
        receiver.start()

        self.set_light(0)
        self.straighten_the_camera()
        self.do_disarm()
        self.set_navigation_mode(VehicleNavigationMode.MANUAL)
        # self.stop_movement()
        if not self.__movements.is_alive():
            self.__movements.start()
        self.stop_arm()

        # Unlocking all servos when initializing (or rebooting) vehicle
        self.servo_lock_service.unlock_all_servos()

        return True

    def __validate_movement_inputs(self, speed, flag):
        """
        Used in Turn a Movement Methods. Validates the inputs.

        Arguments
        ---------
        speed : float, int
            Speed intensity.
        flag : bool
            Movement sense.

        Raises
        ------
        TypeError
            For invalid input types.
        ValueError
            For invalid input values (speed > 100 or < 0).

        """
        if not isinstance(speed, (float, int)) or isinstance(speed, bool):
            # isinstance(speed, bool) is needed on python3, because bool
            # is an instance of int.
            raise TypeError('Input is no a valid type')
        if not (0 <= speed <= 100):
            raise ValueError('Input is out of range')
        if not isinstance(flag, bool):
            raise TypeError('Input is no a valid type')

    @skip_none
    def _do_send_heartbeat(self):
        """
        Send the HEARTBEAT message to the ROV.

        Used in a "RepeatedTimer" thread to maintain communication with
        the ROV.

        """
        self._vehicle.mav.heartbeat_send(
            6,  # type // 6: MAV_TYPE = MAV_TYPE_GCS
            8,  # autopilot // 8: MAV_AUTOPILOT = MAV_AUTOPILOT_INVALID
            192,  # base_mode //
            0,  # custom_mode
            4,  # system_status
            3  # mavlink_version
        )
        time_now = time()
        if (time_now - self.__last_rate_set_time) >= 5.0:
            logger.debug("Setting data rate")
            for _ in range(3):
                self._vehicle.mav.request_data_stream_send(
                    self._vehicle.target_system,
                    self._vehicle.target_component,
                    mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1)
            self.__last_rate_set_time = time_now
        # self._vehicle.mav.rc_channels_override_send(
        #     self._vehicle.target_system,  # target_system
        #     self._vehicle.target_component,  # target_component
        #     *self.__last_rc)  # RC channel list, in microseconds.

    def set_navigation_mode(self, mode):
        """
        Change the ROV navigation mode.

        Arguments
        ---------
        mode : VehicleNavigationMode, str
            It could be either:
            -   A VehicleNavigationMode enum value.
            -   A string with the mode name.

        Raises
        ------
        TypeError
            When the passed mode is not of a valid type.

        """
        # Get mode ID
        if not self._connected:
            return
        if isinstance(mode, VehicleNavigationMode):
            mode_id = mode.value
        elif isinstance(mode, str):
            mode_id = self._vehicle.mode_mapping()[mode]
            if mode == 'ALT_HOLD':
                mode = VehicleNavigationMode.DEPTH_HOLD
            else:
                mode = getattr(VehicleNavigationMode, mode)
        else:
            raise TypeError
        self.state.navigation_mode = mode
        self._vehicle.mav.set_mode_send(
            self._vehicle.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id)

    def _override_rc(self, pwm, channel):
        """
        Overrides the RC Channels.

        The channels are:
        1.  Pitch
        2.  Roll
        3.  Throttle (vertical)
        4.  Yaw
        5.  Forward
        6.  Lateral

        Arguments
        ---------
        pwm : int
            Integer with the pwm value to send to the channel. Must be
            in the range 1100-1900.
        channel : int
            Integer with the number of the PWM channel to modify.

        """
        if self._vehicle is None:
            return
        # rc_channel_value = [65535 for _ in range(8)]
        # rc_channel_value[channel - 1] = int(round(pwm))
        self.__last_rc[channel - 1] = int(round(pwm))
        self._vehicle.mav.rc_channels_override_send(
            self._vehicle.target_system,  # target_system
            self._vehicle.target_component,  # target_component
            *self.__last_rc)  # RC channel list, in microseconds.

    def _update_all_axes(self, pitch_speed, roll_speed, throttle_speed,
                         yaw_speed, forward_speed, lateral_speed):
        if self._vehicle is None:
            return

        # Skipping axes updates when needed
        if not self.update_all_axes:
            return

        rc_channel_values = [int(axis_speed * 4 + 1500)
                             for axis_speed in (pitch_speed, roll_speed,
                                                throttle_speed, yaw_speed,
                                                forward_speed, lateral_speed)]
        rc_channel_values += [65535, 65535]
        self._vehicle.mav.rc_channels_override_send(
            self._vehicle.target_system,  # target_system
            self._vehicle.target_component,  # target_component
            *rc_channel_values)  # RC channel list, in microseconds.
        # Stopping RC channel values log bombing
        # logger.debug(f'RC: {rc_channel_values[:6]}')

    def _do_acquire_data(self):
        """
        Capture a data message.

        Used in telemetry thread. Captures a single data package and
        checks if it's a 'tracked' package type.

        Returns
        -------
        dict, None
            If the message captured is of a "tracked" type, returns the
            package dict, otherwise returns None.

        """
        try:
            from serial import SerialException
            pyserial_importable = True
        except ImportError:
            pyserial_importable = False

        # Mavlink incoming message parsing
        types = ['HEARTBEAT', 'SYS_STATUS', 'ATTITUDE',
                 'SCALED_PRESSURE2', 'STATUSTEXT', 'PARAM_VALUE',
                 'MAV_CMD', 'COMMAND_ACK', 'MAG_CAL_PROGRESS']
        if pyserial_importable:
            try:
                message = self._vehicle.recv_match(type=types, blocking=True,
                                                   timeout=0.5)
            except SerialException as ser_exc:
                message = None
                logger.warning("Serial Port connection error when "
                               "acquiring data: {}".format(str(ser_exc)))
                sleep(3)  # To avoid spamming on logger
        else:
            message = self._vehicle.recv_match(type=types, blocking=True,
                                               timeout=0.5)
        if not message:
            return None
        timestamp = time()
        self.report_heartbeat()
        try:
            message.get_type()
        except AttributeError:
            return None
        msg_dict = message.to_dict()
        event_bus.trigger('new-vehicle-message', msg_dict, timestamp)
        if not isinstance(msg_dict, dict):
            return None
        return msg_dict

    def _do_process_data(self, package):
        """
        Process a single data package.

        Used in telemetry thread. Reads a data package and extracts
        relevant data.

        Arguments
        ---------
        package : dict
            Dictionary with the message data.

        """
        if not isinstance(package, dict):
            raise TypeError('Package is not a dictionary')
        if 'mavpackettype' not in package:
            raise ValueError('Package is not a valid mavlink package')
        package_type = package['mavpackettype']
        if package_type == 'HEARTBEAT':  # 1 Hz package
            if ((package['base_mode'] >> 7) == 0) and self.state.armed:
                logger.debug("Re-arming")
                self.do_arm()

            """
            Check messages HEARTBEAT:
            https://mavlink.io/en/messages/common.html#HEARTBEAT
            """

        elif package_type == 'SYS_STATUS' and \
                USE_PIXHAWK_ELECTRICAL_MEASUREMENTS:
            self.telemetry.voltage.set_millivolts(package['voltage_battery'])
            self.telemetry.current.set_centiamperes(package['current_battery'])
            # if package['voltage_battery'] >= 21200:
            #     self._on_new_power_source('external')
            # else:
            #     self._on_new_power_source('battery')
        elif package_type == 'ATTITUDE':
            self.telemetry.pitch.set_rads(package['pitch'])
            self.telemetry.roll.set_rads(package['roll'])
            self.telemetry.yaw.set_rads(package['yaw'])
            self.telemetry.pitch_speed.set_rad_s(package['pitchspeed'])
            self.telemetry.roll_speed.set_rad_s(package['rollspeed'])
            self.telemetry.yaw_speed.set_rad_s(package['yawspeed'])
        elif package_type == 'SCALED_PRESSURE2':
            self.telemetry.pressure.set_millibar(package['press_abs'])
        elif package_type == 'STATUSTEXT':
            if package['text'].find('Leak Detected') != -1:  # substring found
                self.state.leak_detected = True
                logger.warning('Leak Detected')
                for t in range(20):  # Spam during 20 secs
                    self._loop.call_later(t, event_bus.trigger,
                                          'new-user-notification',
                                          'WARNING LEAK DETECTED \n PELIGRO FILTRACIÓN DE AGUA', 60, 60)
        elif package_type == 'PARAM_VALUE':
            if package['param_id'] == 'MOT_BAT_CURR_MAX' \
                    and 'param_value' in package:
                self.__current_limit_now = int(package['param_value'])
                logger.debug(f"Current limit set to {self.current_limit}A")

        elif package_type == 'COMMAND_ACK':

            command = package.get('command')
            result = package.get('result')

            if command == mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION:
                'Confirmation of Accelerometer Calibration Routine'

                if result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    _msg = "Preflight Calibration command accepted"
                    logger.info(_msg)
                else:
                    _msg = f"Preflight Calibration command rejected with value {result}"
                    logger.warning(_msg)

                send_nav_feedback_msg(_msg)

            elif command == mavutil.mavlink.MAV_CMD_ACCELCAL_VEHICLE_POS:
                'Confirmation of setting vehicle position under accelerometer calibration mode'

                if result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    _msg = "Accelerometer Calibration Position Accepted"
                    logger.info(_msg)
                else:
                    _msg = "Error setting vehicle position for accelerometer calibration"
                    logger.warning(_msg)

                send_nav_feedback_msg(_msg)

            # Delivering motor test commands results to motor test service
            elif command == mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST:
                deliver_motor_test_feedback(result)

            else:
                logger.debug(f"Command ack: {package}")

        elif package_type == 'MAG_CAL_PROGRESS':
            """
            Useful information about magnetometer calibration process.
            """
            pct = package.get('completion_pct')

            # Delivering messages
            logger.debug(f"Magnetometer completion: {pct}%")
            send_mag_calibration_feedback(pct)

        else:
            """
            Dumping some packages here.
            Useful when dealing with new package types for testing and debug.
            """

            mavlink_dump(package)

    def _try_mavlink_serial(self, port, baud=115200):
        from serial import SerialException
        logger.debug("Trying to establish a serial connection on port: {port} "
                     "with baudrate: {baud}".format(**locals()))
        try:
            mavlink_connection = mavutil.mavlink_connection(port, baud)
        except SerialException as ser_exc:
            logger.debug(
                "Serial Port connection error: {}".format(ser_exc.strerror))
            return None
        if mavlink_connection.recv_match(blocking=True, timeout=1.0) \
                is not None:
            logger.info("MAVLink Serial Port connection try succeded "
                        "(port: {port})".format(**locals()))
            return mavlink_connection
        else:
            logger.debug(
                "No MAVLink messages on: \"{port}\", closing...".format(
                    **locals()))
            mavlink_connection.close()
            return None

    def _try_mavlink_udp(self, port):
        logger.debug("Trying to establish an UDP connection on port: "
                     "{port}".format(**locals()))
        try:
            mavlink_connection = mavutil.mavlink_connection(
                'udpin:0.0.0.0:{port}'.format(**locals()))
        except BaseException as udp_exc:
            logger.warning(
                "Unknown exception when connecting to MAVLink by UDP "
                "on port {0}: {1}".format(port, udp_exc))
            return None
        if mavlink_connection.recv_match(blocking=True, timeout=1.0) \
                is not None:
            logger.info("MAVLink UDP connection try succeded "
                        "(port: {port})".format(**locals()))
            return mavlink_connection
        else:
            logger.debug(
                "No MAVLink messages on port: \"{port}\", closing...".format(
                    **locals()))
            mavlink_connection.close()
            return None

    def _try_mavlink_connection(self):
        # Check if pyserial is importable
        if 'serial' not in locals() \
                or not hasattr(serial, 'tools') \
                or not hasattr(serial.tools, 'list_ports'):
            try:
                import serial.tools.list_ports
            except ImportError:
                logger.debug('No pyserial installed, just trying UDP')
                return self._try_mavlink_udp(self.DEFAULT_PORT)
        if self.settings.ap_serial_port == "@discover":
            # Get available ports.
            available_ports_desc = serial.tools.list_ports.comports()
            available_ports = [port_desc.device
                               for port_desc in available_ports_desc]
            if len(available_ports) > 0:
                logger.debug("A serial connection will be tried in the "
                             f"following ports: {available_ports}")
            else:
                logger.info("Not any serial port available")
            # Try MAVLink serial on every available port.
            for port in available_ports:
                link = self._try_mavlink_serial(port)
                if link is not None:
                    if link.wait_heartbeat(blocking=True, timeout=1) is None:
                        logger.warning("MAVLink connected, but no HEARTBEAT "
                                       "message, closing...")
                        link = None
                        continue
                    return link
        elif self.settings.ap_serial_port == "@none":
            logger.debug("It was specified not to use the serial port ("
                         "ap_serial_port: '@none')")
            return self._try_mavlink_udp(self.DEFAULT_PORT)
        else:  # specific serial port
            link = self._try_mavlink_serial(self.settings.ap_serial_port)
            return link if link is not None \
                else self._try_mavlink_udp(self.DEFAULT_PORT)

    def start_connection(self):
        """
        Tries to start a mavlink connection.

        Tries to establish a MAVLink connection using either a valid
        link. Right now it will try a connection with:
        * Serial Ports at 115200 bps, just when pyserial is installed.
        * UDP server (udpin) at port 14500.

        After a connection has been established, it calls the required
        wait_heartbeat() method (to let mavutil set the "target_system"
        and "target_component" attributes). Then it activates the data
        stream by using the "request_data_stream_send" method.

        Returns
        -------
        bool
            **True** if connection succeded, **False** otherwise.

        """
        # Try to get a connection link
        logger.debug("Trying to start a MAVLink connection")
        link = self._try_mavlink_connection()
        if link is not None:
            self._vehicle = link
            logger.debug("Waiting for a HEARTBEAT message...")
            if self._vehicle.wait_heartbeat(blocking=True, timeout=2) is None:
                logger.warning("MAVLink connected, but no HEARTBEAT message, "
                               "closing...")
                return False
            # This for loop was taken from:
            # https://github.com/ArduPilot/pymavlink/blob/master/
            # examples/apmsetrate.py
            for _ in range(3):
                self._vehicle.mav.request_data_stream_send(
                    self._vehicle.target_system,
                    self._vehicle.target_component,
                    mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1)
            self.__last_rate_set_time = time()
            self._connected = True
            if not self.__initialize_vehicle():
                self._connected = False
                return False

            msg = "Vehicle started..."
            logger.info(msg)
            send_nav_feedback_msg(msg)
            return True
        else:
            return False

    def get_mavlink_connection(self):
        return self._vehicle

    def disconnect(self):
        """
        Stop connection with Vehicle.

        Stops the current connection, restoring vehicle configuration
        to 'default' settings and terminating all active threads.

        """
        # Reconfigure default servo parameters
        # Configure Light Servo
        # self._vehicle.mav.param_set_send(
        #     self._vehicle.target_system, self._vehicle.target_component,
        #     b'SERVO9_FUNCTION',
        #     59,
        #     mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

        # Configure Camera Tilt Servo
        # self._vehicle.mav.param_set_send(
        #     self._vehicle.target_system, self._vehicle.target_component,
        #     b'SERVO10_FUNCTION',
        #     7,
        #     mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

        logger.info("Disconnecting")
        self._connected = False
        event_bus.trigger('vehicle-disconnected')
        # self.__heartbeat_handler.stop()
        # self.time.stop()

    def start_stabilize_mode(self):
        """
        Starts "Stabilize Mode".

        Makes the Vehicle stabilize itself and maintain it's heading.

        """
        logger.info('Mode: STABILIZE')
        self.set_navigation_mode('STABILIZE')

    def stop_stabilize_mode(self):
        """
        Stops "Stabilize Mode".

        Returns the Vehicle to "Manual mode".
        """
        logger.info('Mode: MANUAL')
        self.set_navigation_mode('MANUAL')

    def start_depth_hold_mode(self):
        """
        Starts "Depth Hold Mode".

        Makes the Vehicle maintain its depth and heading.
        """
        logger.info('Mode: DEPTH_HOLD')
        self.set_navigation_mode('ALT_HOLD')

    def stop_depth_hold_mode(self):
        """
        Stops "Depth Hold Mode".

        Returns the Vehicle to "Manual Mode".
        """
        logger.info('Mode: MANUAL')
        self.set_navigation_mode('MANUAL')

    @skip_none
    def do_arm(self):
        """
        Arm motors.

        Arms the vehicle, making it able to power the thrusters.
        """

        logger.info("ARMED")
        self.state.armed = True
        self._vehicle.mav.command_long_send(
            self._vehicle.target_system,
            self._vehicle.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1,  # arm the device
            0, 0, 0, 0, 0, 0)
        self.stop_movement()

        report_vehicle_armed(True)
        # self._send_blinking_text('MODE: ARMED')

    @skip_none
    def do_disarm(self):
        """
        Disarm motors.

        Disarms the vehicle, making it unable to move.
        """

        logger.info("DISARMED")
        self.state.armed = False
        self._vehicle.mav.command_long_send(
            self._vehicle.target_system,
            self._vehicle.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0,  # disarm the device
            0, 0, 0, 0, 0, 0)
        self.stop_movement()

        report_vehicle_armed(False)

    def stop_movement(self):
        """
        Stop the vehicle.

        Set all movement and turn speeds to 0.
        """
        if self._vehicle is None:
            return
        logger.debug("Stopping all movements")
        rc_channel_value = [1500, 1500, 1500, 1500, 1500, 1500, 65535, 65535]
        self.__last_rc = list(rc_channel_value)
        self._vehicle.mav.rc_channels_override_send(
            self._vehicle.target_system,  # target_system
            self._vehicle.target_component,  # target_component
            *rc_channel_value)  # RC channel list, in microseconds.

    def move_horizontal(self, speed, go_forward):
        """
        Surge traslational movement method.

        Makes the Vehicle move on the surge direction.

        Arguments
        ---------
        speed : float, int
            Percent of the maximum speed of the vehicle in that
            direction (from 0 to 100).
        go_forward : bool
            If True, the vehicle will move forward, otherwise it will
            move backwards.

        """
        self.__validate_movement_inputs(speed, go_forward)
        if USE_NEW_SPEED_SMOOTHER:
            logger.info(f"Using (((new speedsmoother))).")
            self.__movements.update_forward(speed, go_forward)
        elif USE_OLD_SPEED_SMOOTHER:
            logger.info(f"Using (((old speedsmoother))).")
            logger.debug("Enqueuing (horizontal): {} {}".format(speed,
                                                                go_forward))
            if speed < 3:
                speed = 0.0
            self.horizontal_smoother.target = speed * (1.0 if go_forward
                                                       else -1.0)
        else:
            self._do_move_horizontal(speed, go_forward)

    def _do_move_horizontal(self, speed, go_forward):
        logger.info(
            "Surge %s at %.1f%% speed",
            "forward" if go_forward else "backwards", float(speed))
        if go_forward:
            pwm = 1500 + speed * 4
        else:
            pwm = 1500 - speed * 4
        self._override_rc(pwm, 5)

    def move_lateral(self, speed, go_right):
        """
        Sway traslational movement method.

        Makes the Vehicle move on the sway direction.

        Arguments
        ---------
        speed : float, int
            Percent of the maximum speed of the vehicle in that
            direction (from 0 to 100).
        go_right : bool
            If True, the vehicle will move right, otherwise it will
            move left.

        """
        self.__validate_movement_inputs(speed, go_right)
        if USE_NEW_SPEED_SMOOTHER:
            self.__movements.update_lateral(speed, go_right)
        elif USE_OLD_SPEED_SMOOTHER:
            logger.debug("Enqueuing (lateral): {} {}".format(speed, go_right))
            if speed < 3:
                speed = 0.0
            self.lateral_smoother.target = speed * (1.0 if go_right else -1.0)
        else:
            self._do_move_lateral(speed, go_right)

    def _do_move_lateral(self, speed, go_right):
        logger.info(
            "Sway %s at %.1f%% speed",
            "right" if go_right else "left", float(speed))
        if go_right:
            pwm = 1500 + speed * 4
        else:
            pwm = 1500 - speed * 4
        self._override_rc(pwm, 6)

    def move_vertical(self, speed, go_up):
        """
        Heave traslational movement method.

        Makes the Vehicle move on the heave direction.

        Arguments
        ---------
        speed : float, int
            Percent of the maximum speed of the vehicle in that
            direction (from 0 to 100).
        go_up : bool
            If True, the vehicle will move up, otherwise it will
            move down.

        """
        self.__validate_movement_inputs(speed, go_up)
        if USE_NEW_SPEED_SMOOTHER:
            self.__movements.update_vertical(speed, go_up)
        elif USE_OLD_SPEED_SMOOTHER:
            logger.debug("Enqueuing (vertical): {} {}".format(speed, go_up))
            if speed < 3:
                speed = 0.0
            self.vertical_smoother.target = speed * (1.0 if go_up else -1.0)
        else:
            self._do_move_vertical(speed, go_up)

    def _do_move_vertical(self, speed, go_up):
        logger.info(
            "Heave %s at %.1f%% speed",
            "up" if go_up else "down", float(speed))
        if go_up:
            pwm = 1500 + speed * 4
        else:
            pwm = 1500 - speed * 4
        self._override_rc(pwm, 3)

    def turn_yaw(self, speed, turn_right, target_angle=None):
        """
        Yaw rotational movement method.

        Makes the Vehicle turns in the yaw axis.

        Arguments
        ---------
        speed : float, int
            Percent of the maximum speed of the vehicle in that
            direction (from 0 to 100).
        turn_right : bool
            If True, the vehicle will turn right, otherwise it will
            turn left.
        target_angle : None, optional
            (**Feature not implemented**) This feature doesn't have any
            effect yet.

        """
        self.__validate_movement_inputs(speed, turn_right)
        if USE_NEW_SPEED_SMOOTHER:
            self.__movements.update_yaw(speed, turn_right)
        elif USE_OLD_SPEED_SMOOTHER:
            logger.debug("Enqueuing (yaw): {} {}".format(speed, turn_right))
            if speed < 3:
                speed = 0.0
            self.yaw_smoother.target = speed * (1.0 if turn_right else -1.0)
        else:
            self._do_turn_yaw(speed, turn_right)

    def _do_turn_yaw(self, speed, turn_right, target_angle=None):
        logger.info(
            "Yaw turning %s at %.1f%% speed",
            "right" if turn_right else "left", float(speed))
        if turn_right:
            pwm = 1500 + speed * 4
        else:
            pwm = 1500 - speed * 4
        self._override_rc(pwm, 4)

    def turn_pitch(self, speed, nose_up, target_angle=None):
        """
        Pitch rotational movement method.

        Makes the Vehicle turns in the pitch axis.

        Arguments
        ---------
        speed : float, int
            Percent of the maximum speed of the vehicle in that
            direction (from 0 to 100).
        nose_up : bool
            If True, the vehicle will turn its nose up, otherwise it will
            turn its nose down.
        target_angle : None, optional
            (**Feature not implemented**) This feature doesn't have any
            effect yet.

        """
        self.__validate_movement_inputs(speed, nose_up)
        if USE_NEW_SPEED_SMOOTHER:
            self.__movements.update_pitch(speed, nose_up)
        else:
            logger.debug(
                "Pitch nose %s at %.1f%% speed",
                "up" if nose_up else "down", float(speed))
            if nose_up:
                pwm = 1500 + speed * 4
            else:
                pwm = 1500 - speed * 4
            self._override_rc(pwm, 1)

    def turn_roll(self, speed, go_clockwise, target_angle=None):
        """
        Roll rotational movement method.

        Makes the Vehicle turns in the roll axis.

        Arguments
        ---------
        speed : float, int
            Percent of the maximum speed of the vehicle in that
            direction (from 0 to 100).
        go_clockwise : bool
            If True, the vehicle will turn clockwise, otherwise it will
            turn counter clockwise.
        target_angle : None, optional
            (**Feature not implemented**) This feature doesn't have any
            effect yet.

        """
        self.__validate_movement_inputs(speed, go_clockwise)
        if USE_NEW_SPEED_SMOOTHER:
            self.__movements.update_roll(speed, go_clockwise)
        else:
            logger.debug(
                "Roll turning %s at %.1f%% speed",
                "clockwise" if go_clockwise else "counter-clockwise",
                float(speed))
            if go_clockwise:
                pwm = 1500 + speed * 4
            else:
                pwm = 1500 - speed * 4
            self._override_rc(pwm, 2)

    def set_light(self, intensity):
        """
        Sets the intensity of the auxiliary lights.

        Arguments
        ---------
        intensity : float, int
            Light intensity percent to set (from 0 to 100).

        """
        if self._vehicle is None:
            return

        if not isinstance(intensity, (float, int)):
            raise TypeError('Input is not an int or float')

        if intensity > 100:
            intensity = 100

        if intensity < 0:
            intensity = 0

        "Accepted light controllers here are autopilot (pixhawk) and insytech power manager board."

        if self.settings.lights_controller == "autopilot":

            pwm = int(round(8 * intensity + 1100))
            if self.lights_on_off_control:
                self._vehicle.set_relay(0, intensity > 0)
            else:
                self._vehicle.mav.command_long_send(
                    self._vehicle.target_system,
                    self._vehicle.target_component,
                    mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                    0,  # confimation
                    9,  # RC9, aux 1 (Lights)
                    pwm,
                    0, 0, 0, 0, 0)  # unused parameters

        if self.settings.lights_controller == "power_manager_board_v1":
            """
            Power manager board expects pwm value in 0 to 255 range but intensity variable is incoming in 0 to 100
            range.
            """

            pwm_topic = topic_factory.create_topic("/vehicle/power_manager_board/pwm_control/")
            pwm_value = round(float(intensity)/100.0 * 255.0)

            pwm_topic.send({'pwm_id': 0, 'pwm_value': pwm_value})

        if self.settings.lights_controller == "arduino_acamas":
            """
            First version of Arduino Acamas only support on/off lights.
            """

            lights_topic = topic_factory.create_topic("/rov/arduino_acamas/lights/")

            if intensity > 1.0:
                lights_topic.send(dict(cmd="lights_on"))

            else:
                lights_topic.send(dict(cmd="lights_off"))

        logger.info("Lights set to %d%%", int(intensity))
        self.state.lights_level = intensity
        event_bus.trigger(
            'new-user-notification',
            "New lights level: {}%".format(intensity), 2, 2)

    def get_light(self):
        """
        Actual auxiliary light intensity getter.

        Returns
        -------
        float
            Returns the lights current intensity, in percent.

        """
        return self.state.lights_level

    @staticmethod
    def __calculate_line_eq_coeffs(x1, y1, x2, y2):
        slope = (y2 - y1) / (x2 - x1)
        offset = y1 - slope * x1
        return slope, offset

    def __calculate_pwm_linear_coeffs(self):
        self.__positive_slope, self.__positive_offset = \
            self.__calculate_line_eq_coeffs(
                0.0, self.cam_tilt_pwm_center,
                self.cam_tilt_angle_max, self.cam_tilt_pwm_min
                if self.cam_tilt_pwm_reverse
                else self.cam_tilt_pwm_max)
        self.__negative_slope, self.__negative_offset = \
            self.__calculate_line_eq_coeffs(
                0.0, self.cam_tilt_pwm_center,
                self.cam_tilt_angle_min, self.cam_tilt_pwm_max
                if self.cam_tilt_pwm_reverse
                else self.cam_tilt_pwm_min)

    def __calculate_pwm_regular(self, angle):
        if angle == 0:
            return self.cam_tilt_pwm_center
        elif angle < 0:
            slope = self.__negative_slope
            offset = self.__negative_offset
        else:  # angle > 0
            slope = self.__positive_slope
            offset = self.__positive_offset
        return angle * slope + offset

    def __calculate_pwm_by_degree_unit_steps(self, angle):
        normal_angle = angle + 90
        if self.cam_tilt_pwm_reverse:
            pwm = 1100 + 800 * normal_angle / 181 + 800 / 360
        else:
            pwm = 1900 - 800 * normal_angle / 181 - 800 / 360
        return int(pwm)

    def set_camera_tilt(self, angle):
        """
        Sets the tilt of the camera.

        Parameters
        ----------
        angle : float, int
            Angle to tilt the camera to (from -45 to 45, where 0 is
            centered).

        """
        if self._vehicle is None:
            return
        if not isinstance(angle, (float, int)):
            raise TypeError('Input is not a valid type')
        if self.__degree_unit_range_fix:
            pwm = self.__calculate_pwm_by_degree_unit_steps(angle)
        else:
            pwm = self.__calculate_pwm_regular(angle)
        self._vehicle.mav.command_long_send(
            self._vehicle.target_system,
            self._vehicle.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
            0,  # confimation
            10,  # RC10, aux 2 (camera tilt)
            int(round(pwm)),
            0, 0, 0, 0, 0)  # unused parameters
        degree = u'\u00b0'
        logger.info("Camera Tilt changed to {}".format(int(angle)) + degree +
                    "(PWM: {})".format(pwm))

        # send blinking text
        # self._send_blinking_text(
        #     'CAM TILT: ' + str(-1 * int(angle)) + degree)

    def straighten_the_camera(self):
        """Resets the camera tilt to center."""
        logger.info("Straightening the camera")
        self.set_camera_tilt(0)

    def is_stabilize_mode_on(self):
        """
        Checks if STABILIZE mode is activated.

        Returns
        -------
        bool
            True if the Vehicle is in "Stabilize Mode", False otherwise.

        """
        return self.state.navigation_mode is VehicleNavigationMode.STABILIZE

    def is_depth_hold_mode_on(self):
        """
        Checks if DEPTH_HOLD mode is activated.

        Returns
        -------
        bool
            True if the Vehicle is in "Depth Hold Mode", False
            otherwise.

        """
        return self.state.navigation_mode is VehicleNavigationMode.DEPTH_HOLD

    def toggle_armed(self):
        """Toggles the system state between "Armed" and "Disarmed"."""
        if self.is_armed():
            self.do_disarm()
        else:
            self.do_arm()

    def is_armed(self):
        """
        Checks if the system is "armed" or "disarmed".

        "Armed" means that the system can power the thrusters.

        Returns
        -------
            Returns True if the vehicle is armed, False otherwise.

        """
        return self.state.armed

    def open_arm(self):
        """Opens a connected Arm."""
        self.__arm_controller.open()

    def close_arm(self):
        """Closes a connected Arm."""
        self.__arm_controller.close()

    def stop_arm(self):
        """Stops the connected Arm movement."""
        self.__arm_controller.stop()

    @property
    def cam_tilt_pwm_reverse(self):
        return self.__cam_tilt_pwm_reverse

    @cam_tilt_pwm_reverse.setter
    def cam_tilt_pwm_reverse(self, value):
        pass  # Read-only right now

    @property
    def cam_tilt_pwm_max(self):
        return self.__cam_tilt_pwm_max

    @cam_tilt_pwm_max.setter
    def cam_tilt_pwm_max(self, value):
        pass

    @property
    def cam_tilt_pwm_min(self):
        return self.__cam_tilt_pwm_min

    @cam_tilt_pwm_min.setter
    def cam_tilt_pwm_min(self, value):
        pass

    @property
    def cam_tilt_pwm_center(self):
        return self.__cam_tilt_pwm_center

    def _on_new_power_source(self, power_source: str):
        new_current = self._current_limit_policy.get(power_source,
                                                     self.current_limit)
        self._curr_power_source = power_source
        self.current_limit = new_current

    @property
    def current_limit(self) -> Optional[int]:
        """ Current limit as set on autopilot.

        Returns
        -------
        Optional[int]
            The last confirmed current limit set on autopilot. It will be
            None if the parameter set was not confirmed.
        """
        return self.__current_limit_now

    @current_limit.setter
    def current_limit(self, value):
        if value != self.current_limit:
            self.set_current_limit_on_autopilot(value)

    def set_current_limit_on_autopilot(self, value: int):
        if self._connected:
            if (time() - self.__last_current_limit_set_time) < \
                    CURRENT_LIMIT_SET_MIN_INTERVAL:
                logger.debug("Avoiding current limit set because it changed "
                             "recently.")
                return
            self.__last_current_limit_set_time = time()
            logger.debug(f"Setting the current limit to {value}A")
            self._vehicle.param_set_send(
                'MOT_BAT_CURR_MAX', float(value),
                mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

    @skip_none
    def disable_servo(self, servo_n):

        if servo_n > 8 or servo_n < 1:
            return

        logger.info(f"Disabling servo {servo_n}.")
        self._vehicle.param_set_send(
            f"SERVO{servo_n}_FUNCTION", 0.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

    @skip_none
    def enable_servo(self, servo_n, motor_n):

        logger.info(f"Enabling servo {servo_n} with associated motor {motor_n}.")
        self._vehicle.param_set_send(
            f"SERVO{servo_n}_FUNCTION", motor_n + 32, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

    @skip_none
    def set_preflight_mode(self):

        logger.debug("Setting preflight mode.")

        self._vehicle.mav.command_long_send(
            self._vehicle.target_system,
            self._vehicle.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            mavutil.mavlink.MAV_MODE_PREFLIGHT,
            0, 0, 0, 0, 0, 0, 0)

    @skip_none
    def set_manual_mode(self):
        self.set_navigation_mode(VehicleNavigationMode.MANUAL)

    @skip_none
    def start_accel_calib(self):
        """
        Starting accelerometer calibration routine. Vehicle must be disarmed before start.
        """

        logger.info("Starting accel calibration.")

        self._vehicle.mav.command_long_send(
            self._vehicle.target_system,
            self._vehicle.target_component,
            mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
            0,
            0, 0, 0, 0, 1, 0, 0)

    @skip_none
    def set_vehicle_position(self, position):
        """
        Indicate ardusub the vehicle position over the context of accelerometer calibration.

        :param position: Value from 1 to 6 according to the ardupilot specific message set,
                        https://mavlink.io/en/messages/ardupilotmega.html#ACCELCAL_VEHICLE_POS
        :return: void
        """

        vehicle_position = mavutil.mavlink.enums['ACCELCAL_VEHICLE_POS'][position]
        logger.info(f"Setting vehicle position {vehicle_position.name}")

        self._vehicle.mav.command_long_send(
            self._vehicle.target_system,
            self._vehicle.target_component,
            mavutil.mavlink.MAV_CMD_ACCELCAL_VEHICLE_POS,
            0,
            position, 0, 0, 0, 0, 0, 0)

    @skip_none
    def start_magnetometer_calibration(self):
        """
        Starting magnetometer calibration for all sensors present, with retry on failure, autosave and auto-reboot
        options activated. Also, it's added a 2 seconds delay so vehicle users can have some time before start.

        For more info look documentation about MAV_CMD_DO_START_MAG_CAL,
        https://mavlink.io/en/messages/ardupilotmega.html#MAV_CMD_DO_START_MAG_CAL
        """

        logger.info("Starting magnetometer calibration.")

        self._vehicle.mav.command_long_send(
            self._vehicle.target_system,
            self._vehicle.target_component,
            mavutil.mavlink.MAV_CMD_DO_START_MAG_CAL,
            0,
            0, 1, 1, 2, 1, 0, 0)

    @skip_none
    def stop_magnetometer_calibration(self):

        logger.info("Stopping magnetometer calibration.")

        self._vehicle.mav.command_long_send(
            self._vehicle.target_system,
            self._vehicle.target_component,
            mavutil.mavlink.MAV_CMD_DO_ACCEPT_MAG_CAL,
            0,
            0, 0, 0, 0, 0, 0, 0)

    @skip_none
    def start_motor_test(self, motor_id, motor_throttle):
        """
        Simple interface for start motor tests.

        :param motor_id: Motor id number starting at 0
        :param motor_throttle:
            Motor throttle in pct [0 for full reverse, 50 for idle, and 100 for max forward]
        :return:
        """

        # First we stop axis spam
        self.update_all_axes = False
        self.motor_test_service.start_motor_test(motor_id, motor_throttle)

    @skip_none
    def stop_motor_test(self):
        """
        Stops motor test mode and goes back to normal navigation mode.
        """
        self.motor_test_service.stop_motor_test()
        self.update_all_axes = True

    @skip_none
    def set_motor_direction(self, motor_id, value):
        """
        Setting motor rotation direction without changing wires.
        :param motor_id: motor_id starting at 0
        :param value: 1 for regular, -1 for reverse direction
        """
        msg = f"Setting motor {motor_id} with direction: {value}"
        logger.debug(msg)
        send_nav_feedback_msg(msg)

        if value > 0:
            self._vehicle.param_set_send(f"MOT_{motor_id + 1}_DIRECTION", 1, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

        else:
            self._vehicle.param_set_send(f"MOT_{motor_id + 1}_DIRECTION", -1, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

    @skip_none
    def set_board_orientation(self, board_orientation: int):
        """
        Updates ardusub parameter AHRS_ORIENTATION defines overall board orientation
        relative to ROV position.

        More information at:
        https://ardupilot.org/copter/docs/parameters.html#ahrs-orientation-board-orientation

        :param board_orientation: value from 0 to 43 according to AHRS_ORIENTATION parameter
        """
        if board_orientation > 43 or board_orientation < 0:
            board_orientation = 0

        orientation_parameter = "AHRS_ORIENTATION"

        self._vehicle.param_set_send(orientation_parameter, board_orientation, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        _msg = f"New board orientation: {board_rotations[board_orientation].name}"
        logger.info(_msg)
        send_nav_feedback_msg(_msg)

    @skip_none
    def reboot_autopilot(self):
        """
        Short wrapper for rebooting pixhawk board.
        """
        msg = f"Rebooting board..."

        logger.info(msg)
        send_nav_feedback_msg(msg)
        send_hud_notification(text=msg, duration=5)
        self._vehicle.reboot_autopilot()


if __name__ == '__main__':
    get_rov_logger('DEBUG')
    vehicle = MavlinkVehicleLink()
    vehicle.start()
    import unittest.mock as mock

    with mock.patch.object(vehicle, '_vehicle'):
        # vehicle.set_camera_tilt(-90)
        # vehicle.set_camera_tilt(-45)
        vehicle.set_camera_tilt(50)
        vehicle.set_camera_tilt(40)
        vehicle.set_camera_tilt(30)
        vehicle.set_camera_tilt(20)
        vehicle.set_camera_tilt(10)
        vehicle.set_camera_tilt(-0)
        vehicle.set_camera_tilt(-10)
        vehicle.set_camera_tilt(-20)
        vehicle.set_camera_tilt(-30)
        vehicle.set_camera_tilt(-40)
        vehicle.set_camera_tilt(-50)
        vehicle.set_camera_tilt(-60)
        # vehicle.set_camera_tilt(-70)
        # vehicle.set_camera_tilt(90)


class MavlinkVehicleLinkPwmOutput(PwmOutput):
    DEFAULT_MIN_PWM = 1100
    DEFAULT_MAX_PWM = 1900
    VALID_CHANNELS = tuple([9, 10, 11])

    def __init__(self, channel, link=None, min_pwm=DEFAULT_MIN_PWM,
                 max_pwm=DEFAULT_MAX_PWM):
        if channel not in MavlinkVehicleLinkPwmOutput.VALID_CHANNELS:
            raise ValueError(
                f"Invalid channel (valid channels: {self.VALID_CHANNELS})")
        self.__channel = channel
        self.__min_pwm = min_pwm
        self.__max_pwm = max_pwm
        if link is not None and isinstance(link, MavlinkVehicleLink):
            self.__link = link
        else:
            self.__link = get_last_instance()
        self.__microseconds = 1500

    def set_microseconds(self, us: int):
        if self.max_pwm >= us >= self.min_pwm:
            if self.__link.connected:
                vehicle_interface = self.__link._vehicle
                vehicle_interface.mav.command_long_send(
                    vehicle_interface.target_system,
                    vehicle_interface.target_component,
                    mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                    0,  # confirmation
                    self.channel,  # RC10, aux 2 (camera tilt)
                    us,  # PWM value
                    0, 0, 0, 0, 0)  # unused parameters
                self.__microseconds = us
                logger.debug(
                    f"Updating PWM on servo channel {self.channel}: {us}")
            else:
                logger.warning("MavlinkVehicleLink is not Connected, "
                               "can't set PWM")
        else:
            logger.warning(f"Ignoring an out-of-range PWM write {us}")

    def get_microseconds(self) -> int:
        return self.__microseconds

    @property
    def channel(self):
        return self.__channel

    @property
    def min_pwm(self):
        return self.__min_pwm

    @property
    def max_pwm(self):
        return self.__max_pwm
