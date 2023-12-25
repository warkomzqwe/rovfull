"""
Module for remote control via LAN of vehicle navigation routines.

.. include:: ./readme.md
"""

from pymessaginglib.Topic import topic_factory

from rov_logger.logger import get_rov_logger


logger = get_rov_logger()


class NavRemoteControlService:
    """
    Main responsibility is start and keep running the navigation remote control
    service.

    This service allows a remote computer to execute navigation orders delivered
    directly to the pixhawk autopilot via internal mavlink vehicle link.
    """
    _topic_str = "/vehicle/navigation/rc/"

    def __init__(self, vehicle_com):
        """
        Class subscribe itself to a specific topic for cmd listening

        :param vehicle_com: A working mavlink vehicle link (see
                            mavlink_vehicle_link.py)
        """

        self._vehicle_com = vehicle_com
        self._topic = topic_factory.create_topic(self._topic_str)
        self._topic.attach_callback(self.on_new_msg)
        logger.info("Starting Navigation Remote Control Service.")

    def on_new_msg(self, dic, ip):
        """
        Handler method for incoming commands.
        """

        dic: dict
        cmd = dic.get("cmd")
        args = dic.get("args")

        logger.debug(f"Incoming message: {dic}")

        if cmd == "set_preflight_mode":
            self._vehicle_com.set_preflight_mode()

        elif cmd == "set_manual_mode":
            self._vehicle_com.set_manual_mode()

        elif cmd == "set_armed_mode":
            self._vehicle_com.do_arm()

        elif cmd == "set_disarmed_mode":
            self._vehicle_com.do_disarm()

        elif cmd == "start_accel_calib":
            self._vehicle_com.start_accel_calib()

        elif cmd == "set_vehicle_position" and isinstance(args, tuple or list):
            # first argument is ACCELCAL_VEHICLE_POS enum
            # check https://mavlink.io/en/messages/ardupilotmega.html#ACCELCAL_VEHICLE_POS
            self._vehicle_com.set_vehicle_position(args[0])

        elif cmd == "start_magnetometer_calibration":
            self._vehicle_com.start_magnetometer_calibration()

        elif cmd == "stop_magnetometer_calibration":
            self._vehicle_com.stop_magnetometer_calibration()

        elif cmd == "start_motor_test":
            # we need args motor_id and motor_throttle to run this
            # motor_id index starts at 0
            try:
                self._vehicle_com.start_motor_test(args[0], args[1])

            except (IndexError, TypeError):
                logger.warning(f'Incoming start motor test command lacks arguments')

        elif cmd == "stop_motor_test":
            self._vehicle_com.stop_motor_test()

        elif cmd == "set_motor_direction":
            # We need motor_id and motor_direction_value [1, -1] to run this
            # motor_id index starts at 0
            try:
                self._vehicle_com.set_motor_direction(args[0], args[1])

            except (IndexError, TypeError):
                logger.warning(f'Setting motor direction lacks arguments')

        elif cmd == "set_board_orientation":
            # 1 argument needed AHRS_ORIENTATION
            # value from 0 to 43
            try:
                self._vehicle_com.set_board_orientation(args[0])
            except (IndexError, TypeError):
                logger.warning(f'Setting board orientation lacks arguments')

        elif cmd == "reboot_autopilot":
            self._vehicle_com.reboot_autopilot()

        else:
            logger.warning(f"Command {cmd} for remote control not implemented.\n"
                           f"Source ip is {ip}")
