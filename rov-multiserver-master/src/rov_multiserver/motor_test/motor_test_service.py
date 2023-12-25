"""
Module to keep an interface to control motor test functionality from ardusub.
"""

from threading import Thread, Event, Lock

from pymavlink.mavutil import mavlink
from rov_logger.logger import get_rov_logger

logger = get_rov_logger()


class MotorTestService:

    def __init__(self, vehicle_link):
        """
        Class keeps an easy-to-follow interface to control motor test functionality
        from ardusub.
        :param vehicle_link: A valid MavlinkVehicleLink object with mavlink connectivity
        """

        self._vehicle_link = vehicle_link

        self._motor_test_thread = Thread()

        self._lock = Lock()  # keeping multithreading safety
        self._stop_motor_test_signal = Event()
        self._motor_throttle = 50  # throttle value is percentage [0-100]
        self._motor_id = 0  # current motor under test index start at 0 for motor 1
        self._period = 0.05  # non documented requirement for motor testing in [s]

    def _test_motor_routine(self):
        """
        Main motor test routine extracted from test_motor_mode.py functional test.
        """

        # Manual mode is required
        self._vehicle_link.set_manual_mode()

        # Parameter for mavlink MAV_CMD_DO_MOTOR_TEST command

        # motor id
        throttle_type = 0  # 0=pct, 1=pwm, 2=passthrough pilot
        # motor_throttle
        timeout = 0
        motor_count = 0
        test_order = 2  # MOTOR_TEST_ORDER_BOARD

        while not self._stop_motor_test_signal.wait(self._period):

            if self._vehicle_link is None:
                return

            self._lock.acquire()
            try:
                mavlink_connection = self._vehicle_link.get_mavlink_connection()

                mavlink_connection.mav.command_long_send(
                    mavlink_connection.target_system,
                    mavlink_connection.target_component,
                    mavlink.MAV_CMD_DO_MOTOR_TEST,
                    0,
                    self._motor_id,
                    throttle_type,
                    self._motor_throttle,
                    timeout,
                    motor_count,
                    test_order,
                    0)

            finally:
                self._lock.release()

    def start_motor_test(self, motor_id, motor_throttle):
        """
        Starting motor test. Internally runs a needed thread.

        If already started it could be used for update parameters.

        When having an already running motor it's **HEAVILY** recommended stopping it first.
        System will keep motor running while receiving motor test commands even when
        changing target motor.

        :param motor_id: motor identification starting at 0 for motor 1
        :param motor_throttle: throttle as percentage from 0 to 100
        """

        self._lock.acquire()
        try:
            self._motor_id = motor_id
            self._motor_throttle = motor_throttle

            # If motor test thread is already running we skip this part
            # Only update of motor_id and motor_throttle is passed to thread
            if not self._motor_test_thread.is_alive():

                logger.debug(f'Starting a new motor test thread.')

                self._motor_test_thread = Thread(
                    target=self._test_motor_routine,
                    name='motor_test_thread',
                    daemon=True)
                self._motor_test_thread.start()

        finally:
            self._lock.release()

    def stop_motor_test(self):
        """
        Stopping all motor test threads and waits for termination.
        :return:
        """

        self._stop_motor_test_signal.set()
        self._motor_test_thread.join()
        self._stop_motor_test_signal.clear()
        logger.debug(f'All motor test threads terminated successfully.')
