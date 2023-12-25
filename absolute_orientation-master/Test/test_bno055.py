import os
import unittest
from time import sleep
from unittest.mock import patch, Mock, mock_open, call

from rov_event_bus.bus import event_bus, NOT_URGENT

from absolute_orientation.Bno055 import AbsoluteOrientation, chip_id, chip_id_answer, calib_stat, \
    _Bno055RemoteControl, i2c_addr_sel_low
from absolute_orientation.IAbsoluteOrientation import I2CCom

priority = NOT_URGENT
calib_data_file = 'calib_data_bno055.yaml'
EVENT_NAME = 'absolute_orientation_event'

"""
TODO include test for i2c address behavior.
"""


class FakeI2cCom(I2CCom):

    def __init__(self):
        super().__init__()
        self.sensor_detection = 1
        self.calibration_status = 0
        self.default_value = 0
        self.pointing_i2c_address = 0
        self.chip_i2c_address = 0x28

    def set_i2c_address(self, i2c_address: int):
        self.pointing_i2c_address = i2c_address

    def write(self, register: int, value: int):
        pass

    def read_register(self, register: int) -> int:

        try:
            assert self.pointing_i2c_address == self.chip_i2c_address
        except AssertionError:
            raise IOError

        if register == chip_id:
            return self.sensor_detection

        elif register == calib_stat:
            return self.calibration_status

        else:
            return self.default_value

    def read_register_burst(self, register, n) -> list:
        return [self.default_value] * n

    @property
    def calibration_status(self):
        return self.__calibration_status

    @calibration_status.setter
    def calibration_status(self, c):
        if c:
            self.__calibration_status = 255
        else:
            self.__calibration_status = 0

    @property
    def sensor_detection(self):
        return self.__sensor_detection

    @sensor_detection.setter
    def sensor_detection(self, d):
        if d:
            self.__sensor_detection = chip_id_answer
        else:
            self.__sensor_detection = 0


def get_i2c_com():
    return FakeI2cCom()


class Bno055Test(unittest.TestCase):

    def test_polling(self):

        com = get_i2c_com()
        sensor = AbsoluteOrientation(com)
        pos = sensor.get_position()
        self.assertEqual(pos[0], 0.0)
        self.assertEqual(pos[0], 0.0)
        self.assertEqual(pos[1], 0.0)
        self.assertEqual(pos[2], 0.0)

        # >> Normalize angles
        # This generate absurdly big angles ~2040 degrees.
        com.default_value = 127
        pos = sensor.get_position()
        for p in pos:
            self.assertTrue(-180.0 <= p <= 180.0)

    def __test_events(self):

        sensor = AbsoluteOrientation(get_i2c_com())
        started = sensor.start_daemon()
        self.assertTrue(started)
        self.assertTrue(sensor.is_daemon_running)

        n_events = 0
        old_n_events = 0
        new_yaw = 0.0
        new_pitch = 0.0
        new_roll = 0.0

        def catch_events(**kwargs):
            nonlocal n_events
            nonlocal new_yaw
            nonlocal new_pitch
            nonlocal new_roll
            new_yaw = kwargs['yaw']
            new_pitch = kwargs['pitch']
            new_roll = kwargs['roll']
            n_events += 1

        event_bus.register_callback(EVENT_NAME, catch_events, NOT_URGENT, is_sync=True)

        while n_events < 3:
            new_event = 0 if old_n_events == n_events else 1
            if new_event:
                self.assertEqual(new_yaw, 0.0)
                self.assertEqual(new_pitch, 0.0)
                self.assertEqual(new_roll, 0.0)

            sleep(0.1)
            old_n_events = n_events

        sensor.stop_daemon()
        sleep(sensor._AbsoluteOrientation__delay * 2)
        self.assertFalse(sensor.is_daemon_running)

    def test_events(self):
        # Recursively try pushing events and stopping them.

        for i in range(3):
            # noinspection PyTypeChecker
            self.__test_events()

        # >> Check delivery of last data when daemon is running
        sensor = AbsoluteOrientation(get_i2c_com())
        sensor.start_daemon()
        sensor._AbsoluteOrientation__last_position = (12.0, 12.0, 12.0)
        p = sensor.get_position()
        self.assertEqual(p[0], 12.0)
        self.assertEqual(p[1], 12.0)
        self.assertEqual(p[2], 12.0)

    def test_constructor(self):
        """
        Test constructor options.

        1- delay= delay for events, must be an integer
        2- i2c_address= specify i2c_address of device
        :return:
        """

        # > delay
        self.assertRaises(RuntimeError, AbsoluteOrientation, FakeI2cCom(), delay=1)
        sensor1 = AbsoluteOrientation(get_i2c_com(), delay=0.2)
        sensor2 = AbsoluteOrientation(get_i2c_com(), delay=0.3)

        self.assertEqual(sensor1._AbsoluteOrientation__delay, 0.2)
        self.assertEqual(sensor2._AbsoluteOrientation__delay, 0.3)

        # > i2c_address

        with patch('absolute_orientation.Bno055._Bno055RemoteControl') as rc_class:
            com = get_i2c_com()
            self.assertRaises(RuntimeError, AbsoluteOrientation, com, i2c_address=0.0)
            sensor1 = AbsoluteOrientation(com, i2c_address=1)
            sensor2 = AbsoluteOrientation(com, i2c_address=2)
            self.assertTrue(rc_class.call_args_list[0] == ((com, 1),))
            self.assertTrue(rc_class.call_args_list[1] == ((com, 2),))

    def test_calibration(self):

        com = FakeI2cCom()

        # >> Remove previous calibration data
        try:
            os.remove(calib_data_file)
        except OSError:
            pass

        # >> Try to calibrate when sensor is not detected in the first place.
        com.sensor_detection = 0  # Fake sensor not detected on i2c bus
        sensor = AbsoluteOrientation(com)
        self.assertFalse(sensor.calibration_routine())
        com.sensor_detection = 1  # Connecting fake sensor back again

        # >> Load new sensor without calibration data
        sensor = AbsoluteOrientation(com)
        self.assertFalse(sensor.is_calibrated)

        # >> Failing calibration routine
        sensor._AbsoluteOrientation__calibration_samples = 1  # Forcing calibration to fail
        sensor.calibration_routine()
        sensor = AbsoluteOrientation(com)
        self.assertFalse(sensor.is_calibrated)  # Sensor not calibrated
        self.assertFalse(os.path.exists(calib_data_file))  # File not generated

        # >> Success calibration routine
        sensor = AbsoluteOrientation(com)
        com.calibration_status = 1  # Setting sensor to calibrated mode
        sensor.calibration_routine()
        self.assertTrue(sensor.is_calibrated)  # Sensor calibrated
        self.assertTrue(os.path.exists(calib_data_file))  # File generated

        # New sensor object loading data from previous calibration
        sensor = AbsoluteOrientation(com)
        self.assertTrue(sensor.is_calibrated)

    def test_ioerror_calib_file(self):
        """
        Test io error when writing calib file.
        Reading io error is covered in calibration test.
        """

        m = mock_open()
        com = get_i2c_com()

        # >> Remove previous calibration data
        try:
            os.remove(calib_data_file)
        except OSError:
            pass

        with patch('absolute_orientation.Bno055.open', m):
            m.side_effect = IOError

            sensor = AbsoluteOrientation(com)
            com.calibration_status = 1  # Setting sensor to calibrated mode
            sensor._AbsoluteOrientation__calibration_samples = 1  # Avoid calibration loops
            sensor.calibration_routine()
            self.assertFalse(sensor.is_calibrated)  # Sensor calibrated, False
            self.assertFalse(os.path.exists(calib_data_file))  # File generated, False

    def test_connection(self):

        com = FakeI2cCom()

        # >> Sensor disconnected
        com.sensor_detection = 0  # Fake sensor not connected to i2c
        sensor = AbsoluteOrientation(com)
        self.assertFalse(sensor.is_detected)
        self.assertFalse(sensor.is_daemon_running)

        init = sensor.init_sensor()
        self.assertFalse(init)  # init failure

        calib = sensor.calibration_routine()
        self.assertFalse(calib)  # calibration failure

        daemon = sensor.start_daemon()
        self.assertFalse(daemon)  # nothing works at this point

        self.assertRaises(RuntimeError, sensor.get_position)

        # >> Sensor connected now
        com.sensor_detection = 1
        sensor = AbsoluteOrientation(com)
        self.assertTrue(sensor.is_detected)
        self.assertFalse(sensor.is_daemon_running)

        sensor.start_daemon()
        self.assertTrue(sensor.is_daemon_running)

    @patch('absolute_orientation.Bno055.threading.Thread', autospec=True)
    def test_daemon_behavior(self, thread_mock_class):
        """
        Here we test some expected daemon behavior like not creating a second thread
        when daemon is already running but reporting as running.
        """

        thread_instance = Mock()
        thread_mock_class.return_value = thread_instance

        com = get_i2c_com()
        sensor = AbsoluteOrientation(com)

        # >> Starting daemon normally
        daemon = sensor.start_daemon()
        self.assertTrue(daemon)

        # >> Wrongly starting thread again
        daemon = sensor.start_daemon()
        self.assertTrue(daemon)

        # Checking that not new thread was created when already running daemon
        self.assertEqual(thread_mock_class.call_count, 1)  # Constructed once
        self.assertEqual(thread_instance.start.call_count, 1)  # Started once

    @patch('absolute_orientation.Bno055._Bno055RemoteControl')
    def test_datapath(self, rc_mock_class):

        # Injecting desired rc
        com = get_i2c_com()
        rc_instance = _Bno055RemoteControl(com, i2c_addr_sel_low)
        rc_mock_class.return_value = rc_instance

        # Changing path
        sensor = AbsoluteOrientation(com)
        sensor.set_datapath('../')

        # Check if correct
        self.assertEqual('../' + calib_data_file, rc_instance.CALIB_FILE)

    def test_i2c_address_change(self):

        com = Mock()
        sensor1 = AbsoluteOrientation(com, i2c_address=0x28)
        sensor2 = AbsoluteOrientation(com, i2c_address=0x29)

        self.assertEqual(com.set_i2c_address.call_args_list[0], call(0x28))
        self.assertEqual(com.set_i2c_address.call_args_list[1], call(0x29))


if __name__ == '__main__':
    unittest.main()
