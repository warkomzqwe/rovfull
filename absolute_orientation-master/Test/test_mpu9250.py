import os
import unittest
from collections import defaultdict
from threading import Thread
from time import sleep

from rov_event_bus.bus import event_bus, NOT_URGENT

from absolute_orientation.IAbsoluteOrientation import I2CCom
from absolute_orientation.Mpu9250 import AbsoluteOrientation, \
    normalize_angles, MagnetometerCalibration, _MPU9250RemoteControl

from unittest.mock import patch, mock_open

priority = NOT_URGENT
calib_data_file = 'calib_data_mpu9250.yaml'
EVENT_NAME = 'absolute_orientation_event'
# log = get_rov_logger("INFO")

""" I2C Addresses """
MPU9250 = 0x68
AK8963 = 0x0c

""" MPU9250 Registers"""
WHOAMIADD = 0x75
PWR_MGMT_1 = 0x6b
INTPIN = 0x37
ACCEL_CONFIG_1 = 0x1c
ACCEL_CONFIG_2 = 0x1d
ACCEL_XOUT_H = 0x3b
ACCEL_XOUT_L = 0x3c
ACCEL_YOUT_H = 0x3d
ACCEL_YOUT_L = 0x3e
ACCEL_ZOUT_H = 0x3f
ACCEL_ZOUT_L = 0x40

""" AK8693 Magnetometer Registers """
WHOAMIADD_MAG = 0x00
CNTL1 = 0x0a
CNTL2 = 0x0b
INFO = 0x01
ST1 = 0x02
HXL = 0x03
HXH = 0x04
HYL = 0x05
HYH = 0x06
HZL = 0x07
HZH = 0x08
ST2 = 0x09
ASTC = 0x0c

""" Expected Answers """
WHOAMIADD_ANSWER = 0x71
WHOAMIADD_MAG_ANSWER = 0x48


class FakeI2cCom(I2CCom):
    def __init__(self):
        super().__init__()
        regs = defaultdict(lambda: 0)
        self.regs = regs
        self.blocked = 0

        # >> Register values
        regs[WHOAMIADD] = WHOAMIADD_ANSWER
        regs[WHOAMIADD_MAG] = WHOAMIADD_MAG_ANSWER
        regs[ST1] = 1
        regs[ST2] = 0
        regs[ACCEL_XOUT_H] = 255
        regs[ACCEL_XOUT_L] = 0
        regs[ACCEL_YOUT_H] = 0
        regs[ACCEL_YOUT_L] = 0
        regs[ACCEL_ZOUT_H] = 0
        regs[ACCEL_ZOUT_L] = 0
        regs[HXL] = 0
        regs[HXH] = 0
        regs[HYL] = 255
        regs[HYH] = 0
        regs[HZL] = 0
        regs[HZH] = 0

    def set_i2c_address(self, i2c_address: int):
        pass

    def write(self, register: int, value: int):
        pass

    def read_register(self, register: int) -> int:
        if self.blocked:
            sleep(20)
            return 0
        else:
            return self.regs[register]

    def read_register_burst(self, register, n) -> list:
        data = []
        r = register
        for i in range(n):
            data.append(self.read_register(r))
            r += 1
        return data

    @property
    def sensor_detection(self):
        return self.regs[WHOAMIADD] == WHOAMIADD_ANSWER

    @property
    def mag_detection(self):
        return self.regs[WHOAMIADD_MAG] == WHOAMIADD_MAG_ANSWER

    @sensor_detection.setter
    def sensor_detection(self, v):
        if v:
            self.regs[WHOAMIADD] = WHOAMIADD_ANSWER
        else:
            self.regs[WHOAMIADD] = 0

    @mag_detection.setter
    def mag_detection(self, v):
        if v:
            self.regs[WHOAMIADD_MAG] = WHOAMIADD_MAG_ANSWER
        else:
            self.regs[WHOAMIADD_MAG] = 0


def get_i2c_com():
    return FakeI2cCom()


def get_api(com):
    return AbsoluteOrientation(com)


class Mpu9250Test(unittest.TestCase):
    def_yaw = 180.0
    def_pitch = -90.0
    def_roll = 0.0

    def test_polling(self):

        com = get_i2c_com()
        sensor = AbsoluteOrientation(com)
        pos = sensor.get_position()
        self.assertEqual(pos[0], self.def_yaw)
        self.assertEqual(pos[1], self.def_pitch)
        self.assertEqual(pos[2], self.def_roll)

        # Test sensor not detected but asking for position
        com = get_i2c_com()
        com.sensor_detection = 0
        sensor = get_api(com)
        self.assertRaises(RuntimeError, sensor.get_position)

        # >> Test magnetometer delivering data when overflow
        # Specific MPU9250 behavior
        com = get_i2c_com()
        sensor = get_api(com)

        pos = sensor.get_position()
        self.assertEqual(pos[0], self.def_yaw)
        self.assertEqual(pos[1], self.def_pitch)
        self.assertEqual(pos[2], self.def_roll)

        com.regs[ST2] = 8  # Magnetometer overflow

        pos = sensor.get_position()
        self.assertEqual(pos[0], self.def_yaw)
        self.assertEqual(pos[1], self.def_pitch)
        self.assertEqual(pos[2], self.def_roll)

        # Test polling when events are running
        com = get_i2c_com()
        sensor = get_api(com)
        sensor.start_daemon()
        sleep(0.2)

        # "Move" sensor position,
        # but sensor data should not be read directly from
        # registers when events are running.
        com.blocked = 1
        pos = sensor.get_position()
        self.assertEqual(pos[0], self.def_yaw)
        self.assertEqual(pos[1], self.def_pitch)
        self.assertEqual(pos[2], self.def_roll)

        # Test angle normalization used internally
        self.assertTrue(-180.0 <= normalize_angles(181.0) <= 180.0)
        self.assertTrue(-180.0 <= normalize_angles(-181.0) <= 180.0)
        self.assertTrue(-180.0 <= normalize_angles(892.0) <= 180.0)

        # >> Test wait for magnetometer data ready
        # This is specific behavior for MPU250
        com = get_i2c_com()
        sensor = get_api(com)
        com.regs[ST1] = 0

        def read_mag():
            return sensor.get_position()

        t = Thread(target=read_mag)
        t.daemon = True
        t.start()
        sleep(0.2)

        self.assertTrue(t.is_alive())

        com.regs[ST1] = 1
        sleep(0.2)

        self.assertFalse(t.is_alive())

        # >> Polling without calibration should be successful
        # >> Remove previous calibration data
        try:
            os.remove(calib_data_file)
        except OSError:
            pass

        sensor = get_api(get_i2c_com())
        sensor.get_position()

    def __test_events(self, sensor):

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

        event_bus.register_callback(EVENT_NAME, catch_events, NOT_URGENT, is_sync=False)

        while n_events < 3:
            new_event = 0 if old_n_events == n_events else 1
            if new_event:
                self.assertEqual(new_yaw, self.def_yaw)
                self.assertEqual(new_pitch, self.def_pitch)
                self.assertEqual(new_roll, self.def_roll)

            sleep(0.1)
            old_n_events = n_events

        sensor.stop_daemon()
        event_bus.unregister_callback(EVENT_NAME, catch_events)
        sleep(sensor._AbsoluteOrientation__delay * 4)
        self.assertFalse(sensor.is_daemon_running)

    def test_events(self):
        # Recursively try pushing events and stopping them.
        sensor = get_api(get_i2c_com())

        for i in range(3):
            # noinspection PyTypeChecker
            self.__test_events(sensor)

    # noinspection PyUnresolvedReferences
    def test_constructor(self):
        """
        Test constructor options.

        1- delay= delay for events, must be a float
        """

        self.assertRaises(RuntimeError, AbsoluteOrientation, FakeI2cCom(), delay=1)
        sensor1 = AbsoluteOrientation(get_i2c_com(), delay=0.2)
        sensor2 = AbsoluteOrientation(get_i2c_com(), delay=0.3)

        self.assertEqual(sensor1._AbsoluteOrientation__delay, 0.2)
        self.assertEqual(sensor2._AbsoluteOrientation__delay, 0.3)

    def test_calibration_routine(self):

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

        # >> Success calibration routine
        sensor = AbsoluteOrientation(com)
        sensor.calibration_routine()
        self.assertTrue(sensor.is_calibrated)  # Sensor calibrated
        self.assertTrue(os.path.exists(calib_data_file))  # File generated

        # >> Magnetometer overflow should be reported as a failure for
        # calibration routine
        com.regs[ST2] = 8
        sensor.calibration_routine()
        self.assertFalse(sensor.is_calibrated)  # Sensor calibrated

        # New sensor object loading data from previous calibration
        sensor = AbsoluteOrientation(com)
        self.assertTrue(sensor.is_calibrated)

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

        # >> Sensor connected now
        com.sensor_detection = 1
        sensor = AbsoluteOrientation(com)
        self.assertTrue(sensor.is_detected)
        self.assertFalse(sensor.is_daemon_running)

        sensor.start_daemon()
        self.assertTrue(sensor.is_daemon_running)

        # >> Magnetometer connection also could potentially fail
        com.sensor_detection = 1
        com.mag_detection = 0
        sensor = get_api(com)
        self.assertFalse(sensor.is_detected)

    # noinspection PyUnresolvedReferences
    def test_daemon_behavior(self):
        """
        Here we test some expected daemon behavior like not creating a second thread
        when daemon is already running but reporting as running.
        """

        com = get_i2c_com()
        sensor = AbsoluteOrientation(com)

        # >> Starting daemon normally
        daemon = sensor.start_daemon()
        self.assertTrue(daemon)

        t1 = sensor._AbsoluteOrientation__thread

        # >> Wrongly starting thread again
        daemon = sensor.start_daemon()
        self.assertTrue(daemon)

        t2 = sensor._AbsoluteOrientation__thread

        # Checking that not new thread was created when already running daemon
        self.assertEqual(t1, t2)

    def test_magnetometer_calibration(self):
        """
        Mpu needs manetometer hard and iron bias calibration.

        Class used is MagnetometerCalibration.
        """

        mag_calib = MagnetometerCalibration()
        self.assertFalse(mag_calib.started)

        mag_calib.update(x=1.0, y=1.0, z=1.0)
        mag_calib.update(x=2.0, y=2.0, z=2.0)
        self.assertTrue(mag_calib.started)
        mag_calib.update(x=-1.0, y=-1.0, z=-1.0)
        mag_calib.update(x=-2.0, y=-2.0, z=-2.0)

        self.assertEqual(mag_calib.get_center_x(), 0.0)
        self.assertEqual(mag_calib.get_center_y(), 0.0)
        self.assertEqual(mag_calib.get_center_z(), 0.0)
        self.assertEqual(mag_calib.get_soft_iron_bias(),
                         (1.0, 1.0, 1.0))

    @patch('absolute_orientation.Mpu9250.yaml')
    def test_wrong_calib_file(self, yaml_class):
        """
        Error reading calibration file should be ignored.
        Calibration not done but system should be keep working.
        """

        # >> wrong dic after reading file
        dic = dict(
            maxx=1.0
        )

        # To avoid actually open a file, we use a mock
        open_ = mock_open()

        with patch('absolute_orientation.Mpu9250.open', open_):
            yaml_class.safe_load.return_value = dic

            # Api tries to load calibration data at start
            # but fails
            sensor = get_api(get_i2c_com())
            self.assertTrue(sensor.is_detected)
            self.assertFalse(sensor.is_calibrated)

    @patch('absolute_orientation.Mpu9250._MPU9250RemoteControl')
    def test_datapath(self, rc_mock_class):

        # Injecting desired rc
        com = get_i2c_com()
        rc_instance = _MPU9250RemoteControl(com)
        rc_mock_class.return_value = rc_instance

        # Changing path
        sensor = get_api(com)
        sensor.set_datapath('../')

        # Check if correct
        self.assertEqual('../' + calib_data_file, rc_instance.CALIB_FILE)


if __name__ == '__main__':
    unittest.main()
