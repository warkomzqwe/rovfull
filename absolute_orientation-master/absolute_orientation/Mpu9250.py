"""!
Module to communicate with Mpu9250 IMU Sensor SiP.
"""

import statistics
import threading
from time import sleep
from typing import Tuple

import numpy as np
from rov_logger.logger import get_rov_logger as __logger
from ruamel import yaml

from absolute_orientation.BitManipulation.BitManipulation import *
from absolute_orientation.IAbsoluteOrientation import I2CCom, IAbsoluteOrientation

""" General Parameters """
MAGNETOMETER_SAMPLES = 200
calib_filename = 'calib_data_mpu9250.yaml'

# ****************************************************************************
# *                                 Registers                                *
# ****************************************************************************

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

log = __logger()

# ****************************************************************************
# *                             Static Functions                             *
# ****************************************************************************


def normalize_angles(d):
    """
    Normalize angles between -180 and 180
    :param d: float value for angle in degrees
    :return: normalized value
    """

    while d > 180.0:
        d -= 360.0
    while d < -180.0:
        d += 360.0

    return d


# ****************************************************************************
# *                      General Orientation Calculator                      *
# ****************************************************************************

class OrientationCalc:
    """
    Helper class to transform mpu9250 data into physical orientation variables.

    Delivers yaw, pitch, roll according to acceletometer and magnetometer
    readings.
    """

    def __init__(self):
        self.__orientation = 'NED'
        self.acc_x = 0.0
        self.acc_y = 0.0
        self.acc_z = 0.0
        self.mag_x = 0.0
        self.mag_y = 0.0
        self.mag_z = 0.0
        self.yaw_adjustment = 0.0
        self.pitch_adjustment = 0.0
        self.roll_adjustment = 0.0
        self.yaw_factor = 1.0
        self.pitch_factor = 1.0
        self.roll_factor = 1.0

    def update_acc(self, acc_data: tuple):
        self.acc_x = acc_data[0]
        self.acc_y = acc_data[1]
        self.acc_z = acc_data[2]

    def update_mag(self, mag_data: tuple):
        self.mag_x = mag_data[0]
        self.mag_y = mag_data[1]
        self.mag_z = mag_data[2]

    def get_position(self) -> dict:
        """
        Get position in degrees.

        Equations are extracted from:
        Freescale Semiconductor Application Note Document Number: AN4248
        Implementing a Tilt-Compensated eCompass using Accelerometer and
        Magnetometer Sensors

        :return: A dictionary containing 'yaw', 'pitch' and 'roll' keys.
        """

        from math import atan2, pi, sin, cos

        self.acc_x = float(self.acc_x)
        self.acc_y = float(self.acc_y)
        self.acc_z = float(self.acc_z)
        self.mag_x = float(self.mag_x)
        self.mag_y = float(self.mag_y)
        self.mag_z = float(self.mag_z)

        """" Roll calculations """
        roll_rad = atan2(self.acc_y, self.acc_z)
        roll = 180 * roll_rad / pi

        """ Pitch calculations """
        pitch_rad = atan2(-self.acc_x, self.acc_y * sin(roll_rad) + self.acc_z * cos(roll_rad))
        pitch = 180 * pitch_rad / pi

        """
        Yaw calculations.

        Axis are changed to match physical misplacement of magnetometer SiP package.
        """
        mag_read_x = -self.mag_x
        mag_read_y = self.mag_y
        mag_read_z = self.mag_z

        mag_f_y_comp1 = mag_read_y * cos(roll_rad)
        mag_f_y_comp2 = -mag_read_z * sin(roll_rad)
        mag_f_y = mag_f_y_comp1 + mag_f_y_comp2

        mag_f_x_comp1 = mag_read_x * cos(pitch_rad)
        mag_f_x_comp2 = mag_read_y * sin(roll_rad) * sin(pitch_rad)
        mag_f_x_comp3 = mag_read_z * cos(roll_rad) * sin(pitch_rad)
        mag_f_x = mag_f_x_comp1 + mag_f_x_comp2 + mag_f_x_comp3

        yaw_rad = atan2(mag_f_y, mag_f_x)
        yaw = 180.0 * yaw_rad / pi

        """
        Return includes adjustments for internal team calibration.

        Yaw must be checked to obtain results inside a 180 to -180 degrees range.
        It should be fine using trigonometry but adjustments could change this.
        """

        yaw_adjusted = normalize_angles(yaw - self.yaw_adjustment)
        pitch_adjusted = normalize_angles(pitch - self.pitch_adjustment)
        roll_adjusted = normalize_angles(roll - self.roll_adjustment)

        return {
            'yaw': yaw_adjusted * self.yaw_factor,
            'pitch': pitch_adjusted * self.pitch_factor,
            'roll': roll_adjusted * self.roll_factor,
        }


# ****************************************************************************
# *                                Data Filter                               *
# ****************************************************************************


class RawDataFilter:
    """
    Generic filter for 3 axis data.

    Actual filter for data is settable.
    """

    def __init__(self, max_items):
        self.MAX_ITEMS = max_items  # Buffer max amount

        self.data_x = []
        self.data_y = []
        self.data_z = []

        """
        Filter function is settable.

        Functions takes an array of doubles and must perform some filtering action.
        Defaults to a median filter.
        """
        self.filter_function = np.median

    def append(self, data: tuple):
        self.data_x.append(data[0])
        self._check_max_items(self.data_x)
        self.data_y.append(data[1])
        self._check_max_items(self.data_y)
        self.data_z.append(data[2])
        self._check_max_items(self.data_z)

    def _check_max_items(self, data_list):
        if len(data_list) > self.MAX_ITEMS:
            data_list.pop(0)  # remove oldest item

    def get_filtered_data(self):
        return self.filter_function(self.data_x), \
               self.filter_function(self.data_y), \
               self.filter_function(self.data_z)


# ****************************************************************************
# *                         Magnetometer Calibration                         *
# ****************************************************************************


class MagnetometerCalibration:
    """
    Hard iron bias calibration is done acquiring max and min values along axis.
    With this data a middle point is reported to handle offsets for direct
    measured values.

    Soft iron bias calibration is done checking sensivitiy differences along axis.
    """

    def __init__(self):
        """
        Main responsibility is to handle a record of min and max values detected
        along different axis.

        Also with debugging purposes, data used is recorded.
        """

        # >> Default values to accomplish no intervention when uncalibrated.
        self.max_x = None
        self.min_x = None
        self.max_y = None
        self.min_y = None
        self.max_z = None
        self.min_z = None

        self.started = 0

        self.x_values = []
        self.y_values = []
        self.z_values = []

    def update(self, **kwargs):
        """
        Update object with new magnetometer raw readings.
        :param kwargs: Variable with x=, y=, z= values corresponding to magnetometer
        axis.
        """

        self.started = 1

        def check_none(internal, value):
            if internal is None:
                return value
            else:
                return internal

        for k, v in kwargs.items():
            if k == "x":

                self.x_values.append(v)

                self.max_x = check_none(self.max_x, v)
                self.min_x = check_none(self.min_x, v)

                if v > self.max_x:
                    self.max_x = v

                if v < self.min_x:
                    self.min_x = v

            elif k == "y":

                self.y_values.append(v)

                self.max_y = check_none(self.max_y, v)
                self.min_y = check_none(self.min_y, v)

                if v > self.max_y:
                    self.max_y = v

                if v < self.min_y:
                    self.min_y = v

            elif k == "z":

                self.z_values.append(v)

                self.max_z = check_none(self.max_z, v)
                self.min_z = check_none(self.min_z, v)

                if v > self.max_z:
                    self.max_z = v

                if v < self.min_z:
                    self.min_z = v

    def get_center_x(self):
        """
        Return detected center value for x axis.
        """
        return (self.max_x + self.min_x) / 2.0

    def get_center_y(self):
        """
        Return detected center value for y axis.
        """
        return (self.max_y + self.min_y) / 2.0

    def get_center_z(self):
        """
        Return detected center value for z axis.
        """
        return (self.max_z + self.min_z) / 2.0

    def get_stddev(self) -> tuple:
        """
        Return detected center value for x axis.
        """
        return (statistics.stdev(self.x_values),
                statistics.stdev(self.y_values),
                statistics.stdev(self.z_values)
                )

    def get_soft_iron_bias(self):
        """
        Return a tuple with soft iron bias for every axis x, y, z.

        """
        if not self.started:
            return 1.0, 1.0, 1.0

        x_scale = (self.max_x - self.min_y) / 2.0
        y_scale = (self.max_y - self.min_y) / 2.0
        z_scale = (self.max_z - self.min_z) / 2.0

        avg = (x_scale + y_scale + z_scale) / 3.0

        return (avg / x_scale if x_scale != 0.0 else 1.0,
                avg / y_scale if y_scale != 0.0 else 1.0,
                avg / z_scale if z_scale != 0.0 else 1.0
                )

    def load_from_file(self, filename):
        """
        Load values used to calculate axis middle points from a yaml file.
        """

        try:
            with open(filename, 'r') as fd:
                dic = yaml.safe_load(fd.read())

            self.max_x = dic['max_x']
            self.max_y = dic['max_y']
            self.max_z = dic['max_z']
            self.min_x = dic['min_x']
            self.min_y = dic['min_y']
            self.min_z = dic['min_z']

            self.started = True
            return True

        except KeyError:
            log.error("Calibration wrong file format.")
            self.started = False
            return False

        except IOError:
            log.error("Calibration file read error.")
            self.started = False
            return False

    def save_to_file(self, filename):
        """
        Save values used to calculate axis middle points to a yaml file.
        """

        dic = dict(
            max_x=self.max_x,
            max_y=self.max_y,
            max_z=self.max_z,
            min_x=self.min_x,
            min_y=self.min_y,
            min_z=self.min_z,
            x_values=self.x_values,
            y_values=self.y_values,
            z_values=self.z_values,
        )

        with open(filename, 'w') as fd:
            yaml_string = yaml.safe_dump(dic)
            fd.write(yaml_string)


# ****************************************************************************
# *           Remote Control for MPU9250 Absolute Orientation Sensor         *
# ****************************************************************************


class _MPU9250RemoteControl:
    """
    API to communicate with MPU9250 inertial measurement unit.

    Constructor expects a communication interface which implements the abstract
    class CommunicationInterfaceI2c:

    - __init__(i2c_address: int)
    - set_i2c_address(i2c_address: int)
    - write(register: int, value: int)
    - read_register(register: int) -> int
    """

    CALIB_FILE = calib_filename

    def __init__(self, com_interface: I2CCom):
        """
        Constructor just need a command interface.
        :param com_interface: Check main module notes.
        """
        self.com = com_interface

        """
        When invalid magnetometer data is read from sensors,
        we just deliver the last valid data.
        """
        self.last_valid_mag_data = (0.0, 0.0, 0.0)

        self.__verbose = True
        self.mag_calibration = MagnetometerCalibration()

    def __wait_for_mag_data(self):
        """ Wait for magnetometer data to be ready. """
        iic = self.com
        iic.set_i2c_address(AK8963)

        delay_time = 0.001

        while 1:
            data_ready = iic.read_register(ST1)
            if data_ready:
                break
            else:
                sleep(delay_time)

    def __check_mag_overflow(self):
        """
            Check data from ST2 register for overflow.

            This is mandatory according to datasheet. Data update is blocked until
            ST2 is read.
        """

        iic = self.com
        iic.set_i2c_address(AK8963)
        st2 = iic.read_register(ST2)
        st2_ba = to_bitarray(st2)
        st2_ba = bitarray_fill_8bit(st2_ba)
        overflow_mask = bitarray('00001000')
        overflow = st2_ba & overflow_mask
        return bool(bitarray_to_int(overflow))

    def __read_magnetometer_register(self, reg_h, reg_l):
        """
        Reads a magnetometer pair of registers. It delivers data in [µT]

        :param reg_h: High bits registers
        :param reg_l: Low bits register
        :return: Magnetometer reads in [µT]
        """

        iic = self.com
        iic.set_i2c_address(AK8963)
        scale_factor = 6.6693811074918566775  # Given directly by datasheet.

        mag_h = iic.read_register(reg_h)
        mag_l = iic.read_register(reg_l)
        mag_raw = twos_comp((mag_h << 8) + mag_l, 16)
        mag_scaled = mag_raw / scale_factor
        return mag_scaled

    def __calibrate_mag_reads(self, mag_x, mag_y, mag_z):
        """
        Calibrates raw magnetometer reads according to **hard iron** and
        **soft iron** biases.
        """
        soft_iron_bias = self.mag_calibration.get_soft_iron_bias()

        if self.mag_calibration.started:
            return (mag_x - self.mag_calibration.get_center_x()) * soft_iron_bias[0], \
                   (mag_y - self.mag_calibration.get_center_y()) * soft_iron_bias[1], \
                   (mag_z - self.mag_calibration.get_center_z()) * soft_iron_bias[2]

        else:
            return mag_x, mag_y, mag_z

    def reset(self):
        """ Reset routine for both MPU9250 and AK8963. """
        iic = self.com

        # First we reset the magnetometer
        iic.set_i2c_address(AK8963)

        reset_command = bitarray_to_int(bitarray('00000001'))
        iic.write(CNTL2, reset_command)
        sleep(0.1)
        log.info('Reset done.')

        # Then we reset the imu
        iic.set_i2c_address(MPU9250)

        reset_command = bitarray_to_int(bitarray('10000000'))
        iic.write(PWR_MGMT_1, reset_command)
        sleep(0.1)
        log.info('Reset IMU done.')

    def setup(self) -> bool:
        """ Setup routine.

        Setup routines is done in steps:
        1 - Check if mpu9250 is present
        2 - Enable magnetometer bypass
        3 - Set accelerometer sensitivity to +- 2g
        4 - Set accelerometer LPF to level 6 (matching 32.48ms delay)
        5 - Check if ak8963 is present
        6 - Set magnetometer output bit to 16bit
        7 - Set magnetometer operation to continuous mode 2
        """

        iic = self.com
        iic.set_i2c_address(MPU9250)

        """ 1 - Check if mpu9250 is present. """
        who = iic.read_register(WHOAMIADD)
        if who == WHOAMIADD_ANSWER:
            log.info('MPU9250 is present.')
        else:
            log.error('MPU9250 is NOT present.')
            return False

        """ 2 - Enable magnetometer bypass. """
        bypass_en = bitarray('1')
        bypass_en_mask = bitarray('00000010')
        bypass_en_shift = 1

        insert_bit_mask(iic, INTPIN, bypass_en, bypass_en_mask, bypass_en_shift)
        log.info('INTPIN: ' + to_bin(iic.read_register(INTPIN)) + '\n')
        log.info('Magnetometer enabled.')

        """ 3 - Set accelerometer sensitivity to +- 2g. """
        accel_fs_sel = bitarray('00')  # accel full scale = ±2g (maximum sensitivity)
        accel_fs_sel_mask = bitarray('00011000')
        accel_fs_sel_shift = 3

        insert_bit_mask(iic, ACCEL_CONFIG_1, accel_fs_sel, accel_fs_sel_mask,
                        accel_fs_sel_shift)

        """ 4 - Set accelerometer LPF to level 6 (matching 32.48ms delay). """
        acc_lpf_config = bitarray_fill_8bit(to_bitarray(6))
        acc_lpf_config_mask = bitarray('00001111')
        acc_lpf_config_shift = 0

        insert_bit_mask(iic, ACCEL_CONFIG_2, acc_lpf_config, acc_lpf_config_mask,
                        acc_lpf_config_shift)

        """ 5 - Check if ak8963 is present. """
        iic.set_i2c_address(AK8963)
        who = iic.read_register(WHOAMIADD_MAG)

        if who == WHOAMIADD_MAG_ANSWER:
            log.info('AK8693 is present.')
        else:
            log.error('AK8693 is NOT present.')
            return False

        """ 6 - Set magnetometer output bit to 16bit. """
        output_bit = bitarray('1')  # set to 1 for 16bit, 0 for 14bit
        output_bit_mask = bitarray('00010000')
        output_bit_shift = 4

        insert_bit_mask(iic, CNTL1, output_bit, output_bit_mask,
                        output_bit_shift)

        """ 7 - Set magnetometer operation to continuous mode 2. """
        mode = bitarray('0110')  # continuous measurement mode 2 (100 Hz)
        mode = bitarray_fill_8bit(mode)
        mode_mask = bitarray('00001111')
        mode_bitshift = 0

        insert_bit_mask(iic, CNTL1, mode, mode_mask, mode_bitshift)
        return True

    def get_acc_data(self):
        """
        Delivers accelerometer data in [m/s^2].

        :return: A 3 items tuple containing x, y and z data.
        """

        iic = self.com
        iic.set_i2c_address(MPU9250)
        scale_factor = 16384
        g = 9.80665  # gravity accel [m/s^2]

        """
        Accel data is in two separate register as
        high byte and low byte configuration.
        """

        def read_accelerometer_register(reg_h, reg_l):
            acc_h = iic.read_register(reg_h)
            acc_l = iic.read_register(reg_l)
            acc_raw = twos_comp((acc_h << 8) + acc_l, 16)
            acc_scaled = acc_raw / scale_factor * g
            return -acc_scaled

        acc_x = read_accelerometer_register(ACCEL_XOUT_H, ACCEL_XOUT_L)
        acc_y = read_accelerometer_register(ACCEL_YOUT_H, ACCEL_YOUT_L)
        acc_z = read_accelerometer_register(ACCEL_ZOUT_H, ACCEL_ZOUT_L)

        return acc_x, acc_y, acc_z

    def get_mag_data(self):
        """
        Delivers magnetometer data in [µT].

        :return: A 3 items tuple containing x, y and z data.
        """
        iic = self.com
        iic.set_i2c_address(AK8963)

        self.__wait_for_mag_data()

        """ Actually reads data from sensors. """

        mag_x = self.__read_magnetometer_register(HXH, HXL)
        mag_y = self.__read_magnetometer_register(HYH, HYL)
        mag_z = self.__read_magnetometer_register(HZH, HZL)

        """ Calibrate reads. """
        calibrated_data = self.__calibrate_mag_reads(mag_x, mag_y, mag_z)
        mag_x = calibrated_data[0]
        mag_y = calibrated_data[1]
        mag_z = calibrated_data[2]

        overflow = self.__check_mag_overflow()

        if overflow:
            if self.__verbose:
                log.debug('Magnetometer overflow...ignoring data.')
            return self.last_valid_mag_data

        else:
            self.last_valid_mag_data = (mag_x, mag_y, mag_z)
            return self.last_valid_mag_data

    def upload_magnetometer_calibration(self):
        """
        UpLoad previously saved magnetometer data for calibration.
        """
        return self.mag_calibration.load_from_file(self.CALIB_FILE)

    def save_magnetometer_calibration(self):
        """
        Save previously acquired calibration data.
        """
        self.mag_calibration.save_to_file(self.CALIB_FILE)

    def run_mag_calibration(self):
        """
        Run a simplified calibration routine.

        It's a blocking method and it takes around 30 seconds to complete. Sensor
        should be moved around making an 8 figure in air.
        """

        iic = self.com
        iic.set_i2c_address(AK8963)

        mag_filter = RawDataFilter(5)
        mag_filter.filter_function = np.median

        for i in range(0, MAGNETOMETER_SAMPLES):

            # print("Running sample: %i" % i)
            self.__wait_for_mag_data()

            mag_x = self.__read_magnetometer_register(HXH, HXL)
            mag_y = self.__read_magnetometer_register(HYH, HYL)
            mag_z = self.__read_magnetometer_register(HZH, HZL)

            overflow = self.__check_mag_overflow()
            if overflow:
                return 0

            else:
                mag_filter.append((mag_x, mag_y, mag_z))
                filtered_data = mag_filter.get_filtered_data()

                self.mag_calibration.update(x=filtered_data[0].item(),
                                            y=filtered_data[1].item(),
                                            z=filtered_data[2].item())

        calib_data_stddev = self.mag_calibration.get_stddev()
        log.debug("Standard deviation for magnetometer calibration data: %r" % str(calib_data_stddev))
        return 1


# ****************************************************************************
# *                     Absolute orientation API Exposure                    *
# ****************************************************************************


class AbsoluteOrientation(IAbsoluteOrientation):
    """!
    Mpu9250 adapter to match Absolute Orientation API.
    """

    def __init__(self, com: I2CCom, delay=0.1):
        """!
        Implementation of IAbsoluteOrientation for Mpu9250.

        @param com Api for physical i2c communication.
        @param delay Delay in seconds for absolute orientation events, must be an integer (Default 0.1).

        """
        super().__init__(com)
        self._rc = _MPU9250RemoteControl(com)

        self.__is_detected = False
        self.__is_calibrated = False
        self.__should_run = False
        self.__thread = None
        self.__last_position = None

        # > delay
        if not isinstance(delay, float):
            msg = "Delay must be a float."
            log.error(msg)
            raise RuntimeError(msg)

        self.__delay = delay

        # >> Autostart
        self.init_sensor()

    @property
    def is_detected(self) -> bool:
        return self.__is_detected

    @property
    def is_calibrated(self) -> bool:
        return self.__is_calibrated

    def calibration_routine(self) -> bool:

        if self.is_detected:
            self.__is_calibrated = self._rc.run_mag_calibration()
            self._rc.save_magnetometer_calibration() if self.is_calibrated else None
            return self.is_calibrated
        else:
            log.error("Sensor not detected.")
            self.__is_calibrated = False
            return False

    def init_sensor(self) -> bool:

        # >> Sensor setup
        is_present = self._rc.setup()
        if is_present:
            self._rc.reset()
            sleep(0.1)  # 100ms max reset time
            setup_success = self._rc.setup()
            self.__is_detected = setup_success
        else:
            log.error("Sensor not detected.")
            self.__is_detected = False
            return False

        # >> Sensor upload calibration data
        self.__is_calibrated = self._rc.upload_magnetometer_calibration()
        return self.is_calibrated

    def get_position(self) -> Tuple[float, float, float]:
        """
        Single time orientation access.

        MPU9250 configured with low pass filter for acceletometer. Not present for
        magnetometer so it has to be handled internally.
        :return: A tuple containing yaw, pitch, roll respectively in NED world frame.
        """

        if not self.is_detected:
            msg = "Sensor not detected."
            log.error(msg)
            raise RuntimeError(msg)

        filter_acc = RawDataFilter(1)
        filter_mag = RawDataFilter(3)
        calc = OrientationCalc()

        for i in range(3):
            m = self._rc.get_mag_data()
            filter_mag.append(m)

        acc = self._rc.get_acc_data()
        filter_acc.append(acc)
        mag = filter_mag.get_filtered_data()
        acc = filter_acc.get_filtered_data()

        calc.update_acc(acc)
        calc.update_mag(mag)

        dic = calc.get_position()
        return dic['yaw'], dic['pitch'], dic['roll']

    def set_datapath(self, path: str):
        self._rc.CALIB_FILE = path + calib_filename

