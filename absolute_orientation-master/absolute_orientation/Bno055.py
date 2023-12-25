"""!
Module to communicate with Bno055 Absolute Sensor SiP.
"""

from typing import Tuple
import threading
from ruamel import yaml
from time import sleep
from absolute_orientation.IAbsoluteOrientation import I2CCom, IAbsoluteOrientation
from rov_logger.logger import get_rov_logger
from absolute_orientation.BitManipulation.BitManipulation import *

# >> Parameters
calib_filename = 'calib_data_bno055.yaml'
log = get_rov_logger()

# ****************************************************************************
# *                                 Registers                                *
# ****************************************************************************

# I2C Addresses
i2c_addr_sel_low = 0x28
i2c_addr_sel_high = 0x29

# BNO055 Registers
chip_id = 0x0
opr_mode = 0x3d
axis_map_config = 0x41
axis_map_sign = 0x42
calib_stat = 0x35
eul_heading_msb = 0x1b
eul_heading_lsb = 0x1a
eul_roll_msb = 0x1d
eul_roll_lsb = 0x1c
eul_pitch_msb = 0x1f
eul_pitch_lsb = 0x1e
qua_data_w_lsb = 0x20
acc_offset_x_lsb = 0x55

# Expected answers
chip_id_answer = 0xa0


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


def join_signed_bytes(msb, lsb):
    res = (msb << 8) + lsb
    return twos_comp(res, 16)


# ****************************************************************************
# *           Remote Control for BNO055 Absolute Orientation Sensor          *
# ****************************************************************************


class _Bno055RemoteControl:
    """
    Remote Control class for BNO055.

    List of different available operations for SiP.
    """

    CALIB_FILE = calib_filename

    def __init__(self, com: I2CCom, i2c_address):
        self.com = com
        self.log = get_rov_logger()
        self.yaw_offset = 0.0
        self.roll_offset = 0.0
        self.pitch_offset = 0.0
        self.i2c_address = i2c_address

    def _set_operation_mode_config(self):
        iic = self.com

        config_mode = bitarray('0000')
        opr_mode_mask = bitarray('00001111')
        insert_bit_mask(iic, opr_mode, config_mode, opr_mode_mask, 0)
        self.log.info("OPR_MODE set to CONFIG_MODE.")
        sleep(0.038)  # Time required to change from ndof to config mode, at least 19ms

    def _set_operation_mode_ndof(self):
        iic = self.com
        opr_mode_mask = bitarray('00001111')
        ndof_mode = bitarray('1100')
        insert_bit_mask(iic, opr_mode, ndof_mode, opr_mode_mask, 0)
        self.log.info("OPR_MODE set to NDOF.")
        sleep(0.014)  # Time needed to change from config mode to ndof at least 7ms

    def _get_calibration_status(self):
        iic = self.com
        calib_stat_value = iic.read_register(calib_stat)
        calib_stat_value = to_bitarray(calib_stat_value)
        calib_stat_value = bitarray_fill_8bit(calib_stat_value)
        sys_calib = rightshift(calib_stat_value, 6)
        gyr_calib = rightshift(bitarray('00110000') & calib_stat_value, 4)
        acc_calib = rightshift(bitarray('00001100') & calib_stat_value, 2)
        mag_calib = bitarray('00000011') & calib_stat_value

        sys_calib = bitarray_to_int(sys_calib)
        gyr_calib = bitarray_to_int(gyr_calib)
        acc_calib = bitarray_to_int(acc_calib)
        mag_calib = bitarray_to_int(mag_calib)

        s = "sys_calib: %d; gyr_calib: %d; acc_calib: %d; mag_calib: %d" % (
            sys_calib, gyr_calib, acc_calib, mag_calib
        )
        self.log.debug(s)

        total_calibration = (sys_calib == 3 and gyr_calib == 3 and
                             acc_calib == 3 and mag_calib == 3)

        return total_calibration, sys_calib, gyr_calib, acc_calib, mag_calib

    def _get_calibration_data(self):

        iic = self.com
        data = iic.read_register_burst(acc_offset_x_lsb, 22)
        return data

    def setup(self):

        iic = self.com
        iic.set_i2c_address(self.i2c_address)

        # >>> 1. Check if bno055 is present
        chip_id_value = iic.read_register(chip_id)

        if chip_id_value == chip_id_answer:
            self.log.info("BNO055 detected.")

        else:
            self.log.warning("BNO055 not detected, exit.")
            return 0

        # >>> 2. Back to CONFIG_MODE just in case
        self._set_operation_mode_config()

        # >>> 3. Config axis remap
        # Default reference is ENU,
        # so we change z->-z, y->x, x-> y

        axis_map_config_setting = bitarray('100001')  # 0x21
        # axis_map_config_setting = bitarray('100100')  # 0x24
        mask = bitarray('00111111')
        insert_bit_mask(iic, axis_map_config, axis_map_config_setting, mask, 0)

        xyz_sign_values = bitarray('100')
        mask = bitarray('00000111')
        insert_bit_mask(iic, axis_map_sign, xyz_sign_values, mask, 0)
        self.log.info("Setting world reference to NED convention.")

        axis_remap_values = iic.read_register(axis_map_config)
        axis_sign_values = iic.read_register(axis_map_sign)
        self.log.debug("Axis map values: %s" % to_hex(axis_remap_values))
        self.log.debug("Axis sign values: %s" % to_hex(axis_sign_values))

        # >>> 4. Set OPR_MODE to fusion mode NDOF_FMC_OFF
        self._set_operation_mode_ndof()

        # >>> Debug axis remap config
        axis_config = iic.read_register(axis_map_config)
        axis_config = to_bitarray(axis_config)
        axis_sign = iic.read_register(axis_map_sign)
        axis_sign = to_bitarray(axis_sign)
        self.log.debug('Confirming axis configuration; AXIS_MAP_CONFIG: %s; AXIS_MAP_SIGN: %s'
                       % (axis_config, axis_sign))

        return 1

    def get_absolute_orientation(self) -> dict:
        """
        Absolute orientation for BNO055.

        Sensor works with ENU world frame so an internal conversion to NED is needed.

        :return: dict with Orientation in NED world frame (yaw, pitch, roll keys).
        """
        iic = self.com

        data = iic.read_register_burst(eul_heading_lsb, 6)

        yaw_lsb = data[0]
        yaw_msb = data[1]
        yaw = join_signed_bytes(yaw_msb, yaw_lsb)
        yaw = float(yaw) / 16.0

        roll_lsb = data[2]
        roll_msb = data[3]
        roll = join_signed_bytes(roll_msb, roll_lsb)
        roll = float(roll) / 16.0

        pitch_lsb = data[4]
        pitch_msb = data[5]
        pitch = join_signed_bytes(pitch_msb, pitch_lsb)
        pitch = float(pitch) / 16.0

        # Return ENU to NED converted values.
        yaw = yaw
        roll = -roll
        pitch = -pitch

        return dict(
            yaw=normalize_angles(yaw),
            pitch=normalize_angles(pitch),
            roll=normalize_angles(roll),
        )

    # def get_quaternions(self):
    #
    #     iic = self.com
    #     data = iic.read_register_burst(qua_data_w_lsb, 8)
    #     quaternion_unit_factor = 16384.0
    #
    #     w_lsb = data[0]
    #     w_msb = data[1]
    #
    #     x_lsb = data[2]
    #     x_msb = data[3]
    #
    #     y_lsb = data[4]
    #     y_msb = data[5]
    #
    #     z_lsb = data[6]
    #     z_msb = data[7]
    #
    #     w = join_signed_bytes(w_msb, w_lsb) / quaternion_unit_factor
    #     x = join_signed_bytes(x_msb, x_lsb) / quaternion_unit_factor
    #     y = join_signed_bytes(y_msb, y_lsb) / quaternion_unit_factor
    #     z = join_signed_bytes(z_msb, z_lsb) / quaternion_unit_factor
    #
    #     return w, x, y, z

    def save_calibration_data(self):
        """
        Detects if internal chip calibration is done and saves data to file.

        :return: 1 when internal calibration status is ok and data is saved,
        """

        calibration_status = self._get_calibration_status()
        total_calibration = calibration_status[0]
        sys_calib = calibration_status[1]
        gyr_calib = calibration_status[2]
        acc_calib = calibration_status[3]
        mag_calib = calibration_status[4]

        if total_calibration:
            self.log.info("*** Calibration completed ***")
            data = self._get_calibration_data()

            try:
                with open(self.CALIB_FILE, "w") as f:
                    f.write(yaml.safe_dump(data))
                    return 1

            except IOError:
                self.log.error("Error writing calibration data file.")
                return 0

        else:
            self.log.info("Calibration routine isn't completed (Max level 3): \n"
                          "sys: %s; gyr: %s; acc: %s; mag: %s" %
                          (sys_calib, gyr_calib, acc_calib, mag_calib))
            return 0

    def upload_calibration_data(self):

        iic = self.com
        register_pointer = acc_offset_x_lsb

        try:
            with open(self.CALIB_FILE, "r") as fd:

                yaml_string = fd.read()
                data = yaml.safe_load(yaml_string)

                # Config mode needed for setting calibration values.
                self._set_operation_mode_config()

                # Write actual data
                for d in data:
                    iic.write(register_pointer, d)
                    register_pointer += 1

                # Back to ndof mode
                self._set_operation_mode_ndof()
                return 1

        except IOError:
            self.log.error("Error reading calibration data file.")
            return 0


# ****************************************************************************
# *                     Absolute orientation API Exposure                    *
# ****************************************************************************


class AbsoluteOrientation(IAbsoluteOrientation):
    """!
    Bno055 adapter to match Absolute Orientation API.
    """

    def __init__(self, com: I2CCom, delay=0.1, i2c_address=i2c_addr_sel_low):
        """!
        Implementation of IAbsoluteOrientation for Bno055.

        @param com Api for physical i2c communication.
        @param delay Delay in seconds for absolute orientation events, must be an integer (Default 0.1).
        @param i2c_address Specify i2c_address of device (Default 0x28).

        """
        super().__init__(com)

        self.__is_detected = False
        self.__is_calibrated = False
        self.__should_run = False
        self.__last_position = None  # tuple expected

        self.__calibration_samples = 1000

        # >> kwargs handling

        # > delay
        if not isinstance(delay, float):
            msg = "Delay must be a float."
            log.error(msg)
            raise RuntimeError(msg)

        self.__delay = delay

        # > i2c address
        if not isinstance(i2c_address, int):
            msg = 'I2C Address must be an int.'
            log.error(msg)
            raise RuntimeError(msg)

        # >> Initializing sensor
        self.__rc = _Bno055RemoteControl(com, i2c_address)
        self.init_sensor()
        log.info('Sensor is calibrated: %r' % self.is_calibrated)

    @property
    def is_detected(self) -> bool:
        return self.__is_detected

    @property
    def is_calibrated(self) -> bool:
        return self.__is_calibrated

    def calibration_routine(self) -> bool:

        if not self.is_detected:
            log.error("Sensor is not working.")
            return False

        for i in range(0, self.__calibration_samples):
            if self.__rc.save_calibration_data():
                #  If successful acquiring calibration data
                self.__is_calibrated = True
                return True
            sleep(0.1)

        log.error("*** Calibration NOT completed. ***")
        return False

    def init_sensor(self) -> bool:

        # >> 1. setup routine
        setup = self.__rc.setup()

        if not setup:
            self.__is_detected = False
            self.__is_calibrated = False
            return False

        self.__is_detected = True

        # >> 2. last calibration data upload
        calib = self.__rc.upload_calibration_data()

        if not calib:
            log.error("Calibration data not uploaded. Are you sure calibration was done?.")
            log.info("Sensor is working but uncalibrated.")
            self.__is_calibrated = False
            return False

        self.__is_calibrated = True
        return True

    def get_position(self) -> Tuple[float, float, float]:

        if not self.is_detected:
            log.error("Sensor is not working.")
            raise RuntimeError('Sensor not working')

        dic = self.__rc.get_absolute_orientation()
        return (dic['yaw'],
                dic['pitch'],
                dic['roll'])

    def set_datapath(self, path: str):
        self.__rc.CALIB_FILE = path + calib_filename
