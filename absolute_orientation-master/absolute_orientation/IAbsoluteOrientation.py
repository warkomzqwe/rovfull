"""!
Main interfaces for absolute orientation API.
"""

from typing import Tuple
from abc import ABC, abstractmethod


class I2CCom(ABC):
    """!
    Abstract class to define expected implementation to communicate through i2c.

    Developers must implement this interface according to their physical context.
    """

    @abstractmethod
    def __init__(self):
        """! Default constructor.
        """

    @abstractmethod
    def set_i2c_address(self, i2c_address: int):
        """!
        Possibility to change desired device address.

        @param i2c_address Desired iic address device to communicate with.
        """

    @abstractmethod
    def write(self, register: int, value: int):
        """!
        Write to device internal register.

        @param register Internal register of device.
        @param value 8bit value
        """

    @abstractmethod
    def read_register(self, register: int) -> int:
        """!
        Read device internal register.

        @param register Internal register of device
        @return 8bit register value
        """

    @abstractmethod
    def read_register_burst(self, register, n) -> list:
        """!
        Read n consecutive registers.

        @param register Starting internal register of device
        @param n Amount of registers to read
        @return A list of 8 bit register values [i2c_address, reg, value0, value1...]
        """


class IAbsoluteOrientation(ABC):
    """!
    General Interface to access final orientation sensors data.
    """

    @abstractmethod
    def __init__(self, com: I2CCom, **kwargs):
        """!
        Constructor needs a i2c communication interface.

        Library needs an actual I2C Interface API depending of the physical
        implementation in the machine/computer/embedded system to be used.
        @param com Api for physical i2c communication.
        @param kwargs Options: **delay** for absolute orientation event delays;
        """

    @property
    @abstractmethod
    def is_detected(self) -> bool:
        """!
        ***Property*** Check if sensor is presented.
        @return True if sensor answered without errors.
        """

    @property
    @abstractmethod
    def is_calibrated(self) -> bool:
        """!
        ***Property*** Check if sensor is calibrated.
        @return True if sensor is currently calibrated.
        """

    @abstractmethod
    def calibration_routine(self) -> bool:
        """!
        Run calibration routine.

        Method is blocking and takes time, must be considered.
        @return True if successful calibration.
        """

    @abstractmethod
    def init_sensor(self) -> bool:
        """!
        Steps to configure chip for continuous usage and read/upload calibration data.
        @return True if successful on all init steps.
        """

    @abstractmethod
    def get_position(self) -> Tuple[float, float, float]:
        """!
        Get current position.

        @return A tuple containing yaw, pitch, roll respectively in NED world frame.
        @exception RunTimeError when sensor is not detected/configured.
        """

    @abstractmethod
    def set_datapath(self, path: str):
        """!
        Define path to save data.

        Calibration and configuration are saved/loaded at/from specific path.
        """
