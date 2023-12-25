"""!
Simple factory to access Absolute orientation API for different sensors.

@example bno055_example.py
"""

from absolute_orientation.Mpu9250 import AbsoluteOrientation as Mpu9250_API
from absolute_orientation.Bno055 import AbsoluteOrientation as Bno055_API
from absolute_orientation.IAbsoluteOrientation import I2CCom


def get_bno055(com: I2CCom, **kwargs):
    """!
    Returns an Absolute Orientation API implementation for Bno055.
    """
    return Bno055_API(com, **kwargs)


def get_mpu9250(com: I2CCom, **kwargs):
    """!
    Returns an Absolute Orientation API implementation for Mpu9250.
    """
    return Mpu9250_API(com, **kwargs)


