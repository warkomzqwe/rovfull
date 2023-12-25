import unittest
from unittest.mock import Mock
from absolute_orientation import get_bno055, get_mpu9250
from absolute_orientation.Bno055 import AbsoluteOrientation as Bno055Api
from absolute_orientation.Mpu9250 import AbsoluteOrientation as Mpu9250Api


class SensorFactoryTest(unittest.TestCase):

    def test_factory(self):
        com = Mock()

        bno = get_bno055(com)
        mpu = get_mpu9250(com)
        self.assertTrue(isinstance(bno, Bno055Api))
        self.assertTrue(isinstance(mpu, Mpu9250Api))

        bno = get_bno055(com, delay=0.2)
        mpu = get_mpu9250(com, delay=0.2)
        self.assertTrue(isinstance(bno, Bno055Api))
        self.assertTrue(isinstance(mpu, Mpu9250Api))


if __name__ == '__main__':
    unittest.main()
