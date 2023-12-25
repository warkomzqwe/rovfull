
from pymata_aio.constants import Constants as Ct
from pymata_aio.pymata3 import PyMata3
import asyncio

from absolute_orientation.IAbsoluteOrientation import I2CCom

loop = asyncio.get_event_loop()


def set_event_loop():
    asyncio.set_event_loop(loop)


class Communication(I2CCom):
    """
    This class implements an interface to communicate with i2c devices with
    an Arduino as an intermediary.

    Background work is done with PyMata3 library,
    and Firmata firmware for Arduino.
    """

    DELAY = 0.0001

    def __init__(self):
        super().__init__()
        board = PyMata3()
        board.i2c_config()
        self.board = board
        self.i2c_address = None
        self.data = None

    def set_i2c_address(self, address):
        self.i2c_address = address

    def write(self, register, value):
        set_event_loop()
        self.board.i2c_write_request(self.i2c_address, [register, value])

    def __incoming_data__(self, data):
        self.data = data

    def __incoming_data_burst__(self, data):
        self.data = data

    def read_register(self, register):
        set_event_loop()
        self.board.i2c_write_request(self.i2c_address, [register, ])
        self.board.i2c_read_request(self.i2c_address, register, 1,
                                    Ct.I2C_READ,
                                    cb=self.__incoming_data__, cb_type=Ct.CB_TYPE_DIRECT)

        return self.__wait_for_read()[2]

    def __wait_for_read(self):

        while self.data is None:
            self.board.sleep(self.DELAY)

        data = self.data
        self.data = None

        return data

    def read_register_burst(self, register, n) -> list:
        set_event_loop()
        self.board.i2c_write_request(self.i2c_address, [register, ])
        self.board.i2c_read_request(self.i2c_address, register, n,
                                    Ct.I2C_READ,
                                    cb=self.__incoming_data_burst__, cb_type=Ct.CB_TYPE_DIRECT)

        return self.__wait_for_read()[2:]





