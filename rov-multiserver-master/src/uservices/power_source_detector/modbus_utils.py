import termios
from threading import Lock
from typing import Optional

import minimalmodbus
import serial
from rov_logger.logger import get_rov_logger

logger = get_rov_logger()
_modbus_lock = Lock()


def modbus_safe_call(func):
    def wrapper(*args):
        _self: 'BaseModbusClient' = args[0]
        try:
            _modbus_lock.acquire()
            return func(*args)
        except AttributeError:
            logger.warning("No configured instrument")
        except (termios.error, serial.serialutil.SerialException):
            if isinstance(_self, BaseModbusClient):
                logger.warning("Modbus power source detector connection lost "
                               f"with {_self.port.serial.port}")
                _self.port.serial.close()
                _self._port = None
        except minimalmodbus.NoResponseError:
            logger.warning("No response from modbus")
        except minimalmodbus.InvalidResponseError:
            logger.warning("CRC Error")
        except minimalmodbus.IllegalRequestError as e:
            logger.warning(f"Illegal request: {str(e)}")
        finally:
            _modbus_lock.release()

    return wrapper


class BaseModbusClient:
    def __init__(self, port_name: str, baud_rate: int,
                 slave_address: int):
        self._port_name = port_name
        self._baud_rate = baud_rate
        self._slave_address = slave_address
        self._port: Optional[minimalmodbus.Instrument] = None

    def try_connection(self) -> bool:
        logger.info(f"Trying to connect to {self._port_name}...")
        try:
            self._port = minimalmodbus.Instrument(self._port_name,
                                                  self._slave_address)
            self.port.serial.baudrate = self._baud_rate
            self.port.serial.timeout = 0.1
        except serial.serialutil.SerialException as exc:
            logger.warning(f"SerialException: {exc.strerror}. Retrying "
                           "in 2 seconds...")
            self._port = None
        return self._port is not None

    @property
    def port(self) -> Optional[minimalmodbus.Instrument]:
        return self._port
