import threading
from time import sleep
from typing import Optional, Callable, Any, List

import serial.threaded
from rov_logger.logger import get_rov_logger

from .enums import PowerSourceType
from .base_power_source_detector import BasePowerSourceDetector
from .base_temperature_listener import BaseTemperatureListener

logger = get_rov_logger()


class SerialPowerSourceDetectorProtocol(serial.threaded.LineReader):
    def __init__(self):
        serial.threaded.LineReader.__init__(self)
        self._callback = lambda state: None
        self._temp_callback = lambda temps: None
        self._connected = False

    def set_callback(self,
                     callback: Optional[Callable[[PowerSourceType], Any]]):
        if callable(callback):
            self._callback = callback
        else:
            self._callback = lambda state: None

    def set_temp_callback(self,
                          callback: Optional[Callable[[List[int]], Any]]):
        if callable(callback):
            self._temp_callback = callback
        else:
            self._temp_callback = lambda temps: None

    def clear_callback(self):
        self.set_callback(None)

    def clear_temp_callback(self):
        self.set_temp_callback(None)

    def connection_made(self, transport):
        logger.info("Serial power source detector connection is up on "
                    f"port {transport.serial.name}")
        self._connected = True
        serial.threaded.LineReader.connection_made(self, transport)

    def connection_lost(self, exc):
        logger.warning("Serial power source detector connection lost with "
                       f"port {self.transport.serial.name}")
        self._connected = False
        if exc is not None:
            logger.debug("Serial power source detector exception: "
                         f"{str(exc)}. Closing...")
            try:
                serial.threaded.LineReader.connection_lost(self, exc)
            except type(exc):
                pass
        else:
            serial.threaded.LineReader.connection_lost(self, None)

    def handle_line(self, line: str):
        logger.debug(f'Message received: "{line}"')
        if not self._handle_line_temperature(line) and not \
                self._handle_line_power_source_detector(line):
            logger.warning(f'Invalid message received: "{line}"')

    def _handle_line_power_source_detector(self, line: str) -> bool:
        logger.debug("Processing line as power source detector")
        message_parts = line.split(':')
        if len(message_parts) < 2:
            logger.debug("Split gave less than 2 parts so the message is "
                         "invalid")
            return False
        try:
            key_index = message_parts.index('PowerSource')
            idx = key_index + 1
        except ValueError:
            logger.debug('Message is valid but "PowerSource" key was not '
                         'found')
            return False
        if idx is None:
            logger.debug("Invalid state (ValueError was not raised?)")
            return False
        if idx >= len(message_parts):
            logger.debug('"PowerSource" key found but no value specified on '
                         'message')
            return False
        power_source_str = message_parts[idx]
        # At this point the message should be valid
        logger.debug(f'Valid message: "{line}". Processing...')
        if power_source_str == "External":
            self._callback(PowerSourceType.EXTERNAL_POWER)
        elif power_source_str == "Battery":
            self._callback(PowerSourceType.BATTERY)
        else:
            logger.debug(f'Valid message but invalid value: "{line}"')
        return True

    def _handle_line_temperature(self, line: str) -> bool:
        logger.debug("Trying to parse as temperature message")
        message_parts = line.split(':')
        if len(message_parts) < 2:
            logger.debug("Split gave less than 2 parts so the message is "
                         "invalid")
            return False
        try:
            key_index = message_parts.index('TemperatureRaw')
            idx = key_index + 1
        except ValueError:
            logger.debug('Message is valid but "TemperatureRaw" key was not '
                         'found')
            return False
        if idx is None:
            logger.debug("Invalid state (ValueError was not raised?)")
            return False
        if idx >= len(message_parts):
            logger.debug('"TemperatureRaw" key found but no value specified '
                         'on message')
            return False
        payload = message_parts[idx]
        # At this point the message should be valid
        logger.debug(f'Valid temperature message: "{line}". Processing...')
        temperature_values = []
        for temperature_value in payload.split(';'):
            try:
                temperature_values.append(int(temperature_value))
            except ValueError:
                pass
        if len(temperature_values) > 0:
            self._temp_callback(temperature_values)
        else:
            logger.debug(f'Valid temperature message but invalid value: '
                         f'"{line}"')
        return True

    @property
    def connected(self) -> bool:
        return self._connected


class SerialPowerSourceDetector(BasePowerSourceDetector,
                                BaseTemperatureListener):
    def __init__(
            self, port_name: str, baud_rate: int,
            topic_to_publish: str = BasePowerSourceDetector.DEFAULT_TOPIC,
            output_interface: str =
            BasePowerSourceDetector.DEFAULT_OUTPUT_INTERFACE,
            temperature_topic: str = BaseTemperatureListener.DEFAULT_TOPIC):
        BasePowerSourceDetector.__init__(self, topic_to_publish,
                                         output_interface)
        BaseTemperatureListener.__init__(self, temperature_topic)
        self._baud_rate = baud_rate
        self._port_name = port_name
        self._should_stop = threading.Event()
        self._port: Optional[serial.Serial] = None

    def listen_forever(self):
        # Outer loop:
        while not self._should_stop.is_set():
            try:
                self._port = serial.Serial(port=self._port_name,
                                           baudrate=self._baud_rate)
            except serial.serialutil.SerialException as exc:
                logger.warning(f"SerialException: {exc.strerror}. Retrying "
                               "in 2 seconds...")
                self._port = None
            if self._port is None:
                # Connection failed, sleep for a while before retrying
                sleep(2.0)
                continue
            # Connection succeeded, iterate inside context manager
            with serial.threaded.ReaderThread(
                    self._port, SerialPowerSourceDetectorProtocol) as protocol:
                assert isinstance(protocol, SerialPowerSourceDetectorProtocol)
                protocol.set_callback(self.on_new_power_source)
                protocol.set_temp_callback(self.on_new_temperature_values)
                # Inner loop:
                while protocol.connected and not self._should_stop.is_set():
                    self._should_stop.wait(timeout=1.0)
                protocol.clear_callback()
                protocol.clear_temp_callback()

    def close(self):
        logger.info("Closing serial power source detector...")
        self._should_stop.set()
