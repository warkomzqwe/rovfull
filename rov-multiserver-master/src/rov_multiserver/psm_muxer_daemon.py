"""
PSM Muxer listener.

The PSM Muxer is implemented on an Arduino Due board. It reads the
analog outputs of 2 BlueRobotics PSM R2 modules, printing the values on
its serial output and generating a new current signal in its DAC output.

This script's objective is to get the measured voltages and currents from this
board instead of the autopilot and to publish the obtained values on the
following topic: /vehicle/psm_muxer/

The data_ will be a dict with the following keys:

* **raw_voltage** for the lowest voltage.
* **raw_current** for the sum of both currents.

For practical effects these measurements will represent the whole vehicle's
storage system.
"""
import string
from concurrent.futures import ThreadPoolExecutor
from threading import Lock, Event
from time import time, sleep
from typing import Dict, Optional

import click
import serial
from pymessaginglib import set_interface, set_interface_by_ipv4
from pymessaginglib.Topic import topic_factory, Topic
from rov_logger.logger import get_rov_logger

DUE_SERIAL_PORT = "/dev/ttyACM0"
DUE_SERIAL_BAUD_RATE = 115200
OUTPUT_RATE_MAX_HZ = 4.0
OUTPUT_TOPIC_NAME = "/vehicle/psm_muxer/"
ROV_LOGGER_DEBUG_LEVEL = "INFO"

try:
    set_interface('eth0')
except ValueError:
    set_interface_by_ipv4('192.168.2.255')

logger = get_rov_logger(ROV_LOGGER_DEBUG_LEVEL)


class PSMDueListener:
    def __init__(self, port: str = DUE_SERIAL_PORT):
        self.__port: Optional[serial.Serial] = None
        self.__available = False
        self.__port_name: str = port

    def setup(self):
        try:
            self.__port = serial.Serial(port=self.__port_name,
                                        baudrate=DUE_SERIAL_BAUD_RATE,
                                        timeout=1.0)
            logger.info(f"Serial port {DUE_SERIAL_PORT} successfully "
                        f"connected")
            self.__available = True
        except serial.SerialException as e:
            logger.error(e)
            logger.debug("Sleeping 2 seconds because of serial setup "
                         "failure..")
            sleep(2)

    def available(self) -> bool:
        return self.__available

    def get_data(self) -> Dict[str, float]:
        if not self.__available:
            raise RuntimeError("Serial port is not set-up")
        data_ = {}
        tries = 0
        just_flushed = False
        logger.debug(self.__port.in_waiting)
        if self.__port.in_waiting > 500:
            self.__port.reset_input_buffer()
            just_flushed = True
        while len(data_) < 2 and tries < 4:
            try:
                data_line = self.__port.readline()
                if just_flushed and tries == 0:
                    tries += 1
                    continue
            except serial.SerialException:
                self.__available = False
                self.__port.close()
                logger.warning("Closing serial port")
                raise
            try:
                data_line = data_line.decode('utf-8')
            except UnicodeError:
                continue
            try:
                data_.update(self.parse_data_line(data_line))
            except ValueError:
                pass
        if len(data_) < 2:
            raise ValueError("It was impossible to get the corresponding "
                             "data_")
        logger.debug(f"V: {data_['raw_voltage']} V, "
                     f"C: {data_['raw_current']} A")
        return data_

    @staticmethod
    def __error_exit(err_msg: str):
        logger.error(err_msg)
        raise ValueError(err_msg)

    def __parse_voltage_payload(self, data_line: str, payload: str) -> float:
        if '->' not in payload:
            self.__error_exit(f'Invalid data_ line: "{data_line}"')
        payload = payload.split('->')[1]
        payload = payload.lstrip().rstrip(string.whitespace + 'V')
        return float(payload)

    def parse_data_line(self, data_line: str) -> Dict[str, float]:
        data_line = data_line.rstrip()
        logger.debug(f"Data: {data_line}")
        if ':' not in data_line:
            self.__error_exit(f'Invalid data_ line: "{data_line}"')
        measurement_type_str, payload = data_line.split(":")
        if measurement_type_str == 'C':
            if '=' not in payload:
                self.__error_exit(f'Invalid data_ line: "{data_line}"')
            payload = payload.split('=')[1].lstrip()
            if 'A' not in payload:
                self.__error_exit(f'Invalid data_ line: "{data_line}"')
            payload = payload.split('A')[0].rstrip()
            value = float(payload)
            return {'raw_current': value}
        elif measurement_type_str == 'V':
            if '/' not in payload:
                self.__error_exit(f'Invalid data_ line: "{data_line}"')
            payload1, payload2 = payload.split('/')
            voltage1 = self.__parse_voltage_payload(data_line, payload1)
            voltage2 = self.__parse_voltage_payload(data_line, payload2)
            return {'raw_voltage': min(voltage1, voltage2)}
        else:
            self.__error_exit(f'Invalid measurement typ'
                              f'e: "{measurement_type_str}"')


class PSMDataRateLimitedPublisher:
    def __init__(self):
        self.__should_stop = Event()
        self._data_lock = Lock()
        self._do_publish_lock = Lock()
        self.__last_publish_time = 0.0
        self._publish_in_progress = False
        self.__topic: Topic = topic_factory.create_topic(OUTPUT_TOPIC_NAME)
        self.__max_rate = OUTPUT_RATE_MAX_HZ
        self.__min_period = 1.0 / self.__max_rate
        self._last_data = {}
        self.__executor = ThreadPoolExecutor(
            max_workers=1, thread_name_prefix="PublisherThread")

    def publish(self, data_: Dict[str, float]):
        if self._input_is_valid(data_):
            with self._data_lock:
                self._last_data.update(data_)
        with self._do_publish_lock:
            if not self._publish_in_progress:
                if not self._is_too_soon_to_publish():
                    self._do_publish()
                else:
                    self._publish_in_progress = True
                    self.__executor.submit(self._publish_on_thread)

    def close(self):
        """Properly closes the publisher.

        If a thread is waiting to publish interrupt it and close immediately.
        If no thread is waiting it does nothing."""
        self.__should_stop.set()

    @property
    def topic(self) -> Topic:
        return self.__topic

    @property
    def max_rate(self):
        return self.__max_rate

    @property
    def min_period(self):
        return self.__min_period

    def _is_too_soon_to_publish(self) -> bool:
        time_diff = time() - self.__last_publish_time
        if time_diff >= self.min_period:
            return False  # It's not too soon
        else:
            return True  # It's too soon

    @staticmethod
    def _input_is_valid(data_: Dict[str, float]) -> bool:
        for key in ('raw_voltage', 'raw_current'):
            if key not in data_ or not isinstance(data_[key], float):
                return False
        if len(data_) > 2:
            return False
        return True

    def _do_publish(self):
        with self._data_lock:
            data_to_send = self._last_data
        self.__last_publish_time = time()
        logger.info(f"Publishing: {data_to_send}")
        self.topic.send(data_to_send)

    def _publish_on_thread(self):
        logger.debug("Setting a thread to publish with delay")
        self.__should_stop.clear()
        target_time = self.__last_publish_time + self.min_period
        remaining_time = target_time - time()
        logger.debug(f"The data will be published in {remaining_time*1000} ms")
        self.__should_stop.wait(remaining_time)
        if self.__should_stop.is_set():
            logger.info("Stop was requested")
            return
        with self._do_publish_lock:
            self._do_publish()
            self._publish_in_progress = False


@click.command()
@click.option('-D', '--device', default=DUE_SERIAL_PORT,
              help="Serial port name for the Due platform.")
def main(device):
    listener = PSMDueListener(device)
    publisher = PSMDataRateLimitedPublisher()
    try:
        while True:
            if not listener.available():
                logger.info("Serial listener not available, trying to set it "
                            "up")
                listener.setup()
            else:
                try:
                    data_ = listener.get_data()
                    publisher.publish(data_)
                    sleep(0.2)
                except serial.SerialException as error:
                    logger.warning(f"Serial port failed: {error}")
                except ValueError:
                    pass
    except KeyboardInterrupt:
        logger.warning("User interrupt")
    finally:
        publisher.close()


if __name__ == '__main__':
    main()
