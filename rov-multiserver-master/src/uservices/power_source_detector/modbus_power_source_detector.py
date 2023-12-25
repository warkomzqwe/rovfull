import threading
from time import sleep, time
from typing import Optional, List

from rov_logger.logger import get_rov_logger
from uservices.power_source_detector.modbus_utils import BaseModbusClient, \
    modbus_safe_call

from .enums import PowerSourceType
from .base_power_source_detector import BasePowerSourceDetector
from .base_temperature_listener import BaseTemperatureListener

logger = get_rov_logger()


class ModbusPowerSourceDetector(BasePowerSourceDetector,
                                BaseTemperatureListener, BaseModbusClient):
    SLAVE_ADDRESS = 10
    POWER_SOURCE_REGISTER = 0x0A
    POWER_SOURCE_PUBLISH_INTERVAL = 0.5
    POWER_SOURCE_REQUIRED_REDUNDANCY = 3  # 3 times to validate
    TEMPERATURE_BASE_REGISTER = 0x0B
    N_TEMPERATURES = 5
    TEMP_PUBLISH_INTERVAL = 0.25
    MAX_IDLE_TIME = 0.2
    MAX_N_ERRORS = 10

    def __init__(
            self, port_name: str, baud_rate: int,
            topic_to_publish: str = BasePowerSourceDetector.DEFAULT_TOPIC,
            output_interface: str =
            BasePowerSourceDetector.DEFAULT_OUTPUT_INTERFACE,
            temperature_topic: str = BaseTemperatureListener.DEFAULT_TOPIC):
        BasePowerSourceDetector.__init__(self, topic_to_publish,
                                         output_interface)
        BaseTemperatureListener.__init__(self, temperature_topic)
        self._should_stop = threading.Event()
        BaseModbusClient.__init__(self, port_name, baud_rate,
                                  self.SLAVE_ADDRESS)
        self.__last_ps = None
        self.__last_ps_counter = 0
        self.__last_ps_publish_time = 0.0
        self.__last_temp_publish_time = 0.0

    def listen_forever(self):
        # Outer loop:
        while not self._should_stop.is_set():
            if not self.try_connection():
                # Connection failed, sleep for a while before retrying
                sleep(2.0)
                continue
            successive_comm_errors = 0
            logger.info("Modbus power source detector connection is up on "
                        f"port {self._port.serial.port}")
            while self._port is not None and not self._should_stop.is_set():
                # Get power source
                power_source = self._get_power_source()
                if power_source is not None:
                    successive_comm_errors = 0
                    if power_source != self.__last_ps:
                        self.__last_ps_counter = 0
                        self.__last_ps = power_source
                    else:  # Detected the same power source again
                        self.__last_ps_counter += 1
                    if self.__last_ps_counter >= \
                            self.POWER_SOURCE_REQUIRED_REDUNDANCY:
                        time_elapsed = time() - self.__last_ps_publish_time
                        if time_elapsed >= self.POWER_SOURCE_PUBLISH_INTERVAL:
                            self.__last_ps_counter = 0
                            self.__last_ps_publish_time = time()
                            self.on_new_power_source(power_source)
                else:
                    successive_comm_errors += 1
                # Get temperatures
                temperatures = self._get_temperatures()
                if temperatures is not None:
                    time_elapsed = time() - self.__last_temp_publish_time
                    if time_elapsed >= self.TEMP_PUBLISH_INTERVAL:
                        self.__last_temp_publish_time = time()
                        self.on_new_temperature_values(temperatures)
                # Determine a proper time lapse to sleep
                next_ps_publishing_time = \
                    self.__last_ps_publish_time + \
                    self.POWER_SOURCE_PUBLISH_INTERVAL
                next_temp_publishing_time = \
                    self.__last_temp_publish_time + \
                    self.TEMP_PUBLISH_INTERVAL
                time_to_publish_ps = next_ps_publishing_time - time()
                time_to_publish_temp = next_temp_publishing_time - time()
                if successive_comm_errors > self.MAX_N_ERRORS or \
                        time_to_publish_ps > self.MAX_IDLE_TIME and \
                        time_to_publish_temp > self.MAX_IDLE_TIME:
                    sleep(self.MAX_IDLE_TIME)

    @modbus_safe_call
    def _get_power_source(self) -> Optional[PowerSourceType]:
        register_value = self._port.read_register(self.POWER_SOURCE_REGISTER,
                                                  functioncode=4)
        if register_value == 0:
            return PowerSourceType.BATTERY
        elif register_value == 1:
            return PowerSourceType.EXTERNAL_POWER
        else:
            logger.warning(f"Invalid power source value: {register_value}")
            return None  # invalid response value

    @modbus_safe_call
    def _get_temperatures(self) -> Optional[List[int]]:
        return self._port.read_registers(self.TEMPERATURE_BASE_REGISTER,
                                         self.N_TEMPERATURES, functioncode=4)

    def close(self):
        pass
