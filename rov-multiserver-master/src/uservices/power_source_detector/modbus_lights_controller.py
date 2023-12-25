import threading
import time
from typing import Optional

from pymessaginglib import set_interface
from pymessaginglib.MessagingSystem import receiver
from pymessaginglib.Topic import Topic, topic_factory
from rov_logger.logger import get_rov_logger
from rov_multiserver.vehicle_settings import VehicleSettingsSubject
from uservices.power_source_detector import BaseModbusClient, modbus_safe_call

logger = get_rov_logger()

JOYSTICK_TOPIC = '/client/joystick/'
NOTIFICATIONS_TOPIC = '/gcs/notifications/'
LIGHTS_CYCLE_BUTTON = 'STB'
LIGHTS_100_PERCENT_DUTY_CYCLE = 255
LIGHTS_ZERO_PERCENT_DUTY_CYCLE = 0


class ModbusLightsController(BaseModbusClient):
    SLAVE_ADDRESS = 10
    LIGHTS_DUTY_CYCLE_MB_ADDRESS = 0x50
    PUBLISHING_INTERVAL = 1
    MODBUS_RETRIES = 3

    def __init__(self, port, baudrate, input_topic, output_topic,
                 output_interface):
        BaseModbusClient.__init__(self, port, baudrate, self.SLAVE_ADDRESS)
        self.__last_push_time = None
        self._should_stop = threading.Event()
        self._lights_duty_cycle: Optional[int] = None
        self.__out_topic: Topic = topic_factory.create_topic(output_topic)
        self.__in_topic: Topic = topic_factory.create_topic(input_topic)
        self.__in_topic.attach_callback(self._on_lights_ctrl_msg)
        self.__js_topic: Topic = topic_factory.create_topic(JOYSTICK_TOPIC)
        self.__js_topic.attach_callback(self._on_joystick_msg)
        self.__notifications: Topic = topic_factory.create_topic(
            NOTIFICATIONS_TOPIC)
        receiver.start()
        self.__vehicle_settings = VehicleSettingsSubject()
        try:
            set_interface(output_interface)
        except ValueError:
            pass

    def publishing_loop_on_thread(self):
        self._should_stop.clear()
        threading.Thread(target=self.__publishing_loop,
                         name='LightsStatePublishingThread').start()

    @property
    def connected(self) -> bool:
        return self._port is not None

    @property
    def lights_duty_cycle(self) -> int:
        return self._lights_duty_cycle

    @property
    def lights_intensity(self) -> float:
        return self.__intensity_from_duty_cycle(self.lights_duty_cycle)

    def stop_publishing_loop(self):
        self._should_stop.set()

    def __publishing_loop(self):
        while not self._should_stop.is_set():
            if self.connected:
                self.__read_and_publish()
                self._should_stop.wait(self.PUBLISHING_INTERVAL)
            else:
                if not self.try_connection():
                    self._should_stop.wait(2.0)
                else:
                    logger.info('Lights controller successfully connected')

    def __read_and_publish(self):
        self.__read_lights_state()
        self.__publish_lights_state()

    @modbus_safe_call
    def __read_lights_state(self):
        if self.connected or self.try_connection():
            self._lights_duty_cycle = self.port.read_register(
                self.LIGHTS_DUTY_CYCLE_MB_ADDRESS, functioncode=3)
            logger.debug("Lights duty cycle read: "
                         f"{self.lights_duty_cycle}")

    def __publish_lights_state(self):
        if self.lights_duty_cycle is not None:
            logger.debug("Publishing lights duty cycle: "
                         f"{self.lights_duty_cycle} and intensity: "
                         f"{self.lights_intensity}%")
            self.__out_topic.send(
                {'lights_duty_cycle': self.lights_duty_cycle,
                 'lights_intensity': self.lights_intensity})

    @modbus_safe_call
    def __write_lights_duty_cycle(self, new_duty_cycle):
        if self.connected or self.try_connection():
            self.port.write_register(self.LIGHTS_DUTY_CYCLE_MB_ADDRESS,
                                     new_duty_cycle, functioncode=16)

    def _on_lights_ctrl_msg(self, data, ip):
        if 'lights_duty_cycle' in data:
            duty_cycle = data['lights_duty_cycle']
            logger.debug(f"Message received to set lights duty cycle to"
                         f" {duty_cycle} (from {ip})")
            self.update_lights_duty_cycle(duty_cycle)
        elif 'lights_intensity' in data:
            intensity = data['lights_intensity']
            logger.debug(f"Message received to set lights intensity to"
                         f" {intensity}% (from {ip})")
            self.update_lights_by_intensity(intensity)

    def _on_joystick_msg(self, data, ip):
        if 'button_event' in data:
            try:
                button, state = data['button_event'].split(';')
                if button == LIGHTS_CYCLE_BUTTON:
                    if state == 'P':
                        self.__last_push_time = time.time()
                    elif state == 'R':
                        if self.__last_push_time is not None \
                                and (time.time() - self.__last_push_time) < 1:
                            self.cyclic_step_intensity()
                        self.__last_push_time = None
            except ValueError:
                pass

    def update_lights_duty_cycle(self, new_duty_cycle):
        for _ in range(self.MODBUS_RETRIES):
            self.__write_lights_duty_cycle(new_duty_cycle)
        self.__read_and_publish()
        self.__notifications.send(
            {'text': f"New lights level: {self.lights_intensity}%",
             'period': 2, 'duration': 2})

    def update_lights_by_intensity(self, new_intensity):
        logger.debug(f'Setting lights by intensity to {new_intensity}%')
        self.update_lights_duty_cycle(
            self.__duty_cycle_from_intensity(new_intensity))

    def cyclic_step_intensity(self):
        logger.debug('Stepping lights cyclically...')
        step = self.__vehicle_settings.lights_step_size \
            if not self.__vehicle_settings.lights_on_off_control else 100.0
        new_intensity = self.lights_intensity + step
        if new_intensity > 100.0:
            new_intensity = 0.0
        self.update_lights_by_intensity(new_intensity)

    @staticmethod
    def __duty_cycle_from_intensity(intensity) -> int:
        slope = (LIGHTS_100_PERCENT_DUTY_CYCLE -
                 LIGHTS_ZERO_PERCENT_DUTY_CYCLE) / 100.0
        offset = LIGHTS_ZERO_PERCENT_DUTY_CYCLE
        return int(slope * intensity + offset)

    @staticmethod
    def __intensity_from_duty_cycle(intensity) -> float:
        slope = 100 / (LIGHTS_100_PERCENT_DUTY_CYCLE -
                       LIGHTS_ZERO_PERCENT_DUTY_CYCLE)
        offset = -slope * LIGHTS_ZERO_PERCENT_DUTY_CYCLE
        return int(round(slope * intensity + offset))
