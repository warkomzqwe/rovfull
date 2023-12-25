from abc import ABCMeta

from pymessaginglib import set_interface
from pymessaginglib.Topic import topic_factory, Topic
from rov_logger.logger import get_rov_logger

from .enums import PowerSourceType
from .power_source_detector_interface import IPowerSourceDetector

logger = get_rov_logger()


class BasePowerSourceDetector(IPowerSourceDetector, metaclass=ABCMeta):
    DEFAULT_TOPIC = "/vehicle/power_source/"
    DEFAULT_OUTPUT_INTERFACE = "eth0"

    def __init__(self, topic_to_publish: str = DEFAULT_TOPIC,
                 output_interface: str = DEFAULT_OUTPUT_INTERFACE):
        try:
            set_interface(output_interface)
        except ValueError:
            logger.warning(f"Invalid interface name: {output_interface}, "
                           f"using default...")
            output_interface = 'default interface'
        self._topic: Topic = topic_factory.create_topic(topic_to_publish)
        logger.info('Initializing Power source detector. It will publish '
                    f'the detections on: "{topic_to_publish}" through '
                    f'{output_interface}')

    def on_new_power_source(self, power_source_type: PowerSourceType):
        logger.debug(f"Power source of type: {power_source_type.name}")
        if power_source_type == PowerSourceType.EXTERNAL_POWER:
            data = {'active_power_source': 'external'}
        elif power_source_type == PowerSourceType.BATTERY:
            data = {'active_power_source': 'battery'}
        else:
            data = None
        if data is not None:
            self._topic.send(data)
