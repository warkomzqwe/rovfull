from typing import List

from pymessaginglib.Topic import topic_factory, Topic
from rov_logger.logger import get_rov_logger

logger = get_rov_logger()


class BaseTemperatureListener:
    DEFAULT_TOPIC = "/vehicle/temperature/"

    def __init__(self, topic_to_publish: str = DEFAULT_TOPIC):
        self._temp_topic: Topic = topic_factory.create_topic(topic_to_publish)
        logger.info('Initializing temperature listener. It will publish on: '
                    f'"{topic_to_publish}"')

    def on_new_temperature_values(self, temperature_values: List[int]):
        logger.debug(f"New temperature values: {temperature_values}")
        data = {}
        for idx, value in enumerate(temperature_values):
            data[f'T{idx}'] = value
        if len(data) > 0:
            self._temp_topic.send(data)
