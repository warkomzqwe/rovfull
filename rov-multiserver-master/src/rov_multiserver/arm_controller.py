from pymessaginglib.Topic import Topic, topic_factory
from rov_event_bus.bus import event_bus
from rov_multiserver.pwm import PwmOutput


class ArmController:

    topic: Topic = topic_factory.create_topic("/rov/arduino_acamas/grabber/")

    def __init__(self, pwm: PwmOutput):
        self.__pwm = pwm
        event_bus.register_callback('arm-open-cmd', self.open, is_sync=True)
        event_bus.register_callback('arm-close-cmd', self.close, is_sync=True)
        event_bus.register_callback('arm-stop-cmd', self.stop, is_sync=True)

    def open(self):
        self.topic.send(dict(cmd="grabber_open"))

    def close(self):
        self.topic.send(dict(cmd="grabber_close"))

    def stop(self):
        self.topic.send(dict(cmd="grabber_idle"))
