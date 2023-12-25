from event_bus import EventBus
from pymessaginglib.MessagingSystem import receiver
from pymessaginglib.Topic import topic_factory, TopicObserver, Topic

_bus = EventBus()
_battery_info_event = "battery_info_event"
_voltage_field = "voltage"
_current_field = "current"
_topic: Topic = topic_factory.create_topic("/vehicle/data/")


def _is_valid_msg(dic: dict):
    """
    Check if incoming battery messages are valid
    :param dic: incoming message as dict
    :return: True/False if message is valid/invalid
    """

    try:
        assert dic.get(_voltage_field) is not None
        assert dic.get(_current_field) is not None
        assert isinstance(dic.get(_voltage_field), float)
        assert isinstance(dic.get(_current_field), float)
        return True

    except AssertionError:
        return False


class BatteryObserver(TopicObserver):
    """
    Vehicle data listener
    """

    def on_message_arrival(self, dic: dict, ip: str):

        if not _is_valid_msg(dic):
            return

        _bus.emit(_battery_info_event, dic.get(_voltage_field)/1000.0, dic.get(_current_field)/100.0)


def start_battery_msg_listener():

    battery_observer = BatteryObserver()
    _topic.attach(battery_observer)
    receiver.start()







