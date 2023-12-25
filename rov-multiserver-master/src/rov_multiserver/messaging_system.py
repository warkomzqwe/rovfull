# -*- coding: utf-8 -*-
"""
Messaging system module.

This module handles the communication with the client(s).
It makes use of the
`Python Messaging Lib <http://gitlab.telemcloud.cl/nhasbun/py_messaging_lib>`.
"""
import time
import queue
import threading
import concurrent.futures

from pymessaginglib import set_interface, get_interfaces
from pymessaginglib.Heartbeat import Heartbeat
from pymessaginglib.Topic import TopicObserver, topic_factory, Topic
from pymessaginglib.Discovery import DiscoveryMaster
from pymessaginglib.MessagingSystem import receiver, send
from rov_event_bus.bus import *
from rov_logger.logger import get_rov_logger

logger = get_rov_logger()
if 'br0' in [ x.split(' => ')[0] for x in get_interfaces() ]:
    set_interface('br0')
elif 'eth0' in [ x.split(' => ')[0] for x in get_interfaces() ]:
    set_interface('eth0')


class JoystickInputTopicObserver(TopicObserver):
    """
    Remote joystick observer.

    This class will capture the remote joystick by listening to the
    incoming messages arriving to the "/client/joystick/" topic,
    creating the corresponding "input-event" events.
    """

    def __init__(self):
        super().__init__()
        self.__input_queues = {}
        self.__pool = concurrent.futures.ThreadPoolExecutor(
            max_workers=20,
            thread_name_prefix="JS")
        self.__should_stop = threading.Event()

    def on_message_arrival(self, data, ip):
        """
        Incoming Message callback.

        This method will be called every time a message arrives on the
        "/client/joystick/" topic.

        Parameters
        ----------
        data : dict
            A python dict with the message payload.
            It could have two possible keys: "button_event" and
            "axis_event".
            -   *"button_event"* should have a string describing the
                input-event. It just should trigger an input-event with
                the same string as the only arg.
            -   *"axis_event"* should have a list of strings decribing
                an input-event for each axis and hat. The idea is that
                an input-event should be triggered for every axis that
                changed, so the previous value should be stored.
        ip : str
            A string describing the ip address from which the message
            was generated.

        """
        if isinstance(data, dict) and len(data) > 0:
            if 'axis_event' in data and isinstance(data['axis_event'],
                                                   (list, tuple)):
                for event_str in data['axis_event']:
                    str_pieces = event_str.split(';')
                    if len(str_pieces) > 1 and len(str_pieces[0]) == 3:
                        self._enqueue_input_event(event_str)
                        # event_bus.trigger('input-event', event_str)
            if 'button_event' in data\
                    and isinstance(data['button_event'], str):
                str_pieces = data['button_event'].split(';')
                if len(str_pieces) > 1 and len(str_pieces[0]) == 3:
                    event_bus.trigger('input-event', data['button_event'])

    def _enqueue_input_event(self, event_string):
        # print('qi')
        identifier = event_string.split(';')[0]
        input_queue = self.__input_queues.get(identifier)
        if input_queue is None:  # create queue and its consumer
            # print("creating queue")
            input_queue = queue.Queue(maxsize=1)
            self.__input_queues[identifier] = input_queue
            self._consume_queue_events_by_identifier(identifier)
        try:
            # print("Trying to put on queue")
            input_queue.put_nowait(event_string)
        except queue.Full:
            print("Collision on {}".format(identifier))
            try:
                input_queue.get_nowait()
            except queue.Empty:  # probably this is (almost) impossible
                pass
            input_queue.put_nowait(event_string)
        # print("done enqueuing")

    def _consume_queue_events_by_identifier(self, event_identifier):
        self.__pool.submit(self._consume_queue_events_by_identifier_loop,
                           event_identifier)

    def _consume_queue_events_by_identifier_loop(self, event_identifier):
        inqueue = self.__input_queues.get(event_identifier)
        stop_task = self.__pool.submit(self.__should_stop.wait)
        while True:
            new_event_task = self.__pool.submit(inqueue.get)
            done, undone = concurrent.futures.wait(
                (stop_task, new_event_task),
                return_when=concurrent.futures.FIRST_COMPLETED)
            if stop_task in done:  # stop required
                if not new_event_task.done():
                    try:
                        inqueue.put_nowait(None)
                    except queue.Full:
                        pass
                return
            if new_event_task in done:
                event_bus.trigger('input-event', new_event_task.result())

    def stop(self):
        self.__should_stop.set()
        self.__pool.shutdown()


class SensorTopicObserver(TopicObserver):
    def on_message_arrival(self, data, ip):
        if isinstance(data, dict)\
                and len(data) > 0\
                and 'type' in data\
                and 'index' in data\
                and 'value' in data\
                and 'unit' in data:
            if data['type'] == 'temperature':
                # Process temperature sensor measurement
                self._parse_temperature_measurement(data)
            elif data['type'] == 'altitude':
                # Process altitude sensor measurement
                self._parse_altitude_measurement(data)
            else:
                logger.warning(
                    'Not processing sensor of type: {}'.format(data['type']))

    @staticmethod
    def _parse_measurement(data_dict):
        extras = data_dict.get('extras')
        event_bus.trigger('new-mqttudp-measurement', data_dict['type'],
                          data_dict['index'], data_dict['unit'],
                          data_dict['value'], extras)

    @staticmethod
    def _parse_temperature_measurement(data_dict):
        if data_dict['index'] == 0 or data_dict['index'] == 1:
            SensorTopicObserver._parse_measurement(data_dict)

    @staticmethod
    def _parse_altitude_measurement(data_dict):
        SensorTopicObserver._parse_measurement(data_dict)


class StartingVideoQualityTopicObserver(TopicObserver):
    def __init__(self):
        self.__received = False

    def on_message_arrival(self, dic: dict, ip: str):
        if 'video_codec' in dic and 'video_quality' in dic \
                and not self.__received:
            event_bus.trigger('start-video-quality',
                              dic['video_codec'],
                              dic['video_quality'],
                              ip)
            self.__received = True


class ClientStateTopicObserver(TopicObserver):
    def on_message_arrival(self, data, ip):
        state = data.get('state')
        if state is None:
            logger.warning('Invalid message on client_state topic: '
                           '{}'.format(data))
        if state in ('gallery_started', 'settings_started', 'app_closed',
                     'live_camera_started'):
            event_bus.trigger('client-state-changed', state)
            if state == 'live_camera_started':
                # Update HUD when the client started its camera rendering state
                logger.info("New client connected (IP:{ip})".format(ip=ip))
                event_bus.trigger('video-client-connected', ip)
                event_bus.trigger('client-connected')
            else:
                event_bus.trigger('video-client-disconnected', ip)


class ClientSettingsTopicObserver(TopicObserver):
    def __init__(self):
        self.__last_message_time = 0

    def on_message_arrival(self, dic: dict, ip: str):
        quality = dic.get('video_quality')
        if quality is None:
            logger.warning(f'Invalid message on client_settings topic:{dic}')
        else:
            if (time.time() - self.__last_message_time) > 0.5:
                event_bus.trigger('new-video-quality-requested', int(quality))
                self.__last_message_time = time.time()


class RecordingCommandObserver(TopicObserver):
    def on_message_arrival(self, dic: dict, ip: str):
        cmd = dic.get('cmd')
        if cmd == 'start':
            event_bus.trigger('start-rec-cmd')
        elif cmd == 'stop':
            event_bus.trigger('stop-rec-cmd')
        else:
            pass


class MessagingSystem(object):
    """TODO: Document."""

    def __init__(self):
        """
        Initializes the messaging system.

        1.  Creates a DiscoveryMaster object.
        2.  Create a TopicObserver subclass for each topic of interest.
            By the moment it's just: "/client/joystick/". This topic
            observer is implemented in this module as
            JoystickInputTopicObserver.
        3.  Create the corresponding Topic object using the
            "topic_factory" and attach the JoystickInputTopicObserver to
            it.

        """
        self._discovery_master = DiscoveryMaster()
        self._listening_topics = {}
        # js_topic = '/client/joystick/'
        # self._listening_topics[js_topic] = topic_factory.create_topic(
        # js_topic)
        # self._joystick_observer = JoystickInputTopicObserver()
        # self._listening_topics[js_topic].attach(self._joystick_observer)
        sensors_topic = '/rov/sensor/'
        self._listening_topics[sensors_topic]\
            = topic_factory.create_topic(sensors_topic)
        self._sensors_observer = SensorTopicObserver()
        self._listening_topics[sensors_topic].attach(self._sensors_observer)
        client_state_topic = '/gcs/client_state/'
        self._listening_topics[client_state_topic] = \
            topic_factory.create_topic(client_state_topic)
        self._client_state_observer = ClientStateTopicObserver()
        self._listening_topics[client_state_topic].attach(
            self._client_state_observer)
        client_settings_topic = '/gcs/client_settings/'
        self._listening_topics[client_settings_topic] = \
            topic_factory.create_topic(client_settings_topic)
        self._client_settings_observer = ClientSettingsTopicObserver()
        self._listening_topics[client_settings_topic].attach(
            self._client_settings_observer)
        start_quality_topic = '/start_quality/'
        self._listening_topics[start_quality_topic] = \
            topic_factory.create_topic(start_quality_topic)
        self._start_quality_observer = StartingVideoQualityTopicObserver()
        self._listening_topics[start_quality_topic].attach(
            self._start_quality_observer)
        rec_cmd_topic = '/camera/main/rec_cmd/'
        self._listening_topics[rec_cmd_topic] = \
            topic_factory.create_topic(rec_cmd_topic)
        self._rec_cmd_observer = RecordingCommandObserver()
        self._listening_topics[rec_cmd_topic].attach(
            self._rec_cmd_observer)
        self.__alive = False
        self._vehicle_data_distributor = VehicleDataDistributor(
            max_frequency=5)
        self._notifications_distributor = GCSNotificationsDistributor()
        self._heartbeat = Heartbeat("companion")
        self._heartbeat.delay = 1.5

    def start(self):
        """
        Starts the messaging system.

        It should start the discovery master and the receiver.
        """
        self._discovery_master.start()
        self._vehicle_data_distributor.start()
        self._notifications_distributor.start()
        receiver.start()
        self._heartbeat.start()
        self.__alive = True

    def stop(self):
        """
        Stops the messaging system.

        It should stops the receiver and the discovery system.
        """
        # self._joystick_observer.stop()
        self._discovery_master.stop()
        self._vehicle_data_distributor.stop()
        self._notifications_distributor.stop()
        receiver.stop()
        self._heartbeat.stop()
        self.__alive = False

    def is_alive(self):
        """
        Checks if the system is running.

        Returns
        -------
        bool
            True if system is running. False otherwise.

        """
        return self.__alive


class VehicleDataDistributor(object):
    """
    Sends vehicle data through MQTT-UDP.
    """

    def __init__(self, max_frequency=4):
        """Constructor."""
        self._stopped = False
        self._last_message_time = 0.0
        self.max_frequency = max_frequency
        self._last_data = {}
        self._next_data = {}
        self._input_queue = queue.Queue()
        self._has_output_data = threading.Event()
        self._thread_pool = concurrent.futures.ThreadPoolExecutor(
            thread_name_prefix='VDDThread')

    @property
    def max_frequency(self):
        """Maximum delivery frequency."""
        return 1.0 / self._min_period

    @max_frequency.setter
    def max_frequency(self, new_frequency):
        new_frequency = float(new_frequency)
        if new_frequency > 0.0:
            self._min_period = 1.0 / new_frequency

    def __next_message_time_remaining(self):
        remaining_time\
            = self._last_message_time + self._min_period - time.time()
        if remaining_time <= 0:
            return 0.0
        else:
            return remaining_time

    def start(self):
        event_bus.register_callback('telemetry-value-changed',
                                    self._on_telemetry_value_changed)
        event_bus.register_callback('vehicle-state-value-updated',
                                    self._on_state_value_changed)
        event_bus.register_callback('rtc-second-elapsed',
                                    self._on_rtc_second_elapsed)
        event_bus.register_callback('client-connected',
                                    self._on_video_client_connected,
                                    priority=IMMEDIATE)
        self._stopped = False
        self._thread_pool.submit(self._input_filtering_loop)
        self._thread_pool.submit(self._output_loop)
        self._thread_pool.submit(self._key_data_loop)

    def stop(self):
        event_bus.unregister_callback('telemetry-value-changed',
                                      self._on_telemetry_value_changed)
        event_bus.unregister_callback('vehicle-state-value-updated',
                                      self._on_state_value_changed)
        self._stopped = True
        self._input_queue.put({})  # Just in case it's blocked.
        self._has_output_data.set()  # Just in case it's blocked.
        self._thread_pool.shutdown()

    def _on_telemetry_value_changed(self, telemetry_obj):
        self._input_queue.put(
            {telemetry_obj._attribute_name: telemetry_obj.get()})

    def _on_state_value_changed(self, value_name, type_, value, src):
        if value_name == 'navigation_mode':
            value = value.name
        self._input_queue.put({value_name: value})

    def _on_rtc_second_elapsed(self, real_time):
        self._input_queue.put({
            'date_string': real_time.get_date_str(),
            'time_string': real_time.get_time_str().replace(':', '.')})

    def _input_filtering_loop(self):
        """Put just changing values on the output queue."""
        while not self._stopped:
            new_element = self._input_queue.get(block=True, timeout=None)
            for name in new_element:
                prev_value = self._last_data.get(name)
                if prev_value != new_element[name]:
                    # put it in output_queue
                    self._next_data.update(new_element)
                    if len(self._next_data) > 0:
                        self._has_output_data.set()
            self._input_queue.task_done()

    def _on_video_client_connected(self):
        # Flush "last_data" to re-send everything
        self._last_data.clear()

    def _output_loop(self):
        while not self._stopped:
            self._has_output_data.wait()
            # We have data to output, process every item in the input queue.
            if len(self._next_data) == 0:
                continue
            remaining_time = self.__next_message_time_remaining()
            if remaining_time > 0:
                time.sleep(remaining_time)
            if self._stopped:
                break
            self._input_queue.join()
            # Output data
            self._last_message_time = time.time()
            # send('/vehicle/data/', self._next_data)
            topic = topic_factory.create_topic('/vehicle/data/')
            assert isinstance(topic, Topic)
            send(topic, self._next_data)
            self._last_data.update(self._next_data)
            self._next_data = {}
            self._has_output_data.clear()

    def _key_data_loop(self):
        while not self._stopped:
            event_bus.trigger('client-connected')
            time.sleep(2)


class GCSNotificationsDistributor:
    def start(self):
        event_bus.register_callback('new-user-notification',
                                    self._on_new_user_notification)

    def stop(self):
        event_bus.unregister_callback('new-user-notification',
                                      self._on_new_user_notification)

    @staticmethod
    def _on_new_user_notification(text, period, duration):
        logger.debug("NEW NOTIFICATION: {}".format(text))
        send(topic_factory.create_topic('/gcs/notifications/'),
             {'text': text, 'period': period, 'duration': duration})
        logger.debug("NOTIFICATION SENT: {}".format(text))


if __name__ == '__main__':
    observer = JoystickInputTopicObserver()
    js_topic = topic_factory.create_topic("/client/joystick/")
    assert isinstance(js_topic, Topic)
    js_topic.attach(observer)

    def on_input_event(event_description):
        print(event_description)

    event_bus.register_callback('input-event', on_input_event, is_sync=True)
    receiver.start()

    time.sleep(300)
