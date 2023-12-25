# -*- coding: utf-8 -*-
"""Messaging system module (messaging_system.py) tests."""

import pytest
from unittest.mock import patch, Mock, call
from pymessaginglib.Discovery import DiscoveryMaster
from rov_multiserver.messaging_system import (MessagingSystem,
                                              JoystickInputTopicObserver)
from rov_event_bus.bus import event_bus

MODULE_STRING = 'rov_multiserver.messaging_system.'  # For patching


@pytest.fixture(scope='module')
def joystick_observer():
    """TODO: Document."""
    return JoystickInputTopicObserver()


@pytest.fixture(scope='module')
def mocked_pymessaginglib():
    """TODO: Document."""
    discovery_mock = Mock(spec=DiscoveryMaster)
    discovery_patcher = patch(MODULE_STRING + 'DiscoveryMaster',
                              return_value=discovery_mock)
    topic_factory_patcher = patch(MODULE_STRING + 'topic_factory')
    receiver_patcher = patch(MODULE_STRING + 'receiver')
    topic_mock = Mock()
    discovery_patcher.start()
    topic_factory_mock = topic_factory_patcher.start()
    receiver_mock = receiver_patcher.start()
    topic_factory_mock.create_topic.return_value = topic_mock
    yield discovery_mock, topic_factory_mock, receiver_mock, topic_mock
    discovery_patcher.stop()
    topic_factory_patcher.stop()
    receiver_patcher.stop()


@pytest.fixture
def messaging_system_instance(joystick_observer, mocked_pymessaginglib):
    """TODO: Document."""
    with patch(MODULE_STRING + 'JoystickInputTopicObserver',
               return_value=joystick_observer):
        msg_sys = MessagingSystem()
        yield msg_sys
    if msg_sys.is_alive():
        msg_sys.stop()


@pytest.fixture
def running_messaging_system(messaging_system_instance):
    """TODO: Document."""
    messaging_system_instance.start()
    return messaging_system_instance


@pytest.fixture(params=[
    # Invalid data
    [], '',
    # Empty data dict
    {},
    # Invalid types on correct keys
    {'button_event': object()}, {'axis_event': object()},
    # Invalid values on correct keys
    {'button_event': 'invalid string'},
    {'button_event': 'almostvalid;string'},
    {'axis_event': ['invalid string']},
    {'axis_event': ['almostvalid;string']},
    # Partially invalid values
    {'axis_event': ['LYA;-0.85', 'invalid string']},
    # Valid values
    {'button_event': 'ABT;P'},
    {'axis_event': ['LYA;-0.85']},
    {'axis_event': ['LYA;-0.85', 'LXA;0.34']},
    {'button_event': 'ABT;P', 'axis_event': ['LYA;-0.85']}],
    ids=['[]', "''", '{}', "{'button_event': object()}",
         "{'axis_event': object()}", "{'button_event': 'invalid string'}",
         "{'button_event': 'almostvalid;string'}",
         "{'axis_event': ['invalid string']}",
         "{'axis_event': ['almostvalid;string']}",
         "{'axis_event': ['LYA;-0.85', 'invalid string']}",
         "{'button_event': 'ABT;P'}",
         "{'axis_event': ['LYA;-0.85']}",
         "{'axis_event': ['LYA;-0.85', 'LXA;0.34']}",
         "{'button_event': 'ABT;P', 'axis_event': ['LYA;-0.85']}"])
def message(request):
    """TODO: Document."""
    return request.param


def test_messaging_system_initialization(messaging_system_instance,
                                         joystick_observer,
                                         mocked_pymessaginglib):
    """
    Checks system initialization.

    Expected:
    1.  It should create an attribute called: "_discovery_master" which
        is a DiscoveryMaster instance.
    2.  It should create an attribute called: "_listening_topics" which
        is a dict.
    3.  It should create a Topic object using the topic_factory with
        "/client/joystick/" as its only input argument.
    4.  The "_listening_topics" dict should have a "/client/joystick/"
        key with the created Topic object as its value.
    5.  It should create a JoystickInputTopicObserver object, and attach
        it to the created Topic by its "attach" method.
    """
    msg_sys = messaging_system_instance
    discovery_mock, topic_factory_mock, receiver_mock, topic_mock\
        = mocked_pymessaginglib
    assert hasattr(msg_sys, '_discovery_master')
    assert isinstance(msg_sys._discovery_master, DiscoveryMaster)
    assert hasattr(msg_sys, '_listening_topics')
    assert isinstance(msg_sys._listening_topics, dict)
    assert topic_factory_mock.create_topic.called
    assert topic_factory_mock.create_topic.call_args\
        == call('/client/joystick/')
    assert '/client/joystick/' in msg_sys._listening_topics
    assert msg_sys._listening_topics['/client/joystick/'] is topic_mock
    assert hasattr(msg_sys, '_joystick_observer')
    assert msg_sys._joystick_observer == joystick_observer
    assert topic_mock.attach.called
    assert topic_mock.attach.call_args == call(joystick_observer)


def test_running_messaging_system(running_messaging_system,
                                  mocked_pymessaginglib):
    """
    Checks system start and running.

    Expected:
    1.  DiscoveryMaster.start() should have been called.
    2.  receiver.start() should have been called.
    3.  The "is_alive()" method should return True.
    """
    discovery_mock, topic_factory_mock, receiver_mock, topic_mock\
        = mocked_pymessaginglib
    assert discovery_mock.start.called
    assert receiver_mock.start.called
    assert running_messaging_system.is_alive()


def test_joysick_observer_message_arrival(joystick_observer, message):
    """Check message arrival behaviour."""
    ip = '127.0.0.1'
    callback_mock = Mock()
    callback_mock.reset_mock()
    event_bus.register_callback('input-event', callback_mock, is_sync=True)
    if isinstance(message, dict):
        joystick_observer.on_message_arrival(message, ip)
        if len(message) > 0:
            called_count = 0
            if 'button_event' in message:
                if isinstance(message['button_event'], str):
                    str_pieces = message['button_event'].split(';')
                    if len(str_pieces) > 1 and len(str_pieces[0]) == 3:
                        # valid event, check call.
                        assert call(message['button_event'])\
                            in callback_mock.call_args_list
                        called_count += 1
                else:
                    assert not callback_mock.called
            if 'axis_event' in message\
                    and isinstance(message['axis_event'], list):
                for index, event_str in enumerate(message['axis_event']):
                    if not isinstance(event_str, str):
                        continue
                    str_pieces = event_str.split(';')
                    if len(str_pieces) > 1 and len(str_pieces[0]) == 3:
                        # valid event, check call.
                        assert call(event_str) in callback_mock.call_args_list
                        called_count += 1
            assert callback_mock.call_count == called_count
        else:
            assert not callback_mock.called
    else:
        try:
            joystick_observer.on_message_arrival(message, ip)
        except BaseException:
            pytest.fail("It shouldn't raise an exception")
        finally:
            event_bus.unregister_callback('input-event', callback_mock)
        assert not callback_mock.called
    event_bus.unregister_callback('input-event', callback_mock)


def test_messaging_system_stop(running_messaging_system,
                               mocked_pymessaginglib):
    """Stop checking."""
    discovery_mock, topic_factory_mock, receiver_mock, topic_mock\
        = mocked_pymessaginglib
    running_messaging_system.stop()
    assert discovery_mock.stop.called
    assert receiver_mock.stop.called
    assert running_messaging_system.is_alive() is False
