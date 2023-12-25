#!/usr/bin/env python
"""General utilities module tests."""
import pytest
import rov_multiserver.utils as utils
from time import time
from datetime import datetime
from math import floor
from unittest.mock import patch, Mock
from rov_event_bus.bus import *


@pytest.fixture
def fake_timer():
    """Passes a fake timer (mock)."""
    patcher = patch('rov_multiserver.utils.threading.Timer')
    fake_timer_class = patcher.start()
    fake_timer_instance = Mock()
    fake_timer_class.return_value = fake_timer_instance
    yield (fake_timer_class, fake_timer_instance)
    patcher.stop()


def test_repeated_timer(fake_timer):
    """Tests of RepeatedTimer class."""
    def test_func():
        pass

    fake_timer_class, fake_timer_instance = fake_timer
    repeated_timer = utils.RepeatedTimer(0.5, test_func)
    assert fake_timer_class.called
    args = fake_timer_class.call_args[0]
    assert len(args) == 2
    assert args[0] == 0.5
    assert args[1] == repeated_timer._run
    assert repeated_timer.function is test_func
    assert fake_timer_instance.start.called
    assert repeated_timer.is_running is True
    # Simulate a tick
    repeated_timer._run()
    assert repeated_timer.is_running is True  # It should be still running
    # Stop the RepeatedTimer
    repeated_timer.stop()
    assert repeated_timer.is_running is False
    assert fake_timer_instance.cancel.called


@patch('rov_multiserver.utils.time')
def test_real_time_generator(fake_time, fake_timer):
    """Tests of RealTimeGenerator class."""
    event_called = False
    event_args = None

    def on_rtc_second_elapsed(*args, **kwargs):
        nonlocal event_called
        nonlocal event_args
        event_called = True
        event_args = args

    event_bus.register_callback('rtc-second-elapsed',
                                on_rtc_second_elapsed, is_sync=True)
    fake_timer_class, fake_timer_instance = fake_timer
    time_now = floor(time())
    fake_time.return_value = time_now
    rtc = utils.RealTimeGenerator()
    assert fake_timer_class.called
    args = fake_timer_class.call_args[0]
    assert len(args) == 2
    assert args[0] == rtc.DEFAULT_TICK_INTERVAL
    assert callable(args[1])
    tick_function = args[1]
    assert rtc.get_time_str(True) == datetime.fromtimestamp(
        time_now).strftime('%H:%M:%S')
    assert rtc.get_time_str(False) == datetime.fromtimestamp(
        time_now).strftime('%H:%M')
    assert rtc.get_date_str() == datetime.fromtimestamp(
        time_now).strftime('%d/%m/%Y')
    time_now += 0.9
    fake_time.return_value = time_now
    event_called = False
    tick_function()
    assert not event_called
    time_now += 0.1
    fake_time.return_value = time_now
    event_args = None
    tick_function()
    assert event_called
    assert event_args[0] is rtc
    rtc.stop()
    assert fake_timer_instance.cancel.called
    with patch('rov_multiserver.utils.threading.current_thread')\
            as threading_mock:
        main_thread_mock = Mock()
        threading_mock.return_value = main_thread_mock
        main_thread_mock.is_alive.return_value = True
        fake_timer_instance.cancel.reset_mock()
        rtc = utils.RealTimeGenerator()
        tick_function = fake_timer_class.call_args[0][1]
        tick_function()
        assert not fake_timer_instance.cancel.called
        main_thread_mock.is_alive.return_value = False
        tick_function()
        assert fake_timer_instance.cancel.called
