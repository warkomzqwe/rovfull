"""
General utilities module.
"""

import threading
import socket
from abc import ABCMeta, abstractmethod
from time import time
from datetime import datetime
from rov_event_bus.bus import *


class RepeatedTimer(object):
    '''
    Periodic function call class.
    Creates a thread that calls a function at some given rate until
    stop() is called.
    '''
    def __init__(self, interval, function, *args, **kwargs):
        self._timer = None
        self.interval = interval
        self.function = function
        self.args = args
        self.kwargs = kwargs
        self.is_running = False
        self.start()

    def _run(self):
        self.is_running = False
        self.start()
        self.function(*self.args, **self.kwargs)

    def start(self):
        if not self.is_running:
            self._timer = threading.Timer(self.interval, self._run)
            self._timer.start()
            self.is_running = True

    def stop(self):
        self._timer.cancel()
        if self._timer.is_alive():
            self._timer.join()
        self.is_running = False


# Observer pattern classes
class Observer(object):
    __metaclass__ = ABCMeta

    def __init__(self):
        super(Observer, self).__init__()

    def notify(self, source, time, new_value):
        self.observed_value_changed(source, time, new_value)

    @abstractmethod
    def observed_value_changed(self, source, time, new_value):
        pass


class Observable(object):
    def __init__(self):
        super(Observable, self).__init__()
        self._observed_value = None
        self._last_updated_time = time()
        self._observers = set()

    @property
    def observed_value(self):
        return self._observed_value

    @observed_value.setter
    def observed_value(self, new_value):
        self._observed_value = new_value
        self._last_updated_time = time()
        self._notify_observers()

    def _notify_observers(self):
        for observer in self._observers:
            observer.notify(self, self._last_updated_time,
                            self._observed_value)

    def attach_observer(self, observer):
        if isinstance(observer, Observer):
            self._observers.add(observer)
        else:
            raise TypeError

    def detach_observer(self, observer):
        self._observers.discard(observer)

    def force_notify(self):
        self.observed_value = self._observed_value


class RealTimeGenerator(object):
    DEFAULT_TICK_INTERVAL = 0.1

    def __init__(self):
        super(RealTimeGenerator, self).__init__()
        self.__time = 0.0
        self.__interval = self.DEFAULT_TICK_INTERVAL
        self.__timer = RepeatedTimer(self.__interval, self.__on_tick)
        self.__mainThread = threading.current_thread()
        self.__timer.start()
        self.__on_tick()

    def __on_tick(self):
        if not self.__mainThread.is_alive():
            self.stop()
        tick_time = int(time())
        if tick_time > self.__time:
            self.__time = tick_time
            # generate signal / notify observer
            event_bus.trigger('rtc-second-elapsed', self)

    def stop(self):
        self.__timer.stop()

    def get_time_str(self, with_seconds=True):
        return datetime.fromtimestamp(self.__time).strftime(
            '%H:%M:%S' if with_seconds else '%H:%M')

    def get_date_str(self):
        return datetime.fromtimestamp(self.__time).strftime('%d/%m/%Y')


def get_ip():
    """
    This function is necessary to get the PC's ip to publish it
    using the zeroconf protocol
    """
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        # doesn't even have to be reachable
        s.connect(('10.255.255.255', 1))
        IP = s.getsockname()[0]
    except Exception:
        IP = '127.0.0.1'
    finally:
        s.close()
    return IP
