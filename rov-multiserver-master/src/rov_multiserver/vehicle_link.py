"""Vehicle Link abstract class."""
import asyncio
import sys
import os
import csv
import threading
import time
from typing import Union, Optional, Any, Dict

from datetime import datetime
from abc import ABCMeta, abstractmethod

from pymessaginglib.MessagingSystem import receiver
from pymessaginglib.Topic import topic_factory, Topic, TopicObserver
from rov_multiserver.vehicle_data import VehicleState, VehicleTelemetryData
from rov_logger.logger import get_rov_logger
from rov_event_bus.bus import *
from rov_multiserver.vehicle_link_base import IVehicleLink
from rov_multiserver.vehicle_link_configuration_listener import \
    VehicleLinkConfigurationListener
from rov_multiserver.vehicle_settings import VehicleSettingsSubject

logger = get_rov_logger()

if sys.version_info[1] > 6:
    asyncio_all_tasks = asyncio.all_tasks
    asyncio_current_task = asyncio.current_task
else:
    asyncio_all_tasks = asyncio.Task.all_tasks
    asyncio_current_task = asyncio.Task.current_task

# Module defaults
DEFAULT_CAM_TILT_STEP_ANGLE = 7.5
DEFAULT_CAM_TILT_ANGLE_MAX = 90
DEFAULT_CAM_TILT_ANGLE_MIN = -90
INPUT_GAIN_LIMIT = 1.0
LIGHTS_MAX_LEVEL = 100.0
PITCH_GAIN_UP = 0.8
PITCH_GAIN_DOWN = 1.0
CAM_TILT_PUSH_START_DELAY = 0.5
CAM_TILT_PUSH_STEP_INTERVAL = 0.1
JS_DEAD_ZONE = 0.2  # Joystick below 20% is 0


class DueElectricalMeasurementsObserver(TopicObserver):
    TOPIC_NAME = "/vehicle/psm_muxer/"
    TOPIC: Topic = topic_factory.create_topic(TOPIC_NAME)

    def __init__(self, telemetry):
        self.__telemetry = telemetry
        self.TOPIC.attach(self)

    def on_message_arrival(self, dic: dict, ip: str):
        if 'raw_voltage' in dic:
            self.__telemetry.voltage.set_volts(float(dic['raw_voltage']))
        if 'raw_current' in dic:
            self.__telemetry.current.set_amperes(float(dic['raw_current']))


class JoystickListener(TopicObserver):
    TOPIC_NAME = "/client/joystick/"
    TOPIC: Topic = topic_factory.create_topic(TOPIC_NAME)

    def __init__(self, joystick_map):
        if 'button_event' not in joystick_map or 'axis_event' not in \
                joystick_map:
            raise ValueError("'button_event' and 'axis_event' keys are "
                             "required for the joystick map")
        self._js_map: Dict[str, Dict[str, Any]] = joystick_map
        self.__max_msg_time = 0.0
        self.__mean_msg_time = 0.0
        self.__min_msg_time = 0.0
        self.__n_msgs = 0
        self.__mean_op_time = 0.0
        self.__max_op_time = 0.0
        self.__min_op_time = 0.0
        self.__n_ops = 0

    def on_message_arrival(self, dic: dict, ip: str):
        if isinstance(dic, dict) and len(dic) > 0:
            msg_start = time.perf_counter()
            if 'axis_event' in dic and isinstance(dic['axis_event'],
                                                  (list, tuple)):
                for event_str in dic['axis_event']:
                    try:
                        axis, intensity = event_str.split(';')
                        intensity = float(intensity)
                        if abs(intensity) < JS_DEAD_ZONE:
                            intensity = 0.0
                    except ValueError:
                        continue
                    if axis in self._js_map['axis_event']:
                        op_start = time.perf_counter()
                        self._js_map['axis_event'][axis](intensity)
                        op_time = time.perf_counter() - op_start
                        self.__update_op_time_stats(op_time)
            if 'button_event' in dic\
                    and isinstance(dic['button_event'], str):
                try:
                    button, state = dic['button_event'].split(';')
                    if button in self._js_map['button_event']:
                        if state in self._js_map['button_event'][button]:
                            op_start = time.perf_counter()
                            self._js_map['button_event'][button][state]()
                            op_time = time.perf_counter() - op_start
                            self.__update_op_time_stats(op_time)
                except ValueError:
                    pass
            msg_time = time.perf_counter() - msg_start
            self.__mean_msg_time = \
                (self.__mean_msg_time * self.__n_msgs + msg_time) / \
                (self.__n_msgs + 1)
            self.__max_msg_time = max(self.__max_msg_time, msg_time)
            self.__min_msg_time = min(self.__max_msg_time, msg_time)
            self.__n_msgs += 1

    def __update_op_time_stats(self, op_time):
        self.__mean_op_time = \
            (self.__mean_op_time * self.__n_ops + op_time) / \
            (self.__n_ops + 1)
        self.__max_op_time = max(self.__max_op_time, op_time)
        self.__min_op_time = min(self.__max_op_time, op_time)
        self.__n_ops += 1

    def attach(self):
        self.TOPIC.attach(self)

    def detach(self):
        self.TOPIC.detach(self)

    async def print_times(self):
        while True:
            await asyncio.sleep(5)
            logger.debug(f'JS Messages: {self.__n_msgs} (times min/mean/max: '
                         f'{self.__min_msg_time*1000.0:.1f}/'
                         f'{self.__mean_msg_time*1000.0:.1f}/'
                         f'{self.__max_msg_time*1000.0:.1f} ms)')
            logger.debug(f'JS Operations: {self.__n_ops} (times min/mean/max: '
                         f'{self.__min_op_time*1000.0:.1f}/'
                         f'{self.__mean_op_time*1000.0:.1f}/'
                         f'{self.__max_op_time*1000.0:.1f} ms)')


class VehicleLink(IVehicleLink, metaclass=ABCMeta):
    """Abstract base class for Vehicle communications."""

    SETTINGS_PUBLISHING_TOPIC = '/companion/settings/vehicle/'
    SETTINGS_PUBLISHING_PERIOD: float = 0.5
    HEARTBEAT_TIMEOUT = 5
    DEFAULT_HOST = '0.0.0.0'
    DEFAULT_PORT = 14550
    DEFAULT_FIXED_YAW_POWER = False
    DEFAULT_FIXED_YAW_POWER_FACTOR = 0.5

    def __init__(self, host=DEFAULT_HOST, port=DEFAULT_PORT):
        """Constructor method. TODO: Document."""
        super().__init__()
        self.__is_pushing_cam_tilt = False
        self.__pushing_cam_tilt_stopped = threading.Event()
        self.__client_state: Optional[str] = None
        self.__cam_tilt_step_angle = DEFAULT_CAM_TILT_STEP_ANGLE
        self.__cam_tilt_angle_max = DEFAULT_CAM_TILT_ANGLE_MAX
        self.__cam_tilt_angle_min = DEFAULT_CAM_TILT_ANGLE_MIN
        self._connected = False
        self._connection_event = asyncio.Event()  # "_connected" change
        self._stop_required = asyncio.Event()
        self._heartbeat_reported = asyncio.Event()
        self._new_client_state = asyncio.Event()
        self._new_data_queue = asyncio.Queue()
        self._loop = asyncio.get_event_loop()
        self.host = host
        self.port = port
        self.state = VehicleState()
        self.telemetry = VehicleTelemetryData()
        self.data_logger = VehicleLogger(self)
        self.__settings = VehicleSettingsSubject()
        self.__fixed_yaw_power: bool = self.settings.fixed_yaw_power
        self.__fixed_yaw_power_factor: Union[int, float] = \
            self.settings.fixed_yaw_power_factor
        self.__lights_on_off_control: bool = \
            self.settings.lights_on_off_control
        self.__acceleration_rate: float = \
            self.settings.acceleration_rate
        self.__acceleration_start_threshold: float = \
            self.settings.acceleration_start_threshold
        self.__acceleration_stop_threshold: float = \
            self.settings.acceleration_stop_threshold
        self.__deceleration_rate: float = \
            self.settings.deceleration_rate
        self.__deceleration_start_threshold: float = \
            self.settings.deceleration_start_threshold
        self.__deceleration_stop_threshold: float = \
            self.settings.deceleration_stop_threshold
        self.settings.start_watching_all_settings_with(
            self._on_setting_update_callback)
        self.__config_listener = VehicleLinkConfigurationListener(self)
        DueElectricalMeasurementsObserver(self.telemetry)
        receiver.start()
        if self.settings.vehicle_type == 'net_cleaner':  # net cleaner js map
            js_map = {'axis_event': {
                          'LXA': self._on_move_yaw_cmd,
                          'LYA': self._on_move_surge_cmd,
                          'RXA': self._on_primary_horizontal_movement_cmd,
                          'RYA': self._on_primary_vertical_movement_cmd,
                          'XPD': self._on_secondary_horizontal_movement_cmd,
                          'YPD': self._on_secondary_vertical_movement_cmd},
                      'button_event': {
                          'STB': {'P': self._on_pre_cycle_lights_cmd,
                                  'R': self._on_cycle_lights_cmd},
                          'RBB': {'P': self._on_cam_tilt_up_push,
                                  'R': self._on_cam_tilt_up_release},
                          'RTB': {'P': self._on_cam_tilt_down_push,
                                  'R': self._on_cam_tilt_down_release},
                          'YBT': {'R': self._on_toggle_stabilize_mode_cmd},
                          'XBT': {'R': self._on_toggle_depth_hold_mode_cmd},
                          'LBB': {'R': self._on_input_gain_up_cmd},
                          'LTB': {'R': self._on_input_gain_down_cmd},
                          'ABT': {'R': self._on_do_toggle_armed_mode},
                          'BBT': {'R': self._on_toggle_net_cleaning_mode_cmd},
                          'RSB': {
                              # 'P': lambda: None,
                              'R': self._on_raise_net_cleaning_mode_gain_cmd},
                          'LSB': {
                              # 'P': lambda: None,
                              'R': self._on_reduce_net_cleaning_mode_gain_cmd}
                                        }
                      }
        else:  # General purpose ROV js map
            js_map = {'axis_event': {
                          'LXA': self._on_move_yaw_cmd,
                          'LYA': self._on_move_surge_cmd,
                          'RXA': self._on_primary_horizontal_movement_cmd,
                          'RYA': self._on_primary_vertical_movement_cmd,
                          'XPD': self._on_secondary_horizontal_movement_cmd,
                          'YPD': self._on_secondary_vertical_movement_cmd},
                      'button_event': {
                          'STB': {'P': self._on_pre_cycle_lights_cmd,
                                  'R': self._on_cycle_lights_cmd},
                          'RBB': {'P': self._on_cam_tilt_up_push,
                                  'R': self._on_cam_tilt_up_release},
                          'RTB': {'P': self._on_cam_tilt_down_push,
                                  'R': self._on_cam_tilt_down_release},
                          'YBT': {'R': self._on_toggle_stabilize_mode_cmd},
                          'XBT': {'R': self._on_toggle_depth_hold_mode_cmd},
                          'LBB': {'R': self._on_input_gain_up_cmd},
                          'LTB': {'R': self._on_input_gain_down_cmd},
                          'ABT': {'R': self._on_do_arm_cmd},
                          'BBT': {'R': self._on_do_disarm_cmd},
                          'RSB': {'P': self._on_arm_open_cmd,
                                  'R': self._on_arm_stop_cmd},
                          'LSB': {'P': self._on_arm_close_cmd,
                                  'R': self._on_arm_stop_cmd}}}
        if self.settings.lights_controller == 'arduino' or \
                self.settings.lights_controller == 'psd':
            js_map['button_event'].pop('STB')
            logger.warning("The lights will be controlled by another process")
        self.__js_listener = JoystickListener(js_map)
        self.__pre_cycle_lights_push_time: Optional[Union[float, int]] = None

    async def _connection_loop(self):
        while True:
            try:
                if not self._connected:
                    current_task = self._loop.run_in_executor(
                        None, self.start_connection)
                    await current_task
                    # TODO: handle possible exceptions
                    self._connected = current_task.result()
                    if self._connected:
                        self._connection_event.set()
                        logger.info("Vehicle connection established")
                        await self._loop.run_in_executor(
                            None, self.state.force_notify)
                else:
                    self._connection_event.clear()
                    current_task = asyncio.ensure_future(
                        self._connection_event.wait())
                    await current_task
                current_task = asyncio.ensure_future(
                    asyncio.sleep(0.5))
                await current_task
            except asyncio.CancelledError:
                # cancel everything...
                current_task.cancel()
                await current_task
                break

    async def _process_new_data(self):
        while True:
            try:
                current_task = asyncio.ensure_future(
                    self._new_data_queue.get())
                await current_task
                new_data = current_task.result()
                # TODO: handle possible exceptions
                self._do_process_data(new_data)
            except asyncio.CancelledError:
                # cancel everything...
                current_task.cancel()
                await current_task
                break

    async def _acquire_data(self):
        while True:
            try:
                if not self._connected:
                    self._connection_event.clear()
                    current_task = asyncio.ensure_future(
                        self._connection_event.wait())
                    await current_task
                else:
                    current_task = self._loop.run_in_executor(
                        None, self._do_acquire_data)
                    await current_task
                    new_data = current_task.result()
                    if new_data is not None:
                        current_task = self._new_data_queue.put(new_data)
                        await current_task
            except asyncio.CancelledError:
                # cancel everything...
                current_task.cancel()
                await current_task
                break

    async def _check_heartbeat_timeout(self):
        heartbeat_timeout = asyncio.Event()
        while True:
            try:
                if not self._connected:
                    self._connection_event.clear()
                    current_task = asyncio.ensure_future(
                        self._connection_event.wait())
                    await current_task
                else:
                    heartbeat_timeout.clear()
                    self._heartbeat_reported.clear()
                    report_disconnected = self._loop.call_later(
                        self.HEARTBEAT_TIMEOUT, heartbeat_timeout.set)
                    timeout_waiting_task = asyncio.ensure_future(
                        heartbeat_timeout.wait())
                    report_waiting_task = asyncio.ensure_future(
                        self._heartbeat_reported.wait())
                    done, pending = await asyncio.wait(
                        [timeout_waiting_task, report_waiting_task],
                        return_when=asyncio.FIRST_COMPLETED)
                    if report_waiting_task in done:
                        report_disconnected.cancel()
                    elif timeout_waiting_task in done:
                        self._connected = False
                        logger.info("Vehicle Link disconnected: "
                                    "Heartbeat timeout")
                        self._connection_event.set()
                    for task in pending:  # Create tasks on every iteration
                        task.cancel()
            except asyncio.CancelledError:
                # cancel everything...
                awaitable_tasks = set()
                if 'timeout_waiting_task' in locals() \
                        and not timeout_waiting_task.done():
                    timeout_waiting_task.cancel()
                    awaitable_tasks.add(timeout_waiting_task)
                if 'report_waiting_task' in locals() \
                        and not report_waiting_task.done():
                    report_waiting_task.cancel()
                    awaitable_tasks.add(report_waiting_task)
                if 'current_task' in locals() \
                        and not current_task.done():
                    current_task.cancel()
                    awaitable_tasks.add(current_task)
                if len(awaitable_tasks) > 0:
                    await asyncio.wait(awaitable_tasks)

    async def _send_heartbeat(self):
        while True:
            try:
                if self._connected:
                    current_task = self._loop.run_in_executor(
                        None, self._do_send_heartbeat)
                    await current_task
                    # current_task = asyncio.sleep(1)
                    current_task = asyncio.sleep(0.02)  # 50 Hz
                    await current_task
                else:
                    current_task = self._connection_event.wait()
                    await current_task
            except asyncio.CancelledError:
                if not current_task.done():
                    current_task.cancel()
                await current_task

    async def _datalogger_loop(self):
        data_logger_is_running = False
        while True:
            try:
                if self._connected:
                    if not data_logger_is_running:
                        logger.debug('Vehicle connected, starting data logger')
                        asyncio.ensure_future(self.data_logger.start_async())
                        data_logger_is_running = True
                    current_task = asyncio.ensure_future(
                        self._connection_event.wait())
                    await current_task
                else:
                    if data_logger_is_running:
                        logger.debug('Vehicle disconnected, '
                                     'stopping data logger')
                        await self.data_logger.stop_async()
                        data_logger_is_running = False
                    current_task = asyncio.ensure_future(
                        self._connection_event.wait())
                    await current_task
            except asyncio.CancelledError:
                await self.data_logger.stop_async()
                current_task.cancel()
                await current_task

    async def _publish_settings(self):
        """
        Settings publishing coroutine loop.

        This coroutine publishes periodically the current VehicleLink's
        setting values to the client.
        """
        try:
            while True:
                if self.__client_state == 'settings_started':
                    topic: Topic = topic_factory.create_topic(
                        VehicleLink.SETTINGS_PUBLISHING_TOPIC)
                    topic.send(
                        {'yaw_fixed_power': self.fixed_yaw_power_factor
                         if self.fixed_yaw_power else 0.0,
                         'speed_smoother_acceleration_rate':
                             self.acceleration_rate,
                         'speed_smoother_acceleration_start_threshold':
                             self.acceleration_start_threshold,
                         'speed_smoother_acceleration_stop_threshold':
                             self.acceleration_stop_threshold,
                         'speed_smoother_deceleration_rate':
                             self.deceleration_rate,
                         'speed_smoother_deceleration_start_threshold':
                             self.deceleration_start_threshold,
                         'speed_smoother_deceleration_stop_threshold':
                             self.deceleration_stop_threshold})
                self._new_client_state.clear()
                try:
                    await asyncio.wait_for(
                        self._new_client_state.wait(),
                        VehicleLink.SETTINGS_PUBLISHING_PERIOD)
                except asyncio.TimeoutError:
                    continue
        except asyncio.CancelledError:
            pass

    def _publish_net_cleaning_state(self):
        topic: Topic = topic_factory.create_topic(
                    '/vehicle/net_cleaning_state/')
        topic.send(
            {'net_cleaning_mode': self.is_net_cleaning_mode_on()})

    async def _periodically_publish_net_cleaning_state(self):
        if self.settings.vehicle_type != 'net_cleaner':
            return
        try:
            while True:
                self._publish_net_cleaning_state()
                await asyncio.sleep(2.0)
        except asyncio.CancelledError:
            pass

    async def _main_loop(self):
        tasks = set()
        tasks.add(asyncio.ensure_future(self._acquire_data()))
        tasks.add(asyncio.ensure_future(self._process_new_data()))
        tasks.add(asyncio.ensure_future(self._check_heartbeat_timeout()))
        tasks.add(asyncio.ensure_future(self._connection_loop()))
        tasks.add(asyncio.ensure_future(self._send_heartbeat()))
        tasks.add(asyncio.ensure_future(self._datalogger_loop()))
        tasks.add(asyncio.ensure_future(self._publish_settings()))
        tasks.add(asyncio.ensure_future(
            self._periodically_publish_net_cleaning_state()))
        tasks.add(asyncio.ensure_future(self.__js_listener.print_times()))
        tasks.add(asyncio.ensure_future(
            self.settings.config_file_watcher_loop()))
        try:
            current_task = self._stop_required.wait()
            await current_task
        except asyncio.CancelledError:
            # cancel everything...
            current_task.cancel()
        finally:
            awaitable_tasks = set()
            for task in tasks:
                if not task.done():
                    task.cancel()
                    awaitable_tasks.add(task)
            if len(awaitable_tasks) > 0:
                await asyncio.wait(awaitable_tasks,
                                   return_when=asyncio.ALL_COMPLETED)

    def report_heartbeat(self):
        """Sets the _heartbeat_reported asyncio.Event."""
        if hasattr(self, '_heartbeat_reported'):
            self._loop.call_soon_threadsafe(self._heartbeat_reported.set)

    def start(self, force_now=False):
        """
        Starts the VehicleLink main loop.

        Main loop tasks:

        *   Try to establish the connection. Retry on failure or
            disconnection.
        *   Checks for new data, put it on a queue.
        *   Process data in queue.
        *   Register data arrival time, check when a measurement is
            too old.

        This method returns inmediatly.
        """
        if force_now and not self._loop.is_running():
            self._loop.run_in_executor(None, self._loop.run_forever)
        self._main_task = asyncio.run_coroutine_threadsafe(
            self._main_loop(), self._loop)
        self._register_callbacks()

    def stop(self, wait=False):
        """Stops the VehicleLink main loop."""
        if self._loop.is_closed():
            return
        self._loop.call_soon_threadsafe(self._stop_required.set)
        if self._connected:
            self.disconnect()
        self._unregister_callbacks()
        self.__pushing_cam_tilt_stopped.set()
        self.__config_listener.detach()
        if wait:
            if self._loop.is_running():
                if hasattr(self, '_main_task'):
                    self._main_task.result()
                else:
                    waiter = asyncio.run_coroutine_threadsafe(
                        self._stop_required.wait(), self._loop)
                    waiter.result()
            else:
                if hasattr(self, '_main_task'):
                    self._loop.run_until_complete(
                        asyncio.wrap_future(self._main_task))
                else:
                    self._loop.run_until_complete(self._stop_required.wait())

    @property
    def loop(self):
        """TODO: Document."""
        return self._loop

    @loop.setter
    def loop(self, new_loop):
        if not isinstance(new_loop, asyncio.BaseEventLoop):
            return
        if self._loop.is_running():
            tasks = asyncio_all_tasks(loop=self._loop)
            for task in tasks:
                self._loop.call_soon_threadsafe(task.cancel)
            self._loop.call_soon_threadsafe(self._loop.stop)
            while self._loop.is_running():
                pass
        if not self._loop.is_closed():
            self._loop.close()
        self._stop_required = asyncio.Event()
        self._heartbeat_reported = asyncio.Event()
        self._new_data_queue = asyncio.Queue()
        self._loop = new_loop

    @property
    def connected(self) -> bool:
        return self._connected

    def _register_callbacks(self):
        event_bus.register_callback('client-state-changed',
                                    self._on_client_state_changed)
        self.__js_listener.attach()

    def _unregister_callbacks(self):
        callbacks_to_unregister = {
            'client-state-changed': self._on_client_state_changed}
        for event_name, callback in callbacks_to_unregister.items():
            try:
                event_bus.unregister_callback(event_name, callback)
            except ValueError:
                logger.warning(
                    f"No callback was registered for '{event_name}'")
        self.__js_listener.detach()

    def _on_move_yaw_cmd(self, intensity):
        if self.fixed_yaw_power:
            if intensity != 0:
                self.turn_yaw(self.fixed_yaw_power_factor * 100.0,
                              intensity > 0)
            else:
                self.turn_yaw(0.0, True)
        else:
            self.turn_yaw(
                abs(intensity) * self.state.input_gain *
                INPUT_GAIN_LIMIT * 100.0, intensity > 0)

    def _on_move_surge_cmd(self, intensity):
        self.move_horizontal(
            abs(intensity) * self.state.input_gain *
            INPUT_GAIN_LIMIT * 100.0, intensity < 0)

    def _on_move_sway_cmd(self, intensity):
        self.move_lateral(
            abs(intensity) * self.state.input_gain *
            INPUT_GAIN_LIMIT * 100.0, intensity > 0)

    def _on_move_heave_cmd(self, intensity):
        self.move_vertical(
            abs(intensity) * self.state.input_gain *
            INPUT_GAIN_LIMIT * 100.0, intensity < 0)

    def _on_lights_intensity_cmd(self, intensity):
        if self.lights_on_off_control:
            step = 100.0
        else:
            step = 10.0
        if intensity > 0:
            new_level = self.state.lights_level + step
            if new_level > LIGHTS_MAX_LEVEL:
                new_level = LIGHTS_MAX_LEVEL
        elif intensity < 0:
            new_level = self.state.lights_level - step
            if new_level < 0:
                new_level = 0.0
        if intensity != 0:
            self.set_light(new_level)

    def _on_pre_cycle_lights_cmd(self):
        self.__pre_cycle_lights_push_time = time.time()

    def _on_cycle_lights_cmd(self):
        if self.__pre_cycle_lights_push_time is not None \
                and (time.time() - self.__pre_cycle_lights_push_time) < 1:
            if self.lights_on_off_control:
                step = 100.0
            else:
                step = 25.0
            new_level = self.state.lights_level + step
            if new_level > LIGHTS_MAX_LEVEL:
                new_level = 0
            self.set_light(new_level)
        self.__pre_cycle_lights_push_time = None

    def _on_move_pitch_cmd(self, intensity):
        pitch_gain = PITCH_GAIN_UP if intensity > 0 else PITCH_GAIN_DOWN
        self.turn_pitch(abs(intensity) * pitch_gain * 100.0,
                        intensity > 0)

    def _on_move_roll_cmd(self, intensity):
        self.turn_roll(abs(intensity) * self.state.input_gain *
                       INPUT_GAIN_LIMIT * 100.0, intensity > 0)

    def _on_primary_horizontal_movement_cmd(self, intensity):
        if self.is_stabilize_mode_on() or self.is_depth_hold_mode_on():
            self._on_move_sway_cmd(intensity)
        else:
            self._on_move_roll_cmd(intensity)

    def _on_primary_vertical_movement_cmd(self, intensity):
        if self.is_stabilize_mode_on() or self.is_depth_hold_mode_on():
            if not self.is_net_cleaning_mode_on():
                self._on_move_heave_cmd(intensity)
        else:
            self._on_move_pitch_cmd(intensity)

    def _on_secondary_horizontal_movement_cmd(self, intensity):
        if self.is_stabilize_mode_on() or self.is_depth_hold_mode_on():
            self._on_move_roll_cmd(intensity)
        else:
            self._on_move_sway_cmd(intensity)

    def _on_secondary_vertical_movement_cmd(self, intensity):
        if self.is_stabilize_mode_on() or self.is_depth_hold_mode_on():
            self._on_move_pitch_cmd(intensity)
        else:
            if not self.is_net_cleaning_mode_on():
                self._on_move_heave_cmd(intensity)

    def _on_cam_tilt_up_push(self):
        if not self.__is_pushing_cam_tilt:
            self.__pushing_cam_tilt_stopped.clear()
            self.__start_cam_tilt_continuous_push_up_thread()

    def _on_cam_tilt_down_push(self):
        if not self.__is_pushing_cam_tilt:
            self.__pushing_cam_tilt_stopped.clear()
            self.__start_cam_tilt_continuous_push_down_thread()

    def _on_cam_tilt_up_release(self):
        self.cam_tilt_step(True)
        self.__pushing_cam_tilt_stopped.set()

    def _on_cam_tilt_down_release(self):
        self.cam_tilt_step(False)
        self.__pushing_cam_tilt_stopped.set()

    def _on_toggle_stabilize_mode_cmd(self):
        if self.is_stabilize_mode_on():
            self.stop_stabilize_mode()
        else:
            self.start_stabilize_mode()

    def _on_toggle_depth_hold_mode_cmd(self):
        if self.is_depth_hold_mode_on():
            self.stop_depth_hold_mode()
        else:
            self.start_depth_hold_mode()

    def _on_input_gain_up_cmd(self):
        step = 0.1
        new_value = self.state.input_gain + step
        if new_value > 1.0:
            new_value = 1.0
        event_bus.trigger(
            'new-user-notification',
            "Input Gain: {:.0f}%".format(new_value * 100.0), 2, 2)
        self.state.input_gain = new_value

    def _on_input_gain_down_cmd(self):
        step = 0.1
        new_value = self.state.input_gain - step
        if new_value < 0.1:
            new_value = 0.1
        event_bus.trigger(
            'new-user-notification',
            "Input Gain: {:.0f}%".format(new_value * 100.0), 2, 2)
        self.state.input_gain = new_value

    def _on_do_arm_cmd(self):
        self.do_arm()

    def _on_do_disarm_cmd(self):
        self.do_disarm()

    def _on_do_toggle_armed_mode(self):
        self.toggle_armed()

    def _on_recording_started(self, rec_file_full_path, media_pipeline):
        # event_bus.trigger(
        #         'new-user-notification',
        #         "GRABACIÓN INICIADA", 3, 3)
        logger.debug(
            "Tracking telemetry and state to subtitles for: {}".format(
                rec_file_full_path))
        # this could be called from another thread without a main loop
        asyncio.set_event_loop(self._loop)
        self._subs_logger = VehicleLogger(self, frequency=2, data_fmt='srt')
        self._subs_logger.start()

    def _on_arm_open_cmd(self):
        self.open_arm()

    def _on_arm_close_cmd(self):
        self.close_arm()

    def _on_arm_stop_cmd(self):
        self.stop_arm()

    def _on_activate_net_cleaning_mode_cmd(self):
        self.set_net_cleaning_mode(True)
        event_bus.trigger(
            'new-user-notification',
            "Net-cleaning mode: ON", 2, 2)
        self._publish_net_cleaning_state()

    def _on_deactivate_net_cleaning_mode_cmd(self):
        self.set_net_cleaning_mode(False)
        event_bus.trigger(
            'new-user-notification',
            "Net-cleaning mode: OFF", 2, 2)
        self._publish_net_cleaning_state()

    def _on_toggle_net_cleaning_mode_cmd(self):
        self.set_net_cleaning_mode(not self.is_net_cleaning_mode_on())
        state = "ON" if self.is_net_cleaning_mode_on() else "OFF"
        event_bus.trigger(
            'new-user-notification',
            f"Net-cleaning mode: {state}", 2, 2)
        self._publish_net_cleaning_state()

    def _on_raise_net_cleaning_mode_gain_cmd(self):
        self.step_net_cleaning_mode_gain_up()

    def _on_reduce_net_cleaning_mode_gain_cmd(self):
        self.step_net_cleaning_mode_gain_down()

    def _on_client_state_changed(self, new_state: str):
        """
        Client's state change callback method.

        It will be called every time the client reports a new state,
        for this object to do something whenever necessary.

        Parameters
        ----------
        new_state: str
            A string with the name of the new reported state. It could be
            one of the following:
            - live_camera_started
            - gallery_started
            - settings_started
            - app_closed
        """
        self.__client_state = new_state
        self.loop.call_soon_threadsafe(self._new_client_state.set)

    def _on_setting_update_callback(self, setting_name: str,
                                    setting_value: Any):
        if setting_name == 'fixed_yaw_power':
            self.__fixed_yaw_power = setting_value
        elif setting_name == 'fixed_yaw_power_factor':
            self.__fixed_yaw_power_factor = setting_value
        elif setting_name == 'speed_smoother_acceleration_rate':
            self.__acceleration_rate = setting_value
        elif setting_name == 'speed_smoother_acceleration_start_threshold':
            self.__acceleration_start_threshold = setting_value
        elif setting_name == 'speed_smoother_acceleration_stop_threshold':
            self.__acceleration_stop_threshold = setting_value
        elif setting_name == 'speed_smoother_deceleration_rate':
            self.__deceleration_rate = setting_value
        elif setting_name == 'speed_smoother_deceleration_start_threshold':
            self.__deceleration_start_threshold = setting_value
        elif setting_name == 'speed_smoother_deceleration_stop_threshold':
            self.__deceleration_stop_threshold = setting_value
        else:
            return
        logger.debug(f"Updated {setting_name}: {setting_value}")

    def cam_tilt_step(self, step_up):
        sign = 1.0 if step_up else -1.0
        step = self.cam_tilt_step_angle * sign
        new_angle = self.state.cam_tilt_angle + step
        if new_angle > self.cam_tilt_angle_max:
            new_angle = self.cam_tilt_angle_max
        elif new_angle < self.cam_tilt_angle_min:
            new_angle = self.cam_tilt_angle_min
        self.set_camera_tilt(new_angle)
        degree = u'\u00b0'
        event_bus.trigger(
            'new-user-notification',
            "New camera tilt: {}".format(new_angle) + degree, 2, 2)
        self.state.cam_tilt_angle = new_angle

    @property
    def cam_tilt_angle_max(self):
        return self.__cam_tilt_angle_max

    @cam_tilt_angle_max.setter
    def cam_tilt_angle_max(self, value):
        pass  # Read-only right now

    @property
    def cam_tilt_angle_min(self):
        return self.__cam_tilt_angle_min

    @cam_tilt_angle_min.setter
    def cam_tilt_angle_min(self, value):
        pass  # Read-only right now

    @property
    def cam_tilt_step_angle(self):
        return self.__cam_tilt_step_angle

    @cam_tilt_step_angle.setter
    def cam_tilt_step_angle(self, value):
        pass

    @property
    def fixed_yaw_power(self) -> bool:
        """
        Flag to specify if the power used in yaw is fixed or influenced by
        the input gain.
        Returns
        -------
        bool
            True if the yaw turning speed is fixed. False otherwise.
        """
        return self.__fixed_yaw_power

    @fixed_yaw_power.setter
    def fixed_yaw_power(self, value: bool):
        self.__fixed_yaw_power = bool(value)
        self.settings.fixed_yaw_power = self.fixed_yaw_power

    @property
    def fixed_yaw_power_factor(self) -> float:
        """
        The amount of power used to turn yaw when
        VehicleLink.fixed_yaw_power is True. It should be > 0.0 and <= 1.0

        Returns
        -------
        float
            The factor value.
        """
        return self.__fixed_yaw_power_factor

    @fixed_yaw_power_factor.setter
    def fixed_yaw_power_factor(self, value: Union[int, float]):
        if value <= 0 or value > 1:
            logger.warning("Trying to set a fixed yaw power factor out of "
                           f"range: {value} {'<= 0' if value <= 0 else '> 1'}")
        else:
            self.__fixed_yaw_power_factor = float(value)
            self.settings.fixed_yaw_power_factor = \
                self.fixed_yaw_power_factor

    @property
    def lights_on_off_control(self):
        return self.__lights_on_off_control

    @lights_on_off_control.setter
    def lights_on_off_control(self, value):
        self.__lights_on_off_control = bool(value)
        self.settings.lights_on_off_control = self.lights_on_off_control

    @property
    def acceleration_rate(self) -> float:
        return self.__acceleration_rate

    @acceleration_rate.setter
    def acceleration_rate(self, value):
        self.__acceleration_rate = float(value)
        self.settings.acceleration_rate = self.acceleration_rate

    @property
    def acceleration_start_threshold(self) -> float:
        return self.__acceleration_start_threshold

    @acceleration_start_threshold.setter
    def acceleration_start_threshold(self, value):
        self.__acceleration_start_threshold = float(value)
        self.settings.acceleration_start_threshold = \
            self.acceleration_start_threshold

    @property
    def acceleration_stop_threshold(self) -> float:
        return self.__acceleration_stop_threshold

    @acceleration_stop_threshold.setter
    def acceleration_stop_threshold(self, value):
        self.__acceleration_stop_threshold = float(value)
        self.settings.acceleration_stop_threshold = \
            self.acceleration_stop_threshold

    @property
    def deceleration_rate(self) -> float:
        return self.__deceleration_rate

    @deceleration_rate.setter
    def deceleration_rate(self, value):
        self.__deceleration_rate = float(value)
        self.settings.deceleration_rate = self.deceleration_rate

    @property
    def deceleration_start_threshold(self) -> float:
        return self.__deceleration_start_threshold

    @deceleration_start_threshold.setter
    def deceleration_start_threshold(self, value):
        self.__deceleration_start_threshold = float(value)
        self.settings.deceleration_start_threshold = \
            self.deceleration_start_threshold

    @property
    def deceleration_stop_threshold(self) -> float:
        return self.__deceleration_stop_threshold

    @deceleration_stop_threshold.setter
    def deceleration_stop_threshold(self, value):
        self.__deceleration_stop_threshold = float(value)
        self.settings.deceleration_stop_threshold = \
            self.deceleration_stop_threshold

    @property
    def settings(self):
        return self.__settings

    @abstractmethod
    def start_connection(self):
        """Initializes vehicle connection."""

    @abstractmethod
    def _do_acquire_data(self):
        """
        Used in '__acquire_data' method.

        Attempts to acquire a single telemetry data package from the vehicle.
        Must return said package object if successful or None otherwise.
        """

    @abstractmethod
    def _do_process_data(self, package):
        """
        Used in '__process_new_data' method.

        Process a telemetry data package. Must return a dictionary with
        only the key/value pairs updated from that package.
        """

    @abstractmethod
    def disconnect(self):
        """
        Stop connection method.

        Stops the current connection, terminating all threads related
        to it.
        """

    @abstractmethod
    def start_stabilize_mode(self):
        """
        Starts "Stabilize Mode".

        Makes the Vehicle stabilize itself and maintain it's heading.
        """

    @abstractmethod
    def stop_stabilize_mode(self):
        """
        Stops "Stabilize Mode".

        Returns the Vehicle to "Manual mode".
        """

    @abstractmethod
    def is_stabilize_mode_on(self):
        """
        Getter method.

        Returns True if the Vehicle is in "Stabilize Mode"
        """

    @abstractmethod
    def start_depth_hold_mode(self):
        """
        Starts "Depth Hold Mode".

        Makes the Vehicle maintain its depth and heading.
        """

    @abstractmethod
    def stop_depth_hold_mode(self):
        """
        Stops "Depth Hold Mode".

        Returns the Vehicle to "Manual Mode".
        """

    @abstractmethod
    def is_depth_hold_mode_on(self):
        """
        Getter method.

        Returns True if the Vehicle is in hold depth mode
        """

    @abstractmethod
    def toggle_armed(self):
        """
        Toggle armed state.

        Toggles the vehicle motors between the "Armed" and "Disarmed"
        states.
        """

    @abstractmethod
    def do_arm(self):
        """
        Arm motors.

        Arms the vehicle, making it able to move.
        """

    @abstractmethod
    def do_disarm(self):
        """
        Disarm motors.

        Disarms the vehicle, making it unable to move.
        """

    @abstractmethod
    def is_armed(self):
        """
        Getter method.

        Returns True if the vehicle is armed.
        """

    @abstractmethod
    def stop_movement(self):
        """
        Stop the vehicle.

        Set all movement and turn speeds to 0.
        """

    @abstractmethod
    def move_horizontal(self, speed, go_forward):
        """
        Movement method.

        Makes the Vehicle move forwards or backwards.

        Arguments
        ---------
            speed       Float from 0 to 100 that represents a percent
                        of the maximum speed of the vehicle in that
                        direction.
            go_forward  Flag that indicates if the vehicle should move
                        forward or backwards.
        """

    @abstractmethod
    def move_lateral(self, speed, go_right):
        """
        Movement method.

        Makes the vehicle move sideways, without changing it's heading.

        Arguments
        ---------
            speed       Float from 0 to 100 that represents a percent
                        of the maximum speed of the vehicle in that
                        direction.
            go_right    Flag that indicates if the vehicle should move
                        towards it's left or it's right.
        """

    @abstractmethod
    def move_vertical(self, speed, go_up):
        """
        Movement method.

        Makes the vehicle move upwards or downwards.

        Arguments
        ---------
            speed       Float from 0 to 100 that represents a percent
                        of the maximum speed of the vehicle in that
                        direction.
            go_up       Flag that indicates if the vehicle should move
                        Upwards or Downwards.
        """

    @abstractmethod
    def turn_yaw(self, speed, turn_right, target_angle=None):
        """
        Turning method.

        Makes the vehicle turn on it's z-axis.

        Arguments
        ---------
            speed           Float from 0 to 100 that represents a
                            percentage of the maximum turn speed in
                            that direction.
            turn_right      Flag that indicates if the ROV should turn
                            towards its left or its right.
            target_angle    Float that represents the amount of degrees
                            to turn, relative to its current Yaw.
        """

    @abstractmethod
    def turn_pitch(self, speed, nose_up, target_angle=None):
        """
        Turning method.

        Makes the vehicle turn on it's y-axis.

        Arguments
        ---------
            speed           Float from 0 to 100 that represents a
                            percentage of the maximum turn speed in
                            that direction.
            nose_up         Flag that indicates the direction of the
                            pitch, if true the ROV will turn so its
                            nose moves "up", false will do the
                            opposite.
            target_angle    Float that represents the amount of degrees
                            to turn, relative to its current pitch.
        """

    @abstractmethod
    def turn_roll(self, speed, go_clockwise, target_angle=None):
        """
        Turning method.

        Makes the vehicle turn on it's x-axis

        Arguments
        ---------
            speed           Float from 0 to 100 that represents a
                            percentage of the maximum turn speed in
                            that direction.
            go_clockwise    Flag that indicates if the ROV should turn
                            clockwise or counter-clockwise.
            target_angle    Float that represents the amount of degrees
                            to turn, relative to its current roll.
        """

    @abstractmethod
    def set_light(self, intensity):
        """
        Sets the intensity of the lights.

        Arguments
        ---------
            intensity   Float from 0 to 100 that represents a percent
                        of the maximum illumination possible by the lights.
        """

    @abstractmethod
    def get_light(self):
        """
        Getter method.

        Returns the lights current intensity, in percent.
        """

    @abstractmethod
    def set_camera_tilt(self, angle):
        """
        Sets the inclination of the camera.

        Arguments
        ---------
            angle       Float from 0 to 90 that represents its position
                        in degrees, with 45 being centered and 0 being
                        looking down.
        """

    @abstractmethod
    def straighten_the_camera(self):
        """Resets the camera tilt to center."""

    @abstractmethod
    def open_arm(self):
        """Opens a connected Arm."""

    @abstractmethod
    def close_arm(self):
        """Closes a connected Arm."""

    @abstractmethod
    def stop_arm(self):
        """Stops the connected Arm movement."""

    @abstractmethod
    def set_net_cleaning_mode(self, activate: bool):
        """Set net cleaning mode (ON/OFF)."""

    @abstractmethod
    def is_net_cleaning_mode_on(self) -> bool:
        """Get net cleaning mode state (ON/OFF)."""

    @abstractmethod
    def set_net_cleaning_mode_gain(self, new_gain: Union[float, int]):
        """Set net cleaning mode gain (float between 0 and 1)."""

    @abstractmethod
    def get_net_cleaning_mode_gain(self) -> float:
        """Get current net cleaning mode gain."""

    def toggle_net_cleaning_mode(self):
        self.set_net_cleaning_mode(not self.is_net_cleaning_mode_on())

    def activate_net_cleaning_mode(self):
        self.set_net_cleaning_mode(True)

    def deactivate_net_cleaning_mode(self):
        self.set_net_cleaning_mode(False)

    def step_net_cleaning_mode_gain(self, step_up: bool):
        step = 0.1 if step_up else -0.1
        new_gain = round(self.get_net_cleaning_mode_gain() + step, 1)
        if new_gain > 1.0:
            new_gain = 1.0
        elif new_gain < 0.0:
            new_gain = 0.0
        self.set_net_cleaning_mode_gain(new_gain)

    def step_net_cleaning_mode_gain_up(self):
        self.step_net_cleaning_mode_gain(True)

    def step_net_cleaning_mode_gain_down(self):
        self.step_net_cleaning_mode_gain(False)

    def __cam_tilt_continuous_push_thread(self, direction_up: bool):
        start_delay = CAM_TILT_PUSH_START_DELAY
        step_interval = CAM_TILT_PUSH_STEP_INTERVAL
        logger.debug(f'Starting continuous cam tilt push thread, waiting '
                     f'{start_delay*1000} ms to start stepping...')
        self.__is_pushing_cam_tilt = True
        if self.__pushing_cam_tilt_stopped.wait(start_delay):
            logger.debug("Continuous cam tilt push stopped before stepping")
        else:
            logger.debug(f"Stepping cam tilt every {step_interval*1000} ms")
        while not self.__pushing_cam_tilt_stopped.wait(step_interval):
            prev_angle = self.state.cam_tilt_angle
            self.cam_tilt_step(direction_up)
            if prev_angle == self.state.cam_tilt_angle:
                logger.debug("Reached the max tilt (previous angle: "
                             f"{prev_angle}, new angle: "
                             f"{self.state.cam_tilt_angle}), stopping "
                             f"continuous cam tilt thread")
                self.__pushing_cam_tilt_stopped.set()
        logger.debug("Continuous cam tilt push thread ended")
        self.__is_pushing_cam_tilt = False

    def __start_cam_tilt_continuous_push_up_thread(self):
        threading.Thread(target=self.__cam_tilt_continuous_push_thread,
                         name='ContinuousCamTiltPushThread',
                         args=(True,)).start()

    def __start_cam_tilt_continuous_push_down_thread(self):
        threading.Thread(target=self.__cam_tilt_continuous_push_thread,
                         name='ContinuousCamTiltPushThread',
                         args=(False,)).start()


#    @abstractmethod
#    def face_north(self):
#        """
#        Stabilization method.
#
#        Stabilizes the Vehicle and makes it turn to the North while
#        maintaining depth.
#        """
#
#    @abstractmethod
#    def is_facing_north(self):
#        """
#        Getter method.
#
#        Returns True if the vehicle is facing North
#        """
#
#    @abstractmethod
#    def straighten(self):
#        """
#        Stabilization method.
#
#        Makes the vehicle maintain it's heading.
#        """
#
#    @abstractmethod
#    def is_straightened(self):
#        """
#        Getter method.
#
#        Returns True if the vehicle is straighten (pitch and roll
#        zero).
#        """


class VehicleLogger(object):
    """
    Vehicle data logger.

    Generate records of measurements and states related to vehicle
    activity with a given frequency.
    """

    _supported_formats = {'csv'}
    _fields = [
        {'name': 'navigation_mode',
         'type': 'state', 'extract_method': None},
        {'name': 'armed',
         'type': 'state', 'extract_method': None},
        {'name': 'leak_detected',
         'type': 'state', 'extract_method': None},
        {'name': 'lights_level',
         'type': 'state', 'extract_method': None},
        {'name': 'cam_tilt_angle',
         'type': 'state', 'extract_method': None},
        {'name': 'input_gain',
         'type': 'state', 'extract_method': None},
        {'name': 'roll',
         'type': 'telemetry', 'extract_method': 'get_degrees'},
        {'name': 'pitch',
         'type': 'telemetry', 'extract_method': 'get_degrees'},
        {'name': 'yaw',
         'type': 'telemetry', 'extract_method': 'get_degrees'},
        {'name': 'roll_speed',
         'type': 'telemetry', 'extract_method': 'get_degrees_s'},
        {'name': 'pitch_speed',
         'type': 'telemetry', 'extract_method': 'get_degrees_s'},
        {'name': 'yaw_speed',
         'type': 'telemetry', 'extract_method': 'get_degrees_s'},
        {'name': 'voltage',
         'type': 'telemetry', 'extract_method': 'get_volts'},
        {'name': 'current',
         'type': 'telemetry', 'extract_method': 'get_amperes'},
        {'name': 'pressure',
         'type': 'telemetry', 'extract_method': 'get_bar'},
        {'name': 'altitude',
         'type': 'telemetry', 'extract_method': 'get_meters'},
        {'name': 'power',
         'type': 'telemetry', 'extract_method': 'get_watts'},
        {'name': 'consumed_charge',
         'type': 'telemetry', 'extract_method': 'get_amperehours'},
        {'name': 'consumed_energy',
         'type': 'telemetry', 'extract_method': 'get_watthours'},
        {'name': 'depth',
         'type': 'telemetry', 'extract_method': 'get_meters'},
        {'name': 'heading',
         'type': 'telemetry', 'extract_method': 'get_degrees'}]
    DEFAULT_DATA_FORMAT = 'csv'
    DEFAULT_FREQUENCY = 1
    DEFAULT_SRT_FILE_NAME = '.last_subs.srt'
    DEFAULT_SSA_FILE_NAME = '.last_subs.ass'

    def __init__(self, vehicle, frequency=DEFAULT_FREQUENCY,
                 data_fmt=DEFAULT_DATA_FORMAT):
        """
        Constructor method.

        Parameters
        ----------
        vehicle : VehicleLink
            VehicleLink to extract the data.
        frequency : int, optional
            Data Logging frequency (in Hz). By default it's 1 Hz
        data_fmt : str, optional
            Data format to write in the file. Currently just the 'csv'
            format is supported. By default uses 'csv'.

        """
        self._vehicle = vehicle
        if data_fmt not in VehicleLogger._supported_formats:
            raise ValueError('{} format is not supported'.format(data_fmt))
        self._data_fmt = data_fmt
        self._frequency = frequency
        self._loop = asyncio.get_event_loop()
        self._running_main_loop = None

    def _create_data_file(self):
        self.__create_data_folder()
        filename = self._create_filename()
        datafile_full_path = self._data_path + '/' + filename
        self._field_names = [field['name'] for field in self._fields]
        self._field_names.insert(0, 'timestamp')
        with open(datafile_full_path, 'w+') as datafile:
            writer = csv.DictWriter(datafile, fieldnames=self._field_names)
            writer.writeheader()
        logger.debug(
            'Creating a new data file for the vehicle data logger: {}'.format(
                datafile_full_path))
        return datafile_full_path

    def _create_filename(self):
        return datetime.now().strftime('%Y-%m-%d_%H.%M.%S.csv')

    def __create_data_folder(self):
        self._data_path = os.path.expanduser('~') + '/.rov_multiserver/data'
        os.makedirs(self._data_path, exist_ok=True)

    def _get_data_from_vehicle_as_dict(self):
        data_dict = {'timestamp': time.time()}
        for field in self._fields:
            if field['type'] == 'telemetry':
                data_object_container = self._vehicle.telemetry
            elif field['type'] == 'state':
                data_object_container = self._vehicle.state
            else:
                logger.warning(
                    'Not processing field "{name}": '
                    '"{type}" type is not known'.format(name=field['name'],
                                                        type=field['type']))
                continue
            data_object = getattr(data_object_container, field['name'])
            if field['extract_method'] is None:
                # the attribute itself is the data
                data_dict[field['name']] = data_object
            else:
                #  the attribute has a getter to obtain the data
                data_getter = getattr(data_object, field['extract_method'])
                data_dict[field['name']] = data_getter()
        return data_dict

    def _append_data_csv(self, data_dict):
        with open(self._data_filename, 'a+') as datafile:
            writer = csv.DictWriter(datafile, fieldnames=self._field_names)
            writer.writerow(data_dict)

    def _append_data(self):
        data_dict = self._get_data_from_vehicle_as_dict()
        if self._data_fmt == 'csv':
            self._append_data_csv(data_dict)

    async def _append_data_async(self):
        await self._loop.run_in_executor(None, self._append_data)

    async def _main_loop(self):
        try:
            logger.info("Starting vehicle data logger")
            if self._data_fmt == 'csv':
                self._data_filename = await self._loop.run_in_executor(
                    None, self._create_data_file)
            tick = asyncio.Event()
            period = 1.0 / self._frequency
            tick.set()  # to start inmediatly
            while True:
                await tick.wait()
                appending_task = asyncio.ensure_future(
                    self._append_data_async())
                last_time = self._loop.time()
                tick.clear()
                timer_handle = self._loop.call_at(last_time + period, tick.set)
                await appending_task
        except asyncio.CancelledError:
            timer_handle.cancel()
        finally:
            logger.debug("Stopping vehicle data logger")

    async def start_async(self):
        if self._running_main_loop is not None \
                and not self._running_main_loop.done():
            return
        self._running_main_loop = asyncio.ensure_future(self._main_loop())

    def start(self):
        asyncio.run_coroutine_threadsafe(self.start_async(), self._loop)

    async def stop_async(self):
        if self._running_main_loop is not None \
                and not self._running_main_loop.done():
            self._running_main_loop.cancel()
            await self._running_main_loop

    def stop(self):
        if self._loop.is_running():
            asyncio.run_coroutine_threadsafe(
                self.stop_async(), self._loop).result()
