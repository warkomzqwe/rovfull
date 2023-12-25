"""TODO: Document."""
import asyncio
import threading
import pytest
import sys
import unittest.mock as mock

from pymessaginglib.Topic import Topic, topic_factory
from rov_multiserver.vehicle_link import VehicleLink  # noqa E402

if sys.version_info[1] > 6:
    asyncio_all_tasks = asyncio.all_tasks
else:
    asyncio_all_tasks = asyncio.Task.all_tasks


class _DummyVehicleLink(VehicleLink):
    def _do_acquire_data(self):
        return {}

    def _do_process_data(self, package):
        pass

    def connect(self, ip, port):
        return True

    def disconnect(self):
        pass

    def straighten(self):
        pass

    def is_straightened(self):
        return False

    def start_stabilize_mode(self):
        pass

    def stop_stabilize_mode(self):
        pass

    def is_stabilize_mode_on(self):
        return False

    def start_depth_hold_mode(self):
        pass

    def stop_depth_hold_mode(self):
        pass

    def is_depth_hold_mode_on(self):
        return False

    def toggle_armed(self):
        pass

    def do_arm(self):
        pass

    def do_disarm(self):
        pass

    def is_armed(self):
        return False

    def stop_movement(self):
        pass

    def move_horizontal(self, speed, go_forward):
        pass

    def move_lateral(self, speed, go_right):
        pass

    def move_vertical(self, speed, go_up):
        pass

    def turn_yaw(self, speed, turn_right, target_angle=None):
        pass

    def turn_pitch(self, speed, nose_up, target_angle=None):
        pass

    def turn_roll(self, speed, go_clockwise, target_angle=None):
        pass

    def set_light(self, intensity):
        pass

    def get_light(self):
        pass

    def set_camera_tilt(self, angle):
        pass

    def straighten_the_camera(self):
        pass

    def face_north(self):
        pass

    def is_facing_north(self):
        return False

    def start_connection(self):
        pass


@pytest.fixture
def vehicle_link():
    """TODO: Document."""
    vehicle_link = _DummyVehicleLink()
    yield vehicle_link
    vehicle_link.stop()


def test_vehicle_link_creation(vehicle_link):
    """TODO: Document."""
    assert hasattr(vehicle_link, '_stop_required')
    assert isinstance(vehicle_link._stop_required, asyncio.Event)
    assert hasattr(vehicle_link, '_loop')
    assert isinstance(vehicle_link.loop, asyncio.BaseEventLoop)
    assert hasattr(vehicle_link, '_new_data_queue')
    assert isinstance(vehicle_link._new_data_queue, asyncio.Queue)


def test_vehicle_link_stop(threaded_event_loop, vehicle_link):
    """TODO: Document."""
    assert not vehicle_link._stop_required.is_set()
    vehicle_link.stop(wait=True)
    assert vehicle_link._stop_required.is_set()
    threaded_event_loop.call_soon_threadsafe(threaded_event_loop.stop)
    while threaded_event_loop.is_running():
        pass
    threaded_event_loop.close()
    vehicle_link.stop()  # Just for code coverage


@pytest.mark.skip
@pytest.mark.asyncio
async def test_acquire_data(event_loop, vehicle_link):
    """TODO: Document."""
    should_unblock = threading.Event()

    def do_aquire_data_response():
        nonlocal should_unblock
        yield {'new_data': 12345}
        should_unblock.wait()
        yield None

    vehicle_link._connected = True
    with mock.patch.object(vehicle_link, '_do_acquire_data') \
            as fake_do_acquire_data:
        fake_do_acquire_data.side_effect = do_aquire_data_response()
        main_task = asyncio.ensure_future(
            vehicle_link._acquire_data())
        new_data = await vehicle_link._new_data_queue.get()
        assert new_data == {'new_data': 12345}
        vehicle_link.stop()
        await main_task
        assert main_task.done()
        should_unblock.set()


@pytest.mark.skip
@pytest.mark.asyncio
async def test_process_data(event_loop, vehicle_link):
    """TODO: Document."""
    with mock.patch.object(vehicle_link, '_do_process_data') \
            as fake_do_process_data:
        main_task = asyncio.ensure_future(
            vehicle_link._process_new_data())
        assert not fake_do_process_data.called
        await asyncio.sleep(0.2)
        assert not fake_do_process_data.called
        await vehicle_link._new_data_queue.put({'new_data': 'foo'})
        while not vehicle_link._new_data_queue.empty():
            await asyncio.sleep(0.1)
        await asyncio.sleep(0.1)
        assert fake_do_process_data.called
        assert fake_do_process_data.call_args == mock.call({'new_data': 'foo'})
        vehicle_link.stop()
        await main_task


def test_report_heartbeat(threaded_event_loop, vehicle_link):
    """TODO: Document."""
    heartbeat_report_waiter = asyncio.run_coroutine_threadsafe(
        vehicle_link._heartbeat_reported.wait(),
        threaded_event_loop)
    vehicle_link.report_heartbeat()
    heartbeat_report_waiter.result(timeout=0.05)
    assert vehicle_link._heartbeat_reported.is_set()


@pytest.mark.skip
@pytest.mark.asyncio
async def test_check_heartbeat_timeout(event_loop, vehicle_link):
    """TODO: Document."""
    vehicle_link.HEARTBEAT_TIMEOUT = 0.2
    vehicle_link._connected = True
    main_task = asyncio.ensure_future(vehicle_link._check_heartbeat_timeout())
    await asyncio.sleep(0.25)
    assert not vehicle_link._connected
    vehicle_link._connected = True
    await asyncio.sleep(0.2)
    assert vehicle_link._connected
    vehicle_link.report_heartbeat()
    await asyncio.sleep(0.19)
    assert vehicle_link._connected
    vehicle_link.report_heartbeat()
    await asyncio.sleep(0.19)
    assert vehicle_link._connected
    vehicle_link.stop()
    await main_task


@pytest.mark.skip
@pytest.mark.asyncio
async def test_main_loop(event_loop, vehicle_link):
    """TODO: Document."""
    import time
    import random

    def connection_response(*args, **kwargs):
        time.sleep(0.05 * random.random() + 0.05)
        return mock.DEFAULT

    with mock.patch.object(
            vehicle_link, 'connect',
            side_effect=connection_response,
            return_value=False) as fake_connect:
        main_task = asyncio.ensure_future(vehicle_link._main_loop())
        assert not fake_connect.called
        await asyncio.sleep(0.15)
        assert fake_connect.called
        assert fake_connect.call_args == mock.call(vehicle_link.host,
                                                   vehicle_link.port)
        assert not vehicle_link._connected
        fake_connect.return_value = True
        await asyncio.sleep(0.2)
        assert vehicle_link._connected
    vehicle_link.stop()
    await main_task


def test_loop_property(threaded_event_loop, vehicle_link):
    """TODO: Document."""
    loop = asyncio.new_event_loop()
    vehicle_link.loop = None
    assert vehicle_link.loop is not loop
    vehicle_link.loop = loop
    asyncio.set_event_loop(loop)
    assert vehicle_link.loop is loop


@pytest.mark.skip
def test_vehicle_link_start(event_loop, vehicle_link):
    """TODO: Document."""
    vehicle_link.start()
    assert len(asyncio_all_tasks(loop=event_loop)) == 0
    event_loop.run_in_executor(None, event_loop.run_forever)
    assert len(asyncio_all_tasks(loop=event_loop)) > 0
    vehicle_link.stop(wait=True)
    event_loop.call_soon_threadsafe(event_loop.stop)
    while event_loop.is_running():
        pass
    event_loop.close()
    vehicle_link.stop()
    event_loop = asyncio.new_event_loop()
    asyncio.set_event_loop(event_loop)
    vehicle_link.loop = event_loop
    vehicle_link.start(True)
    assert event_loop.is_running()
    vehicle_link.stop()
    event_loop.call_soon_threadsafe(event_loop.stop)


@pytest.mark.parametrize("actual_angle,step_goes_up,expected_angle",
                         [(10, True, 25), (10, False, -5),
                          (88.43, True, 90), (88.43, False, 73.43),
                          (-82.5, True, -67.5), (-82.5, False, -90)])
def test_cam_tilt_step(vehicle_link, actual_angle, step_goes_up,
                       expected_angle):
    vehicle_link.start()
    with mock.patch.multiple(
            type(vehicle_link), new_callable=mock.PropertyMock,
            cam_tilt_step_angle=mock.DEFAULT, cam_tilt_angle_max=mock.DEFAULT,
            cam_tilt_angle_min=mock.DEFAULT) as property_mocks:
        property_mocks['cam_tilt_angle_max'].return_value = 90
        property_mocks['cam_tilt_angle_min'].return_value = -90
        property_mocks['cam_tilt_step_angle'].return_value = 15
        with mock.patch.multiple(vehicle_link, set_camera_tilt=mock.DEFAULT,
                                 state=mock.DEFAULT) as vehicle_mocks:
            vehicle_mocks['state'].cam_tilt_angle = actual_angle
            fake_set_camera_tilt = vehicle_mocks['set_camera_tilt']
            assert not fake_set_camera_tilt.called
            vehicle_link.cam_tilt_step(step_goes_up)
            assert fake_set_camera_tilt.called
            assert fake_set_camera_tilt.call_args == mock.call(expected_angle)


@pytest.mark.parametrize("yaw_fixed,yaw_fixed_power,input_gain,"
                         "input_intensity,expected_turn_yaw_args",
                         [(False, 1.0, 1.0, 0.5, (50.0, True)),
                          (False, 0.5, 1.0, 0.5, (50.0, True)),
                          (False, 1.0, 0.5, 0.5, (25.0, True)),
                          (False, 1.0, 1.0, 0.25, (25.0, True)),
                          (False, 0.5, 1.0, 0.25, (25.0, True)),
                          (False, 1.0, 0.5, 0.25, (12.5, True)),
                          (False, 1.0, 1.0, -0.5, (50.0, False)),
                          (False, 0.5, 1.0, -0.5, (50.0, False)),
                          (False, 1.0, 0.5, -0.5, (25.0, False)),
                          (False, 1.0, 1.0, -0.25, (25.0, False)),
                          (False, 0.5, 1.0, -0.25, (25.0, False)),
                          (False, 1.0, 0.5, -0.25, (12.5, False)),
                          (False, 1.0, 1.0, 0.0, (0.0, mock.ANY)),
                          (False, 0.5, 1.0, 0.0, (0.0, mock.ANY)),
                          (False, 1.0, 0.5, 0.0, (0.0, mock.ANY)),
                          (True, 1.0, 1.0, 0.5, (100.0, True)),
                          (True, 0.5, 1.0, 0.5, (50.0, True)),
                          (True, 1.0, 0.5, 0.5, (100.0, True)),
                          (True, 1.0, 1.0, 0.25, (100.0, True)),
                          (True, 0.5, 1.0, 0.25, (50.0, True)),
                          (True, 1.0, 0.5, 0.25, (100.0, True)),
                          (True, 1.0, 1.0, -0.5, (100.0, False)),
                          (True, 0.5, 1.0, -0.5, (50.0, False)),
                          (True, 1.0, 0.5, -0.5, (100.0, False)),
                          (True, 1.0, 1.0, -0.25, (100.0, False)),
                          (True, 0.5, 1.0, -0.25, (50.0, False)),
                          (True, 1.0, 0.5, -0.25, (100.0, False)),
                          (True, 1.0, 1.0, 0.0, (0.0, mock.ANY)),
                          (True, 0.5, 1.0, 0.0, (0.0, mock.ANY)),
                          (True, 1.0, 0.5, 0.0, (0.0, mock.ANY))])
def test_on_move_yaw_cmd(vehicle_link, yaw_fixed, yaw_fixed_power, input_gain,
                         input_intensity, expected_turn_yaw_args):
    turn_yaw_mock = mock.MagicMock()
    with mock.patch.multiple(
            type(vehicle_link),
            fixed_yaw_power=mock.PropertyMock(return_value=yaw_fixed),
            fixed_yaw_power_factor=mock.PropertyMock(
                return_value=yaw_fixed_power),
            turn_yaw=turn_yaw_mock):
        with mock.patch.object(vehicle_link, 'state') as state_mock:
            state_mock.input_gain = input_gain
            vehicle_link._on_move_yaw_cmd(input_intensity)
            assert turn_yaw_mock.called
            assert turn_yaw_mock.call_args == \
                   mock.call(*expected_turn_yaw_args)


@pytest.mark.parametrize('state', ['gallery_started', 'settings_started',
                                   'app_closed', 'live_camera_started'])
def test_on_client_state_changed(vehicle_link, state):
    with mock.patch.object(type(vehicle_link), 'loop') as loop_mock:
        assert not loop_mock.call_soon_threadsafe.called
        vehicle_link._on_client_state_changed(state)
        assert vehicle_link._VehicleLink__client_state == state
        assert loop_mock.call_soon_threadsafe.called
        assert loop_mock.call_soon_threadsafe.call_args == mock.call(
            vehicle_link._new_client_state.set)


@pytest.mark.asyncio
async def test_publish_settings(vehicle_link):
    topic: Topic = topic_factory.create_topic(
        VehicleLink.SETTINGS_PUBLISHING_TOPIC)
    send_event = asyncio.Event()
    loop = asyncio.get_event_loop()
    with mock.patch.object(topic, 'send',
                           side_effect=lambda _: loop.call_soon_threadsafe(
                             send_event.set)) as send_mock:
        # just to run the tests faster:
        VehicleLink.SETTINGS_PUBLISHING_PERIOD = 0.15
        wait_time = VehicleLink.SETTINGS_PUBLISHING_PERIOD * 1.2
        publishing_task = asyncio.ensure_future(
            vehicle_link._publish_settings())
        asyncio.get_event_loop().call_soon(vehicle_link._new_client_state.set)
        try:
            await asyncio.wait_for(send_event.wait(), wait_time)
        except asyncio.TimeoutError:
            pass
        assert not send_mock.called
        vehicle_link._VehicleLink__client_state = "settings_started"
        send_event.clear()
        asyncio.get_event_loop().call_soon(vehicle_link._new_client_state.set)
        try:
            await asyncio.wait_for(send_event.wait(), wait_time)
        except asyncio.TimeoutError:
            pass
        assert send_mock.called
        msg = send_mock.call_args[0][0]
        assert isinstance(msg, dict)
        assert 'yaw_fixed_power' in msg
        send_mock.reset_mock()
        assert not send_mock.called
        send_event.clear()
        try:
            await asyncio.wait_for(send_event.wait(), wait_time)
        except asyncio.TimeoutError:
            pass
        assert send_mock.called
        msg = send_mock.call_args[0][0]
        assert isinstance(msg, dict)
        assert 'yaw_fixed_power' in msg
        vehicle_link._VehicleLink__client_state = "live_camera_started"
        send_mock.reset_mock()
        assert not send_mock.called
        send_event.clear()
        try:
            await asyncio.wait_for(send_event.wait(), wait_time)
        except asyncio.TimeoutError:
            pass
        assert not send_mock.called
        publishing_task.cancel()
        try:
            await publishing_task
        except (asyncio.CancelledError, asyncio.TimeoutError):
            pass
