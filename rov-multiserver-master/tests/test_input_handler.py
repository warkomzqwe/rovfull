"""Input handler module tests."""
import pytest
import unittest.mock as mock
from rov_event_bus.bus import event_bus
from rov_multiserver.input_handler import InputHandler, _CommandGenerator


@pytest.fixture
def custom_event_map():
    """Custom "event map" created for testing."""
    return {
        'custom-event-for-button-without-mod': {
            'trigger_name': 'ABT', 'modifier_name': None, 'is_button': True},
        'custom-event-for-axis-without-mod': {
            'trigger_name': 'LXA', 'modifier_name': None, 'is_button': False},
        'custom-event-for-button-with-mod': {
            'trigger_name': 'YBT', 'modifier_name': 'BKB', 'is_button': True},
        'custom-event-for-axis-with-mod': {
            'trigger_name': 'RXA', 'modifier_name': 'BKB', 'is_button': False},
    }


@pytest.fixture
def cmd_gen_with_custom_evmap(custom_event_map):
    """A _CommandGenerator object using a custom event map."""
    _CommandGenerator.event_map = custom_event_map
    cmdgen = _CommandGenerator()
    return cmdgen


@pytest.fixture
def running_cmd_gen_with_custom_evmap(cmd_gen_with_custom_evmap):
    """A started _CommandGenerator object using a custom event map."""
    cmd_gen_with_custom_evmap.start()
    yield cmd_gen_with_custom_evmap
    cmd_gen_with_custom_evmap.stop()


@pytest.fixture
def callbacks_for_custom_evmap(custom_event_map):
    """Callback functions for events in custom event map."""
    callback_map = {}
    for event in custom_event_map:
        callback_map[event] = mock.Mock(name=event + '-mock')
        event_bus.register_callback(event, callback_map[event], is_sync=True)
    yield callback_map
    for event, callback in callback_map.items():
        event_bus.unregister_callback(event, callback)


def test_input_handler():
    """
    Tests for the InputHandler class.

    1.  Check if it creates a _CommandGenerator object and store it on a
        "_cmd_generator" attribute.
    2.  Check if calling start() calls _cmd_generator.start().
    3.  Check if calling stop() calls _cmd_generator.stop().
    """
    with mock.patch('rov_multiserver.input_handler._CommandGenerator',
                    spec=_CommandGenerator):
        handler = InputHandler()
        assert hasattr(handler, '_cmd_generator')
        assert isinstance(handler._cmd_generator, _CommandGenerator)
        assert not handler._cmd_generator.start.called
        handler.start()
        assert handler._cmd_generator.start.called
        assert not handler._cmd_generator.stop.called
        handler.stop()
        assert handler._cmd_generator.stop.called


def test_command_generator():
    """
    Tests for _CommandGenerator class.

    This test checks only the start() and stop() methods.
    Reference sets generation (__generate_reference_tests) is tested on
    test_reference_sets_generation().
    The main callback for 'input-event' is tested on
    test_on_input_event().
    """
    with mock.patch('rov_multiserver.input_handler.event_bus') as fake_bus:
        cmdgen = _CommandGenerator()
        assert not fake_bus.register_callback.called
        cmdgen.start()
        assert fake_bus.register_callback.called
        assert fake_bus.register_callback.call_args == mock.call(
            'input-event', cmdgen.on_input_event, 2, True)
        assert not fake_bus.unregister_callback.called
        cmdgen.stop()
        assert fake_bus.unregister_callback.called
        assert fake_bus.unregister_callback.call_args == mock.call(
            'input-event', cmdgen.on_input_event)


def test_reference_sets_generation(cmd_gen_with_custom_evmap,
                                   custom_event_map):
    """
    Tests for __generate_reference_tests method.

    Uses a custom event_map to test it in the following way:
    1.  Check if every trigger_name is on the trigger_set.
    2.  Check if every modifier is on modifier_map.
    3.  Check if every modifier element value on the map has a push(),
        release(), is_pushed() and is_released() methods.
    """
    expected_trigger_set = {'ABT', 'LXA', 'YBT', 'RXA', 'BKB'}
    cmdgen = cmd_gen_with_custom_evmap
    trigger_set = cmdgen._CommandGenerator__trigger_set
    modifier_map = cmdgen._CommandGenerator__modifier_map
    for event, desc in custom_event_map.items():
        assert desc['trigger_name'] in trigger_set
        if desc['modifier_name'] is not None:
            assert desc['modifier_name'] in modifier_map
    assert trigger_set == expected_trigger_set
    for element, value in modifier_map.items():
        assert hasattr(value, 'push')
        assert hasattr(value, 'release')
        assert hasattr(value, 'is_pushed')
        assert hasattr(value, 'is_released')
        assert value.is_pushed() is False
        assert value.is_released() is True
        value.push()
        assert value.is_pushed() is True
        assert value.is_released() is False
        value.release()
        assert value.is_pushed() is False
        assert value.is_released() is True


def test_on_input_event(running_cmd_gen_with_custom_evmap,
                        callbacks_for_custom_evmap):
    """
    Test the CommandGenerator 'input-event' callback.

    This method indeed is the main functionality of this class.

    """
    cmdgen = running_cmd_gen_with_custom_evmap
    a_button_is_released =\
        callbacks_for_custom_evmap.get('custom-event-for-button-without-mod')
    left_x_axis_has_changed =\
        callbacks_for_custom_evmap.get('custom-event-for-axis-without-mod')
    y_button_is_released_with_back_button_pressed =\
        callbacks_for_custom_evmap.get('custom-event-for-button-with-mod')
    right_x_axis_has_changed_with_back_button_pressed =\
        callbacks_for_custom_evmap.get('custom-event-for-axis-with-mod')
    all_callbacks = [a_button_is_released, left_x_axis_has_changed,
                     y_button_is_released_with_back_button_pressed,
                     right_x_axis_has_changed_with_back_button_pressed]

    assert cmdgen is not None
    for callback in all_callbacks:
        assert callable(callback)
    # First push A button
    event_bus.trigger('input-event', 'ABT;P')
    # Anything should happen
    for callback in all_callbacks:
        assert not callback.called
    # Now release it
    event_bus.trigger('input-event', 'ABT;R')
    assert a_button_is_released.called
    # Reset every mock
    list(map(lambda fake_callback: fake_callback.reset_mock(), all_callbacks))
    # Move left stick horizontally
    event_bus.trigger('input-event', 'LXA;-0.54')
    assert left_x_axis_has_changed.called
    assert left_x_axis_has_changed.call_args == mock.call(float('-0.54'))
    list(map(lambda fake_callback: fake_callback.reset_mock(), all_callbacks))
    # Return left stick to 0
    event_bus.trigger('input-event', 'LXA;0')
    assert left_x_axis_has_changed.called
    assert left_x_axis_has_changed.call_args == mock.call(float(0))
    list(map(lambda fake_callback: fake_callback.reset_mock(), all_callbacks))
    # Push and release YBT
    event_bus.trigger('input-event', 'YBT;P')
    event_bus.trigger('input-event', 'YBT;R')
    for callback in all_callbacks:
        assert not callback.called
    # Push BKB and again, push and release YBT
    event_bus.trigger('input-event', 'BKB;P')
    event_bus.trigger('input-event', 'YBT;P')
    for callback in all_callbacks:
        assert not callback.called
    event_bus.trigger('input-event', 'YBT;R')
    assert y_button_is_released_with_back_button_pressed.called
    event_bus.trigger('input-event', 'BKB;R')
    list(map(lambda fake_callback: fake_callback.reset_mock(), all_callbacks))
    # Move right stick horizontally
    event_bus.trigger('input-event', 'RXA;0.62')
    event_bus.trigger('input-event', 'RXA;0')
    for callback in all_callbacks:
        assert not callback.called
    # Push BKB and again, move right stick horizontally
    event_bus.trigger('input-event', 'BKB;P')
    for callback in all_callbacks:
        assert not callback.called
    event_bus.trigger('input-event', 'RXA;0.62')
    assert right_x_axis_has_changed_with_back_button_pressed.called
    assert right_x_axis_has_changed_with_back_button_pressed.\
        call_args == mock.call(float('0.62'))
    event_bus.trigger('input-event', 'RXA;0')
    event_bus.trigger('input-event', 'BKB;R')
