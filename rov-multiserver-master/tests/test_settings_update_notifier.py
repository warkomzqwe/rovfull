from random import choice

import pytest
import unittest.mock as mock
from rov_multiserver.settings_update_notifier import SettingsUpdateNotifier


@pytest.fixture
def update_notifier():
    return SettingsUpdateNotifier()


class _DummyClass:
    @classmethod
    def bad_signature_class_method_too_few_args(cls):
        pass

    @classmethod
    def bad_signature_class_method_too_many_args(cls, a, b, c):
        pass

    @classmethod
    def good_signature_class_method(cls, a, b):
        pass

    @staticmethod
    def bad_signature_static_method_too_few_args():
        pass

    @staticmethod
    def bad_signature_static_method_too_many_args(a, b, c):
        pass

    @staticmethod
    def good_signature_static_method(a, b):
        pass

    def bad_signature_method_too_few_args(self):
        pass

    def bad_signature_method_too_many_args(self, a, b, c):
        pass

    def good_signature_method(self, a, b):
        pass


def _bad_signature_function_too_few_args():
    pass


def _bad_signature_function_too_many_args(a, b, c):
    pass


def _good_signature_function(a, b):
    pass


def _good_signature_function_extra_args(a, b, c=None, d=None):
    pass


_dummy_obj = _DummyClass()


@pytest.mark.parametrize(
    "callback,should_success",
    [(_DummyClass.good_signature_class_method, True),
     (_DummyClass.bad_signature_class_method_too_few_args, False),
     (_DummyClass.bad_signature_class_method_too_many_args, False),
     (_DummyClass.good_signature_static_method, True),
     (_DummyClass.bad_signature_static_method_too_few_args, False),
     (_DummyClass.bad_signature_static_method_too_many_args, False),
     (_dummy_obj.good_signature_static_method, True),
     (_dummy_obj.bad_signature_static_method_too_few_args, False),
     (_dummy_obj.bad_signature_static_method_too_many_args, False),
     (_dummy_obj.good_signature_method, True),
     (_dummy_obj.bad_signature_method_too_few_args, False),
     (_dummy_obj.bad_signature_method_too_many_args, False),
     (_good_signature_function, True),
     (_good_signature_function_extra_args, True),
     (_bad_signature_function_too_few_args, False),
     (_bad_signature_function_too_many_args, False)])
def test_start_observing_with(update_notifier, callback, should_success):
    if should_success:
        prev_cb_ids = []
        cb_id = None
        for _ in range(50):
            try:
                cb_id = update_notifier.start_observing_with(callback)
            except TypeError:
                pytest.fail("An unexpected TypeError has been raised")
            assert isinstance(cb_id, int)
            assert cb_id not in prev_cb_ids
            prev_cb_ids.append(cb_id)
    else:
        with pytest.raises(TypeError):
            update_notifier.start_observing_with(callback)


def test_stop_observing(update_notifier):
    number_of_callbacks = 50
    for _ in range(number_of_callbacks):
        update_notifier.start_observing_with(mock.MagicMock())
    assert len(update_notifier.active_callbacks) == number_of_callbacks
    invalid_id = update_notifier.active_callbacks[-1] + 5000
    assert invalid_id not in update_notifier.active_callbacks
    with pytest.raises(ValueError):
        update_notifier.stop_observing(invalid_id)
    while len(update_notifier.active_callbacks) > 0:
        cb_to_remove = choice(update_notifier.active_callbacks)
        assert cb_to_remove in update_notifier.active_callbacks
        update_notifier.stop_observing(cb_to_remove)
        assert cb_to_remove not in update_notifier.active_callbacks


def test_notify_update(update_notifier):
    callbacks = [mock.MagicMock() for _ in range(20)]
    update_notifier.notify_update("foo", "bar")
    for callback in callbacks:
        assert not callback.called
    for callback in callbacks[:10]:
        update_notifier.start_observing_with(callback)
    update_notifier.notify_update("foo", "bar")
    for callback in callbacks[:10]:
        assert callback.called
        assert callback.call_args == mock.call("foo", "bar")
        callback.reset_mock()
    for callback in callbacks[10:]:
        assert not callback.called
        update_notifier.start_observing_with(callback)
    for callback in callbacks:
        assert not callback.called
    update_notifier.notify_update("foo", "bar")
    for callback in callbacks:
        assert callback.called
        assert callback.call_args == mock.call("foo", "bar")




