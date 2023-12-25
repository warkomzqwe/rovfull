import pytest
import unittest.mock as mock

from pymessaginglib.Topic import Topic, topic_factory
from rov_multiserver.vehicle_link import VehicleLink
from rov_multiserver.vehicle_link_configuration_listener import TOPIC_NAME, \
    VehicleLinkConfigurationListener


@pytest.fixture
def mocked_topic():
    topic: Topic = topic_factory.create_topic(TOPIC_NAME)
    with mock.patch.multiple(topic, attach=mock.DEFAULT, detach=mock.DEFAULT):
        yield topic


@pytest.fixture
def mocked_vehicle_link():
    vehicle_mock = mock.MagicMock(spec=VehicleLink)
    vehicle_mock.fixed_yaw_power = None
    vehicle_mock.fixed_yaw_power_factor = None
    return vehicle_mock


@pytest.fixture
def config_listener(mocked_topic, mocked_vehicle_link):
    yield VehicleLinkConfigurationListener(mocked_vehicle_link)


def test_init(config_listener, mocked_topic, mocked_vehicle_link):
    assert isinstance(mocked_topic.attach, mock.Mock)
    assert mocked_topic.attach.called
    assert isinstance(mocked_topic.detach, mock.Mock)
    assert not mocked_topic.detach.called
    assert config_listener.link == mocked_vehicle_link


def test_detach(config_listener, mocked_topic):
    assert isinstance(mocked_topic.detach, mock.Mock)
    assert not mocked_topic.detach.called
    config_listener.detach()
    assert mocked_topic.detach.called


@pytest.mark.parametrize("msg, new_fixed_yaw, new_fixed_yaw_factor",
                         [(dict(yaw_fixed_power=0), False, None),
                          (dict(yaw_fixed_power=0.0), False, None),
                          (dict(yaw_fixed_power=0.5), True, 0.5),
                          (dict(yaw_fixed_power=0.75), True, 0.75),
                          (dict(yaw_fixed_power=1.0000005), None, None),
                          (dict(yaw_fixed_power=-0.0000005), None, None)])
@pytest.mark.parametrize("initial_fixed_yaw_factor", [0.0, 0.5, 1.0],
                         ids=["initial_fixed_yaw_factor:0.0",
                              "initial_fixed_yaw_factor:0.5",
                              "initial_fixed_yaw_factor:1.0"])
@pytest.mark.parametrize("initial_fixed_yaw", [True, False],
                         ids=["initial_fixed_yaw:True",
                              "initial_fixed_yaw:False"])
def test_on_message_arrival(config_listener, mocked_vehicle_link,
                            initial_fixed_yaw, initial_fixed_yaw_factor, msg,
                            new_fixed_yaw, new_fixed_yaw_factor):
    mocked_vehicle_link.fixed_yaw_power = initial_fixed_yaw
    mocked_vehicle_link.fixed_yaw_power_factor = initial_fixed_yaw_factor
    config_listener.on_message_arrival(msg, '0.0.0.0')
    if new_fixed_yaw is not None:
        assert mocked_vehicle_link.fixed_yaw_power == new_fixed_yaw
    else:
        assert mocked_vehicle_link.fixed_yaw_power == initial_fixed_yaw
    if new_fixed_yaw_factor is not None:
        assert mocked_vehicle_link.fixed_yaw_power_factor == \
               new_fixed_yaw_factor
    else:
        assert mocked_vehicle_link.fixed_yaw_power_factor == \
               initial_fixed_yaw_factor
