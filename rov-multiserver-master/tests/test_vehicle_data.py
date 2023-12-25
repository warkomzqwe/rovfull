# -*- coding: utf-8 -*-
"""Vehicle data module (vehicle_data.py) tests."""
import pytest
from unittest.mock import patch
from math import radians
from time import time, sleep
from rov_multiserver.vehicle_data import (
    VehicleTelemetryData, Measurement, CalculatedValue, Value,
    PitchMeasurement, RollMeasurement, YawMeasurement,
    PitchSpeedMeasurement, RollSpeedMeasurement, YawSpeedMeasurement,
    Heading, VoltageMeasurement, CurrentMeasurement,
    ConsumedElectricCharge, ElectricalPower, ConsumedElectricEnergy,
    PressureMeasurement, DepthCalculation,
    NamedValue, BoolValue, NumericValue, EnumValue, VehicleState)
from rov_event_bus.bus import *


def test_value_class():
    """Test of Value class usage."""
    with pytest.raises(TypeError):
        Value()

    class FaultyValue(Value):
        _attribute_name = 12
        _base_unit = 'foo'
        _valid_units = {}
        _units_offset = {}
    with pytest.raises(TypeError):
        FaultyValue()

    class FaultyValue(Value):
        _attribute_name = 'foo'
        _base_unit = 4
        _valid_units = {}
        _units_offset = {}
    with pytest.raises(TypeError):
        FaultyValue()

    class FaultyValue(Value):
        _attribute_name = 'foo'
        _base_unit = 'bar'
        _valid_units = 'baz'
        _units_offset = {}
    with pytest.raises(TypeError):
        FaultyValue()

    class FaultyValue(Value):
        _attribute_name = 'foo'
        _base_unit = 'bar'
        _valid_units = {}
        _units_offset = 'baz'
    with pytest.raises(TypeError):
        FaultyValue()

    class FaultyValue(Value):
        _attribute_name = 'foo'
        _base_unit = 'bar'
        _valid_units = {1: 14.0}
        _units_offset = {}
    with pytest.raises(TypeError):
        FaultyValue()

    class FaultyValue(Value):
        _attribute_name = 'foo'
        _base_unit = 'bar'
        _valid_units = {'baz': '14.0'}
        _units_offset = {}
    with pytest.raises(TypeError):
        FaultyValue()

    class FaultyValue(Value):
        _attribute_name = 'foo'
        _base_unit = 'bar'
        _valid_units = {}
        _units_offset = {1: 14.0}
    with pytest.raises(TypeError):
        FaultyValue()

    class FaultyValue(Value):
        _attribute_name = 'foo'
        _base_unit = 'bar'
        _units_offset = {'baz': '14.0'}
        _valid_units = {}
    with pytest.raises(TypeError):
        FaultyValue()

    class TestValue(Value):
        _attribute_name = 'testval'
        _base_unit = 'testunits'
        _valid_units = {'kilotestunits': 0.001, 'specialtestunits': 1.0}
        _units_offset = {'specialtestunits': -50.0}
    testval = TestValue()
    with pytest.raises(TypeError):
        testval.get(unit=1)
    with pytest.raises(ValueError):
        testval.get(unit='nonexistantunit')
    assert hasattr(testval, 'get_base')
    assert hasattr(testval, 'get_testunits')
    assert hasattr(testval, 'get_kilotestunits')
    assert hasattr(testval, 'get_specialtestunits')
    testval._value = 60  # 60 testunits specified as int
    assert testval.get_base() == 60.0
    assert testval.get_testunits() == testval.get_base()
    assert pytest.approx(testval.get_kilotestunits()) == 0.06
    assert pytest.approx(testval.get_specialtestunits()) == 10.0
    testval._value = 6475.6  # testunits specified as float
    assert testval.get_base() == 6475.6
    assert testval.get_testunits() == testval.get_base()
    assert pytest.approx(testval.get_kilotestunits()) == 6.4756
    assert pytest.approx(testval.get_specialtestunits()) == 6425.6


class _SampleMeasurement(Measurement):
        _attribute_name = 'test_measurement'
        _base_unit = 'testunits'
        _valid_units = {'kilotestunits': 0.001, 'specialtestunits': 1.0}
        _units_offset = {'specialtestunits': -50.0}


@patch('rov_multiserver.vehicle_data.event_bus')
def test_measurement_class(fake_bus):
    """Test of Measurement class usage."""
    with pytest.raises(TypeError):
            Measurement()
    fake_bus.trigger.reset_mock()
    time_before = time()
    measurement = _SampleMeasurement()
    time_after = time()
    assert fake_bus.trigger.called
    assert len(fake_bus.trigger.call_args[0]) == 2
    event, measurement_obj = fake_bus.trigger.call_args[0]
    assert event == 'new-measurement-value'
    assert isinstance(measurement_obj, Measurement)
    assert isinstance(measurement_obj, _SampleMeasurement)
    assert measurement_obj == measurement
    assert measurement.get_update_time() >= time_before
    assert measurement.get_update_time() <= time_after
    with pytest.raises(TypeError):
        measurement.set('250.53')
    fake_bus.trigger.reset_mock()
    time_before = time()
    measurement.set_specialtestunits(250.53)  # It should be 300.53 testunits
    time_after = time()
    assert fake_bus.trigger.called
    assert len(fake_bus.trigger.call_args[0]) == 2
    event, measurement_obj = fake_bus.trigger.call_args[0]
    assert event == 'new-measurement-value'
    assert pytest.approx(measurement.get_testunits()) == 300.53
    assert pytest.approx(measurement.get_kilotestunits()) == 0.30053
    assert measurement.get_update_time() >= time_before
    assert measurement.get_update_time() <= time_after


def test_calculated_value_class():
    """Test of CalculatedValue class usage."""
    with pytest.raises(TypeError):
        CalculatedValue()

    class FaultyCalculatedValue(CalculatedValue):
        _attribute_name = 'foo'
        _base_unit = 'bar'
        _valid_units = {}
        _units_offset = {}
        _required_measurements = set()

        def update_value(self):
            pass
    with pytest.raises(RuntimeError):
        FaultyCalculatedValue()

    class FaultyCalculatedValue(CalculatedValue):
        _attribute_name = 'foo'
        _base_unit = 'bar'
        _valid_units = {}
        _units_offset = {}
        _required_measurements = {object}

        def update_value(self):
            pass
    with pytest.raises(RuntimeError):
        FaultyCalculatedValue()

    class TestCalculatedValue(CalculatedValue):
        _attribute_name = 'test_calculated'
        _base_unit = 'tripletestunits'
        _valid_units = {'kilotripletestunits': 0.001}
        _units_offset = {}
        _required_measurements = {_SampleMeasurement}

        def update_value(self):
            self._value = self.test_measurement.get_testunits() * 3

    measurement = _SampleMeasurement()
    calc_value = TestCalculatedValue(measurement)
    original_update_time = calc_value.get_update_time()
    assert calc_value.get_tripletestunits() == 0.0
    assert calc_value.test_measurement == measurement
    measurement.set_testunits(600)
    assert calc_value.get_update_time() > original_update_time
    assert pytest.approx(calc_value.get_tripletestunits()) == 1800
    calc_value_event_called = False  # Flag specifying if callback was called
    calc_value_event_callback_args = tuple()  # Tuple for storing callback args

    def test_callback(*args, **kwargs):
        nonlocal calc_value_event_called
        nonlocal calc_value_event_callback_args
        calc_value_event_called = True
        calc_value_event_callback_args = args

    event_bus.register_callback('calculated-value-updated',
                                test_callback, POTENTIALLY_LATE, is_sync=True)
    # specified as sync to ensure it is called with triggering function.
    measurement.set_kilotestunits(2)
    assert calc_value_event_called
    assert len(calc_value_event_callback_args) == 1
    assert calc_value_event_callback_args[0] == calc_value
    assert calc_value.get_tripletestunits() == 6000
    assert calc_value.get_kilotripletestunits() == 6
    event_bus.unregister_callback('calculated-value-updated', test_callback)
    del calc_value


def test_electrical_power_calculations():
    """Test of ElectricalPower class."""
    voltage = VoltageMeasurement()
    current = CurrentMeasurement()
    power = ElectricalPower(voltage, current)
    assert voltage.get_base() == 0.0
    assert current.get_base() == 0.0
    assert power.get_base() == 0.0
    voltage.set_volts(14)
    voltage.get_millivolts() == voltage.get_volts() * 1000
    assert power.get_watts() == pytest.approx(0.0)
    current.set_amperes(1)
    current.get_centiamperes() == current.get_amperes() * 100
    current.get_milliamperes() == current.get_amperes() * 1000
    assert power.get_watts() == pytest.approx(14)
    assert power.get_milliwatts() == pytest.approx(14000)
    assert power.get_kilowatts() == pytest.approx(0.014)
    voltage.set_volts(14.4)
    voltage.get_millivolts() == voltage.get_volts() * 1000
    assert power.get_watts() == pytest.approx(14.4)
    assert power.get_milliwatts() == pytest.approx(14400)
    assert power.get_kilowatts() == pytest.approx(0.0144)
    current.set_amperes(100)
    current.get_centiamperes() == current.get_amperes() * 100
    current.get_milliamperes() == current.get_amperes() * 1000
    assert power.get_watts() == pytest.approx(1440)
    assert power.get_milliwatts() == pytest.approx(1440000)
    assert power.get_kilowatts() == pytest.approx(1.44)


def test_consumed_electric_charge_calculations():
    """Test of ElectricalPower class."""
    current = CurrentMeasurement()
    consumed_charge = ConsumedElectricCharge(current)
    current.set_amperes(0.0)
    with patch.object(consumed_charge,
                      '_calculate_increment') as counting_mock:
        counting_mock.reset_mock()
        counting_mock.return_value = 0.0
        for _ in range(10):
            consumed_charge.get()
        assert counting_mock.call_count == 0
    current.set_amperes(10)
    assert consumed_charge.get_amperehours() == 0.0
    sleep(0.1)
    assert consumed_charge.get_amperehours() > 0.0
    current.set_amperes(0)
    consumed_charge.reset()
    assert consumed_charge.get() == 0.0
    # Tests faking the time
    with patch('rov_multiserver.vehicle_data.time') as fake_time:
        last_time = time()
        fake_time.return_value = last_time
        assert current.get_base() == 0.0
        assert consumed_charge.get_base() == 0.0
        last_time += 3600  # Simulate that an hour has elapsed
        fake_time.return_value = last_time
        current.set_amperes(1)
        # previous current was zero, so there should be zero consumed charge
        assert consumed_charge.get_amperehours() == 0.0
        assert consumed_charge.get() == consumed_charge.get_amperehours()
        assert consumed_charge.get() == consumed_charge.get_base()
        assert consumed_charge.get_milliamperehours() == 0.0
        assert consumed_charge.get_coulomb() == 0.0
        last_time += 3600  # Simulate that an hour has elapsed
        fake_time.return_value = last_time
        assert consumed_charge.get_amperehours() == pytest.approx(1.0)
        assert consumed_charge.get() == consumed_charge.get_amperehours()
        assert consumed_charge.get() == consumed_charge.get_base()
        assert consumed_charge.get_milliamperehours() == pytest.approx(1000.0)
        assert consumed_charge.get_coulomb() == pytest.approx(3600.0)
        current.set_amperes(2)
        consumed_charge.reset()
        for _ in range(3600):
            last_time += 1
            fake_time.return_value = last_time
            consumed_charge.get()
        assert consumed_charge.get_amperehours() == pytest.approx(2.0)
        assert consumed_charge.get_milliamperehours() == pytest.approx(2000.0)
        assert consumed_charge.get_coulomb() == pytest.approx(7200.0)


def test_depth_calculations():
    """Test of DepthCalculation class."""
    pressure = PressureMeasurement()
    depth = DepthCalculation(pressure, is_saltwater=False)
    depth = DepthCalculation(pressure)
    assert pressure.get() == 0.0
    assert depth.get() == 0.0
    pressure.set(-1562.65)
    assert depth.get() == 0.0
    reference_results = [
        # (bar, m(saltwater), m(freshwater))
        (3.51325, 24.89, 25.48),
        (6.01325, 49.77, 50.97),
        (8.51325, 74.66, 76.45),
        (11.01325, 99.55, 101.94),
        (13.51325, 124.43, 127.42),
        (16.01325, 149.32, 152.91),
        (18.51325, 174.21, 178.39),
        (21.01325, 199.1, 203.88),
        (23.51325, 223.98, 229.36),
        (26.01325, 248.87, 254.85),
        (28.51325, 273.76, 280.33),
        (31.01325, 298.64, 305.82)
    ]
    # made using: http://docs.bluerobotics.com/calc/pressure-depth/
    # (NOTE: atmospheric pressure of (1.01325 bar is summed)
    max_rel_err = 1e-3  # maximum percentage relative error: 0.1%
    for press, sw_depth, fw_depth in reference_results:
        pressure.set_bar(press)
        assert pressure.get_pascal() == pressure.get_millibar() * 100.0
        assert pressure.get_mmhg() == pressure.get_millibar() * 0.750062
        assert pressure.get_atm()\
            == pressure.get_millibar() * 0.00098692316931427
        depth.in_saltwater()
        abs_error = abs(depth.get_meters() - sw_depth)
        rel_error = abs_error / sw_depth
        assert depth.get_meters() == pytest.approx(sw_depth, rel=max_rel_err),\
            "Depth is erroneous by: {:.2f} %".format(rel_error * 100.0)
        assert depth.get_centimeters()\
            == pytest.approx(depth.get_meters() * 100.0)
        assert depth.get_millimeters()\
            == pytest.approx(depth.get_meters() * 1000.0)
        depth.in_freshwater()
        abs_error = abs(depth.get_meters() - fw_depth)
        rel_error = abs_error / fw_depth
        assert depth.get_meters() == pytest.approx(fw_depth, rel=max_rel_err),\
            "Depth is erroneous by: {:.2f} %".format(rel_error * 100.0)
        assert depth.get_centimeters()\
            == pytest.approx(depth.get_meters() * 100.0)
        assert depth.get_millimeters()\
            == pytest.approx(depth.get_meters() * 1000.0)


def test_heading_calculations():
    """Test of Heading class."""
    yaw = YawMeasurement()
    heading = Heading(yaw)
    assert yaw.get() == 0.0
    assert heading.get() == 0.0
    reference_results = [
        # (yaw, heading, cardinality)
        (2, 2, ' N'),
        (47, 47, 'NE'),
        (92, 92, ' E'),
        (137, 137, 'SE'),
        (-178, 182, ' S'),
        (-133, 227, 'SW'),
        (-88, 272, ' W'),
        (-43, 317, 'NW'),
        (-1.33, 358.67, ' N')]
    for yaw_position, heading_position, cardinality in reference_results:
        yaw.set_degrees(yaw_position)
        assert heading.get_degrees() == pytest.approx(heading_position)
        assert heading.get_rads() == pytest.approx(radians(heading_position))
        assert heading.get_cardinality() == cardinality


def test_consumed_energy_calculations():
    """Test of ConsumedElectricEnergy class."""
    voltage = VoltageMeasurement()
    current = CurrentMeasurement()
    power = ElectricalPower(current, voltage)
    consumed_energy = ConsumedElectricEnergy(power)
    assert voltage.get() == 0.0
    assert current.get() == 0.0
    assert power.get() == 0.0
    assert consumed_energy.get() == 0.0
    current.set_amperes(0.0)
    with patch.object(consumed_energy,
                      '_calculate_increment') as counting_mock:
        counting_mock.reset_mock()
        counting_mock.return_value = 0.0
        for _ in range(30):
            consumed_energy.get()
        assert counting_mock.call_count == 0
    voltage.set_volts(14.4)
    current.set_amperes(10)
    assert consumed_energy.get_watthours() == 0.0
    sleep(0.1)
    assert consumed_energy.get_watthours() > 0.0
    voltage.set_volts(0)
    current.set_amperes(0)
    consumed_energy.reset()
    assert consumed_energy.get() == 0.0
    # Tests faking the time
    with patch('rov_multiserver.vehicle_data.time') as fake_time:
        last_time = time()
        fake_time.return_value = last_time
        assert voltage.get_base() == 0.0
        assert current.get_base() == 0.0
        assert consumed_energy.get_base() == 0.0
        last_time += 1800  # Simulate that half an hour has elapsed
        fake_time.return_value = last_time
        voltage.set_volts(16.2)
        last_time += 1800  # Simulate that half an hour has elapsed
        fake_time.return_value = last_time
        current.set_amperes(1)
        # previous power was zero, so there should be zero consumed energy
        assert power.get_watts() == pytest.approx(16.2)
        assert consumed_energy.get_watthours() == 0.0
        assert consumed_energy.get() == consumed_energy.get_watthours()
        assert consumed_energy.get() == consumed_energy.get_base()
        assert consumed_energy.get_milliwatthours() == 0.0
        assert consumed_energy.get_joules() == 0.0
        last_time += 3600  # Simulate that an hour has elapsed
        fake_time.return_value = last_time
        assert consumed_energy.get_watthours() == pytest.approx(16.2)
        assert consumed_energy.get() == consumed_energy.get_watthours()
        assert consumed_energy.get() == consumed_energy.get_base()
        assert consumed_energy.get_milliwatthours() == pytest.approx(16200.0)
        assert consumed_energy.get_joules() == pytest.approx(58320.0)
        current.set_amperes(2)
        voltage.set_volts(15)
        assert power.get_watts() == pytest.approx(30.0)
        consumed_energy.reset()
        for _ in range(3600):
            last_time += 1
            fake_time.return_value = last_time
            consumed_energy.get()
        assert consumed_energy.get_watthours() == pytest.approx(30.0)
        assert consumed_energy.get_milliwatthours() == pytest.approx(30000.0)
        assert consumed_energy.get_joules() == pytest.approx(108000.0)


def test_vehicle_telemetry_class():
    """Tests of VehicleTelemetry class."""
    measurement_obj_triggered_events = []
    calcvalue_obj_triggered_events = []
    telemetry_triggered_events = []
    measurement_objs = [
        PitchMeasurement, RollMeasurement, YawMeasurement,
        PitchSpeedMeasurement, RollSpeedMeasurement, YawSpeedMeasurement,
        VoltageMeasurement, CurrentMeasurement, PressureMeasurement]
    calcvalue_objs = [Heading, ElectricalPower, ConsumedElectricCharge,
                      ConsumedElectricEnergy, DepthCalculation]

    def on_value_subclass_event(obj):
        nonlocal measurement_obj_triggered_events
        nonlocal calcvalue_obj_triggered_events
        if isinstance(obj, CalculatedValue):
            calcvalue_obj_triggered_events.append(obj)
        elif isinstance(obj, Measurement):
            measurement_obj_triggered_events.append(obj)
        else:
            return

    def on_telemetry_event(obj):
        nonlocal telemetry_triggered_events
        telemetry_triggered_events.append(obj)
    event_bus.register_callback('new-measurement-value',
                                on_value_subclass_event,
                                priority=POTENTIALLY_LATE, is_sync=True)
    event_bus.register_callback('calculated-value-updated',
                                on_value_subclass_event,
                                priority=POTENTIALLY_LATE, is_sync=True)
    event_bus.register_callback('telemetry-value-changed',
                                on_telemetry_event,
                                priority=POTENTIALLY_LATE, is_sync=True)
    telemetry = VehicleTelemetryData()
    total_value_subclass_events = len(measurement_obj_triggered_events)
    total_value_subclass_events += len(calcvalue_obj_triggered_events)
    assert total_value_subclass_events >= len(telemetry._objects)
    for obj in measurement_objs:
        assert obj in map(type, measurement_obj_triggered_events),\
            "No class {obj} object has triggered an event".format(**locals)
    for obj in calcvalue_objs:
        assert obj in map(type, calcvalue_obj_triggered_events),\
            "No class {obj} object has triggered an event".format(**locals)
    sleep(0.025)
    # There should be as many "telemetry-value-changed" events as the sum of
    # "new-measurement-value" and "calculated-value-updated" events but
    # without the first "new-measurement-value" events that are called on
    # Measurement class objects creation (because at this time the
    # VehicleTelemetry object hasn't registered its own callbacks yet).
    assert len(telemetry_triggered_events)\
        == total_value_subclass_events - len(measurement_objs)
    repr(telemetry)  # Just to make coverage module happy


def test_named_value_class():
    """Tests of NamedValue class."""
    value = NamedValue('foo')
    assert value._name == 'foo'
    assert value._private_name == '_foo'
    value = NamedValue(name='foo')
    assert value._name == 'foo'
    assert value._private_name == '_foo'
    assert value._convert_to_corresponding_type(1) == 1
    assert value._convert_to_corresponding_type(1.5) == 1.5


def test_bool_value_class():
    """Tests of BoolValue class."""
    value = BoolValue('foo')
    assert value._name == 'foo'
    assert value._private_name == '_foo'
    assert value.DEFAULT is False
    value = BoolValue('bar', True)
    assert value._name == 'bar'
    assert value._private_name == '_bar'
    assert value.DEFAULT is True
    value = BoolValue(default_value=True, name='baz')
    assert value._name == 'baz'
    assert value._private_name == '_baz'
    assert value.DEFAULT is True
    with pytest.raises(TypeError):
        value._is_valid('True')
    with pytest.raises(TypeError):
        value._is_valid(56.7)
    with pytest.raises(TypeError):
        value._is_valid(88)
    assert value._is_valid(True)
    assert value._is_valid(False)


def test_numeric_value_class():
    """Tests of NumericValue class."""
    value = NumericValue('foo', max_value=35, min_value=-35)
    assert value._name == 'foo'
    assert value._private_name == '_foo'
    assert value.DEFAULT == 0.0
    assert not value._is_in_range(-36)
    assert not value._is_in_range(36)
    assert value._is_in_range(5)
    assert value._is_in_range(-35)
    assert value._is_in_range(35.0)
    with pytest.raises(TypeError):
        value._is_valid('45')
    with pytest.raises(ValueError):
        value._is_valid(45)
    value._is_valid(6)
    value._is_valid(6.5)
    value = NumericValue('bar', max_value=12, default_value=10.5,
                         min_value=-65)
    assert value._name == 'bar'
    assert value._private_name == '_bar'
    assert value.DEFAULT == 10.5
    assert not value._is_in_range(-66)
    assert not value._is_in_range(13)
    assert value._is_in_range(5)
    assert value._is_in_range(-65)
    assert value._is_in_range(12)
    value = NumericValue('baz', -55.23, -64.8)
    assert value._name == 'baz'
    assert value._private_name == '_baz'
    assert value.DEFAULT == -55.23
    assert not value._is_in_range(-66)
    assert value._is_in_range(-40)
    assert value._is_in_range(-64.8)
    assert value._is_in_range(-10.0)
    assert value._is_in_range(1240.062)
    assert isinstance(value._convert_to_corresponding_type(-44), float)


def test_enum_value_class():
    """Tests of EnumValue class."""
    from enum import IntEnum

    class TestEnum(IntEnum):
        FOO = 10
        BAR = 25
        BAZ = -5
    value = EnumValue('foobaz', default_value=TestEnum.BAR, ref_enum=TestEnum)
    value = EnumValue('bazfoo', TestEnum, -5)
    value = EnumValue('foobar', object, 0)
    assert value._name == 'foobar'
    assert value._private_name == '_foobar'
    with pytest.raises(RuntimeError):
        value._is_valid(3)
    value = EnumValue('barbaz', TestEnum, TestEnum.FOO)
    assert value._name == 'barbaz'
    assert value._private_name == '_barbaz'
    assert value.DEFAULT == TestEnum.FOO
    assert value._ref_enum == TestEnum
    with pytest.raises(TypeError):
        value._is_valid('3')
    with pytest.raises(ValueError):
        value._is_valid(3)
    assert value._is_valid(TestEnum.FOO)
    assert value._is_valid(TestEnum.BAR)
    assert value._is_valid(TestEnum.BAZ)
    assert value._is_valid(10)
    assert value._is_valid(25)
    assert value._is_valid(-5)


def test_vehicle_state_class():
    """Tests of VehicleState class."""
    state = VehicleState()
    properties = ['navigation_mode', 'armed', 'leak_detected',
                  'lights_level', 'cam_tilt_angle', 'input_gain']
    for prop_name in properties:
        assert hasattr(state, prop_name)
        assert hasattr(state, 'get_' + prop_name)
        assert callable(getattr(state, 'get_' + prop_name))
        assert hasattr(state, 'set_' + prop_name)
        assert callable(getattr(state, 'set_' + prop_name))
        assert hasattr(state, prop_name + '_last_update_time')
        assert callable(getattr(state, prop_name + '_last_update_time'))

    n_attributes = len(dir(state))
    state._VehicleState__convert_values_to_property_attributes([1, 'a', False])
    assert len(dir(state)) == n_attributes

    # Check default values
    assert state.navigation_mode == state.DEFAULT_MODE
    assert state.armed is False
    assert state.leak_detected is False
    assert state.lights_level == 0
    assert state.cam_tilt_angle == 0
    assert state.input_gain == 0.5

    # Prepare callback to capture "vehicle-state-value-updated" events.
    event_kwargs = None
    event_count = 0

    def _on_vehicle_state_value_updated(*args, **kwargs):
        nonlocal event_kwargs
        event_kwargs = kwargs
        nonlocal event_count
        event_count += 1

    event_bus.register_callback('vehicle-state-value-updated',
                                _on_vehicle_state_value_updated,
                                priority=IMMEDIATE, is_sync=True)
    # Test navigation_mode update
    from rov_multiserver.vehicle_data import VehicleNavigationMode
    last_updated_time = time()
    state.navigation_mode = VehicleNavigationMode.DEPTH_HOLD
    assert state.navigation_mode_last_update_time()\
        == pytest.approx(last_updated_time, abs=2e-2)
    assert state.navigation_mode == VehicleNavigationMode.DEPTH_HOLD
    assert event_kwargs is not None
    assert isinstance(event_kwargs, dict)
    assert len(event_kwargs) == 4
    assert event_kwargs.get('value_name') == 'navigation_mode'
    assert event_kwargs.get('type') == EnumValue
    assert event_kwargs.get('value') == VehicleNavigationMode.DEPTH_HOLD
    assert isinstance(event_kwargs.get('src'), EnumValue)
    # Test armed
    event_kwargs = None
    last_updated_time = time()
    assert not state.armed_last_update_time()\
        == pytest.approx(last_updated_time, abs=2e-2)
    state.armed = True
    assert state.armed_last_update_time()\
        == pytest.approx(last_updated_time, abs=2e-2)
    assert state.armed is True
    assert event_kwargs is not None
    assert isinstance(event_kwargs, dict)
    assert len(event_kwargs) == 4
    assert event_kwargs.get('value_name') == 'armed'
    assert event_kwargs.get('type') == BoolValue
    assert event_kwargs.get('value') is True
    assert isinstance(event_kwargs.get('src'), BoolValue)
    # Test leak_detected
    event_kwargs = None
    last_updated_time = time()
    assert not state.leak_detected_last_update_time()\
        == pytest.approx(last_updated_time, abs=2e-2)
    state.leak_detected = True
    assert state.leak_detected_last_update_time()\
        == pytest.approx(last_updated_time, abs=2e-2)
    assert state.leak_detected is True
    assert event_kwargs is not None
    assert isinstance(event_kwargs, dict)
    assert len(event_kwargs) == 4
    assert event_kwargs.get('value_name') == 'leak_detected'
    assert event_kwargs.get('type') == BoolValue
    assert event_kwargs.get('value') is True
    assert isinstance(event_kwargs.get('src'), BoolValue)
    # Test lights_level
    event_kwargs = None
    with pytest.raises(ValueError):
        state.lights_level = 100.5
    with pytest.raises(ValueError):
        state.lights_level = -0.005
    last_updated_time = time()
    assert not state.lights_level_last_update_time()\
        == pytest.approx(last_updated_time, abs=2e-2)
    state.lights_level = 50
    assert state.lights_level_last_update_time()\
        == pytest.approx(last_updated_time, abs=2e-2)
    assert state.lights_level == 50.0
    assert event_kwargs is not None
    assert isinstance(event_kwargs, dict)
    assert len(event_kwargs) == 4
    assert event_kwargs.get('value_name') == 'lights_level'
    assert event_kwargs.get('type') == NumericValue
    assert event_kwargs.get('value') == 50.0
    assert isinstance(event_kwargs.get('src'), NumericValue)
    assert event_kwargs.get('src')._max_value == 100.0
    assert event_kwargs.get('src')._min_value == 0.0
    # Test cam_tilt_angle
    event_kwargs = None
    with pytest.raises(ValueError):
        state.cam_tilt_angle = 45.05
    with pytest.raises(ValueError):
        state.cam_tilt_angle = -45.005
    last_updated_time = time()
    assert not state.cam_tilt_angle_last_update_time()\
        == pytest.approx(last_updated_time, abs=2e-2)
    state.cam_tilt_angle = 25.6
    assert state.cam_tilt_angle_last_update_time()\
        == pytest.approx(last_updated_time, abs=2e-2)
    assert state.cam_tilt_angle == 25.6
    assert event_kwargs is not None
    assert isinstance(event_kwargs, dict)
    assert len(event_kwargs) == 4
    assert event_kwargs.get('value_name') == 'cam_tilt_angle'
    assert event_kwargs.get('type') == NumericValue
    assert event_kwargs.get('value') == 25.6
    assert isinstance(event_kwargs.get('src'), NumericValue)
    assert event_kwargs.get('src')._max_value == 45.0
    assert event_kwargs.get('src')._min_value == -45.0
    # Test input_gain
    event_kwargs = None
    with pytest.raises(ValueError):
        state.input_gain = 1.05
    with pytest.raises(ValueError):
        state.input_gain = 0.05
    last_updated_time = time()
    assert not state.input_gain_last_update_time()\
        == pytest.approx(last_updated_time, abs=2e-2)
    state.input_gain = 0.6
    assert state.input_gain_last_update_time()\
        == pytest.approx(last_updated_time, abs=2e-2)
    assert state.input_gain == 0.6
    assert event_kwargs is not None
    assert isinstance(event_kwargs, dict)
    assert len(event_kwargs) == 4
    assert event_kwargs.get('value_name') == 'input_gain'
    assert event_kwargs.get('type') == NumericValue
    assert event_kwargs.get('value') == 0.6
    assert isinstance(event_kwargs.get('src'), NumericValue)
    assert event_kwargs.get('src')._max_value == 1.0
    assert event_kwargs.get('src')._min_value == 0.1

    event_count = 0
    last_updated_time = time()
    state.force_notify()
    for prop_name in properties:
        time_getter = getattr(state, prop_name + '_last_update_time')
        assert time_getter() == pytest.approx(last_updated_time, abs=2e-2)
    assert event_count == len(properties)

    repr(state)  # Just to make coverage module happy
