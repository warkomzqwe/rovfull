# -*- coding: utf-8 -*-
from abc import ABCMeta, abstractmethod
from math import degrees, radians
from enum import IntEnum
from functools import partial
from threading import Thread
from time import time, sleep
from inspect import isclass
from rov_logger.logger import get_rov_logger
from rov_event_bus.bus import *

logger = get_rov_logger()


# Value abstract base class
class Value(object, metaclass=ABCMeta):
    _valid_units = {}
    _units_offset = {}

    @property
    @classmethod
    @abstractmethod
    def _attribute_name(cls):
        return NotImplementedError

    @property
    @classmethod
    @abstractmethod
    def _base_unit(cls):
        return NotImplementedError

    def __init__(self):
        super(Value, self).__init__()
        # Check property types
        if not isinstance(type(self)._attribute_name, str):
            raise TypeError
        elif not isinstance(self._base_unit, str):
            raise TypeError
        elif not isinstance(self._valid_units, dict):
            raise TypeError
        elif not isinstance(self._units_offset, dict):
            raise TypeError
        for unit_name, unit_conversion in self._valid_units.items():
            if not isinstance(unit_name, str):
                raise TypeError
            if not isinstance(unit_conversion, (int, float)):
                raise TypeError
        for unit_name, unit_offset in self._units_offset.items():
            if not isinstance(unit_name, str):
                raise TypeError
            if not isinstance(unit_offset, (int, float)):
                raise TypeError
        self._valid_units.update({'base': 1.0, self._base_unit: 1.0})
        self._units_offset.update({'base': 0.0, self._base_unit: 0.0})
        for unit in self._valid_units:
            setattr(self, 'get_' + unit, partial(self.get, unit=unit))
        self._last_updated_time = time()
        self._value = 0.0

    def get(self, unit='base'):
        self._validate_unit(unit)
        value = self._value * self._valid_units[unit]
        value += self._units_offset.get(unit, 0.0)
        return value

    def _validate_unit(self, unit):
        if type(unit) is not str:
            raise TypeError(type(unit).__name__,
                            'is not a valid type for unit')
        elif unit not in self._valid_units:
            raise ValueError(unit,
                             'is not a valid unit for',
                             type(self).__name__)

    def get_update_time(self):
        return self._last_updated_time


# Measurements base class (observed)
class Measurement(Value):

    def __init__(self, value=0.0, unit='base'):
        super(Measurement, self).__init__()
        self.set(value, unit=unit)
        for unit in self._valid_units:
            setattr(self, 'set_' + unit, partial(self.set, unit=unit))

    def set(self, value, unit='base'):
        self._validate_unit(unit)
        if not isinstance(value, (float, int)):
            raise TypeError(value, 'must be numeric')
        self._value = float(value) / self._valid_units[unit]
        self._value -= self._units_offset.get(unit, 0.0)
        self._last_updated_time = time()
        event_bus.trigger('new-measurement-value', self)


# Calculated Values base class (observer)
class CalculatedValue(Value):
    __metaclass__ = ABCMeta
    _required_measurements = set()

    @property
    @classmethod
    @abstractmethod
    def _attribute_name(cls):
        return NotImplementedError

    @property
    @classmethod
    @abstractmethod
    def _base_unit(cls):
        return NotImplementedError

    def __init__(self, *args, **kwargs):
        super(CalculatedValue, self).__init__()
        self.__callbacks_registered = False
        self._measurements = set()
        if len(self._required_measurements) == 0:
            raise RuntimeError('At least 1 required measurement must be'
                               ' specified')
        for required_measurement in self._required_measurements:
            if not issubclass(required_measurement,
                              (Measurement, CalculatedValue)):
                raise RuntimeError(required_measurement,
                                   'specified as required measurement is not a'
                                   ' valid Measurement subclass type')
        for input_arg in args:
            # Check if it's an instance from required measurements and if it
            # isn't present in measurements object set yet.
            if not type(input_arg) in self._required_measurements \
                    or type(input_arg) in [
                type(measurement) for measurement in self._measurements]:
                continue
            else:
                self._measurements.add(input_arg)
        if len(self._measurements) < len(self._required_measurements):
            raise RuntimeError(len(self._measurements),
                               'measurement objects provided ({}'
                               ' required)'.format(len(
                                   self._required_measurements)))
        # Add notification callback for every measurement
        for measurement in self._measurements:
            setattr(self, measurement._attribute_name, measurement)
        event_bus.register_callback('new-measurement-value',
                                    self._on_new_measurement_value,
                                    is_sync=True)
        event_bus.register_callback('calculated-value-updated',
                                    self._on_new_measurement_value,
                                    is_sync=True)
        self.__callbacks_registered = True

    @abstractmethod
    def update_value(self):
        """
        Virtual method to update the value.

        It will be called from the 'new-measurement-value' callback when
        a measurement of interest has been updated.

        Returns
        -------
        bool, None
            True if update time was already updated, False or None
            otherwise.

        """
        pass

    def _on_new_measurement_value(self, measurement):
        if measurement in self._measurements:
            if not self.update_value():
                self._last_updated_time = time()
            event_bus.trigger('calculated-value-updated', self)


# Basic Measurements
class AngularPositionMeasurement(Measurement):
    _attribute_name = 'polar_coord'
    _base_unit = 'rads'
    _valid_units = {'degrees': degrees(1.0)}


class RollMeasurement(AngularPositionMeasurement):
    _attribute_name = 'roll'


class PitchMeasurement(AngularPositionMeasurement):
    _attribute_name = 'pitch'


class YawMeasurement(AngularPositionMeasurement):
    _attribute_name = 'yaw'


class AngularSpeedMeasurement(Measurement):
    _attribute_name = 'polar_coord'
    _base_unit = 'rad_s'
    _valid_units = {'degrees_s': degrees(1.0)}


class RollSpeedMeasurement(AngularSpeedMeasurement):
    _attribute_name = 'roll_speed'


class PitchSpeedMeasurement(AngularSpeedMeasurement):
    _attribute_name = 'pitch_speed'


class YawSpeedMeasurement(AngularSpeedMeasurement):
    _attribute_name = 'yaw_speed'


class VoltageMeasurement(Measurement):
    _attribute_name = 'voltage'
    _base_unit = 'millivolts'
    _valid_units = {'volts': 0.001}


class CurrentMeasurement(Measurement):
    _attribute_name = 'current'
    _base_unit = 'centiamperes'
    _valid_units = {'milliamperes': 10.0, 'amperes': 0.01}


class PressureMeasurement(Measurement):
    _attribute_name = 'pressure'
    _base_unit = 'millibar'
    _valid_units = {'bar': 0.001, 'atm': 0.00098692316931427,
                    'mmhg': 0.750062, 'pascal': 100.0}


class TemperatureMeasurement(Measurement):
    _attribute_name = 'temperature'
    _base_unit = 'celsius_degree'
    _valid_units = {'kelvin': 1.0, 'farenheit': 9.0 / 5.0}
    _units_offset = {'kelvin': -273.0, 'farenheit': 32.0}


class PowerHousingTemperatureMeasurement(TemperatureMeasurement):
    """
    Temperature Measurement on the ROV's power housing.

    HACK: This class should be just TemperatureMeasurement with some
    identificator, but there's no "name" nor "index" attribute on
    "Value" class, so this subclass have been created.
    TODO: Update Value class to handle names or attributes.
    """

    _attribute_name = 'power_housing_temperature'


class PowerRegulatorTemperatureMeasurement(TemperatureMeasurement):
    """
    Temperature Measurement on the ROV's power regulator.

    HACK: This class should be just TemperatureMeasurement with some
    identificator, but there's no "name" nor "index" attribute on
    "Value" class, so this subclass have been created.
    TODO: Update Value class to handle names or attributes.
    """

    _attribute_name = 'power_regulator_temperature'


class AltitudeMeasurement(Measurement):
    _attribute_name = 'altitude'
    _base_unit = 'meters'
    _valid_units = {'millimeters': 1000.0, 'centimeters': 100.0}


# Derived Calculations
class ElectricalPower(CalculatedValue):
    _attribute_name = 'power'
    _base_unit = 'watts'
    _valid_units = {'milliwatts': 1000.0, 'kilowatts': 0.001}
    _required_measurements = {VoltageMeasurement, CurrentMeasurement}

    def __init__(self, *args, **kwargs):
        super(ElectricalPower, self).__init__(*args, **kwargs)
        self.update_value()

    def update_value(self):
        self._value = self.voltage.get_volts() * self.current.get_amperes()


class ConsumedElectricCharge(CalculatedValue):
    _attribute_name = 'consumed_charge'
    _base_unit = 'amperehours'
    _valid_units = {'milliamperehours': 1000.0, 'coulomb': 3600.0}
    _required_measurements = {CurrentMeasurement}

    def __init__(self, *args, **kwargs):
        super(ConsumedElectricCharge, self).__init__(*args, **kwargs)
        self._value = 0.0
        self._last_updated_time = 0.0

    def update_value(self):
        if self.get_update_time() == 0.0:  # First update
            self._last_current = self.current.get_amperes()
            return False
        else:
            update_time = time()
            elapsed_time_hours = \
                (update_time - self._last_updated_time) / 3600.0
            self._value += self._calculate_increment(elapsed_time_hours)
            self._last_updated_time = update_time
            return True

    def get(self, unit='base'):  # Override to update value on get
        new_time = time()
        elapsed_time = new_time - self.get_update_time()
        if elapsed_time > 0.050:
            if not self.update_value():
                self._last_updated_time = new_time
        return super().get(unit)

    def _calculate_increment(self, elapsed_time_hours):
        increment = self._last_current * elapsed_time_hours
        self._last_current = self.current.get_amperes()
        return increment

    def reset(self):
        self._value = 0


class ConsumedElectricEnergy(CalculatedValue):
    _attribute_name = 'consumed_energy'
    _base_unit = 'watthours'
    _valid_units = {'milliwatthours': 1000.0, 'joules': 3600.0}
    _required_measurements = {ElectricalPower}

    def __init__(self, *args, **kwargs):
        super(ConsumedElectricEnergy, self).__init__(*args, **kwargs)
        self._value = 0.0
        self._last_updated_time = 0.0

    def update_value(self):
        if self.get_update_time() == 0.0:  # First update
            self._last_power = self.power.get_watts()
            return False
        else:
            update_time = time()
            elapsed_time_hours = \
                (update_time - self._last_updated_time) / 3600.0
            self._value += self._calculate_increment(elapsed_time_hours)
            self._last_updated_time = update_time
            return True

    def get(self, unit='base'):  # Override to update value on get
        new_time = time()
        elapsed_time = new_time - self.get_update_time()
        if elapsed_time > 0.010:
            if not self.update_value():
                self._last_updated_time = new_time
        return super().get(unit)

    def _calculate_increment(self, elapsed_time_hours):
        increment = self._last_power * elapsed_time_hours
        self._last_power = self.power.get_watts()
        return increment

    def reset(self):
        self._value = 0


class DepthCalculation(CalculatedValue):
    """
    Calculates the depth based on a pressure measurement.

    press[Pa] = density[kg/m^3] * gravity_accel[m/s^2] * depth[m]
    => depth[m] = press[Pa] / (density[kg/m^3] * gravity_accel[m/s^2])
    where:
        gravity_accel = 9.80665 [m/s^2]
    and:
        press[Pa] = 100000 * press[bar]
    then:
        depth[m] = (100000 / 9.80665) * (press[bar] / density[kg/m^3])
        depth[m] = 10197.162129779 * press[bar] / density[kg/m^3]

    """

    _attribute_name = 'depth'
    _base_unit = 'meters'
    _valid_units = {'millimeters': 1000.0, 'centimeters': 100.0}
    _required_measurements = {PressureMeasurement}
    DEFAULT_IS_SALTWATER_FLAG = True
    SALTWATER_DENSITY_KG_M3 = 1024.0
    FRESHWATER_DENSITY_KG_M3 = 1000.0
    DEFAULT_ATMOSPHERIC_PRESSURE_BAR = 1.01325
    PRESS_BAR_TO_DEPTH_M_SW = 10197.162129779 / SALTWATER_DENSITY_KG_M3
    PRESS_BAR_TO_DEPTH_M_FW = 10197.162129779 / FRESHWATER_DENSITY_KG_M3

    def __init__(self, *args, **kwargs):
        super(DepthCalculation, self).__init__(*args, **kwargs)
        if 'atmosferic_pressure' in kwargs:
            self.__atmospheric_pressure = float(kwargs['atmosferic_pressure'])
        else:
            self.__atmospheric_pressure \
                = DepthCalculation.DEFAULT_ATMOSPHERIC_PRESSURE_BAR
        if 'is_saltwater' in kwargs:
            self.__is_saltwater = bool(kwargs['is_saltwater'])
        else:
            self.__is_saltwater = DepthCalculation.DEFAULT_IS_SALTWATER_FLAG
        self.update_value()

    def update_value(self):
        if self.__is_saltwater:
            pressure_to_meters_factor = self.PRESS_BAR_TO_DEPTH_M_SW
        else:
            pressure_to_meters_factor = self.PRESS_BAR_TO_DEPTH_M_FW
        self._value = self.pressure.get_bar() - self.__atmospheric_pressure
        self._value *= pressure_to_meters_factor
        if self._value < 0.0:
            self._value = 0.0  # negative depth makes no sense

    def in_freshwater(self):
        self.__is_saltwater = False
        self.update_value()
        self._last_updated_time = time()

    def in_saltwater(self):
        self.__is_saltwater = True
        self.update_value()
        self._last_updated_time = time()


class Heading(CalculatedValue):
    _attribute_name = 'heading'
    _base_unit = 'degrees'
    _valid_units = {'rads': radians(1.0)}
    _required_measurements = {YawMeasurement}

    def __init__(self, *args, **kwargs):
        super(Heading, self).__init__(*args, **kwargs)
        self.update_value()

    def update_value(self):
        value = self.yaw.get_degrees() + 360.0
        if value >= 360.0:
            self._value = value - 360.0
        else:
            self._value = value

    def __get_cardinality(self):
        if self._value >= 337.5 or self._value < 22.5:
            return ' N'
        elif self._value >= 22.5 and self._value < 67.5:
            return 'NE'
        elif self._value >= 67.5 and self._value < 112.5:
            return ' E'
        elif self._value >= 112.5 and self._value < 157.5:
            return 'SE'
        elif self._value >= 157.5 and self._value < 202.5:
            return ' S'
        elif self._value >= 202.5 and self._value < 247.5:
            return 'SW'
        elif self._value >= 247.5 and self._value < 292.5:
            return ' W'
        elif self._value >= 292.5 and self._value < 337.5:
            return 'NW'

    def get_cardinality(self):
        return self.__get_cardinality()

    def __str__(self):
        degree = u'\u00b0'
        return '{0:.2f}' + degree + '{1}'.format(
            self._value,
            self.__get_cardinality())


class VehicleTelemetryData(object):
    def __init__(self):
        self.__callbacks_registered = False
        super(VehicleTelemetryData, self).__init__()
        # Start with basic measurements
        self._objects = [
            RollMeasurement(), PitchMeasurement(), YawMeasurement(),
            RollSpeedMeasurement(), PitchSpeedMeasurement(),
            YawSpeedMeasurement(), VoltageMeasurement(), CurrentMeasurement(),
            # PressureMeasurement(), PowerHousingTemperatureMeasurement(),
            # PowerRegulatorTemperatureMeasurement(), AltitudeMeasurement()
            PressureMeasurement(), AltitudeMeasurement()
        ]
        # Put the newly created objects as attributes of self
        self._objects_to_attributes()

        # Create derived calculation objects
        self._objects.extend([
            ElectricalPower(self.voltage, self.current),
            ConsumedElectricCharge(self.current),
            DepthCalculation(self.pressure),
            Heading(self.yaw)
        ])
        # Put the newly created objects as attributes of self
        self._objects_to_attributes()

        # Create calculation objects based on another calculations objects
        self._objects.extend([ConsumedElectricEnergy(self.power)])
        # Put the newly created objects as attributes of self
        self._objects_to_attributes()
        event_bus.register_callback('new-measurement-value',
                                    self._on_telemetry_value_changed,
                                    priority=IMMEDIATE, is_sync=False)
        event_bus.register_callback('calculated-value-updated',
                                    self._on_telemetry_value_changed,
                                    priority=IMMEDIATE, is_sync=False)
        event_bus.register_callback('new-mqttudp-measurement',
                                    self._on_new_mqttudp_measurement,
                                    priority=IMMEDIATE, is_sync=False)
        self.__callbacks_registered = True
        self.force_notify()

    def _objects_to_attributes(self):
        for object_ in self._objects:
            attribute_name = object_._attribute_name
            if not hasattr(self, attribute_name):
                setattr(self, attribute_name, object_)

    def _on_telemetry_value_changed(self, value_object):
        if value_object in self._objects:
            event_bus.trigger('telemetry-value-changed', value_object)

    def __repr__(self):
        string = '<{0}.{1} object at {2} --- Values: '.format(
            type(self).__module__,
            type(self).__name__,
            hex(id(self))
        )
        for index, object_ in enumerate(self._objects):
            if index > 0:
                string += ', '
            string += '{0}={1:.2f} [{2}]'.format(
                object_._attribute_name,
                object_.get_base(),
                object_._base_unit
            )
        return string + '>'

    def force_notify(self):
        for object_ in self._objects:
            if isinstance(object_, Measurement):
                event_bus.trigger('new-measurement-value', object_)

    def _on_new_mqttudp_measurement(self, sensor_type, idx, unit, val, extras):
        if sensor_type == 'temperature':
            if idx == 0:
                self.power_regulator_temperature.set(val)
            elif idx == 1:
                self.power_housing_temperature.set(val)
        elif sensor_type == 'altitude':
            self.altitude.set(val)


class VehicleNavigationMode(IntEnum):
    MANUAL = 19
    STABILIZE = 0
    DEPTH_HOLD = 2


class NamedValue(object):
    def __init__(self, *args, **kwargs):
        name = None
        for index, arg in enumerate(args):
            if index == 0:
                name = arg
        if 'name' in kwargs:
            name = kwargs.pop('name')
        super(NamedValue, self).__init__()
        self._name = name
        self._private_name = '_' + name

    def _convert_to_corresponding_type(self, value):
        return value  # Subclasses could override this if it's necessary


class BoolValue(NamedValue):
    def __init__(self, *args, **kwargs):
        default_value = False
        for index, arg in enumerate(args):
            if index == 1:
                default_value = arg
        if 'default_value' in kwargs:
            default_value = kwargs.pop('default_value')
        super(BoolValue, self).__init__(*args, **kwargs)
        self.DEFAULT = default_value

    def _is_valid(self, value):
        if not isinstance(value, bool):
            raise TypeError(
                '"{0}" should be of type bool (passing: {1})'.format(
                    self._name,
                    type(value)))
        else:
            return True


class NumericValue(NamedValue):
    def __init__(self, *args, **kwargs):
        min_value = None
        max_value = None
        default_value = 0
        for index, arg in enumerate(args):
            if index == 1:
                default_value = arg
            if index == 2:
                min_value = arg
            if index == 3:
                max_value = arg
        if 'min_value' in kwargs:
            min_value = kwargs.pop('min_value')
        if 'max_value' in kwargs:
            max_value = kwargs.pop('max_value')
        if 'default_value' in kwargs:
            default_value = kwargs.pop('default_value')
        super(NumericValue, self).__init__(*args, **kwargs)
        self._min_value = min_value
        self._max_value = max_value
        self.DEFAULT = default_value

    def _is_valid(self, value):
        if not isinstance(value, (float, int)):
            raise TypeError(
                '"{0}" value should be numeric (passing: {1})'.format(
                    self._name,
                    type(value)))
        elif not self._is_in_range(value):
            raise ValueError(
                '"{0}" should be in range {1} <= x <= {2} (passing: '
                '{3})'.format(
                    self._name,
                    self._min_value,
                    self._max_value,
                    value))
        else:
            return True

    def _convert_to_corresponding_type(self, value):
        return float(value)

    def _is_in_range(self, value):
        if self._max_value is None:
            return value >= self._min_value
        else:
            return value >= self._min_value and value <= self._max_value


class EnumValue(NamedValue):
    def __init__(self, *args, **kwargs):
        ref_enum = None
        default_value = None
        for index, arg in enumerate(args):
            if index == 1:
                ref_enum = arg
            if index == 2:
                default_value = arg
        if 'ref_enum' in kwargs:
            ref_enum = kwargs.pop('ref_enum')
        if 'default_value' in kwargs:
            default_value = kwargs.pop('default_value')
        super(EnumValue, self).__init__(*args, **kwargs)
        self._ref_enum = ref_enum
        self.DEFAULT = default_value

    def _is_valid(self, value):
        if not issubclass(self._ref_enum, IntEnum):
            raise RuntimeError('reference enum: {0} is not an IntEnum '
                               'subclass'.format(
                self._ref_enum.__name__
                if isclass(self._ref_enum)
                else type(self._ref_enum).__name__))
        elif not isinstance(value, self._ref_enum) \
                and not isinstance(value, int):
            raise TypeError(
                '"{0}" should be of type {1} or int (passing: {2})'.format(
                    self._name,
                    self._ref_enum.__name__,
                    type(value).__name__))
        else:
            # The following will raise ValueError if not valid.
            return isinstance(self._convert_to_corresponding_type(value),
                              self._ref_enum)

    def _convert_to_corresponding_type(self, value):
        return self._ref_enum(value)


class VehicleState:
    DEFAULT_MODE = VehicleNavigationMode.MANUAL

    def __init__(self):
        super(VehicleState, self).__init__()
        self.__last_updated_time = {}
        values = [
            EnumValue('navigation_mode', VehicleNavigationMode,
                      self.DEFAULT_MODE),
            BoolValue('armed'),
            BoolValue('leak_detected'),
            NumericValue('lights_level', 0.0, 0.0, 100.0),
            NumericValue('cam_tilt_angle', 0.0, -180.0, 180.0),
            NumericValue('input_gain', 0.5, 0.1, 1.0)
        ]
        self.__convert_values_to_property_attributes(values)
        self.force_notify()
        event_bus.register_callback('client-connected',
                                    self._on_video_client_connected,
                                    priority=POTENTIALLY_LATE)

    def __convert_values_to_property_attributes(self, value_list):
        for value in value_list:
            if not isinstance(value, NamedValue):
                logger.debug("{} is not a valid value".format(value))
                continue
            setter_name = 'set_' + value._name
            setter_method = partial(self.__base_setter, value)
            setattr(self, setter_name, setter_method)
            getter_name = 'get_' + value._name
            getter_method = partial(self.__base_getter, value)
            setattr(self, getter_name, getter_method)
            new_property = property(fget=getattr(self, getter_name),
                                    fset=getattr(self, setter_name))
            setattr(self.__class__, value._name, new_property)
            setattr(self,
                    value._name + '_last_update_time',
                    partial(self._get_last_updated_time, value._name))

    def _get_last_updated_time(self, name):
        return self.__last_updated_time.get(name, 0.0)

    @staticmethod
    def __base_setter(value_object, self, value):
        if value_object._is_valid(value):
            value = value_object._convert_to_corresponding_type(value)
            setattr(self, value_object._private_name, value)
            self.__last_updated_time[value_object._name] = time()
            event_bus.trigger('vehicle-state-value-updated',
                              value_name=value_object._name,
                              type_=(type(value_object)),
                              value=value,
                              src=value_object)

    @staticmethod
    def __base_getter(value_object, self):
        return getattr(self, value_object._private_name,
                       value_object.DEFAULT)

    def __repr__(self):
        string = '<{0}.{1} object at {2} --- Values: '.format(
            type(self).__module__,
            type(self).__name__,
            hex(id(self))
        )
        first_attribute = True
        for attribute in dir(type(self)):
            if not isinstance(getattr(type(self), attribute), property):
                continue
            else:
                if not first_attribute:
                    string += ', '
                string += '{0}: {1}'.format(
                    attribute,
                    str(getattr(self, attribute))
                )
                first_attribute = False
        return string + '>'

    def _on_video_client_connected(self, *args, **kwargs):
        def delayed_call(func):
            sleep(0.5)
            func()

        Thread(target=delayed_call, args=(self.force_notify,),
               name="DelayedForceNotifyThread").start()

    def force_notify(self):
        for attribute in dir(type(self)):
            if isinstance(getattr(type(self), attribute), property) and \
                    attribute != 'observed_value':
                # print attribute
                # print getattr(self, attribute)
                setter_name = 'set_' + attribute
                setter = getattr(self, setter_name)
                setter(self, getattr(self, attribute))


class BoardRotation:
    """Container for available pixhawk rotation configurations"""
    def __init__(self, name, roll, pitch, yaw):
        self.name = name
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw

    def is_90_degrees(self):
        return (self.roll % 90 == 0) and (self.pitch % 90 == 0) and (self.yaw % 90 == 0)

    def __str__(self):
        return self.name


board_rotations = [
    BoardRotation("ROTATION_NONE", 0, 0, 0),
    BoardRotation("ROTATION_YAW_45", 0, 0, 45),
    BoardRotation("ROTATION_YAW_90", 0, 0, 90),
    BoardRotation("ROTATION_YAW_135", 0, 0, 135),
    BoardRotation("ROTATION_YAW_180", 0, 0, 180),
    BoardRotation("ROTATION_YAW_225", 0, 0, 225),
    BoardRotation("ROTATION_YAW_270", 0, 0, 270),
    BoardRotation("ROTATION_YAW_315", 0, 0, 315),
    BoardRotation("ROTATION_ROLL_180", 180, 0, 0),
    BoardRotation("ROTATION_ROLL_180_YAW_45", 180, 0, 45),
    BoardRotation("ROTATION_ROLL_180_YAW_90", 180, 0, 90),
    BoardRotation("ROTATION_ROLL_180_YAW_135", 180, 0, 135),
    BoardRotation("ROTATION_PITCH_180", 0, 180, 0),
    BoardRotation("ROTATION_ROLL_180_YAW_225", 180, 0, 225),
    BoardRotation("ROTATION_ROLL_180_YAW_270", 180, 0, 270),
    BoardRotation("ROTATION_ROLL_180_YAW_315", 180, 0, 315),
    BoardRotation("ROTATION_ROLL_90", 90, 0, 0),
    BoardRotation("ROTATION_ROLL_90_YAW_45", 90, 0, 45),
    BoardRotation("ROTATION_ROLL_90_YAW_90", 90, 0, 90),
    BoardRotation("ROTATION_ROLL_90_YAW_135", 90, 0, 135),
    BoardRotation("ROTATION_ROLL_270", 270, 0, 0),
    BoardRotation("ROTATION_ROLL_270_YAW_45", 270, 0, 45),
    BoardRotation("ROTATION_ROLL_270_YAW_90", 270, 0, 90),
    BoardRotation("ROTATION_ROLL_270_YAW_135", 270, 0, 135),
    BoardRotation("ROTATION_PITCH_90", 0, 90, 0),
    BoardRotation("ROTATION_PITCH_270", 0, 270, 0),
    BoardRotation("ROTATION_PITCH_180_YAW_90", 0, 180, 90),
    BoardRotation("ROTATION_PITCH_180_YAW_270", 0, 180, 270),
    BoardRotation("ROTATION_ROLL_90_PITCH_90", 90, 90, 0),
    BoardRotation("ROTATION_ROLL_180_PITCH_90", 180, 90, 0),
    BoardRotation("ROTATION_ROLL_270_PITCH_90", 270, 90, 0),
    BoardRotation("ROTATION_ROLL_90_PITCH_180", 90, 180, 0),
    BoardRotation("ROTATION_ROLL_270_PITCH_180", 270, 180, 0),
    BoardRotation("ROTATION_ROLL_90_PITCH_270", 90, 270, 0),
    BoardRotation("ROTATION_ROLL_180_PITCH_270", 180, 270, 0),
    BoardRotation("ROTATION_ROLL_270_PITCH_270", 270, 270, 0),
    BoardRotation("ROTATION_ROLL_90_PITCH_180_YAW_90", 90, 180, 90),
    BoardRotation("ROTATION_ROLL_90_YAW_270", 90, 0, 270)
]
"""Available ardusub firmware rotations for pixhawk board orientation"""
