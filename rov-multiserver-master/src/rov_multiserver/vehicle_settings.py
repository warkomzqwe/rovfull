from rov_multiserver.settings_parameter import SettingsFloatParameter, \
    SettingsBoolParameter, SettingStringParameter, SettingsIntegerParameter
from rov_multiserver.settings_subject import SettingsSubject
# noinspection PyPackageRequirements
from singleton import Singleton  # The package's name is python-singleton


class _FixedYawPower(SettingsBoolParameter):
    PARAMETER_NAME = "fixed_yaw_power"
    DEFAULT_VALUE = False

    def __init__(self):
        SettingsBoolParameter.__init__(
            self, self.PARAMETER_NAME, self.DEFAULT_VALUE)


class _FixedYawPowerFactor(SettingsFloatParameter):
    PARAMETER_NAME = "fixed_yaw_power_factor"
    DEFAULT_VALUE = 0.5

    def __init__(self):
        SettingsFloatParameter.__init__(
            self, self.PARAMETER_NAME, self.DEFAULT_VALUE, 0.01, 1.0)


class _LightsOnOffControl(SettingsBoolParameter):
    PARAMETER_NAME = "lights_on_off_control"
    DEFAULT_VALUE = True

    def __init__(self):
        SettingsBoolParameter.__init__(
            self, self.PARAMETER_NAME, self.DEFAULT_VALUE)


class _LightsController(SettingStringParameter):
    """Expected light controller.

    Available options:
        - autopilot [default]: pixhawk controller via aux outputs
        - arduino:  ?? legacy code with no info
        - psd: ?? legacy code with no info
        - power_manager_board_v1: light control via insytech power manager board v1
        - arduino_acamas
    """
    PARAMETER_NAME = "lights_controller"
    DEFAULT_VALUE = "arduino_acamas"

    def __init__(self):
        SettingStringParameter.__init__(self, self.PARAMETER_NAME,
                                        self.DEFAULT_VALUE)


class _LightsStepSize(SettingsIntegerParameter):
    PARAMETER_NAME = "lights_step_size"
    DEFAULT_VALUE = 25

    def __init__(self):
        SettingsIntegerParameter.__init__(self, self.PARAMETER_NAME,
                                          self.DEFAULT_VALUE, 0, 100)


class _AutopilotSerialPort(SettingStringParameter):
    PARAMETER_NAME = "ap_serial_port"
    DEFAULT_VALUE = "@discover"
    """Possible special values:
    * @none: Don't connect by serial port.
    * @discover: Try to connect to various serial ports.
    """

    def __init__(self):
        SettingStringParameter.__init__(self, self.PARAMETER_NAME,
                                        self.DEFAULT_VALUE)


class _SpeedSmootherThreshold(SettingsFloatParameter):
    PARAMETER_NAME = '_threshold'
    DEFAULT_VALUE = 0.0

    def __init__(self):
        SettingsFloatParameter.__init__(self, self.PARAMETER_NAME,
                                        self.DEFAULT_VALUE, 0.0, 100.0)


class _SpeedSmootherAccelerationStartThreshold(_SpeedSmootherThreshold):
    PARAMETER_NAME = 'speed_smoother_acceleration_start_threshold'
    DEFAULT_VALUE = 50.0


class _SpeedSmootherAccelerationStopThreshold(_SpeedSmootherThreshold):
    PARAMETER_NAME = 'speed_smoother_acceleration_stop_threshold'
    DEFAULT_VALUE = 90.0


class _SpeedSmootherDecelerationStartThreshold(_SpeedSmootherThreshold):
    PARAMETER_NAME = 'speed_smoother_deceleration_start_threshold'
    DEFAULT_VALUE = 80.0


class _SpeedSmootherDecelerationStopThreshold(_SpeedSmootherThreshold):
    PARAMETER_NAME = 'speed_smoother_deceleration_stop_threshold'
    DEFAULT_VALUE = 30.0


class _SpeedSmootherRate(SettingsFloatParameter):
    PARAMETER_NAME = '_rate'
    DEFAULT_VALUE = 0.0

    def __init__(self):
        SettingsFloatParameter.__init__(self, self.PARAMETER_NAME,
                                        self.DEFAULT_VALUE, 0.0)


class _SpeedSmootherAccelerationRate(_SpeedSmootherRate):
    PARAMETER_NAME = 'speed_smoother_acceleration_rate'
    DEFAULT_VALUE = 25.0


class _SpeedSmootherDecelerationRate(_SpeedSmootherRate):
    PARAMETER_NAME = 'speed_smoother_deceleration_rate'
    DEFAULT_VALUE = 75.0


class _NetCleaningModeInitialGain(SettingsFloatParameter):
    PARAMETER_NAME = 'net_cleaning_mode_initial_gain'
    DEFAULT_VALUE = 0.7

    def __init__(self):
        SettingsFloatParameter.__init__(self, self.PARAMETER_NAME,
                                        self.DEFAULT_VALUE, 0.0, 1.0)


class _VehicleType(SettingStringParameter):
    PARAMETER_NAME = "vehicle_type"
    DEFAULT_VALUE = "rov"
    """Possible values:
    * rov (default): General purpose ROV.
    * net_cleaner: Net cleaner ROV.
    """

    def __init__(self):
        SettingStringParameter.__init__(self, self.PARAMETER_NAME,
                                        self.DEFAULT_VALUE)


class _CurrentLimit(SettingsIntegerParameter):
    PARAMETER_NAME = '_current_limit'
    DEFAULT_VALUE = 0  # No limit

    def __init__(self):
        SettingsIntegerParameter.__init__(self, self.PARAMETER_NAME,
                                          self.DEFAULT_VALUE, 0)


class _BatteryCurrentLimit(_CurrentLimit):
    PARAMETER_NAME = 'battery_current_limit'
    DEFAULT_VALUE = 28


class _ExternalPowerCurrentLimit(_CurrentLimit):
    PARAMETER_NAME = 'external_power_current_limit'
    DEFAULT_VALUE = 80


class VehicleSettingsSubject(SettingsSubject, metaclass=Singleton):
    SUBJECT_NAME = "vehicle"
    SUBJECT_SETTINGS_FILE = "~/.config/rov-multiserver/vehicle.yaml"

    def __init__(self, config_file_name: str = SUBJECT_SETTINGS_FILE):
        fixed_yaw_power = _FixedYawPower()
        fixed_yaw_power_factor = _FixedYawPowerFactor()
        lights_on_off_control = _LightsOnOffControl()
        ap_serial_port = _AutopilotSerialPort()
        acceleration_start_threshold = \
            _SpeedSmootherAccelerationStartThreshold()
        acceleration_stop_threshold = \
            _SpeedSmootherAccelerationStopThreshold()
        deceleration_start_threshold = \
            _SpeedSmootherDecelerationStartThreshold()
        deceleration_stop_threshold = \
            _SpeedSmootherDecelerationStopThreshold()
        acceleration_rate = _SpeedSmootherAccelerationRate()
        deceleration_rate = _SpeedSmootherDecelerationRate()
        lights_controller = _LightsController()
        lights_step_size = _LightsStepSize()
        net_cleaning_mode_initial_gain = _NetCleaningModeInitialGain()
        vehicle_type = _VehicleType()
        battery_current_limit = _BatteryCurrentLimit()
        external_power_current_limit = _ExternalPowerCurrentLimit()
        SettingsSubject.__init__(self, VehicleSettingsSubject.SUBJECT_NAME,
                                 config_file_name, fixed_yaw_power,
                                 fixed_yaw_power_factor,
                                 lights_on_off_control, lights_controller,
                                 lights_step_size,
                                 ap_serial_port,
                                 acceleration_rate,
                                 acceleration_start_threshold,
                                 acceleration_stop_threshold,
                                 deceleration_rate,
                                 deceleration_start_threshold,
                                 deceleration_stop_threshold,
                                 net_cleaning_mode_initial_gain,
                                 vehicle_type, battery_current_limit,
                                 external_power_current_limit)
        self.__fixed_yaw_power = fixed_yaw_power
        self.__fixed_yaw_power_factor = fixed_yaw_power_factor
        self.__lights_on_off_control = lights_on_off_control
        self.__lights_controller = lights_controller
        self.__lights_step_size = lights_step_size
        self.__ap_serial_port = ap_serial_port
        self.__acceleration_rate = acceleration_rate
        self.__acceleration_start_threshold = acceleration_start_threshold
        self.__acceleration_stop_threshold = acceleration_stop_threshold
        self.__deceleration_rate = deceleration_rate
        self.__deceleration_start_threshold = deceleration_start_threshold
        self.__deceleration_stop_threshold = deceleration_stop_threshold
        self.__net_cleaning_mode_initial_gain = net_cleaning_mode_initial_gain
        self.__vehicle_type = vehicle_type
        self.__battery_current_limit = battery_current_limit
        self.__external_power_current_limit = external_power_current_limit

    @property
    def fixed_yaw_power(self) -> bool:
        return self.__fixed_yaw_power.value

    @fixed_yaw_power.setter
    def fixed_yaw_power(self, value: bool):
        self.__fixed_yaw_power.value = value

    @property
    def fixed_yaw_power_factor(self) -> float:
        return self.__fixed_yaw_power_factor.value

    @fixed_yaw_power_factor.setter
    def fixed_yaw_power_factor(self, value: float):
        self.__fixed_yaw_power_factor.value = value

    @property
    def lights_on_off_control(self) -> bool:
        return self.__lights_on_off_control.value

    @lights_on_off_control.setter
    def lights_on_off_control(self, value: bool):
        self.__lights_on_off_control.value = value

    @property
    def lights_controller(self) -> str:
        return self.__lights_controller.value

    @lights_controller.setter
    def lights_controller(self, value: str):
        self.__lights_controller.value = value

    @property
    def lights_step_size(self) -> int:
        return self.__lights_step_size.value

    @lights_step_size.setter
    def lights_step_size(self, value: int):
        self.__lights_step_size.value = value

    @property
    def ap_serial_port(self) -> str:
        return self.__ap_serial_port.value

    @ap_serial_port.setter
    def ap_serial_port(self, value):
        self.__ap_serial_port.value = value

    @property
    def acceleration_rate(self) -> float:
        return self.__acceleration_rate.value

    @acceleration_rate.setter
    def acceleration_rate(self, value):
        self.__acceleration_rate.value = value

    @property
    def acceleration_start_threshold(self) -> float:
        return self.__acceleration_start_threshold.value

    @acceleration_start_threshold.setter
    def acceleration_start_threshold(self, value):
        self.__acceleration_start_threshold.value = value

    @property
    def acceleration_stop_threshold(self) -> float:
        return self.__acceleration_stop_threshold.value

    @acceleration_stop_threshold.setter
    def acceleration_stop_threshold(self, value):
        self.__acceleration_stop_threshold.value = value

    @property
    def deceleration_rate(self) -> float:
        return self.__deceleration_rate.value

    @deceleration_rate.setter
    def deceleration_rate(self, value):
        self.__deceleration_rate.value = value

    @property
    def deceleration_start_threshold(self) -> float:
        return self.__deceleration_start_threshold.value

    @deceleration_start_threshold.setter
    def deceleration_start_threshold(self, value):
        self.__deceleration_start_threshold.value = value

    @property
    def deceleration_stop_threshold(self) -> float:
        return self.__deceleration_stop_threshold.value

    @deceleration_stop_threshold.setter
    def deceleration_stop_threshold(self, value):
        self.__deceleration_stop_threshold.value = value

    @property
    def net_cleaning_mode_initial_gain(self) -> float:
        return self.__net_cleaning_mode_initial_gain.value

    @net_cleaning_mode_initial_gain.setter
    def net_cleaning_mode_initial_gain(self, value):
        self.__net_cleaning_mode_initial_gain.value = value

    @property
    def vehicle_type(self):
        return self.__vehicle_type.value

    @vehicle_type.setter
    def vehicle_type(self, value):
        self.__vehicle_type.value = value

    @property
    def battery_current_limit(self):
        return self.__battery_current_limit.value

    @battery_current_limit.setter
    def battery_current_limit(self, value):
        self.__battery_current_limit.value = value

    @property
    def external_power_current_limit(self):
        return self.__external_power_current_limit.value

    @external_power_current_limit.setter
    def external_power_current_limit(self, value):
        self.__external_power_current_limit.value = value
