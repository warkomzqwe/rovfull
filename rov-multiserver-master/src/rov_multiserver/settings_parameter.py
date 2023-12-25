from abc import ABCMeta, abstractmethod
from typing import Any, Union, Optional

from rov_multiserver.settings_update_notifier import SettingsUpdateNotifier


class SettingsParameter(SettingsUpdateNotifier, metaclass=ABCMeta):
    """Base class for all parameters."""

    def __init__(self, name: str):
        SettingsUpdateNotifier.__init__(self)
        self.__name: str = str(name)

    @property
    def name(self) -> str:
        return self.__name

    @property
    def value(self) -> Any:
        return self.do_get_value()

    @value.setter
    def value(self, value: Any):
        self.set_value(value)

    def set_value(self, value: Any):
        self.do_set_value(value)
        self.notify_update(self.name, self.value)

    @abstractmethod
    def do_get_value(self) -> Any:
        ...

    @abstractmethod
    def do_set_value(self, value: Any):
        ...


class SettingsNumericParameter(SettingsParameter):
    """Numeric parameter.

    Its value could be an int or float.
    It could have a maximum and a minimum."""

    def __init__(self, name: str, initial_value: Union[int, float],
                 min_allowable_value: Optional[Union[int, float]] = None,
                 max_allowable_value: Optional[Union[int, float]] = None):
        """Constructor method.

        Parameters
        ----------
        name: str
            The name that identifies this setting.
        initial_value: int, float
            The initial value of this setting.
        min_allowable_value: int, float, optional
            The minimum allowable value. If not specified there's no limit
        max_allowable_value: int, float, optional
            The maximum allowable value. If not specified there's no limit
        """
        SettingsParameter.__init__(self, name)
        self.__current_value = initial_value
        self.__min_allowable_value = min_allowable_value
        self.__max_allowable_value = max_allowable_value

    def _check_value_in_range(self, value: Union[int, float]) -> bool:
        """Checks if the value is within the range.

        Parameters
        ----------
        value: int, float
            The value to check.

        Returns
        -------
        bool
            True if the value is in range. False otherwise.
        """
        if self.max_valid_value is not None and value > self.max_valid_value:
            return False
        elif self.min_valid_value is not None and value < self.min_valid_value:
            return False
        else:
            return True

    def do_get_value(self) -> Union[int, float]:
        """Get the current value.

        Returns
        -------
        int, float
            The current value.
        """
        return self.__current_value

    def do_set_value(self, value: Union[int, float]):
        """Set the current value, checking if it's in range.

        Parameters
        ----------
        value: int, float
            The new value to update.

        Raises
        ------
        ValueError
            If the value is out of range.
        """
        if not self._check_value_in_range(value):
            raise ValueError(
                f"The value ({value}) is out of range "
                f"({self.min_valid_value} <= value <= {self.max_valid_value})")
        else:
            self.__current_value = value

    @property
    def min_valid_value(self) -> Optional[Union[int, float]]:
        return self.__min_allowable_value

    @property
    def max_valid_value(self) -> Optional[Union[int, float]]:
        return self.__max_allowable_value


class SettingsFloatParameter(SettingsNumericParameter):
    """Numeric parameter whose value should be of type float."""

    def __init__(self, name: str, initial_value: float,
                 min_allowable_value: Optional[float] = None,
                 max_allowable_value: Optional[float] = None):
        """Constructor method.

        Parameters
        ----------
        name: str
            The name that identifies this setting.
        initial_value: float
            The initial value of this setting.
        min_allowable_value: float, optional
            The minimum allowable value. If not specified there's no limit
        max_allowable_value: float, optional
            The maximum allowable value. If not specified there's no limit
        """
        min_val = float(min_allowable_value) if min_allowable_value is not \
            None else None
        max_val = float(max_allowable_value) if max_allowable_value is not \
            None else None
        SettingsNumericParameter.__init__(self, name, float(initial_value),
                                          min_val, max_val)

    def do_set_value(self, value: float):
        """Set the current value, checking if it's in range.

        Parameters
        ----------
        value: float
            The new value to update.

        Raises
        ------
        ValueError
            If the value is out of range.
        """
        SettingsNumericParameter.do_set_value(self, float(value))


class SettingsIntegerParameter(SettingsNumericParameter):
    """Numeric parameter whose value should be of type int."""

    def __init__(self, name: str, initial_value: int,
                 min_allowable_value: Optional[int] = None,
                 max_allowable_value: Optional[int] = None):
        min_val = int(min_allowable_value) if min_allowable_value is not \
            None else None
        max_val = int(max_allowable_value) if max_allowable_value is not \
            None else None
        SettingsNumericParameter.__init__(self, name, int(initial_value),
                                          min_val, max_val)

    def do_set_value(self, value: int):
        """Set the current value, checking if it's in range.

        Parameters
        ----------
        value: int
            The new value to update.

        Raises
        ------
        ValueError
            If the value is out of range.
        """
        SettingsNumericParameter.do_set_value(self, int(value))


class SettingsBoolParameter(SettingsParameter):
    """Boolean parameter."""

    def __init__(self, name: str, initial_value: bool):
        """Constructor method.

        Parameters
        ----------
        name: str
            The name that identifies this setting.
        initial_value: bool
            The initial value of this setting.
        """
        SettingsParameter.__init__(self, name)
        self.__current_value = bool(initial_value)

    def do_get_value(self) -> bool:
        """Get the current value.

        Returns
        -------
        bool
            The current value.
        """
        return self.__current_value

    def do_set_value(self, value: bool):
        """Set the current value.

        Parameters
        ----------
        value: bool
            The new value to update.
        """
        self.__current_value = bool(value)


class SettingStringParameter(SettingsParameter):
    def __init__(self, name: str, default_value: str):
        SettingsParameter.__init__(self, name)
        self.__current_value: str = str(default_value)

    def do_get_value(self) -> str:
        return self.__current_value

    def do_set_value(self, value: str):
        self.__current_value = str(value)


class SettingsChoiceParameter:
    def __init__(self):
        raise RuntimeError("Not implemented yet")
