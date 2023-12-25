from abc import ABCMeta, abstractmethod
from typing import Union


class IVehicleLink(metaclass=ABCMeta):
    """VehicleLink's base interface class."""
    @property
    @abstractmethod
    def fixed_yaw_power(self) -> bool:
        """
        Flag to specify if the power used in yaw is fixed or influenced by
        the input gain.
        Returns
        -------
        bool
            True if the yaw turning speed is fixed. False otherwise.
        """
        ...

    @fixed_yaw_power.setter
    @abstractmethod
    def fixed_yaw_power(self, value: bool):
        ...

    @property
    @abstractmethod
    def fixed_yaw_power_factor(self) -> float:
        """
        The amount of power used to turn yaw when
        :py:attr:`~fixed_yaw_power` is True. It should be > 0.0 and <=
        1.0

        Returns
        -------
        float
            The factor value.
        """
        ...

    @fixed_yaw_power_factor.setter
    @abstractmethod
    def fixed_yaw_power_factor(self, value: Union[int, float]):
        ...
