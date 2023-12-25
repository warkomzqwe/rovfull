from abc import ABCMeta, abstractmethod

from .enums import PowerSourceType


class IPowerSourceDetector(metaclass=ABCMeta):
    @abstractmethod
    def listen_forever(self):
        """Blocks and waits for a change in the power source.

        It must call "on_new_power_source_type()" every time a change is
        detected.
        """
        ...

    @abstractmethod
    def on_new_power_source(self, power_source_type: PowerSourceType):
        """New power source detected callback method.

        This should be called every time the active power source changes.
        It will typically publish the change using pymessaginglib.

        Parameters
        ----------
        power_source_type: PowerSourceType
            A power source type as specified by the PowerSourceType enum.
        """
        ...

    @abstractmethod
    def close(self):
        """Stop the detector releasing its resources"""
        ...
