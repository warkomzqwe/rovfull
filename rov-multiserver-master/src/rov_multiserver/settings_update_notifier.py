import inspect
from typing import Callable, Any, Dict

SettingsUpdateCallback = Callable[[str, Any], Any]
"""
Callback method/function signature for SettingsUpdateNotifier.

The expected signature is: method(name: str, value: Any) -> Any  
"""


class SettingsUpdateNotifier:
    """Helper class that enables the observer pattern for settings."""
    _CallbackMap = Dict[int, SettingsUpdateCallback]

    def __init__(self):
        self.__last_callback_id = 0
        self.__callbacks_map: SettingsUpdateNotifier._CallbackMap = {}

    def start_observing_with(self, callback: SettingsUpdateCallback) -> int:
        """Subscribe a callback function/method for update notifications.

        Parameters
        ----------
        callback : SettingsUpdateCallback
            The callback function/method for update notifications. It
            should have the signature: function(name: str, value: Any).

        Returns
        -------
        int
            A callback identifier that references this function. It
            could be used to unsubscribe this function.

        Raises
        ------
        TypeError
            If the callback's signature is not as SettingsUpdateCallback or
            it's not a callable object.
        """
        if not callable(callback):
            raise TypeError("The callback function is not a callable object")
        signature = inspect.signature(callback)
        if len(signature.parameters) < 2:
            raise TypeError("The callback function has too few arguments")
        parameters = list(signature.parameters.values())
        for parameter in list(parameters):
            if parameter.kind == inspect.Parameter.KEYWORD_ONLY and \
                    parameter.default == inspect.Parameter.empty:
                raise TypeError(f"The callback function has a keyword-only "
                                f"parameter ('{parameter.name}') with no "
                                f"default value.")
            if parameter.default != inspect.Parameter.empty:
                parameters.remove(parameter)
        if len(parameters) > 2:
            raise TypeError("The callback function has too many arguments")
        callback_id = self.__generate_new_id()
        self.__callbacks_map[callback_id] = callback
        return callback_id

    def stop_observing(self, callback_id: int):
        """Unsubscribe a previously subscribed callback function/method.

        Parameters
        ----------
        callback_id: int
            The callback function/method identifier.

        Raises
        ------
        ValueError
            If there's not any registered function/method with the given
            identifier.
        """
        if callback_id in self.__callbacks_map:
            self.__callbacks_map.pop(callback_id)
        else:
            raise ValueError(f"There's no callback registered for id: "
                             f"{callback_id}")

    def notify_update(self, name: str, value: Any):
        """Notify the value update to all observers.

        Call every registered callback function/method with the name and
        value of the parameter that was updated as the input arguments.

        Parameters
        ----------
        name: str
            The name of the parameter that was recently updated.
        value
            The value of the parameter that was recently updated.

        Returns
        -------

        """
        for callback_function in self.__callbacks_map.values():
            callback_function(name, value)

    @property
    def active_callbacks(self):
        return list(self.__callbacks_map.keys())

    def __generate_new_id(self) -> int:
        """Generate a new callback identifier."""
        new_id = self.__last_callback_id
        self.__last_callback_id += 1
        return new_id
