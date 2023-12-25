import asyncio
from pathlib import PurePath, Path
from typing import Union, Any, Dict

from rov_logger.logger import get_rov_logger
from rov_multiserver.settings_parameter import SettingsParameter
from rov_multiserver.settings_update_notifier import SettingsUpdateCallback
from ruamel import yaml

logger = get_rov_logger()


class SettingsSubject:
    def __init__(self, name: str, file_name: Union[str, PurePath],
                 *settings: SettingsParameter):
        self.__last_config_file_watch_time = None
        self.__name = str(name)
        self.__settings: Dict[str, SettingsParameter] = \
            {setting.name: setting for setting in settings}
        self.__file_path = Path(file_name).expanduser()
        self.__file_data = {}
        self.__update_local_data_from_file()
        for setting in settings:
            setting.start_observing_with(self.on_parameter_update)

    def on_parameter_update(self, parameter_name: str, new_value: Any):
        # This will be called when updated by the network.
        logger.info(f"Parameter {parameter_name} changed to {new_value}, "
                    f"the configuration file will be checked...")
        self.__update_file_from_local_data()

    @property
    def file_path(self) -> Path:
        return self.__file_path

    @property
    def file_name(self):
        return self.file_path.name
    
    @property
    def file_data(self):
        return self.__file_data.copy()

    @property
    def local_data(self) -> Dict[str, Any]:
        return {name: setting.value
                for name, setting in self.__settings.items()}

    @property
    def name(self) -> str:
        return self.__name

    def __update_local_data_from_file(self):
        """Updates local data from configuration file data.

        It opens the YAML configuration file, parses and try to set the
        corresponding values to the settings. In case of failure,
        the settings will preserve their default/previous value.

        An inverse update (from local to file) will be done afterwards to
        fix the file in case of invalid values.

        In case the configuration file doesn't exists, it will be created
        with the default local values.
        """
        if self.file_path.exists():
            with self.file_path.open() as file_stream:
                file_data = yaml.safe_load(file_stream)
                if file_data is not None and isinstance(file_data, dict):
                    self.__file_data.update(file_data)
            for setting_name, data in self.file_data.items():
                try:
                    self.__settings[setting_name].value = data
                    logger.debug(f"Setting '{setting_name}' value in file is:"
                                 f" {data}")
                except (ValueError, TypeError):
                    logger.warning(f"invalid value for {setting_name}: "
                                   f"{data}, using default/previous value: "
                                   f"{self.__settings[setting_name].value}")
                except KeyError:
                    logger.warning(f"Invalid key found on file: {setting_name}")
                    self.__file_data.pop(setting_name)
        self.__update_file_from_local_data()
        self.__last_config_file_watch_time = self.file_path.stat().st_mtime

    def __update_file_from_local_data(self) -> bool:
        """Update configuration file just if necessary.

        Checks if the local data is different to configuration file data. If
        so, updates the configuration file.

        Returns
        -------
        bool
            True if the configuration file was updated. False otherwise.
        """
        logger.debug(f"Checking if configuration file {self.file_name} "
                     f"should be updated...")
        should_update_file = False
        if len(self.__file_data) != len(self.__settings):
            logger.debug(f"File has {len(self.__file_data)} settings while "
                         f"there are {len(self.__settings)} settings "
                         f"locally. Updating file with local values.")
            should_update_file = True
        for key in self.__settings:
            if self.file_data.get(key) is None:
                logger.debug(f"key '{key}' not found in file. Updating file "
                             f"with "
                             f"local values.")
                should_update_file = True
                break
            if self.file_data.get(key) != self.__settings[key].value:
                logger.debug(f"'{key}' needs update (in file value: "
                             f"{self.__file_data[key]}, locally: "
                             f"{self.__settings[key].value}). Updating file "
                             f"with local values.")
                should_update_file = True
                break
        if should_update_file:
            self.__file_data.update(self.local_data)
            self.file_path.parent.mkdir(parents=True, exist_ok=True)
            with self.file_path.open(mode='w') as file_stream:
                yaml.YAML(typ='rt').dump(self.file_data, file_stream)
            logger.debug(f"Configuration file {self.file_name} was updated.")
        return should_update_file

    def start_watching_setting_with(self, setting_name: str,
                                    callback: SettingsUpdateCallback):
        """Subscribe a callback to setting updates.

        Parameters
        ----------
        setting_name: str
            The name of the setting to watch.
        callback: SettingsUpdateCallback
            The callback function/method for update notifications. It
            should have the signature: function(name: str, value: Any).

        Raises
        ------
        ValueError
            If no setting is found with the given name.
        TypeError
            If the callback's signature is not as SettingsUpdateCallback or
            it's not a callable object.
        """
        setting = self.__settings.get(setting_name)
        if setting is None:
            raise ValueError(f"Setting '{setting_name}' not found")
        else:
            setting.start_observing_with(callback)

    def start_watching_all_settings_with(self,
                                         callback: SettingsUpdateCallback):
        """Subscribe a callback for all settings related with this subject.

        Parameters
        ----------
        callback: SettingsUpdateCallback
            The callback function/method for update notifications. It
            should have the signature: function(name: str, value: Any).

        Raises
        ------
        TypeError
            If the callback's signature is not as SettingsUpdateCallback or
            it's not a callable object.
        """
        for setting in self.__settings:
            self.start_watching_setting_with(setting, callback)

    async def config_file_watcher_loop(self, min_interval=1.0):
        try:
            logger.debug(f"Watching config file: {self.file_name}")
            while True:
                if self.file_path.stat().st_mtime != \
                        self.__last_config_file_watch_time:
                    self.__update_local_data_from_file()
                await asyncio.sleep(min_interval)
        except asyncio.CancelledError:
            pass
