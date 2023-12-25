from rov_multiserver.settings_parameter import SettingsBoolParameter
from rov_multiserver.settings_subject import SettingsSubject
from singleton import Singleton


class AutoRecordingEnabled(SettingsBoolParameter):
    PARAMETER_NAME = "enabled"
    DEFAULT_VALUE = False

    def __init__(self):
        SettingsBoolParameter.__init__(
            self, self.PARAMETER_NAME, self.DEFAULT_VALUE)


class AutoRecordingDaemonSettings(SettingsSubject, metaclass=Singleton):
    SUBJECT_NAME = "autorec"
    SUBJECT_SETTINGS_FILE = "~/.config/rov-multiserver/autorec.yaml"

    def __init__(self, config_file_name: str = SUBJECT_SETTINGS_FILE):
        enabled = AutoRecordingEnabled()
        SettingsSubject.__init__(self,
                                 AutoRecordingDaemonSettings.SUBJECT_NAME,
                                 config_file_name, enabled)
        self.__autorec_enabled = enabled

    @property
    def autorec_enabled(self) -> bool:
        return self.__autorec_enabled.value
