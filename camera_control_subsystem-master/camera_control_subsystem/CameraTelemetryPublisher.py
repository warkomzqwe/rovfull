from ruamel import yaml
import pymessaginglib.MessagingSystem as Ms


class CameraTelemetryPublisher:

  def __init__(self):
    self.__telemetry_topic = None

  def set_telemetry_topic(self, topic: str):
    self.__telemetry_topic = topic

  def send_telemetry(self, **kwargs):

    # Ignoring operation without set topic or kwargs
    if self.__telemetry_topic is None:
      return

    dic = dict()
    for key, value in kwargs.items():
      dic[key] = value

    yaml_dic = yaml.safe_dump(dic)
    Ms.send(self.__telemetry_topic, yaml_dic)
