
from ruamel import yaml
from pymessaginglib.Topic import Topic, topic_factory, TopicObserver


class _TelemetryObs(TopicObserver):
  """
  Class receiver of telemetry commands.

  It receives only dictionaries as first entry else message get ignored.
  """

  def __init__(self):
    self.__dic_telemetry = dict()  # Empty dict to start
    self.telemetry_callbacks = set()

  def get_dictionary(self) -> dict:
    return self.__dic_telemetry

  def clear_dictionary(self):
    self.__dic_telemetry.clear()

  def update(self, value: str, ip: str):
    dic = yaml.safe_load(value)
    if not isinstance(dic, dict):
      # Check if data structure is dict
      return

    # Update internal dictionary
    self.__dic_telemetry.update(dic)

    for cb in self.telemetry_callbacks:
      # Passes new incoming data to callbacks
      cb(dic)


class CameraTelemetryListener:

  def __init__(self):
    self._topic_telemetry = None
    self._telemetry_obs = _TelemetryObs()

  def set_telemetry_topic(self, topic: str):

    if self._topic_telemetry is not None:
      # Topic subscription already exist.
      # Clear it first.
      assert isinstance(self._topic_telemetry, Topic)
      self._topic_telemetry.detach(self._telemetry_obs)

    # Create new topic to subscribe
    self._topic_telemetry = topic_factory.create_topic(topic)
    assert isinstance(self._topic_telemetry, Topic)
    self._topic_telemetry.attach(self._telemetry_obs)

  def get_telemetry(self) -> dict:
    return self._telemetry_obs.get_dictionary()

  def clear_telemetry_data(self):
    self._telemetry_obs.clear_dictionary()

  def add_telemetry_callback(self, cb):
    self._telemetry_obs.telemetry_callbacks.add(cb)

  def remove_telemetry_callback(self, cb):
    self._telemetry_obs.telemetry_callbacks.remove(cb)

  def clear_telemetry_callbacks(self):
    self._telemetry_obs.telemetry_callbacks.clear()
