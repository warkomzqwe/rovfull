from secrets import token_hex
from ruamel import yaml

import pymessaginglib.MessagingSystem as Ms
from pymessaginglib.Topic import Topic, topic_factory, TopicObserver


class CameraControlPublisher(TopicObserver):
  """
  Publish camera controls to a predefined topic.
  """

  def __init__(self):
    self.__last_token = None
    self.__ack = False
    self.__bytes_for_token = 4

    self._topic_control = None
    self._topic_control_ack = None

  def _manage_topic_subscription(self, topic: str, topic_pointer: Topic):

    if topic_pointer is not None:
      # Topic subscription already exist.
      # Clear it first.
      assert isinstance(topic_pointer, Topic)
      topic_pointer.detach(self)

    # Create new topic to subscribe
    topic_pointer = topic_factory.create_topic(topic)
    assert isinstance(topic_pointer, Topic)
    topic_pointer.attach(self)

  def set_control_topic(self, topic: str):
    self._topic_control = topic

  def set_control_ack_topic(self, topic: str):
    self._manage_topic_subscription(topic, self._topic_control_ack)

  def __send_preparation(self):
    self.__ack = False  # Change state to wait for ack.

    """ Generate a new token with every new control command. """
    self.__last_token = token_hex(self.__bytes_for_token)

  def __safe_send(self, dic_yaml):
    if self._topic_control is not None:
      Ms.send(self._topic_control, dic_yaml)

  def send_tilt_step(self, tilt_step: int):
    """
    Send new camera tilt step movement requirement.
    :param tilt_step: Amount of steps to move camera
    """

    self.__send_preparation()
    dic = dict(tilt_step=tilt_step, token_hex=self.__last_token)
    dic_yaml = yaml.safe_dump(dic)
    self.__safe_send(dic_yaml)

  def send_pan_step(self, pan_step: int):
    """
    Send new camera pan step movement requirement.
    :param pan_step: Amount of steps to move camera

    """

    self.__send_preparation()
    dic = dict(pan_step=pan_step, token_hex=self.__last_token)
    dic_yaml = yaml.safe_dump(dic)
    self.__safe_send(dic_yaml)

  def send_tilt_speed(self, tilt_speed: float):
    """
    Send new camera tilt speed desired.
    :param tilt_speed: Value between -1.0-1.0
    """

    self.__send_preparation()
    dic = dict(tilt_speed=tilt_speed, token_hex=self.__last_token)
    dic_yaml = yaml.safe_dump(dic)
    self.__safe_send(dic_yaml)

  def send_pan_speed(self, pan_speed: float):
    """
    Send new camera tilt speed desired.
    :param pan_speed: Value between -1.0-1.0
    """

    self.__send_preparation()
    dic = dict(pan_speed=pan_speed, token_hex=self.__last_token)
    dic_yaml = yaml.safe_dump(dic)
    self.__safe_send(dic_yaml)

  def send_reset_position(self):
    """
    Send a reset camera position requirement.
    """

    self.__send_preparation()
    dic = dict(reset_position=0, token_hex=self.__last_token)
    dic_yaml = yaml.safe_dump(dic)
    self.__safe_send(dic_yaml)

  def update(self, value: str, ip: str):
    """
    Process incoming data from ack topic.

    :param value: Data in message, must be a dictionary containing a token_hex key.
    :param ip: Ip from ack sender.
    """

    """ First, process incoming data. """
    dic = yaml.safe_load(value)

    """ Check if data meets the standard. """
    try:
      token = dic['token_hex']
    except KeyError:
      print('CameraControlPublisher: Non existent key data in message.')
      return

    """ If token matches, we have an ack confirmation. """
    if token == self.__last_token:
      self.__ack = True

  def got_ack(self) -> bool:
    return self.__ack
