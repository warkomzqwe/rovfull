from ruamel import yaml

import pymessaginglib.MessagingSystem as Ms
from pymessaginglib.Topic import Topic, topic_factory, TopicObserver


class CameraControlListener(TopicObserver):

  def __init__(self):

    # Callbacks are stored in a set to avoid duplicates.
    self.tilt_step_callbacks = set()
    self.pan_step_callbacks = set()
    self.tilt_speed_callbacks = set()
    self.pan_speed_callbacks = set()
    self.reset_position_callbacks = set()

    self.__topic_control = None
    self.__topic_control_ack = None
    
  def __manage_topic_subscription(self, topic: str, topic_obj: Topic):
    
    if topic_obj is not None:
      # Topic subscription already exist.
      # Clear it first.
      assert isinstance(topic_obj, Topic)
      topic_obj.detach(self)

    # Create new topic to subscribe
    topic_obj = topic_factory.create_topic(topic)
    assert isinstance(topic_obj, Topic)
    topic_obj.attach(self)
    
  def set_control_topic(self, topic: str):
    self.__manage_topic_subscription(topic, self.__topic_control)
    
  def set_control_ack_topic(self, topic: str):
    self.__topic_control_ack = topic
    
  def update(self, value: str, ip: str):
    """ Incoming data in camera control commands topic. """

    data = yaml.safe_load(value)

    """ 
    Check for matching standards incoming data.
    
    If data doesn't match standards it gets ignored but a warning is raised.
    """

    def throw_warning():
      print('CameraControlListener: Incoming data not matching standard.')

    """ 1) Check if data is a dictionary. """
    if not isinstance(data, dict):
      throw_warning()
      return

    """ 2) Check if required token_hex key is present and meets standards. """
    if 'token_hex' not in data:
      throw_warning()
      return

    token_hex = data['token_hex']

    if not isinstance(token_hex, str):
      throw_warning()
      return

    """ 3) Check if relevant fields are present and deliver to callbacks. """
    if 'tilt_step' in data:
      tilt_step = data['tilt_step']
      if not isinstance(tilt_step, int):
        raise ValueError('tilt_step not an integer')

      for callback in self.tilt_step_callbacks:
        callback(tilt_step)

    elif 'pan_step' in data:
      pan_step = data['pan_step']
      if not isinstance(pan_step, int):
        raise ValueError('pan_step not an integer')

      for callback in self.pan_step_callbacks:
        callback(pan_step)

    elif 'tilt_speed' in data:
      tilt_speed = float(data['tilt_speed'])
      if tilt_speed < -1.0 or tilt_speed > 1.0:
        raise ValueError

      for callback in self.tilt_speed_callbacks:
        callback(tilt_speed)

    elif 'pan_speed' in data:
      pan_speed = float(data['pan_speed'])
      if pan_speed < -1.0 or pan_speed > 1.0:
        raise ValueError

      for callback in self.pan_speed_callbacks:
        callback(pan_speed)

    elif 'reset_position' in data:
      for callback in self.reset_position_callbacks:
        callback()

    else:
      """ Not matching standard keys in message. """
      throw_warning()
      return

    """ 
    4) At this point data meets standard and we deliver an ACK message back.
    """
    ack_dic = {
      'token_hex': token_hex
    }

    if self.__topic_control_ack is not None:
      Ms.send(self.__topic_control_ack, yaml.safe_dump(ack_dic))



