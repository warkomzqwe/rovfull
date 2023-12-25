from threading import Lock
from time import sleep

from pymessaginglib.Topic import topic_factory, Topic
from rov_logger.logger import get_rov_logger


class ServoLockService:

    _topic_str = "/vehicle/navigation/servo/"
    _topic_motor_overcurrent_str = "/vehicle/motors/events/"
    _topic_notification_str = "/gcs/notifications/"
    # Servo mapping uses an array for servos from 1 to 8 with associated motors
    # Array contains associated motors number ordering.
    _servo_mapping = [1, 2, 3, 4, 5, 6, 7, 8]

    def __init__(self, vehicle_com):
        self._vehicle_com = vehicle_com
        self.logger = get_rov_logger()
        self._lock = Lock()

        # Messaging related variables
        self._topic: Topic = topic_factory.create_topic(self._topic_str)
        self._topic_motor_overcurrent: Topic = topic_factory.create_topic(self._topic_motor_overcurrent_str)
        self._topic_notifications: Topic = topic_factory.create_topic(self._topic_notification_str)

        self._topic.attach_callback(self.on_message_arrival)
        self._topic_motor_overcurrent.attach_callback(self.on_overcurrent_message_arrival)

    def lock_servo(self, servo_id):
        """Servo expected in range 1 to 8"""

        self._lock.acquire()
        try:
            # Sending msg to system
            msg = f"PROTECTION SYSTEM\nLocking servo {servo_id} (Motor {self._servo_mapping[servo_id - 1]})"
            self.logger.info(msg)
            self._topic_notifications.send(dict(text=msg, duration=10))

            # Checking armed state for rov - useful for later
            rov_was_armed = self._vehicle_com.is_armed()

            # Main locking servo routine
            self._vehicle_com.do_disarm()
            sleep(1.0)
            self._vehicle_com.disable_servo(servo_id)
            sleep(1.0)
            if rov_was_armed:
                self._vehicle_com.do_arm()
        finally:
            self._lock.release()

    def unlock_servo(self, servo_id):
        self.logger.info(f"Unlocking servo {servo_id}.")
        self._vehicle_com.enable_servo(servo_id, self._servo_mapping[servo_id - 1])

    def unlock_all_servos(self):
        for i in range(0, len(self._servo_mapping)):
            self.unlock_servo(i + 1)

    # noinspection PyUnusedLocal
    def on_message_arrival(self, dic, ip):
        lock_servo_id = dic.get("lock_servo")
        unlock_servo_id = dic.get("unlock_servo")

        if lock_servo_id:
            self.lock_servo(lock_servo_id)

        elif unlock_servo_id:
            self.unlock_servo(unlock_servo_id)

        else:
            pass

    # noinspection PyUnusedLocal
    def on_overcurrent_message_arrival(self, dic, ip):
        even_type = dic.get("event_type")
        servo_number = dic.get("servo_number")

        if even_type == "overcurrent" and isinstance(servo_number, int):
            self.logger.warning(f"Overcurrent event in servo: {servo_number}.")
            self.lock_servo(servo_number)
