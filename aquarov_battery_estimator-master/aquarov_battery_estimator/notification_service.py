"""
Low Battery Notification Service
"""
from time import time
from pymessaginglib.MessagingSystem import send
from pymessaginglib.Topic import topic_factory


class LowBatteryNotificationService:

    def __init__(self):
        self.notification_gap = 7
        self.notification_span = 2
        self.critical_battery_level = 25.0
        self.last_notification_timestamp = 0
        self.notification_topic = topic_factory.create_topic("/gcs/notifications/")

    # ** Private methods

    def _send_notification(self):

        dic = dict(
            text="Battery at Critical Level.",
            duration=self.notification_span,
        )

        send(self.notification_topic, dic)

    # ** Public methods

    def update_measure(self, charge_level):

        if charge_level < self.critical_battery_level:

            current_time = int(time())

            time_between_notifications = current_time - self.last_notification_timestamp

            if time_between_notifications > self.notification_gap:

                self._send_notification()
                self.last_notification_timestamp = current_time
