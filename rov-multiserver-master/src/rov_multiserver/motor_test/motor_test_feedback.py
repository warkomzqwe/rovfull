"""
Module for controlling spam of incoming motor test feedback.
"""
from rov_logger.logger import get_rov_logger
from rov_multiserver.messaging_bus.outgoing_messages import send_motor_test_feedback


logger = get_rov_logger()


class _MotorTestFeedback:
    """Handler class for incoming motor test actions

    Class delivers human-readable information via messaging bus or logger
    channels. Internally information is updated too quickly, so we skip
    some extra info to avoid information spam.
    """

    command_result_max_skip = 50  # number of messages to skip from log spam

    def __init__(self):
        self.last_result = -1  # last command result
        self.command_result_count = 0  # simple counter avoids unnecessary spamming log

    @staticmethod
    def _log_command_result(result_value):
        """
        Helper class for deliver different messages according to motor command
        result from COMMAND_ACK mavlink message.
        """

        if result_value == 0:
            logger.debug(f'Motor command test success!')
            send_motor_test_feedback(True)

        else:
            logger.debug(f'Motor command test error with code: {result_value}')
            send_motor_test_feedback(False)

    def on_command_result(self, result):
        """
        Handler for incoming motor test command results from mavlink.

        :param result: Results code according to COMMAND_ACK mavlink messages
        """

        # Forcing new results to be shown immediately
        if self.last_result != result:
            self.command_result_count = self.command_result_max_skip
            self.last_result = result

        # Log message skip control
        if self.command_result_count == self.command_result_max_skip:
            self.command_result_count = 0
            self._log_command_result(result)

        else:
            self.command_result_count += 1


motor_test_feedback = _MotorTestFeedback()


def deliver_motor_test_feedback(feedback):
    motor_test_feedback.on_command_result(result=feedback)
