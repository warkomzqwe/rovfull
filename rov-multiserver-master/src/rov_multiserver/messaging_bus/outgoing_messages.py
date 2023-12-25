"""
Module to put in one file outgoing robotic lan messages
"""

from pymessaginglib.Topic import topic_factory as tp


new_topic = tp.create_topic


class Topics:
    hud_notifications = new_topic("/gcs/notifications/")
    vehicle_navigation_feedback = new_topic("/vehicle/navigation_subsystem/feedback/")


def send_hud_notification(text, duration):
    Topics.hud_notifications.send(dict(text=text, duration=duration))


def send_nav_feedback_msg(msg):
    Topics.vehicle_navigation_feedback.send(dict(msg=msg))


def send_mag_calibration_feedback(pct):
    send_nav_feedback_msg(f"Mag Calibration: {pct}%")


def send_motor_test_feedback(success: bool):
    if success:
        send_nav_feedback_msg("Motor Test Started...")
    else:
        send_nav_feedback_msg("Motor Test Failure...")


def report_vehicle_armed(is_armed):
    if is_armed:
        send_nav_feedback_msg("Motors ARMED")
    else:
        send_nav_feedback_msg("Motors DISARMED")
