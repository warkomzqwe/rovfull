"""
Standalone module for functional testing of **motor test** mavlink command.
"""

from threading import Thread, Event
from time import sleep
from pymavlink import mavutil
from glob import glob


ardupilot_serial_port = glob('/dev/serial/by-id/usb-ArduPilot*')
master = mavutil.mavlink_connection(ardupilot_serial_port[0], baud=115200)
stop_receiving_signal = Event()
stop_motor_test_signal = Event()


master.wait_heartbeat()


def receiver():
    """
    Keeps reading incoming mavlink messages from ardusub.
    """
    while not stop_receiving_signal.wait(0.1):
        try:
            print(master.recv_match().to_dict())

        except AttributeError:
            pass


# Starting ardusub message receiver
t1 = Thread(target=receiver, name='mavlink_receiver_thread')
t1.start()

# Arming the vehicle
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1, 0, 0, 0, 0, 0, 0)


master.motors_armed_wait()


motor_id = 0
throttle_type = 0  # 0=pct, 1=pwm, 2=passthrough pilot
throttle_value = 80
timeout = 0
motor_count = 0
test_order = 2  # MOTOR_TEST_ORDER_BOARD

count = 0

while not stop_motor_test_signal.wait(0.05):
    """
    Command is required to be delivered with 50ms period.
    
    This is not clearly documented in mavlink common messages set:
    https://mavlink.io/en/messages/common.html#MAV_CMD_DO_MOTOR_TEST
    
    Please take care of this when delivering to production.
    """

    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST,
        0,
        motor_id,
        throttle_type,
        throttle_value,
        timeout,
        motor_count,
        test_order,
        0)

    count += 1
    if count == 40:
        stop_motor_test_signal.set()


# Exit program
sleep(5)
stop_receiving_signal.set()
