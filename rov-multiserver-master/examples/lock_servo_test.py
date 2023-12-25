from pymessaginglib import set_broadcast_address
from pymessaginglib.Topic import topic_factory, Topic
from threading import Event
from enum import Enum, auto

topic_str = "/vehicle/navigation/servo/"
topic_motor_overcurrent_str = "/vehicle/motors/events/"
topic: Topic = topic_factory.create_topic(topic_str)
topic_motor_overcurrent = topic_factory.create_topic(topic_motor_overcurrent_str)
set_broadcast_address("127.255.255.255")
stop_loop_signal = Event()


class LockMode(Enum):
    LOCK = auto()
    UNLOCK = auto()
    OVERCURRENT = auto()


servo_ids = [str(x) for x in range(1,9)]
lock_mode = LockMode.LOCK


def enter_lock_mode():
    global lock_mode
    lock_mode = LockMode.LOCK


def enter_unlock_mode():
    global lock_mode
    lock_mode = LockMode.UNLOCK


def enter_overcurrent_mode():
    global lock_mode
    lock_mode = LockMode.OVERCURRENT


def lock_servo(servo_id):
    topic.send(dict(lock_servo=servo_id))


def unlock_servo(servo_id):
    topic.send(dict(unlock_servo=servo_id))


def overcurrent_event(servo_id):
    topic_motor_overcurrent.send(dict(event_type="overcurrent", servo_number=servo_id))


def main():
    global lock_mode
    lock_mode = LockMode.LOCK

    print("Locking servo service test utility. Insytech 2022.\n")
    print("[l] for locking mode")
    print("[u] for unlocking mode")
    print("[o] for overcurrent event mode")
    print("[1-8] key is locking/unlocking servo output.")

    while not stop_loop_signal.wait(0.1):
        key = input("...")

        if key == "l":
            print("Lock mode.")
            enter_lock_mode()

        elif key == "u":
            print("Unlock mode.")
            enter_unlock_mode()

        elif key == "o":
            print("Overcurrent mode.")
            enter_overcurrent_mode()

        elif key in servo_ids:

            if lock_mode == LockMode.LOCK:
                lock_servo(int(key))

            elif lock_mode == LockMode.UNLOCK:
                unlock_servo(int(key))

            elif lock_mode == LockMode.OVERCURRENT:
                overcurrent_event(int(key))

            else:
                pass

        else:
            print("Wrong key...")


if __name__ == '__main__':
    main()
