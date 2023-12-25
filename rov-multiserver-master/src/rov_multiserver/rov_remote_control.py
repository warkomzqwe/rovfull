"""Remote control module."""
from argparse import ArgumentParser, REMAINDER
from threading import Event, Lock
from time import sleep, time

from pymessaginglib import get_interfaces, set_interface
from pymessaginglib.MessagingSystem import send, receiver
from pymessaginglib.Topic import topic_factory, TopicObserver
from rov_logger.logger import get_rov_logger

logger = get_rov_logger()


class _NotificationsObserver(TopicObserver):
    def __init__(self):
        self.__new_notification = Event()
        self.__lock = Lock()
        self.__last_notification = None

    def on_message_arrival(self, dic: dict, ip: str):
        new_notification_text = dic.get('text')
        if new_notification_text is not None:
            with self.__lock:
                self.__last_notification = new_notification_text
            self.new_notification.set()

    @property
    def new_notification(self):
        return self.__new_notification

    @property
    def last_notification(self):
        return self.__last_notification


class _RecStateObserver(TopicObserver):
    def __init__(self):
        self.__new_state = Event()
        self.__state = None

    def on_message_arrival(self, dic: dict, ip: str):
        new_state = dic.get('state')
        if new_state is not None:
            if self.__state != new_state:
                self.__state = new_state
                self.__new_state.set()

    def wait_for_state(self, state, timeout=30):
        starting_time = time()
        if state not in ["IDLE", "STARTING", "REC", "SAVING"]:
            raise ValueError(f"Not a valid state: {state}")
        logger.debug(f"Waiting for recording state: {state}")
        if self.state == state:
            logger.debug(f"Recording state is: {state}")
            return True
        else:
            while self.state != state:
                self.new_state.clear()
                time_left = timeout - time() + starting_time
                if time_left > 0:
                    self.new_state.wait(time_left)
                else:
                    logger.debug("Timeout waiting for recording state: "
                                 f"{state}")
                    return False
            logger.debug(f"Recording state is now: {self.state}")
            return True

    @property
    def new_state(self):
        return self.__new_state

    @property
    def state(self):
        return self.__state


class RovRemoteControl:
    def __init__(self, netiface=None):
        self.__notifications_observer = _NotificationsObserver()
        topic_factory.create_topic(
            "/gcs/notifications/").attach(self.__notifications_observer)
        self.__rec_state_observer = _RecStateObserver()
        topic_factory.create_topic(
            "/camera/main/rec_state/").attach(self.__rec_state_observer)
        receiver.start()
        available_interfaces = self.__get_available_interfaces()
        if netiface is not None and netiface in available_interfaces:
            interface = netiface
        elif netiface is not None and netiface not in available_interfaces:
            interface = available_interfaces[0]
            logger.error(f"Network interface not found: '{netiface}'")
            logger.error(f"Available interfaces: {available_interfaces}). ")
            raise RuntimeError(f"Network interface not found: '{netiface}', "
                               f"available interfaces: {available_interfaces}")
        else:
            interface = available_interfaces[0]
        logger.info(f"Setting output network interface: '{interface}'")
        set_interface(interface)

    def take_snapshot(self, timeout=20):
        logger.debug("Sending BKB push signal")
        send(topic_factory.create_topic("/client/joystick/"),
             dict(button_event="BKB;P"))
        sleep(0.5)
        self.__notifications_observer.new_notification.clear()
        logger.debug("Sending BKB release signal")
        send(topic_factory.create_topic("/client/joystick/"),
             dict(button_event="BKB;R"))
        logger.info("Snapshot requested")
        if not self.__notifications_observer.new_notification.wait(timeout):
            logger.error("Timeout waiting for snapshot notification (maybe no "
                         "server is running in this network?)")
            return
        if "NEW SNAPSHOT" in self.__notifications_observer.last_notification:
            self.__notifications_observer.new_notification.clear()
            logger.info("New snapshot taken, overlaying text...")
            self.__notifications_observer.new_notification.wait(timeout)
        if "SNAPSHOT STORED:" in \
                self.__notifications_observer.last_notification:
            text = self.__notifications_observer.last_notification
            filename = text.split(" ")[-1]
            logger.info(f"Snapshot stored: {filename}")

    def toggle_recording(self):
        starting_state = self.get_recording_state(12)
        if starting_state == 'IDLE':
            self.start_recording()
        elif starting_state == 'REC':
            self.stop_recording()
        else:
            logger.error(f"Invalid state for toggle: {starting_state}")
            return
        logger.info("Recording state toggle requested" +
                    (" (REC Starting)" if starting_state == "IDLE"
                     else " (REC Stopping)"))

    def start_recording(self):
        rec_state = self.get_recording_state(12)
        if rec_state == 'IDLE':
            logger.info("Requesting to start recording...")
            send(topic_factory.create_topic('/camera/main/rec_cmd/'),
                     dict(cmd='start'))
        else:
            logger.warning(f"Can't start recording on state: {rec_state}")

    def stop_recording(self):
        rec_state = self.get_recording_state(12)
        if rec_state == 'REC':
            logger.info("Requesting to stop recording...")
            send(topic_factory.create_topic('/camera/main/rec_cmd/'),
                     dict(cmd='stop'))
        else:
            logger.warning(f"Can't stop recording on state: {rec_state}")

    def recording_clip(self, clip_time):
        clip_time = float(clip_time)
        actual_state = self.get_recording_state(10)
        if actual_state != "IDLE":
            logger.warning("Not in IDLE recording state, can't issue a "
                           "recording clip")
            return
        logger.info(f"Requesting to record a clip of {clip_time} seconds")
        self.toggle_recording()
        sleep(clip_time)
        if self.__rec_state_observer.state != "REC":
            logger.error("The server didn't change its recording state to "
                         "REC (maybe no server is running on this network?)")
            return
        self.toggle_recording()
        self.__rec_state_observer.wait_for_state('IDLE')
        logger.info("Recording stopped")

    def get_recording_state(self, timeout=5):
        timeout = float(timeout)
        if self.__rec_state_observer.state is None and \
                not self.__rec_state_observer.new_state.wait(timeout):
            logger.warning("Timeout waiting for recording state (maybe no "
                           "server is running in this network?)")
            return None
        logger.info(f"Recording state: {self.__rec_state_observer.state}")
        return self.__rec_state_observer.state


    @staticmethod
    def __get_available_interfaces():
        interface_strings = get_interfaces()
        interfaces = []
        for interface_string in interface_strings:
            interfaces.append(interface_string.split(" => ")[0])
        return interfaces


def parse_args():
    parser = ArgumentParser(description="Send a command to the ROV")
    parser.add_argument("-v", "--verbosity", action="count",
                        help="Increase output verbosity (2 times for the "
                             "maximum)")
    parser.add_argument("--interface", "-i", help="Specify an output network "
                                                  "interface")
    parser.add_argument("cmd", help="The command to execute",
                        choices=["snapshot", "rec", "recstate"])
    parser.add_argument("cmdarg", help="(optional) command arguments",
                        nargs='*')
    return parser.parse_args()


def main():
    get_rov_logger('WARNING')
    args = parse_args()
    if args.verbosity is not None:
        if args.verbosity == 1:
            get_rov_logger('INFO')
        elif args.verbosity >= 2:
            get_rov_logger('DEBUG')
    remote_control = RovRemoteControl(args.interface)
    while '' in args.cmdarg:
        args.cmdarg.remove('')  # remove unnecessary empty strings
    logger.info(
        f"Processing command: {args.cmd}" + (f" with args: {args.cmdarg}"
                                             if len(args.cmdarg) > 0
                                             else ""))
    if args.cmd == "snapshot":
        remote_control.take_snapshot()
    elif args.cmd == "rec":
        if len(args.cmdarg) > 0:
            remote_control.recording_clip(args.cmdarg[0])
        else:
            remote_control.toggle_recording()
    elif args.cmd == "recstate":
        if len(args.cmdarg) > 0:
            remote_control.get_recording_state(args.cmdarg[0])
        else:
            remote_control.get_recording_state()

if __name__ == '__main__':
    main()
