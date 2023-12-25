import asyncio
import enum
from argparse import ArgumentParser
from typing import Union

import aiojobs
import async_timeout
from pymessaginglib import set_broadcast_address
from pymessaginglib.MessagingSystem import send, receiver
from pymessaginglib.Topic import TopicObserver, topic_factory
from rov_logger.logger import get_rov_logger
from rov_multiserver.autorec_controller_settings import \
    AutoRecordingDaemonSettings

logger = get_rov_logger()
address = '192.168.2.255'
set_broadcast_address(address)


class AbstractAsyncIOEventLoopHolder:
    def __init__(self):
        self.__loop = asyncio.get_event_loop()

    @property
    def loop(self) -> asyncio.AbstractEventLoop:
        return self.__loop


class DepthObserver(TopicObserver, AbstractAsyncIOEventLoopHolder):
    def __init__(self):
        AbstractAsyncIOEventLoopHolder.__init__(self)
        self.__new_depth_data = asyncio.Event()
        self.__depth = None

    def on_message_arrival(self, dic: dict, ip: str):
        if 'depth' in dic:
            self._update_depth(dic['depth'])

    def _update_depth(self, depth: Union[int, float]):
        self.__depth = float(depth)
        logger.debug(f"D: {self.__depth}")
        self.loop.call_soon_threadsafe(self.new_depth_data.set)

    @property
    def new_depth_data(self) -> asyncio.Event:
        return self.__new_depth_data

    @property
    def depth(self) -> float:
        return self.__depth


class RecordingStateObserver(TopicObserver, AbstractAsyncIOEventLoopHolder):
    def __init__(self):
        AbstractAsyncIOEventLoopHolder.__init__(self)
        self.__new_state = asyncio.Event()
        self.__state = None

    def on_message_arrival(self, dic: dict, ip: str):
        new_state = dic.get('state')
        if new_state is not None:
            logger.debug(f"RS: {new_state}")
            if self.__state != new_state:
                self.__state = new_state
                self.loop.call_soon_threadsafe(self.new_state.set)

    @property
    def new_state(self):
        return self.__new_state

    @property
    def state(self):
        return self.__state


def notify_user(message):
    send(topic_factory.create_topic('/gcs/notifications/'),
         {'text': message, 'period': 3, 'duration': 3})
    logger.debug(f'New notification sent to /gcs/notifications/: "{message}"')


class AutoRecordingDaemon(AbstractAsyncIOEventLoopHolder):
    DEFAULT_REC_STATE_CHANGE_TIMEOUT = 12
    DEFAULT_DEPTH_THRESHOLD = 0.5
    DEFAULT_MAX_CLIP_LENGTH = 300
    DEFAULT_MIN_CLIP_LENGTH = 60

    class State(enum.Enum):
        NOT_ENOUGH_DATA = enum.auto()
        NOT_RECORDING = enum.auto()
        RECORDING = enum.auto()

    def __init__(self):
        AbstractAsyncIOEventLoopHolder.__init__(self)
        depth_topic = topic_factory.create_topic('/vehicle/data/')
        depth_observer = DepthObserver()
        depth_topic.attach(depth_observer)
        rec_state_topic = topic_factory.create_topic('/camera/main/rec_state/')
        rec_state_observer = RecordingStateObserver()
        rec_state_topic.attach(rec_state_observer)
        self.__depth_observer = depth_observer
        self.__rec_state_observer = rec_state_observer
        receiver.start()

    @property
    def depth_observer(self) -> DepthObserver:
        return self.__depth_observer

    @property
    def rec_state_observer(self) -> RecordingStateObserver:
        return self.__rec_state_observer

    async def wait_for_new_depth(self):
        await self.depth_observer.new_depth_data.wait()
        self.depth_observer.new_depth_data.clear()
        return self.depth_observer.depth

    async def wait_for_new_rec_state(self):
        await self.rec_state_observer.new_state.wait()
        self.rec_state_observer.new_state.clear()
        return self.rec_state_observer.state

    async def wait_for_rec_state(self, state: str,
                                 timeout: Union[float, int]) -> bool:
        if state not in ('IDLE', 'STARTING', 'REC', 'SAVING'):
            raise ValueError(f"Not a valid state: {state}")
        logger.debug(f"Waiting for recording state: {state}...")
        if self.rec_state_observer.state == state:
            logger.debug("It's the current state.")
            return True
        try:
            async with async_timeout.timeout(timeout):
                if await self.wait_for_new_rec_state() == state:
                    logger.debug(f"Recording state: {state}")
                    return True
        except asyncio.TimeoutError:
            logger.debug("Timeout waiting...")
            return False

    async def depth_stream_feeding_loop(self, queue: asyncio.Queue):
        while True:
            new_depth = await self.wait_for_new_depth()
            logger.debug(f"new depth: {new_depth}")
            await queue.put(self.depth_observer)

    async def rec_state_stream_feeding_loop(self, queue: asyncio.Queue):
        while True:
            new_rec_state = await self.wait_for_new_rec_state()
            logger.debug(f"new rec state: {new_rec_state}")
            await queue.put(self.rec_state_observer)

    async def main_loop(self):
        stream_queue = asyncio.Queue(1)
        # background loops
        scheduler = await aiojobs.create_scheduler()
        await scheduler.spawn(self.depth_stream_feeding_loop(stream_queue))
        await scheduler.spawn(self.rec_state_stream_feeding_loop(stream_queue))
        # initial state
        last_rec_start_time = None
        state = self.State.NOT_ENOUGH_DATA
        depth_trigger = False
        try:
            while True:
                last_depth = self.depth_observer.depth
                last_rec_state = self.rec_state_observer.state
                if state == self.State.NOT_ENOUGH_DATA:
                    if last_depth is None or last_rec_state is None:
                        logger.info("Not enough data to start, waiting...")
                        await stream_queue.get()
                    else:
                        if last_rec_state == 'IDLE':
                            logger.info("Server is not recording")
                            state = self.State.NOT_RECORDING
                        else:
                            logger.info("Server is recording")
                            state = self.State.RECORDING
                elif state == self.State.NOT_RECORDING:
                    if last_rec_state != 'IDLE':
                        state = self.State.RECORDING
                        continue
                    if last_depth >= self.depth_threshold:
                        logger.info("Depth triggered a start recording")
                        await self.start_recording()
                        state = self.State.RECORDING
                        if not depth_trigger:
                            depth_trigger = True
                            notify_user('RECORDING STARTED')
                    await stream_queue.get()
                elif state == self.State.RECORDING:
                    if last_rec_start_time is None:
                        last_rec_start_time = self.loop.time()
                        logger.debug("Clip starting time: "
                                     f"{last_rec_start_time}")
                    clip_ending_time = last_rec_start_time + \
                        self.max_clip_length
                    logger.debug(f"Ending at: {clip_ending_time}")
                    clip_timeout = clip_ending_time - self.loop.time()
                    if clip_timeout <= 0:
                        clip_timeout = 0.001
                    try:
                        async with async_timeout.timeout(clip_timeout):
                            logger.debug(f"Clip will end in {clip_timeout} "
                                         f"seconds")
                            while True:
                                observer = await stream_queue.get()
                                elapsed_time \
                                    = self.loop.time() - last_rec_start_time
                                if observer is self.depth_observer and \
                                        elapsed_time >= self.min_clip_length\
                                        and self.depth_observer.depth < \
                                        self.depth_threshold:
                                    logger.info("Depth triggered a stop "
                                                "recording")
                                    if depth_trigger:
                                        depth_trigger = False
                                        notify_user('RECORDING STOPPED')
                                    break
                                elif observer is self.rec_state_observer and\
                                        self.rec_state_observer.state != 'REC':
                                    last_rec_state = \
                                        self.rec_state_observer.state
                                    logger.warning(f"Unexpected recording "
                                                   f"state: {last_rec_state}")
                                    break
                    except asyncio.TimeoutError:
                        logger.info("Max clip time elapsed, stopping...")
                    finally:
                        await self.stop_recording()
                        last_rec_start_time = None
                        state = self.State.NOT_RECORDING
        except asyncio.CancelledError:
            await scheduler.close()

    async def stop_recording(self):
        logger.info("Stopping recording...")
        while self.rec_state_observer.state != 'IDLE':
            logger.debug(f"Recording state is not IDLE yet (it is in state: "
                         f"{self.rec_state_observer.state})")
            if self.rec_state_observer.state == 'REC':
                send(topic_factory.create_topic('/camera/main/rec_cmd/'),
                     dict(cmd='stop'))
            await self.wait_for_rec_state('IDLE',
                                          self.rec_state_change_timeout)

    async def start_recording(self):
        logger.info("Starting recording...")
        while self.rec_state_observer.state != 'REC':
            logger.debug(f"Recording state is not REC yet (it is in state: "
                         f"{self.rec_state_observer.state})")
            if self.rec_state_observer.state == 'IDLE':
                send(topic_factory.create_topic('/camera/main/rec_cmd/'),
                     dict(cmd='start'))
            await self.wait_for_rec_state(
                'REC', self.rec_state_change_timeout)

    @property
    def depth_threshold(self):
        return AutoRecordingDaemon.DEFAULT_DEPTH_THRESHOLD

    @property
    def max_clip_length(self):
        return AutoRecordingDaemon.DEFAULT_MAX_CLIP_LENGTH

    @property
    def min_clip_length(self):
        return AutoRecordingDaemon.DEFAULT_MIN_CLIP_LENGTH

    @property
    def rec_state_change_timeout(self):
        return AutoRecordingDaemon.DEFAULT_REC_STATE_CHANGE_TIMEOUT


def parse_args():
    parser = ArgumentParser(description="Send a command to the ROV")
    parser.add_argument("-v", "--verbosity", action="count",
                        help="Increase output verbosity (2 times for the "
                             "maximum)")
    return parser.parse_args()


if __name__ == '__main__':
    args = parse_args()
    if args.verbosity is not None:
        if args.verbosity == 1:
            get_rov_logger('INFO')
        elif args.verbosity >= 2:
            get_rov_logger('DEBUG')
    settings = AutoRecordingDaemonSettings()
    if not settings.autorec_enabled:
        logger.warning("Auto-recording is disabled")
        quit()
    daemon = AutoRecordingDaemon()
    main_task = asyncio.ensure_future(daemon.main_loop())
    loop = asyncio.get_event_loop()
    try:
        loop.run_forever()
    except BaseException as e:
        logger.exception(e)
    finally:
        main_task.cancel()
        loop.run_until_complete(main_task)
        loop.close()
