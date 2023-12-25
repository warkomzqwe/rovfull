import asyncio
import concurrent.futures
import logging
import pkgutil
import signal
import sys
from time import sleep

from rov_logger.logger import get_rov_logger

from rov_multiserver.mavlink_vehicle_link import MavlinkVehicleLink
from rov_multiserver.messaging_system import MessagingSystem
from rov_multiserver.utils.utils import RealTimeGenerator

DEBUG_LEVEL = 'DEBUG'
logger = get_rov_logger(stream_log_level=DEBUG_LEVEL)
DO_AUTOREC = True


def main():
    logger.debug("Creating vehicle link")
    vehicle_link = MavlinkVehicleLink()
    logger.debug("Creating messaging system")
    msg_system = MessagingSystem()
    # Start everything:
    # 1. MavlinkVehicleLink start() create and enqueue the main coroutines.
    vehicle_link.start()
    # 2. MessagingSystem start() initializes the messaging system threads.
    msg_system.start()
    # 3. Start rtc time delivered over event-bus (useful for date overlay on recorded videos).
    real_time_generator = RealTimeGenerator()
    logger.info(f"Starting real time generator with date {real_time_generator.get_date_str()}")

    loop = asyncio.get_event_loop()
    loop.set_debug(True)
    logging.getLogger("asyncio").setLevel(logging.DEBUG)

    async def run_autorec_daemon_as_child_process():
        if DEBUG_LEVEL == 'INFO':
            debug_arg = '-v'
        elif DEBUG_LEVEL == 'DEBUG':
            debug_arg = '-vv'
        else:
            debug_arg = ''
        interpreter_name = sys.executable
        script_name = pkgutil.get_loader(
            'rov_multiserver.autorec_controller_daemon').get_filename()
        proc = await asyncio.create_subprocess_shell(
            f"{interpreter_name} {script_name} {debug_arg}",
            stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.STDOUT)
        try:
            while True:
                line = await proc.stdout.readline()
                print(line.decode('utf-8'), end='')
                # Check if child process has finished.
                try:
                    await asyncio.wait_for(proc.wait(), timeout=0.001)
                except asyncio.TimeoutError:
                    continue  # if alive, wait() should timeout
                logger.warning("autorec daemon child process has finished")
                break  # just possible whenever wait() returns immediately
        except asyncio.CancelledError:
            proc.send_signal(signal.SIGINT)
            await proc.wait()

    if DO_AUTOREC:
        autorec_daemon_task = asyncio.ensure_future(
            run_autorec_daemon_as_child_process())
    else:
        autorec_daemon_task = None

    asyncio_executor = concurrent.futures.ThreadPoolExecutor(
        thread_name_prefix='AsyncioThread', max_workers=3)
    loop.set_default_executor(asyncio_executor)

    try:
        loop.run_forever()
    except KeyboardInterrupt:
        logger.info("Requested to stop (KeyboardInterrupt)")
        if asyncio.isfuture(autorec_daemon_task):
            autorec_daemon_task.cancel()
            if not autorec_daemon_task.done():
                logger.warning("Waiting until autorec daemon child process "
                               "is finished...")
                loop.run_until_complete(autorec_daemon_task)
    finally:
        asyncio_executor.shutdown(wait=False)
        vehicle_link.stop()
        msg_system.stop()
        while loop.is_running():
            loop.call_soon_threadsafe(loop.stop)
            sleep(0.1)
        loop.close()


if __name__ == '__main__':
    main()
