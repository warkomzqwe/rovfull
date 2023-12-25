import threading
import time
from os import PathLike
from pathlib import Path
from typing import Union, Optional

import click
from pymessaginglib import set_interface
from pymessaginglib.Topic import Topic, topic_factory
from rov_logger.logger import get_rov_logger

logger = get_rov_logger()
DEFAULT_TOPIC = '/vehicle/hourmeter/'
DEFAULT_OUTPUT_INTERFACE = 'eth0'
DEFAULT_INTERVAL = 15
DEFAULT_FILE = str(Path('~/.rov_multiserver/data/'
                        'hour_meter.data').expanduser())


class HourMeter:
    def __init__(self, interval: Union[int, float] = DEFAULT_INTERVAL,
                 time_file: Union[PathLike, str] = DEFAULT_FILE,
                 topic: str = DEFAULT_TOPIC,
                 output_interface: str = DEFAULT_OUTPUT_INTERFACE,
                 starting_time: Optional[Union[float, int]] = None):
        if starting_time is None:
            starting_time = time.time()
        logger.info(f"Starting hour meter at time: {starting_time}")
        t_file = Path(time_file)
        if not t_file.exists():
            logger.warning(f"No previous time data found ('{time_file}' file "
                           "not found)")
            accumulated_time = 0.0
        else:
            try:
                accumulated_time = float(t_file.read_text())
            except ValueError:
                logger.warning("File exists but its value is invalid")
                accumulated_time = 0.0
        logger.debug(f'Previous accumulated time: {accumulated_time} seconds')
        self._file = t_file
        self._accumulated_time = accumulated_time
        self._interval = interval
        self._starting_time = starting_time
        self._topic: Topic = topic_factory.create_topic(topic)
        try:
            set_interface(output_interface)
        except ValueError:
            logger.warning(f'Invalid interface name: {output_interface}')
        self._should_stop = threading.Event()

    def run(self):
        self._should_stop.clear()
        self.update()
        try:
            while not self._should_stop.wait(self._interval):
                self.update()
        except (KeyboardInterrupt, SystemExit):
            logger.info("User requested to stop..")
            self.stop()
        finally:
            self.update()

    def update(self):
        total_elapsed_time = time.time() - self._starting_time
        total_elapsed_time += self._accumulated_time
        logger.info(f"Total hours elapsed: {total_elapsed_time/3600}")
        logger.debug(f'Writing time: {total_elapsed_time}')
        self._file.write_text(str(total_elapsed_time))
        self._topic.send({'secs': total_elapsed_time})

    def stop(self):
        self._should_stop.set()


@click.command(help="Initializes a daemon that counts the vehicle's active "
                    "time.")
@click.option('--interval', '-i', default=DEFAULT_INTERVAL,
              help='The minimum time interval in seconds to count (defaults '
                   f'to {DEFAULT_INTERVAL})')
@click.option('--time-file', '-f', default=DEFAULT_FILE,
              help="The file to store the time (defaults to '"
                   f"{DEFAULT_FILE}')")
@click.option('--topic', '-t', default=DEFAULT_TOPIC,
              help="The topic to publish hour meter data to (defaults to "
                   f"'{DEFAULT_TOPIC}')")
@click.option('--output-interface', '-o',
              default=DEFAULT_OUTPUT_INTERFACE,
              help="The network interface to publish to (defaults to "
                   f"'{DEFAULT_OUTPUT_INTERFACE}')")
@click.option('-v', '--verbose', count=True,
              help="Increases verbosity. Set multiple times for even more "
                   "verbosity. From log level ERROR with 0 times to log "
                   "level DEBUG with 3 times (-vvv)")
def main(interval, time_file, topic, output_interface, verbose):
    if verbose >= 3:
        get_rov_logger('DEBUG')
    elif verbose == 2:
        get_rov_logger('INFO')
    elif verbose == 1:
        get_rov_logger('WARNING')
    else:
        get_rov_logger('ERROR')
    starting_time = time.time()
    hour_meter = HourMeter(interval, time_file, topic, output_interface,
                           starting_time)
    hour_meter.run()


if __name__ == '__main__':
    main()
