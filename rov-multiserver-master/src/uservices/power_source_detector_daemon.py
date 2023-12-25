import click
from rov_logger.logger import get_rov_logger
from uservices.power_source_detector import SerialPowerSourceDetector, \
    ModbusPowerSourceDetector
from uservices.power_source_detector.modbus_lights_controller import \
    ModbusLightsController

DEFAULT_SERIAL_PORT = "/dev/ttyUSB0"
DEFAULT_BAUDRATE = 19200
DEFAULT_TOPIC = "/vehicle/power_source/"
DEFAULT_OUTPUT_INTERFACE = "eth0"
DEFAULT_TEMP_TOPIC = "/vehicle/temperature/"
DEFAULT_LED_IN_TOPIC = "/vehicle/lights/input/"
DEFAULT_LED_OUT_TOPIC = "/vehicle/lights/state/"
logger = get_rov_logger()


@click.command(help="Initializes a daemon that listens to active power "
                    "source messages by serial port and publishes this data "
                    "to a topic.")
@click.option('--port', '-p', default=DEFAULT_SERIAL_PORT,
              help=f"The serial port to listen to (defaults to "
                   f"'{DEFAULT_SERIAL_PORT}')")
@click.option('--baudrate', '-b', type=int,
              default=DEFAULT_BAUDRATE,
              help="The baudrate to set in the serial port (defaults to "
                   f"'{DEFAULT_BAUDRATE}')")
@click.option('--topic', '-t', default=DEFAULT_TOPIC,
              help="The topic to publish power source to (defaults to "
                   f"'{DEFAULT_TOPIC}')")
@click.option('--temp-topic', '-T', default=DEFAULT_TEMP_TOPIC,
              help="The topic to publish temperature to (defaults to "
                   f"'{DEFAULT_TEMP_TOPIC}')")
@click.option('--output-interface', '-o',
              default=DEFAULT_OUTPUT_INTERFACE,
              help="The network interface to publish to (defaults to "
                   f"'{DEFAULT_OUTPUT_INTERFACE}')")
@click.option('-v', '--verbose', count=True,
              help="Increases verbosity. Set multiple times for even more "
                   "verbosity. From log level ERROR with 0 times to log "
                   "level DEBUG with 3 times (-vvv)")
@click.option('--protocol', type=click.Choice(['Modbus', 'ASCII'],
                                              case_sensitive=False),
              help="The protocol to obtain the power source data (Modbus by "
                   "default)", default='Modbus')
@click.option('--led-ctrl-topic', '-l', default=DEFAULT_LED_IN_TOPIC,
              help="The topic to listen for lights intensity commands ("
                   f"defaults to '{DEFAULT_LED_IN_TOPIC}')")
@click.option('--led-output-topic', '-L', default=DEFAULT_LED_OUT_TOPIC,
              help="The topic to publish current lights intensity (defaults "
                   f"to '{DEFAULT_LED_OUT_TOPIC}')")
def main(port, baudrate, topic, temp_topic, output_interface, verbose,
         protocol, led_ctrl_topic, led_output_topic):
    log_level = ('ERROR', 'WARNING', 'INFO', 'DEBUG')[verbose]
    get_rov_logger(log_level)
    logger.warning(f"Log level set to: {log_level}")
    lights_ctrl = None  # To make the IDE happy
    if protocol == 'Modbus':
        psd = ModbusPowerSourceDetector(port, baudrate, topic,
                                        output_interface, temp_topic)
        lights_ctrl = ModbusLightsController(port, baudrate, led_ctrl_topic,
                                             led_output_topic,
                                             output_interface)
    else:
        psd = SerialPowerSourceDetector(port, baudrate, topic,
                                        output_interface, temp_topic)
    try:
        if lights_ctrl is not None:
            lights_ctrl.publishing_loop_on_thread()
        psd.listen_forever()
    except (KeyboardInterrupt, SystemExit):
        logger.warning("Stop requested by user")
        psd.close()
        if lights_ctrl is not None:
            lights_ctrl.stop_publishing_loop()


if __name__ == '__main__':
    main()
