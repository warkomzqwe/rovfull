"""
TODO update example to match current API (rov_event_bus is deleted)
"""


from time import sleep

from rov_logger.logger import get_rov_logger

from I2C_Communication.Communication import Communication
from absolute_orientation import get_bno055

log = get_rov_logger("INFO")

com = Communication()
sensor = get_bno055(com)

# ****************************************************************************
# *                           Checking sensor state                          *
# ****************************************************************************

if sensor.is_detected:
    log.info('Sensor working.')
else:
    raise RuntimeError('Sensor connectivity problem.')

if sensor.is_calibrated:
    print(' >> Sensor calibrated <<')
else:
    print(' >> Sensor NOT calibrated <<')

# ****************************************************************************
# *                               Main routine                               *
# ****************************************************************************


def print_orientation_event(**kwargs):
    print(kwargs)


def run():

    print("Choose routine: ")
    print("1. Calibration")
    print("2. Current position", )
    print("3. Try events")
    selection = input()

    try:
        selection = int(selection)
    except ValueError:
        run()
        return

# ****************************************************************************
# *                        Available example routines                        *
# ****************************************************************************
    if selection == 1:
        # >> Run calibration routine
        result = sensor.calibration_routine()
        if result:
            print("Calibration successful.")
        else:
            print("Calibration NOT succesful")
            run()
            return

    elif selection == 2:
        # >> Get orientation once
        print(sensor.get_position())

    elif selection == 3:
        # >> Try orientation by events
        event_bus.register_callback(event_name,
                                    print_orientation_event,
                                    priority,
                                    is_sync=False)
        sensor.start_daemon()
        sleep(30)

    else:
        run()
        return


run()
