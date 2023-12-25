import asyncio
import logging

from pymessaginglib.Topic import topic_factory, Topic
from rov_logger.logger import get_rov_logger
from rov_multiserver.input_handler import InputHandler
from rov_multiserver.mavlink_vehicle_link import MavlinkVehicleLink
from rov_multiserver.messaging_system import MessagingSystem
from rov_multiserver.vehicle_settings import VehicleSettingsSubject
logger = get_rov_logger()


def main():
    get_rov_logger('INFO')
    VehicleSettingsSubject(
        config_file_name="/tmp/rov_multiserver/config/vehicle.yaml")
    mavlink_vehicle = MavlinkVehicleLink()
    mavlink_vehicle.start()
    msg_system = MessagingSystem()
    msg_system.start()
    input_handler = InputHandler()
    input_handler.start()
    loop = asyncio.get_event_loop()
    loop.set_debug(True)
    logging.getLogger("asyncio").setLevel(logging.DEBUG)

    async def update_value_by_messaginglib_with_delay(delay, value):
        logger.info(f"Value will be updated in {delay} seconds...")
        await asyncio.sleep(delay)
        topic: Topic = topic_factory.create_topic('/client/vehicle_settings/')
        logger.info(f"Updating to {value}")
        topic.send({'yaw_fixed_power': value})

    async def move_yaw(duration):
        elapsed_time = 0.0
        initial_time = asyncio.get_event_loop().time()
        js_topic: Topic = topic_factory.create_topic('/client/joystick/')
        while elapsed_time < duration:
            logger.info("Trying with 100%")
            js_topic.send({'axis_event': ['LXA;1.0']})
            await asyncio.sleep(0.25)
            js_topic.send({'axis_event': ['LXA;-1.0']})
            await asyncio.sleep(0.75)
            logger.info("Trying with 75%")
            js_topic.send({'axis_event': ['LXA;0.75']})
            await asyncio.sleep(0.25)
            js_topic.send({'axis_event': ['LXA;-0.75']})
            await asyncio.sleep(0.75)
            logger.info("Trying with 50%")
            js_topic.send({'axis_event': ['LXA;0.5']})
            await asyncio.sleep(0.25)
            js_topic.send({'axis_event': ['LXA;-0.5']})
            await asyncio.sleep(0.75)
            elapsed_time = asyncio.get_event_loop().time() - initial_time

    asyncio.ensure_future(update_value_by_messaginglib_with_delay(3, 0.3))
    asyncio.ensure_future(update_value_by_messaginglib_with_delay(6, 0.75))
    asyncio.ensure_future(update_value_by_messaginglib_with_delay(9, 0.0))
    try:
        loop.run_until_complete(move_yaw(15))
    except KeyboardInterrupt:
        pass
    finally:
        msg_system.stop()
        input_handler.stop()


if __name__ == '__main__':
    main()
