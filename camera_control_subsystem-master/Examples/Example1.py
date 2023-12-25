
from pymessaginglib.MessagingSystem import receiver
from camera_control_subsystem.CameraInterfaceMaster import CameraInterfaceMaster
from camera_control_subsystem.CameraInterfaceClient import CameraInterfaceClient
from time import sleep

# Mandatory start system
receiver.start()

# Interfaces implemented
camMaster = CameraInterfaceMaster()
camClient = CameraInterfaceClient()


# ****************************************************************************
# *                       Client send control commands                       *
# ****************************************************************************

# First we set topics
camera_control_topic = '/rov/camera/control/'
camera_control_ack_topic = '/rov/camera/control/ack/'

camMaster.set_control_topic(camera_control_topic)
camMaster.set_control_ack_topic(camera_control_ack_topic)

camClient.set_control_topic(camera_control_topic)
camClient.set_control_ack_topic(camera_control_ack_topic)


# We register a callback for every action
def tilt_speed_callback(tilt_speed: float):
  # Do something here
  print("Arrival of %0.2f (tilt_speed)." % tilt_speed)


def tilt_step_callback(tilt_step: int):
  # Do something here
  print("Arrival of %i (tilt_step)." % tilt_step)


def pan_speed_callback(pan_speed: float):
  # Do something here
  print("Arrival of %0.2f (pan_speed)." % pan_speed)


def pan_step_callback(pan_step: int):
  # Do something here
  print("Arrival of %i (pan_step)." % pan_step)


# Add the callback
camMaster.tilt_speed_callbacks.add(tilt_speed_callback)
camMaster.tilt_step_callbacks.add(tilt_step_callback)
camMaster.pan_speed_callbacks.add(pan_speed_callback)
camMaster.pan_step_callbacks.add(pan_step_callback)


# Routine to check for ack signal
def wait_for_ack():
  # We wait and ask for an ack
  while True:
    sleep(0.1)
    if camClient.got_ack():
      print('*** ACK back ***\n')
      break
    else:
      print('*** Error (ACK not back) ***\n')


# Send some action
print('Send tilt speed control.')
camClient.send_tilt_speed(0.1)
wait_for_ack()

print('Send pan speed control.')
camClient.send_pan_speed(0.1)
wait_for_ack()

print('Send tilt step control.')
camClient.send_tilt_step(1)
wait_for_ack()

print('Send pan step control.')
camClient.send_pan_step(1)
wait_for_ack()

# Send reset action
print('Sending reset.')
camClient.send_reset_position()
wait_for_ack()


# ****************************************************************************
# *                    GCS send telemetry to tablet client                   *
# ****************************************************************************

# Some polling telemetry examples:


def pretty_print_telemetry(dic):
  print("Telemetry storage:")
  print(dic)


# Client and master must share same topic for telemetry
telemetry_topic = '/system/camera/telemetry'
camMaster.set_telemetry_topic(telemetry_topic)  # publish to this topic
camClient.set_telemetry_topic(telemetry_topic)  # hear this topic

camMaster.send_telemetry(depth=10)
sleep(0.1)  # some time to mqttudp process
telemetry_dic = camClient.get_telemetry()
pretty_print_telemetry(telemetry_dic)

# New telemetry data
camMaster.send_telemetry(temp=21.0, depth=30)
sleep(0.1)
telemetry_dic = camClient.get_telemetry()
pretty_print_telemetry(telemetry_dic)

# Some new sensor
camMaster.send_telemetry(battery_capacity=0.85)
sleep(0.1)
telemetry_dic = camClient.get_telemetry()
pretty_print_telemetry(telemetry_dic)


# ****************************************************************************
# *                            Telemetry callback                            *
# ****************************************************************************


# Telemetry callbacks should have this signature
def some_telemetry_callback(dic: dict):
  # Do something here
  print('Testing callbacks:')
  print(dic)


# register callbacks
camClient.add_telemetry_callback(some_telemetry_callback)

# now master send telemetry
print('\n')
camMaster.send_telemetry(hello='hello')
camMaster.send_telemetry(world='world')
camMaster.send_telemetry(hello='world')
sleep(0.1)





