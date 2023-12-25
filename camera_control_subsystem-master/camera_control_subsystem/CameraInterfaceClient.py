
from camera_control_subsystem.CameraTelemetryListener import CameraTelemetryListener
from camera_control_subsystem.CameraControlPublisher import CameraControlPublisher


class CameraInterfaceClient(CameraControlPublisher, CameraTelemetryListener):

  def __init__(self):
    CameraControlPublisher.__init__(self)
    CameraTelemetryListener.__init__(self)









