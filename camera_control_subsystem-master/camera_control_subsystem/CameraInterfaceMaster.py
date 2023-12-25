from camera_control_subsystem.CameraControlListener import CameraControlListener
from camera_control_subsystem.CameraTelemetryPublisher import CameraTelemetryPublisher


class CameraInterfaceMaster(CameraControlListener, CameraTelemetryPublisher):

  def __init__(self):
    CameraControlListener.__init__(self)
    CameraTelemetryPublisher.__init__(self)



