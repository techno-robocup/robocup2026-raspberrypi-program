import libcamera
import numpy as np
from ultralytics import YOLO

def Linetrace_Camera_precallback_func(request):
  """Linetrace camera pre-callback - imports camera module lazily to avoid circular import."""
  import modules.camera
  modules.camera.Linetrace_Camera_Pre_callback(request)


def Rescue_Camera_precallback_func(request):
  """Rescue camera pre-callback - imports camera module lazily to avoid circular import."""
  import modules.camera
  modules.camera.Rescue_precallback_func(request)


UART_BAUD_RATE = 9600
UART_TIMEOUT = 1
MOTOR_MAX_SPEED = 2000
MOTOR_MIN_SPEED = 1000
Rescue_Camera_Port = 0
Rescue_Camera_Controls = {
    "AfMode": libcamera.controls.AfModeEnum.Continuous,
    "AfSpeed": libcamera.controls.AfSpeedEnum.Fast,
    "AeFlickerMode": libcamera.controls.AeFlickerModeEnum.Manual,
    "AeFlickerPeriod": 10000,
    "AeMeteringMode": libcamera.controls.AeMeteringModeEnum.Matrix,
    "AwbEnable": True,
    "AwbMode": libcamera.controls.AwbModeEnum.Indoor,
    "HdrMode": libcamera.controls.HdrModeEnum.Off
}
Rescue_Camera_Size = (4608, 2592)
Rescue_Camera_Formats = "RGB888"
Rescue_Camera_lores = (Rescue_Camera_Size[0], Rescue_Camera_Size[1])
Rescue_Camera_precallback = Rescue_Camera_precallback_func
MODEL = YOLO("best.pt")
class TargetList(Enum):
  BLACK_BALL = 0
  EXIT = 1
  GREEN_CAGE = 2
  RED_CAGE = 3
  SILVER_BALL = 4

Linetrace_Camera_Port = 1
Linetrace_Camera_Controls = {
    "AfMode": libcamera.controls.AfModeEnum.Manual,
    "LensPosition": 1.0 / 0.03,
    "AeFlickerMode": libcamera.controls.AeFlickerModeEnum.Manual,
    "AeFlickerPeriod": 10000,
    "AeMeteringMode": libcamera.controls.AeMeteringModeEnum.Matrix,
    "AwbEnable": False,
    "AwbMode": libcamera.controls.AwbModeEnum.Indoor,
    "HdrMode": libcamera.controls.HdrModeEnum.Night
}
Linetrace_Camera_Size = (4608, 2592)
Linetrace_Camera_Formats = "RGB888"
Linetrace_Camera_lores = (Linetrace_Camera_Size[0] // 8,
                          Linetrace_Camera_Size[1] // 8)
Linetrace_Camera_precallback = Linetrace_Camera_precallback_func

BLACK_WHITE_THRESHOLD = 80
LINETRACE_CAMERA_LORES_HEIGHT = 180
LINETRACE_CAMERA_LORES_WIDTH = 320
COMPUTING_P = 30

MIN_BLACK_LINE_AREA = 300
MIN_GREEN_AREA = 200
MIN_RED_AREA = 200

lower_green = np.array([20, 130, 90])
upper_green = np.array([100, 255, 255])

lower_red = np.array([160, 70, 110])
upper_red = np.array([179, 255, 255])

if __name__ == "__main__":
  pass
