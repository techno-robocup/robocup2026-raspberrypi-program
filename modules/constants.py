from enum import Enum

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


UART_BAUD_RATE = 4800
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
IMAGE_SZ = Rescue_Camera_Size[0] * Rescue_Camera_Size[
    1]  # Total pixels in rescue image
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

BLACK_WHITE_THRESHOLD = 100
BALL_CATCH_SIZE = 140000
LINETRACE_CAMERA_LORES_HEIGHT = 180
LINETRACE_CAMERA_LORES_WIDTH = 320
LINETRACE_CROP_WIDTH_RATIO = 0.6

# FROM THIS LINE, DO NOT REWRITE
MIN_BLACK_LINE_AREA = 300  # BE CAREFUL WHEN REWRITING THIS VALUE
# END OF REWRITING PROHIBITION

# Line recovery constants - for backing up when losing the line
LINE_RECOVERY_AREA_THRESHOLD = 3000  # Trigger recovery when line area drops below this
LINE_RECOVERY_ANGLE_THRESHOLD = 0.15  # Trigger when angle error exceeds this (radians, ~28.6°)
LINE_RECOVERY_BACKUP_TIME = 1.0  # Seconds to back up
LINE_RECOVERY_BACKUP_SPEED = 1300  # Motor speed for backing up (< 1500 = reverse)
MIN_GREEN_AREA = 200
MIN_RED_AREA = 9000
MIN_OBJECT_AVOIDANCE_LINE_AREA = 5000

TURN_90_TIME = 1.6
TURN_18_TIME = 0.32

# Green mark intersection turning times (seconds)
GREEN_MARK_APPROACH_TIME = 0.5
GREEN_MARK_TURN_180_TIME = 3.5
GREEN_MARK_Y_THRESHOLD_RATIO = 4 / 5  # Mark must be in bottom fifth

# Maximum timeout for line-based turns (safety fallback)
MAX_TURN_90_TIME = 4.0
MAX_TURN_180_TIME = 7.0
TURN_CHECK_DELAY = 0.5  # Delay before checking for line crossings

# Checkpoint position for turn detection (ratio of image dimensions)
# Point at top-center of the image
TURN_CHECKPOINT_X_RATIO = 0.5  # Center horizontally
TURN_CHECKPOINT_Y_RATIO = 0.3  # Lowered to 30% from top
TURN_CHECKPOINT_SIZE = 10  # Size of area to check (pixels)

RESCUE_FLAG_TIME = 5.0

lower_green = np.array([20, 100, 90])
upper_green = np.array([100, 255, 255])

lower_red1 = np.array([150, 130, 100])
upper_red1 = np.array([179, 255, 255])
lower_red2 = np.array([0, 130, 100])
upper_red2 = np.array([20, 255, 255])

GREEN_GYRO_PASS_TIME = 1.0  # Seconds to pass for checking gyro degrees on green mark turn

if __name__ == "__main__":
  pass
