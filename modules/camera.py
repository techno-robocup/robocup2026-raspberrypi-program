import time
import numpy as np
import cv2
import threading
import modules.logger
from typing import Callable, Any, Dict, Tuple
from picamera2 import Picamera2, CompletedRequest

logger = modules.logger.get_logger()

BLACK_WHITE_THRESHOLD = 55

class Camera:
  """Camera class for handling Picamera2 operations."""

  def __init__(self, PORT: int, controls: Dict[str, Any], size: Tuple[int, int],
               formats: str, lores_size: Tuple[int, int],
               pre_callback_func: Callable[[Any], Any]):
    self.PORT = PORT
    self.controls = controls
    self.size = size
    self.format = formats
    self.lores_size = lores_size
    self.pre_callback_func = pre_callback_func
    self.cam = Picamera2(self.PORT)
    self.cam.preview_configuration.main.size = self.size
    self.cam.preview_configuration.main.format = self.format
    self.cam.configure(
        self.cam.create_preview_configuration(
            main={
                "size": self.size,
                "format": self.format
            },
            lores={
                "size": self.lores_size,
                "format": self.format
            },
        ))
    self.cam.pre_callback = self.pre_callback_func
    self.cam.set_controls(self.controls)
    self.is_camera_running = False

  def start_cam(self) -> None:
    """Start the camera if not already running."""
    if not self.is_camera_running:
      self.cam.start()
      self.is_camera_running = True

  def stop_cam(self) -> None:
    """Stop the camera if currently running."""
    if self.is_camera_running:
      self.cam.stop()
      self.is_camera_running = False

def Rescue_precallback_func(request: CompletedRequest) -> None:
  modules.logger.get_logger().info("Rescue Camera pre-callback triggered")
  with MappedArray(request, "lores") as mapped_array:
    image = mapped_array.array
    current_time = time.time()
    cv2.imwrite(f"bin/{current_time:.3f}_rescue_origin.jpg")
    modules.robot.robot.write_rescue_image(image)


def apply_center_vignette(img, strength=0.5):#filter function
  h, w = img.shape[:2]
  kernel_x = cv2.getGaussianKernel(w, w * strength)
  kernel_y = cv2.getGaussianKernel(h, h * strength)
  mask = kernel_y @ kernel_x.T
  mask = mask / mask.max()
  vignette = np.zeros_like(img)
  for i in range(3):  # RGB
      vignette[:,:,i] = img[:,:,i] * mask
  return vignette

def Linetrace_Camera_Pre_callback(request):
  logger.debug("Linetrace Camera Pre call-back called")
  current_time = time.time()
  try:
    with MappedArray(request, "lores") as m:
      image = m.array
      h,w = image.shape[:2]
      crop_w = int(w * 0.9)
      x_start = (w - crop_w) // 2
      image = image[:, x_start:x_start + crop_w]
      image = cv2.resize(image,(w,h), interpolation=cv2.INTER_LINEAR)
      cv2.imwrite(f"bin/{current_time:.3f}_linetrace_origin.jpg", image)
      image = apply_center_vignette(image)
      cv2.imwrite(f"bin/{current_time:.3f}_linetrace_format.jpg",image)
      gray_image = cv2.cvtColor(image,cv2.COLOR_RGB2GRAY)
      _, binary_image = cv2.threshold(gray_image, BLACK_WHITE_THRESHOLD, 255, cv2.THRESH_BINARY_INV)

      cv2.imwrite(f"bin/{current_time:.3f}_binary.jpg")
      

  except SystemExit:
    print("SystemExit caught")
    raise
  except Exception as e:
    logger.error(f"Error in line tracing: {e}")
