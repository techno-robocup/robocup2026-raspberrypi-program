import time
import numpy as np
import threading
import modules.logger
from typing import Callable, Any, Dict, Tuple
from picamera2 import Picamera2, CompletedRequest

logger = modules.logger.get_logger()


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
    modules.robot.robot.write_rescue_image(image)
