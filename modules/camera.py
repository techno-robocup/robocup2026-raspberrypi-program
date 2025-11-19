import time
import numpy as np
import cv2
import threading
import modules.logger
import modules.constants
from typing import Callable, Any, Dict, Tuple
from picamera2 import Picamera2, CompletedRequest

logger = modules.logger.get_logger()
const = modules.constants
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


def detect_green_marks(orig_image: np.ndarray,
                       blackline_image: np.ndarray) -> None:
  """Detect multiple X-shaped green marks and their relationship with black lines."""
  global green_marks, green_black_detected, green_contours

  # Convert to HSV (avoid copying if possible)
  hsv = cv2.cvtColor(orig_image, cv2.COLOR_RGB2HSV)

  # Define green color range (very permissive for dark teal-green)
  lower_green = np.array([20, 130, 90])
  upper_green = np.array([100, 255, 255])

  # Create mask for green color
  green_mask = cv2.inRange(hsv, lower_green, upper_green)

  # Clean up noise with optimized kernel
  kernel = np.ones((3, 3), np.uint8)
  green_mask = cv2.morphologyEx(green_mask,
                                cv2.MORPH_CLOSE,
                                kernel,
                                iterations=2)

  # Save green mask for debugging
  cv2.imwrite(f"bin/{time.time():.3f}_green_mask.jpg", green_mask)

  # Find contours
  green_contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)

  # Reset global variables
  green_marks.clear()
  green_black_detected.clear()

  # Process each contour
  for contour in green_contours:
    if cv2.contourArea(contour) > const.MIN_GREEN_AREA:
      # Get bounding box
      x, y, w, h = cv2.boundingRect(contour)
      logger.debug(f"Green mark found at ({x}, {y}) with size ({w}, {h})")

      # Calculate center point
      center_x = x + w // 2
      center_y = y + h // 2

      # Store mark info
      green_marks.append((center_x, center_y, w, h))

      # Check for black lines around the mark
      black_detections = _check_black_lines_around_mark(blackline_image,
                                                        center_x, center_y, w,
                                                        h)
      green_black_detected.append(black_detections)

      _draw_green_mark_debug(orig_image, x, y, w, h, center_x, center_y,
                               black_detections)

  # Save the image with X marks drawn on it
  cv2.imwrite(f"bin/{time.time():.3f}_green_marks_with_x.jpg", orig_image)

def detect_red_marks(orig_image: np.ndarray) -> None:
  """Detect red marks and set stop_requested flag."""
  global stop_requested, red_contours

  hsv = cv2.cvtColor(orig_image, cv2.COLOR_RGB2HSV)

  lower_red = np.array([160, 70, 110])
  upper_red = np.array([179, 255, 255])

  red_mask = cv2.inRange(hsv, lower_red, upper_red)

  kernel = np.ones((3, 3), np.uint8)
  red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel, iterations=3)

  cv2.imwrite(f"bin/{time.time():.3f}_red_mask.jpg", red_mask)

  red_contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL,
                                     cv2.CHAIN_APPROX_SIMPLE)

  h, w, _ = orig_image.shape
  margin_x = int(w * 0.1)
  margin_y = int(h * 0.1)
  left, right = margin_x, w - margin_x
  top, bottom = margin_y, h - margin_y

  count = 0
  red_valid_contours = []
  for contour in red_contours:
    if cv2.contourArea(contour) > 20:
      x, y, cw, ch = cv2.boundingRect(contour)
      center_x = x + cw // 2
      center_y = y + ch // 2
      if left <= center_x <= right and top <= center_y <= bottom:
        red_valid_contours.append(contour)
        count += 1
  if red_valid_contours:
    cv2.drawContours(orig_image, red_valid_contours, -1, (0, 0, 255), 2)
    for contour in red_valid_contours:
      x, y, cw, ch = cv2.boundingRect(contour)
      center_x = x + cw // 2
      center_y = y + ch // 2
      cv2.circle(orig_image, (center_x, center_y), 5, (0, 0, 255), -1)
    cv2.imwrite(f"bin/{time.time():.3f}_red_detected.jpg", orig_image)
  if count >= 3:
    stop_requested = True

def _check_black_lines_around_mark(blackline_image: np.ndarray, center_x: int,
                                   center_y: int, w: int, h: int) -> np.ndarray:
  """Check for black lines around a mark in four directions."""
  black_detections = np.zeros(4, dtype=np.int8)  # [bottom, top, left, right]

  # Define ROI sizes relative to mark size
  roi_width = int(w * 0.5)
  roi_height = int(h * 0.5)
  black_threshold = 0.75  # 75% of pixels must be black

  # Check bottom
  roi_b = blackline_image[center_y +
                          h // 2:min(center_y + h // 2 +
                                     roi_height, const.LINETRACE_CAMERA_LORES_HEIGHT),
                          center_x - roi_width // 2:center_x + roi_width // 2]
  if black_detectionsroi_b.size > 0 and np.sum(roi_b < const.BLACK_WHITE_THRESHOLD) / roi_b.size <= black_threshold:
    black_detections[0] = 1

  # Check top
  roi_t = blackline_image[max(center_y - h // 2 -
                              roi_height, 0):center_y - h // 2,
                          center_x - roi_width // 2:center_x + roi_width // 2]
  if roi_t.size > 0 and np.sum(roi_t < const.BLACK_WHITE_THRESHOLD) / roi_t.size <= black_threshold:
    black_detections[1] = 1

  # Check left
  roi_l = blackline_image[center_y - roi_height // 2:center_y + roi_height // 2,
                          max(center_x - w // 2 - roi_width, 0):center_x -
                          w // 2]
  if roi_l.size > 0 and np.sum(roi_l < const.BLACK_WHITE_THRESHOLD) / roi_l.size <= black_threshold:
    black_detections[2] = 1

  # Check right
  roi_r = blackline_image[center_y - roi_height // 2:center_y + roi_height // 2,
                          center_x +
                          w // 2:min(center_x + w // 2 +
                                     roi_width, LINETRACE_CAMERA_LORES_WIDTH)]
  if roi_r.size > 0 and np.sum(roi_r < const.BLACK_WHITE_THRESHOLD) / roi_r.size <= black_threshold:
    black_detections[3] = 1

  return



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
      _, binary_image = cv2.threshold(gray_image, const.const.BLACK_WHITE_THRESHOLD, 255, cv2.THRESH_BINARY_INV)

      cv2.imwrite(f"bin/{current_time:.3f}_binary.jpg")


  except SystemExit:
    print("SystemExit caught")
    raise
  except Exception as e:
    logger.error(f"Error in line tracing: {e}")
