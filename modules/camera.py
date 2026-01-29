import threading
import time
from typing import Any, Callable, Dict, List, Optional, Tuple

import cv2
import numpy as np
from numba import jit
from picamera2 import CompletedRequest, MappedArray, Picamera2

import modules.constants as consts
import modules.logger
import modules.robot

# Robot reference - set by robot.py after initialization to avoid circular import
robot = None


def set_robot(robot_instance):
  """Set the robot reference. Called by robot.py after Robot initialization."""
  global robot
  robot = robot_instance


logger = modules.logger.get_logger()

# Depth-Anything-V2 model - lazy loaded
_depth_model = None
_depth_model_lock = threading.Lock()

# Flag to track if depth evaluation is currently running
_depth_evaluation_running = False
_depth_evaluation_lock = threading.Lock()


def get_depth_model():
  """Get or initialize the Depth-Anything-V2 model (lazy loading with thread safety)."""
  global _depth_model
  if _depth_model is None:
    with _depth_model_lock:
      if _depth_model is None:  # Double-check locking
        try:
          import os

          import torch
          from depth_anything_v2.dpt import DepthAnythingV2

          logger.info("Loading Depth-Anything-V2 model...")
          model_configs = {
              'vits': {
                  'encoder': 'vits',
                  'features': 64,
                  'out_channels': [48, 96, 192, 384]
              },
              'vitb': {
                  'encoder': 'vitb',
                  'features': 128,
                  'out_channels': [96, 192, 384, 768]
              },
              'vitl': {
                  'encoder': 'vitl',
                  'features': 256,
                  'out_channels': [256, 512, 1024, 1024]
              }
          }

          # Use small model for Raspberry Pi
          encoder = 'vits'
          device = 'cuda' if torch.cuda.is_available() else 'cpu'

          model = DepthAnythingV2(**model_configs[encoder])

          # Load pretrained weights from checkpoints directory
          checkpoint_path = os.environ.get(
              'DEPTH_MODEL_PATH',
              f'checkpoints/depth_anything_v2_{encoder}.pth')

          if os.path.exists(checkpoint_path):
            logger.info(f"Loading weights from {checkpoint_path}")
            state_dict = torch.load(checkpoint_path, map_location='cpu')
            model.load_state_dict(state_dict)
          else:
            logger.warning(
                f"Weights file not found at {checkpoint_path}. "
                "Model initialized with random weights. "
                "Please download weights from https://huggingface.co/depth-anything/Depth-Anything-V2-Small/resolve/main/depth_anything_v2_vits.pth "
                "and place in checkpoints/ directory, or set DEPTH_MODEL_PATH environment variable."
            )

          model = model.to(device)
          model.train(False)  # Set to evaluation mode (equivalent to .eval())
          _depth_model = model

          logger.info(
              f"Depth-Anything-V2 model loaded successfully on {device}")
        except Exception as e:
          logger.exception(f"Failed to load Depth-Anything-V2 model: {e}")
          _depth_model = None
  return _depth_model


@jit(nopython=True, cache=True)
def _normalize_depth_array(depth: np.ndarray) -> np.ndarray:
  """
  JIT-optimized depth normalization.

  Args:
    depth: Raw depth values

  Returns:
    Normalized depth array (0-255)
  """
  depth_min = depth.min()
  depth_max = depth.max()
  normalized = (depth - depth_min) / (depth_max - depth_min + 1e-8)
  return (normalized * 255).astype(np.uint8)


def predict_depth(image: np.ndarray) -> Optional[np.ndarray]:
  """
  Predict depth map from RGB image using Depth-Anything-V2.

  Args:
    image: RGB image (H, W, 3) as numpy array

  Returns:
    Depth map (H, W) as numpy array, or None if prediction fails
  """
  model = get_depth_model()
  if model is None:
    logger.warning("Depth model not available, skipping depth prediction")
    return None

  try:
    import torch

    # Use the model's infer_image method which handles all preprocessing
    with torch.no_grad():
      depth = model.infer_image(image)
    return depth
  except Exception as e:
    logger.exception(f"Depth prediction failed: {e}")
    return None


def colorize_depth(depth: np.ndarray) -> np.ndarray:
  """
  Convert depth map to colorized visualization using min/max normalization.

  Args:
    depth: Depth map (numpy array)

  Returns:
    Colorized depth map (BGR for OpenCV)
  """
  # Use JIT-optimized normalization
  depth_normalized = _normalize_depth_array(depth)
  depth_colored = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_INFERNO)
  return depth_colored


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


def Rescue_Depth_precallback_func(request: CompletedRequest) -> None:
  """
  Rescue camera pre-callback with depth estimation.
  This is called during LINETRACE mode to perform depth estimation.
  """
  # global robot
  # modules.logger.get_logger().info("Rescue Camera pre-callback triggered")
  try:
    with MappedArray(request, "lores") as m:
      image = m.array
      image = cv2.rotate(image, cv2.ROTATE_180)
      current_time = time.time()
      assert isinstance(robot, modules.robot.Robot)
      cv2.imwrite(f"bin/{current_time:.3f}_rescue_origin.jpg", image)
      robot.write_rescue_image(image)
  except SystemExit:
    logger.error("SystemExit caught")
    raise
  except Exception as e:
    logger.error(f"Error in Rescue: {e}")


def Rescue_precallback_func(request: CompletedRequest) -> None:
  """
  Rescue camera pre-callback that switches behavior based on mode.
  - In linetrace mode: Performs depth estimation
  - In rescue mode: Simple image capture for YOLO (no depth processing)
  """
  # global robot

  # Check if robot is initialized
  if robot is None:
    return

  assert isinstance(robot, modules.robot.Robot)

  # Depth estimation only during linetrace mode
  if not robot.is_rescue_flag:
    # In linetrace mode - perform depth estimation
    Rescue_Depth_precallback_func(request)
  else:
    # In rescue mode - just capture image for YOLO detection
    # In rescue mode we only capture the lores image for YOLO.
    # Depth prediction is expensive and should not run in this hot
    # callback path (it was causing the callback to be called very
    # infrequently). Depth prediction can be run elsewhere if needed.
    with MappedArray(request, "lores") as mapped_array:
      image = mapped_array.array
      image = cv2.rotate(image, cv2.ROTATE_180)
      # current_time = time.time()  NOTE: save in find_best_target
      # cv2.imwrite(f"bin/{current_time:.3f}_rescue_origin.jpg", image)
      # image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
      # depth = predict_depth(image_rgb)
      # if depth is not None:
      #   depth_u8 = _normalize_depth_array(depth)
      #   cv2.imwrite(f"bin/{current_time:.3f}_rescue_depth.jpg", depth_u8)
      if robot is not None:
        robot.write_rescue_image(image)


green_marks: List[Tuple[int, int, int, int]] = []
green_black_detected: List[np.ndarray] = []
green_contours: List[np.ndarray] = []

red_contours: List[np.ndarray] = []


def detect_green_marks(orig_image: np.ndarray,
                       blackline_image: np.ndarray) -> None:
  """Detect multiple X-shaped green marks and their relationship with black lines."""
  # global green_marks, green_black_detected, green_contours, robot

  # Convert to HSV (avoid copying if possible)
  hsv = cv2.cvtColor(orig_image, cv2.COLOR_RGB2HSV)

  # Create mask for green color
  green_mask = cv2.inRange(hsv, consts.lower_green, consts.upper_green)

  # Clean up noise with optimized kernel
  kernel = np.ones((3, 3), np.uint8)
  green_mask = cv2.morphologyEx(green_mask,
                                cv2.MORPH_CLOSE,
                                kernel,
                                iterations=2)

  # Save green mask for debugging
  if not robot.linetrace_stop:
    cv2.imwrite(f"bin/{time.time():.3f}_green_mask.jpg", green_mask)

  # Find contours
  green_contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)

  # Reset global variables
  green_marks.clear()
  green_black_detected.clear()

  # Create a copy for debug visualization (don't modify original image)
  debug_image = None
  if green_contours:
    debug_image = orig_image.copy()

  # Process each contour
  for contour in green_contours:
    if cv2.contourArea(contour) > consts.MIN_GREEN_AREA:
      # Get bounding box
      x, y, w, h = cv2.boundingRect(contour)

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

      # Draw on the debug copy, not the original
      if debug_image is not None:
        _draw_green_mark_debug(debug_image, x, y, w, h, center_x, center_y,
                               black_detections)

  # Save debug image if there were green marks
  if green_marks and debug_image is not None:
    # Add checkpoint visualization to green mark debug images
    h, w = blackline_image.shape[:2]
    checkpoint_x = int(w * consts.TURN_CHECKPOINT_X_RATIO)
    checkpoint_y = int(h * consts.TURN_CHECKPOINT_Y_RATIO)
    checkpoint_size = consts.TURN_CHECKPOINT_SIZE
    is_black = robot.top_checkpoint_black if robot is not None else False
    _draw_checkpoint_debug(debug_image, checkpoint_x, checkpoint_y,
                           checkpoint_size, is_black)
    if not robot.linetrace_stop:
      cv2.imwrite(f"bin/{time.time():.3f}_green_marks_with_x.jpg", debug_image)

  # Write to robot instance
  if robot is not None:
    robot.write_green_marks(green_marks)
    robot.write_green_black_detected(green_black_detected)


def detect_red_marks(orig_image: np.ndarray) -> None:
  """Detect red marks and set stop_requested flag."""
  global red_contours  # , robot

  hsv = cv2.cvtColor(orig_image, cv2.COLOR_BGR2HSV)

  red_mask1 = cv2.inRange(hsv, consts.lower_red1, consts.upper_red1)
  red_mask2 = cv2.inRange(hsv, consts.lower_red2, consts.upper_red2)
  red_mask = cv2.bitwise_or(red_mask1, red_mask2)

  kernel = np.ones((3, 3), np.uint8)
  red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel, iterations=3)

  # if not robot.linetrace_stop:
  #   cv2.imwrite(f"bin/{time.time():.3f}_red_mask.jpg", red_mask)

  red_contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL,
                                     cv2.CHAIN_APPROX_SIMPLE)

  h, w, _ = orig_image.shape
  margin_x = int(w * 0.1)
  margin_y = int(h * 0.1)
  left, right = margin_x, w - margin_x
  top, bottom = margin_y, h - margin_y

  red_valid_contours = []
  for contour in red_contours:
    if cv2.contourArea(contour) > consts.MIN_RED_AREA:
      robot.write_linetrace_stop(True)
    if cv2.contourArea(contour) > 20:
      x, y, cw, ch = cv2.boundingRect(contour)
      center_x = x + cw // 2
      center_y = y + ch // 2
      if left <= center_x <= right and top <= center_y <= bottom:
        red_valid_contours.append(contour)
  if red_valid_contours:
    cv2.drawContours(orig_image, red_valid_contours, -1, (0, 0, 255), 2)
    for contour in red_valid_contours:
      x, y, cw, ch = cv2.boundingRect(contour)
      center_x = x + cw // 2
      center_y = y + ch // 2
      cv2.circle(orig_image, (center_x, center_y), 5, (0, 0, 255), -1)
    # if not robot.linetrace_stop:
    #   cv2.imwrite(f"bin/{time.time():.3f}_red_detected.jpg", orig_image)
  # if len(red_valid_contours) >= 3 and robot is not None:
  #   robot.write_linetrace_stop(True)


@jit(nopython=True, cache=True)
def _count_black_pixels(roi: np.ndarray, threshold: int) -> tuple:
  """
  JIT-optimized black pixel counting.

  Args:
    roi: Region of interest
    threshold: Black/white threshold value

  Returns:
    Tuple of (black_pixel_count, total_pixels)
  """
  if roi.size == 0:
    return 0, 0
  black_count = np.sum(roi < threshold)
  return black_count, roi.size


def _check_black_lines_around_mark(blackline_image: np.ndarray, center_x: int,
                                   center_y: int, w: int, h: int) -> np.ndarray:
  """Check for black lines around a mark in four directions."""
  black_detections = np.zeros(4, dtype=np.int8)  # [bottom, top, left, right]

  # Define ROI sizes relative to mark size
  roi_width = int(w * 0.5)
  roi_height = int(h * 0.5)
  black_threshold_ratio = 0.75  # 75% of pixels must be black

  # Check bottom
  roi_b_y1 = center_y + h // 2
  roi_b_y2 = min(center_y + h // 2 + roi_height,
                 consts.LINETRACE_CAMERA_LORES_HEIGHT)
  roi_b_x1 = center_x - roi_width // 2
  roi_b_x2 = center_x + roi_width // 2
  roi_b = blackline_image[roi_b_y1:roi_b_y2, roi_b_x1:roi_b_x2]
  black_count, total = _count_black_pixels(roi_b, consts.BLACK_WHITE_THRESHOLD)
  if total > 0 and black_count / total <= black_threshold_ratio:
    black_detections[0] = 1

  # Check top
  roi_t_y1 = max(center_y - h // 2 - roi_height, 0)
  roi_t_y2 = center_y - h // 2
  roi_t_x1 = center_x - roi_width // 2
  roi_t_x2 = center_x + roi_width // 2
  roi_t = blackline_image[roi_t_y1:roi_t_y2, roi_t_x1:roi_t_x2]
  black_count, total = _count_black_pixels(roi_t, consts.BLACK_WHITE_THRESHOLD)
  if total > 0 and black_count / total <= black_threshold_ratio:
    black_detections[1] = 1

  # Check left
  roi_l_y1 = center_y - roi_height // 2
  roi_l_y2 = center_y + roi_height // 2
  roi_l_x1 = max(center_x - w // 2 - roi_width, 0)
  roi_l_x2 = center_x - w // 2
  roi_l = blackline_image[roi_l_y1:roi_l_y2, roi_l_x1:roi_l_x2]
  black_count, total = _count_black_pixels(roi_l, consts.BLACK_WHITE_THRESHOLD)
  if total > 0 and black_count / total <= black_threshold_ratio:
    black_detections[2] = 1

  # Check right
  roi_r_y1 = center_y - roi_height // 2
  roi_r_y2 = center_y + roi_height // 2
  roi_r_x1 = center_x + w // 2
  roi_r_x2 = min(center_x + w // 2 + roi_width,
                 consts.LINETRACE_CAMERA_LORES_WIDTH)
  roi_r = blackline_image[roi_r_y1:roi_r_y2, roi_r_x1:roi_r_x2]
  black_count, total = _count_black_pixels(roi_r, consts.BLACK_WHITE_THRESHOLD)
  if total > 0 and black_count / total <= black_threshold_ratio:
    black_detections[3] = 1

  return black_detections


def _draw_green_mark_debug(image: np.ndarray, x: int, y: int, w: int, h: int,
                           center_x: int, center_y: int,
                           black_detections: np.ndarray) -> None:
  """Draw debug visualization for green marks."""
  # Draw X mark
  cv2.line(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
  cv2.line(image, (x + w, y), (x, y + h), (0, 255, 0), 2)
  # Draw center point
  cv2.circle(image, (center_x, center_y), 5, (0, 0, 255), -1)
  # Draw black line detection indicators
  if black_detections[0]:
    cv2.line(image, (center_x - 10, center_y + 10),
             (center_x + 10, center_y + 10), (255, 0, 0), 2)
  if black_detections[1]:
    cv2.line(image, (center_x - 10, center_y - 10),
             (center_x + 10, center_y - 10), (255, 0, 0), 2)
  if black_detections[2]:
    cv2.line(image, (center_x - 10, center_y - 10),
             (center_x - 10, center_y + 10), (255, 0, 0), 2)
  if black_detections[3]:
    cv2.line(image, (center_x + 10, center_y - 10),
             (center_x + 10, center_y + 10), (255, 0, 0), 2)


def _draw_checkpoint_debug(image: np.ndarray, checkpoint_x: int,
                           checkpoint_y: int, checkpoint_size: int,
                           is_black: bool) -> None:
  """Draw checkpoint region on debug image."""
  y1 = max(0, checkpoint_y - checkpoint_size // 2)
  y2 = checkpoint_y + checkpoint_size // 2
  x1 = max(0, checkpoint_x - checkpoint_size // 2)
  x2 = checkpoint_x + checkpoint_size // 2
  # Yellow rectangle for checkpoint
  cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 255), 2)
  # Draw center crosshair
  cv2.line(image, (checkpoint_x - 5, checkpoint_y),
           (checkpoint_x + 5, checkpoint_y), (0, 255, 255), 1)
  cv2.line(image, (checkpoint_x, checkpoint_y - 5),
           (checkpoint_x, checkpoint_y + 5), (0, 255, 255), 1)
  # Add text label with status
  status = "BLACK" if is_black else "CLEAR"
  cv2.putText(image, f"CP:{status}", (x1 - 10, y1 - 5),
              cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)


def find_best_contour(contours: List[np.ndarray], camera_x: int, camera_y: int,
                      last_center: int) -> Optional[np.ndarray]:
  """
  Find the best contour to follow from multiple candidates.
  Prioritizes larger lines first, then center, then bottom of the image.
  Also considers line width and continuity to handle intersections.

  Returns the selected contour or None if no suitable contour found.
  """
  if not contours:
    return None

  # Filter contours by minimum area first
  valid_contours = [(i, contour) for i, contour in enumerate(contours)
                    if cv2.contourArea(contour) >= consts.MIN_BLACK_LINE_AREA]

  if not valid_contours:
    return None

  # Process valid contours
  candidates = []
  image_center = camera_x / 2

  for i, contour in valid_contours:
    # Get bounding box
    rect = cv2.minAreaRect(contour)
    box = cv2.boxPoints(rect)
    # Sort points by y-coordinate (descending)
    box = box[box[:, 1].argsort()[::-1]]

    # Calculate line width at bottom
    width = abs(box[0][0] - box[1][0])
    # Calculate center of contour
    center_x = (box[0][0] + box[1][0]) / 2
    distance_from_center = abs(image_center - center_x)

    # Penalize very wide lines (likely intersections)
    if width > 20:
      distance_from_center *= 2

    # Check if contour extends to bottom of image
    is_bottom = box[0][1] >= (camera_y * 0.95)
    # Calculate contour area (larger = higher priority)
    area = cv2.contourArea(contour)

    candidates.append({
        'index': i,
        'contour': contour,
        'x1': int(box[0][0]),
        'y1': int(box[0][1]),
        'x2': int(box[1][0]),
        'y2': int(box[1][1]),
        'width': width,
        'is_bottom': is_bottom,
        'distance_from_center': distance_from_center,
        'area': area
    })

  # Sort candidates by area (larger first), then center, then bottom
  candidates.sort(
      key=lambda x: (-x['area'], x['distance_from_center'], -x['y1']))

  # Return best contour (largest area, then closest to center, then highest y-coordinate)
  return candidates[0]['contour'] if candidates else None


def calculate_contour_center(contour: np.ndarray) -> Tuple[int, int]:
  """Calculate the center point of a contour."""
  M = cv2.moments(contour)
  if M["m00"] != 0:
    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"])
  else:
    # Fallback to bounding box center
    x, y, w, h = cv2.boundingRect(contour)
    cx = x + w // 2
    cy = y + h // 2

  return cx, cy


@jit(nopython=True, cache=True)
def _compute_slope(cx: int, cy: int, base_x: int, base_y: int) -> float:
  """
  JIT-optimized slope calculation.

  Args:
    cx, cy: Top point coordinates
    base_x, base_y: Base point coordinates

  Returns:
    Calculated slope
  """
  dx = cx - base_x
  if abs(dx) > 0.1:  # Avoid division by near-zero
    return (base_y - cy) / dx
  else:
    return 1e9  # Very large value for vertical line


def calculate_slope(contour: np.ndarray, cx: int, cy: int, img_width: int,
                    img_height: int) -> float:
  """Calculate the slope of the line for steering."""
  try:
    # Set base point using actual image dimensions
    base_x = img_width // 2
    base_y = img_height

    # Use JIT-optimized slope calculation
    return _compute_slope(cx, cy, base_x, base_y)
  except Exception as e:
    logger.exception(f"Error in calculate_slope: {e}")
    return 0.0


def visualize_tracking(image: np.ndarray, contour: np.ndarray, cx: int,
                       cy: int) -> np.ndarray:
  """Create a visualization image showing tracking information."""
  vis_image = image.copy()
  cv2.drawContours(vis_image, [contour], 0, (0, 255, 0), 1)
  cv2.circle(vis_image, (cx, cy), 3, (0, 0, 255), -1)
  h, w = vis_image.shape[:2]
  cv2.line(vis_image, (0, h // 2), (w, h // 2), (255, 0, 0), 1)
  cv2.line(vis_image, (cx, 0), (cx, h), (255, 0, 0), 1)
  return vis_image


def _draw_debug_contours(debug_image: np.ndarray) -> None:
  """Draw debug visualization for all detected contours."""
  for contour in red_contours:
    x, y, w, h = cv2.boundingRect(contour)
    cv2.line(debug_image, (x, y), (x + w, y + h), (0, 0, 255), 2)
    cv2.line(debug_image, (x + w, y), (x, y + h), (0, 0, 255), 2)
    cv2.circle(debug_image, (x + w // 2, y + h // 2), 5, (0, 0, 255), -1)
  for contour, black_detection in zip(green_contours, green_black_detected):
    x, y, w, h = cv2.boundingRect(contour)
    cv2.line(debug_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
    cv2.line(debug_image, (x + w, y), (x, y + h), (0, 255, 0), 2)
    cv2.circle(debug_image, (x + w // 2, y + h // 2), 5, (0, 255, 0), -1)
    center_x, center_y = x + w // 2, y + h // 2
    if black_detection[0]:
      cv2.line(debug_image, (center_x, center_y), (center_x, center_y + 10),
               (255, 0, 0), 2)
    if black_detection[1]:
      cv2.line(debug_image, (center_x, center_y), (center_x, center_y - 10),
               (255, 0, 0), 2)
    if black_detection[2]:
      cv2.line(debug_image, (center_x, center_y), (center_x - 10, center_y),
               (255, 0, 0), 2)
    if black_detection[3]:
      cv2.line(debug_image, (center_x, center_y), (center_x + 10, center_y),
               (255, 0, 0), 2)


LASTBLACKLINE_LOCK = threading.Lock()
lastblackline = consts.LINETRACE_CAMERA_LORES_WIDTH // 2
line_area: Optional[float] = None


@jit(nopython=True, cache=True)
def _apply_contrast_reduction(v_channel: np.ndarray,
                              factor: float,
                              mean: float = 128.0) -> np.ndarray:
  """
  JIT-optimized contrast reduction on V channel.

  Args:
    v_channel: V channel values (uint8)
    factor: Contrast factor (< 1 reduces, > 1 increases)
    mean: Center point for contrast adjustment

  Returns:
    Adjusted V channel values
  """
  v_float = v_channel.astype(np.float32)
  v_adjusted = mean + factor * (v_float - mean)
  return np.clip(v_adjusted, 0, 255).astype(np.uint8)


def reduce_contrast_v(image, factor=0.5):
  """Reduce contrast on V channel only (HSV).
  Factor < 1 reduces contrast, factor > 1 increases contrast."""
  hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
  h, s, v = cv2.split(hsv)
  # Use JIT-optimized contrast reduction
  v = _apply_contrast_reduction(v, factor, 128.0)
  hsv = cv2.merge([h, s, v])
  return cv2.cvtColor(hsv, cv2.COLOR_HSV2RGB)


def reduce_glare_clahe(image, clip_limit=2.0):
  """Use CLAHE on luminance to balance brightness and reduce glare."""
  lab = cv2.cvtColor(image, cv2.COLOR_RGB2LAB)
  luminance, a_channel, b_channel = cv2.split(lab)
  clahe = cv2.createCLAHE(clipLimit=clip_limit, tileGridSize=(8, 8))
  luminance = clahe.apply(luminance)
  lab = cv2.merge([luminance, a_channel, b_channel])
  return cv2.cvtColor(lab, cv2.COLOR_LAB2RGB)


def reduce_glare_combined(image, contrast_factor=0.5, clip_limit=2.0):
  """Combine contrast reduction (V channel) and CLAHE for glare reduction."""
  image = reduce_contrast_v(image, contrast_factor)
  image = reduce_glare_clahe(image, clip_limit)
  return image


@jit(nopython=True, cache=True)
def _check_region_is_black(region: np.ndarray,
                           threshold: float = 127.0) -> bool:
  """
  JIT-optimized check if region is predominantly black.

  Args:
    region: Binary image region (white=255, black=0 in inverted binary)
    threshold: Mean value threshold

  Returns:
    True if region is predominantly black (mean > threshold in inverted binary)
  """
  if region.size == 0:
    return False
  return np.mean(region) > threshold


def Linetrace_Camera_Pre_callback(request):
  global lastblackline  # , LASTBLACKLINE_LOCK
  current_time = time.time()
  try:
    with MappedArray(request, "lores") as m:
      image = m.array
      image = cv2.rotate(image, cv2.ROTATE_180)
      h, w = image.shape[:2]
      crop_w = int(w * consts.LINETRACE_CROP_WIDTH_RATIO)
      x_start = (w - crop_w) // 2
      image = image[:, x_start:x_start + crop_w]
      image = cv2.resize(image, (w, h), interpolation=cv2.INTER_LINEAR)
      if not robot.linetrace_stop:
        cv2.imwrite(f"bin/{current_time:.3f}_linetrace_origin.jpg", image)
      image = reduce_glare_combined(image)
      if not robot.linetrace_stop:
        cv2.imwrite(f"bin/{current_time:.3f}_linetrace_format.jpg", image)
      gray_image = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
      _, binary_image = cv2.threshold(gray_image, consts.BLACK_WHITE_THRESHOLD,
                                      255, cv2.THRESH_BINARY_INV)
      kernel = np.ones((10, 10), np.uint8)
      binary_image = cv2.morphologyEx(binary_image,
                                      cv2.MORPH_CLOSE,
                                      kernel,
                                      iterations=5)

      # Check top checkpoint for turn detection
      checkpoint_x = int(w * consts.TURN_CHECKPOINT_X_RATIO)
      checkpoint_y = int(h * consts.TURN_CHECKPOINT_Y_RATIO)
      checkpoint_size = consts.TURN_CHECKPOINT_SIZE
      # Extract checkpoint region
      y1 = max(0, checkpoint_y - checkpoint_size // 2)
      y2 = min(h, checkpoint_y + checkpoint_size // 2)
      x1 = max(0, checkpoint_x - checkpoint_size // 2)
      x2 = min(w, checkpoint_x + checkpoint_size // 2)
      checkpoint_region = binary_image[y1:y2, x1:x2]
      # Check if majority of checkpoint region is black (white in binary due to THRESH_BINARY_INV)
      # Use JIT-optimized region check
      is_black = _check_region_is_black(checkpoint_region, 127.0)
      if robot is not None:
        robot.write_top_checkpoint_black(is_black)

      # Draw checkpoint visualization on binary image for debugging
      if not robot.linetrace_stop:
        # Convert binary to color for visualization
        binary_debug = cv2.cvtColor(binary_image, cv2.COLOR_GRAY2BGR)
        # Draw checkpoint rectangle (Red if black detected, Green if not)
        color = (0, 0, 255) if is_black else (0, 255, 0)
        cv2.rectangle(binary_debug, (x1, y1), (x2, y2), color, 2)
        # Draw center crosshair
        cv2.line(binary_debug, (checkpoint_x - 5, checkpoint_y),
                 (checkpoint_x + 5, checkpoint_y), color, 1)
        cv2.line(binary_debug, (checkpoint_x, checkpoint_y - 5),
                 (checkpoint_x, checkpoint_y + 5), color, 1)
        # Add text label
        cv2.putText(binary_debug, f"CP:{is_black}", (x1, y1 - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
        cv2.imwrite(f"bin/{current_time:.3f}_linetrace_binary.jpg",
                    binary_debug)

      detect_red_marks(image)
      detect_green_marks(image, binary_image)

      contours, _ = cv2.findContours(binary_image, cv2.RETR_TREE,
                                     cv2.CHAIN_APPROX_SIMPLE)

      if not contours:
        if robot is not None:
          robot.write_linetrace_slope(None)
        return

      best_contour = find_best_contour(contours, w, h, lastblackline)

      if best_contour is None:
        if robot is not None:
          robot.write_linetrace_slope(None)
        return

      cx, cy = calculate_contour_center(best_contour)

      global line_area
      line_area = cv2.contourArea(best_contour)

      with LASTBLACKLINE_LOCK:
        lastblackline = cx
      if robot is not None:
        robot.write_line_area(line_area)
        robot.write_line_center_x(cx)
        robot.write_linetrace_slope(calculate_slope(best_contour, cx, cy, w, h))

      debug_image = visualize_tracking(image, best_contour, cx, cy)
      _draw_debug_contours(debug_image)
      if not robot.linetrace_stop:
        cv2.imwrite(f"bin/{current_time:.3f}_tracking.jpg", debug_image)

  except SystemExit:
    logger.exception("SystemExit caught")
    raise
  except Exception as e:
    logger.exception(f"Error in line tracing: {e}")
