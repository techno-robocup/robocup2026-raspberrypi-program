import time
import numpy as np
import cv2
import threading
import modules.logger
import modules.constants as consts
from typing import Callable, Any, Dict, Tuple, List, Optional
from picamera2 import Picamera2, CompletedRequest, MappedArray
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


def get_depth_model():
  """Get or initialize the Depth-Anything-V2 model (lazy loading with thread safety)."""
  global _depth_model
  if _depth_model is None:
    with _depth_model_lock:
      if _depth_model is None:  # Double-check locking
        try:
          from depth_anything_v2.dpt import DepthAnythingV2
          import torch
          import os

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
          logger.error(f"Failed to load Depth-Anything-V2 model: {e}")
          _depth_model = None
  return _depth_model


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
    logger.error(f"Depth prediction failed: {e}")
    return None


def colorize_depth(depth: np.ndarray) -> np.ndarray:
  """
  Convert depth map to colorized visualization using min/max normalization.

  Args:
    depth: Depth map (numpy array)

  Returns:
    Colorized depth map (BGR for OpenCV)
  """
  # Normalize using min/max for better depth representation
  depth_normalized = (depth - depth.min()) / (depth.max() - depth.min() + 1e-8)
  depth_normalized = (depth_normalized * 255).astype(np.uint8)
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
  global robot
  logger.debug("Rescue Depth Camera pre-callback triggered (linetrace mode)")
  with MappedArray(request, "lores") as mapped_array:
    image = mapped_array.array
    image = cv2.rotate(image, cv2.ROTATE_180)
    current_time = time.time()
    assert isinstance(robot, modules.robot.Robot)

    # Save original image
    cv2.imwrite(f"bin/{current_time:.3f}_linetrace_rescue_origin.jpg", image)

    # Perform depth estimation
    depth_map = predict_depth(image)

    if depth_map is not None:
      # Colorize depth map
      depth_colored = colorize_depth(depth_map)

      # Save depth maps
      cv2.imwrite(f"bin/{current_time:.3f}_linetrace_depth.jpg", depth_colored)

      # Also save raw normalized depth
      depth_normalized = (depth_map - depth_map.min()) / (
          depth_map.max() - depth_map.min() + 1e-8)
      depth_normalized = (depth_normalized * 255).astype(np.uint8)
      cv2.imwrite(f"bin/{current_time:.3f}_linetrace_depth_raw.jpg",
                  depth_normalized)

      logger.info(
          f"Depth prediction completed (linetrace mode), saved to bin/{current_time:.3f}_linetrace_depth.jpg"
      )
    else:
      logger.warning("Depth prediction returned None")

    # Still save the original image to robot (in case needed)
    if robot is not None:
      robot.write_rescue_image(image)



def Rescue_precallback_func(request: CompletedRequest) -> None:
  """
  Rescue camera pre-callback that switches behavior based on mode.
  - In linetrace mode: Performs depth estimation
  - In rescue mode: Simple image capture for YOLO (no depth processing)
  """
  global robot

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
    logger.debug("Rescue Camera pre-callback triggered (rescue mode)")
    with MappedArray(request, "lores") as mapped_array:
      image = mapped_array.array
      image = cv2.rotate(image, cv2.ROTATE_180)
      current_time = time.time()
      cv2.imwrite(f"bin/{current_time:.3f}_rescue_origin.jpg", image)
      if robot is not None:
        robot.write_rescue_image(image)


green_marks: List[Tuple[int, int, int, int]] = []
green_black_detected: List[np.ndarray] = []
green_contours: List[np.ndarray] = []

red_contours: List[np.ndarray] = []


def detect_green_marks(orig_image: np.ndarray,
                       blackline_image: np.ndarray) -> None:
  """Detect multiple X-shaped green marks and their relationship with black lines."""
  global green_marks, green_black_detected, green_contours, robot

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

  # Process each contour
  for contour in green_contours:
    if cv2.contourArea(contour) > consts.MIN_GREEN_AREA:
      # Get bounding box
      x, y, w, h = cv2.boundingRect(contour)
      # logger.debug(f"Green mark found at ({x}, {y}) with size ({w}, {h})")

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
  if green_marks:
    if not robot.linetrace_stop:
      cv2.imwrite(f"bin/{time.time():.3f}_green_marks_with_x.jpg", orig_image)

  # Write to robot instance
  if robot is not None:
    robot.write_green_marks(green_marks)
    robot.write_green_black_detected(green_black_detected)


def detect_red_marks(orig_image: np.ndarray) -> None:
  """Detect red marks and set stop_requested flag."""
  global red_contours, robot

  hsv = cv2.cvtColor(orig_image, cv2.COLOR_RGB2HSV)

  red_mask = cv2.inRange(hsv, consts.lower_red, consts.upper_red)

  kernel = np.ones((3, 3), np.uint8)
  red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel, iterations=3)

  if not robot.linetrace_stop:
    cv2.imwrite(f"bin/{time.time():.3f}_red_mask.jpg", red_mask)

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
    if not robot.linetrace_stop:
      cv2.imwrite(f"bin/{time.time():.3f}_red_detected.jpg", orig_image)
  if len(red_valid_contours) >= 3 and robot is not None:
    robot.write_linetrace_stop(True)


def _check_black_lines_around_mark(blackline_image: np.ndarray, center_x: int,
                                   center_y: int, w: int, h: int) -> np.ndarray:
  """Check for black lines around a mark in four directions."""
  black_detections = np.zeros(4, dtype=np.int8)  # [bottom, top, left, right]

  # Define ROI sizes relative to mark size
  roi_width = int(w * 0.5)
  roi_height = int(h * 0.5)
  black_threshold = 0.75  # 75% of pixels must be black

  # Check bottom
  roi_b = blackline_image[
      center_y + h // 2:min(center_y + h // 2 +
                            roi_height, consts.LINETRACE_CAMERA_LORES_HEIGHT),
      center_x - roi_width // 2:center_x + roi_width // 2]
  if roi_b.size > 0 and np.sum(
      roi_b < consts.BLACK_WHITE_THRESHOLD) / roi_b.size <= black_threshold:
    black_detections[0] = 1

  # Check top
  roi_t = blackline_image[max(center_y - h // 2 -
                              roi_height, 0):center_y - h // 2,
                          center_x - roi_width // 2:center_x + roi_width // 2]
  if roi_t.size > 0 and np.sum(
      roi_t < consts.BLACK_WHITE_THRESHOLD) / roi_t.size <= black_threshold:
    black_detections[1] = 1

  # Check left
  roi_l = blackline_image[center_y - roi_height // 2:center_y + roi_height // 2,
                          max(center_x - w // 2 - roi_width, 0):center_x -
                          w // 2]
  if roi_l.size > 0 and np.sum(
      roi_l < consts.BLACK_WHITE_THRESHOLD) / roi_l.size <= black_threshold:
    black_detections[2] = 1

  # Check right
  roi_r = blackline_image[center_y - roi_height // 2:center_y + roi_height // 2,
                          center_x + w //
                          2:min(center_x + w // 2 +
                                roi_width, consts.LINETRACE_CAMERA_LORES_WIDTH)]
  if roi_r.size > 0 and np.sum(
      roi_r < consts.BLACK_WHITE_THRESHOLD) / roi_r.size <= black_threshold:
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


def calculate_slope(contour: np.ndarray, cx: int, cy: int, img_width: int,
                    img_height: int) -> float:
  """Calculate the slope of the line for steering."""
  try:
    # Set base point using actual image dimensions
    base_x = img_width // 2
    base_y = img_height

    # Calculate slope between top and center points
    if cx != base_x:  # Avoid division by zero or tiny values
      return (base_y - cy) / (cx - base_x)
    else:
      return 10**9
  except Exception as e:
    logger.error(f"Error in calculate_slope: {e}")
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


def reduce_contrast_v(image, factor=0.5):
  """Reduce contrast on V channel only (HSV).
  Factor < 1 reduces contrast, factor > 1 increases contrast."""
  hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
  h, s, v = cv2.split(hsv)
  mean = 128  # Middle gray
  v = v.astype(np.float32)
  v = mean + factor * (v - mean)
  v = np.clip(v, 0, 255).astype(np.uint8)
  hsv = cv2.merge([h, s, v])
  return cv2.cvtColor(hsv, cv2.COLOR_HSV2RGB)


def reduce_glare_clahe(image, clip_limit=2.0):
  """Use CLAHE on luminance to balance brightness and reduce glare."""
  lab = cv2.cvtColor(image, cv2.COLOR_RGB2LAB)
  l, a, b = cv2.split(lab)
  clahe = cv2.createCLAHE(clipLimit=clip_limit, tileGridSize=(8, 8))
  l = clahe.apply(l)
  lab = cv2.merge([l, a, b])
  return cv2.cvtColor(lab, cv2.COLOR_LAB2RGB)


def reduce_glare_combined(image, contrast_factor=0.5, clip_limit=2.0):
  """Combine contrast reduction (V channel) and CLAHE for glare reduction."""
  image = reduce_contrast_v(image, contrast_factor)
  image = reduce_glare_clahe(image, clip_limit)
  return image


def Linetrace_Camera_Pre_callback(request):
  global lastblackline, LASTBLACKLINE_LOCK
  # logger.debug("Linetrace Camera Pre call-back called")
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
      if not robot.linetrace_stop:
        cv2.imwrite(f"bin/{current_time:.3f}_linetrace_binary.jpg",
                    binary_image)

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
      is_black = np.mean(checkpoint_region) > 127
      if robot is not None:
        robot.write_top_checkpoint_black(is_black)

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
        robot.write_linetrace_slope(calculate_slope(best_contour, cx, cy, w, h))

      debug_image = visualize_tracking(image, best_contour, cx, cy)
      _draw_debug_contours(debug_image)
      if not robot.linetrace_stop:
        cv2.imwrite(f"bin/{current_time:.3f}_tracking.jpg", debug_image)

  except SystemExit:
    logger.error("SystemExit caught")
    raise
  except Exception as e:
    logger.error(f"Error in line tracing: {e}")
