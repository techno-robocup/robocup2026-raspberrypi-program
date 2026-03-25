import math
import time
from typing import Any, Callable, Dict, Final, List, Optional, Tuple

import cv2
import numpy as np
from numba import jit
from picamera2 import CompletedRequest, MappedArray, Picamera2
from readerwriterlock import rwlock

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
_depth_model_lock = rwlock.RWLockFairD()


def get_depth_model():
  """Get or initialize the Depth-Anything-V2 model (lazy loading with thread safety)."""
  global _depth_model
  if _depth_model is None:
    with _depth_model_lock.gen_wlock():
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
        robot.write_rescue_saved_time(time.time())


green_marks: List[Tuple[int, int, int, int]] = []
green_black_detected: List[np.ndarray] = []
green_contours: List[np.ndarray] = []


class _GreenTurnTracker:
  """Multi-frame state tracker for green-mark turn maneuvers.

  Accumulates branch-verified direction votes across frames.
  Once committed, the direction is locked and cannot change.
  A grace period keeps the last erasure active through brief
  detection drop-outs.

  Lifecycle:
    IDLE  ->  (branch-verified mark detected)  ->  ACTIVE
    ACTIVE -> (marks disappear for GRACE_FRAMES) -> IDLE
  """

  VOTE_THRESHOLD: Final[int] = 3
  GRACE_FRAMES: Final[int] = 10

  def __init__(self):
    self.reset()

  def reset(self):
    self.active = False
    self.votes = {'l': 0, 'r': 0}
    self.committed_dir: Optional[str] = None  # 'l', 'r', 'u'
    self.line_cx: Optional[int] = None
    self.last_center_y: Optional[int] = None
    self.last_mark_h: Optional[int] = None
    self.miss_count = 0

  def vote(self, direction: str):
    """Record a branch-verified vote. Ignored after U-turn commit."""
    if self.committed_dir == 'u':
      return
    self.votes[direction] = self.votes.get(direction, 0) + 1

  def commit_uturn(self):
    """Commit to U-turn immediately."""
    if not self.committed_dir:
      self.committed_dir = 'u'

  def try_commit(self, allow_upgrade: bool = False):
    """Lock direction once enough votes accumulated.

    Allows upgrade from 'l'/'r' to 'u' when the opposite side also
    reaches threshold — but only when allow_upgrade is True (both
    sides have marks in the current frame), preventing a single
    drifting mark from faking a U-turn.
    """
    l, r = self.votes['l'], self.votes['r']
    if l >= self.VOTE_THRESHOLD and r >= self.VOTE_THRESHOLD and (
        not self.committed_dir or allow_upgrade):
      self.committed_dir = 'u'
    elif not self.committed_dir:
      if l >= self.VOTE_THRESHOLD:
        self.committed_dir = 'l'
        self.votes['r'] = 0  # require fresh evidence for upgrade to 'u'
      elif r >= self.VOTE_THRESHOLD:
        self.committed_dir = 'r'
        self.votes['l'] = 0  # require fresh evidence for upgrade to 'u'

  @property
  def effective_dir(self) -> Optional[str]:
    """Committed direction, or best guess from majority vote."""
    if self.committed_dir:
      return self.committed_dir
    l, r = self.votes['l'], self.votes['r']
    if l > r:
      return 'l'
    if r > l:
      return 'r'
    if l > 0:
      return 'u'
    return None


_green_tracker = _GreenTurnTracker()


def reset_green_tracker() -> None:
  """Reset the green turn tracker (called by main.py after a U-turn)."""
  _green_tracker.reset()


red_contours: List[np.ndarray] = []


def detect_green_marks(orig_image: np.ndarray,
                       skeleton_image: np.ndarray) -> None:
  """Detect multiple X-shaped green marks and their relationship with skeleton lines."""
  # global green_marks, green_black_detected, green_contours, robot

  # Convert to HSV (avoid copying if possible)
  hsv = cv2.cvtColor(orig_image, cv2.COLOR_BGR2HSV)

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

      # Check for skeleton lines around the mark
      black_detections = _check_black_lines_around_mark(skeleton_image,
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
    h, w = skeleton_image.shape[:2]
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


@jit(nopython=True, cache=True)
def _white_row_center(row: np.ndarray) -> int:
  """Return mean x-position of white pixels in a binary row.

  Returns -1 when no white pixels are present.
  """
  white_pixels = np.where(row > 0)[0]
  if white_pixels.size == 0:
    return -1
  return int(np.mean(white_pixels))


def _check_black_lines_around_mark(skeleton_image: np.ndarray, center_x: int,
                                   center_y: int, w: int, h: int) -> np.ndarray:
  """Check for skeleton line pixels around a mark in four directions."""
  black_detections = np.zeros(4, dtype=np.int8)  # [bottom, top, left, right]

  # Define ROI sizes relative to mark size
  roi_width = int(w * 0.5)
  roi_height = int(h * 0.5)
  # Minimum number of skeleton pixels to consider a line present
  min_skeleton_pixels = 3

  # Check bottom
  roi_b_y1 = center_y + h // 2
  roi_b_y2 = min(center_y + h // 2 + roi_height,
                 consts.LINETRACE_CAMERA_LORES_HEIGHT)
  roi_b_x1 = center_x - roi_width // 2
  roi_b_x2 = center_x + roi_width // 2
  roi_b = skeleton_image[roi_b_y1:roi_b_y2, roi_b_x1:roi_b_x2]
  if cv2.countNonZero(roi_b) >= min_skeleton_pixels:
    black_detections[0] = 1

  # Check top
  roi_t_y1 = max(center_y - h // 2 - roi_height, 0)
  roi_t_y2 = center_y - h // 2
  roi_t_x1 = center_x - roi_width // 2
  roi_t_x2 = center_x + roi_width // 2
  roi_t = skeleton_image[roi_t_y1:roi_t_y2, roi_t_x1:roi_t_x2]
  if cv2.countNonZero(roi_t) >= min_skeleton_pixels:
    black_detections[1] = 1

  # Check left
  roi_l_y1 = center_y - roi_height // 2
  roi_l_y2 = center_y + roi_height // 2
  roi_l_x1 = max(center_x - w // 2 - roi_width, 0)
  roi_l_x2 = center_x - w // 2
  roi_l = skeleton_image[roi_l_y1:roi_l_y2, roi_l_x1:roi_l_x2]
  if cv2.countNonZero(roi_l) >= min_skeleton_pixels:
    black_detections[2] = 1

  # Check right
  roi_r_y1 = center_y - roi_height // 2
  roi_r_y2 = center_y + roi_height // 2
  roi_r_x1 = center_x + w // 2
  roi_r_x2 = min(center_x + w // 2 + roi_width,
                 consts.LINETRACE_CAMERA_LORES_WIDTH)
  roi_r = skeleton_image[roi_r_y1:roi_r_y2, roi_r_x1:roi_r_x2]
  if cv2.countNonZero(roi_r) >= min_skeleton_pixels:
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


def _find_line_center_below(binary_image: np.ndarray, mark_center_y: int,
                            mark_h: int) -> Optional[int]:
  """Find the x-center of the approaching line below a green mark.

  Scans the binary image at the bottom portion of the image where the
  approach line is clean (not contaminated by intersection branches).

  Returns:
    The x-coordinate of the line center, or None if no line found.
  """
  h, w = binary_image.shape[:2]
  # Scan at fixed positions in the bottom third of the image.
  # This avoids contamination from intersection branches which
  # appear in the upper/middle rows and would bias np.mean.
  for check_y in [h * 3 // 4, h * 7 // 8, h - 5]:
    check_y = min(h - 1, check_y)
    row = binary_image[check_y, :]
    row_center = _white_row_center(row)
    if row_center >= 0:
      return row_center
  return None


def _has_branch_in_direction(skeleton: np.ndarray, center_x: int, center_y: int,
                             mark_w: int, mark_h: int, direction: str) -> bool:
  """Check if a skeleton branch exists in the given direction.

  A mark can only vote for direction D if a real branch exists in
  direction D at the intersection.  This prevents spurious marks
  (e.g. visible through a T-junction) from voting for a non-existent
  branch.

  Searches ABOVE the mark (lower y, toward the intersection) on the
  mark's own outward side.  For a left mark, searches left of the
  mark center; for a right mark, searches right.  This avoids false
  positives from the approach line (which sits between left and right
  marks) even when the line curves.

  Returns True if enough skeleton pixels are found to confirm a branch.
  """
  h, w = skeleton.shape[:2]
  MIN_BRANCH_PIXELS = 8

  y_top = max(0, center_y - mark_h * 5)
  y_bottom = max(0, center_y - mark_h // 2)
  if y_top >= y_bottom:
    return False

  search_extent = max(mark_w * 5, 80)

  if direction == 'l':
    x_left = max(0, center_x - search_extent)
    x_right = center_x
  else:
    x_left = center_x
    x_right = min(w, center_x + search_extent)

  if x_left >= x_right:
    return False

  roi = skeleton[y_top:y_bottom, x_left:x_right]
  return cv2.countNonZero(roi) >= MIN_BRANCH_PIXELS


def _is_mark_past_intersection(clean_skeleton: np.ndarray, mark_cy: int,
                                mark_h: int, ref_cx: int) -> bool:
  """Check if a mark is past the intersection (above the side lines).

  Looks for skeleton pixels BELOW the mark (higher y, closer to robot)
  that are far from the approach line center.  Such pixels indicate
  horizontal side lines, meaning the intersection is between the robot
  and the mark.  Uses clean_skeleton (mark blobs already removed) to
  avoid confusing other marks' blobs for side lines.
  """
  h, w = clean_skeleton.shape[:2]
  MIN_SIDE_PIXELS = 8

  y_start = min(h - 1, mark_cy + mark_h // 2)
  y_end = h
  if y_start >= y_end:
    return False

  side_margin = 30

  # Left side of approach line
  x_left_end = max(0, ref_cx - side_margin)
  if x_left_end > 0:
    left_roi = clean_skeleton[y_start:y_end, 0:x_left_end]
    if cv2.countNonZero(left_roi) >= MIN_SIDE_PIXELS:
      return True

  # Right side of approach line
  x_right_start = min(w, ref_cx + side_margin)
  if x_right_start < w:
    right_roi = clean_skeleton[y_start:y_end, x_right_start:w]
    if cv2.countNonZero(right_roi) >= MIN_SIDE_PIXELS:
      return True

  return False


def _apply_green_turn_to_binary(binary_image: np.ndarray,
                                skeleton: np.ndarray) -> np.ndarray:
  """Modify binary image to guide the robot through a green-mark turn.

  Uses branch-verified voting: a mark can only vote for direction D
  if a skeleton branch actually exists in direction D above the mark.
  This prevents spurious marks (e.g. visible through a T-junction)
  from voting for non-existent branches.

  Returns:
    Modified binary image (copy) or the original if no modification needed.
  """
  tracker = _green_tracker
  has_marks = bool(green_marks) and bool(green_black_detected)
  h, w = binary_image.shape[:2]

  # ------------------------------------------------------------------
  # 1. No marks visible — grace period or reset.
  # ------------------------------------------------------------------
  if not has_marks:
    tracker.miss_count += 1
    if tracker.active and tracker.miss_count <= tracker.GRACE_FRAMES:
      edir = tracker.effective_dir
      if edir is not None and edir != 'u' and tracker.line_cx is not None:
        return _erase_for_turn(binary_image, edir, tracker.line_cx,
                               tracker.last_center_y, tracker.last_mark_h)
    if tracker.miss_count > tracker.GRACE_FRAMES:
      tracker.reset()
      robot.write_green_turn_direction(None)
    return binary_image

  # ------------------------------------------------------------------
  # 2. Marks visible — update tracker position and line reference.
  # ------------------------------------------------------------------
  tracker.miss_count = 0

  closest_mark = max(green_marks, key=lambda m: m[1])
  tracker.last_center_y = closest_mark[1]
  tracker.last_mark_h = closest_mark[3]

  if not tracker.committed_dir:
    lcx = _find_line_center_below(binary_image, closest_mark[1],
                                  closest_mark[3])
    if lcx is not None:
      tracker.line_cx = lcx

  ref_cx = tracker.line_cx if tracker.line_cx is not None else w // 2

  # Create a clean skeleton with green mark blobs removed so that
  # _has_branch_in_direction and _is_mark_past_intersection do not
  # mistake a mark's own blob (green tape → dark in grayscale → white
  # in binary → skeleton pixels) for a real intersection branch.
  clean_skeleton = skeleton.copy()
  mark_margin = 15
  for mark in green_marks:
    mx, my, mw, mh = mark
    x1 = max(0, mx - mw // 2 - mark_margin)
    y1 = max(0, my - mh // 2 - mark_margin)
    x2 = min(w, mx + mw // 2 + mark_margin)
    y2 = min(h, my + mh // 2 + mark_margin)
    clean_skeleton[y1:y2, x1:x2] = 0

  # ------------------------------------------------------------------
  # 3. Classify marks into left / right relative to approach line.
  # ------------------------------------------------------------------
  SIDE_TOLERANCE = 5
  left_marks: list = []
  right_marks: list = []

  for mark, detection in zip(green_marks, green_black_detected):
    if detection[2] != 1 and detection[3] != 1:
      continue  # no adjacent skeleton lines — not actionable
    if _is_mark_past_intersection(clean_skeleton, mark[1], mark[3], ref_cx):
      continue  # mark is above the side lines — ignore
    cx = mark[0]
    if cx < ref_cx - SIDE_TOLERANCE:
      left_marks.append(mark)
    elif cx > ref_cx + SIDE_TOLERANCE:
      right_marks.append(mark)

  # ------------------------------------------------------------------
  # 4. Branch-verified voting.
  #    U-turns are handled naturally: if both sides accumulate enough
  #    verified votes, try_commit() commits 'u'.  No special dead-end
  #    check is needed — it would bypass per-mark branch verification
  #    and cause false U-turns when a spurious mark is present.
  # ------------------------------------------------------------------
  if tracker.committed_dir != 'u':
    for mark in left_marks:
      if _has_branch_in_direction(clean_skeleton, mark[0], mark[1], mark[2],
                                  mark[3], 'l'):
        tracker.vote('l')
        tracker.active = True

    for mark in right_marks:
      if _has_branch_in_direction(clean_skeleton, mark[0], mark[1], mark[2],
                                  mark[3], 'r'):
        tracker.vote('r')
        tracker.active = True

  # ------------------------------------------------------------------
  # 6. Commit and apply.
  # ------------------------------------------------------------------
  if not tracker.active:
    robot.write_green_turn_direction(None)
    return binary_image

  # Only allow upgrade from l/r to u when both sides have marks in the
  # same frame — prevents a single drifting mark from faking a U-turn.
  both_sides_visible = bool(left_marks) and bool(right_marks)
  tracker.try_commit(allow_upgrade=both_sides_visible)

  turn_dir = tracker.effective_dir
  if turn_dir is None or (tracker.line_cx is None and turn_dir != 'u'):
    robot.write_green_turn_direction(None)
    return binary_image

  robot.write_green_turn_direction(turn_dir)

  if turn_dir == 'u':
    return binary_image

  modified = _erase_for_turn(binary_image, turn_dir, tracker.line_cx,
                             tracker.last_center_y, tracker.last_mark_h)

  # Mask out green mark blobs to prevent competing contours.
  margin = 10
  for mark in green_marks:
    mx, my, mw, mh = mark
    x1 = max(0, mx - mw // 2 - margin)
    y1 = max(0, my - mh // 2 - margin)
    x2 = min(w, mx + mw // 2 + margin)
    y2 = min(h, my + mh // 2 + margin)
    modified[y1:y2, x1:x2] = 0

  return modified


def _erase_for_turn(binary_image: np.ndarray, turn_dir: str, line_cx: int,
                    center_y: Optional[int],
                    mark_h: Optional[int]) -> np.ndarray:
  """Erase the unwanted side of an intersection to guide the robot into a turn.

  Two-zone erasure based on center_y (the transition row):
    - Above center_y (intersection area): aggressively erase from the
      approach line toward the unwanted side.  This fully removes the
      straight continuation (which is wide after morphological close)
      while preserving the desired branch on the opposite side.
    - Below center_y (approach area): erase only the far unwanted side,
      keeping the approach line so the robot can smoothly enter the turn.

  The result is an L-shaped path: the desired branch connects to the
  approach line, guiding the contour tracker into the turn.
  """
  h, w = binary_image.shape[:2]
  modified = binary_image.copy()

  if center_y is None:
    center_y = h // 2
  if mark_h is None:
    mark_h = 20

  # Clamp center_y to 40-60% of image height.
  center_y = max(center_y, h * 2 // 5)
  center_y = min(center_y, h * 3 // 5)

  # line_buffer: margin around line_cx to preserve approach line below
  # center_y.  approach_margin: how far past line_cx to erase above
  # center_y to fully cover the wide straight continuation (~60-80px
  # after morphological close).
  line_buffer = 15
  approach_margin = 45

  if turn_dir == 'r':
    # Above center_y: erase from left edge up to line_cx + approach_margin.
    # Removes left branch + straight continuation; keeps right branch.
    modified[0:center_y, 0:min(w, line_cx + approach_margin)] = 0
    # Below center_y: erase far left side only (preserves approach line).
    modified[center_y:h, 0:max(0, line_cx - line_buffer)] = 0
  elif turn_dir == 'l':
    # Above center_y: erase from line_cx - approach_margin to right edge.
    # Removes right branch + straight continuation; keeps left branch.
    modified[0:center_y, max(0, line_cx - approach_margin):w] = 0
    # Below center_y: erase far right side only (preserves approach line).
    modified[center_y:h, min(w, line_cx + line_buffer):w] = 0
  else:  # 'u'
    modified[0:center_y, :] = 0

  return modified


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


LASTBLACKLINE_LOCK = rwlock.RWLockFairD()
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
  hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
  h, s, v = cv2.split(hsv)
  # Use JIT-optimized contrast reduction
  v = _apply_contrast_reduction(v, factor, 128.0)
  hsv = cv2.merge([h, s, v])
  return cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)


def reduce_glare_clahe(image, clip_limit=2.0):
  """Use CLAHE on luminance to balance brightness and reduce glare."""
  lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
  luminance, a_channel, b_channel = cv2.split(lab)
  clahe = cv2.createCLAHE(clipLimit=clip_limit, tileGridSize=(8, 8))
  luminance = clahe.apply(luminance)
  lab = cv2.merge([luminance, a_channel, b_channel])
  return cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)


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


def _skeletonize_line(binary_image: np.ndarray) -> np.ndarray:
  """Skeletonize the binary image to get 1-pixel-wide line."""
  try:
    return cv2.ximgproc.thinning(binary_image,
                                 thinningType=cv2.ximgproc.THINNING_ZHANGSUEN)
  except AttributeError:
    # Fallback: iterative morphological thinning
    skeleton = np.zeros_like(binary_image)
    element = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))
    img = binary_image.copy()
    while True:
      eroded = cv2.erode(img, element)
      opened = cv2.dilate(eroded, element)
      temp = cv2.subtract(img, opened)
      skeleton = cv2.bitwise_or(skeleton, temp)
      img = eroded.copy()
      if cv2.countNonZero(img) == 0:
        break
    return skeleton


def _get_line_direction_and_project(
    skeleton: np.ndarray,
    start_x: int,
    start_y: int,
    projection_length: int = 150
) -> Tuple[Optional[float], List[Tuple[int, int]]]:
  """Get line direction at (start_x, start_y) from the skeleton, then project forward.

  Returns:
      angle: Line angle in radians (None if insufficient skeleton pixels)
      projected_points: List of (x, y) points along the projected path
  """
  h, w = skeleton.shape[:2]

  # Sample skeleton pixels in a neighborhood around the starting point
  neighborhood = 40
  y_min = max(0, start_y - neighborhood)
  y_max = min(h, start_y + neighborhood)
  x_min = max(0, start_x - neighborhood)
  x_max = min(w, start_x + neighborhood)

  region = skeleton[y_min:y_max, x_min:x_max]
  pts = np.column_stack(np.nonzero(region))  # (row, col) pairs

  if len(pts) < 5:
    return None, []

  # Convert to image coordinates
  pts_xy = pts[:, ::-1].astype(np.float64)  # (col, row) = (x, y)
  pts_xy[:, 0] += x_min
  pts_xy[:, 1] += y_min

  # PCA to find line direction
  mean = np.mean(pts_xy, axis=0)
  centered = pts_xy - mean
  cov = np.cov(centered.T)
  eigenvalues, eigenvectors = np.linalg.eigh(cov)
  # Principal component = direction of largest variance
  principal = eigenvectors[:, np.argmax(eigenvalues)]

  # Make sure the direction points upward (toward top of image = ahead on field)
  if principal[1] > 0:
    principal = -principal

  angle = math.atan2(principal[1], principal[0])

  # Project forward from (start_x, start_y)
  projected_points = []
  for dist in range(0, projection_length, 5):
    px = int(start_x + principal[0] * dist)
    py = int(start_y + principal[1] * dist)
    if 0 <= px < w and 0 <= py < h:
      projected_points.append((px, py))
    else:
      break

  return angle, projected_points


def _check_green_along_projection(
    marks: List[Tuple[int, int, int, int]],
    projected_points: List[Tuple[int, int]],
    tolerance: int = 30,
) -> List[Tuple[Tuple[int, int, int, int], float]]:
  """Check if any green marks are near the projected line path.

  Args:
      marks: List of (cx, cy, w, h) detected green marks
      projected_points: Points along the projected line direction
      tolerance: Max pixel distance from path to count as "on path"

  Returns:
      List of (mark, distance) tuples for marks along the projected path.
      distance is the index along the projection (higher = farther ahead).
  """
  if not marks or not projected_points:
    return []

  results = []
  proj_arr = np.array(projected_points, dtype=np.float64)

  for mark in marks:
    cx, cy, mw, mh = mark
    mark_pt = np.array([cx, cy], dtype=np.float64)
    dists = np.linalg.norm(proj_arr - mark_pt, axis=1)
    min_idx = int(np.argmin(dists))
    min_dist = dists[min_idx]
    if min_dist <= tolerance:
      results.append((mark, float(min_idx)))

  # Sort by distance (closest first)
  results.sort(key=lambda x: x[1])
  return results


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
      gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
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
      skeleton = _skeletonize_line(binary_image)
      detect_green_marks(image, skeleton)

      # Modify binary image to show only the desired path at green
      # mark intersections. The normal line-following algorithm will
      # then naturally steer the robot through the turn.
      binary_image = _apply_green_turn_to_binary(binary_image, skeleton)

      if not robot.linetrace_stop and (green_marks or _green_tracker.active):
        cv2.imwrite(f"bin/{current_time:.3f}_linetrace_green_turn.jpg",
                    binary_image)

      contours, _ = cv2.findContours(binary_image, cv2.RETR_TREE,
                                     cv2.CHAIN_APPROX_SIMPLE)

      if not contours:
        if robot is not None:
          robot.write_linetrace_slope(None)
          robot.write_line_width(None)
        return

      best_contour = find_best_contour(contours, w, h, lastblackline)

      if best_contour is None:
        if robot is not None:
          robot.write_linetrace_slope(None)
          robot.write_line_width(None)
        return

      cx, cy = calculate_contour_center(best_contour)

      global line_area
      line_area = cv2.contourArea(best_contour)

      # Compute contour width (bottom edge of minAreaRect)
      rect = cv2.minAreaRect(best_contour)
      box = cv2.boxPoints(rect)
      box = box[box[:, 1].argsort()[::-1]]
      width = abs(box[0][0] - box[1][0])

      with LASTBLACKLINE_LOCK.gen_wlock():
        lastblackline = cx
      if robot is not None:
        robot.write_line_area(line_area)
        robot.write_line_width(width)
        robot.write_line_center_x(cx)
        robot.write_line_center_y(cy)
        robot.write_linetrace_slope(calculate_slope(best_contour, cx, cy, w, h))

      # --- Green mark look-ahead prediction (runs every frame) ---
      skeleton_angle, projected_points = _get_line_direction_and_project(
          skeleton, cx, cy, projection_length=150)

      if robot is not None:
        if skeleton_angle is not None:
          robot.write_line_skeleton_angle(skeleton_angle)
        if green_marks:
          ahead_results = _check_green_along_projection(green_marks,
                                                        projected_points,
                                                        tolerance=30)
          if ahead_results:
            robot.write_green_ahead(True)
            robot.write_green_ahead_distance(ahead_results[0][1])
          else:
            robot.write_green_ahead(False)
            robot.write_green_ahead_distance(0.0)
        else:
          robot.write_green_ahead(False)
          robot.write_green_ahead_distance(0.0)

      debug_image = visualize_tracking(image, best_contour, cx, cy)
      _draw_debug_contours(debug_image)

      # Draw skeleton and projection debug overlay
      if not robot.linetrace_stop:
        # Overlay skeleton as thin cyan line
        skel_coords = np.nonzero(skeleton)
        if len(skel_coords[0]) > 0:
          debug_image[skel_coords[0], skel_coords[1]] = (255, 255, 0)  # cyan
        # Draw projected path as magenta line
        for i in range(len(projected_points) - 1):
          cv2.line(debug_image, projected_points[i], projected_points[i + 1],
                   (255, 0, 255), 2)
        # Highlight green marks along path
        if green_marks:
          ahead_results = _check_green_along_projection(green_marks,
                                                        projected_points,
                                                        tolerance=30)
          for mark, dist in ahead_results:
            mcx, mcy, mw, mh = mark
            cv2.circle(debug_image, (mcx, mcy), 12, (0, 255, 255), 3)
            cv2.putText(debug_image, f"AHEAD:{dist:.0f}", (mcx + 15, mcy),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
        # Show angle text
        if skeleton_angle is not None:
          cv2.putText(debug_image,
                      f"angle:{math.degrees(skeleton_angle):.1f}deg", (10, 20),
                      cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)

      if not robot.linetrace_stop:
        cv2.imwrite(f"bin/{current_time:.3f}_tracking.jpg", debug_image)

  except SystemExit:
    logger.exception("SystemExit caught")
    raise
  except Exception as e:
    logger.exception(f"Error in line tracing: {e}")
