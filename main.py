import math
import signal
import sys
import threading
import time
from typing import Optional

import cv2

import modules.constants as consts
import modules.logger
import modules.robot

logger = modules.logger.get_logger()

logger.info("Logger initialized")

# Mutex lock for thread-safe YOLO evaluation
yolo_lock = threading.Lock()

robot = modules.robot.robot
uart_dev = modules.robot.uart_io()
uart_devices = uart_dev.list_ports()

# Prioritize USB devices (ESP32 typically appears as /dev/ttyUSB* or /dev/ttyACM*)
usb_devices = [
    d for d in uart_devices if 'USB' in d.device or 'ACM' in d.device
]
if usb_devices:
  selected_device = usb_devices[0]
elif uart_devices:
  selected_device = uart_devices[0]
else:
  logger.error("No UART devices found")
  sys.exit(1)

logger.info(f"Connecting to UART device: {selected_device.device}")
uart_dev.connect(selected_device.device, consts.UART_BAUD_RATE,
                 consts.UART_TIMEOUT)
robot.set_uart_device(uart_dev)

BASE_SPEED = 1680
TURNING_BASE_SPEED = 1600
assert 1500 < BASE_SPEED < 2000
assert 1500 < TURNING_BASE_SPEED < 2000
# assert TURNING_BASE_SPEED < BASE_SPEED
MAX_SPEED = 2000
MIN_SPEED = 1000
KP = 245
DP = 200
BOP = 0.045  # Ball Offset P
BSP = 1.3  # Ball Size P
COP = 0.06  # Cage Offset P
CSP = 1.5
EOP = 0.03  # Exit Offset P
ESP = 2  # Exit Size P

catch_failed_cnt = 0

# Gap recovery state - timestamp of last recovery to prevent immediate re-trigger
last_gap_recovery_time: float = 0.0
GAP_RECOVERY_COOLDOWN = 0.5  # Seconds to wait after recovery before allowing another

RESCUE_IMAGE_WIDTH = 4608
RESCUE_IMAGE_HEIGHT = 2592
RESCUE_CX = RESCUE_IMAGE_WIDTH / 2.0

BALL_Y_2_3 = (RESCUE_IMAGE_HEIGHT * 2 / 3) - 120  # 1728.0 - x
BALL_Y_5_6 = (RESCUE_IMAGE_HEIGHT * 5 / 6) - 200  # 2160.0 - x


def is_valid_number(value) -> bool:
  """Check if value is a valid finite number (int or float, not bool).

  Args:
    value: The value to check.

  Returns:
    True if value is a finite int or float (excluding bool).
  """
  return isinstance(
      value,
      (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def clamp(value: int,
          min_val: int = MIN_SPEED,
          max_val: int = MAX_SPEED) -> int:
  """Clamp value between min and max.

  Args:
    value: The value to clamp.
    min_val: Minimum allowed value (default: MIN_SPEED).
    max_val: Maximum allowed value (default: MAX_SPEED).

  Returns:
    The clamped value within [min_val, max_val].
  """
  return max(min_val, min(max_val, value))


def normalize_rotation_angle(angle: float) -> float:
  """Normalize rotation angle to handle opposite-direction turns.

  When the robot turns slightly in the opposite direction of intended,
  the angle calculation can wrap around (e.g., 359.5° instead of -0.5°).
  This function converts angles >180° to their negative equivalent.

  Args:
    angle: Rotation angle in degrees (0-360 range from modulo calculation).

  Returns:
    Normalized angle in degrees (-180 to 180 range).
  """
  if angle > 180.0:
    return angle - 360.0
  return angle


def should_process_green_mark() -> bool:
  """
  Determine if we should process green marks for intersection turning.

  Returns True if:
  - Green marks are detected with black lines (not just top/bottom)
  - At least one mark is in the bottom 1/3 of the image
  - Either left or right black line is detected (or both)
  """
  green_marks = robot.green_marks
  green_black_detected = robot.green_black_detected

  if not green_black_detected:
    return False

  # Check if any marks have left or right black lines
  # Format: black_detections[0]=bottom, [1]=top, [2]=left, [3]=right
  has_left = False
  has_right = False

  for detection in green_black_detected:
    # Skip if only top/bottom detected (likely not an intersection)
    if detection[0] == 1:  # Has bottom line
      continue
    if detection[1] == 0:  # No top line
      continue

    # Check for left/right black lines
    if detection[2] == 1:  # Has left line
      has_left = True
    if detection[3] == 1:  # Has right line
      has_right = True

  # Check if any mark is in bottom portion of image
  mark_in_bottom = False
  for mark in green_marks:
    _, y, _, _ = mark  # (center_x, center_y, w, h)
    if y > consts.LINETRACE_CAMERA_LORES_HEIGHT * consts.GREEN_MARK_Y_THRESHOLD_RATIO:
      mark_in_bottom = True
      break

  return (has_left or has_right) and mark_in_bottom


def execute_green_mark_turn() -> bool:
  """
  Execute a turn based on detected green marks using gyro verification.

  Turn logic:
  - Both left and right: 180° turn
  - Only left: 90° right turn
  - Only right: 90° left turn

  Use gyro to verify the turn was completed correctly

  Gyro verification:
  - Record initial yaw before turn
  - After turn completes (black line detected), log the gyro rotation
  - This allows verification that the correct angle was achieved

  Returns:
  - True if turn completed successfully
  - False if interrupted by button
  """
  logger.info("Executing green mark turn using visual feedback with gyro verification")

  green_black_detected = robot.green_black_detected

  # Determine which directions have black lines
  has_left = False
  has_right = False

  for detection in green_black_detected:
    # Skip marks with only top/bottom lines
    if detection[0] == 1:
      continue
    if detection[1] == 0:
      continue

    if detection[2] == 1:  # Left black line
      has_left = True
    if detection[3] == 1:  # Right black line
      has_right = True

  # Determine turn direction and target rotation
  if has_left and has_right:
    target_rotation = 180.0
    turn_direction = "left"  # Turn left for 180° (arbitrary choice)
    turn_description = "180°"
  elif has_left:
    target_rotation = 90.0
    turn_direction = "right"
    turn_description = "90° right"
  elif has_right:
    target_rotation = 90.0
    turn_direction = "left"
    turn_description = "90° left"
  else:
    logger.warning("No valid turn direction detected, aborting turn")
    return False

  logger.info(f"Starting {turn_description} turn ({turn_direction})")

  # First, drive forward slightly to clear the intersection marker
  current_gyro_degrees = (math.degrees(
      math.acos(math.cos(robot.roll) * math.cos(robot.pitch))) if
                     robot.roll is not None and robot.pitch is not None else 0)
  start_time = time.time()
  while time.time() - start_time < consts.GREEN_MARK_APPROACH_TIME * (1 if current_gyro_degrees < 10 else 1.5):
    robot.update_button_stat()
    if robot.robot_stop:
      robot.set_speed(1500, 1500)
      robot.send_speed()
      return False
    robot.set_speed(BASE_SPEED, BASE_SPEED)
    robot.send_speed()

  # Record initial yaw before turn for verification
  robot.update_gyro_stat()
  initial_yaw = robot.yaw
  if initial_yaw is None:
    logger.warning("Gyro yaw unavailable at start, will proceed without gyro verification")

  # Turning parameters
  turning_base_speed = TURNING_BASE_SPEED
  turning_speed_delta = turning_base_speed - 1500  # How much above neutral the turning speed is
  max_turn_time = consts.MAX_TURN_90_TIME if target_rotation == 90.0 else consts.MAX_TURN_180_TIME
  started_turning = time.time()
  black_check_enabled = False

  while True:
    robot.update_button_stat()
    if robot.robot_stop:
      robot.set_speed(1500, 1500)
      robot.send_speed()
      return False

    # Safety timeout
    if time.time() - started_turning > max_turn_time:
      logger.warning(f"Turn timeout after {max_turn_time:.1f}s, stopping turn")
      break

    # Update gyro for monitoring
    robot.update_gyro_stat()
    current_yaw = robot.yaw

    # Calculate rotation for logging (if gyro available)
    yaw_diff = 0.0
    if initial_yaw is not None and current_yaw is not None:
      if turn_direction == "left":
        yaw_diff = (initial_yaw - current_yaw + 360) % 360
      else:
        yaw_diff = (current_yaw - initial_yaw + 360) % 360

      # Normalize angle to handle opposite-direction turns
      yaw_diff = normalize_rotation_angle(yaw_diff)

      # Calculate percentage of target rotation completed
      rotation_percentage = (yaw_diff / target_rotation) * 100.0

      # Enable black check mode when within ±20% of target (80-120% range)
      if rotation_percentage >= 66.0 and not black_check_enabled:
        black_check_enabled = True
        logger.info(f"Black check mode enabled at {rotation_percentage:.1f}% of target rotation (gyro: {yaw_diff:.1f}°)")
    else:
      # Without gyro, enable black check after a minimum time
      if time.time() - started_turning > consts.GREEN_GYRO_PASS_TIME and not black_check_enabled:
        black_check_enabled = True
        logger.info("Black check mode enabled (no gyro, time-based)")
      rotation_percentage = 0.0

    # Check if we should stop based on black detection
    if black_check_enabled:
      if robot.top_checkpoint_black:
        logger.info(f"Black detected at top - stopping turn (gyro rotation: {yaw_diff:.1f}°, {rotation_percentage:.1f}%)")
        break
      # Also check for over-rotation if gyro is available
      if initial_yaw is not None and current_yaw is not None and rotation_percentage > 120.0:
        logger.info(f"Exceeded 120% of target rotation - stopping turn (gyro: {yaw_diff:.1f}°)")
        break
    else:
      # Safety check for significant over-rotation even before black check is enabled
      if initial_yaw is not None and current_yaw is not None and rotation_percentage > 150.0:
        logger.warning(f"Significant over-rotation ({rotation_percentage:.1f}%) before black check - stopping turn (gyro: {yaw_diff:.1f}°)")
        break

    # Set fixed turning speeds
    if turn_direction == "left":
      motor_left = 3000 - turning_base_speed
      motor_right = turning_base_speed
    else:
      motor_left = turning_base_speed
      motor_right = 3000 - turning_base_speed

    robot.set_speed(motor_left, motor_right)
    robot.send_speed()

  # Stop after turn
  robot.set_speed(1500, 1500)
  robot.send_speed()

  # Log gyro verification
  robot.update_gyro_stat()
  final_yaw = robot.yaw
  if initial_yaw is not None and final_yaw is not None:
    if turn_direction == "left":
      total_rotation = (final_yaw - initial_yaw + 360) % 360
    else:
      total_rotation = (initial_yaw - final_yaw + 360) % 360

    # Normalize angle to handle opposite-direction turns
    total_rotation = normalize_rotation_angle(total_rotation)

    rotation_error = abs(total_rotation - target_rotation)
    logger.info(f"Gyro verification: rotated {total_rotation:.1f}° (target: {target_rotation:.1f}°, error: {rotation_error:.1f}°)")
  else:
    logger.info("Gyro verification unavailable (yaw data missing)")

  robot.write_last_slope_get_time(time.time())
  return True  # Completed successfully


def should_execute_line_recovery(line_area: Optional[float],
                                 angle_error: Optional[float]) -> bool:
  """
  Check if line recovery should be executed.

  Recovery triggers when:
  1. Line area is below threshold (robot losing sight of line / gap)
  2. Line center x is too far from image center (robot veering off)
  3. Always respects cooldown to prevent rapid repeated recoveries

  Args:
    line_area: Current detected line area in pixels
    angle_error: Current angle error from vertical (radians), can be None

  Returns:
    True if recovery should be executed
  """
  if line_area is None or not is_valid_number(line_area):
    return False

  # Check cooldown to prevent rapid re-triggering during recovery
  if time.time() - last_gap_recovery_time < GAP_RECOVERY_COOLDOWN:
    return False

  # Get line center x-coordinate and check offset from image center
  line_center_x = robot.line_center_x
  image_center_x = consts.LINETRACE_CAMERA_LORES_WIDTH // 2
  x_offset = abs(line_center_x -
                 image_center_x) if line_center_x is not None else 0

  # Check if x-offset is significant
  x_offset_significant = x_offset > consts.LINETRACE_CAMERA_LORES_WIDTH * 0.1

  area_condition = line_area < consts.LINE_RECOVERY_AREA_THRESHOLD
  x_offset_condition = x_offset_significant

  # Only trigger when area is small AND x-offset is significant
  should_recover = area_condition and x_offset_condition

  if should_recover:
    logger.info(
        f"Line recovery triggered: Low area ({line_area:.1f} < {consts.LINE_RECOVERY_AREA_THRESHOLD}) AND large x-offset ({x_offset:.1f}px, center at {line_center_x})"
    )

  return should_recover


def execute_line_recovery() -> bool:
  """
  Execute line recovery by backing up to regain line visibility.

  When the robot loses sight of the line (gap or veering off), this function
  backs up until the line is visible again.

  Returns:
    True if recovery completed successfully
    False if interrupted by button
  """
  global last_gap_recovery_time

  logger.info("Executing line recovery - backing up")
  last_gap_recovery_time = time.time()  # Set cooldown start

  start_time = time.time()
  while robot.line_area is None or robot.line_area <= consts.LINE_RECOVERY_AREA_THRESHOLD * 4:
    robot.update_button_stat()
    if robot.robot_stop:
      robot.set_speed(1500, 1500)
      robot.send_speed()
      return False

    # Back up with both motors at the same speed
    robot.set_speed(consts.LINE_RECOVERY_BACKUP_SPEED,
                    consts.LINE_RECOVERY_BACKUP_SPEED)
    robot.send_speed()

  # Stop after backup
  robot.set_speed(1500, 1500)
  robot.send_speed()

  logger.info(f"Line recovery completed in {time.time() - start_time:.2f}s")
  robot.write_last_slope_get_time(time.time())
  return True


def get_current_angle_error() -> Optional[float]:
  """
  Calculate the current angle error from robot's line slope.

  Returns:
    Angle error in radians, or None if slope is unavailable
  """
  slope = robot.linetrace_slope
  if slope is None or not is_valid_number(slope):
    return None

  angle = math.atan(slope)
  if angle < 0:
    angle += math.pi

  return angle - (math.pi / 2)


def calculate_motor_speeds(slope: Optional[float] = None) -> tuple[int, int]:
  """
  Calculate left and right motor speeds based on line slope and area.

  Uses arctan to convert slope to angle, then calculates the difference
  from π/2 (vertical). This gives a normalized angular error for steering.
  Also reduces speed when the black line gets smaller for better control.
  Reduces speed when gyro angle error is large for stability.

  Args:
    slope: Line slope value. If None, reads from robot.read_linetrace_slope().

  Angle interpretation:
  - angle = π/2: line is vertical (centered), go straight
  - angle < π/2: line tilts right, turn right
  - angle > π/2: line tilts left, turn left
  """
  if slope is None:  # When the were no args
    slope = robot.linetrace_slope
  if slope is None:  # When the robot could not find an appropriate slope
    if time.time() - robot.last_slope_get_time > consts.RESCUE_FLAG_TIME:
      robot.write_is_rescue_flag(True)
      return 1500, 1500
    return BASE_SPEED, BASE_SPEED
  robot.write_last_slope_get_time(time.time())

  assert is_valid_number(slope), str(slope)
  angle = math.atan(slope)
  if angle < 0:
    angle += math.pi

  angle_error = angle - (math.pi / 2)

  steering = int(KP * angle_error)

  # Calculate speed adjustment based on line area
  line_area = robot.line_area
  speed_multiplier = 1.0  # Default: full speed

  if line_area is not None and is_valid_number(line_area):
    # Reduce speed when line gets smaller
    # Area thresholds:
    # > 3000: full speed (100%)
    # 300-3000: power curve for realistic gradual ramp-up
    # < 300: clamped to minimum (30%)
    if line_area < 3000:
      # Power function (quadratic) for realistic response curve
      # Normalized to 0-1 range, then apply pow(x,2) for aggressive acceleration
      normalized = (line_area - consts.MIN_BLACK_LINE_AREA) / (
          3000 - consts.MIN_BLACK_LINE_AREA)
      power_curve = normalized**2  # Quadratic gives aggressive ramp
      # Scale to 0.3-1.0 range
      speed_multiplier = 0.3 + power_curve * 0.7
      speed_multiplier = max(0.3, min(1.0, speed_multiplier))
      logger.info(
          f"Line area: {line_area:.0f}, speed multiplier: {speed_multiplier:.2f}"
      )

  # Get gyro roll angle and reduce speed when tilted significantly
  gyro_roll = math.radians(robot.roll) if robot.roll is not None else None
  gyro_pitch = math.radians(robot.pitch) if robot.pitch is not None else None
  gyro_calculated = (math.degrees(
      math.acos(math.cos(gyro_roll) * math.cos(gyro_pitch))) if
                     gyro_roll is not None and gyro_pitch is not None else None)
  gyro_multiplier = 1.0 if gyro_calculated is None or gyro_calculated < 15 else 0.5

  # Apply speed multiplier only to the increment above 1500 (stop position)
  # 1500 = stop, so we only reduce the forward speed component
  adjusted_base_speed = 1500 + int(
      (BASE_SPEED - 1500) * speed_multiplier * gyro_multiplier)

  # logger.info(f"Current adjusted speed: {clamp(int(adjusted_base_speed - abs(angle_error)**6 * DP), 1500, 2000)}")
  motor_l = clamp(
      clamp(int(adjusted_base_speed - abs(angle_error)**6 * DP), 1500, 2000) -
      steering * gyro_multiplier, MIN_SPEED, MAX_SPEED)
  motor_r = clamp(
      clamp(int(adjusted_base_speed - abs(angle_error)**6 * DP), 1500, 2000) +
      steering * gyro_multiplier, MIN_SPEED, MAX_SPEED)

  return motor_l, motor_r


def signal_handler(sig, frame):
  """Handle SIGINT (Ctrl+C) for graceful shutdown.

  Stops the robot motors and exits the program cleanly.

  Args:
    sig: Signal number received.
    frame: Current stack frame (unused).
  """
  logger.info("Received shutdown signal")
  robot.set_speed(1500, 1500)
  robot.send_speed()
  sys.exit(0)


def sleep_sec(sec: float, function=None) -> int:
  """Sleep for the specified duration while monitoring robot stop button.

  Continuously sends motor speed commands during sleep and can execute
  an optional callback function each iteration.

  Args:
    sec: Duration to sleep in seconds.
    function: Optional callback function to execute each iteration.

  Returns:
    1 if interrupted by robot stop button, 0 if completed normally.
  """
  prev_time = time.time()
  while time.time() - prev_time < sec:
    robot.update_button_stat()
    if robot.robot_stop:
      robot.set_speed(1500, 1500)
      robot.send_speed()
      logger.info("Sleep interrupted by button")
      return 1
    elif function is not None:
      function()
    robot.send_speed()
  return 0


def update_ball_flags(dist: float, y_center: float, w: float, size: float) -> None:
  is_bottom_third = (size > consts.BALL_CATCH_SIZE and y_center > BALL_Y_2_3) or (size > consts.BALL_CATCH_SIZE * 1.1)
  is_bottom_sixth = y_center > BALL_Y_5_6

  if dist is not None:
    half_w = w / 2
    # margin = w * 0.2
    margin = w * 0.15

    ball_left = dist - half_w + RESCUE_CX + margin
    ball_right = dist + half_w + RESCUE_CX - margin

    includes_center = ball_left <= RESCUE_CX <= ball_right
  else:
    includes_center = False

  robot.write_ball_near_flag(is_bottom_sixth)
  robot.write_ball_catch_dist_flag(is_bottom_third)
  robot.write_ball_catch_offset_flag(includes_center)


def update_best_box(
    xywh,
    max_area: float,
) -> tuple[bool, float, float, float, float, float]:
  """
  Update best target info using YOLO xywh.

  Args:
    xywh: box.xywh[0]
    max_area: current max area

  Returns:
    (updated,
      new_max_area,
      dist,
      area,
      y_center,
      w)
  """
  x_center, y_center, w, h = map(float, xywh)
  area = w * h
  dist = x_center - RESCUE_CX
  if area <= max_area:
    return False, max_area, None, None, None, None

  return True, area, dist, area, y_center, w


def draw_ball_debug(image) -> None:
  """
    Draw debug lines used in update_ball_flags():
      - Vertical thresholds (2/3, 5/6 of image height)
      - Horizontal catch tolerance band (ball width)
    """
  HLINE_COLOR = (0, 255, 0)
  CENTER_COLOR = (0, 0, 255)
  cx = int(RESCUE_CX)
  y_2_3 = int(BALL_Y_2_3)
  y_5_6 = int(BALL_Y_5_6)
  cv2.line(image, (0, y_2_3), (RESCUE_IMAGE_WIDTH, y_2_3), HLINE_COLOR, 2)
  cv2.line(image, (0, y_5_6), (RESCUE_IMAGE_WIDTH, y_5_6), HLINE_COLOR, 2)
  cv2.line(image, (cx, 0), (cx, RESCUE_IMAGE_HEIGHT), CENTER_COLOR, 1)


def find_best_target() -> None:
  """Detect and track the best rescue target using YOLO object detection.

  Runs YOLO inference on the rescue camera image to find balls and cages.
  Updates robot state with the offset angle and size of the closest target
  matching the current rescue_target type. Also handles override logic
  when searching for black ball but finding silver ball.

  Updates:
    - robot.rescue_offset: Horizontal offset from image center (pixels).
    - robot.rescue_size: Area of the detected target (pixels^2).
    - robot.rescue_y: Vertical center (pixels) of the best target.
    - robot.ball_catch_dist_flag: True if ball is close enough to catch.
    - robot.rescue_target: May switch to SILVER_BALL on override.
  """
  # Reset ball flag at start - will be set True only if catchable ball detected
  robot.write_ball_catch_dist_flag(False)
  robot.write_ball_catch_offset_flag(False)
  robot.write_ball_near_flag(False)
  # yolo_results = None
  with yolo_lock:
    yolo_results = consts.MODEL(robot.rescue_image, verbose=False)
  current_time = time.time()
  origin_image = robot.rescue_image.copy()
  cv2.imwrite(f"bin/{current_time:.3f}_rescue_origin.jpg", origin_image)
  result_image = robot.rescue_image.copy()
  if yolo_results and isinstance(yolo_results, list) and len(yolo_results) > 0:
    try:
      result_image = yolo_results[0].plot()
    except TypeError as e:
      logger.error(f"Error plotting YOLO result: {e}.")
  draw_ball_debug(result_image)
  cv2.imwrite(f"bin/{current_time:.3f}_rescue_result.jpg", result_image)
  if yolo_results is None or len(yolo_results) == 0:
    logger.info("Target not found")
    robot.write_rescue_offset(None)
    robot.write_rescue_size(None)
    robot.write_rescue_y(None)
    return
  boxes = yolo_results[0].boxes
  if boxes is None or len(boxes) == 0:
    logger.info("Target not found")
    robot.write_rescue_offset(None)
    robot.write_rescue_size(None)
    robot.write_rescue_y(None)
    return
  else:
    detected_classes = []
    best_angle = None
    best_size = None
    y_center = None
    max_area = float(0)
    for box in boxes:
      try:
        cls = int(box.cls[0])
        detected_classes.append(cls)
      except Exception as e:
        logger.exception(f"Error processing detection box: {e}")
        continue
      if robot.rescue_target == consts.TargetList.EXIT.value:
        if cls == consts.TargetList.GREEN_CAGE.value:
          updated, max_area, dist, area, best_y, best_w = update_best_box(
              box.xywh[0], max_area)
          if updated:
            max_area = area
            best_angle = dist
            best_size = area
            y_center = best_y
      elif cls == robot.rescue_target:
        updated, max_area, dist, area, best_y, best_w = update_best_box(
            box.xywh[0], max_area)
        if updated:
          max_area = area
          best_angle = dist
          best_size = area
          y_center = best_y
          if cls in [
              consts.TargetList.SILVER_BALL.value,
              consts.TargetList.BLACK_BALL.value
          ]:
            update_ball_flags(dist, best_y, best_w, best_size)
      elif consts.TargetList.BLACK_BALL.value == robot.rescue_target and cls == consts.TargetList.SILVER_BALL.value:
        robot.write_rescue_turning_angle(0)
        max_area = 0
        updated, max_area, dist, area, best_y, best_w = update_best_box(
            box.xywh[0], max_area)
        if updated:
          max_area = area
          best_angle = dist
          best_size = area
          y_center = best_y
          update_ball_flags(dist, best_y, best_w, best_size)
        robot.write_rescue_target(consts.TargetList.SILVER_BALL.value)
      if cls == consts.TargetList.BLACK_BALL.value and robot.rescue_target == consts.TargetList.SILVER_BALL.value:
        robot.write_detect_black_ball(True)
    if best_angle is None:
      robot.write_rescue_offset(None)
    else:
      robot.write_rescue_offset(float(best_angle))
    if best_size is None:
      robot.write_rescue_size(None)
    else:
      robot.write_rescue_size(int(best_size))
    # Persist best target vertical center (y), if available
    if y_center is None:
      robot.write_rescue_y(None)
    else:
      robot.write_rescue_y(float(y_center))
  if robot.rescue_offset is not None and robot.rescue_size is not None and robot.rescue_y is not None:
    logger.info(
        f"Best target found - Offset: {robot.rescue_offset:.1f}px, Size: {robot.rescue_size}px², Y: {robot.rescue_y:.1f}px"
    )
  else:
    logger.info("No valid target found after processing detections")


def catch_ball() -> int:
  """Execute the ball catching sequence using the robot arm.

  Performs a timed sequence of motor and arm movements to approach,
  lower the arm, grab the ball, and lift it. The sequence includes
  forward movement, arm positioning, and grip activation.

  Returns:
    0 on successful completion (catch verification is not implemented).
  """
  # Store which ball type we're catching
  robot.set_speed(1500, 1500)
  robot.send_speed()
  robot.set_speed(1400, 1400)
  sleep_sec(0.8)
  robot.set_speed(1500, 1500)
  robot.send_speed()
  robot.set_arm(1420, 0)
  robot.send_arm()
  robot.set_speed(1650, 1650)
  sleep_sec(1)
  robot.set_speed(1500, 1500)
  robot.send_speed()
  robot.set_arm(1000, 0)
  robot.send_arm()
  robot.set_speed(1400, 1400)
  sleep_sec(1.2)
  robot.set_speed(1500, 1500)
  robot.send_speed()
  robot.set_arm(1000, 1)
  robot.send_arm()
  sleep_sec(0.5)
  robot.set_arm(3072, 1)
  robot.send_arm()
  sleep_sec(0.3)
  robot.set_arm(3072, 1)
  robot.send_arm()
  return 0


def release_ball() -> bool:
  """Execute the ball release sequence at the cage.

  Drives forward to approach the cage, opens the gripper to release
  the ball, backs up slightly, then performs a 180-degree turn to
  face away from the cage.
  Returns:
    True on successful completion.
  """
  robot.set_speed(1700, 1700)
  sleep_sec(1.5)
  robot.set_speed(1500, 1500)
  robot.send_speed()
  robot.set_speed(1300, 1300)
  sleep_sec(0.3)
  robot.set_speed(1450, 1450)
  robot.set_arm(1700, 0)
  robot.send_arm()
  robot.set_arm(1700, 0)
  robot.send_arm()
  sleep_sec(1.5)
  robot.set_arm(3072, 0)
  robot.send_arm()
  sleep_sec(0.5)
  robot.write_rescue_turning_angle(0)
  robot.set_speed(1300, 1300)
  sleep_sec(0.7)
  robot.set_speed(1750, 1250)
  sleep_sec(consts.TURN_90_TIME)
  robot.set_speed(1500, 1500)
  robot.send_speed()
  return True


def drop_ball() -> bool:
  robot.set_speed(1500, 1500)
  robot.set_arm(1536, 0)
  sleep_sec(0.4)
  robot.set_arm(3072, 0)
  robot.send_arm()
  return 0


def change_position() -> bool:
  """Rotate approximately 30 degrees to search for targets.

  Called when no target is visible. Rotates the robot in place,
  then runs find_best_target() to check for newly visible targets.

  Returns:
    True on successful completion.
  """
  robot.set_speed(1750, 1250)
  sleep_sec(consts.TURN_18_TIME)
  robot.set_speed(1500, 1500)
  sleep_sec(0.1)
  # logger.info(f"Turn degrees{robot.rescue_turning_angle}")
  return True  # Completed successfully


def set_target() -> bool:
  """Set the rescue target based on cumulative rotation angle.

  Determines which target to search for based on how much the robot
  has rotated during the rescue phase:
    - 0-360 degrees: Search for SILVER_BALL
    - 360-720 degrees: Search for BLACK_BALL
    - >720 degrees: Search for EXIT

  Returns:
    True if target was set, False if turning angle was None.
  """
  if robot.rescue_turning_angle is None:
    robot.write_rescue_turning_angle(0)
    return False
  if robot.rescue_turning_angle >= 720:
    robot.write_rescue_target(consts.TargetList.EXIT.value)
  elif robot.rescue_turning_angle >= 360 and robot.detect_black_ball:
    robot.write_rescue_target(consts.TargetList.BLACK_BALL.value)
  else:
    robot.write_rescue_target(consts.TargetList.SILVER_BALL.value)
  return True


def clamp_turning_angle() -> bool:
  angle = robot.rescue_turning_angle
  if angle is None:
    robot.write_rescue_turning_angle(0)
    return False
  if angle >= 720:
    angle = 720
  elif angle >= 360:
    angle = 360
  else:
    angle = 0
  robot.write_rescue_turning_angle(angle)
  return True


def calculate_ball() -> tuple[int, int]:
  """Calculate motor speeds to approach a ball target.

  Uses the ball's horizontal offset for steering and its apparent size
  (area) for speed control. Larger offset = more steering correction.
  Smaller size = faster approach speed (ball is further away).

  Returns:
    Tuple of (left_motor_speed, right_motor_speed) in range [MIN_SPEED, MAX_SPEED].
    Returns (1500, 1500) if target data is unavailable.
  """
  angle = robot.rescue_offset
  size = robot.rescue_size
  if angle is None or size is None:
    logger.warning(
        f"Calculate ball was called, but angle or size is None. angle: {angle}, size: {size}"
    )
    return 1500, 1500
  if not robot.ball_catch_offset_flag:
    diff_angle = angle * BOP
  else:
    diff_angle = 0
  dist_term = 0
  if not robot.ball_catch_dist_flag:
    dist_term = (math.sqrt(consts.BALL_CATCH_SIZE) - math.sqrt(size)) * BSP
    dist_term = int(max(40, min(dist_term, 200)))
  if robot.ball_catch_offset_flag and robot.ball_catch_dist_flag and robot.ball_near_flag:
    diff_angle = 0
    dist_term = -80
  if robot.ball_catch_offset_flag and robot.ball_catch_dist_flag and (not robot.ball_near_flag):
    diff_angle = 0
    dist_term = 0
  if robot.ball_catch_offset_flag and (not robot.ball_catch_dist_flag) and (not robot.ball_near_flag):
    diff_angle = 0
    dist_term *= 1.0
  if (not robot.ball_catch_offset_flag) and robot.ball_catch_dist_flag and robot.ball_near_flag:
    # diff_angle *= -0.5
    diff_angle = 0
    dist_term = -80
  if (not robot.ball_catch_offset_flag) and robot.ball_catch_dist_flag and (not robot.ball_near_flag):  # offset
   # diff_angle *= 1.3
    dist_term = 0
  base_L = 1500 + diff_angle + dist_term
  base_R = 1500 - diff_angle + dist_term
  base_L = int(base_L)
  base_R = int(base_R)
  # logger.info(f"offset: {angle} size:{size}")
  # logger.info(f"diff_angle: {diff_angle} dist_term {dist_term}")
  # logger.info(f"Motor speed L{base_L} R{base_R}")
  base_L, base_R = clamp(base_L, 1300, 1750), clamp(base_R, 1300, 1750)
  logger.info(f"Clamped Motor Speeds L{base_L} R{base_R}")
  return base_L, base_R


def calculate_cage() -> tuple[int, int]:
  """Calculate motor speeds to approach a cage target.

  Uses the cage's horizontal offset for steering correction.
  Applies a constant forward speed bias for steady approach.

  Returns:
    Tuple of (left_motor_speed, right_motor_speed) in range [MIN_SPEED, MAX_SPEED].
    Returns (1500, 1500) if target data is unavailable.
  """
  angle = robot.rescue_offset
  size = robot.rescue_size
  if angle is None or size is None:
    return 1500, 1500
  diff_angle = angle * COP
  diff_min_max = 100
  diff_angle = clamp(diff_angle, -diff_min_max, diff_min_max)
  dist_term = (math.sqrt(consts.IMAGE_SZ * 0.5) -
               math.sqrt(robot.rescue_size)) * CSP
  dist_term = int(max(150, min(dist_term, 200)))
  base_L = 1500 + diff_angle + dist_term
  base_R = 1500 - diff_angle + dist_term
  # logger.info(f"offset: {angle} size:{size}")
  logger.info(f"Motor speed L{base_L} R{base_R}")
  return clamp(int(base_L), MIN_SPEED,
               MAX_SPEED), clamp(int(base_R), MIN_SPEED, MAX_SPEED)


def wall_follow_ccw() -> bool:
  """
  Follow the wall counter-clockwise using ultrasonic[1].
  Returns True if an opening is detected.
  """
  TARGET_MIN = 10.0
  TARGET_MAX = 30.0
  OPEN_THRESHOLD = 70.0
  BASE_SPEED = 1680
  BASE_TURN = 100
  ultrasonic = robot.ultrasonic
  front_dist = ultrasonic[0]
  side_dist = ultrasonic[1]
  logger.info(f"front {front_dist}, side {side_dist}")
  if front_dist is None:
    robot.set_speed(1500, 1500)
    robot.send_speed()
    return False
  if side_dist is None or side_dist <= 0:
    robot.set_speed(1500, 1500)
    robot.send_speed()
    return False

  if side_dist > OPEN_THRESHOLD:
    logger.info("Wall opening detected")
    return True
  elif front_dist <= 4.0:
    robot.set_speed(1250, 1750)
    sleep_sec(consts.TURN_90_TIME * 0.5)
    robot.set_speed(1500, 1500)
    robot.send_speed()
    return False

  turn = 0
  if side_dist > TARGET_MAX:
    turn = BASE_TURN * -1
  if side_dist < TARGET_MIN:
    turn = BASE_TURN
  left_speed = BASE_SPEED - turn
  right_speed = BASE_SPEED + turn
  # logger.info(f"motor speed L{left_speed} R{right_speed}")
  left_speed, right_speed = clamp(left_speed), clamp(right_speed)
  robot.set_speed(left_speed, right_speed)
  robot.send_speed()

  return False


def handle_not_found() -> None:
  change_position()
  # Only call set_target() if searching for balls (rotation-based logic).
  # For cages/exit, keep searching the current target.
  if robot.rescue_target in [
      consts.TargetList.SILVER_BALL.value, consts.TargetList.BLACK_BALL.value
  ]:
    robot.write_rescue_turning_angle(robot.rescue_turning_angle + 18)
    set_target()


def handle_exit() -> None:
  if not robot.has_moved_to_cage:
    # logger.info("Finding Red Cage for exiting")
    if robot.rescue_offset is None:
      return
    else:
      motorl, motorr = calculate_cage()
      robot.set_speed(motorl, motorr)
      robot.send_speed()
      if robot.rescue_size is not None and robot.rescue_size >= consts.IMAGE_SZ * 0.5 and robot.rescue_y is not None and robot.rescue_y > (
          robot.rescue_image.shape[0] * 1 / 2):
        robot.set_speed(1700, 1700)
        sleep_sec(1)
        robot.set_speed(1300, 1300)
        sleep_sec(0.5)
        robot.set_speed(1500, 1500)
        robot.send_speed()
        robot.set_speed(1250, 1750)
        sleep_sec(consts.TURN_90_TIME)
        robot.set_speed(1500, 1500)
        robot.send_speed()
        robot.write_has_moved_to_cage(True)
  else:
    # logger.info("wall follow ccw")
    result = wall_follow_ccw()
    if result:
      robot.set_speed(1500, 1500)
      robot.send_speed()
      robot.set_speed(1650, 1650)
      sleep_sec(2.4)
      robot.set_speed(1750, 1250)
      sleep_sec(consts.TURN_90_TIME)
      while True:
        robot.update_button_stat()
        if robot.robot_stop:
          robot.set_speed(1500, 1500)
          robot.send_speed()
          break

        robot.set_speed(1650, 1650)
        robot.send_speed()

        if robot.linetrace_slope is not None and robot.line_area >= consts.MIN_OBJECT_AVOIDANCE_LINE_AREA:
          logger.info("Line detected, exit rescue mode")
          robot.set_speed(1600, 1600)
          sleep_sec(1.0)
          robot.set_speed(1500, 1500)
          robot.send_speed()
          robot.write_is_rescue_flag(False)
          break


def handle_ball() -> None:
  last_offset_flag = robot.ball_catch_offset_flag
  last_dist_flag = robot.ball_catch_dist_flag
  last_near_flag = robot.ball_near_flag
  motorl, motorr = calculate_ball()
  robot.set_speed(motorl, motorr)
  robot.send_speed()
  if last_offset_flag and last_dist_flag and (not last_near_flag):  # Catch
    catch_ball()
    if robot.rescue_target == consts.TargetList.SILVER_BALL.value:
      robot.write_rescue_target(consts.TargetList.GREEN_CAGE.value)
    elif robot.rescue_target == consts.TargetList.BLACK_BALL.value:
      robot.write_rescue_target(consts.TargetList.RED_CAGE.value)
    # After catching, clear cached target data and force an immediate
    # YOLO evaluation so the robot can detect and move toward the cage.
    robot.write_rescue_offset(None)
    robot.write_rescue_size(None)
    robot.write_rescue_y(None)
    # logger.info(
    #     "Post-catch: reset rescue_offset/size/y and forced YOLO run")


def handle_cage() -> None:
  clamp_turning_angle()
  motorl, motorr = calculate_cage()
  robot.set_speed(motorl, motorr)
  robot.send_speed()
  if robot.rescue_size is not None and robot.rescue_size >= consts.IMAGE_SZ * 0.5 and robot.rescue_y is not None and robot.rescue_y > (
      robot.rescue_image.shape[0] * 1 / 2):
    release_ball()
    set_target()


def is_stopping_by_button() -> None:
  if robot.rescue_target == consts.TargetList.SILVER_BALL.value or robot.rescue_target == consts.TargetList.GREEN_CAGE.value:
    robot.write_rescue_target(consts.TargetList.SILVER_BALL.value)
  elif robot.rescue_target == consts.TargetList.BLACK_BALL.value or robot.rescue_target == consts.TargetList.RED_CAGE.value:
    robot.write_rescue_target(consts.TargetList.BLACK_BALL.value)
  else:
    robot.write_rescue_target(consts.TargetList.EXIT.value)
  robot.set_speed(1500, 1500)
  robot.set_arm(3072, 0)
  robot.send_speed()
  robot.send_arm()
  robot.write_rescue_turning_angle(0)
  logger.info("robot stop true, stopping..")
  robot.write_linetrace_stop(False)
  robot.write_is_rescue_flag(False)
  robot.write_last_slope_get_time(time.time())
  robot.write_ball_catch_dist_flag(False)
  robot.write_ball_catch_offset_flag(False)
  robot.write_ball_near_flag(False)
  robot.write_has_moved_to_cage(False)
  robot.write_detect_black_ball(False)


logger.info("Objects Initialized")

if __name__ == "__main__":
  assert isinstance(robot, modules.robot.Robot)
  # Register signal handler for graceful shutdown
  signal.signal(signal.SIGINT, signal_handler)

  logger.info("Starting program")
  robot.set_speed(1500, 1500)
  robot.set_arm(3072, 0)
  robot.send_arm()
  robot.send_speed()
  robot.write_linetrace_stop(False)
  robot.write_is_rescue_flag(False)
  robot.write_last_slope_get_time(time.time())
  robot.write_rescue_target(consts.TargetList.SILVER_BALL.value)
  while True:
    robot.update_button_stat()
    robot.update_gyro_stat()
    if robot.robot_stop:
      is_stopping_by_button()
    elif robot.is_rescue_flag:
      find_best_target()
      try:
        logger.info(
            f"Searching for target: {consts.TargetList(robot.rescue_target).name} (id={robot.rescue_target})"
        )
      except Exception:
        logger.info(f"Searching for target id: {robot.rescue_target}")
      if not robot.has_moved_to_cage and ((robot.rescue_offset is None) or
                                          (robot.rescue_size is None)):
        logger.debug("not fund")
        handle_not_found()
      elif robot.rescue_target == consts.TargetList.EXIT.value:
        logger.debug("exit")
        handle_exit()
      elif robot.rescue_target == consts.TargetList.BLACK_BALL.value or robot.rescue_target == consts.TargetList.SILVER_BALL.value:
        logger.debug("ball")
        handle_ball()
      else:
        logger.debug("cage")
        handle_cage()
    else:
      if not robot.linetrace_stop:
        ultrasonic_info = robot.ultrasonic
        # Check for green mark intersections before normal line following
        logger.info(ultrasonic_info)
        if should_process_green_mark():
          execute_green_mark_turn()
        elif ultrasonic_info[
            0] <= 3:  # TODO: The index is really wired, the return value is including some bug, but not sure what is the problem
          logger.info("Object avoidance triggered")
          robot.set_speed(1400, 1400)
          sleep_sec(1, robot.send_speed)
          robot.set_speed(1750, 1250)
          sleep_sec(1.7, robot.send_speed)
          robot.set_speed(1580, 1800)
          sleep_sec(1, robot.send_speed)
          object_avoidance_start = time.time()
          while robot.linetrace_slope is None:
            if time.time(
            ) - object_avoidance_start >= 2 and robot.line_area <= consts.MIN_OBJECT_AVOIDANCE_LINE_AREA:
              break
            logger.info("Turning around in object avoidance...")
            robot.write_last_slope_get_time(time.time())
            robot.set_speed(1580, 1800)
            robot.send_speed()
            robot.update_button_stat()
            if robot.robot_stop:
              logger.info("Robot interrupted during object avoidance")
              break
          logger.info(
              f"Ejecting object avoidance by {robot.linetrace_slope} {robot.line_area}"
          )
          robot.set_speed(1600, 1600)
          sleep_sec(1)
          robot.set_speed(1600, 1400)
          sleep_sec(1)
        else:
          # Check if line recovery is needed (small line area + steep angle)
          angle_error = get_current_angle_error()
          line_area = robot.line_area

          if angle_error is not None and should_execute_line_recovery(
              line_area, angle_error):
            execute_line_recovery()
          else:
            motorl, motorr = calculate_motor_speeds()
            robot.set_speed(motorl, motorr)
      else:
        logger.info("Red stop")
        robot.set_speed(1500, 1500)
    robot.send_speed()
logger.info("Program Stop")
