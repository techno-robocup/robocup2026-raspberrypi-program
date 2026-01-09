import modules.constants as consts
import modules.logger
import modules.robot
import time
import signal
import sys
import cv2
import math
import threading
from typing import Optional

logger = modules.logger.get_logger()

logger.debug("Logger initialized")

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
TURNING_BASE_SPEED = 1650
assert 1500 < BASE_SPEED < 2000
assert 1500 < TURNING_BASE_SPEED < 2000
assert TURNING_BASE_SPEED < BASE_SPEED
MAX_SPEED = 2000
MIN_SPEED = 1000
KP = 225
DP = 200
BOP = 0.045  # Ball Offset P
BSP = 1.5  # Ball Size P
COP = 0.03  # Cage Offset P
CSP = 1.5
EOP = 0.03  # Exit Offset P
ESP = 2  # Exit Size P

catch_failed_cnt = 0

# Gap recovery state - timestamp of last recovery to prevent immediate re-trigger
last_gap_recovery_time: float = 0.0
GAP_RECOVERY_COOLDOWN = 0.5  # Seconds to wait after recovery before allowing another


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
  Execute a turn based on detected green marks using line detection at checkpoint.

  Turn logic (line-based with timeout fallback):
  - Both left and right: 180° turn (wait for 2 line crossings)
  - Only left: 90° left turn (wait for 1 line crossing)
  - Only right: 90° right turn (wait for 1 line crossing)

  Line crossing = checkpoint at top of image transitions from not-black to black.

  Returns:
  - True if turn completed successfully
  - False if interrupted by button
  """
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

  # First, drive forward slightly to clear the intersection marker
  start_time = time.time()
  while time.time() - start_time < consts.GREEN_MARK_APPROACH_TIME:
    robot.update_button_stat()
    if robot.robot_stop:
      robot.set_speed(1500, 1500)
      robot.send_speed()
      logger.debug("Turn interrupted by button during approach")
      return False
    robot.set_speed(BASE_SPEED, BASE_SPEED)
    robot.send_speed()

  # Execute turn based on detected directions
  if has_left and has_right:
    start_time = time.time()
    max_time = consts.MAX_TURN_180_TIME
    target_crossings = 2
    line_crossings = 0
    prev_checkpoint_black = False  # Initialize after delay

    while line_crossings < target_crossings:
      elapsed = time.time() - start_time
      if elapsed > max_time:
        logger.warning(f"180° turn timeout after {line_crossings} crossings")
        break
      robot.update_button_stat()
      if robot.robot_stop:
        robot.set_speed(1500, 1500)
        robot.send_speed()
        logger.debug("Turn interrupted by button during 180° turn")
        return False
      robot.set_speed(TURNING_BASE_SPEED,
                      3000 - TURNING_BASE_SPEED)  # Turn left
      robot.send_speed()
      # Only check for line crossing after delay has passed
      if elapsed > consts.TURN_CHECK_DELAY:
        current_checkpoint_black = robot.top_checkpoint_black
        if current_checkpoint_black and not prev_checkpoint_black:
          line_crossings += 1
        prev_checkpoint_black = current_checkpoint_black
    logger.info(f"180° turn completed in {time.time() - start_time:.2f}s")

  elif has_left:
    start_time = time.time()
    max_time = consts.MAX_TURN_90_TIME
    target_crossings = 1
    line_crossings = 0
    prev_checkpoint_black = False  # Initialize after delay

    while line_crossings < target_crossings:
      elapsed = time.time() - start_time
      if elapsed > max_time:
        logger.warning(
            f"90° left turn timeout after {line_crossings} crossings")
        break
      robot.update_button_stat()
      if robot.robot_stop:
        robot.set_speed(1500, 1500)
        robot.send_speed()
        logger.debug("Turn interrupted by button during 90° left turn")
        return False
      robot.set_speed(TURNING_BASE_SPEED,
                      3000 - TURNING_BASE_SPEED)  # Turn left
      robot.send_speed()
      # Only check for line crossing after delay has passed
      if elapsed > consts.TURN_CHECK_DELAY:
        current_checkpoint_black = robot.top_checkpoint_black
        if current_checkpoint_black and not prev_checkpoint_black:
          line_crossings += 1
        prev_checkpoint_black = current_checkpoint_black
    logger.info(f"90° left turn completed in {time.time() - start_time:.2f}s")

  elif has_right:
    start_time = time.time()
    max_time = consts.MAX_TURN_90_TIME
    target_crossings = 1
    line_crossings = 0
    prev_checkpoint_black = False  # Initialize after delay

    while line_crossings < target_crossings:
      elapsed = time.time() - start_time
      if elapsed > max_time:
        logger.warning(
            f"90° right turn timeout after {line_crossings} crossings")
        break
      robot.update_button_stat()
      if robot.robot_stop:
        robot.set_speed(1500, 1500)
        robot.send_speed()
        logger.debug("Turn interrupted by button during 90° right turn")
        return False
      robot.set_speed(3000 - TURNING_BASE_SPEED,
                      TURNING_BASE_SPEED)  # Turn right
      robot.send_speed()
      # Only check for line crossing after delay has passed
      if elapsed > consts.TURN_CHECK_DELAY:
        current_checkpoint_black = robot.top_checkpoint_black
        if current_checkpoint_black and not prev_checkpoint_black:
          line_crossings += 1
        prev_checkpoint_black = current_checkpoint_black
    logger.info(f"90° right turn completed in {time.time() - start_time:.2f}s")

  # Stop after turn
  robot.set_speed(1500, 1500)
  robot.send_speed()
  robot.set_speed(1330, 1330)
  robot.send_speed()
  time.sleep(0.8)
  robot.set_speed(1500, 1500)
  robot.send_speed()
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
  global last_gap_recovery_time
  
  if line_area is None or not is_valid_number(line_area):
    return False
  
  # Check cooldown to prevent rapid re-triggering during recovery
  if time.time() - last_gap_recovery_time < GAP_RECOVERY_COOLDOWN:
    return False
  
  # Get line center x-coordinate and check offset from image center
  line_center_x = robot.line_center_x
  image_center_x = consts.LINETRACE_CAMERA_LORES_WIDTH // 2
  x_offset = abs(line_center_x - image_center_x) if line_center_x is not None else 0
  
  # Check if x-offset is significant
  x_offset_significant = x_offset > consts.LINETRACE_CAMERA_LORES_WIDTH * 0.1

  area_condition = line_area < consts.LINE_RECOVERY_AREA_THRESHOLD
  x_offset_condition = x_offset_significant
  
  # Only trigger when area is small AND x-offset is significant
  should_recover = area_condition and x_offset_condition
  
  if should_recover:
    logger.info(f"Line recovery triggered: Low area ({line_area:.1f} < {consts.LINE_RECOVERY_AREA_THRESHOLD}) AND large x-offset ({x_offset:.1f}px, center at {line_center_x})")
  
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
      logger.debug("Line recovery interrupted by button")
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
      power_curve = normalized ** 2  # Quadratic gives aggressive ramp
      # Scale to 0.3-1.0 range
      speed_multiplier = 0.3 + power_curve * 0.7
      speed_multiplier = max(0.3, min(1.0, speed_multiplier))
      logger.info(
          f"Line area: {line_area:.0f}, speed multiplier: {speed_multiplier:.2f}"
      )

  # Apply speed multiplier only to the increment above 1500 (stop position)
  # 1500 = stop, so we only reduce the forward speed component
  adjusted_base_speed = 1500 + int((BASE_SPEED - 1500) * speed_multiplier)

  # logger.info(f"Current adjusted speed: {clamp(int(adjusted_base_speed - abs(angle_error)**6 * DP), 1500, 2000)}")
  motor_l = clamp(
      clamp(int(adjusted_base_speed - abs(angle_error)**6 * DP), 1500, 2000) -
      steering, MIN_SPEED, MAX_SPEED)
  motor_r = clamp(
      clamp(int(adjusted_base_speed - abs(angle_error)**6 * DP), 1500, 2000) +
      steering, MIN_SPEED, MAX_SPEED)

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
      logger.debug("Sleep interrupted by button")
      return 1
    elif function is not None:
      function()
    robot.send_speed()
  return 0


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
    - robot.rescue_ball_flag: True if ball is close enough to catch.
    - robot.rescue_target: May switch to SILVER_BALL on override.
  """
  # Reset ball flag at start - will be set True only if catchable ball detected
  robot.write_rescue_ball_flag(False)
  # yolo_results = None
  with yolo_lock:
    yolo_results = consts.MODEL(robot.rescue_image, verbose=False)
    robot.write_last_yolo_time(time.time())
  current_time = time.time()
  result_image = robot.rescue_image
  if yolo_results and isinstance(yolo_results, list) and len(yolo_results) > 0:
    try:
      result_image = yolo_results[0].plot()
    except TypeError as e:
      logger.error(f"Error plotting YOLO result: {e}.")
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
    image_height = yolo_results[0].orig_shape[0]
    image_width = yolo_results[0].orig_shape[1]
    detected_classes = []
    best_angle = None
    best_size = None
    best_target_y = None
    best_target_w = None
    # best_target_h = None
    min_dist = float("inf")
    cx = image_width / 2.0
    for box in boxes:
      try:
        cls = int(box.cls[0])
        detected_classes.append(cls)
      except Exception as e:
        logger.exception(f"Error processing detection box: {e}")
        continue
      if cls == robot.rescue_target:
        x_center, y_center, w, h = map(float, box.xywh[0])
        dist = x_center - cx
        area = w * h
        if abs(dist) < min_dist:
          robot.write_rescue_ball_flag(False)
          min_dist = abs(dist)
          best_angle = dist
          best_size = area
          best_target_y = y_center
          best_target_w = w
          # best_target_h = h
          if cls == consts.TargetList.BLACK_BALL.value or cls == consts.TargetList.SILVER_BALL.value:
            is_bottom_third = best_target_y and best_target_y > (image_height *
                                                                 2 / 3)
            if best_angle is not None:
              ball_left = best_angle - best_target_w / 2 + image_width / 2
              ball_right = best_angle + best_target_w / 2 + image_width / 2
              includes_center = ball_left <= image_width / 2 <= ball_right
            else:
              includes_center = False
            if is_bottom_third and includes_center:
              robot.write_rescue_ball_flag(True)
        logger.info(
            f"Detected cls={consts.TargetList(cls).name}, area={area:.1f}, offset={dist:.1f}"
        )
      elif consts.TargetList.BLACK_BALL.value == robot.rescue_target and cls == consts.TargetList.SILVER_BALL.value:
      # elif consts.TargetList.SILVER_BALL.value != robot.rescue_target and cls == consts.TargetList.SILVER_BALL.value:
        logger.info("Override")
        robot.write_rescue_turning_angle(0)
        # if robot.rescue_target in (consts.TargetList.RED_CAGE, consts.TargetList.GREEN_CAGE):
        #   drop_ball()
        x_center, y_center, w, h = map(float, box.xywh[0])
        dist = x_center - cx
        area = w * h
        if abs(dist) < min_dist:
          robot.write_rescue_ball_flag(False)
          min_dist = abs(dist)
          best_angle = dist
          best_size = area
          best_target_y = y_center
          best_target_w = w
          # best_target_h = h
          # Check if ball is close enough to catch (same logic as primary target)
          is_bottom_third = best_target_y and best_target_y > (image_height *
                                                               2 / 3)
          ball_left = best_angle - best_target_w / 2 + image_width / 2
          ball_right = best_angle + best_target_w / 2 + image_width / 2
          includes_center = ball_left <= image_width / 2 <= ball_right
          if is_bottom_third and includes_center:
            robot.write_rescue_ball_flag(True)
        robot.write_rescue_target(consts.TargetList.SILVER_BALL.value)
        logger.info(
            f"Override Detected cls={consts.TargetList(cls).name}, area={area:.1f}, offset={dist:.1f}"
        )
    if best_angle is None:
      robot.write_rescue_offset(None)
    else:
      robot.write_rescue_offset(float(best_angle))
    if best_size is None:
      robot.write_rescue_size(None)
    else:
      robot.write_rescue_size(int(best_size))
    # Persist best target vertical center (y), if available
    if best_target_y is None:
      robot.write_rescue_y(None)
    else:
      robot.write_rescue_y(float(best_target_y))


def catch_ball() -> int:
  """Execute the ball catching sequence using the robot arm.

  Performs a timed sequence of motor and arm movements to approach,
  lower the arm, grab the ball, and lift it. The sequence includes
  forward movement, arm positioning, and grip activation.

  Returns:
    0 on successful completion (catch verification is not implemented).
  """
  logger.debug("Executing catch_ball()")
  # Store which ball type we're catching
  logger.info(
      f"Caught ball type: {consts.TargetList(robot.rescue_target).name}")
  robot.set_speed(1500, 1500)
  robot.send_speed()
  robot.set_speed(1400, 1400)
  sleep_sec(1.5)
  robot.set_speed(1500, 1500)
  robot.send_speed()
  robot.set_arm(1450, 0)
  robot.send_arm()
  robot.set_speed(1650, 1650)
  sleep_sec(2.2)
  robot.set_speed(1500, 1500)
  robot.send_speed()
  robot.set_arm(1000, 0)
  robot.send_arm()
  robot.set_speed(1600, 1600)
  sleep_sec(2)
  robot.set_speed(1400, 1400)
  sleep_sec(2.3)
  robot.set_arm(1000, 1)
  sleep_sec(0.5)
  robot.send_arm()
  robot.set_arm(3072, 1)
  robot.send_arm()
  robot.set_speed(1450, 1450)
  sleep_sec(1)
  robot.set_speed(1500, 1500)
  robot.send_speed()
  return 0
  # prev_time = time.time()
  # while time.time() - prev_time < 0.2:
  #   robot.send_speed()
  # find_best_target()
  # if robot.rescue_offset is None:
  #   logger.info("Catch successful")
  #   robot.write_rescue_target(
  #       consts.TargetList.GREEN_CAGE.value if robot.rescue_target == consts.
  #       TargetList.SILVER_BALL.value else consts.TargetList.RED_CAGE.value)
  #   return 0
  # else:
  #   logger.info("Catch failed")
  #   return 1


def release_ball() -> bool:
  """Execute the ball release sequence at the cage.

  Drives forward to approach the cage, opens the gripper to release
  the ball, backs up slightly, then performs a 180-degree turn to
  face away from the cage.
  Returns:
    True on successful completion.
  """
  logger.debug("Executing release_ball()")
  robot.set_speed(1700, 1700)
  sleep_sec(2.3)
  robot.set_speed(1500, 1500)
  robot.send_speed()
  robot.set_speed(1400, 1400)
  sleep_sec(1)
  robot.set_speed(1500, 1500)
  robot.set_arm(1700, 0)
  robot.send_arm()
  sleep_sec(1.5)
  robot.set_arm(3072, 0)
  robot.send_arm()
  sleep_sec(0.5)
  robot.write_rescue_turning_angle(0)
  robot.set_speed(1400, 1400)
  sleep_sec(2.5)
  robot.set_speed(1750, 1250)
  sleep_sec(consts.TURN_180_TIME)
  robot.set_speed(1500, 1500)
  robot.send_speed()
  return True

def drop_ball() -> bool:
  logger.debug("Drop ball")
  robot.set_speed(1500,1500)
  robot.set_arm(1536,0)
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
  logger.debug("Change position")
  robot.set_speed(1750, 1250)
  sleep_sec(consts.TURN_18_TIME)
  robot.set_speed(1500, 1500)
  sleep_sec(0.1)
  find_best_target()
  # robot.write_rescue_turning_angle(robot.rescue_turning_angle + 30)
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
    # robot.write_rescue_target(consts.TargetList.SILVER_BALL.value)
  elif robot.rescue_turning_angle >= 360:
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
  diff_angle = 0
  if abs(angle) > 30:
    diff_angle = angle * BOP
  else:
    diff_angle = 0
  dist_term = 0
  if consts.BALL_CATCH_SIZE > size:
    dist_term = (math.sqrt(consts.BALL_CATCH_SIZE) - math.sqrt(size))**2 * BSP
    dist_term = int(max(120, min(dist_term, 250)))
  base_L = 1500 + diff_angle + dist_term
  base_R = 1500 - diff_angle + dist_term
  base_L = int(base_L)
  base_R = int(base_R)
  logger.info(f"offset: {angle} size:{size}")
  logger.info(f"diff_angle: {diff_angle} dist_term {dist_term}")
  logger.info(f"Motor speed L{base_L} R{base_R}")
  base_L, base_R = clamp(base_L), clamp(base_R)
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
  dist_term = (math.sqrt(consts.IMAGE_SZ * 0.5) - math.sqrt(robot.rescue_size)) * CSP
  dist_term = int(max(130,dist_term))
  base_L = 1500 + diff_angle + dist_term
  base_R = 1500 - diff_angle + dist_term
  logger.info(f"offset: {angle} size:{size}")
  logger.info(f"Motor speed L{base_L} R{base_R}")
  return clamp(int(base_L), MIN_SPEED,
               MAX_SPEED), clamp(int(base_R), MIN_SPEED, MAX_SPEED)


def calculate_exit() -> tuple[int, int]:
  """Calculate motor speeds to approach the exit target.

  Uses the exit marker's horizontal offset for steering correction.
  Includes a deadband (±10) to reduce oscillation when nearly aligned.

  Returns:
    Tuple of (left_motor_speed, right_motor_speed) in range [MIN_SPEED, MAX_SPEED].
    Returns (1500, 1500) if target data is unavailable.
  """
  angle = robot.rescue_offset
  size = robot.rescue_size
  if angle is None or size is None:
    return 1500, 1500
  if abs(angle) > 30:
    diff_angle = angle * EOP
    if diff_angle > 0:
      diff_angle = max(diff_angle - 10, 0)
    else:
      diff_angle = max(diff_angle + 10, 0)
  else:
    diff_angle = 0
  dist_term = 0
  if consts.BALL_CATCH_SIZE * 3 > size:
    dist_tern = (math.sqrt(consts.BALL_CATCH_SIZE * 3) - math.sqrt(size)) ** 2 * ESP
  dist_term = int(max(150, dist_term))
  base_L = 1500 + diff_angle + dist_term
  base_R = 1500 - diff_angle + dist_term
  logger.info(f"offset: {angle} size: {size}")
  logger.info(f"Motor speed L{base_L} R{base_R}")
  return clamp(int(base_L), MIN_SPEED,
               MAX_SPEED), clamp(int(base_R), MIN_SPEED, MAX_SPEED)


# def retry_catch() -> bool:
#   global catch_failed_cnt
#   catch_failed_cnt += 1
#   prev_time = time.time()
#   robot.set_speed(1300, 1300)
#   while time.time() - prev_time > 2:  #TODO(K10-K10): random walk
#     robot.send_speed()
#     robot.update_button_stat()
#     if robot.robot_stop:
#       robot.set_speed(1500, 1500)
#       robot.send_speed()
#       logger.info("Turn interrupted by button during approach")
#       return False
#   return True

logger.debug("Objects Initialized")

if __name__ == "__main__":
  assert isinstance(robot, modules.robot.Robot)
  # Register signal handler for graceful shutdown
  signal.signal(signal.SIGINT, signal_handler)

  logger.info("Starting program")
  robot.set_speed(1500, 1500)
  robot.set_arm(3072, 0)
  robot.send_arm()
  robot.send_speed()
  robot.write_rescue_turning_angle(0)
  robot.write_rescue_target(consts.TargetList.BLACK_BALL.value)
  robot.write_linetrace_stop(False)
  robot.write_is_rescue_flag(False)
  robot.write_last_slope_get_time(time.time())
  while True:
    robot.update_button_stat()
    if robot.robot_stop:
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
      logger.debug("robot stop true, stopping..")
      robot.write_linetrace_stop(False)
      robot.write_is_rescue_flag(False)
      robot.write_last_slope_get_time(time.time())
      robot.write_rescue_ball_flag(False)
    elif robot.is_rescue_flag:
      find_best_target()
      try:
        logger.info(
            f"Searching for target: {consts.TargetList(robot.rescue_target).name} (id={robot.rescue_target})"
        )
      except Exception:
        logger.info(f"Searching for target id: {robot.rescue_target}")
      if (robot.rescue_offset is None) or (robot.rescue_size is None):
        change_position()
        if not robot.rescue_ball_flag:
          robot.write_rescue_turning_angle(robot.rescue_turning_angle + 18)
        # Only call set_target() if searching for balls (rotation-based logic).
        # For cages/exit, keep searching the current target.
        if robot.rescue_target in [
            consts.TargetList.SILVER_BALL.value,
            consts.TargetList.BLACK_BALL.value
        ]:
          set_target()
      else:
        if robot.rescue_target == consts.TargetList.EXIT.value:
          motorl, motorr = calculate_exit()
          robot.set_speed(motorl, motorr)
          robot.send_speed()
          if robot.linetrace_slope is not None:
            robot.set_speed(1500, 1500)
            robot.send_speed()
            robot.write_is_rescue_flag(False)
        elif robot.rescue_target == consts.TargetList.BLACK_BALL.value or robot.rescue_target == consts.TargetList.SILVER_BALL.value:
          motorl, motorr = calculate_ball()
          robot.set_speed(motorl, motorr)
          robot.send_speed()
          if robot.rescue_ball_flag:
            is_not_took = catch_ball()
            if robot.rescue_target == consts.TargetList.SILVER_BALL.value:
              robot.write_rescue_target(consts.TargetList.GREEN_CAGE.value)
            elif robot.rescue_target == consts.TargetList.BLACK_BALL.value:
              robot.write_rescue_target(consts.TargetList.RED_CAGE.value)
            # After catching, clear cached target data and force an immediate
            # YOLO evaluation so the robot can detect and move toward the cage.
            robot.write_rescue_offset(None)
            robot.write_rescue_size(None)
            robot.write_rescue_y(None)
            robot.write_last_yolo_time(0)
            logger.info(
                "Post-catch: reset rescue_offset/size/y and forced YOLO run")
        else:
          clamp_turning_angle()
          motorl, motorr = calculate_cage()
          robot.set_speed(motorl, motorr)
          robot.send_speed()
          if robot.rescue_size is not None and robot.rescue_size >= consts.IMAGE_SZ * 0.5 and robot.rescue_y is not None and robot.rescue_y > (
              robot.rescue_image.shape[0] * 1 / 2):
            release_ball()
            set_target()
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
          while robot.linetrace_slope is None or robot.line_area <= consts.MIN_OBJECT_AVOIDANCE_LINE_AREA:
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
logger.debug("Program Stop")
