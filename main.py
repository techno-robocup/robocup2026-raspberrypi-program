import math
import signal
import sys
import time
from typing import Optional

import cv2
from readerwriterlock import rwlock

import modules.constants as consts
import modules.logger
import modules.robot

logger = modules.logger.get_logger()

logger.info("Logger initialized")

# Mutex lock for thread-safe YOLO evaluation
yolo_lock = rwlock.RWLockFairD()

robot = modules.robot.robot
uart_dev = modules.robot.uart_io()

# UART device is auto-selected in uart_io.__connect()
uart_dev.connect(consts.UART_BAUD_RATE, consts.UART_TIMEOUT)
robot.set_uart_device(uart_dev)

BASE_SPEED = 1710
assert 1500 < BASE_SPEED < 2000
# assert TURNING_BASE_SPEED < BASE_SPEED
MAX_SPEED = 2000
MIN_SPEED = 1000
KP = 170
KI = 280
KD = 30
DP = 200
INTEGRAL_MAX = 1  # Anti-windup: max |accumulated integral error| in radians*sec
BOP = 0.03  # Ball Offset P
BSP = 0.3  # Ball Size P
COP = 0.06  # Cage Offset P
CSP = 1.5
EOP = 0.03  # Exit Offset P
ESP = 2  # Exit Size P

catch_failed_cnt = 0

# PID state for linetrace steering
_pid_prev_error: float = 0.0
_pid_integral: float = 0.0
_pid_prev_time: Optional[float] = None

# Gap recovery state - timestamp of last recovery to prevent immediate re-trigger
last_gap_recovery_time: float = 0.0
GAP_RECOVERY_COOLDOWN = 0.5  # Seconds to wait after recovery before allowing another

RESCUE_IMAGE_WIDTH = 4608
RESCUE_IMAGE_HEIGHT = 2592
RESCUE_CX = RESCUE_IMAGE_WIDTH / 2.0

BALL_Y_2_3 = (RESCUE_IMAGE_HEIGHT * 2 / 3)  # 1728.0 - x
BALL_Y_5_6 = (RESCUE_IMAGE_HEIGHT * 5 / 6) - 100  # 2160.0 - x


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


# def normalize_rotation_angle(angle: float) -> float:
#   """Normalize rotation angle to handle opposite-direction turns.

#   When the robot turns slightly in the opposite direction of intended,
#   the angle calculation can wrap around (e.g., 359.5° instead of -0.5°).
#   This function converts angles >180° to their negative equivalent.

#   Args:
#     angle: Rotation angle in degrees (0-360 range from modulo calculation).

#   Returns:
#     Normalized angle in degrees (-180 to 180 range).
#   """
#   if angle > 180.0:
#     return angle - 360.0
#   return angle


def should_process_green_mark() -> bool:
  """
  Determine if we should process green marks for intersection turning.

  Returns True if:
  - Green marks are detected with left and/or right black lines
  - At least one mark is in the bottom portion of the image
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
    # Only require left/right lines to determine turn direction.
    # The approaching line may come from the top or bottom depending
    # on the tile layout, so we do not filter on bottom/top.
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


def execute_green_uturn() -> bool:
  """Execute a 180° U-turn (green marks on both sides = dead end).

  Drives forward briefly to center on the intersection, then spins
  180° using fixed motor speeds until the black line is detected again.

  Returns True if completed, False if interrupted by button.
  """
  use_bno = False
  logger.info("Starting 180° U-turn")

  # Drive forward a bit to center on the intersection
  robot.set_speed(1550, 1550)
  robot.send_speed()
  sleep_sec(0.5, robot.send_speed)

  # Stop before turning
  robot.set_speed(1500, 1500)
  robot.send_speed()
  sleep_sec(0.2, robot.send_speed)

  # Spin in place (left motor backward, right motor forward).
  # The robot must detect the black line TWICE: the first crossing is
  # the original approach line, the second is the line it needs to
  # follow back (180° from the start).
  max_turn_time = consts.MAX_TURN_180_TIME
  started_turning = time.time()
  black_check_enabled = False
  black_crosses = 0
  was_on_black = False

  degree = None
  if use_bno:
    degree = robot.yaw

  while True:
    robot.update_button_stat()
    if robot.robot_stop:
      robot.set_speed(1500, 1500)
      robot.send_speed()
      return False

    if time.time() - started_turning > max_turn_time:
      logger.warning(f"U-turn timeout after {max_turn_time:.1f}s")
      break

    if use_bno:
      current_robot_yaw = robot.yaw
      diff = current_robot_yaw - degree
      diff = diff if diff > 0 else diff + 360
      if time.time() - started_turning > 1 and diff > 180 * 1.3:
        logger.warning(
            f"U-turn turning too much. Initial: {degree} Current: {current_robot_yaw}"
        )
        break

    # Enable black line check after passing through the initial dead zone
    past_dead_zone = (time.time() - started_turning
                      > consts.GREEN_GYRO_PASS_TIME)
    if past_dead_zone and not black_check_enabled:
      black_check_enabled = True
      logger.info("U-turn: black check enabled")

    if black_check_enabled:
      on_black = robot.top_checkpoint_black
      if on_black and not was_on_black:
        black_crosses += 1
        logger.info(f"U-turn: black line crossing #{black_crosses}")
      was_on_black = on_black
      if black_crosses >= 2:
        logger.info("U-turn: second black line found — stopping")
        break

    # Spin right (arbitrary direction for 180°)
    robot.set_speed(1750, 1250)
    robot.send_speed()

  robot.set_speed(1500, 1500)
  robot.send_speed()
  robot.write_last_slope_get_time(time.time())
  return True


def should_execute_line_recovery(arg_line_area: Optional[float]) -> bool:
  if arg_line_area is None or not is_valid_number(arg_line_area):
    return False

  if time.time() - last_gap_recovery_time < GAP_RECOVERY_COOLDOWN:
    return False

  line_center_x = robot.line_center_x
  if line_center_x is None:
    return False

  image_width = consts.LINETRACE_CAMERA_LORES_WIDTH
  x_offset = abs(line_center_x - image_width / 2)
  offset_condition = x_offset > image_width / 10
  area_condition = arg_line_area < consts.MIN_BLACK_LINE_AREA * 2

  should_recover = offset_condition and area_condition

  if should_recover:
    logger.info(
        f"Line recovery triggered: x_offset={x_offset:.0f}px, "
        f"area={arg_line_area:.0f} (threshold={consts.MIN_BLACK_LINE_AREA * 2})")

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
  recovery_timeout = 3.0  # Max seconds to back up before giving up
  while robot.line_area is None or robot.line_area <= consts.LINE_RECOVERY_AREA_THRESHOLD * 5:
    if time.time() - start_time > recovery_timeout:
      logger.warning("Line recovery timeout — giving up")
      break
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


_is_in_gap = False
_is_approached_line = False


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
  global _pid_prev_error, _pid_integral, _pid_prev_time, _is_in_gap, _is_approached_line

  if slope is None:  # When the were no args
    slope = robot.linetrace_slope
  if slope is None:  # When the robot could not find an appropriate slope
    if time.time() - robot.last_slope_get_time > consts.RESCUE_FLAG_TIME:
      robot.write_is_rescue_flag(True)
      robot.write_linetrace_slope(None)
      robot.write_line_area(0)
      return 1500, 1500
    # Reset PID state when line is lost
    if _is_in_gap:
      global _is_approached_line
      _is_approached_line = True
    _pid_prev_error = 0.0
    _pid_integral = 0.0
    _pid_prev_time = None
    return BASE_SPEED, BASE_SPEED
  robot.write_last_slope_get_time(time.time())

  is_centered = (robot.line_center_x is not None
                 and abs(robot.line_center_x -
                         (consts.LINETRACE_CAMERA_LORES_WIDTH // 2)) < 60)

  is_short_line = (robot.line_area is not None and robot.line_area
                   < consts.LINE_RECOVERY_AREA_THRESHOLD * 3)

  if _is_in_gap and (not _is_approached_line):
    _pid_prev_error = 0.0
    _pid_integral = 0.0
    _pid_prev_time = None
    return BASE_SPEED, BASE_SPEED

  # Gap recovery complete: line reacquired after angle correction → clear gap state
  if _is_in_gap and _is_approached_line:
    _is_in_gap = False
    _is_approached_line = False

  if is_centered and is_short_line:
    _pid_prev_error = 0.0
    _pid_integral = 0.0
    _pid_prev_time = None
    return BASE_SPEED, BASE_SPEED

  assert is_valid_number(slope), str(slope)
  angle = math.atan(slope)
  if angle < 0:
    angle += math.pi

  local_angle_error = angle - (math.pi / 2)

  # PID calculation
  now = time.time()
  dt = (now - _pid_prev_time) if _pid_prev_time is not None else 0.0
  # Clamp dt to avoid spikes after long pauses (e.g. green-mark turn)
  dt = min(dt, 0.1)

  # Integral term with anti-windup
  _pid_integral += local_angle_error * dt
  _pid_integral = max(-INTEGRAL_MAX, min(INTEGRAL_MAX, _pid_integral))
  # logger.info(f"PID integral: {_pid_integral:.6f}")

  # Derivative term (rate of error change)
  derivative = ((local_angle_error - _pid_prev_error) / dt) if dt > 0 else 0.0

  steering = int(KP * local_angle_error + KI * _pid_integral + KD * derivative)

  _pid_prev_error = local_angle_error
  _pid_prev_time = now

  # Calculate speed adjustment based on line area
  local_line_area = robot.line_area
  speed_multiplier = 1.0  # Default: full speed

  if local_line_area is not None and is_valid_number(local_line_area):
    # Reduce speed when line gets smaller
    # Area thresholds:
    # > 3000: full speed (100%)
    # 300-3000: power curve for realistic gradual ramp-up
    # < 300: clamped to minimum (30%)
    if local_line_area < 3000:
      # Power function (quadratic) for realistic response curve
      # Normalized to 0-1 range, then apply pow(x,2) for aggressive acceleration
      normalized = (local_line_area - consts.MIN_BLACK_LINE_AREA) / (
          3000 - consts.MIN_BLACK_LINE_AREA)
      power_curve = normalized**2  # Quadratic gives aggressive ramp
      # Scale to 0.3-1.0 range
      speed_multiplier = 0.3 + power_curve * 0.7
      speed_multiplier = max(0.3, min(1.0, speed_multiplier))
      logger.info(
          f"Line area: {local_line_area:.0f}, speed multiplier: {speed_multiplier:.2f}"
      )

  # Get gyro roll angle and reduce speed when tilted significantly
  gyro_roll = math.radians(robot.roll) if robot.roll is not None else None
  gyro_pitch = math.radians(robot.pitch) if robot.pitch is not None else None
  gyro_calculated = (math.degrees(
      math.acos(math.cos(gyro_roll) * math.cos(gyro_pitch))) if
                     gyro_roll is not None and gyro_pitch is not None else None)
  gyro_multiplier = 1.0 if gyro_calculated is None or gyro_calculated < 15 else 1.0

  logger.debug(
      f"Angle info: yaw={robot.yaw} roll={robot.roll}, pitch={robot.pitch}, calculated={gyro_calculated}, multiplier={gyro_multiplier:.2f}"
  )

  # Apply speed multiplier only to the increment above 1500 (stop position)
  # 1500 = stop, so we only reduce the forward speed component
  adjusted_base_speed = 1500 + int(
      (BASE_SPEED - 1500) * speed_multiplier * gyro_multiplier)

  # logger.info(f"Current adjusted speed: {clamp(int(adjusted_base_speed - abs(local_angle_error)**6 * DP), 1500, 2000)}")

  # Reduce the speed when the robot on slope and when it tries to go downside
  l_multi = 1
  r_multi = 1
  if robot.pitch < -10:
    if local_angle_error > 0:
      l_multi = 0.3
      r_multi = 0.3
    else:
      l_multi = 1.5
      r_multi = 1.5
  elif robot.pitch > 10:
    if local_angle_error < 0:
      l_multi = 0.3
      r_multi = 0.3
    else:
      l_multi = 1.5
      r_multi = 1.5

  if robot.roll < -10:
    l_multi = 0.6
    r_multi = 0.6

  decel_speed = clamp(int(adjusted_base_speed - abs(local_angle_error)**7 * DP),
                      1500, 2000)
  motor_l = clamp(int(decel_speed - steering * gyro_multiplier * l_multi),
                  MIN_SPEED, MAX_SPEED)
  motor_r = clamp(int(decel_speed + steering * gyro_multiplier * r_multi),
                  MIN_SPEED, MAX_SPEED)

  return motor_l, motor_r


def approached_exact_angle() -> None:
  angle = robot.line_skeleton_angle

  if angle is not None and is_valid_number(angle):
    angle_deg = math.degrees(angle)
    if abs(angle_deg - 90) < 5:
      return
    else:
      turn_duration = consts.TURN_90_TIME / 90 * abs(90 - abs(angle))
      if 90 - abs(angle_deg) > 0:
        robot.set_speed(1600, 1400)
        sleep_sec(turn_duration)
      else:
        robot.set_speed(1400, 1600)
        sleep_sec(turn_duration)
      robot.set_speed(1500, 1500)
      return


def signal_handler(sig, frame):
  """Handle SIGINT (Ctrl+C) for graceful shutdown.

  Stops the robot motors and exits the program cleanly.

  Args:
    sig: Signal number received.
    frame: Current stack frame (unused).
  """
  logger.info("Received shutdown signal")
  logger.info("received signal: " + str(sig))
  logger.info("frame: " + str(frame))
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


def update_ball_flags(dist_offset: float, y_center: float, w: float,
                      size: float) -> None:
  is_bottom_third = y_center > BALL_Y_2_3
  is_bottom_sixth = y_center > BALL_Y_5_6

  if dist_offset is not None:
    ball_x_center = dist_offset + RESCUE_CX
    half_w = w / 2
    margin = w * 0.1
    ball_left_bound = ball_x_center - half_w + margin
    ball_right_bound = ball_x_center + half_w - margin
    includes_center = ball_left_bound <= RESCUE_CX <= ball_right_bound
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
    return False, max_area, dist, area, y_center, w

  return True, area, dist, area, y_center, w


def draw_ball_debug(image) -> None:
  """
    Draw debug lines used in update_ball_flags():
      - Vertical thresholds (2/3, 5/6 of image height)
      - Horizontal catch tolerance band (ball width)
    """
  hline_color = (0, 255, 0)
  center_color = (0, 0, 255)
  cx = int(RESCUE_CX)
  y_2_3 = int(BALL_Y_2_3)
  y_5_6 = int(BALL_Y_5_6)
  status_text = (f"near:{robot.ball_near_flag} "
                 f"offset:{robot.ball_catch_offset_flag} "
                 f"dist:{robot.ball_catch_dist_flag}")
  cv2.putText(image, status_text, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
              (0, 255, 255), 2, cv2.LINE_AA)
  cv2.line(image, (0, y_2_3), (RESCUE_IMAGE_WIDTH, y_2_3), hline_color, 2)
  cv2.line(image, (0, y_5_6), (RESCUE_IMAGE_WIDTH, y_5_6), hline_color, 2)
  cv2.line(image, (cx, 0), (cx, RESCUE_IMAGE_HEIGHT), center_color, 1)


def run_yolo() -> None:
  with yolo_lock.gen_wlock():
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
  robot.write_rescue_boxes(
      yolo_results[0].boxes if yolo_results and len(yolo_results) > 0 else None)


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
  boxes = robot.rescue_boxes
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
        if cls == robot.target_before_exit:
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
      elif consts.TargetList.EXIT.value == robot.rescue_target and cls == consts.TargetList.SILVER_BALL.value:
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
  robot.set_speed(1500, 1500)
  robot.send_speed()
  robot.set_arm(1540, 0)
  robot.send_arm()
  sleep_sec(1)
  robot.set_speed(1750, 1750)
  sleep_sec(1)
  robot.set_speed(1500, 1500)
  robot.send_speed()
  robot.set_arm(920, 0)
  robot.send_arm()
  robot.set_speed(1250, 1250)
  sleep_sec(1)
  robot.set_speed(1500, 1500)
  robot.send_speed()
  robot.set_arm(920, 1)
  robot.send_arm()
  sleep_sec(0.5)
  robot.set_arm(3030, 1)
  robot.send_arm()
  sleep_sec(0.3)
  robot.set_arm(3030, 1)
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
  robot.set_speed(1600, 1600)
  sleep_sec(1.5)
  robot.set_speed(1500, 1500)
  robot.send_speed()
  robot.set_speed(1350, 1350)
  sleep_sec(0.3)
  robot.set_speed(1500, 1500)
  robot.send_speed()
  robot.set_arm(1700, 0)
  robot.send_arm()
  robot.set_arm(1700, 0)
  robot.send_arm()
  sleep_sec(1.5)
  robot.set_speed(1300, 1300)
  sleep_sec(0.7)
  robot.set_arm(3030, 0)
  robot.send_arm()
  sleep_sec(0.5)
  robot.set_speed(1750, 1250)
  sleep_sec(consts.TURN_90_TIME)
  robot.set_speed(1500, 1500)
  robot.send_speed()
  return True


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
  if robot.rescue_turning_angle >= 720 or (robot.rescue_turning_angle >= 360 and
                                           (not robot.detect_black_ball)):
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
  if robot.ball_catch_dist_flag:
    if robot.ball_catch_offset_flag:
      if robot.ball_near_flag:
        dist_term = -100
      else:
        return 1500, 1500
    else:
      diff_angle = 130 if diff_angle > 0 else -130
      base_L = 1500 + diff_angle
      base_R = 1500 - diff_angle
      logger.info(f"Override Motor Speed L{base_L} dist:{base_R}")
      robot.set_speed(base_L, base_R)
      sleep_sec(0.3)
      diff_angle = 0
      dist_term = 0
  else:
    dist_term = (math.sqrt(consts.BALL_CATCH_SIZE) - math.sqrt(size)) * BSP
    dist_term = int(max(0.0, min(dist_term, 150)))
  base_L = 1500 + diff_angle + dist_term
  base_R = 1500 - diff_angle + dist_term
  base_L = int(base_L)
  base_R = int(base_R)
  # logger.info(f"offset: {angle} size:{size}")
  # logger.info(f"diff_angle: {diff_angle} dist_term {dist_term}")
  # logger.info(f"Motor speed L{base_L} R{base_R}")
  base_L, base_R = clamp(base_L, 1300, 1750), clamp(base_R, 1300, 1750)
  logger.info(f"Clamped Motor Speeds L{base_L} R{base_R}")
  logger.info(
      f"catch offset:{robot.ball_catch_offset_flag} dist:{robot.ball_catch_dist_flag} near:{robot.ball_near_flag}"
  )
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


def r_wall_follow_ccw() -> bool:
  ultrasonic = robot.ultrasonic
  l_dist = ultrasonic[0]
  front_dist = robot.avg_ultrasonic[1]

  if front_dist is None or front_dist < 0:
    logger.warning("Front sensor not responding - Jiggling (Left Follow)...")
    while True:
      robot.set_speed(1700, 1700)
      if sleep_sec(0.4) == 1:
        return False
      robot.set_speed(1300, 1300)
      if sleep_sec(0.4) == 1:
        return False
      new_front = robot.ultrasonic[1]
      if new_front is not None and new_front > 0:
        logger.info(f"Front sensor recovered: {new_front}")
        robot.set_speed(1500, 1500)
        robot.send_speed()
        front_dist = new_front
        break
      logger.info("Still no response from front sensor...")

  if l_dist is not None and l_dist > consts.OPEN_THRESHOLD or l_dist == 0:
    logger.info("Wall opening detected (Left) - Exiting control")
    return True

  if l_dist is None or l_dist < 0:
    robot.set_speed(1500, 1500)
    robot.send_speed()
    logger.info("The left ultrasonic sensor is not responding.")
    return False

  if front_dist <= consts.FRONT_FLAG_DIST:
    robot.set_speed(1750, 1250)
    sleep_sec(consts.TURN_90_TIME * 1.3)
    robot.set_speed(1500, 1500)
    robot.send_speed()
    sleep_sec(0.2)
    return False
  target_dist = consts.TARGET_MIN
  error = l_dist - target_dist
  kp = 20.0
  turn = int(error * kp)
  max_turn = consts.BASE_TURN
  turn = max(-max_turn, min(max_turn, turn))
  turn = 0
  left_speed = BASE_SPEED - turn
  right_speed = BASE_SPEED + turn

  left_speed, right_speed = clamp(left_speed), clamp(right_speed)
  robot.set_speed(left_speed, right_speed)
  logger.info(
      f"L-Dist: {l_dist:.1f}, Target: {target_dist}, L:{left_speed} R:{right_speed}"
  )
  robot.send_speed()

  return False


def l_wall_follow_ccw() -> bool:
  ultrasonic = robot.ultrasonic
  front_dist = robot.avg_ultrasonic[1]
  r_dist = ultrasonic[2]
  if front_dist is None or front_dist <= 0:
    logger.warning("Front sensor not responding - Jiggling...")
    while True:
      robot.set_speed(1700, 1700)
      if sleep_sec(0.4) == 1:
        return False
      robot.set_speed(1300, 1300)
      if sleep_sec(0.4) == 1:
        return False
      new_front = robot.ultrasonic[1]
      if new_front is not None and new_front > 0:
        logger.info(f"Front sensor recovered: {new_front}")
        robot.set_speed(1500, 1500)
        robot.send_speed()
        front_dist = new_front
        break

  if r_dist is not None and r_dist > consts.OPEN_THRESHOLD or r_dist == 0:
    logger.info("Wall opening detected (Right) - Exiting control")
    return True

  if r_dist is None or r_dist < 0:
    robot.set_speed(1500, 1500)
    robot.send_speed()
    logger.info("The side ultrasonic sensor is not responding.")
    return False

  if front_dist <= consts.FRONT_FLAG_DIST:
    robot.set_speed(1250, 1750)
    sleep_sec(consts.TURN_90_TIME * 1.3)
    robot.set_speed(1500, 1500)
    robot.send_speed()
    sleep_sec(0.2)
    return False
  target_dist = consts.TARGET_MIN
  error = r_dist - target_dist
  kp = 20.0
  turn = int(error * kp)
  max_turn = consts.BASE_TURN
  turn = max(-max_turn, min(max_turn, turn))
  turn = 0
  left_speed = BASE_SPEED + turn
  right_speed = BASE_SPEED - turn
  left_speed, right_speed = clamp(left_speed), clamp(right_speed)
  robot.set_speed(left_speed, right_speed)
  logger.info(
      f"R-Dist: {r_dist:.1f}, Target: {target_dist}, L:{left_speed} R:{right_speed}"
  )
  robot.send_speed()

  return False


hasFoundExit = -1


def find_cage() -> Optional[int]:
  boxes = robot.rescue_boxes
  if boxes is None or len(boxes) == 0:
    logger.info("Target not found")
    return None

  min_area = consts.IMAGE_SZ * 0.4
  min_y = RESCUE_IMAGE_HEIGHT * 0.5
  cage_classes = [
      consts.TargetList.GREEN_CAGE.value, consts.TargetList.RED_CAGE.value
  ]

  for box in boxes:
    try:
      cls = int(box.cls[0])
      logger.info(f"detected class {consts.TargetList(cls)}")
      if cls not in cage_classes:
        continue
      _, y_center, w, h = map(float, box.xywh[0])
      area = w * h
      logger.info(f"area: {area} cy: {y_center}")
    except Exception as e:
      logger.exception(f"Error processing cage detection: {e}")
      continue

    if area >= min_area and y_center >= min_y:
      return cls

  return None


def handle_before_search() -> None:
  global hasFoundExit
  run_yolo()
  cage_class = find_cage()
  if cage_class is not None:
    logger.info(f"Exit {hasFoundExit} Cage{consts.TargetList(cage_class).name}")
    robot.write_target_before_exit(cage_class)
    robot.set_speed(1500, 1500)
    robot.send_speed()
    return
  if hasFoundExit == -1:
    robot.set_speed(1500, 1500)
    robot.send_speed()
    robot.set_speed(1340, 1380)
    sleep_sec(3.5)
    robot.set_speed(1750, 1250)
    sleep_sec(consts.TURN_90_TIME * 0.8)
    robot.set_speed(1500, 1500)
    robot.send_speed()
    robot.set_speed(1650, 1650)
    prev_time = time.time()
    while time.time() - prev_time < 2.5:
      robot.update_button_stat()
      robot.send_speed()
      if robot.robot_stop:
        robot.set_speed(1500, 1500)
        robot.send_speed()
        logger.info("Sleep interrupted by button")
        break
      if robot.ultrasonic[1] < consts.FRONT_FLAG_DIST:
        run_yolo()
        cage_class = find_cage()
        if cage_class is not None:
          logger.info(
              f"Exit:{hasFoundExit} Cage:{consts.TargetList(cage_class).name}")
          robot.write_target_before_exit(cage_class)
        robot.set_speed(1500, 1500)
        robot.send_speed()
        robot.set_speed(1250, 1750)
        sleep_sec(consts.TURN_90_TIME * 1.2)
        robot.set_speed(1500, 1500)
        robot.send_speed()
        break
      if (robot.linetrace_slope
          is not None) and (robot.line_area
                            >= consts.MIN_OBJECT_AVOIDANCE_LINE_AREA):
        hasFoundExit = 1
        robot.set_speed(1300, 1300)
        sleep_sec(2)
        robot.set_speed(1250, 1750)
        sleep_sec(consts.TURN_90_TIME * 1.3)
        robot.set_speed(1500, 1500)
        robot.send_speed()
        robot.set_speed(1650, 1650)
        sleep_sec(2.5)
        break
    robot.set_speed(1500, 1500)
    robot.send_speed()
    hasFoundExit += 1
  # PIN:
  result = r_wall_follow_ccw()
  if result:
    hasFoundExit = 1
    robot.set_speed(1650, 1650)
    sleep_sec(2.5)
  if (robot.linetrace_slope
      is not None) and (robot.line_area
                        >= consts.MIN_OBJECT_AVOIDANCE_LINE_AREA):
    hasFoundExit = 1
    robot.set_speed(1300, 1300)
    sleep_sec(2)
    robot.set_speed(1250, 1750)
    sleep_sec(consts.TURN_90_TIME * 1.3)
    robot.set_speed(1650, 1650)
    sleep_sec(2.5)


def handle_not_found() -> None:
  change_position()
  # Only call set_target() if searching for balls (rotation-based logic).
  # For cages/exit, keep searching the current target.
  if robot.rescue_target in [
      consts.TargetList.SILVER_BALL.value, consts.TargetList.BLACK_BALL.value
  ]:
    robot.write_rescue_turning_angle(robot.rescue_turning_angle + 18)
    set_target()

has_found_exit = False
exit_angle = None

def handle_exit() -> None:
  global has_found_exit, exit_angle
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
        robot.set_speed(1600, 1600)
        sleep_sec(1)
        robot.set_speed(1300, 1300)
        sleep_sec(0.5)
        robot.set_speed(1500, 1500)
        robot.send_speed()
        # if hasFoundExit > 0:
        robot.set_speed(1750, 1250)
        # else:
        #   robot.set_speed(1250, 1750)
        sleep_sec(consts.TURN_90_TIME * 2)
        robot.set_speed(2000, 2000)
        sleep_sec(1.0)
        robot.set_speed(1500, 1500)
        robot.send_speed()
        robot.send_speed()
        robot.write_has_moved_to_cage(True)
        robot.write_linetrace_slope(None)
        robot.write_target_before_exit(consts.TargetList.EXIT.value)
        robot.write_line_area(0)
  else:
    # logger.info("wall follow ccw")
    if (not has_found_exit) and robot.rescue_offset is None:
      change_position()
      return
    else:
      if not has_found_exit:
        if robot.rescue_offset >= 0:
          exit_angle = "R"
        else:
          exit_angle = "L"
      has_found_exit = True
      motorl, motorr = calculate_cage()
      robot.set_speed(motorl,motorr)
      robot.send_speed()
      if robot.rescue_offset is None:
        if exit_angle == "R":
          robot.set_speed(1680, 1600)
        else:
          robot.set_speed(1600, 1680)
        robot.send_speed()
      if (robot.linetrace_slope
        is not None) and (robot.line_area
                          >= consts.MIN_OBJECT_AVOIDANCE_LINE_AREA):
        logger.info("Line detected, exit rescue mode")
        robot.set_speed(1600, 1600)
        sleep_sec(1.0)
        robot.set_speed(1500, 1500)
        robot.send_speed()
        robot.write_is_rescue_flag(False)


def handle_ball() -> None:
  clamp_turning_angle()
  last_offset_flag = robot.ball_catch_offset_flag
  last_dist_flag = robot.ball_catch_dist_flag
  last_near_flag = robot.ball_near_flag
  motorl, motorr = calculate_ball()
  robot.set_speed(motorl, motorr)
  robot.send_speed()
  if last_offset_flag and last_dist_flag and (not last_near_flag):  # Catch
    robot.set_speed(1500, 1500)
    robot.send_speed()
    while time.time() < robot.rescue_saved_time:
      robot.update_button_stat()
      if robot.robot_stop:
        robot.set_speed(1500, 1500)
        robot.send_speed()
        return
      robot.send_speed()
    run_yolo()
    find_best_target()
    if not (robot.ball_catch_offset_flag and robot.ball_catch_dist_flag and
            (not robot.ball_near_flag)):
      return
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
  if robot.rescue_size is not None and robot.rescue_size >= consts.IMAGE_SZ * 0.6 and robot.rescue_y is not None and robot.rescue_y > (
      robot.rescue_image.shape[0] * 1 / 2):
    release_ball()
    set_target()


def reset_pid_state() -> None:
  """Reset PID state variables (call when switching modes / stopping)."""
  global _pid_prev_error, _pid_integral, _pid_prev_time
  _pid_prev_error = 0.0
  _pid_integral = 0.0
  _pid_prev_time = None


def is_stopping_by_button() -> None:
  global hasFoundExit, _is_in_gap, _is_approached_line, has_found_exit
  _is_in_gap = False
  _is_approached_line = False
  has_found_exit = False
  if robot.target_before_exit == -1:
    hasFoundExit = -1
  robot.set_speed(1500, 1500)
  robot.set_arm(3030, 0)
  robot.send_speed()
  robot.send_arm()
  robot.write_rescue_turning_angle(0)
  robot.write_rescue_target(consts.TargetList.SILVER_BALL.value)
  logger.info("robot stop true, stopping..")
  robot.write_linetrace_stop(False)
  robot.write_is_rescue_flag(False)
  robot.write_last_slope_get_time(time.time())
  robot.write_ball_catch_dist_flag(False)
  robot.write_ball_catch_offset_flag(False)
  robot.write_ball_near_flag(False)
  robot.write_has_moved_to_cage(False)
  robot.write_detect_black_ball(False)
  from modules.camera import reset_green_tracker
  reset_green_tracker()
  robot.write_green_turn_direction(None)
  reset_pid_state()


logger.info("Objects Initialized")

if __name__ == "__main__":
  assert isinstance(robot, modules.robot.Robot)
  # Register signal handler for graceful shutdown
  signal.signal(signal.SIGINT, signal_handler)

  logger.info("Starting program")
  robot.set_speed(1500, 1500)
  robot.set_arm(3030, 0)
  robot.send_arm()
  robot.send_speed()
  robot.write_linetrace_stop(False)
  robot.write_is_rescue_flag(False)
  robot.write_last_slope_get_time(time.time())
  robot.write_rescue_target(consts.TargetList.SILVER_BALL.value)
  robot.write_target_before_exit(consts.TargetList.GREEN_CAGE.value)
  while True:
    robot.update_button_stat()
    robot.update_gyro_stat()
    ultrasonic_info = robot.avg_ultrasonic
    if robot.robot_stop:
      is_stopping_by_button()
    elif robot.is_rescue_flag:
      try:
        logger.info(
            f"Searching for target: {consts.TargetList(robot.rescue_target).name} (id={robot.rescue_target})"
        )
      except Exception:
        logger.info(f"Searching for target id: {robot.rescue_target}")
      if False:
        handle_before_search()
      else:
        run_yolo()
        find_best_target()
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
        logger.info(f"{_pid_integral}")
        # Check for green mark intersections before normal line following
        # logger.info(ultrasonic_info)
        if robot.green_turn_direction == 'u':
          # U-turn: marks on both sides → dead end → physical 180° turn
          logger.info("Green U-turn detected — executing 180° turn")
          execute_green_uturn()
          from modules.camera import reset_green_tracker
          reset_green_tracker()
          robot.write_green_turn_direction(None)
        elif robot.green_turn_direction in ('l', 'r'):
          # Green turn is handled by camera binary image modification.
          # The modified binary shows only the desired path, so normal
          # line-following PID naturally steers through the turn.
          # Slow down for precise turning.
          motorl, motorr = calculate_motor_speeds()
          slowdown = consts.GREEN_AHEAD_SLOWDOWN_SPEED
          motorl = min(motorl, slowdown)
          motorr = min(motorr, slowdown)
          robot.set_speed(motorl, motorr)
          # Limit integral windup during speed-capped turns: allow some
          # integral for steady-state correction but prevent full buildup.
          _pid_integral = max(-0.25, min(0.25, _pid_integral))
          logger.info("Green turn — camera steering active, slowing down")
        elif robot.green_ahead:
          # Green mark detected ahead along line direction — slow down to prepare
          motorl, motorr = calculate_motor_speeds()
          slowdown = consts.GREEN_AHEAD_SLOWDOWN_SPEED
          motorl = min(motorl, slowdown)
          motorr = min(motorr, slowdown)
          robot.set_speed(motorl, motorr)
          logger.info(
              f"Green ahead — slowing down (dist={robot.green_ahead_distance:.0f})"
          )
        elif ultrasonic_info[1] <= 3 and ultrasonic_info[
            1] != -1 and ultrasonic_info[1] != 0:
          logger.info("Object avoidance triggered")
          robot.set_speed(1400, 1400)
          sleep_sec(1, robot.send_speed)
          robot.set_speed(1750, 1250)
          sleep_sec(1.5, robot.send_speed)
          robot.set_speed(1580, 1800)
          sleep_sec(1, robot.send_speed)
          object_avoidance_start = time.time()
          while robot.linetrace_slope is None:
            if time.time(
            ) - object_avoidance_start >= 2 and robot.line_area <= consts.MIN_OBJECT_AVOIDANCE_LINE_AREA:
              break
            logger.info("Turning around in object avoidance...")
            robot.write_last_slope_get_time(time.time())
            robot.set_speed(1590, 1790)
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
          robot.write_linetrace_stop(False)
        else:
            motorl, motorr = calculate_motor_speeds()
            robot.set_speed(motorl, motorr)
      else:
        logger.info("Red stop")
        robot.set_speed(1500, 1500)
    robot.send_speed()
logger.info("Program Stop")
