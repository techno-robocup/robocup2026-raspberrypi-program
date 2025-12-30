import modules.constants as consts
import modules.logger
import modules.robot
import time
import signal
import sys
import cv2
import math
from typing import Optional
from ultralytics import YOLO

logger = modules.logger.get_logger()

logger.debug("Logger initialized")

robot = modules.robot.robot
uart_dev = modules.robot.uart_io()
uart_devices = uart_dev.list_ports()

# Prioritize USsB devices (ESP32 typically appears as /dev/ttyUSB* or /dev/ttyACM*)
usb_devices = [
    d for d in uart_devices if 'USB' in d.device or 'ACM' in d.device
]
selected_device = usb_devices[0] if usb_devices else uart_devices[0]

logger.info(f"Connecting to UART device: {selected_device.device}")
uart_dev.connect(selected_device.device, consts.UART_BAUD_RATE,
                 consts.UART_TIMEOUT)
robot.set_uart_device(uart_dev)

BASE_SPEED = 1730
TURNING_BASE_SPEED = 1650
assert 1500 < BASE_SPEED < 2000
assert 1500 < TURNING_BASE_SPEED < 2000
assert TURNING_BASE_SPEED < BASE_SPEED
MAX_SPEED = 2000
MIN_SPEED = 1000
KP = 230
DP = 160
BOP = 0.05  # Ball Offset P
BSP = 3  # Ball Size P
COP = 0.3  # Cage Offset P
EOP = 1  # Exit Offset P

catch_failed_cnt = 0

def is_valid_number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)

def clamp(value: int, min_val: int = MIN_SPEED, max_val: int = MAX_SPEED) -> int:
  """Clamp value between min and max."""
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

    if detection[2] == 1:  # Left blackfith
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
  robot.set_speed(1350, 1350)
  robot.send_speed()
  time.sleep(0.8)
  robot.set_speed(1500, 1500)
  robot.send_speed()
  return True  # Completed successfully


def should_execute_line_recovery(line_area: Optional[float], angle_error: float) -> bool:
  """
  Check if line recovery should be executed.
  
  Recovery triggers when BOTH conditions are met:
  1. Line area is below threshold (robot losing sight of line)
  2. Angle error is steep (robot is at a significant angle to the line)
  
  Args:
    line_area: Current detected line area in pixels
    angle_error: Current angle error from vertical (radians)
  
  Returns:
    True if recovery should be executed
  """
  if line_area is None or not is_valid_number(line_area):
    return False
  
  area_condition = line_area < consts.LINE_RECOVERY_AREA_THRESHOLD
  angle_condition = abs(angle_error) > consts.LINE_RECOVERY_ANGLE_THRESHOLD
  
  return area_condition and angle_condition


def execute_line_recovery() -> bool:
  """
  Execute line recovery by backing up to regain line visibility.
  
  When the robot is losing the line at a steep angle, this function
  backs up for a short duration to allow the robot to re-acquire the line.
  
  Returns:
    True if recovery completed successfully
    False if interrupted by button
  """
  logger.info("Executing line recovery - backing up")
  
  start_time = time.time()
  while time.time() - start_time < consts.LINE_RECOVERY_BACKUP_TIME:
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
  if slope is None: # When the were no args
    slope = robot.linetrace_slope
  if slope is None: # When the robot could not find an appropriate slope
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
    # > 1000: full speed (100%)
    # 500-1000: gradual reduction
    # < 500: significant reduction (60-80%)
    if line_area < 1000:
      # Linear interpolation between 0.6 (at area=300) and 1.0 (at area=1000)
      speed_multiplier = 0.2 + (line_area - consts.MIN_BLACK_LINE_AREA) / (1000 - consts.MIN_BLACK_LINE_AREA) * 0.8
      speed_multiplier = max(0.6, min(1.0, speed_multiplier))
      logger.info(f"Line area: {line_area:.0f}, speed multiplier: {speed_multiplier:.2f}")

  # Apply speed multiplier only to the increment above 1500 (stop position)
  # 1500 = stop, so we only reduce the forward speed component
  adjusted_base_speed = 1500 + int((BASE_SPEED - 1500) * speed_multiplier)
  
  motor_l = clamp(
      clamp(int(adjusted_base_speed - abs(angle_error)**6 * DP), 1500, 2000) - steering,
      MIN_SPEED, MAX_SPEED)
  motor_r = clamp(
      clamp(int(adjusted_base_speed - abs(angle_error)**6 * DP), 1500, 2000) + steering,
      MIN_SPEED, MAX_SPEED)

  return motor_l, motor_r


def signal_handler(sig, frame):
  """Handle SIGINT for graceful shutdown."""
  logger.info("Received shutdown signal")
  robot.set_speed(1500, 1500)
  robot.send_speed()
  sys.exit(0)

def sleep_sec(sec: float, function = None) -> int:
  """Sleep for the specified number of seconds, checking for robot stop."""
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
  yolo_results = None
  if time.time() - robot.last_yolo_time > 0.1:
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
    logger.debug("Target not found")
    robot.write_rescue_offset(None)
    robot.write_rescue_size(None)
    return
  boxes = yolo_results[0].boxes
  if boxes is None or len(boxes) == 0:
    logger.debug("Target not found")
    robot.write_rescue_offset(None)
    robot.write_rescue_size(None)
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
        logger.debug(
            f"Detected cls={consts.TargetList(cls).name}, area={area:.1f}, offset={dist:.1f}"
        )
      elif consts.TargetList.BLACK_BALL.value == robot.rescue_target and cls == consts.TargetList.SILVER_BALL.value:
        logger.debug("Override")
        robot.write_rescue_turning_angle(0)
        x_center, y_center, w, h = map(float, box.xywh[0])
        dist = x_center - cx
        area = w * h
        if abs(dist) < min_dist:
          min_dist = abs(dist)
          best_angle = dist
          best_size = area
          best_target_y = y_center
          best_target_w = w
          # best_target_h = h
        robot.write_rescue_target(consts.TargetList.SILVER_BALL.value)
        logger.debug(
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


def catch_ball() -> int:
  logger.debug("Executing catch_ball()")
  # Store which ball type we're catching
  logger.info(
      f"Caught ball type: {consts.TargetList(robot.rescue_target).name}")
  robot.set_speed(1500, 1500)
  robot.send_speed()
  robot.set_speed(1400 , 1400)
  sleep_sec(0.8)
  robot.set_speed(1500, 1500)
  robot.send_speed()
  robot.set_arm(1400, 0)
  robot.send_arm()
  robot.set_speed(1650, 1650)
  sleep_sec(2)
  robot.set_speed(1500, 1500)
  robot.send_speed()
  robot.set_arm(1000, 0)
  robot.send_arm()
  robot.set_speed(1600, 1600)
  sleep_sec(2)
  robot.set_speed(1400,1400)
  sleep_sec(1)
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


def release_ball() -> int:
  logger.debug("Executing release_ball()")
  robot.set_speed(1700, 1700)
  sleep_sec(2.2)
  robot.set_speed(1500, 1500)
  robot.send_speed()
  robot.set_speed(1400, 1400)
  sleep_sec(0.5)
  robot.set_speed(1500, 1500)
  robot.set_arm(1536, 0)
  robot.send_speed()
  sleep_sec(1.5)
  robot.set_arm(3072, 0)
  sleep_sec(0.5)
  robot.set_speed(1400, 1400)
  sleep_sec(1)
  robot.set_speed(1750, 1250)
  sleep_sec(consts.TURN_180_TIME)
  robot.set_speed(1500, 1500)
  robot.send_speed()
  set_target()
  return True


def change_position() -> bool:
  logger.debug("Change position")
  prev_time = time.time()
  robot.set_speed(1750, 1250)
  sleep_sec(consts.TURN_30_TIME)
  robot.set_speed(1500, 1500)
  sleep_sec(0.2)
  find_best_target()
  # robot.write_rescue_turning_angle(robot.rescue_turning_angle + 30)
  # logger.info(f"Turn degrees{robot.rescue_turning_angle}")
  return True  # Completed successfully


def set_target() -> bool:
  if robot.rescue_turning_angle is None:
    robot.write_rescue_turning_angle(0)
    return False
  if robot.rescue_turning_angle > 720:
    robot.write_rescue_target(consts.TargetList.EXIT.value)
    # robot.write_rescue_target(consts.TargetList.SILVER_BALL.value)
  elif robot.rescue_turning_angle > 360:
    robot.write_rescue_target(consts.TargetList.BLACK_BALL.value)
  else:
    robot.write_rescue_target(consts.TargetList.SILVER_BALL.value)
  return True


def calculate_ball() -> tuple[int, int]:
  angle = robot.rescue_offset
  size = robot.rescue_size
  if angle is None or size is None:
    return 1500, 1500
  diff_angle = 0
  if abs(angle) > 30:
    diff_angle = angle * BOP
  else:
    diff_angle = 0
  dist_term = 0
  if consts.BALL_CATCH_SIZE > size:
    dist_term = (math.sqrt(consts.BALL_CATCH_SIZE) - math.sqrt(size)) * BSP
  dist_term = int(max(60, dist_term))
  base_L = 1500 + diff_angle + dist_term
  base_R = 1500 - diff_angle + dist_term
  base_L = int(base_L)
  base_R = int(base_R)
  logger.info(f"offset: {angle} size:{size}")
  logger.info(f"diff_angle: {diff_angle} dist_term {dist_term}")
  logger.info(f"Motor speed L{base_L} R{base_R}")
  return clamp(int(base_L), MIN_SPEED,
               MAX_SPEED), clamp(int(base_R), MIN_SPEED, MAX_SPEED)


def calculate_cage() -> tuple[int, int]:
  angle = robot.rescue_offset
  size = robot.rescue_size
  if angle is None or size is None:
    return 1500, 1500
  diff_angle = angle * COP
  base_L = 1500 + diff_angle + 150
  base_R = 1500 - diff_angle + 150
  logger.info(f"offset: {angle} size:{size}")
  logger.info(f"Motor speed L{base_L} R{base_R}")
  return clamp(int(base_L), MIN_SPEED,
               MAX_SPEED), clamp(int(base_R), MIN_SPEED, MAX_SPEED)


def calculate_exit() -> tuple[int, int]:
  angle = robot.rescue_offset
  size = robot.rescue_size
  if angle is None or size is None:
    return 1500, 1500
  diff_angle = angle * EOP
  if diff_angle > 0:
    diff_angle = max(diff_angle - 10, 0)
  if diff_angle < 0:
    diff_angle = min(diff_angle + 10, 0)  # TODO(K10-K10):Fix value
  base_L = 1500 + diff_angle + 150
  base_R = 1500 - diff_angle + 150
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
  # Register signal handler for graceful shutdown
  signal.signal(signal.SIGINT, signal_handler)

  logger.info("Starting program")
  robot.set_speed(1500, 1500)
  robot.set_arm(3072, 0)
  robot.send_arm()
  robot.send_speed()
  robot.write_rescue_turning_angle(0)
  robot.write_last_slope_get_time(time.time())
  while True:
    robot.update_button_stat()
    if robot.robot_stop:
      robot.set_speed(1500, 1500)
      robot.set_arm(3072, 0)
      robot.send_arm()
      robot.write_rescue_turning_angle(0)
      logger.debug("robot stop true, stopping..")
      robot.write_linetrace_stop(False)
      robot.write_is_rescue_flag(False)
      robot.write_last_slope_get_time(time.time())
    elif robot.is_rescue_flag:
      find_best_target()
      if (robot.rescue_offset is None) or (robot.rescue_size is None):
        change_position()
        robot.write_rescue_turning_angle(robot.rescue_turning_angle + 30)
        set_target()
      else:
        if robot.rescue_target == consts.TargetList.EXIT.value:
          motorl, motorr = calculate_exit()
          robot.set_speed(motorl, motorr)
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
              robot.write_rescue_target(consts.TargetList.GREEN_CAGE)
            if robot.rescue_target == consts.TargetList.BLACK_BALL.value:
              robot.write_rescue_target(consts.TargetList.RED_CAGE.value)
        else:
          motorl, motorr = calculate_cage()
          robot.set_speed(motorl, motorr)
          robot.send_speed()
          if robot.rescue_size >= consts.BALL_CATCH_SIZE * 3.8:
            release_ball()
        robot.set_speed(motorl, motorr)
    else:
      if not robot.linetrace_stop:
        ultrasonic_info = robot.ultrasonic
        # Check for green mark intersections before normal line following
        if should_process_green_mark():
          execute_green_mark_turn()
        elif ultrasonic_info[0] <= 3: # TODO: The index is really wired, the return value is including some bug, but not sure what is the problem
          logger.info("Object avoidance triggered")
          robot.set_speed(1400, 1400)
          sleep_sec(1, robot.send_speed)
          robot.set_speed(1750,1250)
          sleep_sec(1.7, robot.send_speed)
          while robot.linetrace_slope is None or robot.line_area <= consts.MIN_OBJECT_AVOIDANCE_LINE_AREA:
            logger.info("Turning around in object avoidance...")
            robot.write_last_slope_get_time(time.time())
            robot.set_speed(1550, 1800)
            robot.send_speed()
            robot.update_button_stat()
            if robot.robot_stop:
              logger.info("Robot interrupted during object avoidance")
              break
          logger.info(f"Ejecting object avoidance by {robot.linetrace_slope} {robot.line_area}")
          robot.set_speed(1600, 1600)
          sleep_sec(1)
        else:
          # Check if line recovery is needed (small line area + steep angle)
          angle_error = get_current_angle_error()
          line_area = robot.line_area
          
          if angle_error is not None and should_execute_line_recovery(line_area, angle_error):
            execute_line_recovery()
          else:
            motorl, motorr = calculate_motor_speeds()
            robot.set_speed(motorl, motorr)
      else:
        logger.info("Red stop")
        robot.set_speed(1500, 1500)
    robot.send_speed()
logger.debug("Program Stop")
