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
uart_dev.connect(uart_devices[0].device, consts.UART_BAUD_RATE,
                 consts.UART_TIMEOUT)
robot.set_uart_device(uart_dev)

BASE_SPEED = 1700
MAX_SPEED = 2000
MIN_SPEED = 1000
KP = 200
DP = 100
P = 0.4
AP = 1
WP = 0.3


def clamp(value: int, min_val: int = 1000, max_val: int = 2000) -> int:
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
  Execute a turn based on detected green marks.

  Turn logic:
  - Both left and right: 180° turn (3.5s)
  - Only left: 90° left turn (1.5s)
  - Only right: 90° right turn (1.5s)

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
  logger.info("Green mark detected - approaching intersection")
  start_time = time.time()
  while time.time() - start_time < consts.GREEN_MARK_APPROACH_TIME:
    robot.update_button_stat()
    if robot.robot_stop:
      robot.set_speed(1500, 1500)
      robot.send_speed()
      logger.info("Turn interrupted by button during approach")
      return False
    robot.set_speed(BASE_SPEED, BASE_SPEED)
    robot.send_speed()

  # Execute turn based on detected directions
  if has_left and has_right:
    logger.info("Green marks: both sides detected - turning 180°")
    start_time = time.time()
    while time.time() - start_time < consts.GREEN_MARK_TURN_180_TIME:
      robot.update_button_stat()
      if robot.robot_stop:
        robot.set_speed(1500, 1500)
        robot.send_speed()
        logger.info("Turn interrupted by button during 180° turn")
        return False
      robot.set_speed(1750, 1250)  # Turn left
      robot.send_speed()
  elif has_left:
    logger.info("Green mark: left side detected - turning 90° left")
    start_time = time.time()
    while time.time() - start_time < consts.TURN_90_LEFT_TIME:
      robot.update_button_stat()
      if robot.robot_stop:
        robot.set_speed(1500, 1500)
        robot.send_speed()
        logger.info("Turn interrupted by button during 90° left")
        return False
      robot.set_speed(1750, 1250)  # Turn left
      robot.send_speed()
  elif has_right:
    logger.info("Green mark: right side detected - turning 90° right")
    start_time = time.time()
    while time.time() - start_time < consts.TURN_90_RIGHT_TIME:
      robot.update_button_stat()
      if robot.robot_stop:
        robot.set_speed(1500, 1500)
        robot.send_speed()
        logger.info("Turn interrupted by button during 90° right")
        return False
      robot.set_speed(1200, 1750)  # Turn right
      robot.send_speed()

  # Stop after turn
  robot.set_speed(1500, 1500)
  robot.send_speed()
  logger.info("Turn complete")
  return True  # Completed successfully


def calculate_motor_speeds(slope: Optional[float] = None) -> tuple[int, int]:
  """
  Calculate left and right motor speeds based on line slope.

  Uses arctan to convert slope to angle, then calculates the difference
  from π/2 (vertical). This gives a normalized angular error for steering.

  Args:
    slope: Line slope value. If None, reads from robot.read_linetrace_slope().

  Angle interpretation:
  - angle = π/2: line is vertical (centered), go straight
  - angle < π/2: line tilts right, turn right
  - angle > π/2: line tilts left, turn left
  """
  if slope is None:
    slope = robot.linetrace_slope

  if slope is None:
    return BASE_SPEED, BASE_SPEED

  angle = math.atan(slope)
  if angle < 0:
    angle += math.pi

  angle_error = angle - (math.pi / 2)

  steering = int(KP * angle_error)

  motor_l = clamp(
      clamp(int(BASE_SPEED - abs(angle_error)**4 * DP), 1500, 2000) - steering,
      MIN_SPEED, MAX_SPEED)
  motor_r = clamp(
      clamp(int(BASE_SPEED - abs(angle_error)**4 * DP), 1500, 2000) + steering,
      MIN_SPEED, MAX_SPEED)

  return motor_l, motor_r


def signal_handler(sig, frame):
  """Handle SIGINT for graceful shutdown."""
  logger.info("Received shutdown signal")
  robot.set_speed(1500, 1500)
  robot.send_speed()
  sys.exit(0)


def find_best_target() -> None:
  logger.debug("Find target")
  yolo_results = consts.MODEL(robot.rescue_image, verbose=False)
  current_time = time.time()
  if yolo_results is None or len(yolo_results) == 0:
    logger.info("Target not found")
    robot.write_rescue_offset(None)
    robot.write_rescue_size(None)
    return
  result_image = yolo_results[0].plot()
  cv2.imwrite(f"bin/{current_time:.3f}_rescue_result.jpg", result_image)
  boxes = yolo_results[0].boxes
  if boxes is None or len(boxes) == 0:
    logger.info("Target not found")
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
    best_target_h = None
    min_dist = float("inf")
    cx = image_width / 2.0
    for box in boxes:
      try:
        cls = int(box.cls[0])
        detected_classes.append(cls)
      except Exception:
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
          best_target_h = h
          if cls == consts.TargetList.BLACK_BALL.value or cls == consts.TargetList.SILVER_BALL.value:
            is_bottom_third = best_target_y and best_target_y > (image_height *
                                                                 3 / 4)
            if best_target_w:
              ball_left = robot.rescue_offset - best_target_w / 2 + image_width / 2
              ball_right = robot.rescue_offset + best_target_w / 2 + image_width / 2
              includes_center = ball_left <= image_width / 2 <= ball_right
            else:
              includes_center = False
            if is_bottom_third and includes_center:
              robot.write_rescue_ball_flag(True)
        logger.info(
            f"Detected cls={consts.TargetList(cls).name}, area={area:.1f}, offset={dist:.1f}"
        )
      elif consts.TargetList.BLACK_BALL.value == robot.rescue_target and cls == consts.TargetList.SILVER_BALL.value:
        logger.info("Override")
        robot.write_rescue_turning_angle(0)
        robot.write_rescue_target(consts.TargetList.SILVER_BALL.value)
        x_center, y_center, w, h = map(float, box.xywh[0])
        dist = x_center - cx
        area = w * h
        if abs(dist) < min_dist:
          min_dist = abs(dist)
          best_angle = dist
          best_size = area
          best_target_y = y_center
          best_target_w = w
          best_target_h = h
        robot.write_rescue_target(consts.TargetList.SILVER_BALL.value)
        logger.info(
            f"Detected cls={consts.TargetList(cls).name}, area={area:.1f}, offset={dist:.1f}"
        )
    robot.write_rescue_offset(best_angle)
    robot.write_rescue_size(best_size)


def catch_ball() -> int:
  logger.debug("Executing catch_ball()")
  # Store which ball type we're catching
  logger.info(
      f"Caught ball type: {consts.TargetList(robot.rescue_target).name}")
  robot.set_speed(1500, 1500)
  robot.set_arm(1400, 0)
  robot.send_speed()
  robot.send_arm()
  robot.set_speed(1650, 1650)
  prev_time = time.time()
  while time.time() - prev_time < 2:
    robot.update_button_stat()
    if robot.robot_stop:
      robot.set_speed(1500, 1500)
      robot.send_speed()
      logger.info("Catch interrupted by button")
      return 1
    robot.send_speed()
  robot.set_speed(1500, 1500)
  robot.send_speed()
  robot.set_arm(1024, 0)
  robot.send_arm()
  robot.set_speed(1600, 1600)
  prev_time = time.time()
  while time.time() - prev_time < 2:
    robot.update_button_stat()
    if robot.robot_stop:
      robot.set_speed(1500, 1500)
      robot.send_speed()
      logger.info("Catch interrupted by button")
      return 1
    robot.send_speed()
  robot.set_arm(1000, 1)
  robot.send_arm()
  robot.set_arm(3072, 1)
  robot.send_arm()
  robot.set_speed(1450, 1450)
  prev_time = time.time()
  while time.time() - prev_time < 1:
    robot.update_button_stat()
    if robot.robot_stop:
      robot.set_speed(1500, 1500)
      robot.send_speed()
      logger.info("Catch interrupted by button")
      return 1
    robot.send_speed()
  robot.set_speed(1500, 1500)
  robot.send_speed()
  find_best_target()
  if robot.rescue_offset is None:
    logger.info("Catch successful")
    robot.write_rescue_target(
        consts.TargetList.GREEN_CAGE.value if robot.rescue_target == consts.
        TargetList.SILVER_BALL.value else consts.TargetList.RED_CAGE.value)
    return 0
  else:
    logger.info("Catch failed")
    return 1


def release_ball() -> bool:
  logger.debug("Executing release_ball()")
  prev_time = time.time()
  robot.set_speed(1700, 1700)
  while time.time() - prev_time < 2.2:
    robot.update_button_stat()
    if robot.robot_stop:
      robot.set_speed(1500, 1500)
      robot.send_speed()
      logger.info("Release interrupted by button")
      return False
    robot.send_speed()
  robot.set_speed(1500, 1500)
  robot.send_speed()
  prev_time = time.time()
  robot.set_speed(1400, 1400)
  while time.time() - prev_time < 0.5:
    robot.update_button_stat()
    if robot.robot_stop:
      robot.set_speed(1500, 1500)
      robot.send_speed()
      logger.info("Release interrupted by button")
      return False
    robot.send_speed()
  robot.set_speed(1500, 1500)
  robot.set_arm(1536, 0)
  robot.send_speed()
  prev_time = time.time()
  while time.time() - prev_time < 1.5:
    robot.update_button_stat()
    if robot.robot_stop:
      robot.set_speed(1500, 1500)
      robot.send_speed()
      logger.info("Release interrupted by button")
      return False
    robot.send_arm()
  robot.set_arm(3072, 0)
  prev_time = time.time()
  while time.time() - prev_time < 0.5:
    robot.update_button_stat()
    if robot.robot_stop:
      robot.set_speed(1500, 1500)
      robot.send_speed()
      logger.info("Release interrupted by button")
      return False
    robot.send_arm()
  prev_time = time.time()
  robot.set_speed(1400, 1400)
  while time.time() - prev_time < 1:
    robot.update_button_stat()
    if robot.robot_stop:
      robot.set_speed(1500, 1500)
      robot.send_speed()
      logger.info("Release interrupted by button")
      return False
    robot.send_speed()
  prev_time = time.time()
  robot.set_speed(1750, 1250)
  while time.time() - prev_time < consts.TURN_180_TIME:
    robot.update_button_stat()
    if robot.robot_stop:
      robot.set_speed(1500, 1500)
      robot.send_speed()
      logger.info("Release interrupted by button")
      return False
    robot.send_speed()
  robot.set_speed(1500, 1500)
  robot.send_speed()
  return True  # Completed successfully


def change_position() -> bool:
  logger.debug("Change position")
  prev_time = time.time()
  robot.set_speed(1750, 1250)
  while time.time() - prev_time < consts.TURN_30_TIME:
    robot.update_button_stat()
    if robot.robot_stop:
      robot.set_speed(1500, 1500)
      robot.send_speed()
      logger.info("Position change interrupted by button")
      return False
    robot.send_speed()
  robot.set_speed(1500, 1500)
  robot.send_speed()
  robot.write_rescue_turning_angle(robot.rescue_turning_angle + 30)
  logger.info(f"Turn degrees{robot.rescue_turning_angle}")
  return True  # Completed successfully


def calculate_ball(angle: Optional[float] = None,
                   size: Optional[int] = None) -> tuple[int, int]:
  if angle is None or size is None:
    return 1500, 1500
  if abs(angle) > 60:
    diff_angle = angle * P
  else:
    diff_angle = 0
  if consts.BALL_CATCH_SIZE > size:
    dist_term = (math.sqrt(consts.BALL_CATCH_SIZE) - math.sqrt(size)) * AP
  dist_term = int(max(60, dist_term))
  base_L = 1500 + diff_angle + dist_term
  base_R = 1500 - diff_angle + dist_term
  logger.info(f"Motor speed L{base_L} R{base_R}")
  return clamp(base_L, MIN_SPEED, MAX_SPEED), clamp(base_R, MIN_SPEED,
                                                    MAX_SPEED)


def calculate_cage(angle: Optional[float] = None,
                   size: Optional[int] = None) -> tuple[int, int]:
  if angle is None or size is None:
    return 1500, 1500
  diff_angle = angle * WP
  base_L = 1500 + diff_angle + 150
  base_R = 1500 - diff_angle + 150
  logger.info(f"Motor speed L{base_L} R{base_R}")
  return clamp(base_L, MIN_SPEED, MAX_SPEED), clamp(base_R, MIN_SPEED,
                                                    MAX_SPEED)


logger.debug("Objects Initialized")

if __name__ == "__main__":
  # Register signal handler for graceful shutdown
  signal.signal(signal.SIGINT, signal_handler)

  logger.info("Starting program")
  robot.set_speed(1500,1500)
  robot.set_arm(3072,0)
  robot.send_arm()
  robot.send_speed()
  while True:
    robot.update_button_stat()
    robot.send_speed()
    if robot.robot_stop:
      robot.set_speed(1500, 1500)
    elif robot.is_rescue_flag:
      find_best_target()
      if (robot.rescue_offset is None) or (robot.rescue_size is None):
        change_position()
      else:
        if robot.rescue_target == consts.TargetList.EXIT.value:
          motorl = 1500
          motorr = 1500
        if robot.rescue_target == consts.TargetList.BLACK_BALL.value or robot.rescue_target == consts.TargetList.SILVER_BALL.value:
          motorl, motorr = calculate_ball(robot.rescue_offset,
                                          robot.rescue_size)
          if robot.rescue_ball_flag:
            is_not_took = catch_ball()
            # TODO: Retry
        else:
          motorl, motorr = calculate_cage(robot.rescue_offset,
                                          robot.rescue_size)
          if robot.rescue_size >= consts.BALL_CATCH_SIZE * 3.8:
            release_ball()
    else:
      if not robot.linetrace_stop:
        # Check for green mark intersections before normal line following
        if should_process_green_mark():
          execute_green_mark_turn()
        else:
          motorl, motorr = calculate_motor_speeds()
          robot.set_speed(motorl, motorr)
      else:
        logger.debug("Red stop")
logger.debug("Program Stop")
