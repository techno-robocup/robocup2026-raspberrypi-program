import modules.constants as consts
import modules.logger
import modules.robot
import time
import signal
import sys
import math

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
KP = 15


def clamp(value: int, min_val: int, max_val: int) -> int:
  """Clamp value between min and max."""
  return max(min_val, min(max_val, value))


def calculate_motor_speeds(slope: float = None) -> tuple[int, int]:
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

  motor_l = clamp(BASE_SPEED - steering, MIN_SPEED, MAX_SPEED)
  motor_r = clamp(BASE_SPEED + steering, MIN_SPEED, MAX_SPEED)

  return motor_l, motor_r



def signal_handler(sig, frame):
  """Handle SIGINT for graceful shutdown."""
  logger.info("Received shutdown signal")
  robot.set_speed(1500, 1500)
  robot.send_speed()
  sys.exit(0)

logger.debug("Objects Initialized")


if __name__ == "__main__":
  # Register signal handler for graceful shutdown
  signal.signal(signal.SIGINT, signal_handler)

  logger.info("Starting program")
  while True:
    if not robot.linetrace_stop:
      motorl, motorr = calculate_motor_speeds()
      robot.set_speed(motorl, motorr)
      robot.send_speed()

logger.debug("Program Stop")