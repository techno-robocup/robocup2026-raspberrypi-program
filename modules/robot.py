import threading
import time
from typing import List, Optional

import numpy as np
import numpy.typing as npt
import serial
import serial.tools.list_ports

import modules.camera
import modules.constants as consts
import modules.logger as logger


class Message:
  """UART message wrapper with ID and content.

  Messages are formatted as "ID MESSAGE\\n" for serial communication.
  Used for request-response matching with ESP32.

  Attributes:
    Id: Unique message identifier for matching requests to responses.
    Message: The message content string.
  """

  def __init__(self, *args):
    """Initialize a Message from ID+content or from a raw string.

    Args:
      *args: Either (id, message) as two arguments, or a single
             string in format "ID MESSAGE" to be parsed.

    Raises:
      ValueError: If single-string format cannot be parsed.
    """
    self.__id: int
    self.__message: str
    if len(args) == 2:
      self.__id = int(args[0])
      self.__message = args[1]
    elif len(args) == 1:
      try:
        temp_id, self.__message = args[0].split(" ", 1)
        self.__id = int(temp_id)
        self.__message = self.__message.strip()
      except ValueError as e:
        logger.get_logger().exception(
            f"Failed to parse message. Expected format 'ID MESSAGE', got: '{args[0]}'. Error: {e}"
        )
        raise
    else:
      logger.get_logger().error(
          f"Invalid number of arguments to construct Message object {args}")

  # @property
  def __str__(self) -> str:
    return f"{self.Id} {self.Message}\n"

  @property
  def Id(self):
    return self.__id

  @property
  def Message(self) -> str:
    return self.__message


class uart_io:
  """Serial port communication handler for ESP32 UART interface.

  Manages serial connection lifecycle and provides request-response
  messaging with automatic ID tracking for reliable communication.

  Attributes:
    __Serial_port: The underlying pyserial Serial object.
    __message_id_increment: Auto-incrementing ID for message matching.
  """

  def __init__(self):
    """Initialize UART handler with no active connection."""
    self.__Serial_port: Optional[serial.Serial] = None
    self.__device_name: Optional[str] = None
    self.__baud_rate: Optional[int] = None
    self.__timeout: Optional[float] = None
    self.__message_id_increment = 0

  def list_ports(self) -> list:
    """List all available serial ports on the system.

    Returns:
      List of serial port info objects from pyserial.
    """
    return list(serial.tools.list_ports.comports())

  def connect(self, port: str, baud_rate: int, timeout: float) -> None:
    """Establish serial connection to the specified port.

    Args:
      port: Serial port device path (e.g., '/dev/ttyUSB0').
      baud_rate: Communication speed in bits per second.
      timeout: Read timeout in seconds.
    """
    self.__device_name = port
    self.__baud_rate = baud_rate
    self.__timeout = timeout
    self.__connect()
    return None

  def __connect(self) -> None:
    logger.get_logger().info(f"Connecting to {self.__device_name}")
    while True:
      assert self.__baud_rate is not None
      self.__Serial_port = serial.Serial(self.__device_name,
                                         self.__baud_rate,
                                         timeout=self.__timeout)
      if self.__Serial_port.isOpen():
        break
    return None

  def isConnected(self) -> bool:
    """Check if serial port is currently open.

    Returns:
      True if connected, False otherwise.
    """
    if self.__Serial_port is None:
      return False
    return self.__Serial_port.isOpen()

  def reConnect(self) -> None:
    """Close and reopen the serial connection."""
    if self.isConnected():
      self.__Serial_port.close()
    self.__connect()
    return None

  def send(self, message: str) -> bool | str:
    """Send a message and wait for matching response.

    Automatically assigns an incrementing ID to the message for
    request-response matching.

    Args:
      message: The command string to send (e.g., "MOTOR 1500 1500").

    Returns:
      The response message content on success, False on timeout or error.
    """
    self.__message_id_increment += 1
    return self.__send(Message(self.__message_id_increment, message))

  def __send(self, message: Message) -> bool | str:
    if self.isConnected():
      assert self.__Serial_port is not None
      self.__Serial_port.write(str(message).encode("ascii"))
      logger.get_logger().debug(f"Sent message: {str(message)}")
      logger.get_logger().debug(
          f"Waiting for reply (timeout: {self.__timeout}s)...")
      while True:
        message_str = self.__Serial_port.read_until(b'\n').decode(
            'ascii').strip()
        if message_str:
          logger.get_logger().debug(f"Raw received from UART: '{message_str}'")
          try:
            retMessage = Message(message_str)
            logger.get_logger().debug(f"Parsed message: {str(retMessage)}")
            if retMessage.Id == message.Id:
              logger.get_logger().debug(
                  f"✓ Reply received: '{retMessage.Message}'")
              return retMessage.Message
            elif retMessage.Id < message.Id:
              logger.get_logger().debug(
                  f"Ignoring old message (ID {retMessage.Id}, expected {message.Id})"
              )
              continue
            else:
              logger.get_logger().warning(
                  f"Received unexpected message ID {retMessage.Id} (sent {message.Id})"
              )
              return False
          except ValueError as e:
            # Log corrupted message and skip it (likely UART data loss)
            logger.get_logger().warning(
                f"Skipping corrupted UART message '{message_str}': {e}")
            continue  # Wait for next message instead of returning False
        else:
          logger.get_logger().error(
              f"✗ No response from ESP32 for message ID {message.Id} (timeout after {self.__timeout}s)"
          )
          return False

  def close(self) -> None:
    """Close the serial connection if open."""
    if self.isConnected():
      self.__Serial_port.close()


class Robot:
  """Main robot controller managing hardware interfaces and state.

  Provides thread-safe access to motors, cameras, sensors, and rescue
  operation state. Communicates with ESP32 via UART for motor control
  and sensor readings.

  The robot operates in two modes:
    - Linetrace mode: Following black line with green mark detection
    - Rescue mode: YOLO-based ball/cage detection and manipulation

  Attributes:
    logger: Logger instance for this class.

  Thread Safety:
    All state properties use locks for safe concurrent access from
    camera callbacks and main control loop.
  """

  def __init__(self):
    """Initialize robot with cameras, locks, and default state.

    Starts both linetrace and rescue cameras immediately.
    Motor speeds default to 1500 (stopped).
    """
    self.logger = logger.get_logger()
    self.__uart_device: Optional[uart_io] = None
    self.__MOTOR_L = 1500
    self.__MOTOR_R = 1500
    self.__MOTOR_ARM = 3072
    self.__MOTOR_WIRE = 0
    self.__Rescue_Camera = modules.camera.Camera(
        consts.Rescue_Camera_Port, consts.Rescue_Camera_Controls,
        consts.Rescue_Camera_Size, consts.Rescue_Camera_Formats,
        consts.Rescue_Camera_lores, consts.Rescue_Camera_precallback)
    self.__Linetrace_Camera = modules.camera.Camera(
        consts.Linetrace_Camera_Port, consts.Linetrace_Camera_Controls,
        consts.Linetrace_Camera_Size, consts.Linetrace_Camera_Formats,
        consts.Linetrace_Camera_lores, consts.Linetrace_Camera_precallback)
    self.__rescue_camera_lock = threading.Lock()
    self.__linetrace_lock = threading.Lock()
    self.__rescue_camera_image: Optional[npt.NDArray[np.uint8]] = None
    self.__Linetrace_Camera.start_cam()
    self.__Rescue_Camera.start_cam()
    self.__rescue_lock = threading.Lock()
    self.__is_rescue_flag = False
    self.__rescue_offset: Optional[float] = None
    self.__rescue_size: Optional[int] = None
    self.__rescue_y: Optional[float] = None
    self.__rescue_target: int = consts.TargetList.SILVER_BALL.value
    self.__rescue_turning_angle: int = 0  # Total revolutions
    self.__ball_catch_dist_flag = False  # catch ball flag
    self.__ball_catch_offset_flag = False
    self.__ball_near_flag = False
    self.__has_moved_to_cage = False
    self.__detect_black_ball = False
    self.__slope = None
    self.__line_area: Optional[float] = None
    self.__line_center_x: Optional[int] = None
    self.__is_stop = False
    self.__robot_stop: bool = False
    self.__top_checkpoint_black: bool = False
    # Green mark detection state
    self.__green_marks_lock = threading.Lock()
    self.__green_marks: List[tuple[int, int, int, int]] = []
    self.__green_black_detected: List[np.ndarray] = []
    self.__last_time_set: Optional[float] = None
    self.__last_slope_get_time: Optional[float] = None
    self.__gyro_lock = threading.Lock()
    self.__yaw: float = 0.0
    self.__roll: float = 0.0
    self.__pitch: float = 0.0
    self.__acc_x: float = 0.0
    self.__acc_y: float = 0.0
    self.__acc_z: float = 0.0
    # Set robot reference in camera module to avoid circular import
    modules.camera.set_robot(self)

  def set_uart_device(self, device: uart_io) -> None:
    """Set the UART device for ESP32 communication.

    Args:
      device: Configured uart_io instance with active connection.
    """
    self.__uart_device = device

  def set_speed(self, motor_l: int, motor_r: int) -> None:
    """
    Sets the motor speed in the robot class

    Args:
        motor_l (int): The left motor's speed
        motor_r (int): The right motor's speed

    Returns:
        None: Always returns None
    """
    self.__MOTOR_L = int(min(max(consts.MOTOR_MIN_SPEED, motor_l),
                         consts.MOTOR_MAX_SPEED))
    self.__MOTOR_R = int(min(max(consts.MOTOR_MIN_SPEED, motor_r),
                         consts.MOTOR_MAX_SPEED))
    self.__last_time_set = time.time()
    return None

  def send_speed(self):
    """
    Sends the speed currently set in the robot class

    Returns:
        string: The reply from ESP32
    """
    assert self.__uart_device is not None
    if self.__MOTOR_L is None or self.__MOTOR_R is None:
      logger.get_logger().error(
          "Motor speeds not set before sending to ESP32.")
      return False
    assert isinstance(self.__MOTOR_L, int), f"Type of self.__MOTOR_L is {type(self.__MOTOR_L)}"
    assert isinstance(self.__MOTOR_R, int), f"Type of self.__MOTOR_R is {type(self.__MOTOR_R)}"
    # if self.__last_time_set is None or time.time() - self.__last_time_set > 0.3:
    #   logger.get_logger().info(
    #       f"Stopping due to last time set too long {self.__last_time_set}")
    # return self.__uart_device.send("MOTOR 1500 1500")
    return self.__uart_device.send(f"MOTOR {self.__MOTOR_L} {self.__MOTOR_R}")

  def set_arm(self, angle: int, wire: int):
    """
    Sets the information of the arm

    Args:
        angle (int): The angle range from 0 ~ 4095
        wire (int): The state of the wire(0 or 1)
    """
    assert wire == 0 or wire == 1
    self.__MOTOR_ARM = angle
    self.__MOTOR_WIRE = wire

  def send_arm(self):
    """
    Sends the information about the arm angle and wire tension

    Returns:
        string: The reply from ESP32
    """
    assert self.__uart_device is not None
    return self.__uart_device.send(
        f"Rescue {self.__MOTOR_ARM:4d} {self.__MOTOR_WIRE}")

  @property
  def ultrasonic(self) -> List[float]:
    """Get ultrasonic sensor readings from ESP32.

    Returns:
      List of distance values in centimeters.
    """
    assert self.__uart_device is not None
    return list(
        map(float,
            Message(self.__uart_device.send("GET usonic")).Message.split()))

  @property
  def button(self) -> bool:
    """Get current button state from ESP32.

    Returns:
      True if button is pressed (ON), False otherwise.
    """
    assert self.__uart_device is not None
    return self.__uart_device.send("GET button") == "ON"

  def write_rescue_image(self, image: npt.NDArray[np.uint8]) -> None:
    """Store rescue camera image for YOLO processing (thread-safe).

    Args:
      image: BGR image array from rescue camera.
    """
    with self.__rescue_camera_lock:
      self.__rescue_camera_image = image.copy()
    return None

  @property
  def rescue_image(self) -> Optional[npt.NDArray[np.uint8]]:
    """Get latest rescue camera image (thread-safe).

    Returns:
      BGR image array, or None if no image captured yet.
    """
    with self.__rescue_camera_lock:
      return self.__rescue_camera_image

  def write_is_rescue_flag(self, flag: bool) -> None:
    """Set rescue mode flag (thread-safe).

    Args:
      flag: True to enter rescue mode, False for linetrace mode.
    """
    with self.__rescue_lock:
      self.__is_rescue_flag = flag

  @property
  def is_rescue_flag(self) -> bool:
    """Check if robot is in rescue mode."""
    return self.__is_rescue_flag

  def write_rescue_offset(self, angle: Optional[float]) -> None:
    """Set horizontal offset to rescue target (thread-safe).

    Args:
      angle: Pixel offset from image center, or None if no target.
    """
    with self.__rescue_lock:
      self.__rescue_offset = angle

  def write_rescue_size(self, size: Optional[int]) -> None:
    """Set detected target size (thread-safe).

    Args:
      size: Target area in pixels^2, or None if no target.
    """
    with self.__rescue_lock:
      self.__rescue_size = size

  def write_rescue_y(self, y: Optional[float]) -> None:
    """Set vertical center (y) of rescue target (thread-safe).

    Args:
      y: Pixel y-coordinate of bbox center, or None if no target.
    """
    with self.__rescue_lock:
      self.__rescue_y = y

  def write_rescue_target(self, target: int) -> None:
    """Set current rescue target type (thread-safe).

    Args:
      target: TargetList enum value (SILVER_BALL, BLACK_BALL, etc.).
    """
    with self.__rescue_lock:
      self.__rescue_target = target

  def write_rescue_turning_angle(self, angle: int) -> None:
    """Set cumulative turning angle during rescue (thread-safe).

    Args:
      angle: Total degrees rotated (0-720+).
    """
    with self.__rescue_lock:
      self.__rescue_turning_angle = angle

  def write_ball_catch_dist_flag(self, flag: bool) -> None:
    """Set ball catch ready flag (thread-safe).

    Args:
      flag: True if ball is close enough to catch.
    """
    with self.__rescue_lock:
      self.__ball_catch_dist_flag = flag

  def write_ball_catch_offset_flag(self, flag: bool) -> None:
    with self.__rescue_lock:
      self.__ball_catch_offset_flag = flag

  def write_ball_near_flag(self, flag: bool) -> None:
    with self.__rescue_lock:
      self.__ball_near_flag = flag

  def write_has_moved_to_cage(self, flag: bool) -> None:
    with self.__rescue_lock:
      self.__has_moved_to_cage = flag

  def update_button_stat(self) -> None:
    """Poll button state from ESP32 and update robot_stop flag.

    Sets robot_stop to True when button is OFF (not pressed).
    """
    response = self.__uart_device.send("GET button")
    logger.get_logger().debug(
        f"Button response: {response} (type: {type(response).__name__})")
    self.__robot_stop = response == "OFF"
    return None

  def update_gyro_stat(self) -> None:
    """Update gyro's angles from ESP32.

    Returns:
      None
    """
    response = self.__uart_device.send("GET bno")
    if not response:
      logger.get_logger().error("Failed to get gyro data from ESP32.")
      return None
    try:
      angles = list(map(float, response.split()))
    except ValueError:
      logger.get_logger().error(
          f"Failed to parse gyro data as floats: '{response}'")
      return None
    if len(angles) != 6:
      logger.get_logger().error(f"Unexpected gyro data format: '{response}'")
      return None
    with self.__gyro_lock:
      self.__yaw, self.__roll, self.__pitch, self.__acc_x, self.__acc_y, self.__acc_z = angles
    return None

  @property
  def yaw(self) -> float:
    """Get current yaw angle from gyro."""
    with self.__gyro_lock:
      return self.__yaw

  @property
  def roll(self) -> float:
    """Get current roll angle from gyro."""
    with self.__gyro_lock:
      return self.__roll

  @property
  def pitch(self) -> float:
    """Get current pitch angle from gyro."""
    with self.__gyro_lock:
      return self.__pitch

  @property
  def acc_x(self) -> float:
    """Get current X-axis acceleration from gyro."""
    with self.__gyro_lock:
      return self.__acc_x

  @property
  def acc_y(self) -> float:
    """Get current Y-axis acceleration from gyro."""
    with self.__gyro_lock:
      return self.__acc_y

  @property
  def acc_z(self) -> float:
    """Get current Z-axis acceleration from gyro."""
    with self.__gyro_lock:
      return self.__acc_z

  @property
  def rescue_offset(self) -> Optional[float]:
    """Get horizontal offset to rescue target (thread-safe)."""
    with self.__rescue_lock:
      return self.__rescue_offset

  @property
  def rescue_size(self) -> Optional[int]:
    """Get detected target size in pixels^2 (thread-safe)."""
    with self.__rescue_lock:
      return self.__rescue_size

  @property
  def rescue_y(self) -> Optional[float]:
    """Get vertical center (y) of rescue target (thread-safe)."""
    with self.__rescue_lock:
      return self.__rescue_y

  @property
  def rescue_target(self) -> int:
    """Get current rescue target type as TargetList value (thread-safe)."""
    with self.__rescue_lock:
      return self.__rescue_target

  @property
  def rescue_turning_angle(self) -> int:
    """Get cumulative turning angle in degrees (thread-safe)."""
    with self.__rescue_lock:
      return self.__rescue_turning_angle

  @property
  def ball_catch_dist_flag(self) -> bool:
    """Check if ball is close enough to catch (thread-safe)."""
    with self.__rescue_lock:
      return self.__ball_catch_dist_flag

  @property
  def ball_catch_offset_flag(self) -> bool:
    with self.__rescue_lock:
      return self.__ball_catch_offset_flag

  @property
  def ball_near_flag(self) -> bool:
    with self.__rescue_lock:
      return self.__ball_near_flag

  @property
  def has_moved_to_cage(self) -> bool:
    with self.__rescue_lock:
      return self.__has_moved_to_cage

  @property
  def detect_black_ball(self) -> bool:
    with self.__rescue_lock:
      return self.__detect_black_ball

  def write_detect_black_ball(self, flag: bool) -> None:
    with self.__rescue_lock:
      self.__detect_black_ball = flag

  def write_linetrace_stop(self, flag: bool) -> None:
    """Set linetrace stop flag (thread-safe).

    Args:
      flag: True to stop linetrace operation.
    """
    with self.__linetrace_lock:
      self.__is_stop = flag

  def write_linetrace_slope(self, slope: Optional[float]) -> None:
    """Set detected line slope (thread-safe).

    Args:
      slope: Line slope value, or None if line not detected.
    """
    with self.__linetrace_lock:
      self.__slope = slope

  @property
  def linetrace_slope(self) -> Optional[float]:
    """Get detected line slope (thread-safe).

    Returns:
      Slope value for steering calculation, or None if unavailable.
    """
    with self.__linetrace_lock:
      return self.__slope

  def write_line_area(self, area: Optional[float]) -> None:
    """Set detected black line area (thread-safe).

    Args:
      area: Line area in pixels, or None if not detected.
    """
    with self.__linetrace_lock:
      self.__line_area = area

  @property
  def line_area(self) -> Optional[float]:
    """Get detected black line area in pixels (thread-safe)."""
    with self.__linetrace_lock:
      return self.__line_area

  def write_line_center_x(self, center_x: Optional[int]) -> None:
    """Set detected line center x-coordinate (thread-safe).

    Args:
      center_x: Line center x-coordinate in pixels, or None if not detected.
    """
    with self.__linetrace_lock:
      self.__line_center_x = center_x

  @property
  def line_center_x(self) -> Optional[int]:
    """Get detected line center x-coordinate in pixels (thread-safe)."""
    with self.__linetrace_lock:
      return self.__line_center_x

  @property
  def linetrace_stop(self) -> bool:
    """Check if linetrace is stopped (thread-safe)."""
    with self.__linetrace_lock:
      return self.__is_stop

  def write_top_checkpoint_black(self, is_black: bool) -> None:
    """Set whether top checkpoint detects black line (thread-safe).

    Args:
      is_black: True if black line detected at top of image.
    """
    with self.__linetrace_lock:
      self.__top_checkpoint_black = is_black

  @property
  def top_checkpoint_black(self) -> bool:
    """Check if top checkpoint detects black line (thread-safe).

    Used for counting line crossings during turns.
    """
    with self.__linetrace_lock:
      return self.__top_checkpoint_black

  def write_green_marks(self, marks: List[tuple[int, int, int, int]]) -> None:
    """Store detected green mark positions (thread-safe).

    Args:
      marks: List of (center_x, center_y, width, height) tuples.
    """
    with self.__green_marks_lock:
      self.__green_marks = marks.copy()

  def write_green_black_detected(self, detections: List[np.ndarray]) -> None:
    """Store black line detection results for each green mark (thread-safe).

    Args:
      detections: List of arrays [bottom, top, left, right] for each mark.
    """
    with self.__green_marks_lock:
      self.__green_black_detected = detections.copy()

  @property
  def green_marks(self) -> List[tuple[int, int, int, int]]:
    """Get detected green mark positions (thread-safe).

    Returns:
      List of (center_x, center_y, width, height) tuples.
    """
    with self.__green_marks_lock:
      return self.__green_marks.copy()

  @property
  def green_black_detected(self) -> List[np.ndarray]:
    """Get black line detections for green marks (thread-safe).

    Returns:
      List of [bottom, top, left, right] detection arrays.
    """
    with self.__green_marks_lock:
      return self.__green_black_detected.copy()

  def write_last_slope_get_time(self, time: float) -> None:
    """Update timestamp of last successful slope detection (thread-safe).

    Used to trigger rescue mode after timeout.

    Args:
      time: Unix timestamp from time.time().
    """
    with self.__linetrace_lock:
      self.__last_slope_get_time = time

  @property
  def last_slope_get_time(self) -> float:
    """Get timestamp of last successful slope detection.

    Returns:
      Unix timestamp, used to detect rescue mode timeout.
    """
    return self.__last_slope_get_time

  @property
  def robot_stop(self) -> bool:
    """Check if robot should stop (button not pressed).

    Returns:
      True if stop button is released (OFF), False otherwise.
    """
    return self.__robot_stop

  @property
  def sum_accel(self) -> float:
    """Calculate total acceleration magnitude from gyro data.

    Returns:
      Magnitude of acceleration vector.
    """
    with self.__gyro_lock:
      return (self.__acc_x**2 + self.__acc_y**2)**0.5


robot = Robot()

if __name__ == "__main__":
  pass
