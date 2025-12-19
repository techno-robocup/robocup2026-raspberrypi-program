import modules.constants as consts
import modules.logger as logger
from typing import Optional, List, Any
import serial
import serial.tools.list_ports
import modules.camera
import threading
import numpy as np
import numpy.typing as npt
import time


class Message:

  def __init__(self, *args):
    self.__id: int
    self.__message: str
    if len(args) == 2:
      self.__id = int(args[0])
      self.__message = args[1]
    elif len(args) == 1:
      temp_id, self.__message = args[0].split(" ", 1)
      self.__id = int(temp_id)
      self.__message = self.__message.strip()
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
  def Message(self):
    return self.__message


class uart_io:

  def __init__(self):
    self.__Serial_port: Optional[serial.Serial] = None
    self.__device_name: Optional[str] = None
    self.__baud_rate: Optional[int] = None
    self.__timeout: Optional[float] = None
    self.__message_id_increment = 0

  def list_ports(self) -> list:
    return list(serial.tools.list_ports.comports())

  def connect(self, port: str, baud_rate: int, timeout: float) -> None:
    self.__device_name = port
    self.__baud_rate = baud_rate
    self.__timeout = timeout
    self.__connect()
    return None

  def __connect(self) -> None:
    logger.get_logger().info(f"Connecting to {self.__device_name}")
    while True:
      assert self.__baud_rate != None
      self.__Serial_port = serial.Serial(self.__device_name,
                                         self.__baud_rate,
                                         timeout=self.__timeout)
      if self.__Serial_port.isOpen():
        break
    return None

  def isConnected(self) -> bool:
    return self.__Serial_port.isOpen()

  def reConnect(self) -> None:
    if self.isConnected():
      self.__Serial_port.close()
    self.__connect()
    return None

  def send(self, message: str) -> bool | str:
    self.__message_id_increment += 1
    return self.__send(Message(self.__message_id_increment, message))

  def __send(self, message: Message) -> bool | str:
    if self.isConnected():
      assert self.__Serial_port != None
      self.__Serial_port.write(str(message).encode("ascii"))
      logger.get_logger().info(f"Sent message: {str(message)}")
      while True:
        message_str = self.__Serial_port.read_until(b'\n').decode(
            'ascii').strip()
        if message_str:
          retMessage = Message(message_str)
          logger.get_logger().debug(f"Received message: {str(retMessage)}")
          if retMessage.Id == message.Id:
            return retMessage.Message
          elif retMessage.Id < message.Id:
            continue
          else:
            return False
        else:
          logger.get_logger().error(
              f"No response from ESP32 for message id {self.__message_id_increment}"
          )
          return False

  def close(self) -> None:
    if self.isConnected():
      self.__Serial_port.close()


class Robot:

  def __init__(self):
    self.logger = logger.get_logger()
    self.__uart_device: Optional[uart_io] = None
    self.__MOTOR_L = 1500
    self.__MOTOR_R = 1500
    self.__MOTOR_ARM = 3072
    self.__MOTOR_WIRE = 0
    self.__Linetrace_Camera = modules.camera.Camera(
        consts.Linetrace_Camera_Port, consts.Linetrace_Camera_Controls,
        consts.Linetrace_Camera_Size, consts.Linetrace_Camera_Formats,
        consts.Linetrace_Camera_lores, consts.Linetrace_Camera_precallback)
    self.__Rescue_Camera = modules.camera.Camera(
        consts.Rescue_Camera_Port, consts.Rescue_Camera_Controls,
        consts.Rescue_Camera_Size, consts.Rescue_Camera_Formats,
        consts.Rescue_Camera_lores, consts.Rescue_Camera_precallback)
    self.__rescue_camera_lock = threading.Lock()
    self.__linetrace_lock = threading.Lock()
    self.__rescue_camera_image: Optional[npt.NDArray[np.uint8]] = None
    self.__Linetrace_Camera.start_cam()
    self.__Rescue_Camera.start_cam()
    self.__rescue_lock = threading.Lock()
    self.__is_rescue_flag = False
    self.__rescue_offset: Optional[float] = None
    self.__rescue_size: Optional[int] = None
    self.__rescue_target: int = consts.TargetList.SILVER_BALL.value
    self.__rescue_turning_angle: int = 0
    self.__rescue_ball_flag = False
    self.__slope = None
    self.__is_stop = False
    self.__robot_stop: bool = False
    self.__top_checkpoint_black: bool = False
    # Green mark detection state
    self.__green_marks_lock = threading.Lock()
    self.__green_marks: List[tuple[int, int, int, int]] = []
    self.__green_black_detected: List[np.ndarray] = []
    self.__last_time_set: float | None = None
    # Set robot reference in camera module to avoid circular import
    modules.camera.set_robot(self)

  def set_uart_device(self, device: uart_io):
    self.__uart_device = device

  def set_speed(self, motor_l: int, motor_r: int) -> None:
    self.__MOTOR_L = min(max(consts.MOTOR_MIN_SPEED, motor_l),
                         consts.MOTOR_MAX_SPEED)
    self.__MOTOR_R = min(max(consts.MOTOR_MIN_SPEED, motor_r),
                         consts.MOTOR_MAX_SPEED)
    self.__last_time_set = time.time()
    return None

  def send_speed(self):
    assert self.__uart_device != None
    assert isinstance(self.__MOTOR_L, int)
    assert isinstance(self.__MOTOR_R, int)
    # if self.__last_time_set is None or time.time() - self.__last_time_set > 0.3:
    #   logger.get_logger().info(
    #       f"Stopping due to last time set too long {self.__last_time_set}")
      # return self.__uart_device.send("MOTOR 1500 1500")
    return self.__uart_device.send(f"MOTOR {self.__MOTOR_L} {self.__MOTOR_R}")

  def set_arm(self, angle: int, wire: int):
    assert wire == 0 or wire == 1
    self.__MOTOR_ARM = angle
    self.__MOTOR_WIRE = wire

  def send_arm(self):
    assert self.__uart_device != None
    return self.__uart_device.send(
        f"Rescue {self.__MOTOR_ARM:4d} {self.__MOTOR_WIRE}")

  @property
  def ultrasonic(self) -> List[int]:
    assert self.__uart_device != None
    return list(map(int,
                    Message(self.__uart_device.send("GET usonic")).Message))

  @property
  def button(self) -> bool:
    assert self.__uart_device != None
    return self.__uart_device.send("GET button") == "ON"

  def write_rescue_image(self, image: npt.NDArray[np.uint8]) -> None:
    with self.__rescue_camera_lock:
      self.__rescue_camera_image = image.copy()
    return None

  @property
  def rescue_image(self) -> Optional[npt.NDArray[np.uint8]]:
    with self.__rescue_camera_lock:
      return self.__rescue_camera_image

  def write_is_rescue_flag(self, flag: bool) -> None:
    with self.__rescue_lock:
      self.__is_rescue_flag = flag

  @property
  def is_rescue_flag(self) -> bool:
    return self.__is_rescue_flag

  def write_rescue_offset(self, angle: Optional[float]) -> None:
    with self.__rescue_lock:
      self.__rescue_offset = angle

  def write_rescue_size(self, size: Optional[int]) -> None:
    with self.__rescue_lock:
      self.__rescue_size = size

  def write_rescue_target(self, target: int) -> None:
    with self.__rescue_lock:
      self.__rescue_target = target

  def write_rescue_turning_angle(self, angle: int) -> None:
    with self.__rescue_lock:
      self.__rescue_turning_angle = angle

  def write_rescue_ball_flag(self, flag: bool) -> None:
    with self.__rescue_lock:
      self.__rescue_ball_flag = flag

  def update_button_stat(self) -> None:
    response = self.__uart_device.send("GET button")
    self.__robot_stop = response == "OFF"
    return None

  @property
  def rescue_offset(self) -> Optional[float]:
    with self.__rescue_lock:
      return self.__rescue_offset

  @property
  def rescue_size(self) -> Optional[int]:
    with self.__rescue_lock:
      return self.__rescue_size

  @property
  def rescue_target(self) -> int:
    with self.__rescue_lock:
      return self.__rescue_target

  @property
  def rescue_turning_angle(self) -> int:
    with self.__rescue_lock:
      return self.__rescue_turning_angle

  @property
  def rescue_ball_flag(self) -> bool:
    with self.__rescue_lock:
      return self.__rescue_ball_flag

  def write_linetrace_stop(self, flag: bool) -> None:
    with self.__linetrace_lock:
      self.__is_stop = flag

  def write_linetrace_slope(self, slope: Optional[float]) -> None:
    with self.__linetrace_lock:
      self.__slope = slope

  @property
  def linetrace_slope(self) -> Optional[float]:
    with self.__linetrace_lock:
      return self.__slope

  @property
  def linetrace_stop(self) -> bool:
    with self.__linetrace_lock:
      return self.__is_stop

  def write_top_checkpoint_black(self, is_black: bool) -> None:
    with self.__linetrace_lock:
      self.__top_checkpoint_black = is_black

  @property
  def top_checkpoint_black(self) -> bool:
    with self.__linetrace_lock:
      return self.__top_checkpoint_black

  def write_green_marks(self, marks: List[tuple[int, int, int, int]]) -> None:
    """Write detected green marks."""
    with self.__green_marks_lock:
      self.__green_marks = marks.copy()

  def write_green_black_detected(self, detections: List[np.ndarray]) -> None:
    """Write green mark black line detections."""
    with self.__green_marks_lock:
      self.__green_black_detected = detections.copy()

  @property
  def green_marks(self) -> List[tuple[int, int, int, int]]:
    """Get detected green marks."""
    with self.__green_marks_lock:
      return self.__green_marks.copy()

  @property
  def green_black_detected(self) -> List[np.ndarray]:
    """Get green mark black line detections."""
    with self.__green_marks_lock:
      return self.__green_black_detected.copy()

  @property
  def robot_stop(self) -> bool:
    return self.__robot_stop


robot = Robot()

if __name__ == "__main__":
  pass
