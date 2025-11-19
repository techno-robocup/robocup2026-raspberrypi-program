import modules.constants as consts
import modules.logger as logger
from typing import Optional, List
import serial
import serial.tools.list_ports

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
      logger.get_logger().error(f"Invalid number of arguments to construct Message object {args}")
  
  @property
  def __str__(self):
    return f"{self.id} {self.message}"

  @property
  def getId(self):
    return self.__id

  @property
  def getMessage(self):
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
    self.connect()
    return None

  def connect(self) -> None:
    logger.get_logger().info(f"Connecting to {self.__device_name}")
    while True:
      self.__Serial_port = serial.Serial(self.__device_name, self.__baud_rate, timeout=self.__timeout_)
      if self.__Serial_port.isOpen():
        break
    return None
  
  def isConnected(self) -> bool:
    return self.__Serial_port.isOpen

  def reConnect(self) -> None:
    if self.isConnected():
      self.__Serial_port.close()
    self.connect()
    return None

  def send(self, message: str) -> bool:
    return self.send(Message(self.__message_id_increment, message))

  def send(self, message: Message) -> bool | str:
    if self.isConnected():
      self.__Serial_port.write(str(message).encode("ascii"))
      while True:
        message_str = self.__Serial_Port.read_until(b'\n').decode('ascii').strip()
        if message_str:
          retMessage = Message(message_str)
          if retMessage.getId() == message.getId():
            return message.getMessage()
          elif retMessage.getId() < message.getId():
            continue
          else:
            return False
        else:
          logger.get_logger().error(f"No response from ESP32 for message id {__message_id_increment}")
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

  def set_uart_device(self, device: uart_io):
    self.__uart_device = device

  def set_speed(self, motor_l: int, motor_r: int):
    self.__MOTOR_L = min(max(consts.MOTOR_MIN_SPEED, motor_l), consts.MOTOR_MAX_SPEED)
    self.__MOTOR_R = min(max(consts.MOTOR_MIN_SPEED, motor_r), consts.MOTOR_MAX_SPEED)

  def send_speed(self):
    return self.__uart_device.send(f"MOTOR {self.__MOTOR_L} {self.__MOTOR_R}")

  def set_arm(self, angle: int, wire: int):
    assert wire == 0 or wire == 1
    self.__Rescue_angle = angle
    self.__Rescue_wire = wire

  def send_arm(self):
    return self.__uart_device.send(f"Rescue {self.__Rescue_angle:4d}{self.__Rescue_wire}")

  def get_ultrasonic(self) -> List[int]:
    return self.__uart_device.send("GET ultrasonic")

if __name__ == "__main__":
  pass