import modules.constants as consts
import modules.logger as logger
from typing import Optional
import serial

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

  def list_ports(self) -> ListPortInfo:
    return list(serial.tools.list_ports.comports())

  def connect(self, port: ListPortInfo, baud_rate: int, timeout: float) -> None:
    self.__device_name = port.device
    self.__baud_rate = baud_rate
    self.__timeout = timeout
    self.connect()
    return None

  def connect(self) -> None:
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

  def send(self, message: Message) -> bool:
    if self.isConnected():
      self.__Serial_port.write(str(message).encode("ascii"))
      while True:
        message_str = self.__Serial_Port.read_until(b'\n').decode('ascii').strip()
        if message_str:
          retMessage = Message(message_str)
          if retMessage.getId == message.getId:
            return True
          elif retMessage.getId < message.getId:
            continue
          else:
            return False
        else:
          return False


class Robot:
  def __init__(self):
    self.logger = logger.get_logger()


if __name__ == "__main__":
  pass