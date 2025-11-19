import modules.constants as consts
import modules.logger as logger

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
  pass

class Robot:
  def __init__(self):
    self.logger = logger.get_logger()


if __name__ == "__main__":
  pass