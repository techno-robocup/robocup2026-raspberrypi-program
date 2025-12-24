import logging


class UnixTimeFormatter(logging.Formatter):

  def formatTime(self, record, datefmt=None):
    return f"{record.created:.3f}"


def get_logger(name="Logger", file="log.log"):
  logger = logging.getLogger(name)

  if not logger.hasHandlers():
    # Set to DEBUG to capture all log levels
    logger.setLevel(logging.DEBUG)
    
    # Enhanced formatter with file, line number, function name, and detailed info
    formatter = UnixTimeFormatter(
      '[%(asctime)s] [%(levelname)-8s] [%(filename)s:%(lineno)d] [%(funcName)s] %(message)s'
    )

    console_handler = logging.StreamHandler()
    console_handler.setFormatter(formatter)
    logger.addHandler(console_handler)

    # File handler with same detailed format
    file_handler = logging.FileHandler(file, mode='a', encoding='utf-8')
    file_handler.setFormatter(formatter)
    logger.addHandler(file_handler)

  return logger


if __name__ == "__main__":
  pass
