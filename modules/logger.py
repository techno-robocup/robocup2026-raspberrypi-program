import logging


class UnixTimeFormatter(logging.Formatter):

  def formatTime(self, record, datefmt=None):
    return f"{record.created:.3f}"


def get_logger(name="Logger", file="log.log"):
  logger = logging.getLogger(name)

  if not logger.hasHandlers():
    logger.setLevel(logging.DEBUG)
    formatter = UnixTimeFormatter('[%(asctime)s] [%(levelname)s] %(message)s')

    console_handler = logging.StreamHandler()
    console_handler.setFormatter(formatter)
    logger.addHandler(console_handler)

    file_handler = logging.FileHandler(file)
    file_handler.setFormatter(formatter)
    logger.addHandler(file_handler)

  return logger
