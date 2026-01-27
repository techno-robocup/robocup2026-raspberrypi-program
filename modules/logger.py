import logging


class UnixTimeFormatter(logging.Formatter):

  def formatTime(self, record, datefmt=None):
    return f"{record.created:.3f}"


class ColoredFormatter(UnixTimeFormatter):
  """Formatter that adds colors to console output based on log level."""

  # ANSI color codes
  COLORS = {
      'DEBUG': '\033[36m',  # Cyan
      'INFO': '\033[32m',  # Green
      'WARNING': '\033[33m',  # Yellow
      'ERROR': '\033[31m',  # Red
      'CRITICAL': '\033[1;31m',  # Bold Red
  }
  RESET = '\033[0m'

  def format(self, record):
    # Add color to the level name
    levelname = record.levelname
    if levelname in self.COLORS:
      record.levelname = f"{self.COLORS[levelname]}{levelname}{self.RESET}"

    # Format the message
    result = super().format(record)

    # Restore original levelname (important for logging internals)
    record.levelname = levelname

    return result


def get_logger(name="Logger", file="log.log"):
  logger = logging.getLogger(name)

  if not logger.hasHandlers():
    # Set to DEBUG to capture all log levels
    logger.setLevel(logging.DEBUG)

    # Console handler with COLORS
    colored_formatter = ColoredFormatter(
        '[%(asctime)s] [%(levelname)-8s] [%(filename)s:%(lineno)d] [%(funcName)s] %(message)s'
    )
    console_handler = logging.StreamHandler()
    console_handler.setFormatter(colored_formatter)
    logger.addHandler(console_handler)

    # File handler WITH colors (so tail -f shows colored output)
    file_handler = logging.FileHandler(file, mode='a', encoding='utf-8')
    file_handler.setFormatter(colored_formatter)  # Now uses colored formatter!
    logger.addHandler(file_handler)

  return logger


if __name__ == "__main__":
  pass
