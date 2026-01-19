import logging
import os

# Global boolean flag to enable/disable conditional logging
# Can be controlled via environment variable ENABLE_CONDITIONAL_LOGS
ENABLE_CONDITIONAL_LOGS = os.environ.get('ENABLE_CONDITIONAL_LOGS', 'true').lower() in ('true', '1', 'yes')


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


class ConditionalLogger:
  """Wrapper for logging.Logger that adds conditional logging support.
  
  This class wraps the standard Python logger and adds a boolean flag check
  before logging. When ENABLE_CONDITIONAL_LOGS is False, only ERROR and CRITICAL
  messages are logged, while DEBUG, INFO, and WARNING are suppressed.
  """
  
  def __init__(self, logger: logging.Logger):
    self._logger = logger
  
  def _should_log(self, level: int) -> bool:
    """Check if the message should be logged based on the conditional flag."""
    global ENABLE_CONDITIONAL_LOGS
    # Always log ERROR and CRITICAL regardless of flag
    if level >= logging.ERROR:
      return True
    # For DEBUG, INFO, WARNING - check the flag
    return ENABLE_CONDITIONAL_LOGS
  
  def debug(self, msg, *args, **kwargs):
    """Log a debug message with conditional check."""
    if self._should_log(logging.DEBUG):
      self._logger.debug(msg, *args, **kwargs)
  
  def info(self, msg, *args, **kwargs):
    """Log an info message with conditional check."""
    if self._should_log(logging.INFO):
      self._logger.info(msg, *args, **kwargs)
  
  def warning(self, msg, *args, **kwargs):
    """Log a warning message with conditional check."""
    if self._should_log(logging.WARNING):
      self._logger.warning(msg, *args, **kwargs)
  
  def error(self, msg, *args, **kwargs):
    """Log an error message (always logged)."""
    self._logger.error(msg, *args, **kwargs)
  
  def critical(self, msg, *args, **kwargs):
    """Log a critical message (always logged)."""
    self._logger.critical(msg, *args, **kwargs)
  
  def exception(self, msg, *args, **kwargs):
    """Log an exception message (always logged)."""
    self._logger.exception(msg, *args, **kwargs)
  
  def setLevel(self, level):
    """Set the logging level."""
    self._logger.setLevel(level)
  
  def hasHandlers(self):
    """Check if logger has handlers."""
    return self._logger.hasHandlers()
  
  def __getattr__(self, name):
    """Proxy any undefined attributes to the wrapped logger for full compatibility."""
    return getattr(self._logger, name)


def get_logger(name="Logger", file="log.log"):
  """Get a conditional logger instance.
  
  Args:
    name: Logger name
    file: Log file path
    
  Returns:
    ConditionalLogger instance that wraps the standard Python logger
  """
  logger = logging.getLogger(name)

  if not logger.hasHandlers():
    # Set to DEBUG to capture all log levels
    logger.setLevel(logging.INFO)

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

  return ConditionalLogger(logger)


def set_conditional_logs(enabled: bool):
  """Set the global conditional logging flag.
  
  Args:
    enabled: True to enable all log levels, False to only log ERROR and CRITICAL
  """
  global ENABLE_CONDITIONAL_LOGS
  ENABLE_CONDITIONAL_LOGS = enabled


def get_conditional_logs() -> bool:
  """Get the current state of the conditional logging flag.
  
  Returns:
    True if conditional logging is enabled, False otherwise
  """
  global ENABLE_CONDITIONAL_LOGS
  return ENABLE_CONDITIONAL_LOGS


if __name__ == "__main__":
  pass
