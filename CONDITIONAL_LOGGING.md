# Conditional Logging Feature

## Overview

The logger now supports conditional logging through a global boolean flag. This allows you to dynamically control which log messages are printed without changing the log level configuration.

## How It Works

- **Default Behavior**: All log levels (DEBUG, INFO, WARNING, ERROR, CRITICAL) are enabled by default
- **Conditional Control**: When conditional logging is disabled, only ERROR and CRITICAL messages are logged
- **Dynamic Control**: The flag can be changed at runtime or via environment variable

## Usage

### Environment Variable

Set the `ENABLE_CONDITIONAL_LOGS` environment variable before running the program:

```bash
# Enable conditional logging (default)
export ENABLE_CONDITIONAL_LOGS=true
python main.py

# Disable conditional logging (only ERROR and CRITICAL)
export ENABLE_CONDITIONAL_LOGS=false
python main.py
```

### Programmatic Control

You can also control the flag programmatically in your code:

```python
import modules.logger as logger_module

# Get logger instance
logger = logger_module.get_logger()

# Check current state
is_enabled = logger_module.get_conditional_logs()
print(f"Conditional logs enabled: {is_enabled}")

# Disable conditional logging
logger_module.set_conditional_logs(False)

# Re-enable conditional logging
logger_module.set_conditional_logs(True)
```

### Log Level Behavior

| Log Level | Conditional Enabled | Conditional Disabled |
|-----------|--------------------|--------------------|
| DEBUG     | ✅ Logged          | ❌ Suppressed      |
| INFO      | ✅ Logged          | ❌ Suppressed      |
| WARNING   | ✅ Logged          | ❌ Suppressed      |
| ERROR     | ✅ Logged          | ✅ Logged          |
| CRITICAL  | ✅ Logged          | ✅ Logged          |
| EXCEPTION | ✅ Logged          | ✅ Logged          |

## Example

```python
import modules.logger as logger_module

logger = logger_module.get_logger()

# These will appear when ENABLE_CONDITIONAL_LOGS=true
logger.debug("Debug message")
logger.info("Info message")
logger.warning("Warning message")

# These always appear regardless of the flag
logger.error("Error message")
logger.critical("Critical message")
```

## Testing

Run the test script to verify the functionality:

```bash
python tests/test_logger.py
```

## Use Cases

1. **Production Environment**: Disable conditional logging to reduce log verbosity in production
2. **Debugging**: Enable conditional logging to see detailed DEBUG and INFO messages
3. **Performance**: Reduce I/O overhead by disabling non-critical logs in performance-sensitive scenarios
4. **Remote Operation**: Disable verbose logging when running the robot remotely to reduce network overhead
