#!/usr/bin/env python3
"""Integration test for conditional logger with real usage patterns."""

import sys
import os

# Add parent directory to path to import modules
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import modules.logger as logger_module

def test_integration():
    """Test logger integration mimicking real codebase usage."""
    
    # Simulate main.py usage
    logger = logger_module.get_logger()
    
    print("\n=== Test 1: Simulating robot initialization ===")
    logger.debug("Logger initialized")
    logger.info("Starting program")
    logger.info("Connecting to UART device: /dev/ttyUSB0")
    
    # Simulate robot operation with various log levels
    print("\n=== Test 2: Simulating robot operation (all logs enabled) ===")
    logger.debug("Turn interrupted by button during approach")
    logger.info("Line recovery completed in 0.52s")
    logger.warning("Gyro yaw unavailable, cannot execute gyro-based turn")
    logger.error("No UART devices found")
    logger.info("Received shutdown signal")
    
    # Disable conditional logging
    print("\n=== Test 3: Production mode (conditional logs disabled) ===")
    logger_module.set_conditional_logs(False)
    logger.debug("This debug message should NOT appear")
    logger.info("This info message should NOT appear")
    logger.warning("This warning should NOT appear")
    logger.error("This error SHOULD appear")
    logger.critical("This critical SHOULD appear")
    
    # Test exception logging (should always work)
    print("\n=== Test 4: Exception logging (always enabled) ===")
    try:
        raise ValueError("Test exception")
    except ValueError as e:
        logger.exception(f"Caught exception: {e}")
    
    # Re-enable
    print("\n=== Test 5: Re-enabling for debugging ===")
    logger_module.set_conditional_logs(True)
    logger.debug("Debug mode re-enabled")
    logger.info("Object avoidance triggered")
    logger.info("Wall opening detected")
    
    # Test logger from different modules (simulating robot.py usage)
    print("\n=== Test 6: Simulating usage from robot.py ===")
    robot_logger = logger_module.get_logger("RobotLogger")
    robot_logger.info("Connecting to /dev/ttyUSB0")
    robot_logger.debug("Sent message: MOTOR 1500 1500")
    robot_logger.error("Failed to get gyro data from ESP32.")
    
    # Test logger from camera.py
    print("\n=== Test 7: Simulating usage from camera.py ===")
    camera_logger = logger_module.get_logger("CameraLogger")
    camera_logger.info("Loading Depth-Anything-V2 model...")
    camera_logger.warning("Depth model not available, skipping depth prediction")
    camera_logger.debug("Linetrace Camera Pre call-back called")
    
    print("\n=== All integration tests completed successfully ===")
    return True

if __name__ == "__main__":
    success = test_integration()
    sys.exit(0 if success else 1)
