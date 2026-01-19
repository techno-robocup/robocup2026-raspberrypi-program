#!/usr/bin/env python3
"""Test script for conditional logger functionality."""

import sys
import os

# Add parent directory to path to import modules
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import modules.logger as logger_module

def test_conditional_logging():
    """Test the conditional logging feature."""
    
    # Get a logger instance
    logger = logger_module.get_logger("TestLogger", "test_log.log")
    
    print("\n=== Test 1: Conditional logging ENABLED (default) ===")
    print(f"Conditional logs enabled: {logger_module.get_conditional_logs()}")
    logger.debug("This is a DEBUG message (should appear)")
    logger.info("This is an INFO message (should appear)")
    logger.warning("This is a WARNING message (should appear)")
    logger.error("This is an ERROR message (always appears)")
    logger.critical("This is a CRITICAL message (always appears)")
    
    print("\n=== Test 2: Conditional logging DISABLED ===")
    logger_module.set_conditional_logs(False)
    print(f"Conditional logs enabled: {logger_module.get_conditional_logs()}")
    logger.debug("This is a DEBUG message (should NOT appear)")
    logger.info("This is an INFO message (should NOT appear)")
    logger.warning("This is a WARNING message (should NOT appear)")
    logger.error("This is an ERROR message (always appears)")
    logger.critical("This is a CRITICAL message (always appears)")
    
    print("\n=== Test 3: Re-enabling conditional logging ===")
    logger_module.set_conditional_logs(True)
    print(f"Conditional logs enabled: {logger_module.get_conditional_logs()}")
    logger.debug("This is a DEBUG message (should appear again)")
    logger.info("This is an INFO message (should appear again)")
    logger.warning("This is a WARNING message (should appear again)")
    logger.error("This is an ERROR message (always appears)")
    
    print("\n=== Tests completed ===")

if __name__ == "__main__":
    test_conditional_logging()
