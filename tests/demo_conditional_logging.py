#!/usr/bin/env python3
"""
Demonstration of the conditional logging feature.

This script shows how the boolean flag controls which log messages appear.
"""

import sys
import os

# Add parent directory to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import modules.logger as logger_module

def main():
    """Demonstrate conditional logging in action."""
    
    print("=" * 70)
    print("CONDITIONAL LOGGING DEMONSTRATION")
    print("=" * 70)
    
    logger = logger_module.get_logger("DemoLogger")
    
    # Part 1: Default behavior (all logs enabled)
    print("\n📝 PART 1: Default Behavior (All logs enabled)")
    print("-" * 70)
    print(f"Conditional logging enabled: {logger_module.get_conditional_logs()}")
    print("\nProducing various log messages:")
    logger.debug("This is a DEBUG message - detailed diagnostic info")
    logger.info("This is an INFO message - general information")
    logger.warning("This is a WARNING message - something to watch out for")
    logger.error("This is an ERROR message - something went wrong")
    logger.critical("This is a CRITICAL message - system failure!")
    
    # Part 2: Disabled conditional logging
    print("\n📝 PART 2: Production Mode (Conditional logs disabled)")
    print("-" * 70)
    logger_module.set_conditional_logs(False)
    print(f"Conditional logging enabled: {logger_module.get_conditional_logs()}")
    print("\nProducing the same log messages:")
    print("(Notice: only ERROR and CRITICAL appear)")
    logger.debug("This DEBUG will NOT appear")
    logger.info("This INFO will NOT appear")
    logger.warning("This WARNING will NOT appear")
    logger.error("This ERROR WILL appear")
    logger.critical("This CRITICAL WILL appear")
    
    # Part 3: Re-enable for debugging
    print("\n📝 PART 3: Debug Mode (Re-enabling conditional logs)")
    print("-" * 70)
    logger_module.set_conditional_logs(True)
    print(f"Conditional logging enabled: {logger_module.get_conditional_logs()}")
    print("\nProducing log messages again:")
    logger.info("All log levels are now visible again")
    logger.debug("Debug information is back")
    logger.warning("Warnings are shown")
    
    # Summary
    print("\n" + "=" * 70)
    print("SUMMARY")
    print("=" * 70)
    print("""
The conditional logging feature provides:
✓ Control over log verbosity without changing log levels
✓ Easy toggle between production and debug modes  
✓ Environment variable support (ENABLE_CONDITIONAL_LOGS)
✓ Programmatic control via set_conditional_logs()
✓ Always shows ERROR and CRITICAL regardless of flag
✓ Full backward compatibility with existing code
    """)
    
    print("Run this script with environment variable to test:")
    print("  ENABLE_CONDITIONAL_LOGS=false python tests/demo_conditional_logging.py")
    print("=" * 70)

if __name__ == "__main__":
    main()
