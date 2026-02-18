"""Test script to read ultrasonic sensor values via UART."""
import time

import modules.constants as consts
from modules.robot import Message, uart_io


def main():
  uart_dev = uart_io()

  print("Connecting to UART device (auto-selecting)...")
  uart_dev.connect(consts.UART_BAUD_RATE, consts.UART_TIMEOUT)
  print("Connected.\n")

  try:
    while True:
      response = uart_dev.send("GET usonic")
      if response and isinstance(response, str):
        values = list(map(float, Message(0, response).Message.split()))
        print(f"Ultrasonic: {values}")
      else:
        print(f"Failed to get ultrasonic data: {response}")
      time.sleep(0.5)
  except KeyboardInterrupt:
    print("\nStopping...")
  finally:
    uart_dev.close()


if __name__ == "__main__":
  main()
