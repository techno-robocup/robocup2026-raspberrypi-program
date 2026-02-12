"""Raw UART I/O tool for sending arbitrary commands to ESP32."""
import sys

import modules.constants as consts
from modules.robot import uart_io


def main():
  uart_dev = uart_io()
  uart_devices = uart_dev.list_ports()

  usb_devices = [
      d for d in uart_devices if 'USB' in d.device or 'ACM' in d.device
  ]
  if usb_devices:
    selected_device = usb_devices[0]
  elif uart_devices:
    selected_device = uart_devices[0]
  else:
    print("No UART devices found")
    sys.exit(1)

  print(f"Connecting to {selected_device.device}...")
  uart_dev.connect(selected_device.device, consts.UART_BAUD_RATE,
                   consts.UART_TIMEOUT)
  print("Connected. Type commands to send (Ctrl+C to quit).\n")
  print("Examples:")
  print("  GET usonic")
  print("  GET button")
  print("  GET bno")
  print("  MOTOR 1500 1500")
  print("  healthcheck")
  print()

  try:
    while True:
      cmd = input("> ").strip()
      if not cmd:
        continue
      response = uart_dev.send(cmd)
      print(f"← {response}")
  except (KeyboardInterrupt, EOFError):
    print("\nClosing...")
  finally:
    uart_dev.close()


if __name__ == "__main__":
  main()
