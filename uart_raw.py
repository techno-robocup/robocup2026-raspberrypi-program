"""Raw UART I/O tool for sending arbitrary commands to ESP32."""
import modules.constants as consts
from modules.robot import uart_io


def main():
  uart_dev = uart_io()

  print("Connecting to UART device (auto-selecting)...")
  uart_dev.connect(consts.UART_BAUD_RATE, consts.UART_TIMEOUT)
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
