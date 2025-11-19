import modules.constants as consts
import modules.logger
import modules.robot

robot = modules.robot.Robot()

if __name__ == "__main__":
  uart_dev = modules.robot.uart_io()
  uart_devices = uart_dev.list_ports()
  uart_dev.connect(uart_devices[0].device, consts.UART_BAUD_RATE, consts.UART_TIMEOUT)
  robot.set_uart_device(uart_dev)
