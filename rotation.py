import modules.robot
import modules.constants as consts

robot = modules.robot.robot
uart_dev = modules.robot.uart_io()
uart_devices = uart_dev.list_ports()

# Prioritize USB devices (ESP32 typically appears as /dev/ttyUSB* or /dev/ttyACM*)
usb_devices = [
    d for d in uart_devices if 'USB' in d.device or 'ACM' in d.device
]
if usb_devices:
  selected_device = usb_devices[0]
elif uart_devices:
  selected_device = uart_devices[0]
else:
  sys.exit(1)

uart_dev.connect(selected_device.device, consts.UART_BAUD_RATE,
                 consts.UART_TIMEOUT)
robot.set_uart_device(uart_dev)

while True:
  robot.update_gyro_stat()
  print(f"Gyro: {robot.acc_x} {robot.acc_y} {robot.acc_z} | {robot.roll} {robot.pitch} {robot.yaw}")
  calculated_angle = math.degrees(math.acos(math.cos(robot.roll)*math.cos(robot.pitch)))
  print(f"Calculated angle: {calculated_angle}")
  if abs(calculated_angle) < 10:
    print("Flat")
    robot.set_speed(consts.LEVEL_TURN_SPEED_F, consts.LEVEL_TURN_SPEED_S)
    robot.send_speed()
  else:
    print("Not flat")
    robot.set_speed(1500, 1500)
    robot.send_speed()