UART_BAUD_RATE = 9600
UART_TIMEOUT = 0.2
MOTOR_MAX_SPEED = 2000
MOTOR_MIN_SPEED = 1000
Rescue_Camera_Port = 0
Rescue_Camera_Controls = {
    "AfMode": controls.AfModeEnum.Continuous,
    "AfSpeed": controls.AfSpeedEnum.Fast,
    "AeFlickerMode": controls.AeFlickerModeEnum.Manual,
    "AeFlickerPeriod": 10000,
    "AeMeteringMode": controls.AeMeteringModeEnum.Matrix,
    "AwbEnable": True,
    "AwbMode": controls.AwbModeEnum.Indoor,
    "HdrMode": controls.HdrModeEnum.Off
}
Rescue_Camera_Size = (4608, 2592)
Rescue_Camera_Formats = "RGB888"
Rescue_Camera_lores = (Rescue_Camera_Size, Rescue_Camera_Size)
Rescue_Camera_precallback = None

Linetrace_Camera_Port = 1
Linetrace_Camera_Controls = {
    "AfMode": controls.AfModeEnum.Manual,
    "LensPosition": 1.0 / 0.03,
    "AeFlickerMode": controls.AeFlickerModeEnum.Manual,
    "AeFlickerPeriod": 10000,
    "AeMeteringMode": controls.AeMeteringModeEnum.Matrix,
    "AwbEnable": False,
    "AwbMode": controls.AwbModeEnum.Indoor,
    "HdrMode": controls.HdrModeEnum.Night
}
Linetrace_Camera_Size = (4608, 2592)
Linetrace_Camera_Formats = "RGB888"
Linetrace_Camera_lores = (Linetrace_Camera_Size // 8,
                          Linetrace_Camera_Size // 8)
Linetrace_Camera_precallback = None

if __name__ == "__main__":
  pass
