# robocup2026-raspberrypi-kanto

2025/12/19
```bash
sudo usermod -aG video robo
sudo usermod -aG dialout robo
```

Creating pyenv
```bash
python3 -m venv .venv --system-site-packages
```

```bash
pip install --break-system-packages uv
```


```bash
uv init
uv add opencv-python ultralytics gradio_imageslider gradio matplotlib torch torchvision pyserial Pillow huggingface-hub depth-anything-v2

```


Modify
```
include-system-site-packages = true
```

## How to run

### Manual execution
```bash
uv run python main.py
```

### Systemd service setup

The robot runs as a systemd service with automatic restart capabilities.

#### Installation

1. Copy service files to systemd directory:
```bash
sudo cp robot.service robot.path robot-restart.service /etc/systemd/system/
```

2. Reload systemd and enable services:
```bash
sudo systemctl daemon-reload
sudo systemctl enable robot.service robot.path
sudo systemctl start robot.service robot.path
```

3. Check status:
```bash
sudo systemctl status robot.service
sudo systemctl status robot.path
```

#### Restart trigger

To restart the robot service remotely or via automation:
```bash
touch /home/robo/restart.trigger
```

This will automatically trigger a service restart via the path monitoring mechanism.

#### Service architecture

- **robot.service**: Main robot process (runs continuously)
- **robot.path**: Monitors `/home/robo/restart.trigger` for changes
- **robot-restart.service**: Oneshot service that restarts robot.service when triggered
