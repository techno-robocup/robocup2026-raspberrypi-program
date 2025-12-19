# robocup2026-raspberrypi-kanto

20205/12/19
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
```
uv run python main.py
```

`touch restart.trigger` on home directory
