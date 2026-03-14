# Copilot instructions (robocup2026-raspberrypi-program)

# Before reading
When you are going to ask an question or prompt a user question, always ask through human-in-the-loop mcp if the mcp is available. If the mcp is available, ask the question through the mcp and wait for the answer. If the mcp is not available, ask the question directly to the user.

## Build / run / lint

### Python environment (uv)
- Install runtime deps: `uv sync`
- Install dev tools (formatter/linter): `uv sync --group dev`
- Run manually (README): `uv run python main.py`

### Format + lint (CI)
CI runs `./format.sh` and expects **no diff** afterwards (`git diff --exit-code`).

- Full formatter/lint: `./format.sh`
- Individual commands (mirrors `format.sh`):
  - `uv run isort .`
  - `uv run yapf --style .style.yapf --in-place --recursive . --exclude .direnv/ --exclude .venv`
  - `uv run flake8 --exclude .venv --ignore E111,E114,E501,E126,W504,E125,W503`

### Depth model setup (Depth-Anything-V2)
- Automated: `./setup_depth.sh` (runs `uv sync`, downloads weights into `checkpoints/`, creates `bin/`)
- Optional override: `DEPTH_MODEL_PATH=/path/to/depth_anything_v2_vits.pth`

## Deployment (systemd + scripts)

### systemd services (README)
- Install:
  - `sudo cp robot.service robot.path robot-restart.service /etc/systemd/system/`
  - `sudo systemctl daemon-reload`
  - `sudo systemctl enable --now robot.service robot.path`
- Trigger a restart (watched by `robot.path`): `touch /home/robo/restart.trigger`

⚠️ `robot.service` hardcodes paths (repo dir + uv path). Keep it in sync with where you deploy
the repo (see `send.sh` / `fetch*.sh` which assume `robocup2026-raspberrypi-program/` on the Pi).

### Push/pull helpers
- Deploy code to the robot and trigger restart: `./send.sh`
- Fetch debug images from the robot: `./fetch.sh`
  - YOLO inputs only: `./fetch_origin.sh`
  - YOLO rendered results only: `./fetch_result.sh`

## High-level architecture (big picture)

### Runtime dataflow
- **Two Picamera2 streams** run continuously (created in `modules/robot.py`):
  - *Linetrace camera* (port `1`): detects black line + green/red marks
  - *Rescue camera* (port `0`): provides frames for YOLO target detection
- **ESP32 over UART** provides sensors + executes motor/arm commands.
- `main.py` is the control loop: it polls robot state and sends motor/arm commands based on mode.

### Key modules
- `main.py`
  - Connects to ESP32 (auto-picks `/dev/ttyUSB*` / `/dev/ttyACM*` first), then runs an infinite loop:
    - **Linetrace mode** (`robot.is_rescue_flag == False`): uses `robot.linetrace_slope/line_area/...`
    - **Rescue mode** (`robot.is_rescue_flag == True`): calls `find_best_target()` (YOLO) + ball/cage/exit logic
  - YOLO inference is guarded by `yolo_lock` (Ultralytics model is not treated as thread-safe).
- `modules/robot.py`
  - `uart_io`: ID-prefixed request/response protocol + healthcheck/reconnect.
  - `Robot`: thread-safe shared state; starts both cameras in `__init__`.
  - **Import side effect**: `robot = Robot()` at module import time starts Picamera2 immediately.
- `modules/camera.py`
  - `Camera`: Picamera2 wrapper; each camera uses a `pre_callback` (hot path).
  - `Linetrace_Camera_Pre_callback(...)`:
    - rotates image 180°, crops + resizes, glare reduction, thresholding, contour selection
    - writes `robot.linetrace_slope`, `robot.line_area`, `robot.line_center_x`, `robot.top_checkpoint_black`
    - detects **green marks** (X markers) and whether black lines exist around them, writes `robot.green_marks/...`
    - detects **red marks** and sets `robot.write_linetrace_stop(True)`
  - `Rescue_precallback_func(...)`: captures lores rescue frame and writes `robot.rescue_image` (+ timestamp).
  - Depth-Anything-V2 is implemented as lazy utilities (`get_depth_model()` / `predict_depth()`).
- `modules/constants.py`
  - Camera controls, thresholds, `TargetList` (YOLO class ID mapping).
  - **Import side effect**: `MODEL = YOLO("best.pt")` loads weights at import time.

## Repo-specific conventions / gotchas

- **Python formatting uses 2-space indents** (`.style.yapf`); rely on `./format.sh` instead of hand-formatting.
- **Thread safety**: shared state uses `readerwriterlock.rwlock.RWLockFairD()` (`gen_rlock()` / `gen_wlock()`).
  Use `robot.write_*()` + properties rather than adding unprotected globals.
- **Motor speed convention** (used everywhere): `1500 == stop`, `>1500` forward, `<1500` reverse; clamp to `[1000, 2000]`.
- **Picamera2 `pre_callback` is a hot path**: avoid adding heavy inference or slow I/O there; prefer the main loop.
- Debug artifacts are written frequently:
  - timestamped images under `bin/` (linetrace origin/binary/tracking; rescue origin/result; masks)
  - logs append to `log.log` (see `modules/logger.py`)
- `modules/constants.py` contains a guarded constant:
  - `MIN_BLACK_LINE_AREA` is marked **"DO NOT REWRITE"** and is used in both `modules/camera.py` and `main.py`.

