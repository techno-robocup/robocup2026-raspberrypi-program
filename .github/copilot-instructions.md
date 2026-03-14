# Copilot instructions (robocup2026-raspberrypi-program)

## Before reading
All agents and automation MUST route every user-facing question, prompt, clarification request, or interactive message through the human-in-the-loop (MCP) before contacting the user. Under no circumstances should an agent contact the user directly without first sending the interaction via the MCP.

## MCP enforcement (non-negotiable)
- Every user-facing interaction must go through MCP first, including: questions, clarifications, progress updates, completion messages, and feedback requests.
- "Operation complete" includes read/search/edit/command/test/documentation steps. After each completed operation, ask the user via MCP whether they want follow-up questions or changes.
- Direct user contact is only allowed when MCP is unavailable and timely input is required. In that case, explicitly note MCP unavailability and continue retrying MCP.
- Do not end a session autonomously. Keep using MCP-mediated interaction until the user explicitly says the session is finished.

- If the MCP is available: send the question via the MCP interface and wait for the MCP's response before proceeding.
- If the MCP is unavailable: the agent MAY prompt the user directly when timely user input is required. The agent should continue attempting to notify/poll the MCP and should record/log that the MCP was unavailable while proceeding.
- After completing any operation, always ask the MCP whether the user wants to ask a follow-up question or provide feedback. If the MCP indicates yes, route that interaction through the MCP; otherwise continue with the next operation.

- Sessions: The assistant must only end a session when the user explicitly instructs it to do so. "End session" is defined as initiating direct prompts to the user without first routing the interaction through the MCP. Agents MUST NOT autonomously end the session or switch to prompting the user directly to close the session unless the user has explicitly granted permission to end the session. If the MCP is unavailable and direct user input is necessary for timely operation, the agent may contact the user for that input but must continue to treat the session as active until the user explicitly confirms they are finished.

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

