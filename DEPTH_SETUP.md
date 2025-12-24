# Depth-Anything-V2 Setup Guide

This project uses Depth-Anything-V2 for depth estimation during linetrace mode.

## Quick Setup (Automated)

Run the setup script to install dependencies and download the model:

```bash
./setup_depth.sh
```

This will:
- Install all required Python dependencies
- Create the `checkpoints/` directory
- Download the small model weights automatically

## Manual Setup

### 1. Install Dependencies

```bash
pip install -r requirements.txt
```

### 2. Create Checkpoints Directory

```bash
mkdir -p checkpoints
```

### 3. Download Model Weights

**Small model (recommended for Raspberry Pi):**
```bash
wget -P checkpoints https://huggingface.co/depth-anything/Depth-Anything-V2-Small/resolve/main/depth_anything_v2_vits.pth
```

Or using curl:
```bash
curl -L -o checkpoints/depth_anything_v2_vits.pth https://huggingface.co/depth-anything/Depth-Anything-V2-Small/resolve/main/depth_anything_v2_vits.pth
```

**Other model sizes (optional):**
- Base: `https://huggingface.co/depth-anything/Depth-Anything-V2-Base/resolve/main/depth_anything_v2_vitb.pth`
- Large: `https://huggingface.co/depth-anything/Depth-Anything-V2-Large/resolve/main/depth_anything_v2_vitl.pth`

Note: Larger models require more computation power and memory.

## Alternative: Custom Model Path

If you prefer to store the model elsewhere, set the `DEPTH_MODEL_PATH` environment variable:

```bash
export DEPTH_MODEL_PATH=/path/to/depth_anything_v2_vits.pth
```

Or add it to your `.envrc` file:
```bash
export DEPTH_MODEL_PATH=/path/to/depth_anything_v2_vits.pth
```

## How It Works

### Mode Switching
- **Linetrace Mode** (`robot.is_rescue_flag == False`):
  - Rescue camera performs depth estimation
  - Saves depth maps to `bin/` directory with timestamp
  - Provides depth information while robot follows the line

- **Rescue Mode** (`robot.is_rescue_flag == True`):
  - Rescue camera callback skips depth processing
  - Only captures images for YOLO object detection
  - Saves processing power for rescue operations

### Output Files
When in linetrace mode, the following files are saved:
- `bin/{timestamp}_depth_input.jpg` - Original RGB image used for depth estimation
- `bin/{timestamp}_linetrace_depth.jpg` - Colorized depth map (INFERNO colormap)
- `bin/{timestamp}_linetrace_depth_raw.jpg` - Normalized depth map (grayscale)

When in rescue mode:
- `bin/{timestamp}_rescue_origin.jpg` - Original RGB image for YOLO detection

## Troubleshooting

### Model fails to load
- Check that the weights file path is correct
- Ensure you have enough memory (depth models can be large)
- Consider using the smaller `vits` model variant

### Slow inference
- Use the `vits` (small) model variant
- Consider reducing input image resolution in `predict_depth()`
- Ensure not running other heavy processes

### CUDA errors
- The code automatically falls back to CPU if CUDA is unavailable
- For Raspberry Pi, CPU mode is expected
