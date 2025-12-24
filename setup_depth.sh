#!/bin/bash
# Setup script for Depth-Anything-V2 integration

set -e

echo "Setting up Depth-Anything-V2 for RoboCup 2026 project..."

# Install Python dependencies
echo "Installing Python dependencies..."
uv sync

# Create checkpoints directory
echo "Creating checkpoints directory..."
mkdir -p checkpoints

# Download small model (recommended for Raspberry Pi)
echo "Downloading Depth-Anything-V2 Small model..."
if command -v wget &>/dev/null; then
  wget -P checkpoints https://huggingface.co/depth-anything/Depth-Anything-V2-Small/resolve/main/depth_anything_v2_vits.pth
elif command -v curl &>/dev/null; then
  curl -L -o checkpoints/depth_anything_v2_vits.pth https://huggingface.co/depth-anything/Depth-Anything-V2-Small/resolve/main/depth_anything_v2_vits.pth
else
  echo "Error: Neither wget nor curl found. Please install one of them."
  exit 1
fi

# Create bin directory if it doesn't exist
echo "Creating bin directory for depth output..."
mkdir -p bin

echo ""
echo "Setup complete!"
echo ""
echo "Depth estimation will run automatically during linetrace mode."
echo "Output files will be saved to the bin/ directory:"
echo "  - bin/{timestamp}_depth_input.jpg"
echo "  - bin/{timestamp}_linetrace_depth.jpg"
echo "  - bin/{timestamp}_linetrace_depth_raw.jpg"
