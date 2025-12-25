#!/bin/bash
# Fast parallel fetch of debug images

echo "Fetching debug images in parallel..."

# Create bin directory if it doesn't exist
mkdir -p ./bin

# Method 1: If you have GNU parallel installed (fastest)
if command -v parallel &>/dev/null; then
  echo "Using GNU parallel for maximum speed..."
  ssh robo@roboberry.local "find robocup2026-raspberrypi-kanto/bin/ -type f -printf '%P\n'" 2>/dev/null |
    parallel -j 16 --bar \
      rsync -az robo@roboberry.local:robocup2026-raspberrypi-kanto/bin/{} ./bin/{}

  # Cleanup deleted files
  rsync -az --size-only --delete --existing robo@roboberry.local:robocup2026-raspberrypi-kanto/bin/ ./bin/

# Method 2: Fallback to xargs (works everywhere)
else
  echo "GNU parallel not found, using xargs (install 'parallel' for faster transfers)..."
  ssh robo@roboberry.local "cd robocup2026-raspberrypi-kanto/bin && find . -type f" 2>/dev/null |
    xargs -P 32 -I {} \
      rsync -az robo@roboberry.local:robocup2026-raspberrypi-kanto/bin/{} ./bin/{}

  # Cleanup deleted files
  rsync -az --size-only --delete robo@roboberry.local:robocup2026-raspberrypi-kanto/bin/ ./bin/
fi

echo "✓ Fetch complete!"
