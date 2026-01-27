rsync -avzc --delete --exclude .direnv/ --exclude .git/ --exclude .venv/ --exclude temp_images . robo@roboberry.local:robocup2026-raspberrypi-program

# Trigger service restart on remote
ssh robo@roboberry.local "touch /home/robo/restart.trigger"
