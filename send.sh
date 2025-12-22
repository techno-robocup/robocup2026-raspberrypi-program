rsync -avzc --delete --exclude .direnv/ --exclude .git/ --exclude .venv/ \
  . robo@roboberry.local:robocup2026-raspberrypi-kanto

# Trigger service restart on remote
ssh robo@roboberry.local "touch /home/robo/restart.trigger"
