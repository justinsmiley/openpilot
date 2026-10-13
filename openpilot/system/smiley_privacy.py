"""smileypilot privacy switch.

BLOCK_COMMA_CLOUD = True means this device never talks to comma's cloud:
  - athenad / manage_athenad (comma connect: remote access, route browsing, athena websocket)
  - loggerd uploader (log / route uploads to comma's servers)
Everything is still recorded locally and can be pulled over SSH (smiley_tools/pull_routes.py).

Not covered here: sunnylink (sunnypilot's own service, separate processes), the updater
(talks to GitHub), and one-time device registration (skipped when a dongle id already exists).
Set to False to restore stock behaviour.
"""
BLOCK_COMMA_CLOUD = True
