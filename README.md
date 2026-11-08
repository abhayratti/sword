# Sword

Local app for Quest camera tracking and xArm 6 pose mirroring.

```sh
./run
```

Open **http://localhost:8765** on the Mac and in Quest Browser. Python 3.11–3.13 is required (`PYTHON=python3.13 ./run` if your default is older). The launcher installs pinned dependencies into `.venv` on first run. Starting the app does not connect to or move the robot.

## Setup

1. **Print** `web/print-kit.pdf` at 100%. Page 1 has ArUco 4x4_50 markers 10 and 20 (50 mm black square). Page 2 is a 9 × 6 inner-corner checkerboard with 25 mm squares.
2. **Mount** marker 10 on your sword and marker 20 on the robot sword, arrows toward the tips. Stand the checkerboard on a fixed support near the robot, keeping it and your marker in the camera view. The board is the shared reference that cancels headset movement.
3. **Link the Quest**: `./quest-link` maps port 8765 over an authorized ADB connection (set `ADB` if it's not on PATH).
4. In Quest settings, allow **Browser → Permissions → Headset cameras**. Open the app in Quest Browser and choose **Camera + printed markers → Start tracking**.
5. **Calibrate the camera** once (Camera setup → capture 12 board views at varied tilts and positions → Save calibration). It is saved in `var/camera.json` per camera and resolution.
6. Hold your normal guard and select **Set my neutral guard**.
7. **Connect**, optionally **Upright home**, then **Start mirroring**. The robot mirrors your motion relative to wherever it is when mirroring starts. **Stop**, the space bar, or stopping tracking ends the session.

Controller and hand-wrist tracking are available as alternatives to markers.

## Robot behavior

All motion settings are in `config.toml`. Each control tick reads the joints, solves native IK for the mirrored pose, and sends a joint velocity toward that solution, capped by `max_speed_deg_s` and `max_accel_deg_s2`. IK targets are clamped to the controller's joint ranges. If your marker leaves the frame, the robot continues along its recent motion for `prediction_s`, then holds that pose; it resumes as soon as the marker returns. The session never ends because of tracking loss. If the reference board leaves the frame, its last measured pose is reused for `board_memory_s`; that is accurate only while your head (the camera) stays still. If IK has no solution, the last reachable target is kept. Tracked poses are smoothed with a One Euro filter (steady when still, responsive when swinging), small deadbands ignore jitter around your guard, flipped marker poses are discarded, and IK solutions that suddenly switch arm configuration are ignored unless they persist. Tune these under `smoothing_*`, `deadband_*`, and `ik_jump_deg`.

Each velocity command expires after `velocity_lease_s` on the controller, so the arm stops if the app dies. Controller errors end the session; warnings are ignored. **Home** and **Start** clear warnings and errors and enable the motors.

There is no app-side workspace, floor, or collision checking. Configure any boundaries you need in xArm Studio (reduced mode / safety boundary), where the firmware enforces them.

## Source

```text
config.toml             configuration
run / quest-link        launcher and headset port mapping
sword_app/
  __main__.py           local HTTP/WebSocket server
  vision.py             marker detection and camera calibration
  state.py              latest pose and neutral guard
  math3d.py             transforms
  robot.py              robot control loop
web/                    interface, camera/WebXR capture, print kit
checks/test_core.py     offline tests (python -m unittest discover checks)
var/                    calibration and session logs (git-ignored)
```
