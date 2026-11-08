# Sword

Local app that makes an xArm 6 block foam-sword strikes. The robot holds a set of taught block poses and moves to the one that covers where your sword is threatening, tracked by a Quest controller taped to your hilt.

```sh
./run
```

Open **http://localhost:8765** on the Mac and in Quest Browser. Python 3.11–3.13 is required (`PYTHON=python3.13 ./run` if your default is older). The launcher installs pinned dependencies into `.venv` on first run. Starting the app does not connect to or move the robot.

## Setup

1. **Tape the right Quest controller** firmly to the sword's hilt. Set `blade_length_m` in `config.toml` to the distance from the controller grip to the blade tip.
2. **Link the Quest**: `./quest-link` maps port 8765 over an authorized ADB connection (set `ADB` if it's not on PATH).
3. In Quest Browser, open the app and press **Start tracking on this headset** (enters passthrough).
4. **Calibrate** with the controller buttons (the controller buzzes on each press):
   - Point the sword **straight up** and press **B**. This is saved in `var/blade.json`; redo it if you re-tape the controller.
   - Hold your normal guard, facing the robot, and press **A**. Press A again any time to re-centre.
5. On the Mac: **Connect**, then **teach the blocks** (below), then **Start blocking**.
   **Stop**, the space bar, or stopping tracking on the headset ends the session.

## Teaching blocks

Only the end pose of each block is recorded; the app plans the motion between them (all joints arrive together).

1. Press **Freedrive on** and move the arm by hand.
2. Put the sword where the block should be and press **Save** next to its name. Poses are saved in `var/blocks.json`.
3. Press **Move here** to replay a block slowly and check it.

Blocks: guard, high, high left, high right, mid left, mid right, low left, low right, centre (thrust). Names are from the **robot's** point of view (stand behind it): your right-side strikes arrive on the robot's left. Untaught blocks fall back to guard. Keep blocks compact and in front of the robot, angle the blade across the incoming strike, avoid poses with J5 near 0° (wrist singularity), and check transitions between neighbouring blocks.

## How blocking works

The blade tip's position relative to your guard picks a zone on a 3×3 grid (left/centre/right × high/mid/low). In the centre, a fast push toward the robot is a thrust; otherwise it is guard. The Mac shows the grid live, so you can check it before the robot moves.

- **Look-ahead**: the zone is chosen from where the tip will be `lookahead_s` ahead, to hide system delay.
- **Hysteresis and dwell**: zone edges overlap by `hysteresis_m`, and the tip must stay in a new zone for `dwell_s` before the robot switches.
- **Strike lock**: above `strike_speed_m_s`, the robot commits to the block for where the strike started and holds it until the tip has been slower for `release_s`.
- **Guard**: when your sword is near your guard, or the controller isn't tracked for `tracking_timeout_s`, the robot returns to guard.

## Robot behaviour

The robot follows the chosen block with coordinated joint-velocity control, capped by `max_speed_deg_s` and `max_accel_deg_s2`. Each velocity command expires after `velocity_lease_s` on the controller, so the arm stops if the app dies. On connect, the app sets the foam sword payload and `collision_sensitivity` (runtime only, not saved to the controller). A collision error (C31) is cleared automatically and the robot returns to guard and keeps blocking (`collision_auto_home`). Any other controller error ends the session; Home, Move here and Start clear errors and enable the motors.

There is no app-side workspace or collision checking beyond the taught poses. Use foam swords, teach blocks that meet strikes in front of the robot rather than reaching toward you, and configure any extra boundaries in xArm Studio.

## Source

```text
config.toml             configuration
run / quest-link        launcher and headset port mapping
sword_app/
  __main__.py           local HTTP/WebSocket server
  blocking.py           threat zones, hysteresis, strike lock
  state.py              controller pose, smoothing, blade/guard calibration
  math3d.py             rotation helpers
  robot.py              robot control loop and teaching
web/                    operator page and WebXR controller tracking
checks/test_core.py     offline tests (python -m unittest discover checks)
var/                    blocks, blade calibration and session logs (git-ignored)
```
