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

Blocks: guard, high, high left, high right, mid left, mid right. Their joint angles are set in the `[blocks]` section of `config.toml`; saving a block in the app overrides it (delete `var/blocks.json` to go back to the config values). Names are from the **robot's** point of view (stand behind it): your right-side strikes arrive on the robot's left. Keep blocks compact and in front of the robot, angle the blade across the incoming strike, and check transitions between neighbouring blocks.

## How blocking works

**Zone calibration (recommended):** after setting your guard, click the right thumbstick. The headset panel walks through each block (guard, high, high left, high right, mid left, mid right): show your wind-up for it and press A. The tip positions are saved in `var/zones.json` relative to your guard, so re-centring with A keeps them. While blocking, the robot picks the recorded zone nearest your tip.

Without zone calibration, a fixed 3×3 grid around your guard is used (left/centre/right × high/mid/low); low on a side uses that side's mid block, and low centre is guard. The Mac shows the grid live, so you can check it before the robot moves.

- **Look-ahead**: the zone is chosen from where the tip will be `lookahead_s` ahead, to hide system delay.
- **Hysteresis and dwell**: the robot switches only when another zone is closer by `hysteresis_m` (or, on the grid, past the edge by that much), and the tip must stay in a new zone for `dwell_s` before the robot switches.
- **Strike lock**: above `strike_speed_m_s`, the robot commits to the block for where the strike started and holds it until the tip has been slower for `release_s`.
- **Guard**: when your sword is near your guard, or the controller isn't tracked for `tracking_timeout_s`, the robot returns to guard.

## Headset controls

Open the app in Quest Browser on the headset and tap **Enter**. A status panel in passthrough shows the next step.

| Button | Action |
|---|---|
| Right **B** | Calibrate the blade (point the sword straight up first) |
| Right **A** | Set or re-centre your guard; during zone calibration, record the shown zone |
| Right thumbstick click | Start zone calibration |
| Left **X** | Start blocking |
| Left **Y** | Stop |

Leaving passthrough stops the robot. The laptop's Stop button and space bar also stop it.

## Attacks

On a random timer (`interval_s`, 5–15 s), the robot attacks with a burst of 1–3 strikes (`combo`) chosen from `[attacks.moves]`: chop, slice right and slice left. Each strike moves to its wind-up and swings straight through to its strike pose, starting the swing `blend_deg` before the wind-up is reached. Then it goes back to blocking. The headset panel shows INCOMING and the strike name. Set `enabled = false` under `[attacks]` for blocking only. Strike end poses decide exactly where the blade goes: keep them at blade height, not at the person.

Joint moves are capped by the SDK at 180°/s and 1146°/s² (the xArm 6 maximums). Keep every pose at least 5° inside the joint limits; fast re-planning overshoots by about 2°.

## Robot behaviour

When the chosen block changes, its joint pose is sent once to the controller in online trajectory mode (mode 6). The controller plans the move at `block_speed_deg_s` / `block_accel_deg_s2`, re-plans smoothly if the block changes mid-move, and stops exactly on the taught pose. Before each move the app sets the foam sword payload and `collision_sensitivity` (runtime only, not saved to the controller). A collision error (C31) is cleared automatically and the robot returns to guard and keeps blocking (`collision_auto_home`). Any other controller error ends the session; Home, Move here and Start clear errors and enable the motors.

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
