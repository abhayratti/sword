# Instructions for CODEX (and other coding agents)

CODEX: read this before running or changing anything in this repo. It controls a real xArm 6 robot arm.

## What this is

A local web app that makes an xArm 6 block foam-sword strikes. A right Quest controller taped to the person's sword hilt is tracked in Quest Browser (WebXR). The app picks one of six taught block poses from where the sword tip is threatening and moves the robot there. See `README.md` for how blocking works.

## Running

Requires Python 3.11–3.13. Run everything from the repo root on the Mac.

```sh
# 1. Offline tests: no robot or headset needed. Run these first, after every change.
python3 -m venv .venv && .venv/bin/python -m pip install -r requirements.txt
.venv/bin/python -m unittest discover checks

# 2. Start the app (installs dependencies into .venv on first run if needed).
./run                      # or: PYTHON=python3.13 ./run

# 3. Map the port to the headset (needs an authorized ADB connection).
./quest-link
```

Then open http://localhost:8765 on the Mac and in Quest Browser on the headset.

Starting the app does **not** connect to or move the robot. The robot only moves when a person presses Connect, Home, Move here, Freedrive, or Start blocking in the page.

## Operator sequence (done by a human, not by you)

1. Headset: press **Start tracking on this headset** (enters passthrough).
2. Controller: point the sword straight up and press **B** (blade calibration, saved to `var/blade.json`).
3. Controller: hold a normal guard facing the robot and press **A** (sets the guard).
4. Mac: **Connect**, check each block with **Move here**, then **Start blocking**.
5. **Stop**, the space bar, or stopping tracking on the headset ends the session.

## Rules for CODEX

- **Never connect to, enable, or move the robot yourself.** Do not call the `/api/action` endpoint, run scripts that import `xarm`, or press buttons via browser automation. Only the human operator moves the robot.
- **Do not run `./run` expecting to test robot behaviour.** Verify changes with the offline tests in `checks/`. They use a fake arm and never touch hardware.
- **Do not change the block joint angles** in `config.toml` `[blocks]` unless explicitly asked. They were set by hand on the real robot. J2 = 118° is 2° from the xArm 6 joint limit (120°); never push it higher.
- **Do not raise `max_speed_deg_s` or `max_accel_deg_s2`** unless explicitly asked. They are deliberately conservative (60°/s, 200°/s²).
- **Do not save controller settings permanently.** The app sets payload and collision sensitivity at runtime only; do not add `save_conf()` calls.
- **Do not commit `var/`.** It holds local calibration (`blade.json`), taught poses (`blocks.json`) and session logs, and is git-ignored.
- Keep the app local: the server binds to 127.0.0.1 and mutation requests need the per-run `X-Sword-Key` token. Do not weaken this.
- Keep Stop working: the Stop button, space bar, stopping on tracker disconnect, and the velocity lease (`velocity_lease_s`, so the arm stops if the app dies) must stay.

## Configuration (`config.toml`)

- `[robot]`: robot IP, speed and acceleration caps, payload, collision sensitivity, auto-return-to-guard on collision (C31).
- `[tracking]`: `blade_length_m` (controller to blade tip; measure it), tracking timeout, smoothing.
- `[blocking]`: zone sizes, hysteresis, dwell, look-ahead, strike lock.
- `[blocks]`: joint angles (degrees, J1–J6) for guard, high, high_left, high_right, mid_left, mid_right. Names are from the robot's point of view. Blocks saved from the app's teach panel override these via `var/blocks.json`; deleting that file restores the config values.

## Code map

```text
sword_app/__main__.py   HTTP/WebSocket server (local only)
sword_app/blocking.py   threat zones, hysteresis, strike lock (pure logic, easy to test)
sword_app/state.py      controller pose, smoothing, blade/guard calibration
sword_app/robot.py      the only code that talks to the robot
sword_app/math3d.py     rotation helpers
web/                    operator page (app.js) and WebXR controller tracking (tracking.js)
checks/test_core.py     offline tests
```

## Debugging

Each blocking session writes `var/run-<timestamp>.jsonl` with the chosen block, tip offset and speed, strike lock, joint angles and commanded velocities per tick. Ask the human for this file instead of guessing what the robot did.
