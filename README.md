# Sword

One local app for Quest camera tracking and xArm 6 non-contact pose mirroring.

```sh
./run
```

Open **http://localhost:8765** on the Mac and in Quest Browser. Python 3.11–3.13 is required; use `PYTHON=python3.13 ./run` if your default Python is older. The launcher installs the pinned dependencies into `.venv` on its first run. Everything runs locally. Starting the app does not connect to, enable, home, or move the robot.

## Setup

1. **Print** `web/print-kit.pdf` at 100%, actual size. Page 1 has ArUco 4x4_50 markers 10, 20, and 30. The black square must measure **50 mm**. Page 2 has a 9 × 6 inner-corner calibration board with **25 mm** squares. Measure the print: email printers may resize it.
2. **Mount** marker 10 flat on your sword and marker 20 flat on the robot sword, with their arrows toward the tips. Put the marker centre at the blade base for the dashed blade estimate; the current measured centre-to-tip offset is 40 cm for both swords. Keep the white border. Stand the large checkerboard flat on a rigid, fixed support near the robot, with the heading upright. Keep its complete pattern and both sword markers in view. The checkerboard now supplies the reference; marker 30 is unused in this mode. The supplied board's asymmetric corner colours fix the detector's 180-degree ordering. A single flat marker will disappear when edge-on or occluded; there is no claim of 360-degree tracking.
3. **Link the Quest** using the included helper: `./quest-link` uses an already authorized USB or wireless ADB connection and maps port 8765. If ADB is not on PATH, set `ADB` to its executable. The helper does not grant permissions, change the Quest boundary, or enable wireless debugging automatically.
4. In Quest settings, open **Browser → Permissions → Headset cameras** and allow access to physical surroundings. Then open the app in Quest Browser and choose **Camera + printed markers → Start tracking**. Accept the page's camera prompt if shown. The separate **Virtual cameras** setting concerns avatar/app imagery. This app requests the environment camera and does not silently substitute the avatar camera. The Mac shows the received image and detected marker outlines. If the browser still does not expose its passthrough camera, the app reports that failure; a native camera adapter would still be needed on that device.
5. **The connected Quest already has a saved camera profile on this Mac.** Its own Camera2 lens parameters, transformed for its active crop and 1280 × 960 output, passed three fresh checkerboard checks (0.58, 1.18, and 1.09 px RMS). The app shows **Factory lens · validation 1.18 px**. No lens parameters were fitted to those checks. This device reported no distortion coefficients, so the checked profile assumes zero residual distortion. For another camera, expand **Camera setup**, keep the board flat against a rigid backing, and capture 12 distinct views at different tilts, distances, and image positions. Select **Save calibration**. The last three captures are held out from that fit. Calibration is saved locally in `var/camera.json` for the exact camera identity and delivered resolution; it is not bundled into the generic source ZIP. These image-consistency checks do not establish physical robot alignment, absolute distance, or fast-swing accuracy.
6. Show marker 10 and the complete fixed checkerboard. Hold your normal guard and select **Set my neutral guard**. Capture combines three valid observations from the last second, allowing brief detection gaps, and requires a fresh current pose. This defines relative movement; there is no fingertip/base/tip calibration. The controller and hand alternatives use the right input and need the same single neutral capture. Hand mode is explicitly a wrist estimate, not a measured sword pose.
7. Select **Connect** for read-only robot feedback. **Upright home** is an explicit, checked movement to the saved Studio guard. **Start mirroring** starts the powered session only with fresh tracking and the robot at that guard. There is no session duration limit. **Stop**, the space bar on the Mac, exiting a tracked XR session, or stopping the tracking device ends the session. Tracking gaps hold motion; fresh qualified input can resume the same session.

Both swords are identified from pixels. Their 3D poses use the measured marker size and calibrated camera. The fixed checkerboard supplies a common reference, cancelling headset movement when it is simultaneously visible. The dashed blade line is a rigid extension using the measured marker-to-tip offset (currently 40 cm for both swords), not independently measured blade bending. The Quest sender reads the camera track directly with a one-frame queue, so capture does not depend on the browser drawing its video element. Frame timestamps must advance; after a delivery gap the stream requalifies before sending. Timestamp clock origins are aligned by the minimum observed delivery offset, which exposes additional queueing but does not measure physical exposure-to-actuation latency. Other browsers can fall back to video-frame callbacks. Keep the tracking page visible, or use its active passthrough session; a hidden or suspended page does not provide motion input.

For a shared debugging view, open [Meta Casting](https://horizon.meta.com/casting/) on the Mac, sign into the headset's Meta account, and select Quick controls → Cast → Web/Computer in the headset. Casting is separate from the camera permission needed by this app. Keep Mac and Quest on the same Wi-Fi network.

The sender encodes the newest frame directly. In an eight-frame check on this Quest, the asynchronous JPEG callback took 183–942 ms while direct encoding took 25–28 ms at the same 1280 × 960 resolution and JPEG quality. Avoiding that deferred callback removes an observed source of stale frames; this small diagnostic is not a camera-to-robot latency measurement.

## Robot behavior

`config.toml` is the only motion configuration. It retains the previously admitted upright guard and limits; no SF1 stroke prior, learned joint weighting, extra home tilt, queued stroke playback, or session timer is used. All six joint commands come from a full position-and-orientation target and native inverse kinematics. Native IK on firmware 2.2.2 cannot be seeded with reference angles, so the driver unwraps periodic joints relative to the measured pose and projects requests into the declared workspace. An unreachable request produces a hold or a feasible response rather than terminating the entire session. Projection is not a guarantee of the mathematically nearest pose.

The response uses the original 0.5 translation gain and 1.0 rotation gain. Soft deadbands ignore the first 8 mm and 2° around neutral. A plain detection blink preserves the last measurement only until its original 150 ms expiry; it never refreshes the timestamp or invents a pose. Ambiguous/invalid poses and transport or reference errors still invalidate input. The saved neutral survives detection gaps, and fresh qualified input resumes without another settling period.

Height projection requests 2.5 mm clearance above the existing modeled floor, with at least 2 mm required after IK and joint projection. Measured feedback permits 0.5 mm tolerance: the observed 0.13 mm undershoot no longer produces an immediate fault. A home path starting just below the nominal floor must move inward without going deeper; the native path checks still run. Other workspace limits are unchanged.

The single writer uses independent velocity commands with a 120 ms hardware expiry. It checks fresh source and encoder data, brakes near configured boundaries, and ends on actual violations or native faults. A reference reset invalidates a target even if it occurs during IK. Stop and faults cancel queued actions. A lost source is never extrapolated indefinitely. A Python command-slew or stopping-path calculation is not a certified hardware stopping model. The firmware's protections remain enabled; no errors are automatically cleared. A native IK no-solution response (10) can latch warning 14. The rejected candidate is discarded, and that specifically attributed warning allows a reachable fallback; an unaccounted-for warning or any motor error still stops the session. The old state-dependent `is_tcp_limit` endpoint test is not used as a joint-path admission test. Homing waits for confirmed encoder/planner idle, computes its relative delta from the controller's commanded position, and checks the outward/return path to restore the checker's intermediate state. Only the outward move is physically executed. A remaining C24 is reported with the native check result, not silently ignored.

This app **does not implement autonomous contact interception, unrestricted striking, or disabling the Quest boundary**. The arm's reach, joint limits, singularities, attached tool, and surrounding objects still matter. Full-arm/environment collision planning is not implemented. A successful IK query does not establish collision-free movement. The saved sword mount is an estimate from the earlier setup and must be checked physically before use.

## Small source tree

```text
config.toml             one configuration
run / quest-link        launch and headset connection
sword_app/
  __main__.py           local HTTP/WebSocket app
  vision.py             marker detection and camera calibration
  state.py              latest pose and neutral reference
  math3d.py             coordinate and rotation transforms
  robot.py              sole hardware writer and motion loop
web/
  index.html            one interface
  app.js / tracking.js  operator view and camera/WebXR capture
  style.css             layout
  print-kit.pdf         two print-ready pages
  vendor/               pinned Three.js and its license
checks/test_core.py     offline regression checks; never imported by the app
var/                    local calibration/logs; excluded from Git
```

Camera frames are held in memory for the live preview. They are not written to disk by default or sent to a cloud service. Calibration stores intrinsics and corner observations; failed fits retain their numeric errors and captured corners in `var/camera-attempt.json` for diagnosis. Motion sessions write structured local logs so requested targets, commanded speeds, and actual encoder motion remain distinguishable. The server binds only to loopback; mutation endpoints require the same-origin application token and the tracking socket accepts one producer. The sender only uses a new sample after receiving the server challenge. Pose age starts at challenge issuance; encoding delay is already included and is not counted twice. A contradictory capture age, expired challenge, or non-advancing camera timestamp cannot refresh input. The 150 ms freshness timeout is unchanged. This age estimate still excludes unknown physical camera latency.

## Verification and current limits

Run the compact offline checks with:

```sh
OPENCV_OPENCL_RUNTIME=disabled .venv/bin/python -m unittest discover -s checks -v
```

The cleanup is a new implementation, not a proven drop-in high-speed controller. Thirty-three software checks cover rigid transforms, all-six-axis response, finite stopping distances, stale/reference-reset input, camera identity, known metric marker poses and lens calibration, failed-fit evidence, missing calibration, duplicate calibration views, reference orientation, bounded pose history, neutral capture across detection gaps, unchanged expiry during a blink, soft deadbands, height projection and inward recovery, the homing protocol, specifically attributed IK warnings, stream expiry, and Stop. Ten headless browser checks exercised actual video-frame capture, JPEG transport, and both sword detections and the fixed checkerboard, plus desktop/mobile layouts and direct frame capture with video display callbacks disabled. These checks do not demonstrate physical high-speed tracking or contact performance.

The app has physically homed the actual xArm 6 and produced measured motion on all six axes during Quest following. Earlier runs had substantial tracking holds and one planning-warning stop. A subsequent 25-second observation moved every joint without a controller warning, but 253 of 297 sampled states were holds, chiefly missing sword/reference detections. After that observation, the same session stopped on a 0.13 mm modeled-height undershoot with zero native errors/warnings. The revised tolerance and inward home path physically returned it to guard with both native path checks passing. The final full-gain/blink-handling profile still needs a live tracking check; its attempted start received no valid sword-plus-board pose. All results are retained in `VERIFICATION.json`.

These observations do not establish accurate sword-tip matching or robust fast-swing performance. Actual Quest camera capture at 1280 × 960, detection of the printed markers and fixed checkerboard, and reload of the physically checked factory camera profile have passed. The earlier 12-view lens fit failed and was not installed; its detailed numeric errors were not retained by that version. The user confirmed the printed black marker squares measure 50 mm. Mounted sword alignment and end-to-end latency remain unverified. Neither hand availability nor synthetic vision tests establish those results.

The earlier experiments, recordings, tests, SF1 analysis, and source snapshots are preserved outside this repository in the workspace archive. They are not dependencies of this app.

## Upstream references

- [Meta browser camera access](https://developers.meta.com/vr/blog/q4-developer-recap-whats-new-across-meta-horizon-os/)
- [Meta camera-permission issue investigation](https://developers.meta.com/vr/support/investigations/1557993751977106/)
- [Meta casting setup](https://www.meta.com/help/quest/192719842695017/)
- [OpenCV ArUco detection](https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html)
- [xArm Python SDK](https://github.com/xArm-Developer/xArm-Python-SDK)

Three.js 0.170.0 is vendored with its MIT license. Python dependencies retain their upstream licenses. This repository contains no account credentials.

The single 50 mm reference marker was not stable enough in the hardware check: its planar orientation was often ambiguous. The same app now supports the larger printed checkerboard as its fixed reference (`vision.reference = "checkerboard"`). The older `"marker"` setting is explicit; the app never switches reference frames automatically. Moving the fixed reference requires setting a new neutral guard. When a sword marker has two plausible poses, its recent measured orientation can select between the current image's candidates. The current board pose compensates for headset movement; history expires after 150 ms and cannot supply an occluded marker. A selection requires a candidate within 20° of that recent orientation and at least 10° closer than the alternative, while preserving the 2 px image-fit requirement. Such poses are labelled `recent_pose_and_image` in telemetry. This reduces some ambiguity; it does not establish ground-truth accuracy or resolve every fast rotation.
