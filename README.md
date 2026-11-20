# VSLAM: visual–inertial SLAM with ArUco landmark anchors

A Flutter app (Android + iOS) for the IE415 course project. The goal is to
estimate the path of a phone carried through a room with ArUco markers on
the walls, build a map of the markers, and show the trajectory relative to
that map.

Development continues on the `iphone-slam` branch, iPhone only, following
`docs/SPEC.md`. Progress, checks run and open items are in
`docs/PROGRESS.md`; the design in `docs/DESIGN.md`; the handover and plan for
the remaining work in `docs/HANDOFF.md`.

## Status

| Area | Status |
|---|---|
| Sensor acquisition | ARKit frames (960×720 gray, 10 Hz, fixed focus) with per-frame intrinsics and pose; CoreMotion accelerometer and gyroscope at 100 Hz; all on one timestamp clock. Device checks passed (WP1). |
| Dataset save / load | **Done in code (WP1).** Every recording is saved to a session folder; sessions can be listed, opened, renamed, exported (zip) and deleted, and replayed through the same interface as live data. |
| Calibration, time offset | **Done (WP2).** Chessboard calibration sheet (OpenCV, k1/k2 only) with ARKit comparison; camera–IMU time offset and rotation from a rotation recording (session screen → IMU alignment). Device checks passed: RMS 0.467 px; R_ci 0.10° from Apple's axes. |
| ArUco detection | **Done in code (WP3).** `DICT_4X4_50` with sub-pixel corners, IPPE pose, the spec's rejection rules; live outlines, IDs and distances on the camera view; session screen → Detect markers writes `results/detections.csv`. Marker sheet `docs/markers_4x4_50_ids0-9.pdf`. Device check passed: 1.006 m at 1 m, 2.058 m at 2 m. |
| Keyframes, baselines, first map | **Done in code (WP4), device check in progress.** Session screen → Build map: keyframes (0.25 m / 15° / marker ≥ 0.3 s), baseline 1 (ARKit odometry chained), baseline 2 (IMU dead reckoning, biases from a still window), each marker placed at first sight; writes `results/traj_arkit.csv`, `traj_imu.csv`, `map.json`; Save map → `maps/<room>.json`. Tested on simulated recordings. |
| Pose-graph optimizer, relocalization, map view, evaluation | Not started (WP5–WP8). |

## What the app does now

- Opens the ARKit camera session and shows the live feed with a tracking
  badge (initialising, tracking, limited with reason, stopped).
- A settings row sets the label, experiment type, marker size and notes of
  the next recording.
- **Start / Stop** records camera frames (JPEG), ARKit poses and IMU samples
  to `Documents/sessions/<date>_<time>_<label>/` (visible in the Files app).
- The folder icon lists saved sessions; each shows its metadata, measured
  rates, frame gaps and first image, and can be renamed, exported or deleted.
- The diagnostics sheet (heart-monitor icon) shows AR support, tracking
  reason, image size and intrinsics, IMU readings and the device model.
- The camera view outlines detected ArUco markers live with their ID and
  distance (green: usable for the map, orange: rejected, with the reason).
- The calibration sheet (grid icon) calibrates the camera from a printed
  chessboard; the session screen runs IMU alignment and marker detection on
  a saved session.
- The session screen's **Build map** selects keyframes, computes the
  ARKit-only and IMU-only baselines and a first marker map, and shows
  keyframe counts, the IMU rest window and biases, how far the IMU drifts
  from ARKit, and each marker's position; **Save map** stores the map under
  `Documents/maps/`.

## Requirements

| | Android | iOS |
|---|---|---|
| OS | 7.0+ (API 24) | 15.0+ |
| AR | [ARCore-supported device](https://developers.google.com/ar/devices) with Google Play Services for AR | ARKit device (A9 or newer) |
| Sensors | Accelerometer and gyroscope | Built in |

Use a real phone. Emulators and simulators can't run AR.

## Build and run

```bash
flutter pub get
flutter run --release   # with a phone connected
```

Developed with Flutter 3.47.9 (Dart 3.13.7) and Xcode 27.2.

- **Android:** Android SDK (API 36) and JDK 17. ARCore comes from Gradle.
- **iOS:** Xcode. Open `ios/Runner.xcworkspace` once and set a development
  team.

## Project layout

```
lib/
  core/       SE(3) poses and rotations
  dataset/    sensor events, session writer / reader / replay, session store
  slam/       AR pose / frame messages and the native AR session service
  vision/     camera model, ArUco detection + pose, detections file, marker sheet
  calibration/ chessboard calibration, camera–IMU alignment
  estimation/ configuration, keyframes, baselines (ARKit, IMU dead reckoning)
  mapping/    marker map, map files
  models/     trajectory and track point types
  services/   IMU streaming, recording session (start / stop, saving)
  screens/    home, sessions list, session detail
  widgets/    AR camera view, marker overlay, status tiles, diagnostics, settings
tools/        make_board.dart (chessboard SVG), make_markers.dart (marker PDF)
android/…/ArPlugin.kt, ArPlatformView.kt   ARCore session, pose + frame streaming
ios/Runner/ArPlugin.swift                  ARKit session, frames + JPEG, CoreMotion IMU
docs/       spec, handout, design, progress, device checks, experiments
test/         unit and widget tests
```

## Tests

```bash
flutter analyze
flutter test
```

The tests cover rotations, SE(3) and the camera model, AR pose parsing,
trajectories, the recording session, the dataset format (write, read back,
replay), the session store, calibration and IMU alignment on synthetic data,
ArUco detection on markers rendered through the calibrated camera model
(ID, corners within 0.5 px, pose, rejection rules), the marker overlay,
keyframe selection on scripted paths, IMU propagation against closed forms,
map files, the WP4 pipeline end to end on a simulated room walk
(`test/support/simulator.dart`), and
widget tests of recording and managing sessions. OpenCV runs under
`flutter test` on macOS. The native ARKit and CoreMotion code can only be tested on a
device: see `docs/DEVICE_CHECKS.md`.

## Next steps

See `docs/SPEC.md` section 5 (work packages WP5–WP8) and `docs/PROGRESS.md`.
