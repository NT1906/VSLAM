# VSLAM: visual–inertial SLAM with ArUco landmark anchors

A Flutter app (Android + iOS) for the IE415 course project. The goal is to
estimate the path of a phone carried through a room with ArUco markers on
the walls, build a map of the markers, and show the trajectory relative to
that map.

Development continues on the `iphone-slam` branch, iPhone only, following
`docs/SPEC.md`. Progress, checks run and open items are in
`docs/PROGRESS.md`; the design in `docs/DESIGN.md`.

## Status

| Area | Status |
|---|---|
| Sensor acquisition | ARKit frames (960×720 gray, 10 Hz, fixed focus) with per-frame intrinsics and pose; CoreMotion accelerometer and gyroscope at 100 Hz; all on one timestamp clock. Device checks pending. |
| Dataset save / load | **Done in code (WP1).** Every recording is saved to a session folder; sessions can be listed, opened, renamed, exported (zip) and deleted, and replayed through the same interface as live data. |
| Calibration, time offset | Not started (WP2). |
| ArUco detection | Not started (WP3). |
| Estimation, map, evaluation | Not started (WP4–WP8). |

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
  vision/     camera intrinsics model
  models/     trajectory and track point types
  services/   IMU streaming, recording session (start / stop, saving)
  screens/    home, sessions list, session detail
  widgets/    AR camera view, status tiles, diagnostics, recording settings
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
replay), the session store, and widget tests of recording and managing
sessions. The native ARKit and CoreMotion code can only be tested on a
device: see `docs/DEVICE_CHECKS.md`.

## Next steps

See `docs/SPEC.md` section 5 (work packages WP2–WP8) and `docs/PROGRESS.md`.
