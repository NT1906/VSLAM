# VSLAM: visual–inertial SLAM with ArUco landmark anchors

A Flutter app (Android + iOS) for the IE415 course project. The goal is to
estimate the path of a phone carried through a room with ArUco markers on
the walls, build a map of the markers, and show the trajectory relative to
that map.

This repository is the **mid-evaluation snapshot**. It covers sensor
acquisition and the recording interface. The marker pipeline and the
estimator are still in progress and are not included yet.

## Mid-evaluation status

| Area | Status |
|---|---|
| Sensor acquisition & calibration | **In progress.** Camera (ARCore/ARKit session) and IMU (accelerometer + gyroscope) are recorded separately. Camera calibration and camera–IMU time sync are not done yet. |
| Fiducial landmarks / ArUco | **Not started.** |
| Visual / visual–inertial odometry | **Feature tracking implemented.** Image features are tracked by the ARCore / ARKit session. The app streams its tracking state, poses and grayscale frames into Dart. |
| User interface | **Start / Stop recording** and a **real-time camera / sensor status display** are ready. |
| Dataset save / load | Not yet. |
| Visualizer (2D/3D trajectory & marker plot) | Not yet. |

## What the app does now

- Opens the native AR camera session (ARCore on Android, ARKit on iOS) and
  shows the live camera feed with a tracking badge (initialising, tracking,
  limited with reason, stopped).
- Streams grayscale camera frames (luma, ~10 Hz) and camera poses from the
  native side over platform channels, and measures the frame rate.
- Streams the accelerometer and gyroscope with `sensors_plus` and measures
  their sample rates. A watchdog reports a missing gyroscope.
- **Start / Stop** begins and ends a recording session. Status tiles show
  camera state, frame rate, tracked/total AR poses, IMU state, IMU rates and
  elapsed time.
- The diagnostics sheet (heart-monitor icon) shows AR support, tracking
  reason, image size and AR SDK intrinsics, and the latest IMU readings.

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

- **Android:** Android SDK (API 36) and JDK 17. ARCore comes from Gradle.
- **iOS:** Xcode. Open `ios/Runner.xcworkspace` once and set a development
  team.

## Project layout

```
lib/
  core/       SE(3) poses and rotations
  slam/       AR pose / frame messages and the native AR session service
  vision/     camera intrinsics model
  models/     trajectory and track point types
  services/   IMU streaming, recording session (start / stop)
  screens/    home screen
  widgets/    AR camera view, sensor status tiles, diagnostics panel
android/…/ArPlugin.kt, ArPlatformView.kt   ARCore session, pose + frame streaming
ios/Runner/ArPlugin.swift                  ARKit session, pose + frame streaming
test/         unit and widget tests
```

## Tests

```bash
flutter analyze
flutter test
```

The tests cover rotations, AR pose parsing and alignment, trajectories, the
recording session (start/stop with AR poses and IMU samples), the IMU
watchdog, and a widget test of the start/stop flow. The native ARCore/ARKit
code can only be tested on a device.

## Next steps

1. Camera calibration (intrinsics and distortion) and synchronised
   camera–IMU logging with verified timestamps.
2. ArUco detection and per-marker pose estimation.
3. Visual-inertial odometry trajectory, then marker-based drift correction.
4. Dataset save/load and the trajectory/marker visualizer.
