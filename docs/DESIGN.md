# VSLAM on iPhone: system design

This document describes the system as it is built so far. It grows with each
work package (see `docs/SPEC.md` section 5 and `docs/PROGRESS.md`). Sections
for parts that do not exist yet are not written in advance.

## 1. Architecture

One Flutter app on the iPhone does acquisition, storage, replay and (in later
packages) processing and display. Native Swift code (`ios/Runner/ArPlugin.swift`)
owns the ARKit session and CoreMotion; everything after acquisition is Dart.

```
ARKit session ──► ArPlugin.swift ──► vslam/ar_pose   (ARKit pose, ~30 Hz)
  (camera,         box filter,   ──► vslam/ar_frames (960×720 gray + JPEG +
   VIO)            JPEG encode                         intrinsics + pose, 10 Hz)
CoreMotion ──────► ImuStream     ──► vslam/imu       (accel + gyro, 100 Hz)
                                         │
                       SlamService / ImuService (Dart)
                                         │
                                 LiveSensorSource ─┐
                                                   ├─► SensorSource ─► processing (WP3+)
  sessions/<folder>/ ─► SessionReader ─► ReplaySource ┘
          ▲
          └── SessionRecorder ◄── LiveSensorSource   (while recording)
```

The key design rule: all processing consumes `SensorSource`
(`lib/dataset/sensor_event.dart`). `LiveSensorSource` (the phone) and
`ReplaySource` (a saved session) both implement it, so a recording can be
re-processed with exactly the code that ran live.

## 2. Frames and conventions

| Symbol | Frame | Definition |
| --- | --- | --- |
| W | room / map | z up; defined in later packages (mapping or anchoring mode) |
| V | VIO world | ARKit world rotated to z up: `arPoseToVio` in `lib/slam/ar_pose.dart` |
| C_k | camera | OpenCV convention: x right, y down, z forward, in the landscape sensor image |
| M_i | marker i | centre origin, x right, y up, z out of the marker |
| B | device / IMU | x right, y toward the top of the screen (portrait), z out of the screen |

- `Pose3` (`lib/core/se3.dart`) is `aTb`: maps points from b to a; `aTb * bTc = aTc`.
- Perturbations are on the right: `retract` applies `R ← R·Exp(δθ)`,
  `t ← t + R·δt` with `δ = [δt; δθ]`; `poseError(E) = [t; Log(R)]`. The test
  `test/se3_camera_test.dart` checks that these two are exact inverses.
- ARKit's raw pose (`camera.transform`) is world +y up, camera looking along
  −z. `arPoseToVio` turns it into `vTc`.

## 3. Acquisition (WP1)

### 3.1 Camera frames

- ARKit captures `capturedImage` (typically 1920×1440, landscape, 60 Hz) with
  `isAutoFocusEnabled = false`, so the lens and therefore the intrinsics stay
  fixed for the whole run (Apple: autofocus is on by default since iOS 11.3).
- At 10 Hz, `ArPlugin.swift` takes the luma plane and box-filters it by the
  integer factor `s = max(1, width / 960)` (2 for 1920 px), giving 960×720.
- Intrinsics: `camera.intrinsics` refers to `camera.imageResolution`, the size
  of `capturedImage` (Apple documentation). An s×s average maps output pixel
  centre u to input coordinate `s·u + (s−1)/2`, so
  `f' = f/s`, `c' = (c + 0.5)/s − 0.5`. `CameraIntrinsics.scaledTo` in Dart is
  the same formula; a test checks they agree for 1920 → 960.
- Frame selection: frames arrive every 1/60 s, and 0.1 s is exactly six
  periods, so a strict `≥ 0.1 s` test skips to the seventh frame about half the
  time because of timestamp jitter (≈ 8.6 Hz). The selector allows 8 ms of
  slack (half a frame period) to get a steady 10 Hz.
- The 960×720 gray image is JPEG-encoded natively (ImageIO, quality 0.95) on
  the frame queue, so the Dart side only writes bytes. Processing decodes this
  JPEG for both live and replayed frames, so both see identical pixels.

### 3.2 IMU

`sensors_plus` 7.1.1 was checked (iOS source, `FPPStreamHandlerPlus.swift`):
its `gameInterval` is 20 ms (50 Hz), and it converts `CMLogItem.timestamp` with
its own boot-time offset, computed at a different moment from the camera's. So
on iOS the app reads CoreMotion itself (`ImuStream` in `ArPlugin.swift`):

- `CMMotionManager` accelerometer and gyroscope at 100 Hz, on one serial queue,
  sent to Dart in batches about every 50 ms (`vslam/imu`).
- Accelerometer: `−acceleration × 9.80665` m/s². CoreMotion reports the
  gravity direction (−1 g on the axis pointing up at rest); the negation gives
  the specific force, +9.81 m/s² on the up axis at rest. This matches the
  convention the Dart code already used with `sensors_plus` (which also
  negates). To be confirmed on the device in WP2.
- Gyroscope: rad/s, right-handed about the device axes, unchanged.
- Android keeps using `sensors_plus` (Android code is out of scope).

### 3.3 Timestamps and synchronisation

`ARFrame.timestamp` and `CMLogItem.timestamp` are both seconds since boot on
the same host clock. `ArPlugin` computes one offset
`bootEpoch = Date() − systemUptime` at start-up, and every frame, pose and IMU
timestamp goes through the same `epochMicros(t) = (bootEpoch + t)·10⁶`. Stored
times are integer microseconds. Because one offset is used, camera–IMU
comparisons do not depend on the wall clock. What remains is the physical
delay between the image exposure and the IMU sample stamps; WP2 estimates it
from a rotation recording.

## 4. Dataset recorder and replay (WP1)

### 4.1 Session layout

```
Documents/sessions/<yyyy-mm-dd>_<hh-mm-ss>_<label>/
  meta.json
  frames.csv        index,t_us,file,fx,fy,cx,cy,tracking_state,tx,ty,tz,qx,qy,qz,qw
  frames/000001.jpg
  arkit_poses.csv   t_us,tx,ty,tz,qx,qy,qz,qw,tracking_state
  imu.csv           t_us,sensor,x,y,z          (sensor = acc | gyro)
  results/          filled by processing in later packages
Documents/recording_settings.json   last used recording settings
```

`frames.csv` has seven more columns than the spec listed: each image's own
ARKit pose. The pose stream in `arkit_poses.csv` is throttled to about 30 Hz
and comes from other ARKit frames, so it cannot give the exact pose of a
saved image. Poses are stored in raw ARKit convention; `vioPose` converts.

`meta.json` holds the schema version, label, experiment type, marker
dictionary and side length, notes, device model, OS, app version and build,
requested and actual image sizes, JPEG quality, start and end time, counts
(frames, frames without image, frame gaps, poses, tracked poses, accelerometer
and gyroscope samples) and a `conventions` block describing units and frames,
so a session explains itself without the code.

### 4.2 Writing without blocking the UI

`SessionWriter` writes CSV rows to buffered `IOSink`s and each JPEG with an
asynchronous file write. Dart runs asynchronous file I/O on its I/O thread
pool, not on the UI isolate, and JPEG encoding already happened natively.
`close()` waits for every pending image write before it writes the final
`meta.json`. A frame gap is counted whenever consecutive frames are more than
1.5× the requested interval apart; it is the number to check for dropped
frames.

### 4.3 Replay

`SessionReader` loads a folder (images stay on disk until `loadJpeg` is
called). `ReplaySource` emits events in timestamp order, breaking ties by
sensor (IMU, pose, frame) and then by file order, so every replay of a session
is identical. Live sources deliver events in arrival order; estimation code
must use `tUs`, not arrival order.

### 4.4 Interface

- Main screen: settings row (label, experiment type, marker side length,
  notes; saved between runs), Start / Stop, live status tiles, diagnostics.
  After Stop a notice shows what was saved, with a button to open it.
- Sessions screen (folder icon): saved sessions, newest first.
- Session screen: first image, metadata, measured frame / pose / IMU rates and
  counts, rename, export (zip through the share sheet), delete (with
  confirmation).
- Sessions are also visible in Finder and the Files app (`UIFileSharingEnabled`).

## 5. Calibration and camera–IMU alignment (WP2)

### 5.1 Camera calibration

Board: a chessboard of 10 × 7 squares (9 × 6 inner corners), 25 mm nominal,
generated as a true-scale SVG by `dart run tools/make_board.dart` (also in
`docs/calibration_board.svg`) with a 100 mm ruler line to confirm the print
scale. A chessboard rather than ChArUco: only `calib3d` is needed, and every
view must show the whole board anyway for good coverage.

Procedure (calibration icon on the home screen, a sheet over the live view):
enter the measured square side, then capture 15–20 views at varied distance,
tilt (up to ≈ 45°) and position, so corners reach the image edges (the
coverage figure counts the cells of a 6 × 4 grid that hold a corner). For each
capture the next ARKit frame's JPEG (960 × 720 gray, the same pixels the
pipeline uses) goes through `findChessboardCorners` (adaptive threshold,
normalise, fast check) and `cornerSubPix` (11 × 11 window), in a background
isolate. Calibrate runs `calibrateCamera` with
`CALIB_USE_INTRINSIC_GUESS | CALIB_ZERO_TANGENT_DIST | CALIB_FIX_K3`, starting
from ARKit's mean intrinsics, so the result is exactly the model of
`CameraIntrinsics`: fx, fy, cx, cy, k1, k2. The RMS reprojection error, the
board, the view count and the mean ARKit intrinsics of the same images are
stored in `Documents/calibration.json`, and the sheet shows OpenCV vs ARKit
side by side. Code: `lib/calibration/camera_calibration.dart`,
`lib/widgets/calibration_panel.dart`.

### 5.2 Time offset and rotation

From a recording with rotation about each phone axis (session screen →
IMU alignment; result in `results/imu_alignment.json`):

- Camera angular velocity from consecutive tracked ARKit poses (~30 Hz),
  converted to the OpenCV camera frame:
  `ω_c(t_k+½) = Log(R_k⁻¹ R_{k+1}) / Δt`, stamped at the midpoint.
- Model: `ω_c(t) = R_ci · ω_i(t + δ)`, gyroscope linearly interpolated.
  δ > 0 means a motion shows up later in the IMU stamps than in the camera
  stamps.
- δ, coarse: minimise `mean (|ω_c(t)| − |ω_i(t + δ)|)²` on a 1 ms grid over
  ±200 ms (magnitudes do not depend on R), parabolic refinement.
- R_ci: least squares `min Σ |ω_c − R ω_i|²`, closed form by Horn's quaternion
  method (largest eigenvector of a 4 × 4 symmetric matrix, Jacobi).
- δ, fine: with R known, minimise the full vector residual on a 0.1 ms grid
  over ±5 ms, refit R; three rounds. (Magnitudes alone are biased by gyro
  noise; on synthetic data the vector refinement is what brings the error
  under 1.5 ms.)
- Only camera rates above 0.1 rad/s are used; too little motion is an error,
  not a guess. Reported: δ, R_ci, RMS residual at δ and at 0, pairs, mean
  rate.

Apple's documented axes: Core Motion device frame x right, y toward the top
of the screen (portrait), z out of the screen; ARKit camera space x along the
long axis from the front camera toward the Home-button end, y up in
landscape, z out of the screen side. With the OpenCV camera (y and z of ARKit
negated) this gives `x_cv = −y_dev, y_cv = −x_dev, z_cv = −z_dev`
(`appleCameraFromDevice`). The estimate is reported together with this
matrix and the angle between them. Sources: the `ARCamera.transform` page
states the camera axes in text (x from the front camera toward the Home
button, z out of the screen side), checked 2026-11-16. The `CMMotionManager`
page gives the device axes only as an illustration, which could not be
loaded outside Apple's viewer; device +z out of the screen is confirmed by
WP1 check 1.3 (+g on +z, screen up), and the rotation estimate checks the rest.

Tests: `test/imu_alignment_test.dart` recovers offsets of +23, −41 and
+7.5 ms (within 1.5 ms) and a rotation a few degrees from Apple's (within
0.5°) from synthetic 30 Hz orientations and noisy 100 Hz gyro;
`test/camera_calibration_test.dart` renders the board through a known
distorted camera, checks detected corners (< 0.5 px) and recovered
intrinsics.

## 6. Implemented, configured, reused

| Component | Status | Notes |
| --- | --- | --- |
| ARKit world tracking (VIO) | reused, configured | gravity alignment, autofocus off; used only as odometry |
| CoreMotion IMU at 100 Hz | configured | read directly in Swift; units and sign documented above |
| Frame reduction, intrinsics rescaling, JPEG | implemented (Swift) | box filter, ImageIO encoder |
| Timestamp unification | implemented | one boot-epoch offset for all streams |
| Dataset format, writer, reader, replay | implemented (Dart) | `lib/dataset/` |
| Sessions list, detail, export | implemented (Dart) | export uses `archive` (zip) and `share_plus` |
| File locations | reused | `path_provider` |
| Chessboard detection, calibration | reused (OpenCV via `dartcv4` 2.3.1), configured | flags in 5.1 |
| Board generator | implemented | `tools/make_board.dart` |
| Time offset and camera–IMU rotation | implemented (Dart) | `lib/calibration/imu_alignment.dart` |
