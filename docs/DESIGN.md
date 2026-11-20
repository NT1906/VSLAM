# VSLAM on iPhone: system design

This document describes the system as it is built so far. It grows with each
work package (see `docs/SPEC.md` section 5 and `docs/PROGRESS.md`). Sections
for parts that do not exist yet are not written in advance.

## 1. Architecture

One Flutter app on the iPhone does acquisition, storage, replay and (in later
packages) processing and display. Native Swift code (`ios/Runner/ArPlugin.swift`)
owns the ARKit session and CoreMotion; everything after acquisition is Dart.

Processing a saved session so far: Detect markers (section 6) → Build
map (section 7: keyframes, two baselines, first marker map).

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
| W | room / map | z up. Mapping mode (WP4): the first keyframe's gravity-aligned frame, origin at its camera centre, x along the horizontal viewing direction (`worldFromVio`). Anchoring mode: WP6 |
| V | VIO world | ARKit world rotated to z up: `arPoseToVio` in `lib/slam/ar_pose.dart` |
| C_k | camera | OpenCV convention: x right, y down, z forward, in the landscape sensor image |
| M_i | marker i | centre origin, x right, y up, z out of the marker |
| B | device / IMU | x right, y toward the top of the screen (portrait), z out of the screen; `R_ci = appleCameraFromDevice` maps B into C |

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

## 6. ArUco detection and marker pose (WP3)

Code: `lib/vision/aruco_detector.dart`, `marker_observation.dart`,
`session_detection.dart`, `marker_sheet.dart`; thresholds in
`lib/estimation/config.dart`; overlay in `lib/widgets/marker_overlay.dart`.

### 6.1 Detection

Each frame's grayscale JPEG (the bytes the recorder saves, so live and
replayed runs see the same pixels) is decoded and passed to OpenCV's
`aruco::ArucoDetector` with dictionary `DICT_4X4_50` and
`cornerRefinementMethod = CORNER_REFINE_SUBPIX` (other detector parameters
are OpenCV 4.13's defaults). Corners come back in the printed marker's order
top-left, top-right, bottom-right, bottom-left, in distorted pixel
coordinates of the 960 × 720 sensor-orientation image. Each detection runs in
a background isolate.

### 6.2 Pose and the planar ambiguity

The marker frame M has its origin at the centre of the black square, x
right, y up, z out of the printed face (spec 4.3). For side s the object
points are `(−s/2, s/2, 0), (s/2, s/2, 0), (s/2, −s/2, 0), (−s/2, −s/2, 0)`,
which is also the order `SOLVEPNP_IPPE_SQUARE` requires.
`cv::solvePnPGeneric(..., SOLVEPNP_IPPE_SQUARE)` returns both solutions of a
planar square: the true pose and its "flip" about the line of sight. The
rotation vector r is converted with `so3Exp(r)` (Rodrigues' formula), giving
`cTm = (R, t)`. For each solution we compute the corner reprojection RMS with
our own camera model, `√(Σ_j |π(cTm·p_j) − u_j|² / 4)`, where π includes k1,
k2; the solutions are sorted by it.

Intrinsics: the WP2 calibration (`Documents/calibration.json`, rescaled to the
frame size if it differs) when it exists, ARKit's per-frame intrinsics
otherwise (`chooseIntrinsics`). The calibration adds the radial distortion
ARKit does not report.

The printed side length s is the only scale in this problem: distance is
proportional to s. The app takes s from the recording settings ("Marker side
(cm)"), and the marker sheet asks for the *measured* printed side, so printer
scaling does not enter.

### 6.3 Rejection rules (spec 4.4)

Applied in this order; the first failing rule is recorded as the status:

| Status | Rule | Default |
| --- | --- | --- |
| `small` | shortest image edge < `minMarkerSidePx` | 20 px |
| `no_pose` | PnP returned no solution | |
| `reproj` | best RMS > `maxReprojectionRmsPx` | 2 px |
| `angle` | angle between marker +z and the direction to the camera > `maxViewAngleDeg` | 65° |
| `ambiguous` | second RMS < `ippeAmbiguityRatio × max(best RMS, cornerSigmaPx)` | 2 × max(best, 0.5 px) |

The ambiguity rule says: the flipped pose would also fit the corners to
within about twice the corner noise, so noise could swap the two. The floor
at the corner noise keeps a nearly perfect best fit from making every
alternative look much worse. Head-on views of small or distant markers are
ambiguous by nature; in the synthetic tests a 15 cm marker at 1.5 m seen 13°
off its normal gave RMS 0.04 px vs 0.86 px (ambiguous), at 30° 0.01 vs
1.8 px (accepted). Distance is nearly the same for both solutions, so the
live view still shows it for ambiguous detections. Rejected detections are
kept in the output with their status; the rules decide which detections may
*initialize* a marker (WP4/WP5). The estimator itself uses corner
reprojection factors, not the PnP pose (spec 4.4).

### 6.4 Output

Session screen → **Detect markers** runs the detector on every saved frame
and writes `results/detections.csv`, one row per detected marker per frame:
`t_us, id, status, u0, v0 … u3, v3, side_px, view_deg, rms_px, tx, ty, tz,
qx, qy, qz, qw, alt_rms_px, alt_tx … alt_qw` (poses are `cTm`, metres and a
unit quaternion; the `alt_` columns are the second IPPE solution). `t_us`
joins the row to `frames.csv`. `results/detections.json` holds the totals:
frames, frames with an accepted marker, accepted per ID, rejected per
status, mean and maximum detection time per frame, marker side and the
intrinsics used.

The live overlay on the camera view runs the same detector on the live
frames (skipping frames while one is being processed) and draws each
marker's outline (green accepted, orange rejected, red dot on the top-left
corner), its ID and distance, and the rejection reason. To place image
pixels on the portrait view it uses ARKit's
`ARFrame.displayTransform(for: .portrait, viewportSize:)`, sent with each
frame; the same mapping is implemented in Dart as a fallback (rotate 90°
clockwise, aspect-fill).

### 6.5 Marker sheet

`dart run tools/make_markers.dart` writes
`docs/markers_4x4_50_ids0-9.pdf`: one A4 page per ID 0–9, a 150 mm black
square (cells from `cv::aruco::generateImageMarker`, so the print is exactly
what the detector decodes), an ID label, a TOP mark and a 100 mm ruler line.
The PDF is written by hand (vector rectangles, Helvetica), no dependency.

## 7. Keyframes, baselines and the first map (WP4)

Session screen → **Build map** runs this whole section on a saved session
(`lib/estimation/session_baselines.dart`); it runs Detect markers first when
`results/detections.csv` does not exist yet. **Save map** then stores the
map as `Documents/maps/<room>.json`.

### 7.1 Keyframes (`lib/estimation/keyframe_selector.dart`)

`KeyframeSelector` decides frame by frame (so the live mode of WP5 can use
it unchanged). Only frames with an ARKit pose can be keyframes. The first
tracked frame is keyframe 0; after that a frame becomes a keyframe when,
relative to the **previous keyframe** (not accumulated along the path):

- the camera moved ≥ 0.25 m (`keyframeTranslationM`), or
- the relative rotation angle is ≥ 15° (`keyframeRotationDeg`), or
- the frame has a usable marker detection and ≥ 0.3 s have passed
  (`keyframeMarkerGapS`).

"Usable" means status `ok` or `ambiguous` (`keyframeMarkerStatuses`): both
have good corners, and the reprojection factor of WP5 needs corners, not a
unique PnP pose. `small`, `no_pose`, `reproj` and `angle` do not count.
Each keyframe records its reasons, the marker IDs it saw, and whether an
untracked frame lay between it and the previous keyframe
(`trackingGapBefore`; WP5 inflates that edge's noise).

Between consecutive keyframes the odometry is
`Z_k = (vTc_k)⁻¹ · vTc_k+1` (`OdometryEdge`), with step length and angle.

### 7.2 Baseline 1: ARKit only

`W` is fixed from keyframe 0: `wTv = (yawTranslation(ψ_0, t_0))⁻¹`, where
`ψ_0` is the heading of the camera's viewing axis in V (`worldFromVio`).
Then

    X_0 = wTv · vTc_0,     X_k+1 = X_k · Z_k

which equals `wTv · vTc_k`: the ARKit trajectory re-expressed in W. It is
built by chaining because that is exactly the odometry-only solution and
initial guess of the WP5 pose graph. Written to `results/traj_arkit.csv`:
`kf, frame_index, t_us, tx, ty, tz, qx, qy, qz, qw, reasons, marker_ids,
tracking_gap` (pose `wTc`, reasons and IDs joined with `|`).

### 7.3 Baseline 2: IMU dead reckoning (`lib/estimation/imu_propagator.dart`)

State `[p, v, q, b_a, b_g]`: device position and velocity in W, orientation
`q_wb`, accelerometer and gyroscope biases (device axes). Measurement model
(specific force, +g up at rest, as recorded):

    ω_m = ω + b_g + n_g
    f_m = R_wbᵀ (a + g e_z) + b_a + n_a      →   a = R_wb (f_m − b_a) − g e_z

Propagation step from t to t + Δt (boundaries at the gyroscope stamps, the
accelerometer interpolated onto them; `ω̄`, `f̄` are the means of the two
end samples):

    q ← q ⊗ Exp((ω̄ − b_g) Δt)
    a  = R_wb(t) Exp((ω̄ − b_g) Δt/2) (f̄ − b_a) − g e_z
    p ← p + v Δt + ½ a Δt²,   v ← v + a Δt

This is exact for constant angular rate and for constant world
acceleration (both tested against the closed form). Biases are constant
during the run.

Initialization: the first window of 2 s (`imuRestSeconds`), starting at or
after keyframe 0 (scanned in 0.1 s steps for up to 15 s), in which the
largest per-axis standard deviation is below 0.03 rad/s (gyroscope) and
0.15 m/s² (accelerometer). There

    b_g = mean(ω_m),     b_a = mean(f_m) − R_wbᵀ g e_z

with `R_wb = R_wc · R_ci` from ARKit's pose in the middle of the window and
`R_ci = appleCameraFromDevice` (WP2 measured 0.10° from it; time offset 0).
Any small tilt error ends up in `b_a`, which is what makes the propagation
start without a spurious acceleration. The propagation starts at the end of
the window from ARKit's pose with v = 0. If no window is still, the biases
are set to 0, the start is keyframe 0, and the screen says "NOT still".
The IMU is placed at the camera centre (lever arm of a few cm, negligible
against the drift). `results/traj_imu.csv`: `kf, t_us, pose (wTc), vx, vy,
vz, dist_to_arkit_m` for each keyframe after the start.

As the spec expects, this diverges. In the simulator with device-like noise
(σ 0.02 m/s², 0.002 rad/s per sample, 2 s rest) the distance to the truth
was 3 cm after 5 s, 21 cm after 10.6 s, 89 cm after 20 s and 5.7 m after
45 s (synthetic numbers, `test/baselines_sim_test.dart`).

### 7.4 First marker map (`lib/mapping/marker_map.dart`)

Each marker is placed once, from its first **accepted** detection in a
keyframe, with that keyframe's baseline-1 pose (spec 4.4, marker
initialization):

    wTm = wTc_k · cTm(best IPPE solution)

So the map carries all the drift ARKit had at the moment of first sight;
WP5 corrects it. Markers detected but never accepted in a keyframe are
listed as "not placed". `results/map.json` and `maps/<room>.json` use one
format (`vslam-marker-map`, version 1): method, frame description,
dictionary, marker side, source session, creation time, and per marker the
pose (`t`, `q` as x, y, z, w), first keyframe and time, distance and viewing
angle of the placing detection, accepted and keyframe observation counts.
`results/baselines.json` summarizes the run (keyframe counts and reasons,
tracking gaps, path length, rest window and biases, IMU-vs-ARKit distance
at 1, 2, 5, 10, 20, 30, 60 s and at the end, placed and unplaced IDs).

### 7.5 Simulator (`test/support/simulator.dart`)

`RoomSimulation` is the spec section 6 simulator, built now so WP5 reuses
it: a 6 × 5 m room with eight 15 cm markers on the walls; a walk around an
ellipse (2.0 × 1.4 m radii, 0.5 m/s, smooth start and stop, two laps,
≈ 21.5 m) looking outward with a slow side-to-side sweep; ARKit poses at
30 Hz and frames at 10 Hz with optional drift (1 % scale, 0.24° of heading
per metre, step noise), an initializing period and a tracking-loss interval;
IMU at 100 Hz from the true motion (finite differences, `R_ci`, biases,
noise); corners projected through the calibrated camera model with 0.5 px
noise, and the app's own IPPE solver and rejection rules. Fixed seeds.

## 8. Implemented, configured, reused

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
| ArUco detection, sub-pixel corners, IPPE PnP | reused (OpenCV via `dartcv4`), configured | `DICT_4X4_50`, `CORNER_REFINE_SUBPIX`, `SOLVEPNP_IPPE_SQUARE` |
| Rejection rules, reprojection RMS, detections file | implemented (Dart) | `lib/vision/`, thresholds in `lib/estimation/config.dart` |
| Live marker overlay | implemented (Dart), ARKit `displayTransform` reused | `lib/widgets/marker_overlay.dart` |
| Marker sheet generator | implemented | `tools/make_markers.dart`, hand-written PDF |
| Keyframe selection, odometry edges | implemented (Dart) | `lib/estimation/keyframe_selector.dart`, thresholds in `SlamConfig` |
| Baseline 1 (ARKit chained odometry) | implemented (Dart) | `lib/estimation/baselines.dart` |
| Baseline 2 (IMU dead reckoning, rest biases) | implemented (Dart) | `lib/estimation/imu_propagator.dart` |
| First marker map, map files | implemented (Dart) | `lib/mapping/marker_map.dart` |
| Room simulator for tests | implemented (Dart, tests) | `test/support/simulator.dart`, uses OpenCV's IPPE through the app |
