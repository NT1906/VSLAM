# Progress

Read with `docs/SPEC.md`. A new session continues from these two files.

Branch: `iphone-slam` (local, not pushed). Toolchain: Flutter 3.47.9 / Dart
3.13.7 (`~/development/flutter`), the version the repo was created with.

## Starting state (commit `df030f6`, 2026-11-11)

- `flutter analyze`: no issues. `flutter test`: 15 tests pass.
- `flutter build ios`: could not run at first, Xcode was not installed on
  this Mac (only the Command Line Tools). Xcode 27.2 (27B52) was then
  installed and selected; the first build check is under WP1 below.
- Spec vs code, found when reading the code:
  - `docs/SPEC.md` and `docs/IE415_handout.pdf` were not in the repo; added
    on this branch.
  - Camera and IMU did not share one clock offset: `ArPlugin.swift` and
    `sensors_plus` each computed `Date() − systemUptime` at different moments.
  - `sensors_plus` `gameInterval` gives 50 Hz, not the ~100 Hz the spec wants.
  - ARKit poses are throttled to 30 Hz in `ArPlugin.swift` (kept).
  - Frames were skipped while the previous one was converting, and the 0.1 s
    selection skipped to the 7th ARKit frame about half the time (≈ 8.6 Hz).

## Section 9 checks (by reading source / documentation)

| Item | Result |
| --- | --- |
| `sensors_plus` 7.1.1 timestamps | `CMLogItem.timestamp` + its own boot offset (not wall clock at receive) |
| `sensors_plus` rate | `gameInterval` = 20 ms → 50 Hz |
| Accelerometer convention | `sensors_plus` reports `−acceleration × 9.81`: +9.81 on the up axis at rest. Kept (with 9.80665) in our own CoreMotion reader. Device confirmation: check 1.3 |
| ARKit autofocus | `ARWorldTrackingConfiguration.isAutoFocusEnabled` (iOS 11.3+), default on; set to false |
| ARKit image / intrinsics | `camera.intrinsics` is for `camera.imageResolution`, the size of `capturedImage` (Apple docs). Actual size per device: check 1.1 |
| `dartcv4` 2.3.1 (for WP3) | modules `calib3d`, `objdetect`, `aruco` are opt-in under `hooks: user_defines: dartcv4: include_modules:`; ChArUco detector and board are bound; `SOLVEPNP_IPPE_SQUARE`, `solvePnP`, `calibrateCamera`, `findChessboardCorners` bound; iOS build uses CMake with the Xcode generator (CMake not installed yet) |
| Package versions | path_provider 2.1.6, share_plus 13.3.1, archive 4.6.0 all accept Dart 3.13 / Flutter 3.47 |

## Work packages

### WP1 dataset recorder and replay: code done and builds, device checks pending

Done:
- Swift: 960×720 frames (integer box filter, intrinsics rescaled), native
  grayscale JPEG (quality 0.95), autofocus off, 10 Hz frame selection fixed,
  CoreMotion accelerometer + gyroscope at 100 Hz on the same timestamp offset
  as the frames (`vslam/imu`), `deviceInfo` method.
- Dart: `lib/dataset/` (event types and `SensorSource`, writer, reader,
  replay, live source, recorder, store), recording settings, sessions list and
  detail screens (open, rename, export as zip through the share sheet,
  delete).
- Docs: `DESIGN.md` (acquisition and dataset sections), `DEVICE_CHECKS.md`
  (WP1), `EXPERIMENTS.md` (recording part).

Checks:
- `flutter analyze`: no issues.
- `flutter test`: 35 tests pass (15 existing + 20 new: SE(3) and camera
  model, dataset round trip of every field, replay order and counts, rates,
  frame-gap counting, recorder, live controller to disk, store
  list/rename/zip/delete/settings, two widget tests of record → list →
  detail → rename → delete).
- `flutter build ios --debug --no-codesign`: passes (Xcode 27.2, iOS SDK
  27.2, 2026-11-11), no compiler warnings or errors from our Swift code. The
  Swift changes (CoreMotion stream, JPEG encoder, autofocus setting) compile.
- Device checks (iPhone 15, `iPhone15,4`, iOS 26.2.1, release build, 2026-11-12):
  - First short session `2026-11-12_08-48-35_run` (9.50 s, hand-held), from
    the session screen: image 960×720 from 1920×1440, JPEG q 0.95;
    intrinsics fx 722.0 fy 722.0 cx 481.4 cy 361.6; 95 frames at 10.0 Hz,
    0 gaps, 0 without image; 286 ARKit poses, all tracked, 30.0 Hz;
    accelerometer and gyroscope 952 each at 100.5 Hz; frames span 9.40 s,
    IMU 9.47 s, IMU starts 0.02 s after the first frame. Live frame-rate tile
    showed 10.1 Hz.
  - 1.1: passes on these numbers (size, rate, device row).
  - 1.2: passes (label `wp1-static`, Calibration / alignment, 15.0 cm, kept
    after force-quit).
  - 1.3 (`2026-11-12_09-01-42_wp1-static`, phone flat, screen up, top end over
    the table edge): 60.37 s; 604 frames at 10.0 Hz, 0 gaps, 0 without image;
    1811 poses, all tracked, 30.0 Hz; accelerometer 6062 and gyroscope 6061
    at 100.5 Hz; frames span 60.32 s, IMU 60.33 s, IMU starts 0.03 s after the
    first frame. From the exported `imu.csv`: acc mean (0.285, 0.165, 9.851)
    m/s², std ≈ 0.007 on each axis, so +g on +z with the screen up: the
    specific-force sign is confirmed. The 0.33 m/s² in x/y is ≈ 1.9° of tilt
    or bias. Gyro means within 0.002 rad/s, std 0.0009, all samples within
    ±0.022 rad/s. Zip contents as expected (604 JPEGs, 960×720, empty
    `results/`). Passes.
  - Finding: the intrinsics are **not** constant within a session even with
    autofocus off and the phone still. In the static run fx = fy ranged
    721.334–721.439 (57 distinct values), cx 477.538–477.616, cy
    361.473–361.553, i.e. about 0.1 px. Between sessions fx was 722.0 (first
    run) and 721.4. Check 1.4's expectation "fx identical on every row" was
    wrong; it now asks for the range. Per-frame intrinsics are stored, so
    nothing is lost; whether to use ARKit's per-frame values or one
    calibrated set is decided in WP2.
  - 1.4 (`2026-11-12_09-05-51_wp1-walk`, hand-held walk with a near/far
    look): 60.44 s; 604 frames at 10.0 Hz, 0 gaps, 0 without image; 1812
    poses, all tracked, 30.0 Hz; accelerometer and gyroscope 6067 each at
    100.5 Hz; IMU starts 0.01 s before the first frame; ARKit path length
    5.73 m. First frame sharp. From the exported `frames.csv`: every frame
    tracked; fx = fy 721.270–721.635 (spread 0.37 px), cx spread 0.33 px,
    cy spread 0.41 px: no refocusing. Passes.
  - 1.5: export by AirDrop works for both sessions; unzipped walk folder has
    `meta.json`, `frames.csv`, `arkit_poses.csv`, `imu.csv`, 604 gray
    960×720 JPEGs in `frames/` and an empty `results/`. Finder, rename and
    delete: pending (user).
  - Fixed after this run: the session screen printed a negative "before"
    time when the IMU started after the first frame; it now says before or
    after.

Bugs the widget tests caught and fixed: rename dialog disposed its text
controller during the closing animation; the sessions list called `setState`
with a closure returning a Future.

## Decisions

- IMU on iOS read directly from CoreMotion in `ArPlugin.swift`
  (spec section 9 fallback): `sensors_plus` gives 50 Hz and a separate clock
  offset. Android still uses `sensors_plus`.
- Accelerometer stored as specific force (+g up at rest), the convention the
  existing Dart code and test fixtures already used.
- Images saved as 8-bit grayscale JPEG (the pipeline only uses luma), encoded
  natively so Dart only writes bytes.
- `frames.csv` gets each image's own ARKit pose (7 extra columns): the pose
  stream is throttled and comes from other frames.
- Replay order: timestamp, then IMU < pose < frame, then file order.
- Start still starts immediately (existing behaviour and test); settings are
  edited beforehand on the settings row and kept in
  `Documents/recording_settings.json`.
- Export zips with no recompression (`store`), since images are already JPEG,
  on a background isolate, into the temporary directory.

## Open questions

- Which iPhone model the team uses (ARKit video format, hence frame size).

## Next

WP2 (calibration, IMU timing and alignment) after the WP1 device checks are
confirmed.
