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

### WP1 dataset recorder and replay: done, device checks passed

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
    960×720 JPEGs in `frames/` and an empty `results/`. Finder: phone →
    Files shows VSLAM with the `sessions` folder. Rename: `wp1-walk` →
    `2026-11-12_09-05-51_wp1-walk-renamed` (date-time prefix kept). Delete:
    the accidental 5-frame session `2026-11-12_09-00-47_wp1-static` asked
    for confirmation and left the list. (Its absence in the Files app was
    not checked separately.) Passes.
  - WP1 device checks 1.1–1.5: **all pass** (2026-11-12). Kept zips:
    `2026-11-12_09-01-42_wp1-static.zip`, `2026-11-12_09-05-51_wp1-walk.zip`
    (exported before the rename; outside git).
  - Fixed after this run: the session screen printed a negative "before"
    time when the IMU started after the first frame; it now says before or
    after.

Bugs the widget tests caught and fixed: rename dialog disposed its text
controller during the closing animation; the sessions list called `setState`
with a closure returning a Future.

### WP2 calibration, IMU timing and alignment: done, device checks passed

Done:
- `dartcv4` 2.3.1 added (CMake 4 via Homebrew). Modules in `pubspec.yaml`:
  imgproc, imgcodecs, calib3d, features2d, flann, objdetect, aruco.
  `include_modules` **replaces** the default list (`hook_helpers/
  user_defines.dart`), so imgproc and imgcodecs must be listed; without them
  `imdecode`/`imencode` are missing at run time (the first test run showed
  this; the iOS build still passed). Signatures used (`findChessboardCorners`,
  `cornerSubPix`, `calibrateCamera`, `imdecode`, `Mat.fromList` needs
  doubles) were read in the package source.
- Board: chessboard 9 × 6 inner corners, 25 mm, `tools/make_board.dart` →
  `docs/calibration_board.svg` (true scale, 100 mm ruler). Pulled forward from
  WP3; WP3 adds the markers.
- Calibration sheet (grid icon on the home screen, `lib/widgets/
  calibration_panel.dart`): 15–20 views, coverage, OpenCV with
  `USE_INTRINSIC_GUESS | ZERO_TANGENT_DIST | FIX_K3`, RMS, OpenCV vs ARKit
  table, saved to `Documents/calibration.json`.
- Time offset and camera–IMU rotation (`lib/calibration/imu_alignment.dart`,
  session screen → IMU alignment → `results/imu_alignment.json`), compared
  with Apple's axes. Method in `DESIGN.md` 5.2.
- Docs: `DESIGN.md` section 5 and table, README status, `DEVICE_CHECKS.md` WP2
  (step by step).

Checks (2026-11-16):
- `flutter analyze`: no issues. `flutter test`: 47 pass (35 before + 12 new),
  run twice. `flutter build ios --debug --no-codesign`: passes.
- OpenCV loads under `flutter test` on macOS, so detection and calibration
  are tested on the Mac, not only on the device.
- Synthetic calibration (board rendered through fx 721.4, cx 477.6, cy 361.5,
  k1 0.08, k2 −0.15; 10 views, JPEG q 95): every corner within 0.5 px;
  result RMS 0.101 px, fx = fy 722.7 (+0.18 %), cx 477.6, cy 361.6, k1 0.079,
  k2 −0.147.
- Synthetic alignment: offsets +23, −41, +7.5, +15 ms recovered within 1.5 ms,
  rotation within 0.5°. The first version used only rate magnitudes and was
  off by 1.7–2.5 ms; the test motion also had abrupt axis switches. Fixed by
  a vector-residual refinement and a smooth test motion (thresholds unchanged).
- Test fixes that were test errors, not code changes: one rendered view put
  part of the board outside the image (u −28…744, v 133…827), moved from 0.30
  to 0.38 m.
- Seen once, not reproduced in two further full runs: `dataset_test` "a
  second session started in the same second" failed in tearDown with
  "Directory not empty" while OpenCV was compiling in parallel. WP1 test,
  unchanged; watch for it.
- Apple axes: `ARCamera.transform` text read 2026-11-16 (x from the front
  camera toward the Home button, z out of the screen side). Core Motion's
  axes are only an image on the `CMMotionManager` page that would not load;
  device +z out of the screen is confirmed by check 1.3.

Device checks (iPhone 15, release build, 2026-11-16):
- 2.0: `flutter run --release` installs; grid icon present, "SLAM tracking".
  First attempt failed with "No supported devices" (phone not connected /
  unavailable in `devicectl`); fine after reconnecting.
- Found on the device and fixed: the number pad covered the whole sheet
  (no Done key, field hidden), the size went back to 25.0 mm after closing
  the sheet (3 views captured at 25.0 were undone), a tap on the camera view
  closed the sheet. Commits `58875e7`, `71f1331`, `a0fa3a6`.
- 2.1: printed with the printer scaling the page: 10 squares = 206 mm, square
  20.6 mm (≈ 82 % of the nominal 25 mm). Used as printed; the square size
  does not enter fx, fy, cx, cy, k1, k2. Ruler line not reported.
- 2.2: 18 views (3 head-on, 4 tilted, 4 corners, 4 edges, 3 rolled), all 54
  corners each, coverage 92 %, square 20.6 mm. Close views were blurred
  (fixed focus since WP1), so no view closer than ≈ 35 cm.
  Result: **RMS 0.467 px**; OpenCV fx 716.5, fy 714.8, cx 473.0, cy 357.4,
  k1 0.0715, k2 −0.1411; ARKit mean fx = fy 721.3, cx 478.5, cy 360.7;
  differences −4.8, −6.5, −5.4, −3.2 px (fx −0.67 %, fy −0.90 %).
  Passes (RMS < 0.5, focal within 1–2 %, principal point within a few px);
  RMS is near the limit, a stiffer board would likely lower it.
- 2.3 first run (`2026-11-16_14-26-21_wp2-rotation`): 27.46 s; 275 frames at
  10.0 Hz, 0 gaps; 822 poses, all tracked, 30.0 Hz; acc and gyro 2756 each at
  100.5 Hz. IMU alignment: R_ci rows (−0.001 −1.000 0.001 / −1.000 0.001
  0.001 / −0.001 −0.001 −1.000), **0.10° from Apple's axes**; offset 0.3 ms;
  residual 0.007 rad/s RMS, **also 0.007 at zero offset**; 498 pairs; mean
  rate 0.20 rad/s.
  Reading: the rotation confirms `appleCameraFromDevice` on the device
  (including the Core Motion axes that could not be read from Apple's
  illustration). The offset is not resolved by this run: the motion was
  slow (0.20 rad/s mean, expected > 1 rad/s), and the residual does not
  change with the offset; it says only that there is no large offset.
  Caveat for the report: ARKit's orientation is itself fused with this
  gyroscope through Apple's factory camera–IMU calibration, so the
  agreement checks our conventions and the shared clock rather than giving
  an independent extrinsic calibration.
- 2.3 second run: skipped (user's time). Not needed downstream: the gyro
  check compares rotations between keyframes ≥ 0.25 s apart, where an offset
  of a few ms is negligible. The estimator uses offset 0 with R_ci =
  `appleCameraFromDevice` (or the measured R_ci, 0.10° apart).
- WP2 device checks: **done** (2026-11-16): 2.0, 2.1, 2.2 pass; 2.3 passes for
  the rotation, the offset is bounded only ("no large offset").

### WP3 ArUco detection and pose: done, device check passed

Done:
- `lib/vision/aruco_detector.dart`: OpenCV `ArucoDetector`, `DICT_4X4_50`,
  `CORNER_REFINE_SUBPIX`; `solvePnPGeneric` with `SOLVEPNP_IPPE_SQUARE`
  (both solutions); runs in a background isolate. Behind a `MarkerDetector`
  interface so UI tests can use a fake.
- `lib/vision/marker_observation.dart`: object points (spec 4.3), reprojection
  RMS with our camera model, the four rejection rules of spec 4.4 plus "no
  pose", `detections.csv` row format and parser.
- `lib/estimation/config.dart`: `SlamConfig` with the detection thresholds
  (20 px, 65°, 2 px, ambiguity 2 × max(best, 0.5 px), corner σ 0.5 px).
  Later packages add their parameters here.
- `lib/vision/session_detection.dart`: intrinsics choice, detection over every
  frame of a saved session → `results/detections.csv` + `detections.json`;
  session screen → **Detect markers**.
- `lib/widgets/marker_overlay.dart`: live outlines, IDs, distances and
  rejection reasons on the camera view; status line with marker side,
  intrinsics source, count and detection time. Pixel → view mapping from
  ARKit's `displayTransform(for: .portrait, viewportSize:)`, now sent by
  `ArPlugin.swift` with each frame (`disp`), with a Dart fallback.
- `lib/vision/marker_sheet.dart`, `tools/make_markers.dart` →
  `docs/markers_4x4_50_ids0-9.pdf` (A4, one ID per page, 150 mm, TOP mark,
  100 mm ruler, instructions to measure the printed side and enter it).
- Docs: `DESIGN.md` section 6 and table (now section 7), README status,
  `DEVICE_CHECKS.md` WP3.

API checks (read in the source before use): dartcv4 2.3.1 `ArucoDetector.create`,
`detectMarkers` → `(VecVecPoint2f, VecI32, VecVecPoint2f)`,
`ArucoDetectorParameters.cornerRefinementMethod` setter (the enum is not
bound: SUBPIX = 1 read in OpenCV 4.13's `aruco_detector.hpp` in the build
tree), `arucoGenerateImageMarker`, `solvePnPGeneric` (C wrapper passes
`reprojectionError` through; we compute RMS ourselves anyway),
`Mat.fromList` with `CV_64FC3` / `CV_64FC2`. OpenCV's own
`solvePnPGeneric` docs give the IPPE_SQUARE point order, which matches spec
4.3.

Checks (2026-11-16):
- `flutter analyze`: no issues.
- `flutter test`: 72 pass (47 before + 25 new).
- `flutter build ios --debug --no-codesign`: passes (Swift `displayTransform`
  change compiles). Two `flutter build ios --release --no-codesign` in a row
  also pass: the link hook wrote the keep-list (137 symbols) and the new
  build phase removed it, so the second build linked normally.
- Detector on rendered markers (calibrated K: fx 716.5, k1 0.0715, k2 −0.1411;
  4 × 4 supersampled ray casting, JPEG q 95): every corner within 0.5 px,
  distance within 1 %, rotation within 2° at 0.4–1.5 m, turned 30–45°, rolled
  up to 170°, near the image corner; also with ARKit K; two markers in one
  image; head-on at 2.5 m → `ambiguous`; 18 px at 6 m → `small`; 72° →
  `angle`. Thresholds were set before the first run and not changed.
- One test failed on its first run and was a test error: the "two markers"
  scene in the session test put marker 8 off-centre (≈ 17°) and turned it
  −30° toward the camera, so it was seen at 13° and correctly called
  ambiguous (RMS 0.04 vs 0.86 px). The scene was changed to turn it away
  (seen at ≈ 47°).
- Rendered-data observation worth knowing for the experiments: head-on views
  are ambiguous already at 1.5 m (13°: alternative RMS 0.86 px), while 30°
  gives a clear winner (1.8 px). Markers should be seen at an angle at least
  sometimes to initialize them; distance is valid either way.

Found and worked around: dartcv4 2.3.1 link-hook bug.
- On AOT builds (`flutter run --release`, `flutter build ios --release`) its
  link hook writes `.dart_tool/hooks_runner/shared/dartcv4/dartcv_keep.txt`,
  a list of symbols the next native build must export. With `aruco` enabled
  it adds the finalizer symbols of *every* module in `contrib.g.dart`
  (freetype, quality, img_hash, ximgproc, xobjdetect), which are not built.
  Every later native build then fails to link ("Undefined symbols
  _cv_freetype_FreeType2_close …"). It hit the first macOS test build of
  WP3 (the file was left by a WP2 release build at 13:35).
- Fix: an Xcode build phase "Clear dartcv4 keep-list" (after Flutter's
  build script) deletes the file, and it was deleted once by hand. Without
  it the library exports everything, which was the behaviour before any
  release build (`treeshake` is off).
- If `flutter test` ever fails with those undefined symbols again, delete
  that file and rerun.
- Also seen: running `dart run tools/make_markers.dart` and `flutter test` at
  the same time makes both fail (they share the OpenCV build folder). Run
  them one after the other.

Device check (iPhone 15, release build, 2026-11-16), user's screenshots:
- 3.0: installs; status line "ArUco 4X4_50 · side 15.0 cm · calibrated K",
  so `Documents/calibration.json` is found and used. Detection 8–11 ms per
  frame on the phone.
- 3.1: page 1 (ID 0) printed at 100 % this time (not 82 %): one cell 25 mm,
  black square 150 mm (6 cells). Ruler line not reported. Side left at the
  default 15.0 cm. Taped on a white sheet on the wall.
- Live overlay: outline on the black square's edges, red dot on the top-left
  corner (next to "ID 0"), ID 0 read correctly; green (accepted) at 1 m,
  orange "ambiguous" at 2 m roughly head-on, as expected (DESIGN 6.3).
- 3.2, distance measured with the tape from the marker centre to the lens:
  **1.006 m at 1.00 m (+0.6 %)**, **2.058 m at 2.00 m (+2.9 %)**. Both within
  3 %; the 2 m result is near the limit (hand-held, tape-to-lens placement
  of about ±1–2 cm is ±0.5–1 % there).
- Discarded attempts: 0.538 m with the page hand-held and curling (distance
  not measured), and 0.928 m with the phone below and to the side of the
  marker over the bed (how 1 m was measured is unknown; it was not the
  straight line to the marker centre). Not used.
- Seen, not a WP3 issue: ARKit showed "Relocalizing" / limited with 0 of 76
  poses tracked during the 1 m and 2 m screenshots (plain wall). Marker
  detection does not use ARKit's pose; recordings for WP4 need "SLAM
  tracking", i.e. a textured scene and a slow start.
- WP3 device check: **passes** (2026-11-16).

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

- WP2: chessboard rather than ChArUco (ChArUco is bound, but only `calib3d`
  is needed and every view should show the whole board anyway).
- WP2: calibration starts from ARKit's mean intrinsics
  (`CALIB_USE_INTRINSIC_GUESS`); the comparison with ARKit is reported, not
  used to constrain the result.
- WP2: time offset model `ω_c(t) = R_ci ω_i(t + δ)`; δ > 0 means IMU stamps
  lag. The ARKit pose stream (30 Hz) is used, not the 10 Hz frames.
- WP3: estimator intrinsics are the WP2 calibration (`Documents/
  calibration.json`, `CalibrationResult.load`), rescaled to the frame size;
  ARKit's per-frame intrinsics when there is no calibration or the aspect
  ratio differs. Reason: the calibration has the radial distortion ARKit does
  not report (≈ 1.5 px at the image corners) and a measured RMS; ARKit's
  per-frame values move only ≈ 0.1–0.4 px within a run. This settles the
  open item from WP2.
- WP3: the saved `detections.csv` comes from a pass over every saved frame
  (session screen → Detect markers), not from the live overlay, which skips
  frames while busy. Both decode the same JPEG bytes, so results are the same
  per frame; the file is complete and reproducible.
- WP3: ambiguity rule made concrete as "second RMS < 2 × max(best RMS,
  0.5 px)" (spec says only "similar"); detections that fail a rule are kept
  in the file with their status, since the rules apply to initialization
  and the marker factor uses corners.
- WP3: marker sheet as PDF (one A4 page per ID) rather than SVG, so ten pages
  print in one go; the printed side is measured and entered, so printer
  scaling does not matter.

## Open questions

- Answered: iPhone 15 (`iPhone15,4`), ARKit image 1920×1440 → 960×720.

## Next

WP4 (keyframes, baselines, first map). Needs the markers (IDs 0–9, all
printed at the same scaling, one side measured) on the walls, and recordings
with ARKit in "SLAM tracking".
