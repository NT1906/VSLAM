# Handoff: continuing the iPhone VSLAM build

For the teammate taking over on branch `iphone-slam`. This file says where
the project stands, how to get a working setup, and what to build next, in
order. It does not replace the three standing documents; read them in this
order:

1. `docs/SPEC.md`: the brief. Rules, design, the work packages (WP1–WP8),
   the tests every package needs (section 6), what to verify before using
   a library (section 9), definition of done (section 10).
2. `docs/PROGRESS.md`: what was done in each package, every check run and
   its numbers, device results, decisions and why.
3. `docs/DESIGN.md`: how the system works now, with the equations; section
   8 is the implemented / configured / reused table the submission needs.

Also: `docs/DEVICE_CHECKS.md` (step-by-step phone checks per package),
`docs/EXPERIMENTS.md` (recording protocol, to be completed), and the course
handout `docs/IE415_handout.pdf`.

## 1. Where things stand

| Package | Code | Tests on the Mac | Device check |
| --- | --- | --- | --- |
| WP1 dataset recorder, replay | done | pass | **passed** (2026-11-12) |
| WP2 calibration, IMU timing and alignment | done | pass | **passed** (RMS 0.467 px; R_ci 0.10° from Apple's axes; time offset only bounded) |
| WP3 ArUco detection and pose | done | pass | **passed** (1.006 m at 1 m, 2.058 m at 2 m) |
| WP4 keyframes, baselines, first map | done | pass | **in progress** on the original phone (4.0 passed); to be done on the new phone (sections 4 and 5 below) |
| WP5 pose-graph optimizer | not started | | |
| WP6 anchoring, relocalization | not started | | |
| WP7 map view, interface | not started | | |
| WP8 evaluation, export | not started | | |

Last full check (WP4 commit `05398ab`): `flutter analyze` no issues,
`flutter test` 96 pass, `flutter build ios --debug --no-codesign` and
`--release --no-codesign` pass.

**Phone change:** WP1–WP3 device results come from the original iPhone 15
(`iPhone15,4`). The work continues on the teammate's own iPhone, so a few
of those checks are repeated first (section 4).

The spec asks to finish a package's device check before starting the next,
because later packages need real data. WP5 can be *coded* against the
simulator before the WP4 recording exists, but its real-data test and
tuning need that recording.

## 2. Setting up a Mac

- Flutter 3.47.9 / Dart 3.13.7 (the repo was made with it; installed at
  `~/development/flutter` on the original Mac). Xcode 27.2 with the iOS
  SDK. CMake (`brew install cmake`): `dartcv4` compiles OpenCV from source.
- `flutter pub get`, then `flutter test`. The **first** test run or build
  compiles OpenCV and takes a long time; later runs are fast.
- iOS: open `ios/Runner.xcworkspace` once in Xcode and pick your own
  development team under Signing for the Runner target (the project has
  the original author's team).
- Install on the phone: `flutter run --release` with the phone connected
  (cable or the same Wi-Fi; `flutter devices` lists it). For a one-shot
  install without staying attached: `flutter run --release --no-resident -d <device id>`.
- No Android work is planned; leave `android/` alone (spec section 1).

### dartcv4 2.3.1 pitfalls (read before your first failure)

- `pubspec.yaml` → `hooks: user_defines: dartcv4: include_modules:`
  **replaces** the default module list. imgproc and imgcodecs must stay in
  it, or `imdecode` / `imencode` are missing at run time.
- Link-hook bug: release builds write
  `.dart_tool/hooks_runner/shared/dartcv4/dartcv_keep.txt`, which breaks the
  next native build with "Undefined symbols _cv_freetype_…". An Xcode build
  phase ("Clear dartcv4 keep-list") deletes it after every iOS build. If
  `flutter test` ever fails with those symbols, delete that file and rerun.
- Never run `dart run tools/…` and `flutter test` at the same time (they
  share the OpenCV build folder and both fail).
- Seen once while OpenCV was compiling in parallel: a dataset test failed in
  tearDown with "Directory not empty". Not reproduced since.

## 3. Rules the work follows (spec sections 2, 3, 6)

- ARKit is only the odometry front end. The final trajectory and map must
  come from our own optimizer with markers as measurement factors.
- Check every library call in the package source before using it, and
  write in `PROGRESS.md` what you checked.
- No stubs, TODOs or placeholder screens in a package called finished.
- If a test fails, fix the cause. Never loosen a threshold, skip a test or
  special-case its input; if a threshold is wrong, stop and explain it with
  numbers. Set test thresholds before the first run. (Test *mistakes* may
  be fixed; say so in PROGRESS, as was done in WP2–WP4.)
- Never write device results nobody measured. Numbers in the app and docs
  come from real recordings or clearly labelled synthetic tests.
- After each package: analyze, all tests, both iOS builds pass; update
  `PROGRESS.md`, `README.md`, `DESIGN.md`; append the device checklist to
  `DEVICE_CHECKS.md`; commit on `iphone-slam`. Don't touch `main`.
- Thresholds and noise parameters live in one place:
  `lib/estimation/config.dart` (`SlamConfig`), each with a comment.

## 4. First: set up your own iPhone (about 30 minutes)

The camera calibration and the marker-distance result belong to one
camera, so they are redone on the new phone before any WP4 recording. The
camera–IMU rotation (WP2 check 2.3) is **not** redone: it is Apple's
documented axis convention, the same on every iPhone (measured 0.10° from
it on the original phone). Nothing is copied from the original phone; the
old recordings are not needed (their results are in `PROGRESS.md`).

Steps, all in `docs/DEVICE_CHECKS.md`:

1. **Install.** Open `ios/Runner.xcworkspace`, Runner target → Signing &
   Capabilities → choose your team. If Xcode says the bundle ID
   `com.vslam.app` is not available to your account, change it to your own
   (e.g. `com.<yourname>.vslam`); nothing else depends on it. On the phone
   turn on Developer Mode (Settings → Privacy & Security) and trust the Mac.
   Then `flutter run --release` with the phone on a cable.
2. **Camera and IMU stream (WP1 check 1.1, 2 min).** Pan until "SLAM
   tracking", open diagnostics (heart-monitor icon). Expected: image
   `960×720`, frames about 10 Hz, your model in the device row. If the image
   is another size, stop: this phone's ARKit video format is not
   1920 × 1440, and the frame reduction in `ios/Runner/ArPlugin.swift`
   (integer box filter) and every pixel threshold (tuned at 960 px width)
   need checking first. Optional: record 10 s and check on the session
   screen that the IMU runs at about 100 Hz.
3. **Calibrate the camera (WP2 checks 2.1–2.2, 15 min).** Print
   `docs/calibration_board.svg`, measure the printed square (the original
   printer gave 20.6 mm instead of 25 mm), enter it, take 15–20 varied views,
   Calibrate. Expected RMS < 0.5 px, focal length within 1–2 % of ARKit's.
   Focus is fixed (autofocus off), so keep views at 35 cm or more. Afterwards
   the status line at the bottom of the camera view says "calibrated K".
4. **Marker distance (WP3 checks 3.1–3.2, 10 min).** Print marker ID 0 at
   100 %, measure the black square (150 mm), enter the side in the settings
   row, tape it on a wall, check the distance at 1 m and 2 m (within 3 %).
5. Record the results (phone model, image size, calibration RMS and
   intrinsics, the two distances) in `PROGRESS.md` under a "new phone"
   heading, then continue with section 5.

## 5. Next step: finish the WP4 device check

Full text in `docs/DEVICE_CHECKS.md` → WP4. State when this was handed over:

- **4.0** passed on the original phone. On the new phone it is covered by
  section 4 (install, "calibrated K" on the status line).
- **4.1 (next).** Print marker IDs 1–4 (pages 2–5 of
  `docs/markers_4x4_50_ids0-9.pdf`) at 100 %, check the square is 150 mm,
  tape IDs 0–4 on at least two walls of a **textured** room at chest
  height, 1–2 m apart. Tape-measure the centre distance ID 0 → ID 1.
  Settings row: label `wp4-room`, experiment Closed loop, side 15.0.
- **4.2.** Record about 90 s: hold still until the badge says "SLAM
  tracking" and 3 s more (IMU bias window), then walk slowly once round the
  room, aiming at each marker from 1–1.5 m **at an angle** for about 2 s,
  back to the start, Stop.
- **4.3.** Session → **Build map**; screenshot the card. Check: IMU rest
  "still", biases small, IMU-vs-ARKit distance growing to metres, IDs 0–4
  all placed with similar z, nothing under "Not placed".
- **4.4.** **Save map** as `lab`; **Export** the zip to the Mac.

Then on the Mac:
- Compare the ID 0–ID 1 distance in `results/map.json` with the tape
  measure; record both in `PROGRESS.md` (WP4 device check section).
- Add the real-data regression test the spec asks for (section 6): replay
  the exported session through `computeBaselines`, check counts and sanity
  (keyframes > 0, all IDs placed, rest window found). Keep the recording
  **out of git**; make the test skip with a clear message when the folder
  is absent, and write in `PROGRESS.md` where the zip lives.

Device facts already learned (keep them in mind for every recording):
- ARKit stays "Relocalizing" in front of a plain wall: always record in a
  textured scene and wait for "SLAM tracking".
- Head-on marker views are "ambiguous" from about 1.5 m; markers need to be
  seen at an angle (≈ 30°+) at least once to be placed.
- Focus is fixed (autofocus off since WP1): nothing is sharp closer than
  about 35 cm.
- Printers may scale pages (the calibration board came out at 82 %): always
  measure the printed marker side and enter it in the settings.
- ARKit's per-frame intrinsics drift 0.1–0.4 px within a run; the estimator
  uses the WP2 calibration instead (`chooseIntrinsics`).

## 6. Map of the code

```
lib/core/          Pose3 (aTb), so3Exp/so3Log, poseError, projectToYaw
lib/dataset/       SensorEvent, SessionWriter/Reader, ReplaySource, store
lib/calibration/   chessboard calibration, imu_alignment (appleCameraFromDevice = R_ci)
lib/vision/        CameraIntrinsics (k1, k2), ArUco detector + IPPE, MarkerObservation,
                   detections.csv, chooseIntrinsics, marker sheet
lib/estimation/    config (SlamConfig), keyframe_selector (Keyframe, OdometryEdge),
                   imu_propagator, baselines, session_baselines (Build map step)
lib/mapping/       marker_map (MarkerMap JSON, first-sight map, MapStore)
lib/screens/       home, sessions list, session detail (Detect markers, Build map, Save map)
lib/widgets/       camera view, marker overlay, calibration panel, diagnostics
test/support/      simulator.dart (RoomSimulation), synthetic_session, marker_render
ios/Runner/ArPlugin.swift   ARKit session, frames, CoreMotion 100 Hz IMU
tools/             make_board.dart, make_markers.dart
```

Conventions that must not change (spec 4.2, DESIGN section 2): `Pose3` is
`aTb` and composes `aTb * bTc = aTc`; perturbations are on the right
(`retract`: `R ← R·Exp(δθ)`, `t ← t + R·δt`, `δ = [δt; δθ]`) and
`poseError(E) = [t; Log(R)]`; use `vioPose` (`vTc`, OpenCV camera, z-up)
never the raw ARKit pose; timestamps are µs on the boot-based epoch clock.

Session results so far: `results/detections.csv` + `detections.json` (WP3),
`imu_alignment.json` (WP2), `traj_arkit.csv`, `traj_imu.csv`, `map.json`,
`baselines.json` (WP4). Formats are in DESIGN sections 6.4 and 7.

## 7. WP5: pose-graph optimizer (the core of the project)

Spec sections 4.4, 4.5 and 6. Suggested files (spec section 8):
`lib/estimation/factors.dart`, `pose_graph.dart`, `lm_solver.dart`,
`gyro_check.dart`.

What already exists to build on:
- Keyframes and odometry: `KeyframeSelector`, `OdometryEdge` (measurement
  `z`, `stepM`, `rotationDeg`, `trackingGap`).
- Initial guess: baseline 1 poses (`chainOdometry`) for `X_k`, the
  first-sight map (`buildFirstSightMap`) for `L_i`.
- Detections per frame time: `readDetections`; corners are in
  `MarkerObservation.corners`, OpenCV order, matching
  `markerObjectPoints(side)`.
- Camera model with distortion: `CameraIntrinsics.project`;
  intrinsics via `chooseIntrinsics(frame intrinsics, calibration)`.
- Simulator: `RoomSimulation` / `SimConfig` with `SimConfig.drifting(seed)`
  (1 % scale, 0.24°/m yaw bias = the spec 4.5 drift), `roomInW(wTv)` to
  compare with truth, `writeSession` to run the real file pipeline.

To build:
1. **Odometry factor** (6-vector): `poseError(Z_k⁻¹ · X_k⁻¹ · X_k+1)`.
   σ_t = 0.002 m + 0.02 · step, σ_r = 0.3° per step; inflate when
   `trackingGap` or the gyro check fires. Add these to `SlamConfig`.
2. **Marker factor** (8-vector): `project(X_k⁻¹ · L_i · p_j) − u_kij` for
   the four corners, σ = 0.5 px at 960 px; Huber (δ = 1.345) on the
   whitened 8-vector norm; then a **gating pass** after first convergence
   (drop observations whose weighted residual is far above the rest,
   re-optimize). Use detections with statuses in
   `keyframeMarkerStatuses` at keyframe times; the PnP pose is only for
   initialization.
3. **Analytic Jacobians** for both factors, each checked against central
   differences at random states (relative error < 1e-6) in a test.
4. **LM on the manifold** (update with `retract`), dense normal equations
   first. Gauge: mapping mode fixes `X_0`. Run in an isolate
   (`Isolate.run`, as the detector does).
5. **Gyro check**: integrate gyroscope between keyframes (reuse
   `ImuSeries`/`quatExp` from `imu_propagator.dart`, `R_ci =
   appleCameraFromDevice`, offset 0) and compare with ARKit's relative
   rotation; above a threshold, inflate that edge's noise.
6. **Loop closure**: a re-seen ID adds a factor to the same landmark; log
   the predicted-vs-stored marker position before re-optimizing.
7. Batch mode from the session screen (writes `results/traj_optimized.csv`
   and an updated `map.json`); incremental mode during live recording
   (re-optimize on a re-seen marker, propagate with odometry in between).
8. Measure one batch solve on the phone; above ≈ 5 s use a Schur complement
   on the marker block (pose block is block-tridiagonal).

Simulator additions WP5 needs (keep `RoomSimulation` backwards compatible):
an odometry **jump** injection (for the gyro check test), **outlier**
corners (one corner moved 30 px in 6 of ~130 detections), a way to remove
all detections for a 20-keyframe stretch (marker loss), and a second run
with a different start pose and heading (relocalization, WP6).

Tests that must pass (spec section 6, thresholds fixed in advance):
- Mapping: trajectory RMSE ≥ 60 % lower and endpoint error ≥ 75 % lower
  than odometry only, on each of five seeds.
- Anchoring: RMSE < 3 cm. Relocalization: RMSE < 3 cm from first fix.
- Marker loss: continuous trajectory, error drops when markers return.
- Convergence from the drifted initial guess in < 50 iterations.
- Gyro check flags an injected jump; optimized error lower with it.
- Outliers: anchoring moves < 1 cm on average; mapping still ≥ 50 % better
  than baseline RMSE; gating reduces the mapping-mode shift.
- Reference numbers to expect: spec 4.5 table (mapping 3–8 cm RMSE vs
  20–32 cm odometry; anchoring 1–2 cm). If yours differ much, find out why.

Note: the spec refers to `tools/reference_sim.py` (the Python reference
simulation behind that table). **It is not in this repository.** Ask
whoever wrote the spec for it, or rely on the table only.

## 8. WP6–WP8 in short

- **WP6 anchoring and relocalization.** `ground_truth/<room>.json` format
  (surveyed marker poses and floor checkpoints, tape-measured room frame);
  anchoring mode = tight priors on surveyed markers; relocalization = load
  `maps/<room>.json` (`MapStore.load`), first pose from the first accepted
  detection `wTc = wTm · (cTm)⁻¹`, markers held fixed; record time to first
  fix. Needs `EXPERIMENTS.md` to say how to survey the room.
- **WP7 map view and interface.** Top-down view: baseline and optimized
  trajectories, markers with ID and facing, scale bar, pinch zoom, tap a
  marker for pose and observation count; processing screen with progress and
  switches (markers on/off; mapping / anchoring / relocalization);
  diagnostics (rates, markers in view, time offset, last optimization time,
  iterations, final cost). Plain design. Readers already exist:
  `readTrajectory`, `MarkerMap.load`.
- **WP8 evaluation and export.** Checkpoint error, endpoint drift, drift per
  metre, heading error, loop-closure residual, marker position/orientation
  error vs survey, relocalization error and time, detection rate,
  per-frame detection time, optimization time; results screen and CSV
  export (`results/metrics.json`); marker-subset reprocessing (sparse vs
  dense). Each metric needs a hand-computed unit test.
- **Experiments** (handout, spec 2.6): simple path, marker-loss path,
  closed loop, repeat run with relocalization on the saved map, one
  difficult condition. Each has an experiment type in the recording
  settings already.

## 9. Known limits and open points

- Camera–IMU time offset: only bounded ("no large offset") by WP2's slow
  rotation run; the estimator uses 0. Fine for keyframes ≥ 0.25 s apart.
- `R_ci` is Apple's documented axes (measured 0.10° from it). ARKit fuses
  the same gyroscope, so this checks conventions rather than giving an
  independent calibration; say so in the report.
- IMU dead reckoning ignores the camera–IMU lever arm (a few cm).
- Mapping mode cannot remove scale and early-yaw error baked in on the first
  lap (spec 4.5); centimetre accuracy is only expected in anchoring mode.
- Calibration RMS 0.467 px is near the 0.5 px limit; a stiffer board would
  likely lower it.
- Recordings are kept outside git. The sessions recorded so far live on the
  original iPhone (Files → On My iPhone → VSLAM → sessions), together with
  that phone's camera calibration (`calibration.json`). The WP1 export zips
  (`2026-11-12_09-01-42_wp1-static.zip`, `2026-11-12_09-05-51_wp1-walk.zip`)
  were not found on the original Mac on 2026-11-19. What to copy over is
  listed in `docs/HANDOVER_CHECKLIST.md`.
