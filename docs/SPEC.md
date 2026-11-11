# VSLAM on iPhone: build specification

This file is the standing brief for finishing the IE415 course project in this repository. Read all of it before changing anything. It was written after reading every source file in the repo at commit `df030f6` and the course handout (`docs/IE415_handout.pdf`). If the code you find disagrees with a statement here, trust the code, say so, and adjust.

## 1. What we are building and why

IE415 (Control of Autonomous Systems) asks for an indoor visual-inertial SLAM system that uses ArUco markers as landmarks. A person carries an iPhone through a room with six to ten markers on the walls. The app records camera and IMU data, estimates the phone's trajectory, builds a map of the markers, and shows the trajectory on that map with quantitative accuracy results.

The repo today is a mid-evaluation snapshot: a Flutter app that runs an ARKit session, streams grayscale frames, poses and IMU samples into Dart, and has a start/stop capture screen. Nothing after acquisition exists. Your job is to build the rest, on iPhone only. Leave `android/` untouched; shared Dart code must keep handling a missing native side the way it does now (`MissingPluginException` is caught).

The team will be graded on understanding as much as on the result, and must explain the system in a live demo. So write code a student can read and defend: clear names, comments that explain the math and the frame conventions, no cleverness for its own sake.

## 2. Rules that come from the handout

These decide the design. Do not trade them away for convenience.

1. ARKit must not be the answer. The handout says that taking a finished ARCore/ARKit trajectory and plotting markers on top is not a complete project. ARKit is used only as an odometry front end. The final trajectory and map come from our own optimizer, where markers are measurement factors.
2. The trajectory must continue when no marker is visible.
3. Every one of the 12 minimum requirements must be demonstrably met: moving camera sequence; IMU acquisition; timestamping and synchronization; camera calibration; ArUco detection with unique IDs; marker pose relative to the camera; a continuous trajectory; marker map construction or loading; trajectory displayed relative to the map; repeated marker observations used for correction, relocalization or loop closure; comparison against at least one baseline; quantitative evaluation.
4. The interface must let a user start and stop acquisition, inspect camera and sensor status, save and load a dataset, start processing, view detected markers, view the trajectory and map, and inspect diagnostics. It must be one coherent app, not a set of scripts.
5. The submission must state which components were implemented, configured, or reused.
6. Five experiments are required: simple path, marker-loss path, closed loop, repeat run with relocalization on a saved map, and one difficult condition.

## 3. Honesty rules for this work

- You cannot walk around a room with the phone. Anything that needs the physical device is verified by the user. Never write or imply that device behaviour was tested unless the user reported the result.
- Never invent experimental numbers, example outputs, or "typical results". Metrics in the app, README and docs come only from real recordings or from clearly labelled synthetic tests.
- No stubs, placeholder screens, hard-coded results or TODOs left behind in a package you report as finished. If something cannot be finished, say exactly what is missing.
- When you rely on a library API or a platform behaviour, check it in the package source or official documentation first. Do not write calls from memory. Section 9 lists the checks that matter most.
- If a test fails, fix the cause. Do not loosen a threshold, skip the test, or special-case the test input. If a threshold in this file turns out to be wrong, stop and explain why with numbers.

## 4. Design

### 4.1 Pipeline

1. **Acquisition (Swift + Dart):** ARKit frames with per-frame intrinsics and pose, and raw accelerometer and gyroscope samples, all timestamped on one clock.
2. **Recorder and replay (Dart):** every session is saved to a folder. A replay source feeds a saved session through exactly the same processing code as live data. All estimation code takes its input from an interface that both sources implement.
3. **ArUco front end (OpenCV through `dartcv4`/`opencv_dart`):** detection, corner refinement, PnP for initialization only.
4. **Keyframes and factors (Dart, ours):** relative-motion factors from ARKit, reprojection factors from marker corners, a gyroscope consistency check.
5. **Pose-graph optimizer (Dart, ours):** Levenberg-Marquardt over keyframe poses and marker poses.
6. **Map, relocalization, visualization, evaluation (Dart, ours).**

### 4.2 Conventions already in the repo. Keep them.

- `Pose3` in `lib/core/se3.dart` is `aTb`: it maps points from frame b to frame a, and composes as `aTb * bTc = aTc`.
- `ArFrame.vioPose` (via `arPoseToVio` in `lib/slam/ar_pose.dart`) gives `vTc`: the camera in OpenCV convention (x right, y down, z forward) expressed in a gravity-aligned, z-up world. Use this, not the raw ARKit pose.
- Perturbations are on the right: `Pose3.retract` applies `R <- R * Exp(dtheta)`, `t <- t + R * dt`, with delta ordered `[dt; dtheta]`. `poseError(e)` returns `[t; Log(R)]`. Build every residual and Jacobian on these two functions so covariances stay consistent.
- `so3Exp`, `so3Log`, `projectToYaw` and `CameraIntrinsics` (pinhole with k1, k2, with `project`, `normalize`, `scaledTo`) already exist. Extend them, do not duplicate them.
- Frames arrive in sensor orientation (landscape) while the UI is locked to portrait. Any overlay must map image pixels to the displayed view explicitly.
- Timestamps are microseconds since the epoch on the boot-based clock (`ArPlugin.epochMicros`).

### 4.3 Frames

- **W**: room or map frame, z up. Mapping mode: W is the first keyframe's gravity-aligned frame. Anchoring mode: W is the tape-measured room frame.
- **V**: ARKit session frame after `arPoseToVio`, z up, arbitrary origin and heading per run.
- **C_k**: camera at keyframe k, OpenCV convention.
- **M_i**: marker i. Origin at the centre, x right, y up, z out of the marker toward the viewer. OpenCV returns corners in the order top-left, top-right, bottom-right, bottom-left, so the object points for side length s are `(-s/2, s/2, 0), (s/2, s/2, 0), (s/2, -s/2, 0), (-s/2, -s/2, 0)`.
- **B**: device (IMU) frame.

Both W and V are gravity aligned, so a transform between them is yaw plus translation. Use `projectToYaw` when aligning them.

### 4.4 State and factors

Unknowns: keyframe poses `X_k = wTc_k` and marker poses `L_i = wTm_i`.

- **Keyframes:** add one after 0.25 m of motion, 15 degrees of rotation, or a marker detection at least 0.3 s after the previous keyframe. Make these values configuration.
- **Odometry factor** between consecutive keyframes: measurement `Z_k = (vTc_k)^-1 * vTc_k+1` from ARKit; residual `poseError(Z_k^-1 * X_k^-1 * X_k+1)`, six numbers. Noise grows with step length and is inflated when ARKit reports limited tracking or when the gyroscope check fires.
- **Marker factor:** reprojection error of the four corners, eight numbers: `project(X_k^-1 * L_i * p_j) - u_kij`, using the calibrated intrinsics and distortion. Noise is in pixels. Use a Huber weight on the eight-vector norm, followed by the gating pass described in section 6. Do not use the PnP pose as the measurement: a planar marker seen head-on has two plausible poses, and reprojection factors avoid baking the wrong one into the map.
- **Marker initialization:** PnP with the IPPE square solver on the first accepted detection, composed with the current keyframe estimate. Reject a detection when the marker side is under 20 px, the viewing angle from the marker normal exceeds 65 degrees, the reprojection RMS exceeds 2 px, or the two IPPE solutions have similar reprojection error.
- **Gyroscope check:** integrate gyroscope samples between two keyframes and compare the rotation with ARKit's relative rotation, after applying the camera-IMU rotation and time offset. A disagreement above a threshold means ARKit corrected itself internally (a jump); inflate that odometry factor's noise. This is one of the places the IMU takes part in estimation.
- **Gauge:** mapping mode fixes the first keyframe. Anchoring mode puts tight priors on surveyed marker poses. Relocalization mode loads marker poses from a saved map and holds them fixed.
- **Loop closure:** seeing a known marker ID again adds a factor to the same landmark. Record the loop-closure residual (predicted versus stored marker position) before re-optimizing.

Put every noise parameter and threshold in one configuration class with documented defaults: odometry translation sigma `0.002 m + 0.02 * step`, rotation sigma 0.3 degrees per step, corner sigma 0.5 px at 960 px width, Huber delta 1.345. These are starting values; they will be tuned on recorded data and reported.

### 4.5 Solver

Levenberg-Marquardt with analytic Jacobians, every one verified against central differences in a test. Run it in an isolate so the UI never blocks.

A reference simulation of exactly this formulation was run before writing this file (6 x 5 m room, eight 15 cm markers, 960 x 720 camera with fx = 725, two laps, 28 m, 153 keyframes, 960 unknowns, drift from 1% scale error and a yaw bias of 0.06 degrees per step). Manifold LM with IRLS Huber converged in 10 to 28 iterations from the drifted odometry initial guess. The script is `tools/reference_sim.py` (Python, needs numpy and scipy; run with `--seed N`, `--mode map|anchor`, `--use 0,2,4,6`, `--outliers 6`). Results over five random seeds for mapping, three for anchoring and two for the four-marker case:

| Mode | Odometry only | Optimized |
| --- | --- | --- |
| Mapping (markers unknown), trajectory RMSE | 20 to 32 cm | 3 to 8 cm |
| Mapping, endpoint error | 25 to 57 cm | 1 to 6 cm |
| Mapping, marker position error (mean) | n/a | 4 to 9 cm |
| Anchoring (markers known), trajectory RMSE | 20 to 32 cm | 1 to 2 cm |
| Four markers instead of eight, mapping RMSE | 22 to 25 cm | about 8 cm |

Two lessons from it. First, mapping mode cannot remove error that the first lap baked into the map (scale and early yaw drift are not observable without known marker positions), so do not expect centimetre accuracy there; anchoring mode is where that appears. Second, a generic least-squares routine with numeric Jacobians stalled on this problem while manifold LM converged quickly, so the convergence test in section 6 matters.

A dense solve at 960 unknowns is fine for batch processing. Measure it on the device. If one batch optimization takes more than about 5 s on the phone, exploit the structure: the pose block is block-tridiagonal and markers form a small dense border, so eliminate with a Schur complement on the marker block.

### 4.6 Baselines

1. ARKit-only trajectory (odometry factors chained, no marker factors). This is the handout's main comparison.
2. IMU-only dead reckoning from raw samples with biases estimated from a few seconds at rest. It will diverge within seconds; that is the point, and it shows the handout's state `[p, v, q, b_a, b_g]` in code.

## 5. Work packages

Work in this order. One package per session is a good size. After each package: all checks in section 6 pass, `docs/PROGRESS.md` is updated, a device checklist for the user is appended to `docs/DEVICE_CHECKS.md`, the work is committed, and you stop and report. Do not start the next package until the user confirms the device checks, because later packages depend on real data from earlier ones.

**WP1. Dataset recorder and replay.** Save frames (JPEG at quality 0.9 or better, written off the main thread, no dropped frames at 10 Hz), frame index with timestamps and per-frame intrinsics, ARKit poses, and IMU samples. Raise the streamed frame width from 640 to 960 px (the current box filter in `ArPlugin.swift` uses an integer factor; keep the intrinsics rescaling correct). Disable ARKit autofocus so intrinsics do not change during a run. Add session metadata (device, OS, app version, experiment type, marker dictionary and side length, notes). Add the replay source and a sessions list (open, rename, delete, export through the share sheet). `Info.plist` already enables file sharing, so sessions under the app's Documents directory are visible in Finder and Files.

Session layout:

```
sessions/<date>_<time>_<label>/
  meta.json
  frames.csv          index, t_us, file, fx, fy, cx, cy, tracking_state
  frames/000001.jpg
  arkit_poses.csv     t_us, tx, ty, tz, qx, qy, qz, qw, tracking_state
  imu.csv             t_us, sensor, x, y, z
  results/            detections.csv, traj_arkit.csv, traj_imu.csv,
                      traj_optimized.csv, map.json, metrics.json
maps/<room>.json
ground_truth/<room>.json
```

**WP2. Calibration, IMU timing and alignment.** A calibration screen that collects 15 to 20 views of a printed board and runs OpenCV calibration with tangential distortion and k3 fixed to zero, so the result fits `CameraIntrinsics` (fx, fy, cx, cy, k1, k2). Store the RMS reprojection error and show a comparison with ARKit's intrinsics at the same resolution. From a short recording where the phone is rotated about each axis, estimate the camera-IMU time offset (the shift that best aligns gyroscope rate with the rotation rate from consecutive ARKit orientations) and the camera-IMU rotation (least-squares alignment of the two angular-velocity sets). Compare the estimated rotation with the axis convention Apple documents and report both.

**WP3. ArUco detection and pose.** `DICT_4X4_50`, subpixel corner refinement, the rejection rules from 4.4, marker side length as a setting. Live overlay of outlines and IDs on the camera view. Write detections to the session. Add a tool that produces print-ready markers (IDs 0 to 9) and the calibration board at true scale, with a printed ruler line so the user can confirm the scale after printing.

**WP4. Keyframes, baselines, first map.** Keyframe selection, baseline 1, baseline 2, and a first marker map built by placing each marker from the baseline pose at first sight. Save and load maps as JSON.

**WP5. Pose-graph optimizer.** Everything in 4.4 and 4.5. Batch mode after a recording, incremental mode during a live recording (re-optimize when a known marker is seen again; between optimizations, propagate the latest optimized keyframe with odometry).

**WP6. Anchoring and relocalization.** Ground-truth file format for a room (surveyed marker poses, floor checkpoints). Anchoring mode. Relocalization: load a map, get the first pose from the first accepted detection (`wTc = wTm * (cTm)^-1`), keep tracking, record time to first fix.

**WP7. Map view and interface.** Top-down view with baseline and optimized trajectories, markers with ID and facing direction, scale bar, pinch zoom, tap a marker for its pose and observation count. Processing screen with progress and switches for markers on/off and for mapping, anchoring or relocalization mode. Extended diagnostics (rates, markers in view, time offset, last optimization time, iterations and final cost). Plain, neutral visual design; no decoration.

**WP8. Evaluation and export.** Checkpoint position error, endpoint drift, drift per metre, heading error, loop-closure residual, marker position and orientation error against the survey, relocalization error and time, detection rate, per-frame detection time and optimization time. A results screen and CSV export. A marker-subset option so the same recording can be reprocessed with only some marker IDs (sparse versus dense comparison).

**Documentation, updated as each package lands:** `README.md` (status, build, how to run each experiment); `docs/DESIGN.md` (architecture, frames, data flow, calibration procedure, estimation method with the equations, and the implemented / configured / reused split); `docs/EXPERIMENTS.md` (room setup, marker placement record, the five protocols, how ground truth is measured).

## 6. Testing

A package is not finished until all of this holds.

**Always:**
- `flutter analyze` reports no issues.
- `flutter test` passes, including the existing tests in `test/`.
- `flutter build ios --debug --no-codesign` succeeds, which proves the Swift code compiles.

**Unit tests to add with the package that introduces the code:**
- SE(3): `retract` and `poseError` are inverse to first order; compose and inverse identities.
- Camera model: project then normalize round trip with distortion; `scaledTo` consistency.
- Every factor: analytic Jacobian against central differences at random states, relative error under 1e-6.
- Dataset: write a synthetic session, read it back, compare every field; replay yields the same event order and counts as the original stream.
- Keyframe selection on a scripted trajectory.
- Time-offset and rotation alignment: recover a known offset and rotation from synthetic gyroscope and orientation data with noise.
- IMU propagation: a constant-velocity and a constant-rotation case against the closed form.
- Metrics: each metric on a hand-computed example.
- Map JSON round trip.

**Synthetic end-to-end tests (the main proof that the estimator is right):** build a simulator in `test/support/` that generates a room, marker poses, a ground-truth walk, noisy corner observations through the real camera model, and drifting odometry. Use fixed random seeds. With the scenario described in 4.5 the tests must show:
- Mapping mode: trajectory RMSE at least 60% lower and endpoint error at least 75% lower than the odometry-only baseline, on each of five seeds.
- Anchoring mode: trajectory RMSE under 3 cm.
- Relocalization: a second synthetic run with a different start pose and heading, localized against the true marker poses as the saved map, has trajectory RMSE under 3 cm from the first fix onward.
- Marker loss: with all detections removed for a 20-keyframe stretch, the trajectory stays continuous and the error drops again when markers return.
- Convergence from the drifted odometry initial guess in under 50 iterations.
- An injected odometry jump is flagged by the gyroscope check, and the optimized error with the check is lower than without it.
- Outliers: with one corner displaced by 30 px in 6 of about 130 detections, anchoring mode moves by under 1 cm on average, and mapping mode still beats the baseline RMSE by at least 50%. In the reference simulation, Huber weighting alone held anchoring to a 0.2 cm mean shift but let mapping mode shift by 4 cm on average (7 cm at worst), so add a gating pass: after the first convergence, drop observations whose weighted residual is far above the rest and re-optimize. Show in a test that gating reduces the mapping-mode shift.

If your numbers differ much from the reference table in 4.5, find out why before moving on.

**Detector tests:** render a marker with OpenCV, warp it with a known pose through the camera model, detect it, and check ID, corners (under 0.5 px) and recovered pose. If the native OpenCV library cannot load under `flutter test` on macOS, keep the detector behind an interface, test everything else with a fake, and list the detector as device-verified only.

**Widget tests:** each new screen renders, and the record, process, view flow works against a synthetic session.

**Regression on real data:** after the user's first successful recording, add a replay test that runs the full pipeline on that session and checks counts and basic sanity. Keep large recordings out of git; document where they live.

**Device checks:** for each package write the exact steps and expected observations in `docs/DEVICE_CHECKS.md`, for example: record 60 s and confirm frame and IMU counts match the expected rates; hold a tape-measured marker at 1, 2 and 3 m and confirm the reported distance is within 3%; walk a closed loop and confirm the optimized endpoint error is smaller than the baseline's.

## 7. Working method

- Start by reading `README.md`, every file under `lib/` and `test/`, `ios/Runner/ArPlugin.swift`, `ios/Runner/AppDelegate.swift`, `pubspec.yaml` and the handout PDF. Run `flutter --version`, `flutter pub get`, `flutter analyze` and `flutter test` and report the starting state before editing.
- Work on a branch named `iphone-slam`. Commit after each coherent step with a message that says what changed and why. Do not push, force-push or touch `main` unless the user asks. The repo is shared with a teammate.
- Keep `docs/PROGRESS.md` current: packages done, checks run with their results, device checks pending, open questions, decisions made and why. A new session must be able to continue from that file and this one alone.
- Ask the user when a decision changes scope or cannot be undone. Otherwise decide, and record the decision.
- Add dependencies only when needed, and say why. Expected ones: `opencv_dart` or `dartcv4`, `path_provider`, a share-sheet package, possibly `archive`.

## 8. Layout for new code

```
lib/dataset/       session_writer, session_reader, replay_source
lib/calibration/   camera_calibrator, time_offset, imu_alignment
lib/vision/        aruco_detector, marker_observation   (camera_model exists)
lib/estimation/    config, keyframe_selector, factors, pose_graph, lm_solver,
                   gyro_check, imu_propagator, relocalizer
lib/mapping/       marker_map, map_store
lib/evaluation/    ground_truth, metrics
lib/screens/       sessions, process, map, calibration, results   (home exists)
lib/widgets/       marker_overlay, map_painter
test/support/      simulator
tools/             marker and board generator
```

## 9. Things to verify before relying on them

- **OpenCV modules.** In `dartcv4` 2.x only `core`, `imgproc` and `imgcodecs` are built by default; `calib3d`, `objdetect` and `aruco` are excluded and must be enabled in `pubspec.yaml` under `hooks: user_defines: dartcv4: include_modules:`. OpenCV is compiled from source on the first build, which needs CMake and takes a long time. Read the current README of the version you install for the exact keys and for iOS notes, and read the Dart binding source for the ArUco, PnP and calibration function names and signatures. Confirm whether ChArUco board detection is bound; if not, use a chessboard.
- **Fallback if that route fails on iOS:** an OpenCV iOS framework called from the Swift plugin through a small Objective-C++ wrapper, returning corners and IDs to Dart. Decide early, in WP3, by building a minimal detection on the device.
- **IMU timestamps and rate.** Open the `sensors_plus` iOS source in the pub cache and check whether event timestamps come from `CMLogItem.timestamp` or from the wall clock, and what rate `SensorInterval.gameInterval` gives. The estimator needs sensor timestamps on the same boot-based clock as `ARFrame.timestamp` and about 100 Hz. If `sensors_plus` does not provide that, read `CMMotionManager` directly in `ArPlugin.swift`.
- **Accelerometer convention.** Confirm units and sign with the phone at rest (magnitude near 9.81 m/s^2 and which way it points) before writing the IMU propagator, and document it.
- **ARKit details.** Confirm the autofocus property on `ARWorldTrackingConfiguration`, the resolution of `capturedImage`, and that `camera.intrinsics` refers to that resolution.
- **Flutter and Dart versions** required by the packages against what is installed.

## 10. Definition of done

- All 12 minimum requirements are met in the app and each is traceable to code and to a line in `docs/DESIGN.md`.
- All tests in section 6 pass and the iOS build succeeds.
- Every device check has been run by the user and its result recorded in `docs/PROGRESS.md`.
- The five experiments can be recorded, processed and evaluated in the app, with baseline and optimized metrics exported.
- README and docs describe the system as it actually is, including its limits and where it fails.
