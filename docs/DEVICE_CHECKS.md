# Device checks

Checks that need the real iPhone. Run them in order and record the result
(numbers, screenshots or "as expected") in `docs/PROGRESS.md`. Nothing in this
file has been run yet unless PROGRESS.md says so.

## WP1: dataset recorder and replay

Build and install: `flutter run --release` with the phone connected (Xcode
needs a development team set once in `ios/Runner.xcworkspace`).

### 1.1 App starts, camera and IMU stream

1. Open the app, allow camera and motion access.
2. Pan slowly until the badge shows "SLAM tracking".
3. Open the diagnostics (heart-monitor icon).

Expected:
- Image `960×720, AR SDK intrinsics`. If it shows another size, write down the
  size: it means this phone's ARKit video format is not 1920×1440.
- Frames about 10 Hz.
- Device row shows the hardware model (e.g. `iPhone15,2`) and iOS version.

### 1.2 Settings are kept

1. Tap the settings row above the tiles. Set label `wp1-static`, experiment
   "Calibration / alignment", marker side to the printed size (e.g. 15.0 cm),
   a note. Save.
2. Force-quit the app and open it again.

Expected: the row shows the same label, experiment and marker size.

### 1.3 60-second static recording (rates, no dropped frames, IMU sign)

1. Lay the phone flat on a table, screen up, camera looking at a textured
   surface, and wait for "SLAM tracking". (Raise one edge on a book if the
   camera sees nothing.)
2. Press Start, wait 60 s by the Elapsed tile, press Stop.
3. Tap Open on the "Saved …" notice.

Expected on the session screen:
- Frames ≈ 600 at 10.0 Hz (±0.2), **0 gaps**, 0 without image.
- ARKit poses ≈ 1800 at about 30 Hz.
- Accelerometer and gyroscope each ≈ 6000 at about 100 Hz.
- Time span: frames and IMU both ≈ 60 s; IMU starts within 0.2 s of the
  first frame.

Then in the Files app (On My iPhone → VSLAM → sessions → this folder), or on
the Mac after export (1.5), open `imu.csv`:
- With the phone flat and screen up, `acc` rows have z ≈ +9.8 and x, y ≈ 0.
- `gyro` rows are all within about ±0.02 rad/s.

Report the measured rates, gap count and a typical `acc` row.

### 1.4 Walking recording (frames under motion, autofocus off)

1. Set label `wp1-walk`, experiment "Simple path".
2. Hold the phone upright and walk slowly around the room for about 60 s,
   including one point where you look at something close (≈ 30 cm) and then
   something far.
3. Stop and open the session.

Expected:
- Frames ≈ 10 Hz, 0 gaps (report the gap count if not 0).
- ARKit poses mostly tracked.
- The first-frame picture is a sharp gray landscape image.
- In `frames.csv`, the `fx` value is identical on every row (autofocus is off,
  so the focal length must not change). Report min and max of `fx`.

### 1.5 Export, Finder, rename, delete

1. On the `wp1-walk` session, tap Export and send the zip to the Mac
   (AirDrop or Save to Files).
2. Unzip on the Mac. Expected: one folder with `meta.json`, `frames.csv`,
   `arkit_poses.csv`, `imu.csv`, `frames/` with about 600 JPEGs (gray,
   960×720), and an empty `results/`.
3. Connect the phone to the Mac, open it in Finder → Files → VSLAM. Expected:
   the `sessions` folder is visible.
4. Back in the app, Rename `wp1-walk` to `wp1-walk-renamed`. Expected: the
   folder name changes, the date-time prefix stays.
5. Record a 5 s throwaway session and Delete it. Expected: a confirmation
   dialog, then it is gone from the list and from the Files app.

Please keep the `wp1-static` and `wp1-walk-renamed` zips: the next package's
regression test replays one of them (they stay out of git).
