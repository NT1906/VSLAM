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
- In `frames.csv`, report min and max of `fx`. With autofocus off the lens
  does not refocus, but ARKit still reports slightly different intrinsics per
  frame: the static run (1.3) spread about 0.1 px (721.33–721.44). A spread of
  a few pixels or more under motion would mean the focus moved.

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

## WP2: calibration, IMU timing and alignment

No ArUco markers are needed for WP2. It uses only the printed chessboard
(2.1–2.2) and an ordinary textured scene (2.3). Markers come in WP3.

You need: a printer, a ruler with millimetres, tape, something flat and stiff
(cardboard, clipboard, hardcover book), the phone, the Mac with a cable,
about 30 minutes, a well-lit room with some texture (books, posters,
furniture; not a blank wall).

### 2.0 Install the WP2 build

1. Connect the phone to the Mac with the cable, unlock it.
2. In Terminal: `cd ~/Documents/IE415/VSLAM && ~/development/flutter/bin/flutter run --release`.
   The first build compiles OpenCV and takes several minutes.
3. The app opens on the phone. Expected: the home screen as in WP1, plus a new
   grid icon (Calibration) in the top bar. The camera shows "SLAM tracking"
   after a short pan.

### 2.1 Print and measure the board

1. Open `docs/calibration_board.svg` on the Mac (double-click opens it in a
   browser or Preview).
2. Print: paper A4 or Letter, orientation **landscape**, scale **100 %**
   (in the print dialog turn off "Fit to page" / "Scale to fit"; choose
   "Actual size"). One copy.
3. Check the print: 10 × 7 black/white squares, a line with "100 mm" below.
4. With the ruler, measure:
   - the ruler line (expected 100 mm; if it is off by more than 1 mm, the
     printer scaled it: print again with the scaling turned off);
   - one full row of 10 squares, edge to edge, and divide by 10. This is the
     square side (expected about 25.0 mm). Measuring 10 squares is more
     accurate than measuring one.
5. Glue or tape the sheet completely flat onto the stiff backing, with no
   bubbles or bent corners. A curved board gives a wrong calibration.

Report: ruler line mm, 10-square length mm, square side mm.

### 2.2 Calibrate the camera

1. Put the board flat on a table, or lean it against a wall, in even light
   with no glare on it.
2. In the app tap the grid icon. A panel opens at the bottom; the camera view
   stays visible above it.
3. Type the square side from 2.1 in mm (e.g. `24.9`) in "Measured square
   side". It can only be changed before the first capture.
4. Capture views. For each: hold the phone so the **whole** board (all 10 × 7
   squares) is in the camera view, hold still for a second, tap **Capture**.
   - "View n: 54 corners" means it was accepted.
   - "Board not found" means some corners were out of view, blurred or had
     glare: move a little and capture again.
   - Undo removes the last view.
5. Take 18 views (between 15 and 20; Capture stops at 20), varying them:
   - 3 head-on: board centred, filling about half the image, at about 30, 45
     and 60 cm;
   - 4 tilted about 30–45°: phone moved left, right, up, down while still
     aiming at the board;
   - 6 with the board near each corner and edge of the camera image (top-left,
     top-right, bottom-left, bottom-right, left edge, right edge), board fully
     visible;
   - 3 with the phone rolled (turned about the lens axis) about 30–90°;
   - 2 close, board filling most of the image.
6. Coverage should read 80 % or more. If it is lower, add views with the board
   near the parts of the image not yet used.
7. Tap **Calibrate** (enabled from 15 views). It takes a few seconds.
8. Expected:
   - "Calibrated and saved to calibration.json", and the table: OpenCV and
     ARKit columns for fx, fy, cx, cy and the difference; k1, k2.
   - RMS under 0.5 px (under 0.3 px is good). Above 1 px means a bent board,
     motion blur or a wrong square size: redo.
   - fx, fy within about 1–2 % of ARKit's (ARKit gave ≈ 721 in WP1); cx, cy
     within a few pixels of ARKit's.
   - k1 and k2 small (|k1| below about 0.3).
9. Close the panel and open it again: the saved result is shown.
10. Take a screenshot of the table.
11. Copy `calibration.json` to the Mac: Finder → the phone → Files → VSLAM →
    drag `calibration.json` out.

Report: the screenshot (or all numbers: views, RMS, fx, fy, cx, cy, k1, k2 and
the ARKit values), coverage %, and `calibration.json`.

### 2.3 Rotation recording and camera–IMU alignment

1. On the home screen, the settings row: label `wp2-rotation`, experiment
   "Calibration / alignment", marker side unchanged, note "rotation about
   each axis". Save.
2. Stand facing a textured scene 1–3 m away (bookshelf, desk, posters). Wait
   for "SLAM tracking".
3. Tap **Start**, then:
   1. 2 s still.
   2. ~5 s: rotate back and forth about the phone's **long axis** (hold it
      upright and twist it like turning a key, ±30–45°, about 1–2 times per
      second).
   3. ~5 s: rotate about the **short axis** (tip the top of the phone toward
      you and away, like nodding, ±30–45°).
   4. ~5 s: rotate about the **screen normal** (like turning a steering
      wheel, ±30–45°).
   5. 2 s still.
   Keep the camera pointed at the textured scene the whole time and do not
   walk. If the badge leaves "SLAM tracking" for more than a moment, stop,
   delete the session and redo it slower.
4. Tap **Stop**, open the folder icon, open the newest `wp2-rotation` session.
   Expected: about 20 s, poses mostly tracked, gyroscope ≈ 100 Hz.
5. Tap **IMU alignment** (bottom of the screen). A second card appears:
   - Time offset in ms (expected: a few ms to a few tens of ms, either sign);
   - R_ci, a 3 × 3 matrix;
   - Apple axes `0 -1 0 / -1 0 0 / 0 0 -1` and the angle between them and
     R_ci (expected under 2–3°; each entry of R_ci close to the Apple
     matrix);
   - Residual: RMS with the offset clearly lower than "with zero offset";
     the number of pairs (expected several hundred) and mean rate (expected
     above 1 rad/s).
   If it says "Not enough rotation", record again rotating more.
6. Screenshot the card. Tap Export and AirDrop the zip to the Mac; keep it
   (it contains `results/imu_alignment.json`).
7. Optional: a second run with faster swings (mean rate above 1 rad/s)
   resolves the time offset; not needed for the later packages.

Report: the screenshot (or all numbers); keep the zip if convenient.

## WP3: ArUco detection and pose

One short check, about 10 minutes. Everything else in WP3 (corner accuracy,
pose, rejection rules, the detections file, the marker sheet) is tested on
the Mac with markers rendered through the phone's calibrated camera model
(`test/aruco_detector_test.dart`). What only the phone can show: the real
camera, the real print, the calibration in use, and that the outline lands on
the marker in the live view.

You need: one printed marker (page 1 of `docs/markers_4x4_50_ids0-9.pdf`,
ID 0), a ruler with millimetres, a tape measure, tape, a wall, the phone and
the Mac with a cable.

### 3.0 Install the WP3 build

1. Connect and unlock the phone.
2. `cd ~/Documents/IE415/VSLAM && ~/development/flutter/bin/flutter run --release`
3. Expected: the home screen as before; at the bottom of the camera view a
   dark line starting "ArUco 4X4_50 · side … cm · calibrated K".
   "calibrated K" means the WP2 calibration was found and is used; "ARKit K"
   would mean it was not.

### 3.1 Print, measure, enter the side

1. Print page 1 only, any scaling (the printer's ≈ 82 % is fine).
2. Measure the black square from its left outer edge to its right outer edge
   in mm (expected about 123 mm at 82 %). Also measure the "100 mm" ruler
   line.
3. Tape the page flat on a wall with the marker centre at about chest height.
4. In the app tap the settings row (above the tiles), set "Marker side (cm)"
   to the measured value (e.g. `12.3`), tap "Save settings".
5. Expected: the status line now reads "side 12.3 cm" (your value).

Report: marker side mm, ruler line mm.

### 3.2 Distance at 1 m and 2 m, IDs live

1. Lay the tape measure on the floor straight out from the wall below the
   marker.
2. Stand so that the **back of the phone** is above the 1.00 m mark, phone
   upright (portrait), back camera facing the marker, lens roughly level with
   the marker centre. Hold still for 2 s.
3. Expected on screen: a coloured outline along the edges of the black
   square, a red dot on its top-left corner (the corner on the left under
   "TOP"), and a label "ID 0  0.xxx m" or "ID 0  1.xxx m". Head-on, the label
   may be orange with "ambiguous": that is the planar flip the spec rejects
   for initialization; the distance is still valid. Within 3 %: 0.970 to
   1.030 m.
4. Screenshot (side button + volume up).
5. Repeat at the 2.00 m mark. Within 3 %: 1.940 to 2.060 m.

Report: the two screenshots (or the two distances, the outline position and
the status line).

If a distance is off by more than 3 %: first check the side entered in the
settings (distance scales with it: 15.0 entered for a 12.3 cm print reads
22 % too far).

## WP4: keyframes, baselines, first map

One recording of about 90 s and a few button presses, about 20 minutes with
printing. Keyframe rules, IMU propagation, both baselines, marker placement
and the map files are tested on the Mac (`test/keyframe_test.dart`,
`test/imu_propagator_test.dart`, `test/baselines_sim_test.dart`,
`test/marker_map_test.dart`, `test/baselines_ui_test.dart`), largely on a
simulated room walk. What only the phone can show: real ARKit drift, the real
IMU at rest and walking, and markers placed from real detections. This
recording is also the first real dataset for WP5, so it is exported to the
Mac.

You need: markers ID 0–4 printed at the **same scaling** (ID 0 from WP3 was
100 %, side 150 mm), tape, a room with texture (furniture, posters,
bookshelves; not a plain wall: ARKit stays "Relocalizing" there), a tape
measure, the phone and the Mac with a cable.

### 4.0 Install the WP4 build

With the phone connected and unlocked, the build is installed from the Mac
(`flutter run --release`). Expected: home screen as before; status line
"ArUco 4X4_50 · side 15.0 cm · calibrated K".

### 4.1 Markers on the walls

1. Print pages 2–5 of `docs/markers_4x4_50_ids0-9.pdf` (IDs 1–4) at 100 %.
   Check one: black square 150 mm.
2. Tape IDs 0–4 flat on the walls around the room at chest height, about
   1–2 m apart, on at least two different walls, each near some texture.
3. Measure with the tape the distance between the centres of ID 0 and ID 1
   (straight line, cm).
4. Settings row: label `wp4-room`, experiment "Closed loop", marker side
   `15.0`, Save settings.

Report: the ID 0–ID 1 distance.

### 4.2 Record

1. Stand in the middle of the room, phone in portrait pointing at a textured
   area. Press Start.
2. Hold the phone still (elbows against the body) until the badge shows
   "SLAM tracking", then keep it still for 3 more seconds (bias window).
3. Walk slowly (half normal pace) once around the room, pointing the phone at
   each marker in turn from about 1–1.5 m, a bit from the side (not
   head-on), for about 2 s each. Return to where you started. Press Stop.

Expected: the recording ends normally; the session list shows `wp4-room`
with about 60–120 s.

### 4.3 Build map

1. Open the session, press **Build map** (it first runs marker detection
   over every frame, so the progress bar runs for a while).
2. Expected card:
   - Keyframes: some hundreds at most (roughly 4 per metre walked plus
     marker and turn keyframes); "0 tracking gaps" if ARKit never lost
     tracking.
   - IMU rest: "still", 0.0–2.0 s (or a little later) after the first
     keyframe; gyro σ below 0.03, acc σ below 0.15.
   - IMU biases: gyro components of order 0.001–0.01 rad/s; accelerometer of
     order 0.01–0.2 m/s².
   - IMU vs ARKit: a few cm at 1–2 s, growing to metres within tens of
     seconds (dead reckoning is expected to diverge).
   - Markers: IDs 0–4 each with a position; the z values similar to one
     another (all taped at chest height); nothing under "Not placed".
3. Screenshot the card (scroll to show all of it if needed).

Report: screenshot(s). The ID 0–ID 1 distance in the map is compared with the
tape measure of 4.1 on the Mac.

### 4.4 Save map and export

1. Press **Save map**, room name `lab`, Save. Expected: the "Saved" row ends
   with `maps/lab.json`.
2. Press **Export**, AirDrop the zip to the Mac (lands in Downloads).

Report: the Saved row and that the zip arrived.
