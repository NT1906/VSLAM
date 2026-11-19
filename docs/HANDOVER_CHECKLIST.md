# Handover checklist: what to give your teammate

Everything the teammate needs that is **not** already in the GitHub
repository, plus the things to confirm with him. Tick each item. The
technical plan for the remaining work is in `docs/HANDOFF.md` (and
`HANDOFF.pdf`); this list is only about handing things over.

## 1. Already on GitHub (just point him to it)

- [ ] Repository `github.com/NT1906/VSLAM`, branch **`iphone-slam`** (all
      work is there; `main` is the old mid-evaluation snapshot). Check that
      he has access to the repository (collaborator or owner).
- [ ] He checks it out: `git clone https://github.com/NT1906/VSLAM.git`,
      then `git checkout iphone-slam`.
- [ ] In the repo, for him to read first: `docs/HANDOFF.md`, `docs/SPEC.md`,
      `docs/PROGRESS.md`, `docs/DESIGN.md`, `docs/DEVICE_CHECKS.md`.
- [ ] Print files are in the repo too: `docs/markers_4x4_50_ids0-9.pdf`
      (markers) and `docs/calibration_board.svg` (chessboard).

## 2. Files that are only on your phone (copy them to him)

The app saves everything under its Documents folder on the iPhone, which
shows in the Files app as **On My iPhone → VSLAM** (and in Finder when the
phone is connected: the phone → Files → VSLAM).

- [ ] **`calibration.json`** (top of the VSLAM folder): the camera
      calibration from WP2 (RMS 0.467 px). It belongs to **your** iPhone
      15's camera. Give it to him if he will keep using your phone; it is
      also the record of the WP2 result.
- [ ] **`sessions/`**: the recordings so far. Easiest: in the app, open each
      session → **Export** → AirDrop or save the zip, then send the zips
      (Google Drive, OneDrive or similar: they are too big for git). The ones
      that matter:
  - `2026-11-12_09-01-42_wp1-static` (60 s, phone still on a table: IMU
    noise and bias reference)
  - `2026-11-12_09-05-51_wp1-walk-renamed` (60 s hand-held walk, 5.7 m)
  - `2026-11-16_14-26-21_wp2-rotation` (27 s rotation run for the
    camera–IMU alignment)
  - any WP3 marker sessions you kept.
  The WP1 zips exported on 2026-11-12 were **not** found on the Mac on
  2026-11-19, so export them again from the phone if he needs them.
- [ ] `recording_settings.json` (same folder): not needed; the app
      recreates it.

## 3. Things that are physical

- [ ] **Printed marker ID 0** (printed at 100 %, black square 150 mm).
- [ ] **Printed chessboard** (printed at about 82 %: square 20.6 mm, not
      25 mm). Only needed if the camera is calibrated again.
- [ ] Markers **IDs 1–4 are not printed yet** (pages 2–5 of the marker
      PDF, at 100 %; WP4 check step 4.1). IDs 5–9 will be needed for the
      experiments (6–10 markers in the room).
- [ ] **The phone.** Decide whether he uses your iPhone 15 or his own (see
      section 5).
- [ ] Tape measure (all device checks and the ground truth for WP6–WP8
      need one).

## 4. Accounts and setup he needs himself

- [ ] A Mac with Xcode (the project used Xcode 27.2), Flutter 3.47.9
      (Dart 3.13.7) and CMake (`brew install cmake`). Setup notes:
      `HANDOFF.md` section 2.
- [ ] His own Apple ID signed in to Xcode, with his team chosen under
      Signing for the Runner target (a free Apple ID works; apps then expire
      after 7 days and are reinstalled with `flutter run --release`).
- [ ] On the phone: Developer Mode on, and his Mac trusted (cable the first
      time).

## 5. Decisions to agree with him

- [ ] **Which phone.** On **your** iPhone 15 nothing needs redoing (with
      `calibration.json` in place). On a **different** iPhone he must redo
      the WP2 calibration (calibration sheet, 15–20 views, check 2.2) and
      the WP3 distance check (1 m and 2 m) before recording, because the
      intrinsics and distortion belong to one camera. Other iPhone models
      may also give a different image size than 1920 × 1440 → 960 × 720.
- [ ] **Where recordings are shared** (a shared drive folder), since they
      stay out of git.
- [ ] **Which room** the experiments use (textured; markers stay up for the
      repeat-run experiment; it must be tape-measured for ground truth in
      WP6).
- [ ] **Who finishes the WP4 device check** (steps 4.1–4.4 in
      `DEVICE_CHECKS.md`; step 4.0 is done on your phone).

## 6. Things to tell him that are not in the documents

- [ ] Anything you know about how the work is split or graded (who
      presents, the deadline, the report format).
- [ ] The spec refers to `tools/reference_sim.py` (the Python simulation
      behind the spec's reference numbers). It is **not in the repository**.
      If you or someone in the team has it, give it to him; otherwise he
      relies on the table in `SPEC.md` 4.5.
- [ ] The project was built with an AI coding assistant following
      `docs/SPEC.md`. `PROGRESS.md` records every check and decision, so he
      can continue the same way or by hand.
