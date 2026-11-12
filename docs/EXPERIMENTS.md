# Experiments

The handout (section 8) requires five experiments in a room with six to ten
ArUco markers. This file covers what the app supports today: recording and
labelling them. Room setup, the marker placement record, ground-truth
measurement and evaluation are added with the packages that build them
(markers WP3, ground truth WP6, metrics WP8).

## Recording an experiment

1. On the main screen, tap the settings row. Set:
   - Label: short and unique, e.g. `loop-02`. It becomes part of the folder
     name.
   - Experiment: one of the five below (or Calibration / Other).
   - Marker side: the measured side of the printed black square, in cm.
   - Notes: lighting, path taken, anything unusual.
2. Wait for "SLAM tracking", press Start, walk the path, press Stop.
3. Open the session from the notice and check: 0 frame gaps, about 10 Hz
   frames and 100 Hz IMU. Re-record if not.

Each recording is saved as `Documents/sessions/<date>_<time>_<label>/`
(layout in `docs/DESIGN.md` section 4.1) and can be exported as a zip.

## The five experiments (handout section 8)

| Experiment type in the app | Handout | What to walk |
| --- | --- | --- |
| Simple path | 1 | straight, L-shaped or rectangular path |
| Marker-loss path | 2 | a stretch where no marker is in view for a measurable time |
| Closed loop | 3 | return to the start region and look at early markers again |
| Repeat run (relocalization) | 4 | a later run in the same room, using a saved map |
| Difficult condition | 5 | poor light, fast motion, oblique views, occlusion or sparse markers; say which in Notes |
