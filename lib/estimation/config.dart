/// Every threshold and noise parameter of the estimation pipeline, in one
/// place (spec section 4.4). Values are starting points; they are tuned on
/// recorded data and the tuned values are reported in `docs/PROGRESS.md`.
///
/// WP3 added the marker-detection part, WP4 the keyframe and IMU
/// dead-reckoning part. Later packages add factor and solver parameters.
class SlamConfig {
  const SlamConfig({
    this.minMarkerSidePx = 20,
    this.maxViewAngleDeg = 65,
    this.maxReprojectionRmsPx = 2.0,
    this.ippeAmbiguityRatio = 2.0,
    this.cornerSigmaPx = 0.5,
    this.keyframeTranslationM = 0.25,
    this.keyframeRotationDeg = 15,
    this.keyframeMarkerGapS = 0.3,
    this.keyframeMarkerStatuses = const {'ok', 'ambiguous'},
    this.gravity = 9.80665,
    this.imuRestSeconds = 2.0,
    this.imuRestSearchSeconds = 15.0,
    this.imuRestMaxGyroStd = 0.03,
    this.imuRestMaxAccStd = 0.15,
  });

  /// A detection is rejected when its shortest image edge is shorter than
  /// this (pixels at the 960 × 720 frame size). Below it the corners carry
  /// too little geometry for a pose.
  final double minMarkerSidePx;

  /// A detection is rejected when the angle between the marker normal and
  /// the ray from the marker centre to the camera exceeds this (degrees).
  /// At grazing angles one marker axis is foreshortened to a few pixels.
  final double maxViewAngleDeg;

  /// A detection is rejected when the best PnP pose reprojects the four
  /// corners with an RMS error above this (pixels): the quad is then not the
  /// image of a flat square (partly occluded, bent paper, false detection).
  final double maxReprojectionRmsPx;

  /// IPPE ambiguity test. A planar square has two pose solutions; the second
  /// is the first one "flipped" about the line of sight. The detection is
  /// rejected as ambiguous when the second solution's reprojection RMS is
  /// below `ippeAmbiguityRatio × max(best RMS, cornerSigmaPx)`: corner noise
  /// of that size could then swap the two. Flooring the best RMS at the
  /// corner noise keeps a near-perfect best fit (synthetic data, RMS ≈ 0.01
  /// px) from making every alternative look "much worse".
  final double ippeAmbiguityRatio;

  /// Standard deviation of a detected corner, pixels at 960 px width
  /// (spec section 4.4). Used here for the ambiguity floor; the marker
  /// factor (WP5) uses it as its measurement noise.
  final double cornerSigmaPx;

  // ---- Keyframes (spec 4.4, WP4) ----------------------------------------

  /// A tracked frame becomes a keyframe when the camera has moved at least
  /// this far (metres) since the previous keyframe.
  final double keyframeTranslationM;

  /// ... or turned at least this much (degrees, angle of the relative
  /// rotation) since the previous keyframe.
  final double keyframeRotationDeg;

  /// ... or it has a usable marker detection and at least this many seconds
  /// have passed since the previous keyframe.
  final double keyframeMarkerGapS;

  /// Detection statuses (`detections.csv`) that count as "a marker
  /// detection" for the keyframe rule. `ok` and `ambiguous` both have good
  /// corners (ambiguous only means the PnP pose is not unique, which the
  /// reprojection factor of WP5 does not need); `small`, `no_pose` and
  /// `reproj` do not, and `angle` (grazing view) gives foreshortened corners.
  final Set<String> keyframeMarkerStatuses;

  // ---- IMU dead reckoning (spec 4.6, baseline 2) ---------------------------

  /// Magnitude of gravity, m/s². The accelerometer bias estimated at rest
  /// absorbs the difference to the local value.
  final double gravity;

  /// Length of the at-rest window the biases are estimated from, seconds.
  final double imuRestSeconds;

  /// The at-rest window is searched from the first keyframe (ARKit
  /// tracking, so the orientation is known) up to this many seconds later.
  final double imuRestSearchSeconds;

  /// A window counts as "at rest" when the largest per-axis standard
  /// deviation of the gyroscope is below this (rad/s) ...
  final double imuRestMaxGyroStd;

  /// ... and that of the accelerometer below this (m/s²). On the iPhone 15
  /// lying on a table both were ≈ 0.001 rad/s and 0.007 m/s² (check 1.3);
  /// the limits leave room for a phone held still by hand, while walking
  /// gives about ten times more.
  final double imuRestMaxAccStd;

  /// Ambiguity threshold on the second solution's RMS for a best RMS of
  /// [bestRms].
  double ambiguityThresholdPx(double bestRms) =>
      ippeAmbiguityRatio * (bestRms > cornerSigmaPx ? bestRms : cornerSigmaPx);
}
