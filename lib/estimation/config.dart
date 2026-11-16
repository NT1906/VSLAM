/// Every threshold and noise parameter of the estimation pipeline, in one
/// place (spec section 4.4). Values are starting points; they are tuned on
/// recorded data and the tuned values are reported in `docs/PROGRESS.md`.
///
/// WP3 adds the marker-detection part. Later packages add keyframe, factor
/// and solver parameters to this class.
class SlamConfig {
  const SlamConfig({
    this.minMarkerSidePx = 20,
    this.maxViewAngleDeg = 65,
    this.maxReprojectionRmsPx = 2.0,
    this.ippeAmbiguityRatio = 2.0,
    this.cornerSigmaPx = 0.5,
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

  /// Ambiguity threshold on the second solution's RMS for a best RMS of
  /// [bestRms].
  double ambiguityThresholdPx(double bestRms) =>
      ippeAmbiguityRatio * (bestRms > cornerSigmaPx ? bestRms : cornerSigmaPx);
}
