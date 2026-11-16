import 'dart:math' as math;

import 'package:vector_math/vector_math_64.dart';

import '../core/se3.dart';
import '../estimation/config.dart';
import 'camera_model.dart';

/// Why a detected marker is not used for pose initialization (spec 4.4).
/// Checked in this order; the first failing rule is recorded.
enum MarkerRejection {
  /// Shortest image edge under [SlamConfig.minMarkerSidePx].
  tooSmall('small'),

  /// PnP returned no solution.
  noPose('no_pose'),

  /// Best pose reprojects the corners worse than
  /// [SlamConfig.maxReprojectionRmsPx].
  highReprojection('reproj'),

  /// Viewing angle from the marker normal above [SlamConfig.maxViewAngleDeg].
  steepView('angle'),

  /// The two IPPE solutions explain the corners about equally well.
  ambiguous('ambiguous');

  const MarkerRejection(this.code);

  /// Value of the `status` column in `detections.csv`.
  final String code;

  static MarkerRejection? fromCode(String code) {
    if (code == acceptedCode) return null;
    return values.firstWhere((r) => r.code == code, orElse: () => throw FormatException('Unknown status "$code"'));
  }

  static const acceptedCode = 'ok';
}

/// One PnP solution: the marker pose in the camera frame (`cTm`, OpenCV
/// camera, marker frame of spec 4.3) and its corner reprojection RMS.
class PnpSolution {
  const PnpSolution(this.cTm, this.rmsPx);

  final Pose3 cTm;
  final double rmsPx;
}

/// Marker-frame corners for side [sideM] in OpenCV's detection order
/// (top-left, top-right, bottom-right, bottom-left). The marker frame has x
/// right, y up and z out of the printed face toward the viewer, which is
/// also the point order `SOLVEPNP_IPPE_SQUARE` requires.
List<Vector3> markerObjectPoints(double sideM) {
  final h = sideM / 2;
  return [Vector3(-h, h, 0), Vector3(h, h, 0), Vector3(h, -h, 0), Vector3(-h, -h, 0)];
}

/// RMS over the four corners of the distance between the detected corner
/// and the projection of the marker corner through [cTm] and [k]
/// (√(Σ|e_j|² / 4), the same definition `cv::solvePnPGeneric` uses for its
/// `reprojectionError` output). Infinite when a corner is behind the camera.
double reprojectionRms(Pose3 cTm, List<Vector2> corners, CameraIntrinsics k, double sideM) {
  final obj = markerObjectPoints(sideM);
  var sum = 0.0;
  for (var j = 0; j < 4; j++) {
    final p = k.project(cTm.transform(obj[j]));
    if (p == null) return double.infinity;
    sum += (p - corners[j]).length2;
  }
  return math.sqrt(sum / 4);
}

/// A detected ArUco marker in one image, with its pose candidates and the
/// outcome of the rejection rules of spec section 4.4.
class MarkerObservation {
  const MarkerObservation({
    required this.id,
    required this.corners,
    required this.solutions,
    required this.sidePx,
    required this.viewAngleDeg,
    required this.rejection,
  });

  /// Applies the rejection rules to a detection. [solutions] are the PnP
  /// candidates in any order; they are sorted by reprojection RMS here.
  factory MarkerObservation.evaluate({
    required int id,
    required List<Vector2> corners,
    required List<PnpSolution> solutions,
    SlamConfig config = const SlamConfig(),
  }) {
    final sorted = [...solutions]..sort((a, b) => a.rmsPx.compareTo(b.rmsPx));
    final side = minEdgePx(corners);
    final angle = sorted.isEmpty ? double.nan : viewAngleOf(sorted.first.cTm);
    MarkerRejection? rejection;
    if (side < config.minMarkerSidePx) {
      rejection = MarkerRejection.tooSmall;
    } else if (sorted.isEmpty) {
      rejection = MarkerRejection.noPose;
    } else if (sorted.first.rmsPx > config.maxReprojectionRmsPx) {
      rejection = MarkerRejection.highReprojection;
    } else if (angle > config.maxViewAngleDeg) {
      rejection = MarkerRejection.steepView;
    } else if (sorted.length > 1 && sorted[1].rmsPx < config.ambiguityThresholdPx(sorted.first.rmsPx)) {
      rejection = MarkerRejection.ambiguous;
    }
    return MarkerObservation(
      id: id,
      corners: corners,
      solutions: sorted,
      sidePx: side,
      viewAngleDeg: angle,
      rejection: rejection,
    );
  }

  /// Dictionary ID (DICT_4X4_50: 0–49).
  final int id;

  /// Sub-pixel corners in image pixels (sensor orientation), in the order
  /// top-left, top-right, bottom-right, bottom-left of the printed marker.
  final List<Vector2> corners;

  /// PnP candidates, best (lowest reprojection RMS) first. IPPE gives two.
  final List<PnpSolution> solutions;

  /// Shortest of the four image edges, pixels.
  final double sidePx;

  /// Angle between the marker normal and the direction to the camera for
  /// the best solution, degrees (0 = seen head-on).
  final double viewAngleDeg;

  /// Null when the detection passed every rule.
  final MarkerRejection? rejection;

  bool get accepted => rejection == null;

  PnpSolution? get best => solutions.isEmpty ? null : solutions.first;
  PnpSolution? get alternative => solutions.length > 1 ? solutions[1] : null;

  /// `cTm` of the best solution.
  Pose3? get cTm => best?.cTm;

  /// Distance from the camera centre to the marker centre, metres.
  double? get distanceM => best?.cTm.translation.length;

  String get statusCode => rejection?.code ?? MarkerRejection.acceptedCode;

  /// Shortest edge of the quadrilateral [c], pixels.
  static double minEdgePx(List<Vector2> c) {
    var m = double.infinity;
    for (var j = 0; j < 4; j++) {
      m = math.min(m, (c[(j + 1) % 4] - c[j]).length);
    }
    return m;
  }

  /// Viewing angle of a marker at `cTm`: the angle between the marker's +z
  /// axis (out of the face) and the vector from the marker centre to the
  /// camera centre, both in camera coordinates. 0° when head-on; above 90°
  /// the camera would be behind the marker.
  static double viewAngleOf(Pose3 cTm) {
    final normal = cTm.rotation.transformed(Vector3(0, 0, 1));
    final toCamera = -cTm.translation;
    final c = normal.dot(toCamera) / toCamera.length;
    return math.acos(c.clamp(-1.0, 1.0)) * 180 / math.pi;
  }

  // ---- results/detections.csv -------------------------------------------

  static const csvHeader = 't_us,id,status,u0,v0,u1,v1,u2,v2,u3,v3,side_px,view_deg,'
      'rms_px,tx,ty,tz,qx,qy,qz,qw,alt_rms_px,alt_tx,alt_ty,alt_tz,alt_qx,alt_qy,alt_qz,alt_qw';

  /// One `detections.csv` row; poses are `cTm` (translation in metres,
  /// quaternion x, y, z, w). Missing solutions leave their fields empty.
  String toCsvRow(int tUs) {
    List<Object> sol(PnpSolution? s) {
      if (s == null) return const ['', '', '', '', '', '', '', ''];
      final t = s.cTm.translation;
      final q = s.cTm.quaternion;
      return [s.rmsPx, t.x, t.y, t.z, q.x, q.y, q.z, q.w];
    }

    return [
      tUs,
      id,
      statusCode,
      for (final c in corners) ...[c.x, c.y],
      sidePx,
      viewAngleDeg.isNaN ? '' : viewAngleDeg,
      ...sol(best),
      ...sol(alternative),
    ].join(',');
  }

  /// Parses a [toCsvRow] row; returns the frame time and the observation.
  static (int, MarkerObservation) fromCsvRow(String line) {
    final f = line.split(',');
    if (f.length != csvHeader.split(',').length) {
      throw FormatException('detections.csv: expected ${csvHeader.split(',').length} fields, got ${f.length}');
    }
    double d(int i) => double.parse(f[i]);
    PnpSolution? sol(int at) {
      if (f[at].isEmpty) return null;
      final q = Quaternion(d(at + 4), d(at + 5), d(at + 6), d(at + 7));
      return PnpSolution(Pose3.fromQuaternion(q, Vector3(d(at + 1), d(at + 2), d(at + 3))), d(at));
    }

    return (
      int.parse(f[0]),
      MarkerObservation(
        id: int.parse(f[1]),
        corners: [for (var j = 0; j < 4; j++) Vector2(d(3 + 2 * j), d(4 + 2 * j))],
        solutions: [?sol(13), ?sol(21)],
        sidePx: d(11),
        viewAngleDeg: f[12].isEmpty ? double.nan : d(12),
        rejection: MarkerRejection.fromCode(f[2]),
      ),
    );
  }
}
