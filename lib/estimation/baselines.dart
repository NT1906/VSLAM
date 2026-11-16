import 'package:vector_math/vector_math_64.dart';

import '../core/se3.dart';
import 'config.dart';
import 'imu_propagator.dart';
import 'keyframe_selector.dart';

/// `wTv`: the map frame W of mapping mode (spec 4.3) seen from the ARKit
/// session frame V, given the first keyframe's pose `vTc_0`.
///
/// W is the gravity-aligned frame of the first keyframe: origin at its
/// camera centre, z up (V is already gravity aligned), x along the
/// horizontal projection of the camera's viewing direction (OpenCV +z).
/// When the camera looks almost straight up or down, x follows the
/// horizontal projection of the image's "up" direction (OpenCV −y) instead.
/// W and V therefore differ only by a yaw and a translation.
Pose3 worldFromVio(Pose3 vTc0) {
  final yaw = headingOfAxis(vTc0.rotation, Vector3(0, 0, 1)) ?? headingOfAxis(vTc0.rotation, Vector3(0, -1, 0)) ?? 0;
  return Pose3.yawTranslation(yaw, vTc0.translation).inverse();
}

/// Baseline 1 (spec 4.6): the ARKit-only trajectory, `X_0 = wTv · vTc_0`
/// and `X_k+1 = X_k · Z_k` with the odometry of each edge. Algebraically
/// this is `wTv · vTc_k` for every keyframe; it is computed by chaining
/// because that is exactly the initial guess and the odometry-only
/// solution of the pose graph in WP5.
List<Pose3> chainOdometry(Pose3 x0, List<OdometryEdge> edges) {
  final out = [x0];
  for (final e in edges) {
    out.add(out.last * e.z);
  }
  return out;
}

/// Result of baseline 2, IMU-only dead reckoning.
class ImuBaseline {
  const ImuBaseline({
    required this.rest,
    required this.restFound,
    required this.start,
    required this.states,
    required this.cameraPoses,
  });

  /// The window the biases came from.
  final RestEstimate rest;

  /// False when no still window was found; the propagation then starts at
  /// the first keyframe with zero biases and an assumed zero velocity.
  final bool restFound;

  /// State at the start of the propagation (end of the rest window).
  final ImuState start;

  /// State at each keyframe time, null for keyframes before [start].
  final List<ImuState?> states;

  /// Camera pose `wTc` at each keyframe implied by [states] (null before
  /// the start), for direct comparison with baseline 1.
  final List<Pose3?> cameraPoses;
}

/// Baseline 2 (spec 4.6): dead reckoning from raw IMU samples.
///
/// 1. Find the first window of `imuRestSeconds` at or after the first
///    keyframe in which the phone is still ([findRestWindow]).
/// 2. Estimate `b_g` and `b_a` there ([estimateRestBiases]), with the
///    device orientation from ARKit at that time.
/// 3. Start at the end of the window from ARKit's pose with v = 0 and
///    propagate ([ImuPropagator]) through every later keyframe time.
///
/// [poseAt] gives ARKit's camera pose `wTc` (already in W) nearest to a
/// time. [rCI] is the camera–IMU rotation (`R_ci`, WP2). The IMU is placed
/// at the camera centre: the lever arm of a few centimetres is far below
/// the drift of dead reckoning. The time offset is taken as 0 (WP2).
ImuBaseline runImuBaseline({
  required ImuSeries acc,
  required ImuSeries gyro,
  required List<Keyframe> keyframes,
  required Pose3 Function(int tUs) poseAt,
  required Matrix3 rCI,
  SlamConfig config = const SlamConfig(),
}) {
  final first = keyframes.first.tUs;
  final found = findRestWindow(acc, gyro, first, config);
  final t0 = found ?? first;
  final restLen = (config.imuRestSeconds * 1e6).round();
  // Orientation during the rest window: from the ARKit pose in its middle.
  final rWbRest = poseAt(t0 + restLen ~/ 2).rotation.multiplied(rCI);
  final rest = estimateRestBiases(acc, gyro, t0, rWbRest, config);
  final startUs = found == null ? first : t0 + restLen;
  final wTcStart = poseAt(startUs);
  final start = ImuState(
    tUs: startUs,
    p: wTcStart.translation.clone(),
    v: Vector3.zero(),
    q: Quaternion.fromRotation(wTcStart.rotation.multiplied(rCI))..normalize(),
    ba: rest.ba.clone(),
    bg: rest.bg.clone(),
  );
  final propagator = ImuPropagator(acc: acc, gyro: gyro, gravity: config.gravity);
  final s = start.copy();
  final states = <ImuState?>[];
  final poses = <Pose3?>[];
  final rIC = rCI.transposed();
  for (final kf in keyframes) {
    if (kf.tUs < startUs) {
      states.add(null);
      poses.add(null);
      continue;
    }
    propagator.propagate(s, kf.tUs);
    states.add(s.copy());
    poses.add(Pose3(s.rWb.multiplied(rIC), s.p.clone()));
  }
  return ImuBaseline(rest: rest, restFound: found != null, start: start, states: states, cameraPoses: poses);
}

/// Time-sorted ARKit poses (any source) with a nearest-in-time lookup.
class TimedPoses {
  TimedPoses(List<(int, Pose3)> poses) : _poses = [...poses]..sort((a, b) => a.$1.compareTo(b.$1));

  final List<(int, Pose3)> _poses;

  bool get isEmpty => _poses.isEmpty;

  /// The pose with the time nearest to [tUs].
  Pose3 nearest(int tUs) {
    if (_poses.isEmpty) throw StateError('no poses');
    var lo = 0;
    var hi = _poses.length - 1;
    while (hi - lo > 1) {
      final mid = (lo + hi) >> 1;
      if (_poses[mid].$1 <= tUs) {
        lo = mid;
      } else {
        hi = mid;
      }
    }
    final a = _poses[lo];
    final b = _poses[hi];
    return ((tUs - a.$1).abs() <= (b.$1 - tUs).abs() ? a : b).$2;
  }
}

/// Distance between IMU and ARKit positions at the first keyframe at least
/// [horizonsS] seconds after the IMU start: `(elapsed s, metres)` for each
/// horizon reached, then the last keyframe.
List<(double, double)> imuDrift(List<Keyframe> keyframes, List<Pose3> arkit, ImuBaseline imu,
    {List<double> horizonsS = const [1, 2, 5, 10, 20, 30, 60]}) {
  final out = <(double, double)>[];
  final startUs = imu.start.tUs;
  int? lastUsed;
  for (final h in horizonsS) {
    for (var k = 0; k < keyframes.length; k++) {
      final pose = imu.cameraPoses[k];
      if (pose == null || keyframes[k].tUs < startUs + h * 1e6) continue;
      if (k != lastUsed) out.add(((keyframes[k].tUs - startUs) / 1e6, (pose.translation - arkit[k].translation).length));
      lastUsed = k;
      break;
    }
  }
  final k = keyframes.length - 1;
  final pose = imu.cameraPoses[k];
  if (pose != null && k != lastUsed) {
    out.add(((keyframes[k].tUs - startUs) / 1e6, (pose.translation - arkit[k].translation).length));
  }
  return out;
}

/// Path length of a keyframe trajectory, metres.
double pathLength(List<Pose3> poses) {
  var sum = 0.0;
  for (var k = 0; k + 1 < poses.length; k++) {
    sum += (poses[k + 1].translation - poses[k].translation).length;
  }
  return sum;
}

