import 'dart:math' as math;

import 'package:vector_math/vector_math_64.dart';

import '../core/rotation.dart';
import '../core/se3.dart';

enum ArTrackingState {
  /// Session running, waiting for the first good pose.
  initializing,

  /// Full 6-DoF tracking.
  tracking,

  /// Tracking degraded or lost (see [ArPose.reason]).
  limited,

  /// Session stopped or failed and will not recover on its own.
  stopped,
}

/// A camera pose reported by ARCore / ARKit.
///
/// Both use a right-handed, gravity-aligned world frame with +y up, and a
/// camera frame that looks down its −z axis.
class ArPose {
  const ArPose({
    required this.state,
    this.translation,
    this.rotation,
    this.reason,
    this.time,
  });

  final ArTrackingState state;

  /// Capture time on the shared sensor clock, when the platform sends it.
  final DateTime? time;
  final Vector3? translation;
  final Quaternion? rotation;

  /// Human-readable reason for [ArTrackingState.limited] / `stopped`.
  final String? reason;

  bool get hasPose => translation != null && rotation != null;

  /// Parses an event from the native `vslam/ar_pose` channel:
  /// `{state, reason?, ts?, t: [x,y,z]?, q: [x,y,z,w]?}`, with `ts` in
  /// microseconds since the epoch.
  factory ArPose.fromMap(Map<Object?, Object?> map) {
    final state = switch (map['state']) {
      'tracking' => ArTrackingState.tracking,
      'initializing' => ArTrackingState.initializing,
      'limited' => ArTrackingState.limited,
      _ => ArTrackingState.stopped,
    };
    final t = (map['t'] as List?)?.map((e) => (e as num).toDouble()).toList();
    final q = (map['q'] as List?)?.map((e) => (e as num).toDouble()).toList();
    final ts = map['ts'] as num?;
    return ArPose(
      state: state,
      time: ts == null ? null : DateTime.fromMicrosecondsSinceEpoch(ts.round()),
      reason: map['reason'] as String?,
      translation: t != null && t.length == 3 ? Vector3(t[0], t[1], t[2]) : null,
      rotation: q != null && q.length == 4 ? Quaternion(q[0], q[1], q[2], q[3]) : null,
    );
  }
}

/// AR world (y up, −z forward) → session frame (z up, +y forward).
Vector3 arToSession(Vector3 v) => Vector3(v.x, -v.z, v.y);

/// Re-expresses AR poses in the session frame shared with dead reckoning:
/// origin at the anchor pose, +y along the camera's horizontal view
/// direction at the anchor.
class SlamAligner {
  Vector3? _origin;
  double _yawOffset = 0;

  bool get isAnchored => _origin != null;

  void reset() => _origin = null;

  void anchor(Vector3 translation, Quaternion rotation) {
    _origin = arToSession(translation);
    final forward = arToSession(rotate(rotation, Vector3(0, 0, -1)));
    final yaw = horizontalHeading(forward, minLength: 0.2) ?? math.pi / 2;
    _yawOffset = math.pi / 2 - yaw;
  }

  Vector3 toSession(Vector3 translation) {
    final origin = _origin;
    if (origin == null) {
      throw StateError('SlamAligner.toSession called before anchor()');
    }
    return rotateAboutZ(arToSession(translation) - origin, _yawOffset);
  }
}

/// AR world (y up) → VIO world used by the estimator (z up).
final Matrix3 _arWorldToVio = Matrix3.columns(Vector3(1, 0, 0), Vector3(0, 0, 1), Vector3(0, -1, 0));

/// AR camera (x right, y up, z back) ← OpenCV camera (x right, y down, z
/// forward).
final Matrix3 _cvToArCamera = Matrix3(1, 0, 0, 0, -1, 0, 0, 0, -1);

/// Converts a native AR camera pose (ARCore `Camera.getPose()`, ARKit
/// `camera.transform`) into `vTc`: the OpenCV-convention camera in the
/// gravity-aligned, z-up VIO world. Both SDKs report the camera relative to
/// the sensor image (landscape), the same image the marker detector sees.
Pose3 arPoseToVio(Vector3 translation, Quaternion rotation) => Pose3(
      _arWorldToVio.multiplied(rotation.normalized().asRotationMatrix()).multiplied(_cvToArCamera),
      _arWorldToVio.transformed(translation),
    );

/// Inverse of [arPoseToVio]: the native AR pose for a `vTc`.
({Vector3 translation, Quaternion rotation}) vioToArPose(Pose3 vTc) {
  final m = _arWorldToVio.transposed();
  return (
    translation: m.transformed(vTc.translation),
    rotation: Quaternion.fromRotation(m.multiplied(vTc.rotation).multiplied(_cvToArCamera))..normalize(),
  );
}
