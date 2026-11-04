import 'dart:math' as math;

import 'package:vector_math/vector_math_64.dart';

/// A rigid-body transform `aTb`: the pose of frame b expressed in frame a.
/// It maps points from b into a: `p_a = R · p_b + t`.
///
/// Composition follows the frame names: `aTb * bTc = aTc`.
class Pose3 {
  Pose3(this.rotation, this.translation);

  Pose3.identity()
      : rotation = Matrix3.identity(),
        translation = Vector3.zero();

  factory Pose3.fromQuaternion(Quaternion q, Vector3 t) =>
      Pose3((q.normalized()).asRotationMatrix(), t.clone());

  /// Yaw [yaw] about +z followed by translation [t] (gravity-aligned frames).
  factory Pose3.yawTranslation(double yaw, Vector3 t) => Pose3(rotZ(yaw), t.clone());

  final Matrix3 rotation;
  final Vector3 translation;

  Pose3 operator *(Pose3 other) => Pose3(
        rotation.multiplied(other.rotation),
        rotation.transformed(other.translation)..add(translation),
      );

  Pose3 inverse() {
    final rt = rotation.transposed();
    return Pose3(rt, -rt.transformed(translation));
  }

  Vector3 transform(Vector3 p) => rotation.transformed(p)..add(translation);

  Quaternion get quaternion => Quaternion.fromRotation(rotation)..normalize();

  Pose3 clone() => Pose3(rotation.clone(), translation.clone());

  /// Right perturbation used by the optimisers, in the child frame:
  /// `R ← R·Exp(δθ)`, `t ← t + R·δt`. [delta] holds `[δt; δθ]` from [offset].
  ///
  /// With this choice the error `Z⁻¹·retract(Z, δ)` is `[δt; δθ]` to first
  /// order, so a covariance estimated in δ is directly the covariance of the
  /// residuals built by [poseError].
  Pose3 retract(List<double> delta, [int offset = 0]) => Pose3(
        rotation.multiplied(so3Exp(Vector3(delta[offset + 3], delta[offset + 4], delta[offset + 5]))),
        translation + rotation.transformed(Vector3(delta[offset], delta[offset + 1], delta[offset + 2])),
      );

  Map<String, dynamic> toJson() {
    final q = quaternion;
    return {
      't': [translation.x, translation.y, translation.z],
      'q': [q.x, q.y, q.z, q.w],
    };
  }

  factory Pose3.fromJson(Map<String, dynamic> json) {
    final t = (json['t'] as List).map((e) => (e as num).toDouble()).toList();
    final q = (json['q'] as List).map((e) => (e as num).toDouble()).toList();
    return Pose3.fromQuaternion(Quaternion(q[0], q[1], q[2], q[3]), Vector3(t[0], t[1], t[2]));
  }

  @override
  String toString() => 'Pose3(t: $translation, rpy: ${rollPitchYaw(rotation)})';
}

/// Rotation of [angle] radians about +z.
Matrix3 rotZ(double angle) => Matrix3.rotationZ(angle);

/// SO(3) exponential map (Rodrigues).
Matrix3 so3Exp(Vector3 w) {
  final theta = w.length;
  if (theta < 1e-12) {
    return Matrix3(1, w.z, -w.y, -w.z, 1, w.x, w.y, -w.x, 1); // I + [w]×, column-major
  }
  final k = w / theta;
  final c = math.cos(theta);
  final s = math.sin(theta);
  final v = 1 - c;
  // Column-major constructor.
  return Matrix3(
    c + k.x * k.x * v, k.y * k.x * v + k.z * s, k.z * k.x * v - k.y * s,
    k.x * k.y * v - k.z * s, c + k.y * k.y * v, k.z * k.y * v + k.x * s,
    k.x * k.z * v + k.y * s, k.y * k.z * v - k.x * s, c + k.z * k.z * v,
  );
}

/// SO(3) logarithm: the rotation vector (axis · angle, |angle| ≤ π).
Vector3 so3Log(Matrix3 r) {
  final q = Quaternion.fromRotation(r)..normalize();
  var w = q.w;
  var v = Vector3(q.x, q.y, q.z);
  if (w < 0) {
    w = -w;
    v = -v;
  }
  final s = v.length;
  if (s < 1e-12) return v * 2.0;
  final angle = 2 * math.atan2(s, w);
  return v * (angle / s);
}

/// Rotation angle of [r] in radians, in [0, π].
double rotationAngle(Matrix3 r) => so3Log(r).length;

/// Residual of an error transform: `[t; Log(R)]`.
List<double> poseError(Pose3 e) {
  final w = so3Log(e.rotation);
  return [e.translation.x, e.translation.y, e.translation.z, w.x, w.y, w.z];
}

/// ZYX Euler angles (roll, pitch, yaw) of [r], radians.
Vector3 rollPitchYaw(Matrix3 r) {
  final yaw = math.atan2(r.entry(1, 0), r.entry(0, 0));
  final pitch = math.asin((-r.entry(2, 0)).clamp(-1.0, 1.0));
  final roll = math.atan2(r.entry(2, 1), r.entry(2, 2));
  return Vector3(roll, pitch, yaw);
}

/// Heading (radians CCW from +x) of the horizontal projection of the
/// rotated [axis], or null when that projection is too short.
double? headingOfAxis(Matrix3 r, Vector3 axis) {
  final v = r.transformed(axis);
  final h = math.sqrt(v.x * v.x + v.y * v.y);
  if (h < 0.2) return null;
  return math.atan2(v.y, v.x);
}

/// Projects a transform between two gravity-aligned frames onto yaw and
/// translation, discarding the (noise-only) roll and pitch.
Pose3 projectToYaw(Pose3 t) {
  final yaw = headingOfAxis(t.rotation, Vector3(1, 0, 0)) ??
      (headingOfAxis(t.rotation, Vector3(0, 1, 0))! - math.pi / 2);
  return Pose3.yawTranslation(yaw, t.translation);
}
