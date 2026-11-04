import 'dart:math' as math;

import 'package:vector_math/vector_math_64.dart';

/// Small, explicit rotation helpers.
///
/// All quaternions here use the Hamilton convention and represent a
/// body-to-world rotation: `rotate(q, v)` maps a vector expressed in the
/// device (body) frame into the world frame.
final Vector3 worldUp = Vector3(0, 0, 1);

/// Rotates [v] by unit quaternion [q] (computes q ⊗ v ⊗ q*).
Vector3 rotate(Quaternion q, Vector3 v) {
  final p = q * Quaternion(v.x, v.y, v.z, 0) * q.conjugated();
  return Vector3(p.x, p.y, p.z);
}

/// Quaternion for a rotation of [angle] radians about unit [axis].
Quaternion axisAngle(Vector3 axis, double angle) {
  final half = angle / 2;
  final s = math.sin(half);
  return Quaternion(axis.x * s, axis.y * s, axis.z * s, math.cos(half));
}

/// Shortest rotation that takes direction [from] onto direction [to].
Quaternion rotationBetween(Vector3 from, Vector3 to) {
  final a = from.normalized();
  final b = to.normalized();
  final d = a.dot(b).clamp(-1.0, 1.0);
  final axis = a.cross(b);
  if (axis.length < 1e-9) {
    if (d > 0) return Quaternion.identity();
    // Opposite vectors: rotate 180° about any perpendicular axis.
    var perp = a.cross(Vector3(1, 0, 0));
    if (perp.length < 1e-6) perp = a.cross(Vector3(0, 1, 0));
    return axisAngle(perp.normalized(), math.pi);
  }
  return axisAngle(axis.normalized(), math.acos(d));
}

/// Heading (radians, counter-clockwise from world +x) of the horizontal
/// projection of [v], or null when [v] is too close to vertical.
double? horizontalHeading(Vector3 v, {double minLength = 1e-3}) {
  if (math.sqrt(v.x * v.x + v.y * v.y) < minLength) return null;
  return math.atan2(v.y, v.x);
}

/// Rotates [v] about the world z axis by [angle] radians.
Vector3 rotateAboutZ(Vector3 v, double angle) {
  final c = math.cos(angle);
  final s = math.sin(angle);
  return Vector3(c * v.x - s * v.y, s * v.x + c * v.y, v.z);
}

/// Wraps [angle] into (-π, π].
double wrapAngle(double angle) {
  var a = angle % (2 * math.pi);
  if (a > math.pi) a -= 2 * math.pi;
  return a;
}
