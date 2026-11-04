import 'dart:math' as math;

import 'package:flutter_test/flutter_test.dart';
import 'package:vector_math/vector_math_64.dart';
import 'package:vslam/core/rotation.dart';

void expectVec(Vector3 actual, Vector3 expected, {double tol = 1e-9}) {
  expect((actual - expected).length, lessThan(tol), reason: '$actual != $expected');
}

void main() {
  test('rotate: +90° about z maps x to y', () {
    expectVec(rotate(axisAngle(Vector3(0, 0, 1), math.pi / 2), Vector3(1, 0, 0)), Vector3(0, 1, 0));
  });

  test('quaternion product composes right-to-left', () {
    final rz = axisAngle(Vector3(0, 0, 1), math.pi / 2);
    final rx = axisAngle(Vector3(1, 0, 0), math.pi / 2);
    // rx first: y -> z, then rz leaves z alone.
    expectVec(rotate(rz * rx, Vector3(0, 1, 0)), Vector3(0, 0, 1));
    // rz first: y -> -x, then rx leaves x alone.
    expectVec(rotate(rx * rz, Vector3(0, 1, 0)), Vector3(-1, 0, 0));
  });

  test('rotationBetween handles general, same and opposite vectors', () {
    for (final pair in [
      (Vector3(0, 9.8, 0), Vector3(0, 0, 1)),
      (Vector3(1, 2, 3), Vector3(-3, 0.5, 2)),
      (Vector3(0, 0, 1), Vector3(0, 0, 1)),
      (Vector3(0, 0, 1), Vector3(0, 0, -1)),
    ]) {
      final q = rotationBetween(pair.$1, pair.$2);
      expectVec(rotate(q, pair.$1.normalized()), pair.$2.normalized(), tol: 1e-6);
    }
  });

  test('rotateAboutZ and wrapAngle', () {
    expectVec(rotateAboutZ(Vector3(0, 1, 5), math.pi / 2), Vector3(-1, 0, 5), tol: 1e-12);
    expect(wrapAngle(3 * math.pi), closeTo(math.pi, 1e-12));
    expect(wrapAngle(-3 * math.pi / 2), closeTo(math.pi / 2, 1e-12));
  });
}
