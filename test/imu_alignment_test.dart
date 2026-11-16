import 'dart:math' as math;

import 'package:flutter_test/flutter_test.dart';
import 'package:vector_math/vector_math_64.dart';
import 'package:vslam/calibration/imu_alignment.dart';
import 'package:vslam/core/se3.dart';

/// Camera angular velocity (camera axes): each axis takes a 4 s turn at
/// large rotation, as in the device protocol, with smooth (sin²) hand-overs
/// so the rate stays continuous like a real hand motion.
Vector3 omegaC(double t) {
  final v = Vector3(0.15 * math.sin(3.1 * t), 0.15 * math.cos(2.3 * t), 0.1 * math.sin(1.7 * t));
  final main = 2.0 * math.sin(2 * math.pi * 0.7 * t);
  for (var axis = 0; axis < 3; axis++) {
    // Envelope peaking in the middle of this axis's 4 s slot, zero at its ends.
    final local = (t - 4 * axis) % 12;
    final env = local < 4 ? math.pow(math.sin(math.pi * local / 4), 2).toDouble() : 0.0;
    v[axis] += main * env;
  }
  return v;
}

double gauss(math.Random r) =>
    math.sqrt(-2 * math.log(1 - r.nextDouble())) * math.cos(2 * math.pi * r.nextDouble());

({List<TimedRotation> cam, List<TimedVector> gyro}) simulate({
  required Matrix3 rCI,
  required int offsetUs,
  required int seed,
  double gyroNoise = 0.01,
  double rotNoise = 0.0005,
}) {
  final rnd = math.Random(seed);
  const t0 = 1700000000000000;
  const durationS = 12.0;
  // Integrate R_wc at 1 kHz.
  var r = Matrix3.identity();
  final cam = <TimedRotation>[];
  const dt = 0.001;
  for (var k = 0; k * dt <= durationS; k++) {
    final t = k * dt;
    if (k % 33 == 0) {
      // ~30 Hz, with orientation noise.
      final n = Vector3(gauss(rnd), gauss(rnd), gauss(rnd)) * rotNoise;
      cam.add(TimedRotation(t0 + (t * 1e6).round(), r.multiplied(so3Exp(n))));
    }
    r = r.multiplied(so3Exp(omegaC(t + dt / 2) * dt));
  }
  final gyro = <TimedVector>[];
  final rIC = rCI.transposed();
  for (var k = 0; k * 0.01 <= durationS + 0.5; k++) {
    final tImu = -0.25 + k * 0.01; // IMU stamp, seconds after t0
    final w = rIC.transformed(omegaC(tImu - offsetUs * 1e-6));
    final n = Vector3(gauss(rnd), gauss(rnd), gauss(rnd)) * gyroNoise;
    gyro.add(TimedVector(t0 + (tImu * 1e6).round(), w + n));
  }
  return (cam: cam, gyro: gyro);
}

void main() {
  test('Jacobi eigen-decomposition of a symmetric matrix', () {
    final m = [
      [4.0, 1.0, 2.0, 0.5],
      [1.0, 3.0, 0.0, 1.0],
      [2.0, 0.0, 5.0, 0.2],
      [0.5, 1.0, 0.2, 1.0],
    ];
    final (values, vectors) = jacobiEigen(m);
    for (var e = 0; e < 4; e++) {
      for (var i = 0; i < 4; i++) {
        var mv = 0.0;
        for (var j = 0; j < 4; j++) {
          mv += m[i][j] * vectors[j][e];
        }
        expect(mv, closeTo(values[e] * vectors[i][e], 1e-9));
      }
    }
  });

  test('alignRotations recovers an exact rotation', () {
    final rnd = math.Random(3);
    final truth = so3Exp(Vector3(0.4, -1.2, 2.0));
    final a = [for (var i = 0; i < 30; i++) Vector3(gauss(rnd), gauss(rnd), gauss(rnd))];
    final b = [for (final v in a) truth.transformed(v)];
    final r = alignRotations(a, b);
    expect(rotationAngle(truth.transposed().multiplied(r)), lessThan(1e-9));
  });

  test('Apple convention matrix is a proper rotation', () {
    final m = appleCameraFromDevice;
    expect(m.determinant(), closeTo(1, 1e-12));
    // Device −y (toward the Home button) is camera +x.
    expect(m.transformed(Vector3(0, -1, 0)), Vector3(1, 0, 0));
    // The rear camera looks out of the back: device −z is camera +z.
    expect(m.transformed(Vector3(0, 0, -1)), Vector3(0, 0, 1));
  });

  for (final (seed, offsetUs) in [(1, 23000), (2, -41000), (3, 7500)]) {
    test('recovers offset $offsetUs µs and rotation from noisy data (seed $seed)', () {
      // Apple's convention perturbed by a few degrees, as a real unit might be.
      final truth = appleCameraFromDevice.multiplied(so3Exp(Vector3(0.03, -0.02, 0.04)));
      final sim = simulate(rCI: truth, offsetUs: offsetUs, seed: seed);
      final est = estimateImuAlignment(sim.cam, sim.gyro);
      expect(est.offsetUs, closeTo(offsetUs, 1500));
      expect(est.angleToDeg(truth), lessThan(0.5));
      expect(est.rmsResidual, lessThan(est.rmsResidualIdentityOffset));
      expect(est.angleToDeg(appleCameraFromDevice), closeTo(rotationAngle(so3Exp(Vector3(0.03, -0.02, 0.04))) * 180 / math.pi, 0.5));
    });
  }

  test('too little motion is reported, not guessed', () {
    final cam = [for (var i = 0; i < 100; i++) TimedRotation(i * 33000, Matrix3.identity())];
    final gyro = [for (var i = 0; i < 300; i++) TimedVector(i * 10000, Vector3.zero())];
    expect(() => estimateImuAlignment(cam, gyro), throwsStateError);
  });
}
