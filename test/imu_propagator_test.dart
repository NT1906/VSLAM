import 'dart:math' as math;

import 'package:flutter_test/flutter_test.dart';
import 'package:vector_math/vector_math_64.dart';
import 'package:vslam/core/se3.dart';
import 'package:vslam/estimation/config.dart';
import 'package:vslam/estimation/imu_propagator.dart';

const g = 9.80665;
final ez = Vector3(0, 0, 1);

/// 100 Hz samples of (specific force, angular rate) from [f] over [seconds].
(ImuSeries, ImuSeries) samples(double seconds, (Vector3, Vector3) Function(double t) f, {int offsetUs = 0}) {
  final n = (seconds * 100).round();
  final t = [for (var i = 0; i <= n; i++) i * 10000];
  final acc = <Vector3>[];
  final gyro = <Vector3>[];
  for (final us in t) {
    final (a, w) = f(us * 1e-6);
    acc.add(a);
    gyro.add(w);
  }
  return (ImuSeries([for (final x in t) x + offsetUs], acc), ImuSeries(t, gyro));
}

ImuState state(Matrix3 r, {Vector3? v, Vector3? ba, Vector3? bg}) => ImuState(
      tUs: 0,
      p: Vector3(1, 2, 3),
      v: v ?? Vector3.zero(),
      q: Quaternion.fromRotation(r)..normalize(),
      ba: ba ?? Vector3.zero(),
      bg: bg ?? Vector3.zero(),
    );

double angleBetween(Matrix3 a, Matrix3 b) => rotationAngle(a.transposed().multiplied(b));

void main() {
  // A device held tilted (not aligned with W), so gravity appears on all
  // three device axes.
  final tilt = so3Exp(Vector3(0.4, -0.3, 1.1));

  test('constant velocity: p = p0 + v·t, orientation unchanged', () {
    final (acc, gyro) = samples(10, (t) => (tilt.transposed().transformed(ez * g), Vector3.zero()));
    final s = state(tilt, v: Vector3(0.5, -0.2, 0.1));
    ImuPropagator(acc: acc, gyro: gyro).propagate(s, 10000000);
    expect((s.p - (Vector3(1, 2, 3) + Vector3(0.5, -0.2, 0.1) * 10)).length, lessThan(1e-9));
    expect((s.v - Vector3(0.5, -0.2, 0.1)).length, lessThan(1e-11));
    expect(angleBetween(s.rWb, tilt), lessThan(1e-12));
  });

  test('constant world acceleration: p = p0 + v0·t + ½·a·t²', () {
    final a = Vector3(0.3, -0.1, 0.05);
    final v0 = Vector3(0.2, 0, 0);
    final (acc, gyro) = samples(10, (t) => (tilt.transposed().transformed(a + ez * g), Vector3.zero()));
    final s = state(tilt, v: v0);
    // Propagate in pieces that do not fall on sample times.
    final p = ImuPropagator(acc: acc, gyro: gyro);
    for (final t in [1234567, 5000001, 7777777, 10000000]) {
      p.propagate(s, t);
      final ts = t * 1e-6;
      final expected = Vector3(1, 2, 3) + v0 * ts + a * (0.5 * ts * ts);
      expect((s.p - expected).length, lessThan(1e-9), reason: 't = $ts');
      expect((s.v - (v0 + a * ts)).length, lessThan(1e-10));
    }
  });

  test('constant rotation about the vertical: R(t) = Exp(ω t ez) R0, p fixed', () {
    // Turning at 0.8 rad/s about world z while held tilted: the gyroscope
    // reads R0ᵀ·(0, 0, ω), the accelerometer R0ᵀ·g·ez, both constant.
    const omega = 0.8;
    final wBody = tilt.transposed().transformed(ez * omega);
    final (acc, gyro) = samples(10, (t) => (tilt.transposed().transformed(ez * g), wBody));
    final s = state(tilt);
    final p = ImuPropagator(acc: acc, gyro: gyro);
    for (final t in [2500000, 6666666, 10000000]) {
      p.propagate(s, t);
      final expected = so3Exp(ez * (omega * t * 1e-6)).multiplied(tilt);
      expect(angleBetween(s.rWb, expected), lessThan(1e-9), reason: 't = $t');
    }
    expect((s.p - Vector3(1, 2, 3)).length, lessThan(1e-9));
    expect(s.v.length, lessThan(1e-10));
  });

  test('constant rotation about a tilted axis: R(t) = R0 Exp(ω t), drift within the discretization bound', () {
    // Body rate ω = 0.5 rad/s about a body axis that is not vertical, so the
    // accelerometer reading (gravity in device axes) turns with time. The
    // rotation is still integrated exactly. Position: the step averages the
    // accelerometer at both ends of the 10 ms step, a chord of the circle
    // gravity traces in device axes. That shortens the part of g normal to
    // the rotation axis by 1 − cos(ωΔt/2), a constant world acceleration:
    // drift ≤ ½ · g · (1 − cos(ωΔt/2)) · T².
    final w = Vector3(0.3, -0.2, 0.36)..normalize();
    w.scale(0.5);
    final (acc, gyro) = samples(10, (t) {
      final r = tilt.multiplied(so3Exp(w * t));
      return (r.transposed().transformed(ez * g), w);
    });
    final s = state(tilt);
    ImuPropagator(acc: acc, gyro: gyro).propagate(s, 10000000);
    expect(angleBetween(s.rWb, tilt.multiplied(so3Exp(w * 10))), lessThan(1e-9));
    final bound = 0.5 * g * (1 - math.cos(0.5 * 0.01 / 2)) * 100;
    expect((s.p - Vector3(1, 2, 3)).length, lessThan(bound));
  });

  test('known biases in the state cancel biased readings exactly', () {
    final ba = Vector3(0.08, -0.05, 0.12);
    final bg = Vector3(0.01, -0.02, 0.004);
    const omega = 0.8;
    final wBody = tilt.transposed().transformed(ez * omega);
    final (acc, gyro) = samples(10, (t) => (tilt.transposed().transformed(ez * g) + ba, wBody + bg));
    final s = state(tilt, ba: ba, bg: bg);
    ImuPropagator(acc: acc, gyro: gyro).propagate(s, 10000000);
    expect(angleBetween(s.rWb, so3Exp(ez * (omega * 10)).multiplied(tilt)), lessThan(1e-9));
    expect((s.p - Vector3(1, 2, 3)).length, lessThan(1e-9));

    // The same readings without the biases in the state drift away.
    final wrong = state(tilt);
    ImuPropagator(acc: acc, gyro: gyro).propagate(wrong, 10000000);
    expect((wrong.p - Vector3(1, 2, 3)).length, greaterThan(1));
  });

  test('accelerometer stamps between gyroscope stamps are interpolated', () {
    // Same constant-acceleration case with the accelerometer 3 ms late.
    final a = Vector3(0.3, -0.1, 0.05);
    final (acc, gyro) = samples(5, (t) => (tilt.transposed().transformed(a + ez * g), Vector3.zero()), offsetUs: 3000);
    final s = state(tilt)..tUs = 10000;
    ImuPropagator(acc: acc, gyro: gyro).propagate(s, 4000000);
    const ts = 3.99;
    expect((s.p - (Vector3(1, 2, 3) + a * (0.5 * ts * ts))).length, lessThan(1e-9));
  });

  group('biases at rest', () {
    test('recovered from a still window with noise', () {
      final rng = math.Random(7);
      double n() => (rng.nextDouble() + rng.nextDouble() + rng.nextDouble() - 1.5) * 2; // σ = 1
      final ba = Vector3(0.05, -0.03, 0.08);
      final bg = Vector3(0.01, -0.02, 0.005);
      final (acc, gyro) = samples(
        4,
        (t) => (
          tilt.transposed().transformed(ez * g) + ba + Vector3(n(), n(), n()) * 0.02,
          bg + Vector3(n(), n(), n()) * 0.002,
        ),
      );
      const config = SlamConfig();
      final r = estimateRestBiases(acc, gyro, 500000, tilt, config);
      expect(r.atRest, isTrue);
      expect(r.samples, 201); // 2 s at 100 Hz, both ends included
      // Mean of 201 samples: σ/√201 = 0.0014 and 0.00014.
      expect((r.ba - ba).length, lessThan(0.01));
      expect((r.bg - bg).length, lessThan(0.001));
      expect(r.accMean.length, closeTo(g, 0.1));
    });

    test('a window with motion is not "at rest" and gives zero biases', () {
      final (acc, gyro) = samples(4, (t) {
        final w = Vector3(0, 0, 0.3 * math.sin(3 * t));
        return (tilt.transposed().transformed(ez * g) + Vector3(0.4 * math.sin(5 * t), 0, 0), w);
      });
      final r = estimateRestBiases(acc, gyro, 0, tilt, const SlamConfig());
      expect(r.atRest, isFalse);
      expect(r.ba.length, 0);
      expect(r.bg.length, 0);
    });

    test('the first still window after a start time is found', () {
      // Moving for 1.5 s, then still.
      final (acc, gyro) = samples(6, (t) {
        final moving = t < 1.5;
        return (
          tilt.transposed().transformed(ez * g) + Vector3(moving ? 0.5 * math.sin(7 * t) : 0, 0, 0),
          Vector3(0, moving ? 0.4 * math.sin(5 * t) : 0, 0),
        );
      });
      const config = SlamConfig();
      final start = findRestWindow(acc, gyro, 200000, config)!;
      // The first 0.1 s grid start whose window holds no moving sample.
      expect(start, 1500000);
      expect(findRestWindow(acc, gyro, 200000, const SlamConfig(imuRestSearchSeconds: 1.0)), isNull);
    });
  });
}
