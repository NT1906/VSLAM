import 'dart:math' as math;

import 'package:flutter_test/flutter_test.dart';
import 'package:vector_math/vector_math_64.dart';
import 'package:vslam/core/se3.dart';
import 'package:vslam/vision/camera_model.dart';

Pose3 randomPose(math.Random rng) {
  double r() => rng.nextDouble() * 2 - 1;
  return Pose3(so3Exp(Vector3(r(), r(), r()) * 2.5), Vector3(r(), r(), r()) * 3);
}

double maxAbs(List<double> a, List<double> b) =>
    [for (var i = 0; i < a.length; i++) (a[i] - b[i]).abs()].reduce(math.max);

double poseDistance(Pose3 a, Pose3 b) {
  final e = poseError(a.inverse() * b);
  return e.map((v) => v.abs()).reduce(math.max);
}

void main() {
  final rng = math.Random(7);

  group('SE(3)', () {
    test('retract and poseError are inverse (exactly, below π)', () {
      for (var i = 0; i < 50; i++) {
        final z = randomPose(rng);
        final delta = [for (var j = 0; j < 6; j++) (rng.nextDouble() * 2 - 1) * 0.5];
        // Z⁻¹·retract(Z, δ) = (Exp(δθ), δt), so poseError recovers δ.
        expect(maxAbs(poseError(z.inverse() * z.retract(delta)), delta), lessThan(1e-12));
        // And the other way round: retract(Z, poseError(Z⁻¹·W)) = W.
        final w = randomPose(rng);
        expect(poseDistance(z.retract(poseError(z.inverse() * w)), w), lessThan(1e-12));
      }
    });

    test('compose and inverse identities', () {
      for (var i = 0; i < 50; i++) {
        final a = randomPose(rng), b = randomPose(rng), c = randomPose(rng);
        expect(poseDistance(a * a.inverse(), Pose3.identity()), lessThan(1e-12));
        expect(poseDistance(a.inverse() * a, Pose3.identity()), lessThan(1e-12));
        expect(poseDistance((a * b) * c, a * (b * c)), lessThan(1e-12));
        expect(poseDistance((a * b).inverse(), b.inverse() * a.inverse()), lessThan(1e-12));
        final p = Vector3(rng.nextDouble(), rng.nextDouble(), rng.nextDouble());
        expect(((a * b).transform(p) - a.transform(b.transform(p))).length, lessThan(1e-12));
      }
    });

    test('so3Log inverts so3Exp', () {
      for (var i = 0; i < 50; i++) {
        final w = Vector3(rng.nextDouble() * 2 - 1, rng.nextDouble() * 2 - 1, rng.nextDouble() * 2 - 1) * 1.5;
        expect((so3Log(so3Exp(w)) - w).length, lessThan(1e-12));
      }
      expect(so3Log(so3Exp(Vector3(1e-14, 0, 0))).x, closeTo(1e-14, 1e-20));
    });

    test('JSON round trip', () {
      final a = randomPose(rng);
      expect(poseDistance(Pose3.fromJson(a.toJson()), a), lessThan(1e-12));
    });
  });

  group('CameraIntrinsics', () {
    const k = CameraIntrinsics(fx: 725, fy: 724, cx: 479.5, cy: 359.5, width: 960, height: 720, k1: 0.08, k2: -0.15);

    test('project then normalize round trip with distortion', () {
      for (var i = 0; i < 200; i++) {
        final p =
            Vector3(rng.nextDouble() * 1.2 - 0.6, rng.nextDouble() * 0.9 - 0.45, 1) * (0.5 + rng.nextDouble() * 4);
        final uv = k.project(p)!;
        final n = k.normalize(uv.x, uv.y);
        expect((n - Vector2(p.x / p.z, p.y / p.z)).length, lessThan(1e-9));
      }
      expect(k.project(Vector3(0, 0, -1)), isNull);
    });

    test('scaledTo maps the same ray to the same pixel position', () {
      final half = k.scaledTo(480, 360);
      for (var i = 0; i < 50; i++) {
        final p = Vector3(rng.nextDouble() - 0.5, rng.nextDouble() - 0.5, 1 + rng.nextDouble());
        final full = k.project(p)!;
        final small = half.project(p)!;
        // Pixel centres: u_small = (u_full + 0.5)/2 − 0.5.
        expect(small.x, closeTo((full.x + 0.5) / 2 - 0.5, 1e-9));
        expect(small.y, closeTo((full.y + 0.5) / 2 - 0.5, 1e-9));
      }
      final back = half.scaledTo(960, 720);
      expect(back.toJson(), k.toJson());
    });

    test('matches the native 2×2 box filter (1920×1440 → 960×720)', () {
      // ArPlugin.swift: f/s and (c + 0.5)/s − 0.5 with s = 1920 / 960 = 2.
      const arkit = CameraIntrinsics(fx: 1450.2, fy: 1450.2, cx: 961.3, cy: 718.9, width: 1920, height: 1440);
      final s = 1920 ~/ 960;
      final swift = [arkit.fx / s, arkit.fy / s, (arkit.cx + 0.5) / s - 0.5, (arkit.cy + 0.5) / s - 0.5];
      final dart = arkit.scaledTo(960, 720);
      expect([dart.fx, dart.fy, dart.cx, dart.cy], swift);
    });
  });
}
