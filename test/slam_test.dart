import 'dart:math' as math;

import 'package:flutter_test/flutter_test.dart';
import 'package:vector_math/vector_math_64.dart';
import 'package:vslam/core/rotation.dart';
import 'package:vslam/models/track_point.dart';
import 'package:vslam/models/trajectory.dart';
import 'package:vslam/slam/ar_pose.dart';

void expectVec(Vector3 actual, Vector3 expected, {double tol = 1e-9}) {
  expect((actual - expected).length, lessThan(tol), reason: '$actual != $expected');
}

void main() {
  group('SlamAligner', () {
    test('camera looking along AR −z: forward motion becomes +y', () {
      final a = SlamAligner()..anchor(Vector3(2, 1.5, 3), Quaternion.identity());
      expectVec(a.toSession(Vector3(2, 1.5, 3)), Vector3.zero());
      expectVec(a.toSession(Vector3(2, 1.5, 2)), Vector3(0, 1, 0));
      expectVec(a.toSession(Vector3(3, 1.5, 3)), Vector3(1, 0, 0));
      expectVec(a.toSession(Vector3(2, 2.5, 3)), Vector3(0, 0, 1));
    });

    test('camera yawed to look along AR +x: that direction becomes +y', () {
      // Rotating −90° about AR +y turns the −z view axis to +x.
      final q = axisAngle(Vector3(0, 1, 0), -math.pi / 2);
      final a = SlamAligner()..anchor(Vector3.zero(), q);
      expectVec(a.toSession(Vector3(1, 0, 0)), Vector3(0, 1, 0), tol: 1e-9);
      // Right-hand side of the camera (AR +z) is session +x.
      expectVec(a.toSession(Vector3(0, 0, 1)), Vector3(1, 0, 0), tol: 1e-9);
    });

    test('pitched camera still yields a horizontal heading', () {
      final q = axisAngle(Vector3(1, 0, 0), -0.6); // looking down a bit
      final a = SlamAligner()..anchor(Vector3.zero(), q);
      expectVec(a.toSession(Vector3(0, 0, -2)), Vector3(0, 2, 0), tol: 1e-9);
    });

    test('ArPose.fromMap parses native events', () {
      final p = ArPose.fromMap({
        'state': 'tracking',
        't': [1, 2.5, -3],
        'q': [0, 0, 0, 1],
      });
      expect(p.state, ArTrackingState.tracking);
      expect(p.hasPose, isTrue);
      expectVec(p.translation!, Vector3(1, 2.5, -3));
      final lost = ArPose.fromMap({'state': 'limited', 'reason': 'Too dark'});
      expect(lost.hasPose, isFalse);
      expect(lost.reason, 'Too dark');
    });
  });

  group('Trajectory', () {
    final t0 = DateTime(2026);
    TrackPoint pt(int ms, double x, double y) =>
        TrackPoint(position: Vector3(x, y, 0), timestamp: t0.add(Duration(milliseconds: ms)));

    test('distance accumulates and positionAt interpolates', () {
      final tr = Trajectory(name: 's', type: TrajectoryType.slam)
        ..add(pt(0, 0, 0))
        ..add(pt(1000, 0, 2))
        ..add(pt(2000, 2, 2));
      expect(tr.distance, closeTo(4, 1e-12));
      expectVec(tr.positionAt(t0.add(const Duration(milliseconds: 500)))!, Vector3(0, 1, 0));
      expectVec(tr.positionAt(t0.add(const Duration(milliseconds: 1500)))!, Vector3(1, 2, 0));
      expect(tr.positionAt(t0.add(const Duration(seconds: 5))), isNull);
    });

    test('TrackPoint JSON round-trips', () {
      final p = TrackPoint(position: Vector3(1, 2, 3), timestamp: t0, confidence: 0.5);
      final q = TrackPoint.fromJson(p.toJson());
      expectVec(q.position, p.position);
      expect(q.timestamp, p.timestamp);
      expect(q.confidence, 0.5);
    });
  });
}
