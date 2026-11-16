import 'dart:math' as math;

import 'package:flutter_test/flutter_test.dart';
import 'package:vector_math/vector_math_64.dart';
import 'package:vslam/core/se3.dart';
import 'package:vslam/estimation/config.dart';
import 'package:vslam/estimation/keyframe_selector.dart';

/// Scripted trajectories at 10 Hz; frame i at t = i · 0.1 s.
const t0 = 1763300000000000;
int tAt(int i) => t0 + i * 100000;

/// A camera looking along world +x (OpenCV z forward), slightly tilted, so
/// the poses are not trivially the identity.
final Matrix3 base = so3Exp(Vector3(0.1, -0.2, 0.3));

List<int> keyframeFrames(KeyframeSelector s) => [for (final k in s.keyframes) k.frameIndex];

void main() {
  test('straight line: a keyframe every 0.25 m of motion', () {
    // 0.06 m per frame: 4 frames = 0.24 m (no), 5 frames = 0.30 m (yes).
    final s = KeyframeSelector();
    for (var i = 0; i <= 40; i++) {
      s.offer(frameIndex: i, tUs: tAt(i), vTc: Pose3(base, Vector3(0.06 * i, 0.5, 1.4)));
    }
    expect(keyframeFrames(s), [0, 5, 10, 15, 20, 25, 30, 35, 40]);
    expect(s.keyframes.first.reasons, {KeyframeReason.first});
    expect(s.keyframes.skip(1).every((k) => k.reasons.length == 1 && k.reasons.contains(KeyframeReason.translation)),
        isTrue);
    final edges = odometryEdges(s.keyframes);
    expect(edges, hasLength(8));
    for (final e in edges) {
      expect(e.stepM, closeTo(0.30, 1e-12));
      expect(e.rotationDeg, closeTo(0, 1e-6));
      expect(e.trackingGap, isFalse);
    }
  });

  test('turning in place: a keyframe every 15° of rotation', () {
    // 4° per frame about the camera's vertical axis: 3 frames = 12° (no),
    // 4 frames = 16° (yes).
    final s = KeyframeSelector();
    for (var i = 0; i <= 24; i++) {
      final r = base.multiplied(so3Exp(Vector3(0, 4 * i * math.pi / 180, 0)));
      s.offer(frameIndex: i, tUs: tAt(i), vTc: Pose3(r, Vector3(1, 2, 1.4)));
    }
    expect(keyframeFrames(s), [0, 4, 8, 12, 16, 20, 24]);
    expect(s.keyframes.skip(1).every((k) => k.reasons.single == KeyframeReason.rotation), isTrue);
    expect(odometryEdges(s.keyframes).map((e) => e.rotationDeg), everyElement(closeTo(16, 1e-9)));
  });

  test('marker detections: a keyframe when ≥ 0.3 s have passed', () {
    final still = Pose3(base, Vector3(1, 2, 1.4));
    // A marker in every frame: keyframes every 3 frames (0.3 s exactly).
    final every = KeyframeSelector();
    for (var i = 0; i <= 10; i++) {
      every.offer(frameIndex: i, tUs: tAt(i), vTc: still, markerIds: const [4]);
    }
    expect(keyframeFrames(every), [0, 3, 6, 9]);
    expect(every.keyframes[1].reasons, {KeyframeReason.marker});
    expect(every.keyframes[1].markerIds, [4]);

    // A marker in every other frame: 0.2 s is too soon, 0.4 s is enough.
    final alternate = KeyframeSelector();
    for (var i = 0; i <= 10; i++) {
      alternate.offer(frameIndex: i, tUs: tAt(i), vTc: still, markerIds: i.isEven ? const [2, 7] : const []);
    }
    expect(keyframeFrames(alternate), [0, 4, 8]);

    // No marker, no motion: only the first frame.
    final none = KeyframeSelector();
    for (var i = 0; i <= 10; i++) {
      none.offer(frameIndex: i, tUs: tAt(i), vTc: still);
    }
    expect(keyframeFrames(none), [0]);
  });

  test('reasons combine; thresholds come from the configuration', () {
    // 0.06 m and 4° per frame with a marker in every frame.
    Pose3 pose(int i) => Pose3(base.multiplied(so3Exp(Vector3(0, 4 * i * math.pi / 180, 0))), Vector3(0.06 * i, 0, 0));
    final s = KeyframeSelector();
    for (var i = 0; i <= 8; i++) {
      s.offer(frameIndex: i, tUs: tAt(i), vTc: pose(i), markerIds: const [1]);
    }
    // Frame 3: marker (0.3 s); 12° and 0.18 m are below the limits.
    // Frame 6: marker again (0.3 s), still 12° / 0.18 m from frame 3.
    expect(keyframeFrames(s), [0, 3, 6]);

    // Larger limits: 0.5 m (9 frames of 0.06 m) and 40° (10 frames of 4°),
    // marker rule 1 s (10 frames).
    final wide = KeyframeSelector(
      const SlamConfig(keyframeTranslationM: 0.5, keyframeRotationDeg: 40, keyframeMarkerGapS: 1.0),
    );
    for (var i = 0; i <= 20; i++) {
      wide.offer(frameIndex: i, tUs: tAt(i), vTc: pose(i), markerIds: const [1]);
    }
    expect(keyframeFrames(wide), [0, 9, 18]);
    expect(wide.keyframes[1].reasons, {KeyframeReason.translation});
  });

  test('untracked frames are never keyframes and mark a tracking gap', () {
    final s = KeyframeSelector();
    for (var i = 0; i <= 20; i++) {
      final tracked = i >= 2 && (i < 8 || i > 11);
      s.offer(
        frameIndex: i,
        tUs: tAt(i),
        vTc: tracked ? Pose3(base, Vector3(0.06 * i, 0, 0)) : null,
        markerIds: const [3],
      );
    }
    // First tracked frame is 2; markers every 0.3 s: 5; frames 8–11 lost;
    // 12 is the first tracked frame after the gap (0.7 s, 0.42 m), then 15, 18.
    expect(keyframeFrames(s), [2, 5, 12, 15, 18]);
    expect(s.keyframes.map((k) => k.trackingGapBefore).toList(), [false, false, true, false, false]);
    expect(odometryEdges(s.keyframes).map((e) => e.trackingGap).toList(), [false, true, false, false]);
    expect(s.keyframes[2].reasons, {KeyframeReason.translation, KeyframeReason.marker});
  });
}
