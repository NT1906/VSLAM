import 'dart:io';
import 'dart:typed_data';

import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:provider/provider.dart';
import 'package:vector_math/vector_math_64.dart';
import 'package:vslam/slam/ar_frame.dart';
import 'package:vslam/slam/ar_pose.dart';
import 'package:vslam/slam/slam_service.dart';
import 'package:vslam/vision/aruco_detector.dart';
import 'package:vslam/vision/camera_model.dart';
import 'package:vslam/vision/marker_observation.dart';
import 'package:vslam/widgets/marker_overlay.dart';

import 'support/marker_render.dart';

const k = CameraIntrinsics(fx: 721.3, fy: 721.3, cx: 478.5, cy: 360.7, width: 960, height: 720);

/// Returns a fixed result and records what it was asked.
class FakeDetector implements MarkerDetector {
  FakeDetector(this.result);

  final FrameDetections result;
  final calls = <(CameraIntrinsics, double)>[];

  @override
  Future<FrameDetections> detect(Uint8List jpeg, CameraIntrinsics intrinsics, double sideM) async {
    calls.add((intrinsics, sideM));
    return result;
  }
}

ArFrame frame({List<double>? display}) => ArFrame(
  time: DateTime(2026, 11, 16),
  received: DateTime(2026, 11, 16),
  width: 960,
  height: 720,
  gray: Uint8List(0),
  intrinsics: k,
  state: ArTrackingState.tracking,
  jpeg: Uint8List.fromList([1, 2, 3]),
  displayTransform: display,
);

void main() {
  group('ImageToView', () {
    // 366 × 500 view: the rotated image (720 × 960) is scaled by
    // max(366/720, 500/960) = 0.5208 to 375 × 500, overflowing by 4.5 px left
    // and right.
    const view = Size(366, 500);
    final m = ImageToView.portraitAspectFill(960, 720, view);
    Vector2 edge(double x, double y) => Vector2(x - 0.5, y - 0.5); // pixel-edge coordinates

    test('portrait aspect fill: image +x runs down the screen, image +y to the left', () {
      final s = 500 / 960;
      expect(m.map(edge(0, 0), view).dx, closeTo(-4.5 + 720 * s, 1e-9)); // image top-left → view top-right
      expect(m.map(edge(0, 0), view).dy, closeTo(0, 1e-9));
      expect(m.map(edge(960, 0), view).dy, closeTo(500, 1e-9)); // image top-right → view bottom-right
      expect(m.map(edge(0, 720), view).dx, closeTo(-4.5, 1e-9)); // image bottom-left → view top-left
      final centre = m.map(edge(480, 360), view);
      expect(centre.dx, closeTo(183, 1e-9));
      expect(centre.dy, closeTo(250, 1e-9));
    });

    test('ARKit transform is used as given when the frame carries one', () {
      final t = ImageToView.fromFrame(frame(display: m.t), view);
      expect(t.t, m.t);
      final identity = ImageToView.fromFrame(frame(display: [1, 0, 0, 1, 0, 0]), view);
      expect(identity.map(edge(480, 360), view), const Offset(183, 250));
      final fallback = ImageToView.fromFrame(frame(), view);
      expect(fallback.t, m.t);
    });
  });

  group('MarkerOverlay', () {
    late Directory tmp;
    setUp(() => tmp = Directory.systemTemp.createTempSync('vslam_overlay_'));
    tearDown(() => tmp.deleteSync(recursive: true));

    Future<SlamService> pump(WidgetTester tester, MarkerDetector detector) async {
      final slam = SlamService();
      await tester.pumpWidget(
        ChangeNotifierProvider.value(
          value: slam,
          child: MaterialApp(
            home: SizedBox(
              width: 366,
              height: 500,
              child: MarkerOverlay(sideM: 0.123, detector: detector, documents: () async => tmp),
            ),
          ),
        ),
      );
      return slam;
    }

    testWidgets('shows ID, distance and status of each detection', (tester) async {
      final accepted = SceneMarker(3, 0.123, markerPose(1.0, yawDeg: 30));
      final ambiguous = SceneMarker(8, 0.123, markerPose(2.0, ox: 0.3, yawDeg: -15));
      MarkerObservation obs(SceneMarker m, MarkerRejection? r) => MarkerObservation(
        id: m.id,
        corners: m.corners(k),
        solutions: [PnpSolution(m.cTm, 0.1)],
        sidePx: 80,
        viewAngleDeg: 30,
        rejection: r,
      );
      final detector = FakeDetector(
        FrameDetections([obs(accepted, null), obs(ambiguous, MarkerRejection.ambiguous)], 14.2),
      );
      final slam = await pump(tester, detector);
      expect(find.textContaining('side 12.3 cm'), findsOneWidget);

      slam.handleFrame(frame());
      await tester.pump();
      await tester.pump();

      expect(detector.calls, hasLength(1));
      expect(detector.calls.single.$2, 0.123);
      expect(detector.calls.single.$1.fx, k.fx); // no calibration in Documents → ARKit's
      expect(find.text('ID 3  1.000 m'), findsOneWidget);
      expect(find.text('ID 8  2.000 m  ambiguous'), findsOneWidget);
      expect(find.textContaining('ARKit K · 2 found · 14 ms'), findsOneWidget);
    });

    testWidgets('a frame without JPEG is skipped', (tester) async {
      final detector = FakeDetector(const FrameDetections([], 1));
      final slam = await pump(tester, detector);
      slam.handleFrame(ArFrame(
        time: DateTime(2026),
        received: DateTime(2026),
        width: 960,
        height: 720,
        gray: Uint8List(0),
        intrinsics: k,
        state: ArTrackingState.tracking,
      ));
      await tester.pump();
      expect(detector.calls, isEmpty);
    });
  });
}
