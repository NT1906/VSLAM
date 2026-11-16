import 'dart:io';
import 'dart:math' as math;

import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:provider/provider.dart';
import 'package:vector_math/vector_math_64.dart';
import 'package:vslam/calibration/board.dart';
import 'package:vslam/calibration/camera_calibration.dart';
import 'package:vslam/calibration/imu_alignment.dart';
import 'package:vslam/calibration/session_alignment.dart';
import 'package:vslam/core/se3.dart';
import 'package:vslam/dataset/sensor_event.dart';
import 'package:vslam/slam/ar_pose.dart';
import 'package:vslam/slam/slam_service.dart';
import 'package:vslam/vision/camera_model.dart';
import 'package:vslam/widgets/calibration_panel.dart';

import 'imu_alignment_test.dart' show simulate;

void main() {
  late Directory tmp;
  setUp(() => tmp = Directory.systemTemp.createTempSync('vslam_calib_'));
  tearDown(() => tmp.deleteSync(recursive: true));

  testWidgets('calibration panel renders, shows the saved result, gates Calibrate', (tester) async {
    const k = CameraIntrinsics(fx: 720.5, fy: 720.9, cx: 478.0, cy: 360.2, k1: 0.05, k2: -0.1, width: 960, height: 720);
    const arkit = CameraIntrinsics(fx: 721.4, fy: 721.4, cx: 477.6, cy: 361.5, width: 960, height: 720);
    await tester.runAsync(() => CalibrationResult(
          intrinsics: k,
          rms: 0.21,
          views: 17,
          board: CalibrationBoard.standard,
          arkitMean: arkit,
          created: DateTime(2026, 11, 16),
        ).save(tmp));

    await tester.pumpWidget(ChangeNotifierProvider(
      create: (_) => SlamService(),
      child: MaterialApp(home: Scaffold(body: SingleChildScrollView(child: CalibrationPanel(documents: () async => tmp)))),
    ));
    for (var i = 0; i < 20 && find.textContaining('RMS 0.210 px').evaluate().isEmpty; i++) {
      await tester.runAsync(() => Future<void>.delayed(const Duration(milliseconds: 20)));
      await tester.pump();
    }
    expect(find.text('Camera calibration'), findsOneWidget);
    expect(find.textContaining('17 views, RMS 0.210 px'), findsOneWidget);
    expect(find.text('720.5'), findsOneWidget); // OpenCV fx
    expect(find.text('721.4'), findsWidgets); // ARKit fx
    expect(find.text('-0.9'), findsOneWidget); // fx difference
    // No views yet: Calibrate is disabled until 15 are captured.
    final calibrate = tester.widget<OutlinedButton>(find.widgetWithText(OutlinedButton, 'Calibrate'));
    expect(calibrate.onPressed, isNull);
    expect(find.textContaining('0 views (15–20)'), findsOneWidget);
    // Typing a square size shows the value captures will use.
    expect(find.text('Captures will use 25.0 mm'), findsOneWidget);
    await tester.enterText(find.byType(TextField), '20.6');
    await tester.pump();
    expect(find.text('Captures will use 20.6 mm'), findsOneWidget);
    await tester.tap(find.byTooltip('Done'));
    await tester.pump();
    expect(find.text('Captures will use 20.6 mm'), findsOneWidget);
  });

  test('alignment from ARKit-convention poses recovers the offset and rotation', () {
    final truth = appleCameraFromDevice.multiplied(so3Exp(Vector3(0.02, 0.01, -0.03)));
    final sim = simulate(rCI: truth, offsetUs: 15000, seed: 7);
    // Store the OpenCV-camera orientations the way ARKit reports them.
    final poses = [
      for (final r in sim.cam)
        () {
          final ar = vioToArPose(Pose3(r.rotation, Vector3.zero()));
          return PoseEvent(r.tUs, ArTrackingState.tracking, translation: ar.translation, rotation: ar.rotation);
        }(),
    ];
    final imu = [for (final g in sim.gyro) ImuEvent(g.tUs, ImuSensor.gyroscope, g.value)];
    final a = alignEvents(poses, imu);
    expect(a.offsetUs, closeTo(15000, 1500));
    expect(a.angleToDeg(truth), lessThan(0.5));
    // The saved file is valid JSON with the Apple comparison.
    final json = a.toJson();
    expect(json['angle_to_apple_convention_deg'] as double, closeTo(rotationAngle(so3Exp(Vector3(0.02, 0.01, -0.03))) * 180 / math.pi, 0.5));
  });
}
