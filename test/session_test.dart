import 'dart:async';

import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:vector_math/vector_math_64.dart';
import 'package:vslam/main.dart';
import 'package:vslam/services/imu_service.dart';
import 'package:vslam/services/session_controller.dart';
import 'package:vslam/slam/ar_pose.dart';
import 'package:vslam/slam/slam_service.dart';

class FakeImu {
  final acc = StreamController<ImuSample>.broadcast();
  final gyro = StreamController<ImuSample>.broadcast();
  DateTime t = DateTime(2026);

  ImuService service() => ImuService(accelerometer: () => acc.stream, gyroscope: () => gyro.stream);

  /// Queues [samples] stationary readings; they are delivered on the next
  /// microtask flush.
  void still(int samples) {
    for (var i = 0; i < samples; i++) {
      t = t.add(const Duration(milliseconds: 20));
      gyro.add(ImuSample(Vector3.zero(), t));
      acc.add(ImuSample(Vector3(0, 9.81, 0), t)); // upright portrait
    }
  }
}

void main() {
  TestWidgetsFlutterBinding.ensureInitialized();
  const channel = MethodChannel('vslam/ar');

  void mockAr(Map<String, Object?> responses) {
    TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.setMockMethodCallHandler(
      channel,
      (call) async => responses[call.method],
    );
  }

  tearDown(() {
    TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.setMockMethodCallHandler(channel, null);
  });

  test('session: start records AR poses and IMU samples, stop ends both', () async {
    mockAr({'checkAvailability': 'supported', 'hasCameraPermission': true});
    final imu = FakeImu();
    final imuService = imu.service();
    final slam = SlamService();
    await slam.initialize();
    expect(slam.isReady, isTrue);

    final session = SessionController(imu: imuService, slam: slam);
    session.start();
    expect(session.isRunning, isTrue);
    expect(slam.isRecording, isTrue);
    expect(imuService.isRunning, isTrue);

    imu.still(50);
    await pumpEventQueue();
    expect(imuService.accelerometerSamples, 50);
    expect(imuService.gyroscopeSamples, 50);
    expect(imuService.lastAccel!.value.y, closeTo(9.81, 1e-9));

    slam.handlePose(const ArPose(state: ArTrackingState.initializing));
    slam.handlePose(
      ArPose(state: ArTrackingState.tracking, translation: Vector3(5, 1.4, -2), rotation: Quaternion.identity()),
    );
    expect(session.poses, 2);
    expect(session.trackedPoses, 1);

    session.stop();
    expect(session.isRunning, isFalse);
    expect(slam.isRecording, isFalse);
    expect(imuService.isRunning, isFalse);

    // Poses after stop are not counted.
    slam.handlePose(const ArPose(state: ArTrackingState.initializing));
    expect(session.poses, 2);
    session.dispose();
  });

  test('SLAM: poses while not tracking are not recorded', () {
    final slam = SlamService()..startRecording();
    slam.handlePose(const ArPose(state: ArTrackingState.limited, reason: 'Too dark'));
    expect(slam.trajectory.isEmpty, isTrue);
    expect(slam.trackingReason, 'Too dark');
  });

  test('SLAM: missing native plugin reports unsupported', () async {
    final slam = SlamService();
    await slam.initialize();
    expect(slam.availability, ArAvailability.unsupported);
  });

  test('IMU: missing gyroscope surfaces an error', () async {
    final imu = FakeImu();
    final service = ImuService(
      accelerometer: () => imu.acc.stream,
      gyroscope: () => const Stream.empty(),
      sensorTimeout: const Duration(milliseconds: 50),
    );
    service.start();
    await Future<void>.delayed(const Duration(milliseconds: 100));
    expect(service.error, contains('gyroscope'));
    expect(service.isRunning, isFalse);
  });

  testWidgets('app renders and starts / stops a recording', (tester) async {
    mockAr({'checkAvailability': 'unsupported'});
    final imu = FakeImu();
    await tester.pumpWidget(VslamApp(imu: imu.service(), slam: SlamService()));
    await tester.pumpAndSettle();

    expect(find.text('VSLAM'), findsOneWidget);
    expect(find.text('SLAM not available'), findsOneWidget);
    expect(find.text('Start'), findsOneWidget);

    await tester.tap(find.text('Start'));
    await tester.pump();
    imu.still(10);
    await tester.pump();
    expect(find.text('Stop'), findsOneWidget);
    expect(find.text('streaming'), findsOneWidget);

    await tester.tap(find.text('Stop'));
    await tester.pump();
    expect(find.text('Start'), findsOneWidget);
    expect(find.text('idle'), findsOneWidget);

    await tester.tap(find.byTooltip('Diagnostics'));
    await tester.pumpAndSettle();
    expect(find.text('not supported on this device'), findsOneWidget);
  });
}
