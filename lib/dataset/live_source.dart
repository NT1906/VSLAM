import 'dart:async';
import 'dart:typed_data';

import '../services/imu_service.dart';
import '../slam/ar_frame.dart';
import '../slam/ar_pose.dart';
import '../slam/slam_service.dart';
import 'sensor_event.dart';

/// The phone's sensors as a [SensorSource]: AR frames and poses from
/// [SlamService] and IMU samples from [ImuService], in arrival order.
class LiveSensorSource implements SensorSource {
  LiveSensorSource({required this.slam, required this.imu, this._clock = DateTime.now});

  final SlamService slam;
  final ImuService imu;
  final DateTime Function() _clock;

  @override
  Stream<SensorEvent> get events {
    final subs = <StreamSubscription<Object?>>[];
    late final StreamController<SensorEvent> out;
    out = StreamController<SensorEvent>(
      sync: true,
      onListen: () {
        subs
          ..add(slam.frames.listen((f) => out.add(frameEvent(f))))
          ..add(slam.poses.listen((p) => out.add(poseEvent(p, _clock()))))
          ..add(imu.samples.listen((s) => out.add(imuEvent(s.$1, s.$2))));
      },
      onCancel: () async {
        await Future.wait(subs.map((s) => s.cancel()));
        subs.clear();
      },
    );
    return out.stream;
  }

  static FrameEvent frameEvent(ArFrame f) {
    final Uint8List? jpeg = f.jpeg;
    return FrameEvent(
      f.time.microsecondsSinceEpoch,
      intrinsics: f.intrinsics,
      state: f.state,
      translation: f.translation,
      rotation: f.rotation,
      gray: f.gray,
      sourceWidth: f.sourceWidth,
      sourceHeight: f.sourceHeight,
      jpegQuality: f.jpegQuality,
      loadJpeg: () => Future.value(jpeg),
    );
  }

  /// [received] stands in for the capture time when the platform sent none.
  static PoseEvent poseEvent(ArPose p, DateTime received) =>
      PoseEvent((p.time ?? received).microsecondsSinceEpoch, p.state, translation: p.translation, rotation: p.rotation);

  static ImuEvent imuEvent(ImuSample s, bool gyro) => ImuEvent(
    s.timestamp.microsecondsSinceEpoch,
    gyro ? ImuSensor.gyroscope : ImuSensor.accelerometer,
    s.value.clone(),
  );
}
