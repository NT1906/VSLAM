import 'dart:math' as math;
import 'dart:typed_data';

import 'package:vector_math/vector_math_64.dart';
import 'package:vslam/dataset/sensor_event.dart';
import 'package:vslam/dataset/session_meta.dart';
import 'package:vslam/slam/ar_pose.dart';
import 'package:vslam/vision/camera_model.dart';

/// A scripted sensor stream for dataset tests: IMU at 100 Hz (accelerometer
/// and gyroscope), ARKit poses at 30 Hz, frames at 10 Hz, with tracking
/// lost for a stretch and some frames without an image.
///
/// The returned list is in the order a [ReplaySource] must reproduce:
/// timestamp, then IMU < pose < frame for equal timestamps. Some timestamps
/// deliberately coincide across sensors to exercise that rule.
class SyntheticSession {
  SyntheticSession({int seed = 1, this.seconds = 3, this.dropFrameIndex}) {
    final rng = math.Random(seed);
    double r() => rng.nextDouble() * 2 - 1;
    const t0 = 1762850000000000; // µs since epoch
    final tagged = <(SensorEvent, int)>[];

    // IMU: 100 Hz; the gyroscope runs 3 ms behind the accelerometer.
    for (var i = 0; i < seconds * 100; i++) {
      final t = t0 + i * 10000;
      tagged.add((ImuEvent(t, ImuSensor.accelerometer, Vector3(r(), 9.81 + r() * 0.1, r())), 0));
      tagged.add((ImuEvent(t + 3000, ImuSensor.gyroscope, Vector3(r(), r(), r()) * 0.5), 0));
    }
    // Poses: 30 Hz on a 1/30 s grid (every third lands on an IMU time).
    for (var i = 0; i < seconds * 30; i++) {
      final t = t0 + (i * 1e6 / 30).round();
      final lost = i >= 30 && i < 45;
      tagged.add((
        PoseEvent(
          t,
          lost ? ArTrackingState.limited : ArTrackingState.tracking,
          translation: lost ? null : Vector3(0.01 * i + r() * 1e-3, 1.4, -0.02 * i),
          rotation: lost ? null : (Quaternion.axisAngle(Vector3(0, 1, 0), 0.01 * i)..normalize()),
        ),
        1,
      ));
    }
    // Frames: 10 Hz on the same grid as IMU and poses.
    for (var i = 0; i < seconds * 10; i++) {
      if (i == dropFrameIndex) continue;
      final t = t0 + i * 100000;
      final lost = i >= 10 && i < 15;
      final jpeg = i % 7 == 3 ? null : Uint8List.fromList(List.generate(64 + i, (j) => (i * 31 + j) & 0xff));
      tagged.add((
        FrameEvent(
          t,
          intrinsics: CameraIntrinsics(
            fx: 725.25 + i * 1e-3,
            fy: 725.5,
            cx: 479.75 + r() * 1e-3,
            cy: 359.5,
            width: 960,
            height: 720,
          ),
          state: lost ? ArTrackingState.limited : ArTrackingState.tracking,
          translation: lost ? null : Vector3(0.03 * i, 1.4, -0.06 * i + r() * 1e-4),
          rotation: lost ? null : (Quaternion.axisAngle(Vector3(0, 1, 0), 0.03 * i)..normalize()),
          loadJpeg: () => Future.value(jpeg),
          sourceWidth: 1920,
          sourceHeight: 1440,
          jpegQuality: 0.95,
        ),
        2,
      ));
      jpegs.add(jpeg);
    }
    tagged.sort((a, b) {
      final dt = a.$1.tUs.compareTo(b.$1.tUs);
      return dt != 0 ? dt : a.$2.compareTo(b.$2);
    });
    events = [for (final t in tagged) t.$1];
  }

  final int seconds;
  final int? dropFrameIndex;
  late final List<SensorEvent> events;

  /// JPEG bytes of each frame, in frame order.
  final List<Uint8List?> jpegs = [];

  Iterable<FrameEvent> get frames => events.whereType<FrameEvent>();
  Iterable<PoseEvent> get poses => events.whereType<PoseEvent>();
  Iterable<ImuEvent> get imu => events.whereType<ImuEvent>();

  static SessionMeta meta({DateTime? created}) => SessionMeta(
    settings: const RecordingSettings(
      label: 'Closed loop #1',
      experiment: ExperimentType.closedLoop,
      markerSideM: 0.152,
      notes: 'synthetic, 3 s',
    ),
    device: const DeviceInfo(model: 'iPhone15,2', os: 'iOS 26.1', appVersion: '0.6.0', appBuild: '2'),
    created: created ?? DateTime(2026, 11, 11, 14, 3, 22),
  );
}
