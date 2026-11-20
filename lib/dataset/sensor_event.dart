import 'dart:typed_data';

import 'package:vector_math/vector_math_64.dart';

import '../core/se3.dart';
import '../slam/ar_pose.dart';
import '../vision/camera_model.dart';

/// One timestamped measurement from the phone, as consumed by every
/// processing step after acquisition.
///
/// Live recording ([LiveSensorSource]) and replay of a saved session
/// ([ReplaySource]) both produce these, so estimation code is written once
/// against [SensorSource] and cannot tell the two apart.
///
/// Time: [tUs] is microseconds since the Unix epoch on the boot-based clock
/// shared by ARKit frames and CoreMotion samples (`ArPlugin.epochMicros`).
sealed class SensorEvent {
  const SensorEvent(this.tUs);

  final int tUs;
}

enum ImuSensor {
  accelerometer('acc'),
  gyroscope('gyro');

  const ImuSensor(this.csvName);

  /// Name used in the `sensor` column of `imu.csv`.
  final String csvName;

  static ImuSensor fromCsv(String name) =>
      values.firstWhere((s) => s.csvName == name, orElse: () => throw FormatException('Unknown IMU sensor "$name"'));
}

/// A raw accelerometer (m/s², specific force, +9.81 along the axis pointing
/// up at rest) or gyroscope (rad/s) sample in device axes: x right, y toward
/// the top of the screen in portrait, z out of the screen.
final class ImuEvent extends SensorEvent {
  const ImuEvent(super.tUs, this.sensor, this.value);

  final ImuSensor sensor;
  final Vector3 value;
}

/// A camera pose reported by ARKit, in ARKit's own convention (world +y up,
/// camera looking down its −z axis). Use [vioPose] for the estimator frame.
final class PoseEvent extends SensorEvent {
  const PoseEvent(super.tUs, this.state, {this.translation, this.rotation});

  final ArTrackingState state;
  final Vector3? translation;
  final Quaternion? rotation;

  bool get tracking => state == ArTrackingState.tracking && translation != null && rotation != null;

  /// `vTc` (OpenCV camera in the z-up VIO world), or null when not tracking.
  Pose3? get vioPose => tracking ? arPoseToVio(translation!, rotation!) : null;
}

/// One grayscale camera image (sensor orientation, landscape) with the ARKit
/// pose and intrinsics of that exact image.
///
/// The image is always available as JPEG bytes through [loadJpeg]: live
/// frames hold the bytes the native side encoded, replayed frames read the
/// saved file. Later processing decodes this JPEG in both cases, so live and
/// replayed runs see identical pixels. [gray] (raw luma) is only set for
/// live frames and is meant for display.
final class FrameEvent extends SensorEvent {
  const FrameEvent(
    super.tUs, {
    required this.intrinsics,
    required this.state,
    required this.loadJpeg,
    this.translation,
    this.rotation,
    this.gray,
    this.sourceWidth,
    this.sourceHeight,
    this.jpegQuality,
  });

  /// Pinhole intrinsics at this image's resolution (`intrinsics.width` ×
  /// `intrinsics.height`).
  final CameraIntrinsics intrinsics;
  final ArTrackingState state;
  final Vector3? translation;
  final Quaternion? rotation;

  /// JPEG bytes of the image, or null when none was captured.
  final Future<Uint8List?> Function() loadJpeg;
  final Uint8List? gray;

  /// Live frames only: size of ARKit's full `capturedImage` and the JPEG
  /// quality the native side used.
  final int? sourceWidth;
  final int? sourceHeight;
  final double? jpegQuality;

  int get width => intrinsics.width;
  int get height => intrinsics.height;

  bool get tracking => state == ArTrackingState.tracking && translation != null && rotation != null;

  /// `vTc` for this image, or null when ARKit was not tracking.
  Pose3? get vioPose => tracking ? arPoseToVio(translation!, rotation!) : null;
}

/// Anything that produces [SensorEvent]s: the live phone sensors or a saved
/// session. Processing code depends only on this interface.
abstract interface class SensorSource {
  /// Events of one run. Each sensor's events are in time order; across
  /// sensors a live source delivers them in arrival order and a replay source
  /// in timestamp order (see [ReplaySource]).
  Stream<SensorEvent> get events;
}
