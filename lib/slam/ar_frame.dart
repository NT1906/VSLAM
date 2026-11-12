import 'dart:typed_data';

import 'package:vector_math/vector_math_64.dart';

import '../core/se3.dart';
import '../vision/camera_model.dart';
import 'ar_pose.dart';

/// A grayscale camera image from the AR session with the camera pose and
/// intrinsics of that exact image.
class ArFrame {
  const ArFrame({
    required this.time,
    required this.received,
    required this.width,
    required this.height,
    required this.gray,
    required this.intrinsics,
    required this.state,
    this.translation,
    this.rotation,
    this.jpeg,
    this.jpegQuality,
    this.sourceWidth,
    this.sourceHeight,
  });

  /// Capture time on the shared sensor clock (epoch µs; on iOS the same
  /// offset is applied to the CoreMotion samples, see `ArPlugin.swift`).
  final DateTime time;

  /// When Dart received the frame.
  final DateTime received;
  final int width;
  final int height;

  /// Luma plane, row-major, `width × height` bytes, in sensor orientation.
  final Uint8List gray;
  final CameraIntrinsics intrinsics;
  final ArTrackingState state;
  final Vector3? translation;
  final Quaternion? rotation;

  /// JPEG of [gray] encoded natively (iOS), for the dataset recorder.
  final Uint8List? jpeg;
  final double? jpegQuality;

  /// Size of the full camera image [gray] was box-filtered from.
  final int? sourceWidth;
  final int? sourceHeight;

  bool get tracking => state == ArTrackingState.tracking && translation != null && rotation != null;

  /// `vTc` for the estimator, or null when not tracking.
  Pose3? get vioPose => tracking ? arPoseToVio(translation!, rotation!) : null;

  /// Parses a `vslam/ar_frames` event:
  /// `{ts, w, h, y: bytes, k: [fx, fy, cx, cy], state, t?, q?, jpg?, jpgQ?,
  /// srcW?, srcH?}`.
  factory ArFrame.fromMap(Map<Object?, Object?> map, DateTime received) {
    final w = (map['w'] as num).toInt();
    final h = (map['h'] as num).toInt();
    final k = (map['k'] as List).map((e) => (e as num).toDouble()).toList();
    final pose = ArPose.fromMap(map);
    final ts = map['ts'] as num?;
    return ArFrame(
      time: ts == null ? received : DateTime.fromMicrosecondsSinceEpoch(ts.round()),
      received: received,
      width: w,
      height: h,
      gray: map['y'] as Uint8List,
      intrinsics: CameraIntrinsics(fx: k[0], fy: k[1], cx: k[2], cy: k[3], width: w, height: h),
      state: pose.state,
      translation: pose.translation,
      rotation: pose.rotation,
      jpeg: map['jpg'] as Uint8List?,
      jpegQuality: (map['jpgQ'] as num?)?.toDouble(),
      sourceWidth: (map['srcW'] as num?)?.toInt(),
      sourceHeight: (map['srcH'] as num?)?.toInt(),
    );
  }
}
