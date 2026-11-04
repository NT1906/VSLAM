import 'package:vector_math/vector_math_64.dart';

/// One sample of a trajectory in the shared session frame.
///
/// Session frame: metres, z up, origin at the point where tracking started,
/// +y pointing the way the phone faced at the start.
class TrackPoint {
  const TrackPoint({
    required this.position,
    required this.timestamp,
    this.confidence,
    this.heading,
  });

  final Vector3 position;
  final DateTime timestamp;

  /// Tracking confidence in 0..1 (SLAM only).
  final double? confidence;

  /// Horizontal heading of the camera / phone forward direction, radians
  /// CCW from +x, when the estimator provides one.
  final double? heading;

  double get x => position.x;
  double get y => position.y;
  double get z => position.z;

  Map<String, dynamic> toJson() => {
        'x': x,
        'y': y,
        'z': z,
        't': timestamp.toIso8601String(),
        if (confidence != null) 'confidence': confidence,
        if (heading != null) 'heading': heading,
      };

  factory TrackPoint.fromJson(Map<String, dynamic> json) => TrackPoint(
        position: Vector3(
          (json['x'] as num).toDouble(),
          (json['y'] as num).toDouble(),
          (json['z'] as num).toDouble(),
        ),
        timestamp: DateTime.parse(json['t'] as String),
        confidence: (json['confidence'] as num?)?.toDouble(),
        heading: (json['heading'] as num?)?.toDouble(),
      );
}
