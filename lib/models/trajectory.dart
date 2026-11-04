import 'dart:collection';

import 'package:vector_math/vector_math_64.dart';

import 'track_point.dart';

enum TrajectoryType { deadReckoning, slam }

/// An append-only, time-ordered list of [TrackPoint]s.
class Trajectory {
  Trajectory({required this.name, required this.type});

  final String name;
  final TrajectoryType type;

  final List<TrackPoint> _points = [];
  double _distance = 0;

  UnmodifiableListView<TrackPoint> get points => UnmodifiableListView(_points);
  bool get isEmpty => _points.isEmpty;
  int get length => _points.length;
  TrackPoint? get first => _points.isEmpty ? null : _points.first;
  TrackPoint? get last => _points.isEmpty ? null : _points.last;

  /// Path length in metres (3D).
  double get distance => _distance;

  void add(TrackPoint point) {
    if (_points.isNotEmpty) {
      _distance += _points.last.position.distanceTo(point.position);
    }
    _points.add(point);
  }

  void clear() {
    _points.clear();
    _distance = 0;
  }

  /// Position at time [t], linearly interpolated between neighbouring
  /// samples. Returns null outside the recorded time range (with
  /// [tolerance] of slack at either end).
  Vector3? positionAt(
    DateTime t, {
    Duration tolerance = const Duration(milliseconds: 250),
  }) {
    if (_points.isEmpty) return null;
    final first = _points.first.timestamp;
    final last = _points.last.timestamp;
    if (t.isBefore(first.subtract(tolerance)) || t.isAfter(last.add(tolerance))) {
      return null;
    }
    if (!t.isAfter(first)) return _points.first.position.clone();
    if (!t.isBefore(last)) return _points.last.position.clone();

    // Binary search for the first sample at or after t.
    var lo = 0;
    var hi = _points.length - 1;
    while (lo < hi) {
      final mid = (lo + hi) >> 1;
      if (_points[mid].timestamp.isBefore(t)) {
        lo = mid + 1;
      } else {
        hi = mid;
      }
    }
    final b = _points[lo];
    final a = _points[lo - 1];
    final span = b.timestamp.difference(a.timestamp).inMicroseconds;
    if (span <= 0) return b.position.clone();
    final f = t.difference(a.timestamp).inMicroseconds / span;
    return a.position + (b.position - a.position) * f;
  }

  Map<String, dynamic> toJson() => {
        'name': name,
        'type': type.name,
        'distance_m': _distance,
        'points': _points.map((p) => p.toJson()).toList(),
      };
}
