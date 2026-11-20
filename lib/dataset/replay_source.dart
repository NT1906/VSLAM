import 'sensor_event.dart';
import 'session_reader.dart';

/// Plays a saved session back as a [SensorSource].
///
/// Events come out in timestamp order. Ties (equal `t_us`) are broken by
/// sensor kind (IMU, then pose, then frame) and then by file order, so the
/// order is fully determined by the files and every replay of a session is
/// identical. Replay runs as fast as the consumer takes events; it does not
/// wait in real time.
class ReplaySource implements SensorSource {
  ReplaySource(this.session);

  final SessionData session;

  /// All events in replay order.
  List<SensorEvent> ordered() {
    final tagged = <(SensorEvent, int, int)>[
      for (final (i, e) in session.imu.indexed) (e, 0, i),
      for (final (i, e) in session.poses.indexed) (e, 1, i),
      for (final (i, f) in session.frames.indexed) (f.event, 2, i),
    ];
    tagged.sort((a, b) {
      final dt = a.$1.tUs.compareTo(b.$1.tUs);
      if (dt != 0) return dt;
      final dk = a.$2.compareTo(b.$2);
      return dk != 0 ? dk : a.$3.compareTo(b.$3);
    });
    return [for (final t in tagged) t.$1];
  }

  @override
  Stream<SensorEvent> get events => Stream.fromIterable(ordered());
}
