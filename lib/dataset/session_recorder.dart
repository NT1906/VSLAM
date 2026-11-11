import 'dart:async';

import 'sensor_event.dart';
import 'session_meta.dart';
import 'session_writer.dart';

/// Connects a [SensorSource] to a [SessionWriter] for the length of one
/// recording.
class SessionRecorder {
  SessionRecorder(SensorSource source, this.writer) {
    _sub = source.events.listen(_onEvent);
  }

  final SessionWriter writer;
  late final StreamSubscription<SensorEvent> _sub;

  // Frames wait for their JPEG bytes; chaining keeps them in arrival order.
  Future<void> _frameChain = Future.value();

  void _onEvent(SensorEvent event) {
    switch (event) {
      case FrameEvent():
        _frameChain = _frameChain.then((_) async {
          try {
            writer.addFrame(event, await event.loadJpeg());
          } catch (_) {
            writer.addFrame(event, null);
          }
        });
      case PoseEvent():
        writer.addPose(event);
      case ImuEvent():
        writer.addImu(event);
    }
  }

  /// Stops listening, finishes every write and returns the final metadata.
  Future<SessionMeta> stop({RecordingSettings? settings}) async {
    await _sub.cancel();
    await _frameChain;
    return writer.close(settings: settings);
  }
}
