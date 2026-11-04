import 'package:flutter/foundation.dart';

import '../slam/ar_pose.dart';
import '../slam/slam_service.dart';
import 'imu_service.dart';

/// Start / stop of one recording session.
///
/// Camera (AR session) and IMU are acquired separately: each keeps its own
/// counters and timestamps. Time synchronisation and writing the run to a
/// dataset come later.
class SessionController extends ChangeNotifier {
  SessionController({required this.imu, required this.slam, this._clock = DateTime.now}) {
    imu.addListener(notifyListeners);
    slam.addListener(notifyListeners);
  }

  final ImuService imu;
  final SlamService slam;
  final DateTime Function() _clock;

  bool _running = false;
  DateTime? _startedAt;
  DateTime? _stoppedAt;
  int _poses = 0;
  int _trackedPoses = 0;

  bool get isRunning => _running;
  DateTime? get startedAt => _startedAt;

  /// Length of the current (or last) session.
  Duration get elapsed {
    final start = _startedAt;
    if (start == null) return Duration.zero;
    return (_stoppedAt ?? _clock()).difference(start);
  }

  /// AR pose events received during the session, and how many were tracked.
  int get poses => _poses;
  int get trackedPoses => _trackedPoses;

  bool get hasData => _startedAt != null;

  String? get lastError => imu.error ?? slam.error;

  void start() {
    if (_running) return;
    _running = true;
    _startedAt = _clock();
    _stoppedAt = null;
    _poses = 0;
    _trackedPoses = 0;
    slam.onPose = _onPose;
    if (slam.isReady) slam.startRecording();
    imu.start();
    notifyListeners();
  }

  void stop() {
    if (!_running) return;
    _running = false;
    _stoppedAt = _clock();
    slam.onPose = null;
    slam.stopRecording();
    imu.stop();
    notifyListeners();
  }

  void reset() {
    if (_running) return;
    _startedAt = null;
    _stoppedAt = null;
    _poses = 0;
    _trackedPoses = 0;
    slam.reset();
    imu.reset();
    notifyListeners();
  }

  void _onPose(ArPose pose, DateTime received) {
    _poses++;
    if (pose.state == ArTrackingState.tracking && pose.hasPose) _trackedPoses++;
  }

  @override
  void dispose() {
    slam.onPose = null;
    imu.removeListener(notifyListeners);
    slam.removeListener(notifyListeners);
    super.dispose();
  }
}
