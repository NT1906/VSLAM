import 'dart:io';

import 'package:flutter/foundation.dart';

import '../dataset/live_source.dart';
import '../dataset/session_meta.dart';
import '../dataset/session_recorder.dart';
import '../dataset/session_store.dart';
import '../dataset/session_writer.dart';
import '../slam/ar_pose.dart';
import '../slam/slam_service.dart';
import 'imu_service.dart';

/// Start / stop of one recording session.
///
/// While running, every AR frame, AR pose and IMU sample is written to a new
/// session folder through [SessionRecorder] (when a [SessionStore] is
/// given; without one, e.g. in tests of the UI alone, nothing is saved).
class SessionController extends ChangeNotifier {
  SessionController({required this.imu, required this.slam, this.store, this._clock = DateTime.now}) {
    imu.addListener(notifyListeners);
    slam.addListener(notifyListeners);
  }

  final ImuService imu;
  final SlamService slam;
  final SessionStore? store;
  final DateTime Function() _clock;

  bool _running = false;
  DateTime? _startedAt;
  DateTime? _stoppedAt;
  int _poses = 0;
  int _trackedPoses = 0;

  RecordingSettings _settings = const RecordingSettings();
  DeviceInfo _device = const DeviceInfo();
  SessionRecorder? _recorder;
  bool _saving = false;
  String? _saveError;
  SessionEntry? _lastSaved;

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

  String? get lastError => _saveError ?? imu.error ?? slam.error;

  /// Settings used for the next recording.
  RecordingSettings get settings => _settings;
  DeviceInfo get device => _device;

  /// True while the last recording's files are being finished.
  bool get isSaving => _saving;

  /// Frames written so far in the current recording.
  int get recordedFrames => _recorder?.writer.frameCount ?? 0;

  /// The most recently saved session.
  SessionEntry? get lastSaved => _lastSaved;

  /// Loads the saved recording settings and the device description.
  Future<void> load() async {
    final info = await slam.deviceInfo();
    if (info != null) _device = DeviceInfo.fromPlatform(info);
    final s = store;
    if (s != null) _settings = await s.loadSettings();
    notifyListeners();
  }

  Future<void> updateSettings(RecordingSettings settings) async {
    _settings = settings;
    notifyListeners();
    await store?.saveSettings(settings);
  }

  void start() {
    if (_running || _saving) return;
    _running = true;
    _startedAt = _clock();
    _stoppedAt = null;
    _poses = 0;
    _trackedPoses = 0;
    _saveError = null;
    _lastSaved = null;
    slam.onPose = _onPose;
    _startRecorder();
    if (slam.isReady) slam.startRecording();
    imu.start();
    notifyListeners();
  }

  void _startRecorder() {
    final s = store;
    if (s == null) return;
    try {
      final writer = SessionWriter.create(
        s.ensureSessionsDir(),
        SessionMeta(settings: _settings, device: _device, created: _startedAt!, requestedFrameHz: slam.frameRate),
      );
      _recorder = SessionRecorder(LiveSensorSource(slam: slam, imu: imu), writer);
    } on FileSystemException catch (e) {
      _saveError = 'Could not create the session folder: ${e.message}';
    }
  }

  /// Stops acquisition at once, then finishes writing the session files.
  /// Returns the saved session's metadata (null when nothing was saved).
  Future<SessionMeta?> stop() async {
    if (!_running) return null;
    _running = false;
    _stoppedAt = _clock();
    slam.onPose = null;
    slam.stopRecording();
    imu.stop();
    final recorder = _recorder;
    _recorder = null;
    _saving = recorder != null;
    notifyListeners();
    if (recorder == null) return null;
    try {
      final meta = await recorder.stop(settings: _settings);
      _lastSaved = SessionEntry(recorder.writer.directory, meta);
      final error = recorder.writer.lastError;
      if (error != null) _saveError = 'Some images could not be saved: $error';
      return meta;
    } on FileSystemException catch (e) {
      _saveError = 'Saving the session failed: ${e.message}';
      return null;
    } finally {
      _saving = false;
      notifyListeners();
    }
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
