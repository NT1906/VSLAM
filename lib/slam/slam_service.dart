import 'dart:async';

import 'package:flutter/foundation.dart';
import 'package:flutter/services.dart';

import '../models/track_point.dart';
import '../models/trajectory.dart';
import '../vision/camera_model.dart';
import 'ar_frame.dart';
import 'ar_pose.dart';

enum ArAvailability { checking, supported, needsInstall, unsupported }

/// Talks to the native AR session (ARCore on Android, ARKit on iOS):
/// tracking state, the VIO camera trajectory in the session frame, and a
/// stream of grayscale camera frames.
class SlamService extends ChangeNotifier {
  SlamService({
    this._channel = const MethodChannel('vslam/ar'),
    this._poseChannel = const EventChannel('vslam/ar_pose'),
    this._frameChannel = const EventChannel('vslam/ar_frames'),
    this._clock = DateTime.now,
  });

  final MethodChannel _channel;
  final EventChannel _poseChannel;
  final EventChannel _frameChannel;
  final DateTime Function() _clock;
  final StreamController<ArFrame> _frames = StreamController.broadcast();
  // Synchronous so listeners see poses in exactly the order they arrive.
  final StreamController<ArPose> _poses = StreamController.broadcast(sync: true);
  StreamSubscription<dynamic>? _frameSub;
  double _frameRateHz = 10;
  int _frameCount = 0;
  DateTime? _frameWindowStart;
  double _measuredFrameHz = 0;
  CameraIntrinsics? _lastIntrinsics;

  /// Every pose event with its receive time (for dataset recording).
  void Function(ArPose pose, DateTime received)? onPose;

  /// Camera frames from the AR session (sensor orientation, luma only).
  Stream<ArFrame> get frames => _frames.stream;

  /// Every pose event from the AR session, in arrival order.
  Stream<ArPose> get poses => _poses.stream;

  /// Frames per second actually arriving from the platform.
  double get frameHz => _measuredFrameHz;
  double get frameRate => _frameRateHz;

  /// Size and AR SDK intrinsics of the most recent camera frame.
  CameraIntrinsics? get lastIntrinsics => _lastIntrinsics;

  /// Asks the platform to stream frames at up to [hz] (0 stops them).
  Future<void> setFrameRate(double hz) async {
    _frameRateHz = hz;
    try {
      await _channel.invokeMethod<void>('setFrameRate', {'hz': hz});
    } on MissingPluginException {
      // No native side (tests, desktop).
    } on PlatformException catch (e) {
      _error = e.message ?? e.code;
    }
  }

  final Trajectory trajectory = Trajectory(name: 'SLAM', type: TrajectoryType.slam);
  final SlamAligner _aligner = SlamAligner();

  ArAvailability _availability = ArAvailability.checking;
  bool _cameraGranted = false;
  bool _recording = false;
  ArPose? _lastPose;
  DateTime? _lastPoint;
  DateTime? _lastNotify;
  StreamSubscription<dynamic>? _poseSub;
  String? _error;

  ArAvailability get availability => _availability;
  bool get cameraGranted => _cameraGranted;

  /// True when the AR view can be shown.
  bool get isReady => _availability == ArAvailability.supported && _cameraGranted;
  bool get isRecording => _recording;
  ArTrackingState get trackingState => _lastPose?.state ?? ArTrackingState.initializing;
  String? get trackingReason => _lastPose?.reason;
  String? get error => _error;

  /// Checks AR support and camera permission. Safe to call repeatedly
  /// (e.g. after returning from the ARCore install screen).
  Future<void> initialize() async {
    _availability = await _checkAvailability();
    if (_availability == ArAvailability.supported) {
      _cameraGranted = await _invoke<bool>('hasCameraPermission') ?? false;
    }
    notifyListeners();
  }

  Future<ArAvailability> _checkAvailability() async {
    // ARCore's check can be transient right after app start; retry briefly.
    for (var attempt = 0; attempt < 10; attempt++) {
      final result = await _invoke<String>('checkAvailability');
      switch (result) {
        case 'supported':
          return ArAvailability.supported;
        case 'needsInstall':
          return ArAvailability.needsInstall;
        case 'unknown':
          await Future<void>.delayed(const Duration(milliseconds: 300));
          continue;
        default:
          return ArAvailability.unsupported;
      }
    }
    return ArAvailability.unsupported;
  }

  /// Opens the Google Play Services for AR install flow (Android).
  Future<void> requestInstall() async {
    final result = await _invoke<String>('requestInstall');
    if (result == 'installed') await initialize();
    // 'requested': the install UI is showing; initialize() runs again when
    // the app resumes.
  }

  Future<void> requestCameraPermission() async {
    _cameraGranted = await _invoke<bool>('requestCameraPermission') ?? false;
    notifyListeners();
  }

  /// Hardware model, OS and app version from the native side, or null
  /// when there is no native side (tests, desktop).
  Future<Map<Object?, Object?>?> deviceInfo() => _invoke<Map<Object?, Object?>>('deviceInfo');

  Future<T?> _invoke<T>(String method) async {
    try {
      return await _channel.invokeMethod<T>(method);
    } on MissingPluginException {
      return null;
    } on PlatformException catch (e) {
      _error = e.message ?? e.code;
      return null;
    }
  }

  /// Subscribes to native pose and frame events. Called when the AR view
  /// is created.
  void attach() {
    _poseSub ??= _poseChannel.receiveBroadcastStream().listen(
      (event) => _onPose(ArPose.fromMap(event as Map<Object?, Object?>)),
      onError: (Object e) {
        _error = e is PlatformException ? (e.message ?? e.code) : '$e';
        notifyListeners();
      },
    );
    _frameSub ??= _frameChannel.receiveBroadcastStream().listen(
      (event) => handleFrame(ArFrame.fromMap(event as Map<Object?, Object?>, _clock())),
      onError: (Object e) {
        _error = e is PlatformException ? (e.message ?? e.code) : '$e';
      },
    );
    setFrameRate(_frameRateHz);
  }

  /// Unsubscribes from native events. Called when the AR view is disposed.
  void detach() {
    _poseSub?.cancel();
    _poseSub = null;
    _frameSub?.cancel();
    _frameSub = null;
    _lastPose = null;
  }

  @visibleForTesting
  void handleFrame(ArFrame frame) {
    final now = frame.received;
    _frameWindowStart ??= now;
    _frameCount++;
    _lastIntrinsics = frame.intrinsics;
    final window = now.difference(_frameWindowStart!).inMilliseconds;
    if (window >= 2000) {
      _measuredFrameHz = _frameCount * 1000 / window;
      _frameCount = 0;
      _frameWindowStart = now;
    }
    _frames.add(frame);
  }

  /// Starts a new recording; its origin is the next tracked pose.
  void startRecording() {
    trajectory.clear();
    _aligner.reset();
    _lastPoint = null;
    _recording = true;
    notifyListeners();
  }

  void stopRecording() {
    _recording = false;
    notifyListeners();
  }

  void reset() {
    _recording = false;
    trajectory.clear();
    _aligner.reset();
    notifyListeners();
  }

  @visibleForTesting
  void handlePose(ArPose pose) => _onPose(pose);

  void _onPose(ArPose pose) {
    final previousState = _lastPose?.state;
    _lastPose = pose;
    _error = null;
    final now = _clock();
    onPose?.call(pose, now);
    _poses.add(pose);

    if (_recording && pose.state == ArTrackingState.tracking && pose.hasPose) {
      if (!_aligner.isAnchored) _aligner.anchor(pose.translation!, pose.rotation!);
      if (_lastPoint == null || now.difference(_lastPoint!) >= const Duration(milliseconds: 100)) {
        _lastPoint = now;
        trajectory.add(TrackPoint(position: _aligner.toSession(pose.translation!), timestamp: now, confidence: 1));
      }
    }

    if (pose.state != previousState ||
        _lastNotify == null ||
        now.difference(_lastNotify!) >= const Duration(milliseconds: 100)) {
      _lastNotify = now;
      notifyListeners();
    }
  }

  @override
  void dispose() {
    detach();
    _frames.close();
    _poses.close();
    super.dispose();
  }
}
