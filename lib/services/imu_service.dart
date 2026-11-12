import 'dart:async';

import 'package:flutter/foundation.dart';
import 'package:flutter/services.dart';
import 'package:sensors_plus/sensors_plus.dart';
import 'package:vector_math/vector_math_64.dart';

/// One IMU reading in the device frame.
class ImuSample {
  const ImuSample(this.value, this.timestamp);

  final Vector3 value;
  final DateTime timestamp;
}

typedef ImuStreamFactory = Stream<ImuSample> Function();

/// Native CoreMotion stream (iOS, `ImuStream` in `ArPlugin.swift`): 100 Hz,
/// timestamps from `CMLogItem.timestamp` converted with the same offset as
/// the camera frames. Batches of `[sensor, t_us, x, y, z]`, sensor 0 =
/// accelerometer, 1 = gyroscope. Created once: both sensor streams share one
/// native subscription.
final Stream<(ImuSample, bool)> _nativeImu = const EventChannel('vslam/imu').receiveBroadcastStream().expand((event) {
  final v = event as Float64List;
  return [
    for (var i = 0; i + 4 < v.length; i += 5)
      (
        ImuSample(Vector3(v[i + 2], v[i + 3], v[i + 4]), DateTime.fromMicrosecondsSinceEpoch(v[i + 1].round())),
        v[i] == 1,
      ),
  ];
});

bool get _useNativeImu => !kIsWeb && defaultTargetPlatform == TargetPlatform.iOS;

/// sensors_plus (Android): `gameInterval` is 20 ms, i.e. 50 Hz.
Stream<ImuSample> _accelerometer() => _useNativeImu
    ? _nativeImu.where((s) => !s.$2).map((s) => s.$1)
    : accelerometerEventStream(samplingPeriod: SensorInterval.gameInterval)
          .map((e) => ImuSample(Vector3(e.x, e.y, e.z), e.timestamp));

Stream<ImuSample> _gyroscope() => _useNativeImu
    ? _nativeImu.where((s) => s.$2).map((s) => s.$1)
    : gyroscopeEventStream(samplingPeriod: SensorInterval.gameInterval)
          .map((e) => ImuSample(Vector3(e.x, e.y, e.z), e.timestamp));

/// Streams the phone's accelerometer and gyroscope, measures their sample
/// rates and keeps the latest readings for the status display.
///
/// On iOS the samples come from CoreMotion through `ArPlugin.swift` at
/// 100 Hz, timestamped on the same clock as the camera frames. Any residual
/// camera–IMU delay is estimated from recorded data in WP2.
///
/// Units: accelerometer m/s² as specific force (+9.81 along the axis that
/// points up at rest), gyroscope rad/s; device axes x right, y toward the
/// top of the screen, z out of the screen.
class ImuService extends ChangeNotifier {
  ImuService({
    ImuStreamFactory accelerometer = _accelerometer,
    ImuStreamFactory gyroscope = _gyroscope,
    this._clock = DateTime.now,
    this._sensorTimeout = const Duration(seconds: 4),
  }) : _accelerometerFactory = accelerometer,
       _gyroscopeFactory = gyroscope;

  final ImuStreamFactory _accelerometerFactory;
  final ImuStreamFactory _gyroscopeFactory;
  final DateTime Function() _clock;
  final Duration _sensorTimeout;

  StreamSubscription<ImuSample>? _accSub;
  StreamSubscription<ImuSample>? _gyroSub;
  Timer? _sensorWatchdog;
  DateTime? _lastNotify;
  String? _error;

  /// Every raw sample; `gyro` tells which sensor.
  void Function(ImuSample sample, bool gyro)? onSample;

  // Synchronous so listeners see samples in arrival order.
  final StreamController<(ImuSample, bool)> _samples = StreamController.broadcast(sync: true);

  /// Every raw sample as `(sample, isGyro)`, while running.
  Stream<(ImuSample, bool)> get samples => _samples.stream;

  int _accWindow = 0, _gyroWindow = 0;
  DateTime? _rateWindowStart;
  double _accHz = 0, _gyroHz = 0;
  int _accTotal = 0, _gyroTotal = 0;
  ImuSample? _lastAccel, _lastGyro;

  bool get isRunning => _accSub != null;
  String? get error => _error;

  /// Measured sample rates, Hz.
  double get accelerometerHz => _accHz;
  double get gyroscopeHz => _gyroHz;

  /// Samples received since [start].
  int get accelerometerSamples => _accTotal;
  int get gyroscopeSamples => _gyroTotal;

  /// Latest readings (m/s² and rad/s, device axes).
  ImuSample? get lastAccel => _lastAccel;
  ImuSample? get lastGyro => _lastGyro;

  void start() {
    if (isRunning) return;
    _error = null;
    _accTotal = 0;
    _gyroTotal = 0;
    _accWindow = 0;
    _gyroWindow = 0;
    _rateWindowStart = null;
    _accSub = _accelerometerFactory().listen((s) => _onSample(s, false), onError: _onSensorError);
    _gyroSub = _gyroscopeFactory().listen((s) => _onSample(s, true), onError: _onSensorError);
    _sensorWatchdog = Timer(_sensorTimeout, () {
      if (isRunning && _gyroTotal == 0) {
        _fail('No gyroscope data. This device may not have a gyroscope.');
      }
    });
    notifyListeners();
  }

  void stop() {
    _cancelSensors();
    notifyListeners();
  }

  void reset() {
    _cancelSensors();
    _accTotal = 0;
    _gyroTotal = 0;
    _accHz = 0;
    _gyroHz = 0;
    _lastAccel = null;
    _lastGyro = null;
    _error = null;
    notifyListeners();
  }

  void _cancelSensors() {
    _sensorWatchdog?.cancel();
    _sensorWatchdog = null;
    _accSub?.cancel();
    _gyroSub?.cancel();
    _accSub = null;
    _gyroSub = null;
  }

  void _fail(String message) {
    _cancelSensors();
    _error = message;
    notifyListeners();
  }

  void _onSensorError(Object error) => _fail('Sensor error: $error');

  void _onSample(ImuSample s, bool gyro) {
    onSample?.call(s, gyro);
    _samples.add((s, gyro));
    if (gyro) {
      _gyroWindow++;
      _gyroTotal++;
      _lastGyro = s;
    } else {
      _accWindow++;
      _accTotal++;
      _lastAccel = s;
    }
    final now = _clock();
    _rateWindowStart ??= now;
    final ms = now.difference(_rateWindowStart!).inMilliseconds;
    if (ms >= 2000) {
      _accHz = _accWindow * 1000 / ms;
      _gyroHz = _gyroWindow * 1000 / ms;
      _accWindow = 0;
      _gyroWindow = 0;
      _rateWindowStart = now;
    }
    // Sensors arrive at 50–100 Hz per stream; repaint at most ~10 Hz.
    if (_lastNotify == null || now.difference(_lastNotify!) >= const Duration(milliseconds: 100)) {
      _lastNotify = now;
      notifyListeners();
    }
  }

  @override
  void dispose() {
    _cancelSensors();
    _samples.close();
    super.dispose();
  }
}
