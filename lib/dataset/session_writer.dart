import 'dart:convert';
import 'dart:io';
import 'dart:typed_data';

import 'package:vector_math/vector_math_64.dart';

import '../slam/ar_pose.dart';
import 'sensor_event.dart';
import 'session_files.dart';
import 'session_meta.dart';

/// Writes one recording to a session folder (layout in [SessionFiles]).
///
/// Nothing here blocks the UI: CSV rows go into buffered [IOSink]s and JPEG
/// files are written with asynchronous file I/O, which Dart runs on its I/O
/// thread pool, not on the UI isolate. JPEG encoding already happened on the
/// native side. [close] waits for every pending write before it returns.
class SessionWriter {
  SessionWriter._(this.directory, this._meta, this._frames, this._poses, this._imu);

  /// Creates `<sessionsRoot>/<date>_<time>_<label>/` and opens its files.
  static SessionWriter create(Directory sessionsRoot, SessionMeta meta) {
    final base = SessionFiles.folderName(meta.created, meta.settings.label);
    var dir = Directory('${sessionsRoot.path}/$base');
    for (var n = 2; dir.existsSync(); n++) {
      dir = Directory('${sessionsRoot.path}/${base}_$n');
    }
    Directory('${dir.path}/${SessionFiles.frameDir}').createSync(recursive: true);
    Directory('${dir.path}/${SessionFiles.results}').createSync();
    File('${dir.path}/${SessionFiles.meta}').writeAsStringSync(_encodeMeta(meta));

    IOSink open(String name, String header) => File('${dir.path}/$name').openWrite()..writeln(header);
    return SessionWriter._(
      dir,
      meta,
      open(SessionFiles.frames, SessionFiles.framesHeader),
      open(SessionFiles.poses, SessionFiles.posesHeader),
      open(SessionFiles.imu, SessionFiles.imuHeader),
    );
  }

  final Directory directory;
  SessionMeta _meta;
  final IOSink _frames;
  final IOSink _poses;
  final IOSink _imu;
  final Set<Future<void>> _pendingImages = {};
  bool _closed = false;

  int _frameCount = 0;
  int _framesWithoutImage = 0;
  int _frameGaps = 0;
  int? _lastFrameTUs;
  int _poseCount = 0;
  int _trackedPoses = 0;
  int _accCount = 0;
  int _gyroCount = 0;
  int? _startTUs;
  int? _endTUs;
  Object? _lastError;

  SessionMeta get meta => _meta;
  int get frameCount => _frameCount;
  int get pendingImageWrites => _pendingImages.length;

  /// The most recent file-system error, if any write failed.
  Object? get lastError => _lastError;

  SessionCounts get counts => SessionCounts(
    frames: _frameCount,
    framesWithoutImage: _framesWithoutImage,
    frameGaps: _frameGaps,
    poses: _poseCount,
    trackedPoses: _trackedPoses,
    accelerometer: _accCount,
    gyroscope: _gyroCount,
  );

  /// Saves one frame. [jpeg] null records the frame with an empty file name.
  void addFrame(FrameEvent frame, Uint8List? jpeg) {
    if (_closed) return;
    _seen(frame.tUs);
    final index = ++_frameCount;
    final last = _lastFrameTUs;
    if (last != null && frame.tUs - last > 1.5e6 / _meta.requestedFrameHz) _frameGaps++;
    _lastFrameTUs = frame.tUs;
    if (_meta.imageWidth == null) {
      _meta = _meta.copyWith(
        imageWidth: frame.width,
        imageHeight: frame.height,
        sourceWidth: frame.sourceWidth,
        sourceHeight: frame.sourceHeight,
        jpegQuality: frame.jpegQuality,
      );
    }

    var file = '';
    if (jpeg != null && jpeg.isNotEmpty) {
      file = SessionFiles.frameFile(index);
      late final Future<void> write;
      write = File('${directory.path}/$file')
          .writeAsBytes(jpeg, flush: false)
          .then<void>(
            (_) {},
            onError: (Object e) {
              _framesWithoutImage++;
              _lastError = e;
            },
          )
          .whenComplete(() => _pendingImages.remove(write));
      _pendingImages.add(write);
    } else {
      _framesWithoutImage++;
    }
    final k = frame.intrinsics;
    _frames.writeln(
      [
        index,
        frame.tUs,
        file,
        k.fx,
        k.fy,
        k.cx,
        k.cy,
        frame.state.name,
        ..._poseFields(frame.tracking ? frame.translation : null, frame.tracking ? frame.rotation : null),
      ].join(','),
    );
  }

  void addPose(PoseEvent pose) {
    if (_closed) return;
    _seen(pose.tUs);
    _poseCount++;
    if (pose.tracking) _trackedPoses++;
    _poses.writeln(
      [
        pose.tUs,
        ..._poseFields(pose.tracking ? pose.translation : null, pose.tracking ? pose.rotation : null),
        pose.state.name,
      ].join(','),
    );
  }

  void addImu(ImuEvent sample) {
    if (_closed) return;
    _seen(sample.tUs);
    if (sample.sensor == ImuSensor.accelerometer) {
      _accCount++;
    } else {
      _gyroCount++;
    }
    final v = sample.value;
    _imu.writeln('${sample.tUs},${sample.sensor.csvName},${v.x},${v.y},${v.z}');
  }

  /// Waits for all writes, closes the files and writes the final
  /// `meta.json` with counts and time span. [settings] replaces the
  /// recording settings when the person edited them during the run.
  Future<SessionMeta> close({RecordingSettings? settings}) async {
    if (_closed) return _meta;
    _closed = true;
    while (_pendingImages.isNotEmpty) {
      await Future.wait(_pendingImages.toList());
    }
    await Future.wait([_frames.close(), _poses.close(), _imu.close()]);
    _meta = _meta.copyWith(settings: settings, startTUs: _startTUs, endTUs: _endTUs, counts: counts);
    await File('${directory.path}/${SessionFiles.meta}').writeAsString(_encodeMeta(_meta));
    return _meta;
  }

  void _seen(int tUs) {
    if (_startTUs == null || tUs < _startTUs!) _startTUs = tUs;
    if (_endTUs == null || tUs > _endTUs!) _endTUs = tUs;
  }

  static List<Object> _poseFields(Vector3? t, Quaternion? q) =>
      t == null || q == null ? const ['', '', '', '', '', '', ''] : [t.x, t.y, t.z, q.x, q.y, q.z, q.w];

  static String _encodeMeta(SessionMeta meta) => const JsonEncoder.withIndent('  ').convert(meta.toJson());
}

/// Tracking state as written in the CSV files.
ArTrackingState trackingStateFromCsv(String name) => ArTrackingState.values.firstWhere(
  (s) => s.name == name,
  orElse: () => throw FormatException('Unknown tracking state "$name"'),
);
