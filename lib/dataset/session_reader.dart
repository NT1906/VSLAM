import 'dart:convert';
import 'dart:io';
import 'dart:typed_data';

import 'package:vector_math/vector_math_64.dart';

import '../vision/camera_model.dart';
import 'sensor_event.dart';
import 'session_files.dart';
import 'session_meta.dart';
import 'session_writer.dart' show trackingStateFromCsv;

/// One row of `frames.csv` with the frame it describes.
class FrameRecord {
  const FrameRecord(this.index, this.file, this.event);

  final int index;

  /// Path relative to the session folder; empty when no image was saved.
  final String file;
  final FrameEvent event;
}

/// A saved session loaded into memory (images stay on disk until asked for).
class SessionData {
  const SessionData({
    required this.directory,
    required this.meta,
    required this.frames,
    required this.poses,
    required this.imu,
  });

  final Directory directory;
  final SessionMeta meta;
  final List<FrameRecord> frames;
  final List<PoseEvent> poses;
  final List<ImuEvent> imu;

  int count(ImuSensor sensor) => imu.where((e) => e.sensor == sensor).length;

  /// Mean rate of [timesUs] over its own span, Hz (0 with fewer than two).
  static double rateHz(Iterable<int> timesUs) {
    final t = timesUs.toList();
    if (t.length < 2) return 0;
    final span = t.last - t.first;
    return span <= 0 ? 0 : (t.length - 1) * 1e6 / span;
  }

  double get frameHz => rateHz(frames.map((f) => f.event.tUs));
  double get poseHz => rateHz(poses.map((p) => p.tUs));
  double imuHz(ImuSensor sensor) => rateHz(imu.where((e) => e.sensor == sensor).map((e) => e.tUs));
}

/// Reads a session folder written by [SessionWriter].
abstract final class SessionReader {
  static Future<SessionMeta> readMeta(Directory dir) async {
    final text = await File('${dir.path}/${SessionFiles.meta}').readAsString();
    return SessionMeta.fromJson(jsonDecode(text) as Map<String, dynamic>);
  }

  static Future<SessionData> read(Directory dir) async {
    final meta = await readMeta(dir);
    final width = meta.imageWidth ?? 0;
    final height = meta.imageHeight ?? 0;

    final frames = <FrameRecord>[];
    for (final row in await _rows(dir, SessionFiles.frames, SessionFiles.framesHeader)) {
      final file = row[2];
      final path = '${dir.path}/$file';
      frames.add(
        FrameRecord(
          int.parse(row[0]),
          file,
          FrameEvent(
            int.parse(row[1]),
            intrinsics: CameraIntrinsics(
              fx: double.parse(row[3]),
              fy: double.parse(row[4]),
              cx: double.parse(row[5]),
              cy: double.parse(row[6]),
              width: width,
              height: height,
            ),
            state: trackingStateFromCsv(row[7]),
            translation: _vec(row, 8),
            rotation: _quat(row, 11),
            loadJpeg: () => _loadImage(file.isEmpty ? null : path),
          ),
        ),
      );
    }

    final poses = [
      for (final row in await _rows(dir, SessionFiles.poses, SessionFiles.posesHeader))
        PoseEvent(int.parse(row[0]), trackingStateFromCsv(row[8]), translation: _vec(row, 1), rotation: _quat(row, 4)),
    ];

    final imu = [
      for (final row in await _rows(dir, SessionFiles.imu, SessionFiles.imuHeader))
        ImuEvent(
          int.parse(row[0]),
          ImuSensor.fromCsv(row[1]),
          Vector3(double.parse(row[2]), double.parse(row[3]), double.parse(row[4])),
        ),
    ];

    return SessionData(directory: dir, meta: meta, frames: frames, poses: poses, imu: imu);
  }

  static Future<Uint8List?> _loadImage(String? path) async {
    if (path == null) return null;
    final f = File(path);
    return await f.exists() ? f.readAsBytes() : null;
  }

  static Future<List<List<String>>> _rows(Directory dir, String name, String header) async {
    final file = File('${dir.path}/$name');
    if (!await file.exists()) return const [];
    final lines = await file.readAsLines();
    if (lines.isEmpty) return const [];
    if (lines.first.trim() != header) {
      throw FormatException('$name: unexpected header "${lines.first}"');
    }
    final columns = header.split(',').length;
    final rows = <List<String>>[];
    for (var i = 1; i < lines.length; i++) {
      final line = lines[i];
      if (line.trim().isEmpty) continue;
      final row = line.split(',');
      if (row.length != columns) {
        throw FormatException('$name line ${i + 1}: expected $columns columns, got ${row.length}');
      }
      rows.add(row);
    }
    return rows;
  }

  static Vector3? _vec(List<String> row, int at) =>
      row[at].isEmpty ? null : Vector3(double.parse(row[at]), double.parse(row[at + 1]), double.parse(row[at + 2]));

  static Quaternion? _quat(List<String> row, int at) => row[at].isEmpty
      ? null
      : Quaternion(
          double.parse(row[at]),
          double.parse(row[at + 1]),
          double.parse(row[at + 2]),
          double.parse(row[at + 3]),
        );
}
