import 'dart:convert';
import 'dart:io';

import '../dataset/sensor_event.dart';
import '../dataset/session_files.dart';
import '../dataset/session_reader.dart';
import 'imu_alignment.dart';

/// File in `results/` that holds the camera-IMU alignment of a session.
const imuAlignmentFile = 'imu_alignment.json';

/// Runs [estimateImuAlignment] on a saved session: tracked ARKit poses
/// (`arkit_poses.csv`, ~30 Hz, OpenCV camera axes) against the gyroscope.
ImuAlignment alignSession(SessionData data) => alignEvents(data.poses, data.imu);

/// [alignSession] on raw events: ARKit poses (ARKit camera convention,
/// converted to the OpenCV camera here) and IMU samples.
ImuAlignment alignEvents(List<PoseEvent> poses, List<ImuEvent> imu) {
  final cam = [
    for (final p in poses)
      if (p.vioPose case final pose?) TimedRotation(p.tUs, pose.rotation),
  ];
  final gyro = [
    for (final e in imu)
      if (e.sensor == ImuSensor.gyroscope) TimedVector(e.tUs, e.value),
  ];
  return estimateImuAlignment(cam, gyro);
}

/// Writes [result] to `results/imu_alignment.json` and returns the file.
Future<File> saveAlignment(Directory session, ImuAlignment result) async {
  final dir = Directory('${session.path}/${SessionFiles.results}');
  await dir.create(recursive: true);
  final file = File('${dir.path}/$imuAlignmentFile');
  await file.writeAsString(const JsonEncoder.withIndent('  ').convert(result.toJson()));
  return file;
}
