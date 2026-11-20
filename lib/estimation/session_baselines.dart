import 'dart:convert';
import 'dart:io';

import 'package:vector_math/vector_math_64.dart';

import '../calibration/camera_calibration.dart';
import '../calibration/imu_alignment.dart';
import '../core/se3.dart';
import '../dataset/sensor_event.dart';
import '../dataset/session_files.dart';
import '../dataset/session_reader.dart';
import '../mapping/marker_map.dart';
import '../vision/aruco_detector.dart';
import '../vision/marker_observation.dart';
import '../vision/session_detection.dart';
import 'baselines.dart';
import 'config.dart';
import 'imu_propagator.dart';
import 'keyframe_selector.dart';

/// `results/traj_arkit.csv`: baseline 1 at every keyframe.
const trajArkitFile = 'traj_arkit.csv';

/// `results/traj_imu.csv`: baseline 2 at every keyframe after its start.
const trajImuFile = 'traj_imu.csv';

/// `results/map.json`: the first marker map.
const mapFile = 'map.json';

/// `results/baselines.json`: summary of the WP4 run.
const baselinesSummaryFile = 'baselines.json';

const trajArkitHeader = 'kf,frame_index,t_us,tx,ty,tz,qx,qy,qz,qw,reasons,marker_ids,tracking_gap';
const trajImuHeader = 'kf,t_us,tx,ty,tz,qx,qy,qz,qw,vx,vy,vz,dist_to_arkit_m';

/// Everything WP4 computes for one session.
class BaselineRun {
  const BaselineRun({
    required this.sessionName,
    required this.config,
    required this.keyframes,
    required this.edges,
    required this.wTv,
    required this.arkit,
    required this.imu,
    required this.imuMissing,
    required this.map,
    required this.detectedIds,
    required this.frames,
    required this.trackedFrames,
  });

  final String sessionName;
  final SlamConfig config;
  final List<Keyframe> keyframes;
  final List<OdometryEdge> edges;

  /// The map frame W seen from ARKit's frame V.
  final Pose3 wTv;

  /// Baseline 1: `wTc_k` per keyframe.
  final List<Pose3> arkit;

  /// Baseline 2, or null when the session has no IMU samples.
  final ImuBaseline? imu;
  final String? imuMissing;
  final MarkerMap map;

  /// Every marker ID in the detections file (any status).
  final List<int> detectedIds;
  final int frames;
  final int trackedFrames;

  double get pathLengthM => pathLength(arkit);

  /// Markers detected but never placed (no accepted detection in a
  /// keyframe).
  List<int> get unplacedIds => [for (final id in detectedIds) if (map[id] == null) id];

  Map<KeyframeReason, int> get reasonCounts => {
        for (final r in KeyframeReason.values) r: keyframes.where((k) => k.reasons.contains(r)).length,
      };

  int get trackingGaps => edges.where((e) => e.trackingGap).length;

  List<(double, double)> get imuDriftRows => imu == null ? const [] : imuDrift(keyframes, arkit, imu!);

  Map<String, dynamic> toJson() {
    final i = imu;
    return {
      'session': sessionName,
      'frames': frames,
      'tracked_frames': trackedFrames,
      'keyframes': keyframes.length,
      'keyframe_reasons': {for (final e in reasonCounts.entries) e.key.code: e.value},
      'tracking_gaps': trackingGaps,
      'keyframe_config': {
        'translation_m': config.keyframeTranslationM,
        'rotation_deg': config.keyframeRotationDeg,
        'marker_gap_s': config.keyframeMarkerGapS,
        'marker_statuses': config.keyframeMarkerStatuses.toList()..sort(),
      },
      'frame_w': mappingFrameDescription,
      'w_T_v': wTv.toJson(),
      'arkit_path_length_m': pathLengthM,
      'imu': i == null
          ? {'missing': imuMissing}
          : {
              'rest_found': i.restFound,
              'rest': i.rest.toJson(),
              'start_t_us': i.start.tUs,
              'start_after_first_keyframe_s': (i.start.tUs - keyframes.first.tUs) / 1e6,
              'distance_to_arkit': [
                for (final (s, d) in imuDriftRows) {'elapsed_s': s, 'm': d},
              ],
              'r_ci': 'appleCameraFromDevice (WP2: measured 0.10 deg from it), time offset 0',
              'gravity_m_s2': config.gravity,
            },
      'markers_placed': map.ids,
      'markers_unplaced': unplacedIds,
    };
  }
}

/// Runs keyframe selection, baseline 1, baseline 2 and the first marker map
/// on a loaded session and its detections ([readDetections]).
BaselineRun computeBaselines(
  SessionData data,
  List<(int, MarkerObservation)> detections, {
  SlamConfig config = const SlamConfig(),
  Matrix3? rCI,
  DateTime? created,
}) {
  final rotation = rCI ?? appleCameraFromDevice;
  final byTime = <int, List<MarkerObservation>>{};
  for (final (t, m) in detections) {
    (byTime[t] ??= []).add(m);
  }

  // Keyframes.
  final selector = KeyframeSelector(config);
  var tracked = 0;
  for (final (i, f) in data.frames.indexed) {
    final vTc = f.event.vioPose;
    if (vTc != null) tracked++;
    final ids = {
      for (final m in byTime[f.event.tUs] ?? const <MarkerObservation>[])
        if (config.keyframeMarkerStatuses.contains(m.statusCode)) m.id,
    }.toList()
      ..sort();
    selector.offer(frameIndex: i, tUs: f.event.tUs, vTc: vTc, markerIds: ids);
  }
  final keyframes = selector.keyframes;
  if (keyframes.isEmpty) {
    throw StateError('No frame with ARKit tracking: there is no trajectory to build. '
        'Record in a textured scene and wait for "SLAM tracking" before moving.');
  }
  final edges = odometryEdges(keyframes);

  // Baseline 1.
  final wTv = worldFromVio(keyframes.first.vTc);
  final arkit = chainOdometry(wTv * keyframes.first.vTc, edges);

  // Baseline 2.
  final acc = <int>[];
  final accV = <Vector3>[];
  final gyro = <int>[];
  final gyroV = <Vector3>[];
  for (final e in data.imu) {
    if (e.sensor == ImuSensor.accelerometer) {
      acc.add(e.tUs);
      accV.add(e.value);
    } else {
      gyro.add(e.tUs);
      gyroV.add(e.value);
    }
  }
  ImuBaseline? imu;
  String? imuMissing;
  if (acc.length < 2 || gyro.length < 2) {
    imuMissing = 'no IMU samples';
  } else {
    final poses = TimedPoses([
      for (final p in data.poses)
        if (p.vioPose case final v?) (p.tUs, v),
      for (final f in data.frames)
        if (f.event.vioPose case final v?) (f.event.tUs, v),
    ]);
    imu = runImuBaseline(
      acc: ImuSeries(acc, accV),
      gyro: ImuSeries(gyro, gyroV),
      keyframes: keyframes,
      poseAt: (t) => wTv * poses.nearest(t),
      rCI: rotation,
      config: config,
    );
  }

  final name = data.directory.uri.pathSegments.where((s) => s.isNotEmpty).last;
  final map = buildFirstSightMap(
    keyframes: keyframes,
    poses: arkit,
    detections: byTime,
    markerSideM: data.meta.settings.markerSideM,
    usableStatuses: config.keyframeMarkerStatuses,
    sourceSession: name,
    created: created ?? DateTime.now(),
  );
  return BaselineRun(
    sessionName: name,
    config: config,
    keyframes: keyframes,
    edges: edges,
    wTv: wTv,
    arkit: arkit,
    imu: imu,
    imuMissing: imuMissing,
    map: map,
    detectedIds: ({for (final (_, m) in detections) m.id}.toList()..sort()),
    frames: data.frames.length,
    trackedFrames: tracked,
  );
}

List<Object> _pose(Pose3 p) {
  final q = p.quaternion;
  return [p.translation.x, p.translation.y, p.translation.z, q.x, q.y, q.z, q.w];
}

/// Writes `traj_arkit.csv`, `traj_imu.csv`, `map.json` and
/// `baselines.json` into the session's `results/`.
Future<void> writeBaselineResults(Directory session, BaselineRun run) async {
  final dir = Directory('${session.path}/${SessionFiles.results}');
  await dir.create(recursive: true);

  final arkit = StringBuffer()..writeln(trajArkitHeader);
  for (final kf in run.keyframes) {
    arkit.writeln([
      kf.index,
      kf.frameIndex,
      kf.tUs,
      ..._pose(run.arkit[kf.index]),
      [for (final r in KeyframeReason.values) if (kf.reasons.contains(r)) r.code].join('|'),
      kf.markerIds.join('|'),
      kf.trackingGapBefore ? 1 : 0,
    ].join(','));
  }
  await File('${dir.path}/$trajArkitFile').writeAsString(arkit.toString());

  final imu = StringBuffer()..writeln(trajImuHeader);
  if (run.imu case final i?) {
    for (final kf in run.keyframes) {
      final s = i.states[kf.index];
      final pose = i.cameraPoses[kf.index];
      if (s == null || pose == null) continue;
      imu.writeln([
        kf.index,
        kf.tUs,
        ..._pose(pose),
        s.v.x,
        s.v.y,
        s.v.z,
        (pose.translation - run.arkit[kf.index].translation).length,
      ].join(','));
    }
  }
  await File('${dir.path}/$trajImuFile').writeAsString(imu.toString());

  await run.map.save(File('${dir.path}/$mapFile'));
  await File('${dir.path}/$baselinesSummaryFile')
      .writeAsString(const JsonEncoder.withIndent('  ').convert(run.toJson()));
}

/// One row of a trajectory file: keyframe index, time and camera pose `wTc`.
class TrajectoryRow {
  const TrajectoryRow(this.kf, this.tUs, this.wTc);

  final int kf;
  final int tUs;
  final Pose3 wTc;
}

/// Reads `traj_arkit.csv` or `traj_imu.csv` (columns found by name).
Future<List<TrajectoryRow>> readTrajectory(File file) async {
  final lines = await file.readAsLines();
  if (lines.isEmpty) throw FormatException('${file.path}: empty');
  final cols = lines.first.split(',');
  int col(String name) {
    final i = cols.indexOf(name);
    if (i < 0) throw FormatException('${file.path}: no column "$name"');
    return i;
  }

  final kf = col('kf'), t = col('t_us'), tx = col('tx'), qx = col('qx');
  return [
    for (final line in lines.skip(1))
      if (line.isNotEmpty)
        () {
          final f = line.split(',');
          double d(int i) => double.parse(f[i]);
          return TrajectoryRow(
            int.parse(f[kf]),
            int.parse(f[t]),
            Pose3.fromQuaternion(Quaternion(d(qx), d(qx + 1), d(qx + 2), d(qx + 3)), Vector3(d(tx), d(tx + 1), d(tx + 2))),
          );
        }(),
  ];
}

/// The session-screen action: detects markers first when
/// `results/detections.csv` does not exist yet, then computes and writes
/// the WP4 results. [onProgress] reports detection progress (0–1).
Future<BaselineRun> processSessionBaselines(
  SessionData data, {
  CalibrationResult? calibration,
  MarkerDetector detector = const ArucoMarkerDetector(),
  SlamConfig config = const SlamConfig(),
  void Function(double fraction)? onProgress,
}) async {
  final csv = File('${data.directory.path}/${SessionFiles.results}/$detectionsFile');
  if (!await csv.exists()) {
    await detectSession(
      data,
      calibration: calibration,
      detector: detector,
      onProgress: (done, total) => onProgress?.call(done / total),
    );
  }
  final run = computeBaselines(data, await readDetections(data.directory), config: config);
  await writeBaselineResults(data.directory, run);
  return run;
}
