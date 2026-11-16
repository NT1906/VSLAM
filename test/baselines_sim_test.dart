import 'dart:io';
import 'dart:math' as math;

import 'package:flutter_test/flutter_test.dart';
import 'package:vector_math/vector_math_64.dart';
import 'package:vslam/core/se3.dart';
import 'package:vslam/dataset/session_files.dart';
import 'package:vslam/dataset/session_reader.dart';
import 'package:vslam/estimation/keyframe_selector.dart';
import 'package:vslam/estimation/session_baselines.dart';
import 'package:vslam/mapping/marker_map.dart';
import 'package:vslam/vision/session_detection.dart';

import 'support/simulator.dart';

/// Runs the WP4 pipeline exactly as the session screen does (files in,
/// files out) on a simulated recording.
Future<(RoomSimulation, BaselineRun, Directory)> runSim(Directory tmp, SimConfig config) async {
  final sim = RoomSimulation(config);
  final dir = await sim.writeSession(tmp);
  final data = await SessionReader.read(dir);
  final run = computeBaselines(data, await readDetections(dir), created: DateTime(2026, 11, 16));
  await writeBaselineResults(dir, run);
  return (sim, run, dir);
}

/// True camera pose of a keyframe, in W.
Pose3 truthInW(RoomSimulation sim, BaselineRun run, Keyframe kf) =>
    sim.roomInW(run.wTv) * sim.frames.firstWhere((f) => f.tUs == kf.tUs).truth;

void main() {
  late Directory tmp;
  setUp(() => tmp = Directory.systemTemp.createTempSync('vslam_wp4_'));
  tearDown(() => tmp.deleteSync(recursive: true));

  test('perfect odometry and IMU: both baselines follow the truth, markers land on the walls', () async {
    final (sim, run, dir) = await runSim(
      tmp,
      const SimConfig(accBias: (0.06, -0.04, 0.09), gyroBias: (0.004, -0.006, 0.002)),
    );

    // Keyframes: first tracked frame at 0.5 s, then the walk.
    expect(run.keyframes.first.tUs, RoomSimulation.usAt(0.5));
    expect(run.keyframes.first.reasons, {KeyframeReason.first});
    expect(run.trackedFrames, run.frames - 5); // 0.0–0.4 s initializing
    expect(run.trackingGaps, 0);

    // W: first keyframe at the origin, looking along +x, z up.
    final x0 = run.arkit.first;
    expect(x0.translation.length, lessThan(1e-12));
    expect(headingOfAxis(x0.rotation, Vector3(0, 0, 1)), closeTo(0, 1e-12));

    // Baseline 1 equals the truth (no drift) after the files' round trip.
    for (final kf in run.keyframes) {
      final t = truthInW(sim, run, kf);
      expect((run.arkit[kf.index].translation - t.translation).length, lessThan(1e-9));
      expect(rotationAngle(run.arkit[kf.index].rotation.transposed().multiplied(t.rotation)), lessThan(1e-9));
    }

    // Baseline 2: rest window found right at the first keyframe, biases
    // recovered, and with a noise-free IMU the dead reckoning stays on the
    // truth. This checks R_ci, the gravity sign and the specific-force
    // convention end to end: any of them wrong gives metres within seconds.
    final imu = run.imu!;
    expect(imu.restFound, isTrue);
    expect(imu.rest.t0Us, RoomSimulation.usAt(0.5));
    expect(imu.start.tUs, RoomSimulation.usAt(2.5));
    expect((imu.rest.ba - Vector3(0.06, -0.04, 0.09)).length, lessThan(1e-6));
    expect((imu.rest.bg - Vector3(0.004, -0.006, 0.002)).length, lessThan(1e-9));
    var maxErr = 0.0;
    for (final kf in run.keyframes) {
      final pose = imu.cameraPoses[kf.index];
      if (pose == null) continue;
      final err = (pose.translation - truthInW(sim, run, kf).translation).length;
      maxErr = math.max(maxErr, err);
      if (kf.tUs - imu.start.tUs <= 5e6) expect(err, lessThan(0.01));
    }
    expect(maxErr, lessThan(0.10));

    // First-sight map: all eight markers placed close to the truth.
    expect(run.map.ids, [0, 1, 2, 3, 4, 5, 6, 7]);
    expect(run.unplacedIds, isEmpty);
    final room = sim.roomInW(run.wTv);
    final posErr = <double>[];
    final normalErr = <double>[];
    for (final m in sim.markers) {
      final est = run.map[m.id]!;
      final truth = room * m.wTm;
      posErr.add((est.wTm.translation - truth.translation).length);
      final nEst = est.wTm.rotation.transformed(Vector3(0, 0, 1));
      final nTrue = truth.rotation.transformed(Vector3(0, 0, 1));
      normalErr.add(math.acos(nEst.dot(nTrue).clamp(-1.0, 1.0)) * 180 / math.pi);
      expect(est.keyframeObservations, greaterThan(0));
      expect(est.acceptedObservations, greaterThanOrEqualTo(1));
      expect(run.keyframes[est.firstKeyframe!].markerIds, contains(m.id));
    }
    final mean = posErr.reduce((a, b) => a + b) / posErr.length;
    final meanN = normalErr.reduce((a, b) => a + b) / normalErr.length;
    // ignore: avoid_print
    print('perfect odometry: ${run.keyframes.length} keyframes, path ${run.pathLengthM.toStringAsFixed(2)} m, '
        'IMU max ${(maxErr * 100).toStringAsFixed(2)} cm, marker position mean ${(mean * 100).toStringAsFixed(2)} cm '
        'max ${(posErr.reduce(math.max) * 100).toStringAsFixed(2)} cm, normal mean ${meanN.toStringAsFixed(2)}° '
        'max ${normalErr.reduce(math.max).toStringAsFixed(2)}°');
    expect(mean, lessThan(0.02));
    expect(posErr.reduce(math.max), lessThan(0.05));
    expect(meanN, lessThan(5));
    expect(normalErr.reduce(math.max), lessThan(10));

    // Files: one row per keyframe, read back to the same poses.
    final results = '${dir.path}/${SessionFiles.results}';
    final arkit = await readTrajectory(File('$results/$trajArkitFile'));
    expect(arkit, hasLength(run.keyframes.length));
    for (final row in arkit) {
      expect((row.wTc.translation - run.arkit[row.kf].translation).length, lessThan(1e-12));
      expect(row.tUs, run.keyframes[row.kf].tUs);
    }
    final imuRows = await readTrajectory(File('$results/$trajImuFile'));
    expect(imuRows.first.tUs, greaterThanOrEqualTo(imu.start.tUs));
    expect(imuRows, hasLength(imu.states.whereType<Object>().length));
    final saved = await MarkerMap.load(File('$results/$mapFile'));
    expect(saved.ids, run.map.ids);
    expect(saved.sourceSession, run.sessionName);
    expect(File('$results/$baselinesSummaryFile').existsSync(), isTrue);
  });

  test('drifting odometry: baseline 1 is the chained ARKit odometry, and the map inherits its drift', () async {
    final (sim, run, _) = await runSim(tmp, const SimConfig.drifting(seed: 3));
    // Chaining Z_k reproduces wTv · vTc_k of every keyframe.
    for (final kf in run.keyframes) {
      final direct = run.wTv * kf.vTc;
      expect((run.arkit[kf.index].translation - direct.translation).length, lessThan(1e-9));
    }
    // The drift is real: the end of the second lap is off the truth by the
    // accumulated heading error (here ~5°) and the 1 % scale.
    final last = run.keyframes.last;
    final endErr = (run.arkit.last.translation - truthInW(sim, run, last).translation).length;
    expect(endErr, greaterThan(0.05));
    // ignore: avoid_print
    print('drifting odometry: ${run.keyframes.length} keyframes, path ${run.pathLengthM.toStringAsFixed(2)} m, '
        'endpoint error ${(endErr * 100).toStringAsFixed(1)} cm, markers ${run.map.ids}');
    expect(run.map.ids, hasLength(8));
  });

  test('device-like IMU noise: dead reckoning drifts away (that is the point of baseline 2)', () async {
    final (sim, run, _) = await runSim(
      tmp,
      const SimConfig(accBias: (0.06, -0.04, 0.09), gyroBias: (0.004, -0.006, 0.002), accNoise: 0.02, gyroNoise: 0.002),
    );
    final imu = run.imu!;
    expect(imu.restFound, isTrue);
    final last = run.keyframes.last;
    final endErr = (imu.cameraPoses.last!.translation - truthInW(sim, run, last).translation).length;
    final drift = run.imuDriftRows;
    // ignore: avoid_print
    print('noisy IMU: ${[for (final (s, d) in drift) '${s.toStringAsFixed(1)} s: ${d.toStringAsFixed(3)} m'].join(', ')}');
    expect(endErr, greaterThan(0.3));
    expect(drift.first.$2, lessThan(drift.last.$2));
  });

  test('a stretch without ARKit tracking leaves a gap flag, not a keyframe', () async {
    final (_, run, _) = await runSim(tmp, const SimConfig(laps: 1, lostS: (10.0, 12.0)));
    final gapEdges = run.edges.where((e) => e.trackingGap).toList();
    expect(gapEdges, hasLength(1));
    final after = run.keyframes[gapEdges.single.to];
    final before = run.keyframes[gapEdges.single.from];
    expect(before.tUs, lessThan(RoomSimulation.usAt(10.0)));
    expect(after.tUs, greaterThanOrEqualTo(RoomSimulation.usAt(12.0)));
    for (final kf in run.keyframes) {
      expect(kf.tUs >= RoomSimulation.usAt(10.0) && kf.tUs < RoomSimulation.usAt(12.0), isFalse);
    }
  });

  test('the same session gives identical results twice', () async {
    final sim = RoomSimulation(const SimConfig.drifting(seed: 5, laps: 1));
    final dir = await sim.writeSession(tmp);
    final data = await SessionReader.read(dir);
    final det = await readDetections(dir);
    final a = computeBaselines(data, det, created: DateTime(2026));
    final b = computeBaselines(data, det, created: DateTime(2026));
    expect(a.map.encode(), b.map.encode());
    expect(a.toJson().toString(), b.toJson().toString());
  });
}
