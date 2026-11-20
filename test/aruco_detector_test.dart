import 'dart:io';
import 'dart:math' as math;
import 'dart:typed_data';

import 'package:flutter_test/flutter_test.dart';
import 'package:vector_math/vector_math_64.dart';
import 'package:vslam/calibration/board.dart';
import 'package:vslam/calibration/camera_calibration.dart';
import 'package:vslam/core/se3.dart';
import 'package:vslam/dataset/sensor_event.dart';
import 'package:vslam/dataset/session_meta.dart';
import 'package:vslam/dataset/session_reader.dart';
import 'package:vslam/dataset/session_writer.dart';
import 'package:vslam/estimation/config.dart';
import 'package:vslam/slam/ar_pose.dart';
import 'package:vslam/vision/aruco_detector.dart';
import 'package:vslam/vision/camera_model.dart';
import 'package:vslam/vision/marker_observation.dart';
import 'package:vslam/vision/marker_sheet.dart';
import 'package:vslam/vision/session_detection.dart';

import 'support/marker_render.dart';

/// The phone's WP2 calibration (check 2.2), 960 × 720.
const calibrated = CameraIntrinsics(
  fx: 716.5,
  fy: 714.8,
  cx: 473.0,
  cy: 357.4,
  k1: 0.0715,
  k2: -0.1411,
  width: 960,
  height: 720,
);

/// ARKit's mean intrinsics from the same check (no distortion).
const arkit = CameraIntrinsics(fx: 721.3, fy: 721.3, cx: 478.5, cy: 360.7, width: 960, height: 720);

double rotationErrorDeg(Pose3 a, Pose3 b) => rotationAngle(a.rotation.transposed().multiplied(b.rotation)) * 180 / math.pi;

FrameDetections detect(List<SceneMarker> scene, CameraIntrinsics k, {double? sideM}) =>
    detectMarkersSync(renderMarkers(scene, k), k, sideM ?? scene.first.sideM, const SlamConfig());

/// Checks ID, corners (spec: under 0.5 px) and distance (under 1 %) of a
/// single rendered marker, and returns the observation.
MarkerObservation expectFound(SceneMarker m, CameraIntrinsics k) {
  final found = detect([m], k);
  expect(found.markers.map((o) => o.id), [m.id]);
  final o = found.markers.single;
  final truth = m.corners(k);
  for (var j = 0; j < 4; j++) {
    expect((o.corners[j] - truth[j]).length, lessThan(0.5), reason: 'corner $j: ${o.corners[j]} vs ${truth[j]}');
  }
  final d = m.cTm.translation.length;
  expect((o.distanceM! - d).abs() / d, lessThan(0.01), reason: 'distance ${o.distanceM} vs $d');
  return o;
}

void main() {
  group('rendered markers through the calibrated camera', () {
    // Spec section 6 "Detector tests": render, detect, check ID, corners
    // (under 0.5 px) and the recovered pose.
    final accepted = <String, SceneMarker>{
      '1 m, turned 30°, tilted 10°, rolled 15°': SceneMarker(3, 0.15, markerPose(1.0, yawDeg: 30, pitchDeg: 10, rollDeg: 15)),
      '0.6 m, turned 45°, off-centre': SceneMarker(0, 0.15, markerPose(0.6, yawDeg: -45, ox: 0.25, oy: -0.15)),
      '1.5 m, tilted 40°, near the image corner': SceneMarker(9, 0.15, markerPose(1.5, pitchDeg: 40, ox: -0.45, oy: 0.3)),
      '0.4 m, rolled 170°, turned 35°': SceneMarker(7, 0.15, markerPose(0.4, rollDeg: 170, yawDeg: 35)),
    };
    for (final MapEntry(key: name, value: m) in accepted.entries) {
      test('$name: accepted, corners < 0.5 px, distance < 1 %, rotation < 2°', () {
        final o = expectFound(m, calibrated);
        expect(o.rejection, isNull, reason: 'rms ${o.best?.rmsPx}, alt ${o.alternative?.rmsPx}, angle ${o.viewAngleDeg}');
        expect(o.solutions, hasLength(2));
        expect(rotationErrorDeg(o.cTm!, m.cTm), lessThan(2));
        expect((o.cTm!.translation - m.cTm.translation).length / m.cTm.translation.length, lessThan(0.01));
        expect(o.best!.rmsPx, lessThan(0.5));
      });
    }

    test('with ARKit intrinsics (no distortion) the pose is recovered too', () {
      final m = SceneMarker(5, 0.15, markerPose(1.2, yawDeg: 35, pitchDeg: -15));
      final o = expectFound(m, arkit);
      expect(o.accepted, isTrue);
      expect(rotationErrorDeg(o.cTm!, m.cTm), lessThan(2));
    });

    test('two markers in one image: both IDs, each with its own pose', () {
      final a = SceneMarker(3, 0.15, markerPose(1.0, yawDeg: 30, ox: -0.25));
      final b = SceneMarker(7, 0.15, markerPose(1.4, yawDeg: -40, ox: 0.25));
      final found = detect([a, b], calibrated);
      final byId = {for (final o in found.markers) o.id: o};
      expect(byId.keys.toSet(), {3, 7});
      for (final m in [a, b]) {
        final o = byId[m.id]!;
        final truth = m.corners(calibrated);
        for (var j = 0; j < 4; j++) {
          expect((o.corners[j] - truth[j]).length, lessThan(0.5));
        }
        expect(o.accepted, isTrue);
        expect((o.cTm!.translation - m.cTm.translation).length / m.cTm.translation.length, lessThan(0.01));
      }
    });

    test('an image without markers gives no detections', () {
      final found = detectMarkersSync(renderMarkers(const [], calibrated), calibrated, 0.15, const SlamConfig());
      expect(found.markers, isEmpty);
      expect(found.detectMs, greaterThan(0));
    });
  });

  group('rejection rules (spec 4.4) on rendered markers', () {
    test('head-on at 2.5 m: the two IPPE poses fit equally well → ambiguous', () {
      final m = SceneMarker(2, 0.15, markerPose(2.5, yawDeg: 3));
      final o = expectFound(m, calibrated);
      expect(o.rejection, MarkerRejection.ambiguous, reason: 'rms ${o.best!.rmsPx} vs ${o.alternative!.rmsPx}');
    });

    test('a marker under 20 px is rejected as too small', () {
      // 15 cm at 6 m: 716.5 × 0.15 / 6 ≈ 18 px.
      final m = SceneMarker(4, 0.15, markerPose(6.0, yawDeg: 20));
      final found = detect([m], calibrated);
      expect(found.markers.map((o) => o.id), [4]);
      expect(found.markers.single.sidePx, lessThan(20));
      expect(found.markers.single.rejection, MarkerRejection.tooSmall);
    });

    test('a marker seen 72° off its normal is rejected as too steep', () {
      final m = SceneMarker(6, 0.15, markerPose(0.5, yawDeg: 72));
      final found = detect([m], calibrated);
      expect(found.markers.map((o) => o.id), [6]);
      final o = found.markers.single;
      expect(o.viewAngleDeg, closeTo(72, 3));
      expect(o.rejection, MarkerRejection.steepView);
    });

    test('a wrong marker side scales the distance and nothing else', () {
      // The printed side is the only scale in PnP: entering 150 mm for a
      // marker printed at 82 % puts it 1/0.82 times too far.
      final m = SceneMarker(1, 0.123, markerPose(1.0, yawDeg: 30));
      final right = detect([m], calibrated, sideM: 0.123).markers.single;
      final wrong = detect([m], calibrated, sideM: 0.150).markers.single;
      expect(right.distanceM, closeTo(1.0, 0.01));
      expect(wrong.distanceM! / right.distanceM!, closeTo(0.150 / 0.123, 1e-6));
    });
  });

  group('MarkerObservation.evaluate', () {
    final corners = [Vector2(400, 300), Vector2(500, 300), Vector2(500, 400), Vector2(400, 400)];
    final pose = markerPose(1.0, yawDeg: 30);

    test('sorts solutions by RMS and accepts a clear winner', () {
      final o = MarkerObservation.evaluate(
        id: 1,
        corners: corners,
        solutions: [PnpSolution(pose, 1.5), PnpSolution(pose, 0.2)],
      );
      expect(o.best!.rmsPx, 0.2);
      expect(o.alternative!.rmsPx, 1.5);
      expect(o.accepted, isTrue);
      expect(o.viewAngleDeg, closeTo(30, 1e-9));
    });

    test('ambiguity threshold is 2 × max(best RMS, 0.5 px)', () {
      MarkerRejection? r(double best, double alt) =>
          MarkerObservation.evaluate(id: 1, corners: corners, solutions: [PnpSolution(pose, best), PnpSolution(pose, alt)])
              .rejection;
      expect(r(0.2, 0.99), MarkerRejection.ambiguous);
      expect(r(0.2, 1.01), isNull);
      expect(r(0.8, 1.59), MarkerRejection.ambiguous);
      expect(r(0.8, 1.61), isNull);
    });

    test('rules in order: small, no pose, reprojection, angle', () {
      final tiny = [Vector2(0, 0), Vector2(19, 0), Vector2(19, 19), Vector2(0, 19)];
      expect(MarkerObservation.evaluate(id: 1, corners: tiny, solutions: [PnpSolution(pose, 9)]).rejection,
          MarkerRejection.tooSmall);
      expect(MarkerObservation.evaluate(id: 1, corners: corners, solutions: const []).rejection, MarkerRejection.noPose);
      expect(MarkerObservation.evaluate(id: 1, corners: corners, solutions: [PnpSolution(pose, 2.1)]).rejection,
          MarkerRejection.highReprojection);
      expect(
        MarkerObservation.evaluate(id: 1, corners: corners, solutions: [PnpSolution(markerPose(1, yawDeg: 66), 0.1)])
            .rejection,
        MarkerRejection.steepView,
      );
      expect(
        MarkerObservation.evaluate(id: 1, corners: corners, solutions: [PnpSolution(markerPose(1, yawDeg: 64), 0.1)])
            .rejection,
        isNull,
      );
    });

    test('reprojectionRms is zero at the true pose and matches a known shift', () {
      final m = SceneMarker(0, 0.15, markerPose(1.0, yawDeg: 20));
      final truth = m.corners(calibrated);
      expect(reprojectionRms(m.cTm, truth, calibrated, 0.15), lessThan(1e-9));
      final shifted = [for (final c in truth) c + Vector2(3, 4)];
      expect(reprojectionRms(m.cTm, shifted, calibrated, 0.15), closeTo(5, 1e-9));
    });

    test('detections.csv row round trip', () {
      final o = MarkerObservation.evaluate(
        id: 12,
        corners: corners,
        solutions: [PnpSolution(pose, 0.25), PnpSolution(markerPose(1.1, yawDeg: -28), 0.4)],
      );
      final (t, back) = MarkerObservation.fromCsvRow(o.toCsvRow(1762850000123456));
      expect(t, 1762850000123456);
      expect(back.id, 12);
      expect(back.rejection, o.rejection);
      expect(back.sidePx, o.sidePx);
      expect(back.viewAngleDeg, closeTo(o.viewAngleDeg, 1e-12));
      for (var j = 0; j < 4; j++) {
        expect(back.corners[j], o.corners[j]);
      }
      for (var i = 0; i < 2; i++) {
        expect(back.solutions[i].rmsPx, o.solutions[i].rmsPx);
        expect((back.solutions[i].cTm.translation - o.solutions[i].cTm.translation).length, lessThan(1e-12));
        expect(rotationErrorDeg(back.solutions[i].cTm, o.solutions[i].cTm), lessThan(1e-6));
      }
      final none = MarkerObservation.evaluate(id: 3, corners: corners, solutions: const []);
      final (_, noneBack) = MarkerObservation.fromCsvRow(none.toCsvRow(1));
      expect(noneBack.solutions, isEmpty);
      expect(noneBack.rejection, MarkerRejection.noPose);
      expect(noneBack.viewAngleDeg.isNaN, isTrue);
    });
  });

  group('intrinsics choice', () {
    final cal = CalibrationResult(
      intrinsics: calibrated,
      rms: 0.467,
      views: 18,
      board: CalibrationBoard.standard,
      arkitMean: arkit,
      created: DateTime(2026, 11, 16),
    );

    test('calibration when present, rescaled to the frame size', () {
      final (k, src) = chooseIntrinsics(arkit, cal);
      expect(src, IntrinsicsSource.calibration);
      expect(k.fx, calibrated.fx);
      expect(k.k1, calibrated.k1);
      final small = arkit.scaledTo(480, 360);
      final (k2, _) = chooseIntrinsics(small, cal);
      expect(k2.fx, closeTo(calibrated.fx / 2, 1e-12));
      expect(k2.cx, closeTo((calibrated.cx + 0.5) / 2 - 0.5, 1e-12));
      expect(k2.width, 480);
    });

    test('ARKit without a calibration or when the aspect ratio differs', () {
      expect(chooseIntrinsics(arkit, null).$2, IntrinsicsSource.arkit);
      final wide = CameraIntrinsics(fx: 700, fy: 700, cx: 639.5, cy: 359.5, width: 1280, height: 720);
      final (k, src) = chooseIntrinsics(wide, cal);
      expect(src, IntrinsicsSource.arkit);
      expect(k.width, 1280);
    });
  });

  group('marker sheet', () {
    test('cells: 6 × 6 with a black border and the dictionary bits inside', () {
      final ids = [for (var id = 0; id < 10; id++) markerCells(id)];
      for (final cells in ids) {
        expect(cells, hasLength(6));
        for (var i = 0; i < 6; i++) {
          expect(cells[0][i] && cells[5][i] && cells[i][0] && cells[i][5], isTrue);
        }
      }
      // All ten interiors differ.
      final interiors = {for (final c in ids) c.sublist(1, 5).map((r) => r.sublist(1, 5).join()).join('|')};
      expect(interiors, hasLength(10));
    });

    test('PDF: ten A4 pages, valid cross-reference table', () {
      final pdf = markerSheetPdf(ids: List.generate(10, (i) => i));
      final text = String.fromCharCodes(pdf);
      expect(text, startsWith('%PDF-1.4'));
      expect(text, contains('/Count 10'));
      expect(RegExp(r'/Type /Page ').allMatches(text), hasLength(10));
      expect(text, contains('/MediaBox [0 0 595.276 841.890]'));
      // Every xref entry points at the start of its object.
      final xrefAt = int.parse(RegExp(r'startxref\n(\d+)').firstMatch(text)!.group(1)!);
      expect(text.substring(xrefAt), startsWith('xref'));
      final entries = RegExp(r'(\d{10}) 00000 n ').allMatches(text.substring(xrefAt)).toList();
      for (var i = 0; i < entries.length; i++) {
        final offset = int.parse(entries[i].group(1)!);
        expect(text.substring(offset), startsWith('${i + 1} 0 obj'));
      }
      // A solid 150 mm square (425.197 pt) with 25 mm (70.866 pt) white cells.
      expect(RegExp(r' 425\.197 425\.197 re f').allMatches(text), hasLength(10));
      expect(text, contains(' 70.866 70.866 re f'));
    });
  });

  group('detection over a saved session', () {
    late Directory tmp;
    setUp(() => tmp = Directory.systemTemp.createTempSync('vslam_detect_'));
    tearDown(() => tmp.deleteSync(recursive: true));

    test('writes detections.csv and detections.json for every frame', () async {
      const side = 0.15;
      final writer = SessionWriter.create(
        tmp,
        SessionMeta(
          settings: const RecordingSettings(label: 'markers', markerSideM: side),
          device: const DeviceInfo(),
          created: DateTime(2026, 11, 16, 10),
        ),
      );
      final scenes = <List<SceneMarker>?>[
        [SceneMarker(3, side, markerPose(1.0, yawDeg: 30))],
        [],
        null, // a frame without an image
        // Off-centre by ±0.3 (≈ 17°) and turned away from the camera: seen at
        // about 42° and 47°. (Turned toward the camera instead, the angles
        // nearly cancel and the head-on view is correctly "ambiguous".)
        [SceneMarker(3, side, markerPose(1.1, yawDeg: -25, ox: -0.3)), SceneMarker(8, side, markerPose(1.5, yawDeg: 30, ox: 0.3))],
        [SceneMarker(2, side, markerPose(2.5, yawDeg: 3))], // ambiguous
      ];
      const t0 = 1763280000000000;
      for (final (i, scene) in scenes.indexed) {
        final Uint8List? jpeg = scene == null ? null : renderMarkers(scene, arkit);
        writer.addFrame(
          FrameEvent(t0 + i * 100000, intrinsics: arkit, state: ArTrackingState.tracking, loadJpeg: () async => jpeg),
          jpeg,
        );
      }
      await writer.close();

      final data = await SessionReader.read(writer.directory);
      final progress = <int>[];
      final summary = await detectSession(data, onProgress: (done, _) => progress.add(done));
      expect(progress, [1, 2, 3, 4, 5]);
      expect(summary.frames, 5);
      expect(summary.framesWithImage, 4);
      expect(summary.framesWithAccepted, 2);
      expect(summary.accepted, {3: 2, 8: 1});
      expect(summary.rejected, {'ambiguous': 1});
      expect(summary.intrinsicsSource, IntrinsicsSource.arkit);
      expect(summary.sideM, side);

      final rows = await readDetections(writer.directory);
      expect([for (final (t, _) in rows) (t - t0) ~/ 100000], [0, 3, 3, 4]);
      expect(rows.map((r) => r.$2.id).toList(), [3, 3, 8, 2]);
      expect(rows.first.$2.distanceM, closeTo(1.0, 0.01));
      expect(File('${writer.directory.path}/results/detections.json').existsSync(), isTrue);
    });
  });
}
