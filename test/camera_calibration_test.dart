import 'dart:math' as math;
import 'dart:typed_data';

import 'package:dartcv4/dartcv.dart' as cv;
import 'package:flutter_test/flutter_test.dart';
import 'package:vector_math/vector_math_64.dart';
import 'package:vslam/calibration/board.dart';
import 'package:vslam/calibration/camera_calibration.dart';
import 'package:vslam/core/se3.dart';
import 'package:vslam/vision/camera_model.dart';

/// Renders the printed board (white paper with margin, on a grey table) as
/// seen by [cam] from board-frame camera pose [bTc], 2×2 supersampled, and
/// returns it as a grayscale JPEG.
Uint8List renderBoard(CalibrationBoard board, CameraIntrinsics cam, Pose3 bTc) {
  final sq = board.squareM;
  final nx = board.cols + 1;
  final ny = board.rows + 1;
  final pixels = Uint8List(cam.width * cam.height);
  final o = bTc.translation;
  for (var v = 0; v < cam.height; v++) {
    for (var u = 0; u < cam.width; u++) {
      var sum = 0.0;
      for (final (du, dv) in const [(-0.25, -0.25), (0.25, -0.25), (-0.25, 0.25), (0.25, 0.25)]) {
        final n = cam.normalize(u + du, v + dv);
        final d = bTc.rotation.transformed(Vector3(n.x, n.y, 1));
        final s = -o.z / d.z;
        final x = o.x + s * d.x;
        final y = o.y + s * d.y;
        final i = (x / sq).floor() + 1;
        final j = (y / sq).floor() + 1;
        double value;
        if (i >= 0 && i < nx && j >= 0 && j < ny) {
          value = (i + j).isEven ? 20 : 235;
        } else if (x > -sq - 0.015 && x < (nx - 1) * sq + 0.015 && y > -sq - 0.015 && y < (ny - 1) * sq + 0.015) {
          value = 235; // paper margin
        } else {
          value = 120; // table
        }
        sum += value;
      }
      pixels[v * cam.width + u] = (sum / 4).round();
    }
  }
  final mat = cv.Mat.fromList(cam.height, cam.width, cv.MatType.CV_8UC1, pixels);
  final (ok, jpeg) = cv.imencode('.jpg', mat, params: cv.VecI32.fromList([cv.IMWRITE_JPEG_QUALITY, 95]));
  mat.dispose();
  expect(ok, isTrue);
  return jpeg;
}

/// Camera pose looking at the board centre from [dist] metres, after
/// tilting by [rx], [ry] and rolling by [rz] radians.
Pose3 viewPose(CalibrationBoard board, double rx, double ry, double rz, double dist, {double sx = 0, double sy = 0}) {
  final centre = Vector3(board.cols - 1, board.rows - 1, 0) * (board.squareM / 2) + Vector3(sx, sy, 0);
  final r = so3Exp(Vector3(rx, 0, 0)).multiplied(so3Exp(Vector3(0, ry, 0))).multiplied(so3Exp(Vector3(0, 0, rz)));
  final o = centre - r.transformed(Vector3(0, 0, dist));
  return Pose3(r, o);
}

void main() {
  const truth = CameraIntrinsics(fx: 721.4, fy: 721.4, cx: 477.6, cy: 361.5, k1: 0.08, k2: -0.15, width: 960, height: 720);
  const board = CalibrationBoard.standard;

  test('detects the rendered board and recovers known intrinsics', () async {
    final arkit = const CameraIntrinsics(fx: 722.0, fy: 722.0, cx: 481.4, cy: 361.6, width: 960, height: 720);
    final poses = [
      viewPose(board, 0.0, 0.0, 0.0, 0.33),
      viewPose(board, 0.35, 0.0, 0.1, 0.35),
      viewPose(board, -0.35, 0.1, -0.1, 0.35),
      viewPose(board, 0.0, 0.4, 0.2, 0.36),
      viewPose(board, 0.1, -0.4, -0.2, 0.36),
      viewPose(board, 0.25, 0.25, 0.5, 0.38, sx: 0.05), // at 0.30 m part of the board left the image
      viewPose(board, -0.25, -0.25, -0.4, 0.30, sx: -0.05),
      viewPose(board, 0.2, -0.3, 0.0, 0.28, sy: 0.04),
      viewPose(board, -0.2, 0.3, 0.3, 0.28, sy: -0.04),
      viewPose(board, 0.0, 0.0, math.pi / 2, 0.42),
    ];
    final views = <BoardView>[];
    for (final p in poses) {
      final jpeg = renderBoard(board, truth, p);
      final corners = await detectBoard(jpeg, board);
      expect(corners, isNotNull, reason: 'board not found in view ${views.length}');
      // Check against the true projection (either corner order).
      final expected = [for (final q in board.objectPoints()) truth.project(p.inverse().transform(q))!];
      final err0 = corners!.first.distanceTo(expected.first);
      final err1 = corners.first.distanceTo(expected.last);
      final ordered = err0 <= err1 ? expected : expected.reversed.toList();
      var maxErr = 0.0;
      for (var i = 0; i < corners.length; i++) {
        maxErr = math.max(maxErr, corners[i].distanceTo(ordered[i]));
      }
      expect(maxErr, lessThan(0.5), reason: 'corner error in view ${views.length}');
      views.add(BoardView(corners: corners, arkit: arkit));
    }
    final r = await calibrate(views, board);
    // ignore: avoid_print
    print('synthetic calibration: RMS ${r.rms.toStringAsFixed(3)} px, ${r.intrinsics}');
    expect(r.rms, lessThan(0.3));
    expect(r.intrinsics.fx, closeTo(truth.fx, truth.fx * 0.005));
    expect(r.intrinsics.fy, closeTo(truth.fy, truth.fy * 0.005));
    expect(r.intrinsics.cx, closeTo(truth.cx, 2));
    expect(r.intrinsics.cy, closeTo(truth.cy, 2));
    expect(r.intrinsics.k1, closeTo(truth.k1, 0.03));
    expect(r.intrinsics.k2, closeTo(truth.k2, 0.1));
    expect(r.arkitMean.fx, 722.0);
    final json = CalibrationResult.fromJson(r.toJson());
    expect(json.intrinsics.fx, r.intrinsics.fx);
    expect(json.rms, r.rms);
  }, timeout: const Timeout(Duration(minutes: 3)));

  test('no board in the image gives null', () async {
    final mat = cv.Mat.zeros(720, 960, cv.MatType.CV_8UC1);
    final (_, jpeg) = cv.imencode('.jpg', mat);
    expect(await detectBoard(jpeg, board), isNull);
  });

  test('board SVG has true-scale size and a 100 mm ruler', () {
    final svg = boardSvg(board);
    expect(svg, contains('width="280.0mm"'));
    expect(svg, contains('100 mm'));
    expect(RegExp('<rect x=').allMatches(svg).length, 35); // half of 10 × 7 squares are black
  });
}
