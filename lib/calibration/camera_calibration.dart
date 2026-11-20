import 'dart:convert';
import 'dart:io';
import 'dart:isolate';
import 'dart:typed_data';

import 'package:dartcv4/dartcv.dart' as cv;
import 'package:vector_math/vector_math_64.dart';

import '../vision/camera_model.dart';
import 'board.dart';

/// Detected inner corners of one board view, in pixels, row by row.
class BoardView {
  const BoardView({required this.corners, required this.arkit});

  final List<Vector2> corners;

  /// ARKit's intrinsics for the same image, for the comparison.
  final CameraIntrinsics arkit;

  Map<String, dynamic> toJson() => {
        'corners': [for (final c in corners) [c.x, c.y]],
        'arkit': arkit.toJson(),
      };
}

/// Finds the chessboard in a grayscale JPEG and refines the corners to
/// sub-pixel accuracy. Runs OpenCV in a background isolate; returns null when
/// the full board is not found.
Future<List<Vector2>?> detectBoard(Uint8List jpeg, CalibrationBoard board) async {
  final flat = await Isolate.run(() => _detect(jpeg, board.cols, board.rows));
  if (flat == null) return null;
  return [for (var i = 0; i < flat.length; i += 2) Vector2(flat[i], flat[i + 1])];
}

List<double>? _detect(Uint8List jpeg, int cols, int rows) {
  final img = cv.imdecode(jpeg, cv.IMREAD_GRAYSCALE);
  try {
    if (img.isEmpty) return null;
    final (found, corners) = cv.findChessboardCorners(
      img,
      (cols, rows),
      flags: cv.CALIB_CB_ADAPTIVE_THRESH + cv.CALIB_CB_NORMALIZE_IMAGE + cv.CALIB_CB_FAST_CHECK,
    );
    if (!found || corners.length != cols * rows) return null;
    final refined = cv.cornerSubPix(img, corners, (11, 11), (-1, -1), (cv.TERM_COUNT + cv.TERM_EPS, 30, 1e-3));
    return [for (var i = 0; i < refined.length; i++) ...[refined[i].x, refined[i].y]];
  } finally {
    img.dispose();
  }
}

/// Result of a calibration, stored as `Documents/calibration.json`.
class CalibrationResult {
  const CalibrationResult({
    required this.intrinsics,
    required this.rms,
    required this.views,
    required this.board,
    required this.arkitMean,
    required this.created,
  });

  /// fx, fy, cx, cy, k1, k2 (tangential distortion and k3 fixed to zero).
  final CameraIntrinsics intrinsics;

  /// RMS reprojection error in pixels, as returned by `cv::calibrateCamera`.
  final double rms;
  final int views;
  final CalibrationBoard board;

  /// Mean of ARKit's per-frame intrinsics over the calibration views, at the
  /// same resolution.
  final CameraIntrinsics arkitMean;
  final DateTime created;

  Map<String, dynamic> toJson() => {
        'intrinsics': intrinsics.toJson(),
        'rms_px': rms,
        'views': views,
        'board': board.toJson(),
        'arkit_mean': arkitMean.toJson(),
        'created': created.toIso8601String(),
        'flags': 'CALIB_ZERO_TANGENT_DIST | CALIB_FIX_K3',
      };

  factory CalibrationResult.fromJson(Map<String, dynamic> j) => CalibrationResult(
        intrinsics: CameraIntrinsics.fromJson(j['intrinsics'] as Map<String, dynamic>),
        rms: (j['rms_px'] as num).toDouble(),
        views: (j['views'] as num).toInt(),
        board: CalibrationBoard.fromJson(j['board'] as Map<String, dynamic>),
        arkitMean: CameraIntrinsics.fromJson(j['arkit_mean'] as Map<String, dynamic>),
        created: DateTime.parse(j['created'] as String),
      );

  static File file(Directory documents) => File('${documents.path}/calibration.json');

  Future<void> save(Directory documents) =>
      file(documents).writeAsString(const JsonEncoder.withIndent('  ').convert(toJson()));

  static Future<CalibrationResult?> load(Directory documents) async {
    final f = file(documents);
    if (!await f.exists()) return null;
    return CalibrationResult.fromJson(jsonDecode(await f.readAsString()) as Map<String, dynamic>);
  }
}

/// Runs `cv::calibrateCamera` on [views] with tangential distortion and k3
/// fixed to zero, starting from ARKit's mean intrinsics.
Future<CalibrationResult> calibrate(List<BoardView> views, CalibrationBoard board, {DateTime? now}) async {
  if (views.length < 3) throw StateError('Need at least 3 views, have ${views.length}');
  final width = views.first.arkit.width;
  final height = views.first.arkit.height;
  final arkit = meanIntrinsics([for (final v in views) v.arkit]);
  final obj = [for (final p in board.objectPoints()) [p.x, p.y, p.z]];
  final img = [
    for (final v in views) [for (final c in v.corners) [c.x, c.y]],
  ];
  final init = [arkit.fx, arkit.fy, arkit.cx, arkit.cy];
  final out = await Isolate.run(() => _calibrate(obj, img, width, height, init));
  return CalibrationResult(
    intrinsics: CameraIntrinsics(
      fx: out[1],
      fy: out[2],
      cx: out[3],
      cy: out[4],
      k1: out[5],
      k2: out[6],
      width: width,
      height: height,
    ),
    rms: out[0],
    views: views.length,
    board: board,
    arkitMean: arkit,
    created: now ?? DateTime.now(),
  );
}

/// Returns [rms, fx, fy, cx, cy, k1, k2].
List<double> _calibrate(List<List<double>> obj, List<List<List<double>>> img, int width, int height, List<double> init) {
  final objectPoints = cv.VecVecPoint3f.fromList([
    for (var v = 0; v < img.length; v++) [for (final p in obj) cv.Point3f(p[0], p[1], p[2])],
  ]);
  final imagePoints = cv.VecVecPoint2f.fromList([
    for (final view in img) [for (final p in view) cv.Point2f(p[0], p[1])],
  ]);
  final k = cv.Mat.fromList(3, 3, cv.MatType.CV_64FC1, <double>[init[0], 0.0, init[2], 0.0, init[1], init[3], 0.0, 0.0, 1.0]);
  final d = cv.Mat.zeros(1, 5, cv.MatType.CV_64FC1);
  final (rms, cam, dist, rvecs, tvecs) = cv.calibrateCamera(
    objectPoints,
    imagePoints,
    (width, height),
    k,
    d,
    flags: cv.CALIB_USE_INTRINSIC_GUESS + cv.CALIB_ZERO_TANGENT_DIST + cv.CALIB_FIX_K3,
  );
  final result = [
    rms,
    cam.atF64(0, i1: 0),
    cam.atF64(1, i1: 1),
    cam.atF64(0, i1: 2),
    cam.atF64(1, i1: 2),
    dist.atF64(0, i1: 0),
    dist.atF64(0, i1: 1),
  ];
  for (final m in [k, d, rvecs, tvecs]) {
    m.dispose();
  }
  objectPoints.dispose();
  imagePoints.dispose();
  return result;
}

CameraIntrinsics meanIntrinsics(List<CameraIntrinsics> ks) {
  double mean(double Function(CameraIntrinsics) f) => ks.fold(0.0, (a, k) => a + f(k)) / ks.length;
  return CameraIntrinsics(
    fx: mean((k) => k.fx),
    fy: mean((k) => k.fy),
    cx: mean((k) => k.cx),
    cy: mean((k) => k.cy),
    width: ks.first.width,
    height: ks.first.height,
  );
}

/// Fraction of a [gx] × [gy] grid over the image that contains at least one
/// detected corner: a quick measure of how well the views cover the frame.
double coverage(List<BoardView> views, int width, int height, {int gx = 6, int gy = 4}) {
  final hit = <int>{};
  for (final v in views) {
    for (final c in v.corners) {
      final x = (c.x / width * gx).floor().clamp(0, gx - 1);
      final y = (c.y / height * gy).floor().clamp(0, gy - 1);
      hit.add(y * gx + x);
    }
  }
  return hit.length / (gx * gy);
}
