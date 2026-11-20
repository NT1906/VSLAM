import 'dart:math' as math;
import 'dart:typed_data';

import 'package:dartcv4/dartcv.dart' as cv;
import 'package:vector_math/vector_math_64.dart';
import 'package:vslam/core/se3.dart';
import 'package:vslam/vision/camera_model.dart';
import 'package:vslam/vision/marker_observation.dart';
import 'package:vslam/vision/marker_sheet.dart';

/// A printed marker in a rendered scene: ID, side of the black square, and
/// its pose in the camera frame (`cTm`, marker frame of spec 4.3).
class SceneMarker {
  const SceneMarker(this.id, this.sideM, this.cTm);

  final int id;
  final double sideM;
  final Pose3 cTm;

  /// True corner pixels (OpenCV order) through [k], including distortion.
  List<Vector2> corners(CameraIntrinsics k) => [for (final p in markerObjectPoints(sideM)) k.project(cTm.transform(p))!];
}

/// Renders [markers] (each on white paper with a margin of one cell around
/// the black square) over a mid-grey background, as seen by [k] with its
/// radial distortion, by casting a ray through every pixel. Pixels near
/// the paper are 4 × 4 supersampled so edges land with sub-pixel accuracy.
/// Returns a grayscale JPEG (quality 95, like the app's frames).
Uint8List renderMarkers(List<SceneMarker> markers, CameraIntrinsics k) {
  final pixels = Uint8List(k.width * k.height)..fillRange(0, k.width * k.height, 120);
  for (final m in markers) {
    final cells = markerCells(m.id);
    final cell = m.sideM / cells.length;
    final h = m.sideM / 2;
    final paper = h + cell;
    final mTc = m.cTm.inverse();
    final o = mTc.translation;

    // Image bounding box of the paper (with distortion), plus 2 px.
    final outline = [
      for (final p in [Vector3(-paper, paper, 0), Vector3(paper, paper, 0), Vector3(paper, -paper, 0), Vector3(-paper, -paper, 0)])
        k.project(m.cTm.transform(p))!,
    ];
    final u0 = math.max(0, outline.map((p) => p.x).reduce(math.min).floor() - 2);
    final u1 = math.min(k.width - 1, outline.map((p) => p.x).reduce(math.max).ceil() + 2);
    final v0 = math.max(0, outline.map((p) => p.y).reduce(math.min).floor() - 2);
    final v1 = math.min(k.height - 1, outline.map((p) => p.y).reduce(math.max).ceil() + 2);

    for (var v = v0; v <= v1; v++) {
      for (var u = u0; u <= u1; u++) {
        var sum = 0.0;
        var onPaper = 0;
        for (var sy = 0; sy < 4; sy++) {
          for (var sx = 0; sx < 4; sx++) {
            final n = k.normalize(u - 0.375 + sx * 0.25, v - 0.375 + sy * 0.25);
            final d = mTc.rotation.transformed(Vector3(n.x, n.y, 1));
            final s = -o.z / d.z;
            if (s <= 0) {
              sum += pixels[v * k.width + u];
              continue;
            }
            final x = o.x + s * d.x;
            final y = o.y + s * d.y;
            if (x.abs() >= paper || y.abs() >= paper) {
              sum += pixels[v * k.width + u];
              continue;
            }
            onPaper++;
            if (x.abs() < h && y.abs() < h) {
              final c = ((x + h) / cell).floor().clamp(0, cells.length - 1);
              final r = ((h - y) / cell).floor().clamp(0, cells.length - 1);
              sum += cells[r][c] ? 20 : 235;
            } else {
              sum += 235;
            }
          }
        }
        if (onPaper > 0) pixels[v * k.width + u] = (sum / 16).round();
      }
    }
  }
  final mat = cv.Mat.fromList(k.height, k.width, cv.MatType.CV_8UC1, pixels);
  final (ok, jpeg) = cv.imencode('.jpg', mat, params: cv.VecI32.fromList([cv.IMWRITE_JPEG_QUALITY, 95]));
  mat.dispose();
  if (!ok) throw StateError('imencode failed');
  return jpeg;
}

/// Marker pose seen from [dist] metres: the camera looks at the marker
/// centre, which sits at image offset ([ox], [oy]) in normalised
/// coordinates; the marker is turned by [yawDeg] about its vertical axis,
/// [pitchDeg] about its horizontal axis, and rolled by [rollDeg] in its
/// plane. With all angles zero the marker faces the camera upright
/// (marker +y = image up, marker +z toward the camera).
Pose3 markerPose(double dist, {double yawDeg = 0, double pitchDeg = 0, double rollDeg = 0, double ox = 0, double oy = 0}) {
  const deg = math.pi / 180;
  // Upright and facing the camera: marker x → camera x, marker y → camera
  // −y, marker z → camera −z.
  final facing = Matrix3(1, 0, 0, 0, -1, 0, 0, 0, -1); // column-major, diagonal
  final turn = so3Exp(Vector3(0, yawDeg * deg, 0))
      .multiplied(so3Exp(Vector3(pitchDeg * deg, 0, 0)))
      .multiplied(so3Exp(Vector3(0, 0, rollDeg * deg)));
  final t = Vector3(ox, oy, 1)..scale(dist / math.sqrt(1 + ox * ox + oy * oy));
  return Pose3(facing.multiplied(turn), t);
}
