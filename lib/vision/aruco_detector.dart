import 'dart:isolate';
import 'dart:typed_data';

import 'package:dartcv4/dartcv.dart' as cv;
import 'package:vector_math/vector_math_64.dart';

import '../core/se3.dart';
import '../estimation/config.dart';
import 'camera_model.dart';
import 'marker_observation.dart';

/// Markers found in one image and how long finding them took.
class FrameDetections {
  const FrameDetections(this.markers, this.detectMs);

  final List<MarkerObservation> markers;

  /// Wall time of decode + detection + PnP for the whole image, ms.
  final double detectMs;

  Iterable<MarkerObservation> get accepted => markers.where((m) => m.accepted);
}

/// Finds markers in a grayscale JPEG. The app uses [ArucoMarkerDetector];
/// the interface lets UI tests run without OpenCV.
abstract interface class MarkerDetector {
  /// [intrinsics] must be for the image's resolution; [sideM] is the printed
  /// side of the black marker square.
  Future<FrameDetections> detect(Uint8List jpeg, CameraIntrinsics intrinsics, double sideM);
}

/// `cv::aruco::CORNER_REFINE_SUBPIX`. dartcv4 2.3.1 exposes the
/// `cornerRefinementMethod` setter but not the enum; the value is the
/// second entry of `enum CornerRefineMethod` in OpenCV 4.13's
/// `objdetect/aruco_detector.hpp` (NONE = 0, SUBPIX = 1, CONTOUR = 2,
/// APRILTAG = 3).
const int cornerRefineSubpix = 1;

/// ArUco detection with OpenCV (`DICT_4X4_50`, sub-pixel corner refinement)
/// followed by `solvePnPGeneric` with `SOLVEPNP_IPPE_SQUARE`, which returns
/// both poses of the planar-square ambiguity. Each call runs in a background
/// isolate so the UI never waits for OpenCV.
class ArucoMarkerDetector implements MarkerDetector {
  const ArucoMarkerDetector({this.config = const SlamConfig()});

  final SlamConfig config;

  @override
  Future<FrameDetections> detect(Uint8List jpeg, CameraIntrinsics intrinsics, double sideM) {
    final config = this.config;
    return Isolate.run(() => detectMarkersSync(jpeg, intrinsics, sideM, config));
  }
}

/// The work of [ArucoMarkerDetector.detect] on the calling isolate.
FrameDetections detectMarkersSync(Uint8List jpeg, CameraIntrinsics k, double sideM, SlamConfig config) {
  final watch = Stopwatch()..start();
  final img = cv.imdecode(jpeg, cv.IMREAD_GRAYSCALE);
  final dictionary = cv.ArucoDictionary.predefined(cv.PredefinedDictionaryType.DICT_4X4_50);
  final params = cv.ArucoDetectorParameters.empty()..cornerRefinementMethod = cornerRefineSubpix;
  final detector = cv.ArucoDetector.create(dictionary, params);
  try {
    if (img.isEmpty) return FrameDetections(const [], watch.elapsedMicroseconds / 1000);
    final (corners, ids, rejected) = detector.detectMarkers(img);
    final found = <(int, List<Vector2>)>[];
    for (var i = 0; i < ids.length; i++) {
      final quad = corners[i];
      found.add((ids[i], [for (var j = 0; j < 4; j++) Vector2(quad[j].x, quad[j].y)]));
    }
    corners.dispose();
    ids.dispose();
    rejected.dispose();
    final markers = [
      for (final (id, quad) in found)
        MarkerObservation.evaluate(id: id, corners: quad, solutions: solveMarkerPnp(quad, k, sideM), config: config),
    ];
    return FrameDetections(markers, watch.elapsedMicroseconds / 1000);
  } finally {
    img.dispose();
    detector.dispose();
    params.dispose();
    dictionary.dispose();
  }
}

/// Both IPPE solutions for one marker, with reprojection RMS computed by
/// our own camera model ([reprojectionRms]), so the numbers the rejection
/// rules see are the ones the estimator's marker factor will use.
///
/// The distortion vector is (k1, k2, p1 = 0, p2 = 0, k3 = 0), the model
/// calibration fitted and [CameraIntrinsics] implements.
List<PnpSolution> solveMarkerPnp(List<Vector2> corners, CameraIntrinsics k, double sideM) {
  final obj = markerObjectPoints(sideM);
  final objectPoints = cv.Mat.fromList(1, 4, cv.MatType.CV_64FC3, <double>[
    for (final p in obj) ...[p.x, p.y, p.z],
  ]);
  final imagePoints = cv.Mat.fromList(1, 4, cv.MatType.CV_64FC2, <double>[
    for (final c in corners) ...[c.x, c.y],
  ]);
  final camera = cv.Mat.fromList(3, 3, cv.MatType.CV_64FC1, <double>[k.fx, 0.0, k.cx, 0.0, k.fy, k.cy, 0.0, 0.0, 1.0]);
  final dist = cv.Mat.fromList(1, 5, cv.MatType.CV_64FC1, <double>[k.k1, k.k2, 0.0, 0.0, 0.0]);
  try {
    final (n, rvecs, tvecs, errors) = cv.solvePnPGeneric(
      objectPoints,
      imagePoints,
      camera,
      dist,
      flags: cv.SOLVEPNP_IPPE_SQUARE,
    );
    final out = <PnpSolution>[];
    for (var i = 0; i < n; i++) {
      final r = rvecs[i];
      final t = tvecs[i];
      // The rotation vector is axis × angle; Rodrigues' formula is so3Exp.
      final cTm = Pose3(
        so3Exp(Vector3(r.atF64(0, i1: 0), r.atF64(1, i1: 0), r.atF64(2, i1: 0))),
        Vector3(t.atF64(0, i1: 0), t.atF64(1, i1: 0), t.atF64(2, i1: 0)),
      );
      out.add(PnpSolution(cTm, reprojectionRms(cTm, corners, k, sideM)));
    }
    rvecs.dispose();
    tvecs.dispose();
    errors.dispose();
    return out;
  } finally {
    objectPoints.dispose();
    imagePoints.dispose();
    camera.dispose();
    dist.dispose();
  }
}
