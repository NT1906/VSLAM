import 'dart:convert';
import 'dart:io';

import '../calibration/camera_calibration.dart';
import '../dataset/session_files.dart';
import '../dataset/session_reader.dart';
import 'aruco_detector.dart';
import 'camera_model.dart';
import 'marker_observation.dart';

/// `results/detections.csv`: one row per detected marker per frame.
const detectionsFile = 'detections.csv';

/// `results/detections.json`: what [detectSession] did and found.
const detectionsSummaryFile = 'detections.json';

/// Where a frame's intrinsics come from.
enum IntrinsicsSource {
  /// The OpenCV calibration in `Documents/calibration.json` (WP2), with
  /// k1, k2 distortion.
  calibration,

  /// ARKit's per-frame intrinsics (pinhole, no distortion), used when no
  /// calibration exists.
  arkit,
}

/// Intrinsics for marker pose estimation on a frame whose ARKit intrinsics
/// are [arkit]: the saved calibration, rescaled to the frame size, when one
/// exists and has the frame's aspect ratio; ARKit's otherwise.
///
/// Why the calibration first: it adds the radial distortion ARKit does not
/// report (k1 0.0715, k2 −0.1411 on the test phone, about 1.5 px at the
/// image corners), and its RMS is measured. ARKit's per-frame values move by
/// a few tenths of a pixel within a session (check 1.3), which is below
/// what matters here.
(CameraIntrinsics, IntrinsicsSource) chooseIntrinsics(CameraIntrinsics arkit, CalibrationResult? calibration) {
  final c = calibration?.intrinsics;
  if (c == null || c.width * arkit.height != c.height * arkit.width) return (arkit, IntrinsicsSource.arkit);
  final k = c.width == arkit.width ? c : c.scaledTo(arkit.width, arkit.height);
  return (k, IntrinsicsSource.calibration);
}

/// Totals of one detection run over a saved session.
class DetectionSummary {
  const DetectionSummary({
    required this.frames,
    required this.framesWithImage,
    required this.framesWithAccepted,
    required this.accepted,
    required this.rejected,
    required this.meanDetectMs,
    required this.maxDetectMs,
    required this.sideM,
    required this.intrinsicsSource,
    required this.intrinsics,
  });

  final int frames;
  final int framesWithImage;

  /// Frames with at least one accepted marker.
  final int framesWithAccepted;

  /// Accepted detections per marker ID.
  final Map<int, int> accepted;

  /// Rejected detections per rejection status code.
  final Map<String, int> rejected;
  final double meanDetectMs;
  final double maxDetectMs;
  final double sideM;
  final IntrinsicsSource intrinsicsSource;

  /// Intrinsics of the first frame with an image (the calibration, rescaled,
  /// or ARKit's).
  final CameraIntrinsics? intrinsics;

  int get acceptedTotal => accepted.values.fold(0, (a, b) => a + b);
  int get rejectedTotal => rejected.values.fold(0, (a, b) => a + b);

  Map<String, dynamic> toJson() => {
    'frames': frames,
    'frames_with_image': framesWithImage,
    'frames_with_accepted_marker': framesWithAccepted,
    'accepted_by_id': {for (final e in accepted.entries) '${e.key}': e.value},
    'rejected_by_reason': rejected,
    'detect_ms_mean': meanDetectMs,
    'detect_ms_max': maxDetectMs,
    'marker_side_m': sideM,
    'dictionary': 'DICT_4X4_50',
    'intrinsics_source': intrinsicsSource.name,
    'intrinsics': intrinsics?.toJson(),
  };
}

/// Runs the detector on every saved frame of [data] (the same JPEG pixels
/// the live view saw) and writes `results/detections.csv` and
/// `results/detections.json`. Marker side comes from the session's settings.
Future<DetectionSummary> detectSession(
  SessionData data, {
  CalibrationResult? calibration,
  MarkerDetector detector = const ArucoMarkerDetector(),
  void Function(int done, int total)? onProgress,
}) async {
  final side = data.meta.settings.markerSideM;
  final results = Directory('${data.directory.path}/${SessionFiles.results}');
  await results.create(recursive: true);
  final csv = File('${results.path}/$detectionsFile').openWrite()..writeln(MarkerObservation.csvHeader);
  final accepted = <int, int>{};
  final rejected = <String, int>{};
  var withImage = 0;
  var withAccepted = 0;
  var totalMs = 0.0;
  var maxMs = 0.0;
  var source = calibration == null ? IntrinsicsSource.arkit : IntrinsicsSource.calibration;
  CameraIntrinsics? firstK;
  try {
    for (var i = 0; i < data.frames.length; i++) {
      final frame = data.frames[i].event;
      final jpeg = await frame.loadJpeg();
      if (jpeg != null && jpeg.isNotEmpty) {
        withImage++;
        final (k, src) = chooseIntrinsics(frame.intrinsics, calibration);
        firstK ??= k;
        source = src;
        final found = await detector.detect(jpeg, k, side);
        totalMs += found.detectMs;
        if (found.detectMs > maxMs) maxMs = found.detectMs;
        var any = false;
        for (final m in found.markers) {
          csv.writeln(m.toCsvRow(frame.tUs));
          if (m.accepted) {
            accepted[m.id] = (accepted[m.id] ?? 0) + 1;
            any = true;
          } else {
            rejected[m.statusCode] = (rejected[m.statusCode] ?? 0) + 1;
          }
        }
        if (any) withAccepted++;
      }
      onProgress?.call(i + 1, data.frames.length);
    }
  } finally {
    await csv.close();
  }
  final summary = DetectionSummary(
    frames: data.frames.length,
    framesWithImage: withImage,
    framesWithAccepted: withAccepted,
    accepted: Map.fromEntries(accepted.entries.toList()..sort((a, b) => a.key.compareTo(b.key))),
    rejected: rejected,
    meanDetectMs: withImage == 0 ? 0 : totalMs / withImage,
    maxDetectMs: maxMs,
    sideM: side,
    intrinsicsSource: source,
    intrinsics: firstK,
  );
  await File('${results.path}/$detectionsSummaryFile')
      .writeAsString(const JsonEncoder.withIndent('  ').convert(summary.toJson()));
  return summary;
}

/// Reads `results/detections.csv` of a session: (frame time µs, observation)
/// in file order. Empty when the session has not been processed.
Future<List<(int, MarkerObservation)>> readDetections(Directory session) async {
  final file = File('${session.path}/${SessionFiles.results}/$detectionsFile');
  if (!await file.exists()) return const [];
  final lines = await file.readAsLines();
  if (lines.isEmpty || lines.first != MarkerObservation.csvHeader) {
    throw FormatException('${file.path}: unexpected header');
  }
  return [
    for (final line in lines.skip(1))
      if (line.isNotEmpty) MarkerObservation.fromCsvRow(line),
  ];
}
