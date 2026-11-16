import 'dart:async';
import 'dart:io';
import 'dart:math' as math;

import 'package:flutter/material.dart';
import 'package:path_provider/path_provider.dart';
import 'package:provider/provider.dart';
import 'package:vector_math/vector_math_64.dart' show Vector2;

import '../calibration/camera_calibration.dart';
import '../slam/ar_frame.dart';
import '../slam/slam_service.dart';
import '../vision/aruco_detector.dart';
import '../vision/marker_observation.dart';
import '../vision/session_detection.dart';

/// Maps image pixels (sensor orientation, landscape) to positions in the
/// portrait camera view, through an affine map of normalized coordinates
/// `[a, b, c, d, tx, ty]` (`CGAffineTransform` convention):
///
///     X = a·x + c·y + tx,   Y = b·x + d·y + ty
///
/// where (x, y) = ((u + 0.5) / W, (v + 0.5) / H) is a pixel centre as a
/// fraction of the image (OpenCV puts pixel centres on integers) and (X, Y)
/// a fraction of the view.
class ImageToView {
  const ImageToView(this.t, this.imageWidth, this.imageHeight);

  /// The transform ARKit reports for the view (`ArFrame.displayTransform`).
  factory ImageToView.fromFrame(ArFrame frame, Size view) {
    final t = frame.displayTransform;
    return t != null && t.length == 6
        ? ImageToView(t, frame.width, frame.height)
        : ImageToView.portraitAspectFill(frame.width, frame.height, view);
  }

  /// What the camera view does when ARKit's transform is not available:
  /// rotate the landscape image 90° clockwise into portrait (image +x
  /// points down the screen, image +y to the left; the same axes as
  /// `appleCameraFromDevice`) and scale it to fill the view, cropping the
  /// overflow equally on both sides.
  factory ImageToView.portraitAspectFill(int width, int height, Size view) {
    // Rotated, the image is `height` wide and `width` tall.
    final s = math.max(view.width / height, view.height / width);
    final ox = (view.width - height * s) / 2;
    final oy = (view.height - width * s) / 2;
    // X = (ox + H·s·(1 − y)) / vw,  Y = (oy + W·s·x) / vh.
    return ImageToView(
      [0, width * s / view.height, -height * s / view.width, 0, (ox + height * s) / view.width, oy / view.height],
      width,
      height,
    );
  }

  final List<double> t;
  final int imageWidth;
  final int imageHeight;

  Offset map(Vector2 pixel, Size view) {
    final x = (pixel.x + 0.5) / imageWidth;
    final y = (pixel.y + 0.5) / imageHeight;
    return Offset((t[0] * x + t[2] * y + t[4]) * view.width, (t[1] * x + t[3] * y + t[5]) * view.height);
  }
}

/// Live marker detection drawn over the camera view: each marker's outline
/// (green when accepted, orange when a rule of spec 4.4 rejected it, with a
/// dot on its top-left corner), its ID, distance and, when rejected, why.
///
/// Detection runs on the live frames' JPEGs (the bytes the recorder saves)
/// in a background isolate; frames arriving while one is being processed
/// are skipped. Intrinsics: the saved calibration when there is one, ARKit's
/// per-frame values otherwise ([chooseIntrinsics]).
class MarkerOverlay extends StatefulWidget {
  const MarkerOverlay({super.key, required this.sideM, this.detector = const ArucoMarkerDetector(), this.documents});

  /// Printed marker side from the recording settings, metres.
  final double sideM;
  final MarkerDetector detector;

  /// Where `calibration.json` lives; the app's Documents folder by default.
  final Future<Directory> Function()? documents;

  @override
  State<MarkerOverlay> createState() => _MarkerOverlayState();
}

class _MarkerOverlayState extends State<MarkerOverlay> {
  StreamSubscription<ArFrame>? _sub;
  CalibrationResult? _calibration;
  bool _busy = false;
  ArFrame? _frame;
  FrameDetections? _found;
  IntrinsicsSource? _source;
  String? _error;

  @override
  void initState() {
    super.initState();
    _loadCalibration();
    _sub = context.read<SlamService>().frames.listen(_onFrame);
  }

  Future<void> _loadCalibration() async {
    try {
      final dir = await (widget.documents ?? getApplicationDocumentsDirectory)();
      final c = await CalibrationResult.load(dir);
      if (mounted) setState(() => _calibration = c);
    } catch (_) {
      // No Documents folder (tests, desktop) or unreadable file: ARKit's
      // intrinsics are used, and the status line says so.
    }
  }

  void _onFrame(ArFrame frame) {
    final jpeg = frame.jpeg;
    if (_busy || jpeg == null) return;
    _busy = true;
    final (k, source) = chooseIntrinsics(frame.intrinsics, _calibration);
    widget.detector.detect(jpeg, k, widget.sideM).then((found) {
      if (!mounted) return;
      setState(() {
        _frame = frame;
        _found = found;
        _source = source;
        _error = null;
      });
    }, onError: (Object e) {
      if (mounted) setState(() => _error = '$e');
    }).whenComplete(() => _busy = false);
  }

  @override
  void dispose() {
    _sub?.cancel();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    final frame = _frame;
    final found = _found;
    return IgnorePointer(
      child: LayoutBuilder(
        builder: (context, constraints) {
          final size = constraints.biggest;
          final mapping = frame == null ? null : ImageToView.fromFrame(frame, size);
          final markers = found?.markers ?? const <MarkerObservation>[];
          return Stack(
            fit: StackFit.expand,
            children: [
              if (mapping != null) CustomPaint(painter: _OutlinePainter(markers, mapping)),
              if (mapping != null)
                for (final m in markers) _label(m, _centre(m, mapping, size), size),
              Positioned(left: 8, bottom: 8, right: 8, child: _status()),
            ],
          );
        },
      ),
    );
  }

  static Offset _centre(MarkerObservation m, ImageToView mapping, Size size) {
    final ps = [for (final c in m.corners) mapping.map(c, size)];
    return ps.reduce((a, b) => a + b) / 4;
  }

  Widget _label(MarkerObservation m, Offset at, Size size) {
    final d = m.distanceM;
    final text = 'ID ${m.id}${d == null ? '' : '  ${d.toStringAsFixed(3)} m'}'
        '${m.accepted ? '' : '  ${_reason(m.rejection!)}'}';
    return Positioned(
      left: (at.dx - 70).clamp(0, math.max(0, size.width - 140)),
      top: (at.dy - 12).clamp(0, math.max(0, size.height - 24)),
      width: 140,
      child: Center(
        child: DecoratedBox(
          decoration: BoxDecoration(
            color: (m.accepted ? Colors.green.shade800 : Colors.deepOrange.shade800).withValues(alpha: 0.85),
            borderRadius: BorderRadius.circular(6),
          ),
          child: Padding(
            padding: const EdgeInsets.symmetric(horizontal: 6, vertical: 2),
            child: Text(text, style: const TextStyle(color: Colors.white, fontSize: 12, fontWeight: FontWeight.w600)),
          ),
        ),
      ),
    );
  }

  static String _reason(MarkerRejection r) => switch (r) {
    MarkerRejection.tooSmall => 'too small',
    MarkerRejection.noPose => 'no pose',
    MarkerRejection.highReprojection => 'bad fit',
    MarkerRejection.steepView => 'too steep',
    MarkerRejection.ambiguous => 'ambiguous',
  };

  Widget _status() {
    final found = _found;
    final k = switch (_source) {
      IntrinsicsSource.calibration => 'calibrated K',
      IntrinsicsSource.arkit => 'ARKit K',
      null => _calibration == null ? 'ARKit K' : 'calibrated K',
    };
    final text = _error != null
        ? 'Marker detection failed: $_error'
        : 'ArUco 4X4_50 · side ${(widget.sideM * 100).toStringAsFixed(1)} cm · $k'
              '${found == null ? '' : ' · ${found.markers.length} found · ${found.detectMs.round()} ms'}';
    return Align(
      alignment: Alignment.bottomLeft,
      child: DecoratedBox(
        decoration: BoxDecoration(color: Colors.black.withValues(alpha: 0.6), borderRadius: BorderRadius.circular(12)),
        child: Padding(
          padding: const EdgeInsets.symmetric(horizontal: 8, vertical: 4),
          child: Text(text, style: const TextStyle(color: Colors.white, fontSize: 11)),
        ),
      ),
    );
  }
}

class _OutlinePainter extends CustomPainter {
  _OutlinePainter(this.markers, this.mapping);

  final List<MarkerObservation> markers;
  final ImageToView mapping;

  @override
  void paint(Canvas canvas, Size size) {
    for (final m in markers) {
      final color = m.accepted ? Colors.greenAccent : Colors.orangeAccent;
      final ps = [for (final c in m.corners) mapping.map(c, size)];
      final path = Path()..addPolygon(ps, true);
      canvas.drawPath(
        path,
        Paint()
          ..color = color
          ..style = PaintingStyle.stroke
          ..strokeWidth = 2.5,
      );
      // Top-left corner of the printed marker (first OpenCV corner).
      canvas.drawCircle(ps.first, 4, Paint()..color = Colors.redAccent);
    }
  }

  @override
  bool shouldRepaint(_OutlinePainter old) => old.markers != markers || old.mapping != mapping;
}
