import 'package:vector_math/vector_math_64.dart';

/// Pinhole camera with two-term radial distortion, OpenCV conventions:
/// camera frame x right, y down, z forward; pixel centres on integers.
///
///     x' = x/z,  y' = y/z,  r² = x'² + y'²
///     d  = 1 + k1·r² + k2·r⁴
///     u  = fx·d·x' + cx,  v = fy·d·y' + cy
class CameraIntrinsics {
  const CameraIntrinsics({
    required this.fx,
    required this.fy,
    required this.cx,
    required this.cy,
    required this.width,
    required this.height,
    this.k1 = 0,
    this.k2 = 0,
  });

  final double fx;
  final double fy;
  final double cx;
  final double cy;
  final int width;
  final int height;
  final double k1;
  final double k2;

  bool get hasDistortion => k1 != 0 || k2 != 0;

  /// Pixel of camera-frame point [p], or null when it is behind the camera.
  Vector2? project(Vector3 p) {
    if (p.z <= 1e-9) return null;
    return projectNormalized(p.x / p.z, p.y / p.z);
  }

  Vector2 projectNormalized(double x, double y) {
    final r2 = x * x + y * y;
    final d = 1 + k1 * r2 + k2 * r2 * r2;
    return Vector2(fx * d * x + cx, fy * d * y + cy);
  }

  /// Undistorted normalised image coordinates (x/z, y/z) of pixel (u, v).
  Vector2 normalize(double u, double v) {
    final xd = (u - cx) / fx;
    final yd = (v - cy) / fy;
    if (!hasDistortion) return Vector2(xd, yd);
    // Fixed-point inversion of the radial model; converges in a few steps
    // for the mild distortion of phone cameras.
    var x = xd;
    var y = yd;
    for (var i = 0; i < 8; i++) {
      final r2 = x * x + y * y;
      final d = 1 + k1 * r2 + k2 * r2 * r2;
      x = xd / d;
      y = yd / d;
    }
    return Vector2(x, y);
  }

  /// The same camera at a different image resolution (pixel-centre aware).
  CameraIntrinsics scaledTo(int newWidth, int newHeight) {
    final sx = newWidth / width;
    final sy = newHeight / height;
    return CameraIntrinsics(
      fx: fx * sx,
      fy: fy * sy,
      cx: (cx + 0.5) * sx - 0.5,
      cy: (cy + 0.5) * sy - 0.5,
      width: newWidth,
      height: newHeight,
      k1: k1,
      k2: k2,
    );
  }

  Map<String, dynamic> toJson() => {
        'fx': fx,
        'fy': fy,
        'cx': cx,
        'cy': cy,
        'width': width,
        'height': height,
        'k1': k1,
        'k2': k2,
      };

  factory CameraIntrinsics.fromJson(Map<String, dynamic> json) => CameraIntrinsics(
        fx: (json['fx'] as num).toDouble(),
        fy: (json['fy'] as num).toDouble(),
        cx: (json['cx'] as num).toDouble(),
        cy: (json['cy'] as num).toDouble(),
        width: (json['width'] as num).toInt(),
        height: (json['height'] as num).toInt(),
        k1: (json['k1'] as num?)?.toDouble() ?? 0,
        k2: (json['k2'] as num?)?.toDouble() ?? 0,
      );

  @override
  String toString() => 'fx ${fx.toStringAsFixed(1)}, fy ${fy.toStringAsFixed(1)}, '
      'cx ${cx.toStringAsFixed(1)}, cy ${cy.toStringAsFixed(1)} @ $width×$height'
      '${hasDistortion ? ', k1 ${k1.toStringAsFixed(3)}, k2 ${k2.toStringAsFixed(3)}' : ''}';
}
