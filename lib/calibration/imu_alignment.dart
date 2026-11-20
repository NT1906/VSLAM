import 'dart:math' as math;

import 'package:vector_math/vector_math_64.dart';

import '../core/se3.dart';

/// A timestamped orientation `R_wc` of the OpenCV camera in some world frame.
class TimedRotation {
  const TimedRotation(this.tUs, this.rotation);

  final int tUs;
  final Matrix3 rotation;
}

/// A timestamped gyroscope sample (rad/s, device / IMU axes).
class TimedVector {
  const TimedVector(this.tUs, this.value);

  final int tUs;
  final Vector3 value;
}

/// Camera-IMU time offset and rotation estimated from one recording.
///
/// Model: `ω_c(t) = R_ci · ω_i(t + offset)`, where `ω_c` is the camera's
/// angular velocity (camera axes) from consecutive ARKit orientations stamped
/// `t`, and `ω_i` is the gyroscope (IMU axes). A positive [offsetUs] means a
/// rotation appears in the gyroscope stamps later than in the camera stamps;
/// to put an IMU sample on the camera clock, subtract [offsetUs].
class ImuAlignment {
  const ImuAlignment({
    required this.offsetUs,
    required this.rCI,
    required this.rmsResidual,
    required this.rmsResidualIdentityOffset,
    required this.samples,
    required this.meanRate,
  });

  final double offsetUs;

  /// `R_ci`: maps an IMU-frame vector into the OpenCV camera frame.
  final Matrix3 rCI;

  /// RMS of `|ω_c − R_ci·ω_i|` (rad/s) at the estimated offset.
  final double rmsResidual;

  /// The same with zero time offset (same rotation), to show the offset helps.
  final double rmsResidualIdentityOffset;

  /// Number of angular-velocity pairs used.
  final int samples;

  /// Mean camera rotation rate over the used pairs (rad/s): excitation.
  final double meanRate;

  /// Angle (degrees) between [rCI] and [expected].
  double angleToDeg(Matrix3 expected) =>
      rotationAngle(expected.transposed().multiplied(rCI)) * 180 / math.pi;

  Map<String, dynamic> toJson() => {
        'offset_us': offsetUs,
        'R_ci_rows': [
          for (var r = 0; r < 3; r++) [for (var c = 0; c < 3; c++) rCI.entry(r, c)],
        ],
        'rms_residual_rad_s': rmsResidual,
        'rms_residual_zero_offset_rad_s': rmsResidualIdentityOffset,
        'samples': samples,
        'mean_rate_rad_s': meanRate,
        'angle_to_apple_convention_deg': angleToDeg(appleCameraFromDevice),
      };
}

/// `R_ci` implied by Apple's documented axes (see `docs/DESIGN.md`, 5.2).
///
/// Core Motion device axes: x right, y toward the top of the screen in
/// portrait, z out of the screen. ARKit camera space: x along the long axis
/// from the front camera toward the bottom (Home button side), z out of the
/// screen side, y = z × x, which is device +x. The OpenCV camera flips y and
/// z of the ARKit camera, so:
///
///     x_cv = −y_dev,  y_cv = −x_dev,  z_cv = −z_dev
final Matrix3 appleCameraFromDevice = Matrix3(
  // Column-major: columns are the images of device x, y, z.
  0, -1, 0, //
  -1, 0, 0, //
  0, 0, -1, //
);

/// Camera angular velocity in camera axes, one per consecutive pair, stamped
/// at the midpoint: `ω = Log(R_k⁻¹ R_{k+1}) / Δt`.
List<TimedVector> cameraRates(List<TimedRotation> rotations) {
  final out = <TimedVector>[];
  for (var k = 0; k + 1 < rotations.length; k++) {
    final a = rotations[k];
    final b = rotations[k + 1];
    final dt = (b.tUs - a.tUs) * 1e-6;
    if (dt <= 1e-4 || dt > 0.2) continue;
    final w = so3Log(a.rotation.transposed().multiplied(b.rotation)) / dt;
    out.add(TimedVector((a.tUs + b.tUs) ~/ 2, w));
  }
  return out;
}

/// Linear interpolation of [samples] (time-ordered) at [tUs], or null outside.
Vector3? interpolate(List<TimedVector> samples, double tUs) {
  if (samples.isEmpty || tUs < samples.first.tUs || tUs > samples.last.tUs) return null;
  var lo = 0;
  var hi = samples.length - 1;
  while (hi - lo > 1) {
    final mid = (lo + hi) >> 1;
    if (samples[mid].tUs <= tUs) {
      lo = mid;
    } else {
      hi = mid;
    }
  }
  final a = samples[lo];
  final b = samples[hi];
  final span = b.tUs - a.tUs;
  if (span <= 0) return a.value.clone();
  final f = (tUs - a.tUs) / span;
  return a.value * (1 - f) + b.value * f;
}

/// Estimates the time offset and rotation.
///
/// 1. Offset: the shift minimising the mean squared difference of rate
///    magnitudes `|ω_c(t)|` and `|ω_i(t + δ)|` (independent of the rotation),
///    on a 1 ms grid over ±[searchUs], refined with a parabola through the
///    best grid point and its neighbours.
/// 2. Rotation: least squares `min Σ |ω_c − R ω_i|²` in closed form (Horn's
///    quaternion method: largest eigenvector of a 4×4 symmetric matrix).
ImuAlignment estimateImuAlignment(
  List<TimedRotation> cameraOrientations,
  List<TimedVector> gyro, {
  int searchUs = 200000,
  int stepUs = 1000,
  double minRate = 0.1,
}) {
  final cam = cameraRates(cameraOrientations).where((s) => s.value.length >= minRate).toList();
  if (cam.length < 20 || gyro.length < 20) {
    throw StateError('Not enough rotation: ${cam.length} camera rates above $minRate rad/s, ${gyro.length} gyro samples');
  }

  double cost(double offset) {
    var sum = 0.0;
    var n = 0;
    for (final s in cam) {
      final g = interpolate(gyro, s.tUs + offset);
      if (g == null) continue;
      final d = s.value.length - g.length;
      sum += d * d;
      n++;
    }
    // Penalise shifts that leave too little overlap.
    return n < cam.length ~/ 2 ? double.infinity : sum / n;
  }

  final steps = searchUs ~/ stepUs;
  var best = 0;
  var bestCost = double.infinity;
  final costs = <int, double>{};
  for (var i = -steps; i <= steps; i++) {
    final c = cost(i * stepUs.toDouble());
    costs[i] = c;
    if (c < bestCost) {
      bestCost = c;
      best = i;
    }
  }
  var offset = best * stepUs.toDouble();
  final cm = costs[best - 1];
  final cp = costs[best + 1];
  if (cm != null && cp != null && cm.isFinite && cp.isFinite) {
    final denom = cm - 2 * bestCost + cp;
    if (denom > 0) offset += 0.5 * (cm - cp) / denom * stepUs;
  }

  Matrix3 fitRotation(double offset) {
    final ps = _pairs(cam, gyro, offset);
    return alignRotations(ps.map((p) => p.$2).toList(), ps.map((p) => p.$1).toList());
  }

  // 3. Refine: with the rotation known, the full vector residual pins the
  //    offset down better than magnitudes (noise biases |ω|). Alternate a
  //    0.1 ms search over ±5 ms with the rotation fit.
  var r = fitRotation(offset);
  for (var iter = 0; iter < 3; iter++) {
    var bestFine = offset;
    var bestFineCost = double.infinity;
    for (var i = -50; i <= 50; i++) {
      final o = offset + i * 100.0;
      var sum = 0.0;
      var n = 0;
      for (final s in cam) {
        final g = interpolate(gyro, s.tUs + o);
        if (g == null) continue;
        sum += (s.value - r.transformed(g)).length2;
        n++;
      }
      if (n < cam.length ~/ 2) continue;
      final c = sum / n;
      if (c < bestFineCost) {
        bestFineCost = c;
        bestFine = o;
      }
    }
    offset = bestFine;
    r = fitRotation(offset);
  }

  final pairs = _pairs(cam, gyro, offset);

  double rms(List<(Vector3, Vector3)> ps) {
    var s = 0.0;
    for (final (c, i) in ps) {
      s += (c - r.transformed(i)).length2;
    }
    return math.sqrt(s / ps.length);
  }

  return ImuAlignment(
    offsetUs: offset,
    rCI: r,
    rmsResidual: rms(pairs),
    rmsResidualIdentityOffset: rms(_pairs(cam, gyro, 0)),
    samples: pairs.length,
    meanRate: pairs.fold(0.0, (a, p) => a + p.$1.length) / pairs.length,
  );
}

List<(Vector3, Vector3)> _pairs(List<TimedVector> cam, List<TimedVector> gyro, double offset) => [
      for (final s in cam)
        if (interpolate(gyro, s.tUs + offset) case final g?) (s.value, g),
    ];

/// Rotation `R` minimising `Σ |b_k − R a_k|²` (Horn 1987, unit quaternions).
Matrix3 alignRotations(List<Vector3> a, List<Vector3> b) {
  // S_xy = Σ a_x b_y.
  final s = List.generate(3, (_) => List.filled(3, 0.0));
  for (var k = 0; k < a.length; k++) {
    for (var i = 0; i < 3; i++) {
      for (var j = 0; j < 3; j++) {
        s[i][j] += a[k][i] * b[k][j];
      }
    }
  }
  final sxx = s[0][0], sxy = s[0][1], sxz = s[0][2];
  final syx = s[1][0], syy = s[1][1], syz = s[1][2];
  final szx = s[2][0], szy = s[2][1], szz = s[2][2];
  final n = [
    [sxx + syy + szz, syz - szy, szx - sxz, sxy - syx],
    [syz - szy, sxx - syy - szz, sxy + syx, szx + sxz],
    [szx - sxz, sxy + syx, -sxx + syy - szz, syz + szy],
    [sxy - syx, szx + sxz, syz + szy, -sxx - syy + szz],
  ];
  final (values, vectors) = jacobiEigen(n);
  var bestI = 0;
  for (var i = 1; i < 4; i++) {
    if (values[i] > values[bestI]) bestI = i;
  }
  final q = Quaternion(vectors[1][bestI], vectors[2][bestI], vectors[3][bestI], vectors[0][bestI])..normalize();
  return q.asRotationMatrix();
}

/// Eigen-decomposition of a small symmetric matrix by cyclic Jacobi
/// rotations. Returns the eigenvalues and a matrix whose columns are the
/// matching unit eigenvectors.
(List<double>, List<List<double>>) jacobiEigen(List<List<double>> input) {
  final n = input.length;
  final a = [for (final row in input) List<double>.of(row)];
  final v = List.generate(n, (i) => List.generate(n, (j) => i == j ? 1.0 : 0.0));
  for (var sweep = 0; sweep < 100; sweep++) {
    var off = 0.0;
    for (var p = 0; p < n; p++) {
      for (var q = p + 1; q < n; q++) {
        off += a[p][q] * a[p][q];
      }
    }
    if (off < 1e-30) break;
    for (var p = 0; p < n; p++) {
      for (var q = p + 1; q < n; q++) {
        if (a[p][q].abs() < 1e-300) continue;
        final theta = (a[q][q] - a[p][p]) / (2 * a[p][q]);
        final t = theta.sign == 0
            ? 1.0
            : theta.sign / (theta.abs() + math.sqrt(theta * theta + 1));
        final c = 1 / math.sqrt(t * t + 1);
        final sn = t * c;
        for (var k = 0; k < n; k++) {
          final akp = a[k][p];
          final akq = a[k][q];
          a[k][p] = c * akp - sn * akq;
          a[k][q] = sn * akp + c * akq;
        }
        for (var k = 0; k < n; k++) {
          final apk = a[p][k];
          final aqk = a[q][k];
          a[p][k] = c * apk - sn * aqk;
          a[q][k] = sn * apk + c * aqk;
        }
        for (var k = 0; k < n; k++) {
          final vkp = v[k][p];
          final vkq = v[k][q];
          v[k][p] = c * vkp - sn * vkq;
          v[k][q] = sn * vkp + c * vkq;
        }
      }
    }
  }
  return ([for (var i = 0; i < n; i++) a[i][i]], v);
}
